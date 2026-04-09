import os
import time
import math
import threading
from collections import deque
from tqdm import tqdm
import numpy as np
import cv2
import pyrealsense2 as rs
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import robotiq_gripper
from robot_env_corrected.time_env import time_ms
from robot_env_corrected.transformations import axis_to_euler, euler_to_axis, axis_to_quat, add_poses, Euler2Axis_Pose, quat_to_euler
from robot_env_corrected.subprocess_utils import run_threaded_command
# from ur10e.robot_env_corrected.multi_camera_wrapper_rtde import MultiCameraWrapper
import h5py
from pathlib import Path
import pyrealsense2 as rs
import cv2
import numpy as np
from robot_env_corrected.subprocess_utils import run_threaded_command

class MultiCameraWrapper:
    def __init__(self):
        print("[Init] Starting camera setup...")

        print("[Init] Setting up Robot Wrist Camera pipeline and config")
        self.pipeline_robot = rs.pipeline()
        self.config_robot = rs.config()
        self.config_robot.enable_device('218622276856')
        print("[Init] Device: Intel Realsense D405 - Robot Wrist Camera")
        self.config_robot.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

        print("[Init] Setting up User Camera pipeline and config")
        self.pipeline_user = rs.pipeline()
        self.config_user = rs.config()
        self.config_user.enable_device('128422270567')
        print("[Init] Device: Intel Realsense D405 - User Camera")
        self.config_user.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

        print("[Init] Launching set_trajectory_mode in separate thread")
        run_threaded_command(self.set_trajectory_mode)

    def set_trajectory_mode(self):
        print("[set_trajectory_mode] Starting Robot Wrist Camera pipeline...")
        try:
            self.pipeline_robot.start(self.config_robot)
        except Exception as e:
            print(f"[set_trajectory_mode] Failed to start Robot Wrist Camera pipeline: {e}")
        else:
            print("[set_trajectory_mode] Robot Wrist Camera started.")

        print("[set_trajectory_mode] Starting User Camera pipeline...")
        try:
            self.pipeline_user.start(self.config_user)
        except Exception as e:
            print(f"[set_trajectory_mode] Failed to start User Camera pipeline: {e}")
        else:
            print("[set_trajectory_mode] User Camera started.")


    def read_cameras(self):
        # print("[read_cameras] Waiting for frames from Robot Wrist Camera...")
        frames_robot = self.pipeline_robot.wait_for_frames()
        color_frame_robot = frames_robot.get_color_frame()
        # print("[read_cameras] Got Robot Wrist Camera frame.")
        robot_frame = np.asanyarray(color_frame_robot.get_data())
        robot_frame = cv2.resize(robot_frame, (640, 480))
        robot_frame = cv2.rotate(robot_frame, cv2.ROTATE_90_CLOCKWISE)
        padding = np.zeros((640, 640, 3), dtype=np.uint8)
        padding[:,:480, :] = robot_frame
        robot_frame = padding


        # print("[read_cameras] Waiting for frames from User Camera...")
        frames_user = self.pipeline_user.wait_for_frames()
        color_frame_user = frames_user.get_color_frame()
        # print("[read_cameras] Got User Camera frame.")
        user_frame = np.asanyarray(color_frame_user.get_data())
        user_frame = user_frame[0:430, 5:500]
        user_frame = cv2.resize(user_frame, (640, 640))

        cv2.imshow("Wrist", robot_frame)
        cv2.imshow("POV", user_frame)
        # print(robot_frame.shape)
        cv2.waitKey(1)
        # print("[read_cameras] Returning frames.")
        return robot_frame, user_frame

    def stop_recording(self):
        print("[stop_recording] Stopping Robot Wrist Camera pipeline...")
        self.pipeline_robot.stop()
        print("[stop_recording] Stopping User Camera pipeline...")
        self.pipeline_user.stop()
        print("[stop_recording] Cameras stopped.")


class RobotAction:
    def __init__(self, robot_ip="192.168.0.110", acceleration=0.5, velocity=0.5,
                 do_reset=True, control_hz=5):
        self.robot_ip = robot_ip
        self.acceleration = acceleration
        self.velocity = velocity
        self.control_hz = control_hz

        self.pose = []
        self.gripper_current_state = 0
        self.first_run = True
        self.initial_pos = None
        self.sum_deltas = np.zeros(3)
        self.error_history = []
        self.pose_history = []
        self.joint_history = []
        self.delta_history = []
        self.timestamp_history = []
        self.freq_history = []

        rtde_frequency = 500.0
        flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
        ur_cap_port = 50002
        rt_receive_priority = 90
        rt_control_priority = 85

        self.rtde_r = RTDEReceive(self.robot_ip, rtde_frequency, [], True, False, rt_receive_priority)
        self.rtde_c = RTDEControl(self.robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)

        self.gripper = robotiq_gripper.RobotiqGripper()
        self.gripper.connect(self.robot_ip, 63352)
        self.gripper.activate()
        time.sleep(1)

        self.camera_reader = MultiCameraWrapper()

        self.camera_frame_history = []

        self.output_dir = "outputs"
        os.makedirs(self.output_dir, exist_ok=True)
        self.npy_save_path = "test.npz"

        run_threaded_command(self._update_actual_pose)

        if do_reset:
            self.reset()

    def reset(self):
        self.move_to_start_position()
        time.sleep(1)


    def move_to_start_position(self, start_position=(
        math.radians(262.85),
        math.radians(-87.14),
        math.radians(111.61),
        math.radians(246.68),
        math.radians(-89.5),
        math.radians(-10.81)
    )):
        self.rtde_c.moveJ(start_position)

    def _update_actual_pose(self, hz_pose=30):
        while True:
            time.sleep(1 / hz_pose)
            self.pose = np.array(self.rtde_r.getActualTCPPose())

    def get_observation_xyz(self):
        tcp_pose = self.pose
        position = tcp_pose[:3]
        euler_angles = tcp_pose[3:]
        gripper_state = 0 if self.gripper.is_open() else 1
        return list(position), list(euler_angles), gripper_state

    def update_commands(self, new_action, non_blocking, velocity, accel):
        dt = 1.0 / self.control_hz
        lookahead_time = 0.03
        gain = 1000

        current_pos = self.pose[:3]
        current_euler = axis_to_euler(self.pose[3:])
        current_euler_pose = np.concatenate([current_pos, current_euler])

        target_euler = add_poses(new_action, current_euler_pose)
        self.target = Euler2Axis_Pose(target_euler).tolist()

        self.rtde_c.servoL(self.target, velocity, accel, dt, lookahead_time, gain)

    def send_gripper_command(self, gripper_value, speed=70, force=50):
        self.gripper.move(gripper_value, speed, force)
        
    def pad_to_640x640(self, image):
        """Pads image with black pixels to make it 640x640. Assumes image is smaller or equal."""
        h, w = image.shape[:2]
        pad_height = max(0, 640 - h)
        pad_width = max(0, 640 - w)

        padding = ((0, pad_height), (0, pad_width), (0, 0)) if image.ndim == 3 else ((0, pad_height), (0, pad_width))
        return np.pad(image, padding, mode='constant', constant_values=0)


    def get_observation(self):
        """
        convert [x,y,z,wx,wy,wz] -> [x,y,z, euler angles]
        """
        state_dict= {}
        tcp_pose = self.pose
        robot_pos = tcp_pose[:3]
        robot_quat = axis_to_quat(tcp_pose[3:])
        
        joints = np.array(self.rtde_r.getActualQ()) 
        
        if self.gripper.is_open():
            gripper_state = 0
        else:
            gripper_state = 1
            
        action_blocked = False 
        
        tcp_pose_formatted = np.concatenate([joints, robot_pos, robot_quat, [1 - gripper_state], [action_blocked]])
        
        state_dict["robot_state"] = tcp_pose_formatted
        secondpov_frame, primary_img = self.camera_reader.read_cameras()


        state_dict["image"] = np.array(primary_img) 
        state_dict["wrist_image"] = np.array(secondpov_frame) 


   
        return state_dict
    
    def get_observation_legacy(self):
        """
        convert [x,y,z,wx,wy,wz] -> [x,y,z, euler angles]
        """
        state_dict= {}
       
        tcp_pose = self.pose
        robot_pos = tcp_pose[:3]
        robot_quat = axis_to_quat(tcp_pose[3:]) 
        
        joints = np.array(self.rtde_r.getActualQ()) 
        if self.gripper.is_open():
            gripper_state = 0
        else:
            gripper_state = 1
            
        action_blocked = False
        
        tcp_pose_formatted = np.concatenate([joints, robot_pos, robot_quat, [1 - gripper_state], [action_blocked]])
        
        state_dict["robot_state"] = tcp_pose_formatted 
        secondpov_frame, primary_img = self.camera_reader.read_cameras()
        
        print(np.array(primary_img).shape)
        print(np.array(primary_img).shape)

        state_dict["image"] = np.array(primary_img)
        state_dict["wrist_image"] = np.array(secondpov_frame)
        # stacked_viewframes = np.hstack((secondpov_frame, primary_img))
        # cv2.imshow("Camera Views", stacked_viewframes)
        # cv2.waitKey(1)
        return state_dict 



    
    def send_action(self, action, non_blocking=True, velocity=0.25, accel=1.2,
                    feedback_enabled=False, feedback_multiplier=0.5, target_freq=15.0):
        step_start = time.time()

        if self.first_run:
            pos, _, _ = self.get_observation_xyz()
            self.initial_pos = np.array(pos, dtype=np.float64)
            self.sum_deltas = np.zeros(3, dtype=np.float64)
            self.first_run = False

        initial_delta = np.array(action[:3], dtype=np.float64)
        delta = initial_delta.copy()
        max_delta_cm = 1 / self.control_hz

        self.expected_pos = self.initial_pos + self.sum_deltas + delta
        pos, _, _ = self.get_observation_xyz()
        current_pos = np.array(pos, dtype=np.float64)
        self.error = current_pos - self.expected_pos

        if feedback_enabled:
            correction = -self.error * feedback_multiplier
            delta += correction

        corrected_mag = np.linalg.norm(delta)
        if corrected_mag > max_delta_cm:
            delta = delta / corrected_mag * max_delta_cm

        self.sum_deltas += initial_delta
        self.expected_pos = self.initial_pos + self.sum_deltas

        pose_action = np.array(action[:6], dtype=np.float64)
        pose_action[:3] = delta
        gripper_state = float(action[-1])
        self.update_commands(pose_action, non_blocking, velocity, accel)

        if gripper_state != self.gripper_current_state:
            self.send_gripper_command(int(gripper_state * 255))
            self.gripper_current_state = gripper_state

        t_now = time.time()

        current_joints = np.array(self.rtde_r.getActualQ())
        self.joint_history.append(current_joints)

        self.pose_history.append(self.pose.copy())
        self.delta_history.append(delta.copy())
        self.timestamp_history.append(t_now)

        elapsed = time.time() - step_start

        if len(self.timestamp_history) > 1:
            dt_instant = self.timestamp_history[-1] - self.timestamp_history[-2]
            freq_instant = 1.0 / dt_instant if dt_instant > 0 else 0.0
        else:
            freq_instant = 0.0

        window = 1.0
        t_now = self.timestamp_history[-1]
        recent_ts = [t for t in self.timestamp_history if t_now - t <= window]

        if len(recent_ts) > 1:
            freq_rolling = (len(recent_ts) - 1) / (recent_ts[-1] - recent_ts[0])
        else:
            freq_rolling = 0.0

        self.freq_history.append(freq_rolling)

        # if elapsed < 1.0 / target_freq:
        #     time.sleep(1.0 / target_freq - elapsed)



        # print(f"Loop Time: {elapsed:.3f}s | Target: {1.0 / target_freq:.3f}s | "
        #     f"Freq Instant: {freq_instant:.2f} Hz | Freq Rolling({window}s): {freq_rolling:.2f} Hz \n")
        if feedback_enabled:
            print(f"Position Error: x={self.error[0]:.4f}, y={self.error[1]:.4f}, z={self.error[2]:.4f}")
            print(f"Action sent: x={pose_action[0]:.4f}, y={pose_action[1]:.4f}, z={pose_action[2]:.4f}")
            print(f"Action recieved from model: x={action[0]:.4f}, y={action[1]:.4f}, z={action[2]:.4f}\n")

        return self.error.copy()




    def stop(self):
        cv2.destroyAllWindows()


    def shutdown(self):
        if hasattr(self, 'rtde_c') and self.rtde_c is not None:
            try:
                self.rtde_c.stopScript()
            except Exception as e:
                print(f"Warning: could not stop RTDE script cleanly: {e}")
            self.rtde_c.disconnect()
            self.rtde_c = None

        if hasattr(self, 'rtde_r') and self.rtde_r is not None:
            self.rtde_r.disconnect()
            self.rtde_r = None

        if hasattr(self, 'gripper'):
            try:
                self.gripper.disconnect()
            except Exception as e:
                print(f"Warning: failed to disconnect gripper: {e}")

        cv2.destroyAllWindows()
        self.camera_reader.stop_recording()

        print("Shutdown completed successfully.")

