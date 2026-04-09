
import os
import time
import math
import threading
import logging
from collections import deque
from tqdm import tqdm
import numpy as np
import cv2
import pyrealsense2 as rs

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import robotiq_gripper
from robot_env.time_env import time_ms
from robot_env.transformations import axis_to_euler, euler_to_axis, axis_to_quat, add_poses, Euler2Axis_Pose, quat_to_euler
from robot_env.subprocess_utils import run_threaded_command

# New
from robot_env.mae_sdk_sensureal.mae_fts_sdk.src import mae_fts_sdk as mae
from robot_env.mae_sdk_sensureal.mae_sdk.src.mae_sdk import log_utils

import h5py
from pathlib import Path



class MultiCameraViewer:
    def __init__(self):
        print("Initializing Intel RealSense D405 cameras...")

        # Robot wrist camera pipeline
        self.pipeline_robot = rs.pipeline()
        self.config_robot = rs.config()
        self.config_robot.enable_device('218622276856')
        self.config_robot.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

        # User camera pipeline
        self.pipeline_user = rs.pipeline()
        self.config_user = rs.config()
        self.config_user.enable_device('128422270567')
        self.config_user.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

        # Start pipelines
        self.pipeline_robot.start(self.config_robot)
        self.pipeline_user.start(self.config_user)

        # Delay and FPS tracking for robot camera
        self.delay_history = deque()
        self.fps_history = deque()
        self.delay_history_max_time = 10.0  # seconds
        self.fps_history_max_time = 5.0     # seconds
        self.last_frame_time = None

    def read_cameras(self):
        now = time.time()

        # Robot wrist camera frames (blocking)
        robot_frames = self.pipeline_robot.wait_for_frames()
        robot_color = robot_frames.get_color_frame()
        robot_image = np.asanyarray(robot_color.get_data())
        robot_image = cv2.resize(robot_image, (640, 480))

        # Calculate delay for robot camera frame
        frame_ts_sec = robot_color.get_timestamp() / 1000.0
        delay = now - frame_ts_sec
        self.delay_history.append((now, delay))

        # Calculate FPS for robot camera frame
        if self.last_frame_time is not None:
            frame_interval = now - self.last_frame_time
            if frame_interval > 0:
                fps = 1.0 / frame_interval
                self.fps_history.append((now, fps))
        self.last_frame_time = now

        # Clean old delay entries
        while self.delay_history and (now - self.delay_history[0][0]) > self.delay_history_max_time:
            self.delay_history.popleft()

        # Clean old FPS entries
        while self.fps_history and (now - self.fps_history[0][0]) > self.fps_history_max_time:
            self.fps_history.popleft()

        # Average FPS over history for smoother output
        fps_values = [f for t, f in self.fps_history]
        avg_fps = sum(fps_values) / len(fps_values) if fps_values else 0.0

        # User camera frames (blocking)
        user_frames = self.pipeline_user.wait_for_frames()
        user_color = user_frames.get_color_frame()
        user_image = np.asanyarray(user_color.get_data())

        # Crop and resize user image
        user_image = user_image[0:430, 5:500]
        user_image = cv2.resize(user_image, (640, 640))
        cv2.imshow("Wrist", robot_image)
        cv2.imshow("POV", user_image)
        # print(robot_frame.shape)
        cv2.waitKey(1)


        # Rotate robot image 90 degrees clockwise
        rotated_robot_image = cv2.rotate(robot_image, cv2.ROTATE_90_CLOCKWISE)

        # Compose combined image side-by-side
        combined_height = max(rotated_robot_image.shape[0], user_image.shape[0])
        combined_width = rotated_robot_image.shape[1] + user_image.shape[1]
        combined_image = np.zeros((combined_height, combined_width, 3), dtype=np.uint8)
        combined_image[:rotated_robot_image.shape[0], :rotated_robot_image.shape[1]] = rotated_robot_image
        combined_image[:user_image.shape[0], rotated_robot_image.shape[1]:] = user_image

        return combined_image, rotated_robot_image, user_image, list(self.delay_history), avg_fps

    def stop(self):
        self.pipeline_robot.stop()
        self.pipeline_user.stop()
        
class ForceTorqueSensor:
    def __init__(self, fts_ip="192.168.1.11", fts_port=10547, log_force=False):
        self.AVAILABLE_FTS_COMMANDS = {str(key): key for key in mae.FTS_COMMAND_RESPONSE.keys()}
        self.logger = log_utils.get_logger(f"{Path(__file__).stem}", log_level=logging.WARNING)
        self.communication_interface = mae.UdpCommunication(
            ip_address=fts_ip,
            port=fts_port,
            timeout_sec=1,
            log_level=logging.WARNING,
        )
        self.log_force=log_force
        self.communication_interface.connect()
        time.sleep(1)
        # Zero the sensor before use
        self.send_command("FtsCommand.TRANSDUCER_SET")
        time.sleep(1)
        self.send_command("FtsCommand.BIAS_SET")
        time.sleep(1)
        self.send_command("FtsCommand.SETTINGS_SAVE")
        time.sleep(1)
        
    def send_command(self, new_command):
        command = self.AVAILABLE_FTS_COMMANDS[new_command]
        self.communication_interface.send_request(command)
        if self.log_force:
            print(f"Sent: {command}.")
        elif new_command != "FtsCommand.STREAM_FT_START":
            print(f"Sent: {command}.")
        # Some Commands don't have a response, hence response_type can be None
        response_type = mae.FTS_COMMAND_RESPONSE[command]
    
        if response_type is not None:
            byte_stream = self.communication_interface.waits_response_bytes()
            if byte_stream is None:
                self.logger.error(f"No Response. Timed-out.")
    
            response = response_type(byte_stream)
            if self.log_force:
                print(f"Received: {response}.")

            return response.ft_sample
        
    def stop(self):
        self.communication_interface.disconnect()

class RobotAction:
    def __init__(self, robot_ip="192.168.0.110", acceleration=0.5, velocity=0.5,
                 do_reset=True, control_hz=30, haptic_feedback=False):
        self.robot_ip = robot_ip
        self.acceleration = acceleration
        self.velocity = velocity
        self.control_hz = control_hz

        self.pose = []
        self.gripper_current_state = 0
        self.first_run = True
        self.initial_pos = None
        self.expected_pos = np.zeros(3)
        self.sum_deltas = np.zeros(3)
        self.delta_no_error_history = []
        self.pose_history = []
        self.joint_history = []
        self.joint_deltas_history = []

        self.delta_history = []
        self.timestamp_history = []
        self.freq_history = []
        self.gripper_history = []

        # RTDE interfaces
        rtde_frequency = 500.0
        flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
        ur_cap_port = 50002
        rt_receive_priority = 90
        rt_control_priority = 85

        self.rtde_r = RTDEReceive(self.robot_ip, rtde_frequency, [], True, False, rt_receive_priority)
        self.rtde_c = RTDEControl(self.robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)

        # Gripper
        self.gripper = robotiq_gripper.RobotiqGripper()
        self.gripper.connect(self.robot_ip, 63352)
        self.gripper.activate()
        time.sleep(1)

        # Multi-camera viewer
        self.camera_viewer = MultiCameraViewer()

        # Frame history to save
        self.camera_frame_history = []

        # For saving data
        #self.output_dir = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/tests"
        # self.output_dir = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/light/haptic"
        # self.output_dir = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/light/vr"
        # self.output_dir = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/wiping/haptic"
        # self.output_dir = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/wiping/vr"
        # self.output_dir = "/media/Data/demoaccount2/robot_manipulation/data_with_fts_2"
        self.output_dir = "/media/Data/demoaccount2/robot_manipulation/Collected_Data/test_040226"
        os.makedirs(self.output_dir, exist_ok=True)
        self.npy_save_path = "test.npz"

        # Pose polling thread
        run_threaded_command(self._update_actual_pose)

        if do_reset:
            self.reset()
        
        # Force Torque Sensor
        if haptic_feedback:
            self.fts = ForceTorqueSensor()                      
        else:
            self.fts = None                    

        self.has_shutdown = False
        self.deg_counter = 0
        self.ft_vec = [0,0,0,0,0,0]
        self.ft_history = []

    def reset(self):
        self.move_to_start_position()
        time.sleep(5)


    def move_to_start_position(self):
        # # WIPE START POSITION
        start_position=(
        math.radians(262.85),
        math.radians(-87.14),
        math.radians(111.61),
        math.radians(246.68),
        math.radians(-89.5),
        math.radians(-10.81)
        )

        # LIGHT START POSITION
        # start_position=(
        # math.radians(256.97),
        # math.radians(-61.94),
        # math.radians(133.80),
        # math.radians(109.75),
        # math.radians(-76.24),
        # math.radians(-4.66)
        # )

        # # Sample change Rz
        # start_position=(
        # math.radians(264.32),
        # math.radians(-70.62),
        # math.radians(91.80),
        # math.radians(248.34),
        # math.radians(-89.14),
        # math.radians(65.29)
        # )

        # # Sample change Ry
        # start_position=(
        # math.radians(254.07),
        # math.radians(-85.70),
        # math.radians(110.68),
        # math.radians(278.28),
        # math.radians(-29.18),
        # math.radians(-40.08)
        # )

        # Sample change Rx
        # start_position=(
        # math.radians(263.67),
        # math.radians(-75.42),
        # math.radians(92.94),
        # math.radians(308.42),
        # math.radians(-94.43),
        # math.radians(-6.90)
        # )


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
        lookahead_time = 0.1
        gain = 2000

        current_pos = self.pose[:3]
        current_euler = axis_to_euler(self.pose[3:])
        current_euler_pose = np.concatenate([current_pos, current_euler])

        target_euler = add_poses(new_action, current_euler_pose)
        self.target = Euler2Axis_Pose(target_euler).tolist()

        self.rtde_c.servoL(self.target, velocity, accel, dt, lookahead_time, gain)

    def send_gripper_command(self, gripper_value, speed=70, force=50):
        self.gripper.move(gripper_value, speed, force)
    
    def get_observation(self):
        """
        convert [x,y,z,wx,wy,wz] -> [x,y,z, euler angles]
        """
        state_dict= {}
        # state_dict should collect:
            # robot_state: [6 joints, x, y, z, rx, ry, rz, gripper, action_blocked]
            
        
        #get TCP pose (position+ then convert to euler angles)
        # Get the 6D TCP pose (x, y, z, rx, ry, rz)
        tcp_pose = self.pose
        robot_pos = tcp_pose[:3]
        robot_quat = axis_to_quat(tcp_pose[3:]) # saved in quat
        
        joints = np.array(self.rtde_r.getActualQ()) # should return 6 joints
        
        #need to add gripper action reading in the future as well
        # Determine the gripper's state
        if self.gripper.is_open():
            gripper_state = 0
        else:
            gripper_state = 1
            
        action_blocked = False # switch this to True if the gripper is implementing blocking, otherwise leave as false
        
        # flip the gripper state when saving
        tcp_pose_formatted = np.concatenate([joints, robot_pos, robot_quat, [1 - gripper_state], [action_blocked]])
        
        state_dict["robot_state"] = tcp_pose_formatted # save robot state
        # state_dict["cartesian_position"] = tcp_pose #pase axis_angle
        
        # state_dict["gripper_position"] = gripper_state
        

        # self.count += 1 # update counter index
        
        # also record actions
        # need to flip the gripper status to be the same as Octo dataset, since 1 should be open and 0 should be closed
        # robotPoseRow = np.append(tcp_pose, (round(1 - gripper_state)))
        # self.poseWriter.writerow(robotPoseRow)
        # return state_dict

    def get_fts_observation(self):
        """
        Get the force-torque sensor reading.
        Adjust coordinate frame for the force vector based on TCP orientation.
        Returns:
            A list of 6 floats representing the transformed force-torque sensor reading [Fx, Fy, Fz, Tx, Ty, Tz].
        """

        # Recieve raw force torque vector in FTS frame
        ft_vec_raw = np.array(self.fts.send_command("FtsCommand.STREAM_FT_START"))
        # Extract the force vector
        force_vec_raw = ft_vec_raw[:3]
        # Transform the force vector to TCP frame
        fx, fy, fz = force_vec_raw
        force_vec_tcp_raw = np.array([fy, -fx, fz])
        # Extract the torque vector
        torque_vec = ft_vec_raw[3:]

        # Extract the rotation vector describing TCP orientation
        eef_rot_vec = self.pose[3:]
        eef_rot_mag = np.linalg.norm(eef_rot_vec)
        eef_rot_axis = eef_rot_vec/eef_rot_mag
        # Declare the TCP frame
        x_axis = np.array([1, 0, 0])
        y_axis = np.array([0, 1, 0])
        z_axis = np.array([0, 0, 1])
        # Apply Rodrigues' rotation formula to find new TCP axes orientation
        eef_x_axis = ((1 - math.cos(eef_rot_mag)) * (np.dot(x_axis, eef_rot_axis)) * (eef_rot_axis) + (x_axis) * (math.cos(eef_rot_mag)) + (np.cross(eef_rot_axis, x_axis)) * (math.sin(eef_rot_mag)))
        eef_y_axis = ((1 - math.cos(eef_rot_mag)) * (np.dot(y_axis, eef_rot_axis)) * (eef_rot_axis) + (y_axis) * (math.cos(eef_rot_mag)) + (np.cross(eef_rot_axis, y_axis)) * (math.sin(eef_rot_mag)))
        eef_z_axis = ((1 - math.cos(eef_rot_mag)) * (np.dot(z_axis, eef_rot_axis)) * (eef_rot_axis) + (z_axis) * (math.cos(eef_rot_mag)) + (np.cross(eef_rot_axis, z_axis)) * (math.sin(eef_rot_mag)))
        # Define the rotation matrix described by the new TCP axes
        eef_rot_matrix = np.column_stack([eef_x_axis, eef_y_axis, eef_z_axis])
        # Transform the force vector based off the TCP orientation
        force_vec_tcp = eef_rot_matrix @ force_vec_tcp_raw

        # Transform the force vector to Haply frame
        fx_tcp, fy_tcp, fz_tcp = force_vec_tcp
        force_vec = np.array([-fy_tcp, fx_tcp, fz_tcp])

        # Combine new force vector and raw torque vector to recieve force torque vector
        self.ft_vec = (np.concatenate((force_vec, torque_vec))).tolist()

        return self.ft_vec

    def send_action(self, action, non_blocking=True, velocity=0.25, accel=1.2,
                    feedback_enabled=True, feedback_multiplier=1.2, target_freq=30.0):
        step_start = time.time()

        # 1. Initialize on first call
        if self.first_run:
            pos, _, _ = self.get_observation_xyz()

            # print(pos)
            self.initial_pos = np.array(pos, dtype=np.float64)
            self.sum_deltas = np.zeros(3, dtype=np.float64)
            self.first_run = False

        # 2. Compute delta and feedback
        inital_action = action
        initial_delta = np.array(action[:3], dtype=np.float64)
        self.delta_no_error_history.append(initial_delta.copy())
        delta = initial_delta.copy()
        max_delta_cm = 1 / self.control_hz

        t_now = time.time()
        if len(self.timestamp_history) > 0:
            dt = t_now - self.timestamp_history[-1]
            # safeguard dt bounds
            if dt <= 0 or dt > 1.0:
                dt = 1.0 / target_freq
        else:
            dt = 1.0 / target_freq
                                                                                    
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

        # 3. Send robot pose command
        pose_action = np.array(action[:6], dtype=np.float64)
        pose_action[:3] = delta
        gripper_state = float(action[-1])
        self.update_commands(pose_action, non_blocking, velocity, accel)

        # 4. Send gripper command
        if gripper_state != self.gripper_current_state:
            self.send_gripper_command(int(gripper_state * 255))
            self.gripper_current_state = gripper_state

        # 5. Record robot state
        t_now = time.time()

        current_joints = np.array(self.rtde_r.getActualQ())
        try:
            prev_joints = self.joint_history[-1]
        except:
            print("first iteration, prev_joints = current_joints")
            prev_joints = current_joints
        self.joint_history.append(current_joints)

        self.joint_deltas_history.append(current_joints-prev_joints)
        self.pose_history.append(self.pose.copy())
        self.delta_history.append(delta.copy())
        self.timestamp_history.append(t_now)
        self.gripper_history.append(gripper_state)

        # 6. Read & store camera frames
        combined_img, robot_img, user_img, _, _ = self.camera_viewer.read_cameras()

        # Pad robot to 640x640
        padded_robot_img = np.zeros((640, 640, 3), dtype=np.uint8)
        padded_robot_img[:, :480, :] = robot_img

        _, robot_jpg = cv2.imencode('.jpg', padded_robot_img, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        _, user_jpg = cv2.imencode('.jpg', user_img, [int(cv2.IMWRITE_JPEG_QUALITY), 75])


        self.camera_frame_history.append((robot_jpg.tobytes(), user_jpg.tobytes()))

        # 7. Record FTS data
        self.ft_history.append(np.array(self.ft_vec))

        # 8. Timing and frequency calculations
        elapsed = time.time() - step_start

        # Instantaneous frequency: between last two timestamps
        # if len(self.timestamp_history) > 1:
        #     dt_instant = self.timestamp_history[-1] - self.timestamp_history[-2]
        #     freq_instant = 1.0 / dt_instant if dt_instant > 0 else 0.0
        # else:
        #     freq_instant = 0.0

        # Rolling frequency over last 5 seconds window
        window = 1.0
        t_now = self.timestamp_history[-1]
        recent_ts = [t for t in self.timestamp_history if t_now - t <= window]

        if len(recent_ts) > 1:
            freq_rolling = (len(recent_ts) - 1) / (recent_ts[-1] - recent_ts[0])
        else:
            freq_rolling = 0.0

        # Append rolling freq for history if needed
        self.freq_history.append(freq_rolling)

        # Sleep to maintain target frequency
        if elapsed < 1.0 / target_freq:
            time.sleep(1.0 / target_freq - elapsed)

        # Debug print
        # print(f"\033c", end="")
        # print(f"Got action: {inital_action}")
        # print(f"Loop Time: {elapsed:.3f}s | Target: {1.0 / target_freq:.3f}s | "
        #    f"Freq Instant: {freq_instant:.2f} Hz | Freq Rolling({window}s): {freq_rolling:.2f} Hz")
        # if feedback_enabled:
        #     print(f"Position Error: x={self.error[0]:.4f}, y={self.error[1]:.4f}, z={self.error[2]:.4f}")

        return self.error.copy()

    def save_data(self, split="train"):
        """
        Save data in .h5 format with images saved as separate jpg files.
        """
        # print("First check")
        base_dir = f"{self.output_dir}/{split}"
        npz_dir = f"{self.output_dir}/{split}_npz"
        os.makedirs(base_dir, exist_ok=True)
        os.makedirs(npz_dir, exist_ok=True)

        # === AUTO-INCREMENT FILE NAME ===
        existing_files = sorted(Path(base_dir).glob("episode_*.h5"))
        next_id = 1 if not existing_files else int(existing_files[-1].stem.split('_')[-1]) + 1
        file_name = f"episode_{next_id:05d}"
        h5_path = os.path.join(base_dir, file_name + ".h5")
        npz_path = os.path.join(npz_dir, file_name + ".npz")

        # === PREPARE EPISODE DATA ===
        episode_info = np.array([
            (
                b"Turn on the light.",
                1,  # is_success
                1   # episode_id
            )
        ], dtype=[("language_instruction", "S256"), ("is_success", "i4"), ("episode_id", "i4")])

        # === SAVE COMPRESSED IMAGE FILES ===
        image_dir = os.path.join(base_dir, file_name + "_images")
        os.makedirs(image_dir, exist_ok=True)

        image_filenames = []
        wrist_filenames = []

        for idx, (robot_jpg_bytes, user_jpg_bytes) in enumerate(tqdm(self.camera_frame_history, desc="Saving JPGs")):
            wrist_path = os.path.join(image_dir, f"wrist_{idx:04d}.jpg")
            user_path = os.path.join(image_dir, f"user_{idx:04d}.jpg")

            # Save the jpg bytes directly to disk
            with open(wrist_path, 'wb') as f:
                f.write(robot_jpg_bytes)

            with open(user_path, 'wb') as f:
                f.write(user_jpg_bytes)

            # Store relative paths for later reference
            wrist_filenames.append(os.path.relpath(wrist_path, base_dir))
            image_filenames.append(os.path.relpath(user_path, base_dir))

        # === ROBOT STATES ===
        robot_states = np.array([
            np.concatenate([
                #joint positions omitted here can be added back in with self.joint_history in the loop: for gripper, pose in zip(self.gripper_history, self.pose_history)
                joints,
                pose[:3],  #last 3 items of pose omitted here
                axis_to_quat(pose[3:]),
                [1 - gripper],  # use gripper from history (inverted)
                [0]
            ])
            for joints, gripper, pose in zip(self.joint_history, self.gripper_history, self.pose_history)
        ])
        actions = np.array([
            np.append(joint_delta[:6], [1 - gripper])  # inverted gripper per step
            for gripper, joint_delta in zip(self.gripper_history, self.joint_deltas_history)
        ])
        actions_tcp_space = np.array([
            np.append(delta[:6], [1 - gripper])  # inverted gripper per step
            for gripper, delta in zip(self.gripper_history, self.delta_history)
        ])
        actions_tcp_raw = np.array([
            np.append(delta[:6], [1 - gripper])  # inverted gripper per step
            for gripper, delta in zip(self.gripper_history, self.delta_no_error_history)
        ])

        # Saving FTS history
        fts_info = np.array(self.ft_history)

        # === SAVE TO HDF5 ===
        print(f"Saving metadata to {h5_path}")
        with h5py.File(h5_path, 'w') as f:
            f.create_dataset("episode_info", data=episode_info)

            # Save image paths as variable length strings
            dt = h5py.string_dtype(encoding='utf-8')
            f.create_dataset("image_filenames", data=np.array(image_filenames, dtype=object), dtype=dt)
            f.create_dataset("wrist_image_filenames", data=np.array(wrist_filenames, dtype=object), dtype=dt)

            f.create_dataset("robot_state", data=robot_states.astype(np.float32))
            f.create_dataset("action_tcp", data=actions_tcp_space.astype(np.float32))
            f.create_dataset("action", data=actions.astype(np.float32))
            f.create_dataset("action_raw", data=actions_tcp_raw.astype(np.float32))

            # Creating FTS dataset
            f.create_dataset("fts_info", data=fts_info.astype(np.float32))

        print("Metadata save complete.")

        # === SAVE TO NPZ (optional legacy format) ===
        print(f"Also saving legacy .npz to {npz_path}")
        np.savez_compressed(
            npz_path,
            final_pose=np.array(self.pose),
            pose_history=np.array(self.pose_history),
            delta_history=np.array(self.delta_history),
            timestamps=np.array(self.timestamp_history),
            frequency_history=np.array(self.freq_history),
            camera_frames=np.array(self.camera_frame_history, dtype=object),
            gripper_history=np.array(self.gripper_history)
        )
        print("NPZ save complete.")
        print(self.gripper_history)
        self.shutdown()

    def stop(self):
        self.camera_viewer.stop()
        cv2.destroyAllWindows()
        self.fts.stop()


    def shutdown(self):

        # For FTS
        self.has_shutdown = True

        # Stop RTDE control interface script execution
        if hasattr(self, 'rtde_c') and self.rtde_c is not None:
            try:
                self.rtde_c.stopScript()
            except Exception as e:
                print(f"Warning: could not stop RTDE script cleanly: {e}")
            self.rtde_c.disconnect()
            self.rtde_c = None

        # Stop RTDE receive interface
        if hasattr(self, 'rtde_r') and self.rtde_r is not None:
            self.rtde_r.disconnect()
            self.rtde_r = None

        # Disconnect gripper
        if hasattr(self, 'gripper'):
            try:
                self.gripper.disconnect()
            except Exception as e:
                print(f"Warning: failed to disconnect gripper: {e}")

        # Stop camera pipelines
        if hasattr(self, 'camera_viewer'):
            self.camera_viewer.stop()

        # Close all OpenCV windows
        cv2.destroyAllWindows()
        
        # Stop FTS conncetion
        self.fts.stop()

        print("Shutdown completed successfully.")
