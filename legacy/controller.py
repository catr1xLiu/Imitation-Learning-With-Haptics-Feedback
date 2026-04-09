import numpy as np
import sys
from robot_env.RTDE_RW_test_collect import RobotAction

class RobotArmController:
    def __init__(self, control_hz=10, time_step=0.2,
                 initial_pos=None, initial_ori=None):
        self.env = RobotAction(control_hz=control_hz)
        self.env.reset()
        self.time_step = time_step
        self.paused = False
        self.xmin = -.795
        self.xmax = .3
        self.ymin = 0.457
        self.ymax = 1.14
        self.zmin = 0.174     # optional if you want to check z axis limits
        self.control_hz = control_hz
        self.zmax = 0.95
        self.pre_gripper = 1
        self.limit_tolerance = 0.01
        # initialize pose
        if initial_pos is None:
            initial_pos = np.zeros(3)
        if initial_ori is None:
            initial_ori = np.zeros(3)
        self.curr_pos = np.array(initial_pos, dtype=float)
        self.last_pos = self.curr_pos.copy()
        self.curr_ori = np.array(initial_ori, dtype=float)
        self.last_ori = self.curr_ori.copy()

    def move_to_delta(self, tgt_pos, tgt_ori=[0 , 0 , 0], gripper=1):
        self.pre_gripper = gripper
        self.env.send_action(np.concatenate([tgt_pos, tgt_ori, [gripper]]), non_blocking=True, target_freq=self.control_hz)

    def jog(self, delta_pos):
        dp = np.array(delta_pos)*self.time_step
        self.curr_pos += dp
        self.env.send_action(np.concatenate([dp, np.zeros(3), [self.pre_gripper]]), non_blocking=True)
        return dp, np.zeros(3)

    def get_position_and_orientation(self):
        pos_list, ori_list, _ = self.env.get_observation_xyz()
        self.check_bounds_and_stop(pos_list)
        current_pose = np.concatenate([np.array(pos_list), np.array(ori_list)])
        delta_pose = np.zeros(6)
        return (delta_pose, current_pose)

    def pause_robot(self, flag):
        self.paused = flag

    def emergency_stop(self):
        print("Software Stop -> GUI Button/Keyboard/Bounds Triggered")

        self.pause_robot(True)
        self.env.send_action(np.zeros(7), non_blocking=True)
        sys.exit(0)

    def check_bounds_and_stop(self, pos_list):
        tol = self.limit_tolerance
        x, y, z = pos_list  # unpack the single position
        
        if x >= self.xmax - tol or x <= self.xmin + tol:
            print(f"{x} is out of bounds for [{self.xmin + tol},{self.xmax - tol}]")
            self.emergency_stop()
        elif y >= self.ymax - tol or y <= self.ymin + tol:
            print(f"{y} is out of bounds for [{self.ymin + tol},{self.ymax - tol}]")
            self.emergency_stop()
        elif z >= self.zmax or z <= self.zmin:
            print(f"{z} is out of bounds for [{self.zmin},{self.zmax}]")

            self.emergency_stop()
