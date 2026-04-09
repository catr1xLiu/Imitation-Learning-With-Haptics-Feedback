import time
from collections import defaultdict
from copy import deepcopy
import os 
import pyrealsense2 as rs
import cv2
import numpy as np
import PySpin
import csv
import datetime
import math
from ur10e_tele.trajectory_utils.trajectory_writer_test_collect import TrajectoryWriter
from teleop_controls.misc.time import time_ms
from controllers.haplybooleanGripper import VRPolicy
from ur10e_tele.RTDE_RW_test_collect import RobotAction

ENABLE_ROBOT = False  # Set False to disable actuation and print actions only

def horizon_reset(env, controller, control_hz):
    if ENABLE_ROBOT:
        env.close()
        env = RobotAction(control_hz=control_hz)
        env.reset()
    controller = VRPolicy()
    print("horizon_reset env.reset")
    return env, controller

def collect_trajectory(
    env=False,
    controller=None,
    eps_horizon=None,
    save_filepath="/home/demoaccount/Data/demoaccount2/haply_data_collect/outputs",
    task="",
    save_data=False,
    save_hz=5,
    save_images=False,
    control_hz=5,
    reset_robot=True,
):
    assert controller is not None

    controller.reset_state()
    if ENABLE_ROBOT and reset_robot:
        env.reset()

    num_eps = 0
    step_counter = 0
    episode_name = os.path.join(save_filepath, f"episode_{num_eps}.h5")

    if save_data:
        traj_writer = TrajectoryWriter(episode_name, save_images=save_images)
        save_step = int(env.control_hz / save_hz)

    while True:
        controller_info = controller.get_info()
        if not controller_info["movement_enabled"]:
            continue

        step_counter += 1
        control_timestamps = {"step_start": time_ms()}
        obs = env.get_observation() if ENABLE_ROBOT else {}
        obs["controller_info"] = controller_info

        control_timestamps["policy_start"] = time_ms()
        action = controller.forward(obs)

        control_timestamps["sleep_start"] = time_ms()
        comp_time = time_ms() - control_timestamps["step_start"]
        sleep_left = (1000 / control_hz) - comp_time
        if sleep_left > 0:
            time.sleep(sleep_left / 1000)

        control_timestamps["control_start"] = time_ms()
        if ENABLE_ROBOT:
            env.send_action(action)
        else:
            os.system("clear")
            print("Action:", np.round(action, 4))

        if save_data and step_counter % save_step == 0:
            control_timestamps["step_end"] = time_ms()
            obs["timestamps"] = control_timestamps
            traj_writer.write_timestep(obs, action, task)

        if (controller_info["success"] or controller_info["failure"]) and num_eps <= eps_horizon:
            print("saving data")
            success_flag = False if controller_info["failure"] else True
            traj_writer.write_episode_info(task, success_flag, num_eps)
            traj_writer.close()
            print("traj writer close")

            env, controller = horizon_reset(env, controller, control_hz)
            print("reset env to default")

            num_eps += 1
            if num_eps < eps_horizon:
                episode_name = os.path.join(save_filepath, f"episode_{num_eps}.h5")
                traj_writer = TrajectoryWriter(episode_name, save_images=save_images)
                print(f"Data starts to save in new hdf5 file: {episode_name}")

        end_traj = eps_horizon is not None and num_eps >= eps_horizon
        if cv2.waitKey(1) == 27 or end_traj:
            print("Exiting: 'Esc' key pressed or trajectory completed.")
            if ENABLE_ROBOT:
                env.close()
            return
