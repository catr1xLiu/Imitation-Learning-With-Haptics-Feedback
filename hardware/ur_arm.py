"""
hardware/ur_arm.py — Universal Robots arm driver.

Manages two RTDE interfaces to the UR controller:
  - RTDEReceive: polled at POSE_POLL_HZ (500 Hz) by a background thread.
    Provides joint angles and TCP pose, written into a locked RobotState.
  - RTDEControl: driven by the ServoStreamer background thread at SERVO_HZ
    (100 Hz) via servoL(). The control loop writes a TargetPose; ServoStreamer
    picks it up and streams it to the robot independently of the loop rate.

In simulation mode (config.SIM_MODE = True), both interfaces connect to
config.SIM_ROBOT_IP (URSim via Docker) instead of config.ROBOT_IP. The driver
code is identical in both modes — only the IP address changes.

Reference: legacy/robot_env/RTDE_RW_test_collect.py
    RTDEReceive / RTDEControl setup:  lines 196–197
    _update_actual_pose thread:       lines 298–301
    update_commands (servoL):         lines 310–322
"""
