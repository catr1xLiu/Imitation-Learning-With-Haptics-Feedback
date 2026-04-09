"""
hardware/cameras.py — Intel RealSense D405 dual-camera driver.

Manages two RealSense pipelines (wrist camera and user POV camera) on separate
background threads. Each thread calls wait_for_frames() in a tight loop and
writes the latest frame into a shared, locked CameraState. The control loop
reads the most recent frame without blocking — fixing the critical timing issue
in the original codebase where two sequential wait_for_frames() calls consumed
up to 133 ms inside the main control loop.

In simulation mode (config.SIM_MODE = True), the driver is instantiated in stub
mode: background threads publish blank numpy arrays of the correct shape at
config.CAMERA_HZ without initialising any RealSense hardware.

Reference: legacy/robot_env/RTDE_RW_test_collect.py
    MultiCameraViewer class:         lines 29–114
    Blocking frame reads (the bug):  lines 60, 91
"""
