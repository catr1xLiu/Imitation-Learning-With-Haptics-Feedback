"""
config.py — Central configuration for the robot manipulation system.

This is the single source of truth for every hardware address, device identifier,
timing parameter, control gain, and file path in the codebase. No other module
should hardcode any of these values.

Simulation mode:
    Set SIM_MODE = True to run against URSim (the official UR Docker simulator)
    instead of the physical robot. See CLAUDE.md for the Docker start command.
    URArm and Gripper connect to SIM_ROBOT_IP; FTSensor and CameraSystem are
    disabled / stubbed; force feedback to the Haply device is turned off.
"""
