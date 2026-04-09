"""
tests/test_ur_commands.py — Integration tests for the URArm driver in isolation.

Connects to URSim (the official UR Docker simulator) and exercises the URArm
driver directly, without the rest of RobotEnv. Tests cover:

  - RTDE connect / disconnect lifecycle
  - servoL acceptance: verifies the real RTDE stack accepts the formatted command
  - ServoStreamer timing: asserts calls land within tolerance of SERVO_HZ
  - State polling: getActualTCPPose and getActualQ return plausible values

Scope distinction vs test_env_step.py:
  This file tests URArm in isolation. If something fails here, the fault is in
  the arm driver specifically. test_env_step.py tests the full RobotEnv with all
  five drivers composed — a failure there could be anywhere in the stack.

Requirements:
  - URSim running locally via Docker (see CLAUDE.md for the start command)
  - config.SIM_MODE = True  (set automatically by the test fixture)

Run with:
  uv run pytest tests/test_ur_commands.py -m sim

Reference: legacy/robot_env/RTDE_RW_test_collect.py
    update_commands / servoL:  lines 310–322
    _update_actual_pose:       lines 298–301
"""
