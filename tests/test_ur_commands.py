"""
tests/test_ur_commands.py — Unit tests for UR arm command formatting.

Tests that URArm.set_target() correctly formats a Cartesian delta into a
servoL call with the expected arguments, and that the ServoStreamer thread
respects SERVO_HZ timing.

RTDEControl and RTDEReceive are replaced with unittest.mock.MagicMock objects —
no RTDE connection or URSim instance is required for these tests.

Reference: legacy/robot_env/RTDE_RW_test_collect.py
    update_commands / servoL:  lines 310–322
"""
