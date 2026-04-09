"""
tests/test_fts_transform.py — Unit tests for force-torque sensor coordinate transform.

Tests that the raw [Fx, Fy, Fz, Tx, Ty, Tz] readings from the sensor frame are
correctly transformed to the robot TCP frame given a known TCP orientation.

Uses known input/output pairs derived from the Rodrigues' rotation formula
implementation in the legacy code. No hardware or UDP connection required.

Reference: legacy/robot_env/RTDE_RW_test_collect.py
    get_fts_observation():  lines 370–412
"""
