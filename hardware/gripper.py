"""
hardware/gripper.py — Robotiq gripper driver.

Wraps the Robotiq TCP socket protocol. The gripper communicates over a direct
TCP connection to the UR controller's IP on port 63352, using proprietary
ASCII string commands (ACT, GTO, POS, SPE, FOR).

In simulation mode (config.SIM_MODE = True), connects to config.SIM_ROBOT_IP.
URSim exposes the same port, so real gripper command sequences can be validated
against the simulator.

Reference: legacy/robotiq_gripper.py (socket wrapper)
           legacy/robot_env/RTDE_RW_test_collect.py lines 200–203, 324–325
"""
