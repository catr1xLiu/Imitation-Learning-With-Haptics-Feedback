"""
tests/test_env_step.py — Integration tests for a full control loop step.

Runs one complete env.step() cycle — from reading HapticState through issuing
servoL and collecting a StepSnapshot — against real RTDE protocol endpoints.

Requirements:
    - URSim running locally via Docker (see CLAUDE.md for the start command)
    - config.SIM_MODE = True  (set automatically by the test fixture)
    - FTSensor and CameraSystem are automatically stubbed in sim mode

These tests validate that the RTDE command sequence, state polling, and data
recording pipeline work correctly before the system is used with the physical
robot. They do NOT test physics or motion accuracy.
"""
