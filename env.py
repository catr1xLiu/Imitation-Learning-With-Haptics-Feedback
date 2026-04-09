"""
env.py — RobotEnv orchestrator.

Composes the five hardware drivers (URArm, Gripper, CameraSystem, FTSensor,
HapticDevice) into a single environment interface used by the control loop in
main.py. Responsible for:

  - Connecting and disconnecting all drivers in the correct order.
  - Running the control loop step: read HapticState → compute target pose →
    write TargetPose → collect StepSnapshot → pass to Recorder.
  - Routing force feedback from FTSensor to HapticDevice (disabled in sim mode).
  - Delegating data serialisation to data/recorder.py.

The control loop never blocks on hardware I/O. All sensor state is read from
thread-safe dataclasses maintained by background threads inside each driver.
See docs/CodebaseStatusSummary.md Section 6.2 for the full threading diagram.

Reference: legacy/robot_env/RTDE_RW_test_collect.py (RobotAction.send_action)
"""
