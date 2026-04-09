import time
import numpy as np
import math
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

ROBOT_IP = "192.168.0.110"  # Change to your UR10e's IP address
CONTROL_FREQUENCY = 125.0   # Hz
DT = 1.0 / CONTROL_FREQUENCY
NUM_STEPS = 100             # Number of steps in control loop
DELTA_Z = 0.001             # Each step: move +1mm in Z direction

# Connect to RTDE interfaces
rtde_c = RTDEControl(ROBOT_IP)
rtde_r = RTDEReceive(ROBOT_IP)

start_position = (
        math.radians(262.85),
        math.radians(-87.14),
        math.radians(111.61),
        math.radians(246.68),
        math.radians(-89.5),
        math.radians(-10.81)
        )

rtde_c.moveJ(start_position)

try:
    print("Starting servo loop...")

    # Start from current pose
    initial_pose = rtde_r.getActualTCPPose()
    target_pose = initial_pose.copy()

    # Start servo mode
    rtde_c.servoL(initial_pose, 0.5, 0.5, DT, 0.1, 300)

    for step in range(NUM_STEPS):
        start_time = time.time()

        # Incrementally update Z
        target_pose[2] += DELTA_Z

        # Send command
        #rtde_c.servoL(target_pose, a=0.5, v=0.25, t=DT, lookahead_time=0.1, gain=300)
        rtde_c.servoL(target_pose, 0.5, 0.25, DT, 0.1, 300)


        # Enforce control rate
        elapsed = time.time() - start_time
        if elapsed < DT:
            time.sleep(DT - elapsed)
        else:
            print(f"[WARN] Loop overran: {elapsed:.4f}s")

    # Hold final target pose for a short duration
    for _ in range(50):  # ~0.4 seconds at 125 Hz
        rtde_c.servoL(target_pose, 0.5, 0.25, DT, 0.1, 300)
        time.sleep(DT)
        
    print("Stopping servo...")
    rtde_c.servoStop()

    # Wait briefly for robot to stabilize
    time.sleep(0.5)

    # Read final pose from robot
    actual_pose = rtde_r.getActualTCPPose()

    # Compute and report pose error
    pos_error = np.array(actual_pose[:3]) - np.array(target_pose[:3])
    rot_error = np.array(actual_pose[3:]) - np.array(target_pose[3:])

    print("\n--- Pose Verification ---")
    print(f"Target Position (x,y,z): {target_pose[:3]}")
    print(f"Actual Position (x,y,z): {actual_pose[:3]}")
    print(f"Position Error (m): {pos_error}")
    print(f"Rotational Error (rad): {rot_error}")
    print("-------------------------\n")

finally:
    rtde_c.stopScript()
