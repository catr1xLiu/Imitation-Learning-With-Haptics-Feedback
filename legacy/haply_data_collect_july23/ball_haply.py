#!/usr/bin/env python

"""This example demonstrates how to display a haptic ball with manual port scan"""

__author__ = "Antoine Weill--Duflos"
__copyright__ = "Copyright 2023, HaplyRobotics"

import HaplyHardwareAPI
import time
import math

# Attempt to connect manually to known ports
def find_working_port(possible_ports):
    for port in possible_ports:
        try:
            print(f"[INFO] Trying port: {port}")
            com_stream = HaplyHardwareAPI.SerialStream(port)
            inverse3 = HaplyHardwareAPI.Inverse3(com_stream)
            resp = inverse3.device_wakeup_dict()
            print(f"[SUCCESS] Connected to {port} with device ID: {resp['device_id']}")
            return inverse3
        except Exception as e:
            print(f"[WARN] Failed to connect on {port}: {e}")
    return None

# List of ports to try
ports_to_try = ["/dev/ttyACM0", "/dev/ttyACM1"]
inverse3 = find_working_port(ports_to_try)

if inverse3 is None:
    print("[ERROR] Could not connect to any device. Exiting.")
    exit(1)

start_time = time.perf_counter()
loop_time = 0.001  # 1ms loop time
forces = [0, 0, 0]


def force_sphere(sphere_center, sphere_radius, device_position, stiffness):
    distance = math.sqrt(
        sum([(device_position[i] - sphere_center[i]) ** 2 for i in range(3)])
    )
    if distance > sphere_radius:
        return [0, 0, 0]
    else:
        # Compute the normalized direction of the force
        direction = [
            (device_position[i] - sphere_center[i]) / sphere_radius
            for i in range(3)
        ]
        # Compute the force
        force = [
            direction[i] * (sphere_radius - distance) * stiffness for i in range(3)
        ]
        return force


# Main control loop
while True:
    print("[DEBUG] Calling end_effector_force...")
    position, velocity = inverse3.end_effector_force(forces)
    print("[DEBUG] Position:", position)
    forces = force_sphere([0, -0.14, 0.2], 0.08, position, stiffness=800)
    print("position:", position)

    while time.perf_counter() - start_time < loop_time:
        pass
    start_time = time.perf_counter()
