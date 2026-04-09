#!/usr/bin/env python
"""
This example demonstrates how to send a force to the end effector.
"""

__author__ = "Antoine Weill--Duflos"
__copyright__ = "Copyright 2023, HaplyRobotics"

import HaplyHardwareAPI
import usb.core
import time

# Find and reset USB device
dev = usb.core.find(idVendor=0x16c0, idProduct=0x0483)
if dev is None:
    raise ValueError('Device not Found')

dev.reset()
print("Device reset successfully.")
time.sleep(1)

# Detect and connect to Inverse3 device
connected_devices = HaplyHardwareAPI.detect_inverse3s()
com_stream = HaplyHardwareAPI.SerialStream(connected_devices[0])
inverse3 = HaplyHardwareAPI.Inverse3(com_stream)

# Wake up device
response_to_wakeup = inverse3.device_wakeup_dict()
print("Connected to device {}".format(response_to_wakeup["device_id"]))

# Control loop
start_time = time.perf_counter()
loop_time = 0.01  # 10 ms loop time
forces = [0, 1, 0]  # 1N in the y direction

while True:
    position, velocity = inverse3.end_effector_force(forces)
    print("Position: {}".format(position))

    # Wait until loop_time has passed
    while time.perf_counter() - start_time < loop_time:
        pass
    start_time = time.perf_counter()
