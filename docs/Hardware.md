# Hardware Architecture & Protocols

The robot manipulation pipeline orchestrates communication across five distinct hardware systems. This document details the exact protocols, libraries, and critical Python implementation lines used to interface with the hardware.

## 1. Haptic Controller (Haply Inverse3 & Verse Grip)

The Haply Inverse3 is a 3-DOF force-feedback arm, and the Verse Grip provides user orientation and button inputs.

*   **Protocol:** Local WebSockets over TCP (`ws://localhost:10001`).
*   **Library:** Standard Python `websockets` (specifically `websockets.sync.client`) and `orjson` for fast JSON encoding/decoding.
*   **Data Structure:** JSON objects containing `cursor_position`, `cursor_velocity`, `orientation`, and `buttons`.

### Implementation Details
The communication is handled in `haply_data_collect_july23/teleop_controls/haply_reader/reader.py` inside the `HapticReader` class.

**Connecting and Receiving Data:**
```python
# Line 165: Establish WebSocket connection
with connect(self.uri) as ws:

# Line 173: Poll the WebSocket for the hardware state JSON
response = ws.recv()
data = orjson.loads(response)
```

**Sending Force Feedback:**
To apply physical force back to the human operator's hand, a JSON command is built and dispatched:
```python
# Lines 245-258: Format and send the force feedback instruction
request_msg = {
    "inverse3": [
        {
            "device_id": inverse3_device_id,
            "commands": {
                "set_cursor_force": {
                    "values": self.current_force # {"x": fx, "y": fy, "z": fz}
                }
            }
        }
    ]
}
ws.send(orjson.dumps(request_msg))
```

## 2. Universal Robots (UR) Arm

The primary manipulator execution hardware.

*   **Protocol:** Real-Time Data Exchange (RTDE) over TCP/IP (Operating at 500Hz).
*   **Library:** `ur_rtde` (A 3rd-party C++/Python library specifically designed for UR robots).

### Implementation Details
The communication is handled in `robot_env/RTDE_RW_test_collect.py` inside the `RobotAction` class.

**Initialization:**
```python
# Lines 196-197: Setup Receive and Control RTDE interfaces
self.rtde_r = RTDEReceive(self.robot_ip, rtde_frequency, [], True, False, rt_receive_priority)
self.rtde_c = RTDEControl(self.robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)
```

**Reading Robot State:**
```python
# Line 301: Get the 6D Tool Center Point (TCP) Pose [x, y, z, rx, ry, rz]
self.pose = np.array(self.rtde_r.getActualTCPPose())

# Line 342: Get the physical rotation of all 6 joints
joints = np.array(self.rtde_r.getActualQ())
```

**Executing Movement:**
Cartesian movements are sent continuously using `servoL`.
```python
# Line 322: Stream Cartesian target waypoints to the robot's servo controller
self.rtde_c.servoL(self.target, velocity, accel, dt, lookahead_time, gain)
```

## 3. Robotiq Gripper

The end-effector attached to the UR Arm.

*   **Protocol:** Direct TCP Socket Connection routed through the UR controller's IP address on port `63352`.
*   **Library:** Custom `socket` wrapper inside `robotiq_gripper.py`.
*   **Data Structure:** Proprietary ASCII string commands (e.g., `ACT`, `GTO`, `POS`, `SPE`, `FOR`).

### Implementation Details
The communication is instantiated in `robot_env/RTDE_RW_test_collect.py`.

**Connecting:**
```python
# Line 201: Establish TCP socket connection
self.gripper.connect(self.robot_ip, 63352)
```

**Sending Gripper Commands:**
Inside the `robotiq_gripper.py` wrapper, standard Python `socket` sends byte-encoded strings. The main script triggers this via:
```python
# Line 325: Command gripper position (0-255), speed (0-255), and force (0-255)
self.gripper.move(gripper_value, speed, force)
```

## 4. Force-Torque Sensor (Sensureal MAE)

A sensor mounted at the robot wrist to measure interaction forces.

*   **Protocol:** Custom UDP Socket communication on Port `10547` to IP `192.168.1.11`.
*   **Library:** Custom embedded MAE SDK (`mae_fts_sdk`).
*   **Data Structure:** Byte arrays mapped via Enum configurations. Returns 6D vector `[Fx, Fy, Fz, Tx, Ty, Tz]`.

### Implementation Details
The communication is managed by `ForceTorqueSensor` inside `robot_env/RTDE_RW_test_collect.py` and routed through `fts_udp.py`.

**Connecting:**
```python
# Lines 124-129: Initialize UDP Socket wrapper
self.communication_interface = mae.UdpCommunication(
    ip_address=fts_ip,
    port=fts_port,
    timeout_sec=1,
    log_level=logging.WARNING,
)
```

**Sending Requests and Reading Data:**
```python
# Line 143: Send an Enum-mapped byte command over UDP
self.communication_interface.send_request(command)

# Line 152: Block and poll the socket for the response byte stream
byte_stream = self.communication_interface.waits_response_bytes()
```

## 5. Camera System (Intel RealSense D405)

Two cameras record the teleoperation session: one on the robot's wrist and one from the operator's point of view.

*   **Protocol:** USB Video Class (UVC) via proprietary RealSense pipelines.
*   **Library:** Official Intel RealSense SDK `pyrealsense2` and OpenCV `cv2`.

### Implementation Details
The communication is handled by `MultiCameraViewer` inside `robot_env/RTDE_RW_test_collect.py`.

**Initialization:**
```python
# Lines 34-46: Configure and start the hardware pipelines targeting specific serial numbers
self.pipeline_robot = rs.pipeline()
self.config_robot = rs.config()
self.config_robot.enable_device('218622276856')
self.config_robot.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
self.pipeline_robot.start(self.config_robot)
```

**Reading Frames:**
Frames are pulled via hardware blocking calls to ensure temporal sync.
```python
# Line 60: Wait for the hardware to deliver the next frame buffer
robot_frames = self.pipeline_robot.wait_for_frames()

# Line 61-62: Extract RGB data directly into a NumPy array
robot_color = robot_frames.get_color_frame()
robot_image = np.asanyarray(robot_color.get_data())
```