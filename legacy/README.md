# Robot Imitation Learning Data Collection

This repository contains a teleoperation and data collection suite designed for robot imitation learning. It allows a human operator to control a Universal Robots (UR) robotic arm using a Haply Inverse3 haptic device, while synchronously recording multimodal sensory data (RGB cameras, force-torque, robot kinematics, and operator actions). 

The generated data is packaged into HDF5 (`.h5`) format, optimized for training offline reinforcement learning, behavior cloning, or diffusion policies (e.g., Octo, RLDS frameworks).

## Code Structure

The codebase is modularized into three main functional domains:

1. **Main Controller (`run_haply.py`)**: 
   The entry point. It runs a state machine that listens for keyboard inputs (Start, Save, Discard, Quit) and manages the high-level application lifecycle and control loops.
2. **Environment & Hardware Aggregator (`robot_env/RTDE_RW_test_collect.py`)**:
   Contains the `RobotAction` class which acts as the central hub. It aggregates the UR arm (`ur_rtde`), Robotiq Gripper, Intel RealSense Cameras (`MultiCameraViewer`), and the Sensureal Force-Torque Sensor (`ForceTorqueSensor`). It also handles formatting and saving the dataset to disk.
3. **Teleoperation Interface (`haply_data_collect_july23/`)**:
   Handles the user input device. `haply_barebones.py` (`HapticVisualizer`) converts raw device positions into scaled Cartesian deltas. `reader.py` (`HapticReader`) manages the WebSocket communication with the Haply device to read state and optionally send force-feedback.

## Execution Flow

1. **Initialization:** The user runs `python run_haply.py`. The script starts a background keyboard listener and idles.
2. **Setup:** When the user presses `s` (Start), the `run_session` loop begins. It connects to the UR robot, Robotiq gripper, Realsense cameras, F/T sensor, and the Haply haptic device. The robot moves to a hardcoded starting pose.
3. **Control Loop (6 Hz Target Frequency):**
   * **Read User Input:** Queries the Haply device for 3D positional deltas, gripper toggles, and button states. (Note: Rotational deltas are intentionally zeroed out in `run_haply.py`).
   * **Apply Gains:** Multiplies the positional deltas by a configurable gain (e.g., `POS_GAIN = 1.5`) to map human workspace to robot workspace.
   * **Send Commands:** Dispatches the Cartesian target to the UR robot (`servoL`) and gripper target over TCP/IP.
   * **Record State:** Polls the robot's actual joint positions, TCP pose, Realsense RGB frames, and the Force-Torque sensor. Saves them into a RAM buffer.
4. **Session End:** The loop continues until the user presses `e` (Save) or `d` (Discard).
5. **Data Serialization:** If `e` was pressed, the script dumps the RAM buffer into the `Collected_Data/` directory.

## Data Flow & Formatting

During the control loop, data is synchronized and stored in memory. Upon saving, it writes to two formats simultaneously:

* **HDF5 (`.h5`)**: The primary data format used for machine learning.
  * `robot_state`: Array of shape `[steps, 15]` containing joints (6), TCP pos (3), TCP orientation quat (4), inverted gripper state (1), and blocked state (1).
  * `action`: Array containing joint deltas and gripper actions.
  * `fts_info`: Array of shape `[steps, 6]` containing transformed force `[Fx, Fy, Fz]` and raw torque `[Tx, Ty, Tz]`.
  * `image_filenames` & `wrist_image_filenames`: Metadata strings pointing to `.jpg` files saved to a subdirectory.
  * Images are stored directly as `.jpg` binaries inside an accompanying `episode_XXXXX_images/` folder to save space.
* **NPZ (`.npz`)**: A legacy NumPy compressed format containing raw history arrays (`pose_history`, `delta_history`, `timestamps`, etc.) for debugging or older pipelines.
