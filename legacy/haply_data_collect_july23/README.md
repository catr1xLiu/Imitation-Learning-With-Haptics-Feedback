# UR10e Teleoperation & Data Collection with Haply Inverse 3

This project enables teleoperation of a UR10e robotic arm using an haply inverse 3 haptic device. It supports real-time control and data recording for a variety of manipulation tasks.

---

## 📁 Directory Structure

All scripts and utilities are organized to support:
- **teleop_controls** oculus reader (not used) repo and misc utils
- **ur10e_tele** working folder
- **camera_utils** contains camera wrapper
- **trajectory_utils** folder for data saving code

---

## 🚀 Quick Start

### Step 1: Configure Task Parameters

Edit the following constants in  
`ur10e_tele/collect_URrobot_trajectory_test.py`:

```python
task = "place the pink stuffed animal in the box"  # Change this as needed
task_folder = "stuffed_toy"                        # 'stuffed_toy' or 'pills'
control_hz = 15                                    # Control frequency
save_hz = 5                                        # Save frequency
eps_horizon = 5                                    # Episode horizon
right_controller = True                            # Set to False if using left hand
save_data = True                                   # Whether to save data
```

For demo purposes set save_data to true and control frequency to 500hz for minimal latency. (code can't actually run at 500hz without a realtime kernel but setting it at 500hz will make the code run as fast as possible)

For saving data keeping control_hz at 15 is recommneded. Save_hz should also be a multiple of your control frequency.
---

### Step 2: Run the Data Collection Script

From the **project root**, launch the script:

```bash
python ./ur10e_tele/collect_URrobot_trajectory_test.py
```

The program will:
- Start listening for haptic device input
- Record joint states, images, and metadata
- Save each session under the `outputs/{task_folder}/session_*` directory

After each episode - making sure movement is enabled:
- Press **S** on keyboard for a successful trial
- Press **F** on keyboard for a failed trial  
The program exits automatically after the desired number of episodes or if **CTRL+C** key is pressed.


If **CTRL+C** is pressed or the program crashes, remove the last potentially corrupted episode and adjust your eps_horizon value to complete the remaining data collection.


---

### Step 3: Validate Collected Data

Use the notebook below to inspect and validate recorded trajectories:

```bash
ur10e_tele/trajectory_utils/trajectory_reader_rtde.ipynb
```

---

### Step 4: Review Outputs

All data will be stored in:

```
outputs/{task_folder}/session_*/
```
---

## Notes

- Ensure the robot is properly connected and the haply inverse3 are active before starting.
    - the USB dongle for the pen may need to be connected to an extension cable for the connection to be read if the haptic device is placed further away from computer
    - run `systemctl restart haply-inverse-service.service` and `systemctl status haply-inverse-service.service` to check that the haply device is connected 
    - follow Haply Inverse3 guide [Here]https://docs.haply.co/docs/quick-start for calibration guide and more info on the haptic device
    - run `/home/demoaccount/Data/demoaccount2/haply_data_collect/teleop_controls/haply_reader/reader.py` to check that values are being read correctly from the pen and haply device
    - the light on the device should be green when running the program

## VersaGrip Pen Orientation Instructions

Follow the video demo to ensure the VersaGrip pen is correctly oriented at the beginning of each episode for intuitive operation.

### Resetting Orientation During a Trial

To recalibrate the pen’s orientation mid-trial:

1. Press button **A** to disable robot movement.
2. Reorient the VersaGrip pen as shown in the initial setup video.
3. Press and hold button **C** on the pen for **3.5 seconds**, or until a notice appears in your terminal indicating that orientation has been recalibrated.

## Walkthrough video link: https://drive.google.com/file/d/1Tnioaj8p4TGmFcqL6aXYD0sESuIxJJB4/view?usp=drive_link




systemctl restart haply-inverse-service.service && systemctl status haply-inverse-service.service

