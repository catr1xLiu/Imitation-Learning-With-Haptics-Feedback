"""
data/recorder.py — HDF5 episode recorder.

Accepts StepSnapshot objects from the control loop (via a queue) and writes
them to disk in HDF5 format. Writing is asynchronous: a background thread drains
the queue so that disk I/O does not stall the control loop.

Output structure mirrors the legacy format for compatibility with existing
data-loading pipelines:
    episode_XXXXX.h5
        robot_state         float32 [steps, 15]
        action              float32 [steps, 7]
        action_tcp          float32 [steps, 7]
        fts_info            float32 [steps, 6]
        image_filenames     str     [steps]
        wrist_image_filenames str   [steps]
    episode_XXXXX_images/
        wrist_XXXX.jpg
        user_XXXX.jpg

Reference: legacy/robot_env/RTDE_RW_test_collect.py  RobotAction.save_data()  lines 541–651
"""
