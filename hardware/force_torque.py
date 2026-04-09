"""
hardware/force_torque.py — Sensureal MAE force-torque sensor driver.

Polls the FTS over UDP via the vendored MAE SDK at config.FTS_HZ. Each reading
is a 6D vector [Fx, Fy, Fz, Tx, Ty, Tz] in the sensor frame. The driver applies
a coordinate frame transform to express forces in the robot TCP frame before
writing to the shared FTSState.

The MAE SDK is located at legacy/robot_env/mae_sdk_sensureal/ and is added to
sys.path at import time. It is a proper Python package (has pyproject.toml) but
is vendored rather than installed, so it is not managed via uv.

In simulation mode (config.SIM_MODE = True), the driver is instantiated in
disabled mode: no UDP socket is opened, and the background thread publishes
zero-valued FTSState readings at config.FTS_HZ. Force feedback to the Haply
device is also disabled by env.py when SIM_MODE is set.

Reference: legacy/robot_env/RTDE_RW_test_collect.py
    ForceTorqueSensor class:         lines 120–163
    get_fts_observation (transform): lines 370–412
"""
