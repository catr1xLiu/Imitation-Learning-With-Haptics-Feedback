"""
data/snapshot.py — StepSnapshot dataclass.

A StepSnapshot captures the complete state of the system at one control step.
Each sensor carries its own timestamp (not a single shared timestamp) so that
temporal misalignment between sensors is explicit and measurable during analysis.

Fields:
    control_ts      float       time.perf_counter() when the control loop ran
    robot_state     ndarray     [joints(6), pos(3), quat(4), gripper(1), blocked(1)]
    robot_ts        float       timestamp of the robot state reading
    camera_wrist    bytes       JPEG-encoded wrist camera frame
    camera_user     bytes       JPEG-encoded user POV camera frame
    camera_ts       float       timestamp of the camera frame capture
    ft_vec          ndarray     [Fx, Fy, Fz, Tx, Ty, Tz] in TCP frame (zeros in sim)
    ft_ts           float       timestamp of the FTS reading
    action          ndarray     [dx, dy, dz, drx, dry, drz, gripper] — commanded delta

Reference: docs/CodebaseStatusSummary.md Section 6.4
"""
