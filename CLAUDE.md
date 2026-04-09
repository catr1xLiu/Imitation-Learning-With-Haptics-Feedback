# CLAUDE.md — Agent Instructions

## Project Overview

Teleoperation and imitation learning data collection system for a Universal Robots arm
controlled via a Haply Inverse3 haptic device. The system records multimodal demonstrations
(RGB cameras, force-torque, robot kinematics, operator actions) into HDF5 files for training
robot manipulation policies.

Architecture decisions and the rationale behind the rewrite are documented in
`docs/CodebaseStatusSummary.md`. Read that first to understand what problems this
codebase is solving and why the code is structured the way it is.

---

## Directory Structure

```
hardware/       New hardware drivers — one file per device (write here)
data/           StepSnapshot dataclass and HDF5 recorder (write here)
tests/          Unit and integration tests (write here)
docs/           Architecture and hardware documentation (read)
legacy/         Original codebase — READ-ONLY REFERENCE, never modify
main.py         Program entry point
env.py          RobotEnv orchestrator
config.py       Single source of truth for all settings
```

### legacy/ is read-only

`legacy/` contains the original student codebase. **Never modify files in legacy/.**
Read them to understand existing hardware communication logic, coordinate frame
transforms, and data format. Key files to reference:

| File | What it contains |
|---|---|
| `legacy/robot_env/RTDE_RW_test_collect.py` | RTDE communication, FTS reading, camera capture, HDF5 saving, coordinate transforms |
| `legacy/haply_data_collect_july23/teleop_controls/haply_reader/reader.py` | Haply WebSocket protocol, force filter implementation |
| `legacy/haply_data_collect_july23/haply_barebones.py` | HapticVisualizer — actual class starts at **line 663** (lines 1-662 are commented-out dead code) |
| `legacy/run_haply.py` | Original main loop and session lifecycle |

---

## Architecture Principles

### One thread per hardware device

Each driver in `hardware/` runs a background thread at the device's native rate.
The control loop in `env.py` reads from thread-safe state dataclasses and **never
blocks on hardware I/O**. See `docs/CodebaseStatusSummary.md` Section 6.2 for the
full threading diagram.

```
HapticDevice thread   ~100 Hz  →  HapticState  (threading.Lock)
CameraSystem thread    15 Hz   →  CameraState  (threading.Lock)
FTSensor thread       ~100 Hz  →  FTSState     (threading.Lock)
PosePoller thread      500 Hz  →  RobotState   (threading.Lock)
ServoStreamer thread  ~100 Hz  ←  TargetPose   (threading.Lock)

Control loop  (6–30 Hz) — reads all states, computes TargetPose, never waits
```

`ServoStreamer` calls `servoL` at `SERVO_HZ` (100 Hz) independently of the control
loop rate. This decouples teleoperation frequency from robot servo frequency.

### Simulation mode (URSim)

`config.py` has a `SIM_MODE` flag. When `True`, the system targets URSim (the official
UR Docker simulator) instead of real hardware. Start URSim with:

```bash
docker run --rm -it \
  -p 5900:5900 -p 29999:29999 -p 30001-30004:30001-30004 \
  universalrobots/ursim_e-series
```

Behaviour per driver in sim mode:

| Driver | SIM_MODE = False | SIM_MODE = True |
|---|---|---|
| URArm | RTDE → ROBOT_IP | RTDE → SIM_ROBOT_IP (127.0.0.1) |
| Gripper | TCP → ROBOT_IP:63352 | TCP → SIM_ROBOT_IP:63352 |
| FTSensor | UDP → FTS_IP | disabled — returns zeros |
| CameraSystem | RealSense pipelines | stubbed — returns blank frames |
| HapticDevice | WebSocket (real hardware) | WebSocket (real hardware, unchanged) |
| Force feedback | enabled | disabled |

The `URArm` and `Gripper` driver code is **identical** in both modes. Only the IP
changes. This validates the full RTDE protocol before touching physical hardware.

### Hardware driver interface

All drivers in `hardware/` inherit from `HardwareDriver` in `hardware/_base.py`.
The interface is:

```python
class HardwareDriver(ABC):
    def connect(self) -> None
    def disconnect(self) -> None
    def is_connected(self) -> bool
```

Each driver extends this with device-specific `get_state()` methods that return
the typed state dataclasses defined in `data/snapshot.py`.

### StepSnapshot — explicit temporal alignment

Every recorded step stores an individual timestamp per sensor, not one shared
timestamp. This makes sensor misalignment visible during analysis.

```python
@dataclass
class StepSnapshot:
    control_ts: float
    robot_state: np.ndarray;  robot_ts: float
    camera_wrist: bytes;      camera_user: bytes;  camera_ts: float
    ft_vec: np.ndarray;       ft_ts: float
    action: np.ndarray
```

---

## config.py is the only place for magic values

No IP address, serial number, file path, gain, or frequency appears anywhere
except `config.py`. If you need a new constant, add it there first.

---

## Dependency Management

Use `uv` exclusively.
- Add a package: `uv add <package>`
- Run the project: `uv run main.py`
- Run tests: `uv run pytest`
- Never edit the `[project.dependencies]` section of `pyproject.toml` manually.

The MAE force-torque SDK is vendored in `legacy/robot_env/mae_sdk_sensureal/`.
It is not a proper Python package — import it by ensuring `legacy/` is on `sys.path`
or via the path handling in `hardware/force_torque.py`.

---

## Constraints

- **Python 3.14** — check that `pyrealsense2` has 3.14 wheels before running on
  hardware. If not, pin to 3.12 in `.python-version` and `pyproject.toml`.
- **Camera serial numbers** in `config.py` are tied to specific physical cameras.
  Do not change without verifying with `rs-enumerate-devices`.
- **`servoL` must be called from `ServoStreamer`** at `SERVO_HZ`, not from the
  control loop. Calling it ad-hoc at low frequency causes jerky robot motion.
- **Never commit to `Collected_Data/`** — robot data is gitignored by design.
