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

## Code Style

**Black** is the formatter. Line length 88, Python 3.12 target. A pre-commit hook
runs Black automatically on every `git commit` — no manual step required.

To format manually:
```bash
uv run black .
```

Do not argue with Black's output. If a line looks odd after formatting, that is
the canonical style.

---

## Dependency Management

Use `uv` exclusively.
- Add a package: `uv add <package>`
- Run the project: `uv run main.py`
- Run tests: `uv run pytest`
- Never edit the `[project.dependencies]` section of `pyproject.toml` manually.

The MAE force-torque SDK (`mae-fts-sdk`, `mae-sdk`) is vendored in
`legacy/robot_env/mae_sdk_sensureal/` and installed as a local path dependency
via `[tool.uv.sources]` in `pyproject.toml`. Import it normally:
`from mae_fts_sdk import ...`

---

## Testing Standards

### Test categories and markers

Four categories of test exist, each with a pytest marker:

| Marker | Requires | Location |
|---|---|---|
| *(none)* | Nothing — pure logic, no I/O | `tests/` |
| `sim` | URSim running via Docker | `tests/` |
| `hardware` | Physical device connected | `tests/hardware/` |
| `interactive` | Human operating the device | `tests/hardware/` |

Run selectively:
```bash
uv run pytest                          # unit tests only (no markers)
uv run pytest -m sim                   # URSim tests
uv run pytest -m hardware              # all hardware tests
uv run pytest -m "hardware and not interactive"  # automated hardware only
```

### Unit tests — `tests/`

Four files covering the logic layers that can be exercised without hardware:

| File | What it tests |
|---|---|
| `test_haptic_decoding.py` | Haply WebSocket JSON → `HapticState`; delta computation |
| `test_fts_transform.py` | FTS coordinate frame transform (Rodrigues formula) |
| `test_ur_commands.py` | `URArm` driver in isolation against URSim (`@pytest.mark.sim`) |
| `test_env_step.py` | Full `env.step()` against URSim with FTS/cameras stubbed (`@pytest.mark.sim`) |

`test_ur_commands.py` uses URSim rather than mocks. The scope distinction between
it and `test_env_step.py` is: failure in `test_ur_commands` points to the `URArm`
driver; failure in `test_env_step` could be anywhere in `RobotEnv`.

### Hardware tests — `tests/hardware/`

Peripheral smoke tests run against real hardware with the robot arm **not enabled**.
Each file targets one device in isolation:

**`test_fts_hardware.py`**
- UDP connection and initialisation (TRANSDUCER_SET + BIAS_SET)
- Noise floor: 200 samples at rest, all channels within ±0.3 N / ±0.05 Nm
- Data rate: 5-second stream asserted within ±10% of `FTS_HZ`
- Coordinate transform: hand-apply a downward push, assert dominant TCP-frame Z force

**`test_gripper_hardware.py`**
- TCP connection to port 63352 and activation cycle
- Open/close cycle × 3 with `is_open()` assertion each way
- Partial position: command 128/255, assert readback within ±10 counts
- Speed/force parameter range: min and max values accepted without fault codes

**`test_haply_hardware.py`**
- WebSocket connection and first valid JSON within 2 seconds
- Data rate: >50 Hz over 5-second window
- Button detection: operator presses A then B; rising edge asserted per button (`@pytest.mark.interactive`)
- Workspace coverage: operator moves device for 10 s; position spans >50% of expected range (`@pytest.mark.interactive`)
- Force feedback send: 0.5 N constant force command; no WebSocket error raised
- Quaternion validity: `|q| ≈ 1.0` (within 0.01) over 200 samples

### conftest.py conventions

`tests/hardware/conftest.py` provides a shared fixture that:
- Skips the entire hardware suite gracefully (not fails) if the target device is
  unreachable, rather than producing cryptic timeout errors.
- Asserts `SIM_MODE = False` and raises a clear error if someone accidentally
  runs hardware tests against the sim configuration.
- Prints the measured data rate and any dropped-packet warnings to stdout so they
  are visible in the pytest summary even on a passing run.

---

## Constraints

- **Python 3.12** — pinned in `.python-version`. `ur-rtde` requires a C++ build
  with Boost and has no Python 3.14 wheel. Do not bump the version without
  verifying all compiled packages resolve cleanly.
- **Camera serial numbers** in `config.py` are tied to specific physical cameras.
  Do not change without verifying with `rs-enumerate-devices`.
- **`servoL` must be called from `ServoStreamer`** at `SERVO_HZ`, not from the
  control loop. Calling it ad-hoc at low frequency causes jerky robot motion.
- **Never commit to `Collected_Data/`** — robot data is gitignored by design.
