# Plan: Hardware Communication Infrastructure

## Context

All hardware driver files, `config.py`, and `data/snapshot.py` are currently docstring-only stubs.
The architecture is fully specified in `docs/CodebaseStatusSummary.md`. The goal is to implement
all five hardware drivers, the shared state dataclasses, constants, and the full test suite — from
pure unit tests through URSim integration tests to physical hardware smoke tests (robot arm disabled).

---

## Implementation Order

### 1. `config.py` — All constants

Add every constant referenced in the codebase (no magic values elsewhere):

```python
SIM_MODE = True              # flip to False for real hardware
SIM_ROBOT_IP = "127.0.0.1"

ROBOT_IP = "192.168.0.110"
FTS_IP = "192.168.1.11"
FTS_PORT = 10547
CAMERA_SERIAL_WRIST = "218622276856"
CAMERA_SERIAL_USER  = "128422270567"
HAPLY_URI = "ws://localhost:10001"

CONTROL_HZ   = 30
SERVO_HZ     = 100
POSE_POLL_HZ = 500
CAMERA_HZ    = 15
FTS_HZ       = 100

POS_GAIN        = 1.5
SERVO_LOOKAHEAD = 0.1
SERVO_GAIN      = 2000

OUTPUT_DIR = "Collected_Data"
```

### 2. `hardware/_base.py` — HardwareDriver ABC

```python
from abc import ABC, abstractmethod

class HardwareDriver(ABC):
    @abstractmethod
    def connect(self) -> None: ...
    @abstractmethod
    def disconnect(self) -> None: ...
    @abstractmethod
    def is_connected(self) -> bool: ...
```

### 3. `data/snapshot.py` — State dataclasses + StepSnapshot

Six dataclasses. All timestamps use `time.perf_counter()`.

```python
@dataclass class RobotState:  joints(6), tcp_pose(6), timestamp
@dataclass class CameraState: frame_wrist(H,W,3), frame_user(H,W,3), timestamp
@dataclass class FTSState:    ft_vec(6), timestamp
@dataclass class HapticState: position(3), velocity(3), orientation(4), buttons: dict, timestamp
@dataclass class TargetPose:  pose(6), timestamp
@dataclass class StepSnapshot: control_ts, robot_state(15), robot_ts, camera_wrist(bytes),
                                camera_user(bytes), camera_ts, ft_vec(6), ft_ts, action(7)
```

### 4. `hardware/haptic.py` — HapticDevice

Key implementation details from `legacy/haply_data_collect_july23/teleop_controls/haply_reader/reader.py`:
- WebSocket URI from `config.HAPLY_URI` (`ws://localhost:10001`)
- `websockets.sync.client.connect()` in background thread; reconnect on disconnect
- JSON parsed with `orjson.loads()`: fields `inverse3[0].state.cursor_position/cursor_velocity`,
  `wireless_verse_grip[0].state.buttons.{a,b}`, `wireless_verse_grip[0].state.orientation`
- State stored as `HapticState`, protected by `threading.Lock`
- Force feedback: send `{"inverse3": [{"device_id": ..., "commands": {"set_cursor_force": {"values": {x,y,z}}}}]}`
- `SIM_MODE` does NOT disable haptic — device is always physical hardware, but force feedback is skipped in sim

### 5. `hardware/force_torque.py` — FTSensor

From `legacy/robot_env/RTDE_RW_test_collect.py` lines 120–163, 370–412:
- `mae.UdpCommunication(FTS_IP, FTS_PORT, timeout_sec=1)` from vendored SDK
- Init sequence: `TRANSDUCER_SET` → sleep 1s → `BIAS_SET` → sleep 1s → `SETTINGS_SAVE`
- Background thread: `STREAM_FT_START` → parse `waits_response_bytes()` → `FTSState`
- Coordinate transform (Rodrigues, legacy lines 370–412):
  ```
  raw → [fy, -fx, fz] → rotate by eef_rot_vec → [-fy_tcp, fx_tcp, fz_tcp]
  ```
- `SIM_MODE=True`: skip hardware init; thread returns zero `FTSState` at `FTS_HZ`

### 6. `hardware/cameras.py` — CameraSystem

From `legacy/robot_env/RTDE_RW_test_collect.py` lines 29–114:
- Two `rs.pipeline()` instances, one per serial number from `config.CAMERA_SERIAL_*`
- Each on its own background thread calling `pipeline.wait_for_frames()`; result written to locked `CameraState`
- `SIM_MODE=True`: no pipeline init; thread writes blank `np.zeros((H, W, 3), uint8)` at `CAMERA_HZ`

### 7. `hardware/gripper.py` — Gripper

From `legacy/robotiq_gripper.py` (TCP socket to `ROBOT_IP:63352`):
- TCP socket, activate, open/close via `POS`/`SPE`/`FOR`/`GTO`/`ACT` commands
- `SIM_MODE`: connects to `SIM_ROBOT_IP:63352` — code path identical, only IP changes
- Exposes: `open()`, `close()`, `move(position, speed, force)`, `is_open() -> bool`

### 8. `hardware/ur_arm.py` — URArm

From `legacy/robot_env/RTDE_RW_test_collect.py` lines 165–197, 298–322:
- **PosePoller thread** (`POSE_POLL_HZ=500 Hz`): `RTDEReceive(ip, 500.0)` → `getActualTCPPose()` + `getActualQ()` → `RobotState`
- **ServoStreamer thread** (`SERVO_HZ=100 Hz`): `RTDEControl(ip, 500.0, flags, ur_cap_port=50002)` → reads `TargetPose` → calls `servoL(pose, vel, acc, 1/SERVO_HZ, SERVO_LOOKAHEAD, SERVO_GAIN)`
- `SIM_MODE`: `ip = SIM_ROBOT_IP`; code identical

### 9. `pyproject.toml` — Register missing pytest markers

Add `hardware` and `interactive` markers to `[tool.pytest.ini_options]`:
```toml
markers = [
    "sim: tests requiring URSim (docker)",
    "hardware: tests requiring physical device connected",
    "interactive: tests requiring human operator",
]
```

---

## Test Implementation

### `tests/conftest.py` (new file)

```python
@pytest.fixture(autouse=True)
def require_sim(request):
    if request.node.get_closest_marker("sim"):
        # assert URSim reachable on SIM_ROBOT_IP:30001, else skip
```

### `tests/test_haptic_decoding.py` — Pure unit tests (no markers)

Implement with hard-coded JSON payloads — no hardware, no network:
1. Full valid message → `HapticState` fields correct (position, velocity, orientation, buttons)
2. Missing `wireless_verse_grip` key → no crash, buttons default to `{a: False, b: False}`
3. Button rising-edge detection: `a: False` → `a: True` asserts rising edge, not level
4. Delta computation: two consecutive `HapticState` positions → 3D positional delta matches expected

Reference: `legacy/.../reader.py` lines 175–219; `legacy/.../haply_barebones.py` lines 800–857

### `tests/test_fts_transform.py` — Pure unit tests (no markers)

1. Identity rotation (zero axis-angle) → raw frame values pass through with sign swap only
2. Known 90° rotation around Z → verify expected TCP-frame output
3. Pure X-force in raw frame → correct TCP-frame component (from legacy lines 370–412)
4. Torques pass through separately from forces

Reference: `legacy/robot_env/RTDE_RW_test_collect.py` lines 370–412

### `tests/test_ur_commands.py` — `@pytest.mark.sim`

Requires URSim container running. `SIM_MODE=True` set via module-level monkeypatch of `config`.
1. `URArm.connect()` completes without exception
2. `URArm.is_connected()` returns `True`
3. `PosePoller` produces `RobotState` within 50 ms of connect
4. `ServoStreamer` sends `servoL` calls at `SERVO_HZ ± 10%` (measure over 1 s)
5. `getActualTCPPose()` returns 6-element array with plausible home position
6. `URArm.disconnect()` cleans up; `is_connected()` returns `False`

### `tests/hardware/conftest.py` (new file)

Per CLAUDE.md conventions:
- `autouse` fixture asserts `SIM_MODE = False`, raises `RuntimeError` otherwise
- Per-device fixture that pings device IP; `pytest.skip()` (not fail) if unreachable
- Captures and prints measured data rate to stdout on pass

### `tests/hardware/test_fts_hardware.py` — `@pytest.mark.hardware`

1. UDP connect + TRANSDUCER_SET + BIAS_SET sequence completes in < 5 s
2. Noise floor: 200 samples at rest — all within ±0.3 N / ±0.05 Nm
3. Data rate: 5-second stream — within ±10% of `FTS_HZ`
4. Coordinate transform smoke: manually apply downward push → TCP-frame Z force dominant

### `tests/hardware/test_gripper_hardware.py` — `@pytest.mark.hardware`

1. TCP connect to port 63352 and activation cycle without fault
2. Open/close × 3 with `is_open()` assertion each way
3. Partial position: command 128/255, readback within ±10 counts
4. Speed/force range: min and max values accepted without fault codes

### `tests/hardware/test_haply_hardware.py` — mixed markers

- `@pytest.mark.hardware`: connection, data rate, force feedback send, quaternion validity
- `@pytest.mark.interactive`: button detection (A then B), workspace coverage (>50% range in 10 s)

1. WebSocket connect + first valid JSON within 2 s
2. Data rate: >50 Hz over 5-second window
3. `|q| ≈ 1.0` within 0.01 over 200 samples
4. Force feedback: send 0.5 N constant force — no WebSocket exception raised
5. *(interactive)* Button A rising edge asserted, then button B rising edge
6. *(interactive)* Move device 10 s; position spans >50% of expected workspace range

---

## Testing Sessions

### Session 1: Dry run (no hardware, current machine)

```bash
uv run pytest                   # unit tests only — haptic_decoding + fts_transform
```

All tests should pass without any hardware or Docker. If failures occur, iterate on the
pure-logic implementation (JSON parsing, Rodrigues math) before moving forward.

### Session 2: URSim integration (Docker on this machine)

```bash
# Terminal 1 — start URSim
docker run --rm -it \
  -p 5900:5900 -p 29999:29999 -p 30001-30004:30001-30004 \
  universalrobots/ursim_e-series

# Terminal 2 — confirm SIM_MODE=True in config.py, then:
uv run pytest -m sim
```

Tests: `test_ur_commands.py`. Validates RTDE connect/disconnect, PosePoller latency,
ServoStreamer call rate, and `servoL` acceptance before any physical arm is involved.

### Session 3: Hardware smoke tests (devices connected, robot arm disabled, ≤ 2 h)

Set `SIM_MODE = False` in `config.py`. Robot arm stays powered off or e-stopped throughout.

```bash
# Automated hardware tests (no interactive required)
uv run pytest -m "hardware and not interactive"

# Then interactive tests (operator at Haply device)
uv run pytest -m interactive
```

**Order within session:**
1. FTS tests (~10 min) — UDP connection, noise floor, data rate, manual push
2. Gripper tests (~15 min) — TCP connect, open/close cycle, position/speed/force params
3. Haply automated tests (~10 min) — WebSocket, data rate, quaternion, force feedback
4. Haply interactive tests (~20 min) — button edges, workspace coverage

---

## Critical Files

| File | Status | Action |
|---|---|---|
| `config.py` | stub | implement |
| `hardware/_base.py` | stub | implement |
| `data/snapshot.py` | stub | implement |
| `hardware/haptic.py` | stub | implement |
| `hardware/force_torque.py` | stub | implement |
| `hardware/cameras.py` | stub | implement |
| `hardware/gripper.py` | stub | implement |
| `hardware/ur_arm.py` | stub | implement |
| `pyproject.toml` | partial | add `hardware` + `interactive` markers |
| `tests/conftest.py` | missing | create |
| `tests/test_haptic_decoding.py` | stub | implement |
| `tests/test_fts_transform.py` | stub | implement |
| `tests/test_ur_commands.py` | stub | implement |
| `tests/hardware/conftest.py` | missing | create |
| `tests/hardware/test_fts_hardware.py` | missing | create |
| `tests/hardware/test_gripper_hardware.py` | missing | create |
| `tests/hardware/test_haply_hardware.py` | missing | create |

## Reference Files (read-only)

| Legacy File | Used For |
|---|---|
| `legacy/robot_env/RTDE_RW_test_collect.py` lines 120–197, 298–322, 370–412 | RTDE setup, FTS protocol, coord transform |
| `legacy/haply_data_collect_july23/teleop_controls/haply_reader/reader.py` lines 144–258 | WebSocket JSON format, force feedback |
| `legacy/haply_data_collect_july23/haply_barebones.py` lines 663+ | State management, delta computation |
| `legacy/robotiq_gripper.py` | Gripper TCP protocol |
| `legacy/robot_env/mae_sdk_sensureal/` | Vendored FTS SDK — import as `mae_fts_sdk` |
