# Codebase Status Summary

**Project:** Robot Teleoperation & Imitation Learning Data Collection  
**Date:** April 2026  
**Scope:** Architecture review covering code maintainability, run-loop correctness, and data quality implications

---

## 1. Overview

This document summarises the current state of the teleoperation codebase, identifying structural and technical issues that should be addressed before the system is used for large-scale data collection or extended further. The intent is not to criticise the original work — this code successfully runs a complex multi-hardware system, which is a genuine achievement — but to establish an honest baseline from which an improved version can be built.

The system coordinates five independent hardware devices: a Universal Robots arm (UR), a Robotiq gripper, two Intel RealSense cameras, a Sensureal force-torque sensor, and a Haply Inverse3 haptic controller. Getting all of these talking simultaneously is non-trivial. However, the way they are currently integrated creates problems that affect both the safety and quality of data collection.

---

## 2. Code Readability and Maintainability

### 2.1 God Class Architecture

The central file `robot_env/RTDE_RW_test_collect.py` contains a single class, `RobotAction`, that is responsible for nearly everything: connecting to the robot arm, operating the gripper, capturing camera frames, reading the force-torque sensor, recording all data to memory, and serialising that data to disk. This file is approximately 500 lines long.

In software engineering, this is known as a "God class" — a single unit that knows and does too much. The practical consequence is that the code is difficult to understand in isolation and difficult to test. If a developer wants to change how images are saved, they must navigate the same file that also controls the robot's servo commands. If a bug is introduced in the gripper logic, the camera capture and the FTS reading are also potentially affected, because they share state and run in the same code path.

A cleaner structure would isolate each hardware device into its own self-contained module, with a thin orchestration layer on top that composes them.

### 2.2 Substantial Volume of Dead Code

The file `haply_data_collect_july23/haply_barebones.py` — which contains the `HapticVisualizer` class used by the main entry point — begins with approximately **660 lines of entirely commented-out code**. This is two previous versions of the same class that were not removed as the code evolved.

Dead code creates a significant maintenance burden. It is unclear which version is canonical, whether the commented code contains logic that is still relevant, and whether any of it was intentionally preserved. A reader trying to understand the system must mentally filter out more than half of the file before reaching the actual implementation at line 663.

### 2.3 Multiple Duplicate Directories

The repository contains several directories that appear to be copies or earlier versions of the same code:

- `robot_env/RTDE_RW_test_collect.py`
- `haply_data_collect_july23/ur10e_tele/RTDE_RW_test_collect.py`
- `haply_data_collect_july23/robot_env_corrected/RTDE_RW_test_collect.py`

It is not immediately clear which version is authoritative. When a bug is fixed or a parameter changed in one copy, the others silently diverge. This is a common source of subtle errors.

### 2.4 Hardcoded Configuration Throughout Source Code

Robot IP addresses, camera serial numbers, file output paths, and tuning parameters are scattered directly in source code with no central configuration file:

```python
# robot_env/RTDE_RW_test_collect.py
self.robot_ip = "192.168.0.110"
self.config_robot.enable_device('218622276856')  # camera serial number
self.output_dir = "/media/Data/demoaccount2/robot_manipulation/Collected_Data/test_040226"
```

Changing the output directory for a new experiment, or connecting to a different robot, requires finding and editing values buried in the implementation. A configuration file or command-line arguments would isolate these decisions from the code itself.

### 2.5 Commented-Out Experimental Configurations

`move_to_start_position()` contains five different hardcoded start poses, all but one of which are commented out with labels like "WIPE START POSITION", "LIGHT START POSITION", and "Sample change Rz". This is a common and understandable pattern during active development, but it has the same problem as dead code: it is unclear what the current intended configuration is, and the file accumulates context that is increasingly difficult to reason about.

### 2.6 Magic Numbers

Throughout the codebase, numerical constants appear without explanation:

```python
self.rtde_c.servoL(self.target, velocity, accel, dt, 0.1, 2000)
#                                                      ^^^  ^^^^
#                                      lookahead_time (s)   gain (unitless)
```

These values have significant physical implications for robot behaviour but carry no documentation. A new engineer, or even the original author returning after several months, would need to consult the UR documentation to understand what these parameters do and whether the values are appropriate.

---

## 3. Run-Loop Correctness and Timing

This section requires some background for readers unfamiliar with real-time systems.

**Background — Blocking vs. Non-Blocking Operations:**  
In software, a *blocking* operation is one where the program stops and waits until the operation completes before doing anything else. A *non-blocking* operation returns immediately, and the result arrives later. When a control loop must run at a fixed rate (for example, 6 times per second), blocking operations inside that loop are dangerous: if one hardware device is slow to respond, the entire loop stalls, and the timing guarantee is broken.

**Background — Threads:**  
A *thread* is a way of running multiple tasks simultaneously within a single program. A well-designed control loop runs each hardware device on its own thread, so that a slow camera or network delay on one device does not stall the others. A shared piece of state accessed from two threads simultaneously requires a *lock* — a mechanism that ensures only one thread reads or writes the value at a time. Without a lock, the two threads can interfere with each other unpredictably, a condition called a *race condition*.

### 3.1 Camera Reads Block the Control Loop

**This is the most significant timing issue in the codebase.**

Both RealSense cameras are configured to run at 15 frames per second, meaning one new frame becomes available every ~66 milliseconds. The call `pipeline.wait_for_frames()` is a blocking call — it halts the entire thread until the next frame arrives from the hardware.

In `send_action()`, this call is made **twice, sequentially**, once for each camera:

```python
# robot_env/RTDE_RW_test_collect.py — inside send_action(), called every control step
robot_frames = self.pipeline_robot.wait_for_frames()   # blocks up to 66ms
# ...
user_frames = self.pipeline_user.wait_for_frames()     # blocks up to 66ms again
```

The control loop is configured to run at 6 Hz, meaning each step should complete in approximately 166 milliseconds. The camera reads alone can consume up to **133 milliseconds** of that budget, leaving only 33 milliseconds for robot communication, force-torque reads, data recording, and the sleep that maintains the target rate.

In practice, the actual step duration is determined by where in the camera frame cycle the call happens to land, making the timing of each iteration unpredictable. The control loop does not run at a stable 6 Hz; it runs at whatever rate the cameras allow.

The correct design runs each camera on its own dedicated thread, which continuously captures frames and writes them to a shared buffer. The control loop then reads the most recently captured frame without waiting, completing in a fraction of a millisecond.

### 3.2 Race Condition on Robot Pose

The robot's current TCP pose (its 6D position and orientation in space) is stored in `self.pose`. This value is updated by a background thread running at 30 Hz:

```python
def _update_actual_pose(self, hz_pose=30):
    while True:
        time.sleep(1 / hz_pose)
        self.pose = np.array(self.rtde_r.getActualTCPPose())
```

The main control loop reads `self.pose` to compute the next movement target. However, there is no lock protecting this variable. If the background thread writes a new pose value at exactly the same moment the main loop is reading it, the main loop may receive a partially overwritten value — a mix of the old and new pose. In Python, because of how NumPy arrays are updated, this is a realistic failure mode rather than a theoretical one.

Additionally, because this thread polls at 30 Hz, the pose used by the control loop may be up to **33 milliseconds stale** at any given moment. The robot arm is moving continuously; a 33ms lag in pose feedback directly affects the accuracy of position error correction.

### 3.3 Race Condition on Force-Torque Data

The force-torque sensor reading is stored in `self.ft_vec`. This value is written by the Haply WebSocket background thread and read by the main control loop:

```python
# Written in HapticReader.connect_and_read() — a background thread
force_vector = self.force_filter.feed(self.force_reader.get_fts_observation()[:3], ...)
# ...
self.ft_vec = ...  # set on RobotAction from the Haply thread

# Read in send_action() — the main thread
self.ft_history.append(np.array(self.ft_vec))
```

There is no lock on `self.ft_vec`. This is the same class of race condition as described above, with the added complication that the writer and reader are in completely separate modules.

### 3.4 Force-Torque Sensor Rate is Coupled to Haply WebSocket Rate

The force-torque sensor should operate on its own independent schedule. Currently, it is read inside the Haply device's WebSocket communication thread. This means the force-torque sensor is only polled as fast as the Haply device sends messages — if the Haply connection is slow or drops a packet, the force data also stalls. Conversely, a slow UDP response from the force-torque sensor will delay the Haply WebSocket loop, degrading haptic feedback to the operator.

These two hardware devices have no physical relationship and should not be coupled at the software level.

### 3.5 servoL Called at 6 Hz Against a 500 Hz Interface

**Background — servoL:**  
`servoL` is a UR-specific command that streams continuous Cartesian position targets to the robot's internal servo controller. It is designed to be called at a high, consistent rate — the UR documentation specifies it should be called at the same frequency as the RTDE interface (here configured to 500 Hz). When called at a high rate, the robot's internal controller can smoothly interpolate between targets. When called at a very low rate, the robot must make larger positional jumps between updates, resulting in jerky or imprecise motion.

The current code configures the RTDE interface at 500 Hz but only calls `servoL` once per control loop iteration, at **6 Hz**. The `lookahead_time` parameter partially mitigates this by allowing the robot controller to smooth its trajectory, but the fundamental constraint remains: the robot receives a new target only 6 times per second. For tasks requiring fine positional precision, this is a meaningful limitation.

### 3.6 No Per-Sensor Timestamps in the Recorded Dataset

When a dataset step is recorded, a single timestamp is stored for the entire step. There is no record of when each individual sensor reading was actually captured.

Consider what this means: the camera frame recorded at step N may have been captured up to 66 ms before the control command for step N was issued (because `wait_for_frames()` returns the next available frame, not a frame captured at that exact moment). The force-torque reading is whatever value was last written by the Haply thread, which could be arbitrarily stale. The robot pose is from the background thread, up to 33 ms old.

A policy trained on this dataset sees observations that appear temporally aligned but are not. For tasks where precise force feedback or visual timing matters, this misalignment degrades the quality of the training signal.

---

## 4. Summary of Issues

| Issue | Category | Severity |
|---|---|---|
| God class — `RobotAction` does everything | Architecture | High |
| ~660 lines of dead code in `haply_barebones.py` | Readability | Medium |
| Three versions of `RTDE_RW_test_collect.py` | Maintainability | Medium |
| Hardcoded paths, IPs, serial numbers in source | Configuration | Medium |
| Blocking camera reads in main control thread | Timing | **Critical** |
| Race condition on `self.pose` (no lock) | Correctness | High |
| Race condition on `self.ft_vec` (no lock) | Correctness | High |
| FTS coupled to Haply WebSocket thread | Architecture | High |
| `servoL` called at 6 Hz instead of ~100 Hz | Motion quality | Medium |
| No per-sensor timestamps in recorded data | Data quality | High |
| Magic numbers without documentation | Readability | Low |

---

## 5. Recommended Path Forward

The issues above share a common root cause: the codebase grew incrementally, with each hardware device added on top of an existing structure rather than designed as part of a coherent system. The recommended approach is a structured rewrite rather than continued patching of the current code.

The target architecture separates each hardware device into an independent driver module with a clean, documented interface. Each driver runs on its own thread at its native hardware rate. A thin orchestration layer composes the drivers and presents a unified environment interface to the control loop. This design directly resolves the blocking, race conditions, and coupling problems described above.

For simulation, the priority is validating the **communication protocol** with the robot, not physics. URSim — the official Universal Robots simulator distributed as a Docker image — runs the actual UR controller software and speaks the full RTDE protocol. The `ur_rtde` library connects to URSim identically to the real robot; no code changes are required, only an IP address change. This gives high confidence that RTDE command sequencing, `servoL` streaming, state polling, and error handling will behave correctly on hardware. Force-feedback is disabled in simulation mode since there is no physical contact.

The existing code remains a useful reference throughout this process — the hardware communication logic, coordinate frame transformations, and data serialisation format are all directly reusable.

---

## 6. Proposed Architecture

### 6.1 Directory Structure

```
hardware/
    ur_arm.py           # URArm driver — wraps ur_rtde
    gripper.py          # Gripper driver — wraps Robotiq TCP socket
    cameras.py          # CameraSystem driver — wraps two RealSense pipelines
    force_torque.py     # FTSensor driver — wraps MAE UDP SDK; supports disabled mode
    haptic.py           # HapticDevice driver — wraps Haply WebSocket
    _base.py            # Abstract base class shared by all drivers

env.py                  # RobotEnv — composes drivers, exposes step() interface
config.py               # All IPs, serial numbers, gains, paths, and SIM_MODE flag
run.py                  # Entry point — keyboard listener, session lifecycle

data/
    recorder.py         # Writes StepSnapshot objects to HDF5
    snapshot.py         # StepSnapshot dataclass definition

tests/
    test_haptic_decoding.py
    test_fts_transform.py
    test_ur_commands.py
    test_env_step.py    # Runs against URSim; FTS and cameras stubbed
```

### 6.2 Threading Model

Each hardware device runs on its own dedicated background thread operating at its native hardware rate. The control loop interacts only with in-memory state objects — it never blocks on hardware I/O.

```
┌─────────────────────────────────────────────────────────────────┐
│ Background Threads (always running during a session)            │
│                                                                 │
│  HapticDevice thread   ~100 Hz   ──► HapticState  (lock)       │
│  CameraSystem thread    15 Hz    ──► CameraState  (lock)       │
│  FTSensor thread       ~100 Hz   ──► FTSState     (lock)       │
│  PosePoller thread      500 Hz   ──► RobotState   (lock)       │
│  ServoStreamer thread   ~100 Hz   ◄── TargetPose   (lock)       │
└─────────────────────────────────────────────────────────────────┘
                               │ read/write via locks
┌─────────────────────────────────────────────────────────────────┐
│ Control Loop  (6–30 Hz, never blocks on hardware)               │
│                                                                 │
│  1. Read HapticState  → compute positional delta                │
│  2. Read RobotState   → apply error correction                  │
│  3. Write TargetPose  → ServoStreamer picks it up at 100 Hz     │
│  4. Send gripper command if state changed                       │
│  5. Read CameraState, FTSState, RobotState → build StepSnapshot │
│  6. Pass StepSnapshot to Recorder                               │
└─────────────────────────────────────────────────────────────────┘
```

**ServoStreamer** is a new thread not present in the current code. It reads the latest `TargetPose` and calls `servoL` at ~100 Hz, completely decoupled from the control loop rate. This allows the robot's internal servo controller to receive frequent, smooth position updates regardless of how fast the control loop is running.

### 6.3 Shared State and Locking

Each state object is a small dataclass carrying the latest sensor reading and a timestamp. Access is protected by a `threading.Lock`. Because reads are fast (copying a few floats), lock contention is negligible.

```python
@dataclass
class RobotState:
    joints: np.ndarray      # shape (6,)   — joint angles in radians
    tcp_pose: np.ndarray    # shape (6,)   — [x, y, z, rx, ry, rz]
    timestamp: float        # time.perf_counter() at moment of capture

@dataclass
class CameraState:
    frame_wrist: np.ndarray # shape (H, W, 3)
    frame_user: np.ndarray  # shape (H, W, 3)
    timestamp: float

@dataclass
class FTSState:
    ft_vec: np.ndarray      # shape (6,) — [Fx, Fy, Fz, Tx, Ty, Tz]
    timestamp: float

@dataclass
class HapticState:
    position: np.ndarray    # shape (3,)
    velocity: np.ndarray    # shape (3,)
    orientation: np.ndarray # shape (4,) quaternion
    buttons: dict
    timestamp: float
```

### 6.4 Step Snapshot — Explicit Temporal Alignment

Every recorded step carries the timestamp of each individual sensor reading, not a single shared timestamp. This makes temporal misalignment visible and measurable during analysis.

```python
@dataclass
class StepSnapshot:
    control_ts:   float          # when the control loop issued this step

    robot_state:  np.ndarray     # [joints(6), pos(3), quat(4), gripper(1), blocked(1)]
    robot_ts:     float

    camera_wrist: bytes          # JPEG-encoded
    camera_user:  bytes          # JPEG-encoded
    camera_ts:    float

    ft_vec:       np.ndarray     # [Fx, Fy, Fz, Tx, Ty, Tz]
    ft_ts:        float

    action:       np.ndarray     # [dx, dy, dz, drx, dry, drz, gripper]
```

### 6.5 Hardware Driver Interface

All five drivers implement a common base interface, which allows the orchestration layer (`RobotEnv`) and the test suite to treat real hardware and simulation backends identically.

```python
class HardwareDriver(ABC):
    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def is_connected(self) -> bool: ...
```

Each driver then extends this with device-specific methods:

```python
class URArm(HardwareDriver):
    def get_state(self) -> RobotState: ...          # reads from shared RobotState
    def set_target(self, pose: np.ndarray) -> None: # writes to shared TargetPose
    def move_to_joints(self, joints: np.ndarray) -> None:  # blocking moveJ for reset

class FTSensor(HardwareDriver):
    def get_state(self) -> FTSState: ...

class CameraSystem(HardwareDriver):
    def get_state(self) -> CameraState: ...

class HapticDevice(HardwareDriver):
    def get_state(self) -> HapticState: ...
    def send_force(self, force: np.ndarray) -> None: ...

class Gripper(HardwareDriver):
    def move(self, position: int, speed: int, force: int) -> None: ...
    def is_open(self) -> bool: ...
```

### 6.6 Simulation Mode — URSim

The simulation strategy prioritises validating the RTDE communication protocol over physics fidelity, since the system's primary purpose is real-robot data collection.

**What URSim is:** Universal Robots distributes an official Docker image that runs the actual UR controller software in a virtual machine. It speaks the complete RTDE protocol on the standard ports. The `ur_rtde` library connects to it identically to the real robot — the same `RTDEReceive`, `RTDEControl`, `servoL`, `getActualQ`, and `getActualTCPPose` calls, against the same port numbers, with the same timing behaviour.

**How simulation mode works** in this architecture is through a single `SIM_MODE` flag in `config.py`. `RobotEnv` reads this flag and adjusts how it instantiates each driver:

```
┌─────────────────────────────────────────────────────────────┐
│              SIM_MODE = False  (production)                 │
│                                                             │
│  URArm      → RTDE  → ROBOT_IP (192.168.0.110)             │
│  Gripper    → TCP   → ROBOT_IP port 63352                   │
│  CameraSystem → RealSense pipelines                         │
│  FTSensor   → UDP   → FTS_IP  (192.168.1.11)               │
│  HapticDevice → WebSocket → localhost:10001 (real device)   │
│  Force feedback → enabled                                   │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│              SIM_MODE = True   (development / testing)      │
│                                                             │
│  URArm      → RTDE  → SIM_ROBOT_IP (127.0.0.1 / URSim)    │
│  Gripper    → TCP   → SIM_ROBOT_IP port 63352               │
│  CameraSystem → stub (returns blank frames, no hardware)    │
│  FTSensor   → disabled (returns zeros, no UDP connection)   │
│  HapticDevice → WebSocket → localhost:10001 (real device)   │
│  Force feedback → disabled                                  │
└─────────────────────────────────────────────────────────────┘
```

The `URArm` and `Gripper` drivers are **identical** in both modes — the only difference is the IP they connect to. This means every RTDE command, every `servoL` call, every state poll, and every gripper command is exercised against the real UR controller stack before the physical robot is touched.

The `FTSensor` driver accepts a `disabled` flag. When disabled, its background thread publishes zero-valued `FTSState` readings at the configured rate without attempting a UDP connection. The control loop and recorder are unaware of the difference. Force feedback to the Haply device is gated off separately in `env.py` when `SIM_MODE` is set.

The `CameraSystem` driver accepts a `stub` flag. When stubbed, it returns blank frames of the correct shape and dtype at the configured rate. This is sufficient for validating the recording pipeline without physical cameras.

**Starting URSim:**
```bash
docker run --rm -it \
  -p 5900:5900 \
  -p 29999:29999 \
  -p 30001-30004:30001-30004 \
  universalrobots/ursim_e-series
```

Then set `SIM_MODE = True` and `SIM_ROBOT_IP = "127.0.0.1"` in `config.py` and run the system normally.

### 6.7 Configuration

All deployment-specific values are consolidated in `config.py`. No IP address, serial number, or file path appears anywhere else in the codebase.

```python
# config.py

# --- Simulation mode ---
SIM_MODE          = False          # Set True to run against URSim
SIM_ROBOT_IP      = "127.0.0.1"   # URSim via Docker on localhost

# --- Hardware IPs and identifiers ---
ROBOT_IP          = "192.168.0.110"
FTS_IP            = "192.168.1.11"
FTS_PORT          = 10547
CAMERA_SERIAL_WRIST = "218622276856"
CAMERA_SERIAL_USER  = "128422270567"
HAPLY_URI         = "ws://localhost:10001"

# --- Timing ---
CONTROL_HZ        = 30
SERVO_HZ          = 100
POSE_POLL_HZ      = 500
CAMERA_HZ         = 15
FTS_HZ            = 100

# --- Control gains ---
POS_GAIN          = 1.5
SERVO_LOOKAHEAD   = 0.1
SERVO_GAIN        = 2000

# --- Data ---
OUTPUT_DIR        = "/media/Data/collected"
```

### 6.8 Resolving Each Identified Issue

| Issue from Section 4 | Resolution in proposed architecture |
|---|---|
| God class `RobotAction` | Split into five driver classes + thin `RobotEnv` orchestrator |
| Dead and duplicate code | Single canonical implementation per device, no legacy copies retained |
| Hardcoded configuration | Centralised `config.py` |
| Blocking camera reads | `CameraSystem` thread captures continuously; control loop reads latest frame without waiting |
| Race conditions on shared state | All shared state protected by `threading.Lock` inside state dataclasses |
| FTS coupled to Haply thread | `FTSensor` runs on its own independent thread |
| `servoL` at 6 Hz | Dedicated `ServoStreamer` thread calls `servoL` at 100 Hz |
| No per-sensor timestamps | `StepSnapshot` records individual timestamp for each sensor |
| No simulation support | `SIM_MODE` flag: URArm/Gripper point at URSim IP; FTSensor disabled (returns zeros); CameraSystem stubbed; force feedback disabled |
