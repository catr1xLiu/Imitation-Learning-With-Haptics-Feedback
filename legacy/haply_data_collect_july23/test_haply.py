import os
import time
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pynput import keyboard
from scipy.spatial.transform import Rotation as R
from teleop_controls.haply_reader.reader import HapticReader
from teleop_controls.misc.subprocess_utils import run_threaded_command
from teleop_controls.misc.transformations import quat_to_rmat, rmat_to_quat, quat_to_euler
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
tri_prism = None
ground_plane = None

# --- Coordinate Mapping ---
mapping = {
    "robot_x": "haply_x",
    "robot_y": "haply_y",
    "robot_z": "haply_z",
    "flip": [1, 1, 1],
    "quat_order": [3, 1, 2, 0]
}

# --- Configuration ---
CONTROL_HZ = 15
XYZ_SCALE = 1.5
ROT_SCALE = 1
LINE_LENGTH = 0.1

success = False
failure = False
reset_pose_key = 'r'
runtime_thread = None
running = True

# --- Visualization Setup ---
pos_history = []
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
sc = ax.scatter([], [], [], c='blue', marker='o')
line, = ax.plot([], [], [], 'r-', linewidth=2)
ax.set_xlim(-0.3, 0.3)
ax.set_ylim(-0.3, 0.3)
ax.set_zlim(-0.3, 0.3)
ax.set_xlabel('Y')
ax.set_ylabel('X')
ax.set_zlabel('Z')


def on_press(key):
    global success, failure
    try:
        if key.char == 's':
            success = True
        elif key.char == 'f':
            failure = True
        elif key.char == reset_pose_key:
            controller.reset_state()
            print("[INFO] Orientation reset to home pose")
    except AttributeError:
        pass


def on_release(key):
    global success, failure, running
    if key == keyboard.Key.esc:
        running = False
        plt.close()
        return False
    try:
        if key.char == 's':
            success = False
        elif key.char == 'f':
            failure = False
    except AttributeError:
        pass


def start_listener():
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()


class VRPolicy:
    def __init__(self):
        self.haply_reader = HapticReader()
        self.haply_to_global_mat = np.eye(4)
        self.pos_action_gain = XYZ_SCALE
        self.rot_action_gain = ROT_SCALE
        self.spatial_coeff = 1
        self.buttonb = False
        self.reset_state()
        run_threaded_command(self._update_internal_state)
        run_threaded_command(start_listener())
        self.calibrated = False
        self.awaiting_calibration = True
        print("[CALIBRATION] Place the pen upright at origin. Press ‘a’ to confirm and calibrate.")


    def reset_state(self):
        self._state = {
            "poses": {},
            "buttons": {'a': False, 'b': False, 'c': False},
            "movement_enabled": False,
            "controller_on": True,
        }
        self.reset_orientation = True
        self.reset_origin = False
        self.reset_orientation_button_state = False
        self.vr_state = None
        self.previous_button_a_state = False
        self.previous_button_b_state = False
        self.previous_button_c_state = False
        self.update_sensor = False

    def _trigger_reset_orientation_delay(self):
        time.sleep(3.5)
        self.reset_orientation_button_state = True

    def _update_internal_state(self, num_wait_sec=5, hz=100):
        last_read_time = time.time()
        while running:
            time.sleep(1 / hz)
            poses, buttons = self.haply_reader.get_device_state()
            self._state["controller_on"] = (time.time() - last_read_time) < num_wait_sec
            if poses == {}:
                continue

            current_a = buttons['a']
            if current_a and not self.previous_button_a_state:
                if self.awaiting_calibration:
                    print("[CALIBRATION] Calibrated! Starting teleop...")
                    try:
                        inv = np.linalg.inv(np.asarray(self._state["poses"]))
                    except:
                        inv = np.eye(4)
                        print("[WARN] Invalid pose during calibration")
                    self.haply_to_global_mat = inv
                    self.awaiting_calibration = False
                    self.calibrated = True
                    self._state["movement_enabled"] = True
                else:
                    self._state["movement_enabled"] = not self._state["movement_enabled"]
                    self.reset_origin = True

            self.previous_button_a_state = current_a


            current_b = buttons['b']
            if current_b and not self.previous_button_b_state:
                self.buttonb = not self.buttonb
            self.previous_button_b_state = current_b

            current_c = buttons['c']
            if current_c and not self.previous_button_c_state:
                self._trigger_reset_orientation_delay()
                self.previous_button_c_state = current_c
                continue
            if not current_c:
                self.reset_orientation_button_state = False
            self.previous_button_c_state = current_c

            self.reset_orientation = self.reset_orientation or self.reset_orientation_button_state
            quat = poses["orientation"]
            position = poses["position"]
            rot_mat = quat_to_rmat(quat)
            T = np.eye(4)
            T[:3, :3] = rot_mat
            T[:3, 3] = position
            self._state["poses"] = T.tolist()
            self._state["buttons"] = buttons
            last_read_time = time.time()
            if self._state["movement_enabled"]:
                self.update_sensor = True
            stop_updating = self.reset_orientation_button_state or self._state["movement_enabled"]
            if self.reset_orientation:
                try:
                    inv = np.linalg.inv(np.asarray(self._state["poses"]))
                except:
                    inv = np.eye(4)
                    self.reset_orientation = True
                self.haply_to_global_mat = inv
                if stop_updating:
                    self.reset_orientation = False
                    self.reset_orientation_button_state = False

    def _process_reading(self):
        T = np.asarray(self._state["poses"])
        T = self.haply_to_global_mat @ T

        pos = T[:3, 3]
        rot_mat = T[:3, :3]

        # Position remapping
        haply_axes = {"haply_x": pos[0], "haply_y": pos[1], "haply_z": pos[2]}
        new_pos = np.array([
            haply_axes[mapping["robot_x"]],
            haply_axes[mapping["robot_y"]],
            haply_axes[mapping["robot_z"]],
        ]) * np.array(mapping.get("flip", [1, 1, 1]))

        # Orientation remapping
        quat = rmat_to_quat(rot_mat)
        quat = np.array(quat)[mapping.get("quat_order", [0, 1, 2, 3])]
        rot_mat_mapped = quat_to_rmat(quat)

        self.vr_state = {
            "pos": self.spatial_coeff * new_pos,
            "quat": quat,
            "euler": quat_to_euler(quat),
            "rot_mat": rot_mat_mapped,
            "gripper": self.buttonb
        }

    def get_info(self):
        global success, failure
        return {
            "success": success,
            "failure": failure,
            "movement_enabled": self._state["movement_enabled"] if self.calibrated else False,
            "controller_on": self._state["controller_on"]
        }



    def forward(self):
        if not self.calibrated or self._state["poses"] == {}:
            return np.zeros(7)
        if self.update_sensor:
            self._process_reading()
            self.update_sensor = False
        if self.vr_state is None:
            return np.zeros(7)
        pos = self.vr_state["pos"] * self.pos_action_gain
        rot = self.vr_state["euler"] * self.rot_action_gain
        grip = [float(self.vr_state["gripper"])]
        return np.concatenate([pos, rot, grip])

def update_plot(frame):
    global tri_prism, ground_plane

    if controller.vr_state:
        pos = controller.vr_state["pos"]
        rot_mat = controller.vr_state["rot_mat"]
        pos_history.append(pos)
        if len(pos_history) > 50:
            pos_history.pop(0)
        positions = np.array(pos_history)
        sc._offsets3d = (positions[:, 0], positions[:, 1], positions[:, 2])

        # --- Draw triangular prism as stylus ---
        stylus_length = LINE_LENGTH
        stylus_radius = 0.01  # narrow base triangle

        tip = pos  # pen tip at end effector
        dir_z = rot_mat @ np.array([0, 0, stylus_length])  # forward
        dir_x = rot_mat @ np.array([stylus_radius, 0, 0])
        dir_y = rot_mat @ np.array([0, stylus_radius, 0])

        base1 = tip + dir_z + dir_x
        base2 = tip + dir_z - dir_x
        base3 = tip + dir_z + dir_y

        verts = [[tip, base1, base2], [tip, base2, base3], [tip, base3, base1], [base1, base2, base3]]

        if tri_prism:
            tri_prism.remove()
        tri_prism = Poly3DCollection(verts, color='orange', alpha=0.7)
        ax.add_collection3d(tri_prism)

        # --- Draw ground plane ---
        ground_center = np.array([0.0, 0.0, 0.0])
        ground_size = [0.3, 0.3, 0.002]  # width, depth, height (thin)

        gx, gy, gz = ground_size
        corners = np.array([
            [-gx / 2, -gy / 2, -gz / 2],
            [-gx / 2,  gy / 2, -gz / 2],
            [ gx / 2,  gy / 2, -gz / 2],
            [ gx / 2, -gy / 2, -gz / 2],
        ]) + ground_center

        ground_faces = [[corners[i] for i in [0, 1, 2, 3]]]

        if ground_plane:
            ground_plane.remove()
        ground_plane = Poly3DCollection(ground_faces, color='lightgray', alpha=0.3)
        ax.add_collection3d(ground_plane)

    return sc, tri_prism, ground_plane


def runtime_loop():
    while running:
        info = controller.get_info()
        if not info["controller_on"]:
            print("[WARN] Controller not connected")
            time.sleep(1.0)
            continue

        if info["movement_enabled"]:
            action = controller.forward()
            print("[ACTION]", np.round(action, 4))
        else:
            print("[INFO] Movement disabled. Press 'a' to enable.")
        time.sleep(1.0 / CONTROL_HZ)


if __name__ == "__main__":
    controller = VRPolicy()
    print("\n[INFO] Controller initialized.")
    print(" - Press 'a' to toggle movement")
    print(" - Press 's' for success, 'f' for failure")
    print(" - Press 'r' to reset orientation to home")
    print(" - Press 'esc' to quit\n")

    runtime_thread = threading.Thread(target=runtime_loop)
    runtime_thread.start()

    ani = FuncAnimation(fig, update_plot, interval=100)
    plt.show()
    running = False
    runtime_thread.join()
    print("[INFO] Exited cleanly.")
