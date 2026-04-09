

import h5py
import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
import time
import os

# === CONFIG ===
USE_RAW_IMAGES = False
episode_id = 1
base_dir = "/media/Data/demoaccount2/robot_manipulation/data_with_fts_2/train"
#base_dir = "/media/Data/demoaccount2/robot_manipulation/wipe_yellow_cloth_HAPLY/train/"
h5_file = f"{base_dir}/episode_{episode_id:05d}.h5"
npz_file = f"/media/Data/demoaccount2/robot_manipulation/data_with_fts_2/train_npz/episode_{episode_id:05d}.npz"
skip_frames = 2

# === UTILS ===
def put_text_with_bg(img, text, pos, font, scale, color, thickness, bg_color, padding=2):
    (text_w, text_h), baseline = cv2.getTextSize(text, font, scale, thickness)
    x, y = pos
    cv2.rectangle(img, (x - padding, y - text_h - padding), (x + text_w + padding, y + baseline + padding), bg_color, cv2.FILLED)
    cv2.putText(img, text, (x, y), font, scale, color, thickness, lineType=cv2.LINE_AA)

def resize_fixed(img, width=480, height=360):
    return cv2.resize(img, (width, height))

def draw_joint_wheels(joint_angles, size=(480, 360)):
    img = np.zeros((size[1], size[0], 3), dtype=np.uint8)
    center_y = size[1] // 2
    radius = 30
    spacing = 60
    for i, angle in enumerate(joint_angles):
        cx = spacing * (i + 1)
        cy = center_y
        cv2.circle(img, (cx, cy), radius, (255, 255, 255), 2)
        end_x = int(cx + radius * np.cos(angle))
        end_y = int(cy + radius * np.sin(angle))
        cv2.line(img, (cx, cy), (end_x, end_y), (0, 255, 0), 2)
        cv2.putText(img, f'J{i+1}', (cx - 10, cy + radius + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    return img

# === LOAD DATA ===
with h5py.File(h5_file, "r") as h5:
    if USE_RAW_IMAGES:
        images = h5["image"][:]
        wrist_images = h5["wrist_image"][:]
    else:
        image_filenames = h5["image_filenames"][:]
        wrist_filenames = h5["wrist_image_filenames"][:]
        images, wrist_images = [], []
        for img_path, wrist_path in zip(image_filenames, wrist_filenames):
            img_full_path = os.path.join(base_dir, img_path.decode() if isinstance(img_path, bytes) else img_path)
            wrist_full_path = os.path.join(base_dir, wrist_path.decode() if isinstance(wrist_path, bytes) else wrist_path)
            user_img = cv2.imread(img_full_path)
            wrist_img = cv2.imread(wrist_full_path)
            if user_img is None or wrist_img is None:
                raise FileNotFoundError(f"Could not load images: {img_full_path} or {wrist_full_path}")
            images.append(user_img)
            wrist_images.append(wrist_img)
        images = np.array(images)
        wrist_images = np.array(wrist_images)
    robot_states = h5["robot_state"][:]

npz_data = np.load(npz_file, allow_pickle=True)
freq_history = npz_data["frequency_history"]
freq_history = np.where(freq_history == 0, 1e-6, freq_history)

images = images[skip_frames:]
wrist_images = wrist_images[skip_frames:]
robot_states = robot_states[skip_frames:]
freq_history = freq_history[skip_frames:]

timestamps = np.cumsum(1.0 / freq_history)
timestamps -= timestamps[0]

n_frames = images.shape[0]
tcp_positions = robot_states[:, 6:9]

# === ADVANCED STATISTICS ===
print("=== Dataset Summary ===")
print(f"Episode ID: {episode_id}")
print(f"Frame count: {n_frames}")
print(f"Total duration: {timestamps[-1]:.2f} s")
print(f"Frequency: min={freq_history.min():.2f}, max={freq_history.max():.2f}, mean={freq_history.mean():.2f} Hz")
print("Robot state shape:", robot_states.shape)
print("TCP X range:", tcp_positions[:,0].min(), "-", tcp_positions[:,0].max())
print("TCP Y range:", tcp_positions[:,1].min(), "-", tcp_positions[:,1].max())
print("TCP Z range:", tcp_positions[:,2].min(), "-", tcp_positions[:,2].max())

# === Derivative statistics ===
tcp_vel = np.gradient(tcp_positions, axis=0)
joint_vel = np.gradient(robot_states[:, :6], axis=0)
# === Selective Robot State Plot with Labels ===
state_indices_to_plot = [6, 7, 8]

state_labels = {
    0: "Joint 1",
    1: "Joint 2",
    2: "Joint 3",
    3: "Joint 4",
    4: "Joint 5",
    5: "Joint 6",
    6: "TCP X",
    7: "TCP Y",
    8: "TCP Z",
    9: "TCP QX",
    10: "TCP QY",
    11: "TCP QZ",
    12: "TCP QW",
    13: "Gripper",
    14: "Force Trigger",
}

fig, axes = plt.subplots(len(state_indices_to_plot), 1, figsize=(10, 2 * len(state_indices_to_plot)), sharex=True)

for ax, i in zip(axes, state_indices_to_plot):
    ax.plot(timestamps, robot_states[:, i], color='tab:blue')
    ax.set_ylabel(state_labels.get(i, f"State {i}"))
    ax.grid(True)

axes[-1].set_xlabel("Time (s)")
plt.suptitle(f"Selected Robot States (Episode {episode_id})", fontsize=16)
plt.tight_layout(rect=[0, 0, 1, 0.98])
os.makedirs("visualization_outputs", exist_ok=True)
plt.savefig(f"visualization_outputs/visualization_episode_{episode_id:05d}_robot_state_selected.png", dpi=150)
plt.close(fig)


# === TCP Trajectory 3D ===
fig = plt.figure(figsize=(6, 5))
ax = fig.add_subplot(111, projection='3d')
ax.plot(tcp_positions[:,0], tcp_positions[:,1], tcp_positions[:,2], label="TCP Path", c='blue')
ax.scatter(tcp_positions[0,0], tcp_positions[0,1], tcp_positions[0,2], c='green', label="Start", s=60)
ax.scatter(tcp_positions[-1,0], tcp_positions[-1,1], tcp_positions[-1,2], c='red', label="End", s=60)
ax.set_title("TCP 3D Trajectory")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.legend()
plt.tight_layout()
plt.savefig(f"visualization_outputs/visualization_episode_{episode_id:05d}_tcp_3d_test.png")
plt.close(fig)

print("Summary plots saved.")
