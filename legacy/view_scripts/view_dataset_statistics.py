import os
import h5py
import numpy as np
import matplotlib.pyplot as plt

# === CONFIG ===
dataset_title = "Force-Torque Sensor Simulation" 
base_dir = "/media/Data/demoaccount2/robot_manipulation/data_with_fts_2/train/"
# base_dir = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/wiping/vr/train/"
skip_frames = 2
save_dir = "visualization_outputs"
os.makedirs(save_dir, exist_ok=True)

# === COLLECT ALL EPISODES ===
episode_files = sorted([f for f in os.listdir(base_dir) if f.endswith('.h5')])
all_states = []
all_fts_states = []
all_timestamps = []

for fname in episode_files:
    h5_path = os.path.join(base_dir, fname)
    ep_id = int(fname.split('_')[-1].split('.')[0])

    with h5py.File(h5_path, "r") as h5:
        if "robot_state" not in h5:
            print(f"Skipping {fname} - no 'robot_state'")
            continue

        robot_states = h5["robot_state"][:][skip_frames:]

        if robot_states.size == 0 or robot_states.shape[1] != 15:
          print(f"Skipping {fname} - invalid robot_state shape {robot_states.shape}")
          continue

        if robot_states.ndim == 1:
          robot_states = robot_states[np.newaxis, :]

        if "fts_info" not in h5:
            print(f"{fname} has no 'fts_info'")
        else:
            fts_info = h5["fts_info"][:][skip_frames:]
            all_fts_states.append(np.array(fts_info))

        # Load frequency from matching NPZ
        npz_path = f"/media/Data/demoaccount2/robot_manipulation/data_with_fts_2/train_npz/episode_{ep_id:05d}.npz"
        # npz_path = f"/media/Data/demoaccount2/robot_manipulation/paper_extension_data/wiping/vr/train_npz/episode_{ep_id:05d}.npz"
        if not os.path.exists(npz_path):
            print(f"Missing NPZ for {fname}, skipping timestamp info.")
            timestamps = np.arange(len(robot_states))
        else:
            npz_data = np.load(npz_path, allow_pickle=True)
            freq_history = npz_data["frequency_history"][skip_frames:]
            freq_history = np.where(freq_history == 0, 1e-6, freq_history)
            timestamps = np.cumsum(1.0 / freq_history)
            timestamps -= timestamps[0]

        all_states.append(robot_states)
        all_timestamps.append(timestamps)

# === CONCATENATE ALL EPISODES ===
robot_states_all = np.concatenate(all_states, axis=0)
timestamps_all = np.concatenate(all_timestamps)

# === PLOT METRICS ===
fig, axes = plt.subplots(5, 3, figsize=(18, 12), sharex=True)
fig.suptitle("Robot State: Mean ± Std. Deviation Across All Episodes", fontsize=16)
axes = axes.flatten()

# for states in all_fts_states
#     print(states)

for i in range(15):
    state_series = robot_states_all[:, i]
    # Compute running mean and std with smoothing
    window = 30
    mean = np.convolve(state_series, np.ones(window)/window, mode='valid')
    std = np.array([
        np.std(state_series[max(0, j - window):j]) for j in range(window, len(state_series)+1)
    ])
    time = timestamps_all[window-1:]

    ax = axes[i]
    ax.plot(time, mean, label=f"State {i}")
    ax.fill_between(time, mean - std, mean + std, alpha=0.3)
    ax.set_ylabel(f"State {i}")
    ax.grid(True)

axes[-1].set_xlabel("Time (s)")
plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig(os.path.join(save_dir, "robot_state_variance_all.png"))
plt.show()

# === PLOT TCP 3D TRAJECTORY ===
from mpl_toolkits.mplot3d import Axes3D

fts_enabled = bool(all_fts_states)

if fts_enabled:
  all_states = list(zip(all_states, all_fts_states))

fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

for states in all_states:
    if fts_enabled:
        tcp_all, ft_vector = states
        tcp = np.array(tcp_all[:, 6:9])
        tcp_x, tcp_y, tcp_z = tcp.T
        force_vector = np.array(ft_vector[:, 0:3])
        fx, fy, fz = force_vector.T
        for i in range(1, len(tcp)):
            current_force = [fx[i-1], fy[i-1], fz[i-1]]
            norm = np.linalg.norm(current_force)
            ax.plot(tcp_x[i-1:i+1], tcp_y[i-1:i+1], tcp_z[i-1:i+1], c=(norm, 0, 1 - norm), lw=0.5)
    else: 
        tcp = states[:, 6:9]
        ax.plot(tcp[:, 0], tcp[:, 1], tcp[:, 2], c="blue", lw=0.5)

    # Start point
    ax.scatter(tcp[0, 0], tcp[0, 1], tcp[0, 2],
               c='lightgreen', s=5, alpha=0.8)

    # End point
    ax.scatter(tcp[-1, 0], tcp[-1, 1], tcp[-1, 2],
               c='red', s=5, alpha=0.8)

ax.set_title(dataset_title)
ax.set_xlabel("X (m)", labelpad=10)
ax.set_ylabel("Y (m)", labelpad=10)
ax.set_zlabel("Z (m)", labelpad=10)
ax.view_init(elev=30, azim=125)
plt.tight_layout()
plt.savefig(os.path.join(save_dir, "tcp_trajectory_3d_test.png"))
plt.show()
