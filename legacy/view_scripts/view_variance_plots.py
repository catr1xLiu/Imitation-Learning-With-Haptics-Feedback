import os
import h5py
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

# === CONFIG ===
base_dir = "/media/Data/demoaccount2/robot_manipulation/wipe_yellow_cloth_HAPLY/train"
npz_dir = "/media/Data/demoaccount2/robot_manipulation/wipe_yellow_cloth_HAPLY/train_npz"
output_dir = "visualization_outputs"
state_indices_to_plot = [6, 7, 8]  # TCP X, Y, Z
state_labels = {
    0: "Joint 1", 1: "Joint 2", 2: "Joint 3", 3: "Joint 4", 4: "Joint 5", 5: "Joint 6",
    6: "TCP X", 7: "TCP Y", 8: "TCP Z", 9: "TCP QX", 10: "TCP QY", 11: "TCP QZ", 12: "TCP QW",
    13: "Gripper", 14: "Force Trigger",
}
colors = ['red', 'orange', 'gold']
n_uniform_steps = 100

# === Collect aligned data ===
episode_files = sorted([f for f in os.listdir(base_dir) if f.endswith('.h5')])
aligned_states_real_time = {i: [] for i in state_indices_to_plot}
aligned_states_normalized = {i: [] for i in state_indices_to_plot}
timestamps_real_time = []

for h5_filename in tqdm(episode_files, desc="Processing episodes"):
    episode_id = int(h5_filename.split('_')[1].split('.')[0])
    h5_path = os.path.join(base_dir, h5_filename)
    npz_path = os.path.join(npz_dir, f"episode_{episode_id:05d}.npz")

    with h5py.File(h5_path, 'r') as h5:
        robot_states = h5["robot_state"][:]

    npz_data = np.load(npz_path, allow_pickle=True)
    freq_history = npz_data["frequency_history"]
    freq_history = np.where(freq_history == 0, 1e-6, freq_history)
    timestamps = np.cumsum(1.0 / freq_history)
    timestamps -= timestamps[0]

    min_len = min(len(timestamps), len(robot_states))
    timestamps = timestamps[:min_len]
    robot_states = robot_states[:min_len]

    # Real-time aligned
    timestamps_real_time.append(timestamps)
    for idx in state_indices_to_plot:
        aligned_states_real_time[idx].append(np.interp(
            np.linspace(timestamps[0], timestamps[-1], n_uniform_steps),
            timestamps, robot_states[:, idx]))

    # Normalized index-aligned
    for idx in state_indices_to_plot:
        interp_values = np.interp(
            np.linspace(0, min_len - 1, n_uniform_steps),
            np.arange(min_len), robot_states[:min_len, idx])
        aligned_states_normalized[idx].append(interp_values)

# === Compute mean and std ===
def compute_mean_std(aligned_data):
    mean_std = {}
    for idx in state_indices_to_plot:
        data = np.array(aligned_data[idx])
        mean_std[idx] = (np.mean(data, axis=0), np.std(data, axis=0))
    return mean_std

mean_std_real = compute_mean_std(aligned_states_real_time)
mean_std_norm = compute_mean_std(aligned_states_normalized)

# === Plotting ===
def plot_mean_std(mean_std_data, x_vals, title, filename):
    fig, ax = plt.subplots(len(state_indices_to_plot), 1, figsize=(10, 2.5 * len(state_indices_to_plot)), sharex=True)
    for i, idx in enumerate(state_indices_to_plot):
        mean, std = mean_std_data[idx]
        ax[i].plot(x_vals, mean, color=colors[i % len(colors)], label=state_labels.get(idx, f"State {idx}"))
        ax[i].fill_between(x_vals, mean - std, mean + std, color=colors[i % len(colors)], alpha=0.3)
        ax[i].set_ylabel(state_labels.get(idx, f"State {idx}"))
        ax[i].grid(True)
        ax[i].legend()
    ax[-1].set_xlabel("Time (s)" if "real" in filename else "Task Progress (%)")
    plt.suptitle(title, fontsize=16)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    os.makedirs(output_dir, exist_ok=True)
    plt.savefig(os.path.join(output_dir, filename), dpi=150)
    plt.close(fig)

# Real-time plot
x_real = np.linspace(0, max(ts[-1] for ts in timestamps_real_time), n_uniform_steps)
plot_mean_std(mean_std_real, x_real, "Mean and Variance of Robot States (Real Time)", "robot_state_variance_real_time.png")

# Normalized plot
x_norm = np.linspace(0, 100, n_uniform_steps)
plot_mean_std(mean_std_norm, x_norm, "Mean and Variance of Robot States (Normalized Time)", "robot_state_variance_normalized.png")
