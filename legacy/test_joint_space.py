# import h5py
# import numpy as np
# import matplotlib.pyplot as plt
# from tqdm import tqdm

# file = "/media/Data/demoaccount2/robot_manipulation/autogen_AB_random_large_5hz_with_pad/train/episode_00001.h5"

# def print_h5_shapes(file_path):
#     def recursively_print(name, obj):
#         if isinstance(obj, h5py.Dataset):
#             print(f"{name}: shape={obj.shape}, dtype={obj.dtype}")
#     with h5py.File(file_path, 'r') as f:
#         f.visititems(recursively_print)

# print_h5_shapes(file)

# with h5py.File(file, "r") as f:
#     joint_deltas = np.array(f["action"])[:, :6]                 # [T, 6]
#     robot_state = np.array(f["robot_state"])[:, :6]             # [T, 6]
#     tcp_deltas = np.array(f["action_tcp"])                      # [T, ?]
#     tcp_dof = tcp_deltas.shape[1]

# # Truncate everything to the same length in time
# T = min(len(joint_deltas), len(robot_state), len(tcp_deltas))
# joint_deltas = joint_deltas[:T]
# robot_state = robot_state[:T]
# tcp_deltas = tcp_deltas[:T]

# joint_global = robot_state
# cumsum_joint = np.cumsum(joint_deltas, axis=0)

# column_titles = [
#     "joint_deltas", 
#     "TCP deltas", 
#     "global_joints", 
#     "sum of joint deltas (should ≈ global_joints)"
# ]

# fig, axs = plt.subplots(tcp_dof, 4, figsize=(20, 2.5 * tcp_dof))
# time = np.arange(T)

# for i in tqdm(range(tcp_dof), desc="Plotting DoF components"):
#     axs[i, 0].plot(time, joint_deltas[:, i])
#     axs[i, 1].plot(time, tcp_deltas[:, i])
#     axs[i, 2].plot(time, joint_global[:, i])
#     axs[i, 3].plot(time, cumsum_joint[:, i])
#     axs[i, 0].set_ylabel(f"Dim {i}")

# for j in range(4):
#     axs[0, j].set_title(column_titles[j])

# for ax in axs.flatten():
#     ax.grid(True)

# plt.tight_layout()
# plt.savefig("debug_action_tcp_grid.png")
# print("Saved to debug_action_tcp_grid.png")
import h5py
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

file = "/media/Data/demoaccount2/robot_manipulation/autogen_AB_random_large_5hz_with_pad/train/episode_00001.h5"

def print_h5_shapes(file_path):
    def recursively_print(name, obj):
        if isinstance(obj, h5py.Dataset):
            print(f"{name}: shape={obj.shape}, dtype={obj.dtype}")
    with h5py.File(file_path, 'r') as f:
        f.visititems(recursively_print)

print("Inspecting HDF5 file structure...")
print_h5_shapes(file)

with h5py.File(file, "r") as f:
    joint_deltas = np.array(f["action"])[:, :6]                 # [T, 6]
    robot_state = np.array(f["robot_state"])[:, :6]             # [T, 6]
    tcp_deltas = np.array(f["action_tcp"])                      # [T, tcp_dof]
    raw_actions = np.array(f["action_raw"])                    # [T, raw_dof]

# Align sequence lengths
T = min(len(joint_deltas), len(robot_state), len(tcp_deltas), len(raw_actions))
joint_deltas = joint_deltas[:T]
robot_state = robot_state[:T]
tcp_deltas = tcp_deltas[:T]
raw_actions = raw_actions[:T]

# Precompute derived signals
joint_global = robot_state
cumsum_joint = np.cumsum(joint_deltas, axis=0)

tcp_dof = tcp_deltas.shape[1]
raw_dof = raw_actions.shape[1]
max_dof = max(tcp_dof, raw_dof)

column_titles = [
    "joint_deltas",
    "TCP deltas",
    "action_raw",
    "global_joints",
    "sum of joint deltas (should ≈ global_joints)"
]

fig, axs = plt.subplots(max_dof, 5, figsize=(25, 2.5 * max_dof))
time = np.arange(T)

print("Plotting...")
for i in tqdm(range(max_dof), desc="Plotting DoF components"):
    if i < joint_deltas.shape[1]:
        axs[i, 0].plot(time, joint_deltas[:, i])
    if i < tcp_deltas.shape[1]:
        axs[i, 1].plot(time, tcp_deltas[:, i])
    if i < raw_actions.shape[1]:
        axs[i, 2].plot(time, raw_actions[:, i])
    if i < robot_state.shape[1]:
        axs[i, 3].plot(time, joint_global[:, i])
    if i < cumsum_joint.shape[1]:
        axs[i, 4].plot(time, cumsum_joint[:, i])
    axs[i, 0].set_ylabel(f"Dim {i}")

for j in range(5):
    axs[0, j].set_title(column_titles[j])

for ax in axs.flatten():
    ax.grid(True)

plt.tight_layout()
plt.savefig("debug_action_tcp_raw_grid.png")
print("Saved plot to debug_action_tcp_raw_grid.png")
