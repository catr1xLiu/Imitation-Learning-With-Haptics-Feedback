# Script to compute correct KNN-based Action Variance + Jerkiness (paper-accurate)

import os
import h5py
import numpy as np
from sklearn.neighbors import NearestNeighbors


# ======================================================
# CONFIG
# ======================================================

DATASETS = ["paper_dataset1", "paper_dataset2"]

SUBFOLDERS = [
    "lightbulb_VR",
    "lightbulb_HAPTIC",
    "wipe_yellow_cloth_VR",
    "wipe_yellow_cloth_HAPLY"
]

EPISODES_PER_FOLDER = 100

STATE_DIM = 6        # first 6 dims (x,y,z,rx,ry,rz)
ACTION_DIM = 6       # first 6 dims of the action (the true action key)
K = 5                # nearest neighbors for both metrics


# ======================================================
# HDF5 LOADING
# ======================================================

def load_episode_actions_and_states(path):
    """
    Loads robot_state[:6] and action[:6].
    If corrupted (1D), returns empty arrays to skip.
    """
    try:
        with h5py.File(path, "r") as f:
            rs = np.array(f["robot_state"][:])
            ac = np.array(f["action"][:])
    except:
        print(f"[SKIP] {path}: failed to read.")
        return np.zeros((0, STATE_DIM)), np.zeros((0, ACTION_DIM))

    # robot_state must be 2D
    if rs.ndim != 2 or ac.ndim != 2:
        print(f"[SKIP] {path}: robot_state or action has bad shape {rs.shape}, {ac.shape}.")
        return np.zeros((0, STATE_DIM)), np.zeros((0, ACTION_DIM))

    # slice first 6 dims as required by paper
    states = rs[:, :STATE_DIM]
    actions = ac[:, :ACTION_DIM]

    return states, actions


def load_folder(folder_path):
    """Load states and actions for all episodes in a folder."""
    all_states = []
    all_actions = []

    for i in range(1, EPISODES_PER_FOLDER + 1):
        ep_path = os.path.join(folder_path, f"episode_{i:05d}.h5")
        if os.path.exists(ep_path):
            S, A = load_episode_actions_and_states(ep_path)
            if len(S) > 0:
                all_states.append(S)
                all_actions.append(A)

    return all_states, all_actions


# ======================================================
# ACTION VARIANCE (CORRECT KNN-BASED, STATE-SPACE NEIGHBORS)
# ======================================================

def compute_action_variance(all_states, all_actions, K=5):
    """
    Computes global Action Variance:
    - KNN in state space
    - variance of each action vs mean neighbor action
    """

    S = np.concatenate(all_states, axis=0)
    A = np.concatenate(all_actions, axis=0)
    N = len(S)

    # KNN on states
    nbrs = NearestNeighbors(n_neighbors=K, algorithm='auto').fit(S)
    _, idx = nbrs.kneighbors(S)

    vals = []
    for i in range(N):
        a_i = A[i]
        neigh_actions = A[idx[i]]
        mean_a = neigh_actions.mean(axis=0)
        vals.append(np.sum((a_i - mean_a) ** 2))

    return float(np.mean(vals))


# ======================================================
# JERKINESS (SECOND DERIVATIVE OF ACTION SEQUENCE)
# ======================================================

def compute_jerkiness(all_actions):
    """
    Jerkiness as defined in the paper:
    jerk(t) = a(t+1) - 2a(t) + a(t-1)
    episode jerkiness = mean( || jerk(t) ||_2 )
    """

    episode_vals = []

    for A in all_actions:
        T = len(A)
        if T < 3:
            continue

        # second derivative
        jerk = A[2:] - 2*A[1:-1] + A[:-2]   # (T-2, 6)

        # L2 magnitude
        jerk_mag = np.linalg.norm(jerk, axis=1)

        # Episode jerkiness
        episode_vals.append(float(jerk_mag.mean()))

    if len(episode_vals) == 0:
        return {"jerk_mean": 0.0, "jerk_std": 0.0}

    return {
        "jerk_mean": float(np.mean(episode_vals)),
        "jerk_std": float(np.std(episode_vals)),
    }


# ======================================================
# MAIN DRIVER
# ======================================================

results = {}

for dataset in DATASETS:
    dataset_results = {}

    for folder in SUBFOLDERS:
        folder_path = os.path.join(dataset, folder, "train")
        print(f"\nProcessing {folder_path} ...")

        all_states, all_actions = load_folder(folder_path)

        # Action Variance
        av = compute_action_variance(all_states, all_actions, K=K)

        # Correct Jerkiness (using action)
        jerk = compute_jerkiness(all_actions)

        dataset_results[folder] = {
            "action_variance": av,
            "jerkiness": jerk
        }

        print(f"  Action Variance: {av:.6f}")
        print(f"  Jerkiness Mean: {jerk['jerk_mean']:.6f}")
        print(f"  Jerkiness Std:  {jerk['jerk_std']:.6f}")

    results[dataset] = dataset_results

print("\n\n==== Final Results ====")
print(results)
