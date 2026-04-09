import numpy as np
import h5py
import matplotlib.pyplot as plt
from pathlib import Path

# Config
DATA_DIR = Path("/home/demoaccount/Data/haply_trajectories")
SUFFIX = ".h5"

def get_latest_h5(path: Path):
    files = sorted(path.glob(f"*{SUFFIX}"), key=lambda p: p.stat().st_mtime, reverse=True)
    return files[0] if files else None

def main():
    file_path = get_latest_h5(DATA_DIR)
    if not file_path:
        print("No h5 file found.")
        return

    with h5py.File(file_path, 'r') as f:
        actions = np.array(f['action'])  # [T, 7]

    fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    labels = ['dx', 'dy', 'dz', 'dα', 'dβ', 'dγ']
    t = np.arange(actions.shape[0])

    for i in range(6):
        ax = axes[i // 2, i % 2]
        ax.plot(t, actions[:, i])
        ax.set_ylabel(labels[i])
        ax.grid(True)

    axes[-1, 0].set_xlabel("Timestep")
    axes[-1, 1].set_xlabel("Timestep")
    fig.suptitle(file_path.name, fontsize=14)
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    out_path = file_path.with_suffix(".png")
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"Saved: {out_path}")

if __name__ == "__main__":
    main()
