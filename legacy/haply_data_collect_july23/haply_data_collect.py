import time
import h5py
import numpy as np
from datetime import timedelta
from pathlib import Path
import sys
sys.path.append('/home/demoaccount/Data/demoaccount2/robot_diffusion/diffusion_policy_jul24')

from haply_data_collect_july23.haply_barebones import HapticVisualizer  # Adjust path
# from robot_env_corrected.RTDE_RW_test_collect import RobotAction  # Not used here

SAVE_PATH = Path("/home/demoaccount/Data/haply_trajectories")
SAVE_PATH.mkdir(parents=True, exist_ok=True)
FILENAME = SAVE_PATH / f"haply_recording_{time.strftime('%Y%m%d_%H%M%S')}.h5"
DURATION_SECS = 80 
CONTROL_HZ = 15

INTERVAL = 1.0 / CONTROL_HZ

def main():
    viz = HapticVisualizer(display_plot=True)
    actions = []
    t_start = time.time()
    t_end = t_start + DURATION_SECS

    try:
        while time.time() < t_end:
            action = viz.get_action(mode='euler', seq='yxz')
            actions.append(action.copy())

            remaining = int(t_end - time.time())
            print(f"\rRecording [{len(actions)} steps] | Time left: {str(timedelta(seconds=remaining))}", end='', flush=True)

            time.sleep(INTERVAL)
    except KeyboardInterrupt:
        print("\nInterrupted by user. Saving data...")
    finally:
        viz.stop()
        actions = np.stack(actions, axis=0)
        with h5py.File(FILENAME, "w") as f:
            f.create_dataset("action", data=actions)
        print(f"\nSaved {len(actions)} steps to {FILENAME}")

if __name__ == "__main__":
    main()