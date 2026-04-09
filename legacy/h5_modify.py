import h5py
import os
import re

# Config
DATA_DIR = "/media/Data/demoaccount2/robot_manipulation/paper_extension_data/wiping/haptic/train/"  # directory to search
NEW_INSTRUCTION = "Wipe the table with the yellow cloth."  # new instruction

EPISODE_PATTERN = re.compile(r"^episode_\d{5}\.h5$")

def update_language_instruction(file_path, new_instruction):
    """Update the language_instruction in episode_info dataset of one .h5 file."""
    try:
        with h5py.File(file_path, "r+") as f:
            if "episode_info" not in f:
                print(f"[SKIP] {file_path}: no episode_info dataset")
                return

            dset = f["episode_info"]

            record = dset[0]

            record["language_instruction"] = new_instruction.encode("utf-8")

            dset[0] = record

            print(f"[OK] Updated language_instruction in {file_path}")
    except Exception as e:
        print(f"[ERROR] Could not update {file_path}: {e}")


def process_directory(data_dir, new_instruction):
    for filename in os.listdir(data_dir):
        if EPISODE_PATTERN.match(filename):
            file_path = os.path.join(data_dir, filename)
            update_language_instruction(file_path, new_instruction)

if __name__ == "__main__":
    process_directory(DATA_DIR, NEW_INSTRUCTION)
