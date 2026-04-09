import os
import h5py
import numpy as np
import matplotlib.pyplot as plt
import cv2
from tqdm import tqdm
def print_structure(name, obj):
    if isinstance(obj, h5py.Group):
        print(f"[Group] {name}")
    elif isinstance(obj, h5py.Dataset):
        print(f"[Dataset] {name} | shape: {obj.shape} | dtype: {obj.dtype}")

def inspect_h5_structure(file_path):
    with h5py.File(file_path, 'r') as f:
        f.visititems(print_structure)



def plot_robot_state(robot_state, output_dir, episode_name):
    # robot_state shape: (T, 15)
    # Indices mapping
    joint_indices = range(6)
    xyz_indices = range(6, 9)
    quat_indices = range(9, 13)
    status_indices = range(13, 15)

    time = np.arange(robot_state.shape[0])

    plt.figure(figsize=(20, 15))
    labels = [f"joint_{i+1}" for i in joint_indices] + \
             ["pos_x", "pos_y", "pos_z"] + \
             ["quat_w", "quat_x", "quat_y", "quat_z"] + \
             ["status_1", "status_2"]

    for i in range(robot_state.shape[1]):
        plt.subplot(5, 3, i+1)
        plt.plot(time, robot_state[:, i])
        plt.title(labels[i])
        plt.xlabel("Time step")
        plt.tight_layout()

    plot_path = os.path.join(output_dir, f"{episode_name}_robot_state.png")
    plt.savefig(plot_path)
    plt.close()
    print(f"Saved robot state plot to {plot_path}")

def create_video_from_frames(frames, output_file, fps=30):
    # frames shape: (T, H, W, C)
    height, width = frames[0].shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video_writer = cv2.VideoWriter(output_file, fourcc, fps, (width, height))

    for frame in tqdm(frames, desc=f"Writing video {output_file}"):
        # Convert RGB to BGR for OpenCV
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        video_writer.write(frame_bgr)

    video_writer.release()
    print(f"Saved video to {output_file}")

def visualize_episode(h5_file_path, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    episode_name = os.path.splitext(os.path.basename(h5_file_path))[0]

    with h5py.File(h5_file_path, "r") as f:
        robot_state = f["robot_state"][:]       # (T, 15)
        images = f["image"][:]                   # (T, H, W, 3)
        wrist_images = f["wrist_image"][:]      # (T, H, W, 3)

    plot_robot_state(robot_state, output_dir, episode_name)

    video_path_image = os.path.join(output_dir, f"{episode_name}_user_camera.mp4")
    create_video_from_frames(images, video_path_image)

    video_path_wrist = os.path.join(output_dir, f"{episode_name}_wrist_camera.mp4")
    create_video_from_frames(wrist_images, video_path_wrist)

if __name__ == "__main__":
    h5_file = "data_zippered/train/episode_00001.h5"  # Example file path
    inspect_h5_structure(h5_file)

    output_dir = "visualizations/episode_00001"
    visualize_episode(h5_file, output_dir)
