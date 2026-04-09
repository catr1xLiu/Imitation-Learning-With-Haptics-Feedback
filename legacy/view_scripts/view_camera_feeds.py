import h5py
import numpy as np
import cv2
import os
from datetime import timedelta

# === CONFIG ===
USE_RAW_IMAGES = False
episode_id = 16
base_dir = "data/train"
h5_file = f"{base_dir}/episode_{episode_id:05d}.h5"
npz_file = f"data/train_npz/episode_{episode_id:05d}.npz"
output_file = f"episode_{episode_id:05d}_side_by_side.mp4"
skip_frames = 2
frame_width, frame_height = 480, 360
output_fps = 14.3

# === UTILS ===
def put_text_with_bg(img, text, pos, font, scale, color, thickness, bg_color, padding=2):
    (text_w, text_h), baseline = cv2.getTextSize(text, font, scale, thickness)
    x, y = pos
    cv2.rectangle(img, (x - padding, y - text_h - padding),
                  (x + text_w + padding, y + baseline + padding), bg_color, cv2.FILLED)
    cv2.putText(img, text, (x, y), font, scale, color, thickness, lineType=cv2.LINE_AA)

def resize_fixed(img, width=480, height=360):
    return cv2.resize(img, (width, height))

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
                raise FileNotFoundError(f"Missing: {img_full_path} or {wrist_full_path}")
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

# === SETUP VIDEO WRITER ===
out_size = (frame_width * 2, frame_height)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video_writer = cv2.VideoWriter(output_file, fourcc, output_fps, out_size)

font = cv2.FONT_HERSHEY_SIMPLEX

for img1, img2, ts in zip(images, wrist_images, timestamps):
    img1 = resize_fixed(img1, frame_width, frame_height)
    img2 = resize_fixed(img2, frame_width, frame_height)
    combined = np.hstack((img1, img2))

    time_str = str(timedelta(seconds=ts))[:11]
    put_text_with_bg(combined, f"Time: {time_str}", (10, 30), font, 0.8, (255, 255, 255), 2, (0, 0, 0))

    video_writer.write(combined)

video_writer.release()
print(f"Video saved to: {output_file}")
