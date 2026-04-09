import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os
from collections import deque

class MultiCameraViewer:
    def __init__(self):
        print("Initializing Intel RealSense D405 cameras...")

        # Initialize RealSense pipeline and config for robot wrist camera
        self.pipeline_robot = rs.pipeline()
        self.config_robot = rs.config()
        self.config_robot.enable_device('218622276856')
        self.config_robot.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

        # Initialize RealSense pipeline and config for user camera
        self.pipeline_user = rs.pipeline()
        self.config_user = rs.config()
        self.config_user.enable_device('128422270567')
        self.config_user.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

        # Start both pipelines
        self.pipeline_robot.start(self.config_robot)
        self.pipeline_user.start(self.config_user)

        # For delay and FPS tracking (robot camera only)
        self.delay_history = deque()
        self.fps_history = deque()
        self.delay_history_max_time = 10.0  # seconds
        self.fps_history_max_time = 5.0     # seconds
        self.last_frame_time = None

    def read_cameras(self):
        now = time.time()

        # Read robot wrist camera frames (blocking)
        robot_frames = self.pipeline_robot.wait_for_frames()
        robot_color = robot_frames.get_color_frame()
        robot_image = np.asanyarray(robot_color.get_data())
        robot_image = cv2.resize(robot_image, (640, 480))

        # Calculate delay for robot camera frame
        frame_ts_sec = robot_color.get_timestamp() / 1000.0
        delay = now - frame_ts_sec
        self.delay_history.append((now, delay))

        # Calculate FPS for robot camera frame
        if self.last_frame_time is not None:
            frame_interval = now - self.last_frame_time
            if frame_interval > 0:
                fps = 1.0 / frame_interval
                self.fps_history.append((now, fps))
        self.last_frame_time = now

        # Clean old delay entries
        while self.delay_history and (now - self.delay_history[0][0]) > self.delay_history_max_time:
            self.delay_history.popleft()

        # Clean old FPS entries
        while self.fps_history and (now - self.fps_history[0][0]) > self.fps_history_max_time:
            self.fps_history.popleft()

        # Average FPS over history for smoother output
        fps_values = [f for t, f in self.fps_history]
        avg_fps = sum(fps_values) / len(fps_values) if fps_values else 0.0

        # Clear terminal and print delay and FPS info
        os.system('cls' if os.name == 'nt' else 'clear')
        print(f"Robot camera delay (seconds): {delay:.4f}")
        print(f"Robot camera FPS (avg over last {self.fps_history_max_time} sec): {avg_fps:.2f}")

        # Read user camera frames (blocking)
        user_frames = self.pipeline_user.wait_for_frames()
        user_color = user_frames.get_color_frame()
        user_image = np.asanyarray(user_color.get_data())

        # Crop and resize user image
        user_image = user_image[0:430, 5:500]  # crop region
        user_image = cv2.resize(user_image, (640, 640))

        # Rotate robot image 90 degrees clockwise
        rotated_robot_image = cv2.rotate(robot_image, cv2.ROTATE_90_CLOCKWISE)

        # Create combined canvas side-by-side (height=max height, width=sum widths)
        combined_height = max(rotated_robot_image.shape[0], user_image.shape[0])
        combined_width = rotated_robot_image.shape[1] + user_image.shape[1]
        combined_image = np.zeros((combined_height, combined_width, 3), dtype=np.uint8)

        # Place robot image on left
        combined_image[:rotated_robot_image.shape[0], :rotated_robot_image.shape[1]] = rotated_robot_image
        # Place user image on right
        combined_image[:user_image.shape[0], rotated_robot_image.shape[1]:] = user_image

        # Return combined, separate robot and user images, and full delay history
        return combined_image, robot_image, user_image, list(self.delay_history)

    def stop(self):
        self.pipeline_robot.stop()
        self.pipeline_user.stop()

if __name__ == "__main__":
    viewer = MultiCameraViewer()

    try:
        while True:
            combined_img, robot_img, user_img, delays = viewer.read_cameras()
            cv2.imshow("Multi Camera Viewer", combined_img)

            # Quit on 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        viewer.stop()
        cv2.destroyAllWindows()
