import time
import numpy as np
from threading import Lock
from pynput import keyboard

from robot_env.RTDE_RW_test_collect import RobotAction
from haply_data_collect_july23.haply_barebones import HapticVisualizer

# --- config ---
POS_GAIN = 1.5
ROT_GAIN = 0.8
STILL_THRESHOLD = 0.0
HAPTIC_FEEDBACK = True
CONTROL_HZ = 6


class HaplyController:
    """Controls the data collection loop and keyboard state for the Haply device."""

    def __init__(self):
        self.state_lock = Lock()
        self.want_start = False
        self.want_save_end = False
        self.want_discard = False
        self.want_quit = False

    def on_press(self, key):
        """Callback for pynput keyboard listener."""
        try:
            k = key.char
        except AttributeError:
            k = None

        with self.state_lock:
            if key == keyboard.Key.esc:
                self.want_quit = True
                print("⏹ QUIT")
            elif k == "s":
                self.want_start = True
                # Consume any pending end/discard for a clean start
                self.want_save_end = False
                self.want_discard = False
                print("▶ START")
            elif k == "e":
                self.want_save_end = True
                print("💾 SAVE END")
            elif k == "d":
                self.want_discard = True
                print("🗑️ DISCARD")

    def run_session(self, hz=30):
        """Runs a single recording session."""
        dt = 1.0 / hz
        env = RobotAction(control_hz=hz, haptic_feedback=HAPTIC_FEEDBACK)
        viz = HapticVisualizer(
            display_plot=False, haptic_feedback=HAPTIC_FEEDBACK, force_reader=env
        )

        t_next = time.perf_counter()
        save_flag = False
        discard_flag = False

        try:
            while viz.running:
                # 1. Check keyboard state
                with self.state_lock:
                    if self.want_quit or self.want_discard:
                        discard_flag = True
                        break
                    if self.want_save_end:
                        save_flag = True
                        break

                # 2. Get input from the Haply device
                # The returned action is assumed to be 7D: [x, y, z, rx, ry, rz, gripper]
                raw_action = viz.get_action(mode="euler", seq="xyz", degrees=False)

                # 3. Check for stillness (ignoring the gripper value at the end)
                if np.all(np.abs(raw_action[:6]) <= STILL_THRESHOLD):
                    time.sleep(0.01)
                    continue

                # 4. Process the action
                pos_delta = np.asarray(raw_action[:3]) * POS_GAIN

                # Note: The original code zeroes out the rotational delta completely after
                # calculating it. If you ever want to re-enable rotation, you would do:
                # rot_delta = np.asarray(raw_action[3:6]) * ROT_GAIN
                rot_delta = np.zeros(3)

                gripper = [raw_action[-1]]

                # Recombine into a 7D action array
                action_to_send = np.concatenate([pos_delta, rot_delta, gripper])

                # 5. Send the action to the robot environment
                t_next += dt
                env.send_action(
                    action_to_send,
                    non_blocking=True,
                    target_freq=hz,
                    feedback_enabled=False,
                )

                # 6. Sleep to maintain the target control frequency (hz)
                sleep_for = t_next - time.perf_counter()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    t_next = time.perf_counter()

        finally:
            # Teardown: Save data if requested, and gracefully shut down the environment
            if save_flag:
                try:
                    env.save_data()
                except Exception as e:
                    print(f"⚠️ save_data() failed: {e}")
            try:
                env.shutdown()
            except Exception as e:
                print(f"⚠️ shutdown() failed: {e}")

        return "save" if save_flag else ("discard" if discard_flag else "end")

    def start(self):
        """Main loop managing the application lifecycle and idle states."""
        print("Ready. s=start, e=save+end, d=discard, Esc=quit")
        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()

        try:
            while True:
                # Check if we should start a session or quit
                with self.state_lock:
                    if self.want_quit:
                        break
                    start_now = self.want_start
                    if start_now:
                        self.want_start = False
                        self.want_save_end = False
                        self.want_discard = False

                if not start_now:
                    time.sleep(0.05)
                    continue

                # Block and run the session until finished
                result = self.run_session(hz=CONTROL_HZ)

                # Reset flags upon returning to the idle state
                with self.state_lock:
                    self.want_save_end = False
                    self.want_discard = False

                if result == "save":
                    print("✅ Saved. Ready.")
                elif result == "discard":
                    print("✖ Discarded. Ready.")
                else:
                    print("⏸ Ended. Ready.")
        finally:
            listener.stop()
            print("Bye.")


if __name__ == "__main__":
    controller = HaplyController()
    controller.start()
