# import time
# import numpy as np
# from threading import Lock
# from pynput import keyboard
# from scipy.spatial.transform import Rotation as R
# import sys
# import os

# # Add the folder path to sys.path
# sys.path.append('/media/Data/demoaccount2/teleop_data_collect')

# from controllers.BooleanGripperCartesianAction import VRPolicy
# from robot_env.RTDE_RW_test_collect import RobotAction

# POS_GAIN = 1.5
# ROT_GAIN = 0.8
# STILL_THRESHOLD = 5e-4

# # --- global flags ---
# state_lock = Lock()
# want_start = False
# want_save_end = False
# want_discard = False
# want_quit = False

# def on_press(key):
#     global want_start, want_save_end, want_discard, want_quit
#     try:
#         k = key.char
#     except AttributeError:
#         k = None
#     with state_lock:
#         if key == keyboard.Key.esc:
#             want_quit = True
#             print("⏹ QUIT")
#         elif k == 's':
#             want_start = True
#             want_save_end = False
#             want_discard = False
#             print("▶ START")
#         elif k == 'e':
#             want_save_end = True
#             print("💾 SAVE END")
#         elif k == 'd':
#             want_discard = True
#             print("🗑️ DISCARD")

# def quat_inverse(q):
#     w, x, y, z = q
#     norm_sq = w*w + x*x + y*y + z*z
#     return np.array([w, -x, -y, -z]) / norm_sq

# def quat_multiply(q1, q2):
#     w1, x1, y1, z1 = q1
#     w2, x2, y2, z2 = q2
#     w = w1*w2 - x1*x2 - y1*y2 - z1*z2
#     x = w1*x2 + x1*w2 + y1*z2 - z1*y2
#     y = w1*y2 - x1*z2 + y1*w2 + z1*x2
#     z = w1*z2 + x1*y2 - y1*x2 + z1*w2
#     return np.array([w, x, y, z])

# def run_session(hz=6, right_controller=True):
#     dt = 1.0 / hz
#     env = RobotAction(control_hz=hz)
#     controller = VRPolicy(right_controller=right_controller)

#     prev_pos = None
#     prev_quat = None

#     t_next = time.perf_counter()
#     save_flag = False
#     discard_flag = False

#     print("Waiting for VR controller to be ready...")

#     try:
#         while True:
#             with state_lock:
#                 if want_quit:
#                     discard_flag = True
#                     break
#                 if want_save_end:
#                     save_flag = True
#                     break
#                 if want_discard:
#                     discard_flag = True
#                     break

#             if controller._state["poses"] == {} or not controller._state["controller_on"]:
#                 print("Waiting for VR controller to move...")
#                 time.sleep(0.5)
#                 continue

#             controller._process_reading()
#             vr_state = controller.vr_state

#             curr_pos = np.array(vr_state['pos'])
#             curr_quat = np.array(vr_state['quat'])  # [w,x,y,z]

#             if prev_pos is None or prev_quat is None:
#                 prev_pos = curr_pos
#                 prev_quat = curr_quat
#                 time.sleep(0.01)
#                 continue

#             # Compute delta position
#             delta_pos = curr_pos - prev_pos

#             # Compute delta quaternion: q_delta = q_curr * inverse(q_prev)
#             q_prev_inv = quat_inverse(prev_quat)
#             delta_quat = quat_multiply(curr_quat, q_prev_inv)

#             # Convert delta quaternion to Euler angles (radians)
#             # scipy expects [x,y,z,w]
#             r = R.from_quat(delta_quat[[1, 2, 3, 0]])
#             delta_euler = r.as_euler('xyz', degrees=False)

#             # Check if movement is above threshold
#             if np.all(np.abs(np.concatenate([delta_pos, delta_euler])) < STILL_THRESHOLD):
#                 time.sleep(0.01)
#                 continue

#             pos_delta_scaled = delta_pos * POS_GAIN
#             rot_delta_scaled = delta_euler * ROT_GAIN

#             # The gripper state (boolean or float) from vr_state
#             gripper_val = vr_state.get('gripper', 0.0)

#             # Compose 7-element action vector [x,y,z, rx, ry, rz, gripper]
#             action7 = np.concatenate([pos_delta_scaled, rot_delta_scaled, [gripper_val]])

#             env.send_action(action7, non_blocking=True, target_freq=hz, feedback_enabled=False)

#             prev_pos = curr_pos
#             prev_quat = curr_quat

#             t_next += dt
#             sleep_for = t_next - time.perf_counter()
#             if sleep_for > 0:
#                 time.sleep(sleep_for)
#             else:
#                 t_next = time.perf_counter()

#     finally:
#         if save_flag:
#             try:
#                 env.save_data()
#             except Exception:
#                 print("⚠️ save_data() failed")
#         try:
#             env.shutdown()
#         except Exception:
#             print("⚠️ shutdown() failed")

#     return 'save' if save_flag else ('discard' if discard_flag else 'end')

# def main():
#     global want_start, want_save_end, want_discard, want_quit
#     print("Ready. s=start, e=save+end, d=discard, Esc=quit")
#     listener = keyboard.Listener(on_press=on_press)
#     listener.start()

#     try:
#         while True:
#             with state_lock:
#                 if want_quit:
#                     break
#                 start_now = want_start
#                 if start_now:
#                     want_start = False
#                     want_save_end = False
#                     want_discard = False

#             if not start_now:
#                 time.sleep(0.05)
#                 continue

#             result = run_session(hz=6, right_controller=True)

#             with state_lock:
#                 want_save_end = False
#                 want_discard = False

#             if result == 'save':
#                 print("✅ Saved. Ready.")
#             elif result == 'discard':
#                 print("✖ Discarded. Ready.")
#             else:
#                 print("⏸ Ended. Ready.")

#     finally:
#         listener.stop()
#         print("Bye.")

# if __name__ == "__main__":
#     main()


# import time
# import numpy as np
# from threading import Lock
# from pynput import keyboard
# from scipy.spatial.transform import Rotation as R
# import sys
# import os

# sys.path.append('/media/Data/demoaccount2/teleop_data_collect')

# from controllers.BooleanGripperCartesianAction import VRPolicy
# from robot_env.RTDE_RW_test_collect import RobotAction

# POS_GAIN = 1.5
# ROT_GAIN = 0.8
# STILL_THRESHOLD = 5e-4

# state_lock = Lock()
# want_start = False
# want_save_end = False
# want_discard = False
# want_quit = False

# def on_press(key):
#     global want_start, want_save_end, want_discard, want_quit
#     try:
#         k = key.char
#     except AttributeError:
#         k = None
#     with state_lock:
#         if key == keyboard.Key.esc:
#             want_quit = True
#             print("⏹ QUIT")
#         elif k == 's':
#             want_start = True
#             want_save_end = False
#             want_discard = False
#             print("▶ START")
#         elif k == 'e':
#             want_save_end = True
#             print("💾 SAVE END")
#         elif k == 'd':
#             want_discard = True
#             print("🗑️ DISCARD")

# def quat_inverse(q):
#     w, x, y, z = q
#     return np.array([w, -x, -y, -z]) / (w*w + x*x + y*y + z*z)

# def quat_multiply(q1, q2):
#     w1, x1, y1, z1 = q1
#     w2, x2, y2, z2 = q2
#     return np.array([
#         w1*w2 - x1*x2 - y1*y2 - z1*z2,
#         w1*x2 + x1*w2 + y1*z2 - z1*y2,
#         w1*y2 - x1*z2 + y1*w2 + z1*x2,
#         w1*z2 + x1*y2 - y1*x2 + z1*w2
#     ])

# def block_until_enabled(controller, dt):
#     """Blocks sending until info['movement_enabled'] is True. Returns synced (pos, quat)."""
#     while True:
#         with state_lock:
#             if want_quit or want_save_end or want_discard:
#                 return None, None, True
#         if controller._state.get("poses", {}) == {} or not controller._state.get("controller_on", False):
#             time.sleep(dt)
#             continue
#         controller._process_reading()
#         vr_state = controller.vr_state
#         info = controller.get_info()
#         if info.get("movement_enabled", False):
#             pos = np.array(vr_state['pos'])
#             quat = np.array(vr_state['quat'])  # [w,x,y,z]
#             return pos, quat, False
#         time.sleep(dt)

# def run_session(hz=6, right_controller=True):
#     dt = 1.0 / hz
#     env = RobotAction(control_hz=hz)
#     controller = VRPolicy(right_controller=right_controller)

#     prev_pos = None
#     prev_quat = None

#     t_next = time.perf_counter()
#     save_flag = False
#     discard_flag = False

#     print("Waiting for VR controller to be ready...")

#     try:
#         while True:
#             with state_lock:
#                 if want_quit:
#                     discard_flag = True
#                     break
#                 if want_save_end:
#                     save_flag = True
#                     break
#                 if want_discard:
#                     discard_flag = True
#                     break

#             if controller._state["poses"] == {} or not controller._state["controller_on"]:
#                 print("Waiting for VR controller to move...")
#                 time.sleep(0.5)
#                 continue

#             controller._process_reading()
#             vr_state = controller.vr_state
#             info = controller.get_info()

#             # BLOCK until movement_enabled is pressed
#             while not info.get("movement_enabled", False):
#                 time.sleep(0.01)
#                 controller._process_reading()
#                 info = controller.get_info()
#                 vr_state = controller.vr_state
#                 prev_pos = np.array(vr_state['pos'])
#                 prev_quat = np.array(vr_state['quat'])

#             curr_pos = np.array(vr_state['pos'])
#             curr_quat = np.array(vr_state['quat'])  # [w,x,y,z]

#             if prev_pos is None or prev_quat is None:
#                 prev_pos = curr_pos
#                 prev_quat = curr_quat
#                 time.sleep(0.01)
#                 continue

#             delta_pos = curr_pos - prev_pos
#             q_prev_inv = quat_inverse(prev_quat)
#             delta_quat = quat_multiply(curr_quat, q_prev_inv)
#             r = R.from_quat(delta_quat[[1, 2, 3, 0]])
#             delta_euler = r.as_euler('xyz', degrees=False)

#             if np.all(np.abs(np.concatenate([delta_pos, delta_euler])) < STILL_THRESHOLD):
#                 time.sleep(0.01)
#                 continue

#             pos_delta_scaled = delta_pos * POS_GAIN
#             rot_delta_scaled = delta_euler * ROT_GAIN
#             gripper_val = vr_state.get('gripper', 0.0)

#             action7 = np.concatenate([pos_delta_scaled, rot_delta_scaled, [gripper_val]])
#             env.send_action(action7, non_blocking=True, target_freq=hz, feedback_enabled=False)

#             prev_pos = curr_pos
#             prev_quat = curr_quat

#             t_next += dt
#             sleep_for = t_next - time.perf_counter()
#             if sleep_for > 0:
#                 time.sleep(sleep_for)
#             else:
#                 t_next = time.perf_counter()

#     finally:
#         if save_flag:
#             try:
#                 print(f"Tried")
#                 env.save_data()
#             except Exception:
#                 print("⚠️ save_data() failed")
#         try:
#             env.shutdown()
#         except Exception:
#             print("⚠️ shutdown() failed")

#     return 'save' if save_flag else ('discard' if discard_flag else 'end')

# def main():
#     global want_start, want_save_end, want_discard, want_quit
#     print("Ready. s=start, e=save+end, d=discard, Esc=quit")
#     listener = keyboard.Listener(on_press=on_press)
#     listener.start()

#     try:
#         while True:
#             with state_lock:
#                 if want_quit:
#                     break
#                 start_now = want_start
#                 if start_now:
#                     want_start = False
#                     want_save_end = False
#                     want_discard = False

#             if not start_now:
#                 time.sleep(0.05)
#                 continue

#             result = run_session(hz=6, right_controller=True)

#             with state_lock:
#                 want_save_end = False
#                 want_discard = False

#             if result == 'save':
#                 print("✅ Saved. Ready.")
#             elif result == 'discard':
#                 print("✖ Discarded. Ready.")
#             else:
#                 print("⏸ Ended. Ready.")

#     finally:
#         listener.stop()
#         print("Bye.")

# if __name__ == "__main__":
#     main()



import time
import numpy as np
from threading import Lock
from pynput import keyboard
from scipy.spatial.transform import Rotation as R
import sys
import os

# Add the folder path to sys.path
sys.path.append('/media/Data/demoaccount2/teleop_data_collect')

from controllers.BooleanGripperCartesianAction import VRPolicy
from robot_env.RTDE_RW_test_collect import RobotAction

POS_GAIN = 1
ROT_GAIN = 0.3
STILL_THRESHOLD = 5e-4

# --- global flags ---
state_lock = Lock()
want_start = False
want_save_end = False
want_discard = False
want_quit = False

def on_press(key):
    global want_start, want_save_end, want_discard, want_quit
    try:
        k = key.char
    except AttributeError:
        k = None
    with state_lock:
        if key == keyboard.Key.esc:
            want_quit = True
            print("⏹ QUIT")
        elif k == 's':
            want_start = True
            want_save_end = False
            want_discard = False
            print("▶ START")
        elif k == 'e':
            want_save_end = True
            print("💾 SAVE END")
        elif k == 'd':
            want_discard = True
            print("🗑️ DISCARD")

def quat_inverse(q):
    w, x, y, z = q
    n2 = w*w + x*x + y*y + z*z
    return np.array([w, -x, -y, -z]) / (n2 if n2 > 0 else 1.0)

def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def _fmt3(v):
    return f"[{v[0]:.4f} {v[1]:.4f} {v[2]:.4f}]"
def run_session(hz=6, right_controller=True):
    dt = 1.0 / hz
    env = RobotAction(control_hz=hz)
    controller = VRPolicy(right_controller=right_controller)

    prev_pos = None
    prev_quat = None
    just_enabled = False  # <-- edge flag

    t_next = time.perf_counter()
    save_flag = False
    discard_flag = False

    def _fmt3(v): return f"[{v[0]:.4f} {v[1]:.4f} {v[2]:.4f}]"

    print("Waiting for VR controller to be ready...")

    try:
        while True:
            with state_lock:
                if want_quit:
                    discard_flag = True
                    break
                if want_save_end:
                    print("want_save_end true")
                    save_flag = True
                    break
                if want_discard:
                    discard_flag = True
                    break

            # Need live controller
            if controller._state.get("poses", {}) == {} or not controller._state.get("controller_on", False):
                print("Waiting for VR controller to move...")
                time.sleep(0.5)
                continue

            # Read current frame
            controller._process_reading()
            vr_state = controller.vr_state
            info = controller.get_info()

            # -------- BLOCK WHILE DISABLED --------
            if not info.get("movement_enabled", False):
                # Stay here until enabled, printing DISABLED status
                while True:
                    controller._process_reading()
                    vr_state = controller.vr_state
                    info = controller.get_info()
                    g = vr_state.get('gripper', 0.0)
                    print(f"[Δpos: [0.0000 0.0000 0.0000], Δrot: [0.0000 0.0000 0.0000], gripper: {g:.4f}, DISABLED]")
                    with state_lock:
                        if want_quit:
                            discard_flag = True
                            break
                        if want_discard:
                            discard_flag = True
                            break
                        if want_save_end:
                            save_flag = True
                            break
                    if info.get("movement_enabled", False):
                        # we will resync once on the next loop body
                        just_enabled = True
                        break
                    time.sleep(0.01)
                # after enabling, fall through

            # Refresh once after unblock
            controller._process_reading()
            vr_state = controller.vr_state
            curr_pos = np.array(vr_state['pos'], dtype=float)
            curr_quat = np.array(vr_state['quat'], dtype=float)  # [w,x,y,z]

            # -------- RESYNC ONCE ON ENABLE EDGE --------
            if just_enabled or prev_pos is None or prev_quat is None:
                prev_pos = curr_pos
                prev_quat = curr_quat
                just_enabled = False
                print(f"[Δpos: [0.0000 0.0000 0.0000], Δrot: [0.0000 0.0000 0.0000], "
                      f"gripper: {vr_state.get('gripper', 0.0):.4f}, ENABLED (RESYNC)]")
                # skip sending this tick to avoid snap
                t_next = time.perf_counter()
                time.sleep(0.01)
                continue

            # -------- NORMAL DELTAS --------
            delta_pos = curr_pos - prev_pos
            dq = quat_multiply(curr_quat, quat_inverse(prev_quat))
            n = np.linalg.norm(dq)
            if n > 0:
                dq = dq / n
            try:
                delta_euler = R.from_quat(dq[[1, 2, 3, 0]]).as_euler('xyz', degrees=False)
            except Exception:
                delta_euler = np.zeros(3, dtype=float)

            # Threshold check (still print so you can see loop health)
            if np.all(np.abs(np.concatenate([delta_pos, delta_euler])) < STILL_THRESHOLD):
                print(f"[Δpos: {_fmt3(np.zeros(3))}, Δrot: {_fmt3(np.zeros(3))}, "
                      f"gripper: {vr_state.get('gripper', 0.0):.4f}, ENABLED]")
                prev_pos = curr_pos
                prev_quat = curr_quat
                time.sleep(0.01)
                # pacing
                t_next += dt
                sleep_for = t_next - time.perf_counter()
                if sleep_for > 0: time.sleep(sleep_for)
                else: t_next = time.perf_counter()
                continue

            pos_delta_scaled = delta_pos * POS_GAIN
            rot_delta_scaled = delta_euler * ROT_GAIN
            gripper_val = vr_state.get('gripper', 0.0)

            print(f"[Δpos: {_fmt3(pos_delta_scaled)}, Δrot: {_fmt3(rot_delta_scaled)}, "
                  f"gripper: {gripper_val:.4f}, ENABLED]")

            # Send action
            action7 = np.concatenate([pos_delta_scaled, rot_delta_scaled, [gripper_val]])
            env.send_action(action7, non_blocking=True, target_freq=hz, feedback_enabled=False)

            # Update history
            prev_pos = curr_pos
            prev_quat = curr_quat

            # pacing
            t_next += dt
            sleep_for = t_next - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                t_next = time.perf_counter()

    finally:
        if save_flag:
            try:
                print("[INFO] Attempting to save data...")
                env.save_data()
                print("[INFO] Data saved.")
            except Exception:
                print("⚠️ save_data() failed")
        try:
            env.shutdown()
        except Exception:
            print("⚠️ shutdown() failed")

    return 'save' if save_flag else ('discard' if discard_flag else 'end')

def main():
    global want_start, want_save_end, want_discard, want_quit
    print("Ready. s=start, e=save+end, d=discard, Esc=quit")
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    try:
        while True:
            with state_lock:
                if want_quit:
                    break
                start_now = want_start
                if start_now:
                    want_start = False
                    want_save_end = False
                    want_discard = False

            if not start_now:
                time.sleep(0.05)
                continue

            result = run_session(hz=6, right_controller=True)

            with state_lock:
                want_save_end = False
                want_discard = False

            if result == 'save':
                print("✅ Saved. Ready.")
            elif result == 'discard':
                print("✖ Discarded. Ready.")
            else:
                print("⏸ Ended. Ready.")

    finally:
        listener.stop()
        print("Bye.")

if __name__ == "__main__":
    main()
