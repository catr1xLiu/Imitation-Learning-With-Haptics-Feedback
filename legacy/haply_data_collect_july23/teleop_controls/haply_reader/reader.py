import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from misc.subprocess_utils import run_threaded_command
import math
import numpy as np
import threading
import time
import orjson
from collections import deque
from websockets.sync.client import connect  # Requires websockets >= 12

class ForceFilter:
    def __init__(self,
                 lowpass_fc=23, # Hz
                 max_rate=15.0, # N
                 scale=0.24):

        self.lowpass_fc = lowpass_fc
        self.max_rate = max_rate

        self.ema = None
        self.last_out = [0.0, 0.0, 0.0]
        self.last_t = None

        self.scale = scale

        self.wall_mode = [False, False, False]
        self.wall_position = np.zeros(3)
        self.cursor_position = np.zeros(3)
        self.cursor_position_offset = np.zeros(3)
        self.robot_position_at_entry = np.zeros(3)
        self.wall_blend = 0.0  # 0 = pure sensor force, 1 = pure wall force

    def alpha(self, dt):
        if self.lowpass_fc <= 0 or dt <= 0:
            return 1.0
        RC = 1.0 / (2.0 * math.pi * self.lowpass_fc)
        return dt / (RC + dt)

    def feed(self, raw_ft, poses, active, begin_run,
             F_ENTER=8.0,
             F_EXIT=5.0,
             K_MASTER=3500.0,   # Wall stiffness (N/m) 
             B_MASTER=8.0,      # Wall damping (N·s/m) 
             BLEND_RATE=10.0):  # Rate of blending between modes (1/s)
        """
        raw_ft : [fx, fy, fz]
        poses: dict with "position" and "velocity"
        active: motion status of robot
        begin_run:
        returns: smoothed [fx, fy, fz]
        """

        # Accumulate position offset of cursor when user is not active 
        if not active and begin_run and self.wall_mode[2]:
            neg_pos = [-v for v in poses["position"]]
            self.cursor_position_offset = neg_pos + self.cursor_position
            return [0, 0, 0]

        now = time.time()
        
        fx, fy, fz = raw_ft

        if self.last_t is None:
            dt = 1.0 / 100.0
        else:
            dt = now - self.last_t
            if dt <= 0 or dt > 1.0:
                dt = 1.0 / 100.0
        self.last_t = now
        
        # Low pass
        a = self.alpha(dt)
        if self.ema is None:
            self.ema = np.array([fx, fy, fz])
        else:
            self.ema = np.array([
                a*fx + (1-a)*self.ema[0],
                a*fy + (1-a)*self.ema[1],
                a*fz + (1-a)*self.ema[2],
            ])

        out = self.ema

        # Rate limiter
        max_step = self.max_rate * dt
        out = np.array([
            self.last_out[i] + max(-max_step, min(out[i] - self.last_out[i], max_step))
            for i in range(3)
        ])

        # Virtualization
        f_wall = out.copy()
        pos = np.array(poses["position"])
        vel = np.array(poses["velocity"])
        self.cursor_position = pos
        
        if not self.wall_mode[2] and out[2] > F_ENTER and raw_ft[2] > F_ENTER:
            self.wall_mode[2] = True
            
            estimated_penetration = -(out[2] / K_MASTER)  # F = k*x, so x = F/k
            
            # Place wall surface behind current position
            self.wall_position = pos[2] - estimated_penetration
            
            
        elif self.wall_mode[2]:
            d = -(pos[2] + self.cursor_position_offset[2] - self.wall_position)

            if ((out[2] < F_EXIT and
                raw_ft[2]< F_EXIT) or
                (raw_ft[2] < 0.5)): 
                self.wall_mode[2] = False
                self.wall_blend = 0.0
                self.cursor_position_offset[2] = 0
        
        if self.wall_mode[2]:
            d = -(pos[2] + self.cursor_position_offset[2] - self.wall_position)
            v_n = -vel[2]
            if d > 0:
                # Spring-damper force along wall normal
                f_wall[2] = K_MASTER * d + B_MASTER * v_n
            else:
                # If pulled back to or past the wall surface, apply zero force
                f_wall[2] = 0
        
        # Gradually blend between sensor force and wall force
        target_blend = 1.0 if self.wall_mode[2] else 0.0
        blend_step = BLEND_RATE * dt
        
        if self.wall_blend < target_blend:
            self.wall_blend = min(target_blend, self.wall_blend + blend_step)
        elif self.wall_blend > target_blend:
            self.wall_blend = max(target_blend, self.wall_blend - blend_step)
        f_output = (1.0 - self.wall_blend) * out + self.wall_blend * f_wall

        out = [float(f_output[0]), float(f_output[1]), float(f_output[2])]
        
        self.last_out = out
        return [float(o) * self.scale for o in out]
    
class HapticReader:
    def __init__(self, uri='ws://localhost:10001', haptic_feedback=False, force_reader=None):
        self.uri = uri
        self.force_reader = force_reader
        self.current_force ={}
        self.poses = {}
        self.buttons = {}
        self.running = True
        self._lock = threading.Lock()
        self.haptic_feedback = haptic_feedback
        if haptic_feedback:  
            self.force_filter = ForceFilter()
            self.begin_run = False
        run_threaded_command(self.connect_and_read)

    def connect_and_read(self):
        first_message = True

        inverse3_device_id = None

        try:
            with connect(self.uri) as ws:

                active = False
                prev_button_a = False

                while self.running:
                    
                    # Read haptic status
                    response = ws.recv()
                    data = orjson.loads(response)
                    inverse3_devices = data.get("inverse3", [])
                    verse_grip_devices = data.get("wireless_verse_grip", [])

                    inverse3_data = inverse3_devices[0] if inverse3_devices else {}
                    verse_grip_data = verse_grip_devices[0] if verse_grip_devices else {}

                    if first_message:
                        first_message = False
                        if not inverse3_data:
                            print("No Inverse3 device found.")
                            break
                        if not verse_grip_data:
                            print("No Wireless Verse Grip device found.")
                        inverse3_device_id = inverse3_data.get("device_id")
                        handedness = inverse3_data.get("config", {}).get("handedness")
                        print(f"Inverse3 device ID: {inverse3_device_id}, Handedness: {handedness}")

                    position_raw = inverse3_data["state"].get("cursor_position", {})
                    velocity_raw = inverse3_data["state"].get("cursor_velocity", {})
                    buttons = verse_grip_data.get("state", {}).get("buttons", {})
                    orientation_raw = verse_grip_data.get("state", {}).get("orientation", {})

                    position = [
                        position_raw.get("x", 0.0),
                        position_raw.get("y", 0.0),
                        position_raw.get("z", 0.0)
                    ]
                    orientation = [
                        orientation_raw.get("x", 0.0),
                        orientation_raw.get("y", 0.0),
                        orientation_raw.get("z", 0.0),
                        orientation_raw.get("w", 1.0)
                    ]
                    velocity = [
                        velocity_raw.get("x", 0.0),
                        velocity_raw.get("y", 0.0),
                        velocity_raw.get("z", 0.0)
                    ]

                    with self._lock:
                        self.poses = {
                            "position": position,
                            "velocity": velocity,
                            "orientation": orientation
                        }
                        self.buttons = buttons

                    # Force Feedback
                    fx, fy, fz = [0, 0, 0]
                    
                    # Check active state
                    if self.haptic_feedback:
                        # KILL process if run_session has finished
                        if self.force_reader.has_shutdown:
                            return 0

                        a_pressed = bool(self.buttons.get('a', False))
                        if a_pressed and not prev_button_a:
                            active = not active
                        prev_button_a = a_pressed

                        if not self.begin_run and active:
                            self.begin_run = True

                        # Only apply force if active
                        force_vector = self.force_filter.feed((self.force_reader.get_fts_observation())[:3], self.poses, active, self.begin_run)
                        fx, fy, fz = force_vector

                    self.current_force = {"x": fx, "y": fy, "z": fz}

                    request_msg = {
                        "inverse3": [
                            {
                                "device_id": inverse3_device_id,
                                "commands": {
                                    "set_cursor_force": {
                                        "values": self.current_force
                                    }
                                }
                            }
                        ]
                    }

                    ws.send(orjson.dumps(request_msg))

        except Exception as e:
            print(f"WebSocket connection error: {e}")

    def stop(self):
        self.running = False

    def get_device_state(self):
        with self._lock:
            return self.poses, self.buttons

def main():
    haptic_reader = HapticReader()
    # Start the threaded WebSocket reader
    try:
        while haptic_reader.running:
            poses, buttons = haptic_reader.get_device_state()
            print("Poses:", poses)
            print("Buttons:", buttons)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("Stopping...")
        haptic_reader.stop()


if __name__ == '__main__':
    main()
