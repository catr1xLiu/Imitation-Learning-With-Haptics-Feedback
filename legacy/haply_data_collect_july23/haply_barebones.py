# import time
# import threading
# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# from pynput import keyboard
# from teleop_controls.haply_reader.reader import HapticReader
# from teleop_controls.misc.transformations import quat_to_rmat, quat_to_euler
# from scipy.spatial.transform import Rotation as R
# from robot_env_corrected.RTDE_RW_test_collect import RobotAction

# class HapticVisualizer:
#     """
#     Streams live pose and orientation from the Haply device and computes delta-based actions.

#     Attributes:
#         pose (np.ndarray of shape (3,)): current [x, y, z] position (m)
#         orient (np.ndarray of shape (3,3)): current rotation matrix
#         quat (np.ndarray of shape (4,)): current quaternion [x, y, z, w]
#         prev_pose, prev_euler, prev_quat: previous state values to compute deltas
#     """

#     def __init__(self):
#         # Shared state
#         self.running = True
#         self.calibrated = False
#         self.haply_to_global = np.eye(4)
#         self.pose = np.zeros(3)
#         self.orient = np.eye(3)
#         self.quat = np.array([0, 0, 0, 1])
#         self.mode = 0
#         self.view_idx = 0
#         self.views = [(0, 270), (30, 45), (90, 0), (0, 0), (0, 90)]
#         # Store previous state for delta calculations
#         self.prev_pose = self.pose.copy()
#         self.prev_euler = np.zeros(3)
#         self.prev_quat = self.quat.copy()

#         # Keyboard listener
#         listener = keyboard.Listener(on_press=self.on_press)
#         listener.daemon = True
#         listener.start()
#         # Haptic reader thread
#         self.reader = HapticReader()
#         t = threading.Thread(target=self._update_loop, daemon=True)
#         t.start()
#         # Setup interactive plot
#         plt.ion()
#         self._init_plot()

#     def interpret_rotation(self, quat, mode):
#         if mode == 0:
#             Rm = R.from_quat(quat).as_matrix()[:, [2, 0, 1]]
#             return Rm, R.from_matrix(Rm).as_euler('zyx', True)
#         return np.eye(3), np.zeros(3)

#     def on_press(self, key):
#         c = getattr(key, 'char', None)
#         if c == 'a':
#             if not self.calibrated:
#                 T = np.eye(4); T[:3, 3] = self.pose
#                 self.haply_to_global = np.linalg.inv(T)
#                 self.calibrated = True
#             else:
#                 self.mode = (self.mode + 1) % len(self.views)
#         elif c == 'z':
#             self.view_idx = (self.view_idx + 1) % len(self.views)
#             elev, azim = self.views[self.view_idx]
#             self.ax.view_init(elev=elev, azim=azim)
#         elif key == keyboard.Key.esc:
#             self.stop(); plt.close(self.fig)

#     def _update_loop(self):
#         while self.running:
#             poses, _ = self.reader.get_device_state()
#             if poses:
#                 self.quat = poses['orientation']
#                 pos = poses['position']
#                 rot, _ = self.interpret_rotation(self.quat, self.mode)
#                 T = np.eye(4); T[:3, 3] = pos; T[:3, :3] = rot
#                 if self.calibrated: T = self.haply_to_global @ T
#                 self.pose = T[:3, 3]; self.orient = T[:3, :3]
#             time.sleep(0.05)

#     def _init_plot(self):
#         self.fig = plt.figure()
#         self.ax = self.fig.add_subplot(111, projection='3d')
#         self.ax.set(xlim=(-0.2, 0.2), ylim=(-0.2, 0.2), zlim=(0, 0.2),
#                     xlabel='X', ylabel='Y', zlabel='Z')
#         elev, azim = self.views[self.view_idx]
#         self.ax.view_init(elev=elev, azim=azim)
#         # ground plane
#         xx, yy = np.meshgrid(np.linspace(-0.2, 0.2, 10), np.linspace(-0.2, 0.2, 10))
#         self.ax.plot_surface(xx, yy, np.zeros_like(xx), color='gray', alpha=0.5)
#         # cylinder mesh
#         r, h = 0.01, 0.05
#         th = np.linspace(0, 2*np.pi, 20)
#         zc = np.array([0, -h])
#         tg, zcg = np.meshgrid(th, zc)
#         self.xc = r * np.cos(tg); self.yc = r * np.sin(tg); self.zg = zcg
#         # initialize plot elements
#         o = self.pose
#         self.pt, = self.ax.plot([o[0]], [o[1]], [o[2]], 'ko')
#         self.lines = []
#         ends = [o + self.orient[:, i] * 0.05 for i in range(3)]
#         for end, style in zip(ends, ['r-', 'g-', 'b-']):
#             ln, = self.ax.plot([o[0], end[0]], [o[1], end[1]], [o[2], end[2]], style)
#             self.lines.append(ln)
#         pts = np.vstack([self.xc.flatten(), self.yc.flatten(), self.zg.flatten()])
#         pw = (self.orient @ pts).T + o
#         X = pw[:, 0].reshape(self.zg.shape)
#         Y = pw[:, 1].reshape(self.zg.shape)
#         Z = pw[:, 2].reshape(self.zg.shape)
#         self.cyl = self.ax.plot_surface(X, Y, Z, alpha=0.7)
#         self.ani = FuncAnimation(self.fig, self._animate, interval=50)
#         self.fig.canvas.draw()

#     def _animate(self, _):
#         o = self.pose
#         ends = [o + self.orient[:, i] * 0.05 for i in range(3)]
#         self.pt.set_data([o[0]], [o[1]]); self.pt.set_3d_properties([o[2]])
#         for ln, end in zip(self.lines, ends):
#             ln.set_data([o[0], end[0]], [o[1], end[1]])
#             ln.set_3d_properties([o[2], end[2]])
#         if self.cyl: self.cyl.remove()
#         pts = np.vstack([self.xc.flatten(), self.yc.flatten(), self.zg.flatten()])
#         pw = (self.orient @ pts).T + o
#         X = pw[:, 0].reshape(self.zg.shape)
#         Y = pw[:, 1].reshape(self.zg.shape)
#         Z = pw[:, 2].reshape(self.zg.shape)
#         self.cyl = self.ax.plot_surface(X, Y, Z, alpha=0.7)
#         self.fig.canvas.draw_idle()
#         return [self.pt] + self.lines + [self.cyl]

#     def get_state(self):
#         return {'position': self.pose.copy(), 'orientation': self.orient.copy(), 'quaternion': self.quat.copy()}

#     def get_euler(self, seq='zyx', degrees=False):
#         return R.from_matrix(self.orient).as_euler(seq, degrees)

#     def stop(self):
#         self.running = False

#     def get_action(self, mode='euler', seq='zyx', degrees=False):
#         """
#         Return a delta-based action vector:
#             - mode='quat': [dx, dy, dz, dqx, dqy, dqz, dqw]
#             - mode='euler': [dx, dy, dz, dalpha, dbeta, dgamma]
#         """
#         curr_pose = self.pose.copy()
#         if mode == 'quat':
#             # position delta
#             dp = curr_pose - self.prev_pose
#             # quaternion delta
#             curr_quat = self.quat.copy()
#             dq = (R.from_quat(curr_quat) * R.from_quat(self.prev_quat).inv()).as_quat()
#             action = np.concatenate([dp, dq])
#             self.prev_pose = curr_pose
#             self.prev_quat = curr_quat
#             return action
#         elif mode == 'euler':
#             dp = curr_pose - self.prev_pose
#             curr_euler = R.from_matrix(self.orient).as_euler(seq, degrees)
#             da = curr_euler - self.prev_euler
#             action = np.concatenate([dp, da])
#             self.prev_pose = curr_pose
#             self.prev_euler = curr_euler

#             x = action[0]
#             y = action[1]

#             action[0] = y *1.5
#             action[1] = x *1.5
#             action[2] = action[2] *1.5
#             action[3] = -action[3]
#             action[4] = -action[4]
#             action[5] = -action[5]
#             return action
#         else:
#             raise ValueError(f"Unknown mode '{mode}', choose 'quat' or 'euler'.")


# # Example usage
# if __name__ == '__main__':
#     viz = HapticVisualizer()
#     env = RobotAction(control_hz=15)
#     env.reset()
#     try:
#         while viz.running:
#             action = viz.get_action(mode='euler', seq='yxz')
#             env.send_action(action)
#             plt.pause(0.001)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         viz.stop()



# import time
# import threading
# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# from pynput import keyboard
# from teleop_controls.haply_reader.reader import HapticReader
# from teleop_controls.misc.transformations import quat_to_rmat, quat_to_euler
# from scipy.spatial.transform import Rotation as R
# from robot_env_corrected.RTDE_RW_test_collect import RobotAction

# class HapticVisualizer:
#     """
#     Streams live pose and orientation from the Haply device and computes delta-based actions,
#     with optional gripper control and button gating.

#     Attributes:
#         pose (np.ndarray of shape (3,)): current [x, y, z] position (m)
#         orient (np.ndarray of shape (3,3)): current rotation matrix
#         quat (np.ndarray of shape (4,)): current quaternion [x, y, z, w]
#         prev_pose, prev_euler, prev_quat: previous state values to compute deltas
#         gripper_state: current gripper position (1.0=open, 0.3=closed)
#         gripper_target: next target gripper position
#         gripper_counter: steps remaining in transition
#         buttons: latest button states from HaplyReader
#     """

#     def __init__(self):
#         # Shared state
#         self.running = True
#         self.calibrated = False
#         self.haply_to_global = np.eye(4)
#         self.pose = np.zeros(3)
#         self.orient = np.eye(3)
#         self.quat = np.array([0, 0, 0, 1])
#         self.mode = 0
#         self.view_idx = 0
#         self.views = [(0, 270), (30, 45), (90, 0), (0, 0), (0, 90)]
#         # Store previous state for delta calculations
#         self.prev_pose = self.pose.copy()
#         self.prev_euler = np.zeros(3)
#         self.prev_quat = self.quat.copy()
#         # Gripper control state
#         self.gripper_state = 1.0
#         self.gripper_target = 1.0
#         self.gripper_counter = 0
#         # Button gating
#         self.buttons = {}
#         # Static configuration
#         self.scalar_xyz = np.array([5, 5, 5])  # scaling for dx, dy, dz
#         self.negate_angles = True                   # whether to invert angle deltas
#         self.gripper_steps = 3                      # number of deltas to transition

#         # Keyboard listener (for view/calibration)
#         listener = keyboard.Listener(on_press=self.on_press)
#         listener.daemon = True
#         listener.start()
#         # Haptic reader thread
#         self.reader = HapticReader()
#         t = threading.Thread(target=self._update_loop, daemon=True)
#         t.start()
#         # Setup interactive plot
#         plt.ion()
#         self._init_plot()

#     def interpret_rotation(self, quat, mode):
#         if mode == 0:
#             Rm = R.from_quat(quat).as_matrix()[:, [2, 0, 1]]
#             return Rm, R.from_matrix(Rm).as_euler('zyx', True)
#         return np.eye(3), np.zeros(3)

#     def on_press(self, key):
#         c = getattr(key, 'char', None)
#         if c == 'a':
#             if not self.calibrated:
#                 T = np.eye(4); T[:3, 3] = self.pose
#                 self.haply_to_global = np.linalg.inv(T)
#                 self.calibrated = True
#             else:
#                 self.mode = (self.mode + 1) % len(self.views)
#         elif c == 'z':
#             self.view_idx = (self.view_idx + 1) % len(self.views)
#             elev, azim = self.views[self.view_idx]
#             self.ax.view_init(elev=elev, azim=azim)
#         elif key == keyboard.Key.esc:
#             self.stop()
#             plt.close(self.fig)

#     def _update_loop(self):
#         while self.running:
#             poses, buttons = self.reader.get_device_state()
#             self.buttons = buttons or {}
#             if poses:
#                 self.quat = poses['orientation']
#                 pos = poses['position']
#                 rot, _ = self.interpret_rotation(self.quat, self.mode)
#                 T = np.eye(4)
#                 T[:3, 3] = pos
#                 T[:3, :3] = rot
#                 if self.calibrated:
#                     T = self.haply_to_global @ T
#                 self.pose = T[:3, 3]
#                 self.orient = T[:3, :3]
#             time.sleep(0.05)

#     def _init_plot(self):
#         self.fig = plt.figure()
#         self.ax = self.fig.add_subplot(111, projection='3d')
#         self.ax.set(
#             xlim=(-0.2, 0.2),
#             ylim=(-0.2, 0.2),
#             zlim=(0, 0.2),
#             xlabel='X',
#             ylabel='Y',
#             zlabel='Z'
#         )
#         elev, azim = self.views[self.view_idx]
#         self.ax.view_init(elev=elev, azim=azim)
#         # ground plane
#         xx, yy = np.meshgrid(np.linspace(-0.2, 0.2, 10), np.linspace(-0.2, 0.2, 10))
#         self.ax.plot_surface(xx, yy, np.zeros_like(xx), color='gray', alpha=0.5)
#         # cylinder mesh
#         r, h = 0.01, 0.05
#         th = np.linspace(0, 2*np.pi, 20)
#         zc = np.array([0, -h])
#         tg, zcg = np.meshgrid(th, zc)
#         self.xc = r * np.cos(tg)
#         self.yc = r * np.sin(tg)
#         self.zg = zcg
#         # initialize plot elements
#         o = self.pose
#         self.pt, = self.ax.plot([o[0]], [o[1]], [o[2]], 'ko')
#         self.lines = []
#         ends = [o + self.orient[:, i] * 0.05 for i in range(3)]
#         for end, style in zip(ends, ['r-', 'g-', 'b-']):
#             ln, = self.ax.plot(
#                 [o[0], end[0]],
#                 [o[1], end[1]],
#                 [o[2], end[2]],
#                 style
#             )
#             self.lines.append(ln)
#         pts = np.vstack([self.xc.flatten(), self.yc.flatten(), self.zg.flatten()])
#         pw = (self.orient @ pts).T + o
#         X = pw[:, 0].reshape(self.zg.shape)
#         Y = pw[:, 1].reshape(self.zg.shape)
#         Z = pw[:, 2].reshape(self.zg.shape)
#         self.cyl = self.ax.plot_surface(X, Y, Z, alpha=0.7)
#         self.ani = FuncAnimation(self.fig, self._animate, interval=50)
#         self.fig.canvas.draw()

#     def _animate(self, _):
#         o = self.pose
#         ends = [o + self.orient[:, i] * 0.05 for i in range(3)]
#         self.pt.set_data([o[0]], [o[1]])
#         self.pt.set_3d_properties([o[2]])
#         for ln, end in zip(self.lines, ends):
#             ln.set_data([o[0], end[0]], [o[1], end[1]])
#             ln.set_3d_properties([o[2], end[2]])
#         if self.cyl:
#             self.cyl.remove()
#         pts = np.vstack([self.xc.flatten(), self.yc.flatten(), self.zg.flatten()])
#         pw = (self.orient @ pts).T + o
#         X = pw[:, 0].reshape(self.zg.shape)
#         Y = pw[:, 1].reshape(self.zg.shape)
#         Z = pw[:, 2].reshape(self.zg.shape)
#         self.cyl = self.ax.plot_surface(X, Y, Z, alpha=0.7)
#         self.fig.canvas.draw_idle()
#         return [self.pt] + self.lines + [self.cyl]

#     def get_state(self):
#         return {
#             'position': self.pose.copy(),
#             'orientation': self.orient.copy(),
#             'quaternion': self.quat.copy()
#         }

#     def get_euler(self, seq='zyx', degrees=False):
#         return R.from_matrix(self.orient).as_euler(seq, degrees)

#     def stop(self):
#         self.running = False

#     def get_action(self, mode='euler', seq='zyx', degrees=False):
#         """
#         Return a delta-based action vector (+ gripper), with hardcoded X/Y swap:
#             - mode='quat': [dy, dx, dz, dqx, dqy, dqz, dqw, gripper]
#             - mode='euler': [dy, dx, dz, dalpha, dbeta, dgamma, gripper]

#         Behavior:
#         1) All axes are zero unless button 'A' is held.
#         2) Button 'B' toggles gripper over gripper_steps.
#         3) Scaling applied, then X/Y swapped.
#         4) Angle inversion if configured.
#         """
#         dp = np.zeros(3)
#         da = np.zeros(3)
#         dq = np.zeros(4)
#         # gating by button A
#         if self.buttons.get('a', False):
#             dp = self.pose - self.prev_pose
#             if mode == 'quat':
#                 curr_quat = self.quat.copy()
#                 dq = (R.from_quat(curr_quat) * R.from_quat(self.prev_quat).inv()).as_quat()
#             else:
#                 curr_euler = R.from_matrix(self.orient).as_euler(seq, degrees)
#                 da = curr_euler - self.prev_euler
#                 if self.negate_angles:
#                     da = -da
#         # scale
#         dp = dp * self.scalar_xyz
#         # hardcode swap X<->Y
#         dp = np.array([dp[1], dp[0], dp[2]])
#         # update prev
#         self.prev_pose = self.pose.copy()
#         self.prev_quat = self.quat.copy()
#         self.prev_euler = R.from_matrix(self.orient).as_euler(seq, degrees)
#         # gripper
#         if self.buttons.get('b', False) and self.gripper_counter == 0:
#             self.gripper_target = 0.3 if self.gripper_state > 0.9 else 1.0
#             self.gripper_counter = self.gripper_steps
#         if self.gripper_counter > 0:
#             step = (self.gripper_target - self.gripper_state) / self.gripper_counter
#             self.gripper_state += step
#             self.gripper_counter -= 1
#         # assemble
#         if mode == 'quat':
#             return np.concatenate([dp, dq, [self.gripper_state]])
#         return np.concatenate([dp, da, [self.gripper_state]])

# # Example usage
# if __name__ == '__main__':
#     viz = HapticVisualizer()
#     env = RobotAction(control_hz=15)
#     env.reset()
#     try:
#         while viz.running:
#             action = viz.get_action(mode='euler', seq='yxz')
#             env.send_action(action)
#             plt.pause(0.001)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         viz.stop()



# import time
# import threading
# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# from pynput import keyboard

# import sys
# sys.path.append('/home/demoaccount/Data/demoaccount2/robot_diffusion/diffusion_policy_jul24')
# from haply_data_collect_july23.teleop_controls.haply_reader.reader import HapticReader
# from scipy.spatial.transform import Rotation as R
# # from robot_env_corrected.RTDE_RW_test_collect import RobotAction

# class HapticVisualizer:
#     """
#     Streams live pose and orientation from the Haply device and computes delta-based actions,
#     with gripper control and active/inactive toggle via Haply button A.

#     Attributes:
#         pose: current [x, y, z]
#         orient: current rotation matrix
#         quat: current quaternion
#         prev_pose, prev_euler, prev_quat: for delta computation
#         gripper_state: open (1.0) or closed (0.3), transitions over gripper_steps
#         active: whether motion deltas are enabled (toggled by Haply A)
#         buttons: latest button states from HapticReader
#     """
#     def __init__(self, display_plot=True):
#         # State
#         self.running = True
#         self.calibrated = False
#         self.haply_to_global = np.eye(4)
#         self.pose = np.zeros(3)
#         self.orient = np.eye(3)
#         self.quat = np.array([0,0,0,1])
#         # Delta history
#         self.prev_pose = self.pose.copy()
#         self.prev_euler = np.zeros(3)
#         self.prev_quat = self.quat.copy()
#         # Gripper
#         self.gripper_state = 1.0
#         self.gripper_target = 1.0
#         self.gripper_counter = 0
#         self.gripper_steps = 5
#         # Toggle active
#         self.active = False
#         self.prev_button_a = False
#         # Config
#         self.scalar_xyz = np.array([1.5,1.5,1.5])
#         self.scalar_rot = np.array([.5,.5,-.5])
#         self.negate_angles = True
#         # Buttons
#         self.buttons = {}
#         # Views
#         self.mode = 0
#         self.view_idx = 0
#         self.views = [(0,270),(30,45),(90,0),(0,0),(0,90)]
#         # Threads
#         listener = keyboard.Listener(on_press=self.on_press)
#         listener.daemon = True
#         listener.start()
#         self.reader = HapticReader()
#         threading.Thread(target=self._update_loop, daemon=True).start()
#         # Plot
#         self.display_plot = True  # Default behavior
#         self.fig = None
#         if self.display_plot:
#             plt.ion()
#             self._init_plot()


#     def interpret_rotation(self, quat, mode):
#         if mode==0:
#             Rm = R.from_quat(quat).as_matrix()[:,[2,0,1]]
#             return Rm, R.from_matrix(Rm).as_euler('zyx',True)
#         return np.eye(3), np.zeros(3)

#     def on_press(self, key):
#         c = getattr(key,'char',None)
#         if c=='a':
#             if not self.calibrated:
#                 T=np.eye(4);T[:3,3]=self.pose
#                 self.haply_to_global=np.linalg.inv(T)
#                 self.calibrated=True
#             else:
#                 self.mode=(self.mode+1)%len(self.views)
#         elif c=='z':
#             self.view_idx=(self.view_idx+1)%len(self.views)
#             elev,azim=self.views[self.view_idx]
#             self.ax.view_init(elev=elev,azim=azim)
#         elif key==keyboard.Key.esc:
#             self.stop();plt.close(self.fig)

#     def _update_loop(self):
#         while self.running:
#             poses, buttons = self.reader.get_device_state()
#             self.buttons = buttons or {}
#             # toggle active on Haply A rising edge
#             a_pressed = self.buttons.get('a',False)
#             if a_pressed and not self.prev_button_a:
#                 self.active = not self.active
#             self.prev_button_a = a_pressed
#             # update pose/orient
#             if poses:
#                 self.quat = poses['orientation']
#                 pos = poses['position']
#                 rot,_ = self.interpret_rotation(self.quat,self.mode)
#                 T=np.eye(4);T[:3,3]=pos;T[:3,:3]=rot
#                 if self.calibrated:
#                     T=self.haply_to_global@T
#                 self.pose=T[:3,3]
#                 self.orient=T[:3,:3]
#             time.sleep(0.05)

#     def _init_plot(self):
#         self.fig=plt.figure()
#         self.ax=self.fig.add_subplot(111,projection='3d')
#         self.ax.set(xlim=(-0.2,0.2),ylim=(-0.2,0.2),zlim=(0,0.2),
#                     xlabel='X',ylabel='Y',zlabel='Z')
#         elev,azim=self.views[self.view_idx]
#         self.ax.view_init(elev=elev,azim=azim)
#         # ground plane
#         xx,yy=np.meshgrid(np.linspace(-0.2,0.2,10),np.linspace(-0.2,0.2,10))
#         self.ax.plot_surface(xx,yy,np.zeros_like(xx),color='gray',alpha=0.5)
#         # cylinder
#         r,h=0.01,0.05
#         th=np.linspace(0,2*np.pi,20)
#         zc=np.array([0,-h])
#         tg,zcg=np.meshgrid(th,zc)
#         self.xc=r*np.cos(tg);self.yc=r*np.sin(tg);self.zg=zcg
#         # init elements
#         o=self.pose
#         self.pt,=self.ax.plot([o[0]],[o[1]],[o[2]],'ko')
#         self.lines=[]
#         for i,style in enumerate(['r-','g-','b-']):
#             end=o+self.orient[:,i]*0.05
#             ln,=self.ax.plot([o[0],end[0]],[o[1],end[1]],[o[2],end[2]],style)
#             self.lines.append(ln)
#         pts=np.vstack([self.xc.flatten(),self.yc.flatten(),self.zg.flatten()])
#         pw=(self.orient@pts).T+o
#         X=pw[:,0].reshape(self.zg.shape);Y=pw[:,1].reshape(self.zg.shape);Z=pw[:,2].reshape(self.zg.shape)
#         self.cyl=self.ax.plot_surface(X,Y,Z,alpha=0.7)
#         self.ani=FuncAnimation(self.fig,self._animate,interval=50)
#         self.fig.canvas.draw()

#     def _animate(self,_):
#         if not self.display_plot:
#             return
#         o=self.pose
#         ends=[o+self.orient[:,i]*0.05 for i in range(3)]
#         self.pt.set_data([o[0]],[o[1]]);self.pt.set_3d_properties([o[2]])
#         for ln,end in zip(self.lines,ends):
#             ln.set_data([o[0],end[0]],[o[1],end[1]]);ln.set_3d_properties([o[2],end[2]])
#         if self.cyl: self.cyl.remove()
#         pts=np.vstack([self.xc.flatten(),self.yc.flatten(),self.zg.flatten()])
#         pw=(self.orient@pts).T+o
#         X=pw[:,0].reshape(self.zg.shape);Y=pw[:,1].reshape(self.zg.shape);Z=pw[:,2].reshape(self.zg.shape)
#         self.cyl=self.ax.plot_surface(X,Y,Z,alpha=0.7)
#         self.fig.canvas.draw_idle()
#         return [self.pt]+self.lines+[self.cyl]

#     def get_action(self,mode='euler',seq='zyx',degrees=False):
#         dp=np.zeros(3);da=np.zeros(3);dq=np.zeros(4)
#         if self.active:
#             dp=self.pose-self.prev_pose
#             if mode=='quat':
#                 cq=self.quat.copy();dq=(R.from_quat(cq)*R.from_quat(self.prev_quat).inv()).as_quat()
#             else:
#                 ce=R.from_matrix(self.orient).as_euler(seq,degrees)
#                 da = (ce - self.prev_euler) * self.scalar_rot
#                 if self.negate_angles: da=-da
#         # scale & swap XY
#         dp=dp*self.scalar_xyz;dp=np.array([dp[1],-dp[0],dp[2]])
#         # update prevs
#         self.prev_pose=self.pose.copy();self.prev_quat=self.quat.copy()
#         self.prev_euler=R.from_matrix(self.orient).as_euler(seq,degrees)
#         # gripper
#         if self.buttons.get('b',False) and self.gripper_counter==0:
#             self.gripper_target=0.2 if self.gripper_state>0.9 else 1.0
#             self.gripper_counter=self.gripper_steps
#         if self.gripper_counter>0:
#             step=(self.gripper_target-self.gripper_state)/self.gripper_counter
#             self.gripper_state+=step;self.gripper_counter-=1
#         return np.concatenate([dp, dq, [self.gripper_state]]) if mode=='quat' else np.concatenate([dp,da,[self.gripper_state]])

#     def stop(self):
#         self.running=False

# if __name__=='__main__':
#     from robot_env_corrected.RTDE_RW_test_collect import RobotAction

#     viz=HapticVisualizer()
#     env=RobotAction(control_hz=15)
#     env.reset()
#     try:
#         while viz.running:
#             act=viz.get_action(mode='euler',seq='yxz')
#             env.send_action(act)
#             plt.pause(0.001)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         viz.stop()


import time
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pynput import keyboard
from scipy.spatial.transform import Rotation as R

import sys
from haply_data_collect_july23.teleop_controls.haply_reader.reader import HapticReader

class HapticVisualizer:
    def __init__(self, display_plot=True, gripper_open=1.0, gripper_closed=0.0, gripper_steps=5, haptic_feedback=False, force_reader=None):
        # State
        self.running = True
        self.calibrated = False
        self.haply_to_global = np.eye(4)
        self.pose = np.zeros(3)
        self.orient = np.eye(3)
        self.quat = np.array([0,0,0,1])

        # Deltas
        self.prev_pose = self.pose.copy()
        self.prev_euler = np.zeros(3)
        self.prev_quat = self.quat.copy()

        # Gripper
        self.gripper_open = float(gripper_open)
        self.gripper_closed = float(gripper_closed)
        self.gripper_steps = int(gripper_steps)
        self.gripper_state = self.gripper_open
        self.gripper_target = self.gripper_open
        self.gripper_counter = 0

        # Toggle active
        self.active = False
        self.prev_button_a = False
        self.prev_button_b = False

        # Config
        self.scalar_xyz = np.array([1.5,1.5,1.5])
        self.scalar_rot = np.array([.5,.5,-.5])
        self.negate_angles = True

        # Buttons
        self.buttons = {}

        # Views
        self.mode = 0
        self.view_idx = 0
        self.views = [(0,270),(30,45),(90,0),(0,0),(0,90)]

        # Keyboard listener
        listener = keyboard.Listener(on_press=self.on_press)
        listener.daemon = True
        listener.start()

        # Haply reader + update thread
        self.reader = HapticReader(haptic_feedback=haptic_feedback, force_reader=force_reader)
        threading.Thread(target=self._update_loop, daemon=True).start()

        # Plot
        self.display_plot = bool(display_plot)
        self.fig = None
        if self.display_plot:
            plt.ion()
            self._init_plot()

    def interpret_rotation(self, quat, mode):
        if mode == 0:
            Rm = R.from_quat(quat).as_matrix()[:, [2, 0, 1]]
            return Rm, R.from_matrix(Rm).as_euler('zyx', True)
        return np.eye(3), np.zeros(3)

    def on_press(self, key):
        c = getattr(key, 'char', None)
        if c == 'a':
            if not self.calibrated:
                T = np.eye(4); T[:3, 3] = self.pose
                self.haply_to_global = np.linalg.inv(T)
                self.calibrated = True
            else:
                self.mode = (self.mode + 1) % len(self.views)
        elif c == 'z' and self.display_plot and hasattr(self, 'ax'):
            self.view_idx = (self.view_idx + 1) % len(self.views)
            elev, azim = self.views[self.view_idx]
            self.ax.view_init(elev=elev, azim=azim)
        elif key == keyboard.Key.esc:
            self.stop()
            if self.display_plot and self.fig is not None:
                plt.close(self.fig)

    def _update_loop(self):
        while self.running:
            poses, buttons = self.reader.get_device_state()
            self.buttons = buttons or {}

            # Active toggle (A) — rising edge
            a_pressed = bool(self.buttons.get('a', False))
            if a_pressed and not self.prev_button_a:
                self.active = not self.active
            self.prev_button_a = a_pressed

            # Gripper toggle (B) — rising edge
            b_pressed = bool(self.buttons.get('b', False))
            if b_pressed and not self.prev_button_b:
                # flip target between open and closed
                self.gripper_target = self.gripper_closed if self.gripper_target == self.gripper_open else self.gripper_open
                self.gripper_counter = self.gripper_steps
            self.prev_button_b = b_pressed

            # Pose/orient update
            if poses:
                self.quat = poses['orientation']
                pos = poses['position']
                rot, _ = self.interpret_rotation(self.quat, self.mode)
                T = np.eye(4); T[:3, 3] = pos; T[:3, :3] = rot
                if self.calibrated:
                    T = self.haply_to_global @ T
                self.pose = T[:3, 3]
                self.orient = T[:3, :3]

            # # Gripper smoothing
            # if self.gripper_counter > 0:
            #     step = (self.gripper_target - self.gripper_state) / self.gripper_counter
            #     self.gripper_state += step
            #     self.gripper_counter -= 1

            time.sleep(0.05)

    def _init_plot(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set(xlim=(-0.2, 0.2), ylim=(-0.2, 0.2), zlim=(0, 0.2),
                    xlabel='X', ylabel='Y', zlabel='Z')
        elev, azim = self.views[self.view_idx]
        self.ax.view_init(elev=elev, azim=azim)

        xx, yy = np.meshgrid(np.linspace(-0.2, 0.2, 10), np.linspace(-0.2, 0.2, 10))
        self.ax.plot_surface(xx, yy, np.zeros_like(xx), color='gray', alpha=0.5)

        r, h = 0.01, 0.05
        th = np.linspace(0, 2*np.pi, 20)
        zc = np.array([0, -h])
        tg, zcg = np.meshgrid(th, zc)
        self.xc = r*np.cos(tg); self.yc = r*np.sin(tg); self.zg = zcg

        o = self.pose
        self.pt, = self.ax.plot([o[0]], [o[1]], [o[2]], 'ko')
        self.lines = []
        for i, style in enumerate(['r-', 'g-', 'b-']):
            end = o + self.orient[:, i]*0.05
            ln, = self.ax.plot([o[0], end[0]], [o[1], end[1]], [o[2], end[2]], style)
            self.lines.append(ln)

        pts = np.vstack([self.xc.flatten(), self.yc.flatten(), self.zg.flatten()])
        pw = (self.orient @ pts).T + o
        X = pw[:, 0].reshape(self.zg.shape); Y = pw[:, 1].reshape(self.zg.shape); Z = pw[:, 2].reshape(self.zg.shape)
        self.cyl = self.ax.plot_surface(X, Y, Z, alpha=0.7)
        self.ani = FuncAnimation(self.fig, self._animate, interval=50)
        self.fig.canvas.draw()

    def _animate(self, _):
        if not self.display_plot:
            return
        o = self.pose
        ends = [o + self.orient[:, i]*0.05 for i in range(3)]
        self.pt.set_data([o[0]], [o[1]]); self.pt.set_3d_properties([o[2]])
        for ln, end in zip(self.lines, ends):
            ln.set_data([o[0], end[0]], [o[1], end[1]]); ln.set_3d_properties([o[2], end[2]])
        if self.cyl: self.cyl.remove()
        pts = np.vstack([self.xc.flatten(), self.yc.flatten(), self.zg.flatten()])
        pw = (self.orient @ pts).T + o
        X = pw[:, 0].reshape(self.zg.shape); Y = pw[:, 1].reshape(self.zg.shape); Z = pw[:, 2].reshape(self.zg.shape)
        self.cyl = self.ax.plot_surface(X, Y, Z, alpha=0.7)
        self.fig.canvas.draw_idle()
        return [self.pt] + self.lines + [self.cyl]

    def get_action(self, mode='euler', seq='zyx', degrees=False):
        dp = np.zeros(3); da = np.zeros(3); dq = np.zeros(4)
        if self.active:
            dp = self.pose - self.prev_pose
            if mode == 'quat':
                cq = self.quat.copy()
                dq = (R.from_quat(cq) * R.from_quat(self.prev_quat).inv()).as_quat()
            else:
                ce = R.from_matrix(self.orient).as_euler(seq, degrees)
                da = (ce - self.prev_euler) * self.scalar_rot
                if self.negate_angles:
                    da = -da

        dp = dp * self.scalar_xyz
        dp = np.array([dp[1], -dp[0], dp[2]])  # swap XY

        self.prev_pose = self.pose.copy()
        self.prev_quat = self.quat.copy()
        self.prev_euler = R.from_matrix(self.orient).as_euler(seq, degrees)

        # Use current smoothed gripper_state
        g = float(self.gripper_target)

        if mode == 'quat':
            return np.concatenate([dp, dq, [g]])
        else:
            # print(np.concatenate([dp, da, [g]]))
            return np.concatenate([dp, da, [g]])
        

    def stop(self):
        self.running = False
