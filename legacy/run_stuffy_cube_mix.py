# from gui import RobotArmGUI 
# from controller import RobotArmController
# import numpy as np
# import re, sys, time, random

# # === your existing helpers (unchanged) =======================================
# def generate_noisy_bezier_points(A, B, n, num_ctrl=3, radius=0.03, noise_suppresion=100):
#     A, B = np.array(A, dtype=float), np.array(B, dtype=float)
#     dist = np.linalg.norm(B - A)
#     arc_h = dist / 3.0
#     up = np.array([0, 0, 1], dtype=float)

#     t_ctrl = np.linspace(0, 1, num_ctrl + 2)[1:-1]
#     pts = [A]
#     for t in t_ctrl:
#         base = (1 - t) * A + t * B + arc_h * 4 * t * (1 - t) * up
#         d = np.random.randn(3); d /= np.linalg.norm(d)
#         r = radius * np.random.rand() ** (1/3)
#         pts.append(base + d * r)
#     pts.append(B)

#     segs = len(pts) - 1
#     per_seg = int(np.ceil(n / segs))
#     mild_dev = dist / noise_suppresion
#     traj = []
#     for i in range(segs):
#         P0, P2 = pts[i], pts[i + 1]
#         d = P2 - P0
#         o = np.random.randn(3); o -= o.dot(d) / np.dot(d, d) * d; o /= np.linalg.norm(o)
#         P1 = (P0 + P2) / 2 + mild_dev * o
#         t_vals = np.linspace(0, 1, per_seg)
#         for t in t_vals:
#             p1 = (1 - t) * P0 + t * P1
#             p2 = (1 - t) * P1 + t * P2
#             traj.append((1 - t) * p1 + t * p2)

#     traj = np.array(traj)
#     if traj.shape[0] > n: traj = traj[:n]
#     traj[0] = A; traj[-1] = B
#     return traj

# def points_to_deltas(points):
#     points = np.array(points)
#     return points[1:] - points[:-1]

# def moving_average(data, window_size=5):
#     data = np.array(data, dtype=float)
#     N, D = data.shape
#     if N < window_size: return data
#     left = (window_size - 1) // 2
#     right = window_size - 1 - left
#     padded = np.pad(data, pad_width=((left, right), (0,0)), mode='edge')
#     kernel = np.ones(window_size) / window_size
#     smoothed = np.stack([np.convolve(padded[:,i], kernel, mode='valid') for i in range(D)], axis=1)
#     return smoothed

# PAUSE_RE = re.compile(r'^pause_(\d+)(?:_gripper_([0-9]*\.?[0-9]+)_to_([0-9]*\.?[0-9]+))?$')
# LINEAR_MOVE_RE = re.compile(r'^([xyz])_(\d+)_times_(-?[0-9]*\.?[0-9]+)$')

# def generate_full_trajectory(waypoints,n_per_segment=100,smooth_window=10,use_orientations=False):
#     pts, oris, grip = [], [], []
#     current_gripper = 0.0
#     for wp in waypoints:
#         if isinstance(wp, (list, tuple)) and len(wp) == 1 and isinstance(wp[0], str):
#             m_linear = LINEAR_MOVE_RE.match(wp[0])
#             if m_linear:
#                 axis, count_str, step_str = m_linear.groups()
#                 count, step = int(m_linear.group(2)), float(m_linear.group(3))
#                 last_p = pts[-1]; idx = {'x':0,'y':1,'z':2}[axis.lower()]
#                 step_per_move = step / count
#                 for i in range(1, count+1):
#                     new_p = last_p.copy(); new_p[idx] += step_per_move * i
#                     pts.append(new_p); oris.append(new_p if use_orientations else np.zeros(3)); grip.append(current_gripper)
#                 continue
#             m = PAUSE_RE.match(wp[0])
#             if m:
#                 dur = int(m.group(1)); g0 = g1 = current_gripper
#                 if m.group(2): g0, g1 = float(m.group(2)), float(m.group(3))
#                 last_p = pts[-1]; last_o = oris[-1]
#                 pts.extend([last_p.copy() for _ in range(dur)])
#                 oris.extend([last_o.copy() for _ in range(dur)])
#                 move_steps = int(dur * 0.6); hold_steps = dur - move_steps
#                 gs = np.linspace(g0, g1, max(move_steps,1)).tolist()
#                 if move_steps == 0: gs = [g1] * 1
#                 gs += [g1] * hold_steps
#                 grip.extend(gs[:dur])
#                 current_gripper = g1
#                 continue
#         P = np.array(wp, dtype=float)
#         if not pts:
#             pts.append(P); oris.append(P if use_orientations else np.zeros(3)); grip.append(current_gripper)
#         else:
#             seg = generate_noisy_bezier_points(
#                 pts[-1], P, n=n_per_segment,
#                 num_ctrl=np.random.randint(1,4),
#                 radius=np.random.uniform(0.03,0.07),
#                 noise_suppresion=int(np.exp(np.random.uniform(np.log(3), np.log(50))))
#             )
#             for p in seg[1:]:
#                 pts.append(p); oris.append(p if use_orientations else np.zeros(3)); grip.append(current_gripper)
#     pts = moving_average(np.array(pts), smooth_window)
#     oris = moving_average(np.array(oris), smooth_window)
#     return pts, oris, np.array(grip)

# # === new helpers for two-object logic ========================================
# def z_down(count, dz): return [f'z_{count}_times_{-abs(dz)}']
# def z_up(count, dz):   return [f'z_{count}_times_{abs(dz)}']

# def sample_stuffy_and_cube_next(workspace, stuffy_size_xy):
#     x_min, x_max = workspace['x_min'], workspace['x_max']
#     y_min, y_max = workspace['y_min'], workspace['y_max']
#     z = workspace['z']
#     sx_half, sy_half = stuffy_size_xy[0]/2, stuffy_size_xy[1]/2

#     sx = random.uniform(x_min + sx_half, x_max - sx_half)
#     sy = random.uniform(y_min + sy_half, y_max - sy_half)

#     def inside_stuffy_foot(cx, cy):
#         return (sx - sx_half <= cx <= sx + sx_half) and (sy - sy_half <= cy <= sy + sy_half)

#     while True:
#         cx = random.uniform(x_min, x_max)
#         cy = random.uniform(y_min, y_max)
#         if not inside_stuffy_foot(cx, cy): break

#     return [sx, sy, z], [cx, cy, z]

# def pick_and_place_segment(pick_xy, drop_xy, descend_abs):
#     return [
#         pick_xy,
#         z_down(25, descend_abs),
#         ['pause_20_gripper_0_to_0.60'],
#         z_up(10, 0.08),
#         drop_xy,
#         z_down(25, descend_abs),
#         ['pause_20_gripper_0.60_to_0'],
#         z_up(10, 0.08),
#     ]

# def generate_waypoints_dual(i, cube_pts, stuffy_pts, cube_box, stuffy_box, cube_desc=0.065, stuffy_desc=None):
#     if stuffy_desc is None: stuffy_desc = cube_desc/2.0
#     home = [0,0,0]
#     first = random.choice(['cube','stuffy'])

#     def seq_to_box(obj):
#         if obj == 'cube':
#             return pick_and_place_segment(cube_pts[i], cube_box, cube_desc)
#         else:
#             return pick_and_place_segment(stuffy_pts[i], stuffy_box, stuffy_desc)

#     def seq_from_box(obj):
#         if obj == 'cube':
#             return pick_and_place_segment(cube_box, cube_pts[i+1], cube_desc)
#         else:
#             return pick_and_place_segment(stuffy_box, stuffy_pts[i+1], stuffy_desc)

#     order = [first, 'stuffy' if first=='cube' else 'cube']

#     wp = [home]
#     for obj in order:
#         wp += seq_to_box(obj)
#         wp += [home, ['pause_10']]
#     for obj in order:
#         wp += seq_from_box(obj)
#         if obj != order[-1]:
#             wp += [home, ['pause_10']]
#     wp += [home, ['pause_10']]
#     return wp

# # === main/run with two objects ===============================================
# def run_episode(i, total_episodes, cube_pts, stuffy_pts, cube_box, stuffy_box):
#     waypoints = generate_waypoints_dual(i, cube_pts, stuffy_pts, cube_box, stuffy_box,
#                                         cube_desc=0.085, stuffy_desc=0.025)
#     points, orientations, gripper = generate_full_trajectory(
#         waypoints, n_per_segment=29, smooth_window=5, use_orientations=True
#     )
#     deltas = points_to_deltas(points)
#     orientations = [[0,0,0] for _ in points]
#     hz = 5
#     ctrl = RobotArmController(control_hz=hz, time_step=1/hz, initial_pos=points[0], initial_ori=orientations[0])

#     extra_sets = [[
#         [-0.795, 0.457, 0.179], [-0.795, 1.14, 0.179], [0.3, 1.14, 0.179], [0.3, 0.457, 0.179],
#         [-0.795, 0.457, 0.95],  [-0.795, 1.14, 0.95],  [0.3, 1.14, 0.95],  [0.3, 0.457, 0.95],
#     ]]
#     RobotArmGUI(ctrl, deltas, orientations, update_hz=hz, extra_point_sets=extra_sets, gripper=gripper)

# def main():
#     total_episodes = 100
#     episode_times = []

#     workspace = dict(x_min=-0.312, x_max=0.059, y_min=-0.176, y_max=0.38, z=-0.185+0.07)

#     # cube setup (same idea as your previous)
#     cube_box = [0.21, 0.2455, workspace['z'] -0.01]
#     cube_init = [-0.241, -0.0255, workspace['z']]

#     # stuffy setup (USER: set these)
#     STUFFY_INIT = [-0.241, 0.20, workspace['z']]
#     STUFFY_BOX  = [0.21, -0.0255,  workspace['z'] - 0.01]

#     # stuffy footprint = half of workspace range in x and y
#     x_range = workspace['x_max'] - workspace['x_min']
#     y_range = workspace['y_max'] - workspace['y_min']
#     stuffy_size_xy = (0.5 * x_range, 0.5 * y_range)

#     cube_pts   = [cube_init]
#     stuffy_pts = [STUFFY_INIT]

#     for _ in range(total_episodes - 1):
#         s_next, c_next = sample_stuffy_and_cube_next(workspace, stuffy_size_xy)
#         stuffy_pts.append(s_next)
#         cube_pts.append(c_next)

#     for i in range(total_episodes - 1):
#         print(f"\033cEpisode #{i+1}")
#         t0 = time.perf_counter()
#         run_episode(i, total_episodes, cube_pts, stuffy_pts, cube_box, STUFFY_BOX)
#         dt = time.perf_counter() - t0
#         episode_times.append(dt)
#         if i > 0:
#             avg = sum(episode_times)/len(episode_times)
#             left = (total_episodes - 1) - (i + 1)
#             print(f"Estimated time remaining: {avg*left:.2f} seconds")

#     print("All episodes completed.")
#     sys.exit(0)

# if __name__ == "__main__":
#     main()
from gui import RobotArmGUI
from controller import RobotArmController
import numpy as np
import re, sys, time, random

# === grips ===
CUBE_GRIP_CLOSE   = 0.60
STUFFY_GRIP_CLOSE = 1.00

# === helpers ===
def generate_noisy_bezier_points(A, B, n, num_ctrl=3, radius=0.03, noise_suppresion=100):
    A, B = np.array(A, dtype=float), np.array(B, dtype=float)
    dist = np.linalg.norm(B - A)
    arc_h = dist / 3.0
    up = np.array([0, 0, 1], dtype=float)

    t_ctrl = np.linspace(0, 1, num_ctrl + 2)[1:-1]
    pts = [A]
    for t in t_ctrl:
        base = (1 - t) * A + t * B + arc_h * 4 * t * (1 - t) * up
        d = np.random.randn(3); d /= np.linalg.norm(d)
        r = radius * np.random.rand() ** (1/3)
        pts.append(base + d * r)
    pts.append(B)

    segs = len(pts) - 1
    per_seg = int(np.ceil(n / segs))
    mild_dev = dist / noise_suppresion
    traj = []
    for i in range(segs):
        P0, P2 = pts[i], pts[i + 1]
        d = P2 - P0
        o = np.random.randn(3); o -= o.dot(d) / np.dot(d, d) * d; o /= np.linalg.norm(o)
        P1 = (P0 + P2) / 2 + mild_dev * o
        t_vals = np.linspace(0, 1, per_seg)
        for t in t_vals:
            p1 = (1 - t) * P0 + t * P1
            p2 = (1 - t) * P1 + t * P2
            traj.append((1 - t) * p1 + t * p2)

    traj = np.array(traj)
    if traj.shape[0] > n: traj = traj[:n]
    traj[0] = A; traj[-1] = B
    return traj

def points_to_deltas(points):
    points = np.array(points)
    return points[1:] - points[:-1]

def moving_average(data, window_size=5):
    data = np.array(data, dtype=float)
    N, D = data.shape
    if N < window_size: return data
    left = (window_size - 1) // 2
    right = window_size - 1 - left
    padded = np.pad(data, pad_width=((left, right), (0,0)), mode='edge')
    kernel = np.ones(window_size) / window_size
    smoothed = np.stack([np.convolve(padded[:,i], kernel, mode='valid') for i in range(D)], axis=1)
    return smoothed

PAUSE_RE = re.compile(r'^pause_(\d+)(?:_gripper_([0-9]*\.?[0-9]+)_to_([0-9]*\.?[0-9]+))?$')
LINEAR_MOVE_RE = re.compile(r'^([xyz])_(\d+)_times_(-?[0-9]*\.?[0-9]+)$')

def generate_full_trajectory(waypoints, n_per_segment=100, smooth_window=10, use_orientations=False):
    pts, oris, grip = [], [], []
    current_gripper = 0.0
    for wp in waypoints:
        if isinstance(wp, (list, tuple)) and len(wp) == 1 and isinstance(wp[0], str):
            m_linear = LINEAR_MOVE_RE.match(wp[0])
            if m_linear:
                axis, count_str, step_str = m_linear.groups()
                count, step = int(count_str), float(step_str)
                last_p = pts[-1]; idx = {'x':0,'y':1,'z':2}[axis.lower()]
                step_per_move = step / count
                for i in range(1, count+1):
                    new_p = last_p.copy(); new_p[idx] += step_per_move * i
                    pts.append(new_p); oris.append(new_p if use_orientations else np.zeros(3)); grip.append(current_gripper)
                continue
            m = PAUSE_RE.match(wp[0])
            if m:
                dur = int(m.group(1)); g0 = g1 = current_gripper
                if m.group(2): g0, g1 = float(m.group(2)), float(m.group(3))
                last_p = pts[-1]; last_o = oris[-1]
                pts.extend([last_p.copy() for _ in range(dur)])
                oris.extend([last_o.copy() for _ in range(dur)])
                move_steps = int(dur * 0.6); hold_steps = dur - move_steps
                gs = np.linspace(g0, g1, max(move_steps,1)).tolist()
                if move_steps == 0: gs = [g1]
                gs += [g1] * hold_steps
                grip.extend(gs[:dur])
                current_gripper = g1
                continue
        P = np.array(wp, dtype=float)
        if not pts:
            pts.append(P); oris.append(P if use_orientations else np.zeros(3)); grip.append(current_gripper)
        else:
            seg = generate_noisy_bezier_points(
                pts[-1], P, n=n_per_segment,
                num_ctrl=np.random.randint(1,4),
                radius=np.random.uniform(0.03,0.07),
                noise_suppresion=int(np.exp(np.random.uniform(np.log(3), np.log(50))))
            )
            for p in seg[1:]:
                pts.append(p); oris.append(p if use_orientations else np.zeros(3)); grip.append(current_gripper)
    pts = moving_average(np.array(pts), smooth_window)
    oris = moving_average(np.array(oris), smooth_window)
    return pts, oris, np.array(grip)

def z_down(count, dz): return [f'z_{count}_times_{-abs(dz)}']
def z_up(count, dz):   return [f'z_{count}_times_{abs(dz)}']

def sample_stuffy_and_cube_next(workspace, stuffy_size_xy):
    x_min, x_max = workspace['x_min'], workspace['x_max']
    y_min, y_max = workspace['y_min'], workspace['y_max']
    z = workspace['z']
    sx_half, sy_half = stuffy_size_xy[0]/2, stuffy_size_xy[1]/2

    sx = random.uniform(x_min + sx_half, x_max - sx_half)
    sy = random.uniform(y_min + sy_half, y_max - sy_half)

    def inside_stuffy_foot(cx, cy):
        return (sx - sx_half <= cx <= sx + sx_half) and (sy - sy_half <= cy <= sy + sy_half)

    while True:
        cx = random.uniform(x_min, x_max)
        cy = random.uniform(y_min, y_max)
        if not inside_stuffy_foot(cx, cy): break

    return [sx, sy, z], [cx, cy, z]

def pick_and_place_segment(pick_xy, drop_xy, descend_abs, grip_close):
    return [
        pick_xy,
        z_down(25, descend_abs),
        [f'pause_20_gripper_0_to_{grip_close}'],
        z_up(10, 0.08),
        drop_xy,
        z_down(25, descend_abs),
        [f'pause_20_gripper_{grip_close}_to_0'],
        z_up(10, 0.08),
    ]

def generate_waypoints_dual(i, cube_pts, stuffy_pts, cube_box, stuffy_box,
                            cube_desc=0.085, stuffy_desc=None,
                            cube_grip=CUBE_GRIP_CLOSE, stuffy_grip=STUFFY_GRIP_CLOSE):
    if stuffy_desc is None: stuffy_desc = cube_desc/2.0
    home = [0,0,0]
    first = random.choice(['cube','stuffy'])

    def seq_to_box(obj):
        if obj == 'cube':
            return pick_and_place_segment(cube_pts[i], cube_box, cube_desc, cube_grip)
        else:
            return pick_and_place_segment(stuffy_pts[i], stuffy_box, stuffy_desc, stuffy_grip)

    def seq_from_box(obj):
        if obj == 'cube':
            return pick_and_place_segment(cube_box, cube_pts[i+1], cube_desc, cube_grip)
        else:
            return pick_and_place_segment(stuffy_box, stuffy_pts[i+1], stuffy_desc, stuffy_grip)

    order = [first, 'stuffy' if first=='cube' else 'cube']

    wp = [home]
    for obj in order:
        wp += seq_to_box(obj)
        wp += [home, ['pause_10']]
    for obj in order:
        wp += seq_from_box(obj)
        if obj != order[-1]:
            wp += [home, ['pause_10']]
    wp += [home, ['pause_10']]
    return wp

# === episode/run ===
def run_episode(i, total_episodes, cube_pts, stuffy_pts, cube_box, stuffy_box):
    waypoints = generate_waypoints_dual(
        i, cube_pts, stuffy_pts, cube_box, stuffy_box,
        cube_desc=0.085, stuffy_desc=0.025,
        cube_grip=CUBE_GRIP_CLOSE, stuffy_grip=STUFFY_GRIP_CLOSE
    )
    points, orientations, gripper = generate_full_trajectory(
        waypoints, n_per_segment=29, smooth_window=5, use_orientations=True
    )
    deltas = points_to_deltas(points)
    orientations = [[0,0,0] for _ in points]
    hz = 5
    ctrl = RobotArmController(control_hz=hz, time_step=1/hz, initial_pos=points[0], initial_ori=orientations[0])

    extra_sets = [[
        [-0.795, 0.457, 0.179], [-0.795, 1.14, 0.179], [0.3, 1.14, 0.179], [0.3, 0.457, 0.179],
        [-0.795, 0.457, 0.95],  [-0.795, 1.14, 0.95],  [0.3, 1.14, 0.95],  [0.3, 0.457, 0.95],
    ]]
    RobotArmGUI(ctrl, deltas, orientations, update_hz=hz, extra_point_sets=extra_sets, gripper=gripper)

def main():
    total_episodes = 100
    episode_times = []

    workspace = dict(x_min=-0.312, x_max=0.059, y_min=-0.176, y_max=0.38, z=-0.185+0.07)

    cube_box  = [0.21,  0.2455, workspace['z'] - 0.01]
    cube_init = [-0.241, -0.0255, workspace['z']]

    STUFFY_INIT = [-0.241, 0.20,  workspace['z']]
    STUFFY_BOX  = [ 0.21, -0.0255, workspace['z'] - 0.01]

    x_range = workspace['x_max'] - workspace['x_min']
    y_range = workspace['y_max'] - workspace['y_min']
    stuffy_size_xy = (0.5 * x_range, 0.5 * y_range)

    cube_pts   = [cube_init]
    stuffy_pts = [STUFFY_INIT]

    for _ in range(total_episodes - 1):
        s_next, c_next = sample_stuffy_and_cube_next(workspace, stuffy_size_xy)
        stuffy_pts.append(s_next)
        cube_pts.append(c_next)

    for i in range(total_episodes - 1):
        print(f"\033cEpisode #{i+1}")
        t0 = time.perf_counter()
        run_episode(i, total_episodes, cube_pts, stuffy_pts, cube_box, STUFFY_BOX)
        dt = time.perf_counter() - t0
        episode_times.append(dt)
        if i > 0:
            avg = sum(episode_times)/len(episode_times)
            left = (total_episodes - 1) - (i + 1)
            print(f"Estimated time remaining: {avg*left:.2f} seconds")

    print("All episodes completed.")
    sys.exit(0)

if __name__ == "__main__":
    main()
