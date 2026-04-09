from gui import RobotArmGUI
from controller import RobotArmController
import numpy as np
import re
import sys
import time
import random

def generate_noisy_bezier_points(A, B, n, num_ctrl=3, radius=0.03, noise_suppresion=100):
    A, B = np.array(A, dtype=float), np.array(B, dtype=float)
    dist = np.linalg.norm(B - A)
    arc_h = dist / 3.0
    up = np.array([0, 0, 1], dtype=float)

    # build control-point list
    t_ctrl = np.linspace(0, 1, num_ctrl + 2)[1:-1]
    pts = [A]
    for t in t_ctrl:
        base = (1 - t) * A + t * B + arc_h * 4 * t * (1 - t) * up
        d = np.random.randn(3)
        d /= np.linalg.norm(d)
        r = radius * np.random.rand() ** (1/3)
        pts.append(base + d * r)
    pts.append(B)

    # sample each quadratic-bezier segment
    segs = len(pts) - 1
    per_seg = int(np.ceil(n / segs))
    mild_dev = dist / noise_suppresion
    traj = []

    for i in range(segs):
        P0, P2 = pts[i], pts[i + 1]
        d = P2 - P0
        o = np.random.randn(3)
        o -= o.dot(d) / np.dot(d, d) * d
        o /= np.linalg.norm(o)
        P1 = (P0 + P2) / 2 + mild_dev * o

        t_vals = np.linspace(0, 1, per_seg)
        for t in t_vals:
            p1 = (1 - t) * P0 + t * P1
            p2 = (1 - t) * P1 + t * P2
            traj.append((1 - t) * p1 + t * p2)

    traj = np.array(traj)

    # **force exact length and endpoints**
    if traj.shape[0] > n:
        traj = traj[:n]
    traj[0] = A
    traj[-1] = B

    return traj

def points_to_deltas(points):
    """
    Converts an array of points into relative delta commands.
    
    Parameters:
        points (np.ndarray): Array of shape (n, 3) representing 3D points.
        
    Returns:
        deltas (np.ndarray): Array of shape (n-1, 3) representing deltas between consecutive points.
    """
    points = np.array(points)
    deltas = points[1:] - points[:-1]
    return deltas

def moving_average(data, window_size=5):
    data = np.array(data, dtype=float)
    N, D = data.shape
    if N < window_size:
        return data

    # how many to pad on each side
    left = (window_size - 1) // 2
    right = window_size - 1 - left

    # pad by repeating the first/last row
    padded = np.pad(
        data,
        pad_width=((left, right), (0,0)),
        mode='edge'
    )

    kernel = np.ones(window_size) / window_size

    # convolve along each column, 'valid' gives exactly N outputs
    smoothed = np.stack([
        np.convolve(padded[:,i], kernel, mode='valid')
        for i in range(D)
    ], axis=1)

    return smoothed

# Regular expressions for parsing commands in waypoints
PAUSE_RE = re.compile(r'^pause_(\d+)(?:_gripper_([0-9]*\.?[0-9]+)_to_([0-9]*\.?[0-9]+))?$')
LINEAR_MOVE_RE = re.compile(r'^([xyz])_(\d+)_times_(-?[0-9]*\.?[0-9]+)$')

def generate_full_trajectory(waypoints,n_per_segment=100,smooth_window=10,use_orientations=False):
    pts = []
    oris = []
    grip = []
    current_gripper = 0.0

    for wp in waypoints:
        # --- check linear move command ---
        if isinstance(wp, (list, tuple)) and len(wp) == 1 and isinstance(wp[0], str):
            m_linear = LINEAR_MOVE_RE.match(wp[0])
            if m_linear:
                axis, count_str, step_str = m_linear.groups()
                count = int(count_str)
                step = float(step_str)

                # Get last point as start
                last_p = pts[-1]
                idx = {'x': 0, 'y': 1, 'z': 2}[axis.lower()]
                step_per_move = step / count
                for i in range(1, count + 1):
                    new_p = last_p.copy()
                    new_p[idx] += step_per_move * i
                    pts.append(new_p)
                    oris.append(new_p if use_orientations else np.zeros(3))
                    grip.append(current_gripper)

                continue

            # --- check pause or pause+gripper command ---
            m = PAUSE_RE.match(wp[0])
            if m:
                dur = int(m.group(1))
                # default: hold gripper constant
                g0 = g1 = current_gripper
                # if user specified _gripper_x_to_y
                if m.group(2):
                    g0, g1 = float(m.group(2)), float(m.group(3))
                # extend points & ori by repeating last
                last_p = pts[-1]
                last_o = oris[-1]
                pts.extend([last_p.copy() for _ in range(dur)])
                oris.extend([last_o.copy() for _ in range(dur)])
                # linearly interpolate gripper over dur
                move_steps = int(dur * 0.6)
                hold_steps = dur - move_steps

                # Linearly interpolate over the first 60%
                gs = np.linspace(g0, g1, move_steps).tolist()

                # Hold at g1 for the remaining 40%
                gs += [g1] * hold_steps

                grip.extend(gs)
                current_gripper = g1
                continue

        # --- normal waypoint ---
        P = np.array(wp, dtype=float)
        if not pts:
            pts.append(P)
            oris.append(P if use_orientations else np.zeros(3))
            grip.append(current_gripper)
        else:
            seg = generate_noisy_bezier_points(
                pts[-1], P,
                n=n_per_segment,
                num_ctrl=np.random.randint(1, 4),
                radius=np.random.uniform(0.03, 0.07),
                noise_suppresion=int(np.exp(np.random.uniform(np.log(3), np.log(50))))
            )
            # drop duplicate start, extend
            for p in seg[1:]:
                pts.append(p)
                oris.append(p if use_orientations else np.zeros(3))
                grip.append(current_gripper)

    pts = np.array(pts)
    oris = np.array(oris)
    # smooth only the spatial/orientation curves
    pts = moving_average(pts, smooth_window)
    oris = moving_average(oris, smooth_window)

    return pts, oris, np.array(grip)
import random
import time
import sys

def generate_waypoints(i, random_points):
    """
    Generate waypoints for episode i using random_points[i] and random_points[i+1].
    Episode flow:
      random_points[i] -> pick -> target -> drop
      target -> pick -> random_points[i+1] -> drop
    """
    home_pos = [0, 0, 0]
    predef_target = [0.21, 0.2455, -0.185+0.04]  # fixed Z coordinate

    wp = [home_pos]

    # Move cube from random_points[i] to target
    wp += [
        random_points[i],                # hover pickup point
        ['z_25_times_-0.065'],            # descend
        ['pause_20_gripper_0_to_0.40'], # grab
        ['z_10_times_0.08'],             # ascend

        predef_target,                   # drop point
        ['z_25_times_-0.07'],
        ['pause_30_gripper_0.40_to_0'],
        ['z_10_times_0.08'],

        home_pos,
        ['pause_10'],

        # Move cube from target to random_points[i+1]
        predef_target,
        ['z_25_times_-0.07'],
        ['pause_20_gripper_0_to_0.40'],
        ['z_10_times_0.08'],

        random_points[i + 1],
        ['z_25_times_-0.065'],
        ['pause_30_gripper_0.40_to_0'],
        ['z_10_times_0.08'],
    ]

    wp += [home_pos, ['pause_10']]
    return wp


def run_episode(i, total_episodes, random_points):
    # Generate waypoints for current episode
    waypoints = generate_waypoints(i, random_points)

    points, orientations, gripper = generate_full_trajectory(
        waypoints, n_per_segment=29, smooth_window=5, use_orientations=True
    )

    deltas = points_to_deltas(points)
    orientations = [[0, 0, 0] for _ in points]
    hz = 5

    ctrl = RobotArmController(
        control_hz=hz,
        time_step=1 / hz,
        initial_pos=points[0],
        initial_ori=orientations[0]
    )

    extra_sets = [
        [
            [-0.795, 0.457, 0.179],
            [-0.795, 1.14, 0.179],
            [0.3, 1.14, 0.179],
            [0.3, 0.457, 0.179],
            [-0.795, 0.457, 0.95],
            [-0.795, 1.14, 0.95],
            [0.3, 1.14, 0.95],
            [0.3, 0.457, 0.95],
        ]
    ]

    RobotArmGUI(ctrl, deltas, orientations, update_hz=hz, extra_point_sets=extra_sets, gripper=gripper)


def main():
    total_episodes = 100
    episode_times = []

    # Workspace bounding box for random points
    box_bounds = {
        'x_min': -0.312,
        'x_max': 0.059,
        'y_min': -0.176,
        'y_max': 0.38,
        'z': -0.185 + 0.04
    }

    initial_target = [-0.241, -0.0255, box_bounds['z']]

    # Generate all random points once for the whole session
    random_points = [initial_target]
    for _ in range(total_episodes - 1):
        x = random.uniform(box_bounds['x_min'], box_bounds['x_max'])
        y = random.uniform(box_bounds['y_min'], box_bounds['y_max'])
        random_points.append([x, y, box_bounds['z']])
    print(np.mean(random_points))
    time.sleep(2)
    for i in range(total_episodes - 1):  # -1 since we use i and i+1
        print(f"\033c", end="")  # Clear terminal
        print(f"Episode #{i + 1}")

        start_time = time.perf_counter()
        run_episode(i, total_episodes, random_points)
        end_time = time.perf_counter()

        elapsed = end_time - start_time
        episode_times.append(elapsed)

        if i > 0:
            avg_time = sum(episode_times) / len(episode_times)
            episodes_left = (total_episodes - 1) - (i + 1)
            eta = avg_time * episodes_left
            print(f"Estimated time remaining: {eta:.2f} seconds")

    print("All episodes completed.")
    sys.exit(0)


if __name__ == "__main__":
    main()
