"""Synthetic orchard-like trajectories with known ground truth."""
import numpy as np
from scipy.spatial.transform import Rotation

from slameval.trajectory import Trajectory


def lawnmower_pose(s, row_length=40.0, radius=2.0, n_rows=4, z_amp=0.3):
    """Position and yaw at distance s along a lawn-mower path: straight rows
    along +x / -x joined by semicircular turns. Returns (p (3,), yaw)."""
    turn_len = np.pi * radius
    s_rem = s
    for i in range(n_rows):
        y0 = i * 2 * radius
        forward = i % 2 == 0
        if s_rem <= row_length or i == n_rows - 1:
            d = min(s_rem, row_length)
            x = d if forward else row_length - d
            yaw = 0.0 if forward else np.pi
            return np.array([x, y0, z_amp * np.sin(2 * np.pi * s / 60.0)]), yaw
        s_rem -= row_length
        if s_rem <= turn_len:
            phi = s_rem / radius
            if forward:   # left turn around (L, y0 + r)
                th = -np.pi / 2 + phi
                c = np.array([row_length, y0 + radius])
                yaw = th + np.pi / 2
            else:         # right turn around (0, y0 + r)
                th = -np.pi / 2 - phi
                c = np.array([0.0, y0 + radius])
                yaw = th - np.pi / 2
            xy = c + radius * np.array([np.cos(th), np.sin(th)])
            return np.array([xy[0], xy[1], z_amp * np.sin(2 * np.pi * s / 60.0)]), yaw
        s_rem -= turn_len
    raise AssertionError("unreachable")


def path_length(row_length=40.0, radius=2.0, n_rows=4):
    return n_rows * row_length + (n_rows - 1) * np.pi * radius


def true_trajectory(rate_hz, speed=1.0, t0=1000.0, t_offset=0.0, **kw):
    """LiDAR poses (yaw only) sampled at rate_hz; t_offset shifts the first stamp."""
    total = path_length(**kw)
    t = np.arange(0.0, total / speed, 1.0 / rate_hz) + t_offset
    t = t[t * speed <= total]
    poses = [lawnmower_pose(ti * speed, **kw) for ti in t]
    p = np.array([q[0] for q in poses])
    yaw = np.array([q[1] for q in poses])
    q = Rotation.from_euler("z", yaw).as_quat()
    return Trajectory(t + t0, p, q)


def transform(traj: Trajectory, R0: Rotation, t0):
    """Express a trajectory in another world frame: p' = R0 p + t0, R' = R0 R."""
    p = R0.apply(traj.p) + t0
    q = (R0 * Rotation.from_quat(traj.q)).as_quat()
    return Trajectory(traj.t.copy(), p, q)


R_TEST = Rotation.from_euler("zyx", [37.0, 2.0, -1.5], degrees=True)
T_TEST = np.array([12.3, -45.6, 7.8])
