"""Turn recorded method outputs into LiDAR-pose trajectories (Section 10).

Each evaluated output is declared in configs/methods.yaml as, e.g.,

    eval_outputs:
      fe:
        kind: topic                 # poses from one topic, header stamps
        topic: /laser_odom_to_init
        group: fe                   # fe | full
        left: {R: [[...], [...], [...]], t: [0, 0, 0]}    # world fix, optional
        right: {R: [[...], [...], [...]], t: [0, 0, 0]}   # output body -> LiDAR, optional

The LiDAR pose in a z-up world frame is T_W_L = left * T_out * right, with
4x4 homogeneous transforms; matrices are written out explicitly to avoid
Euler-angle conventions. Two special kinds serve hdl_graph_slam:

    kind: compose_latest   T(t) = T_corr(latest at or before t) * T_odom(t)
      correction_topic, odom_topic
    kind: hdl_dump         keyframe 'estimate' poses from the dump directory
      dump_dir (relative to the run directory)
"""
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from . import bagio
from .trajectory import Trajectory


def to_matrix(spec):
    T = np.eye(4)
    if spec:
        T[:3, :3] = np.asarray(spec.get("R", np.eye(3)), dtype=float)
        T[:3, 3] = np.asarray(spec.get("t", [0, 0, 0]), dtype=float)
        if not np.allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-6) or np.linalg.det(T[:3, :3]) < 0:
            raise ValueError(f"not a rotation matrix: {spec}")
    return T


def poses_to_matrices(traj: Trajectory):
    T = np.tile(np.eye(4), (len(traj), 1, 1))
    T[:, :3, :3] = Rotation.from_quat(traj.q).as_matrix()
    T[:, :3, 3] = traj.p
    return T


def matrices_to_traj(t, T):
    return Trajectory(t, T[:, :3, 3], Rotation.from_matrix(T[:, :3, :3]).as_quat())


def apply_frames(traj: Trajectory, spec):
    L, Rt = to_matrix(spec.get("left")), to_matrix(spec.get("right"))
    T = L[None] @ poses_to_matrices(traj) @ Rt[None]
    return matrices_to_traj(traj.t, T)


def compose_latest(corr: Trajectory, odom: Trajectory, corr_key=None, odom_key=None):
    """T_map_lidar(t) = T_map_odom(latest correction at or before t) * T_odom_lidar(t).

    By default "at or before" compares header stamps. For the online output of
    hdl_graph_slam pass the recording (receipt) times as corr_key/odom_key: its
    map->odom correction is stamped with the newest keyframe but published up to
    seconds later, so comparing header stamps would apply corrections before they
    existed. Odometry poses before the first correction use the identity."""
    Tc, To = poses_to_matrices(corr), poses_to_matrices(odom)
    ck = corr.t if corr_key is None else np.asarray(corr_key)
    ok = odom.t if odom_key is None else np.asarray(odom_key)
    order = np.argsort(ck, kind="stable")
    ck, Tc = ck[order], Tc[order]
    j = np.searchsorted(ck, ok, side="right") - 1
    Tcorr = np.where((j >= 0)[:, None, None], Tc[np.clip(j, 0, None)], np.eye(4)[None])
    return matrices_to_traj(odom.t, Tcorr @ To)


def read_hdl_dump(dump_dir):
    """hdl_graph_slam dump: one directory per keyframe with a 'data' file that
    holds 'stamp <sec> <nsec>' and a 4x4 'estimate' matrix."""
    t, T = [], []
    for d in sorted(Path(dump_dir).iterdir()):
        f = d / "data"
        if not f.is_file():
            continue
        lines = f.read_text().splitlines()
        stamp, est = None, None
        for i, line in enumerate(lines):
            parts = line.split()
            if parts and parts[0] == "stamp":
                stamp = int(parts[1]) + int(parts[2]) * 1e-9
            if parts and parts[0] == "estimate":
                est = np.array([[float(x) for x in lines[i + 1 + r].split()] for r in range(4)])
        if stamp is not None and est is not None:
            t.append(stamp)
            T.append(est)
    if not t:
        raise FileNotFoundError(f"no keyframes in {dump_dir}")
    return matrices_to_traj(np.asarray(t), np.asarray(T))


def _receipt_for(traj: Trajectory, info):
    """Receipt (recording) time of each sample of a de-duplicated trajectory."""
    raw_t = np.asarray(info["raw_stamps"])
    rec = np.asarray(info["record_times"])
    first = {}
    for st, r in zip(raw_t, rec):
        first.setdefault(st, r)
    return np.array([first[st] for st in traj.t])


def load_output(run_dir, spec):
    """Trajectory of the LiDAR pose for one declared output, plus info."""
    run_dir = Path(run_dir)
    bag = [run_dir / "outputs.bag"]
    kind = spec.get("kind", "topic")
    if kind == "topic":
        traj, info = bagio.read_poses(bag, spec["topic"])
    elif kind == "compose_latest":
        corr, ci = bagio.read_poses(bag, spec["correction_topic"])
        odom, oi = bagio.read_poses(bag, spec["odom_topic"])
        if spec.get("associate_by", "receipt") == "receipt":
            # Trajectory() sorts by stamp and drops repeated stamps; recover the
            # matching receipt times by stamp.
            ck = _receipt_for(corr, ci)
            traj = compose_latest(corr, odom, ck, _receipt_for(odom, oi))
        else:
            traj = compose_latest(corr, odom)
        ci.pop("record_times", None)
        oi.pop("record_times", None)
        info = {"correction": ci, "odom": oi, "associate_by": spec.get("associate_by", "receipt")}
    elif kind == "hdl_dump":
        traj = read_hdl_dump(run_dir / spec["dump_dir"])
        info = {"dump_dir": spec["dump_dir"], "count": len(traj)}
    else:
        raise ValueError(f"unknown output kind {kind}")
    return apply_frames(traj, spec), info
