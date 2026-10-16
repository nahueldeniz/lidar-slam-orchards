"""Trajectory container and TUM-format input/output."""
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class Trajectory:
    """Timestamped positions, optionally with orientations.

    t: (N,) seconds (message header stamps), p: (N, 3) metres,
    q: (N, 4) unit quaternions in (x, y, z, w) order, or None.
    On construction the samples are sorted by time and repeated stamps are
    dropped (first kept); the counts are kept for reporting.
    """

    t: np.ndarray
    p: np.ndarray
    q: Optional[np.ndarray] = None
    n_unsorted: int = field(default=0, init=False)
    n_duplicate_stamps: int = field(default=0, init=False)

    def __post_init__(self):
        self.t = np.asarray(self.t, dtype=np.float64).reshape(-1)
        self.p = np.asarray(self.p, dtype=np.float64).reshape(-1, 3)
        if self.q is not None:
            self.q = np.asarray(self.q, dtype=np.float64).reshape(-1, 4)
            if len(self.q) != len(self.t):
                raise ValueError("q and t lengths differ")
        if len(self.p) != len(self.t):
            raise ValueError("p and t lengths differ")
        self.n_unsorted = int(np.sum(np.diff(self.t) < 0))
        order = np.argsort(self.t, kind="stable")
        t = self.t[order]
        keep = np.ones(len(t), dtype=bool)
        keep[1:] = np.diff(t) > 0
        self.n_duplicate_stamps = int(np.sum(~keep))
        idx = order[keep]
        self.t, self.p = self.t[idx], self.p[idx]
        if self.q is not None:
            self.q = self.q[idx]

    def __len__(self):
        return len(self.t)

    @property
    def has_orientation(self):
        return self.q is not None

    def rotations(self) -> Rotation:
        if self.q is None:
            raise ValueError("trajectory has no orientation")
        return Rotation.from_quat(self.q)

    def subset(self, idx) -> "Trajectory":
        return Trajectory(self.t[idx], self.p[idx], None if self.q is None else self.q[idx])

    def finite_mask(self):
        m = np.isfinite(self.t) & np.all(np.isfinite(self.p), axis=1)
        if self.q is not None:
            m &= np.all(np.isfinite(self.q), axis=1)
        return m


def read_tum(path) -> Trajectory:
    """Read 'timestamp tx ty tz qx qy qz qw' lines; '#' lines are comments."""
    data = np.loadtxt(path, comments="#", ndmin=2)
    if data.size == 0:
        return Trajectory(np.zeros(0), np.zeros((0, 3)), np.zeros((0, 4)))
    if data.shape[1] != 8:
        raise ValueError(f"{path}: expected 8 columns, got {data.shape[1]}")
    return Trajectory(data[:, 0], data[:, 1:4], data[:, 4:8])


def write_tum(path, traj: Trajectory, header: Optional[str] = None):
    q = traj.q if traj.q is not None else np.tile([0.0, 0.0, 0.0, 1.0], (len(traj), 1))
    data = np.column_stack([traj.t, traj.p, q])
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(path, data, fmt=["%.9f"] + ["%.6f"] * 3 + ["%.9f"] * 4,
               header=header or "", comments="# ")
