"""Rigid alignment without scale (Section 11.2), using evo's Umeyama solver."""
from dataclasses import dataclass

import numpy as np
from evo.core.geometry import GeometryException, umeyama_alignment


@dataclass
class Alignment:
    R: np.ndarray            # (3, 3)
    t: np.ndarray            # (3,)
    n_used: int
    # Ratio of the 2nd to the 1st singular value of the centred ground-truth
    # points used for the fit; near 0 means nearly collinear points, so the
    # rotation about the line is poorly determined.
    shape_ratio: float

    def apply(self, p):
        return (self.R @ np.asarray(p, dtype=float).T).T + self.t


def _shape_ratio(p):
    c = p - p.mean(axis=0)
    sv = np.linalg.svd(c, compute_uv=False)
    return float(sv[1] / sv[0]) if sv[0] > 0 else 0.0


def fit(p_est, p_gt) -> Alignment:
    """(R*, t*) = argmin sum_k || p_gt_k - (R p_est_k + t) ||^2, R in SO(3)."""
    p_est = np.asarray(p_est, dtype=float)
    p_gt = np.asarray(p_gt, dtype=float)
    if len(p_est) < 3:
        raise GeometryException("need at least three points")
    R, t, _ = umeyama_alignment(p_est.T, p_gt.T, with_scale=False)
    return Alignment(R=R, t=t, n_used=len(p_est), shape_ratio=_shape_ratio(p_gt))


def fit_global(p_est, p_gt) -> Alignment:
    """Alignment A: all associated poses."""
    return fit(p_est, p_gt)


def fit_initial(p_est, p_gt, s_gt, max_distance) -> Alignment:
    """Alignment B: only poses whose ground-truth distance travelled since the
    first associated pose is at most max_distance."""
    s_gt = np.asarray(s_gt, dtype=float)
    mask = (s_gt - s_gt[0]) <= max_distance
    return fit(np.asarray(p_est)[mask], np.asarray(p_gt)[mask])
