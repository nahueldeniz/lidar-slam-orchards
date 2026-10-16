"""Time association of estimated poses with ground truth (Section 11.1)."""
from dataclasses import dataclass

import numpy as np
from scipy.interpolate import CubicSpline


@dataclass
class Association:
    idx: np.ndarray          # indices of the kept estimate samples
    t: np.ndarray            # their stamps
    p_gt: np.ndarray         # ground-truth position linearly interpolated at t
    n_total: int             # estimate samples offered
    n_outside_span: int      # discarded: outside the ground-truth time span
    n_gap: int               # discarded: bracketing ground-truth samples too far apart
    gt_rate_hz: float
    est_rate_hz: float

    @property
    def n(self):
        return len(self.idx)

    @property
    def n_discarded(self):
        return self.n_outside_span + self.n_gap


def median_rate(t):
    dt = np.diff(np.asarray(t, dtype=float))
    dt = dt[dt > 0]
    return float(1.0 / np.median(dt)) if len(dt) else float("nan")


def bracket(gt_t, t):
    """Indices (i0, i1) of the ground-truth samples bracketing each t, and the
    interpolation weight w so that x(t) = (1 - w) x[i0] + w x[i1]."""
    n = len(gt_t)
    j = np.clip(np.searchsorted(gt_t, t, side="right"), 1, n - 1)
    i0, i1 = j - 1, j
    w = (t - gt_t[i0]) / (gt_t[i1] - gt_t[i0])
    return i0, i1, w


def associate(est_t, gt_t, gt_p, max_gap_s) -> Association:
    est_t = np.asarray(est_t, dtype=float)
    gt_t = np.asarray(gt_t, dtype=float)
    gt_p = np.asarray(gt_p, dtype=float)
    if len(gt_t) < 2:
        raise ValueError("ground truth needs at least two samples")
    inside = (est_t >= gt_t[0]) & (est_t <= gt_t[-1])
    i0, i1, w = bracket(gt_t, est_t)
    gap = gt_t[i1] - gt_t[i0]
    exact = (est_t == gt_t[i0]) | (est_t == gt_t[i1])
    ok_gap = (gap <= max_gap_s + 1e-9) | exact
    keep = inside & ok_gap
    idx = np.flatnonzero(keep)
    wk = w[idx][:, None]
    p = (1.0 - wk) * gt_p[i0[idx]] + wk * gt_p[i1[idx]]
    return Association(
        idx=idx, t=est_t[idx], p_gt=p, n_total=len(est_t),
        n_outside_span=int(np.sum(~inside)), n_gap=int(np.sum(inside & ~ok_gap)),
        gt_rate_hz=median_rate(gt_t), est_rate_hz=median_rate(est_t))


def interp_scalar(gt_t, values, t):
    """Linear interpolation of a per-ground-truth-sample scalar at times t."""
    return np.interp(t, gt_t, values)


def spline_vs_linear_rms(gt_t, gt_p, t):
    """RMS distance between cubic-spline and linear interpolation of the ground
    truth at times t: an estimate of the interpolation error (1 Hz ground truth)."""
    if len(t) == 0:
        return float("nan")
    cs = CubicSpline(gt_t, gt_p, axis=0)
    lin = np.column_stack([np.interp(t, gt_t, gt_p[:, k]) for k in range(3)])
    return float(np.sqrt(np.mean(np.sum((cs(t) - lin) ** 2, axis=1))))
