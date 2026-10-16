"""Evaluation of one trajectory output against ground truth (Sections 10-11).

evaluate_output() returns long-format rows for results_long.csv, per-segment
rows for segments.csv, heading rows for heading.csv and run diagnostics.
"""
from dataclasses import dataclass, field

import numpy as np
from evo.core.geometry import GeometryException

from . import align, associate as assoc_mod, metrics, path
from .trajectory import Trajectory


@dataclass
class GroundTruth:
    traj: Trajectory              # local ENU (z up) positions of the reference point
    arc: path.ArcLength           # distance travelled at each ground-truth sample
    nominal_period_s: float
    turn: np.ndarray              # turn label per ground-truth sample

    @property
    def t(self):
        return self.traj.t

    @property
    def p(self):
        return self.traj.p


def prepare_ground_truth(traj: Trajectory, nominal_period_s, cfg) -> GroundTruth:
    arc = path.arc_length(traj.p, cfg["path_length"]["min_step_m"])
    tc = cfg["turn"]
    turn = path.turn_flags(arc, arc.s, tc["heading_baseline_m"], tc["window_m"], tc["threshold_deg"])
    return GroundTruth(traj=traj, arc=arc, nominal_period_s=nominal_period_s, turn=turn)


@dataclass
class EvalResult:
    rows: list = field(default_factory=list)
    segments: list = field(default_factory=list)
    heading: list = field(default_factory=list)
    diag: dict = field(default_factory=dict)


def _gt_speed(gt: GroundTruth, t, half_window):
    a = np.column_stack([np.interp(t - half_window, gt.t, gt.p[:, k]) for k in range(3)])
    b = np.column_stack([np.interp(t + half_window, gt.t, gt.p[:, k]) for k in range(3)])
    return np.linalg.norm((b - a)[:, :2], axis=1) / (2.0 * half_window)


def estimated_positions(est: Trajectory, lever_arm):
    """Positions of the evaluated point: the LiDAR origin, or the antenna
    p_ant = p_lidar + R_lidar l when a lever arm l (LiDAR frame) is given."""
    if lever_arm is None:
        return est.p.copy()
    return est.p + est.rotations().apply(np.asarray(lever_arm, dtype=float))


def time_offset_scan(est_t, p_hat, gt: GroundTruth, max_gap_s, cfg):
    """delta* = argmin RMSE(A) with the ground-truth stamps shifted by +delta.
    delta* > 0 means the estimate stamps lag the ground truth. Reported only."""
    tc = cfg["time_offset"]
    deltas = np.round(np.arange(tc["min_s"], tc["max_s"] + tc["step_s"] / 2, tc["step_s"]), 6)
    rmse = np.full(len(deltas), np.nan)
    for i, d in enumerate(deltas):
        a = assoc_mod.associate(est_t, gt.t + d, gt.p, max_gap_s)
        if a.n < 3:
            continue
        try:
            al = align.fit_global(p_hat[a.idx], a.p_gt)
        except GeometryException:
            continue
        e = al.apply(p_hat[a.idx]) - a.p_gt
        rmse[i] = np.sqrt(np.mean(np.sum(e ** 2, axis=1)))
    if np.all(np.isnan(rmse)):
        return {"delta_star_s": float("nan"), "rmse_at_delta_star": float("nan"),
                "rmse_at_zero": float("nan"), "at_boundary": False}
    k = int(np.nanargmin(rmse))
    zero = int(np.argmin(np.abs(deltas)))
    return {"delta_star_s": float(deltas[k]), "rmse_at_delta_star": float(rmse[k]),
            "rmse_at_zero": float(rmse[zero]), "at_boundary": k in (0, len(deltas) - 1)}


def _segments(p_hat_k, p_gt_k, s_k, gt: GroundTruth, cfg):
    sc, tc = cfg["segments"], cfg["turn"]
    out = []
    total = gt.arc.total
    for L in sc["lengths_m"]:
        for m in range(int(total // L)):
            a, b = m * L, (m + 1) * L
            in_gt = (gt.arc.s >= a) & (gt.arc.s < b)
            turn_frac = float(np.mean(gt.turn[in_gt])) if in_gt.any() else float("nan")
            row = {"L_m": L, "seg_id": m, "s_start_m": a, "s_end_m": b,
                   "turn_fraction": turn_frac,
                   "class": path.segment_class(turn_frac, tc["mixed_low"], tc["mixed_high"]),
                   "n": 0, "status": "ok"}
            mask = (s_k >= a) & (s_k < b)
            n = int(mask.sum())
            row["n"] = n
            if n < sc["min_poses"]:
                row["status"] = "too_few_poses"
            elif s_k[mask].max() - s_k[mask].min() < sc["min_span_fraction"] * L:
                row["status"] = "incomplete_span"
            else:
                try:
                    al = align.fit(p_hat_k[mask], p_gt_k[mask])
                    comp = metrics.error_components(al.apply(p_hat_k[mask]) - p_gt_k[mask])
                    for dim in ("3d", "2d"):
                        row[f"rmse_{dim}"] = float(np.sqrt(np.mean(comp[dim] ** 2)))
                        row[f"end_error_{dim}"] = float(comp[dim][-1])
                    row["shape_ratio"] = al.shape_ratio
                except GeometryException:
                    row["status"] = "degenerate"
            out.append(row)
    return out


def _heading(est: Trajectory, a, R_star, s_k, gt: GroundTruth, forward_axis, cfg):
    hc, tc = cfg["heading"], cfg["turn"]
    psi_gt = path.heading_at(gt.arc, s_k, hc["baseline_m"])
    speed = _gt_speed(gt, a.t, hc["speed_half_window_s"])
    R_hat = est.rotations()[a.idx]
    f = R_hat.apply(np.asarray(forward_axis, dtype=float))   # forward axis in method world
    f = (R_star @ f.T).T                                       # ... in ground-truth frame
    psi_hat = np.arctan2(f[:, 1], f[:, 0])
    d = path.wrap_angle(psi_hat - psi_gt)
    valid = (speed > hc["min_speed_mps"]) & np.isfinite(d)
    row_flag = ~path.turn_flags(gt.arc, s_k, tc["heading_baseline_m"], tc["window_m"], tc["threshold_deg"])
    out = []
    for subset, m in (("all", valid), ("row", valid & row_flag)):
        dd = np.rad2deg(d[m])
        if len(dd) == 0:
            out.append({"subset": subset, "n": 0})
            continue
        circ_mean = np.rad2deg(np.arctan2(np.mean(np.sin(d[m])), np.mean(np.cos(d[m]))))
        out.append({"subset": subset, "n": int(m.sum()),
                    "rmse_deg": float(np.sqrt(np.mean(dd ** 2))),
                    "mean_deg": float(np.mean(dd)),
                    "max_abs_deg": float(np.max(np.abs(dd))),
                    "circular_mean_deg": float(circ_mean)})
    return out


def evaluate_output(est: Trajectory, gt: GroundTruth, cfg, meta, lever_arm=None,
                    forward_axis=(1.0, 0.0, 0.0), do_time_offset=True) -> EvalResult:
    """Evaluate one output. `meta` holds method, seq, config, rep and output.
    Positions are evaluated without the lever arm and, when `lever_arm` is
    given and the output has orientation, also with it."""
    res = EvalResult()
    res.diag = dict(meta)
    finite = est.finite_mask()
    res.diag.update({"n_samples": len(est), "n_unsorted": est.n_unsorted,
                     "n_duplicate_stamps": est.n_duplicate_stamps})
    res.diag.update(metrics.divergence(est.p, cfg["divergence"]["max_jump_m"]))
    est = est.subset(np.flatnonzero(finite))

    max_gap = cfg["association"]["max_gap_periods"] * gt.nominal_period_s
    a = assoc_mod.associate(est.t, gt.t, gt.p, max_gap)
    res.diag.update({"n_poses": a.n, "n_discarded": a.n_discarded,
                     "n_outside_span": a.n_outside_span, "n_gap": a.n_gap,
                     "gt_rate_hz": a.gt_rate_hz, "est_rate_hz": a.est_rate_hz,
                     "gt_path_length_m": gt.arc.total})
    if a.n < 3:
        res.diag["status"] = "no_overlap"
        return res

    s_k = np.interp(a.t, gt.t, gt.arc.s)
    path_len = float(s_k[-1] - s_k[0])
    gt_span = gt.t[-1] - gt.t[0]
    res.diag.update({
        "path_length_m": path_len,
        "coverage_time": float((a.t[-1] - a.t[0]) / gt_span) if gt_span > 0 else float("nan"),
        "coverage_distance": path_len / gt.arc.total if gt.arc.total > 0 else float("nan"),
        "max_output_gap_s": float(np.max(np.diff(a.t))) if a.n > 1 else float("nan"),
        "spline_vs_linear_rms_m": assoc_mod.spline_vs_linear_rms(gt.t, gt.p, a.t),
        "status": "ok",
    })

    variants = [False]
    if lever_arm is not None and est.has_orientation:
        variants.append(True)
    base = {k: meta[k] for k in ("method", "seq", "config", "rep", "output")}
    for use_l in variants:
        p_hat = estimated_positions(est, lever_arm if use_l else None)
        p_k = p_hat[a.idx]
        tag = "with" if use_l else "without"
        R_star = None
        for name in ("A", "B"):
            try:
                if name == "A":
                    al = align.fit_global(p_k, a.p_gt)
                    R_star = al.R
                else:
                    al = align.fit_initial(p_k, a.p_gt, s_k, cfg["alignment_B"]["initial_distance_m"])
                    res.diag[f"B_n_used_{tag}"] = al.n_used
                    res.diag[f"B_shape_ratio_{tag}"] = al.shape_ratio
            except GeometryException as exc:
                res.diag[f"{name}_error_{tag}"] = str(exc)
                continue
            comp = metrics.error_components(al.apply(p_k) - a.p_gt)
            for dim, err in comp.items():
                for metric, value in metrics.stats(err, path_len).items():
                    res.rows.append({**base, "alignment": name, "lever_arm": tag, "dim": dim,
                                     "metric": metric, "value": value, "n_poses": a.n,
                                     "n_discarded": a.n_discarded, "path_length_m": path_len})
        for seg in _segments(p_k, a.p_gt, s_k, gt, cfg):
            res.segments.append({**base, "lever_arm": tag, **seg})
        if R_star is not None and est.has_orientation and forward_axis is not None:
            for h in _heading(est, a, R_star, s_k, gt, forward_axis, cfg):
                res.heading.append({**base, "lever_arm": tag, **h})
        if do_time_offset:
            to = time_offset_scan(est.t, p_hat, gt, max_gap, cfg)
            res.diag.update({f"{k}_{tag}": v for k, v in to.items()})
    return res
