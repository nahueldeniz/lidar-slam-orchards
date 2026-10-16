"""Position-error statistics (Section 11.3-11.4)."""
import numpy as np

STAT_NAMES = ("rmse", "mean", "median", "sd", "max", "final", "rmse_pct", "final_pct")


def error_components(e):
    """Per-pose error for each dimension of the residual e (N, 3), local ENU.
    3d: ||e||; 2d: horizontal ||e_xy||; z: vertical |e_z|."""
    e = np.asarray(e, dtype=float)
    return {
        "3d": np.linalg.norm(e, axis=1),
        "2d": np.linalg.norm(e[:, :2], axis=1),
        "z": np.abs(e[:, 2]),
    }


def stats(err, path_length_m):
    """RMSE, mean, median, SD (population), max, final error and the RMSE and
    final error as a percentage of the path length."""
    err = np.asarray(err, dtype=float)
    if len(err) == 0:
        return {k: float("nan") for k in STAT_NAMES}
    rmse = float(np.sqrt(np.mean(err ** 2)))
    final = float(err[-1])
    pct = (100.0 / path_length_m) if path_length_m > 0 else float("nan")
    return {
        "rmse": rmse,
        "mean": float(np.mean(err)),
        "median": float(np.median(err)),
        "sd": float(np.std(err)),
        "max": float(np.max(err)),
        "final": final,
        "rmse_pct": rmse * pct,
        "final_pct": final * pct,
    }


def divergence(p, max_jump):
    """Count non-finite poses and jumps larger than max_jump between consecutive poses."""
    p = np.asarray(p, dtype=float)
    finite = np.all(np.isfinite(p), axis=1)
    jumps = np.linalg.norm(np.diff(p[finite], axis=0), axis=1) if finite.sum() > 1 else np.zeros(0)
    return {"n_nonfinite": int(np.sum(~finite)), "n_jumps": int(np.sum(jumps > max_jump)),
            "max_step_m": float(jumps.max()) if len(jumps) else 0.0}
