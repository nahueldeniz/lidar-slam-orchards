#!/usr/bin/env python3
"""Ground-truth processing and quality report (brief Section 9).

For every sequence in datasets/sequences.yaml writes gt/<seq>/:
  gt_tum.txt       ground-truth positions in TUM format (identity orientation)
  gt_quality.json  frame, rate, span, gaps, populated fields, path length and,
                   when the base bags are available, RTK fix status, reported
                   covariance, satellite counts and the consistency check
                   against the raw NavSatFix converted to local ENU
  gt_status.pdf/.png  trajectory coloured by RTK fix status
The parts that need the base bags are reported as "pending" until downloaded.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import yaml

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R / "eval"))
from slameval import align, associate, bagio, path  # noqa: E402
from slameval.trajectory import Trajectory, write_tum  # noqa: E402

# NavSatFix status codes (sensor_msgs/NavSatStatus).
STATUS_NAMES = {-1: "no fix", 0: "fix", 1: "SBAS fix", 2: "GBAS fix"}
PUBLISHED_LENGTH_M = {  # Teng et al. (2023), Table 4; Pullally: Nazate-Burgos et al. (2026), Table 5 (~150 m)
    "pl_1": 150.0, "cf_2": 524.6, "cf_3": 2035.2, "cf_4": 896.2, "cf_5": 865.3, "cf_6": 356.6, "cf_7": 1212.7}


def data_root(seqs=None):
    """Bag root: $SLAMBENCH_DATA if set, else data_root in datasets/sequences.yaml,
    else datasets/."""
    import os
    import yaml
    if os.environ.get("SLAMBENCH_DATA"):
        return Path(os.environ["SLAMBENCH_DATA"]).resolve()
    seqs = seqs or yaml.safe_load((R / "datasets" / "sequences.yaml").read_text())
    root = str(seqs.get("data_root") or "")
    return Path(root).resolve() if root and "$" not in root else (R / "datasets").resolve()

def navsatfix_gt(cfg):
    """NavSatFix ground truth -> local ENU (origin at the first fix), plus a
    precision estimate from the data: scatter while the robot stands still and
    the noise of second differences of consecutive samples."""
    import pymap3d
    from slameval.trajectory import Trajectory as T
    d = bagio.read_navsatfix([R / "datasets" / cfg["gt_bag"]], cfg["gt_topic"])
    P = np.column_stack(pymap3d.geodetic2enu(d["lat"], d["lon"], d["alt"], d["lat"][0], d["lon"][0], d["alt"][0]))
    t = d["t"]
    vals, cnt = np.unique(d["status"], return_counts=True)
    sig = np.sqrt(np.clip(d["cov"][:, [0, 4, 8]], 0, None))
    still = []
    for i in range(0, len(t), 5):
        j = np.searchsorted(t, t[i] + 10.0)
        if j < len(t) and t[j - 1] - t[i] > 9.0 and np.hypot(*(P[j - 1, :2] - P[i, :2])) / (t[j - 1] - t[i]) < 0.01:
            still.append(P[i:j].std(axis=0))
    ok = (np.diff(t[:-1]) < 1.5 * cfg["gt_nominal_period_s"]) & (np.diff(t[1:]) < 1.5 * cfg["gt_nominal_period_s"])
    dd = P[2:] - 2 * P[1:-1] + P[:-2]
    rtk = {"status": "ok", "topic": cfg["gt_topic"], "count": int(len(t)),
           "median_rate_hz": associate.median_rate(t), "frame_id": sorted(set(d["frame_id"])),
           "fix_status_counts": {STATUS_NAMES.get(int(v), str(v)): int(c) for v, c in zip(vals, cnt)},
           "fix_status_percent": {STATUS_NAMES.get(int(v), str(v)): 100.0 * c / len(t) for v, c in zip(vals, cnt)},
           "covariance_type_values": np.unique(d["cov_type"]).tolist(),
           "sigma_enu_m": {k: {"median": float(np.median(sig[:, i])), "p95": float(np.percentile(sig[:, i], 95)),
                               "max": float(sig[:, i].max())} for i, k in enumerate(("E", "N", "U"))},
           "precision_from_data": {
               "still_windows_10s": len(still),
               "still_std_m_median": np.median(still, axis=0).tolist() if still else None,
               "second_difference_noise_m": {"horizontal_per_axis": float(np.sqrt(np.mean(np.sum(dd[ok][:, :2] ** 2, 1)) / 2 / 6)),
                                             "vertical": float(np.sqrt(np.mean(dd[ok][:, 2] ** 2) / 6))}},
           "origin_lat_lon_alt": [float(d["lat"][0]), float(d["lon"][0]), float(d["alt"][0])]}
    return T(t, P), rtk, d["status"]


def gt_report(seq, cfg, eval_cfg):
    if cfg.get("gt_type") == "navsatfix":
        gt, _, _ = navsatfix_gt(cfg)
        info = {"msgtype": "sensor_msgs/msg/NavSatFix", "frame_ids": ["local ENU from NavSatFix"], "child_frame_ids": []}
    else:
        gt, info = bagio.read_poses([R / "datasets" / cfg["gt_bag"]], cfg["gt_topic"])
    dt = np.diff(gt.t)
    gap_s = 2.0 * cfg["gt_nominal_period_s"]   # 0.2 s for CitrusFarm (10 Hz), 1.0 s for Pullally (2 Hz)
    gaps = [(float(gt.t[i] - gt.t[0]), float(dt[i])) for i in np.flatnonzero(dt > gap_s)]
    q_ok = bool(gt.q is not None and np.any(np.abs(gt.q).sum(axis=1) > 0))
    arc = path.arc_length(gt.p, eval_cfg["path_length"]["min_step_m"])
    steps = np.linalg.norm(np.diff(gt.p, axis=0), axis=1)
    rep = {
        "seq": seq, "gt_bag": cfg["gt_bag"], "topic": cfg["gt_topic"], "msgtype": info["msgtype"],
        "frame_ids": info["frame_ids"], "child_frame_ids": info["child_frame_ids"],
        "count": len(gt), "t_start": float(gt.t[0]), "t_end": float(gt.t[-1]),
        "span_s": float(gt.t[-1] - gt.t[0]), "median_rate_hz": associate.median_rate(gt.t),
        "gap_threshold_s": gap_s, "gaps": gaps, "n_duplicate_stamps": gt.n_duplicate_stamps,
        "orientation_populated": q_ok,
        "path_length_3d_all_samples_m": float(steps.sum()),
        "path_length_3d_min_step_m": arc.total,
        "path_length_published_m": PUBLISHED_LENGTH_M.get(seq),
        "position_first": gt.p[0].tolist(),
    }
    return Trajectory(gt.t, gt.p), rep


def read_covariance_flags(bag, topic):
    """Whether pose covariance in the ground-truth Odometry messages is populated."""
    from rosbags.highlevel import AnyReader
    with AnyReader([Path(bag)], default_typestore=bagio.TYPESTORE) as r:
        conns = [c for c in r.connections if c.topic == topic]
        nz = 0
        n = 0
        for c, _, raw in r.messages(connections=conns):
            m = r.deserialize(raw, c.msgtype)
            nz += int(np.any(np.asarray(m.pose.covariance) != 0))
            n += 1
    return {"pose_covariance_populated_fraction": nz / n if n else None}


def rtk_report(seq, cfg, gt: Trajectory):
    """RTK status, reported covariance, satellites and ENU consistency from the base bags."""
    bags = [data_root() / b for b in cfg["bags"]]
    if not all(b.exists() for b in bags):
        return {"status": "pending: base bags not downloaded"}, None
    import pymap3d
    from scipy.spatial.transform import Rotation
    rtk_topic = cfg.get("rtk_topic", "/piksi/navsatfix_best_fix")
    d = bagio.read_navsatfix(bags, rtk_topic)
    vals, cnt = np.unique(d["status"], return_counts=True)
    sig = np.sqrt(np.clip(d["cov"][:, [0, 4, 8]], 0, None))
    out = {
        "status": "ok", "topic": rtk_topic, "count": int(len(d["t"])),
        "median_rate_hz": associate.median_rate(d["t"]),
        "fix_status_counts": {STATUS_NAMES.get(int(v), str(v)): int(c) for v, c in zip(vals, cnt)},
        "fix_status_percent": {STATUS_NAMES.get(int(v), str(v)): 100.0 * c / len(d["t"])
                               for v, c in zip(vals, cnt)},
        "covariance_type_values": np.unique(d["cov_type"]).tolist(),
        "sigma_enu_m": {k: {"median": float(np.median(sig[:, i])), "p95": float(np.percentile(sig[:, i], 95)),
                            "max": float(sig[:, i].max()), "n_unique": int(len(np.unique(sig[:, i])))}
                        for i, k in enumerate(("E", "N", "U"))},
    }
    rs = receiver_state(bags, cfg.get("receiver_state_topic", "/piksi/debug/receiver_state"))
    if rs:
        out["receiver_state"] = rs
    e, n, u = pymap3d.geodetic2enu(d["lat"], d["lon"], d["alt"], d["lat"][0], d["lon"][0], d["alt"][0])
    enu = np.column_stack([e, n, u])
    a = associate.associate(d["t"], gt.t, gt.p, 1.5 * cfg["gt_nominal_period_s"])
    al = align.fit(a.p_gt, enu[a.idx])
    res = al.apply(a.p_gt) - enu[a.idx]
    out["enu_consistency"] = {
        "n": a.n, "rms_3d_m": float(np.sqrt(np.mean(np.sum(res ** 2, axis=1)))),
        "rms_horizontal_m": float(np.sqrt(np.mean(np.sum(res[:, :2] ** 2, axis=1)))),
        "rms_vertical_m": float(np.sqrt(np.mean(res[:, 2] ** 2))),
        "gt_to_enu_rotation_zyx_deg": Rotation.from_matrix(al.R).as_euler("zyx", degrees=True).tolist(),
    }
    status_at_gt = np.interp(gt.t, d["t"], d["status"].astype(float))
    return out, np.round(status_at_gt).astype(int)


def receiver_state(bags, topic):
    from rosbags.highlevel import AnyReader
    sats, modes = [], {}
    try:
        with AnyReader([Path(b) for b in bags], default_typestore=bagio.TYPESTORE) as r:
            conns = [c for c in r.connections if c.topic == topic]
            if not conns:
                return None
            for c, _, raw in r.messages(connections=conns):
                m = r.deserialize(raw, c.msgtype)
                if hasattr(m, "num_sat"):
                    sats.append(int(m.num_sat))
                mode = getattr(m, "fix_mode", None)
                if mode is not None:
                    modes[str(mode)] = modes.get(str(mode), 0) + 1
    except Exception as exc:   # message definition differences: report, do not fail
        return {"error": repr(exc)}
    s = np.asarray(sats)
    return {"topic": topic, "num_sat": {"min": int(s.min()), "median": float(np.median(s)),
                                        "max": int(s.max())} if len(s) else None,
            "fix_mode_counts": modes}


def plot_status(seq, gt: Trajectory, status, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6, 6))
    if status is None:
        ax.plot(gt.p[:, 0], gt.p[:, 1], lw=1.0, color="0.3", label="Ground truth (fix status pending)")
    else:
        for v in np.unique(status):
            m = status == v
            ax.scatter(gt.p[m, 0], gt.p[m, 1], s=2, label=STATUS_NAMES.get(int(v), str(v)))
    ax.plot(*gt.p[0, :2], "o", color="green", label="Start")
    ax.plot(*gt.p[-1, :2], "s", color="red", label="End")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"{seq}: ground truth coloured by RTK fix status")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    for ext in ("pdf", "png"):
        fig.savefig(out_dir / f"gt_status.{ext}", dpi=200)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seqs", nargs="*")
    args = ap.parse_args()
    seqs = yaml.safe_load((R / "datasets" / "sequences.yaml").read_text())["sequences"]
    eval_cfg = yaml.safe_load((R / "eval" / "config" / "eval.yaml").read_text())
    for seq in args.seqs or list(seqs):
        cfg = seqs[seq]
        out_dir = R / "gt" / seq
        out_dir.mkdir(parents=True, exist_ok=True)
        gt, rep = gt_report(seq, cfg, eval_cfg)
        if cfg.get("gt_type") == "navsatfix":
            _, rtk, st = navsatfix_gt(cfg)
            status = st.astype(int)
        else:
            rep.update(read_covariance_flags(R / "datasets" / cfg["gt_bag"], cfg["gt_topic"]))
            rtk, status = rtk_report(seq, cfg, gt)
        rep["rtk"] = rtk
        write_tum(out_dir / "gt_tum.txt", gt,
                  header=f"{seq} ground truth from {cfg['gt_bag']} {cfg['gt_topic']}; "
                         "positions only (identity orientation)")
        (out_dir / "gt_quality.json").write_text(json.dumps(rep, indent=2) + "\n")
        plot_status(seq, gt, status, out_dir)
        print(f"{seq}: {rep['count']} samples, {rep['median_rate_hz']:.2f} Hz, "
              f"{rep['path_length_3d_all_samples_m']:.1f} m (published {rep['path_length_published_m']}), "
              f"RTK: {rtk['status']}")


if __name__ == "__main__":
    main()
