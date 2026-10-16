#!/usr/bin/env python3
"""First look at CitrusFarm gt.bag files (brief Section 9; not the full GT processing).

Usage: python -I gt_overview.py <ground_truth_dir> <out_json>
  <ground_truth_dir> contains <folder>/gt.bag and <folder>/gt.csv.

For every gt.bag: connections (topic, type, count), bag start/end, and for each
nav_msgs/Odometry topic: frame_id / child_frame_id values, header-stamp span,
rate and inter-message intervals, gaps > 0.2 s, header-stamp vs bag-time
offset, whether orientation, pose covariance and twist are populated, position
extent and path length. Also compares gt.bag with gt.csv.
Writes a JSON summary and prints a Markdown report to stdout.
"""
import json
import os
import sys
from collections import Counter
from datetime import datetime, timezone

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

GAP_S = 0.2


def utc(t):
    return datetime.fromtimestamp(t, tz=timezone.utc).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] + " UTC"


def odom_stats(msgs):
    t_hdr = np.array([m["t_hdr"] for m in msgs])
    t_bag = np.array([m["t_bag"] for m in msgs])
    pos = np.array([m["p"] for m in msgs])
    quat = np.array([m["q"] for m in msgs])
    pcov = np.array([m["pcov"] for m in msgs])
    tw = np.array([m["tw"] for m in msgs])
    tcov = np.array([m["tcov"] for m in msgs])
    dt = np.diff(t_hdr)
    seg = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    seg2 = np.linalg.norm(np.diff(pos[:, :2], axis=0), axis=1)
    pdiag = pcov[:, [0, 7, 14, 21, 28, 35]]
    uq = np.unique(quat.round(9), axis=0)
    gaps = [(float(t_hdr[i] - t_hdr[0]), float(dt[i])) for i in np.where(dt > GAP_S)[0]]
    return {
        "count": len(msgs),
        "frame_id": dict(Counter(m["frame_id"] for m in msgs)),
        "child_frame_id": dict(Counter(m["child_frame_id"] for m in msgs)),
        "t_first_hdr": float(t_hdr[0]), "t_last_hdr": float(t_hdr[-1]),
        "t_first_hdr_utc": utc(t_hdr[0]), "t_last_hdr_utc": utc(t_hdr[-1]),
        "span_s": float(t_hdr[-1] - t_hdr[0]),
        "rate_hz_count_over_span": float((len(msgs) - 1) / (t_hdr[-1] - t_hdr[0])),
        "dt_median_s": float(np.median(dt)), "dt_mean_s": float(dt.mean()),
        "dt_min_s": float(dt.min()), "dt_max_s": float(dt.max()),
        "dt_p01_s": float(np.percentile(dt, 1)), "dt_p99_s": float(np.percentile(dt, 99)),
        "n_nonpositive_dt": int((dt <= 0).sum()),
        "n_gaps_gt_0p2s": len(gaps), "gaps_gt_0p2s_(t_from_start, dt)": gaps[:50],
        "hdr_minus_bag_s": {"min": float((t_hdr - t_bag).min()), "median": float(np.median(t_hdr - t_bag)),
                            "max": float((t_hdr - t_bag).max())},
        "first_position": pos[0].tolist(),
        "pos_min": pos.min(0).tolist(), "pos_max": pos.max(0).tolist(),
        "path_length_3d_m": float(seg.sum()), "path_length_2d_m": float(seg2.sum()),
        "end_minus_start_m": (pos[-1] - pos[0]).tolist(),
        "orientation_unique_values(first 5)": uq[:5].tolist(), "orientation_n_unique": int(len(uq)),
        "orientation_populated": bool(not (len(uq) == 1 and (np.allclose(uq[0], 0) or np.allclose(uq[0], [0, 0, 0, 1])))),
        "pose_cov_all_zero": bool(np.allclose(pcov, 0)),
        "pose_cov_n_unique_matrices": int(len(np.unique(pcov.round(12), axis=0))),
        "pose_cov_diag_min": pdiag.min(0).tolist(), "pose_cov_diag_median": np.median(pdiag, 0).tolist(),
        "pose_cov_diag_max": pdiag.max(0).tolist(),
        "pose_cov_offdiag_all_zero": bool(np.allclose(pcov - np.array([np.diag(np.diag(c.reshape(6, 6))).ravel() for c in pcov]), 0)),
        "twist_all_zero": bool(np.allclose(tw, 0)), "twist_cov_all_zero": bool(np.allclose(tcov, 0)),
    }


def read_csv(path):
    rows = []
    with open(path) as f:
        header = [ln for ln in (next(f) for _ in range(3))]
        for ln in f:
            if ln.strip() and not ln.startswith("#"):
                rows.append([float(x) for x in ln.split(",")])
    return header, np.array(rows)


def main(gt_dir, out_json):
    ts = get_typestore(Stores.ROS1_NOETIC)
    result = {}
    for folder in sorted(os.listdir(gt_dir)):
        bag = os.path.join(gt_dir, folder, "gt.bag")
        if not os.path.isfile(bag):
            continue
        r = {"connections": [], "odometry": {}}
        with Reader(bag) as rd:
            r["bag_start"] = rd.start_time * 1e-9
            r["bag_end"] = rd.end_time * 1e-9
            r["bag_duration_s"] = (rd.end_time - rd.start_time) * 1e-9
            r["bag_start_utc"] = utc(r["bag_start"])
            for c in rd.connections:
                r["connections"].append({"topic": c.topic, "msgtype": c.msgtype, "count": c.msgcount})
            per_topic = {}
            for conn, t, raw in rd.messages():
                if conn.msgtype != "nav_msgs/msg/Odometry":
                    continue
                m = ts.deserialize_ros1(raw, conn.msgtype)
                p, q = m.pose.pose.position, m.pose.pose.orientation
                tl, an = m.twist.twist.linear, m.twist.twist.angular
                per_topic.setdefault(conn.topic, []).append({
                    "t_bag": t * 1e-9,
                    "t_hdr": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                    "frame_id": m.header.frame_id, "child_frame_id": m.child_frame_id,
                    "p": [p.x, p.y, p.z], "q": [q.x, q.y, q.z, q.w],
                    "pcov": list(m.pose.covariance), "tw": [tl.x, tl.y, tl.z, an.x, an.y, an.z],
                    "tcov": list(m.twist.covariance)})
            for topic, msgs in per_topic.items():
                r["odometry"][topic] = odom_stats(msgs)
                # compare with gt.csv
                csvp = os.path.join(gt_dir, folder, "gt.csv")
                if os.path.isfile(csvp):
                    hdr, arr = read_csv(csvp)
                    t_hdr = np.array([m["t_hdr"] for m in msgs])
                    t_bag = np.array([m["t_bag"] for m in msgs])
                    pos = np.array([m["p"] for m in msgs])
                    cmp = {"csv_header_lines": [h.rstrip("\n") for h in hdr], "csv_rows": int(len(arr))}
                    if len(arr) == len(msgs):
                        cmp["max_abs_dt_vs_header_s"] = float(np.abs(arr[:, 0] - t_hdr).max())
                        cmp["max_abs_dt_vs_bagtime_s"] = float(np.abs(arr[:, 0] - t_bag).max())
                        cmp["max_abs_dpos_m"] = float(np.abs(arr[:, 1:4] - pos).max())
                        cmp["csv_quaternion_all_zero"] = bool(np.allclose(arr[:, 4:8], 0))
                    r["odometry"][topic]["csv_compare"] = cmp
        result[folder] = r
    with open(out_json, "w") as f:
        json.dump(result, f, indent=1)

    # Markdown to stdout
    print("| Folder | Topic | Type | Count | frame_id | child_frame_id | Header span (s) | Rate (Hz) | dt median / min / max (s) | Gaps > 0.2 s | Orientation | Pose cov. | Twist |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for folder, r in result.items():
        for topic, s in r["odometry"].items():
            typ = next(c["msgtype"] for c in r["connections"] if c["topic"] == topic)
            ori = "populated" if s["orientation_populated"] else f"not populated ({s['orientation_unique_values(first 5)'][0]})"
            cov = "all zero" if s["pose_cov_all_zero"] else (
                f"diag median {[round(v, 6) for v in s['pose_cov_diag_median'][:3]]} (xyz)"
                + (", off-diag 0" if s["pose_cov_offdiag_all_zero"] else ", off-diag non-zero"))
            print(f"| {folder} | `{topic}` | {typ} | {s['count']} | {', '.join(s['frame_id'])} | {', '.join(s['child_frame_id'])} "
                  f"| {s['span_s']:.2f} | {s['rate_hz_count_over_span']:.3f} | {s['dt_median_s']:.4f} / {s['dt_min_s']:.4f} / {s['dt_max_s']:.4f} "
                  f"| {s['n_gaps_gt_0p2s']} | {ori} | {cov} | {'all zero' if s['twist_all_zero'] else 'populated'} |")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
