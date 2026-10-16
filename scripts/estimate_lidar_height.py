#!/usr/bin/env python3
"""LiDAR height above the ground from the data (brief Section 5.1).

For each of the first N scans of a sequence: keep points 2-20 m away
horizontally and below the sensor, fit a plane with RANSAC (inlier threshold
5 cm, normal within 20 deg of the LiDAR z axis), and take the distance from the
LiDAR origin to the plane. Reports the median and spread over the scans in
datasets/lidar_height_<seq>.json. Nahuel approves the value used (C4).
"""
import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np
import yaml

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R / "eval"))
from slameval import bagio  # noqa: E402

PF_DTYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def pointcloud2_xyz(msg):
    """(N, 3) float array of x, y, z from a sensor_msgs/PointCloud2."""
    fields = {f.name: f for f in msg.fields}
    endian = ">" if msg.is_bigendian else "<"
    dt = np.dtype({"names": ["x", "y", "z"],
                   "formats": [endian + PF_DTYPES[fields[k].datatype] for k in "xyz"],
                   "offsets": [fields[k].offset for k in "xyz"], "itemsize": msg.point_step})
    n = msg.width * msg.height
    arr = np.frombuffer(np.asarray(msg.data, dtype=np.uint8).tobytes(), dtype=dt, count=n)
    xyz = np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(float)
    return xyz[np.all(np.isfinite(xyz), axis=1)]


def ground_height(xyz, rng, iters=300, thresh=0.05, max_tilt_deg=20.0,
                  r_min=2.0, r_max=20.0):
    """Distance from the origin to the RANSAC ground plane, or None."""
    r = np.hypot(xyz[:, 0], xyz[:, 1])
    cand = xyz[(r > r_min) & (r < r_max) & (xyz[:, 2] < 0)]
    if len(cand) < 50:
        return None
    best_n, best_inl = None, 0
    cos_max = np.cos(np.deg2rad(max_tilt_deg))
    for _ in range(iters):
        p = cand[rng.choice(len(cand), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if abs(n[2]) < cos_max:
            continue
        d = -n @ p[0]
        inl = np.sum(np.abs(cand @ n + d) < thresh)
        if inl > best_inl:
            best_inl, best_n = inl, (n, d)
    if best_n is None:
        return None
    n, d = best_n
    inliers = cand[np.abs(cand @ n + d) < thresh]
    # Least-squares refit on the inliers.
    c = inliers.mean(axis=0)
    _, _, vt = np.linalg.svd(inliers - c)
    n = vt[2] if vt[2][2] > 0 else -vt[2]
    return float(abs(n @ c)), float(np.degrees(np.arccos(abs(n[2])))), int(len(inliers))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("seqs", nargs="+")
    ap.add_argument("--scans", type=int, default=100)
    args = ap.parse_args()
    seqs = yaml.safe_load((R / "datasets" / "sequences.yaml").read_text())["sequences"]
    sys.path.insert(0, str(R / "scripts"))
    from inventory import data_root
    root = data_root()
    from rosbags.highlevel import AnyReader
    for seq in args.seqs:
        cfg = seqs[seq]
        rng = np.random.default_rng(0)
        heights, tilts, n_in = [], [], []
        # The first N scans are always in the first bag; opening only it avoids
        # reading the indexes of every bag of the sequence.
        with AnyReader([root / cfg["bags"][0]], default_typestore=bagio.TYPESTORE) as rd:
            conns = [c for c in rd.connections if c.topic == cfg["lidar_topic"]]
            for k, (c, _, raw) in enumerate(rd.messages(connections=conns)):
                if k >= args.scans:
                    break
                res = ground_height(pointcloud2_xyz(rd.deserialize(raw, c.msgtype)), rng)
                if res:
                    heights.append(res[0])
                    tilts.append(res[1])
                    n_in.append(res[2])
        h = np.asarray(heights)
        out = {"seq": seq, "scans_used": len(h), "scans_requested": args.scans,
               "height_median_m": float(np.median(h)) if len(h) else None,
               "height_iqr_m": [float(np.percentile(h, 25)), float(np.percentile(h, 75))] if len(h) else None,
               "plane_tilt_median_deg": float(np.median(tilts)) if tilts else None,
               "inliers_median": float(np.median(n_in)) if n_in else None,
               "method": "RANSAC plane, points 2-20 m horizontal range below the sensor, "
                         "5 cm threshold, normal within 20 deg of LiDAR z, least-squares refit"}
        (R / "datasets" / f"lidar_height_{seq}.json").write_text(json.dumps(out, indent=2) + "\n")
        print(json.dumps(out))


if __name__ == "__main__":
    main()
