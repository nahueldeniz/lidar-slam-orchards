#!/usr/bin/env python3
"""Summarise a NavSatFix topic in a ROS 1 bag: gaps, local-ENU extent, path length.

Usage: python -I navsatfix_track.py <bag> <topic> [gap_s]
ENU origin = first fix (pymap3d.geodetic2enu, WGS84). Read-only; prints to stdout.
"""
import sys
from collections import Counter
from datetime import datetime, timezone

import numpy as np
import pymap3d
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore


def main(bag, topic, gap_s=5.0):
    ts = get_typestore(Stores.ROS1_NOETIC)
    t, lla, sde = [], [], []
    with Reader(bag) as rd:
        conns = [c for c in rd.connections if c.topic == topic]
        for conn, tb, raw in rd.messages(connections=conns):
            m = ts.deserialize_ros1(raw, conn.msgtype)
            t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            lla.append((m.latitude, m.longitude, m.altitude))
            sde.append(round(float(np.sqrt(m.position_covariance[0])), 4))
    t, lla = np.array(t), np.array(lla)
    e, n, u = pymap3d.geodetic2enu(lla[:, 0], lla[:, 1], lla[:, 2], *lla[0])
    enu = np.c_[e, n, u]
    dt = np.diff(t)
    cut = np.where(dt > gap_s)[0]
    print(f"{len(t)} fixes, header span {t[-1]-t[0]:.1f} s, median dt {np.median(dt):.3f} s")
    print("sigma_E values (m) and counts:", sorted(Counter(sde).items()))
    bounds = [0] + list(cut + 1) + [len(t)]
    for a, b in zip(bounds[:-1], bounds[1:]):
        seg = enu[a:b]
        L2 = np.linalg.norm(np.diff(seg[:, :2], axis=0), axis=1).sum()
        L3 = np.linalg.norm(np.diff(seg, axis=0), axis=1).sum()
        s0 = datetime.fromtimestamp(t[a], tz=timezone.utc).strftime("%H:%M:%S")
        s1 = datetime.fromtimestamp(t[b - 1], tz=timezone.utc).strftime("%H:%M:%S")
        print(f"segment {s0}-{s1} UTC: {b-a} fixes, {t[b-1]-t[a]:.1f} s, path 2D {L2:.1f} m / 3D {L3:.1f} m, "
              f"E [{seg[:,0].min():.1f},{seg[:,0].max():.1f}] N [{seg[:,1].min():.1f},{seg[:,1].max():.1f}] "
              f"U [{seg[:,2].min():.2f},{seg[:,2].max():.2f}], start->end {np.linalg.norm(seg[-1,:2]-seg[0,:2]):.2f} m")
    for i in cut:
        print(f"gap {dt[i]:.1f} s after {datetime.fromtimestamp(t[i], tz=timezone.utc).strftime('%H:%M:%S')} UTC")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 5.0)
