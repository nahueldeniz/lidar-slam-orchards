#!/usr/bin/env python3
"""Generic ROS 1 bag overview: topics, types, counts, time span, rates.

Usage: python -I bag_overview.py <bag> [<bag> ...]
For sensor_msgs/NavSatFix topics also prints status counts, covariance type and
position-covariance statistics; for nav_msgs/Odometry, frame ids and whether
orientation/covariance are populated; for PointCloud2, frame id and size.
Read-only; prints to stdout.
"""
import sys
from collections import Counter, defaultdict
from datetime import datetime, timezone

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore


def utc(t):
    return datetime.fromtimestamp(t, tz=timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")


def main(paths):
    ts = get_typestore(Stores.ROS1_NOETIC)
    for path in paths:
        print(f"## {path}")
        with Reader(path) as rd:
            t0, t1 = rd.start_time * 1e-9, rd.end_time * 1e-9
            print(f"bag start {utc(t0)}  end {utc(t1)}  duration {t1 - t0:.2f} s")
            stamps = defaultdict(list)
            extra = defaultdict(lambda: defaultdict(Counter))
            nums = defaultdict(lambda: defaultdict(list))
            for c in rd.connections:
                print(f"  {c.topic:40s} {c.msgtype:40s} {c.msgcount}")
            for conn, t, raw in rd.messages():
                stamps[conn.topic].append(t * 1e-9)
                mt = conn.msgtype
                if mt in ("sensor_msgs/msg/NavSatFix", "nav_msgs/msg/Odometry", "sensor_msgs/msg/PointCloud2",
                          "geometry_msgs/msg/PoseStamped"):
                    m = ts.deserialize_ros1(raw, mt)
                    e = extra[conn.topic]
                    e["frame_id"][m.header.frame_id] += 1
                    nums[conn.topic]["hdr"].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
                    if mt == "sensor_msgs/msg/NavSatFix":
                        e["status"][int(m.status.status)] += 1
                        e["service"][int(m.status.service)] += 1
                        e["cov_type"][int(m.position_covariance_type)] += 1
                        cv = np.array(m.position_covariance)
                        nums[conn.topic]["sd_e"].append(np.sqrt(max(cv[0], 0)))
                        nums[conn.topic]["sd_n"].append(np.sqrt(max(cv[4], 0)))
                        nums[conn.topic]["sd_u"].append(np.sqrt(max(cv[8], 0)))
                        nums[conn.topic]["lat"].append(m.latitude)
                        nums[conn.topic]["lon"].append(m.longitude)
                        nums[conn.topic]["alt"].append(m.altitude)
                    elif mt == "nav_msgs/msg/Odometry":
                        e["child_frame_id"][m.child_frame_id] += 1
                        q = m.pose.pose.orientation
                        e["quat_is_identity_or_zero"][bool(np.allclose([q.x, q.y, q.z], 0))] += 1
                        e["pose_cov_all_zero"][bool(np.allclose(m.pose.covariance, 0))] += 1
                        p = m.pose.pose.position
                        nums[conn.topic]["x"].append(p.x)
                        nums[conn.topic]["y"].append(p.y)
                        nums[conn.topic]["z"].append(p.z)
                    elif mt == "sensor_msgs/msg/PointCloud2":
                        nums[conn.topic]["npts"].append(m.width * m.height)
                        e["fields"][",".join(f.name for f in m.fields)] += 1
            for topic, st in stamps.items():
                st = np.array(st)
                if len(st) > 1:
                    dt = np.diff(st)
                    print(f"  [{topic}] n={len(st)} span={st[-1] - st[0]:.2f} s rate={(len(st) - 1) / (st[-1] - st[0]):.3f} Hz "
                          f"dt median={np.median(dt):.4f} max={dt.max():.3f} gaps>2*median={(dt > 2 * np.median(dt)).sum()}")
                for k, cnt in extra[topic].items():
                    print(f"     {k}: {dict(cnt)}")
                for k, v in nums[topic].items():
                    v = np.array(v)
                    if k == "hdr":
                        print(f"     header stamp span {v.min():.3f} .. {v.max():.3f} ({v.max() - v.min():.2f} s); "
                              f"hdr-bag median {np.median(v - st):.4f} s")
                    else:
                        print(f"     {k}: min {v.min():.6f} median {np.median(v):.6f} max {v.max():.6f}")


if __name__ == "__main__":
    main(sys.argv[1:])
