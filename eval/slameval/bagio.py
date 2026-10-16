"""Read ROS 1 bags without ROS (rosbags), using message header stamps."""
from collections import defaultdict
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

from .trajectory import Trajectory

TYPESTORE = get_typestore(Stores.ROS1_NOETIC)


def _reader(bags):
    return AnyReader([Path(b) for b in bags], default_typestore=TYPESTORE)


def _stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _pose_fields(msg, msgtype):
    """(position, orientation, child_frame_id) for the supported pose types."""
    if msgtype == "nav_msgs/msg/Odometry":
        return msg.pose.pose.position, msg.pose.pose.orientation, msg.child_frame_id
    if msgtype == "geometry_msgs/msg/PoseStamped":
        return msg.pose.position, msg.pose.orientation, ""
    if msgtype == "geometry_msgs/msg/PoseWithCovarianceStamped":
        return msg.pose.pose.position, msg.pose.pose.orientation, ""
    if msgtype == "geometry_msgs/msg/TransformStamped":
        return msg.transform.translation, msg.transform.rotation, msg.child_frame_id
    raise TypeError(f"unsupported pose message type {msgtype}")


def read_poses(bags, topic):
    """All poses on `topic`, stamped with header.stamp (not recording time).
    Returns (Trajectory, info) where info has the message type, frame ids and
    the median delay between header stamp and recording time."""
    t, p, q, rec = [], [], [], []
    frames, children = set(), set()
    msgtype = None
    with _reader(bags) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise KeyError(f"topic {topic} not in {bags}")
        for conn, t_rec, raw in reader.messages(connections=conns):
            msgtype = conn.msgtype
            msg = reader.deserialize(raw, conn.msgtype)
            pos, ori, child = _pose_fields(msg, conn.msgtype)
            t.append(_stamp(msg.header))
            rec.append(t_rec * 1e-9)
            p.append((pos.x, pos.y, pos.z))
            q.append((ori.x, ori.y, ori.z, ori.w))
            frames.add(msg.header.frame_id)
            children.add(child)
    t, rec = np.asarray(t), np.asarray(rec)
    info = {"topic": topic, "msgtype": msgtype, "count": len(t),
            "frame_ids": sorted(frames), "child_frame_ids": sorted(children - {""}),
            "median_record_minus_stamp_s": float(np.median(rec - t)) if len(t) else float("nan"),
            "record_times": rec, "raw_stamps": t}
    return Trajectory(t, np.asarray(p).reshape(-1, 3), np.asarray(q).reshape(-1, 4)), info


def read_navsatfix(bags, topic):
    """NavSatFix fields as arrays: t, lat, lon, alt, status, service,
    covariance (N, 9) and covariance type."""
    cols = defaultdict(list)
    with _reader(bags) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise KeyError(f"topic {topic} not in {bags}")
        for conn, t_rec, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            cols["t"].append(_stamp(m.header))
            cols["t_rec"].append(t_rec * 1e-9)
            cols["lat"].append(m.latitude)
            cols["lon"].append(m.longitude)
            cols["alt"].append(m.altitude)
            cols["status"].append(m.status.status)
            cols["service"].append(m.status.service)
            cols["cov"].append(np.asarray(m.position_covariance, dtype=float))
            cols["cov_type"].append(m.position_covariance_type)
            cols["frame_id"].append(m.header.frame_id)
    return {k: np.asarray(v) for k, v in cols.items()}


def topic_summary(bags):
    """rosbag-info-like summary of one or more ROS 1 bags, from the bag index
    only (message data are not read): per topic type, count, first and last
    recording time and median rate, plus overall start, end and duration."""
    from rosbags.rosbag1 import Reader as Bag1Reader

    times, types = defaultdict(list), {}
    start, end, total = np.inf, -np.inf, 0
    for b in bags:
        with Bag1Reader(Path(b)) as reader:
            for conn in reader.connections:
                ts = [ix.time for ix in reader.indexes.get(conn.id, [])]
                times[conn.topic].extend(ts)
                types[conn.topic] = conn.msgtype
                total += len(ts)
            start = min(start, reader.start_time)
            end = max(end, reader.end_time)
    out = {}
    for topic, ts in sorted(times.items()):
        ts = np.sort(np.asarray(ts, dtype=np.int64)) * 1e-9
        dt = np.diff(ts)
        med = float(np.median(dt)) if len(dt) else 0.0
        out[topic] = {"type": types[topic], "count": len(ts),
                      "start": float(ts.min()) if len(ts) else None,
                      "end": float(ts.max()) if len(ts) else None,
                      "median_rate_hz": 1.0 / med if med > 0 else None}
    out["_bag"] = {"files": [str(b) for b in bags], "start": start * 1e-9, "end": end * 1e-9,
                   "duration_s": (end - start) * 1e-9, "message_count": total}
    return out
