"""End-to-end test: synthetic run directory (ROS 1 outputs.bag, frames.csv,
resources.csv, power.csv, manifest.json) through output loading, frame
conversion, evaluation and resource summary."""
import json
from pathlib import Path

import numpy as np
import pytest
import yaml
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation

from slameval import outputs, resources
from slameval.evaluate import evaluate_output, prepare_ground_truth
from slameval.trajectory import Trajectory

from synthetic import R_TEST, T_TEST, transform, true_trajectory

TS = get_typestore(Stores.ROS1_NOETIC)
CFG = yaml.safe_load((Path(__file__).resolve().parents[1] / "config" / "eval.yaml").read_text())


def odom_msg(t, p, q, frame="odom", child="lidar"):
    T = TS.types
    sec, nsec = int(t), int(round((t - int(t)) * 1e9))
    if nsec == 1_000_000_000:
        sec, nsec = sec + 1, 0
    header = T["std_msgs/msg/Header"](seq=0, stamp=T["builtin_interfaces/msg/Time"](sec=sec, nanosec=nsec),
                                      frame_id=frame)
    pose = T["geometry_msgs/msg/Pose"](position=T["geometry_msgs/msg/Point"](x=p[0], y=p[1], z=p[2]),
                                       orientation=T["geometry_msgs/msg/Quaternion"](x=q[0], y=q[1], z=q[2], w=q[3]))
    zero3 = T["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=0.0)
    return T["nav_msgs/msg/Odometry"](
        header=header, child_frame_id=child,
        pose=T["geometry_msgs/msg/PoseWithCovariance"](pose=pose, covariance=np.zeros(36)),
        twist=T["geometry_msgs/msg/TwistWithCovariance"](
            twist=T["geometry_msgs/msg/Twist"](linear=zero3, angular=zero3), covariance=np.zeros(36)))


def write_bag(path, topics):
    with Writer(path) as w:
        for topic, traj in topics.items():
            conn = w.add_connection(topic, "nav_msgs/msg/Odometry", typestore=TS)
            for t, p, q in zip(traj.t, traj.p, traj.q):
                w.write(conn, int(t * 1e9), TS.serialize_ros1(odom_msg(t, p, q), "nav_msgs/msg/Odometry"))


@pytest.fixture(scope="module")
def run_dir(tmp_path_factory):
    d = tmp_path_factory.mktemp("run")
    truth = true_trajectory(10.0)
    # The method reports the pose of a body frame rotated 90 deg about z from
    # the LiDAR (as if it used a rotated convention): T_out = T_lidar * C^-1.
    C = Rotation.from_euler("z", 90, degrees=True)
    est = transform(truth, R_TEST, T_TEST)
    q_out = (Rotation.from_quat(est.q) * C.inv()).as_quat()
    write_bag(d / "outputs.bag", {"/est": Trajectory(est.t, est.p, q_out)})
    t0, t1 = truth.t[0], truth.t[-1]
    # frames.csv: every LiDAR stamp received at stamp+0.01 s, outputs at +0.05 s,
    # 5 % of the scans have no output.
    lines = ["wall_time,ros_time,topic,datatype,seq,stamp,size_bytes"]
    for i, t in enumerate(truth.t):
        st = f"{int(t)}.{int(round((t % 1) * 1e9)):09d}"
        lines.append(f"{t + 0.01:.9f},{t:.9f},/velodyne_points,sensor_msgs/PointCloud2,{i},{st},1000")
        if i % 20 != 0:
            lines.append(f"{t + 0.05:.9f},{t:.9f},/est,nav_msgs/Odometry,{i},{st},700")
    (d / "frames.csv").write_text("\n".join(lines) + "\n")
    ts = np.arange(t0 - 5, t1 + 5, 1.0)
    res = ["wall_time,container,role,cpu_pct,cpu_usage_usec,mem_current,mem_peak,mem_anon,pids,nprocs,"
           "rss,uss,threads,sys_cpu_pct,sys_mem_used,sys_swap_used,logger_cpu_s"]
    for k, t in enumerate(ts):
        res.append(f"{t:.3f},sb_method,method,150.0,{int(k * 1.5e6)},1e9,1.2e9,8e8,10,3,"
                   f"{5e8 + k * 1e5:.0f},{4e8:.0f},12,20,8e9,0,0.1")
    (d / "resources.csv").write_text("\n".join(res) + "\n")
    pw = ["wall_time,ac_online,rapl_package-0_intel-rapl:0_uj,bat_power_uw"]
    for k, t in enumerate(ts):
        pw.append(f"{t:.3f},1,{(k * 15_000_000) % 262143328850},0")   # 15 W
    (d / "power.csv").write_text("\n".join(pw) + "\n")
    (d / "power.json").write_text(json.dumps({"rapl": [{"id": "intel-rapl:0", "name": "package-0",
                                                        "max": 262143328850, "readable": True}]}))
    (d / "manifest.json").write_text(json.dumps({"status": "ok", "playback_start": t0, "playback_end": t1}))
    return d, truth, C


def test_output_frames_and_accuracy(run_dir):
    d, truth, C = run_dir
    spec = {"kind": "topic", "topic": "/est", "right": {"R": C.as_matrix().tolist()}}
    traj, info = outputs.load_output(d, spec)
    assert info["frame_ids"] == ["odom"] and info["child_frame_ids"] == ["lidar"]
    gt = prepare_ground_truth(Trajectory(truth.t, truth.p), 0.1, CFG)
    res = evaluate_output(traj, gt, CFG, {"method": "m", "seq": "s", "config": "D", "rep": 1,
                                          "output": "full"}, do_time_offset=False)
    rmse = [r["value"] for r in res.rows if r["alignment"] == "A" and r["dim"] == "3d" and r["metric"] == "rmse"]
    assert rmse[0] < 1e-5
    row = {h["subset"]: h for h in res.heading}["row"]
    assert abs(row["mean_deg"]) < 0.05      # frame correction restores the LiDAR heading
    # Without the right-hand correction the heading is off by 90 degrees.
    traj_raw, _ = outputs.load_output(d, {"kind": "topic", "topic": "/est"})
    res_raw = evaluate_output(traj_raw, gt, CFG, {"method": "m", "seq": "s", "config": "D", "rep": 1,
                                                  "output": "full"}, do_time_offset=False)
    assert {h["subset"]: h for h in res_raw.heading}["row"]["mean_deg"] == pytest.approx(-90, abs=0.1)


def test_tum_source_matches_bag(run_dir, tmp_path):
    """Public release: scripts/evaluate_runs.py --source tum evaluates the <output>.tum
    written from outputs.bag; it must give the same results up to the TUM text precision."""
    from slameval.trajectory import read_tum, write_tum
    d, truth, C = run_dir
    spec = {"kind": "topic", "topic": "/est", "right": {"R": C.as_matrix().tolist()}}
    traj, _ = outputs.load_output(d, spec)
    write_tum(tmp_path / "full.tum", traj)
    traj_tum = read_tum(tmp_path / "full.tum")
    assert np.allclose(traj_tum.t, traj.t, rtol=0, atol=1e-9)   # stamps written with %.9f
    gt = prepare_ground_truth(Trajectory(truth.t, truth.p), 0.1, CFG)
    meta = {"method": "m", "seq": "s", "config": "D", "rep": 1, "output": "full"}
    a = evaluate_output(traj, gt, CFG, meta, lever_arm=[-0.2, 0.0, 0.15], do_time_offset=False)
    b = evaluate_output(traj_tum, gt, CFG, meta, lever_arm=[-0.2, 0.0, 0.15], do_time_offset=False)
    assert [(r["alignment"], r["metric"]) for r in a.rows] == [(r["alignment"], r["metric"]) for r in b.rows]
    # Positions are written to 1 um. Alignment A (whole run) keeps that order of error;
    # alignment B (first 10 m only, applied to the whole run) amplifies it (Makefile, check-results).
    for al, atol in (("A", 2e-6), ("B", 2e-3)):
        va = [r["value"] for r in a.rows if r["alignment"] == al]
        vb = [r["value"] for r in b.rows if r["alignment"] == al]
        assert va and np.allclose(va, vb, rtol=1e-6, atol=atol), al
    assert a.diag["n_duplicate_stamps"] == b.diag["n_duplicate_stamps"] == 0


def test_resource_summary(run_dir):
    d, truth, _ = run_dir
    s = resources.summarise_run(d, 16, "/velodyne_points", ["/est"], baseline_power_w=10.0)
    assert s["cpu_mean_pct_one_core"] == pytest.approx(150.0)
    assert s["cpu_mean_pct_all_cores"] == pytest.approx(150.0 / 16)
    assert s["power_mean_w"] == pytest.approx(15.0, rel=1e-6)
    assert s["power_incremental_w"] == pytest.approx(5.0, rel=1e-5)
    rt = s["realtime"]["/est"]
    assert rt["processed_scan_ratio"] == pytest.approx(0.95, abs=0.001)
    assert rt["latency_median_s"] == pytest.approx(0.04, abs=1e-6)
    assert s["cpu_ms_per_scan"] == pytest.approx(1000 * 1.5 * (len(truth.t) / 10 - 0.9) / len(truth.t), rel=0.02)


def test_compose_latest():
    t = np.arange(10.0)
    odom = Trajectory(t, np.column_stack([t, np.zeros(10), np.zeros(10)]), np.tile([0, 0, 0, 1.0], (10, 1)))
    corr = Trajectory(np.array([3.5]), np.array([[0.0, 5.0, 0.0]]), np.array([[0, 0, 0, 1.0]]))
    out = outputs.compose_latest(corr, odom)
    assert np.allclose(out.p[:4, 1], 0) and np.allclose(out.p[4:, 1], 5)


def test_hdl_dump(tmp_path):
    for k in range(3):
        kd = tmp_path / f"{k:06d}"
        kd.mkdir()
        E = np.eye(4)
        E[:3, 3] = [k, 2 * k, 0]
        mat = "\n".join(" ".join(str(x) for x in row) for row in E)
        (kd / "data").write_text(f"stamp {100 + k} 500000000\nestimate\n{mat}\nodom\n{mat}\n")
    traj = outputs.read_hdl_dump(tmp_path)
    assert np.allclose(traj.t, [100.5, 101.5, 102.5])
    assert np.allclose(traj.p[:, 1], [0, 2, 4])


def test_realtime_matches_microsecond_truncated_stamps():
    import pandas as pd
    rows = []
    for i in range(10):
        st_in = f"1000.{i * 100_000_000 + 123_456_789:09d}"
        rows.append({"wall_time": 1.0 + i, "topic": "/pts", "stamp": st_in})
        if i % 2 == 0:   # output stamp truncated to microseconds, as PCL headers do
            st_out = f"1000.{(i * 100_000_000 + 123_456_789) // 1000 * 1000:09d}"
            rows.append({"wall_time": 1.03 + i, "topic": "/odom", "stamp": st_out})
    rt = resources.realtime(pd.DataFrame(rows), "/pts", ["/odom"])
    assert rt["/odom"]["n_matched_stamps"] == 5
    assert rt["/odom"]["processed_scan_ratio"] == pytest.approx(0.5)
    assert rt["/odom"]["latency_median_s"] == pytest.approx(0.03)


def test_compose_latest_by_receipt_time():
    t = np.arange(10.0)
    odom = Trajectory(t, np.column_stack([t, np.zeros(10), np.zeros(10)]), np.tile([0, 0, 0, 1.0], (10, 1)))
    # Correction stamped 3.0 (keyframe time) but received only at 6.5.
    corr = Trajectory(np.array([3.0]), np.array([[0.0, 5.0, 0.0]]), np.array([[0, 0, 0, 1.0]]))
    by_stamp = outputs.compose_latest(corr, odom)
    by_receipt = outputs.compose_latest(corr, odom, np.array([6.5]), t + 0.01)
    assert np.allclose(by_stamp.p[3:, 1], 5) and np.allclose(by_stamp.p[:3, 1], 0)
    assert np.allclose(by_receipt.p[7:, 1], 5) and np.allclose(by_receipt.p[:7, 1], 0)
