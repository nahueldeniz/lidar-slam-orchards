"""Synthetic-data test of the figure and table scripts (brief Sections 12-14).

Builds a small fake campaign in eval/tests/synthetic_results/ with the real
file layout and column names: ground truth (gt/<seq>/gt_tum.txt,
gt_quality.json), trajectories (runs/<m>/<seq>/<config>/rep<r>/<output>.tum),
and eval/results_long.csv, segments.csv, heading.csv, run_diagnostics.csv and
resources_summary.csv produced by the real slameval.evaluate_output and
slameval.resources.summarise_run, flattened as scripts/evaluate_runs.py does.
It includes failed runs, a missing output, a missing power value, a failed
sensitivity level and a sequence without heading reference. Then it runs
every figure and table function on it and checks the files and a few values.
"""
import importlib.util
import json
import shutil
from pathlib import Path

import numpy as np
import pandas as pd
import pytest
import yaml
from scipy.spatial.transform import Rotation

from slameval import report, resources
from slameval.evaluate import evaluate_output, prepare_ground_truth
from slameval.trajectory import Trajectory, write_tum

from synthetic import transform, true_trajectory

ROOT = Path(__file__).resolve().parents[2]
SYN = Path(__file__).resolve().parent / "synthetic_results"
CFG = yaml.safe_load((ROOT / "eval" / "config" / "eval.yaml").read_text())
SEQS = yaml.safe_load((ROOT / "datasets" / "sequences.yaml").read_text())["sequences"]
METHODS = report.METHODS
OUTPUTS = {"hdl": {"fe": "fe", "full": "full", "full_final": "full"}, "lego": {"fe": "fe", "full": "full"},
           "aloam": {"fe": "fe", "full": "full"}, "dlo": {"full": "full"}, "kiss": {"full": "full"},
           "mesh": {"full": "full"}}
DRIFT = {"hdl": 0.004, "lego": 0.006, "aloam": 0.01, "dlo": 0.005, "kiss": 0.008, "mesh": 0.03}
# Shapes: (row length m, rows, turn radius m, rotate 90 deg so the rows run along y)
SHAPE = {"cf_2": (35, 3, 2.0, True), "cf_3": (45, 3, 2.0, True), "cf_6": (40, 3, 2.0, False),
         "cf_7": (35, 3, 2.0, True), "cf_5": (35, 3, 2.0, False), "pl_1": (30, 3, 3.0, False)}
FAILED = {("hdl", "cf_6", "D", 3), ("mesh", "pl_1", "D", 1), ("kiss", "cf_5", "OAT_e_0.5", 1)}
NO_OUTPUT = {("aloam", "cf_2", "D", 2, "fe")}
NO_POWER = {("kiss", "cf_2", "D", 1)}
SENS = {   # same structure as configs/<m>/sensitivity.yaml
    "hdl": [("a", "downsample resolution", "m", "FE+BE", 0.1, [0.05, 0.3], "launch_arg", "downsample_resolution"),
            ("f", "max. correspondence distance", "m", "FE+BE", 2.0, [1.0, 3.0], "launch_arg",
             "reg_max_correspondence_distance"),
            ("c_fe", "keyframe delta translation (odometry)", "m", "FE+BE", 1.0, [0.5, 2.0], "launch_arg",
             "fe_keyframe_delta_trans"),
            ("c_be", "keyframe delta translation (graph)", "m", "BE", 2.0, [1.0, 4.0], "launch_arg",
             "be_keyframe_delta_trans")],
    "lego": [("b", "segment theta", "deg", "pre-processing (FE+BE)", 60, [10, 30], "constant", "SEGMENT_THETA_DEG"),
             ("g", "nearest-feature search distance (squared)", "m^2", "FE", 25, [10, 50], "constant",
              "NEAREST_FEATURE_SEARCH_SQ_DIST"),
             ("d", "edge threshold", "curvature", "FE", 0.1, [0.05, 0.2], "constant", "EDGE_THRESHOLD"),
             ("a", "mapping process interval", "s", "BE", 0.3, [0.15, 0.6], "constant", "MAPPING_PROCESS_INTERVAL")],
    "aloam": [("d", "nearby scan", "scan lines", "FE", 2.5, [1.5, 4], "constant", "NEARBY_SCAN"),
              ("e", "Huber loss (odometry)", "m", "FE", 0.1, [0.05, 0.2], "constant", "ODOM_HUBER_DELTA"),
              ("f", "Huber loss (mapping)", "m", "BE", 0.1, [0.05, 0.2], "constant", "MAPPING_HUBER_DELTA"),
              ("c", "plane resolution", "m", "BE", 0.4, [0.2, 0.8], "launch_param", "mapping_plane_resolution")],
    "dlo": [("b", "scan voxel size", "m", "FE", 0.25, [0.15, 0.5], "key", "dlo/odomNode/voxelFilter/scan/res"),
            ("c", "sub-map voxel size", "m", "FE", 0.5, [0.3, 1.0], "key", "dlo/odomNode/voxelFilter/submap/res"),
            ("d2", "keyframe rotation threshold", "deg", "FE", 45.0, [22.5, 90.0], "key", "dlo/odomNode/keyframe/threshR"),
            ("e", "sub-map keyframes (k-nearest, convex, concave)", "keyframes", "FE", 10, [5, 20], "key",
             "dlo/odomNode/submap/keyframe/knn")],
    "kiss": [("a", "initial threshold", "m", "FE", 2.0, [1.0, 4.0], "launch_arg", "initial_threshold"),
             ("c", "max. points per voxel", "points", "FE", 20, [10, 50], "launch_arg", "max_points_per_voxel"),
             ("d", "voxel size", "m", "FE", 1.0, [0.5, 2.0], "launch_arg", "voxel_size"),
             ("e", "minimum range", "m", "FE", 5.0, [2.5, 0.5], "launch_arg", "min_range")],
    "mesh": [("b", "minimum range", "m", "FE", 0.5, [1.0, 3.0], "key", "slamesher/range_min"),
             ("d", "margin for old cells", "scan steps", "FE", -1, [100, 500], "key", "slamesher/num_margin_old_cell"),
             ("e", "cell (grid) size", "m", "FE", 1.2, [1.0, 2.4], "key", "slamesher/grid"),
             ("g", "GP test points per cell side", "points", "FE", 6, [3, 10], "key", "slamesher/num_test")],
}


def _load_script(name):
    spec = importlib.util.spec_from_file_location(f"_{name}", ROOT / "scripts" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _truth(seq, rate, t_offset=0.0):
    row, n, rad, rot = SHAPE[seq]
    tr = true_trajectory(rate, t_offset=t_offset, row_length=row, n_rows=n, radius=rad)
    if rot:
        tr = transform(tr, Rotation.from_euler("z", 90, degrees=True), np.zeros(3))
    return tr


def _estimate(truth, gt, m, factor, rng):
    """Truth + drift growing with distance (more in turns) + noise + yaw error,
    expressed in an arbitrary method world frame."""
    s = np.interp(truth.t, gt.t, gt.arc.s)
    turn = np.interp(s, gt.arc.s, gt.turn.astype(float))
    k = DRIFT[m] * factor * (1 + 0.1 * rng.normal())
    phi = rng.uniform(0, 2 * np.pi)
    drift = k * s[:, None] * np.array([np.cos(phi), np.sin(phi), 0.15])
    bump = 0.15 * factor * np.convolve(turn, np.ones(20) / 20, mode="same")[:, None] * np.array([1.0, -0.5, 0.0])
    p = truth.p + drift + bump + rng.normal(0, 0.02 + 0.2 * k, truth.p.shape)
    yaw_err = Rotation.from_euler("z", rng.normal(0, np.deg2rad(1.5), len(truth.t)) + 2 * k * s / 100)
    q = (yaw_err * Rotation.from_quat(truth.q)).as_quat()
    R0 = Rotation.from_euler("zyx", rng.uniform(-180, 180, 3) * [1, 0.02, 0.02], degrees=True)
    return transform(Trajectory(truth.t, p, q), R0, rng.normal(0, 50, 3))


def _resource_template(tmp):
    """Keys of one resources_summary.csv row, from the real summarise_run on a tiny run."""
    tmp.mkdir(parents=True)
    t = 1000.0 + np.arange(0.0, 20.0, 0.1)
    lines = ["wall_time,ros_time,topic,datatype,seq,stamp,size_bytes"]
    for i, ti in enumerate(t):
        st = f"{int(ti)}.{int(round((ti % 1) * 1e9)):09d}"
        lines.append(f"{ti + 0.01:.9f},{ti:.9f},/velodyne_points,sensor_msgs/PointCloud2,{i},{st},1000")
        lines.append(f"{ti + 0.05:.9f},{ti:.9f},/est,nav_msgs/Odometry,{i},{st},700")
    (tmp / "frames.csv").write_text("\n".join(lines) + "\n")
    ts = np.arange(995.0, 1025.0, 1.0)
    res = ["wall_time,container,role,cpu_pct,cpu_usage_usec,mem_current,mem_peak,mem_anon,pids,nprocs,"
           "rss,uss,threads,sys_cpu_pct,sys_mem_used,sys_swap_used,logger_cpu_s"]
    res += [f"{x:.3f},sb_method,method,150.0,{int(k * 1.5e6)},1e9,1.2e9,8e8,10,3,5e8,4e8,12,20,8e9,0,0.1"
            for k, x in enumerate(ts)]
    (tmp / "resources.csv").write_text("\n".join(res) + "\n")
    pw = ["wall_time,ac_online,rapl_package-0_intel-rapl:0_uj,bat_power_uw"]
    pw += [f"{x:.3f},1,{k * 15_000_000},0" for k, x in enumerate(ts)]
    (tmp / "power.csv").write_text("\n".join(pw) + "\n")
    (tmp / "power.json").write_text(json.dumps({"rapl": [{"id": "intel-rapl:0", "name": "package-0",
                                                          "max": 262143328850, "readable": True}]}))
    (tmp / "manifest.json").write_text(json.dumps({"status": "ok", "playback_start": t[0], "playback_end": t[-1]}))
    s = resources.summarise_run(tmp, 16, "/velodyne_points", ["/est"], baseline_power_w=10.0)
    rt = s.pop("realtime")
    return list(s), [k for k in rt["/est"]]


def _resource_row(meta, keys, rt_keys, m, factor, rng, status="ok"):
    row = {**meta, "status": status}
    if status != "ok":
        return row
    base = {"hdl": (180, 0.8, 6), "lego": (95, 0.4, 4), "aloam": (120, 0.35, 5), "dlo": (140, 0.6, 5),
            "kiss": (60, 0.3, 3), "mesh": (250, 1.5, 9)}[m]
    cpu = base[0] * factor * (1 + 0.05 * rng.normal())
    vals = {"cpu_mean_pct_one_core": cpu, "cpu_p95_pct_one_core": cpu * 1.4, "cpu_mean_pct_all_cores": cpu / 16,
            "cpu_p95_pct_all_cores": cpu * 1.4 / 16, "rss_peak_gb": base[1] * factor * (1 + 0.03 * rng.normal()),
            "rss_mean_gb": base[1] * 0.8, "rss_final_gb": base[1] * 0.9, "uss_peak_gb": base[1] * 0.9,
            "cgroup_mem_peak_gb": base[1] * 1.1, "power_mean_w": 18 + base[2], "power_incremental_w": base[2] *
            (1 + 0.1 * rng.normal()), "energy_wh": (18 + base[2]) * 0.1, "playback_s": 380.0, "n_input_scans": 3800}
    for k in keys:
        if k != "status":
            row[k] = vals.get(k, 1.0)
    if (meta["method"], meta["seq"], meta["config"], meta["rep"]) in NO_POWER:
        row["power_mean_w"] = row["power_incremental_w"] = row["energy_wh"] = np.nan
    ratio = {"hdl": 0.99, "lego": 0.97, "aloam": 0.95, "dlo": 0.9, "kiss": 0.995, "mesh": 0.7}[m]
    topics = [report.REALTIME_TOPIC[m]] + (["/laser_odom_to_init"] if m in ("lego", "aloam") else [])
    for i, tp in enumerate(topics):
        v = {"n_output": 3700, "n_matched_stamps": 3700, "processed_scan_ratio": ratio - 0.3 * i,
             "latency_median_s": 0.02 * factor, "latency_p95_s": 0.05 * factor}
        for k in rt_keys:
            row[f"{tp}:{k}"] = v.get(k, 0.0)
    return row


def _gt_quality(seq, gt):
    q = {"seq": seq, "median_rate_hz": 1.0 / SEQS[seq]["gt_nominal_period_s"], "gaps": [],
         "path_length_3d_all_samples_m": float(gt.arc.total)}
    if SEQS[seq]["dataset"] == "citrusfarm":
        q["rtk"] = {"status": "ok", "fix_status_percent": {"GBAS fix": 100.0},
                    "sigma_enu_m": {"E": {"median": 0.07}, "N": {"median": 0.07}, "U": {"median": 0.1}},
                    "receiver_state": {"fix_mode_counts": {"FIXED_RTK": 990, "FLOAT_RTK": 10 if seq == "cf_2" else 0}},
                    "enu_consistency": {"rms_horizontal_m": 0.02 + 0.01 * int(seq[-1]) / 7,
                                        "rms_vertical_m": 0.001}}
    else:
        q["rtk"] = {"status": "ok", "fix_status_percent": {"GBAS fix": 100.0},
                    "sigma_enu_m": {"E": {"median": 0.8}, "N": {"median": 0.8}, "U": {"median": 1.6}}}
    return q


def _write_configs(d):
    rows = []
    for m, params in SENS.items():
        doc = {"method": m, "status": "LOCKED", "sequence": "cf_5", "parameters": [
            {"id": i, "name": n, "unit": u, "module": mod, "default": dflt, "levels": lv, "changes": [{kind: key}]}
            for i, n, u, mod, dflt, lv, kind, key in params]}
        (d / m).mkdir(parents=True)
        (d / m / "sensitivity.yaml").write_text(yaml.safe_dump(doc, sort_keys=False))
        rows += [f"| {m} | {n} (`{key}`) | {mod} | {dflt} | {lv} | synthetic reason {i} |"
                 for i, n, u, mod, dflt, lv, kind, key in params]
    (d / "SENSITIVITY_DESIGN.md").write_text(
        "# Synthetic design\n\n## Selected parameters\n\n| Method | Parameter (key) | Module | Default | Levels | "
        "Reason |\n|---|---|---|---|---|---|\n" + "\n".join(rows) + "\n\n## Not selected\n\n- none\n")


def _write_env(d):
    d.mkdir(parents=True)
    pins_a = {"_comment": "synthetic", "hdl": {
        "hdl_graph_slam": {"url": "https://example.org/hdl_graph_slam", "branch": "master", "sha": "95b8dce41c1066",
                           "commit_date": "2024-07-16T10:39:14+09:00", "patches": ["env/patches/hdl/0001-x.patch (logging only)"]},
        "g2o": {"source": "apt", "package": "ros-melodic-libg2o", "version": "2018.3.25"},
        "apt_pins": {"ros-melodic-geodesy": "0.5.3", "note": "ignored"},
        "build": {"verdict": "ignored"}},
        "kiss": {"kiss-icp": {"url": "https://example.org/kiss-icp", "tag": "v0.3.0", "sha": "460c6f2cc3616c",
                              "patches": []},
                 "fetchcontent_dependencies": {"sophus": {"url": "https://example.org/sophus.tar.gz",
                                                          "sha256": "0a8a2c39fed766ab"}}}}
    pins_b = {"lego": {"repository": "https://example.org/LeGO-LOAM", "branch": "master", "head_sha": "896a7a95a8bc51",
                       "head_commit_date": "2020-07-02", "patches": ["env/patches/lego/0001-y.patch"],
                       "dependency_required_by_readme": {"name": "GTSAM", "tag": "4.0.0-alpha2",
                                                         "tag_commit_sha": "c82fe1fde2fc98"}},
              "aloam": {"repository": "https://example.org/A-LOAM", "branch": "devel", "head_sha": "e51f88c12b483c",
                        "dependency_required_by_readme": {"name": "Ceres Solver", "version_stated": None,
                                                          "chosen_for_docker": "1.12.0 (build arg)",
                                                          "candidate_tags": {"1.12.0": {"tag_commit_sha": "029799d757b4"}}}}}
    (d / "src_pins_hdl_kiss.json").write_text(json.dumps(pins_a))
    (d / "src_pins_lego_aloam.json").write_text(json.dumps(pins_b))
    images = {f"slambench/{m}:{c}": {"tag": f"slambench/{m}:{c}", "returncode": 0, "labels": {"slambench.method": m},
                                     "id": f"sha256:{m}{c}0123456789abcdef"}
              for m, c in [("hdl", "melodic"), ("lego", "D"), ("lego", "OAT_b_10"), ("aloam", "D"), ("kiss", "melodic")]}
    (d / "images.json").write_text(json.dumps(images))
    (d / "machine.json").write_text(json.dumps({
        "collected_utc": "2026-10-12T00:00:00Z", "dmi": {"sys_vendor": "Synthetic", "product_name": "Laptop"},
        "cpu": {"Model name": "Synthetic CPU", "Core(s) per socket": "8", "CPU(s)": "16", "CPU max MHz": "4000"},
        "memory": {"mem_total_kb": 16 * 1024 ** 2, "swap_total_kb": 2 * 1024 ** 2},
        "storage": {"disks": [{"model": "SSD ", "size_gb": 512}]}, "os": {"pretty_name": "Linux", "kernel": "6.8"},
        "docker": {"server": "27", "cgroup_version": "2"},
        "power_settings": {"scaling_governor": "performance", "scaling_driver": "intel_pstate",
                           "energy_performance_preference": "performance", "acpi_platform_profile": "performance",
                           "intel_pstate_no_turbo": "0"}}))


def _runs():
    """(method, seq, config, rep, factor, outputs) of the synthetic campaign."""
    out = []
    for s in ("cf_2", "cf_3", "cf_6", "cf_7"):
        for m in METHODS:
            out += [(m, s, "D", r, 1.0, OUTPUTS[m]) for r in (1, 2, 3)]
        out += [("hdl", s, "D_nofloor", r, 1.3, OUTPUTS["hdl"]) for r in (1, 2, 3)]
    out += [(m, "pl_1", "D", 1, 2.0, OUTPUTS[m]) for m in METHODS]
    for m in METHODS:
        out += [(m, "cf_5", "D", r, 1.0, OUTPUTS[m]) for r in (1, 2, 3)]
        for k, (pid, *_rest, levels, _kind, _key) in enumerate(SENS[m]):
            for j, v in enumerate(levels):
                f = 0.6 + 0.25 * k + 0.5 * j
                out.append((m, "cf_5", report.oat_name(pid, v), 1, f, {"full": "full"}))
    return out


@pytest.fixture(scope="module")
def synth():
    if SYN.exists():
        shutil.rmtree(SYN)
    for sub in ("eval", "runs", "gt", "empty/eval", "empty/gt", "empty/configs", "empty/env"):
        (SYN / sub).mkdir(parents=True)
    keys, rt_keys = _resource_template(SYN / "tiny_run")
    _write_configs(SYN / "configs")
    _write_env(SYN / "env")
    gts = {}
    for s in SHAPE:
        period = SEQS[s]["gt_nominal_period_s"]
        tr = _truth(s, 1.0 / period)
        la = SEQS[s].get("lever_arm_lidar_to_antenna_m")
        p = tr.p + (Rotation.from_quat(tr.q).apply(la) if la is not None else 0.0)
        gt_traj = Trajectory(tr.t, p)
        write_tum(SYN / "gt" / s / "gt_tum.txt", gt_traj, header=f"synthetic {s}")
        gts[s] = prepare_ground_truth(gt_traj, period, CFG)
        (SYN / "gt" / s / "gt_quality.json").write_text(json.dumps(_gt_quality(s, gts[s])))
    rows, segs, heads, diags, res_rows = [], [], [], [], []
    for n, (m, s, c, r, f, outs) in enumerate(_runs()):
        rng = np.random.default_rng(n)
        meta_base = {"method": m, "seq": s, "config": c, "rep": r}
        failed = (m, s, c, r) in FAILED
        res_rows.append(_resource_row(meta_base, keys, rt_keys, m, f, rng, "failed" if failed else "ok"))
        rate = 4.0 if s == "pl_1" else 10.0
        truth = _truth(s, rate, t_offset=0.03)
        for o, group in outs.items():
            meta = {**meta_base, "output": o}
            d = {**meta, "group": group, "run_status": "failed" if failed else "ok", "eval_config_hash": "synthetic"}
            if failed:
                diags.append({**d, "status": "run_failed"})
                continue
            if (m, s, c, r, o) in NO_OUTPUT:
                diags.append({**d, "status": "no_output", "error": "topic not recorded"})
                continue
            fo = f * (1.6 if o == "fe" else 0.8 if o == "full_final" else 1.0)
            if c == "D_nofloor" and o == "fe":
                fo = 1.6
            est = _estimate(truth, gts[s], m, fo, rng)
            if o == "full_final":
                est = est.subset(np.arange(0, len(est), 10))
            write_tum(SYN / "runs" / m / s / c / f"rep{r}" / f"{o}.tum", est)
            res = evaluate_output(est, gts[s], CFG, meta, lever_arm=SEQS[s].get("lever_arm_lidar_to_antenna_m"),
                                  forward_axis=SEQS[s].get("lidar_forward_axis"), do_time_offset=False)
            rows += res.rows
            segs += res.segments
            heads += res.heading
            diag = {**d, **{k: v for k, v in res.diag.items() if k not in meta}}
            diag["coverage_ok"] = diag.get("coverage_time", 0) >= CFG["coverage"]["min_fraction"]
            diag["diverged"] = diag.get("n_jumps", 0) > 0 or diag.get("n_nonfinite", 0) > 0
            diags.append(diag)
    cols = ["method", "seq", "config", "rep", "output", "alignment", "lever_arm", "dim", "metric",
            "value", "n_poses", "n_discarded", "path_length_m"]
    ev = SYN / "eval"
    pd.DataFrame(rows, columns=cols).to_csv(ev / "results_long.csv", index=False)
    pd.DataFrame(segs).to_csv(ev / "segments.csv", index=False)
    pd.DataFrame(heads).to_csv(ev / "heading.csv", index=False)
    pd.DataFrame(diags).to_csv(ev / "run_diagnostics.csv", index=False)
    pd.DataFrame(res_rows).to_csv(ev / "resources_summary.csv", index=False)
    return {"keys": keys, "rt_keys": rt_keys}


def _args(root, out):
    return ["--results-dir", str(root / "eval"), "--runs-dir", str(root / "runs"), "--gt-dir", str(root / "gt"),
            "--configs-dir", str(root / "configs"), "--env-dir", str(root / "env"), "--out-dir", str(out)]


def test_resource_columns_used_exist(synth):
    used = {"cpu_mean_pct_one_core", "cpu_p95_pct_one_core", "rss_peak_gb", "uss_peak_gb", "power_mean_w",
            "energy_wh", "power_incremental_w"}
    assert used <= set(synth["keys"]) | {"power_incremental_w"}
    assert {"processed_scan_ratio", "latency_median_s", "latency_p95_s"} <= set(synth["rt_keys"])


def test_figures(synth):
    mf = _load_script("make_figures")
    out = SYN / "figs"
    status = mf.main(_args(SYN, out))
    assert set(status) == set(mf.FIGURES)
    assert all(v.startswith("ok") for v in status.values()), status
    names = ["F0_ground_truth", "F2_error_vs_distance", "F3_error_boxplots", "F4_rmse_matrix",
             "F4_supp_rmse_matrix_fe", "F5_rmse_components", "F6_resources", "F7_floor_ablation",
             "F8_segment_drift", "F9_ground_truth_quality", "F10_heading_error", "F11_sensitivity_rmse",
             "F11_supp_sensitivity_cpu", "F11_supp_sensitivity_rss"]
    names += [f"F1_trajectories_{s}" for s in ("cf_2", "cf_3", "cf_6", "cf_7", "pl_1")]
    names += [f"F1_supp_fe_vs_full_{s}" for s in ("cf_2", "cf_3", "cf_6", "cf_7", "pl_1")]
    for n in names:
        for ext in ("pdf", "png"):
            assert (out / f"{n}.{ext}").stat().st_size > 1000, n


def test_tables(synth):
    mt = _load_script("make_tables")
    out = SYN / "tables"
    status = mt.main(_args(SYN, out))
    assert all(v.startswith("ok") for v in status.values()), status
    for n in ("T1_ground_truth", "T2_software", "T3_parameters", "T4_accuracy", "T4_accuracy_B",
              "T5_resources", "T6_floor_ablation", "T7_machine"):
        assert (out / f"{n}.csv").exists() and (out / f"{n}.tex").exists(), n
        tex = (out / f"{n}.tex").read_text()
        assert r"\toprule" in tex and "±" not in tex and "Δ" not in tex
    t4 = pd.read_csv(out / "T4_accuracy.csv")
    hdl6 = t4[(t4["Method"] == "hdl Graph SLAM") & (t4["Sequence"] == "CitrusFarm 06") &
              (t4["Output"] == "full output (headline)")].iloc[0]
    assert hdl6["Runs"] == "2/3" and "±" in hdl6["RMSE (m)"] and "run failed (1)" in hdl6["Flags"]
    mesh_pl = t4[(t4["Method"] == "SLAMesh") & (t4["Sequence"] == "Pullally run 1")].iloc[0]
    assert mesh_pl["RMSE (m)"].startswith("failed") and mesh_pl["Lever arm"] == "without"
    lego_pl = t4[(t4["Method"] == "LeGO-LOAM") & (t4["Sequence"] == "Pullally run 1")].iloc[0]
    assert lego_pl["Runs"] == "1/1" and "±" not in lego_pl["RMSE (m)"]
    aloam_fe = t4[(t4["Method"] == "A-LOAM") & (t4["Sequence"] == "CitrusFarm 02") &
                  (t4["Output"] == "front end")].iloc[0]
    assert aloam_fe["Runs"] == "2/3" and "no output (1)" in aloam_fe["Flags"]
    assert set(t4.loc[t4["Sequence"].str.startswith("CitrusFarm"), "Lever arm"]) == {"with"}
    t3 = pd.read_csv(out / "T3_parameters.csv")
    kiss_e = t3[(t3["Method"] == "KISS-ICP") & (t3["ID"] == "e")].iloc[0]
    assert kiss_e["ΔRMSE level 2 (m)"] == "failed" and kiss_e["Reason"] == "synthetic reason e"
    assert t3["ΔRMSE level 1 (m)"].str.match(r"^[+-]\d").sum() == len(t3)
    t5 = pd.read_csv(out / "T5_resources.csv")
    kiss2 = t5[(t5["Method"] == "KISS-ICP") & (t5["Sequence"] == "CitrusFarm 02")].iloc[0]
    assert kiss2["Runs"] == "3/3" and "±" in kiss2["Power mean (W)"]   # 2 values, missing one noted
    assert t5.loc[t5["Method"] == "hdl Graph SLAM", "Processed scans (%)"].str.startswith("99.0").any()
    t6 = pd.read_csv(out / "T6_floor_ablation.csv")
    assert len(t6) == 4 * 3
    t2 = pd.read_csv(out / "T2_software.csv")
    assert {"hdl_graph_slam", "GTSAM", "Ceres Solver", "Docker image"} <= set(t2["Component"])


def test_pose_errors_match_results_long(synth):
    ctx = report.Context(results_dir=SYN / "eval", runs_dir=SYN / "runs", gt_dir=SYN / "gt")
    for m, s, o in (("lego", "cf_3", "full"), ("hdl", "pl_1", "full_final"), ("kiss", "cf_7", "full")):
        a = ctx.accuracy(m, s, "D", o)
        r = sorted(a.values)[0]
        # The .tum files store positions to 1 um, so the recomputation agrees to that
        # rounding (amplified by the extrapolation of alignment B from the first 10 m).
        assert ctx.pose_errors(m, s, "D", r, o).rmse == pytest.approx(a.values[r], abs=1e-5)
        b = ctx.accuracy(m, s, "D", o, alignment="B")
        assert ctx.pose_errors(m, s, "D", r, o, alignment="B").rmse == pytest.approx(b.values[r], rel=1e-4)


def test_everything_skips_without_inputs(synth):
    empty = SYN / "empty"
    mf, mt = _load_script("make_figures"), _load_script("make_tables")
    st = {**mf.main(_args(empty, empty / "figs")), **mt.main(_args(empty, empty / "tables"))}
    assert all(v.startswith("skipped") for v in st.values()), st
    assert not (empty / "figs").exists() and not (empty / "tables").exists()


def test_aggregation_helpers():
    a = report.Agg(values={1: 1.0, 2: 2.0, 3: 3.0}, n_total=3)
    assert a.mean == 2.0 and a.sd == pytest.approx(1.0) and a.text() == "2.00 ± 1.00"
    assert report.Agg(values={1: 0.5, 3: 0.7}, n_total=3).text() == "0.600 ± 0.141 (2 of 3)"
    assert report.Agg(values={1: 0.5}, n_total=1).text() == "0.500"
    assert report.Agg(values={}, n_total=2).text() == "failed (0 of 2)" and report.Agg().text() == "N/A"
    assert report.oat_name("c_fe", 0.5) == "OAT_c_fe_0.5" and report.oat_name("d", -1) == "OAT_d_m1"
    assert report.parse_oat("OAT_c_fe_0.5") == ("c_fe", "0.5") and report.parse_oat("OAT_d_m1") == ("d", "-1")
    assert report.parse_oat("D") is None
    p = yaml.safe_load("- {id: e, name: sub-map keyframes (k-nearest, convex, concave), unit: keyframes}")[0]
    assert report.param_name(p) == "sub-map keyframes (k-nearest, convex, concave)"
    assert report.log_bins(0.12, 7.0) == [0.1, 0.2, 0.5, 1, 2, 5, 10]
    b = report.log_bins(0.05, 300.0)
    assert b[0] <= 0.05 and b[-1] >= 300 and len(b) <= 8
    x = np.arange(10000.0)
    y = np.sin(x / 50.0)
    y[1234] = 9.0
    xd, yd = report.decimate_minmax(x, y, 1000)
    assert len(xd) <= 1002 and yd.max() == 9.0 and xd[0] == 0 and xd[-1] == 9999
