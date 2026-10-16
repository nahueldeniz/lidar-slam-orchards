"""Unit tests for the evaluation definitions (brief Section 11.8)."""
from pathlib import Path

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from slameval import align, associate, path
from slameval.evaluate import evaluate_output, prepare_ground_truth
from slameval.trajectory import Trajectory, read_tum, write_tum

from synthetic import R_TEST, T_TEST, path_length, transform, true_trajectory

CFG = yaml.safe_load((Path(__file__).resolve().parents[1] / "config" / "eval.yaml").read_text())
META = {"method": "synthetic", "seq": "lawnmower", "config": "D", "rep": 1, "output": "full"}


def gt_from(traj, period=0.1):
    return prepare_ground_truth(Trajectory(traj.t, traj.p), period, CFG)


def metric(res, alignment="A", lever="without", dim="3d", name="rmse"):
    vals = [r["value"] for r in res.rows if r["alignment"] == alignment and
            r["lever_arm"] == lever and r["dim"] == dim and r["metric"] == name]
    assert len(vals) == 1
    return vals[0]


@pytest.fixture(scope="module")
def truth10():
    return true_trajectory(10.0)


def test_alignment_recovers_rigid_transform(truth10):
    est = transform(truth10, R_TEST, T_TEST)
    al = align.fit_global(est.p, truth10.p)
    assert np.allclose(al.R, R_TEST.inv().as_matrix(), atol=1e-9)
    assert np.allclose(al.apply(est.p), truth10.p, atol=1e-9)


def test_noise_free_rmse_is_zero_with_offset_stamps(truth10):
    # Estimate sampled between ground-truth stamps: linear interpolation of a
    # piecewise-linear-in-time straight row is exact, turns add tiny chord error.
    est = transform(true_trajectory(10.0, t_offset=0.05), R_TEST, T_TEST)
    res = evaluate_output(est, gt_from(truth10), CFG, META, do_time_offset=False)
    assert metric(res) < 2e-3
    assert res.diag["n_discarded"] <= 1


def test_noise_level_recovered(truth10):
    rng = np.random.default_rng(1)
    sigma = 0.05
    est = transform(truth10, R_TEST, T_TEST)
    est = Trajectory(est.t, est.p + rng.normal(0, sigma, est.p.shape), est.q)
    res = evaluate_output(est, gt_from(truth10), CFG, META, do_time_offset=False)
    assert metric(res) == pytest.approx(sigma * np.sqrt(3), rel=0.05)
    assert metric(res, dim="2d") == pytest.approx(sigma * np.sqrt(2), rel=0.05)
    assert metric(res, dim="z", name="mean") == pytest.approx(sigma * np.sqrt(2 / np.pi), rel=0.08)


def test_time_offset_recovered(truth10):
    lag = 0.2
    est = transform(truth10, R_TEST, T_TEST)
    est = Trajectory(est.t + lag, est.p, est.q)     # estimate stamps late by 0.2 s
    res = evaluate_output(est, gt_from(truth10), CFG, META)
    assert res.diag["delta_star_s_without"] == pytest.approx(lag, abs=0.011)
    assert res.diag["rmse_at_delta_star_without"] < 0.01
    assert res.diag["rmse_at_zero_without"] > 0.05


def test_lever_arm(truth10):
    lever = np.array([-0.220, 0.0, 0.153])         # CitrusFarm LiDAR -> antenna
    antenna = truth10.p + Rotation.from_quat(truth10.q).apply(lever)
    gt = gt_from(Trajectory(truth10.t, antenna))
    est = transform(truth10, R_TEST, T_TEST)
    res = evaluate_output(est, gt, CFG, META, lever_arm=lever, do_time_offset=False)
    assert metric(res, lever="with") < 1e-6
    # Without compensation the horizontal 0.22 m offset rotates with the
    # vehicle, so the rigid alignment cannot remove it.
    assert metric(res, lever="without") > 0.1


def test_association_discards_gaps_and_outside_span(truth10):
    gt_t, gt_p = truth10.t, truth10.p
    keep = (gt_t < gt_t[0] + 50.0) | (gt_t > gt_t[0] + 50.5)  # 0.6 s hole
    est_t = np.concatenate([[gt_t[0] - 1.0], np.linspace(gt_t[0] + 10, gt_t[0] + 60, 501),
                            [gt_t[-1] + 1.0]])
    a = associate.associate(est_t, gt_t[keep], gt_p[keep], 0.15)
    assert a.n_outside_span == 2
    in_hole = (est_t > gt_t[0] + 49.95) & (est_t < gt_t[0] + 50.55)
    assert a.n_gap == int(in_hole.sum())
    assert a.n == len(est_t) - 2 - a.n_gap
    # An estimate stamp equal to a ground-truth stamp next to a gap is kept.
    edge = gt_t[keep][np.argmax(np.diff(gt_t[keep]))]
    assert associate.associate(np.array([edge]), gt_t[keep], gt_p[keep], 0.15).n == 1


def test_interpolation_linear_between_samples():
    gt_t = np.array([0.0, 1.0, 2.0])
    gt_p = np.array([[0, 0, 0], [1, 2, 3], [3, 2, 1]], dtype=float)
    a = associate.associate(np.array([0.25, 1.5]), gt_t, gt_p, 1.5)
    assert np.allclose(a.p_gt, [[0.25, 0.5, 0.75], [2.0, 2.0, 2.0]])


def test_initial_alignment_uses_first_10_m(truth10):
    est = transform(truth10, R_TEST, T_TEST)
    # Drift: 1 % lateral error that only starts after 20 m.
    s = np.interp(truth10.t, truth10.t, np.linspace(0, path_length(), len(truth10)))
    drift = np.column_stack([np.zeros(len(s)), 0.01 * np.clip(s - 20, 0, None), np.zeros(len(s))])
    est = Trajectory(est.t, est.p + R_TEST.apply(drift), est.q)
    res = evaluate_output(est, gt_from(truth10), CFG, META, do_time_offset=False)
    assert 95 <= res.diag["B_n_used_without"] <= 105            # ~10 m at 0.1 m per sample
    assert metric(res, "B", name="final") == pytest.approx(0.01 * (path_length() - 20), rel=0.05)
    assert metric(res, "A") < metric(res, "B")


def test_heading_error(truth10):
    bias = np.deg2rad(3.0)
    est = transform(truth10, R_TEST, T_TEST)
    q = (Rotation.from_quat(est.q) * Rotation.from_euler("z", bias)).as_quat()
    res = evaluate_output(Trajectory(est.t, est.p, q), gt_from(truth10), CFG, META,
                          do_time_offset=False)
    by_subset = {h["subset"]: h for h in res.heading}
    row = by_subset["row"]
    assert row["mean_deg"] == pytest.approx(3.0, abs=0.05)
    assert row["rmse_deg"] == pytest.approx(3.0, abs=0.05)
    # Over turns the 2 m chord course differs slightly from the tangent.
    assert by_subset["all"]["mean_deg"] == pytest.approx(3.0, abs=0.5)


def test_turn_labels_on_lawnmower(truth10):
    arc = path.arc_length(truth10.p, 0.1)
    tc = CFG["turn"]
    flags = path.turn_flags(arc, arc.s, tc["heading_baseline_m"], tc["window_m"], tc["threshold_deg"])
    s = arc.s
    mid_row = (s > 10) & (s < 30)
    turn_len = np.pi * 2.0
    mid_turn = (s > 40 + turn_len / 2 - 0.5) & (s < 40 + turn_len / 2 + 0.5)
    assert not flags[mid_row].any()
    assert flags[mid_turn].all()


def test_arc_length_ignores_stationary_jitter():
    rng = np.random.default_rng(2)
    line = np.column_stack([np.linspace(0, 100, 1001), np.zeros(1001), np.zeros(1001)])
    still = np.tile(line[-1], (600, 1)) + rng.normal(0, 0.01, (600, 3))
    arc = path.arc_length(np.vstack([line, still]), 0.1)
    assert arc.total == pytest.approx(100.0, abs=0.15)


def test_segments_count_and_noise_free_error(truth10):
    est = transform(truth10, R_TEST, T_TEST)
    res = evaluate_output(est, gt_from(truth10), CFG, META, do_time_offset=False)
    total = path_length()
    for L in CFG["segments"]["lengths_m"]:
        segs = [s for s in res.segments if s["L_m"] == L and s["lever_arm"] == "without"]
        assert len(segs) == int(total // L)
        ok = [s for s in segs if s["status"] == "ok"]
        assert ok and max(s["rmse_3d"] for s in ok) < 1e-6
    classes = {s["class"] for s in res.segments if s["L_m"] == 10}
    assert {"row", "turn"} <= classes or {"row", "mixed"} <= classes


def test_spline_vs_linear_small_for_dense_gt(truth10):
    rms = associate.spline_vs_linear_rms(truth10.t, truth10.p, truth10.t[:-1] + 0.05)
    assert 0 < rms < 0.01


def test_long_rows_have_required_columns(truth10):
    est = transform(truth10, R_TEST, T_TEST)
    res = evaluate_output(est, gt_from(truth10), CFG, META, do_time_offset=False)
    required = {"method", "seq", "config", "rep", "output", "alignment", "lever_arm", "dim",
                "metric", "value", "n_poses", "n_discarded", "path_length_m"}
    assert all(required <= set(r) for r in res.rows)
    assert len(res.rows) == 2 * 3 * 8          # alignments x dims x statistics


def test_tum_roundtrip(tmp_path, truth10):
    f = tmp_path / "x.tum"
    write_tum(f, truth10)
    back = read_tum(f)
    assert np.allclose(back.t, truth10.t, atol=1e-9)
    assert np.allclose(back.p, truth10.p, atol=1e-6)
    assert np.allclose(np.abs(np.sum(back.q * truth10.q, axis=1)), 1.0, atol=1e-8)


def test_duplicate_and_unsorted_stamps_handled():
    t = np.array([0.0, 2.0, 1.0, 2.0])
    tr = Trajectory(t, np.arange(12.0).reshape(4, 3))
    assert tr.n_unsorted == 1 and tr.n_duplicate_stamps == 1
    assert np.allclose(tr.t, [0.0, 1.0, 2.0])
