#!/usr/bin/env python3
"""Automatic check of repetition 1 before the remaining repetitions start
(brief Section 6, "Order", and the acceptance checks of Section 15).

Checks every run of schedules/batch1.txt:
  - a manifest exists and its status is ok (after at most one documented retry)
  - no IMU or GNSS publisher at any of the three checks (LiDAR only)
  - playback quality ok (lag behind the bag timeline <= max_playback_lag_s)
  - the headline output exists and covers >= 95 % of the sequence, or the run
    is listed as flagged (method behaviour, not a protocol fault)
  - hdl: front-end output with and without floor detection agrees (the floor
    constraint is back-end only), within 5 % RMSE
Writes logs/check_rep1.md and exits 0 when no protocol fault was found, so the
chained launcher (scripts/launch_campaign.sh) can start batch 2. Method
failures and coverage flags are reported but do not block batch 2: they are
results, not protocol faults.
"""
import json
import sys
from pathlib import Path

import yaml

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R / "eval"))
from slameval import outputs, resources  # noqa: E402
from slameval.evaluate import evaluate_output, prepare_ground_truth  # noqa: E402
from slameval.trajectory import read_tum  # noqa: E402


def rmse_a(traj, gt, cfg, meta):
    res = evaluate_output(traj, gt, cfg, meta, do_time_offset=False)
    v = [r["value"] for r in res.rows if r["alignment"] == "A" and r["dim"] == "3d" and r["metric"] == "rmse"]
    return (v[0] if v else float("nan")), res.diag


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", default=str(R / "runs"))
    ap.add_argument("--schedule", default=str(R / "schedules" / "batch1.txt"))
    ap.add_argument("--report", default=str(R / "logs" / "check_rep1.md"))
    ap.add_argument("--dry", action="store_true", help="do not modify schedules/batch2.txt")
    args = ap.parse_args()
    cfg = yaml.safe_load((R / "eval" / "config" / "eval.yaml").read_text())
    methods = yaml.safe_load((R / "configs" / "methods.yaml").read_text())["methods"]
    seqs = yaml.safe_load((R / "datasets" / "sequences.yaml").read_text())["sequences"]
    settings = yaml.safe_load((R / "env" / "run_settings.yaml").read_text())
    headline = yaml.safe_load((R / "eval" / "config" / "outputs.yaml").read_text())["headline"]
    lines, faults, flags, redo = ["# Repetition 1 check", ""], [], [], []
    gts, fe = {}, {}
    for line in Path(args.schedule).read_text().split("\n"):
        if not line.strip():
            continue
        m, s, c, r = line.split()
        run = Path(args.runs) / m / s / c / f"rep{r}"
        tag = f"{m} {s} {c} rep{r}"
        man_file = run / "manifest.json"
        if not man_file.exists():
            redo.append(f"{m} {s} {c} {r}")
            continue
        man = json.loads(man_file.read_text())
        if man["status"] == "infra_fault":
            redo.append(f"{m} {s} {c} {r}")
            continue
        if man["status"] != "ok":
            flags.append(f"{tag}: status {man['status']} {man.get('error') or ''}")
            continue
        imu = {k: v for k, v in man["checks"].items() if k.startswith("imu_gnss") and v}
        if imu:
            faults.append(f"{tag}: IMU/GNSS publishers {imu}")
        q = man.get("playback_quality", {})
        if q.get("max_lag_s", 99) > settings.get("max_playback_lag_s", 0.5):
            faults.append(f"{tag}: playback lag {q.get('max_lag_s')}")
        if s not in gts:
            gts[s] = prepare_ground_truth(read_tum(R / "gt" / s / "gt_tum.txt"),
                                          seqs[s]["gt_nominal_period_s"], cfg)
        meta = {"method": m, "seq": s, "config": c, "rep": int(r), "output": headline[m]}
        try:
            traj, _ = outputs.load_output(run, methods[m]["eval_outputs"][headline[m]])
            rm, diag = rmse_a(traj, gts[s], cfg, meta)
            cov = diag.get("coverage_time", 0.0)
            note = "" if cov >= cfg["coverage"]["min_fraction"] else "  FLAG: coverage < 95 %"
            if note:
                flags.append(f"{tag}: coverage {cov:.2f}")
            lines.append(f"- {tag}: RMSE A 3D {rm:.3f} m, coverage {cov:.2f}, playback lag "
                         f"{q.get('max_lag_s', float('nan')):.2f} s{note}")
        except Exception as exc:
            flags.append(f"{tag}: headline output missing ({exc})")
            continue
        if m == "hdl":
            t, _ = outputs.load_output(run, methods[m]["eval_outputs"]["fe"])
            fe[(s, c)] = rmse_a(t, gts[s], cfg, {**meta, "output": "fe"})[0]
    for s in {k[0] for k in fe}:
        a, b = fe.get((s, "D")), fe.get((s, "D_nofloor"))
        if a is not None and b is not None:
            ok = abs(a - b) <= 0.05 * max(a, b)
            lines.append(f"- hdl front end {s}: D {a:.3f} m vs D_nofloor {b:.3f} m -> "
                         f"{'consistent' if ok else 'DIFFERENT (check)'}")
            if not ok:
                flags.append(f"hdl front end differs with/without floor detection on {s}")
    if redo and not args.dry:
        # Runs with a final infrastructure fault (or no manifest) are redone in
        # batch 2 from the internal-disk copy of their sequence.
        rf = R / "logs" / "redo_rep1.txt"
        rf.write_text("".join(x + "\n" for x in redo))
        import subprocess
        res = subprocess.run([sys.executable, str(R / "scripts" / "make_schedule.py"), "batch2",
                              "--redo-file", str(rf)], capture_output=True, text=True)
        if res.returncode != 0:
            faults.append(f"could not add the redo runs to batch 2: {res.stderr.strip()[-300:]}")
    lines += ["", "## Re-queued into batch 2 (infrastructure faults, redone from the internal copy)", ""] + \
             [f"- {x}" for x in redo or ["none"]]
    lines += ["", "## Protocol faults (block batch 2)", ""] + [f"- {x}" for x in faults or ["none"]]
    lines += ["", "## Flags (reported, do not block)", ""] + [f"- {x}" for x in flags or ["none"]]
    Path(args.report).parent.mkdir(parents=True, exist_ok=True)
    Path(args.report).write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 1 if faults else 0


if __name__ == "__main__":
    sys.exit(main())
