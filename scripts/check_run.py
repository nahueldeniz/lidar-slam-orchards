#!/usr/bin/env python3
"""Quick check of one or more run directories (dry runs, and the repetition-1
check of brief Section 6): status, message counts, LiDAR-only checks,
processed-scan ratio, and the RMSE of every declared output (alignment A, 3D,
with and without the lever arm). Prints only; writes nothing."""
import json
import sys
from pathlib import Path

import numpy as np
import yaml

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R / "eval"))
from slameval import outputs, resources  # noqa: E402
from slameval.evaluate import evaluate_output, prepare_ground_truth  # noqa: E402
from slameval.trajectory import read_tum  # noqa: E402


def main():
    cfg = yaml.safe_load((R / "eval" / "config" / "eval.yaml").read_text())
    methods = yaml.safe_load((R / "configs" / "methods.yaml").read_text())["methods"]
    seqs = yaml.safe_load((R / "datasets" / "sequences.yaml").read_text())["sequences"]
    for run in sys.argv[1:]:
        run = Path(run)
        man = json.loads((run / "manifest.json").read_text())
        m, s = man["method"], man["seq"]
        mc, sc = methods[m], seqs[s]
        print(f"=== {m} {s} {man['config']} rep{man['rep']}: {man['status']} {man.get('error') or ''}")
        bad = {k: v for k, v in man["checks"].items() if k.startswith("imu_gnss") and v}
        print(f"  messages {man.get('message_counts')}; IMU/GNSS publishers: {bad or 'none'}; "
              f"exit {man.get('method_exit')}")
        q = resources.playback_quality(resources.frames_table(run), sc["lidar_topic"])
        print(f"  playback: max lag {q.get('max_lag_s', float('nan')):.2f} s, bursts {q.get('n_bursts_lt20ms')}, "
              f"stalls {q.get('n_stalls')} -> {'ok' if q.get('ok') else 'FAULT'}")
        topics = [t for g in ("fe", "full") for t in mc["outputs"].get(g, [])]
        rt = resources.realtime(resources.frames_table(run), sc["lidar_topic"], topics)
        for t in topics:
            if t in rt:
                print(f"  {t}: processed {rt[t]['processed_scan_ratio']:.3f}, latency median "
                      f"{1000 * rt[t]['latency_median_s']:.1f} ms")
        gt = prepare_ground_truth(read_tum(R / "gt" / s / "gt_tum.txt"), sc["gt_nominal_period_s"], cfg)
        for name, spec in mc["eval_outputs"].items():
            try:
                traj, _ = outputs.load_output(run, spec)
            except Exception as exc:  # report and continue
                print(f"  {name}: no output ({exc})")
                continue
            res = evaluate_output(traj, gt, cfg, {"method": m, "seq": s, "config": man["config"],
                                                  "rep": man["rep"], "output": name},
                                  lever_arm=sc.get("lever_arm_lidar_to_antenna_m"),
                                  forward_axis=sc.get("lidar_forward_axis"), do_time_offset=False)
            vals = {(r["lever_arm"]): r["value"] for r in res.rows
                    if r["alignment"] == "A" and r["dim"] == "3d" and r["metric"] == "rmse"}
            d = res.diag
            print(f"  {name}: n={d.get('n_poses')} ({d.get('est_rate_hz', np.nan):.1f} Hz), path "
                  f"{d.get('path_length_m', np.nan):.0f} m, RMSE A 3D {vals}, jumps {d.get('n_jumps')}")


if __name__ == "__main__":
    main()
