#!/usr/bin/env python3
"""Write demo/paper_<seq>.csv: the paper's values for one sequence, which
demo/report.py compares the demo run with.

Reads the aggregated results of the paper (eval/results_long.csv and
eval/resources_summary.csv of the Zenodo package, DOI 10.5281/zenodo.23405930)
and keeps, per method: the headline output (eval/config/outputs.yaml),
configuration D, alignment A, 3D RMSE with the main lever-arm setting, for each
repetition, with mean, sample SD, minimum and maximum; and the processed-scan
ratio of the headline topic.

Usage: make_paper_reference.py --results-dir <zenodo package>/eval [--seq cf_6]
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R / "eval"))
from slameval import report  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results-dir", required=True, help="eval/ folder of the Zenodo package")
    ap.add_argument("--seq", default="cf_6")
    ap.add_argument("--out", help="default: demo/paper_<seq>.csv")
    args = ap.parse_args()
    ctx = report.Context(results_dir=args.results_dir)
    rows = []
    for m in report.METHODS:
        hl = ctx.headline(m)
        a = ctx.accuracy(m, args.seq, "D", hl)
        if not a.n_ok:
            continue
        vals = [a.values[r] for r in sorted(a.values)]
        rt = ctx.realtime_column(m)
        psr = ctx.resource(m, args.seq, "D", rt) if rt else report.Agg()
        rows.append({
            "method": m, "label": report.style.LABEL[m], "output": hl, "lever_arm": a.lever,
            "n_runs": a.n_ok, "rmse_runs_m": " ".join(f"{v:.4f}" for v in vals),
            "rmse_mean_m": round(a.mean, 4), "rmse_sd_m": round(a.sd, 4) if a.n_ok > 1 else np.nan,
            "rmse_min_m": round(min(vals), 4), "rmse_max_m": round(max(vals), 4),
            "realtime_topic": rt.split(":")[0] if rt else "",
            "processed_scan_ratio_mean": round(psr.mean, 4) if psr.n_ok else np.nan,
        })
    out = Path(args.out or R / "demo" / f"paper_{args.seq}.csv")
    pd.DataFrame(rows).to_csv(out, index=False)
    print(f"{len(rows)} methods -> {out}")


if __name__ == "__main__":
    main()
