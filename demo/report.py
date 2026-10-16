#!/usr/bin/env python3
"""Demo report (demo/run_demo.sh, step 6).

Reads what scripts/evaluate_runs.py wrote for the demo runs (<out>/eval/*.csv,
<out>/runs/<method>/<seq>/D/rep1/) and writes
  <out>/results.csv               one row per method: demo values and the paper's
  <out>/results.md                the same as a readable summary, with notes
  <out>/trajectories_<seq>.png/.pdf  top view, one panel per method over the ground truth
The accuracy figure is the paper's headline: configuration D, headline output
of each method (eval/config/outputs.yaml), alignment A (rigid, whole run), 3D
position RMSE with the LiDAR-to-antenna lever arm compensated. The paper's
values (demo/paper_<seq>.csv, from demo/make_paper_reference.py) are mean, SD
and range over its three runs on the benchmark laptop.

Usage: report.py --out demo_output [--seq cf_6] [--methods "hdl lego ..."]
       [--elapsed-s N] [--duration S]
"""
import argparse
import importlib.util
import json
import os
import platform
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R / "eval"))
from slameval import report, style  # noqa: E402
from slameval.report import fmt  # noqa: E402

LABEL = style.LABEL


def machine():
    info = {"logical_cpus": os.cpu_count(), "kernel": platform.release()}
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                info["cpu"] = line.split(":", 1)[1].strip()
                break
        mem = {l.split(":")[0]: int(l.split()[1]) for l in Path("/proc/meminfo").read_text().splitlines()}
        info["memory_gib"] = round(mem["MemTotal"] / 2 ** 20, 1)
        for line in Path("/etc/os-release").read_text().splitlines():
            if line.startswith("PRETTY_NAME="):
                info["os"] = line.split("=", 1)[1].strip('"')
    except (OSError, KeyError, ValueError):
        pass
    try:
        info["docker"] = subprocess.run(["docker", "version", "--format", "{{.Server.Version}}"],
                                        capture_output=True, text=True, timeout=20).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        pass
    return info


def num(x):
    try:
        x = float(x)
    except (TypeError, ValueError):
        return float("nan")
    return x


def compare(x, ref):
    """Plain statement of where the demo value lies against the paper's runs."""
    if ref is None:
        return "no paper value"
    if not np.isfinite(x):
        return "no demo value"
    lo, hi = ref["rmse_min_m"], ref["rmse_max_m"]
    tol = 0.0005   # the table shows millimetres; the paper values are stored to 0.1 mm
    if lo - tol <= x <= hi + tol:
        return f"inside the range of the paper's {int(ref['n_runs'])} runs"
    d = x - hi if x > hi else lo - x
    return f"{'above' if x > hi else 'below'} the paper's range by {d:.3f} m"


def load_make_figures():
    spec = importlib.util.spec_from_file_location("_make_figures", R / "scripts" / "make_figures.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="demo output folder (runs/, eval/)")
    ap.add_argument("--seq", default="cf_6")
    ap.add_argument("--methods", default=" ".join(report.METHODS))
    ap.add_argument("--reference", help="paper values (default demo/paper_<seq>.csv)")
    ap.add_argument("--elapsed-s", type=float, help="total demo time, for the summary")
    ap.add_argument("--duration", type=float, help="seconds played, if not the whole bag")
    args = ap.parse_args()
    out = Path(args.out)
    seq = args.seq
    methods = [m for m in report.METHODS if m in args.methods.split()]
    ctx = report.Context(results_dir=out / "eval", runs_dir=out / "runs")
    has_gt = ctx.gt_file(seq).exists()
    ref_file = Path(args.reference) if args.reference else R / "demo" / f"paper_{seq}.csv"
    refs = {r["method"]: r for r in pd.read_csv(ref_file).to_dict("records")} if ref_file.exists() else {}
    diag = ctx.table("run_diagnostics")
    res = ctx.table("resources_summary")

    rows = []
    for m in methods:
        hl = ctx.headline(m)
        run = out / "runs" / m / seq / "D" / "rep1"
        row = {"method": m, "label": LABEL[m], "output": hl, "seq": seq}
        man_f = run / "manifest.json"
        if not man_f.exists():
            row["run_status"] = "not run"
            rows.append(row)
            continue
        man = json.loads(man_f.read_text())
        row["run_status"] = man.get("status")
        row["run_time_s"] = round(man["end"] - man["start"], 1) if man.get("end") and man.get("start") else np.nan
        row["playback_max_lag_s"] = (man.get("playback_quality") or {}).get("max_lag_s")
        row["image"] = (man.get("image") or {}).get("name")
        d = diag[(diag["method"] == m) & (diag["seq"] == seq) & (diag["output"] == hl) & (diag["rep"] == 1)] \
            if diag is not None else pd.DataFrame()
        if len(d):
            d = d.iloc[0]
            row["eval_status"] = d.get("status")
            for k in ("n_poses", "coverage_time", "diverged"):
                if k in d:
                    row[k] = d[k]
        a = ctx.accuracy(m, seq, "D", hl)
        row["rmse_3d_m"] = a.values.get(1, np.nan)
        row["lever_arm"] = a.lever
        if res is not None:
            rr = res[(res["method"] == m) & (res["seq"] == seq) & (res["rep"] == 1)]
            if len(rr):
                rr = rr.iloc[0]
                for col, key in ((ctx.realtime_column(m), "processed_scan_ratio"),
                                 (ctx.realtime_column(m, "latency_median_s"), "latency_median_s"),
                                 ("cpu_mean_pct_one_core", "cpu_mean_pct_one_core"),
                                 ("rss_peak_gb", "rss_peak_gb"), ("n_input_scans", "n_input_scans")):
                    if col and col in rr:
                        row[key] = num(rr[col])
        ref = refs.get(m)
        if ref:
            row.update({"paper_rmse_mean_m": ref["rmse_mean_m"], "paper_rmse_sd_m": ref["rmse_sd_m"],
                        "paper_rmse_min_m": ref["rmse_min_m"], "paper_rmse_max_m": ref["rmse_max_m"],
                        "paper_rmse_runs_m": ref["rmse_runs_m"],
                        "paper_processed_scan_ratio": ref["processed_scan_ratio_mean"]})
            row["diff_to_paper_mean_m"] = row["rmse_3d_m"] - ref["rmse_mean_m"]
        row["vs_paper"] = ("partial run (not comparable)" if args.duration else
                           compare(row["rmse_3d_m"], ref) if has_gt else "no ground truth")
        rows.append(row)
    df = pd.DataFrame(rows)
    df.to_csv(out / "results.csv", index=False, float_format="%.6g")

    # ------------------------------------------------------------ figure
    fig_name = f"trajectories_{seq}"
    fig_ok = False
    if has_gt:
        mf = load_make_figures()
        style.setup()
        panels = []
        for r in rows:
            m = r["method"]
            pe = ctx.pose_errors(m, seq, "D", 1, r["output"]) if np.isfinite(num(r.get("rmse_3d_m"))) else None
            ref = refs.get(m)
            ptxt = f"paper {ref['rmse_mean_m']:.3f} ± {ref['rmse_sd_m']:.3f}" if ref else ""
            if pe is None:
                panels.append((LABEL[m] + ("\n" + ptxt if ptxt else ""), [], r.get("run_status") or "N/A"))
                continue
            panels.append((f"{LABEL[m]}\ndemo {pe.rmse:.3f}\n{ptxt}",
                           [(pe.p_all[:, :2], style.COLOUR[m], "-", 0.9)], ""))
        if any(p[1] for p in panels):
            # Legend title of the figure (make_figures uses the sequence name).
            ctx.seq_label = lambda s: (f"{report.SEQ_LABEL.get(s, s)}: 3D RMSE (m) of the demo run and of "
                                       "the paper (mean ± SD of 3 runs), alignment A")
            nc = min(len(panels), 3)   # at most 3 panels per row, so the titles fit
            layout = (int(np.ceil(len(panels) / nc)), nc)
            style.save(mf._trajectory_figure(ctx, seq, panels, layout=layout), out, fig_name)
            del ctx.seq_label   # back to the class default
            fig_ok = True

    # ------------------------------------------------------------ markdown
    mi = machine()
    L = []
    L.append(f"# Demo results: {ctx.seq_label(seq)}\n")
    L.append(f"Made on {datetime.now().strftime('%Y-%m-%d %H:%M')} by `demo/run_demo.sh`"
             + (f" in {args.elapsed_s / 60:.0f} min" if args.elapsed_s else "") + ".\n")
    L.append(f"- Machine: {mi.get('cpu', '?')}, {mi.get('logical_cpus')} logical CPUs, "
             f"{mi.get('memory_gib', '?')} GiB memory; {mi.get('os', '?')}, kernel {mi.get('kernel')}; "
             f"Docker {mi.get('docker', '?')}.")
    L.append("- Paper's machine: Intel Core i7-13620H laptop (16 logical CPUs, 15.2 GiB memory), "
             "with the method pinned to the 12 logical CPUs of the performance cores.")
    pins = set()
    for r in rows:
        mf_ = out / "runs" / r["method"] / seq / "D" / "rep1" / "manifest.json"
        if mf_.exists():
            cs = (json.loads(mf_.read_text()).get("checks") or {}).get("cpuset") or {}
            pins.add((cs.get("method"), cs.get("infra")))
    if pins:
        pin_txt = "; ".join("no CPU pinning" if p == (None, None) else
                            f"method container on CPUs {p[0] or 'any'}, infrastructure on CPUs {p[1] or 'any'}"
                            for p in sorted(pins, key=str))
        L.append(f"- CPU pinning in this demo: {pin_txt}.")
    played = f"the first {args.duration:g} s" if args.duration else "the whole bag (302 s)"
    L.append(f"- Each method ran once, configuration D, LiDAR topic only, real-time playback of {played}.")
    if has_gt:
        L.append("- Accuracy as in the paper: headline output of each method, rigid alignment over the whole "
                 "run (alignment A), 3D position RMSE against the RTK ground truth, LiDAR-to-antenna lever "
                 "arm compensated. Paper values: mean ± SD and range over its 3 runs.")
    L.append("")
    if not has_gt:
        L.append(f"No ground truth for `{seq}` (`gt/{seq}/gt_tum.txt`): trajectories only, in "
                 f"`runs/<method>/{seq}/D/rep1/<output>.tum`.\n")
    L.append("| Method | Demo RMSE (m) | Paper RMSE (m), mean ± SD | Paper runs (m) | Demo vs paper | "
             "Processed-scan ratio, demo (paper) | Run |")
    L.append("|---|---|---|---|---|---|---|")
    for r in rows:
        ref = refs.get(r["method"])
        rm = num(r.get("rmse_3d_m"))
        demo = fmt(rm, 3) if np.isfinite(rm) else "N/A"
        if ref:
            paper = f"{ref['rmse_mean_m']:.3f} ± {ref['rmse_sd_m']:.3f}"
            runs = ", ".join(f"{float(v):.3f}" for v in str(ref["rmse_runs_m"]).split())
            pp = f"{ref['processed_scan_ratio_mean']:.3f}"
        else:
            paper = runs = pp = "N/A"
        psr = num(r.get("processed_scan_ratio"))
        L.append(f"| {r['label']} | {demo} | {paper} | {runs} | {r.get('vs_paper', '')} | "
                 f"{psr:.3f} ({pp}) | {r.get('run_status')} |" if np.isfinite(psr) else
                 f"| {r['label']} | {demo} | {paper} | {runs} | {r.get('vs_paper', '')} | N/A ({pp}) | "
                 f"{r.get('run_status')} |")
    L.append("")
    if fig_ok:
        L.append(f"![Top view of the trajectories]({fig_name}.png)\n")
        L.append(f"`{fig_name}.png` / `.pdf`: top view, one panel per method (aligned with alignment A) "
                 "over the ground truth (grey).\n")
    if has_gt and refs:
        L += how_to_read(refs)
    L.append("- Data: CitrusFarm dataset, H. Teng, Y. Wang, X. Song and K. Karydis (2023), "
             "https://ucr-robotics.github.io/Citrus-Farm-Dataset/, CC BY-SA 4.0. Ground truth: "
             "`gt/README.md`.\n" if ctx.seqs.get(seq, {}).get("dataset") == "citrusfarm" else "")
    L.append("Per-run details: `runs/<method>/<seq>/D/rep1/manifest.json`; evaluation tables: `eval/`; "
             "logs: `logs/`. All values of this table: `results.csv`.")
    (out / "results.md").write_text("\n".join(L) + "\n")

    show = ["label", "run_status", "rmse_3d_m", "paper_rmse_mean_m", "paper_rmse_sd_m", "vs_paper",
            "processed_scan_ratio", "run_time_s"]
    with pd.option_context("display.width", 200, "display.max_columns", 20):
        print(df[[c for c in show if c in df.columns]].to_string(index=False))
    print(f"-> {out / 'results.csv'}, {out / 'results.md'}" + (f", {out / fig_name}.png/.pdf" if fig_ok else ""))
    for n in sorted(ctx.notes):
        print(f"note: {n}")


def how_to_read(refs):
    """Notes under the table; the examples of run-to-run spread come from the paper's values."""
    wide = [r for r in refs.values() if r["rmse_max_m"] - r["rmse_min_m"] >= 0.05]
    narrow = [r for r in refs.values() if r["rmse_max_m"] - r["rmse_min_m"] < 0.001]
    runs = lambda r: ", ".join(f"{float(v):.2f}" for v in str(r["rmse_runs_m"]).split())  # noqa: E731
    L = ["## How to read these numbers\n"]
    L.append("- **Real-time results depend on the machine.** The bag is played at its recorded rate and "
             "each method processes the scans as they arrive. A slower or busier machine makes a method "
             "skip or delay scans (see the processed-scan ratio), which can change its trajectory. Other "
             "programs running during the demo have the same effect.")
    if "dlo" in refs:
        L.append("- **CPU pinning matters for DLO.** Its NanoGICP registration uses one OpenMP thread per "
                 "CPU of the container, and its sums depend on the number of threads. Without pinning, "
                 "DLO runs with all logical CPUs (16 on the paper's laptop instead of 12). To pin the "
                 "containers as in the paper on a similar hybrid CPU: "
                 "`make demo CPUSET_METHOD=0-11 CPUSET_INFRA=12-15`.")
    text = ("- **Run-to-run spread is expected.** Several methods are not deterministic under real-time "
            "playback. In the paper, the three runs on the same machine gave "
            + "; ".join(f"{r['label']} {runs(r)} m" for r in wide)
            + ". Differences within that spread are expected, and a demo value inside it agrees with "
            "the paper.")
    if narrow:
        text += (" " + " and ".join(r["label"] for r in narrow) + " gave the same value to within "
                 "1 mm in all three runs; their range is therefore very narrow, and on another machine "
                 "a value about a centimetre outside it is still consistent with the paper.")
    L.append(text)
    L.append("- The paper reports three runs per method on four CitrusFarm sequences and one Pullally "
             "sequence; the demo is one run on one sequence. It shows that the pipeline works on your "
             "machine; it does not replace the paper's evaluation.")
    return L


if __name__ == "__main__":
    main()
