#!/usr/bin/env python3
"""Evaluate every run under runs/ (brief Sections 10-12).

For each run directory with a manifest: convert each declared output
(configs/methods.yaml, eval_outputs) to the LiDAR pose, write <output>.tum next
to the raw data, evaluate against gt/<seq>/gt_tum.txt and collect
  eval/results_long.csv      accuracy, long format (Section 15 columns)
  eval/segments.csv          segment drift (Section 11.5)
  eval/heading.csv           heading error (Section 11.6)
  eval/run_diagnostics.csv   association counts, coverage, divergence, time offset
  eval/resources_summary.csv CPU, memory, power and real-time behaviour
Failed runs and missing outputs are listed with their status, never dropped.

Trajectory source (--source, public release): "bag" converts the raw recording
(outputs.bag, hdl_dump/) and writes <output>.tum, as in the measured campaign;
"tum" reads the <output>.tum files that the campaign's evaluation wrote (the
public package ships these instead of the raw recordings) and writes nothing
into the run folders; "auto" (default) uses the raw recording when a run has
outputs.bag and the .tum files otherwise. The .tum files hold the LiDAR pose
after frame conversion and de-duplication, so both sources give the same
results up to the text precision of the TUM files (1 um in position).

Public demo (demo/run_demo.sh): --runs/--out point to the demo's folders, and
--n-cpu gives the number of logical CPUs of the machine that made the runs
(default: env/machine.json, the benchmark laptop, if present, else this
machine). A sequence without gt/<seq>/gt_tum.txt (your own bag) is not scored:
each output is still converted to <output>.tum and listed in
run_diagnostics.csv with status no_ground_truth.
"""
import argparse
import hashlib
import json
import os
import sys
from pathlib import Path

# One maths thread per worker process: with --jobs, multithreaded BLAS in every
# worker oversubscribes the CPU (load > 100 on 16 threads) and runs far slower.
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import numpy as np
import pandas as pd
import yaml

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R / "eval"))
from slameval import outputs, resources  # noqa: E402
from slameval.evaluate import evaluate_output, prepare_ground_truth  # noqa: E402
from slameval.trajectory import read_tum, write_tum  # noqa: E402


def file_hash(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()[:16]


def baseline_power(seq):
    """Mean package power of the infrastructure-only baseline (ii) for seq,
    over the window in which LiDAR messages were played."""
    seqs = yaml.safe_load((R / "datasets" / "sequences.yaml").read_text())["sequences"]
    vals = []
    for man in sorted((R / "baselines" / seq / "infra").glob("rep[0-9]/manifest.json")):
        if json.loads(man.read_text()).get("status") != "ok":
            continue
        s = resources.summarise_run(man.parent, 1, seqs[seq]["lidar_topic"], [])
        if np.isfinite(s.get("power_mean_w", np.nan)):
            vals.append(s["power_mean_w"])
    return float(np.mean(vals)) if vals else None


_CTX = {}
_N_CPU = None   # --n-cpu (set in each worker by _init_worker)


def _init_worker(n_cpu):
    global _N_CPU
    _N_CPU = n_cpu


def _context():
    """Configuration shared by every run (loaded once per worker process)."""
    if not _CTX:
        _CTX["cfg"] = yaml.safe_load((R / "eval" / "config" / "eval.yaml").read_text())
        _CTX["methods"] = yaml.safe_load((R / "configs" / "methods.yaml").read_text())["methods"]
        _CTX["seqs"] = yaml.safe_load((R / "datasets" / "sequences.yaml").read_text())["sequences"]
        machine = R / "env" / "machine.json"
        _CTX["n_cpu"] = _N_CPU or (int(json.loads(machine.read_text())["cpu"]["CPU(s)"]) if machine.exists()
                                   else os.cpu_count())
        _CTX["cfg_hash"] = file_hash(R / "eval" / "config" / "eval.yaml")
        _CTX["gts"], _CTX["base_pw"] = {}, {}
    return _CTX


def evaluate_run(man_file, do_time_offset=True, source="auto"):
    """All outputs of one run: (rows, segments, heading, diagnostics, resource row)."""
    ctx = _context()
    cfg, methods, seqs = ctx["cfg"], ctx["methods"], ctx["seqs"]
    run = Path(man_file).parent
    if source == "auto":
        source = "bag" if (run / "outputs.bag").exists() else "tum"
    man = json.loads(Path(man_file).read_text())
    method, seq, config, rep = man["method"], man["seq"], man["config"], man["rep"]
    mcfg, scfg = methods[method], seqs[seq]
    if seq not in ctx["gts"]:
        gt_file = R / "gt" / seq / "gt_tum.txt"
        ctx["gts"][seq] = prepare_ground_truth(read_tum(gt_file), scfg["gt_nominal_period_s"], cfg) \
            if gt_file.exists() else None
        ctx["base_pw"][seq] = baseline_power(seq)
    gt = ctx["gts"][seq]
    rows, segs, heads, diags = [], [], [], []
    meta_base = {"method": method, "seq": seq, "config": config, "rep": rep}
    topics = [t for g in ("fe", "full") for t in mcfg["outputs"].get(g, [])]
    s = resources.summarise_run(run, ctx["n_cpu"], scfg["lidar_topic"], topics,
                                baseline_power_w=ctx["base_pw"][seq])
    rt = s.pop("realtime", {})
    flat = {**meta_base, **s, "n_input_scans": rt.get("n_input_scans")}
    for topic, v in rt.items():
        if isinstance(v, dict):
            for k, x in v.items():
                flat[f"{topic}:{k}"] = x
    for name, spec in mcfg["eval_outputs"].items():
        meta = {**meta_base, "output": name}
        d = {**meta, "group": spec.get("group"), "run_status": man.get("status"),
             "eval_config_hash": ctx["cfg_hash"]}
        if man.get("status") != "ok":
            diags.append({**d, "status": "run_failed"})
            continue
        if source == "tum":
            tum = run / f"{name}.tum"
            if not tum.exists():
                diags.append({**d, "status": "no_output", "error": f"{tum.name} not found"})
                continue
            traj = read_tum(tum)
        else:
            try:
                traj, info = outputs.load_output(run, spec)
            except (KeyError, FileNotFoundError) as exc:
                diags.append({**d, "status": "no_output", "error": str(exc)})
                continue
            write_tum(run / f"{name}.tum", traj, header=f"{method} {seq} {config} rep{rep} {name}: "
                      "LiDAR pose in the method's world frame, header stamps")
        if gt is None:   # no ground truth for this sequence: trajectory only
            diags.append({**d, "status": "no_ground_truth", "n_samples": len(traj)})
            continue
        res = evaluate_output(traj, gt, cfg, meta,
                              lever_arm=scfg.get("lever_arm_lidar_to_antenna_m"),
                              forward_axis=scfg.get("lidar_forward_axis"),   # None: no heading error
                              do_time_offset=do_time_offset)
        rows += res.rows
        segs += res.segments
        heads += res.heading
        diag = {**d, **{k: v for k, v in res.diag.items() if k not in meta}}
        diag["coverage_ok"] = diag.get("coverage_time", 0) >= cfg["coverage"]["min_fraction"]
        diag["diverged"] = diag.get("n_jumps", 0) > 0 or diag.get("n_nonfinite", 0) > 0
        diags.append(diag)
    return rows, segs, heads, diags, flat


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", default=str(R / "runs"))
    ap.add_argument("--out", default=str(R / "eval"))
    ap.add_argument("--no-time-offset", action="store_true", help="skip the time-offset scan (faster)")
    ap.add_argument("--jobs", type=int, default=1, help="parallel worker processes")
    ap.add_argument("--source", choices=["auto", "bag", "tum"], default="auto",
                    help="trajectory source (see above; default: auto)")
    ap.add_argument("--n-cpu", type=int, help="logical CPUs of the machine that made the runs "
                    "(default: env/machine.json if present, else this machine)")
    args = ap.parse_args()
    _init_worker(args.n_cpu)

    man_files = [m for m in sorted(Path(args.runs).glob("*/*/*/rep*/manifest.json"))
                 # rep<r>_failed_<time>: superseded by a documented retry
                 if m.parent.name[3:].isdigit()]
    rows, segs, heads, diags, res_rows = [], [], [], [], []
    if args.jobs > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=args.jobs, initializer=_init_worker, initargs=(args.n_cpu,)) as ex:
            results = list(ex.map(evaluate_run, man_files, [not args.no_time_offset] * len(man_files),
                                  [args.source] * len(man_files)))
    else:
        results = [evaluate_run(m, not args.no_time_offset, args.source) for m in man_files]
    for r_rows, r_segs, r_heads, r_diags, r_res in results:   # deterministic order (sorted runs)
        rows += r_rows
        segs += r_segs
        heads += r_heads
        diags += r_diags
        res_rows.append(r_res)
    cfg = _context()["cfg"]

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    cols =["method", "seq", "config", "rep", "output", "alignment", "lever_arm", "dim", "metric",
            "value", "n_poses", "n_discarded", "path_length_m"]
    pd.DataFrame(rows, columns=cols).to_csv(out / "results_long.csv", index=False)
    pd.DataFrame(segs).to_csv(out / "segments.csv", index=False)
    pd.DataFrame(heads).to_csv(out / "heading.csv", index=False)
    pd.DataFrame(diags).to_csv(out / "run_diagnostics.csv", index=False)
    pd.DataFrame(res_rows).to_csv(out / "resources_summary.csv", index=False)
    n_bag = sum((Path(m).parent / "outputs.bag").exists() for m in man_files)
    used = args.source if args.source != "auto" else \
        f"auto: {n_bag} run(s) from outputs.bag, {len(man_files) - n_bag} from <output>.tum"
    print(f"{len(diags)} outputs evaluated from {len(res_rows)} runs -> {out} (trajectory source {used})")


if __name__ == "__main__":
    main()
