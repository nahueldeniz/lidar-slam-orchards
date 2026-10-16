#!/usr/bin/env python3
"""Run a schedule file sequentially (one run at a time, cool-down included).

Start it detached and leave the machine idle, e.g.
  tmux new -d -s slambench '.venv/bin/python scripts/run_batch.py schedules/test_rep1.txt --phase test'
Runs that already have a manifest are skipped, so an interrupted batch can be
restarted with the same command. A failed run is logged and the batch goes on.
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

R = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("schedule")
    ap.add_argument("--phase", choices=["tuning", "test"], required=True)
    args = ap.parse_args()
    all_lines = [l.split() for l in Path(args.schedule).read_text().splitlines() if l.strip()]
    lines = [l for l in all_lines if l[0] not in ("stage", "unstage")]   # runs, baselines and redos
    log = (R / "logs")
    log.mkdir(exist_ok=True)
    log_file = log / f"batch_{Path(args.schedule).stem}_{time.strftime('%Y%m%dT%H%M%S')}.log"
    failed = []
    with open(log_file, "w") as lf:
        i = 0
        for line in all_lines:
            if line[0] in ("stage", "unstage"):
                # Copy sequences to (or delete them from) the internal disk, between runs only.
                extra = ["--remove"] if line[0] == "unstage" else []
                lf.write(f"[{time.strftime('%F %T')}] {line[0]} {' '.join(line[1:])}\n")
                lf.flush()
                res = subprocess.run([sys.executable, str(R / "scripts" / "stage_local.py")] + extra + line[1:],
                                     stdout=lf, stderr=subprocess.STDOUT)
                lf.write(f"[{time.strftime('%F %T')}] {line[0]} finished (rc={res.returncode}); "
                         "sequences without a verified local copy are played from the drive\n")
                lf.flush()
                continue
            i += 1
            redo = line[0] == "redo"
            if redo:   # a repetition-1 run that ended with an infrastructure fault
                line = line[1:]
            m, s, c, r = line
            if m == "baseline":   # line: baseline <seq> <idle|infra> <rep>
                out = R / "baselines" / s / c / f"rep{r}"
                cmd = [sys.executable, str(R / "scripts" / "run_one.py"), "--seq", s, "--baseline", c,
                       "--rep", r, "--phase", args.phase, "--idle-seconds", "300",
                       "--baseline-play-seconds", "300"]
            else:
                out = R / "runs" / m / s / c / f"rep{r}"
                cmd = [sys.executable, str(R / "scripts" / "run_one.py"), "--method", m,
                       "--seq", s, "--config", c, "--rep", r, "--phase", args.phase]
            if (out / "manifest.json").exists():
                st = json.loads((out / "manifest.json").read_text()).get("status")
                if not (redo and st != "ok"):
                    lf.write(f"skip {m} {s} {c} {r} (manifest exists, status {st})\n")
                    continue
                # Redo from the internal copy; run_one keeps the faulted run as rep<r>_failed_<time>.
                cmd = cmd + ["--retry", f"redo from the internal-disk copy after a final {st} in batch 1"]
            lf.write(f"[{time.strftime('%F %T')}] {i}/{len(lines)} {m} {s} {c} rep{r}\n")
            lf.flush()
            while True:
                res = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT)
                if res.returncode != 3:
                    break
                # Pre-check failed (machine not idle, settings changed): nothing was
                # recorded; wait and try the same run again.
                lf.write(f"[{time.strftime('%F %T')}]   not started (pre-check); waiting 5 min\n")
                lf.flush()
                time.sleep(300)
            man_file = out / "manifest.json"
            status = json.loads(man_file.read_text()).get("status") if man_file.exists() else None
            if status == "infra_fault":
                q = json.loads(man_file.read_text()).get("playback_quality")
                lf.write(f"  infrastructure fault ({q}); one documented retry\n")
                lf.flush()
                res = subprocess.run(cmd + ["--retry", f"infrastructure fault: playback quality {q}"],
                                     stdout=lf, stderr=subprocess.STDOUT)
            if res.returncode != 0:
                failed.append(f"{m} {s} {c} {r}")
        lf.write(f"done; failed: {failed}\n")
    print(f"log: {log_file}; failed: {len(failed)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
