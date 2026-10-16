#!/usr/bin/env python3
"""Power baselines per sequence (brief Section 7):
(i) idle: loggers and roscore only, for --idle-seconds;
(ii) infra: roscore + full playback + recorder + frame logger, no method.
Method power is reported as run power minus the mean of baseline (ii) for the
same sequence. Run one baseline repetition per repetition block (--rep), so
slow drift (temperature, battery ageing) shows up in the baselines too.
"""
import argparse
import subprocess
import sys
from pathlib import Path

import yaml

R = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seqs", nargs="*")
    ap.add_argument("--rep", type=int, default=1)
    ap.add_argument("--idle-seconds", type=int, default=300)
    args = ap.parse_args()
    seqs = yaml.safe_load((R / "datasets" / "sequences.yaml").read_text())["sequences"]
    failed = []
    for seq in args.seqs or [s for s, v in seqs.items() if v.get("enabled", True)]:
        for kind in ("idle", "infra"):
            out = R / "baselines" / seq / kind / f"rep{args.rep}" / "manifest.json"
            if out.exists():
                continue
            cmd = [sys.executable, str(R / "scripts" / "run_one.py"), "--seq", seq, "--baseline", kind,
                   "--rep", str(args.rep), "--phase", "test", "--idle-seconds", str(args.idle_seconds)]
            if subprocess.run(cmd).returncode != 0:
                failed.append(f"{seq} {kind}")
    print(f"failed: {failed}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
