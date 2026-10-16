#!/usr/bin/env python3
"""Freeze every configuration before the first measured run (brief Section
5.0, step 6; Plan A has no tuned T, so D, hdl D_nofloor and the one-at-a-time
configurations are all fixed in advance). Writes configs/FROZEN.json with a
SHA-256 per configuration directory; scripts/run_one.py refuses measured runs
whose configuration hash differs. Re-running only adds new configurations; a
changed one is an error.
"""
import json
import sys
from pathlib import Path

import yaml

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R / "scripts"))
from run_one import hash_dir  # noqa: E402

METHODS = ["hdl", "lego", "aloam", "dlo", "kiss", "mesh"]


def main():
    out = R / "configs" / "FROZEN.json"
    frozen = json.loads(out.read_text()) if out.exists() else {}
    changed = []
    for m in METHODS:
        cfgs = yaml.safe_load((R / "configs" / m / "configs.yaml").read_text())
        for c in cfgs["test"] + cfgs["oat"]:
            key, h = f"{m}/{c}", hash_dir(R / "configs" / m / c)
            if key in frozen and frozen[key] != h:
                changed.append(key)
            frozen.setdefault(key, h)
    if changed:
        raise SystemExit(f"configurations changed after freezing: {changed}")
    out.write_text(json.dumps(frozen, indent=2, sort_keys=True) + "\n")
    print(f"{len(frozen)} configurations frozen in {out.relative_to(R)}")


if __name__ == "__main__":
    main()
