#!/usr/bin/env python3
"""Check local copies of the CitrusFarm base bags (the 2024 copies on the
external drive) against the official sizes and MD5 sums in
datasets/citrusfarm_files.csv. Writes datasets/local_copies_md5.json.
A copy that does not match must not be used (download it instead)."""
import csv
import hashlib
import json
import sys
from pathlib import Path

import yaml

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R / "scripts"))
from inventory import data_root  # noqa: E402


def md5(path):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 24), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    official = {r["file"]: r for r in csv.DictReader(open(R / "datasets" / "citrusfarm_files.csv"))}
    seqs = yaml.safe_load((R / "datasets" / "sequences.yaml").read_text())["sequences"]
    root = data_root()
    out, ok = {}, True
    for s, cfg in seqs.items():
        if cfg["dataset"] != "citrusfarm" or not cfg.get("enabled", True):
            continue
        for b in cfg["bags"]:
            p = root / b
            ref = official[Path(b).name]
            if not p.exists():
                out[b] = {"status": "missing"}
                continue
            size_ok = p.stat().st_size == int(ref["size_bytes"])
            m = md5(p) if size_ok else None
            match = size_ok and m == ref["md5_official"]
            ok &= match
            out[b] = {"seq": s, "size_ok": size_ok, "md5": m, "md5_official": ref["md5_official"],
                      "status": "ok" if match else "MISMATCH"}
            print(f"{s} {Path(b).name}: {out[b]['status']}", flush=True)
    (R / "datasets" / "local_copies_md5.json").write_text(json.dumps(out, indent=2) + "\n")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
