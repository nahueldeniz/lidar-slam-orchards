#!/usr/bin/env python3
"""Copy sequences from the external drive to the internal disk (13 October:
the USB drive reset its link mid-run, so the most-used sequences are played
from the internal disk). Each bag is copied, then checked against the official
size and MD5 (datasets/citrusfarm_files.csv); only when every bag of a sequence
matches is STAGED_OK.json written, and only then does scripts/run_one.py use
the local copy. Run it only when no measured run is in progress.

  stage_local.py cf_5 cf_2 cf_6
  stage_local.py --remove cf_5        (delete a local copy; marker first)
A copy is skipped when it would leave less than 5 GB free on the internal disk.
"""
import csv
import hashlib
import json
import shutil
import sys
import time
from pathlib import Path

import yaml

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R / "scripts"))
from inventory import data_root  # noqa: E402


def local_root():
    """$SLAMBENCH_LOCAL if set, else local_data_root in env/run_settings.yaml."""
    import os
    root = os.environ.get("SLAMBENCH_LOCAL") or \
        yaml.safe_load((R / "env" / "run_settings.yaml").read_text()).get("local_data_root")
    if not root:
        raise SystemExit("no local copy folder: set SLAMBENCH_LOCAL or local_data_root in env/run_settings.yaml")
    return Path(root)


def remove(seqs, names):
    root = local_root()
    for s in names:
        marker = root / f"STAGED_OK_{s}.json"
        if marker.exists():
            marker.unlink()          # first, so no run can pick up a half-deleted copy
        freed = 0
        for b in seqs[s]["bags"]:
            f = root / b
            if f.exists():
                freed += f.stat().st_size
                f.unlink()
        print(f"{s}: local copy removed ({freed / 1e9:.2f} GB freed)", flush=True)
    return 0


def md5(path):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 24), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    seqs = yaml.safe_load((R / "datasets" / "sequences.yaml").read_text())["sequences"]
    if sys.argv[1:2] == ["--remove"]:
        return remove(seqs, sys.argv[2:])
    official = {r["file"]: r for r in csv.DictReader(open(R / "datasets" / "citrusfarm_files.csv"))}
    src_root, dst_root = data_root(), local_root()
    ok_all = True
    for s in sys.argv[1:]:
        cfg = seqs[s]
        report = {"seq": s, "source_root": str(src_root), "bags": {}, "started": time.time()}
        ok = True
        dst_root.mkdir(parents=True, exist_ok=True)
        need = sum(int(official[Path(b).name]["size_bytes"]) if Path(b).name in official
                   else (src_root / b).stat().st_size for b in cfg["bags"]
                   if not ((dst_root / b).exists()))
        free = shutil.disk_usage(dst_root).free
        if free - need < 5e9:
            print(f"{s}: NOT staged: needs {need / 1e9:.1f} GB, {free / 1e9:.1f} GB free (keeps 5 GB)", flush=True)
            ok_all = False
            continue
        for b in cfg["bags"]:
            src, dst = src_root / b, dst_root / b
            ref = official.get(Path(b).name)
            want_size = int(ref["size_bytes"]) if ref else src.stat().st_size
            dst.parent.mkdir(parents=True, exist_ok=True)
            if not (dst.exists() and dst.stat().st_size == want_size):
                t0 = time.time()
                tmp = dst.with_suffix(".part")
                with open(src, "rb") as fi, open(tmp, "wb") as fo:
                    shutil.copyfileobj(fi, fo, 1 << 24)
                tmp.rename(dst)
                print(f"{s}: copied {Path(b).name} ({want_size / 1e9:.2f} GB) in {time.time() - t0:.0f} s", flush=True)
            size_ok = dst.stat().st_size == want_size
            h = md5(dst) if ref else None
            match = size_ok and (ref is None or h == ref["md5_official"])
            report["bags"][b] = {"size_ok": size_ok, "md5": h, "md5_official": ref["md5_official"] if ref else None,
                                 "ok": match}
            ok &= match
            print(f"{s}: {Path(b).name} {'verified' if match else 'MISMATCH'}", flush=True)
        report["ok"] = ok
        report["finished"] = time.time()
        marker = dst_root / f"STAGED_OK_{s}.json"
        if ok:
            marker.write_text(json.dumps(report, indent=2) + "\n")
        elif marker.exists():
            marker.unlink()
        ok_all &= ok
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
