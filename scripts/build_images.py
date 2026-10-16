#!/usr/bin/env python3
"""Build the method images (brief Section 3) and record what was built.

Runtime-configured methods get one image, slambench/<method>:melodic.
Methods whose parameters are compile-time constants (image template with
"{config}" in configs/methods.yaml) get one image per configuration that has
a constants.env; every KEY=VALUE line is passed as --build-arg, and the image
is labelled with the SHA-256 of constants.env so scripts/run_one.py can check
that the image matches the configuration it runs. Identical constants hit the
Docker layer cache, so they cost no extra disk.

Writes env/build_logs/<image>.log and env/images.json (image IDs, labels,
build time), the source for table T2. Refuses to build when free disk space
is below --min-free-gb.
"""
import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

import yaml

R = Path(__file__).resolve().parents[1]


def build(tag, dockerfile, build_args, labels, log_name):
    cmd = ["docker", "build", "-f", str(dockerfile), "-t", tag]
    for k, v in build_args.items():
        cmd += ["--build-arg", f"{k}={v}"]
    for k, v in labels.items():
        cmd += ["--label", f"{k}={v}"]
    cmd.append(str(R / "env"))
    log = R / "env" / "build_logs" / f"{log_name}.log"
    t0 = time.time()
    with open(log, "w") as f:
        rc = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT).returncode
    info = {"tag": tag, "returncode": rc, "seconds": round(time.time() - t0), "log": str(log.relative_to(R)),
            "build_args": build_args, "labels": labels}
    if rc == 0:
        info["id"] = subprocess.run(["docker", "image", "inspect", "-f", "{{.Id}}", tag],
                                    capture_output=True, text=True).stdout.strip()
    print(f"{tag}: {'ok' if rc == 0 else 'FAILED'} ({info['seconds']} s)")
    return info


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("methods", nargs="+")
    ap.add_argument("--configs", nargs="*", help="compile-time methods: only these configurations")
    ap.add_argument("--min-free-gb", type=float, default=15.0)
    args = ap.parse_args()
    free = shutil.disk_usage("/var/lib/docker" if Path("/var/lib/docker").exists() else "/").free / 1e9
    if free < args.min_free_gb:
        raise SystemExit(f"only {free:.1f} GB free for Docker; refusing to build (OPEN_QUESTIONS.md Q1)")
    methods = yaml.safe_load((R / "configs" / "methods.yaml").read_text())["methods"]
    out_file = R / "env" / "images.json"
    images = json.loads(out_file.read_text()) if out_file.exists() else {}
    built = []
    for m in args.methods:
        spec = methods[m]
        dockerfile = R / "env" / "docker" / m / "Dockerfile"
        if "{config}" in spec["image"]:
            cfgs = sorted(d.name for d in (R / "configs" / m).iterdir()
                          if (d / "constants.env").exists() and (not args.configs or d.name in args.configs))
            for c in cfgs:
                env_file = R / "configs" / m / c / "constants.env"
                consts = dict(l.split("=", 1) for l in env_file.read_text().splitlines()
                              if l.strip() and not l.startswith("#") and not l.startswith("EXPECT_"))
                labels = {"slambench.method": m, "slambench.config": c,
                          "slambench.constants_sha256": hashlib.sha256(env_file.read_bytes()).hexdigest()}
                tag = spec["image"].format(config=c)
                images[tag] = build(tag, dockerfile, consts, labels, f"{m}_{c}")
                built.append(tag)
        else:
            tag = spec["image"]
            images[tag] = build(tag, dockerfile, {}, {"slambench.method": m}, m)
            built.append(tag)
        out_file.write_text(json.dumps(images, indent=2) + "\n")
    return 0 if all(images[t]["returncode"] == 0 for t in built) else 1


if __name__ == "__main__":
    sys.exit(main())
