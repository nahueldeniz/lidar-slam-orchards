#!/usr/bin/env python3
"""Print a JSON snapshot of the volatile host state for a run manifest:
power settings, temperatures, AC/battery, load, memory and free disk.
Needs no root."""
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "env"))
from collect_machine import power_settings, read  # noqa: E402


def temps():
    out = {}
    for z in Path("/sys/class/thermal").glob("thermal_zone*"):
        t = read(z / "temp")
        if t:
            out[read(z / "type")] = int(t) / 1000.0
    return out


def supply():
    out = {}
    for d in Path("/sys/class/power_supply").glob("*"):
        typ = read(d / "type")
        if typ == "Mains":
            out["ac_online"] = read(d / "online")
        elif typ == "Battery":
            out.update({"battery_capacity_pct": read(d / "capacity"),
                        "battery_status": read(d / "status")})
    return out


def main():
    repo = Path(__file__).resolve().parents[1]
    vm = Path("/proc/meminfo").read_text().splitlines()
    mem = {line.split(":")[0]: int(line.split()[1]) for line in vm
           if line.split(":")[0] in ("MemTotal", "MemAvailable", "SwapTotal", "SwapFree")}
    snap = {
        "time": time.time(),
        "power_settings": power_settings(),
        "temperatures_c": temps(),
        "supply": supply(),
        "loadavg": os.getloadavg(),
        "meminfo_kb": mem,
        "disk_free_gb": round(shutil.disk_usage(repo).free / 1e9, 1),
        "docker_running": subprocess.run(["docker", "ps", "--format", "{{.Names}}"],
                                         capture_output=True, text=True).stdout.split(),
    }
    print(json.dumps(snap))


if __name__ == "__main__":
    main()
