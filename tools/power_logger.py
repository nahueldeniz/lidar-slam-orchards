#!/usr/bin/env python3
"""Power, temperature and frequency logger (brief Section 7), sampled at 1 Hz.

Primary: RAPL energy counters (package-0, core, uncore, psys) from
/sys/class/powercap. Power is computed afterwards from energy differences;
counter wrap-around is handled with max_energy_range_uj. Reading energy_uj
needs root unless its permissions were changed (OPEN_QUESTIONS.md Q2);
unreadable domains are logged as empty and listed in <out>.json.
Optional: battery power_now (whole laptop, only meaningful on battery).
Also logs the package temperature, mean CPU frequency and AC state.
"""
import argparse
import csv
import json
import signal
import sys
import time
from pathlib import Path

POWERCAP = Path("/sys/class/powercap")


def read(path):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return None


def rapl_domains():
    out = []
    for d in sorted(POWERCAP.glob("intel-rapl:*")) + sorted(POWERCAP.glob("amd-rapl:*")):
        name = read(d / "name")
        out.append({"id": d.name, "name": name, "path": d / "energy_uj",
                    "max": int(read(d / "max_energy_range_uj") or 0),
                    "readable": read(d / "energy_uj") is not None})
    return out


def battery_dir():
    for d in sorted(Path("/sys/class/power_supply").glob("BAT*")):
        return d
    return None


def ac_online():
    for d in Path("/sys/class/power_supply").glob("*"):
        if read(d / "type") == "Mains":
            return read(d / "online")
    return None


def pkg_temp_path():
    for z in Path("/sys/class/thermal").glob("thermal_zone*"):
        if read(z / "type") == "x86_pkg_temp":
            return z / "temp"
    return None


def mean_freq_mhz():
    vals = [int(v) for v in (read(p) for p in
            Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpufreq/scaling_cur_freq")) if v]
    return sum(vals) / len(vals) / 1000.0 if vals else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    domains = rapl_domains()
    bat = battery_dir()
    temp = pkg_temp_path()
    meta = {"rapl": [{k: (str(v) if k == "path" else v) for k, v in d.items()} for d in domains],
            "battery": str(bat) if bat else None, "pkg_temp": str(temp) if temp else None}
    Path(args.out).with_suffix(".json").write_text(json.dumps(meta, indent=2) + "\n")

    fields = ["wall_time", "ac_online"] + [f"rapl_{d['name']}_{d['id']}_uj" for d in domains] + \
             ["bat_power_uw", "bat_voltage_uv", "bat_capacity_pct", "bat_status",
              "pkg_temp_mc", "cpu_freq_mean_mhz"]
    stop = False

    def handler(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(fields)
        next_t = time.monotonic()
        while not stop:
            row = [f"{time.time():.3f}", ac_online()]
            row += [read(d["path"]) if d["readable"] else None for d in domains]
            if bat:
                row += [read(bat / "power_now"), read(bat / "voltage_now"),
                        read(bat / "capacity"), read(bat / "status")]
            else:
                row += [None] * 4
            row += [read(temp) if temp else None, mean_freq_mhz()]
            w.writerow(row)
            f.flush()
            next_t += args.interval
            time.sleep(max(0.0, next_t - time.monotonic()))
    return 0


def energy_to_power(t, e_uj, max_uj):
    """Power (W) between consecutive samples from a wrapping energy counter."""
    import numpy as np
    t = np.asarray(t, dtype=float)
    e = np.asarray(e_uj, dtype=float)
    de = np.diff(e)
    de[de < 0] += max_uj
    return de * 1e-6 / np.diff(t)


if __name__ == "__main__":
    sys.exit(main())
