#!/usr/bin/env python3
"""Record the specification of the benchmark machine to env/machine.json.

Reads only world-readable sources (sysfs, /proc, standard tools), so it needs
no root. Run it again whenever the machine configuration may have changed;
each run manifest stores its own snapshot of the volatile fields.
"""
import json
import os
import platform
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


def read(path, default=None):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return default


def run(cmd):
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
        return out.stdout.strip() if out.returncode == 0 else None
    except (OSError, subprocess.TimeoutExpired):
        return None


def lscpu():
    info = {}
    text = run(["lscpu"]) or ""
    for line in text.splitlines():
        if ":" in line:
            k, v = line.split(":", 1)
            info[k.strip()] = v.strip()
    keep = ["Model name", "CPU(s)", "Thread(s) per core", "Core(s) per socket",
            "Socket(s)", "CPU max MHz", "CPU min MHz", "L1d cache", "L2 cache",
            "L3 cache", "Vendor ID", "CPU family", "Model", "Stepping"]
    return {k: info.get(k) for k in keep}


def cpu_topology():
    """Count performance and efficiency cores on hybrid Intel CPUs."""
    types = {}
    for name in ("cpu_core", "cpu_atom"):
        cpus = read(f"/sys/devices/{name}/cpus")
        if cpus:
            types[name] = cpus
    return types


def rapl_domains():
    domains = []
    base = Path("/sys/class/powercap")
    if not base.exists():
        return domains
    for d in sorted(base.glob("intel-rapl:*")) + sorted(base.glob("amd-rapl:*")):
        e = d / "energy_uj"
        domains.append({
            "path": str(d),
            "name": read(d / "name"),
            "max_energy_range_uj": read(d / "max_energy_range_uj"),
            "energy_uj_readable_by_user": os.access(e, os.R_OK),
        })
    return domains


def power_supply():
    out = {}
    for d in sorted(Path("/sys/class/power_supply").glob("*")):
        entry = {"type": read(d / "type")}
        for f in ("online", "status", "capacity", "energy_full",
                  "energy_full_design", "cycle_count", "model_name",
                  "manufacturer", "technology"):
            v = read(d / f)
            if v is not None:
                entry[f] = v
        entry["power_now_available"] = (d / "power_now").exists()
        out[d.name] = entry
    return out


def power_settings():
    cpu0 = Path("/sys/devices/system/cpu/cpu0/cpufreq")
    return {
        "scaling_driver": read(cpu0 / "scaling_driver"),
        "scaling_governor": read(cpu0 / "scaling_governor"),
        "energy_performance_preference": read(cpu0 / "energy_performance_preference"),
        "intel_pstate_status": read("/sys/devices/system/cpu/intel_pstate/status"),
        "intel_pstate_no_turbo": read("/sys/devices/system/cpu/intel_pstate/no_turbo"),
        "power_profiles_daemon": run(["powerprofilesctl", "get"]),
        "acpi_platform_profile": read("/sys/firmware/acpi/platform_profile"),
        "acpi_platform_profile_choices": read("/sys/firmware/acpi/platform_profile_choices"),
        "asus_throttle_thermal_policy": read("/sys/devices/platform/asus-nb-wmi/throttle_thermal_policy"),
        "transparent_hugepage": read("/sys/kernel/mm/transparent_hugepage/enabled"),
    }


def storage(paths):
    out = {}
    for p in paths:
        try:
            du = shutil.disk_usage(p)
        except OSError:
            continue
        dev = run(["findmnt", "-no", "SOURCE,FSTYPE", "--target", p])
        out[p] = {"mount_source_fstype": dev, "total_gb": round(du.total / 1e9, 1),
                  "free_gb": round(du.free / 1e9, 1)}
    disks = []
    for b in sorted(Path("/sys/block").glob("nvme*")) + sorted(Path("/sys/block").glob("sd*")):
        disks.append({"device": b.name, "model": read(b / "device/model"),
                      "size_gb": round(int(read(b / "size", "0")) * 512 / 1e9, 1),
                      "rotational": read(b / "queue/rotational")})
    return {"filesystems": out, "disks": disks}


def gpu():
    nv = run(["nvidia-smi", "--query-gpu=name,driver_version,memory.total",
              "--format=csv,noheader"])
    vga = run(["sh", "-c", "lspci | grep -iE 'vga|3d'"])
    return {"nvidia_smi": nv, "lspci": vga.splitlines() if vga else None,
            "note": "No method in this benchmark uses the GPU."}


def docker():
    return {
        "client": run(["docker", "version", "--format", "{{.Client.Version}}"]),
        "server": run(["docker", "version", "--format", "{{.Server.Version}}"]),
        "storage_driver": run(["docker", "info", "--format", "{{.Driver}}"]),
        "root_dir": run(["docker", "info", "--format", "{{.DockerRootDir}}"]),
        "cgroup_driver": run(["docker", "info", "--format", "{{.CgroupDriver}}"]),
        "cgroup_version": run(["docker", "info", "--format", "{{.CgroupVersion}}"]),
    }


def main():
    repo = Path(__file__).resolve().parents[1]
    os_release = dict(
        line.split("=", 1) for line in (read("/etc/os-release") or "").splitlines() if "=" in line)
    spec = {
        "collected_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "hostname": platform.node(),
        "dmi": {k: read(f"/sys/class/dmi/id/{k}") for k in
                ("sys_vendor", "product_name", "product_version", "board_name",
                 "bios_version", "bios_date")},
        "os": {"pretty_name": os_release.get("PRETTY_NAME", "").strip('"'),
               "kernel": platform.release(), "kernel_version": platform.version(),
               "cmdline": read("/proc/cmdline")},
        "cpu": lscpu(),
        "cpu_hybrid_topology": cpu_topology(),
        "memory": {"mem_total_kb": None, "swap_total_kb": None},
        "power_settings": power_settings(),
        "rapl_domains": rapl_domains(),
        "power_supply": power_supply(),
        "storage": storage(["/", str(repo)]),
        "gpu": gpu(),
        "docker": docker(),
        "python_host": sys.version.split()[0],
    }
    for line in (read("/proc/meminfo") or "").splitlines():
        if line.startswith("MemTotal:"):
            spec["memory"]["mem_total_kb"] = int(line.split()[1])
        if line.startswith("SwapTotal:"):
            spec["memory"]["swap_total_kb"] = int(line.split()[1])
    out = repo / "env" / "machine.json"
    out.write_text(json.dumps(spec, indent=2) + "\n")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
