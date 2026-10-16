#!/usr/bin/env python3
"""Per-container resource logger (brief Section 7), sampled at a fixed rate on the host.

For every watched container (role 'method' or 'infra') it logs, from the
container's cgroup v2:
  cpu_pct      CPU use over the last interval, 100 % = one logical core
               (cpu.stat usage_usec, exact, includes short-lived processes)
  mem_current  memory.current (bytes; includes page cache)
  mem_peak     memory.peak (high-water mark of memory.current)
  mem_anon     anonymous memory from memory.stat
and, from psutil over the PIDs in cgroup.procs:
  rss, uss     sums over the container's processes (USS needs the processes to
               run as the same user as the logger; otherwise it is empty)
  threads, nprocs
plus system-wide totals (CPU %, used = total - available, swap used) and the
logger's own CPU time. Containers that do not exist yet, or have exited, are
logged with empty fields, so the logger can start before the method.
"""
import argparse
import csv
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import psutil

CGROUP_ROOTS = ("/sys/fs/cgroup/system.slice/docker-{id}.scope", "/sys/fs/cgroup/docker/{id}")


def container_cgroup(name):
    try:
        cid = subprocess.run(["docker", "inspect", "-f", "{{.Id}}", name], capture_output=True,
                             text=True, timeout=5).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return None
    if not cid:
        return None
    for pattern in CGROUP_ROOTS:
        p = Path(pattern.format(id=cid))
        if p.exists():
            return p
    return None


def read_int(path):
    try:
        return int(Path(path).read_text().split()[0])
    except (OSError, ValueError, IndexError):
        return None


def read_kv(path):
    out = {}
    try:
        for line in Path(path).read_text().splitlines():
            k, v = line.split()
            out[k] = int(v)
    except (OSError, ValueError):
        pass
    return out


class Watched:
    def __init__(self, name, role):
        self.name, self.role = name, role
        self.cg = None
        self.last_usage = None
        self.last_t = None
        self.procs = {}

    def sample(self, now):
        if self.cg is None or not self.cg.exists():
            self.cg = container_cgroup(self.name)
            self.last_usage = None
            self.procs = {}
        row = {"container": self.name, "role": self.role}
        if self.cg is None or not self.cg.exists():
            return row
        usage = read_kv(self.cg / "cpu.stat").get("usage_usec")
        if usage is not None and self.last_usage is not None and now > self.last_t:
            row["cpu_pct"] = 100.0 * (usage - self.last_usage) / ((now - self.last_t) * 1e6)
        row["cpu_usage_usec"] = usage
        self.last_usage, self.last_t = usage, now
        row["mem_current"] = read_int(self.cg / "memory.current")
        row["mem_peak"] = read_int(self.cg / "memory.peak")
        row["mem_anon"] = read_kv(self.cg / "memory.stat").get("anon")
        row["pids"] = read_int(self.cg / "pids.current")
        rss = uss = threads = 0
        uss_ok = True
        pids = []
        try:
            pids = [int(x) for x in (self.cg / "cgroup.procs").read_text().split()]
        except OSError:
            pass
        for pid in pids:
            try:
                proc = self.procs.get(pid) or psutil.Process(pid)
                self.procs[pid] = proc
                with proc.oneshot():
                    rss += proc.memory_info().rss
                    threads += proc.num_threads()
                    if uss_ok:
                        try:
                            uss += proc.memory_full_info().uss
                        except psutil.AccessDenied:
                            uss_ok = False
            except (psutil.NoSuchProcess, psutil.ZombieProcess):
                self.procs.pop(pid, None)
        row.update({"nprocs": len(pids), "rss": rss, "uss": uss if uss_ok else None,
                    "threads": threads})
        return row


FIELDS = ["wall_time", "container", "role", "cpu_pct", "cpu_usage_usec", "mem_current",
          "mem_peak", "mem_anon", "pids", "nprocs", "rss", "uss", "threads",
          "sys_cpu_pct", "sys_mem_used", "sys_swap_used", "logger_cpu_s"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--watch", action="append", default=[], metavar="NAME:ROLE",
                    help="container name and role (method|infra); repeatable")
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    watched = [Watched(*w.split(":", 1)) for w in args.watch]
    stop = False

    def handler(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    me = psutil.Process(os.getpid())
    psutil.cpu_percent(None)
    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        next_t = time.monotonic()
        while not stop:
            now = time.monotonic()
            wall = time.time()
            vm, sw = psutil.virtual_memory(), psutil.swap_memory()
            system = {"wall_time": f"{wall:.3f}", "sys_cpu_pct": psutil.cpu_percent(None),
                      "sys_mem_used": vm.total - vm.available, "sys_swap_used": sw.used,
                      "logger_cpu_s": sum(me.cpu_times()[:2])}
            for c in watched:
                w.writerow({**system, **c.sample(now)})
            if not watched:
                w.writerow(system)
            f.flush()
            next_t += args.interval
            time.sleep(max(0.0, next_t - time.monotonic()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
