#!/usr/bin/env python3
"""Execute one benchmark run: method x sequence x configuration x repetition
(brief Section 6). Also runs the power baselines (--baseline idle|infra).

Containers: the infrastructure container (roscore, rosbag play/record, frame
logger, service calls) and the method container are separate, both with
--network host, so the method's CPU and memory come from its own cgroup.
Host-side loggers sample resources and power at 1 Hz from before the method
starts until after it stops.

Everything a run produces goes to runs/<method>/<seq>/<config>/rep<r>/:
outputs.bag, frames.csv, resources.csv, power.csv(+.json), console.log,
infra.log, params_effective.yaml, rostopic_*.txt, manifest.json.

Demo profile (--demo, public demonstration, demo/run_demo.sh): the same run
steps on any Linux machine with Docker. It leaves out everything that was
specific to the benchmark laptop: no CPU pinning, no idle-machine, power-
setting, mains-power or data-drive (USB link) checks, no RAPL power logger,
no internal-disk staging copy, and a short cool-down. The resource logger
still runs; without cgroup v2 it records empty CPU and memory fields. The
configuration must still match configs/FROZEN.json and, for compile-time
methods, the image label. A playback lag above max_playback_lag_s is recorded
in manifest.json but does not mark the run as an infrastructure fault.
Options for the demo (and dry runs): --bag (play these files instead of the
registry's bags), --image and --infra-image (registry names), --out-root.
"""
import argparse
import hashlib
import json
import os
import shlex
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import psutil
import yaml

R = Path(__file__).resolve().parents[1]
# Host-side loggers run with the evaluation environment (.venv, make venv); if
# it is somewhere else, with the interpreter running this script.
PY = [str(R / ".venv" / "bin" / "python") if (R / ".venv" / "bin" / "python").exists() else sys.executable]
C_INFRA, C_METHOD = "sb_infra", "sb_method"
INFRA_IMAGE = "slambench/infra:melodic"
ROS_ENV = ["-e", "ROS_MASTER_URI=http://127.0.0.1:11311", "-e", "ROS_IP=127.0.0.1",
           "-e", "HOME=/tmp"]
IMU_GNSS_TYPES = ("sensor_msgs/Imu", "sensor_msgs/NavSatFix", "gps_common/GPSFix",
                  "nmea_msgs/Sentence", "geographic_msgs/GeoPointStamped")


def utc(t=None):
    return datetime.fromtimestamp(t if t is not None else time.time(), timezone.utc).isoformat()


class Run:
    def __init__(self, args):
        self.args = args
        self.settings = yaml.safe_load((R / "env" / "run_settings.yaml").read_text())
        self.methods = yaml.safe_load((R / "configs" / "methods.yaml").read_text())
        self.seqs = yaml.safe_load((R / "datasets" / "sequences.yaml").read_text())
        self.seq = self.seqs["sequences"][args.seq]
        self.baseline = args.baseline
        self.demo = args.phase == "demo"
        self.infra_image = args.infra_image or INFRA_IMAGE
        # --bag: play these files instead of the registry's bags (demo and dry runs).
        self.bags = [Path(b).resolve() for b in args.bag] if args.bag else None
        if self.baseline:
            self.out = R / "baselines" / args.seq / self.baseline / f"rep{args.rep}"
            self.m = None
        else:
            self.m = self.methods["methods"][args.method]
            if args.out_root:
                root = Path(args.out_root).resolve()
            else:
                root = R / ("runs_dry" if args.phase in ("dry", "demo") else "runs")
            self.out = root / args.method / args.seq / args.config / f"rep{args.rep}"
        self.manifest = {"method": args.method, "seq": args.seq, "config": args.config,
                         "rep": args.rep, "phase": args.phase, "baseline": self.baseline,
                         "dry_run_seconds": args.dry_run_seconds, "events": [], "checks": {},
                         "status": "started"}
        self.loggers = []

    # ---------------------------------------------------------------- helpers
    def log(self, msg):
        line = f"[{utc()}] {msg}"
        print(line, flush=True)
        self.manifest["events"].append(line)

    def sh(self, cmd, check=True, timeout=120):
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        if check and res.returncode != 0:
            raise RuntimeError(f"{shlex.join(cmd)} failed ({res.returncode}): {res.stderr.strip()}")
        return res

    def infra(self, *cmd, detach=False, timeout=120, check=True):
        return self.sh(["docker", "exec"] + (["-d"] if detach else []) +
                       [C_INFRA, "/ros_entrypoint.sh"] + list(cmd), check=check, timeout=timeout)

    def cpuset(self, role):
        if getattr(self.args, "no_cpuset", False) or self.demo:
            return []
        cs = self.settings.get("cpuset", {}).get(role)
        return ["--cpuset-cpus", str(cs)] if cs else []

    # ---------------------------------------------------------------- steps
    def precheck(self):
        """Step 1: idle machine, fixed power settings, no stale containers."""
        for c in (C_INFRA, C_METHOD):
            if self.sh(["docker", "ps", "-aq", "-f", f"name=^{c}$"]).stdout.strip():
                # Left over from an interrupted run: always one of ours.
                self.sh(["docker", "rm", "-f", c], check=False)
                self.log(f"removed stale container {c}")
        if self.demo:
            self.precheck_demo()
            return
        snap = json.loads(self.sh(PY + [str(R / "tools" / "host_state.py")]).stdout)
        self.manifest["host_state_start"] = snap
        expected = self.settings.get("expected_power_settings") or {}
        diff = {k: (v, snap["power_settings"].get(k)) for k, v in expected.items()
                if str(snap["power_settings"].get(k)) != str(v)}
        if diff and not self.args.allow_settings_mismatch:
            raise RuntimeError(f"power settings differ from env/run_settings.yaml: {diff}")
        self.manifest["checks"]["power_settings_mismatch"] = diff
        self.local_root = local_copy_root(self.settings, self.args.seq, self.seq)
        self.manifest["data_source"] = str(self.local_root) if self.local_root else "external drive"
        link = data_link(self.seqs)
        self.manifest["checks"]["data_link"] = link
        min_speed = self.settings.get("min_data_link_mbps", 0)
        if self.local_root is not None:
            pass   # bags on the internal disk: the drive's link does not matter
        elif not link.get("mounted") and not self.bags:
            raise RuntimeError(f"data drive not mounted: {link}")
        if link.get("usb_speed_mbps") is not None and link["usb_speed_mbps"] < min_speed \
                and not self.args.allow_settings_mismatch:
            # On 12-13 October the drive ran on a degraded USB link for 7 h and
            # most runs stalled; never record runs on a slow link.
            raise RuntimeError(f"data drive on a slow USB link: {link}")
        if self.settings.get("require_ac_online") and snap["supply"].get("ac_online") != "1" \
                and not self.args.allow_settings_mismatch:
            raise RuntimeError("laptop not on mains power")
        idle = self.settings["idle"]
        deadline = time.time() + idle["max_wait_s"]
        while True:
            samples = [psutil.cpu_percent(interval=1.0) for _ in range(idle["window_s"])]
            mean = sum(samples) / len(samples)
            if mean < idle["max_mean_cpu_pct"] or self.args.skip_idle_check:
                break
            if time.time() > deadline:
                raise RuntimeError(f"machine not idle: mean CPU {mean:.1f} % over {idle['window_s']} s")
            self.log(f"waiting for idle machine (mean CPU {mean:.1f} %)")
        self.manifest["checks"]["idle_mean_cpu_pct"] = mean
        self.manifest["checks"]["idle_check_skipped"] = bool(self.args.skip_idle_check)
        self.manifest["page_cache_dropped"] = False   # needs root; same for every run

    def precheck_demo(self):
        """Demo profile: no machine-specific checks; only that the bags exist.
        The host state (load, memory, temperatures) is recorded if readable."""
        self.local_root = None
        bags = self.bags or [data_root(self.seqs) / b for b in self.seq["bags"]]
        missing = [str(b) for b in bags if not Path(b).is_file()]
        if missing:
            raise RuntimeError(f"bag file(s) not found: {missing}")
        self.manifest["data_source"] = [str(b) for b in bags]
        res = self.sh(PY + [str(R / "tools" / "host_state.py")], check=False)
        try:
            self.manifest["host_state_start"] = json.loads(res.stdout)
        except json.JSONDecodeError:
            self.manifest["host_state_start"] = None
        self.manifest["checks"].update({"profile": "demo", "idle_check_skipped": True,
                                        "power_settings_checked": False, "cpuset": None})

    def bag_mounts(self):
        """(docker -v arguments, paths of the bags inside the infrastructure
        container). Registry bags: the data root at /data. --bag files: each
        parent folder at /data/<i>, read-only."""
        if not self.bags:
            data = self.local_root or data_root(self.seqs)
            return ["-v", f"{data}:/data:ro"], [f"/data/{b}" for b in self.seq["bags"]]
        parents = list(dict.fromkeys(b.parent for b in self.bags))
        vols = []
        for i, p in enumerate(parents):
            vols += ["-v", f"{p}:/data/{i}:ro"]
        return vols, [f"/data/{parents.index(b.parent)}/{b.name}" for b in self.bags]

    def start_infra(self):
        """Step 2: fresh roscore, /use_sim_time before any node."""
        vols, _ = self.bag_mounts()
        self.sh(["docker", "run", "-d", "--name", C_INFRA, "--network", "host",
                 "--user", f"{os.getuid()}:{os.getgid()}", *ROS_ENV, *self.cpuset("infra"),
                 "-e", "ROS_LOG_DIR=/out/roslog_infra", *vols,
                 "-v", f"{self.out}:/out", self.infra_image, "roscore"])
        self.manifest["infra_image"] = self.infra_image
        for _ in range(60):
            if self.infra("rosparam", "list", check=False, timeout=10).returncode == 0:
                break
            time.sleep(0.5)
        else:
            raise RuntimeError("roscore did not come up")
        self.infra("rosparam", "set", "/use_sim_time", "true")
        self.log("roscore up, /use_sim_time true")

    def start_loggers(self):
        """Step 3: host loggers first, so their first samples precede the method."""
        res = subprocess.Popen(PY + [str(R / "tools" / "resource_logger.py"),
                                     "--watch", f"{C_METHOD}:method", "--watch", f"{C_INFRA}:infra",
                                     "--out", str(self.out / "resources.csv")])
        self.loggers = [res]
        if not self.demo:   # RAPL needs read access to energy_uj (a chmod by root)
            self.loggers.append(subprocess.Popen(PY + [str(R / "tools" / "power_logger.py"),
                                                       "--out", str(self.out / "power.csv")]))
        topics = [self.seq["lidar_topic"]] + ([] if self.baseline else self.output_topics())
        self.infra("rosrun", "slambench_tools", "frame_logger", "/out/frames.csv", *topics, detach=True)
        self.log("loggers started")
        time.sleep(3)

    def output_topics(self):
        outs = self.m["outputs"]
        return [t for group in ("fe", "full") for t in outs.get(group, [])]

    def method_files(self):
        cfg_dir = R / "configs" / self.args.method / self.args.config
        if not cfg_dir.is_dir():
            raise RuntimeError(f"missing configuration directory {cfg_dir}")
        return cfg_dir

    def launch_args(self, cfg_dir):
        """fixed_args (methods.yaml) + dataset_args (sequence sensor settings) +
        launch_args.yaml of the configuration. PENDING values are refused,
        except when overridden with --arg on a dry run."""
        fmt = {"method": self.args.method, "config": self.args.config}
        args = {k: str(v).format(**fmt) for k, v in (self.m.get("fixed_args") or {}).items()}
        sensor = self.seq.get("sensor", {})
        for arg, key in (self.m.get("dataset_args") or {}).items():
            args[arg] = str(sensor.get(key, "PENDING_missing_" + key))
        f = cfg_dir / "launch_args.yaml"
        if f.exists():
            args.update({k: str(v) for k, v in (yaml.safe_load(f.read_text()) or {}).items()})
        for kv in self.args.arg or []:
            k, v = kv.split("=", 1)
            args[k] = v
        pending = {k: v for k, v in args.items() if "PENDING" in v}
        if pending:
            raise RuntimeError(f"launch arguments not decided yet: {pending}")
        return args

    def start_method(self):
        """Step 4: method without visualisation; wait until its nodes are up."""
        cfg_dir = self.method_files()
        fmt = {"config": self.args.config}
        image = self.args.image or self.m["image"].format(**fmt)
        info = json.loads(self.sh(["docker", "image", "inspect", image]).stdout)[0]
        self.manifest["image"] = {"name": image, "id": info["Id"],
                                  "repo_digests": info.get("RepoDigests"),
                                  "labels": (info.get("Config") or {}).get("Labels")}
        self.manifest["config_hash"] = hash_dir(cfg_dir)
        consts = cfg_dir / "constants.env"
        if consts.exists():
            # Compile-time configurations: the image must have been built from
            # exactly this constants.env (label set by scripts/build_images.py).
            want = hashlib.sha256(consts.read_bytes()).hexdigest()
            got = (self.manifest["image"]["labels"] or {}).get("slambench.constants_sha256")
            if got != want:
                raise RuntimeError(f"{image} was not built from {consts} (label {got}, file {want})")
            self.manifest["constants"] = consts.read_text()
        if self.args.phase in ("test", "tuning", "demo"):
            # Section 5.0 step 6: measured runs only with frozen configurations
            # (and the demo only with the configurations of the paper).
            frozen_file = R / "configs" / "FROZEN.json"
            frozen = json.loads(frozen_file.read_text()) if frozen_file.exists() else {}
            key = f"{self.args.method}/{self.args.config}"
            if frozen.get(key) != self.manifest["config_hash"]:
                raise RuntimeError(f"{key} is not frozen or changed after freezing")
        args = self.launch_args(cfg_dir)
        self.manifest["launch_args"] = args
        launch = f"/configs/{self.args.method}/{self.args.config}/{self.m['launch']}"
        cmd = ["roslaunch", launch] + [f"{k}:={v}" for k, v in args.items()]
        self.manifest["method_command"] = cmd
        self.sh(["docker", "run", "-d", "--name", C_METHOD, "--network", "host",
                 "--user", f"{os.getuid()}:{os.getgid()}", *ROS_ENV, *self.cpuset("method"),
                 "-e", "ROS_LOG_DIR=/out/roslog_method",
                 "-v", f"{R / 'configs'}:/configs:ro", "-v", f"{self.out}:/out", image] + cmd)
        time.sleep(self.settings["method_startup_wait_s"])
        state = self.sh(["docker", "inspect", "-f", "{{.State.Running}}", C_METHOD]).stdout.strip()
        if state != "true":
            raise RuntimeError("method container exited during start-up")
        subs = self.subscribers(self.seq["lidar_topic"])
        method_subs = [s for s in subs if "frame_logger" not in s]
        self.manifest["checks"]["lidar_subscribers"] = subs
        if not method_subs:
            raise RuntimeError(f"no method node subscribed to {self.seq['lidar_topic']}")
        self.infra("bash", "-c", "rosparam dump /out/params_effective.yaml")
        self.snapshot_topics("after_method_start")
        self.log(f"method up: {image}")

    def subscribers(self, topic):
        out = self.infra("rostopic", "info", topic, check=False).stdout
        subs, on = [], False
        for line in out.splitlines():
            if line.startswith("Subscribers:"):
                on = True
            elif on and line.strip().startswith("*"):
                subs.append(line.strip()[2:].split(" ")[0])
        return subs

    def snapshot_topics(self, tag):
        """Publishers/types of every topic; flags any IMU/GNSS publisher."""
        out = self.infra("rostopic", "list", "-v", check=False).stdout
        (self.out / f"rostopic_{tag}.txt").write_text(out)
        published = out.split("Subscribed topics:")[0]
        bad = [line.strip() for line in published.splitlines()
               if any(t in line for t in IMU_GNSS_TYPES)]
        self.manifest["checks"][f"imu_gnss_publishers_{tag}"] = bad
        if bad:
            self.log(f"WARNING: IMU/GNSS publishers present ({tag}): {bad}")

    def start_recorder(self):
        """Step 5: record Odometry/Pose topics (never nav_msgs/Path). For the
        infrastructure baseline, the recorder subscribes to every method's
        output topics, none of which is published."""
        if self.baseline:
            topics = sorted({t for m in self.methods["methods"].values()
                             for g in ("fe", "full") for t in m["outputs"].get(g, [])})
        else:
            topics = self.output_topics() + self.m.get("record_extra", [])
        self.infra("rosbag", "record", "--tcpnodelay", "-O", "/out/outputs.bag", *topics, detach=True)
        time.sleep(2)
        self.log(f"recording {topics}")

    def play(self):
        """Step 6: all bags of the sequence, chronological, LiDAR topic only.
        --topics goes last because it consumes every following argument."""
        _, bags = self.bag_mounts()
        cmd = ["rosbag", "play", "--clock", "-r", "1.0", "-q"]
        duration = self.args.dry_run_seconds or self.seq.get("play_duration_s")
        if self.baseline == "infra" and self.args.baseline_play_seconds:
            duration = self.args.baseline_play_seconds
        if duration:
            cmd += ["-u", str(duration)]
        self.manifest["play_duration_s"] = duration
        if self.seq.get("play_skip_empty_s"):
            cmd += [f"--skip-empty={self.seq['play_skip_empty_s']}"]
        cmd += bags + ["--topics", self.seq["lidar_topic"]]
        self.manifest["play_command"] = cmd
        self.manifest["playback_start"] = time.time()
        self.log("playback started")
        t_mid = None
        proc = subprocess.Popen(["docker", "exec", C_INFRA, "/ros_entrypoint.sh"] + cmd,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        while proc.poll() is None:
            time.sleep(1)
            if t_mid is None and time.time() - self.manifest["playback_start"] > 30:
                self.snapshot_topics("during_playback")
                t_mid = time.time()
        (self.out / "play.log").write_text(proc.stdout.read())
        self.manifest["playback_end"] = time.time()
        self.manifest["playback_returncode"] = proc.returncode
        self.log(f"playback finished (rc={proc.returncode})")
        if proc.returncode != 0:
            raise RuntimeError("rosbag play failed")

    def wait_for_quiet(self):
        """Step 7: wait until no output for quiet_s (max max_wait_s); hdl dump."""
        quiet, max_wait = self.settings["quiet_s"], self.settings["max_wait_after_playback_s"]
        outputs = set(self.output_topics())
        t0 = time.time()
        while time.time() - t0 < max_wait:
            last = last_output_wall_time(self.out / "frames.csv", outputs)
            if last is None or time.time() - last >= quiet:
                break
            time.sleep(1)
        self.manifest["wait_after_playback_s"] = time.time() - t0
        for call in self.m.get("post_playback_calls", []):
            # Run in the method container: it has the method's service types
            # (e.g. hdl_graph_slam/DumpGraph) and the same /out mount.
            res = self.sh(["docker", "exec", C_METHOD, "/ros_entrypoint.sh", "bash", "-c", call],
                          check=False, timeout=300)
            self.manifest.setdefault("post_playback_calls", []).append(
                {"call": call, "rc": res.returncode, "stdout": res.stdout[-2000:], "stderr": res.stderr[-2000:]})
            self.log(f"post-playback call rc={res.returncode}: {call}")
        self.snapshot_topics("after_playback")

    def stop_all(self):
        """Step 8: recorder, method, frame logger, roscore, host loggers; logs."""
        if self.container_exists(C_INFRA):
            self.infra("pkill", "-INT", "-f", "rosbag/record", check=False)
            for _ in range(60):
                if not (self.out / "outputs.bag.active").exists():
                    break
                time.sleep(0.5)
        if self.container_exists(C_METHOD):
            self.sh(["docker", "kill", "-s", "INT", C_METHOD], check=False)
            for _ in range(40):
                if self.sh(["docker", "inspect", "-f", "{{.State.Running}}", C_METHOD],
                           check=False).stdout.strip() != "true":
                    break
                time.sleep(0.5)
            self.sh(["docker", "stop", "-t", "5", C_METHOD], check=False)
            st = json.loads(self.sh(["docker", "inspect", C_METHOD]).stdout)[0]["State"]
            self.manifest["method_exit"] = {"code": st.get("ExitCode"), "oom_killed": st.get("OOMKilled")}
            logs = self.sh(["docker", "logs", "--timestamps", C_METHOD], check=False)
            (self.out / "console.log").write_text(logs.stdout + logs.stderr)
            self.sh(["docker", "rm", C_METHOD], check=False)
        if self.container_exists(C_INFRA):
            self.infra("pkill", "-INT", "-f", "frame_logger", check=False)
            time.sleep(1)
            self.sh(["docker", "stop", "-t", "10", C_INFRA], check=False)
            logs = self.sh(["docker", "logs", "--timestamps", C_INFRA], check=False)
            (self.out / "infra.log").write_text(logs.stdout + logs.stderr)
            self.sh(["docker", "rm", C_INFRA], check=False)
        time.sleep(2)
        for p in self.loggers:
            p.send_signal(2)
        for p in self.loggers:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
        self.log("all stopped")

    def container_exists(self, name):
        return bool(self.sh(["docker", "ps", "-aq", "-f", f"name=^{name}$"], check=False).stdout.strip())

    def write_manifest(self):
        """Step 9."""
        sys.path.insert(0, str(R / "eval"))
        from slameval import resources
        if self.baseline == "idle":   # no playback in an idle baseline
            q = {"ok": True, "reason": "idle baseline, no playback"}
        else:
            q = resources.playback_quality(resources.frames_table(self.out), self.seq["lidar_topic"],
                                           self.settings.get("max_playback_lag_s", 0.5))
        self.manifest["playback_quality"] = q
        if self.manifest["status"] == "ok" and not q.get("ok") and self.args.phase not in ("dry", "demo"):
            # Playback fell behind the bag timeline (e.g. a slow drive): an
            # infrastructure fault, eligible for one documented retry.
            self.manifest["status"] = "infra_fault"
            self.log(f"playback quality check failed: {q}")
        self.manifest["end"] = time.time()
        self.manifest["end_utc"] = utc(self.manifest["end"])
        try:
            self.manifest["host_state_end"] = json.loads(
                self.sh(PY + [str(R / "tools" / "host_state.py")], check=False).stdout)
        except json.JSONDecodeError:
            pass
        self.manifest["message_counts"] = count_frames(self.out / "frames.csv")
        (self.out / "manifest.json").write_text(json.dumps(self.manifest, indent=2, default=str) + "\n")

    def execute(self):
        if (self.out / "manifest.json").exists():
            old = json.loads((self.out / "manifest.json").read_text())
            if not self.args.retry:
                raise SystemExit(f"{self.out} already has a manifest (status {old.get('status')}); "
                                 "use --retry 'reason' for a documented infrastructure retry")
            stamp = datetime.now().strftime("%Y%m%dT%H%M%S")
            self.out.rename(self.out.with_name(f"{self.out.name}_failed_{stamp}"))
            self.manifest["retry_reason"] = self.args.retry
        try:
            self.precheck()
        except RuntimeError as exc:
            # Nothing has started: leave no run directory, so the batch can try
            # the same run again later (exit code 3 = not started).
            print(f"[{utc()}] NOT STARTED: {exc}", flush=True)
            return 3
        self.out.mkdir(parents=True, exist_ok=True)
        self.manifest["start"] = time.time()
        self.manifest["start_utc"] = utc(self.manifest["start"])
        try:
            self.start_infra()
            self.start_loggers()
            if self.baseline == "idle":
                time.sleep(self.args.idle_seconds)
            else:
                if not self.baseline:
                    self.start_method()
                self.start_recorder()
                self.play()
                if not self.baseline:
                    self.wait_for_quiet()
            self.manifest["status"] = "ok"
        except Exception as exc:   # keep everything for the failure report
            self.manifest["status"] = "failed"
            self.manifest["error"] = repr(exc)
            self.log(f"FAILED: {exc!r}")
        finally:
            self.stop_all()
            self.write_manifest()
            cool = 5 if self.args.phase in ("dry", "demo") else self.settings["cooldown_s"]
            self.log(f"cooling down {cool} s")
            time.sleep(cool)
        return 0 if self.manifest["status"] == "ok" else 1


def local_copy_root(settings, seq_name, seq):
    """The internal-disk copy of a sequence, if scripts/stage_local.py verified
    it (STAGED_OK_<seq>.json) and every bag is present with the expected size.
    Location: $SLAMBENCH_LOCAL if set, else local_data_root in env/run_settings.yaml
    (null = no local copies), as in scripts/stage_local.py."""
    root = os.environ.get("SLAMBENCH_LOCAL") or settings.get("local_data_root")
    if not root:
        return None
    root = Path(root)
    marker = root / f"STAGED_OK_{seq_name}.json"
    if not marker.exists():
        return None
    try:
        rep = json.loads(marker.read_text())
    except (OSError, json.JSONDecodeError):
        return None
    if not rep.get("ok") or any(not (root / b).exists() for b in seq["bags"]):
        return None
    return root


def data_link(seqs):
    """Mount and USB link speed of the drive that holds the bags."""
    root = data_root(seqs)
    out = {"root": str(root), "mounted": root.exists() and any(root.iterdir())}
    src = subprocess.run(["findmnt", "-no", "SOURCE", "--target", str(root)],
                         capture_output=True, text=True).stdout.strip()
    out["device"] = src
    if src.startswith("/dev/"):
        dev = Path("/sys/class/block") / Path(src).name
        p = dev.resolve()
        while p != p.parent:
            if (p / "speed").exists() and (p / "idVendor").exists():
                out["usb_speed_mbps"] = float((p / "speed").read_text().strip())
                out["usb_device"] = p.name
                break
            p = p.parent
        ra = Path("/sys/block") / dev.resolve().parent.name / "queue" / "read_ahead_kb"
        if ra.exists():
            out["read_ahead_kb"] = int(ra.read_text())
    return out


def data_root(seqs=None):
    """Bag root: $SLAMBENCH_DATA if set, else data_root in datasets/sequences.yaml,
    else datasets/."""
    import os
    import yaml
    if os.environ.get("SLAMBENCH_DATA"):
        return Path(os.environ["SLAMBENCH_DATA"]).resolve()
    seqs = seqs or yaml.safe_load((R / "datasets" / "sequences.yaml").read_text())
    root = str(seqs.get("data_root") or "")
    return Path(root).resolve() if root and "$" not in root else (R / "datasets").resolve()

def hash_dir(path):
    h = hashlib.sha256()
    for f in sorted(Path(path).rglob("*")):
        if f.is_file():
            h.update(str(f.relative_to(path)).encode())
            h.update(f.read_bytes())
    return h.hexdigest()


def last_output_wall_time(frames_csv, outputs):
    try:
        with open(frames_csv, "rb") as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell() - 65536))
            lines = f.read().decode(errors="replace").splitlines()[1:]
    except OSError:
        return None
    for line in reversed(lines):
        parts = line.split(",")
        if len(parts) >= 3 and parts[2] in outputs:
            return float(parts[0])
    return None


def count_frames(frames_csv):
    counts = {}
    try:
        with open(frames_csv) as f:
            next(f)
            for line in f:
                parts = line.split(",")
                if len(parts) >= 3:
                    counts[parts[2]] = counts.get(parts[2], 0) + 1
    except (OSError, StopIteration):
        pass
    return counts


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--method")
    ap.add_argument("--seq", required=True)
    ap.add_argument("--config", default="D")
    ap.add_argument("--rep", type=int, default=1)
    ap.add_argument("--phase", choices=["tuning", "test", "dry", "demo"],
                    help="required unless --demo is given (--demo = --phase demo)")
    ap.add_argument("--demo", action="store_true",
                    help="portable demo profile (see above): no pinning, no machine checks, no RAPL")
    ap.add_argument("--baseline", choices=["idle", "infra"])
    ap.add_argument("--idle-seconds", type=int, default=300)
    ap.add_argument("--baseline-play-seconds", type=float,
                    help="infrastructure baseline: play only this many seconds (power is stationary)")
    ap.add_argument("--dry-run-seconds", type=float, help="play only the first N seconds (dry and demo runs)")
    ap.add_argument("--retry", metavar="REASON", help="documented retry after an infrastructure fault")
    ap.add_argument("--skip-idle-check", action="store_true", help="dry runs only")
    ap.add_argument("--allow-settings-mismatch", action="store_true", help="dry runs only")
    ap.add_argument("--no-cpuset", action="store_true", help="dry runs only: no CPU pinning")
    ap.add_argument("--arg", action="append", metavar="NAME=VALUE",
                    help="override a launch argument (dry runs only)")
    ap.add_argument("--bag", action="append", metavar="FILE",
                    help="play this bag instead of the sequence's registry bags (repeatable, in "
                         "chronological order; dry and demo runs only)")
    ap.add_argument("--image", help="method image to run instead of configs/methods.yaml "
                                    "(e.g. a registry name; same build, same labels)")
    ap.add_argument("--infra-image", help=f"infrastructure image (default {INFRA_IMAGE})")
    ap.add_argument("--out-root", help="folder for <method>/<seq>/<config>/rep<r>/ "
                                       "(default runs/, or runs_dry/ for dry and demo runs)")
    args = ap.parse_args()
    if args.demo:
        if args.phase not in (None, "demo"):
            ap.error("--demo cannot be combined with --phase " + args.phase)
        args.phase = "demo"
    if args.phase is None:
        ap.error("--phase is required (or --demo)")
    if not args.baseline and not args.method:
        ap.error("--method is required unless --baseline is given")
    if args.phase == "demo" and args.baseline:
        ap.error("--demo runs a method; power baselines need the measured set-up")
    if args.phase != "dry" and (args.skip_idle_check or args.allow_settings_mismatch
                                or args.arg or args.no_cpuset):
        ap.error("--skip-idle-check, --allow-settings-mismatch, --no-cpuset and --arg "
                 "are for --phase dry only (the demo profile implies the first three)")
    if args.phase not in ("dry", "demo") and (args.dry_run_seconds or args.bag):
        ap.error("--dry-run-seconds and --bag are for dry and demo runs only")
    if args.baseline:
        args.method = "baseline"
        args.config = args.baseline
    return Run(args).execute()


if __name__ == "__main__":
    sys.exit(main())
