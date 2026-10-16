"""Per-run resource, power and real-time summaries (Sections 7-8)."""
import json
from pathlib import Path

import numpy as np
import pandas as pd


def _window(df, t0, t1):
    return df[(df["wall_time"] >= t0) & (df["wall_time"] <= t1)]


def frames_table(run_dir):
    f = Path(run_dir) / "frames.csv"
    if not f.exists():
        return None
    # Stamps stay strings ("sec.nsec") so input/output matching is exact.
    return pd.read_csv(f, dtype={"topic": str, "datatype": str, "stamp": str})


def stamp_ns(s):
    """'sec.nsec' string -> integer nanoseconds (exact, no float rounding)."""
    sec, _, frac = str(s).partition(".")
    return int(sec) * 1_000_000_000 + int((frac + "000000000")[:9])


def realtime(frames, lidar_topic, output_topics, tol_ns=2_000):
    """Processed-scan ratio (output stamps matching input stamps / input scans)
    and latency (output receipt - input receipt for the same stamp).
    Stamps match within tol_ns because some methods (hdl_graph_slam, via PCL
    headers) truncate stamps to microseconds."""
    out = {}
    if frames is None or frames.empty:
        return out
    inp = frames[frames["topic"] == lidar_topic].drop_duplicates("stamp")
    in_ns = np.array([stamp_ns(x) for x in inp["stamp"]], dtype=np.int64)
    order = np.argsort(in_ns)
    in_ns, in_wall = in_ns[order], inp["wall_time"].to_numpy()[order]
    n_in = len(in_ns)
    out["n_input_scans"] = n_in
    for topic in output_topics:
        o = frames[frames["topic"] == topic].drop_duplicates("stamp")
        o_ns = np.array([stamp_ns(x) for x in o["stamp"]], dtype=np.int64)
        o_wall = o["wall_time"].to_numpy()
        lat = np.zeros(0)
        matched_inputs = set()
        if n_in and len(o_ns):
            j = np.clip(np.searchsorted(in_ns, o_ns), 1, n_in - 1) if n_in > 1 else np.zeros(len(o_ns), int)
            cand = np.stack([j - 1, j]) if n_in > 1 else np.stack([j, j])
            d = np.abs(in_ns[cand] - o_ns[None])
            best = cand[np.argmin(d, axis=0), np.arange(len(o_ns))]
            ok = np.min(d, axis=0) <= tol_ns
            lat = o_wall[ok] - in_wall[best[ok]]
            matched_inputs = set(best[ok].tolist())
        out[topic] = {
            "n_output": int(len(o_ns)),
            "n_matched_stamps": len(matched_inputs),
            "processed_scan_ratio": len(matched_inputs) / n_in if n_in else float("nan"),
            "latency_median_s": float(np.median(lat)) if len(lat) else float("nan"),
            "latency_p95_s": float(np.percentile(lat, 95)) if len(lat) else float("nan"),
        }
    return out


def summarise_run(run_dir, n_logical_cpus, lidar_topic, output_topics, rapl_domain="package-0",
                  baseline_power_w=None):
    run_dir = Path(run_dir)
    man = json.loads((run_dir / "manifest.json").read_text())
    t0, t1 = man.get("playback_start"), man.get("playback_end")
    s = {"status": man.get("status")}
    if t0 is None or t1 is None:
        return s
    # Measurement window = while LiDAR data actually flowed (frame logger), not
    # the rosbag play command: opening large bags on the external drive takes
    # up to a minute before the first message is published.
    frames = frames_table(run_dir)
    if frames is not None and lidar_topic:
        lid = frames[frames["topic"] == lidar_topic]["wall_time"]
        if len(lid):
            s["bag_open_s"] = float(lid.min() - t0)
            t0, t1 = float(lid.min()), float(lid.max())
    s["playback_s"] = t1 - t0
    res = pd.read_csv(run_dir / "resources.csv")
    m = _window(res[res["role"] == "method"], t0, t1)
    s["resource_samples"] = int(len(m))
    s["resource_first_before_playback_s"] = float(t0 - res["wall_time"].min())
    s["resource_last_after_playback_s"] = float(res["wall_time"].max() - t1)
    if len(m) and m["cpu_pct"].notna().any():   # baselines have no method container
        cpu = m["cpu_pct"].dropna()
        s.update({
            "cpu_mean_pct_one_core": float(cpu.mean()),
            "cpu_p95_pct_one_core": float(np.percentile(cpu, 95)) if len(cpu) else float("nan"),
            "cpu_mean_pct_all_cores": float(cpu.mean() / n_logical_cpus),
            "cpu_p95_pct_all_cores": float(np.percentile(cpu, 95) / n_logical_cpus) if len(cpu) else float("nan"),
            "rss_peak_gb": float(m["rss"].max() / 1e9),
            "rss_mean_gb": float(m["rss"].mean() / 1e9),
            "rss_final_gb": float(m["rss"].iloc[-1] / 1e9),
            "uss_peak_gb": float(m["uss"].max() / 1e9) if m["uss"].notna().any() else float("nan"),
            "cgroup_mem_peak_gb": float(m["mem_peak"].max() / 1e9),
            "threads_max": int(m["threads"].max()),
        })
        usage = m["cpu_usage_usec"].dropna()
        cpu_s = (usage.iloc[-1] - usage.iloc[0]) * 1e-6 if len(usage) > 1 else float("nan")
        s["cpu_time_s"] = float(cpu_s)
    rt = realtime(frames, lidar_topic, output_topics)
    s["realtime"] = rt
    if rt.get("n_input_scans") and "cpu_time_s" in s:
        s["cpu_ms_per_scan"] = 1000.0 * s["cpu_time_s"] / rt["n_input_scans"]
    pw_file = run_dir / "power.csv"
    if pw_file.exists():
        pw = pd.read_csv(pw_file)
        col = next((c for c in pw.columns if c.startswith(f"rapl_{rapl_domain}_")), None)
        w = _window(pw, t0, t1)
        if col and w[col].notna().sum() > 1:
            meta = json.loads(pw_file.with_suffix(".json").read_text())
            max_uj = next(d["max"] for d in meta["rapl"] if col.startswith(f"rapl_{d['name']}_{d['id']}"))
            e = w[col].to_numpy(dtype=float)
            de = np.diff(e)
            de[de < 0] += max_uj
            energy_j = de.sum() * 1e-6
            dur = w["wall_time"].iloc[-1] - w["wall_time"].iloc[0]
            s["power_mean_w"] = energy_j / dur
            s["energy_wh"] = energy_j / 3600.0
            if baseline_power_w is not None:
                s["power_incremental_w"] = s["power_mean_w"] - baseline_power_w
        else:
            s["power_mean_w"] = float("nan")
            s["power_note"] = f"RAPL {rapl_domain} not readable (OPEN_QUESTIONS.md Q2)"
    return s

def playback_quality(frames, lidar_topic, max_lag_s=0.5):
    """How faithfully the LiDAR stream was replayed. The lag is measured
    against the bag clock (the simulated time at each receipt, published by
    rosbag play from the recording times), so jitter between recording times
    and header stamps in the original data (up to 3.4 s in Pullally) is not
    mistaken for a playback stall; silences skipped by --skip-empty are
    removed. Falls back to header stamps if no clock was logged. Also counts
    bursts (< 20 ms between scans) and stalls (> 2.5 nominal periods). A lag
    above max_lag_s marks an infrastructure fault."""
    if frames is None or frames.empty:
        return {"ok": False, "reason": "no frames"}
    lid = frames[frames["topic"] == lidar_topic].drop_duplicates("stamp")
    if len(lid) < 3:
        return {"ok": False, "reason": "too few LiDAR messages"}
    w = lid["wall_time"].to_numpy(dtype=float)
    st = np.array([stamp_ns(x) for x in lid["stamp"]], dtype=np.int64) * 1e-9
    clock = pd.to_numeric(lid["ros_time"], errors="coerce").to_numpy(dtype=float) \
        if "ros_time" in lid else np.full(len(lid), np.nan)
    use_clock = np.all(np.isfinite(clock)) and np.ptp(clock) > 0
    ref = clock if use_clock else st
    order = np.argsort(w, kind="stable")
    w, ref, st = w[order], ref[order], st[order]
    dw, dref = np.diff(w), np.diff(ref)
    inc = dw - dref
    inc[dref > 30.0] = 0.0          # silence skipped by rosbag play --skip-empty
    lag = np.concatenate([[0.0], np.cumsum(inc)])
    lag -= np.min(lag)
    period = float(np.median(np.diff(np.sort(st))))
    q = {"max_lag_s": float(lag.max()), "p99_lag_s": float(np.percentile(lag, 99)),
         "n_bursts_lt20ms": int(np.sum(dw < 0.02)), "n_stalls": int(np.sum(dw > 2.5 * period)),
         "nominal_period_s": period, "reference": "bag clock" if use_clock else "header stamps"}
    q["ok"] = q["max_lag_s"] <= max_lag_s
    return q
