"""Helpers shared by scripts/make_figures.py and scripts/make_tables.py
(brief Sections 12-14).

Everything here reads pipeline outputs only:
  <results>/results_long.csv, segments.csv, heading.csv, run_diagnostics.csv,
  resources_summary.csv                       (scripts/evaluate_runs.py)
  <runs>/<method>/<seq>/<config>/rep<r>/<output>.tum
  <gt>/<seq>/gt_tum.txt, gt_quality.json      (scripts/process_gt.py)
  <configs>/<method>/sensitivity.yaml, <configs>/SENSITIVITY_DESIGN.md
  eval/config/outputs.yaml (headline output, main lever-arm setting: the single switch)

Aggregation rule (brief Section 12): mean +- SD (sample SD, ddof = 1) over
repetitions per method x sequence x configuration x output; never across
sequences. A repetition that was attempted but has no value (run status not
ok, no output, no overlap with the ground truth) counts as a failure: it is
reported as "k of n" or "failed", never dropped silently.
"""
import json
import math
from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path

import numpy as np
import pandas as pd
import yaml
from evo.core.geometry import GeometryException

from . import align, associate as assoc_mod, metrics, style
from .evaluate import estimated_positions, prepare_ground_truth
from .trajectory import read_tum

ROOT = Path(__file__).resolve().parents[2]
METHODS = style.METHODS
SEQ_LABEL = {**style.SEQ_LABEL, "pl_1": "Pullally run 1"}
OUTPUT_LABEL = {"fe": "front end", "full": "full output", "full_final": "full output, final keyframes"}
CONFIG_LABEL = {"D": "D (repository defaults)", "D_nofloor": "D without floor detection"}
DIM_LABEL = {"3d": "3D", "2d": "Horizontal", "z": "Vertical"}
# Topic whose processed-scan ratio and latency describe the headline output
# (brief Section 6.1). hdl's online output is defined at the /odom stamps.
# If a run recorded the topic under another name, realtime_column() falls
# back to the only topic present, or to a topic with the same last component.
REALTIME_TOPIC = {"hdl": "/odom", "lego": "/integrated_to_init", "aloam": "/aft_mapped_to_init_high_frec",
                  "dlo": "/robot/dlo/odom_node/odom", "kiss": "/kiss/odometry", "mesh": "/lidar_odometry"}
RESULT_FILES = ("results_long", "segments", "heading", "run_diagnostics", "resources_summary")


# ---------------------------------------------------------------- formatting
def decimals(x):
    """Decimals so that a value and its SD read at about three significant digits."""
    a = abs(x) if x is not None and np.isfinite(x) else 1.0
    return 3 if a < 1 else 2 if a < 10 else 1 if a < 100 else 0


def fmt(x, d=None, sign=False):
    if x is None or not np.isfinite(x):
        return "N/A"
    d = decimals(x) if d is None else d
    return f"{x:+.{d}f}" if sign else f"{x:.{d}f}"


@dataclass
class Agg:
    """Values of one quantity over the repetitions of one method x sequence x
    configuration (x output). values: rep -> value for repetitions with a
    value; n_total: repetitions attempted (with or without a value)."""
    values: dict = field(default_factory=dict)
    n_total: int = 0
    lever: str = ""

    def __post_init__(self):
        self.n_total = max(self.n_total, len(self.values))

    @property
    def n_ok(self):
        return len(self.values)

    @property
    def arr(self):
        return np.array(list(self.values.values()), dtype=float)

    @property
    def mean(self):
        return float(np.mean(self.arr)) if self.n_ok else float("nan")

    @property
    def sd(self):
        return float(np.std(self.arr, ddof=1)) if self.n_ok > 1 else float("nan")

    @property
    def status(self):
        if self.n_total == 0:
            return "N/A"
        return "failed" if self.n_ok == 0 else "ok"

    def text(self, d=None, sd=True, counts=True, scale=1.0):
        """'1.23 ± 0.04', '1.23 ± 0.04 (2 of 3)', '1.23' (one run), 'failed (0 of 3)', 'N/A'."""
        if self.status == "N/A":
            return "N/A"
        if self.status == "failed":
            return f"failed (0 of {self.n_total})"
        m = self.mean * scale
        d = decimals(m) if d is None else d
        s = f"{m:.{d}f}"
        if sd and self.n_ok > 1:
            s += f" ± {self.sd * scale:.{d}f}"
        if counts and self.n_ok < self.n_total:
            s += f" ({self.n_ok} of {self.n_total})"
        return s


def fmt_level(v):
    """Level value as written in configuration names (= scripts/make_configs.py fmt)."""
    if isinstance(v, bool):
        return str(v).lower()
    return str(v).replace("-", "m")


PARAM_FIELDS = {"id", "name", "unit", "module", "default", "levels", "changes"}


def param_name(p):
    """Parameter name from a sensitivity.yaml entry. In a YAML flow mapping an
    unquoted comma ends the scalar, so 'name: a (b, c, d)' loads as name 'a (b'
    plus null-valued keys 'c' and 'd)'; those pieces are joined back."""
    extra = [str(k) for k, v in p.items() if k not in PARAM_FIELDS and v is None]
    return ", ".join([str(p.get("name", p.get("id", "")))] + extra)


def oat_name(param_id, value):
    return f"OAT_{param_id}_{fmt_level(value)}"


def parse_oat(config):
    """'OAT_c_fe_0.5' -> ('c_fe', '0.5'); negative values are written with 'm'."""
    if not str(config).startswith("OAT_"):
        return None
    pid, _, val = config[4:].rpartition("_")
    if not pid:
        return None
    return pid, (("-" + val[1:]) if val.startswith("m") else val)


def parse_design_table(text):
    """Rows of the 'Selected parameters' table of configs/SENSITIVITY_DESIGN.md,
    per method in table order: param, module, default, levels, reason."""
    out, in_table = {}, False
    for line in text.splitlines():
        ls = line.strip()
        if ls.startswith("| Method"):
            in_table = True
            continue
        if not in_table:
            continue
        if not ls.startswith("|"):
            in_table = False
            continue
        if set(ls.replace("|", "").strip()) <= set("-: "):
            continue
        cells = [c.strip().replace("`", "") for c in ls.strip("|").split("|")]
        if len(cells) >= 6:
            out.setdefault(cells[0], []).append(
                {"param": cells[1], "module": cells[2], "default": cells[3], "levels": cells[4],
                 "reason": cells[5]})
    return out


# ---------------------------------------------------------------- plotting helpers
def tint(colour, f):
    """Mix a colour with white: f = 1 gives the colour, f = 0 white."""
    from matplotlib.colors import to_rgb
    c = np.array(to_rgb(colour))
    return tuple(1.0 - f * (1.0 - c))


def decimate_minmax(x, y, n_max=3000):
    """Keep the minimum and maximum of y in each of n_max/2 consecutive chunks,
    so a dense line looks the same at print resolution with far fewer vertices."""
    x, y = np.asarray(x), np.asarray(y)
    if len(x) <= n_max:
        return x, y
    edges = np.linspace(0, len(x), n_max // 2 + 1).astype(int)
    idx = [0, len(x) - 1]
    for a, b in zip(edges[:-1], edges[1:]):
        if b > a:
            seg = y[a:b]
            idx += [a + int(np.nanargmin(seg)), a + int(np.nanargmax(seg))] if np.isfinite(seg).any() else [a]
    idx = np.unique(idx)
    return x[idx], y[idx]


def stride(p, n_max=4000):
    """Every k-th row of p (first and last kept), for plotting dense paths."""
    if len(p) <= n_max:
        return p
    k = int(np.ceil(len(p) / n_max))
    return np.vstack([p[::k], p[-1:]])


def turn_intervals(gt):
    """[(s_start, s_end)] distance intervals of ground-truth samples labelled turn."""
    f = np.asarray(gt.turn, dtype=int)
    edges = np.flatnonzero(np.diff(np.r_[0, f, 0]))
    s = gt.arc.s
    return [(float(s[a]), float(s[b - 1])) for a, b in zip(edges[::2], edges[1::2]) if s[b - 1] > s[a]]


def log_bins(vmin, vmax, max_classes=7):
    """Nice logarithmic class boundaries (1-2-5, 1-3, decades) covering [vmin, vmax]."""
    vmin = max(vmin, 1e-3)
    vmax = max(vmax, vmin * 1.0001)
    for mant in ((1, 2, 5), (1, 3), (1,)):
        nice = [m * 10.0 ** e for e in range(-3, 6) for m in mant]
        lo = max([b for b in nice if b <= vmin], default=nice[0])
        hi = min([b for b in nice if b >= vmax], default=nice[-1])
        b = [x for x in nice if lo <= x <= hi]
        if len(b) < 2:
            b = [lo, nice[nice.index(lo) + 1]]
        if len(b) - 1 <= max_classes:
            return b
    e0, e1 = math.floor(math.log10(vmin)), math.ceil(math.log10(vmax))
    step = math.ceil((e1 - e0) / max_classes)
    return [10.0 ** e for e in range(e0, e1 + step, step)]


# ---------------------------------------------------------------- per-pose errors
@dataclass
class PoseErrors:
    t: np.ndarray        # associated stamps (s)
    s: np.ndarray        # ground-truth distance travelled at t (m)
    p: np.ndarray        # aligned estimate at the associated stamps
    p_gt: np.ndarray     # ground truth at the associated stamps
    err: dict            # '3d', '2d', 'z' per-pose error (m)
    p_all: np.ndarray    # every finite pose, aligned (for drawing)

    @property
    def rmse(self):
        return float(np.sqrt(np.mean(self.err["3d"] ** 2)))


def compute_pose_errors(est, gt, cfg, lever_arm=None, alignment="A"):
    """Per-pose errors exactly as slameval.evaluate.evaluate_output computes
    them (same association, lever arm and alignment), for plotting."""
    est = est.subset(np.flatnonzero(est.finite_mask()))
    max_gap = cfg["association"]["max_gap_periods"] * gt.nominal_period_s
    a = assoc_mod.associate(est.t, gt.t, gt.p, max_gap)
    if a.n < 3:
        return None
    s_k = np.interp(a.t, gt.t, gt.arc.s)
    p_hat = estimated_positions(est, lever_arm if (lever_arm is not None and est.has_orientation) else None)
    p_k = p_hat[a.idx]
    try:
        if alignment == "A":
            al = align.fit_global(p_k, a.p_gt)
        else:
            al = align.fit_initial(p_k, a.p_gt, s_k, cfg["alignment_B"]["initial_distance_m"])
    except GeometryException:
        return None
    pa = al.apply(p_k)
    return PoseErrors(t=a.t, s=s_k, p=pa, p_gt=a.p_gt, err=metrics.error_components(pa - a.p_gt),
                      p_all=al.apply(p_hat))


# ---------------------------------------------------------------- context
def add_path_args(ap, out_help="output directory"):
    ap.add_argument("--results-dir", help="folder with results_long.csv etc. (default: eval/)")
    ap.add_argument("--runs-dir", help="run folders with <output>.tum (default: runs/)")
    ap.add_argument("--gt-dir", help="ground truth folders gt/<seq>/ (default: gt/)")
    ap.add_argument("--configs-dir", help="configs/ with <method>/sensitivity.yaml (default: configs/)")
    ap.add_argument("--env-dir", help="env/ with src_pins_*.json, images.json, machine.json (default: env/)")
    ap.add_argument("--out-dir", help=out_help)


def context_from_args(args, default_out):
    return Context(results_dir=args.results_dir, runs_dir=args.runs_dir, gt_dir=args.gt_dir,
                   configs_dir=args.configs_dir, env_dir=args.env_dir, out_dir=args.out_dir or default_out)


class Context:
    """Paths, configuration and cached pipeline outputs for one figure/table build."""

    def __init__(self, root=ROOT, results_dir=None, runs_dir=None, gt_dir=None, configs_dir=None,
                 env_dir=None, out_dir=None, sequences_file=None, eval_cfg_file=None, outputs_file=None):
        root = Path(root)
        self.root = root
        self.results_dir = Path(results_dir or root / "eval")
        self.runs_dir = Path(runs_dir or root / "runs")
        self.gt_dir = Path(gt_dir or root / "gt")
        self.configs_dir = Path(configs_dir or root / "configs")
        self.env_dir = Path(env_dir or root / "env")
        self.out_dir = Path(out_dir or root / "figs")
        self.sequences_file = Path(sequences_file or root / "datasets" / "sequences.yaml")
        self.eval_cfg_file = Path(eval_cfg_file or root / "eval" / "config" / "eval.yaml")
        self.outputs_file = Path(outputs_file or root / "eval" / "config" / "outputs.yaml")
        self.notes = set()          # printed by the scripts after the build
        self._tables, self._gt, self._pe = {}, {}, {}

    # -------------------------------------------------- configuration
    @cached_property
    def seqs(self):
        return yaml.safe_load(self.sequences_file.read_text())["sequences"]

    @cached_property
    def cfg(self):
        return yaml.safe_load(self.eval_cfg_file.read_text())

    @cached_property
    def out_cfg(self):
        return yaml.safe_load(self.outputs_file.read_text())

    def enabled_seqs(self):
        return [s for s, v in self.seqs.items() if v.get("enabled", True)]

    def test_seqs(self):
        """Test sequences in registry order (CitrusFarm test sequences, then Pullally)."""
        return [s for s in self.enabled_seqs() if self.seqs[s]["role"] == "test"]

    @staticmethod
    def seq_label(seq):
        return SEQ_LABEL.get(seq, seq)

    def lever_vec(self, seq):
        return self.seqs[seq].get("lever_arm_lidar_to_antenna_m")

    def lever(self, seq):
        """Main lever-arm setting of seq ('with' or 'without'), from outputs.yaml."""
        setting = self.out_cfg.get("main_lever_arm", {}).get(self.seqs[seq]["dataset"], "without")
        return "with" if setting == "with" and self.lever_vec(seq) is not None else "without"

    def headline(self, method):
        return self.out_cfg.get("headline", {}).get(method, "full")

    # -------------------------------------------------- pipeline tables
    def table(self, name):
        """A results table as a DataFrame, or None when missing or empty."""
        if name not in self._tables:
            f = self.results_dir / f"{name}.csv"
            df = None
            if f.exists():
                try:
                    df = pd.read_csv(f)
                except pd.errors.EmptyDataError:
                    df = None
                if df is not None and df.empty:
                    df = None
            if df is not None:
                for c in ("method", "seq", "config", "output"):
                    if c in df.columns:
                        df[c] = df[c].astype(str)
            self._tables[name] = df
        return self._tables[name]

    @cached_property
    def groups(self):
        """(method, output) -> 'fe' | 'full' from run_diagnostics.csv."""
        g = {}
        diag = self.table("run_diagnostics")
        if diag is not None and "group" in diag.columns:
            for (m, o), x in diag.groupby(["method", "output"]):
                v = x["group"].dropna()
                if len(v):
                    g[(m, o)] = str(v.iloc[0])
        return g

    def group(self, method, output):
        return self.groups.get((method, output), "fe" if str(output).startswith("fe") else "full")

    def outputs(self, method):
        """Evaluated outputs of a method: headline first, then front end, then the rest."""
        found = set()
        for name in ("results_long", "run_diagnostics"):
            df = self.table(name)
            if df is not None:
                found |= set(df.loc[df["method"] == method, "output"].dropna())
        hl = self.headline(method)
        return sorted(found, key=lambda o: (o != hl, self.group(method, o) != "fe", o))

    def fe_output(self, method):
        fe = [o for o in self.outputs(method) if self.group(method, o) == "fe"]
        return fe[0] if fe else None

    def output_label(self, method, output):
        lab = OUTPUT_LABEL.get(output, str(output).replace("_", " "))
        return lab + (" (headline)" if output == self.headline(method) else "")

    # -------------------------------------------------- accuracy
    @cached_property
    def _acc(self):
        idx = {}
        df = self.table("results_long")
        if df is not None:
            keys = ["method", "seq", "config", "output", "alignment", "lever_arm", "dim", "metric"]
            for k, g in df.groupby(keys, sort=False):
                idx[k] = {int(r): float(v) for r, v in zip(g["rep"], g["value"]) if np.isfinite(v)}
        return idx

    @cached_property
    def _attempts(self):
        """(method, seq, config, output) -> repetitions with a diagnostics row (any status)."""
        idx = {}
        diag = self.table("run_diagnostics")
        if diag is not None:
            for k, g in diag.groupby(["method", "seq", "config", "output"]):
                idx[k] = {int(r) for r in g["rep"]}
        return idx

    def configs(self, method, seq):
        out = set()
        for name in ("results_long", "run_diagnostics"):
            df = self.table(name)
            if df is not None:
                out |= set(df.loc[(df["method"] == method) & (df["seq"] == seq), "config"])
        return out

    def accuracy(self, method, seq, config, output, metric="rmse", dim="3d", alignment="A", lever=None):
        lever = lever or self.lever(seq)
        vals = self._acc.get((method, seq, config, output, alignment, lever, dim, metric), {})
        if not vals and lever == "with":
            alt = self._acc.get((method, seq, config, output, alignment, "without", dim, metric), {})
            if alt:   # output without orientation: evaluate_output gives only 'without'
                self.notes.add(f"{method} {seq} {config} {output}: no lever-arm variant, "
                               "values without lever arm used")
                vals, lever = alt, "without"
        reps = self._attempts.get((method, seq, config, output), set()) | set(vals)
        return Agg(values=dict(vals), n_total=len(reps), lever=lever)

    def flags(self, method, seq, config, output):
        """Counts of runs with coverage below the threshold, divergence or no output."""
        diag = self.table("run_diagnostics")
        out = {"coverage": 0, "diverged": 0, "no_output": 0, "run_failed": 0}
        if diag is None:
            return out
        d = diag[(diag["method"] == method) & (diag["seq"] == seq) & (diag["config"] == config) &
                 (diag["output"] == output)]
        st = d["status"].astype(str) if "status" in d.columns else pd.Series(dtype=str)
        out["run_failed"] = int((st == "run_failed").sum())
        out["no_output"] = int(st.isin(["no_output", "no_overlap"]).sum())
        ok = d[st == "ok"]
        if "coverage_ok" in ok.columns:
            out["coverage"] = int((ok["coverage_ok"].astype(str).str.lower() == "false").sum())
        if "diverged" in ok.columns:
            out["diverged"] = int((ok["diverged"].astype(str).str.lower() == "true").sum())
        return out

    def representative_rep(self, method, seq, config, output):
        """Repetition with the median RMSE (alignment A, 3D, main lever arm); with
        an even count the lower of the two middle ones. None if no run has a value."""
        a = self.accuracy(method, seq, config, output)
        if not a.n_ok:
            return None, a
        reps = sorted(a.values, key=lambda r: (a.values[r], r))
        return reps[(len(reps) - 1) // 2], a

    # -------------------------------------------------- resources
    def _resource_rows(self, method, seq, config):
        df = self.table("resources_summary")
        if df is None:
            return None
        return df[(df["method"] == method) & (df["seq"] == seq) & (df["config"] == config)]

    def resource(self, method, seq, config, column, scale=1.0):
        """Agg over repetitions; runs whose status is not ok count as failures;
        an ok run without this measurement is left out and noted."""
        rows = self._resource_rows(method, seq, config)
        if rows is None or rows.empty or column is None or column not in rows.columns:
            return Agg()
        ok = rows["status"].astype(str) == "ok"
        vals = {}
        for r, v in zip(rows.loc[ok, "rep"], rows.loc[ok, column]):
            v = pd.to_numeric(v, errors="coerce")
            if np.isfinite(v):
                vals[int(r)] = float(v) * scale
            else:
                self.notes.add(f"{column} not available: {method} {seq} {config} rep{int(r)}")
        return Agg(values=vals, n_total=len(vals) + int((~ok).sum()))

    def run_count(self, method, seq, config):
        rows = self._resource_rows(method, seq, config)
        if rows is None or rows.empty:
            return 0, 0
        return int((rows["status"].astype(str) == "ok").sum()), len(rows)

    def realtime_column(self, method, stat="processed_scan_ratio"):
        df = self.table("resources_summary")
        if df is None:
            return None
        rows = df[df["method"] == method]
        suffix = ":" + stat
        cand = [c[: -len(suffix)] for c in df.columns if c.endswith(suffix) and rows[c].notna().any()]
        pref = REALTIME_TOPIC.get(method)
        if pref in cand:
            topic = pref
        elif len(cand) == 1:
            topic = cand[0]
        else:
            same = [t for t in cand if pref and t.rsplit("/", 1)[-1] == pref.rsplit("/", 1)[-1]]
            topic = same[0] if len(same) == 1 else None
        if topic is None:
            if cand:
                self.notes.add(f"{method}: real-time topic ambiguous among {cand}; reported as N/A")
            return None
        if topic != pref:
            self.notes.add(f"{method}: real-time metrics from {topic} (expected {pref})")
        return topic + suffix

    # -------------------------------------------------- ground truth and trajectories
    def gt_file(self, seq):
        return self.gt_dir / seq / "gt_tum.txt"

    def gt(self, seq):
        if seq not in self._gt:
            self._gt[seq] = prepare_ground_truth(read_tum(self.gt_file(seq)),
                                                 self.seqs[seq]["gt_nominal_period_s"], self.cfg)
        return self._gt[seq]

    def quality(self, seq):
        f = self.gt_dir / seq / "gt_quality.json"
        return json.loads(f.read_text()) if f.exists() else None

    def tum_path(self, method, seq, config, rep, output):
        return self.runs_dir / method / seq / config / f"rep{int(rep)}" / f"{output}.tum"

    def pose_errors(self, method, seq, config, rep, output, alignment="A", lever=None):
        """Per-pose errors of one run, recomputed from <run>/<output>.tum with the
        same lever-arm setting as the tabulated values (accuracy())."""
        lever = lever or self.accuracy(method, seq, config, output, alignment=alignment).lever
        key = (method, seq, config, int(rep), output, alignment, lever)
        if key not in self._pe:
            f = self.tum_path(method, seq, config, rep, output)
            pe = None
            if f.exists() and self.gt_file(seq).exists():
                est = read_tum(f)
                pe = compute_pose_errors(est, self.gt(seq), self.cfg,
                                         self.lever_vec(seq) if lever == "with" else None, alignment)
            elif not f.exists():
                self.notes.add(f"missing trajectory {f}")
            self._pe[key] = pe
        return self._pe[key]

    # -------------------------------------------------- sensitivity design
    @cached_property
    def designs(self):
        out = {}
        for m in METHODS:
            f = self.configs_dir / m / "sensitivity.yaml"
            if f.exists():
                out[m] = yaml.safe_load(f.read_text())
        return out

    @cached_property
    def design_reasons(self):
        f = self.configs_dir / "SENSITIVITY_DESIGN.md"
        return parse_design_table(f.read_text()) if f.exists() else {}

    def sensitivity_seq(self, method=None):
        d = self.designs.get(method) if method else next(iter(self.designs.values()), None)
        return (d or {}).get("sequence", "cf_5")

    def sensitivity_levels(self, method):
        """[(param, value, config name)] in design order. Without a design file,
        the OAT_ configurations found in the results are used."""
        d = self.designs.get(method)
        if d:
            return [(p, v, oat_name(p["id"], v)) for p in d["parameters"] for v in p["levels"]]
        seq = self.sensitivity_seq(method)
        out = []
        for c in sorted(self.configs(method, seq)):
            pv = parse_oat(c)
            if pv:
                out.append(({"id": pv[0], "name": pv[0], "unit": ""}, pv[1], c))
        return out
