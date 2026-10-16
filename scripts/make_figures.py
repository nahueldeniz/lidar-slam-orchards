#!/usr/bin/env python3
"""Figures F0-F11 (brief Section 13), vector PDF plus PNG, into figs/.

Each figure function reads only pipeline outputs (gt/, eval/*.csv, runs/*.tum,
configs/*/sensitivity.yaml). A figure whose inputs do not exist yet is skipped
with a message; it is never drawn from placeholder data. Values that do not
exist (no run, failed run, no heading reference) are shown as "N/A" or
"failed", never interpolated.

Conventions (eval/slameval/style.py): one fixed colour, marker and line style
per method in a fixed order; ground truth in neutral grey; small multiples for
trajectory overlays; one y-axis per panel; text in neutral ink; units on every
axis; British English. Unless stated otherwise every figure shows
configuration D, the headline output of each method (eval/config/outputs.yaml,
the single switch), alignment A and the main lever-arm setting per dataset.

  F0  ground-truth trajectories per sequence
  F1  top-view trajectories, one panel per method over the ground truth, per
      sequence (repetition with the median RMSE); supplementary: front end vs full
  F2  3D position error vs distance travelled, turns shaded (median-RMSE repetition)
  F3  box plots of the 3D position error per pose (all successful repetitions pooled)
  F4  RMSE matrix, mean ± SD over repetitions; supplementary: with front-end rows
  F5  3D vs horizontal vs vertical RMSE
  F6  CPU, peak RSS, incremental package power, processed-scan ratio
  F7  hdl with vs without floor detection
  F8  segment RMSE in rows, mixed and turn segments, L = 10, 50, 100 m
  F9  ground-truth quality (rate, RTK fixed, ENU consistency)
  F10 heading error, all samples vs row samples
  F11 one-at-a-time sensitivity on cf_5: ΔRMSE vs D (supplementary: ΔCPU, ΔRSS)

Usage: make_figures.py [F1 F4 ...] [--results-dir eval] [--runs-dir runs]
       [--gt-dir gt] [--configs-dir configs] [--out-dir figs]
"""
import argparse
import sys
import textwrap
from pathlib import Path

import matplotlib.ticker
import numpy as np

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R / "eval"))
from slameval import report, style  # noqa: E402
from slameval.report import Agg, fmt, tint  # noqa: E402

plt = style.plt
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch, Rectangle  # noqa: E402

SHORT_SEQ = {"cf_2": "CF\n02", "cf_3": "CF\n03", "cf_6": "CF\n06", "cf_7": "CF\n07", "cf_5": "CF\n05", "pl_1": "PL\n1"}
FIGS = R / "figs"
METHODS = style.METHODS
LABEL = style.LABEL
NA_FILL = "#f0efec"        # neutral cell for N/A (diverging midpoint grey of the palette)
SEQ_RAMP = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]  # blue 100..700
BAND = "#d9d8d3"


def save(ctx, fig, name):
    style.save(fig, ctx.out_dir, name)
    return name


def na_bar(ax, x, text):
    """Explicit N/A at a bar position (axis-fraction height, independent of the data scale)."""
    ax.text(x, 0.02, text, ha="center", va="bottom", rotation=90, fontsize=6.5,
            color=style.INK_SECONDARY, transform=ax.get_xaxis_transform())


def status_word(a: Agg):
    return "N/A" if a.status == "N/A" else f"failed (0/{a.n_total})" if a.status == "failed" else ""


def panel_grid(n, ncols, w=7.2, h_panel=2.2, extra=0.0, **kw):
    nrows = int(np.ceil(n / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(w, h_panel * nrows + extra), squeeze=False, **kw)
    return fig, axes


def method_legend(fig, y=1.0, extra=(), ncol=None, kind="patch"):
    if kind == "line":
        h = [Line2D([], [], color=style.COLOUR[m], ls=style.LINESTYLE[m], lw=1.4, label=LABEL[m]) for m in METHODS]
    else:
        h = [Patch(facecolor=style.COLOUR[m], edgecolor=style.COLOUR[m], label=LABEL[m]) for m in METHODS]
    h += list(extra)
    fig.legend(handles=h, loc="upper center", bbox_to_anchor=(0.5, y), ncol=ncol or min(len(h), 7),
               frameon=False, handlelength=2.0, columnspacing=1.2)


def legend_above(fig, handles, title=None, ncol=None, pad=0.008):
    """Figure legend placed just above the highest visible axes (after layout);
    style.save crops the unused space (savefig.bbox = tight)."""
    fig.canvas.draw()
    r = fig.canvas.get_renderer()
    inv = fig.transFigure.inverted()
    top = max(inv.transform(ax.get_tightbbox(r))[1, 1] for ax in fig.axes if ax.get_visible())
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(0.5, top + pad), ncol=ncol or len(handles),
               frameon=False, title=title, title_fontsize=9)


def seq_ticks(ax, seqs, rotation=0, wrap=False):
    ax.set_xticks(range(len(seqs)))
    labels = [report.SEQ_LABEL.get(s, s).replace(" (tuning)", "") for s in seqs]
    if wrap:
        labels = [lab.replace(" ", "\n", 1) for lab in labels]
    ax.set_xticklabels(labels, rotation=rotation, ha="right" if rotation else "center")


def method_ticks(ax, rotation=30):
    ax.set_xticks(range(len(METHODS)))
    ax.set_xticklabels([LABEL[m] for m in METHODS], rotation=rotation, ha="right" if rotation else "center")
    ax.set_xlim(-0.6, len(METHODS) - 0.4)
    ax.grid(axis="x", visible=False)


# ---------------------------------------------------------------- F0
def f0_ground_truth(ctx):
    """F0: ground-truth trajectories per sequence (replaces the old Fig. 2)."""
    seqs = [s for s in ctx.test_seqs() if ctx.gt_file(s).exists()]
    if not seqs:
        return "skipped: no ground truth yet"
    n = len(seqs)
    cols = min(n, 2)
    rows = int(np.ceil(n / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(3.4 * cols, 3.2 * rows), squeeze=False)
    for ax, s in zip(axes.flat, seqs):
        gt = ctx.gt(s).traj
        q = ctx.quality(s) or {}
        ax.plot(gt.p[:, 0], gt.p[:, 1], color=style.INK_SECONDARY, lw=0.9)
        ax.plot(*gt.p[0, :2], "o", ms=6, mfc="white", mec=style.INK, label="Start")
        ax.plot(*gt.p[-1, :2], "s", ms=6, mfc=style.INK, mec=style.INK, label="End")
        ax.set_aspect("equal", adjustable="datalim")
        length = q.get("path_length_3d_all_samples_m")
        ax.set_title(ctx.seq_label(s) + (f" ({length:.0f} m)" if length else ""))
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
    for ax in list(axes.flat)[n:]:
        ax.set_visible(False)
    axes.flat[0].legend(loc="best", frameon=False)
    fig.tight_layout()
    save(ctx, fig, "F0_ground_truth")
    return "ok"


# ---------------------------------------------------------------- F1
def _limits(xy):
    lo, hi = xy.min(axis=0), xy.max(axis=0)
    pad = max(0.08 * float((hi - lo).max()), 3.0)
    return (lo[0] - pad, hi[0] + pad), (lo[1] - pad, hi[1] + pad)


def _trajectory_figure(ctx, seq, panels, legend_extra=(), layout=None):
    """panels: [(title, [(xy, colour, linestyle, lw)], note)] drawn over the ground truth.
    layout: (rows, columns); default chosen from the number of panels and the aspect
    ratio of the ground truth (the paper's figures use the default)."""
    g = ctx.gt(seq).p[:, :2]
    xl, yl = _limits(g)
    aspect = (yl[1] - yl[0]) / (xl[1] - xl[0])
    n = len(panels)
    if layout:
        nr, nc = layout
    elif n <= 3 or aspect >= 1.6:
        nr, nc = 1, n
    elif aspect >= 0.6:
        nr, nc = 2, int(np.ceil(n / 2))
    else:
        nr, nc = int(np.ceil(n / 2)), 2
    pw = 7.2 / nc
    ph = float(np.clip(pw * aspect, 1.2, min(8.0 / nr, 5.0))) + 0.55
    fig, axes = plt.subplots(nr, nc, figsize=(7.2, ph * nr + 0.7), sharex=True, sharey=True, squeeze=False)
    gs = report.stride(g)
    for ax, (title, series, note) in zip(axes.flat, panels):
        ax.plot(gs[:, 0], gs[:, 1], color=style.GT_COLOUR, lw=2.0, alpha=0.55, solid_capstyle="round", zorder=1)
        outside = 0.0
        for xy, col, ls, lw in series:
            xy = report.stride(xy)
            ax.plot(xy[:, 0], xy[:, 1], color=col, ls=ls, lw=lw, zorder=2)
            out = (xy[:, 0] < xl[0]) | (xy[:, 0] > xl[1]) | (xy[:, 1] < yl[0]) | (xy[:, 1] > yl[1])
            outside = max(outside, float(out.mean()))
        ax.plot(*g[0], "o", ms=5, mfc="white", mec=style.INK, zorder=3)
        ax.plot(*g[-1], "s", ms=4.5, mfc=style.INK, mec=style.INK, zorder=3)
        ax.set_xlim(xl)
        ax.set_ylim(yl)
        ax.set_aspect("equal", adjustable="box")
        ax.set_title(title, fontsize=8)
        ax.xaxis.set_major_locator(plt.MaxNLocator(3))
        if outside > 0.01:
            note = (note + "\n" if note else "") + f"{100 * outside:.0f} % of poses outside the plot"
        if note:
            ax.text(0.5, 0.5, note, transform=ax.transAxes, ha="center", va="center", fontsize=7,
                    color=style.INK_SECONDARY, bbox=dict(boxstyle="round", fc="white", ec="none", alpha=0.85))
    for ax in list(axes.flat)[n:]:
        ax.set_visible(False)
    for ax in axes[-1, :]:
        ax.set_xlabel("x (m)")
    for ax in axes[:, 0]:
        ax.set_ylabel("y (m)")
    h = [Line2D([], [], color=style.GT_COLOUR, lw=2.0, alpha=0.55, label="Ground truth"),
         Line2D([], [], ls="", marker="o", ms=5, mfc="white", mec=style.INK, label="Start"),
         Line2D([], [], ls="", marker="s", ms=4.5, mfc=style.INK, mec=style.INK, label="End")] + list(legend_extra)
    fig.tight_layout()
    legend_above(fig, h, title=ctx.seq_label(seq))
    return fig


def f1_trajectories(ctx):
    """F1: one panel per method (headline output, configuration D, alignment A,
    repetition with the median RMSE) over the grey ground truth, per sequence.
    Supplementary F1_supp_fe_vs_full_<seq>: front end (dashed) vs full output
    (solid) of the same repetition, for methods with a separate front end."""
    if ctx.table("results_long") is None:
        return "skipped: no results_long.csv"
    seqs = [s for s in ctx.test_seqs() if ctx.gt_file(s).exists()]
    made = []
    for s in seqs:
        panels, supp, any_data = [], [], False
        for m in METHODS:
            hl = ctx.headline(m)
            rep, a = ctx.representative_rep(m, s, "D", hl)
            pe = ctx.pose_errors(m, s, "D", rep, hl) if rep is not None else None
            if pe is None:
                note = status_word(a) or "N/A"
                if rep is not None:
                    note = "N/A (trajectory file missing)"
                panels.append((LABEL[m], [], note))
                continue
            any_data = True
            panels.append((f"{LABEL[m]}\nRMSE {fmt(pe.rmse)} m\n(rep {rep})",
                           [(pe.p_all[:, :2], style.COLOUR[m], "-", 0.9)], ""))
            fe = ctx.fe_output(m)
            if fe and fe != hl:
                pf = ctx.pose_errors(m, s, "D", rep, fe)
                series = [(pe.p_all[:, :2], style.COLOUR[m], "-", 0.9)]
                title = f"{LABEL[m]} (rep {rep})\nRMSE full {fmt(pe.rmse)} m"
                if pf is not None:
                    series.insert(0, (pf.p_all[:, :2], tint(style.COLOUR[m], 0.75), (0, (3, 1.5)), 0.9))
                    title += f"\nfront end {fmt(pf.rmse)} m"
                else:
                    title += "\nfront end N/A"
                supp.append((title, series, ""))
        if not any_data:
            print(f"  F1 {s}: no trajectories")
            continue
        made.append(save(ctx, _trajectory_figure(ctx, s, panels), f"F1_trajectories_{s}"))
        if supp:
            extra = [Line2D([], [], color=style.INK_SECONDARY, lw=0.9, label="Full output (headline)"),
                     Line2D([], [], color=tint(style.INK_SECONDARY, 0.75), lw=0.9, ls=(0, (3, 1.5)),
                            label="Front end")]
            made.append(save(ctx, _trajectory_figure(ctx, s, supp, extra), f"F1_supp_fe_vs_full_{s}"))
    return f"ok ({len(made)} files)" if made else "skipped: no trajectories (<run>/<output>.tum)"


def f1_composite(ctx, width_in=5.1, layout=(("cf_2",), ("cf_3",), ("cf_6", "cf_7")), scale_bar_m=20.0):
    """F1_trajectories_composite: the CitrusFarm panels of F1 on one page at the
    final printed size (width_in = text width of the journal class). Every
    sequence is rotated by -90 degrees, (x, y) -> (y, -x), so the rows run
    horizontally; a rotation, never a reflection, so the shape is unchanged.
    Each sequence is a block of method panels at one common scale (repetition
    with the median RMSE), with a scale bar instead of repeated axis ticks.
    layout: groups of sequences placed side by side (3 panel columns for a
    full-width block, 2 for a shared row)."""
    if ctx.table("results_long") is None:
        return "skipped: no results_long.csv"
    have = {s for s in ctx.test_seqs() if ctx.gt_file(s).exists()}
    layout = [tuple(s for s in grp if s in have) for grp in layout]
    layout = [grp for grp in layout if grp]
    if not layout:
        return "skipped: no CitrusFarm ground truth"
    rot = lambda xy: np.column_stack([xy[:, 1], -xy[:, 0]])  # noqa: E731
    gap_x, group_gap, title_h, head_h, row_gap, block_gap, legend_h, pad = 0.07, 0.22, 0.15, 0.18, 0.03, 0.10, 0.22, 3.5
    groups = []                      # [(height, [(seq, g, lo, hi, x0, pw, ph, ncol)])]
    for grp in layout:
        ncol = 3 if len(grp) == 1 else 2
        gw = (width_in - (len(grp) - 1) * group_gap) / len(grp)
        pw = (gw - (ncol - 1) * gap_x) / ncol
        blocks, gh = [], 0.0
        for j, s in enumerate(grp):
            g = rot(ctx.gt(s).p[:, :2])
            lo, hi = g.min(axis=0) - pad, g.max(axis=0) + pad
            ph = pw * (hi[1] - lo[1]) / (hi[0] - lo[0])
            nrow = int(np.ceil(len(METHODS) / ncol))
            gh = max(gh, head_h + nrow * (title_h + ph) + (nrow - 1) * row_gap)
            blocks.append((s, g, lo, hi, j * (gw + group_gap), pw, ph, ncol))
        groups.append((gh, gw, blocks))
    height = legend_h + sum(g[0] for g in groups) + block_gap * (len(groups) - 1)
    fig = plt.figure(figsize=(width_in, height))
    fx = lambda v: v / width_in  # noqa: E731
    fy = lambda v: 1 - v / height  # top-down inches -> figure fraction  # noqa: E731
    y, k = legend_h, 0
    for gh, gw, blocks in groups:
        for s, g, lo, hi, x0, pw, ph, ncol in blocks:
            yh = fy(y + head_h * 0.45)
            fig.text(fx(x0), yh, f"({chr(97 + k)}) {ctx.seq_label(s)}", fontsize=8, fontweight="bold",
                     va="center", ha="left")
            k += 1
            bar, x1 = scale_bar_m * pw / (hi[0] - lo[0]), x0 + gw
            fig.add_artist(Line2D([fx(x1 - bar), fx(x1)], [yh] * 2, color=style.INK, lw=1.2, solid_capstyle="butt"))
            fig.text(fx(x1 - bar - 0.05), yh, f"{scale_bar_m:.0f} m", fontsize=7, va="center", ha="right")
            gs = report.stride(g)
            for i, m in enumerate(METHODS):
                r, c = divmod(i, ncol)
                top = y + head_h + r * (title_h + ph + row_gap) + title_h
                ax = fig.add_axes([fx(x0 + c * (pw + gap_x)), fy(top + ph), fx(pw), ph / height])
                ax.plot(gs[:, 0], gs[:, 1], color=style.GT_COLOUR, lw=1.6, alpha=0.55, solid_capstyle="round",
                        zorder=1)
                hl = ctx.headline(m)
                rep, a = ctx.representative_rep(m, s, "D", hl)
                pe = ctx.pose_errors(m, s, "D", rep, hl) if rep is not None else None
                if pe is None:
                    title = f"{LABEL[m]}: " + (status_word(a) or "N/A")
                else:
                    xy = report.stride(rot(pe.p_all[:, :2]))
                    ax.plot(xy[:, 0], xy[:, 1], color=style.COLOUR[m], ls="-", lw=0.7, zorder=2)
                    title = f"{LABEL[m]}  {fmt(pe.rmse)} m"
                    out = (xy[:, 0] < lo[0]) | (xy[:, 0] > hi[0]) | (xy[:, 1] < lo[1]) | (xy[:, 1] > hi[1])
                    if out.mean() > 0.01:
                        ax.text(0.99, 0.04, f"{100 * out.mean():.0f} % of poses outside", transform=ax.transAxes,
                                ha="right", va="bottom", fontsize=5.5, color=style.INK_SECONDARY, zorder=4,
                                bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.85))
                ax.plot(*g[0], "o", ms=3.5, mfc="white", mec=style.INK, mew=0.7, zorder=3)
                ax.plot(*g[-1], "s", ms=3.0, mfc=style.INK, mec=style.INK, zorder=3)
                ax.set_xlim(lo[0], hi[0])
                ax.set_ylim(lo[1], hi[1])
                ax.set_xticks([])
                ax.set_yticks([])
                ax.grid(False)
                for sp in ax.spines.values():
                    sp.set_visible(True)
                    sp.set_color(style.GRID)
                    sp.set_linewidth(0.6)
                ax.set_title(title, fontsize=7, loc="left", pad=1.5)
        y += gh + block_gap
    h = [Line2D([], [], color=style.GT_COLOUR, lw=1.6, alpha=0.55, label="Ground truth (RTK-GNSS)"),
         Line2D([], [], color=style.INK_SECONDARY, lw=0.7, label="Estimate (colour per method)"),
         Line2D([], [], ls="", marker="o", ms=3.5, mfc="white", mec=style.INK, label="Start"),
         Line2D([], [], ls="", marker="s", ms=3.0, mfc=style.INK, mec=style.INK, label="End")]
    fig.legend(handles=h, loc="center", bbox_to_anchor=(0.5, fy(legend_h * 0.4)), ncol=4, frameon=False, fontsize=7,
               handlelength=1.8, columnspacing=1.2)
    save(ctx, fig, "F1_trajectories_composite")
    return f"ok ({width_in:.2f} x {height:.2f} in)"


# ---------------------------------------------------------------- F2
def f2_error_vs_distance(ctx, scale="auto"):
    """F2: 3D position error vs ground-truth distance travelled, one panel per
    sequence, one line per method (repetition with the median RMSE); turns
    (ground-truth labels, eval.yaml 'turn') shaded. Logarithmic y-axis when the
    errors span more than a factor of 30 (scale='auto')."""
    if ctx.table("results_long") is None:
        return "skipped: no results_long.csv"
    seqs = [s for s in ctx.test_seqs() if ctx.gt_file(s).exists()]
    data = {}
    for s in seqs:
        for m in METHODS:
            hl = ctx.headline(m)
            rep, a = ctx.representative_rep(m, s, "D", hl)
            pe = ctx.pose_errors(m, s, "D", rep, hl) if rep is not None else None
            data[(s, m)] = (pe, a)
    if not any(pe is not None for pe, _ in data.values()):
        return "skipped: no trajectories (<run>/<output>.tum)"
    errs = [pe.err["3d"] for pe, _ in data.values() if pe is not None]
    lo = min(np.median(e) for e in errs)
    hi = max(np.percentile(e, 99) for e in errs)
    log = scale == "log" or (scale == "auto" and hi > 30 * max(lo, 1e-3))
    fig, axes = plt.subplots(len(seqs), 1, figsize=(7.2, 1.75 * len(seqs) + 0.6), squeeze=False)
    for ax, s in zip(axes[:, 0], seqs):
        gt = ctx.gt(s)
        for a0, a1 in report.turn_intervals(gt):
            ax.axvspan(a0, a1, color=BAND, alpha=0.6, lw=0, zorder=0)
        na = []
        for m in METHODS:
            pe, a = data[(s, m)]
            if pe is None:
                na.append(f"{LABEL[m]} {status_word(a) or 'N/A'}")
                continue
            x, y = report.decimate_minmax(pe.s, pe.err["3d"])
            if log:
                y = np.maximum(y, 1e-3)
            ax.plot(x, y, color=style.COLOUR[m], ls=style.LINESTYLE[m], lw=0.9, zorder=2)
        ax.set_xlim(0, gt.arc.total)
        if log:
            ax.set_yscale("log")
        else:
            ax.set_ylim(bottom=0)
        ax.set_title(ctx.seq_label(s), loc="left", fontsize=8.5)
        ax.set_ylabel("Position error (m)")
        if na:
            ax.text(1.0, 1.02, "Not shown: " + "; ".join(na), transform=ax.transAxes, ha="right", va="bottom",
                    fontsize=6.5, color=style.INK_SECONDARY)
    axes[-1, 0].set_xlabel("Distance travelled (m)")
    method_legend(fig, kind="line", extra=[Patch(color=BAND, alpha=0.6, label="Turn (ground truth)")], ncol=7)
    fig.tight_layout(rect=(0, 0, 1, 1 - 0.35 / fig.get_size_inches()[1]))
    if log:
        fig.text(0.005, 0.005, "Logarithmic error axis.", fontsize=6.5, color=style.INK_SECONDARY)
    save(ctx, fig, "F2_error_vs_distance")
    return "ok" + (" (log scale)" if log else "")


# ---------------------------------------------------------------- F3
def f3_error_boxplots(ctx):
    """F3: distribution of the 3D position error per pose, per method, one
    panel per sequence; all successful repetitions pooled. Boxes: quartiles and
    median; whiskers 1.5 IQR; points beyond are drawn small. Points above the
    plotted range are summarised by their maximum."""
    if ctx.table("results_long") is None:
        return "skipped: no results_long.csv"
    seqs = [s for s in ctx.test_seqs() if ctx.gt_file(s).exists()]
    fig, axes = panel_grid(len(seqs), 2, h_panel=2.3)
    any_data = False
    for ax, s in zip(axes.flat, seqs):
        data, pos, cols = [], [], []
        for i, m in enumerate(METHODS):
            hl = ctx.headline(m)
            a = ctx.accuracy(m, s, "D", hl)
            pes = [ctx.pose_errors(m, s, "D", r, hl) for r in sorted(a.values)]
            pes = [p for p in pes if p is not None]
            if not pes:
                na_bar(ax, i, status_word(a) or "N/A")
                continue
            data.append(np.concatenate([p.err["3d"] for p in pes]))
            pos.append(i)
            cols.append(style.COLOUR[m])
        if data:
            any_data = True
            bp = ax.boxplot(data, positions=pos, widths=0.6, patch_artist=True, whis=1.5, showfliers=True,
                            medianprops=dict(color=style.INK, lw=1.2))
            for k, c in enumerate(cols):
                bp["boxes"][k].set(facecolor=tint(c, 0.35), edgecolor=c, lw=1.0)
                for part in ("whiskers", "caps"):
                    for line in bp[part][2 * k: 2 * k + 2]:
                        line.set(color=c, lw=1.0)
                bp["fliers"][k].set(marker=".", ms=1.5, mfc=c, mec=c, alpha=0.35, rasterized=True)
            top = max(float(w.get_ydata().max()) for w in bp["whiskers"]) * 1.25
            ax.set_ylim(0, top)
            for k, d in enumerate(data):
                if d.max() > top:
                    ax.text(pos[k], top, f"max {fmt(d.max())}", ha="center", va="bottom", fontsize=6,
                            color=style.INK_SECONDARY)
        else:
            ax.set_ylim(0, 1)
        ax.set_xlim(-0.6, len(METHODS) - 0.4)
        method_ticks(ax)
        ax.set_title(ctx.seq_label(s), fontsize=8.5)
        ax.set_ylabel("Position error (m)")
    for ax in list(axes.flat)[len(seqs):]:
        ax.set_visible(False)
    if not any_data:
        plt.close(fig)
        return "skipped: no trajectories (<run>/<output>.tum)"
    fig.tight_layout()
    save(ctx, fig, "F3_error_boxplots")
    return "ok"


# ---------------------------------------------------------------- F4
def _rmse_matrix(ctx, rows, seqs, name):
    """rows: [(method, output, label)]; cells: RMSE (alignment A, 3D) mean ± SD."""
    cells = {(i, j): ctx.accuracy(m, s, "D", o) for i, (m, o, _) in enumerate(rows) for j, s in enumerate(seqs)}
    means = [a.mean for a in cells.values() if a.n_ok]
    if not means:
        return None
    bounds = report.log_bins(min(means), max(means))
    ncls = len(bounds) - 1
    ramp = [SEQ_RAMP[int(round(k))] for k in np.linspace(0, len(SEQ_RAMP) - 1, ncls)]
    nr, nc = len(rows), len(seqs)
    fig = plt.figure(figsize=(7.2, 0.42 * nr + 1.3))
    ax = fig.add_axes([0.25, 0.04 + 0.5 / fig.get_size_inches()[1], 0.62, 1 - (0.5 + 0.75) / fig.get_size_inches()[1]])
    for (i, j), a in cells.items():
        y = nr - 1 - i
        if a.n_ok:
            k = int(np.clip(np.searchsorted(bounds, a.mean, side="right") - 1, 0, ncls - 1))
            fc, tc = ramp[k], ("white" if k >= ncls / 2 else style.INK)
            txt = a.text(counts=False)
            sub = f"{a.n_ok} of {a.n_total} run{'s' * (a.n_total > 1)}" if a.n_ok < a.n_total else \
                ("1 run" if a.n_ok == 1 else "")
        else:
            fc, tc = NA_FILL, style.INK_SECONDARY
            txt = "N/A" if a.status == "N/A" else "failed"
            sub = f"0 of {a.n_total} run{'s' * (a.n_total > 1)}" if a.status == "failed" else ""
        ax.add_patch(Rectangle((j, y), 1, 1, facecolor=fc, edgecolor="white", lw=2))
        ax.text(j + 0.5, y + (0.58 if sub else 0.5), txt, ha="center", va="center", fontsize=7.5, color=tc)
        if sub:
            ax.text(j + 0.5, y + 0.25, sub, ha="center", va="center", fontsize=6, color=tc)
    ax.set_xlim(0, nc)
    ax.set_ylim(0, nr)
    ax.set_xticks(np.arange(nc) + 0.5)
    ax.set_xticklabels([ctx.seq_label(s) for s in seqs])
    ax.xaxis.tick_top()
    ax.set_yticks(np.arange(nr) + 0.5)
    ax.set_yticklabels([lab for _, _, lab in rows][::-1])
    ax.tick_params(length=0)
    ax.grid(False)
    for sp in ax.spines.values():
        sp.set_visible(False)
    cax = fig.add_axes([0.89, ax.get_position().y0, 0.018, ax.get_position().height])
    from matplotlib.colors import BoundaryNorm, ListedColormap
    from matplotlib.colorbar import ColorbarBase
    cb = ColorbarBase(cax, cmap=ListedColormap(ramp), norm=BoundaryNorm(bounds, ncls), ticks=bounds,
                      spacing="uniform")
    cb.ax.set_yticklabels([f"{b:g}" for b in bounds])
    cb.set_label("RMSE (m)")
    cb.outline.set_visible(False)
    fig.text(0.25, ax.get_position().y0 - 0.12 / fig.get_size_inches()[1], "Mean ± SD over repetitions; "
             "alignment A, 3D error, configuration D.", fontsize=6.5, color=style.INK_SECONDARY, va="top")
    return save(ctx, fig, name)


def f4_rmse_matrix(ctx):
    """F4: RMSE matrix, rows = methods (headline output, configuration D),
    columns = test sequences incl. Pullally; discrete colour classes, values
    annotated. Supplementary: front-end rows added under each method."""
    if ctx.table("results_long") is None:
        return "skipped: no results_long.csv"
    seqs = ctx.test_seqs()
    rows = [(m, ctx.headline(m), LABEL[m]) for m in METHODS]
    if _rmse_matrix(ctx, rows, seqs, "F4_rmse_matrix") is None:
        return "skipped: no RMSE values"
    supp = []
    for m in METHODS:
        supp.append((m, ctx.headline(m), f"{LABEL[m]}, full"))
        fe = ctx.fe_output(m)
        if fe and fe != ctx.headline(m):
            supp.append((m, fe, f"{LABEL[m]}, front end"))
    _rmse_matrix(ctx, supp, seqs, "F4_supp_rmse_matrix_fe")
    return "ok"


# ---------------------------------------------------------------- grouped bars
def _bars(ax, x0, aggs, colour, width, fills, scale=1.0):
    """Adjacent bars at x0 (centred), one per Agg, filled with tints of colour."""
    k = len(aggs)
    xs = x0 + (np.arange(k) - (k - 1) / 2) * width
    if k > 1 and not any(a.n_ok for a in aggs):
        word = {status_word(a) or "N/A" for a in aggs}
        na_bar(ax, x0, word.pop() if len(word) == 1 else "N/A")
        return xs
    for x, a, f in zip(xs, aggs, fills):
        if a.n_ok:
            ax.bar(x, a.mean * scale, width=width * 0.92, color=tint(colour, f), edgecolor=colour, lw=0.7, zorder=2)
            if a.n_ok > 1:
                ax.errorbar(x, a.mean * scale, yerr=a.sd * scale, fmt="none", ecolor=style.INK_SECONDARY,
                            elinewidth=0.7, capsize=1.2, zorder=3)
        else:
            na_bar(ax, x, status_word(a) or "N/A")
    return xs


def f5_rmse_components(ctx):
    """F5: RMSE of the 3D, horizontal and vertical error per method and
    sequence (same runs; single y-axis per panel; mean ± SD over repetitions)."""
    if ctx.table("results_long") is None:
        return "skipped: no results_long.csv"
    seqs = ctx.test_seqs()
    fills = (1.0, 0.5, 0.18)
    fig, axes = panel_grid(len(seqs) + 1, 2, h_panel=2.2)
    for ax, s in zip(axes.flat, seqs):
        for i, m in enumerate(METHODS):
            aggs = [ctx.accuracy(m, s, "D", ctx.headline(m), dim=d) for d in ("3d", "2d", "z")]
            _bars(ax, i, aggs, style.COLOUR[m], 0.27, fills)
        ax.set_ylim(bottom=0)
        method_ticks(ax)
        ax.set_title(ctx.seq_label(s), fontsize=8.5)
        ax.set_ylabel("RMSE (m)")
    leg = list(axes.flat)[len(seqs)]
    leg.axis("off")
    leg.legend(handles=[Patch(facecolor=tint(style.INK_SECONDARY, f), edgecolor=style.INK_SECONDARY, label=lab)
                        for f, lab in zip(fills, ("3D", "Horizontal", "Vertical"))],
               loc="center", frameon=False, title="Error component\n(bar fill; hue = method)")
    for ax in list(axes.flat)[len(seqs) + 1:]:
        ax.set_visible(False)
    fig.tight_layout()
    save(ctx, fig, "F5_rmse_components")
    return "ok"


def f6_resources(ctx):
    """F6: CPU (mean, % of one logical core), peak RSS (GB), incremental
    package power (W, minus the infrastructure baseline) and processed-scan
    ratio (%) per method and sequence; mean ± SD over repetitions."""
    if ctx.table("resources_summary") is None:
        return "skipped: no resources_summary.csv"
    seqs = ctx.test_seqs()
    panels = [("cpu_mean_pct_one_core", "CPU (% of one core)", 1.0),
              ("rss_peak_gb", "Peak RSS (GB)", 1.0),
              ("power_incremental_w", "Incremental power (W)", 1.0),
              ("processed_scan_ratio", "Processed scans (%)", 100.0)]
    fig, axes = plt.subplots(4, 1, figsize=(7.2, 7.6), sharex=True, squeeze=False)
    w = 0.13
    for ax, (col, ylab, sc) in zip(axes[:, 0], panels):
        for i, m in enumerate(METHODS):
            c = ctx.realtime_column(m) if col == "processed_scan_ratio" else col
            for j, s in enumerate(seqs):
                a = ctx.resource(m, s, "D", c, scale=sc)
                _bars(ax, j + (i - 2.5) * w, [a], style.COLOUR[m], w, (1.0,))
        ax.axhline(0, color=style.INK_SECONDARY, lw=0.6)
        if col == "processed_scan_ratio":
            ax.set_ylim(0, 105)
        ax.set_ylabel(ylab)
        ax.grid(axis="x", visible=False)
    seq_ticks(axes[-1, 0], seqs)
    method_legend(fig, ncol=6)
    fig.tight_layout(rect=(0, 0, 1, 1 - 0.3 / 7.6))
    save(ctx, fig, "F6_resources")
    return "ok"


def f7_floor_ablation(ctx):
    """F7: hdl Graph SLAM RMSE with (D) and without (D_nofloor) floor
    detection, per sequence, for the full outputs (online and final keyframes)
    and the front end; bars = mean ± SD, dots = repetitions."""
    if ctx.table("results_long") is None:
        return "skipped: no results_long.csv"
    m = "hdl"
    outs = [o for o in ("full", "full_final") if o in ctx.outputs(m)]
    fe = ctx.fe_output(m)
    if fe:
        outs.append(fe)
    seqs = [s for s in ctx.test_seqs() if "D_nofloor" in ctx.configs(m, s)]
    if not outs or not seqs:
        return "skipped: no hdl D_nofloor results"
    fig, axes = plt.subplots(1, len(outs), figsize=(7.2, 2.6), squeeze=False)
    col = style.COLOUR[m]
    for ax, o in zip(axes[0], outs):
        for j, s in enumerate(seqs):
            aggs = [ctx.accuracy(m, s, c, o) for c in ("D", "D_nofloor")]
            xs = _bars(ax, j, aggs, col, 0.36, (1.0, 0.3))
            for x, a in zip(xs, aggs):
                if a.n_ok:
                    ax.plot(np.full(a.n_ok, x), a.arr, ".", ms=3, color=style.INK, zorder=4)
        ax.set_ylim(bottom=0)
        if all(ctx.seqs[s]["dataset"] == "citrusfarm" for s in seqs):
            ax.set_xticks(range(len(seqs)))
            ax.set_xticklabels([ctx.seq_label(s).split()[-1] for s in seqs])
            ax.set_xlabel("CitrusFarm sequence")
        else:
            seq_ticks(ax, seqs, rotation=45)
        ax.grid(axis="x", visible=False)
        ax.set_title(report.OUTPUT_LABEL.get(o, o).capitalize(), fontsize=8.5)
        ax.set_ylabel("RMSE (m)")
    h = [Patch(facecolor=col, edgecolor=col, label="With floor detection (D)"),
         Patch(facecolor=tint(col, 0.3), edgecolor=col, label="Without floor detection"),
         Line2D([], [], ls="", marker=".", ms=5, color=style.INK, label="Repetition")]
    fig.tight_layout()
    legend_above(fig, h)
    save(ctx, fig, "F7_floor_ablation")
    return "ok"


# ---------------------------------------------------------------- F8
def f8_segment_drift(ctx, metric="rmse_3d"):
    """F8: per-segment RMSE (each segment aligned on its own, Section 11.5) in
    row, mixed and turn segments, for L = 10, 50, 100 m; marker = median over
    the segments of all repetitions, whisker = interquartile range."""
    seg = ctx.table("segments")
    if seg is None:
        return "skipped: no segments.csv"
    seqs = [s for s in ctx.test_seqs() if (seg["seq"] == s).any()]
    if not seqs:
        return "skipped: no segments for the test sequences"
    Ls = sorted(seg["L_m"].dropna().unique())
    seg = seg[(seg["config"] == "D") & (seg["status"] == "ok")]
    classes = (("row", 1.0), ("mixed", 0.4), ("turn", 0.0))
    fig, axes = plt.subplots(len(seqs), len(Ls), figsize=(7.2, 1.65 * len(seqs) + 0.9), squeeze=False,
                             sharex=True)
    for r, s in enumerate(seqs):
        for c, L in enumerate(Ls):
            ax = axes[r, c]
            d0 = seg[(seg["seq"] == s) & (seg["L_m"] == L) & (seg["lever_arm"] == ctx.lever(s))]
            for i, m in enumerate(METHODS):
                dm = d0[(d0["method"] == m) & (d0["output"] == ctx.headline(m))]
                if dm.empty:
                    na_bar(ax, i, "N/A")
                    continue
                for k, (cl, f) in enumerate(classes):
                    v = dm.loc[dm["class"] == cl, metric].dropna().to_numpy()
                    if not len(v):
                        continue
                    x = i + (k - 1) * 0.24
                    q1, med, q3 = np.percentile(v, [25, 50, 75])
                    ax.plot([x, x], [q1, q3], color=style.COLOUR[m], lw=0.9, zorder=2)
                    ax.plot(x, med, marker=style.MARKER[m], ms=4, mfc=tint(style.COLOUR[m], f) if f else "white",
                            mec=style.COLOUR[m], mew=0.9, ls="", zorder=3)
            ax.set_ylim(bottom=0)
            ax.set_xlim(-0.6, len(METHODS) - 0.4)
            method_ticks(ax, rotation=45)
            if r == 0:
                ax.set_title(f"L = {L:g} m", fontsize=8.5)
            if c == 0:
                ax.set_ylabel(f"{ctx.seq_label(s)}\nSegment RMSE (m)", fontsize=8)
    h = [Line2D([], [], ls="", marker="o", ms=4.5, mec=style.INK_SECONDARY,
                mfc=tint(style.INK_SECONDARY, f) if f else "white", label=lab)
         for (cl, f), lab in zip(classes, ("Row", "Mixed (20–80 % turn)", "Turn"))]
    h.append(Line2D([], [], color=style.INK_SECONDARY, lw=0.9, label="Interquartile range"))
    fig.legend(handles=h, loc="upper center", bbox_to_anchor=(0.5, 1.0), ncol=4, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, 1 - 0.3 / fig.get_size_inches()[1]))
    save(ctx, fig, "F8_segment_drift")
    return "ok"


# ---------------------------------------------------------------- F9
def f9_ground_truth_quality(ctx):
    """F9: ground-truth quality per sequence from gt/<seq>/gt_quality.json:
    rate, RTK fixed epochs (receiver solution type; Pullally: NavSatFix status 2)
    and RMS difference between the ground truth and the raw RTK fixes in local
    ENU after a rigid fit (horizontal and vertical)."""
    seqs = [s for s in ctx.enabled_seqs() if ctx.quality(s)]
    if not seqs:
        return "skipped: no gt_quality.json"
    grey = style.GT_COLOUR
    fig, axes = plt.subplots(1, 3, figsize=(7.2, 2.5), squeeze=False)
    ax_rate, ax_fix, ax_enu = axes[0]
    starred = False
    for j, s in enumerate(seqs):
        q = ctx.quality(s)
        ax_rate.bar(j, q.get("median_rate_hz", np.nan), width=0.6, color=grey, zorder=2)
        rtk = q.get("rtk", {})
        fixed = None
        if rtk.get("status") == "ok":
            modes = (rtk.get("receiver_state") or {}).get("fix_mode_counts")
            if modes:
                fixed = 100.0 * modes.get("FIXED_RTK", 0) / sum(modes.values())
            elif "fix_status_percent" in rtk:
                fixed = rtk["fix_status_percent"].get("GBAS fix", 0.0)
                starred = True
        if fixed is None:
            na_bar(ax_fix, j, "N/A")
        else:
            ax_fix.bar(j, fixed, width=0.6, color=grey, zorder=2)
            ax_fix.text(j, fixed + 1, (f"{fixed:.1f}" if fixed < 99.95 else "100") +
                        ("*" if not (rtk.get("receiver_state") or {}) else ""),
                        ha="center", va="bottom", fontsize=6.5, color=style.INK)
        enu = rtk.get("enu_consistency")
        if enu:
            ax_enu.bar(j - 0.17, 100 * enu["rms_horizontal_m"], width=0.32, color=grey, zorder=2)
            ax_enu.bar(j + 0.17, 100 * enu["rms_vertical_m"], width=0.32, color=tint(grey, 0.35),
                       edgecolor=grey, lw=0.7, zorder=2)
        else:
            na_bar(ax_enu, j, "N/A")
    ax_rate.set_ylabel("Rate (Hz)")
    ax_fix.set_ylabel("RTK fixed (%)")
    ax_fix.set_ylim(0, 112)
    ax_enu.set_ylabel("RMS difference (cm)")
    ax_enu.legend(handles=[Patch(color=grey, label="Horizontal"),
                           Patch(facecolor=tint(grey, 0.35), edgecolor=grey, label="Vertical")],
                  frameon=False, loc="upper center", fontsize=7, ncol=2, handlelength=1.2, columnspacing=0.8)
    for ax, t in zip(axes[0], ("Rate", "Fixed solutions", "ENU consistency")):
        ax.set_xticks(range(len(seqs)))
        ax.set_xticklabels([SHORT_SEQ.get(q, q) for q in seqs], rotation=0, fontsize=7)
        ax.grid(axis="x", visible=False)
        ax.set_title(t, fontsize=8.5)
        ax.set_ylim(bottom=0)
    ax_enu.set_ylim(top=ax_enu.get_ylim()[1] * 1.3)
    fig.tight_layout()
    if starred:
        fig.text(0.01, -0.02, textwrap.fill("CF: CitrusFarm (CF 05: first 380 s, sensitivity sequence). PL 1: Pullally run 1. * NavSatFix status 2 (no receiver solution type recorded). Right: RMS difference between the reference and the raw RTK fixes after a rigid fit.", 150), fontsize=6.5,
                 color=style.INK_SECONDARY)
    save(ctx, fig, "F9_ground_truth_quality")
    return "ok"


# ---------------------------------------------------------------- F10
def f10_heading(ctx):
    """F10: heading-error RMSE (wrapped yaw of the aligned estimate vs ground-
    truth course over ground, speed > threshold; Section 11.6) for all samples
    and for row samples only; mean ± SD over repetitions. Sequences without a
    heading reference (Pullally: LiDAR mounting yaw not published) are N/A."""
    hd = ctx.table("heading")
    if hd is None:
        return "skipped: no heading.csv"
    seqs = ctx.test_seqs()
    fills = (1.0, 0.35)
    fig, axes = panel_grid(len(seqs) + 1, 2, h_panel=2.1)
    for ax, s in zip(axes.flat, seqs):
        ax.set_title(ctx.seq_label(s), fontsize=8.5)
        d0 = hd[(hd["seq"] == s) & (hd["config"] == "D") & (hd["lever_arm"] == ctx.lever(s))]
        if d0.empty:
            why = ("no heading reference\n(LiDAR mounting yaw not published)"
                   if ctx.seqs[s].get("lidar_forward_axis") is None else "no heading results")
            ax.text(0.5, 0.5, "N/A: " + why, transform=ax.transAxes, ha="center", va="center", fontsize=7.5,
                    color=style.INK_SECONDARY)
            ax.set_yticks([])
            ax.set_xticks([])
            for sp in ax.spines.values():
                sp.set_visible(False)
            ax.grid(False)
            continue
        for i, m in enumerate(METHODS):
            hl = ctx.headline(m)
            reps = ctx.accuracy(m, s, "D", hl)   # attempted repetitions (failures included)
            aggs = []
            for sub in ("all", "row"):
                d = d0[(d0["method"] == m) & (d0["output"] == hl) & (d0["subset"] == sub)]
                vals = {int(r): float(v) for r, v in zip(d["rep"], d["rmse_deg"]) if np.isfinite(v)}
                aggs.append(Agg(values=vals, n_total=reps.n_total))
            _bars(ax, i, aggs, style.COLOUR[m], 0.38, fills)
        ax.set_ylim(bottom=0)
        method_ticks(ax)
        ax.set_ylabel("Heading RMSE (°)")
    leg = list(axes.flat)[len(seqs)]
    leg.axis("off")
    leg.legend(handles=[Patch(facecolor=tint(style.INK_SECONDARY, f), edgecolor=style.INK_SECONDARY, label=lab)
                        for f, lab in zip(fills, ("All samples", "Row samples only"))],
               loc="center", frameon=False, title="Samples (bar fill; hue = method)")
    for ax in list(axes.flat)[len(seqs) + 1:]:
        ax.set_visible(False)
    fig.tight_layout()
    save(ctx, fig, "F10_heading_error")
    return "ok"


# ---------------------------------------------------------------- F11
def _sens_value(ctx, m, seq, config, kind):
    if kind == "rmse":
        return ctx.accuracy(m, seq, config, ctx.headline(m))
    col = {"cpu": "cpu_mean_pct_one_core", "rss": "rss_peak_gb"}[kind]
    return ctx.resource(m, seq, config, col)


def _unit(u):
    u = "" if u in ("", None) else str(u)
    return {"deg": "°", "m^2": "m²"}.get(u, u)


def _sensitivity_figure(ctx, kind, xlabel, name, ref_unit):
    have = {m: ctx.sensitivity_levels(m) for m in METHODS}
    if not any(have.values()):
        return None
    fig, axes = plt.subplots(3, 2, figsize=(7.2, 9.8), squeeze=False)
    plotted = False
    any_behind = False
    for ax, m in zip(axes.flat, METHODS):
        levels = have[m]
        seq = ctx.sensitivity_seq(m)
        ref = _sens_value(ctx, m, seq, "D", kind)
        ref_txt = f"D: {ref.text()} {ref_unit} (n = {ref.n_ok})" if ref.n_ok else f"D: {ref.text()}"
        ax.set_title(f"{LABEL[m]}\n{ref_txt}", fontsize=8.5)
        if not levels:
            ax.text(0.5, 0.5, "N/A: no sensitivity runs", transform=ax.transAxes, ha="center", va="center",
                    fontsize=7.5, color=style.INK_SECONDARY)
            ax.set_xticks([])
            ax.set_yticks([])
            continue
        # One header row per parameter (name and default), then one row per level.
        ys, labels, headers, y, last = [], [], [], -0.8, None
        dvs = []
        for p, v, cname in levels:
            if p["id"] != last:
                y += 0.8 if last is None else 1.3
                unit = _unit(p.get("unit"))
                dflt = f" (D: {p['default']}{'' if unit == '°' else ' '}{unit})" if "default" in p else ""
                dflt = dflt.replace(" ", "\u00a0").replace("\u00a0(", " (", 1)   # keep "(D: value unit)" on one line
                headers.append((y, textwrap.fill(report.param_name(p) + dflt, 40)))
                y += 0.85
                last = p["id"]
            else:
                y += 0.85
            unit = _unit(p.get("unit"))
            ys.append(y)
            labels.append(f"{v}{'' if unit == '°' else ' '}{unit}".strip())
            a = _sens_value(ctx, m, seq, cname, kind)
            if a.n_ok and ref.n_ok:
                dv = a.mean - ref.mean
                dvs.append(dv)
                rt_col = ctx.realtime_column(m) if ctx.table("resources_summary") is not None else None
                pr = ctx.resource(m, seq, cname, rt_col) if rt_col else None
                behind = pr is not None and pr.n_ok and pr.mean < 0.95
                ax.plot([0, dv], [y, y], color=style.COLOUR[m], lw=1.0, zorder=2)
                ax.plot(dv, y, marker=style.MARKER[m], ms=5, color=style.COLOUR[m], ls="", zorder=3,
                        mfc="white" if behind else style.COLOUR[m])
                if behind:   # fell behind real time: RMSE covers only part of the run
                    ax.annotate(f"{100 * pr.mean:.0f} % of scans", xy=(dv, y), xytext=(0, 5),
                                textcoords="offset points", ha="center", va="bottom", fontsize=6,
                                color=style.INK_SECONDARY)
                    any_behind = True
                plotted = True
            else:
                word = "D: N/A" if not ref.n_ok else ("failed" if a.status == "failed" else "N/A")
                ax.text(0, y, f" {word}", ha="left", va="center", fontsize=6.5, color=style.INK_SECONDARY)
        for yh, text in headers:
            ax.annotate(text, xy=(0, yh), xycoords=("axes fraction", "data"), xytext=(-3, 0),
                        textcoords="offset points", ha="right", va="center", fontsize=7, color=style.INK)
        if ref.n_ok > 1:
            ax.axvspan(-ref.sd, ref.sd, color=BAND, alpha=0.7, lw=0, zorder=0)
        ax.axvline(0, color=style.INK_SECONDARY, lw=0.8, zorder=1)
        ax.set_yticks(ys)
        ax.set_yticklabels(labels, fontsize=6.8)
        ax.set_ylim(max(ys) + 0.6, -0.6)
        ax.grid(axis="y", visible=False)
        ax.margins(x=0.15)
        label = xlabel
        adv = np.abs(np.asarray(dvs, dtype=float))
        if len(adv) > 2 and np.median(adv) > 0 and adv.max() / np.median(adv) > 30:
            # One level dwarfs the others: linear near zero, logarithmic beyond, so
            # small but real effects stay readable next to a divergent level.
            lt = max(float(np.percentile(adv, 75)), 1e-3)
            ax.set_xscale("symlog", linthresh=lt, linscale=1.0)
            lo, hi = ax.get_xlim()
            ticks = [t for t in (-100, -10, -1, -0.1, 0, 0.1, 1, 10, 100) if lo <= t <= hi
                     and (t == 0 or abs(t) >= lt)]
            ax.set_xticks(ticks)
            ax.xaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:g}".replace("-", "\u2212")))
            ax.xaxis.set_minor_locator(matplotlib.ticker.NullLocator())
            label = xlabel + f"\n(linear within ±{lt:.2g}, logarithmic beyond)"
        ax.set_xlabel(label)
    h = [Patch(color=BAND, alpha=0.7, label="Run-to-run spread of D (± 1 SD)"),
         Line2D([], [], color=style.INK_SECONDARY, lw=0.8, label="Mean of D")]
    if any_behind:
        h.append(Line2D([], [], marker="o", ls="", mfc="white", mec=style.INK_SECONDARY,
                        label="Fell behind real time (< 95 % of scans processed)"))
    fig.tight_layout(h_pad=1.5, w_pad=1.0)
    legend_above(fig, h)
    save(ctx, fig, name)
    return "ok" if plotted else "ok (no level has a value yet)"


def f11_sensitivity(ctx):
    """F11: one-at-a-time sensitivity on the sensitivity sequence (cf_5, first
    380 s): ΔRMSE of each parameter level relative to the mean of the D runs
    (headline output, alignment A, 3D), one panel per method; shaded band =
    ± 1 SD of the D runs (run-to-run spread). Supplementary: ΔCPU and ΔRSS."""
    if ctx.table("results_long") is None:
        return "skipped: no results_long.csv"
    st = _sensitivity_figure(ctx, "rmse", "ΔRMSE vs D (m)", "F11_sensitivity_rmse", "m")
    if st is None:
        return "skipped: no sensitivity design or OAT results"
    if ctx.table("resources_summary") is not None:
        _sensitivity_figure(ctx, "cpu", "ΔCPU vs D (% of one core)",
                            "F11_supp_sensitivity_cpu", "% of one core")
        _sensitivity_figure(ctx, "rss", "ΔRSS peak vs D (GB)", "F11_supp_sensitivity_rss", "GB")
    else:
        print("  F11 supplementary: skipped (no resources_summary.csv)")
    return st


def f8_supp_turn_labels(ctx):
    """Validation of the row/turn labels (brief Section 11.5): ground-truth path
    of every sequence, samples labelled turn (heading change > threshold over the
    window) drawn dark, row samples light."""
    seqs = [s for s in ("cf_2", "cf_3", "cf_6", "cf_7", "cf_5", "pl_1") if ctx.gt_file(s).exists()]
    if not seqs:
        return "skipped: no ground truth"
    tc = ctx.cfg["turn"]
    cols = 3
    rows = int(np.ceil(len(seqs) / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(7.2, 3.1 * rows), squeeze=False)
    for ax, s in zip(axes.flat, seqs):
        gt = ctx.gt(s)
        p, turn = gt.p, np.asarray(gt.turn, dtype=bool)
        ax.plot(p[~turn, 0], p[~turn, 1], ".", ms=1.2, color="#c9c8c3", label="Row")
        ax.plot(p[turn, 0], p[turn, 1], ".", ms=1.6, color=style.INK, label="Turn")
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_title(f"{style.SEQ_LABEL.get(s, s)}\n{100 * turn.mean():.0f} % of samples labelled turn", fontsize=8)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
    for ax in list(axes.flat)[len(seqs):]:
        ax.set_visible(False)
    h = [Line2D([], [], marker="o", ls="", color=style.INK, label="Turn"),
         Line2D([], [], marker="o", ls="", color="#c9c8c3", label="Row")]
    fig.suptitle(f"Turn: heading change > {tc['threshold_deg']:g}° over {tc['window_m']:g} m "
                 f"(heading over a {tc['heading_baseline_m']:g} m baseline)", fontsize=8, y=1.0)
    fig.tight_layout()
    legend_above(fig, h)
    save(ctx, fig, "F8_supp_turn_labels")
    return "ok"


FIGURES = {"F0": f0_ground_truth, "F1": f1_trajectories, "F1_comp": f1_composite, "F2": f2_error_vs_distance, "F3": f3_error_boxplots,
           "F4": f4_rmse_matrix, "F5": f5_rmse_components, "F6": f6_resources, "F7": f7_floor_ablation,
           "F8": f8_segment_drift, "F8_supp": f8_supp_turn_labels, "F9": f9_ground_truth_quality, "F10": f10_heading, "F11": f11_sensitivity}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("only", nargs="*", help="figure IDs (default: all)")
    report.add_path_args(ap, out_help="output folder (default: figs/)")
    args = ap.parse_args(argv)
    style.setup()
    ctx = report.context_from_args(args, FIGS)
    status = {}
    for fid, fn in FIGURES.items():
        if args.only and fid not in args.only:
            continue
        status[fid] = fn(ctx)
        print(f"{fid}: {status[fid]}")
    for n in sorted(ctx.notes):
        print(f"note: {n}")
    return status


if __name__ == "__main__":
    main()
