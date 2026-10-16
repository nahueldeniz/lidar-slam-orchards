#!/usr/bin/env python3
"""Point-cloud map figures (F12): orchard map of a sequence built by placing
LiDAR scans along one method's trajectory (default LeGO-LOAM, the most accurate
with configuration D), aligned to the ground truth exactly as in the evaluation
(alignment A, main lever-arm setting, repetition with the median RMSE).

The map is an illustration built from estimated poses (the ground truth has no
orientation), and the caption must say so.

Two steps:
  build   read every N-th scan from the bags (external drive), transform it
          with the aligned LiDAR pose, voxel-downsample, and cache the points
          (float32 x, y, z in the ground-truth frame) in eval/maps/<seq>_<method>.npy.
  render  canopy height model (CHM) from the cached points: per cell, the
          highest point above the local ground (low percentile per tile),
          hill-shaded. Top: the whole field with the ground truth. Bottom: the
          first and the last headland turn with the six methods' trajectories
          (headline output, configuration D, the same repetitions as Fig. F1)
          over the CHM in grey. A sequence that is longer in y than in x is
          rotated by -90 degrees, (x, y) -> (y, -x), as in F1_trajectories_composite.

  make_maps.py build cf_7 [--every 5] [--voxel 0.15] [--range 25]
  make_maps.py render cf_7 [--cell 0.2]
"""
import argparse
import sys
from pathlib import Path

import numpy as np

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R / "eval"))
sys.path.insert(0, str(R / "scripts"))
from slameval import align, associate, bagio, report, style  # noqa: E402
from slameval.evaluate import estimated_positions  # noqa: E402
from slameval.trajectory import read_tum  # noqa: E402

plt = style.plt
CACHE = R / "eval" / "maps"
WIDTH_IN = 5.1          # text width of the journal class (372 pt)


def context():
    return report.Context(out_dir=R / "figs")


def lidar_poses(ctx, method, seq):
    """LiDAR poses of the representative run in the ground-truth frame, with the
    alignment of the evaluation (computed on the antenna positions)."""
    hl = ctx.headline(method)
    rep, acc = ctx.representative_rep(method, seq, "D", hl)
    if rep is None:
        raise ValueError(f"no successful run of {method} on {seq}")
    est = read_tum(ctx.tum_path(method, seq, "D", rep, hl))
    est = est.subset(np.flatnonzero(est.finite_mask()))
    gt = ctx.gt(seq)
    lever = ctx.lever_vec(seq) if acc.lever == "with" else None
    a = associate.associate(est.t, gt.t, gt.p, ctx.cfg["association"]["max_gap_periods"] * gt.nominal_period_s)
    p_hat = estimated_positions(est, lever if est.has_orientation else None)
    al = align.fit_global(p_hat[a.idx], a.p_gt)
    from scipy.spatial.transform import Rotation
    return est.t, al.apply(est.p), Rotation.from_matrix(al.R) * est.rotations(), rep


def voxel_down(points, voxel):
    key = np.floor(points / voxel).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return points[idx]


def build(ctx, seq, method, every, max_range, voxel):
    from rosbags.highlevel import AnyReader
    from estimate_lidar_height import pointcloud2_xyz
    from inventory import data_root
    cfg = ctx.seqs[seq]
    t_pose, pw, Rw, rep = lidar_poses(ctx, method, seq)
    bags = [data_root() / b for b in cfg["bags"]]
    chunks, n_used = [], 0
    with AnyReader(bags, default_typestore=bagio.TYPESTORE) as rd:
        conns = [c for c in rd.connections if c.topic == cfg["lidar_topic"]]
        for k, (c, _, raw) in enumerate(rd.messages(connections=conns)):
            if k % every:
                continue
            m = rd.deserialize(raw, c.msgtype)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            j = int(np.clip(np.searchsorted(t_pose, t), 0, len(t_pose) - 1))
            if j > 0 and abs(t_pose[j - 1] - t) < abs(t_pose[j] - t):
                j -= 1
            if abs(t_pose[j] - t) > 0.06:
                continue
            xyz = pointcloud2_xyz(m)
            r = np.linalg.norm(xyz[:, :2], axis=1)
            xyz = xyz[(r > 1.0) & (r < max_range)]
            chunks.append(Rw[j].apply(xyz) + pw[j])
            n_used += 1
            if len(chunks) % 50 == 0:   # keep memory bounded
                chunks = [voxel_down(np.vstack(chunks), voxel)]
    pts = voxel_down(np.vstack(chunks), voxel).astype(np.float32)
    CACHE.mkdir(parents=True, exist_ok=True)
    out = CACHE / f"{seq}_{method}.npy"
    np.save(out, pts)
    print(f"{seq}: {len(pts)} points from {n_used} scans (every {every}th), {method} rep{rep} -> {out.relative_to(R)}")


# ---------------------------------------------------------------- render
def canopy_height_model(xy, z, lo, shape, cell, tile=4.0, q=5.0, min_pts=20):
    """Per cell, the highest point above the local ground; local ground = q-th
    percentile of z per tile (tiles with few points take the nearest tile)."""
    import pandas as pd
    from scipy import ndimage
    ti = np.floor((xy - lo) / tile).astype(int)
    df = pd.DataFrame({"i": ti[:, 0], "j": ti[:, 1], "z": z})
    g = df.groupby(["i", "j"]).z
    gq = g.quantile(q / 100)[g.size() >= min_pts]
    G = np.full(ti.max(axis=0) + 1, np.nan)
    G[gq.index.get_level_values(0), gq.index.get_level_values(1)] = gq.values
    idx = ndimage.distance_transform_edt(np.isnan(G), return_distances=False, return_indices=True)
    G = ndimage.median_filter(G[tuple(idx)], size=3, mode="nearest")
    h = z - G[ti[:, 0], ti[:, 1]]
    ci = np.floor((xy - lo) / cell).astype(int)
    ok = (ci[:, 0] >= 0) & (ci[:, 1] >= 0) & (ci[:, 0] < shape[0]) & (ci[:, 1] < shape[1])
    H = np.full(shape, -np.inf)
    np.maximum.at(H, (ci[ok, 0], ci[ok, 1]), h[ok])
    H[~np.isfinite(H)] = np.nan
    return H.T                                            # rows = y (origin lower)


def shaded_rgb(H, cell, cmap, h_ground=0.5, h_max=4.0, ground_rgb=(0.91, 0.90, 0.87), lo_c=0.25, fill=2):
    """Hill-shaded RGBA image of a CHM; empty cells transparent."""
    from matplotlib.colors import LightSource
    from scipy import ndimage
    Hd = H.copy()
    for _ in range(fill):   # close the small gaps of a sparse scan pattern (max over 3 x 3 where empty)
        mx = ndimage.maximum_filter(np.nan_to_num(Hd, nan=-1.0), size=3)
        gap = np.isnan(Hd) & (mx > -1.0)
        Hd[gap] = mx[gap]
    valid = ~np.isnan(Hd)
    Hc = np.clip(np.nan_to_num(Hd, nan=0.0), 0, h_max)
    Hc[Hc < h_ground] = 0.0                      # flat ground: shade the canopy only
    shade = LightSource(azdeg=315, altdeg=45).hillshade(Hc[::-1], vert_exag=1.5, dx=cell, dy=cell)[::-1]
    canopy = valid & (Hd >= h_ground)
    rgb = np.empty(H.shape + (3,))
    rgb[:] = ground_rgb
    frac = lo_c + (1 - lo_c) * np.clip((Hd - h_ground) / (h_max - h_ground), 0, 1)
    rgb[canopy] = cmap(frac[canopy])[:, :3]
    rgb *= (0.62 + 0.38 * shade)[..., None]
    rgba = np.concatenate([rgb, valid[..., None].astype(float)], axis=-1)
    return rgba


def turn_centres(p, step=0.5, half=10, thr_deg=40.0, min_total_deg=60.0):
    """Centres of the headland turns of a ground-truth path (heading change over
    2 x half x step metres above thr_deg, total change of a turn >= min_total_deg),
    with the distance travelled at each centre."""
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    s = np.arange(0, d[-1], step)
    xs, ys = np.interp(s, d, p[:, 0]), np.interp(s, d, p[:, 1])
    hd = np.unwrap(np.arctan2(np.gradient(ys), np.gradient(xs)))
    turn = np.zeros(len(s), bool)
    turn[half:-half] = np.degrees(np.abs(hd[2 * half:] - hd[:-2 * half])) > thr_deg
    out, k = [], 0
    while k < len(s):
        if turn[k]:
            e = k
            while e + 1 < len(s) and turn[e + 1]:
                e += 1
            if np.degrees(abs(hd[min(e + half, len(s) - 1)] - hd[max(k - half, 0)])) >= min_total_deg:
                c = (k + e) // 2
                out.append((np.array([xs[c], ys[c]]), s[c]))
            k = e + 1
        else:
            k += 1
    return out


def render(ctx, seq, method, cell, zoom_m=16.0, zoom_aspect=0.78):
    from matplotlib import patheffects
    from matplotlib.colors import LinearSegmentedColormap
    from matplotlib.lines import Line2D
    pts = np.load(CACHE / f"{seq}_{method}.npy").astype(np.float64)
    g3 = ctx.gt(seq).p
    rotate = np.ptp(g3[:, 1]) > np.ptp(g3[:, 0])
    rot = (lambda xy: np.column_stack([xy[:, 1], -xy[:, 0]])) if rotate else (lambda xy: xy)  # noqa: E731
    xy, z, g = rot(pts[:, :2]), pts[:, 2], rot(g3[:, :2])
    # Field extent: ground-truth bounding box plus a margin (drops far, sparse returns).
    margin = 8.0
    lo, hi = g.min(axis=0) - margin, g.max(axis=0) + margin
    shape = tuple(np.ceil((hi - lo) / cell).astype(int))
    H = canopy_height_model(xy, z, lo, shape, cell)
    extent = (lo[0], lo[0] + shape[0] * cell, lo[1], lo[1] + shape[1] * cell)
    green = LinearSegmentedColormap.from_list("canopy", ["#e3edc4", "#9cc46b", "#4f8f3a", "#1e5128", "#0c2e17"])
    grey = LinearSegmentedColormap.from_list("canopy_grey", ["#f1f0ec", "#c9c8c3", "#8f8e89", "#5d5c58"])
    over = shaded_rgb(H, cell, green)

    # Trajectories of the six methods (exactly the F1 trajectories), rotated.
    traj = {}
    for m in style.METHODS:
        hl = ctx.headline(m)
        rep, _ = ctx.representative_rep(m, seq, "D", hl)
        pe = ctx.pose_errors(m, seq, "D", rep, hl) if rep is not None else None
        if pe is not None:
            traj[m] = (rot(pe.p_all[:, :2]), pe.rmse, rep)
    turns = turn_centres(g)
    half = np.array([zoom_m / 2, zoom_m * zoom_aspect / 2])   # zoom box half-size (m)
    zooms = [turns[0], turns[-1]] if len(turns) >= 2 else turns
    # Source of any statement about the zooms: per method, the 3D error (alignment A) of the
    # poses whose ground truth lies in each zoom box, and the share of them drawn inside it.
    for k, (c, s_k) in enumerate(zooms):
        print(f"{seq} zoom ({chr(98 + k)}) at {s_k:.0f} m:")
        for m in style.METHODS:
            hl = ctx.headline(m)
            rep, _ = ctx.representative_rep(m, seq, "D", hl)
            pe = ctx.pose_errors(m, seq, "D", rep, hl) if rep is not None else None
            if pe is None:
                continue
            inb = np.all(np.abs(rot(pe.p_gt[:, :2]) - c) < half, axis=1)
            if inb.any():
                drawn = np.all(np.abs(rot(pe.p[:, :2]) - c) < half, axis=1)[inb].mean()
                print(f"  {m:6s} rep{rep}: n={inb.sum():4d}  3D error mean {pe.err['3d'][inb].mean():6.2f} m, "
                      f"max {pe.err['3d'][inb].max():6.2f} m; {100 * drawn:3.0f} % inside the box")
    # Layout (inches, top-down): legend/colour-bar row, overview, zoom row, method legend.
    W, head, gap, leg = WIDTH_IN, 0.46, 0.30, 0.42
    ov_h = W * (extent[3] - extent[2]) / (extent[1] - extent[0])
    zw = (W - 0.15) / 2
    zh = zw * zoom_aspect
    height = head + ov_h + gap + zh + leg
    fig = plt.figure(figsize=(W, height))
    fy = lambda v: 1 - v / height  # noqa: E731
    halo = [patheffects.withStroke(linewidth=1.8, foreground="white")]

    ax = fig.add_axes([0, fy(head + ov_h), 1, ov_h / height])
    ax.imshow(over, origin="lower", extent=extent, interpolation="nearest", zorder=0)
    ax.plot(g[:, 0], g[:, 1], color=style.INK, lw=0.6, path_effects=halo, zorder=2)
    ax.plot(*g[0], "o", ms=4, mfc="white", mec=style.INK, mew=0.8, zorder=3)
    ax.plot(*g[-1], "s", ms=3.5, mfc=style.INK, mec="white", mew=0.5, zorder=3)
    for k, (c, s_k) in enumerate(zooms):
        ax.add_patch(plt.Rectangle(c - half, 2 * half[0], 2 * half[1], fill=False, ec=style.INK, lw=0.9, zorder=4))
        ax.text(c[0] - half[0] + 0.6, c[1] + half[1] - 0.6, chr(98 + k), fontsize=7, fontweight="bold",
                va="top", ha="left", zorder=5, path_effects=halo)
    # 20 m scale bar in the first map corner clear of the zoom boxes and the path.
    bar = 20.0
    boxes = [(c - half - 1.0, c + half + 1.0) for c, _ in zooms]
    for x0, y0 in [(extent[1] - 2 - bar, extent[2] + 2.0), (extent[0] + 2, extent[2] + 2.0),
                   (extent[1] - 2 - bar, extent[3] - 5.0), (extent[0] + 2, extent[3] - 5.0)]:
        blo, bhi = np.array([x0 - 1.0, y0 - 1.0]), np.array([x0 + bar + 1.0, y0 + 4.0])
        clear = not any(np.all(blo < b_hi) and np.all(b_lo < bhi) for b_lo, b_hi in boxes)
        clear &= not np.any(np.all((g > blo) & (g < bhi), axis=1))
        if clear:
            break
    ax.plot([x0, x0 + bar], [y0, y0], color=style.INK, lw=1.5, solid_capstyle="butt", zorder=5,
            path_effects=[patheffects.withStroke(linewidth=3, foreground="white")])
    ax.text(x0 + bar / 2, y0 + 1.0, f"{bar:.0f} m", ha="center", va="bottom", fontsize=7, path_effects=halo, zorder=5)
    ax.set_xlim(extent[:2])
    ax.set_ylim(extent[2:])
    ax.set_axis_off()
    fig.text(0, fy(head * 0.22), f"(a) {ctx.seq_label(seq)}: map from {style.LABEL[method]} poses", fontsize=7.5,
             fontweight="bold", va="center", ha="left")

    # Header: ground-truth legend (left) and height colour bar (right).
    hh = [Line2D([], [], color=style.INK, lw=0.6, label="Ground truth (RTK-GNSS)"),
          Line2D([], [], ls="", marker="o", ms=4, mfc="white", mec=style.INK, label="Start"),
          Line2D([], [], ls="", marker="s", ms=3.5, mfc=style.INK, mec=style.INK, label="End")]
    fig.legend(handles=hh, loc="center left", bbox_to_anchor=(0, fy(head * 0.68)), ncol=3, frameon=False,
               fontsize=7, handlelength=1.6, columnspacing=1.0, borderaxespad=0)
    cax = fig.add_axes([0.64, fy(head * 0.62), 0.34, 0.07 / height])
    hv = np.linspace(0, 4, 256)
    frac = np.where(hv < 0.5, np.nan, 0.25 + 0.75 * (hv - 0.5) / 3.5)
    row = np.array([green(f)[:3] if np.isfinite(f) else (0.91, 0.90, 0.87) for f in frac])
    cax.imshow(row[None], aspect="auto", extent=(0, 4, 0, 1))
    cax.set_yticks([])
    cax.set_xticks([0, 1, 2, 3, 4])
    cax.tick_params(labelsize=6.5, length=2, pad=1)
    cax.grid(False)
    cax.set_title("Height above ground (m)", fontsize=7, pad=2)

    for k, (c, s_k) in enumerate(zooms):
        az = fig.add_axes([k * (zw + 0.15) / W, fy(head + ov_h + gap + zh), zw / W, zh / height])
        zc = cell / 2
        zlo = c - zoom_m / 2 - 4.0
        zshape = tuple(np.ceil(np.full(2, zoom_m + 8.0) / zc).astype(int))
        sel = np.all((xy >= zlo) & (xy < zlo + zoom_m + 8.0), axis=1)
        Hz = canopy_height_model(xy[sel], z[sel], zlo, zshape, zc)
        under = shaded_rgb(Hz, zc, grey, ground_rgb=(0.975, 0.972, 0.965), lo_c=0.0, fill=3)
        az.imshow(under, origin="lower", extent=(zlo[0], zlo[0] + zshape[0] * zc, zlo[1], zlo[1] + zshape[1] * zc),
                  interpolation="bilinear", zorder=0)
        az.plot(g[:, 0], g[:, 1], color=style.GT_COLOUR, lw=3.0, alpha=0.6, solid_capstyle="round", zorder=1)
        for m, (pm, _, _) in traj.items():
            az.plot(pm[:, 0], pm[:, 1], color=style.COLOUR[m], ls=style.LINESTYLE[m], lw=1.0, zorder=2)
        az.set_xlim(c[0] - zoom_m / 2, c[0] + zoom_m / 2)
        az.set_ylim(c[1] - half[1], c[1] + half[1])
        az.set_xticks([])
        az.set_yticks([])
        az.grid(False)
        for sp in az.spines.values():
            sp.set_visible(True)
            sp.set_color(style.INK_SECONDARY)
            sp.set_linewidth(0.6)
        which = "First" if k == 0 else "Last"
        az.set_title(f"({chr(98 + k)}) {which} headland turn, {s_k:.0f} m travelled", fontsize=7.5, loc="left",
                     pad=2, fontweight="bold")
        b5 = 5.0
        bx, by = c[0] + half[0] - 0.8 - b5, c[1] - half[1] + 0.8
        az.plot([bx, bx + b5], [by, by], color=style.INK, lw=1.5, solid_capstyle="butt", zorder=5,
                path_effects=[patheffects.withStroke(linewidth=3, foreground="white")])
        az.text(bx + b5 / 2, by + 0.3, "5 m", ha="center", va="bottom", fontsize=7, path_effects=halo, zorder=5)
    hm = [Line2D([], [], color=style.GT_COLOUR, lw=3.0, alpha=0.6, label="Ground truth")]
    hm += [Line2D([], [], color=style.COLOUR[m], ls=style.LINESTYLE[m], lw=1.2,
                  label=f"{style.LABEL[m]} ({report.fmt(traj[m][1])} m)") for m in style.METHODS if m in traj]
    fig.legend(handles=hm, loc="center", bbox_to_anchor=(0.5, fy(height - leg * 0.45)), ncol=4, frameon=False,
               fontsize=6.5, handlelength=2.4, columnspacing=1.0)
    style.save(fig, R / "figs", f"F12_map_{seq}")
    print(f"{seq}: figs/F12_map_{seq}.pdf ({W:.2f} x {height:.2f} in; turns at "
          + ", ".join(f"{s_k:.0f} m" for _, s_k in zooms) + ")")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", choices=["build", "render"])
    ap.add_argument("seqs", nargs="+")
    ap.add_argument("--method", default="lego", help="method whose poses place the scans")
    ap.add_argument("--every", type=int, default=5, help="build: use every N-th scan")
    ap.add_argument("--voxel", type=float, default=0.15, help="build: voxel size (m)")
    ap.add_argument("--range", type=float, default=25.0, help="build: maximum horizontal range (m)")
    ap.add_argument("--cell", type=float, default=0.2, help="render: CHM cell size (m)")
    args = ap.parse_args()
    style.setup()
    ctx = context()
    for seq in args.seqs:
        if args.step == "build":
            build(ctx, seq, args.method, args.every, args.range, args.voxel)
        else:
            render(ctx, seq, args.method, args.cell)


if __name__ == "__main__":
    main()
