"""Figure style shared by every figure (brief Section 13).

One fixed colour per method (categorical slots 1-6 of the validated reference
palette, light mode). Six hues cannot all be told apart when every pair is on
screen at once (validator: all-pairs CVD dE 3.2 green/orange), so every method
also has a fixed marker and line style, trajectory overlays use small
multiples (one method per panel over grey ground truth), and categorical axes
always carry the method name. Text stays in neutral ink.
"""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

METHODS = ["hdl", "lego", "aloam", "dlo", "kiss", "mesh"]
LABEL = {"hdl": "hdl Graph SLAM", "lego": "LeGO-LOAM", "aloam": "A-LOAM",
         "dlo": "DLO", "kiss": "KISS-ICP", "mesh": "SLAMesh"}
COLOUR = {"hdl": "#2a78d6", "lego": "#eb6834", "aloam": "#1baf7a",
          "dlo": "#eda100", "kiss": "#e87ba4", "mesh": "#008300"}
MARKER = {"hdl": "o", "lego": "s", "aloam": "^", "dlo": "D", "kiss": "v", "mesh": "P"}
LINESTYLE = {"hdl": "-", "lego": "--", "aloam": "-.", "dlo": ":", "kiss": (0, (5, 1, 1, 1)),
             "mesh": (0, (3, 1, 1, 1, 1, 1))}
GT_COLOUR = "#8a8984"          # ground truth: neutral grey, never a series hue
INK = "#0b0b0b"
INK_SECONDARY = "#52514e"
GRID = "#e4e3df"
SEQ_LABEL = {"cf_2": "CitrusFarm 02", "cf_3": "CitrusFarm 03", "cf_6": "CitrusFarm 06",
             "cf_7": "CitrusFarm 07", "cf_4": "CitrusFarm 04", "cf_5": "CitrusFarm 05 (sensitivity, first 380 s)",
             "pl_1": "Pullally run 1"}


def setup():
    plt.rcParams.update({
        "font.size": 9, "axes.titlesize": 9, "axes.labelsize": 9, "legend.fontsize": 8,
        "xtick.labelsize": 8, "ytick.labelsize": 8,
        "text.color": INK, "axes.labelcolor": INK, "xtick.color": INK_SECONDARY,
        "ytick.color": INK_SECONDARY, "axes.edgecolor": INK_SECONDARY,
        "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
        "axes.spines.top": False, "axes.spines.right": False,
        "lines.linewidth": 1.2, "savefig.bbox": "tight", "pdf.fonttype": 42, "ps.fonttype": 42,
    })


def save(fig, out_dir, name):
    out_dir.mkdir(parents=True, exist_ok=True)
    for ext in ("pdf", "png"):
        fig.savefig(out_dir / f"{name}.{ext}", dpi=300)
    plt.close(fig)
