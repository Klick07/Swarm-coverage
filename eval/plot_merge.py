"""Top-down plot of the merged map: each robot's Swarm-SLAM trajectory vs ground truth,
all placed with ONE joint SE(3) alignment.

Usage: python -m eval.plot_merge <out.png> <run_dir>[:title] [<run_dir>[:title] ...]
Needs eval/{optimized,gt}_rN.tum from eval.cslam_eval in each run dir.
"""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from eval.cslam_eval import ate, run_sequences

# Reference categorical palette, slots 1-3 in fixed order (validated: CVD dE 9.2, normal 27.6).
ROBOT_COLORS = ["#2a78d6", "#eb6834", "#1baf7a"]
SURFACE, INK, INK_2, GT_GRAY, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#b9b8b2", "#e8e7e3"


def load(run, kind, r):
    d = np.loadtxt(f"{run}/eval/{kind}_r{r}.tum")
    return d[:, 0], d[:, 1:4]


def panel(ax, run, title):
    est = {r: load(run, "optimized", r) for r in range(3)}
    gt = {r: load(run, "gt", r) for r in range(3)}
    E = np.vstack([est[r][1] for r in range(3)])
    G = np.vstack([gt[r][1] for r in range(3)])
    rmse, R, t = ate(E, G)
    for r in range(3):
        g = gt[r][1]
        ax.plot(g[:, 0], g[:, 1], color=GT_GRAY, lw=1.0, zorder=1)
    for r in range(3):
        p = est[r][1] @ R.T + t
        ax.plot(p[:, 0], p[:, 1], color=SURFACE, lw=3.5, zorder=2, solid_capstyle="round")  # 2px surface ring
        # All flights land at the same pad, so end-of-line labels collide: name robots in the legend.
        ax.plot(p[:, 0], p[:, 1], color=ROBOT_COLORS[r], lw=2.0, zorder=3, solid_capstyle="round",
                label=f"Robot {r} · {run_sequences(run)[r]} (Swarm-SLAM)")
    ax.set_title(f"{title}\njoint ATE {rmse:.3f} m (one alignment for all robots)",
                 fontsize=10, color=INK, loc="left")
    ax.set_aspect("equal")
    ax.grid(color=GRID, lw=0.6)
    ax.set_facecolor(SURFACE)
    for s in ax.spines.values():
        s.set_color(GRID)
    ax.tick_params(colors=INK_2, labelsize=8)
    ax.set_xlabel("x (m)", color=INK_2, fontsize=8)
    ax.set_ylabel("y (m)", color=INK_2, fontsize=8)
    return rmse


def main(out, *runs):
    fig, axes = plt.subplots(1, len(runs), figsize=(6.2 * len(runs), 5.6), squeeze=False, facecolor=SURFACE)
    for ax, spec in zip(axes[0], runs):
        run, _, title = spec.partition(":")
        panel(ax, run, title or run)
    for ax in axes[0]:
        ax.plot([], [], color=GT_GRAY, lw=1.0, label="ground truth (Leica / Vicon)")
        # Below the axes: in a small room the paths fill the whole plot.
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=2, fontsize=8, frameon=False, labelcolor=INK)
    fig.suptitle("Swarm-SLAM merged map, EuRoC, top-down", fontsize=12, color=INK, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(out, dpi=160, facecolor=SURFACE)
    print("wrote", out)


if __name__ == "__main__":
    main(*sys.argv[1:])
