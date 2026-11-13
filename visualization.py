"""Map and path plotting."""
from __future__ import annotations
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

def plot_map(grid, start, targets, path=None, discovered=None, title="Search and Rescue Robot", output_path=None):
    fig, ax = plt.subplots(figsize=(12, 7))
    display = np.where(grid == 1, 1, 0)
    ax.imshow(display, cmap=plt.matplotlib.colors.ListedColormap(["#f7f7f7", "#333333"]),
              vmin=0, vmax=1, interpolation="nearest")
    if path:
        rr = [p[0] for p in path]
        cc = [p[1] for p in path]
        ax.plot(cc, rr, color="#e69f00", linewidth=2, label="Robot path")
    ax.scatter([start[1]], [start[0]], marker="s", s=90, c="#2676d2", label="Start")
    if targets:
        ax.scatter([t[1] for t in targets], [t[0] for t in targets],
                   marker="*", s=140, c="#2ca25f", label="Targets")
    if discovered:
        ax.scatter([p[1] for p in discovered], [p[0] for p in discovered],
                   marker="x", s=65, c="#d62728", label="Newly discovered obstacles")
    ax.set_xticks(range(grid.shape[1]))
    ax.set_yticks(range(grid.shape[0]))
    ax.grid(which="major", color="#bdbdbd", linewidth=.35)
    ax.set_xlabel("Column")
    ax.set_ylabel("Row")
    ax.set_title(title)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=4)
    fig.tight_layout()
    if output_path:
        fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
