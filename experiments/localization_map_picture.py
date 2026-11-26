"""Picture of the localization map (navigation stage 2; design plan 13.1; decision row M345).

    MUJOCO_GL=egl python experiments/localization_map_picture.py [--out PATH]

Three panels on the same metre axes: the map's cells (outside the room, free, occupied), the distance-lookup grid as a colour map, and the
free-floor mask (where the robot's base centre may stand). The three stage 1 poses are marked on every panel.
"""

import argparse
import math
import os
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

import lidar_test as lt  # noqa: E402
from src.localization_map import FREE, OCCUPIED, OUTSIDE, load_or_build_localization_map  # noqa: E402
from src.task_config import TASK_CFG  # noqa: E402

DISPLAY_CAP_M = 3.0  # m: the distance panel shows distances up to this (the stored grid is not capped)


def main():
    """Builds (or loads) the maps and saves the picture; prints the map size and the cache source."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "docs" / "reports" / "20261126_localization_map.png"))
    args = ap.parse_args()
    sim = lt.load_task()
    base_z = lt.settle_base_height(sim)
    sx, sy = TASK_CFG["staging_xy"]
    back = TASK_CFG["staging_approach_back_m"]
    lt.place(sim, sx, sy, -math.pi / 2, base_z, TASK_CFG["pedestal_q"])
    maps, source, secs = load_or_build_localization_map(sim.model, sim.data)
    m = maps["meta"]
    nx, ny, cell = m["shape"][1], m["shape"][0], m["cell_m"]
    extent = (m["x0"], m["x0"] + nx * cell, m["y0"], m["y0"] + ny * cell)
    poses = [("staging stance", sx, sy), ("approach spot", sx, sy + back), ("middle of the room", 0.0, -2.2)]

    fig, axes = plt.subplots(1, 3, figsize=(19, 7), sharex=True, sharey=True)
    ax = axes[0]
    ax.imshow(maps["state"], origin="lower", extent=extent, cmap=ListedColormap(["#d9d9d9", "#ffffff", "#222222"]), vmin=0, vmax=2,
              interpolation="nearest")
    ix0, ix1, iy0, iy1 = m["room_inner"][0], m["room_inner"][1], m["room_inner"][2], m["room_inner"][3]
    ax.plot([ix0, ix1, ix1, ix0, ix0], [iy0, iy0, iy1, iy1, iy0], "--", color="#1f5fbf", lw=1.0)
    ax.legend(handles=[Patch(color="#d9d9d9", label="outside the room"), Patch(color="#ffffff", ec="#888888", label="free"),
                       Patch(color="#222222", label="occupied (fixed geoms at the lidar's height)")],
              loc="lower left", fontsize=8)
    ax.set_title(f"localization map: {nx} x {ny} cells of {cell * 100:.0f} cm\nslice at {m['slice_z_m']:.4f} m above the floor", fontsize=10)

    ax = axes[1]
    shown = np.minimum(maps["distance"], DISPLAY_CAP_M)
    im = ax.imshow(shown, origin="lower", extent=extent, cmap="viridis", vmin=0, vmax=DISPLAY_CAP_M, interpolation="nearest")
    cb = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.02)
    cb.set_label("distance to the nearest occupied cell, m\n(shown up to 3 m; stored uncapped)", fontsize=8)
    ax.set_title("distance-lookup grid (the particle filter's lookup)", fontsize=10)

    ax = axes[2]
    ax.imshow(maps["floor"], origin="lower", extent=extent, cmap=ListedColormap(["#ffffff", "#2e7d32"]), vmin=0, vmax=1, interpolation="nearest")
    ax.legend(handles=[Patch(color="#2e7d32", label="floor: the base centre may stand"), Patch(color="#ffffff", ec="#888888", label="not floor")],
              loc="lower left", fontsize=8)
    ax.set_title("free-floor mask (planner's free space, free for every heading)", fontsize=10)

    for ax in axes:
        for label, x, y in poses:
            ax.plot(x, y, "+", color="#d07a00", ms=12, mew=2)
            ax.annotate(label, (x, y), xytext=(6, 6), textcoords="offset points", fontsize=7, color="#7a4a00")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_aspect("equal")
        ax.grid(alpha=0.2)
    fig.suptitle("Navigation stage 2: the localization map at the lidar's scan plane, its distance grid and the floor mask (2 cm cells, PROVISIONAL)",
                 fontsize=11)
    fig.tight_layout()
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_name(out.stem + ".tmp.png")
    fig.savefig(tmp, dpi=100)
    os.replace(tmp, out)
    print(f"map source {source} ({secs:.2f} s); {nx} x {ny} cells; picture {out}", flush=True)


if __name__ == "__main__":
    main()
