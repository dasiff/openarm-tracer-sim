"""Reach heat maps of the bench top, from the FILTERED reach maps (data/reach_map_before_bench, the bench is in the maps).

Modes (run from the repo root with the project venv):
    python docs/reports/reach_heatmap_make.py --geometry
        One MuJoCo process: reads the robot geometry (stance_finder.robot_geometry) and the bench top from the loaded
        model and writes data/reach_heatmap/geometry.json. Do not run it while another MuJoCo process is running.
    python docs/reports/reach_heatmap_make.py --render
        Counts the parking spots that reach each bench point (no MuJoCo) and writes docs/reports/reach_heatmap_counts.npz
        and the PNGs: reach_heatmap_combined.png and reach_heatmap_bin_NN.png (12 bins).
    python docs/reports/reach_heatmap_make.py --sweep docs/reports/layout_sweep_current_20261106.json
        Part c: the failing layouts of the layout sweep on the combined heat map (reach_heatmap_failures.png).

Projection: the maps are in the arm's shoulder frame. Each bench point is taken into the shoulder frame of a parking spot with
the same transform the stance selection uses (src.stance_select.shoulder_xy for the shoulder's world xy, src.bench_stance.bench_frame
for the heading, src.stance_finder._torso_z for the height). The pedestal command is PEDESTAL_HEIGHT_M (0.16 m).
A parking spot is a floor cell (FLOOR_STEP_M) outside the bench top's footprint; it counts for a bench point if some heading
(HEADING_STEP_DEG) and either arm reaches the point from a map. Per-bin counts use only that bin's map.
"""
import argparse
import json
import math
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))
os.chdir(PROJECT_ROOT)

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from src import bench_stance as bs  # noqa: E402
from src.task_config import BEAKER_CFG, PICK_PLACE_CFG  # noqa: E402

MAPS_DIR = "data/reach_map_before_bench"          # the filtered maps (bench in the map)
GEOMETRY_JSON = Path("data/reach_heatmap/geometry.json")
COUNTS_NPZ = Path("docs/reports/reach_heatmap_counts.npz")
OUT = Path("docs/reports")
N_BINS = 12
PEDESTAL_HEIGHT_M = 0.16                           # David: the pedestal height used to project the maps onto the bench
BENCH_STEP_M = 0.05                                # bench grid cell (the points the counts are made for)
FLOOR_STEP_M = 0.05                                # parking spot grid (floor cells)
FLOOR_HALF_M = 0.80                                # each point looks at floor cells this far around it (the reach is about 0.7 m from the shoulder)
HEADING_STEP_DEG = 5.0                             # the stance finder's heading step (STANCE_CFG["heading_step_deg"])


def grasp_height():
    """The height of the pad centre on a beaker standing on the bench top: the pick's grasp fraction of the beaker height."""
    return PICK_PLACE_CFG["grasp_height_frac"] * BEAKER_CFG["height"]


def bench_top(geom):
    """The bench top's outline and surface height, from the model's geom (metres)."""
    cx, cy, cz = geom["centre"]
    hx, hy, hz = geom["half"]
    return {"x": [cx - hx, cx + hx], "y": [cy - hy, cy + hy], "surface_z": cz + hz}


def project(maps, rg, bench, z_point, step, half, q):
    """Counts the parking spots per bench point.

    Args:
        maps (dict): {(side, bin): ReachMap} (the filtered set).
        rg (dict): From stance_finder.robot_geometry, as JSON: "shoulder", "torso_z0", "lift_factor", "base_z".
        bench (dict): From bench_top.
        z_point (float): The height of the reach point, metres.
        step (float): The bench grid step.
        half (float): The half width of the floor window around each point.
        q (float): The pedestal command, metres.

    Returns:
        tuple: (xs, ys (bench point centres), total (ny, nx) int, per_bin (N_BINS, ny, nx) int).
    """
    xs = np.arange(bench["x"][0] + step / 2, bench["x"][1], step)
    ys = np.arange(bench["y"][0] + step / 2, bench["y"][1], step)
    fstep = FLOOR_STEP_M
    headings = np.radians(np.arange(0.0, 360.0, HEADING_STEP_DEG))
    c, s = np.cos(headings)[:, None], np.sin(headings)[:, None]
    total = np.zeros((len(ys), len(xs)), int)
    per_bin = np.zeros((N_BINS, len(ys), len(xs)), int)
    in_bench = lambda X, Y: (X >= bench["x"][0]) & (X <= bench["x"][1]) & (Y >= bench["y"][0]) & (Y <= bench["y"][1])   # noqa: E731
    for iy, py in enumerate(ys):
        for ix, px in enumerate(xs):
            fx = np.arange(px - half, px + half, fstep) + fstep / 2
            fy = np.arange(py - half, py + half, fstep) + fstep / 2
            X, Y = np.meshgrid(fx, fy)
            X, Y = X.ravel(), Y.ravel()
            keep = ~in_bench(X, Y)                                                       # a base cannot stand on the bench top
            dx, dy = px - X, py - Y
            reach = np.zeros((N_BINS, X.size), bool)
            for arm in ("left", "right"):
                sx, sy, sz = rg["shoulder"][arm]
                zt = rg["base_z"] + rg["torso_z0"] + rg["lift_factor"] * q + sz          # the shoulder's height above the floor (stance_finder._torso_z + shoulder z)
                zr = z_point - zt
                rx = c * dx + s * dy - sx                                                # the point in the shoulder frame (as stance_finder._region)
                ry = -s * dx + c * dy - sy
                for b in range(N_BINS):
                    rm = maps[(arm, b)]
                    iz = int(np.floor((zr - rm.origin[2]) / rm.cube))
                    if not 0 <= iz < rm.cubes.shape[2]:
                        continue
                    jx = np.floor((rx - rm.origin[0]) / rm.cube).astype(int)
                    jy = np.floor((ry - rm.origin[1]) / rm.cube).astype(int)
                    inb = (jx >= 0) & (jx < rm.cubes.shape[0]) & (jy >= 0) & (jy < rm.cubes.shape[1])
                    hit = rm.cubes[:, :, iz][np.clip(jx, 0, rm.cubes.shape[0] - 1), np.clip(jy, 0, rm.cubes.shape[1] - 1)] & inb
                    reach[b] |= hit.any(axis=0)
            reach &= keep[None, :]
            total[iy, ix] = int(reach.any(axis=0).sum())
            per_bin[:, iy, ix] = reach.sum(axis=1)
    return xs, ys, total, per_bin


def draw_bench(ax, bench):
    x0, x1 = bench["x"]
    y0, y1 = bench["y"]
    ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, edgecolor="white", linewidth=2.0))
    ax.set_xlim(x0 - 0.15, x1 + 0.15)
    ax.set_ylim(y0 - 0.15, y1 + 0.15)
    ax.set_aspect("equal")
    ax.set_xlabel("x, m (bench frame: along the bench)")
    ax.set_ylabel("y, m (across the bench)")


def heat_panel(ax, xs, ys, values, bench, vmax, title):
    step = xs[1] - xs[0]
    im = ax.imshow(values, origin="lower", extent=[xs[0] - step / 2, xs[-1] + step / 2, ys[0] - step / 2, ys[-1] + step / 2], cmap="inferno", vmin=0, vmax=vmax, interpolation="nearest")
    draw_bench(ax, bench)
    ax.set_title(title, fontsize=10)
    return im


def render(counts_npz, bench):
    d = np.load(counts_npz)
    xs, ys, total, per_bin = d["xs"], d["ys"], d["total"], d["per_bin"]
    vmax = int(total.max())
    fig, ax = plt.subplots(figsize=(9.5, 4.6))
    im = heat_panel(ax, xs, ys, total, bench, vmax, f"Parking spots that reach each bench point (filtered maps, pedestal {PEDESTAL_HEIGHT_M:.2f} m, point height {grasp_height() + bench['surface_z']:.3f} m)")
    fig.colorbar(im, ax=ax, label="parking spots (floor cells, any heading and arm)")
    fig.tight_layout()
    fig.savefig(OUT / "reach_heatmap_combined.png", dpi=130)
    plt.close(fig)
    for b in range(N_BINS):
        fig, ax = plt.subplots(figsize=(9.5, 4.6))
        im = heat_panel(ax, xs, ys, per_bin[b], bench, vmax, f"Approach bin {b:02d} (centre {b * 360 / N_BINS:.0f} deg from the robot's forward axis): parking spots that reach from this bin")
        fig.colorbar(im, ax=ax, label="parking spots (same colour scale as the combined map)")
        fig.tight_layout()
        fig.savefig(OUT / f"reach_heatmap_bin_{b:02d}.png", dpi=130)
        plt.close(fig)
    print("wrote combined and per-bin PNGs; vmax", vmax, flush=True)


def classify(row):
    """The failure type of one sweep layout, from its stop message (the sweep's own words).

    "repositioning": the plan needed the robot to move (navigation, transit or a second run) and the stance search could not give it.
    "no_cover": no stance covers the objects at all (the overlay found no spot).
    "full_check": stances were found, but the full check rejected every candidate.
    "pass": the layout passed.
    """
    if row["ok"]:
        return "pass"
    stop = row["stop"] or ""
    if any(w in stop for w in ("navigation", "transit", "second run", "run from")):
        return "repositioning"
    if stop.startswith("no stance covers"):
        return "no_cover"
    if stop.startswith("no stance passed the full check"):
        return "full_check"
    return "other"


def overlay_failures(sweep_json, counts_npz, bench):
    """Part c: the sweep's layouts on the combined heat map, marked by failure type; writes reach_heatmap_failures.png and a stats file."""
    d = json.loads(Path(sweep_json).read_text())
    c = np.load(counts_npz)
    xs, ys, total = c["xs"], c["ys"], c["total"]
    vmax = int(total.max())
    style = {"repositioning": ("magenta", "X"), "no_cover": ("cyan", "o"), "full_check": ("lime", "s"), "other": ("white", "^"), "pass": ("white", "*")}
    cells = {k: [] for k in style}
    for r in d["layouts"]:
        cells[classify(r)].append(r)
    fig, ax = plt.subplots(figsize=(9.5, 4.6))
    im = heat_panel(ax, xs, ys, total, bench, vmax, f"Layout sweep ({d['mode']} mode, {d['n']} layouts, {d['pass']} pass) on the combined reach heat map")
    for k, rows in cells.items():
        if not rows:
            continue
        col, mk = style[k]
        ax.scatter([r["beaker_xy"][0] for r in rows], [r["beaker_xy"][1] for r in rows], c=col, marker="o", s=18, edgecolors="black", linewidths=0.4, label=f"{k}: beaker ({len(rows)})")
        ax.scatter([r["hotplate_xy"][0] for r in rows], [r["hotplate_xy"][1] for r in rows], c=col, marker="s", s=18, edgecolors="black", linewidths=0.4, alpha=0.6, label=f"{k}: hotplate")
    ax.legend(fontsize=7, loc="upper right", framealpha=0.85)
    fig.colorbar(im, ax=ax, label="parking spots (floor cells, any heading and arm)")
    fig.tight_layout()
    fig.savefig(OUT / "reach_heatmap_failures.png", dpi=130)
    plt.close(fig)

    def count_at(xy):
        ix, iy = int(np.argmin(np.abs(xs - xy[0]))), int(np.argmin(np.abs(ys - xy[1])))
        return int(total[iy, ix])
    stats = {"sweep": sweep_json, "pass": d["pass"], "n": d["n"], "classes": {}, "all_usable_grid_median_count": None}
    for k, rows in cells.items():
        if rows:
            stats["classes"][k] = {"layouts": len(rows), "median_count_at_beaker": float(np.median([count_at(r["beaker_xy"]) for r in rows])),
                                   "median_count_at_hotplate": float(np.median([count_at(r["hotplate_xy"]) for r in rows]))}
    stats["all_usable_grid_median_count"] = float(np.median(total[(np.abs(xs[None, :]) <= bench["x"][1] - 0.05) & (np.abs(ys[:, None]) <= bench["y"][1] - 0.05)]))
    tmp = str(OUT / "reach_heatmap_failures_stats.json") + ".tmp"
    Path(tmp).write_text(json.dumps(stats, indent=1))
    os.replace(tmp, OUT / "reach_heatmap_failures_stats.json")
    print(json.dumps(stats, indent=1), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--geometry", action="store_true", help="one MuJoCo process: write data/reach_heatmap/geometry.json")
    ap.add_argument("--render", action="store_true", help="counts and PNGs from the filtered maps")
    ap.add_argument("--sweep", help="layout sweep JSON for part c")
    ap.add_argument("--step", type=float, default=BENCH_STEP_M)
    args = ap.parse_args()

    if args.geometry:
        from src.stance_finder import robot_geometry
        from src.task_beaker_hotplate import load_task
        sim = load_task()
        m = sim.model
        g = m.geom("bench_top").id
        rg = robot_geometry(m)
        out = {"shoulder": {k: [float(v) for v in rg["shoulder"][k]] for k in ("left", "right")}, "torso_z0": float(rg["torso_z0"]), "lift_factor": float(rg["lift_factor"]),
               "base_z": float(rg["base_z"]), "bench": {"centre": [float(v) for v in sim.data.geom_xpos[g]], "half": [float(v) for v in m.geom_size[g]]}}
        GEOMETRY_JSON.parent.mkdir(parents=True, exist_ok=True)
        tmp = str(GEOMETRY_JSON) + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(out, f, indent=1)
        os.replace(tmp, GEOMETRY_JSON)
        print("geometry", json.dumps(out), flush=True)
        return

    geom = json.loads(GEOMETRY_JSON.read_text())
    bench = bench_top({"centre": geom["bench"]["centre"], "half": geom["bench"]["half"]})
    if args.render or not args.sweep:
        maps = bs.load_maps(MAPS_DIR)
        assert len(maps) == 2 * N_BINS, f"expected {2 * N_BINS} maps in {MAPS_DIR}, got {len(maps)}"
        z_point = bench["surface_z"] + grasp_height()
        xs, ys, total, per_bin = project(maps, geom, bench, z_point, args.step, FLOOR_HALF_M, PEDESTAL_HEIGHT_M)
        tmp = str(COUNTS_NPZ) + ".tmp.npz"
        np.savez(tmp, xs=xs, ys=ys, total=total, per_bin=per_bin, z_point=z_point, pedestal=PEDESTAL_HEIGHT_M)
        os.replace(tmp, COUNTS_NPZ)
        print(f"counts: {len(xs)} x {len(ys)} bench points, reach point height {z_point:.4f} m, max {int(total.max())} parking spots", flush=True)
        render(COUNTS_NPZ, bench)
    if args.sweep:
        overlay_failures(args.sweep, COUNTS_NPZ, bench)


if __name__ == "__main__":
    main()
