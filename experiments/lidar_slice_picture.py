"""Top-down picture of the lidar's view at the scan plane height (navigation stage 1; design plan 13.1; rows M328, M341).

    MUJOCO_GL=egl python experiments/lidar_slice_picture.py [--out PATH]

Three panels: the staging stance, the staging approach spot (0.2063 m behind it), and a pose in the middle of the room.
The grey areas are what crosses the scan plane: each non-robot primitive geom of the compiled model is tested exactly at the plane's
height (boxes, cylinders, capsules and spheres; a mesh would be listed as not drawable). The bench top is drawn only as a dashed outline, labelled
"bench top, above scan plane". Blue dots are the range returns; grey dots are "too close"; orange lines and dots are beams that
are self-blocked (they stop at the robot's own part, shown as the orange end). The robot's chassis is drawn as its bounding box.
"""

import argparse
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

import lidar_test as lt  # noqa: E402
from src.lidar import Lidar, RANGE, SELF_BLOCKED, TOO_CLOSE  # noqa: E402
from src.localization_map import inside_primitive  # noqa: E402  (the one exact containment test, shared with the localization map)
from src.task_beaker_hotplate import load_task  # noqa: E402
from src.task_config import TASK_CFG  # noqa: E402

GRID_M = 0.03  # m: the spacing of the grid on which the slice is computed
EDGE_X = -6.5  # m: a start point outside the room's walls (the walls are at x = -6)


GEOM_SPHERE, GEOM_CAPSULE, GEOM_CYLINDER, GEOM_BOX = 2, 3, 5, 6  # MuJoCo geom type codes for the primitives the scene uses
SIGNS = np.array([[a, b, c] for a in (-1.0, 1.0) for b in (-1.0, 1.0) for c in (-1.0, 1.0)])


def solid_mask(model, data, robot, z, xs, ys):
    """What crosses the plane at height z, on the grid (rows ys, columns xs): True where a non-robot primitive geom contains the point.

    Each primitive is tested exactly (boxes, cylinders, capsules, spheres); a geom that does not reach the plane is skipped.
    Returns (mask, names of geoms the picture cannot handle, mesh geoms that reach the plane).
    """
    out = np.zeros((len(ys), len(xs)), dtype=bool)
    unsupported = []
    for g in range(model.ngeom):
        if robot[g] or model.geom_type[g] == 0:  # the robot is drawn by its pose; a plane (the floor) never crosses a height above it
            continue
        centre = data.geom_xpos[g]
        rot = data.geom_xmat[g].reshape(3, 3)
        corners = centre + (model.geom_aabb[g, :3] + SIGNS * model.geom_aabb[g, 3:]) @ rot.T
        if not (corners[:, 2].min() <= z <= corners[:, 2].max()):
            continue
        kind = int(model.geom_type[g])
        if kind not in (GEOM_BOX, GEOM_SPHERE, GEOM_CYLINDER, GEOM_CAPSULE):
            unsupported.append(model.geom(g).name or f"geom {g}")
            continue
        i0 = int(np.searchsorted(xs, corners[:, 0].min(), side="left"))
        i1 = int(np.searchsorted(xs, corners[:, 0].max(), side="right"))
        j0 = int(np.searchsorted(ys, corners[:, 1].min(), side="left"))
        j1 = int(np.searchsorted(ys, corners[:, 1].max(), side="right"))
        if i1 <= i0 or j1 <= j0:
            continue
        gx, gy = np.meshgrid(xs[i0:i1], ys[j0:j1])
        points = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, z)], axis=1) - centre
        inside = inside_primitive(kind, model.geom_size[g], points @ rot)  # the geom's own frame: rot.T applied to each point
        out[j0:j1, i0:i1] |= inside.reshape(gx.shape)
    return out, unsupported


def footprint_box(model, data, body_name):
    """The world bounding box (x0, x1, y0, y1) of the body's chassis mesh."""
    g = model.geom("base_link_geom").id
    signs = np.array([[sx, sy, sz] for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)])
    geom_rot = np.zeros(9)
    mujoco.mju_quat2Mat(geom_rot, model.geom_quat[g])
    corners = model.geom_pos[g] + (model.geom_aabb[g, :3] + signs * model.geom_aabb[g, 3:]) @ geom_rot.reshape(3, 3).T  # base frame
    b = model.body(body_name).id
    world = data.xpos[b] + corners @ data.xmat[b].reshape(3, 3).T
    return world[:, 0].min(), world[:, 0].max(), world[:, 1].min(), world[:, 1].max()


def draw_panel(ax, mask, extent, scan, lidar, pose_label, centre, box, bench_box, title):
    """One panel: the grey slice, the bench outline, the robot, and the scan's points."""
    ax.imshow(mask, origin="lower", extent=extent, cmap="Greys", vmin=0, vmax=1.6, interpolation="nearest")
    x0, x1, y0, y1 = bench_box
    ax.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0], "--", color="#4a7fb0", lw=1.0,
            label="bench top, above scan plane (outline only)")
    bx0, bx1, by0, by1 = box
    ax.plot([bx0, bx1, bx1, bx0, bx0], [by0, by0, by1, by1, by0], "-", color="#d07a00", lw=1.2, label="robot chassis (bounding box)")
    origin = scan.origin
    dirs = np.stack([np.cos(scan.yaw + lidar.beam_angles), np.sin(scan.yaw + lidar.beam_angles)], axis=1)
    rng_mask = scan.states == RANGE
    close_mask = scan.states == TOO_CLOSE
    blocked = scan.states == SELF_BLOCKED
    pts = origin[:2] + dirs[rng_mask] * scan.hit_dist[rng_mask, None]
    ax.plot(pts[:, 0], pts[:, 1], ".", ms=3, color="#1f5fbf", label="range return")
    if np.any(close_mask):
        cp = origin[:2] + dirs[close_mask] * scan.hit_dist[close_mask, None]
        ax.plot(cp[:, 0], cp[:, 1], ".", ms=3, color="#888888", label="too close")
    for k in np.flatnonzero(blocked):
        end = origin[:2] + dirs[k] * scan.hit_dist[k]
        ax.plot([origin[0], end[0]], [origin[1], end[1]], "-", color="#e0401c", lw=0.4, alpha=0.6)
        ax.plot(end[0], end[1], "o", ms=2.5, color="#e0401c")
    ax.plot(origin[0], origin[1], "+", color="black", ms=9, mew=1.5, label="lidar")
    ax.annotate("", xy=(centre[0] + 0.35 * math.cos(scan.yaw), centre[1] + 0.35 * math.sin(scan.yaw)), xytext=centre,
                arrowprops=dict(arrowstyle="->", color="black", lw=1.2))
    ax.set_xlim(centre[0] - 3.2, centre[0] + 3.2)
    ax.set_ylim(centre[1] - 3.2, centre[1] + 3.2)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"{title}\n{pose_label}", fontsize=9)
    ax.grid(alpha=0.2)


def main():
    """Builds the picture and saves it; prints the scan plane height."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "docs" / "reports" / "20261125_lidar_scan_slice.png"))
    args = ap.parse_args()
    sim = load_task()
    m, d = sim.model, sim.data
    base_z = lt.settle_base_height(sim)
    lidar = Lidar(m, seed=1, noise=False)
    robot = lidar.robot_geom
    sx, sy = TASK_CFG["staging_xy"]
    back = TASK_CFG["staging_approach_back_m"]
    poses = [("staging stance", sx, sy, -math.pi / 2),
             ("staging approach spot (0.2063 m behind the stance)", sx, sy + back, -math.pi / 2),
             ("middle of the room (open floor)", 0.0, -2.2, 0.0)]
    lt.place(sim, sx, sy, -math.pi / 2, base_z, TASK_CFG["pedestal_q"])
    plane_z = float(lidar.scan(d).origin[2])
    xs = np.arange(-5.9, 5.9 + 1e-9, GRID_M)
    ys = np.arange(-5.9, 5.9 + 1e-9, GRID_M)
    print(f"scan plane height above the floor: {plane_z:.4f} m (deck top {lidar.deck_top_z:.4f} m above the base origin, "
          f"base origin {base_z:.4f} m above the floor)", flush=True)
    print(f"computing the slice on {len(xs)} x {len(ys)} points ...", flush=True)
    mask, unsupported = solid_mask(m, d, robot, plane_z, xs, ys)
    if unsupported:
        print("geoms the picture cannot draw (not primitives): " + ", ".join(unsupported), flush=True)
    extent = (xs[0], xs[-1], ys[0], ys[-1])
    bench_geom = m.geom("bench_top").id  # the bench top's outline, from the compiled model (the top is above the scan plane)
    bench_centre, bench_half = d.geom_xpos[bench_geom], m.geom_size[bench_geom]
    bench_box = (bench_centre[0] - bench_half[0], bench_centre[0] + bench_half[0],
                 bench_centre[1] - bench_half[1], bench_centre[1] + bench_half[1])
    fig, axes = plt.subplots(1, 3, figsize=(16, 6.2))
    summary = []
    for ax, (label, x, y, yaw) in zip(axes, poses):
        lt.place(sim, x, y, yaw, base_z, TASK_CFG["pedestal_q"])
        scan = lidar.scan(d)
        box = footprint_box(m, d, "base_link")
        n_range = int(np.sum(scan.states == RANGE))
        n_block = int(np.sum(scan.states == SELF_BLOCKED))
        summary.append(f"{label}: range {n_range}, self-blocked {n_block}, too close {int(np.sum(scan.states == TOO_CLOSE))}")
        draw_panel(ax, mask, extent, scan, lidar, f"base origin ({x:.3f}, {y:.3f}) m, heading {math.degrees(yaw):.0f} deg",
                   (x, y), box, bench_box, label)
    axes[0].legend(loc="lower left", fontsize=7)
    fig.suptitle(f"Simulated lidar at the scan plane, {plane_z:.3f} m above the floor (placeholder unit, PROVISIONAL, M328). "
                 "Grey: what crosses the plane. Orange: self-blocked beams.", fontsize=10)
    fig.tight_layout()
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    tmp = str(Path(args.out)).replace(".png", ".tmp.png")
    fig.savefig(tmp, dpi=110)
    Path(tmp).replace(args.out)
    print("\n".join(summary), flush=True)
    print(f"picture: {args.out}", flush=True)


if __name__ == "__main__":
    main()
