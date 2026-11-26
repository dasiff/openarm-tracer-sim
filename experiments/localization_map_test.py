"""Tests of the localization map (navigation stage 2; design plan 13.1; decision row M345).

    MUJOCO_GL=egl python experiments/localization_map_test.py [--json PATH]

Exits 0 when every check passes. Checks:
  1 known geometry: the bench legs' cells are occupied, the cells just outside them are free, the wall lines are occupied, and the bench
    top's footprint away from the legs is free;
  2 the distance grid at five known points, against the distance worked out from the scene's box and cylinder geometry (one cell, 0.02 m);
  3 the lidar agreement (the key test): from the three stage 1 poses, each beam's range predicted by stepping through the map, against the
    noise-free stage 1 scan, for every beam that is not self-blocked, no return or too close;
  4 the floor mask: the three stage 1 poses stand on floor; a point in a bench leg and a point outside the room do not;
  5 the cache: a second load comes from the cache; a changed cell size rebuilds.
Check 6 (the manual baseline comparison) is run by hand; see the stage report.
"""

import argparse
import json
import math
import shutil
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

import lidar_test as lt  # noqa: E402
from src import robot_specs  # noqa: E402
from src.lidar import Lidar, RANGE, SELF_BLOCKED, STATE_NAMES  # noqa: E402
from src.localization_map import (FREE, OCCUPIED, OUTSIDE, cell_of, fixed_geoms, inside_primitive,  # noqa: E402
                                  load_or_build_localization_map, predict_ranges)
from src.task_config import TASK_CFG  # noqa: E402

RESULTS = []
TEST_CACHE = ROOT / "data" / "logs" / "localization_map_test_cache"
POSES = None  # filled in main(): the three stage 1 poses


def check(name, ok, detail):
    """Records and prints one check."""
    RESULTS.append({"check": name, "pass": bool(ok), "detail": detail})
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}", flush=True)


def analytic_distance(model, data, geoms, z, px, py):
    """The distance from a point to the nearest surface of the given geoms at height z, worked out from their geometry (not from the map).

    Boxes (axis-aligned in this scene), cylinders and capsules are handled exactly at the plane's height; the point is outside them all.
    """
    p = np.array([px, py, z])
    best = math.inf
    for g in geoms:
        centre = data.geom_xpos[g]
        rot = data.geom_xmat[g].reshape(3, 3)
        local = rot.T @ (p - centre)
        size = model.geom_size[g]
        kind = int(model.geom_type[g])
        corners = centre + (model.geom_aabb[g, :3] + np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]) * model.geom_aabb[g, 3:]) @ rot.T
        if not (corners[:, 2].min() <= z <= corners[:, 2].max()):
            continue
        if kind == 6:  # box
            dx = max(abs(local[0]) - size[0], 0.0)
            dy = max(abs(local[1]) - size[1], 0.0)
            d = math.hypot(dx, dy)
        elif kind in (3, 5):  # capsule / cylinder: radius size[0] around a vertical axis
            d = max(math.hypot(local[0], local[1]) - size[0], 0.0)
        else:  # sphere
            d = max(float(np.linalg.norm(local)) - size[0], 0.0)
        best = min(best, d)
    return best


def check_geometry(maps, model, data):
    """Check 1: the bench legs, the walls and the bench top's footprint."""
    failures = []
    for name, (cx, cy) in [("fl", (-2.0, 0.7)), ("fr", (2.0, 0.7)), ("bl", (-2.0, -0.7)), ("br", (2.0, -0.7))]:
        ix, iy, on = cell_of(maps, (cx, cy))
        occ = on and maps["state"][iy, ix] == OCCUPIED
        out_x = cx - 0.10 * math.copysign(1.0, cx)  # 0.04 m beyond the leg's outer face
        jx, jy, on2 = cell_of(maps, (out_x, cy))
        free = on2 and maps["state"][jy, jx] == FREE
        if not (occ and free):
            failures.append(f"leg {name}: centre occupied {occ}, outside free {free}")
    for label, (x, y) in [("east wall", (5.99, 0.0)), ("west wall", (-5.99, 0.0)), ("north wall", (0.0, 5.99)), ("south wall", (0.0, -5.99))]:
        ix, iy, on = cell_of(maps, (x, y))
        if not (on and maps["state"][iy, ix] == OCCUPIED):
            failures.append(f"{label} not occupied")
    for label, (x, y) in [("bench top, centre", (0.0, 0.0)), ("bench top, right of centre", (1.0, 0.3))]:
        ix, iy, on = cell_of(maps, (x, y))
        if not (on and maps["state"][iy, ix] == FREE):
            failures.append(f"{label} not free")
    check("1 known geometry: leg cells occupied, cells outside the legs free, walls occupied, bench top footprint free",
          not failures, "all as expected" if not failures else "; ".join(failures))


def check_distances(maps, model, data, z, geoms):
    """Check 2: the stored distance at five known points against the geometry, one cell (0.02 m) of tolerance."""
    points = [("1.0 m from the east wall", (4.92, 0.0)), ("between the front legs, 0.7 m out", (0.0, 0.7)),
              ("0.5 m from the front-left leg", (-1.44, 0.7)), ("0.3 m from the south wall", (0.0, -5.62)),
              ("in front of the back counter", (0.0, -2.0))]
    rows, worst = [], 0.0
    for label, (x, y) in points:
        ix, iy, on = cell_of(maps, (x, y))
        stored = float(maps["distance"][iy, ix]) if on else float("nan")
        expected = analytic_distance(model, data, geoms, z, x, y)
        err = abs(stored - expected)
        worst = max(worst, err)
        rows.append(f"{label}: stored {stored:.4f} m, geometry {expected:.4f} m")
    check("2 distance grid: five known points within one cell (0.02 m)", worst <= 0.02,
          f"worst difference {worst:.4f} m; " + " | ".join(rows))


def check_lidar_agreement(maps, sim, lidar_seeded, base_z):
    """Check 3: the predicted ranges (from the map) against the stage 1 noise-free scan, at the three poses."""
    m, d = sim.model, sim.data
    total_usable = total_agree = 0
    worst_all = []
    per_pose = []
    for label, x, y, yaw in POSES:
        lt.place(sim, x, y, yaw, base_z, TASK_CFG["pedestal_q"])
        lidar = Lidar(m, seed=1, noise=False)
        scan = lidar.scan(d)
        dirs = np.stack([np.cos(scan.yaw + lidar.beam_angles), np.sin(scan.yaw + lidar.beam_angles)], axis=1)
        predicted = predict_ranges(maps, scan.origin[:2], dirs, lidar.specs["max_range_m"], step=0.002)
        usable = scan.states == RANGE
        real = scan.hit_dist
        err = np.abs(predicted - real)
        agree = usable & np.isfinite(predicted) & (err <= 0.02)
        n_usable, n_agree = int(usable.sum()), int(agree.sum())
        total_usable += n_usable
        total_agree += n_agree
        per_pose.append(f"{label}: {n_agree} of {n_usable} beams agree ({100.0 * n_agree / max(n_usable, 1):.1f} %)")
        order = [k for k in np.argsort(np.where(usable & np.isfinite(err), -err, np.inf)) if usable[k]]
        for k in order[:3]:
            hit = m.geom(scan.hit_geom[k]).name
            worst_all.append((float(err[k]), label, int(k), float(predicted[k]), float(real[k]), hit))
        if label == POSES[0][0]:
            pass
    share = 100.0 * total_agree / max(total_usable, 1)
    worst_all.sort(key=lambda r: -r[0])
    top = "; ".join(f"beam {k} deg at {lab} ({hit}): predicted {p:.4f} m, real {r:.4f} m, off by {e * 1000:.1f} mm"
                    for e, lab, k, p, r, hit in worst_all[:3])
    check("3 lidar agreement: predicted and real ranges agree within 0.02 m for every usable beam",
          total_agree == total_usable,
          f"{total_agree} of {total_usable} usable beams agree ({share:.2f} %). " + " | ".join(per_pose) + ". Worst three: " + top)
    return {"agree": total_agree, "usable": total_usable, "share_pct": share,
            "worst": [{"error_m": e, "pose": lab, "beam_deg": k, "predicted_m": p, "real_m": r, "hit": hit}
                      for e, lab, k, p, r, hit in worst_all[:3]]}


def check_floor(maps, sim):
    """Check 4: the stage 1 poses stand on floor; a bench leg and a point outside the room do not."""
    bad = []
    for label, x, y, _ in POSES:
        ix, iy, on = cell_of(maps, (x, y))
        if not (on and maps["floor"][iy, ix]):
            bad.append(f"{label} not on floor")
    ix, iy, on = cell_of(maps, (-2.0, 0.7))
    if on and maps["floor"][iy, ix]:
        bad.append("bench leg counted as floor")
    ix, iy, on = cell_of(maps, (7.0, 0.0))
    if on and maps["floor"][iy, ix]:
        bad.append("outside the room counted as floor")
    check("4 free-floor mask: the three stage 1 poses are floor; a bench leg and a point outside the room are not", not bad,
          "as expected" if not bad else "; ".join(bad))


def main():
    """Runs the checks; prints a summary; exits 1 if any fails."""
    global POSES
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default=None)
    args = ap.parse_args()
    t0 = time.time()
    sim = lt.load_task()
    m, d = sim.model, sim.data
    base_z = lt.settle_base_height(sim)
    sx, sy = TASK_CFG["staging_xy"]
    back = TASK_CFG["staging_approach_back_m"]
    POSES = [("staging stance", sx, sy, -math.pi / 2), ("staging approach spot", sx, sy + back, -math.pi / 2),
             ("middle of the room", 0.0, -2.2, 0.0)]
    lt.place(sim, sx, sy, -math.pi / 2, base_z, TASK_CFG["pedestal_q"])
    if TEST_CACHE.exists():
        shutil.rmtree(TEST_CACHE)
    maps, source, build_s = load_or_build_localization_map(m, d, cache_dir=TEST_CACHE)
    meta = maps["meta"]
    z = meta["slice_z_m"]
    geoms = fixed_geoms(m)
    print(f"map: {meta['shape'][1]} x {meta['shape'][0]} cells of {meta['cell_m']} m, x {meta['x0']:.3f} to {meta['x0'] + meta['shape'][1] * meta['cell_m']:.3f} m, "
          f"y {meta['y0']:.3f} to {meta['y0'] + meta['shape'][0] * meta['cell_m']:.3f} m; slice height {z:.4f} m; "
          f"{meta['fixed_geoms']} fixed geoms; source {source} in {build_s:.2f} s", flush=True)
    check_geometry(maps, m, d)
    check_distances(maps, m, d, z, geoms)
    agreement = check_lidar_agreement(maps, sim, None, base_z)
    check_floor(maps, sim)
    # check 5: the cache
    cache_file = TEST_CACHE / "localization_map.npz"
    t1 = time.time()
    cached_maps, source2, _ = load_or_build_localization_map(m, d, cache_dir=TEST_CACHE)
    cache_s = time.time() - t1
    same = np.array_equal(cached_maps["state"], maps["state"]) and np.array_equal(cached_maps["floor"], maps["floor"]) \
        and np.array_equal(cached_maps["distance"], maps["distance"])
    check("5 cache: a second load comes from the cache and returns the same grids", source2 == "cache" and same,
          f"source {source2}, load {cache_s:.3f} s (build took {build_s:.2f} s); file {cache_file.stat().st_size / 1e6:.2f} MB")
    changed = {**robot_specs.LOCALIZATION_MAP_SPECS, "cell_m": 0.03}
    _, source3, build3 = load_or_build_localization_map(m, d, specs=changed, cache_dir=TEST_CACHE)
    check("5 cache: a changed cell size (test copy of the config) rebuilds the maps", source3 == "built",
          f"source {source3} with cell 0.03 m, built in {build3:.2f} s")
    failed = [r["check"] for r in RESULTS if not r["pass"]]
    print(f"{len(RESULTS) - len(failed)} of {len(RESULTS)} checks pass; {time.time() - t0:.1f} s", flush=True)
    if args.json:
        Path(args.json).write_text(json.dumps({"results": RESULTS, "agreement": agreement, "meta": meta, "build_s": build_s,
                                               "cache_load_s": cache_s, "cache_file_mb": cache_file.stat().st_size / 1e6}, indent=1, default=str))
    if failed:
        print("FAILED: " + "; ".join(failed), flush=True)
        sys.exit(1)
    sys.exit(0)


if __name__ == "__main__":
    main()
