"""The localization map: a horizontal slice of the fixed scene at the lidar's scan plane, with a distance-lookup grid (navigation stage 2).

Design plan section 13.1 (the two-maps rule); decision row M345. The particle filter (a later stage) compares each lidar scan with this
map, as AMCL does: a beam's predicted range is found by stepping along it until it reaches an occupied cell, and the stored distance lets
it measure how far a scan point is from the nearest obstacle.

Two maps, two jobs (design plan 13.1):
  * The planner grid (src/nav_grid.py, 5 cm cells, the A* planner) is for collision-free paths. It is not changed by this module.
  * This module's map is one slice at the lidar's scan plane height (LIDAR_SPECS and the model, as src/lidar.py). It holds only what
    the lidar can see. The bench is open underneath, so it appears as its legs only.

Build: the fixed scene's geoms (not the robot, not anything a task moves: a body with a joint, such as the beaker or the hotplate, is
left out) are tested exactly at the plane height, cell by cell: a cell is occupied when its centre lies inside a primitive geom (box,
cylinder, capsule or sphere). The same test is used by experiments/lidar_slice_picture.py.

Arrays (rows are y, columns are x; cell (iy, ix) has its centre at x0 + (ix + 0.5) * cell, y0 + (iy + 0.5) * cell):
  state     uint8: OUTSIDE (0) outside the room's interior and not occupied; FREE (1); OCCUPIED (2).
  distance  float32, metres: from the cell centre to the nearest occupied cell centre (exact Euclidean distance transform, no cap);
            0 on occupied cells.
  floor     bool: the robot's base centre may stand here. It is the planner grid's free space (src/nav_grid.NavGrid, with the robot's
            footprint at the drive pedestal height, free for all 8 headings) on the cell that holds this cell's centre, and the cell is
            FREE.

Cache: data/localization_map/localization_map.npz (path from LOCALIZATION_MAP_SPECS). It holds a staleness key, a SHA-256 over the version,
the cell size, the margin, the slice height, the room, the planner settings and the geometry of every fixed geom. A cache whose key differs
is rebuilt.
"""

import hashlib
import json
import math
import os
import time
from pathlib import Path

import mujoco
import numpy as np
from scipy import ndimage

from src import robot_specs
from src.lidar import robot_geom_mask

ROOT = Path(__file__).resolve().parents[1]
VERSION = "localization_map_v1"  # changing this string forces a rebuild of every cache
OUTSIDE, FREE, OCCUPIED = 0, 1, 2
GEOM_PLANE, GEOM_SPHERE, GEOM_CAPSULE, GEOM_CYLINDER, GEOM_BOX = 0, 2, 3, 5, 6
EXCLUDED_NAME_PREFIXES = ("fc_slot", "task_beaker", "task_hotplate")  # object slots and task objects, as src/nav_footprint.obstacles_from_model
_SIGNS = np.array([[a, b, c] for a in (-1.0, 1.0) for b in (-1.0, 1.0) for c in (-1.0, 1.0)])


def inside_primitive(kind, size, local):
    """Says which points lie inside a primitive geom: the exact test used for every cell.

    Args:
        kind (int): The geom's MuJoCo type: GEOM_BOX, GEOM_CYLINDER, GEOM_CAPSULE or GEOM_SPHERE.
        size (numpy.ndarray): The geom's size as MuJoCo stores it (box: half sizes; cylinder and capsule: radius, half length; sphere: radius).
        local (numpy.ndarray): (N, 3) points in the geom's own frame, metres.

    Returns:
        numpy.ndarray: Boolean, N values.
    """
    if kind == GEOM_BOX:
        return np.all(np.abs(local) <= size, axis=1)
    if kind == GEOM_SPHERE:
        return np.linalg.norm(local, axis=1) <= size[0]
    if kind == GEOM_CYLINDER:
        return (np.hypot(local[:, 0], local[:, 1]) <= size[0]) & (np.abs(local[:, 2]) <= size[1])
    zc = np.clip(local[:, 2], -size[1], size[1])  # capsule: a segment of half length size[1] along z, radius size[0]
    return np.linalg.norm(local - np.stack([np.zeros_like(zc), np.zeros_like(zc), zc], axis=1), axis=1) <= size[0]


def _has_free_joint_above(model, body):
    """True when the body, or a body above it, has a free joint: it can move anywhere (the robot's base, a task object such as the beaker)."""
    b = int(body)
    while b > 0:
        for j in range(int(model.body_jntadr[b]), int(model.body_jntadr[b]) + int(model.body_jntnum[b])):
            if int(model.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE):
                return True
        b = int(model.body_parentid[b])
    return False


def fixed_geoms(model):
    """The ids of the fixed scene's geoms the lidar can see: not the robot, not a plane, not a free-moving object or an object slot.

    A body with a hinge or slide joint (a cabinet door, say) is kept: it is fixed to the scene, and the lidar sees it.

    Args:
        model (mujoco.MjModel): The compiled model.

    Returns:
        list of int: The geom ids, in model order.
    """
    robot = robot_geom_mask(model)
    out = []
    for g in range(model.ngeom):
        if robot[g] or int(model.geom_type[g]) == GEOM_PLANE:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        if name.startswith(EXCLUDED_NAME_PREFIXES) or _has_free_joint_above(model, model.geom_bodyid[g]):
            continue
        out.append(g)
    return out


def room_bounds(model, data):
    """The room's interior: the inside faces of the four walls (geoms wall_w, wall_e, wall_s, wall_n), and the walls' outside faces.

    Args:
        model (mujoco.MjModel): The compiled model.
        data (mujoco.MjData): Its data (the walls are fixed, so their positions are the model's).

    Returns:
        tuple: (xmin, xmax, ymin, ymax) of the interior, and (ox0, ox1, oy0, oy1) of the walls' outside faces, metres.
    """
    def box(name):
        g = model.geom(name).id
        return data.geom_xpos[g], model.geom_size[g]
    (pw, sw), (pe, se), (ps, ss), (pn, sn) = box("wall_w"), box("wall_e"), box("wall_s"), box("wall_n")
    inner = (pw[0] + sw[0], pe[0] - se[0], ps[1] + ss[1], pn[1] - sn[1])
    outer = (pw[0] - sw[0], pe[0] + se[0], ps[1] - ss[1], pn[1] + sn[1])
    return inner, outer


def plane_height(model, data, specs=None):
    """The scan plane's height above the floor, metres: the lidar's origin height at the robot's settled base (as src/lidar.py makes it)."""
    from src.lidar import deck_mount
    specs = robot_specs.LIDAR_SPECS if specs is None else specs
    offset, _, _ = deck_mount(model, specs["mount_height_above_deck_m"])
    base = model.body("base_link").id
    return float(data.xpos[base][2] + offset[2])


def slice_occupancy(model, data, geom_ids, z, xs, ys):
    """Which cells of the grid have their centre inside one of the given geoms at height z.

    Args:
        model (mujoco.MjModel): The compiled model.
        data (mujoco.MjData): Its data (the geoms' world poses).
        geom_ids (iterable of int): The geoms to test.
        z (float): The plane's height, metres.
        xs (numpy.ndarray): The cell-centre x values (columns), metres, increasing.
        ys (numpy.ndarray): The cell-centre y values (rows), metres, increasing.

    Returns:
        numpy.ndarray: Boolean (len(ys), len(xs)), True where a cell centre is inside a geom. Geoms that do not reach z are skipped.
    """
    out = np.zeros((len(ys), len(xs)), dtype=bool)
    for g in geom_ids:
        centre = data.geom_xpos[g]
        rot = data.geom_xmat[g].reshape(3, 3)
        corners = centre + (model.geom_aabb[g, :3] + _SIGNS * model.geom_aabb[g, 3:]) @ rot.T
        if not (corners[:, 2].min() <= z <= corners[:, 2].max()):
            continue
        i0 = int(np.searchsorted(xs, corners[:, 0].min(), side="left"))
        i1 = int(np.searchsorted(xs, corners[:, 0].max(), side="right"))
        j0 = int(np.searchsorted(ys, corners[:, 1].min(), side="left"))
        j1 = int(np.searchsorted(ys, corners[:, 1].max(), side="right"))
        if i1 <= i0 or j1 <= j0:
            continue
        gx, gy = np.meshgrid(xs[i0:i1], ys[j0:j1])
        points = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, z)], axis=1) - centre
        inside = inside_primitive(int(model.geom_type[g]), model.geom_size[g], points @ rot)
        out[j0:j1, i0:i1] |= inside.reshape(gx.shape)
    return out


def planner_floor(model, data, xs, ys, pedestal_q):
    """Where the robot's base centre may stand, from the planner grid's free space (reused as it is: src/nav_grid.NavGrid).

    The planner's footprint (src/nav_footprint.robot_footprint) and obstacles (src/nav_footprint.obstacles_from_model) give its grid;
    a base centre may stand on a planner cell that is free for every heading (blocked_for is false for k = 0 to 7).

    Args:
        model (mujoco.MjModel): The compiled model.
        data (mujoco.MjData): Its data with the arms at home (the footprint is taken from it).
        xs (numpy.ndarray): Cell-centre x values, metres.
        ys (numpy.ndarray): Cell-centre y values, metres.
        pedestal_q (float): The pedestal command the footprint is taken at (the drive height).

    Returns:
        tuple: (boolean grid (len(ys), len(xs)) of the planner's free-for-all-headings cells at each localization cell's centre,
        footprint polygon as a numpy array, planner grid origin and cell size, the planner's shape).
    """
    from src.nav_footprint import obstacles_from_model, robot_footprint
    from src.nav_grid import NavGrid
    from src.task_config import NAV_CFG
    fp = robot_footprint(model, data, pedestal_q)
    obstacles = obstacles_from_model(model, data)
    xo = np.concatenate([np.asarray(o["outline"])[:, 0] for o in obstacles])
    yo = np.concatenate([np.asarray(o["outline"])[:, 1] for o in obstacles])
    bounds = (math.floor(xo.min()) - 0.5, math.ceil(xo.max()) + 0.5, math.floor(yo.min()) - 0.5, math.ceil(yo.max()) + 0.5)
    grid = NavGrid(fp["polygon"], NAV_CFG, bounds, fp["h_top"])
    grid.rebuild(obstacles)
    free_all = ~np.logical_or.reduce([grid.blocked_for(k) for k in range(8)])
    ix = np.floor((xs - grid.origin[0]) / grid.res).astype(int)
    iy = np.floor((ys - grid.origin[1]) / grid.res).astype(int)
    inside_x = (ix >= 0) & (ix < grid.shape[1])
    inside_y = (iy >= 0) & (iy < grid.shape[0])
    floor = np.zeros((len(ys), len(xs)), dtype=bool)
    rows = np.flatnonzero(inside_y)
    cols = np.flatnonzero(inside_x)
    floor[np.ix_(rows, cols)] = free_all[np.ix_(iy[rows], ix[cols])]
    return floor, np.asarray(fp["polygon"], float), (grid.origin.copy(), grid.res, grid.shape)


def _staleness_key(model, data, specs, geom_ids, z, inner, outer, footprint, pedestal_q):
    """The SHA-256 of everything the maps depend on; a cache with another key is rebuilt."""
    from src.task_config import NAV_CFG
    h = hashlib.sha256()
    h.update(json.dumps({"version": VERSION, "cell_m": specs["cell_m"], "margin_m": specs["margin_m"],
                         "slice_z_m": round(z, 12), "room_inner": [round(v, 12) for v in inner],
                         "walls_outer": [round(v, 12) for v in outer], "pedestal_q": pedestal_q,
                         "nav_cell": NAV_CFG["cell"], "nav_min_block_height": NAV_CFG["min_block_height"]},
                        sort_keys=True).encode())
    for g in geom_ids:
        h.update((mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or "").encode())
        h.update(np.array([int(model.geom_type[g])], dtype=np.int64).tobytes())
        h.update(np.ascontiguousarray(model.geom_size[g], dtype=np.float64).tobytes())
        h.update(np.ascontiguousarray(data.geom_xpos[g], dtype=np.float64).tobytes())
        h.update(np.ascontiguousarray(data.geom_xmat[g], dtype=np.float64).tobytes())
    h.update(np.ascontiguousarray(footprint, dtype=np.float64).tobytes())
    return h.hexdigest()


def _grid_axes(outer, margin, cell):
    """The cell-centre axes of the map: the walls' outside faces plus the margin, in whole cells."""
    x0, x1 = outer[0] - margin, outer[1] + margin
    y0, y1 = outer[2] - margin, outer[3] + margin
    nx = int(round((x1 - x0) / cell))
    ny = int(round((y1 - y0) / cell))
    xs = x0 + (np.arange(nx) + 0.5) * cell
    ys = y0 + (np.arange(ny) + 0.5) * cell
    return xs, ys, (x0, y0)


def build_localization_map(model, data, specs=None, pedestal_q=None):
    """Builds the three grids from the scene (no cache).

    Args:
        model (mujoco.MjModel): The compiled model with the robot and the scene.
        data (mujoco.MjData): Its data with the kinematics up to date, the arms at home.
        specs (dict or None): LOCALIZATION_MAP_SPECS. Defaults to robot_specs.LOCALIZATION_MAP_SPECS.
        pedestal_q (float or None): The pedestal command for the planner footprint. Defaults to TASK_CFG's pedestal_q.

    Returns:
        dict: "state", "distance", "floor" (the arrays), and "meta": x0, y0, cell_m, shape, slice_z_m, room_inner, walls_outer,
        fixed_geoms (count), excluded (not used), key, build_s.
    """
    from src.task_config import TASK_CFG
    specs = dict(robot_specs.LOCALIZATION_MAP_SPECS if specs is None else specs)
    pedestal_q = TASK_CFG["pedestal_q"] if pedestal_q is None else pedestal_q
    t0 = time.time()
    cell = float(specs["cell_m"])
    z = plane_height(model, data)
    inner, outer = room_bounds(model, data)
    xs, ys, (x0, y0) = _grid_axes(outer, specs["margin_m"], cell)
    geoms = fixed_geoms(model)
    floor_planner, footprint, _ = planner_floor(model, data, xs, ys, pedestal_q)
    key = _staleness_key(model, data, specs, geoms, z, inner, outer, footprint, pedestal_q)
    occupied = slice_occupancy(model, data, geoms, z, xs, ys)
    X, Y = np.meshgrid(xs, ys)
    in_room = (X > inner[0]) & (X < inner[1]) & (Y > inner[2]) & (Y < inner[3])
    state = np.where(occupied, OCCUPIED, np.where(in_room, FREE, OUTSIDE)).astype(np.uint8)
    distance = ndimage.distance_transform_edt(~occupied) * cell
    floor = floor_planner & (state == FREE)
    return {"state": state, "distance": distance.astype(np.float32), "floor": floor,
            "meta": {"version": VERSION, "x0": float(x0), "y0": float(y0), "cell_m": cell, "shape": [int(s) for s in state.shape],
                     "slice_z_m": float(z), "room_inner": [float(v) for v in inner], "walls_outer": [float(v) for v in outer],
                     "fixed_geoms": len(geoms), "key": key, "build_s": round(time.time() - t0, 3)}}


def cache_path(specs=None):
    """The cache file's path: LOCALIZATION_MAP_SPECS["cache_dir"] under the project root, file localization_map.npz."""
    specs = robot_specs.LOCALIZATION_MAP_SPECS if specs is None else specs
    return ROOT / specs["cache_dir"] / "localization_map.npz"


def _save(path, maps):
    """Writes the maps to a temporary file and renames it over the old one (an interrupted write leaves the old file intact)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.stem + ".tmp.npz")
    np.savez_compressed(tmp, state=maps["state"], distance=maps["distance"], floor=maps["floor"], meta=np.array(json.dumps(maps["meta"])))
    os.replace(tmp, path)


def _load(path):
    """Reads a cache file written by _save()."""
    with np.load(path) as f:
        return {"state": f["state"], "distance": f["distance"], "floor": f["floor"], "meta": json.loads(str(f["meta"]))}


def load_or_build_localization_map(model, data, specs=None, cache_dir=None, pedestal_q=None):
    """Returns the maps from the cache when its staleness key matches the scene; otherwise builds them and writes the cache.

    Args:
        model (mujoco.MjModel): The compiled model.
        data (mujoco.MjData): Its data (kinematics up to date, arms at home).
        specs (dict or None): LOCALIZATION_MAP_SPECS (a test may pass a changed copy). Defaults to the config.
        cache_dir (str or Path or None): Folder for the cache. Defaults to specs["cache_dir"] under the project root.
        pedestal_q (float or None): The pedestal command for the planner footprint. Defaults to TASK_CFG's.

    Returns:
        tuple: (maps, source, seconds): maps as build_localization_map returns them, source "cache" or "built", and the seconds it took.
    """
    specs = dict(robot_specs.LOCALIZATION_MAP_SPECS if specs is None else specs)
    path = (ROOT / specs["cache_dir"] if cache_dir is None else Path(cache_dir)) / "localization_map.npz"
    t0 = time.time()
    if path.exists():
        cached = _load(path)
        # the key depends on the scene, so it is recomputed from the scene before the cache is trusted
        fresh_key = build_key_only(model, data, specs, pedestal_q)
        if cached["meta"].get("key") == fresh_key:
            return cached, "cache", round(time.time() - t0, 3)
    maps = build_localization_map(model, data, specs, pedestal_q)
    _save(path, maps)
    return maps, "built", round(time.time() - t0, 3)


def build_key_only(model, data, specs=None, pedestal_q=None):
    """The staleness key the scene and the specs would give now (computed without building the grids)."""
    from src.task_config import TASK_CFG
    specs = dict(robot_specs.LOCALIZATION_MAP_SPECS if specs is None else specs)
    pedestal_q = TASK_CFG["pedestal_q"] if pedestal_q is None else pedestal_q
    z = plane_height(model, data)
    inner, outer = room_bounds(model, data)
    xs, ys, _ = _grid_axes(outer, specs["margin_m"], float(specs["cell_m"]))
    from src.nav_footprint import robot_footprint
    footprint = np.asarray(robot_footprint(model, data, pedestal_q)["polygon"], float)
    return _staleness_key(model, data, specs, fixed_geoms(model), z, inner, outer, footprint, pedestal_q)


def cell_of(maps, xy):
    """The (ix, iy) of a world point and whether it lies on the grid."""
    ix = int(math.floor((xy[0] - maps["meta"]["x0"]) / maps["meta"]["cell_m"]))
    iy = int(math.floor((xy[1] - maps["meta"]["y0"]) / maps["meta"]["cell_m"]))
    ny, nx = maps["meta"]["shape"]
    return ix, iy, (0 <= ix < nx and 0 <= iy < ny)


def predict_ranges(maps, origin, directions, max_range, step=0.002):
    """The range each beam would read in this map: the distance to the first occupied cell along it, stepping `step` metres at a time.

    Args:
        maps (dict): The localization maps.
        origin (numpy.ndarray): (2,) the lidar's world x, y, metres.
        directions (numpy.ndarray): (N, 2) unit horizontal directions.
        max_range (float): Beams are followed this far, metres.
        step (float): The stepping distance, metres. Defaults to 2 mm.

    Returns:
        numpy.ndarray: (N,) the predicted ranges, NaN where no occupied cell is reached within max_range.
    """
    steps = np.arange(step, max_range + step / 2, step)
    pts = origin[None, None, :] + directions[:, None, :] * steps[None, :, None]  # (N, S, 2)
    m = maps["meta"]
    ix = np.floor((pts[..., 0] - m["x0"]) / m["cell_m"]).astype(int)
    iy = np.floor((pts[..., 1] - m["y0"]) / m["cell_m"]).astype(int)
    ny, nx = m["shape"]
    on = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    occ = np.zeros(ix.shape, dtype=bool)
    occ[on] = maps["state"][iy[on], ix[on]] == OCCUPIED
    first = np.argmax(occ, axis=1)
    hit = occ[np.arange(occ.shape[0]), first]
    return np.where(hit, steps[first], np.nan)
