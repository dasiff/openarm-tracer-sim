"""Usable edges of the furniture the objects stand on (David, 2026-11-10; decisions-log M300).

A usable edge is an edge, or a part of an edge, of a floor-standing piece of furniture with a rectangular top (bench, table, cabinet, shelf) that the robot can stand in front of: the floor strip
out from the edge, as deep as the robot base is long and as wide as the edge part, holds no obstacle of the world model (walls, other furniture, the legs of other furniture). A cabinet against a wall
therefore comes out with one usable edge, the door side, without any tagging.

    tops = furniture_tops(obstacles)                       # floor-standing rectangular tops, from the scene's obstacles (src/nav_footprint.obstacles_from_model)
    segs = usable_segments(tops, obstacles, base_length_m)  # [{"furniture", "edge", "side", "p0", "p1", "n", "length", "top_z"}, ...]
    attach(world, segs)                                    # writes each segment into the world model: the surface entry's properties["usable_edges"]

Contents:
    base_length(model, data): the robot base's length along its heading, from the chassis geoms (metres).
    furniture_tops(obstacles): the rectangular tops that stand on the floor.
    usable_segments(tops, obstacles, base_length_m): the usable edge segments of each top.
    attach(world, segments): stores the segments in the world model.
"""

import mujoco
import numpy as np
from scipy.spatial import ConvexHull

from src.home_clearance_map import surface_samples


MIN_EDGE_M = 0.02        # m: an open part of an edge shorter than this is dropped: a sliver is no place to stand [row M300]
TOP_MAX_M = 1.2          # m: a top above this is a wall or a tall unit, not a bench-type surface (the room's wall is 3 m high) [row M300]
TOP_MIN_M = 0.4          # m: a top below this is a low shelf or a seat, not a work surface [row M300]
FLOOR_TOL_M = 0.05       # m: a leg or a base whose bottom is this close to the floor stands on the floor [row M300]
LEG_MIN_SIDE_M = 0.2     # m: a box thinner than this (a leg, a post) is not a top or a block of furniture [row M300]
BASE_PREFIX = "contact_"  # the robot's wheel and caster contact geoms (names): they set the chassis length with the base geom


def base_length(model, data, spacing=0.01):
    """The robot base's length along its heading: the x extent of the chassis (the base geom and the wheel and caster contacts) at the drive pose.

    Args:
        model (mujoco.MjModel): The compiled model.
        data (mujoco.MjData): Its data, with the robot at rest (a kinematics pass done).
        spacing (float): The surface sample spacing, metres.

    Returns:
        float: Metres.
    """
    root = model.body("base_link").id
    pts = []
    for g in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        if name != "base_link_geom" and not name.startswith(BASE_PREFIX):
            continue
        if int(model.geom_bodyid[g]) != root:
            continue
        pts.append(surface_samples(model, data, g, spacing))
    P = np.concatenate(pts)
    return float(P[:, 0].max() - P[:, 0].min())


def _polygon_area(poly):
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def _inside(points, poly, tol=0.0):
    """True for each point inside (or within tol of) the convex counter-clockwise polygon."""
    ok = np.ones(len(points), bool)
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        e = b - a
        cross = e[0] * (points[:, 1] - a[1]) - e[1] * (points[:, 0] - a[0])
        ok &= cross >= -tol * np.hypot(*e)
    return ok


def _clip(poly, a, b, c):
    """Sutherland-Hodgman: the part of the polygon where a*x + b*y + c >= 0 (the polygon is a list of (x, y))."""
    out = []
    n = len(poly)
    for i in range(n):
        p, q = poly[i], poly[(i + 1) % n]
        fp, fq = a * p[0] + b * p[1] + c, a * q[0] + b * q[1] + c
        if fp >= 0:
            out.append(p)
        if (fp >= 0) != (fq >= 0):
            t = fp / (fp - fq)
            out.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])))
    return out


def _obstacle_outline(o):
    return np.asarray(o["outline"], float)


def _centre(outline):
    return outline.mean(axis=0)


def furniture_tops(obstacles):
    """The floor-standing rectangular tops in the scene's obstacles.

    A top is a rectangular obstacle (a convex outline with 4 corners) whose top face is between TOP_MIN_M and TOP_MAX_M above the floor and whose underside is above the floor
    (a bench top: its legs stand under it) or on the floor (a cabinet block: its own bottom is the floor), and that has at least one leg or base on the floor under it.
    Where two tops have the same outline (a counter top and its base, a shelf and the cart top) only the highest is kept.

    Args:
        obstacles (list of dict): {"id", "outline" ((K, 2) world xy, counter-clockwise hull), "height_range" ((z_low, z_high))}.

    Returns:
        list of dict: {"id" (the obstacle's id), "outline" ((4, 2) counter-clockwise), "top_z" (metres, the top face), "floor" (the id(s) of the supports)}.
    """
    cands = []
    for o in obstacles:
        P = _obstacle_outline(o)
        if len(P) != 4:
            continue
        z_lo, z_hi = o["height_range"]
        if not (TOP_MIN_M <= z_hi <= TOP_MAX_M):
            continue
        ext = P.max(axis=0) - P.min(axis=0)
        if min(ext) < LEG_MIN_SIDE_M:
            continue
        cands.append(o)
    tops = []
    for o in cands:
        P = _obstacle_outline(o)
        z_lo, z_hi = o["height_range"]
        supports = [s["id"] for s in obstacles if s["height_range"][0] <= FLOOR_TOL_M and _inside(_centre(_obstacle_outline(s))[None, :], P, tol=0.02)[0]]
        if not supports:
            continue
        if z_lo > FLOOR_TOL_M and z_lo < TOP_MIN_M:
            continue
        tops.append({"id": o["id"], "outline": P, "top_z": float(z_hi), "floor": supports})
    kept = []
    for t in sorted(tops, key=lambda t: -t["top_z"]):                 # the highest first: a counter top before its base
        if not any(_same_outline(t["outline"], k["outline"], 0.01) for k in kept):
            kept.append(t)
    return kept


def _same_outline(P, Q, tol):
    """True when two outlines have the same corners (in any order) to within tol metres."""
    if len(P) != len(Q):
        return False
    return bool(np.allclose(np.sort(P, axis=0), np.sort(np.asarray(Q, float), axis=0), atol=tol))


def _edge_frames(P):
    """The four edges of a counter-clockwise rectangle: (a, b, u, n, length), n the outward unit normal."""
    out = []
    for i in range(len(P)):
        a, b = P[i], P[(i + 1) % len(P)]
        e = b - a
        L = float(np.hypot(*e))
        u = e / L
        n = np.array([u[1], -u[0]])                 # counter-clockwise outline: the right-hand normal points out
        out.append((a, b, u, n, L))
    return out


def _side_name(n):
    if abs(n[0]) >= abs(n[1]):
        return "+x" if n[0] > 0 else "-x"
    return "+y" if n[1] > 0 else "-y"


def usable_segments(tops, obstacles, base_length_m, min_edge_m=MIN_EDGE_M):
    """The usable edge segments of each top: the open parts of its four edges, given the floor strip (depth = base length) out from each edge.

    Args:
        tops (list of dict): furniture_tops(obstacles).
        obstacles (list of dict): All the scene's obstacles (the furniture's own legs and the things on it are left out by their position).
        base_length_m (float): The robot base's length: the depth of the strip, metres.
        min_edge_m (float): The shortest open part kept, metres.

    Returns:
        list of dict: {"furniture" (the top's id), "edge" (0-3, the edge's index in the outline), "side" ("+x", "-x", "+y" or "-y": the outward direction),
            "p0", "p1" ((2,) world xy of the usable part's ends, p0 to p1 along the edge), "n" ((2,) the outward unit normal), "length" (metres), "top_z" (metres)}.
    """
    segs = []
    for top in tops:
        P = top["outline"]
        own = {o["id"] for o in obstacles if _inside(_centre(_obstacle_outline(o))[None, :], P, tol=0.02)[0]}
        others = [o for o in obstacles if o["id"] not in own]
        for k, (a, b, u, n, L) in enumerate(_edge_frames(P)):
            blocked = []
            for o in others:
                Q = _obstacle_outline(o)
                d = Q - a
                tq = np.column_stack([d @ u, d @ n])                       # the obstacle in the edge's frame: t along the edge, s out from it
                poly = [tuple(p) for p in tq]
                clipped = poly
                for (ca, cb, cc) in ((1.0, 0.0, 0.0), (-1.0, 0.0, L), (0.0, 1.0, -1e-3), (0.0, -1.0, base_length_m)):   # t >= 0, t <= L, s >= 1 mm, s <= base length
                    clipped = _clip(clipped, ca, cb, cc)
                    if not clipped:
                        break
                if clipped:
                    ts = [p[0] for p in clipped]
                    blocked.append((min(ts), max(ts)))
            free, t_at = [], 0.0
            for lo, hi in sorted(blocked):
                if lo > t_at:
                    free.append((t_at, min(lo, L)))
                t_at = max(t_at, hi)
            if t_at < L:
                free.append((t_at, L))
            for t0, t1 in free:
                if t1 - t0 < min_edge_m:
                    continue
                segs.append({"furniture": top["id"], "edge": k, "side": _side_name(n), "p0": (a + t0 * u).tolist(), "p1": (a + t1 * u).tolist(), "n": n.tolist(),
                             "length": float(t1 - t0), "top_z": top["top_z"]})
    return segs


def attach(world, tops, segments, tol=0.01):
    """Stores the usable edge segments in the world model: each surface entry (the bench) whose outline and top height match a top gets properties["usable_edges"].

    Args:
        world (WorldModel): The knowledge base.
        tops (list of dict): furniture_tops(obstacles).
        segments (list of dict): usable_segments(...).
        tol (float): The match tolerance on the outline corners and the top height, metres.

    Returns:
        dict: {entry id: the segments stored on it}.
    """
    out = {}
    for e in world.entries():
        if not e.properties.get("surface") or e.outline is None:
            continue
        P = np.asarray(e.outline, float)
        match = [t for t in tops if _same_outline(P, t["outline"], tol) and abs(t["top_z"] - float(e.properties["top_z"])) < tol]
        mine = [s for s in segments if match and s["furniture"] == match[0]["id"]]
        e.properties["usable_edges"] = mine
        out[e.id] = mine
    return out
