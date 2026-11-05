"""World-model collision check for the grasp poses: the robot's hand, fingers and forearm against one oriented box per object.

The objects come only from the world model (boxes_from_world: a position, a shape and a height per entry). The robot's own
geometry comes from its own model: each collision geom of the group's bodies, as the convex hull of its vertices, sampled on the
hull surface (robot_group_points). Nothing here reads a simulator geom of an object. Distances are signed: negative means
overlap, measured as the depth to the nearest face of the box.

    boxes = boxes_from_world(world)
    groups = robot_group_points(model, data, "left")       # for the pose now in data
    res = check_boxes(groups, boxes, margin)               # per group, per box, and the clearance capped at the margin

Exemptions: the target beaker is not checked against the fingers (finger contact is the grasp). A box that is supported by another
(the beaker on the hotplate) is an object pair, not a robot pair, so it is never computed here.
"""

import math

import mujoco
import numpy as np
from scipy.spatial import ConvexHull

BODY_GROUPS = {"hand": ("hand",), "fingers": ("left_finger", "right_finger"), "forearm": ("link4", "link5", "link6", "link7")}
SURFACE_SUBDIV = 2          # points per hull edge is this + 1: the sampling density on each hull triangle


def boxes_from_world(world):
    """One oriented box per object or fixture of the world model: {"id", "center" (3,), "yaw" (rad), "half" (3,)}.

    An object's position is its bottom centre and a fixture's its top centre (world model convention); its height is the entry's own
    "height" property. A cylinder shape gives a square box of its radius. Surface roots (the bench) have no box here.
    """
    boxes = []
    for e in world.entries():
        if e.properties.get("surface"):
            continue
        sh = e.properties.get("shape")
        if not sh:
            continue
        if sh["kind"] == "cylinder":
            half_xy = (float(sh["radius"]), float(sh["radius"]))
        elif sh["kind"] == "box":
            half_xy = (float(sh["half"][0]), float(sh["half"][1]))
        else:
            continue
        h = float(e.properties.get("height", 0.0))
        p = np.asarray(e.position, float)
        z0, z1 = (p[2], p[2] + h) if e.kind == "object" else (p[2] - h, p[2])
        boxes.append({"id": e.id, "center": np.array([p[0], p[1], 0.5 * (z0 + z1)]), "yaw": 0.0, "half": np.array([half_xy[0], half_xy[1], 0.5 * (z1 - z0)])})
    return boxes


def box_signed_distance(P, box):
    """Signed distance of each point (N, 3) to an oriented box: the outside distance, or minus the depth inside."""
    cs, sn = math.cos(box["yaw"]), math.sin(box["yaw"])
    R = np.array([[cs, -sn, 0.0], [sn, cs, 0.0], [0.0, 0.0, 1.0]])
    local = (P - box["center"]) @ R
    d = np.abs(local) - box["half"]
    outside = np.linalg.norm(np.maximum(d, 0.0), axis=1)
    inside = d.max(axis=1)
    return np.where(inside <= 0.0, inside, outside)


def _local_vertices(model, g):
    """Vertices of one geom in its own frame, or None for an unsupported type."""
    t, size = model.geom_type[g], model.geom_size[g]
    if t == mujoco.mjtGeom.mjGEOM_BOX:
        return np.array([[sx * size[0], sy * size[1], sz * size[2]] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
    if t == mujoco.mjtGeom.mjGEOM_CYLINDER:
        ang = np.linspace(0.0, 2 * math.pi, 16, endpoint=False)
        ring = np.stack([size[0] * np.cos(ang), size[0] * np.sin(ang)], axis=1)
        return np.vstack([np.column_stack([ring, np.full(16, s * size[1])]) for s in (-1.0, 1.0)])
    if t == mujoco.mjtGeom.mjGEOM_SPHERE:
        return np.array([[sx * size[0], sy * size[0], sz * size[0]] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
    if t == mujoco.mjtGeom.mjGEOM_MESH:
        did = model.geom_dataid[g]
        va, vn = model.mesh_vertadr[did], model.mesh_vertnum[did]
        return np.array(model.mesh_vert[va:va + vn], float)
    return None


def _hull_surface(W, subdiv):
    """Points on the convex hull surface of W: the hull's vertices and a barycentric grid on every hull triangle."""
    if len(W) < 4:
        return W
    try:
        hull = ConvexHull(W)
    except Exception:
        return W
    uv = np.array([(i / subdiv, j / subdiv) for i in range(subdiv + 1) for j in range(subdiv + 1 - i)], float)
    B = np.column_stack([1.0 - uv[:, 0] - uv[:, 1], uv])             # barycentric weights of the grid points on a triangle
    S = W[hull.simplices]                                            # (T, 3, 3)
    pts = np.einsum("kj,tjd->tkd", B, S).reshape(-1, 3)
    return np.vstack([W[hull.vertices], pts])


def robot_group_points(model, data, side, subdiv=SURFACE_SUBDIV):
    """Surface points of the robot's hand, fingers and forearm for the pose in `data` (world frame), from the robot's own collision geoms."""
    out = {}
    for group, names in BODY_GROUPS.items():
        bodies = {model.body(f"openarm_{side}_{n}").id for n in names}
        chunks = []
        for g in range(model.ngeom):
            if model.geom_bodyid[g] not in bodies or not (model.geom_contype[g] or model.geom_conaffinity[g]):
                continue
            loc = _local_vertices(model, g)
            if loc is None or len(loc) == 0:
                continue
            W = (data.geom_xmat[g].reshape(3, 3) @ loc.T).T + data.geom_xpos[g]
            chunks.append(_hull_surface(W, subdiv))
        out[group] = np.vstack(chunks) if chunks else np.zeros((0, 3))
    return out


def check_boxes(groups, boxes, margin, target_ids=("beaker",)):
    """Per group: the signed distance to every box (exemptions applied), the smallest over the boxes, and the clearance (that minimum capped at the margin).

    Returns {group: {"per_box": {box id: distance}, "min": distance or None, "clearance": min(margin, min) or margin when no box}}.
    """
    res = {}
    for group, P in groups.items():
        per = {}
        for b in boxes:
            if group == "fingers" and b["id"] in target_ids:
                continue                                              # finger contact with the target beaker is the grasp, not a collision
            if len(P) == 0:
                continue
            per[b["id"]] = float(box_signed_distance(P, b).min())
        mn = min(per.values()) if per else None
        res[group] = {"per_box": per, "min": mn, "clearance": margin if mn is None else min(margin, mn)}
    return res
