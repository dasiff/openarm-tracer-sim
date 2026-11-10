"""Builds the safety filter's robot shapes from the robot model (SF-5) and writes them to a file.

Loads the robot model only (models/robot/combined_robot.xml plus the battery and the eagle camera's
body; no scene) and writes data/safety_filter/robot_shapes.json, stamped with the robot model's
fingerprint. The filter places every shape from the joint state with its own forward kinematics,
so each shape is stored as body-local points (its body and local coordinates), never as world poses.

Shapes (both arms separately, never mirrored):
  upper arm   capsule from the joint 2 anchor (body link2) to the joint 4 anchor (body link4); radius
              = the largest distance of the link 1-3 collision geometry from that segment over the sampled poses
  forearm     capsule from the joint 4 anchor (body link4) to the joint 7 anchor (body link7); radius over
              the link 4-6 collision geometry
  shoulder    capsule fitted to the link 0 collision geometry in the link 0 frame (fixed to the torso)
  link 7      one capsule fitted to the link 7 collision geometry, in the link 7 frame
  palm        two capsules fitted to the hand collision geometry, in the link 7 frame (the hand body is rigid to link 7)
  fingers     one capsule per finger, fitted to that finger body's collision geometry, in its own frame
  base        boxes: the tightest box in each geom's own frame for the base mesh and the chassis links;
  pedestal    boxes: the pedestal base and the pedestal top (box geoms, exact);
  torso       one vertical capsule around the torso mesh (the torso body frame, z up);
  eagle cam   box (exact, the eagle camera's body); battery box (exact).
  Head and face: visual only, not included.

Upper arm and forearm radii: the links move relative to their segment, so the radius is the maximum over
the joint-limit corners of the joints involved (upper arm: joints 1-3; forearm: joints 4-6) plus random
poses within the joint limits. The other joints stay at zero (they do not change these geometries relative
to the segment: the upper arm's and forearm's ends are on the joint axes of their own bodies). The maximum is
reported at 1000 and at 2000 samples; if it still grows, the file says so.

Run:  python tools/build_filter_shapes.py   (writes data/safety_filter/robot_shapes.json atomically)
"""

import argparse
import json
import os
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import minimize

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.safety_filter.config import SHAPE_BUILD  # noqa: E402
from src.safety_filter.robot_model import ROBOT_XML, load_robot_only_model, robot_fingerprint  # noqa: E402

DEFAULT_OUT = ROOT / "data" / "safety_filter" / "robot_shapes.json"
SIDES = ("left", "right")


# ---------- geometry helpers ----------

def quat_to_mat(q):
    out = np.zeros(9)
    mujoco.mju_quat2Mat(out, np.asarray(q, dtype=float))
    return out.reshape(3, 3)


def geom_verts_geomframe(m, g):
    """Vertices of a mesh or box geom, in the geom's own frame (MuJoCo collides with their convex hull)."""
    t = m.geom_type[g]
    if t == mujoco.mjtGeom.mjGEOM_MESH:
        did = m.geom_dataid[g]
        adr, n = m.mesh_vertadr[did], m.mesh_vertnum[did]
        return m.mesh_vert[adr:adr + n].astype(float)
    if t == mujoco.mjtGeom.mjGEOM_BOX:
        s = m.geom_size[g]
        return np.array([[a * s[0], b * s[1], c * s[2]] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)], float)
    raise ValueError(f"geom {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)}: type {t} has no vertices here")


def world_verts(m, d, g):
    return geom_verts_geomframe(m, g) @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]


def verts_in_body(m, d, g, body):
    """The geom's vertices in the frame of a body (the body's pose at the current data)."""
    pw = world_verts(m, d, g)
    Rb = d.xmat[body].reshape(3, 3)
    return (pw - d.xpos[body]) @ Rb


def seg_dist(P, A, B):
    AB = B - A
    L2 = max(float(AB @ AB), 1e-18)
    t = np.clip((P - A) @ AB / L2, 0.0, 1.0)
    return np.linalg.norm(P - (A + t[:, None] * AB), axis=1)


def fit_capsule(P, steps):
    """Smallest-radius capsule (free ends) enclosing the points P: Nelder-Mead on the two end points.
    The radius is always the largest distance of a point from the segment, so the result encloses P."""
    c = P.mean(axis=0)
    _, _, Vt = np.linalg.svd(P - c, full_matrices=False)
    u = Vt[0]
    t = (P - c) @ u
    x0 = np.r_[c + t.min() * u, c + t.max() * u]
    f = lambda x: float(seg_dist(P, x[:3], x[3:]).max())
    res = minimize(f, x0, method="Nelder-Mead",
                   options={"xatol": 1e-10, "fatol": 1e-12, "maxfev": steps, "maxiter": steps})
    x = res.x if f(res.x) <= f(x0) else x0
    return x[:3], x[3:], f(x)


def fit_two_capsules(P, steps):
    """Two capsules, each enclosing its own share of the points (split along the principal axis).
    Every point lies inside its own capsule, so the pair encloses P. The split with the least total volume wins."""
    c = P.mean(axis=0)
    _, _, Vt = np.linalg.svd(P - c, full_matrices=False)
    order = np.argsort((P - c) @ Vt[0])
    best = None
    for frac in np.linspace(0.2, 0.8, 25):
        k = int(round(frac * len(P)))
        parts = [P[order[:k]], P[order[k:]]]
        caps = [fit_capsule(p, steps) for p in parts]
        vol = sum(np.pi * r ** 2 * (np.linalg.norm(B - A) + 4.0 / 3.0 * r) for A, B, r in caps)
        if best is None or vol < best[0]:
            best = (vol, caps)
    return best[1]


def fit_vertical_capsule(P, steps):
    """One capsule with its axis along z: the ends at the lowest and highest vertex, the radius from the
    smallest circle around the xy points (found by Nelder-Mead on the centre)."""
    xy = P[:, :2]
    f = lambda c: float(np.linalg.norm(xy - c, axis=1).max())
    c0 = xy.mean(axis=0)
    res = minimize(f, c0, method="Nelder-Mead", options={"xatol": 1e-10, "fatol": 1e-12, "maxfev": steps})
    c = res.x if f(res.x) <= f(c0) else c0
    r = f(c)
    z0, z1 = float(P[:, 2].min()), float(P[:, 2].max())
    return np.r_[c, z0], np.r_[c, z1], r


def geom_by_name(m, name):
    g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
    if g < 0:
        raise SystemExit(f"robot model has no geom named {name}")
    return g


def body_by_name(m, name):
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
    if b < 0:
        raise SystemExit(f"robot model has no body named {name}")
    return b


def joint_by_name(m, name):
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
    if j < 0:
        raise SystemExit(f"robot model has no joint named {name}")
    return j


def capsule_entry(sid, arm, part, body, A, B, r, geoms, note=""):
    return {"id": sid, "arm": arm, "part": part, "kind": "capsule", "frame": body,
            "ends_local": [[float(v) for v in A], [float(v) for v in B]],
            "ends_body": [body, body], "radius": float(r), "geoms": geoms, "note": note}


def capsule_entry2(sid, arm, part, bodies, A, B, r, geoms, note=""):
    """A capsule whose two ends are in different bodies (the arm segments)."""
    return {"id": sid, "arm": arm, "part": part, "kind": "capsule", "frame": "per end",
            "ends_local": [[float(v) for v in A], [float(v) for v in B]],
            "ends_body": list(bodies), "radius": float(r), "geoms": geoms, "note": note}


def box_entry(sid, arm, part, body, center, quat, half, geoms, note=""):
    return {"id": sid, "arm": arm, "part": part, "kind": "box", "frame": body,
            "center_local": [float(v) for v in center], "quat_local": [float(v) for v in quat],
            "half": [float(v) for v in half], "geoms": geoms, "note": note}


# ---------- arm segments: sampled over joint limits ----------

def convex_hull_vertices(m, g):
    """Vertices of a mesh geom's convex hull in the geom frame (MuJoCo collides with the hull; the hull encloses all)."""
    from scipy.spatial import ConvexHull
    V = geom_verts_geomframe(m, g)
    return V[ConvexHull(V).vertices]


def fit_three_capsules(P, steps):
    """Three capsules, each enclosing its own share of the points (consecutive groups along the principal axis).
    Two cut fractions on a 0.1 grid; the split with the least total volume wins."""
    import itertools
    c = P.mean(axis=0)
    _, _, Vt = np.linalg.svd(P - c, full_matrices=False)
    order = np.argsort((P - c) @ Vt[0])
    n = len(P)
    best = None
    for f1, f2 in itertools.combinations(np.round(np.arange(0.1, 1.0, 0.1), 3), 2):
        k1, k2 = int(round(f1 * n)), int(round(f2 * n))
        parts = [P[order[:k1]], P[order[k1:k2]], P[order[k2:]]]
        if any(len(p) < 4 for p in parts):
            continue
        caps = [fit_capsule(p, steps) for p in parts]
        vol = sum(np.pi * r ** 2 * (np.linalg.norm(B - A) + 4.0 / 3.0 * r) for A, B, r in caps)
        if best is None or vol < best[0]:
            best = (vol, caps)
    return best[1]


def fit_link_pieces(P, count, steps):
    """The capsules (A, B, r) in the body frame that enclose the points P with `count` capsules (minimum total volume)."""
    if count == 1:
        return [fit_capsule(P, steps)]
    if count == 2:
        return fit_two_capsules(P, steps)
    if count == 3:
        return fit_three_capsules(P, steps)
    raise ValueError("count must be 1, 2 or 3")


def link_body_points(m, g):
    """Hull vertices of a link's collision mesh in the link's own body frame (rigid to the body: exact for every pose)."""
    H = convex_hull_vertices(m, g)
    return H @ quat_to_mat(m.geom_quat[g]).T + m.geom_pos[g]


LINK_SEGMENT = {1: "upper_arm", 2: "upper_arm", 3: "upper_arm", 4: "forearm", 5: "forearm", 6: "forearm"}
SEGMENT_OF_PART = {"shoulder_mount": "shoulder", "link7": "hand", "palm": "hand", "finger": "hand"}


def build_arm(m, d, side, cfg, rng, shapes, report):
    """The arm's shapes: links 1 to 6 per rigid body (SF-18, the count per link from SHAPE_BUILD link_capsules),
    the shoulder mount (link 0), link 7, the palm and the fingers (unchanged from SF-3 and SF-15)."""
    S = SHAPE_BUILD
    steps = S["fit_steps"]
    report.setdefault(side, {})
    d.qpos[:] = m.qpos0
    mujoco.mj_kinematics(m, d)
    b = {k: body_by_name(m, f"openarm_{side}_link{k}") for k in range(0, 8)}
    counts = S["link_capsules"][side]
    cache = cfg if isinstance(cfg, dict) else {}
    for k in range(1, 7):
        n = int(counts[k])
        g = geom_by_name(m, f"openarm_{side}_link{k}_collision")
        key = (side, k, n)
        if key not in cache:
            cache[key] = fit_link_pieces(link_body_points(m, g), n, steps)
        pieces = cache[key]
        report[side][f"link{k}"] = {"count": n, "radii": [float(r) for _, _, r in pieces],
                                    "volume_m3": float(sum(np.pi * r ** 2 * (np.linalg.norm(B - A) + 4 / 3 * r) for A, B, r in pieces))}
        for j, (A, B, r) in enumerate(pieces, start=1):
            sid = f"{side}_link{k}" if n == 1 else f"{side}_link{k}_{j}"
            e = capsule_entry(sid, side, f"link{k}", f"openarm_{side}_link{k}", A, B, r,
                              [f"openarm_{side}_link{k}_collision"],
                              note=f"capsule {j} of {n} on link {k} (own frame, SF-18)")
            e["segment"] = LINK_SEGMENT[k]
            shapes.append(e)

    # Shoulder mount (link 0, fixed to the torso): free capsule in the link 0 frame.
    g0 = geom_by_name(m, f"openarm_{side}_link0_collision")
    A, B, r = fit_capsule(verts_in_body(m, d, g0, b[0]), steps)
    shapes.append(capsule_entry(f"{side}_shoulder_mount", side, "shoulder_mount", f"openarm_{side}_link0",
                                A, B, r, [f"openarm_{side}_link0_collision"],
                                note="fitted to the link 0 collision mesh; fixed to the torso"))

    # Link 7: N capsules in the link 7 frame (SF-15). The link 7 mesh is rigid to the frame: the fit is exact for every pose.
    g7 = geom_by_name(m, f"openarm_{side}_link7_collision")
    P7 = verts_in_body(m, d, g7, b[7])
    n7 = S["link7_capsules"]
    fits7 = [fit_capsule(P7, steps)] if n7 == 1 else fit_two_capsules(P7, steps)
    for k, (A, B, r) in enumerate(fits7, start=1):
        sid = f"{side}_link7" if n7 == 1 else f"{side}_link7_{k}"
        shapes.append(capsule_entry(sid, side, "link7", f"openarm_{side}_link7", A, B, r,
                                    [f"openarm_{side}_link7_collision"],
                                    note=f"capsule {k} of {n7} in the link 7 frame (SF-15)"))
    report[side]["link7"] = {"count": n7, "radii": [float(r) for _, _, r in fits7]}

    # Palm: two capsules in the link 7 frame around the hand body (fingers excluded).
    gh = geom_by_name(m, f"openarm_{side}_hand_collision")
    P7 = verts_in_body(m, d, gh, b[7])
    for i, (A, B, r) in enumerate(fit_two_capsules(P7, steps), start=1):
        shapes.append(capsule_entry(f"{side}_palm_{i}", side, "palm", f"openarm_{side}_link7", A, B, r,
                                    [f"openarm_{side}_hand_collision"],
                                    note="two capsules around the hand body, in the link 7 frame (SF-3)"))

    # Fingers: one capsule each, in the finger body's own frame.
    for finger in ("right", "left"):
        body = f"openarm_{side}_{finger}_finger"
        gf = geom_by_name(m, f"{body}_collision")
        A, B, r = fit_capsule(verts_in_body(m, d, gf, body_by_name(m, body)), steps)
        shapes.append(capsule_entry(f"{side}_finger_{finger}", side, "finger", body, A, B, r,
                                    [f"{body}_collision"], note="one capsule, the finger body's frame (SF-3)"))
    return cache


def build_common(m, d, shapes):
    d.qpos[:] = m.qpos0
    mujoco.mj_kinematics(m, d)
    # Base (chassis mesh and chassis links): tightest box in each geom's own frame.
    for gname in ("base_link_geom", "right_link_geom", "left_Link_geom", "right_front_link_geom",
                  "right_rear_link_geom", "left_front_link_geom", "left_rear_link_geom"):
        g = geom_by_name(m, gname)
        V = geom_verts_geomframe(m, g)
        lo, hi = V.min(axis=0), V.max(axis=0)
        c_geom = (lo + hi) / 2
        Rg = quat_to_mat(m.geom_quat[g])
        center = m.geom_pos[g] + Rg @ c_geom
        body = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
        shapes.append(box_entry(f"base_{gname}", "common", "base", body, center, m.geom_quat[g],
                                (hi - lo) / 2, [gname],
                                note="tightest box in the geom's own frame (mesh)"))
    # Pedestal (box geoms, exact).
    for gname, part in (("pedestal_base_collision", "pedestal_base"), ("pedestal_top_collision", "pedestal_top")):
        g = geom_by_name(m, gname)
        body = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
        shapes.append(box_entry(part, "common", part, body, m.geom_pos[g], m.geom_quat[g], m.geom_size[g], [gname],
                                note="box geom, exact"))
    # Torso: one vertical capsule around the torso mesh (body openarm_body_link0).
    gt = geom_by_name(m, "openarm_body_link0_collision")
    bt = body_by_name(m, "openarm_body_link0")
    A, B, r = fit_vertical_capsule(geom_verts_geomframe(m, gt) @ quat_to_mat(m.geom_quat[gt]).T + m.geom_pos[gt],
                                   SHAPE_BUILD["fit_steps"])
    shapes.append(capsule_entry("torso", "common", "torso", "openarm_body_link0", A, B, r,
                                ["openarm_body_link0_collision"], note="vertical capsule around the torso mesh"))
    # Eagle camera body and battery: exact boxes of their own bodies.
    for body_name in ("eagle_cam_body", "battery"):
        gids = [g for g in range(m.ngeom) if m.geom_bodyid[g] == body_by_name(m, body_name)
                and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX]
        if len(gids) != 1:
            raise SystemExit(f"expected one box on {body_name}, found {len(gids)}")
        g = gids[0]
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)
        shapes.append(box_entry(body_name, "common", body_name, body_name, m.geom_pos[g], m.geom_quat[g],
                                m.geom_size[g], [name], note="box geom, exact"))


SWEEP_FILE = ROOT / "data" / "safety_filter" / "never_close_sweep.json"


def load_never_close(fp, shapes):
    """The never-close list (SF-23) for these shapes, from the sweep file made by tools/never_close_sweep.py.
    A shape pair is listed when the sweep listed the two shapes' real-geom sets; an entry carries the sweep numbers.
    A sweep made for another robot model is not used (empty list, with a warning)."""
    empty = {"rule": "SF-23", "fingerprint": fp, "pairs": [], "swept": 0, "not_swept": 0}
    if not SWEEP_FILE.exists():
        print("warning: no never_close_sweep.json; the never-close list is empty")
        return empty
    sw = json.loads(SWEEP_FILE.read_text(encoding="utf-8"))
    if sw["fingerprint"] != fp:
        print("warning: never_close_sweep.json is for another robot model; the never-close list is empty")
        return empty
    by_key = {}
    for e in sw["entries"]:
        by_key[tuple(sorted([tuple(sorted(e["a_geoms"])), tuple(sorted(e["b_geoms"]))]))] = e
    pairs, swept, not_swept = [], 0, 0
    ids = {s["id"]: s for s in shapes}
    for e in sw["entries"]:
        for a, b in e["shape_pairs"]:
            if a not in ids or b not in ids:
                continue
            if e["status"] == "not swept":
                not_swept += 1
                continue
            swept += 1
            if e["listed"]:
                pairs.append({"a": a, "b": b, "variables": e["variables"], "steps": e["steps"],
                              "step_units": e["step_units"], "bound_m": e["bound_m"], "min_gap_m": e["min_gap_m"],
                              "min_pose": e["min_pose"], "margin_m": e["margin_m"]})
    return {"rule": "SF-23", "fingerprint": fp, "pairs": pairs, "swept": swept, "not_swept": not_swept,
            "note": "listed only if min real gap - step-size bound >= margin + 1 cm (hand-hand 2 cm, other 4 cm); tools/never_close_sweep.py"}


def build(out_path, counts=None, cache=None):
    """Writes the shapes file. counts: {"left": {1..6: n}, "right": {...}} (default: SHAPE_BUILD link_capsules).
    cache: dict of per-link fits (key (side, link, count)) reused across calls."""
    m = load_robot_only_model()
    d = mujoco.MjData(m)
    fp = robot_fingerprint(m)
    rng = np.random.default_rng(SHAPE_BUILD["seed"])
    if counts is not None:
        SHAPE_BUILD["link_capsules"] = counts
    cache = {} if cache is None else cache
    shapes, report = [], {}
    for side in SIDES:
        cache = build_arm(m, d, side, cache, rng, shapes, report)
    build_common(m, d, shapes)
    for sp in shapes:
        if "segment" not in sp:
            sp["segment"] = SEGMENT_OF_PART.get(sp["part"], "body")
    never_close = load_never_close(fp, shapes)
    doc = {
        "schema": 2,
        "robot_model": str(ROBOT_XML.relative_to(ROOT)),
        "robot_model_includes": ["battery (src/battery.py)", "eagle camera body (src/cameras.py)"],
        "fingerprint": fp,
        "build": {"tool": "tools/build_filter_shapes.py", "config": "src/safety_filter/config.py SHAPE_BUILD"},
        "link_fit": report,
        "shapes": shapes,
        "never_close": never_close,
    }
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp = out_path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    os.replace(tmp, out_path)
    return doc, cache


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    args = ap.parse_args()
    doc, _ = build(args.out)
    print(f"wrote {args.out}: {len(doc['shapes'])} shapes, fingerprint {doc['fingerprint'][:16]}")
    for side, parts in doc["link_fit"].items():
        for part, v in parts.items():
            print(f"  {side} {part}: {v}")


if __name__ == "__main__":
    main()
