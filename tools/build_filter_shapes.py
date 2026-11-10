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
            "ends_local": [[round(float(v), 9) for v in A], [round(float(v), 9) for v in B]],
            "ends_body": [body, body], "radius": round(float(r), 9), "geoms": geoms, "note": note}


def capsule_entry2(sid, arm, part, bodies, A, B, r, geoms, note=""):
    """A capsule whose two ends are in different bodies (the arm segments)."""
    return {"id": sid, "arm": arm, "part": part, "kind": "capsule", "frame": "per end",
            "ends_local": [[round(float(v), 9) for v in A], [round(float(v), 9) for v in B]],
            "ends_body": list(bodies), "radius": round(float(r), 9), "geoms": geoms, "note": note}


def box_entry(sid, arm, part, body, center, quat, half, geoms, note=""):
    return {"id": sid, "arm": arm, "part": part, "kind": "box", "frame": body,
            "center_local": [round(float(v), 9) for v in center], "quat_local": [round(float(v), 9) for v in quat],
            "half": [round(float(v), 9) for v in half], "geoms": geoms, "note": note}


# ---------- arm segments: sampled over joint limits ----------

def pose_samples(m, joint_ids, rng, n_first, n_second):
    """Joint-limit corners of the given joints, then random poses within their limits.
    Returns (first set = corners + n_first random, second set = n_second more random)."""
    lim = np.array([m.jnt_range[j] for j in joint_ids], float)
    corners = np.array([[lim[i, b] for i, b in enumerate(bits)] for bits in
                        np.array(np.meshgrid(*([[0, 1]] * len(joint_ids)))).T.reshape(-1, len(joint_ids))])
    first = rng.uniform(lim[:, 0], lim[:, 1], size=(n_first, len(joint_ids)))
    second = rng.uniform(lim[:, 0], lim[:, 1], size=(n_second, len(joint_ids)))
    return np.vstack([corners, first]), second


def segment_radius(m, d, seg_ends, geoms, joint_ids, samples):
    """Largest distance of the geoms' vertices from the segment, over the sampled poses.
    seg_ends: ((joint id for end A), (joint id for end B)); the ends are the joint anchors (world, from the pose)."""
    ja, jb = seg_ends
    worst = []
    for q in samples:
        d.qpos[:] = m.qpos0
        d.qpos[[m.jnt_qposadr[j] for j in joint_ids]] = q
        mujoco.mj_kinematics(m, d)
        A, B = d.xanchor[ja].copy(), d.xanchor[jb].copy()
        P = np.vstack([world_verts(m, d, g) for g in geoms])
        worst.append(float(seg_dist(P, A, B).max()))
    return float(np.max(worst)) if worst else 0.0


def build_arm(m, d, side, cfg, rng, shapes, report):
    S = SHAPE_BUILD
    steps = S["fit_steps"]
    d.qpos[:] = m.qpos0
    mujoco.mj_kinematics(m, d)

    # Upper arm: joint 2 anchor (body link2, local jnt_pos) to joint 4 anchor (body link4).
    j = {k: joint_by_name(m, f"openarm_{side}_joint{k}") for k in range(1, 8)}
    b = {k: body_by_name(m, f"openarm_{side}_link{k}") for k in range(0, 8)}
    geoms = lambda ks: [f"openarm_{side}_link{k}_collision" for k in ks]

    def anchor_local(k):
        return np.asarray(m.jnt_pos[j[k]], float)

    # ends at the joint anchors at the reference pose (for the stored local coordinates)
    seg_up = (j[2], j[4])
    seg_fore = (j[4], j[7])
    up_geoms = [geom_by_name(m, g) for g in geoms((1, 2, 3))]
    fo_geoms = [geom_by_name(m, g) for g in geoms((4, 5, 6))]

    up_joints = [j[1], j[2], j[3]]
    fo_joints = [j[4], j[5], j[6]]
    up_first, up_second = pose_samples(m, up_joints, rng, S["random_poses_first_set"], S["random_poses_second_set"])
    fo_first, fo_second = pose_samples(m, fo_joints, rng, S["random_poses_first_set"], S["random_poses_second_set"])
    up_r1000 = segment_radius(m, d, seg_up, up_geoms, up_joints, up_first)
    up_r2000 = max(up_r1000, segment_radius(m, d, seg_up, up_geoms, up_joints, up_second))
    fo_r1000 = segment_radius(m, d, seg_fore, fo_geoms, fo_joints, fo_first)
    fo_r2000 = max(fo_r1000, segment_radius(m, d, seg_fore, fo_geoms, fo_joints, fo_second))
    report[side] = {
        "upper_arm": {"radius_at_1000": up_r1000, "radius_at_2000": up_r2000,
                      "still_grows": bool(up_r2000 > up_r1000 + 1e-9), "poses": len(up_first) + len(up_second)},
        "forearm": {"radius_at_1000": fo_r1000, "radius_at_2000": fo_r2000,
                    "still_grows": bool(fo_r2000 > fo_r1000 + 1e-9), "poses": len(fo_first) + len(fo_second)},
    }
    # Reference pose verts for the stored ends: at zero pose, the ends are body-local jnt_pos.
    shapes.append(capsule_entry2(f"{side}_upper_arm", side, "upper_arm", (b[2], b[4]),
                                 anchor_local(2), anchor_local(4), up_r2000, geoms((1, 2, 3)),
                                 note="ends: joint 2 anchor (body link2) and joint 4 anchor (body link4); radius over 2000 sampled poses"))
    shapes[-1]["ends_body"] = [f"openarm_{side}_link2", f"openarm_{side}_link4"]
    shapes[-1]["radius_at_1000"] = round(up_r1000, 9)
    shapes.append(capsule_entry2(f"{side}_forearm", side, "forearm", (b[4], b[7]),
                                 anchor_local(4), anchor_local(7), fo_r2000, geoms((4, 5, 6)),
                                 note="ends: joint 4 anchor (body link4) and joint 7 anchor (body link7); radius over 2000 sampled poses"))
    shapes[-1]["ends_body"] = [f"openarm_{side}_link4", f"openarm_{side}_link7"]
    shapes[-1]["radius_at_1000"] = round(fo_r1000, 9)

    # Shoulder mount (link 0, fixed to the torso): free capsule in the link 0 frame.
    g0 = geom_by_name(m, f"openarm_{side}_link0_collision")
    A, B, r = fit_capsule(verts_in_body(m, d, g0, b[0]), steps)
    shapes.append(capsule_entry(f"{side}_shoulder_mount", side, "shoulder_mount", f"openarm_{side}_link0",
                                A, B, r, [f"openarm_{side}_link0_collision"],
                                note="fitted to the link 0 collision mesh; fixed to the torso"))

    # Link 7: one capsule in the link 7 frame.
    g7 = geom_by_name(m, f"openarm_{side}_link7_collision")
    A, B, r = fit_capsule(verts_in_body(m, d, g7, b[7]), steps)
    shapes.append(capsule_entry(f"{side}_link7", side, "link7", f"openarm_{side}_link7", A, B, r,
                                [f"openarm_{side}_link7_collision"], note="one capsule, link 7 frame (SF-3)"))

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


def build(out_path):
    m = load_robot_only_model()
    d = mujoco.MjData(m)
    fp = robot_fingerprint(m)
    rng = np.random.default_rng(SHAPE_BUILD["seed"])
    shapes, report = [], {}
    for side in SIDES:
        build_arm(m, d, side, None, rng, shapes, report)
    build_common(m, d, shapes)
    doc = {
        "schema": 1,
        "robot_model": str(ROBOT_XML.relative_to(ROOT)),
        "robot_model_includes": ["battery (src/battery.py)", "eagle camera body (src/cameras.py)"],
        "fingerprint": fp,
        "build": {"tool": "tools/build_filter_shapes.py", "config": "src/safety_filter/config.py SHAPE_BUILD"},
        "arm_segment_sampling": report,
        "shapes": shapes,
    }
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp = out_path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    os.replace(tmp, out_path)
    return doc


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    args = ap.parse_args()
    doc = build(args.out)
    print(f"wrote {args.out}: {len(doc['shapes'])} shapes, fingerprint {doc['fingerprint'][:16]}")
    for side, r in doc["arm_segment_sampling"].items():
        for part, v in r.items():
            print(f"  {side} {part}: r@1000 {v['radius_at_1000']:.6f} r@2000 {v['radius_at_2000']:.6f} "
                  f"grows {v['still_grows']} poses {v['poses']}")


if __name__ == "__main__":
    main()
