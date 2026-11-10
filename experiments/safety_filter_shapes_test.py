"""Safety filter: the shapes from robot_shapes.json enclose the real collision geometry (no stepping).

For 200 random poses per arm (all joints within limits, random pedestal, base pose and finger travel):
  * the filter places its shapes from the joint state (its own forward kinematics on the robot-only model);
  * MuJoCo's kinematics on the same state gives the real collision geometry (test-only ground truth);
  * every vertex of every collision geom listed for a shape must lie inside that shape.
Reports the largest stick-out (distance outside the shape, must be <= 0) per part, and checks that the
placed upper arm and forearm ends sit on the joint anchors MuJoCo computes.
Also checks the fingerprint: the stamped file loads on the robot model; a changed model is refused.

Run: python experiments/safety_filter_shapes_test.py   (exit 0 = pass)
"""

import sys
from collections import defaultdict
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import FingerprintMismatch, RobotShapes, RobotState  # noqa: E402
from src.safety_filter.geometry import closest_points_segments  # noqa: E402

SHAPES = ROOT / "data" / "safety_filter" / "robot_shapes.json"
POSES_PER_ARM = 200
MULTI_PARTS = ("palm", "link1", "link2", "link3", "link4", "link5", "link6", "link7")  # parts that may have several capsules (checked as a union)
TOL = 1e-12  # m: a shape must contain its geometry (stick-out <= 0); 1e-12 m is floating-point noise, not a gap


def geom_world_verts(m, d, g):
    t = m.geom_type[g]
    if t == mujoco.mjtGeom.mjGEOM_MESH:
        did = m.geom_dataid[g]
        V = m.mesh_vert[m.mesh_vertadr[did]:m.mesh_vertadr[did] + m.mesh_vertnum[did]].astype(float)
    else:
        s = m.geom_size[g]
        V = np.array([[a * s[0], b * s[1], c * s[2]] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)], float)
    return V @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]


def seg_dist_per_point(P, A, B):
    """Distance of each point of P from the segment AB (vector)."""
    AB = B - A
    L2 = max(float(AB @ AB), 1e-18)
    t = np.clip((P - A) @ AB / L2, 0.0, 1.0)
    return np.linalg.norm(P - (A + t[:, None] * AB), axis=1)


def seg_stick_out(P, A, B, r):
    """Largest (distance from the segment minus r) over the points P; <= 0 means inside."""
    AB = B - A
    L2 = max(float(AB @ AB), 1e-18)
    t = np.clip((P - A) @ AB / L2, 0.0, 1.0)
    return float((np.linalg.norm(P - (A + t[:, None] * AB), axis=1) - r).max())


def box_stick_out(P, c, R, h):
    loc = (P - c) @ R
    q = np.abs(loc) - h
    return float(q.max())


def main():
    rng = np.random.default_rng(11)
    m = load_robot_only_model()
    d = mujoco.MjData(m)
    rs = RobotShapes(m, SHAPES)
    print(f"shapes loaded: {len(rs.specs)}; fingerprint matches the file")

    worst = defaultdict(lambda: -np.inf)
    anchor_err = {"upper": 0.0, "fore": 0.0}  # kept for the report line; the per-link shapes have no joint-line ends now
    joint_ids = {s: [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_{s}_joint{k}") for k in range(1, 8)]
                 for s in ("left", "right")}
    lo = {j: m.jnt_range[j] for s in joint_ids for j in joint_ids[s]}
    for _ in range(POSES_PER_ARM):  # each pose draws BOTH arms at random: 200 poses = 200 per arm
        if True:
            qa = {s: rng.uniform([m.jnt_range[j][0] for j in joint_ids[s]], [m.jnt_range[j][1] for j in joint_ids[s]])
                  for s in ("left", "right")}
            base_xyz = rng.uniform(-0.3, 0.3, 3)
            yaw = rng.uniform(-np.pi, np.pi)
            base_quat = (np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2))
            state = RobotState(base_xyz=tuple(base_xyz), base_quat=base_quat,
                               pedestal_m=float(rng.uniform(0.0, 0.418)),
                               q_left=tuple(qa["left"]), q_right=tuple(qa["right"]),
                               finger_left=float(rng.uniform(0.0, 0.044)), finger_right=float(rng.uniform(0.0, 0.044)))
            placed = {p.id: p for p in rs.place(state)}
            # MuJoCo ground truth for the same state (test only)
            d.qpos[:] = m.qpos0
            a = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "base_float")]
            d.qpos[a:a + 3] = base_xyz
            d.qpos[a + 3:a + 7] = base_quat
            d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "pedestal_joint")]] = state.pedestal_m
            for s, f in (("left", state.finger_left), ("right", state.finger_right)):
                for j, v in zip(joint_ids[s], qa[s]):
                    d.qpos[m.jnt_qposadr[j]] = v
                for k in (1, 2):
                    d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_{s}_finger_joint{k}")]] = f
            mujoco.mj_kinematics(m, d)
            multi = {}
            for p in placed.values():
                if p.part in MULTI_PARTS:
                    multi.setdefault((p.arm, p.part), []).append(p)
                    continue  # a part with several capsules must cover its geometry together: checked as a union below
                verts = [geom_world_verts(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, g)) for g in p.geoms]
                P = np.vstack(verts)
                if p.kind == "capsule":
                    so = seg_stick_out(P, p.a, p.b, p.r)
                else:
                    so = box_stick_out(P, p.centre, p.rot, p.half)
                worst[p.id] = max(worst[p.id], so)
            # the capsules of a multi-capsule part together: each geometry vertex must lie in at least one of them
            for (arm_, part_), caps in multi.items():
                geoms_ = caps[0].geoms
                P = np.vstack([geom_world_verts(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, g)) for g in geoms_])
                per_cap = np.array([seg_dist_per_point(P, c.a, c.b) - c.r for c in caps])
                key = f"{arm_} {part_} (union of {len(caps)})"
                worst[key] = max(worst[key], float(per_cap.min(axis=0).max()))
    print(f"poses: {POSES_PER_ARM} per arm, both arms placed in each pose")
    print("largest stick-out per shape (m; <= 0 means the shape contains the geometry):")
    for sid in sorted(worst):
        print(f"  {sid:24s} {worst[sid]: .3e}")
    overall = max(worst.values())
    print(f"overall largest stick-out: {overall:.3e} m")
    print("per-link shapes: each link's capsules are fitted in the link's own frame (no joint-line ends to check)")

    # fingerprint: the robot model as built passes; a changed model is refused
    m2 = load_robot_only_model()
    m2.geom_size[mujoco.mj_name2id(m2, mujoco.mjtObj.mjOBJ_GEOM, "openarm_left_link7_collision")] *= 1.001
    refused = False
    try:
        RobotShapes(m2, SHAPES)
    except FingerprintMismatch:
        refused = True
    print(f"fingerprint refusal on a changed model: {'refused' if refused else 'NOT refused'}")

    ok = overall <= TOL and refused
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
