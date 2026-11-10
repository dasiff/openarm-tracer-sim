"""Torso share of an over-coverage (kinematics only; nothing stepped).

For a pair (filter shape S of part X, real geometry of part Y), the over-coverage is  real gap - shape gap.
Two of the shapes are replaced by their real geometry, one at a time:
  A = filter shape of X  to the REAL geometry of Y      (the shape's share: X's shape vs the real Y)
  B = REAL geometry of X to the filter shape of Y        (the other shape's share: Y's shape vs the real X)
  Q = REAL X to REAL Y, S = filter X to filter Y (the pair as now).
Then  Q - A  = X-side over-coverage (the shape of X against the real Y),  Q - B = Y-side over-coverage.
The pairs:
  right_upper_arm vs torso at home (P1, pedestal 0.155 m): X = right_upper_arm (links 1-3), Y = torso
  left_forearm_1 vs torso at M9:                          X = left_forearm_1 (links 4-6), Y = torso
  left_forearm_1 vs right_finger_right at M7:             X = left_forearm_1, Y = right finger

MuJoCo placeholders: two unused geoms of the scene are re-typed as capsules for the shapes (measurement only).
"""

import math
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from safety_filter_pick_scene import setup, FINGER_OPEN, FINGER_HELD  # noqa: E402
from safety_filter_pick_scene import set_scene  # noqa: E402
from src.task_config import TASK_CFG  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes, RobotState  # noqa: E402
from src.safety_filter.touch import gap_between  # noqa: E402

SHAPES = ROOT / "data" / "safety_filter" / "robot_shapes.json"
HOME_L = np.array(TASK_CFG["home_q_left"], float)
HOME_R = np.array(TASK_CFG["home_q_right"], float)
PED = 0.155


def place_capsule(m, d, g, A, B, r):
    mid = (A + B) / 2
    L = np.linalg.norm(B - A)
    m.geom_type[g] = mujoco.mjtGeom.mjGEOM_CAPSULE
    m.geom_size[g] = [r, L / 2, 0.0]
    q = np.zeros(4)
    mujoco.mju_quatZ2Vec(q, (B - A) / max(L, 1e-12))
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, q)
    d.geom_xpos[g] = mid
    d.geom_xmat[g] = R


def dmin(m, d, ga, gb):
    fr = np.zeros(6)
    return min(mujoco.mj_geomDistance(m, d, a, b, 0.5, fr) for a in ga for b in gb)


def gid(m, n):
    g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
    assert g >= 0, n
    return g


def case(label, ctx, rs, pose, shape_id, y_geoms, y_capsule_id, x_geoms):
    """shape_id: the filter shape X (capsule); x_geoms: real geoms of X; y: real geoms of Y (or y_capsule_id: a filter capsule of Y)."""
    m, d = ctx["m"], ctx["d"]
    q_l, q_r, f_l, f_r, held = pose
    set_scene(ctx, q_l, q_r, f_l, f_r, held)
    state = RobotState(base_xyz=(ctx["sx"], ctx["sy"], 0.0),
                       base_quat=(math.cos(ctx["yaw"] / 2), 0, 0, math.sin(ctx["yaw"] / 2)),
                       pedestal_m=PED, q_left=tuple(q_l), q_right=tuple(q_r), finger_left=f_l, finger_right=f_r)
    placed = {p.id: p for p in rs.place(state)}
    X = placed[shape_id]
    gX = [gid(m, n) for n in x_geoms]
    gY = [gid(m, n) for n in y_geoms] if y_geoms else None
    ga, gb = gid(m, "task_hotplate_base"), gid(m, "task_hotplate_top_plate")
    if y_capsule_id is None:
        Y = None
        capY = None
    else:
        Y = placed[y_capsule_id]
        capY = Y
    # filter-shape gap (the pair as now)
    if y_capsule_id is None:
        S = gap_between(X, placed["torso"])[0]
        yreal = gY
        Ylabel = "torso"
    else:
        S = gap_between(X, Y)[0]
        yreal = None
        Ylabel = y_capsule_id
    # real-real
    if yreal is None:
        yreal = gY
    Q = dmin(m, d, gX, yreal)
    # A: filter X to the real Y
    place_capsule(m, d, ga, X.a, X.b, X.r)
    A = dmin(m, d, [ga], yreal)
    # B: real X to the filter Y
    if capY is None:
        T = placed["torso"]
        place_capsule(m, d, gb, T.a, T.b, T.r)
    else:
        place_capsule(m, d, gb, capY.a, capY.b, capY.r)
    B = dmin(m, d, gX, [gb])
    return {"label": label, "X": shape_id, "Y": Ylabel, "S": S, "Q": Q, "A": A, "B": B}


def show(r):
    print(f"{r['label']:44s} {r['X']} vs {r['Y']}: shape {r['S'] * 1e3:+7.1f}  real {r['Q'] * 1e3:+7.1f}  "
          f"| filter X to REAL Y {r['A'] * 1e3:+7.1f}  (X-side over-coverage {(r['Q'] - r['A']) * 1e3:+6.1f} mm)  "
          f"| REAL X to filter Y {r['B'] * 1e3:+7.1f}  (Y-side over-coverage {(r['Q'] - r['B']) * 1e3:+6.1f} mm)")


def main():
    rs = RobotShapes(load_robot_only_model(), SHAPES)
    ctx = setup(verbose=True, pedestal=PED)
    m = ctx["m"]
    fo = FINGER_OPEN
    right_links = [f"openarm_right_link{k}_collision" for k in (1, 2, 3)]
    left_links_fore = [f"openarm_left_link{k}_collision" for k in (4, 5, 6)]
    torso = ["openarm_body_link0_collision"]
    out = []
    # 1. P1 home, right upper arm vs torso
    pose_home = (HOME_L, HOME_R, fo, fo, None)
    out.append(case("P1 home: right_upper_arm vs torso", ctx, rs, pose_home, "right_upper_arm", torso, None, right_links))
    # 2. M9 left forearm 1 vs torso
    q9 = ctx["poses"]["M9"]
    pose9 = (q9, HOME_R, fo, fo, None)
    out.append(case("M9: left_forearm_1 vs torso", ctx, rs, pose9, "left_forearm_1", torso, None, left_links_fore))
    # 3. M7 left forearm 1 vs right finger (filter finger capsule as the Y capsule, real right finger mesh as Y real)
    q7 = ctx["poses"]["M7"]
    pose7 = (q7, HOME_R, FINGER_HELD, fo, None)
    fingers = ["openarm_right_right_finger_collision", "openarm_right_left_finger_collision"]
    out.append(case("M7: left_forearm_1 vs right_finger_right", ctx, rs, pose7, "left_forearm_1", ["openarm_right_right_finger_collision"], "right_finger_right", left_links_fore))
    print()
    for r in out:
        show(r)
    return 0


if __name__ == "__main__":
    sys.exit(main())
