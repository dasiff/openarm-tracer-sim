"""Torso exemption evidence (SF-9 method, re-run after the forearm and link 7 pieces; kinematics only).

For every arm shape of both arms: the gap to the torso capsule at the zero pose, and the smallest gap and the number
of poses (of 200 random poses, each arm drawn at random in each pose) in which it overlaps the torso.
Rule (SF-9): a shape is exempt from the torso only if it overlaps the torso at the zero pose (touches it by construction).
Prints the table and the exempt set that the rule gives; the touch test checks config.TORSO_TOUCHING_IDS against it.
"""

import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.safety_filter.config import TORSO_TOUCHING_IDS  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes, RobotState  # noqa: E402
from src.safety_filter.touch import gap_between  # noqa: E402

SHAPES = ROOT / "data" / "safety_filter" / "robot_shapes.json"
PARTS = ("upper_arm", "forearm", "shoulder_mount", "link7", "palm", "finger")
N = 200


def torso_gaps(rs, state):
    placed = rs.place(state)
    torso = next(p for p in placed if p.id == "torso")
    return {p.id: gap_between(p, torso)[0] for p in placed if p.arm in ("left", "right") and p.part in PARTS}


def evidence():
    m = load_robot_only_model()
    rs = RobotShapes(m, SHAPES)
    zero = RobotState(pedestal_m=0.15, finger_left=0.02, finger_right=0.02)  # the SF-9 zero pose (pedestal 0.15 m, fingers 0.02 m)
    z = torso_gaps(rs, zero)
    rng = np.random.default_rng(5)
    jl = {s: [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_{s}_joint{k}") for k in range(1, 8)] for s in ("left", "right")}
    mins, counts = {}, {}
    for _ in range(N):
        q = {s: rng.uniform([m.jnt_range[j][0] for j in jl[s]], [m.jnt_range[j][1] for j in jl[s]]) for s in ("left", "right")}
        st = RobotState(pedestal_m=0.15, q_left=tuple(q["left"]), q_right=tuple(q["right"]), finger_left=0.02, finger_right=0.02)
        g = torso_gaps(rs, st)
        for k, v in g.items():
            mins[k] = min(mins.get(k, np.inf), v)
            counts[k] = counts.get(k, 0) + (v < 0)
    rows = []
    for k in sorted(z):
        rows.append((k, z[k], mins[k], counts[k]))
        print(f"{k:24s} zero {z[k] * 1e3:+8.1f} mm   min over {N} poses {mins[k] * 1e3:+8.1f} mm   overlap in {counts[k]:3d}/{N}")
    exempt = {k for k, zz, _, _ in rows if zz < 0}
    print("exempt by the SF-9 rule (overlap at the zero pose):", sorted(exempt))
    print("config TORSO_TOUCHING_IDS:", sorted(TORSO_TOUCHING_IDS))
    return exempt


if __name__ == "__main__":
    evidence()
