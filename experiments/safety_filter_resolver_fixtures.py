"""Shared fixtures of the Stage 2 resolver tests and the Kong comparison: the robot-only model, the shapes, the resolver with
Kong's converter, a Jacobian IK made of repeated single Jacobian steps, and a few hand-made poses. No physics, no scene."""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src import task_config as tc  # noqa: E402
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.arm_conversion import make_solver  # noqa: E402
from src.safety_filter.resolver import Resolver  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes, RobotState  # noqa: E402
from src.safety_filter.touch import gap_between  # noqa: E402

DT = 1.0 / 150.0   # s: the control loop's tick (LOOP_CFG["source_hz"] = 150, src/control_loop.py); the tests read no simulator
HOME = np.array([tc.TASK_CFG["home_q_left"], tc.TASK_CFG["home_q_right"]], dtype=float)
MIRROR = np.array([-1, -1, -1, 1, -1, -1, -1], dtype=float)   # right joints = MIRROR x left joints at the zombie pose (mirror symmetry of the robot)
_CACHE = {}


def model_and_shapes():
    if "ms" not in _CACHE:
        m = load_robot_only_model()
        _CACHE["ms"] = (m, RobotShapes(m, ROOT / cfg.SHAPES_FILE))
    return _CACHE["ms"]


def solver():
    if "solver" not in _CACHE:
        _CACHE["solver"] = make_solver()
    return _CACHE["solver"]


def new_resolver(with_solver=True):
    _, sh = model_and_shapes()
    return Resolver(sh, DT, solver() if with_solver else None)


def state_of(q, base=None):
    return RobotState(q_left=tuple(q[0]), q_right=tuple(q[1])) if base is None else base


def ik(res, q0, side, pos, rot, iters=40):
    """Joint angles of one arm that put the hand at pos, rot: repeated single Jacobian steps (our model's Jacobian)."""
    q = np.array(q0, float)
    i = 0 if side == "left" else 1
    for _ in range(iters):
        q[i] = res.kin.jacobian_step(state_of(q), side, pos, rot)
    return q


def hand_hand_gap(q, kinds=("palm", "finger", "link7")):
    """Smallest exact gap between a hand shape of the left arm and one of the right arm at the joint angles q: (gap, (id, id))."""
    _, sh = model_and_shapes()
    placed = sh.place(state_of(q))
    best = (9.0, None)
    for a in placed:
        for b in placed:
            if a.arm == "left" and b.arm == "right" and a.part in kinds and b.part in kinds:
                g = float(gap_between(a, b)[0])
                if g < best[0]:
                    best = (g, (a.id, b.id))
    return best


def left_hand_at_y(res, y):
    """Left arm joint angles that put the left hand at lateral position y (m), other coordinates and orientation as at the zombie pose;
    the right arm at its zombie pose."""
    p, r = res.kin.hand_pose(state_of(HOME), "left")
    return ik(res, HOME, "left", np.array([p[0], y, p[2]]), r)
