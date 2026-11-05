"""Cheap and slow passes of the stance selection, on the world-model clearance (stage 3). Off by default: see stance_check_mode.

Per candidate (a stance, an arm, an approach bin, a pad target) the IK is the verification's: the zombie start only (M187), the bin's
approach constraint, the pick tolerance. The clearance is the smallest hand/finger/forearm distance to each world-model box
(src/world_collision.py); the margin (world_margin_m) caps it.

Hard reject (both passes): the grasp IK fails, or the grasp pose penetrates a box (sign only).
Cheap pass: the grasp clears every box by the margin. If any candidate passes, the cheap pass is done.
Slow pass (only when nothing passes cheap). The candidates that failed the cheap pass without a hard reject are:
  class (a): the grasp is within the margin and neither the pre-grasp nor the over-the-top pose clears it (padding is the cause);
  class (b): the grasp is within the margin, but the pre-grasp or the over-the-top pose does clear it (over-reachable).
  The slow pass evaluates all three poses. Everything not hard-rejected is accepted, with the clearance (the smallest over the three
  poses, capped at the margin) as a soft penalty for the ranking.
"""

import math

import mujoco
import numpy as np

from src.arm_kinematics import arm_chain, batch_fk, solve_ik
from src.gripper_geometry import grasp_cfg
from src.grasp_rules import bin_center, joint_bounds, make_constraint
from src.world_collision import check_boxes, robot_group_points

POSES = ("grasp", "pre_grasp", "over")
OVER_ABOVE_TOP_M = 0.05        # the over-the-top pose: this far above the hotplate top (provisional, stage 3)


class ArmPoses:
    """The IK set-up of one arm: its chain, grasp configuration, joint bounds, pad centre and zombie pose (the verification's start)."""

    def __init__(self, model, side, home_q):
        self.side = side
        self.C = grasp_cfg(model, side)
        self.chain = arm_chain(model, side)
        self.lo, self.hi = joint_bounds(model, side, self.C)
        self.pad = np.array(self.C["pad_center_in_tcp"], float)
        self.zombie = np.array(home_q, float)


def to_torso(stance, P, rg):
    """World points (N, 3) into the torso frame of a stance (x, y, yaw, pedestal command)."""
    x, y, th, q = stance
    c, s = math.cos(th), math.sin(th)
    dx, dy = P[:, 0] - x, P[:, 1] - y
    tz = rg["base_z"] + rg["torso_z0"] + rg["lift_factor"] * q
    return np.stack([c * dx + s * dy, -s * dx + c * dy, P[:, 2] - tz], axis=-1)


def pose_targets(beaker_xy, zg, bin_world_angle, top_z, standoff):
    """World pad-centre targets of the three poses for one bin. The pre-grasp is `standoff` back along the bin's world direction."""
    G = np.asarray(beaker_xy, float)
    u = np.array([math.cos(bin_world_angle), math.sin(bin_world_angle)])
    pre = G - standoff * u
    return {"grasp": np.array([G[0], G[1], zg]),
            "pre_grasp": np.array([pre[0], pre[1], zg]),
            "over": np.array([pre[0], pre[1], top_z + OVER_ABOVE_TOP_M])}


def pose_clearance(model, data, side, boxes, margin):
    """Smallest hand/finger/forearm distance to any box, and the same per group, for the pose now in `data` (metres; no cap)."""
    groups = robot_group_points(model, data, side)
    res = check_boxes(groups, boxes, margin)
    per_group = {g: res[g]["min"] for g in res if res[g]["min"] is not None}
    return min(per_group.values()) if per_group else None, per_group


def evaluate_cell(model, data, arm, stance, rg, boxes, margin, bins, beaker_xy, zg, top_z, standoff, tol_m, iters):
    """Per bin and pose: IK accepted, the joint vector, the IK error, and the raw smallest distance to the boxes (None if IK failed).

    The three poses of a bin are solved in one batch (each from the zombie start, the verification's rule). Sets the robot in `data`
    to each accepted pose in turn (kinematics only). Returns {bin: {pose: dict}}.
    """
    out = {}
    for k in bins:
        a = math.radians(30.0 * k)
        tg = pose_targets(beaker_xy, zg, a + stance[2], top_z, standoff)
        T = np.stack([to_torso(stance, tg[name][None], rg)[0] for name in POSES])
        Q, _, _ = solve_ik(arm.chain, np.repeat(arm.zombie[None], len(POSES), axis=0), arm.lo, arm.hi,
                           make_constraint(T, k, arm.C), arm.pad, iters, arm.C["ik_damping"], arm.C["ik_max_step"])
        p, Rf = batch_fk(arm.chain, Q, arm.pad)
        row = {}
        for i, name in enumerate(POSES):
            err = float(np.linalg.norm(p[i] - T[i]))
            wa = Rf[i] @ np.array(arm.C["approach_axis"], float)
            dang = (math.atan2(wa[1], wa[0]) - bin_center(k, arm.C) + math.pi) % (2 * math.pi) - math.pi
            elev = math.degrees(math.asin(max(-1.0, min(1.0, wa[2]))))
            ok = err <= tol_m and abs(dang) <= math.radians(arm.C["bin_half_deg"]) and abs(elev) <= arm.C["level_deg"]
            d_raw = None
            if ok:
                data.qpos[arm_qadr(model, arm.side)] = Q[i]
                mujoco.mj_forward(model, data)
                d_raw, _ = pose_clearance(model, data, arm.side, boxes, margin)
            row[name] = {"ik": bool(ok), "err_m": err, "raw_m": d_raw}
        out[k] = row
    return out


def arm_qadr(model, side):
    from src.robot_helpers import arm_ids
    return arm_ids(model, side)["qadr"]


def classify(row, margin):
    """One bin's verdict from its three poses: 'hard' (grasp IK fails or the grasp penetrates), 'cheap' (grasp clears the margin),
    'a' (grasp within the margin; pre-grasp and over do not clear it), 'b' (grasp within the margin; pre-grasp or over clears it)."""
    g = row["grasp"]
    if not g["ik"] or g["raw_m"] is None or g["raw_m"] < 0.0:
        return "hard"
    if g["raw_m"] >= margin:
        return "cheap"
    clears = [row[p]["raw_m"] is not None and row[p]["raw_m"] >= margin for p in ("pre_grasp", "over")]
    return "b" if any(clears) else "a"


def soft_clearance(row, margin):
    """The soft ranking value of a non-hard-rejected bin: the smallest raw distance over the three poses that solved, capped at the margin."""
    vals = [row[p]["raw_m"] for p in POSES if row[p]["raw_m"] is not None]
    return min(margin, min(vals)) if vals else None
