"""Arm kinematics for the resolver on OUR MuJoCo model, and the conversion back to joint angles (SF-1, SF-32).

  * hand pose (link 7 frame), the shoulder / elbow / wrist points S, E, W of one arm;
  * one Jacobian step (damped least squares, position and orientation) on our model;
  * the converter: Kong's solver with its own collision avoidance OFF, given DIRECTIONS only (upper arm = our
    joint 3 axis, forearm = our joint 5 axis) and the commanded hand orientation (link 7 orientation times C_HAND).
    The solver is injected (make_solver builds Kong's); this module never imports the simulator or the scene.

S is where the axes of joints 1 and 2 meet, W is the origin of joint 7, E is where the line through S along the
joint 3 axis meets the line through W along the joint 5 axis (the SEW points of the solver; checked against its
own forward kinematics in experiments/safety_filter_resolver_test.py).
"""

from dataclasses import replace

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from src.safety_filter import config as cfg
from src.safety_filter.shapes import ARM_JOINTS, SIDES, RobotShapes, RobotState

SOLVER_UPPER_LEN_M = 0.3   # m: lengths of the direction-only frame handed to the solver (any value; the solver reads directions) [SF-1]


def make_solver():
    """Kong's solver as a converter: collision avoidance off (SF-1). Needs the licensed geo_kin wheel (Linux)."""
    from src.geo_kin_self_collision_filter import SafetyFilter
    return SafetyFilter(collision_avoidance=False)


def _line_closest(p1, d1, p2, d2):
    """Midpoint of the closest points of two lines (point, direction)."""
    n = np.cross(d1, d2)
    nn = float(n @ n)
    if nn < 1e-12:
        return 0.5 * (p1 + p2)
    t1 = float(np.cross(p2 - p1, d2) @ n) / nn
    t2 = float(np.cross(p2 - p1, d1) @ n) / nn
    return 0.5 * ((p1 + t1 * d1) + (p2 + t2 * d2))


def rotation_error_rad(r_target, r_now):
    """Angle (rad) of the rotation that takes r_now to r_target."""
    return float(np.linalg.norm(Rotation.from_matrix(r_target @ r_now.T).as_rotvec()))


class ArmKinematics:
    """Kinematics of both arms on the robot-only model of a RobotShapes object (no scene, no world)."""

    def __init__(self, shapes: RobotShapes, solver=None):
        self.shapes = shapes
        self.model, self.data = shapes.model, shapes.data
        self.solver = solver
        m = self.model
        bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
        self.hand_body = {s: bid(f"openarm_{s}_link7") for s in SIDES}
        self.body0 = bid("openarm_body_link0")
        self.jids = {s: list(shapes._joint[s]) for s in SIDES}
        self.dofs = {s: np.array([m.jnt_dofadr[j] for j in self.jids[s]]) for s in SIDES}
        self.limits = {s: np.array([m.jnt_range[j] for j in self.jids[s]]) for s in SIDES}

    # ---- kinematics (each call sets the state; callers that need several things use the *_now helpers) ----
    def set_state(self, state: RobotState):
        self.shapes.set_state(state)

    def hand_pose_now(self, side):
        """(position (3,), rotation (3,3)) of link 7 of one arm for the state last set."""
        b = self.hand_body[side]
        return self.data.xpos[b].copy(), self.data.xmat[b].reshape(3, 3).copy()

    def hand_pose(self, state, side):
        self.set_state(state)
        return self.hand_pose_now(side)

    def sew_now(self, side):
        """(S, E, W) in the world for the state last set."""
        d = self.data
        a1, a2 = d.xaxis[self.jids[side][0]], d.xaxis[self.jids[side][1]]
        p1, p2 = d.xanchor[self.jids[side][0]], d.xanchor[self.jids[side][1]]
        s = _line_closest(p1, a1, p2, a2)
        w = d.xanchor[self.jids[side][6]].copy()
        u, l = d.xaxis[self.jids[side][2]].copy(), d.xaxis[self.jids[side][4]].copy()
        e = _line_closest(s, u, w, l)
        return s, e, w

    def sew(self, state, side):
        self.set_state(state)
        return self.sew_now(side)

    def jacobian_step(self, state, side, target_pos, target_rot):
        """One damped least-squares Jacobian step on our model: new joint angles (7,) of the arm that move the hand
        (link 7 frame) from its pose at `state` toward target_pos (m) and target_rot (3x3). Exactly one step, no iteration."""
        self.set_state(state)
        pos, rot = self.hand_pose_now(side)
        jp, jr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
        mujoco.mj_comPos(self.model, self.data)           # the Jacobian needs the joint axes in the world (cdof)
        mujoco.mj_jacBody(self.model, self.data, jp, jr, self.hand_body[side])
        J = np.vstack([jp[:, self.dofs[side]], jr[:, self.dofs[side]]])
        err = np.concatenate([np.asarray(target_pos, float) - pos, Rotation.from_matrix(np.asarray(target_rot) @ rot.T).as_rotvec()])
        lam = cfg.RESOLVER["jacobian_damping"]
        dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(6), err)
        q = np.array(state.q_left if side == "left" else state.q_right, float)
        return q + dq

    def convert_directions(self, state, side, upper_dir, fore_dir, hand_rot):
        """Joint angles (7,) of one arm from directions only, by the injected solver (collision avoidance off).

        upper_dir, fore_dir: unit vectors (world) for the upper arm (joint 3 axis) and the forearm (joint 5 axis).
        hand_rot (3x3, world): the link 7 orientation wanted. The other arm is handed its own directions at `state`
        (the solver returns it unchanged to rounding; only the asked arm is returned).
        Raises RuntimeError when no solver is set or the solver fails."""
        if self.solver is None:
            raise RuntimeError("NO_SOLVER: the resolver was built without a converter")
        from geo_kin_core.types import RetargetFrame, SEWPose
        from src.geo_kin_self_collision_filter import C_HAND
        self.set_state(state)
        rb = self.data.xmat[self.body0].reshape(3, 3).copy()
        poses = {}
        fk = self.solver.session.robot_fk_at(q_right=np.asarray(state.q_right, float), q_left=np.asarray(state.q_left, float))
        for s in SIDES:
            u = self.data.xaxis[self.jids[s][2]].copy()
            l = self.data.xaxis[self.jids[s][4]].copy()
            rh = self.data.xmat[self.hand_body[s]].reshape(3, 3).copy()
            if s == side:
                u, l, rh = np.asarray(upper_dir, float), np.asarray(fore_dir, float), np.asarray(hand_rot, float)
            S = np.asarray(fk[s]["S"], float)
            E = S + SOLVER_UPPER_LEN_M * (rb.T @ u)
            W = E + SOLVER_UPPER_LEN_M * (rb.T @ l)
            poses[s] = SEWPose(S=S, E=E, W=W, R_world_wrist=(rb.T @ rh) @ C_HAND)
        frame = RetargetFrame(left_sew=poses["left"], right_sew=poses["right"],
                              R_world_upper_body=np.asarray(fk["R_base_body"], float),
                              p_world_upper_body=np.asarray(fk["p_base_body"], float))
        ql, qr = np.asarray(state.q_left, float), np.asarray(state.q_right, float)
        try:
            out_l, out_r = self.solver.solve(frame, ql, qr, reset=cfg.RESOLVER["solver_current_reset"], q_reset=(ql, qr))
        except Exception as exc:  # the solver is a compiled wheel: any failure means hold still
            raise RuntimeError(f"SOLVER_FAILED: {exc}") from exc
        return np.asarray(out_l if side == "left" else out_r, float)


def with_arms(state: RobotState, q):
    """The state with both arms' joints replaced; q is a (2, 7) array (left, right)."""
    return replace(state, q_left=tuple(float(v) for v in q[0]), q_right=tuple(float(v) for v in q[1]))
