"""Compiled forward kinematics of the robot-only model (Braking Phase O, SF-103, David).

The filter places every shape from the joint state each tick. Until now that ran MuJoCo's mj_kinematics (and mj_comPos and mj_jacBody for
the hand Jacobian) and then rebuilt the shape arrays in Python. This module does the same chain in Numba: the fixed parameters of the
kinematic tree are read once from the MuJoCo model (RobotKinematics.__init__), and update() multiplies the rotation matrices down the
tree for one joint state. No MuJoCo call is made per tick.

Same numbers as MuJoCo, body by body: a body's frame is its parent's frame times its own body_pos and body_quat; each joint then moves
the frame (a hinge turns about its axis through its anchor, a slide moves along its axis, a free joint sets the frame from qpos); the
anchor and axis of every joint are stored (xanchor, xaxis); xipos is the body's centre of mass. The hand Jacobian is the one of
mj_jacBody: the columns of the arm joints at the hand body's frame origin (xpos), which is the point mj_jacBody uses (checked, SF-103).

The arrays have the shapes and the layout of MjData's (xpos (nbody, 3), xmat (nbody, 9) row-major, xipos (nbody, 3), xanchor and
xaxis (njnt, 3)), and the joint and body indices are MuJoCo's, so code that reads MjData reads these arrays the same way.
Ball joints are not in the robot model and are refused at build time.
"""

import mujoco
import numpy as np
from numba import njit

FREE, SLIDE, HINGE = int(mujoco.mjtJoint.mjJNT_FREE), int(mujoco.mjtJoint.mjJNT_SLIDE), int(mujoco.mjtJoint.mjJNT_HINGE)


@njit(cache=True)
def _cross(a0, a1, a2, b0, b1, b2):
    return a1 * b2 - a2 * b1, a2 * b0 - a0 * b2, a0 * b1 - a1 * b0


@njit(cache=True)
def quat_to_mat(q, out):
    """Rotation matrix of a unit quaternion (w, x, y, z) into out (3, 3)."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    out[0, 0] = 1 - 2 * (y * y + z * z)
    out[0, 1] = 2 * (x * y - w * z)
    out[0, 2] = 2 * (x * z + w * y)
    out[1, 0] = 2 * (x * y + w * z)
    out[1, 1] = 1 - 2 * (x * x + z * z)
    out[1, 2] = 2 * (y * z - w * x)
    out[2, 0] = 2 * (x * z - w * y)
    out[2, 1] = 2 * (y * z + w * x)
    out[2, 2] = 1 - 2 * (x * x + y * y)


@njit(cache=True)
def axis_angle_mat(ax, ang, out):
    """Rotation of angle ang (rad) about the unit axis ax (3,) into out (3, 3) (Rodrigues)."""
    c, s = np.cos(ang), np.sin(ang)
    t = 1.0 - c
    x, y, z = ax[0], ax[1], ax[2]
    out[0, 0] = c + t * x * x
    out[0, 1] = t * x * y - s * z
    out[0, 2] = t * x * z + s * y
    out[1, 0] = t * x * y + s * z
    out[1, 1] = c + t * y * y
    out[1, 2] = t * y * z - s * x
    out[2, 0] = t * x * z - s * y
    out[2, 1] = t * y * z + s * x
    out[2, 2] = c + t * z * z


@njit(cache=True)
def kinematics(qpos, qpos0, parent, bpos, bmat, ipos, jstart, jnum, jtype, jqadr, jpos, jaxis, xpos, xmat, xipos, xanchor, xaxis):
    """mj_kinematics for the robot-only tree. Writes xpos (nb, 3), xmat (nb, 9), xipos (nb, 3), xanchor and xaxis (nj, 3) in place.
    Bodies are in MuJoCo's order (a parent comes before its children); body 0 is the world."""
    nb = parent.shape[0]
    Rp = np.empty((3, 3))
    R = np.empty((3, 3))
    Rq = np.empty((3, 3))
    Rj = np.empty((3, 3))
    o = np.empty(3)
    xpos[0, :] = 0.0
    xmat[0, :] = 0.0
    xmat[0, 0] = 1.0
    xmat[0, 4] = 1.0
    xmat[0, 8] = 1.0
    xipos[0, :] = 0.0
    for i in range(1, nb):
        p = parent[i]
        for a in range(3):
            for b in range(3):
                Rp[a, b] = xmat[p, 3 * a + b]
        for a in range(3):
            o[a] = xpos[p, a] + Rp[a, 0] * bpos[i, 0] + Rp[a, 1] * bpos[i, 1] + Rp[a, 2] * bpos[i, 2]
        for a in range(3):
            for b in range(3):
                R[a, b] = Rp[a, 0] * bmat[i, 0, b] + Rp[a, 1] * bmat[i, 1, b] + Rp[a, 2] * bmat[i, 2, b]
        for j in range(jstart[i], jstart[i] + jnum[i]):
            q = jqadr[j]
            if jtype[j] == FREE:
                # the free joint sets the frame from qpos (the body's own pos and quat are not used)
                for a in range(3):
                    o[a] = qpos[q + a]
                nrm = np.sqrt(qpos[q + 3] ** 2 + qpos[q + 4] ** 2 + qpos[q + 5] ** 2 + qpos[q + 6] ** 2)
                quat = np.empty(4)
                for a in range(4):
                    quat[a] = qpos[q + 3 + a] / nrm
                quat_to_mat(quat, R)
                for a in range(3):
                    xanchor[j, a] = o[a]
                    xaxis[j, a] = 0.0
            elif jtype[j] == HINGE:
                # the anchor and the axis are taken before the joint turns the frame; the frame turns about the axis
                ax = jaxis[j]
                for a in range(3):
                    xanchor[j, a] = o[a] + R[a, 0] * jpos[j, 0] + R[a, 1] * jpos[j, 1] + R[a, 2] * jpos[j, 2]
                    xaxis[j, a] = R[a, 0] * ax[0] + R[a, 1] * ax[1] + R[a, 2] * ax[2]
                axis_angle_mat(ax, qpos[q] - qpos0[q], Rj)
                for a in range(3):
                    for b in range(3):
                        Rq[a, b] = R[a, 0] * Rj[0, b] + R[a, 1] * Rj[1, b] + R[a, 2] * Rj[2, b]
                for a in range(3):
                    for b in range(3):
                        R[a, b] = Rq[a, b]
                for a in range(3):
                    o[a] = xanchor[j, a] - (R[a, 0] * jpos[j, 0] + R[a, 1] * jpos[j, 1] + R[a, 2] * jpos[j, 2])
            elif jtype[j] == SLIDE:
                # MuJoCo takes the anchor before the slide moves the frame (checked against mj_kinematics, SF-103)
                ax = jaxis[j]
                d = qpos[q] - qpos0[q]
                for a in range(3):
                    xanchor[j, a] = o[a] + R[a, 0] * jpos[j, 0] + R[a, 1] * jpos[j, 1] + R[a, 2] * jpos[j, 2]
                    xaxis[j, a] = R[a, 0] * ax[0] + R[a, 1] * ax[1] + R[a, 2] * ax[2]
                    o[a] = o[a] + xaxis[j, a] * d
        for a in range(3):
            xpos[i, a] = o[a]
            xipos[i, a] = o[a] + R[a, 0] * ipos[i, 0] + R[a, 1] * ipos[i, 1] + R[a, 2] * ipos[i, 2]
            for b in range(3):
                xmat[i, 3 * a + b] = R[a, b]


@njit(cache=True)
def hand_jacobian(point, xanchor, xaxis, joints, jp, jr):
    """mj_jacBody of a body for the hinge joints `joints` (one column each): the linear part jp (3, n) is axis x (point - anchor),
    the angular part jr (3, n) is the axis. point is the body's frame origin (xpos): the point mj_jacBody uses."""
    for k in range(joints.shape[0]):
        j = joints[k]
        ax, ay, az = xaxis[j, 0], xaxis[j, 1], xaxis[j, 2]
        dx, dy, dz = point[0] - xanchor[j, 0], point[1] - xanchor[j, 1], point[2] - xanchor[j, 2]
        cx, cy, cz = _cross(ax, ay, az, dx, dy, dz)
        jp[0, k], jp[1, k], jp[2, k] = cx, cy, cz
        jr[0, k], jr[1, k], jr[2, k] = ax, ay, az


class RobotKinematics:
    """The kinematic tree of one robot-only MjModel, with its state arrays (the same layout as MjData's).

    update(qpos) runs the chain for one joint state (qpos: the model's nq values, as MuJoCo's qpos). The arrays xpos, xmat, xipos,
    xanchor and xaxis are then the values mj_kinematics would give. Fixed parameters are copied from the model here, once."""

    def __init__(self, model):
        if np.any(model.jnt_type == int(mujoco.mjtJoint.mjJNT_BALL)):
            raise ValueError("the compiled kinematics has no ball joints; the robot model has one")
        self.nbody, self.njnt, self.nq = int(model.nbody), int(model.njnt), int(model.nq)
        self.parent = np.asarray(model.body_parentid, np.int64).copy()
        self.jstart = np.asarray(model.body_jntadr, np.int64).copy()
        self.jnum = np.asarray(model.body_jntnum, np.int64).copy()
        self.jtype = np.asarray(model.jnt_type, np.int64).copy()
        self.jqadr = np.asarray(model.jnt_qposadr, np.int64).copy()
        self.jpos = np.asarray(model.jnt_pos, float).copy()
        self.jaxis = np.asarray(model.jnt_axis, float).copy()
        self.bpos = np.asarray(model.body_pos, float).copy()
        self.ipos = np.asarray(model.body_ipos, float).copy()
        self.bmat = np.zeros((self.nbody, 3, 3))
        for b in range(self.nbody):
            quat_to_mat(np.asarray(model.body_quat[b], float), self.bmat[b])
        self.qpos0 = np.asarray(model.qpos0, float).copy()
        self.xpos = np.zeros((self.nbody, 3))
        self.xmat = np.zeros((self.nbody, 9))
        self.xipos = np.zeros((self.nbody, 3))
        self.xanchor = np.zeros((self.njnt, 3))
        self.xaxis = np.zeros((self.njnt, 3))

    def update(self, qpos):
        kinematics(np.asarray(qpos, float), self.qpos0, self.parent, self.bpos, self.bmat, self.ipos, self.jstart, self.jnum, self.jtype,
                   self.jqadr, self.jpos, self.jaxis, self.xpos, self.xmat, self.xipos, self.xanchor, self.xaxis)


@njit(cache=True)
def _to_world(xpos, xmat, body, v, out):
    """World position of the body-local point v of one body: out = xpos[body] + xmat[body] @ v."""
    for i in range(3):
        out[i] = xpos[body, i] + xmat[body, 3 * i] * v[0] + xmat[body, 3 * i + 1] * v[1] + xmat[body, 3 * i + 2] * v[2]


@njit(cache=True)
def place_kernel(xpos, xmat, cap_ba, cap_bb, cap_la, cap_lb, cap_t0, cap_t1, box_b, box_c, box_rl, A, B, C, R):
    """The shape arrays of RobotShapes.place_arrays from the body frames, in place (SF-103; the same formulas as the numpy version):
    a capsule's ends are A0, B0 (the body-local ends mapped to the world); its covered part is A = A0 + t0 (B0 - A0) and B = A0 + t1 (B0 - A0).
    A box's centre is C = xpos + xmat @ centre_local and its rotation R = xmat @ quat_local_matrix."""
    a0 = np.empty(3)
    b0 = np.empty(3)
    for k in range(cap_ba.shape[0]):
        _to_world(xpos, xmat, cap_ba[k], cap_la[k], a0)
        _to_world(xpos, xmat, cap_bb[k], cap_lb[k], b0)
        for i in range(3):
            A[k, i] = a0[i] + cap_t0[k] * (b0[i] - a0[i])
            B[k, i] = a0[i] + cap_t1[k] * (b0[i] - a0[i])
    for k in range(box_b.shape[0]):
        b = box_b[k]
        out = np.empty(3)
        _to_world(xpos, xmat, b, box_c[k], out)
        for i in range(3):
            C[k, i] = out[i]
            for j in range(3):
                R[k, i, j] = xmat[b, 3 * i] * box_rl[k, 0, j] + xmat[b, 3 * i + 1] * box_rl[k, 1, j] + xmat[b, 3 * i + 2] * box_rl[k, 2, j]
