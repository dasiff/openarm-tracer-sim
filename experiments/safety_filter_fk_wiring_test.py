"""Wiring of the compiled arm chain into the filter (Braking Phase O, SF-104): RobotShapes and ArmKinematics against the old MuJoCo path.

100 random joint states (base pose, pedestal, both arms, fingers, each within its range). For each state, the reference is MuJoCo's own
forward kinematics (mj_kinematics) with the old formulas:
  * place_arrays: capsule ends A, B and box centres C and rotations R (the einsum formulas of the MuJoCo-based code);
  * hand_frame: link 7 position and rotation of each arm;
  * jacobian_step: one damped least-squares step with mj_comPos + mj_jacBody (the old code), for a random target pose near the hand.

PASS: the worst difference of the shape arrays and hand frames is under 1e-9 m and the worst joint-angle difference of the Jacobian step
is under 1e-9 rad (both logged).
Run: python experiments/safety_filter_fk_wiring_test.py   (exit 0 on pass)
"""
import os
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.safety_filter import config as sfcfg  # noqa: E402
from src.safety_filter.arm_conversion import ArmKinematics  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import SIDES, RobotShapes, RobotState  # noqa: E402

N_STATES = 100
TOL_M = 1e-9
TOL_RAD = 1e-9


def random_state(model, shapes, rng):
    q_arm = {}
    for s in SIDES:
        q_arm[s] = tuple(float(rng.uniform(*model.jnt_range[j])) for j in shapes._joint[s])
    quat = rng.normal(size=4)
    return RobotState(base_xyz=tuple(rng.uniform(-0.5, 0.5, 3)), base_quat=tuple(quat / np.linalg.norm(quat)),
                      pedestal_m=float(rng.uniform(0.0, 0.418)), q_left=q_arm["left"], q_right=q_arm["right"],
                      finger_left=float(rng.uniform(0.0, 0.044)), finger_right=float(rng.uniform(0.0, 0.044)))


def mujoco_qpos(shapes, st):
    m = shapes.model
    q = np.array(m.qpos0, float)
    q[shapes._adr_base:shapes._adr_base + 3] = st.base_xyz
    q[shapes._adr_base + 3:shapes._adr_base + 7] = st.base_quat
    q[shapes._adr_ped] = st.pedestal_m
    for side, qa, f in (("left", st.q_left, st.finger_left), ("right", st.q_right, st.finger_right)):
        q[shapes._adr_arm[side]] = qa
        q[shapes._adr_fing[side]] = f
    return q


def old_place_arrays(shapes, d):
    """The formulas of the MuJoCo-based place_arrays, on MuJoCo's MjData d."""
    xpos = d.xpos
    xmat = d.xmat.reshape(-1, 3, 3)
    A = B = C = R = None
    if len(shapes._cap_idx):
        A0 = xpos[shapes._cap_ba] + np.einsum("kij,kj->ki", xmat[shapes._cap_ba], shapes._cap_la)
        B0 = xpos[shapes._cap_bb] + np.einsum("kij,kj->ki", xmat[shapes._cap_bb], shapes._cap_lb)
        A = A0 + shapes._cap_t0[:, None] * (B0 - A0)
        B = A0 + shapes._cap_t1[:, None] * (B0 - A0)
    if len(shapes._box_idx):
        C = xpos[shapes._box_b] + np.einsum("kij,kj->ki", xmat[shapes._box_b], shapes._box_c)
        R = np.einsum("kij,kjl->kil", xmat[shapes._box_b], shapes._box_rl)
    return A, B, C, R


def old_jacobian_step(model, d, dofs, body, q_arm, target_pos, target_rot, lam):
    """The old ArmKinematics.jacobian_step, on MuJoCo's data d (the state already set and kinematics run)."""
    pos = d.xpos[body].copy()
    rot = d.xmat[body].reshape(3, 3).copy()
    mujoco.mj_comPos(model, d)
    jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, d, jp, jr, body)
    J = np.vstack([jp[:, dofs], jr[:, dofs]])
    err = np.concatenate([np.asarray(target_pos, float) - pos, Rotation.from_matrix(np.asarray(target_rot) @ rot.T).as_rotvec()])
    dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(6), err)
    return np.asarray(q_arm, float) + dq


def main():
    model = load_robot_only_model()
    shapes = RobotShapes(model, ROOT / sfcfg.SHAPES_FILE)
    kin = ArmKinematics(shapes)
    d = mujoco.MjData(model)
    rng = np.random.default_rng(20261119)
    lam = sfcfg.RESOLVER["jacobian_damping"]
    worst = {"shape arrays (m)": 0.0, "hand frame position (m)": 0.0, "hand frame rotation": 0.0, "Jacobian step (rad)": 0.0}
    for _ in range(N_STATES):
        st = random_state(model, shapes, rng)
        q = mujoco_qpos(shapes, st)
        d.qpos[:] = q
        mujoco.mj_kinematics(model, d)
        shapes.set_state(st)
        A, B, C, R = shapes.place_arrays()
        A0, B0, C0, R0 = old_place_arrays(shapes, d)
        for new, old in ((A, A0), (B, B0), (C, C0), (R, R0)):
            if old is not None:
                worst["shape arrays (m)"] = max(worst["shape arrays (m)"], float(np.abs(new - old).max()))
        for s in SIDES:
            b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"openarm_{s}_link7")
            pos, rot = shapes.hand_frame(s)
            worst["hand frame position (m)"] = max(worst["hand frame position (m)"], float(np.abs(pos - d.xpos[b]).max()))
            worst["hand frame rotation"] = max(worst["hand frame rotation"], float(np.abs(rot - d.xmat[b].reshape(3, 3)).max()))
            # the Jacobian step toward a target a few millimetres and a few degrees away from the hand
            tgt_pos = d.xpos[b] + rng.normal(scale=0.01, size=3)
            tgt_rot = (Rotation.from_rotvec(rng.normal(scale=0.05, size=3)) * Rotation.from_matrix(rot)).as_matrix()
            new_q = kin.jacobian_step(st, s, tgt_pos, tgt_rot)
            q_arm = st.q_left if s == "left" else st.q_right
            old_q = old_jacobian_step(model, d, kin.dofs[s], b, q_arm, tgt_pos, tgt_rot, lam)
            # restore the state's kinematics for the next side (the old step ran mj_comPos on d)
            d.qpos[:] = q
            mujoco.mj_kinematics(model, d)
            worst["Jacobian step (rad)"] = max(worst["Jacobian step (rad)"], float(np.abs(new_q - old_q).max()))
    print(f"{N_STATES} random states, both arms; reference = MuJoCo mj_kinematics / mj_comPos / mj_jacBody with the old formulas")
    for k, v in worst.items():
        print(f"  worst {k:26s} {v:.3e}")
    ok = max(worst["shape arrays (m)"], worst["hand frame position (m)"], worst["hand frame rotation"]) < TOL_M and worst["Jacobian step (rad)"] < TOL_RAD
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
