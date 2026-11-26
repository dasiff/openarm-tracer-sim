"""Compiled arm kinematics (Braking Phase O, SF-103): src/safety_filter/arm_fk.py against MuJoCo's own forward kinematics.

100 random joint states, every joint of the robot-only model (the base's free joint, the pedestal, the wheels, the arm and finger joints),
each within its limits. For each state: body frames (xpos, xmat), body centres of mass (xipos), the anchors and axes of the hinge and
slide joints (xanchor, xaxis) against mj_kinematics, and the Jacobian of each hand (link 7, at its frame origin) against mj_comPos +
mj_jacBody (the arm joint columns, as ArmKinematics.jacobian_step uses them).

PASS: the worst position error (bodies, centres of mass, anchors) is under 1e-4 m, and the worst rotation and axis and Jacobian errors
are under 1e-6. The worst cases are printed.
Run: python experiments/safety_filter_fk_test.py   (exit 0 on pass)
"""
import os
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.safety_filter.arm_fk import RobotKinematics, hand_jacobian  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402

N_STATES = 100
POS_TOL_M = 1e-4          # the brief: compiled positions match MuJoCo FK to 1e-4 m
ANG_TOL = 1e-6            # rotations, axes and Jacobian entries (dimensionless)
FREE = int(mujoco.mjtJoint.mjJNT_FREE)


def random_state(model, rng):
    """qpos with every joint drawn inside its limits (free joint: a random unit quaternion and a position near the origin)."""
    q = np.array(model.qpos0, float)
    for j in range(model.njnt):
        a = int(model.jnt_qposadr[j])
        if int(model.jnt_type[j]) == FREE:
            q[a:a + 3] = rng.uniform(-0.5, 0.5, 3)
            quat = rng.normal(size=4)
            q[a + 3:a + 7] = quat / np.linalg.norm(quat)
        else:
            lo, hi = model.jnt_range[j] if model.jnt_limited[j] else (-np.pi, np.pi)
            q[a] = rng.uniform(lo, hi)
    return q


def main():
    model = load_robot_only_model()
    data = mujoco.MjData(model)
    kin = RobotKinematics(model)
    rng = np.random.default_rng(20261118)
    free = np.array([int(model.jnt_type[j]) == FREE for j in range(model.njnt)])
    hands = {s: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"openarm_{s}_link7") for s in ("left", "right")}
    arms = {s: np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_{s}_joint{k}") for k in range(1, 8)]) for s in ("left", "right")}
    worst = {"body position (m)": 0.0, "body centre of mass (m)": 0.0, "joint anchor (m)": 0.0, "body rotation": 0.0, "joint axis": 0.0,
             "hand Jacobian": 0.0}
    for n in range(N_STATES):
        q = random_state(model, rng)
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        kin.update(q)
        worst["body position (m)"] = max(worst["body position (m)"], float(np.abs(kin.xpos - data.xpos).max()))
        worst["body centre of mass (m)"] = max(worst["body centre of mass (m)"], float(np.abs(kin.xipos - data.xipos).max()))
        worst["body rotation"] = max(worst["body rotation"], float(np.abs(kin.xmat - data.xmat).max()))
        hinge_slide = ~free
        worst["joint anchor (m)"] = max(worst["joint anchor (m)"], float(np.abs(kin.xanchor[hinge_slide] - data.xanchor[hinge_slide]).max()))
        worst["joint axis"] = max(worst["joint axis"], float(np.abs(kin.xaxis[hinge_slide] - data.xaxis[hinge_slide]).max()))
        for s in ("left", "right"):
            b, joints = hands[s], arms[s]
            jp_ref, jr_ref = np.zeros((3, model.nv)), np.zeros((3, model.nv))
            mujoco.mj_jacBody(model, data, jp_ref, jr_ref, b)
            dofs = model.jnt_dofadr[joints]
            ref = np.vstack([jp_ref[:, dofs], jr_ref[:, dofs]])
            jp, jr = np.zeros((3, 7)), np.zeros((3, 7))
            hand_jacobian(kin.xpos[b], kin.xanchor, kin.xaxis, joints.astype(np.int64), jp, jr)
            ours = np.vstack([jp, jr])
            worst["hand Jacobian"] = max(worst["hand Jacobian"], float(np.abs(ours - ref).max()))
    print(f"{N_STATES} random joint states, {model.nbody} bodies, {model.njnt} joints (free base included)")
    for k, v in worst.items():
        print(f"  worst {k:26s} {v:.3e}")
    pos_ok = max(worst["body position (m)"], worst["body centre of mass (m)"], worst["joint anchor (m)"]) < POS_TOL_M
    ang_ok = max(worst["body rotation"], worst["joint axis"], worst["hand Jacobian"]) < ANG_TOL
    print("PASS" if pos_ok and ang_ok else "FAIL")
    return 0 if pos_ok and ang_ok else 1


if __name__ == "__main__":
    sys.exit(main())
