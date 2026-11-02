"""Static check of the gravity model at the pick's IN waypoint (bounce branch, decisions-log M185, M223). Short; no episode.

    MUJOCO_GL=egl python experiments/gravity_static_check.py <probe_joints.json> <out.md>

For each IN attempt in the probe (joints at the end of the move, from experiments/pick_error_probe.py), with the hand empty and sag off:
the left arm is set to those joints, its position targets are held at those joints, the simulation settles for 3 s, and per left arm joint:
  (a) servo_Nm   the torque the position servo applies at rest (MuJoCo actuator_force)
  (b) model_Nm   the gravity model's holding torque (src/gravity_ff.py torques(), the same class the loop uses, both arms)
  (c) diff_Nm    (a) minus (b)
  (d) diff_over_kp   (c) divided by the joint's kp: the angle (rad) the model and the servo disagree by
Also the joint's drop: settled joint minus the held target (rad), to compare with (d).
Read-only: the state is set once at the start and then only stepped (no controller, no feedforward applied to the targets).
"""

import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = "/home/vboxuser/projects/openarm-tracer-sim"
sys.path.insert(0, ROOT)
sys.path.insert(0, ROOT + "/experiments")

import demo_video as dv                          # noqa: E402  (MOVED: the task config the runs use)
from src.task_beaker_hotplate import load_task   # noqa: E402
from src.gravity_ff import ArmGravityFeedforward  # noqa: E402

SETTLE_S = 10.0


def main():
    joints_path, out_path = sys.argv[1], sys.argv[2]
    probe = json.loads(Path(joints_path).read_text())
    sim = load_task(dv.MOVED, cameras=False)
    model, data = sim.model, sim.data
    arm_act = {s: [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{s}_joint{i}_ctrl") for i in range(1, 8)] for s in ("left", "right")}
    fing_act = {s: [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{s}_gripper_ctrl")] for s in ("left", "right")}
    gff = ArmGravityFeedforward(model, arm_act, fing_act, ("left", "right"))
    left_q = [model.jnt_qposadr[model.actuator_trnid[a, 0]] for a in arm_act["left"]]
    left_dof = [int(model.jnt_dofadr[model.actuator_trnid[a, 0]]) for a in arm_act["left"]]
    kp = model.actuator_gainprm[arm_act["left"], 0].copy()
    lines = ["| attempt | joint | servo_Nm | model_Nm | diff_Nm | diff_over_kp_rad | settled minus target (rad) | max joint speed at end (rad/s) |", "|---|---|---|---|---|---|---|---|"]
    q_init = data.qpos.copy()                                      # the task's own start state (objects, pedestal, arms), kept for each attempt
    finger_open = dv.MOVED.get("finger_open", 0.044)               # the open finger position the pick uses (src/mpvi_modules.py)
    for rec in probe:
        q0 = np.array(rec["q_rad"], float)                         # the probe's measured left arm joints at the end of IN
        data.qpos[:] = q_init
        data.qvel[:] = 0.0
        for k, qa in enumerate(left_q):
            data.qpos[qa] = q0[k]
        mujoco.mj_forward(model, data)
        for k, a in enumerate(arm_act["left"]):
            data.ctrl[a] = q0[k]                                   # hold the target at the start pose
        for a in fing_act["left"] + fing_act["right"]:
            data.ctrl[a] = finger_open                             # the hand is empty, the finger is open
        steps = int(round(SETTLE_S / model.opt.timestep))
        for _ in range(steps):
            mujoco.mj_step(model, data)
        settled = np.array([data.qpos[qa] for qa in left_q])
        speed = float(np.max(np.abs(data.qvel[left_dof])))         # joint speed at the end of the settle (rad/s): near 0 means at rest
        tau_model = gff.torques(data)[:7]                          # the gravity model at the settled pose, left arm joints 1 to 7
        servo = np.array([data.actuator_force[a] for a in arm_act["left"]])
        diff = servo - tau_model
        over_kp = diff / kp
        tcp, pad_c = gff.pads["left"]
        pad_pt = data.xpos[tcp] + data.xmat[tcp].reshape(3, 3) @ pad_c
        jacp = np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, jacp, None, pad_pt, tcp)
        dq = over_kp                                             # the joint residual left by the model (rad), per left arm joint
        dp = jacp[:, left_dof] @ dq                              # the pad centre displacement it implies (m)
        lines.append(f"| {rec['attempt']} | pad | | | | | implied pad displacement from the residual (mm): x {1000 * dp[0]:+.2f}, y {1000 * dp[1]:+.2f}, z {1000 * dp[2]:+.2f}, norm {1000 * np.linalg.norm(dp):.2f} | |")
        for k in range(7):
            lines.append(f"| {rec['attempt']} | L{k + 1} | {servo[k]:+.3f} | {tau_model[k]:+.3f} | {diff[k]:+.4f} | {over_kp[k]:+.5f} | {settled[k] - q0[k]:+.5f} | {speed:.2e} |")
    text = "\n".join(lines)
    print(text)
    Path(out_path).write_text(text + "\n")


if __name__ == "__main__":
    main()
