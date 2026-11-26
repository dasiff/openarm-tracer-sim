"""Does the task hotplate's position change the solver's output? (SF-120 Part 1 check, 2026-11-23). Read-only: no simulation steps.

Takes the recorded solver inputs at the first tick where the grasp and held runs differ (t = 3.70 s, trim forced to zero in both
recordings, so the target is the source's forward kinematics), rebuilds the left arm's solver on the task scene with the hotplate at
the grasp test's position (x 0.80) and at the held run's position (x 0.65), and compares each solver output with the recorded output
of each run at that tick.

    python experiments/arms_hotplate_check.py --grasp GRASP.npz --held HELD.npz --out DIR
"""

import argparse
import json
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "experiments"))
os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from src.mink_arm import MinkArm  # noqa: E402
from src.robot_helpers import arm_ids  # noqa: E402
from src.task_beaker_hotplate import load_task  # noqa: E402
from src.task_config import MINK_CFG, TASK_CFG  # noqa: E402

HOME_LEFT = np.asarray(TASK_CFG["home_q_left"], float)


def solver_output(model, data, q_src, q_meas, q_cmd_prev):
    """One solver step on the given inputs, the way SagLayer.correct does it with the trim at zero."""
    arm = MinkArm(model, data, "left", HOME_LEFT, MINK_CFG, dt=1.0 / 150.0)
    arm.q_cmd = q_cmd_prev.copy()
    scratch = mujoco.MjData(model)
    scratch.qpos[:] = data.qpos
    scratch.qpos[arm.qadr] = q_src
    mujoco.mj_kinematics(model, scratch)
    p_ref, R_ref = arm.pad_pose(scratch)
    data.qpos[arm.qadr] = q_meas
    mujoco.mj_forward(model, data)
    return arm.step(data, p_ref, R_ref), arm


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--grasp", required=True)
    ap.add_argument("--held", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--t", type=float, default=3.70)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    g = np.load(a.grasp)
    h = np.load(a.held)
    tg = g["left_t"]
    k = int(np.argmin(np.abs(tg - a.t)))
    kh = int(np.argmin(np.abs(h["left_t"] - a.t)))
    q_src_g, q_meas_g, q_cmd_prev = g["left_q_src"][k], g["left_q_meas"][k], g["left_q_sent"][k - 1]
    q_src_h, q_meas_h = h["left_q_src"][kh], h["left_q_meas"][kh]
    res = {"t": round(float(tg[k]), 4),
           "inputs_identical_src": bool(np.array_equal(q_src_g, q_src_h)),
           "inputs_identical_meas": bool(np.array_equal(q_meas_g, q_meas_h)),
           "inputs_identical_q_cmd_prev": bool(np.array_equal(g["left_q_sent"][k - 1], h["left_q_sent"][kh - 1])),
           "recorded_grasp_sent": np.round(g["left_q_sent"][k], 6).tolist(),
           "recorded_held_sent": np.round(h["left_q_sent"][kh], 6).tolist()}

    sim = load_task(cameras=False)
    model, data = sim.model, sim.data
    hp = model.body("task_hotplate").id
    x_default = float(model.body_pos[hp][0])
    res["hotplate_x_default_m"] = round(x_default, 4)

    out_default, arm_default = solver_output(model, data, q_src_g, q_meas_g, q_cmd_prev)
    model.body_pos[hp][0] = 0.65                       # the held run's hotplate position (measured: x 0.65)
    mujoco.mj_forward(model, data)
    out_held_x, arm_held = solver_output(model, data, q_src_g, q_meas_g, q_cmd_prev)
    model.body_pos[hp][0] = x_default

    res["solver_hotplate_default"] = np.round(out_default, 6).tolist()
    res["solver_hotplate_0p65"] = np.round(out_held_x, 6).tolist()
    res["max_abs_diff_between_hotplate_positions"] = float(np.max(np.abs(out_default - out_held_x)))
    res["max_abs_diff_default_vs_recorded_grasp"] = float(np.max(np.abs(out_default - g["left_q_sent"][k])))
    res["max_abs_diff_0p65_vs_recorded_held"] = float(np.max(np.abs(out_held_x - h["left_q_sent"][kh])))
    res["max_abs_diff_default_vs_recorded_held"] = float(np.max(np.abs(out_default - h["left_q_sent"][kh])))
    with open(out / "arms_hotplate_check.json", "w") as f:
        json.dump(res, f, indent=1)
    print(json.dumps(res, indent=1), flush=True)


if __name__ == "__main__":
    main()
