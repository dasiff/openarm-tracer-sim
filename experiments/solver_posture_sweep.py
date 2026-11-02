"""Offline posture-weight sweep at the pick's IN move (scratch config only; no dynamics). Bounce branch, decisions-log M223 limits.

    MUJOCO_GL=egl python experiments/solver_posture_sweep.py <joint_probe.csv> <out.md>

Same method as experiments/solver_offline_check.py: the whole state from the recording checkpoint at tick 12,900, the arm at the commanded
joints at the end of IN, the waypoint held. Here the posture target is the commanded joints at the START of the IN move (the first IN row),
and the posture weight is set to 0.01, 0.003, 0.001 and 0.0001 in scratch copies of MINK_CFG (MINK_CFG itself is not changed).
The solver is run until it stops moving (max joint change under 1e-9 rad per step) or 5000 steps, then the miss is reported in mm and degrees.
The redundant freedom (7 joints for a 6-dimensional pose) is shown as the change of each joint, and of the elbow joint (L4) in particular.
"""

import csv
import glob
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = "/home/vboxuser/projects/openarm-tracer-sim"
sys.path.insert(0, ROOT)
sys.path.insert(0, ROOT + "/experiments")

import demo_video as dv                          # noqa: E402
from src.mink_arm import MinkArm                 # noqa: E402
from src.run_record import STATE_SPEC            # noqa: E402
from src.task_beaker_hotplate import load_task   # noqa: E402
from src.task_config import MINK_CFG             # noqa: E402

MAX_STEPS = 5000


def pad_from_q(arm, data, q):
    d2 = mujoco.MjData(arm.model)
    d2.qpos[:] = data.qpos
    d2.qpos[arm.qadr] = q
    mujoco.mj_kinematics(arm.model, d2)
    return arm.pad_pose(d2)


def rot_deg(Ra, Rb):
    return float(np.degrees(np.linalg.norm(Rotation.from_matrix(Ra.T @ Rb).as_rotvec())))


def main():
    probe_csv, out_path = sys.argv[1], sys.argv[2]
    rows = list(csv.DictReader(open(probe_csv)))
    in_rows = [r for r in rows if r["el"] and float(r["el"]) <= 15.2]
    first, last = in_rows[0], in_rows[-1]
    q_start = np.array([float(first[f"qcmd_{k}"]) for k in range(1, 8)])     # the posture target: the IN move's start
    q_end = np.array([float(last[f"qcmd_{k}"]) for k in range(1, 8)])        # where the solver run starts: the commanded joints at the end of IN
    a_pos = np.array([float(last[f"a_{c}"]) for c in "xyz"])
    a_R = np.array([float(last[f"a_R{k}"]) for k in range(9)]).reshape(3, 3)

    sim = load_task(dv.MOVED, cameras=False)
    model, data = sim.model, sim.data
    tick_end = int(float(last["t"]) * 150)
    ck_tick = (tick_end // 150) * 150
    state = None
    for f in sorted(glob.glob(str(Path(probe_csv).parent / "recording" / "chunks" / "*.npz"))):
        z = np.load(f)
        if ck_tick in list(z["ck_ticks"]):
            state = z["ck_states"][list(z["ck_ticks"]).index(ck_tick)]
            break
    if state is None:
        sys.exit("no checkpoint at the IN end tick: the sweep needs the recording of the joint probe")
    mujoco.mj_setState(model, data, state, STATE_SPEC)
    mujoco.mj_forward(model, data)

    lines = [f"- start (commanded joints at the end of IN, rad): {np.round(q_end, 4).tolist()}",
             f"- posture target (commanded joints at the start of IN, rad): {np.round(q_start, 4).tolist()}",
             f"- waypoint (mm): {np.round(a_pos * 1000, 2).tolist()}",
             "",
             "| posture weight | steps to stop | miss (mm) | rot miss (deg) | L1 change | L2 | L3 | L4 (elbow) | L5 | L6 | L7 | |change| (rad) |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for w in (0.01, 0.003, 0.001, 0.0001):
        cfg = {**MINK_CFG, "posture_cost": w}
        arm = MinkArm(model, data, "left", q_start, cfg=cfg)          # the posture target is q_start (the constructor's home_q)
        arm.q_cmd = q_end.copy()
        steps = MAX_STEPS
        for it in range(1, MAX_STEPS + 1):
            before = arm.q_cmd.copy()
            arm.step(data, a_pos, a_R)
            if np.max(np.abs(arm.q_cmd - before)) < 1e-9:
                steps = it
                break
        p, R = pad_from_q(arm, data, arm.q_cmd)
        d = arm.q_cmd - q_end
        miss = 1000 * np.linalg.norm(a_pos - p)
        deg = rot_deg(R, a_R)
        lines.append(f"| {w} | {steps if steps < MAX_STEPS else 'not stopped (' + str(MAX_STEPS) + ')'} | {miss:.4f} | {deg:.4f} | "
                     + " | ".join(f"{x:+.4f}" for x in d) + f" | {np.linalg.norm(d):.4f} |")
    text = "\n".join(lines)
    print(text)
    Path(out_path).write_text(text + "\n")


if __name__ == "__main__":
    main()
