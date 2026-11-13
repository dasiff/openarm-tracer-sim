"""Scratch (2026-11-12, nothing in src/ is changed): the pad-to-beaker offset the real run had when its place started, from a run's episode log.
    python experiments/scratch_real_place_offset.py <run folder>
Forward kinematics of the measured left arm and base at the first row of the place phase, minus the logged beaker position (the beaker's bottom centre, as plan_place's b)."""
import csv, math, sys
from pathlib import Path
import numpy as np, mujoco
ROOT = Path("/home/vboxuser/projects/openarm-tracer-sim"); sys.path.insert(0, str(ROOT))
from src.task_beaker_hotplate import load_task
from src.task_config import TASK_CFG
from src.gripper_geometry import gripper_frame
run = Path(sys.argv[1]); m = load_task(TASK_CFG, cameras=False).model; d = mujoco.MjData(m)
rows = list(csv.DictReader(open(run / "episode" / "episode_log.csv")))
tcp = m.body("openarm_left_hand_tcp").id; pad = np.asarray(gripper_frame(m, "left")["pad_center_in_tcp"], float)
def pad_at(r):
    d.qpos[:] = m.qpos0; a = m.joint("base_float").qposadr[0]; y = math.radians(float(r["base_yaw_deg"]))
    d.qpos[a:a + 7] = [float(r["base_x"]), float(r["base_y"]), 0.1527, math.cos(y / 2), 0, 0, math.sin(y / 2)]
    d.qpos[m.joint("pedestal_joint").qposadr[0]] = float(r["pedestal_m"])
    for i in range(1, 8): d.qpos[m.joint(f"openarm_left_joint{i}").qposadr[0]] = float(r[f"left_joint{i}"])
    mujoco.mj_kinematics(m, d); return d.xpos[tcp] + d.xmat[tcp].reshape(3, 3) @ pad
for label, want in (("end of pick IN (grasp, before CLOSE)", "2/4 PICK: IN"), ("end of pick UP (lift)", "2/4 PICK: UP"), ("place start", "4/4 PLACE: starting")):
    idx = [i for i, r in enumerate(rows) if r["phase"].replace(" [running]", "") == want]; r = rows[idx[-1] if want != "4/4 PLACE: starting" else idx[0]]
    p = pad_at(r); b = np.array([float(r["beaker_x"]), float(r["beaker_y"]), float(r["beaker_z"])])
    print(f"{label}: t {r['sim_time']} pad {np.round(p, 4).tolist()} beaker {np.round(b, 4).tolist()} offset pad-beaker {np.round(p - b, 4).tolist()}")
