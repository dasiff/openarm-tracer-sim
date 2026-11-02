"""Read-only attribution at the end of IN (bounce branch): rotation contributions per joint from the rotational Jacobian, and the servo gains.

    MUJOCO_GL=egl python experiments/pick_rot_attrib.py <joint_probe.csv>

Uses the joint probe's last IN row before the timeout reset (t <= 86.2 s): the measured joints, the commanded joints, and the recording
checkpoint state. Contribution of joint k to (c minus b) rotation: Jr_k times (measured minus commanded), with Jr_k the pad frame's
rotational Jacobian (world). Checked against the measured rotation between the commanded and measured pad frames.
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
from src.robot_helpers import arm_ids            # noqa: E402
from src.run_record import STATE_SPEC            # noqa: E402
from src.task_beaker_hotplate import load_task   # noqa: E402

probe_csv = sys.argv[1]
rows = [r for r in csv.DictReader(open(probe_csv)) if r["el"] and float(r["el"]) <= 15.2 and float(r["t"]) <= 86.2]
r = rows[-1]
f = lambda k: float(r[k])
sim = load_task(dv.MOVED, cameras=False)
model, data = sim.model, sim.data
ck = (int(float(r["t"]) * 150) // 150) * 150
state = None
for fn in sorted(glob.glob(str(Path(probe_csv).parent / "recording" / "chunks" / "*.npz"))):
    z = np.load(fn)
    if ck in list(z["ck_ticks"]):
        state = z["ck_states"][list(z["ck_ticks"]).index(ck)]
        break
mujoco.mj_setState(model, data, state, STATE_SPEC)
ids = arm_ids(model, "left")
qadr, vadr, act = ids["qadr"], ids["vadr"], ids["act"]
qmeas = np.array([f(f"qmeas_{k}") for k in range(1, 8)])
qcmd = np.array([f(f"qcmd_{k}") for k in range(1, 8)])
data.qpos[qadr] = qmeas
mujoco.mj_forward(model, data)
tcp = model.body("openarm_left_hand_tcp").id
R_pad = data.xmat[tcp].reshape(3, 3)
Rb = np.array([f(f"b_R{k}") for k in range(9)]).reshape(3, 3)
Rc = np.array([f(f"c_R{k}") for k in range(9)]).reshape(3, 3)
meas_rot = np.degrees(np.linalg.norm(Rotation.from_matrix(Rb.T @ Rc).as_rotvec()))
Jr = np.zeros((3, model.nv))                      # the rotational Jacobian of the tool point body (the same at any point of the body)
Jp = np.zeros((3, model.nv))
mujoco.mj_jacBody(model, data, Jp, Jr, tcp)
print(f"row t={r['t']}; measured rotation between commanded and measured pad frames: {meas_rot:.3f} deg")
tot = np.zeros(3)
for k in range(7):
    dq = qmeas[k] - qcmd[k]
    c = np.degrees(Jr[:, vadr[k]] * dq)
    tot += c
    kp = model.actuator_gainprm[act[k], 0]
    print(f" L{k + 1}: meas-cmd {1000 * dq:+.3f} mrad, rotation contribution (deg) x {c[0]:+.3f} y {c[1]:+.3f} z {c[2]:+.3f} (norm {np.linalg.norm(c):.3f}), kp {kp:.1f} N m/rad")
print(f" sum of rotation contributions (deg): {np.round(tot, 3).tolist()}, norm {np.linalg.norm(tot):.3f} deg")
