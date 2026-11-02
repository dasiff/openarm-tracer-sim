"""Read-only stage probe of the pick's IN move, sag gain 0 (bounce branch; decisions-log M215, M223).

    MUJOCO_GL=egl python experiments/pick_stage_probe.py <out_dir>

Same scenario as experiments/bounce_ramp_run.py. Changes nothing: it wraps MinkArm.step and reads the state the solver already has.
For each tick of the first IN move (attempt 1) it writes <out_dir>/stage_probe.csv with:
  t, el                       time (s) and time since the IN move's start
  a_x..a_z (m), a_R (9)       the waypoint target pad pose (the move's target and rotation: the pose the move is heading to)
  ref_x..ref_z (m)            the solver's reference pad position this tick (the move's formula, no sag)
  b_x..b_z (m), b_R (9)       the pad pose from the COMMANDED joints (the last joint command, forward kinematics, same pad function as the measurement)
  c_x..c_z (m), c_R (9)       the measured pad pose (MinkArm.pad_pose)
  qcmd_1..7, qmeas_1..7 (rad) commanded and measured joints of the left arm
  va_1..7 (rad/s)             the joint velocity the solver chose on the last solve
  frame_err_0..5              the frame task's error vector at the solve (3 position m, 3 orientation rad), mink's compute_error
  post_err_1..7 (rad)         the posture task's error vector at the solve (joints 1 to 7)
  vel_bind                    number of joints at the velocity limit (|va| >= max_joint_speed)
  jl_slack_min (rad)          smallest distance from a joint to its limit with the margin (negative = outside)
  coll_min_dist (m)           smallest distance over the collision pairs of the left arm's collision limits (from mujoco.mj_geomDistance)
  coll_n                      number of collision pairs in the arm's collision limits
Also <out_dir>/solver_config.json: the solver settings used (weights, damping, solver name, limits).
"""

import csv
import gc
import json
import sys
import types
from argparse import Namespace
from pathlib import Path

import mink
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = "/home/vboxuser/projects/openarm-tracer-sim"
sys.path.insert(0, ROOT)
sys.path.insert(0, ROOT + "/experiments")

import src.episode_runner as er                 # noqa: E402
import src.mink_arm as ma                       # noqa: E402
from src.rate_limit import min_jerk             # noqa: E402
from src.task_config import MINK_CFG            # noqa: E402
import demo_video as dv                         # noqa: E402

out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)
cur = {"data": None, "tick": -1}
step_fns = []
tracked = {}
LAST = {}          # the last solve of the left arm: the frame and posture residuals, the joint velocity, the limit state
rows = []
ARM_DOF = 7

orig_step = ma.MinkArm.step


def step_logged(self, data, position, rotation):
    """Calls the original step; keeps what the solve used (the configuration, the frame and posture residuals, the velocity it chose)."""
    if self.side != "left":
        return orig_step(self, data, position, rotation)
    q_before = self.q_cmd.copy()
    out_q = orig_step(self, data, position, rotation)
    va = (self.q_cmd - q_before) / self.dt
    LAST.update(
        frame_err=np.asarray(self.frame.compute_error(self.config), float),
        post_err=np.asarray(self.posture.compute_error(self.config), float)[self.vadr] if self.posture is not None else np.zeros(7),
        va=np.asarray(va, float),
        vel_bind=int(np.sum(np.abs(va) >= self.cfg["max_joint_speed"] * (1 - 1e-6))),
        arm=self,
    )
    return out_q


ma.MinkArm.step = step_logged


def closure(fn):
    d = {}
    for name, cell in zip(fn.__code__.co_freevars, fn.__closure__ or ()):
        try:
            d[name] = cell.cell_contents
        except ValueError:
            pass
    return d


def find_step_fns():
    if step_fns:
        return
    for o in gc.get_objects():
        if isinstance(o, types.FunctionType) and o.__code__.co_name == "step_inner" and "mpvi_modules" in o.__code__.co_filename:
            step_fns.append(o)


def reference(st, t0, t):
    el = t - t0
    u = min(el / st["T"], 1.0)
    s = min_jerk(u)
    if "v0" in st:
        p = st["p0"] + st["v0"] * st["T"] * (u - 2 * u * u + u ** 3) + (st["target"] - st["p0"]) * (3 * u * u - 2 * u ** 3)
    else:
        p = st["p0"] + (st["target"] - st["p0"]) * s
    return p


def pad_from_q(arm, data, q):
    d2 = mujoco.MjData(arm.model)
    d2.qpos[:] = data.qpos
    d2.qpos[arm.qadr] = q
    mujoco.mj_kinematics(arm.model, d2)
    return arm.pad_pose(d2)


def on_tick(obs, loop):
    data = cur["data"]
    t = float(data.time)
    find_step_fns()
    for fn in step_fns:
        c = closure(fn)
        S, C, arm = c.get("S"), c.get("C"), c.get("arm")
        if not S or not S.get("started") or not S.get("steps") or arm is None or arm.side != "left":
            continue
        i = S["i"]
        if i >= len(S["steps"]):
            continue
        st = S["steps"][i]
        if not (st["name"] == "IN" and st["kind"] == "move" and "p0" in st and S.get("t0") is not None):
            continue
        if "A" not in tracked and len(tracked) == 0:
            tracked["A"] = {"st": st, "t0": S["t0"], "arm": arm, "rows": 0}
        if "A" in tracked and tracked["A"]["st"] is st:
            tracked["A"]["t0"] = S["t0"]
            break
    if "A" not in tracked:
        return
    rec = tracked["A"]
    st, arm = rec["st"], rec["arm"]
    p_ref = reference(st, rec["t0"], t)
    a_pos, a_R = np.asarray(st["target"], float), np.asarray(st["R"], float)
    b_pos, b_R = pad_from_q(arm, data, arm.q_cmd)
    c_pos, c_R = arm.pad_pose(data)
    q_meas = data.qpos[arm.qadr].copy()
    slack = float(np.min(np.minimum(arm.q_cmd - (arm.limits[0].lower[arm.qadr]), arm.limits[0].upper[arm.qadr] - arm.q_cmd)))
    dists = []
    for lim in arm.limits:
        if isinstance(lim, mink.CollisionAvoidanceLimit):
            for g1, g2 in lim.geom_id_pairs:
                dists.append(mujoco.mj_geomDistance(arm.model, data, int(g1), int(g2), 0.2, np.zeros(6)))
    row = {"t": t, "el": t - rec["t0"],
           "a_x": a_pos[0], "a_y": a_pos[1], "a_z": a_pos[2], "ref_x": p_ref[0], "ref_y": p_ref[1], "ref_z": p_ref[2],
           "b_x": b_pos[0], "b_y": b_pos[1], "b_z": b_pos[2], "c_x": c_pos[0], "c_y": c_pos[1], "c_z": c_pos[2],
           "jl_slack_min": slack, "coll_min_dist": min(dists) if dists else float("nan"), "coll_n": len(dists)}
    for k in range(9):
        row[f"a_R{k}"] = a_R.reshape(-1)[k]
        row[f"b_R{k}"] = b_R.reshape(-1)[k]
        row[f"c_R{k}"] = c_R.reshape(-1)[k]
    for k in range(7):
        row[f"qcmd_{k + 1}"] = arm.q_cmd[k]
        row[f"qmeas_{k + 1}"] = q_meas[k]
    if LAST.get("arm") is arm:
        for k in range(7):
            row[f"va_{k + 1}"] = LAST["va"][k]
            row[f"post_err_{k + 1}"] = LAST["post_err"][k]
        for k in range(6):
            row[f"frame_err_{k}"] = LAST["frame_err"][k]
        row["vel_bind"] = LAST["vel_bind"]
    rows.append(row)


orig_run_loop = er.run_loop


def run_loop_probed(model, data, sources, cfg, viewer=None):
    cur["data"] = data
    inner = cfg["on_tick"]

    def wrapped(obs, loop):
        on_tick(obs, loop)
        return inner(obs, loop)

    cfg["on_tick"] = wrapped
    return orig_run_loop(model, data, sources, cfg, viewer=viewer)


er.run_loop = run_loop_probed

settings = {"solver": MINK_CFG["solver"], "damping": MINK_CFG["damping"], "position_cost": MINK_CFG["position_cost"],
            "orientation_cost": MINK_CFG["orientation_cost"], "posture_cost": MINK_CFG["posture_cost"],
            "joint_margin_rad": MINK_CFG["joint_margin_rad"], "max_joint_speed": MINK_CFG["max_joint_speed"],
            "collision_gain": MINK_CFG["collision_gain"], "min_distance": MINK_CFG["min_distance"], "dt_s": 1.0 / 150.0,
            "solver_tolerance": "not set (no tol argument passed to mink.solve_ik; qpsolvers daqp default)"}
(out / "solver_config.json").write_text(json.dumps(settings, indent=1, default=str))

args = Namespace(record_only=str(out), instruction="put the beaker on the hotplate", hz=30, name="demo_moved", no_bridge=False)
rc = dv.record_only(args)

cols = list(rows[0].keys()) if rows else []
with open(out / "stage_probe.csv.tmp", "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=cols)
    w.writeheader()
    for r in rows:
        w.writerow(r)
(out / "stage_probe.csv.tmp").replace(out / "stage_probe.csv")
print("stage probe done: rc", rc, "rows", len(rows), flush=True)
sys.exit(rc)
