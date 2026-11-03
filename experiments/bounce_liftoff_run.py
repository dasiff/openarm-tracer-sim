"""Lift-off weighing run (branch held-weight-ramp-sag-off; decisions-log M235 to M239): one run of the full plan, or a pick-only run for the sweep.

    MUJOCO_GL=egl python experiments/bounce_liftoff_run.py <out_dir> [--rate N_PER_S] [--thr MM] [--pick-only]

The defaults are the config values as committed. --rate and --thr set PICK_PLACE_CFG for this process only (liftoff_rate_n_per_s, liftoff_rise_mm): the sweep
reports them, and no default changes. --pick-only stops the run when the place's first step begins (after the pick's UP step has finished).

Per tick, for the left arm, it writes <out_dir>/liftoff_ticks.csv (a temporary file, renamed at the end):
  tick, t_sim                          the loop's tick and the simulation time (s)
  pad_x_m, pad_y_m, pad_z_m            measured pad centre (m, world frame)
  mass_kg, target_kg                   held mass the feedforward used (kg) and the ramp target (kg)
  pad_force_N                          the weight term's force, mass times g (N): the force the held-mass feedforward carries (validation)
  held_tau_L1..L7                      the held part of the feedforward (N m) per left joint
  beaker_z_m                           beaker body z (m, world frame) (validation only; never used by control)
  beaker_bench_contacts                contacts between the beaker and the bench geoms this tick (validation only)
  grip_cmd_m                           the gripper command (finger travel, m) the step sent (blank when none)
  grip_applied_m                       the gripper actuator's control value on this tick (m)
  ramp_mode, ramp_rate_kg_s, ramp_start_kg, ramp_target_kg   the ramp state of the side (rate ramp or target ramp)
  weigh_state, weigh_base_m, weigh_rise_m, weigh_hold   the WEIGH step's state (blank outside WEIGH)
  C_p_x..z, C_r_x..z                   the sag correction at the start of the tick (m, rad)
  label, frozen, err_m, ref_z_m, rot_err_deg   active step, freeze flag, pad error to the waypoint, reference z, rotation error (deg, a move only)
  trace_ok                             1 when the trace belongs to this tick
"""

import argparse
import csv
import sys
from argparse import Namespace
from pathlib import Path

import mujoco
import numpy as np

ROOT = "/home/vboxuser/projects/openarm-tracer-sim"
sys.path.insert(0, ROOT)
sys.path.insert(0, ROOT + "/experiments")

import src.episode_runner as er                 # noqa: E402
import src.gravity_ff as gff                    # noqa: E402
import src.held_mass_ramp as hmr                # noqa: E402
import src.mpvi_modules as mpm                  # noqa: E402
from src.task_config import PICK_PLACE_CFG      # noqa: E402
import demo_video as dv                         # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("out")
ap.add_argument("--rate", type=float, default=None, help="liftoff_rate_n_per_s for this process (N/s)")
ap.add_argument("--thr", type=float, default=None, help="liftoff_rise_mm for this process (mm)")
ap.add_argument("--pick-only", action="store_true")
args_cli = ap.parse_args()
if args_cli.rate is not None:
    PICK_PLACE_CFG["liftoff_rate_n_per_s"] = args_cli.rate
if args_cli.thr is not None:
    PICK_PLACE_CFG["liftoff_rise_mm"] = args_cli.thr
print(f"[bounce_liftoff_run] liftoff_rate_n_per_s = {PICK_PLACE_CFG['liftoff_rate_n_per_s']}, liftoff_rise_mm = {PICK_PLACE_CFG['liftoff_rise_mm']}, "
      f"pick_only = {args_cli.pick_only}", flush=True)

G = 9.81
out = Path(args_cli.out)
out.mkdir(parents=True, exist_ok=True)
cur = {"tick": -1, "loop": None, "seen_up": False, "stopped": False}
rows = []
extra = {"bench_ids": None, "beaker_body": None, "grip_act": None}


def _ids(model):
    if extra["bench_ids"] is None:
        extra["bench_ids"] = {g for g in range(model.ngeom) if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g])) or "").startswith("bench")}
        extra["beaker_body"] = model.body("task_beaker").id
        extra["grip_act"] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_gripper_ctrl")
        extra["finger_bodies"] = {model.body("openarm_left_left_finger").id, model.body("openarm_left_right_finger").id}


def _contacts(model, data):
    _ids(model)
    beaker = extra["beaker_body"]
    n = 0
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        b1, b2 = int(model.geom_bodyid[g1]), int(model.geom_bodyid[g2])
        if (b1 == beaker and g2 in extra["bench_ids"]) or (b2 == beaker and g1 in extra["bench_ids"]):
            n += 1
    return n


def _beaker_forces(model, data):
    """Preload logging (read only). Returns (table normal N, pad vertical N, pad contacts): forces on the beaker, world z, positive = up.

    mj_contactForce gives the force in the contact frame (row 0 = normal). The force on the beaker is +frame^T f when the beaker is geom2 and -frame^T f when it is geom1.
    """
    _ids(model)
    beaker = extra["beaker_body"]
    tbl, pad, n_pad = 0.0, 0.0, 0
    f6 = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        b1, b2 = int(model.geom_bodyid[g1]), int(model.geom_bodyid[g2])
        if beaker not in (b1, b2):
            continue
        other_geom, other_body = (g1, b1) if b2 == beaker else (g2, b2)
        mujoco.mj_contactForce(model, data, i, f6)
        f_world = c.frame.reshape(3, 3).T @ f6[:3]
        f_on_beaker_z = (1.0 if b2 == beaker else -1.0) * f_world[2]
        if other_geom in extra["bench_ids"]:
            tbl += f_on_beaker_z
        elif other_body in extra["finger_bodies"]:
            pad += f_on_beaker_z
            n_pad += 1
    return tbl, pad, n_pad


orig_torques = gff.ArmGravityFeedforward.torques


def torques_logged(self, data):
    """Calls the original, then reads the state of the same tick (one call per tick from the loop's feedforward)."""
    tau = orig_torques(self, data)
    if cur["tick"] < 0 or self.sides != ("left", "right"):
        return tau
    m, d = self.model, self.scratch
    tcp, pad = self.pads["left"]
    point = d.xpos[tcp] + d.xmat[tcp].reshape(3, 3) @ pad
    mass = hmr.mass_at(m, "left", data.time)
    rec = hmr.RAMP.get(id(m), {}).get("left")
    jac = np.zeros((3, m.nv))
    mujoco.mj_jac(m, d, jac, None, point, tcp)
    held = mass * G * jac[2]
    _ids(m)
    row = {"tick": cur["tick"], "t_sim": float(data.time), "pad_x_m": float(point[0]), "pad_y_m": float(point[1]), "pad_z_m": float(point[2]),
           "mass_kg": mass, "target_kg": rec["target"] if rec else 0.0, "pad_force_N": mass * G,
           "beaker_z_m": float(data.xpos[extra["beaker_body"]][2]), "beaker_bench_contacts": _contacts(m, data),
           "grip_applied_m": float(data.ctrl[extra["grip_act"]])}
    for k in range(7):
        row[f"held_tau_L{k + 1}"] = float(held[self.dofs[k][0]])
    tbl_n, pad_n, n_pad = _beaker_forces(m, data)
    row["tbl_N"], row["pad_N"], row["pad_contacts"] = tbl_n, pad_n, n_pad
    row["beaker_wt_N"] = float(m.body_mass[extra["beaker_body"]]) * G
    act = np.array([float(data.actuator_force[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"left_joint{k + 1}_ctrl")]) for k in range(7)])
    bias = np.array([float(data.qfrc_bias[self.dofs[k][0]]) for k in range(7)])
    jp_full = np.zeros((3, m.nv))
    mujoco.mj_jac(m, d, jp_full, None, point, tcp)
    jp_arm = np.stack([jp_full[:, self.dofs[k][0]] for k in range(7)], axis=1)
    f_arm = np.linalg.lstsq(jp_arm.T, act - bias, rcond=None)[0]
    row["arm_pad_Fz_N"] = float(f_arm[2])
    for k in range(7):
        row[f"act_L{k + 1}"] = float(act[k])
        row[f"bias_L{k + 1}"] = float(bias[k])
    tr = mpm.TRACE.get("left")
    ok = tr is not None and tr["tick"] == cur["tick"]
    g = (lambda key: tr.get(key)) if ok else (lambda key: None)
    grip = g("grip")
    row["grip_cmd_m"] = float(grip["position"]) if isinstance(grip, dict) and "position" in grip else (float(grip) if isinstance(grip, (int, float)) else "")
    ramp = g("ramp")
    row["ramp_mode"] = "" if ramp is None else ("rate" if ramp.get("rate") is not None else "target")
    row["ramp_rate_kg_s"] = "" if ramp is None or ramp.get("rate") is None else float(ramp["rate"])
    row["ramp_start_kg"] = "" if ramp is None else float(ramp["start"])
    row["ramp_target_kg"] = "" if ramp is None else float(ramp["target"])
    wg = g("weigh") or {}
    row["weigh_state"] = wg.get("state", "")
    row["weigh_base_m"] = wg.get("base", "")
    row["weigh_rise_m"] = wg.get("rise", "")
    row["weigh_hold"] = wg.get("n", "")
    ref = g("ref")
    row["ref_z_m"] = float(ref[2]) if ref is not None else ""
    row["err_m"] = g("err") if g("err") is not None else ""
    row["rot_err_deg"] = g("rot_err_deg") if g("rot_err_deg") is not None else ""
    row["label"] = g("label") or ""
    for ax, k in zip("xyz", range(3)):
        row[f"C_p_{ax}"] = float(tr["C_p"][k]) if ok else ""
        row[f"C_r_{ax}"] = float(tr["C_r"][k]) if ok else ""
    row["frozen"] = int(tr["frozen"]) if ok else ""
    row["trace_ok"] = int(ok)
    rows.append(row)
    return tau


gff.ArmGravityFeedforward.torques = torques_logged

orig_run_loop = er.run_loop


def run_loop_logged(model, data, sources, cfg, viewer=None):
    inner = cfg["on_tick"]

    def on_tick(obs, loop):
        cur["tick"] = int(obs["tick"])
        tr = mpm.TRACE.get("left")
        label = (tr or {}).get("label", "")
        if label == "pick: UP":
            cur["seen_up"] = True
        if args_cli.pick_only and cur["seen_up"] and label.startswith("place") and not cur["stopped"]:
            cur["stopped"] = True
            print(f"[bounce_liftoff_run] pick-only: the place begins at tick {obs['tick']}; the run stops", flush=True)
            loop["stop"]("pick only: UP finished")
        return inner(obs, loop)

    cfg["on_tick"] = on_tick
    return orig_run_loop(model, data, sources, cfg, viewer=viewer)


er.run_loop = run_loop_logged

args = Namespace(record_only=str(out), instruction="put the beaker on the hotplate", hz=30, name="demo_moved", no_bridge=False)
rc = dv.record_only(args)

cols = (["tick", "t_sim", "pad_x_m", "pad_y_m", "pad_z_m", "mass_kg", "target_kg", "pad_force_N"] + [f"held_tau_L{k}" for k in range(1, 8)]
        + ["beaker_z_m", "beaker_bench_contacts", "grip_cmd_m", "grip_applied_m", "ramp_mode", "ramp_rate_kg_s", "ramp_start_kg", "ramp_target_kg",
           "weigh_state", "weigh_base_m", "weigh_rise_m", "weigh_hold", "ref_z_m", "err_m", "rot_err_deg", "label"]
        + [f"C_p_{a}" for a in "xyz"] + [f"C_r_{a}" for a in "xyz"] + ["frozen", "trace_ok"]
        + ["tbl_N", "pad_N", "pad_contacts", "beaker_wt_N", "arm_pad_Fz_N"] + [f"act_L{k}" for k in range(1, 8)] + [f"bias_L{k}" for k in range(1, 8)])
tmp = out / "liftoff_ticks.csv.tmp"
with open(tmp, "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(cols)
    for r in rows:
        w.writerow([r[c] for c in cols])
tmp.replace(out / "liftoff_ticks.csv")
print(f"bounce_liftoff_run done: rc {rc}, ticks logged {len(rows)}", flush=True)
sys.exit(rc)
