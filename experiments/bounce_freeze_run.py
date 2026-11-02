"""Freeze experiment, part 2: one bounce run with per-tick logging (branch held-weight-ramp-sag-off; decisions-log M225 to M228).

    MUJOCO_GL=egl python experiments/bounce_freeze_run.py <out_dir> <A|B>

Run A: the build as committed (sag_reset_at_skill_start False, the config value).
Run B: the same, with PICK_PLACE_CFG["sag_reset_at_skill_start"] set to True in this process only (the config key, no code change).

The scenario is the one of experiments/bounce_ramp_run.py. Nothing in the simulation is changed: the launcher wraps the gravity feedforward once per tick and
reads the state it already computes, and it reads the trace that src/mpvi_modules.py keeps for the arm side (logging only).

Per tick, for the left arm, it writes <out_dir>/bounce_ticks.csv (written to a temporary file and renamed):
  tick, t_sim             the loop's tick and the simulation time (s)
  pad_z_m                 measured height of the left pad centre (m, world frame)
  mass_kg, target_kg      the held mass the feedforward used, and the ramp's target (kg)
  held_tau_L1..L7         the held part of the feedforward (N m) per left arm joint
  ref_x_m, ref_y_m, ref_z_m   the reference pad centre of the current move BEFORE the sag trim is added (m); blank when the step is not a move
  err_m                   norm of the measured pad centre minus the move's waypoint (m); blank when not a move
  label                   the active skill and step, for example "PICK: IN" (blank when none)
  C_p_x..C_p_z, C_r_x..C_r_z   the sag correction at the start of the tick (m, and rotation vector in rad)
  frozen                  1 when the sag correction is frozen on this tick (decisions-log M226), else 0
  trace_ok                1 when the trace belongs to this tick (0: the step did not run on this tick, so the trace columns are blank)
"""

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

G = 9.81
out = Path(sys.argv[1])
mode = sys.argv[2]
if mode not in ("A", "B"):
    sys.exit("usage: bounce_freeze_run.py <out_dir> <A|B>")
PICK_PLACE_CFG["sag_reset_at_skill_start"] = (mode == "B")
print(f"[bounce_freeze_run] run {mode}: sag_reset_at_skill_start = {PICK_PLACE_CFG['sag_reset_at_skill_start']}", flush=True)
out.mkdir(parents=True, exist_ok=True)
cur = {"tick": -1}
rows = []
trace_miss = {"n": 0}

orig_torques = gff.ArmGravityFeedforward.torques


def torques_logged(self, data):
    """Calls the original, then reads the left pad, the held part and the trace of the same tick (one call per tick from the loop's feedforward)."""
    tau = orig_torques(self, data)
    if cur["tick"] < 0 or self.sides != ("left", "right"):
        return tau
    m, d = self.model, self.scratch                  # the scratch copy holds this tick's kinematics (set by the call above)
    tcp, pad = self.pads["left"]
    point = d.xpos[tcp] + d.xmat[tcp].reshape(3, 3) @ pad
    mass = hmr.mass_at(m, "left", data.time)
    rec = hmr.RAMP.get(id(m), {}).get("left")
    jac = np.zeros((3, m.nv))
    mujoco.mj_jac(m, d, jac, None, point, tcp)
    held = mass * G * jac[2]                         # the held part per degree of freedom, same as torques()
    row = {"tick": cur["tick"], "t_sim": float(data.time), "pad_z_m": float(point[2]), "mass_kg": mass,
           "target_kg": rec["target"] if rec else 0.0}
    for k in range(7):
        row[f"held_tau_L{k + 1}"] = float(held[self.dofs[k][0]])
    tr = mpm.TRACE.get("left")
    ok = tr is not None and tr["tick"] == cur["tick"]
    if not ok:
        trace_miss["n"] += 1
    ref = tr["ref"] if ok else None
    row["ref_x_m"], row["ref_y_m"], row["ref_z_m"] = ((float(ref[0]), float(ref[1]), float(ref[2])) if ref is not None else ("", "", ""))
    row["err_m"] = (tr["err"] if ok and tr["err"] is not None else "")
    row["label"] = tr["label"] if ok else ""
    for k, ax in enumerate("xyz"):
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
        return inner(obs, loop)

    cfg["on_tick"] = on_tick
    return orig_run_loop(model, data, sources, cfg, viewer=viewer)


er.run_loop = run_loop_logged

args = Namespace(record_only=str(out), instruction="put the beaker on the hotplate", hz=30, name="demo_moved", no_bridge=False)
rc = dv.record_only(args)

cols = (["tick", "t_sim", "pad_z_m", "mass_kg", "target_kg"] + [f"held_tau_L{k}" for k in range(1, 8)]
        + ["ref_x_m", "ref_y_m", "ref_z_m", "err_m", "label"] + [f"C_p_{a}" for a in "xyz"] + [f"C_r_{a}" for a in "xyz"] + ["frozen", "trace_ok"])
tmp = out / "bounce_ticks.csv.tmp"
with open(tmp, "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(cols)
    for r in rows:
        w.writerow([r[c] for c in cols])
tmp.replace(out / "bounce_ticks.csv")
print(f"bounce_freeze_run done: run {mode}, rc {rc}, ticks logged {len(rows)}, ticks without a trace {trace_miss['n']}", flush=True)
sys.exit(rc)
