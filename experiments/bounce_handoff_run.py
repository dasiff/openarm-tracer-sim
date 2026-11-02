"""Trim handoff experiment, part 2: one bounce run with per-tick logging (branch held-weight-ramp-sag-off; decisions-log M230 to M232).

    MUJOCO_GL=egl python experiments/bounce_handoff_run.py <out_dir>

The build as committed; the config as committed (sag_reset_at_skill_start False). The scenario is the one of experiments/bounce_ramp_run.py.
The launcher wraps the gravity feedforward once per tick and reads the trace that src/mpvi_modules.py keeps for the arm side (logging only).

Per tick, for the left arm, it writes <out_dir>/bounce_ticks.csv (temporary file, then renamed):
  tick, t_sim, pad_x_m, pad_y_m, pad_z_m   measured pad centre (m, world frame)
  mass_kg, target_kg                        the held mass the feedforward used, and the ramp target (kg)
  held_tau_L1..L7                           held part of the feedforward (N m) per left joint
  ref_x_m..ref_z_m                          reference pad centre of the move before the trim is added (m); blank when the step is not a move
  err_m                                     norm of measured minus waypoint (m)
  rot_err_deg                               rotation error of the pad to the reference (deg, a move only)
  label                                     active skill and step (for example "pick: IN")
  C_p_x..z, C_r_x..z                        sag correction at the start of the tick (m, rotation vector rad)
  H_ref_p_x..z, H_ref_r_x..z                saved reference (ref, M231); blank when none
  H_full_p_x..z, H_full_r_x..z              latched full value (full, M231); blank when none
  dir                                       ramp direction (+1 ramp-in, -1 ramp-out, 0 none)
  prog                                      ramp progress (minimum jerk, 0 to 1; 1 when no ramp)
  frozen                                    1 when the load change is frozen (M226), else 0
  trace_ok                                  1 when the trace belongs to this tick
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
import demo_video as dv                         # noqa: E402

G = 9.81
out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)
cur = {"tick": -1}
rows = []
trace_miss = {"n": 0}

orig_torques = gff.ArmGravityFeedforward.torques


def _xyz(v, k):
    return "" if v is None else float(v[k])


def torques_logged(self, data):
    """Calls the original, then reads the left pad, the held part and the trace of the same tick."""
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
    row = {"tick": cur["tick"], "t_sim": float(data.time), "pad_x_m": float(point[0]), "pad_y_m": float(point[1]), "pad_z_m": float(point[2]),
           "mass_kg": mass, "target_kg": rec["target"] if rec else 0.0}
    for k in range(7):
        row[f"held_tau_L{k + 1}"] = float(held[self.dofs[k][0]])
    tr = mpm.TRACE.get("left")
    ok = tr is not None and tr["tick"] == cur["tick"]
    if not ok:
        trace_miss["n"] += 1
    g = (lambda key: tr[key]) if ok else (lambda key: None)
    ref = g("ref")
    row["ref_x_m"], row["ref_y_m"], row["ref_z_m"] = ((float(ref[0]), float(ref[1]), float(ref[2])) if ref is not None else ("", "", ""))
    row["err_m"] = g("err") if g("err") is not None else ""
    row["rot_err_deg"] = g("rot_err_deg") if g("rot_err_deg") is not None else ""
    row["label"] = g("label") or ""
    for ax, k in zip("xyz", range(3)):
        row[f"C_p_{ax}"] = float(tr["C_p"][k]) if ok else ""
        row[f"C_r_{ax}"] = float(tr["C_r"][k]) if ok else ""
    for name, key in (("H_ref", "H_ref"), ("H_full", "H_full")):
        pair = g(key)
        for ax, k in zip("xyz", range(3)):
            row[f"{name}_p_{ax}"] = _xyz(pair[0], k) if pair is not None else ""
            row[f"{name}_r_{ax}"] = _xyz(pair[1], k) if pair is not None else ""
    row["dir"] = g("dir") if g("dir") is not None else ""
    row["prog"] = g("prog") if g("prog") is not None else ""
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

cols = (["tick", "t_sim", "pad_x_m", "pad_y_m", "pad_z_m", "mass_kg", "target_kg"] + [f"held_tau_L{k}" for k in range(1, 8)]
        + ["ref_x_m", "ref_y_m", "ref_z_m", "err_m", "rot_err_deg", "label"]
        + [f"C_p_{a}" for a in "xyz"] + [f"C_r_{a}" for a in "xyz"]
        + [f"H_ref_p_{a}" for a in "xyz"] + [f"H_ref_r_{a}" for a in "xyz"]
        + [f"H_full_p_{a}" for a in "xyz"] + [f"H_full_r_{a}" for a in "xyz"]
        + ["dir", "prog", "frozen", "trace_ok"])
tmp = out / "bounce_ticks.csv.tmp"
with open(tmp, "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(cols)
    for r in rows:
        w.writerow([r[c] for c in cols])
tmp.replace(out / "bounce_ticks.csv")
print(f"bounce_handoff_run done: rc {rc}, ticks logged {len(rows)}, ticks without a trace {trace_miss['n']}", flush=True)
sys.exit(rc)
