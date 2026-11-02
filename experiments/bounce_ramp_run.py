"""Bounce experiment run with the held-mass ramp (branch held-weight-ramp-sag-off, decisions-log M215 to M223).

    MUJOCO_GL=egl python experiments/bounce_ramp_run.py <out_dir>

The same scenario as the other bounce runs (experiments/demo_video.py record_only, held-object compensation on, the same instruction). Nothing
in the simulation is changed: the launcher wraps the gravity feedforward once per tick and reads the state it already computes.

Per tick, for the left arm (the one that holds the beaker in this scenario), it writes <out_dir>/bounce_ticks.csv:
  tick, t_sim        the loop's tick number and the simulation time (s)
  pad_z_m            height of the left pad centre (m, world frame): the "hand height" of the metrics
  mass_kg            the held mass the feedforward used on this tick (from the ramp, M217)
  target_kg          the ramp's target on this tick (0 when no ramp has been set)
  held_tau_L1..L7    the held part of the feedforward (N m) on each left arm joint: m g J_z of the pad centre, straight down

The recording, episode log and summary are written by demo_video.record_only into <out_dir> as usual.
Read-only: no change to the simulation's state, its inputs, or its random draws.
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
import demo_video as dv                         # noqa: E402

G = 9.81
out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)
cur = {"tick": -1}
rows = []

orig_torques = gff.ArmGravityFeedforward.torques


def torques_logged(self, data):
    """Calls the original, then reads the left pad and the held part of the same tick (one call per tick from the loop's feedforward)."""
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

cols = ["tick", "t_sim", "pad_z_m", "mass_kg", "target_kg"] + [f"held_tau_L{k}" for k in range(1, 8)]
tmp = out / "bounce_ticks.csv.tmp"
with open(tmp, "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(cols)
    for r in rows:
        w.writerow([r[c] for c in cols])
tmp.replace(out / "bounce_ticks.csv")
print("bounce run done: rc", rc, "ticks logged", len(rows), flush=True)
sys.exit(rc)
