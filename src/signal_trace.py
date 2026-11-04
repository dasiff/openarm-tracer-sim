"""DIAGNOSTIC LOGGING ONLY: raw signals for the holding and stillness work (stage 1, decisions M246 and its addenda).

Turned on by the environment variable OPENARM_SIGNAL_TRACE=1 (off by default). Then one row is written at every control tick to
<out_dir>/signal_trace.csv. The module only READS simulation values and writes them to that file. It writes nothing back into the
simulation, calls no function that changes state, and changes no command, setting or number. Since stage 1b it also computes
LOG-ONLY holding and still verdicts, and hand speed and acceleration, from this tick and past ticks only (so it could run live).
Nothing reads these verdicts: no control code uses them. Their thresholds are the PROVISIONAL values in HOLD_STILL_CFG (src/task_config.py).

Units: metres (m), radians (rad), radians per second (rad/s), newtons (N), kilograms (kg), seconds (s).
Terms: "finger1" and "finger2" are the two finger joint positions (qpos); 0 is shut and about 0.044 is open. "actuator force" is what
MuJoCo's position actuator for the gripper is pushing with, signed, from the simulated servo model. "grip force cmd" is the
commanded force cap per finger (the controller's command, not a measurement). "held mass" is the held-mass value the feedforward
uses at this tick, recomputed here with the same formula as src/held_mass_ramp.mass_at, without writing its record.
Names starting with sim_only_ are SIMULATOR-ONLY diagnostics: a real robot may not have them, and the holding rule must not use them.
"""

import csv
import os
from pathlib import Path

import mujoco
import numpy as np

from src import held_mass_ramp as hmr
from src.gripper_geometry import gripper_frame
from src.rate_limit import min_jerk
from src.robot_helpers import arm_ids
from src.task_config import HOLD_STILL_CFG

ENV = "OPENARM_SIGNAL_TRACE"
SIDES = ("left", "right")
FILE_NAME = "signal_trace.csv"


def enabled():
    """True only when the environment variable OPENARM_SIGNAL_TRACE is exactly "1"."""
    return os.environ.get(ENV) == "1"


def columns():
    cols = ["tick", "sim_time_s", "step_label"]
    for s in SIDES:
        cols += [f"{s}_finger1_m", f"{s}_finger2_m", f"{s}_gripper_actuator_force_N", f"{s}_grip_force_cmd_N", f"{s}_gripper_target_m",
                 f"{s}_pad_x_m", f"{s}_pad_y_m", f"{s}_pad_z_m"]
        cols += [f"{s}_q{k}_rad" for k in range(1, 8)] + [f"{s}_qd{k}_rad_s" for k in range(1, 8)]
        cols += [f"{s}_held_mass_kg"]
        # stage 1b, log only: the raw condition flags, the holding verdict, hand speed and acceleration, and the still verdict
        cols += [f"{s}_squeeze_ok", f"{s}_closure_in_range", f"{s}_holding_verdict", f"{s}_hand_speed_m_s", f"{s}_hand_accel_m_s2",
                 f"{s}_speed_below_limit", f"{s}_still_verdict"]
    for s in SIDES:
        cols += [f"sim_only_{s}_pad_contact_z_N"]
    cols += ["sim_only_beaker_bench_contacts"]
    return cols


def _held_mass(model, side, t):
    """The same formula as held_mass_ramp.mass_at, read-only (no write to the record's `now`)."""
    rec = hmr._record(model, side)
    if rec is None:
        return 0.0
    if rec.get("rate") is not None:
        return float(rec["start"] + rec["rate"] * (t - rec["t0"]))
    u = (t - rec["t0"]) / rec["dur"]
    return float(rec["start"] + (rec["target"] - rec["start"]) * min_jerk(u))


class SignalTrace:
    """Opens <out_dir>/signal_trace.csv and writes one row per control tick. Diagnostic logging only."""

    def __init__(self, out_dir, model):
        self.model = model
        self.arm = {s: arm_ids(model, s) for s in SIDES}
        self.tcp = {s: int(model.body(f"openarm_{s}_hand_tcp").id) for s in SIDES}
        self.pad = {s: np.asarray(gripper_frame(model, s)["pad_center_in_tcp"], float) for s in SIDES}
        self.beaker = int(model.body("task_beaker").id)
        self.finger_side = {int(model.body(f"openarm_{s}_{k}_finger").id): s for s in SIDES for k in ("left", "right")}
        self.bench = {g for g in range(model.ngeom)
                      if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g])) or "").startswith("bench")}
        self.f6 = np.zeros(6)
        # stage 1b state for the log-only verdicts: values from earlier ticks only
        self.prev_t = None
        self.prev_pad = {s: None for s in SIDES}
        self.prev_speed = {s: None for s in SIDES}
        self.hold_count = {s: 0 for s in SIDES}
        self.holding = {s: False for s in SIDES}
        self.still_count = {s: 0 for s in SIDES}
        self.file = open(Path(out_dir) / FILE_NAME, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(columns())

    def _pad_contacts(self, data):
        """SIM ONLY. Vertical contact force (N, up positive) on the beaker from each hand's finger bodies, and the count of beaker-bench contacts."""
        z = {s: 0.0 for s in SIDES}
        bench = 0
        m = self.model
        for i in range(data.ncon):
            c = data.contact[i]
            b1, b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
            if self.beaker not in (b1, b2):
                continue
            other_geom, other_body = (c.geom1, b1) if b2 == self.beaker else (c.geom2, b2)
            if other_geom in self.bench:
                bench += 1
                continue
            side = self.finger_side.get(other_body)
            if side is None:
                continue
            mujoco.mj_contactForce(m, data, i, self.f6)
            f_world = c.frame.reshape(3, 3).T @ self.f6[:3]
            z[side] += (1.0 if b2 == self.beaker else -1.0) * float(f_world[2])
        return z, bench

    def _verdicts(self, s, t, dt, force, finger1, pad_w):
        """Stage 1b, LOG ONLY, one hand, this tick and past ticks only (causal). Returns the seven logged values for this hand.

        Hand speed (m/s) = |pad_w(now) - pad_w(previous tick)| / dt, with dt = t(now) - t(previous tick) (about 1/150 s).
        Hand acceleration (m/s^2, signed, positive = speeding up) = (speed(now) - speed(previous tick)) / dt.
        Both are "nan" where there is no earlier tick to use (the first tick; acceleration also on the second tick).
        Squeeze: |actuator force| >= squeeze_threshold_N (total force, N). Closure: grasp_closure_min_m <= finger1 <= grasp_closure_max_m.
        Holding verdict (decision M250): ON only while both tests pass; turns ON after both have held for holding_confirm_ticks ticks in a
        row; turns OFF at once when either test fails (no confirming delay on the way off).
        Still verdict: ON after the speed has stayed below still_speed_limit_m_s for still_confirm_ticks ticks in a row; OFF as soon as it
        goes above the limit. Acceleration has no limit yet (not decided), so it forms no verdict.
        """
        cfg = HOLD_STILL_CFG
        squeeze_ok = abs(force) >= cfg["squeeze_threshold_N"]
        closure_ok = cfg["grasp_closure_min_m"] <= finger1 <= cfg["grasp_closure_max_m"]
        both = squeeze_ok and closure_ok
        self.hold_count[s] = self.hold_count[s] + 1 if both else 0
        # Decision M250 (Nov 4): holding is on only while BOTH tests pass. It turns on after both have held for the confirming
        # time, and turns off at once (no delay) when either test fails.
        self.holding[s] = both and (self.holding[s] or self.hold_count[s] >= cfg["holding_confirm_ticks"])
        if self.prev_pad[s] is None or dt is None or dt <= 0.0:
            speed = float("nan")
        else:
            speed = float(np.linalg.norm(pad_w - self.prev_pad[s]) / dt)
        if self.prev_speed[s] is None or dt is None or dt <= 0.0 or np.isnan(speed) or np.isnan(self.prev_speed[s]):
            accel = float("nan")
        else:
            accel = float((speed - self.prev_speed[s]) / dt)
        below = (not np.isnan(speed)) and speed < cfg["still_speed_limit_m_s"]
        self.still_count[s] = self.still_count[s] + 1 if below else 0
        still = self.still_count[s] >= cfg["still_confirm_ticks"]
        self.prev_pad[s] = pad_w
        self.prev_speed[s] = speed
        return [f"{int(squeeze_ok)}", f"{int(closure_ok)}", f"{int(self.holding[s])}", f"{speed:.9f}", f"{accel:.9f}",
                f"{int(below)}", f"{int(still)}"]

    def write(self, obs, phase, data):
        """One row for this tick. Reads only."""
        m = self.model
        t = float(obs["time"])
        ap = obs["applied"]
        dt = None if self.prev_t is None else t - self.prev_t
        self.prev_t = t
        row = [int(obs["tick"]), f"{t:.6f}", phase]
        for s in SIDES:
            a = self.arm[s]
            qadr, vadr = a["qadr"], a["vadr"]
            fq = [int(m.jnt_qposadr[j]) for j in a["finger_jnt"]]
            tcp = self.tcp[s]
            pad_w = data.xpos[tcp] + data.xmat[tcp].reshape(3, 3) @ self.pad[s]
            force = float(data.actuator_force[a['gripper']])
            row += [f"{data.qpos[fq[0]]:.9f}", f"{data.qpos[fq[1]]:.9f}",
                    f"{force:.9f}",
                    f"{float(ap['gripper_force'][s]):.9f}", f"{float(ap['gripper'][s]):.9f}",
                    f"{pad_w[0]:.9f}", f"{pad_w[1]:.9f}", f"{pad_w[2]:.9f}"]
            row += [f"{data.qpos[q]:.9f}" for q in qadr] + [f"{data.qvel[v]:.9f}" for v in vadr]
            row += [f"{_held_mass(m, s, t):.9f}"]
            row += self._verdicts(s, t, dt, force, float(data.qpos[fq[0]]), pad_w)
        z, bench = self._pad_contacts(data)
        row += [f"{z[s]:.9f}" for s in SIDES] + [bench]
        self.writer.writerow(row)

    def close(self):
        self.file.close()
