"""Synthetic tests for the weighing helper in the control loop (decisions M258): the pause, the still wait, the weighing, the failed weigh,
the repeat rule and the once-per-grasp rule. No simulation is stepped, except case (g), which replays the recorded lift-off of the registered
200 g run through the helper's lift phase (its pad heights, masses and times come from the file; no physics).

    python experiments/weigh_helper_test.py      (exit code 0 when every case passes)
"""

import csv
import os
import re
import sys

import numpy as np

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, REPO)

from src import held_mass_ramp as hmr  # noqa: E402
from src.holding import HoldDetector  # noqa: E402
from src.open_catch import OpenCatch, SourceGate  # noqa: E402
from src.task_config import PICK_PLACE_CFG  # noqa: E402
from src.weigh_helper import LIFT, SETTLE, STILL, WeighHelper  # noqa: E402

DT = 1.0 / 150.0
HPZ = 0.09                                           # hand height per kilogram for the synthetic cases (m/kg)
CLOSE = {"position": 0.0, "max_force": 8.0}
BASELINE = os.path.join(REPO, "experiments", "baselines", "retired", "held_200g_20261104", "liftoff_ticks.csv")   # the old-code run (a560eb6), retired Nov 4
RUN_LOG = os.path.join(REPO, "data", "logs")


class FakeModel:
    """Only a key for the held-mass records (held_mass_ramp keys them by id(model))."""


class FakeData:
    def __init__(self):
        self.time = 0.0
        self.qpos = np.zeros(4)
        self.qvel = np.zeros(20)
        self.actuator_force = np.zeros(2)
        self.pad = np.zeros(3)


class Hold:
    """Stands in for HoldDetector in the gate: the holding flag the test sets."""

    def __init__(self, rig):
        self.rig = rig

    @property
    def holding(self):
        return self.rig.holding


def arm_cmd(tag):
    return {"mode": "arms", "arm_targets": {"left": [float(tag)] * 7}, "gripper": {"left": CLOSE}}


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    return bool(ok)


class Rig:
    """One hand, driven the way the control loop drives it each tick: the weighing helper, the source gate, the source, the hold steps."""

    def __init__(self, items, pad_z, holding=False, qvel=0.0):
        self.model = FakeModel()
        hmr.reset(self.model)
        self.data = FakeData()
        self.pad_z = pad_z                               # function of time (s) -> pad height (m)
        self.qvel = qvel                                 # joint speed (rad/s) the arm shows
        self.items = items
        self.holding = holding
        self.frame = 0
        self.asks = []
        self.played = []
        self.log = []
        self.gate = SourceGate(5)
        self.catch = {"left": OpenCatch(self.model, "left", 0, 1)}
        self.hold = {"left": Hold(self)}
        self.weigh = WeighHelper(self.model, "left", lambda d: d.pad, lambda d: HPZ, list(range(7)), PICK_PLACE_CFG)
        self.acts = []

    def ask(self):
        self.asks.append(self.frame)
        return {"mode": "chunk", "commands": self.items} if self.items else None

    def tick(self):
        d = self.data
        d.time = self.frame * DT
        d.pad = np.array([0.0, 0.0, self.pad_z(d.time)])
        d.qvel[:7] = self.qvel
        act = self.weigh.step(d, d.time, self.frame, self.holding)
        if act is not None:
            self.acts.append((self.frame, act))
        if act == "pause":
            assert self.gate.hold_source("left", "weigh")
        elif act == "resume":
            self.gate.release_source("left", "weigh")
        out = self.gate.output("policy", self.ask, self.frame)
        if out is not None:
            cmds = self.gate.filter("policy", [out] if isinstance(out, dict) else out, self.catch, self.hold, d.time, self.frame, self.log)
            self.played += [(self.frame, c) for c in cmds]
        self.gate.hold_steps(self.catch, self.hold, d, d.time, self.frame, lambda s, v, f: None, self.log)
        self.frame += 1

    def run(self, n):
        for _ in range(n):
            self.tick()

    def paused(self):
        return "policy" in self.gate.paused_by

    def kinds(self):
        return [e["kind"] for e in self.weigh.log]

    def arm_played_after(self, frame):
        return [f for f, c in self.played if f >= frame and c.get("arm_targets")]


STEP_PAD = lambda t: 1.0                                  # the pad stays at 1.0 m


def case_a():
    """Holding turns on, not yet weighed: the source is paused, its chunk is discarded, it is not asked, the targets are held."""
    rig = Rig([arm_cmd(k) for k in range(50)], STEP_PAD)
    rig.run(10)                                           # plays arm 0 (frame 0) and arm 1 (frame 5); the chunk's arm 2 is due at frame 10
    rig.holding = True
    rig.run(1)                                            # frame 10: the holding turns on
    paused = rig.paused() and rig.weigh.state == STILL
    chunk_gone = "policy" not in rig.gate.chunk_of
    asks_before = list(rig.asks)
    rig.run(40)                                           # the pause is running (the still wait is not over)
    during = rig.arm_played_after(10)
    ok = paused and chunk_gone and rig.asks == asks_before == [0] and not during and rig.acts[0] == (10, "pause")
    return check("(a) holding turns on, not weighed: the source is paused; its chunk is discarded; it is not asked; no commands pass",
                 ok, f"paused {paused}, chunk discarded {chunk_gone}, asks {rig.asks}, commands after the trigger {during}")


def rising_pad(rig):
    """The pad stays at 1.0 m until the held mass reaches 0.15 kg, then rises 0.5 mm (a lift-off the test can see)."""
    return lambda t: 1.0 + (0.0005 if hmr.mass_at(rig.model, "left", t) >= 0.15 else 0.0)


def case_b():
    """The pause waits for the still verdict, then the weighing runs, the held mass is set, and the source is asked again."""
    rig = Rig([arm_cmd(k) for k in range(50)], None)
    rig.pad_z = rising_pad(rig)
    rig.run(10)                                            # the source has commanded the hand before the grasp holds
    rig.holding = True
    rig.run(1000)
    res = {e["kind"]: e for e in rig.weigh.log}
    resume = next(f for f, a in rig.acts if a == "resume")
    ok_wait = rig.weigh.log and res.get("pause_start", {}).get("tick") == 10
    ok_weighed = "weighed" in res and rig.weigh.weighed
    est = res["weighed"]["est_kg"] if "weighed" in res else None
    ok_set = est is not None and abs(hmr._record(rig.model, "left")["target"] - est) < 1e-12
    ok_asked = rig.asks[0] == 0 and len(rig.asks) >= 2 and rig.asks[1] == resume      # the loop steps the weighing before the sources: asked on the resume tick
    ok = ok_wait and ok_weighed and ok_set and ok_asked
    return check("(b) the pause waits for the still verdict; the weighing runs; the held mass is set to the estimate; the source is asked again",
                 ok, f"pause at 10, lift started {res.get('lift_start', {}).get('tick')}, weighed at {res.get('weighed', {}).get('tick')} "
                 f"(estimate {est}), resume {resume}, asks {rig.asks}")


def case_c():
    """The same trigger repeated during the pause is ignored and logged; the pause is not restarted."""
    rig = Rig([arm_cmd(k) for k in range(50)], STEP_PAD)
    rig.run(10)
    rig.holding = True
    rig.run(10)
    out = rig.weigh.on_trigger(rig.data.time, rig.frame)
    repeats = [e for e in rig.weigh.log if e["kind"] == "repeat_ignored"]
    starts = [e for e in rig.weigh.log if e["kind"] == "pause_start"]
    ok = out is None and len(repeats) == 1 and len(starts) == 1 and rig.paused() and rig.weigh.state == STILL
    return check("(c) the same trigger repeated during the pause is ignored and logged; the pause is not restarted", ok,
                 f"returned {out}, repeat logged {len(repeats)}, pause starts {len(starts)}, still paused {rig.paused()}")


def case_d():
    """Holding turns off during the pause (before the weighing): the pause is cancelled, commands pass at once, the held mass stays zero,
    the event is a failed weigh."""
    rig = Rig([arm_cmd(k) for k in range(50)], STEP_PAD)
    rig.run(10)
    rig.holding = True
    rig.run(10)
    rig.holding = False
    off = rig.frame
    rig.run(3)
    failed = [e for e in rig.weigh.log if e["kind"] == "failed_weigh"]
    rec = hmr._record(rig.model, "left")
    mass = hmr.mass_at(rig.model, "left", rig.data.time)
    passed = rig.arm_played_after(off)
    ok = len(failed) == 1 and failed[0]["state"] == STILL and not rig.paused() and passed and mass == 0.0 and (rec is None or rec["target"] == 0.0)
    return check("(d) holding off during the pause: cancelled; commands pass at once; held mass stays zero; logged as a failed weigh", ok,
                 f"failed weigh logged {len(failed)} in state {failed[0]['state'] if failed else None}, paused {rig.paused()}, "
                 f"commands after the drop {len(passed)}, held mass {mass}")


def case_e():
    """Holding turns off during the weighing (the held mass is rising): cancelled, the mass ramps to zero, commands pass, logged as a failed weigh."""
    rig = Rig([arm_cmd(k) for k in range(50)], lambda t: 1.0)
    rig.run(10)
    rig.holding = True
    rig.run(10)
    while rig.weigh.state != LIFT and rig.frame < 400:
        rig.tick()
    rig.run(20)                                           # the mass is rising now
    mass_before = hmr.mass_at(rig.model, "left", rig.data.time)
    rig.holding = False
    off = rig.frame
    rig.tick()
    rig.run(120)                                          # 0.8 s: the ramp to zero (1 s) is under way; check it reaches zero after 1 s
    rig.run(90)
    failed = [e for e in rig.weigh.log if e["kind"] == "failed_weigh"]
    mass_end = hmr.mass_at(rig.model, "left", rig.data.time)
    passed = rig.arm_played_after(off)
    ok = (len(failed) == 1 and failed[0]["state"] == LIFT and not rig.paused() and passed and mass_before > 0.0 and mass_end == 0.0)
    return check("(e) holding off during the weighing: cancelled; the held mass ramps to zero; commands pass; logged as a failed weigh", ok,
                 f"mass before the drop {mass_before:.4f} kg, failed weigh state {failed[0]['state'] if failed else None}, "
                 f"mass 1.5 s after {mass_end}, commands after the drop {len(passed)}")


def case_f():
    """A grasp already weighed does not trigger again while it holds; a new grasp (holding off, then on) is weighed once more."""
    rig = Rig([arm_cmd(k) for k in range(50)], None)
    rig.pad_z = rising_pad(rig)
    rig.run(10)                                            # the source has commanded the hand before the grasp holds
    rig.holding = True
    rig.run(1000)
    weighed_once = rig.weigh.weighed
    starts_before = len([e for e in rig.weigh.log if e["kind"] == "pause_start"])
    rig.run(200)
    starts_after = len([e for e in rig.weigh.log if e["kind"] == "pause_start"])
    not_paused = not rig.paused()
    rig.holding = False
    rig.tick()
    rig.holding = True
    rig.tick()
    starts_new = len([e for e in rig.weigh.log if e["kind"] == "pause_start"])
    ok = weighed_once and starts_before == 1 and starts_after == 1 and not_paused and starts_new == 2
    return check("(f) a grasp already weighed does not trigger again; a new grasp (off then on) triggers once more", ok,
                 f"weighed {weighed_once}, pause starts {starts_before} then {starts_after}, paused after {not not_paused}, new grasp starts {starts_new}")


def case_g():
    """Lift-off through the helper against the recorded old-code lift-off of the registered 200 g run. The run's per-tick log is written one
    tick late (its "detected" row is the old log's tick 11851 plus one), so tick T is fed the data of row T+1. Before the lift the pad heights
    are the logged pad heights. During the lift the controller's own pad height is the file's baseline plus its logged rise (the logger's
    pad_z_m differs from the controller's by a few micrometres, so it is not used there). The helper's lift phase must then give the old
    detection tick, rise, mass and estimate. The hand height per kilogram is the run's own logged value."""
    with open(BASELINE, newline="") as f:
        r = csv.reader(f)
        head = next(r)
        rows = list(r)
    c = {k: head.index(k) for k in ("tick", "t_sim", "pad_z_m", "mass_kg", "target_kg", "weigh_state", "weigh_base_m", "weigh_rise_m")}
    by_tick = {int(x[c["tick"]]): x for x in rows}
    det_row = next(x for x in rows if x[c["weigh_state"]] == "detected")
    lift_row = next(x for x in rows if x[c["weigh_state"]] == "lift")
    T_LIFT = int(lift_row[c["tick"]]) - 1                   # the lift starts on this tick (the old log numbering)
    T_DET = int(det_row[c["tick"]]) - 1                     # the detection, on the old log's tick 11851
    run_log = open(os.path.join(RUN_LOG, "held200_c1_20261104_184244.log")).read()
    old_log_det = int(re.search(r"lift-off at tick (\d+)", run_log).group(1))
    hpz = float(re.search(r"hand height per kg ([0-9.]+)", run_log).group(1))
    base_file = float(lift_row[c["weigh_base_m"]])

    def data_of(T):
        x = by_tick[T + 1]
        d = FakeData()
        d.time = float(x[c["t_sim"]])
        rise = float(x[c["weigh_rise_m"]]) if x[c["weigh_rise_m"]] else 0.0   # blank on the transition row: no rise is taken there
        z = float(x[c["pad_z_m"]]) if T < T_LIFT else base_file + rise
        d.pad = np.array([0.0, 0.0, z])
        return d

    model = FakeModel()
    hmr.reset(model)
    helper = WeighHelper(model, "left", lambda dd: dd.pad, lambda dd: hpz, list(range(7)), PICK_PLACE_CFG)
    helper.prev_holding = True                              # the grasp was already seen holding
    T0 = T_LIFT - 30                                        # the joints settle 0.2 s (30 ticks) before the lift starts
    helper.state = SETTLE
    helper.zh = [data_of(T).pad[2] for T in range(T0 - 10, T0)]
    det_tick = None
    for T in range(T0, T_DET + 1):
        d = data_of(T)
        d.qvel[:7] = 0.0                                    # the joints are still in the replay
        if helper.step(d, d.time, T, True) == "resume":
            det_tick = T
    det = next(e for e in helper.log if e["kind"] == "weighed")
    lift_start = next(e for e in helper.log if e["kind"] == "lift_start")
    file_rise = float(det_row[c["weigh_rise_m"]])
    file_est = float(det_row[c["target_kg"]])
    file_m = float(det_row[c["mass_kg"]])
    ok = (det_tick == old_log_det and lift_start["tick"] == T_LIFT
          and abs(helper.base - base_file) < 1e-6              # the baseline is from the logged pad heights (their own rounding)
          and abs(det["rise_m"] - file_rise) < 1e-7            # the baseline's own difference (about 2.5e-8 m) carries into the rise
          and abs(det["m_det_kg"] - file_m) < 1e-7             # the file's time and mass rounding (measured difference about 1e-8 kg)
          and abs(det["est_kg"] - file_est) < 1e-5)            # the log prints the hand height per kg to 5 decimals (measured difference about 3e-7 kg)
    return check("(g) the weighing result equals the old code's result for the same input (recorded lift-off of the 200 g run)", ok,
                 f"detection tick {det_tick} (old log {old_log_det}), lift start {lift_start['tick']} (old {T_LIFT}), "
                 f"baseline {helper.base:.9f} (old {base_file:.9f}), rise {det['rise_m'] * 1e3:.5f} mm (old {file_rise * 1e3:.5f}), "
                 f"mass at detection {det['m_det_kg']:.6f} (old {file_m:.6f}), estimate {det['est_kg']:.6f} kg (old {file_est:.6f})")


def case_config():
    ok = PICK_PLACE_CFG["liftoff_rate_n_per_s"] == 0.5 and PICK_PLACE_CFG["liftoff_rise_mm"] == 0.3 and PICK_PLACE_CFG["liftoff_hold_ticks"] == 5 \
        and PICK_PLACE_CFG["liftoff_max_mass_kg"] == 1.0
    return check("config: weighing values are the existing ones (rate 0.5 N/s, rise 0.3 mm, 5 ticks, cap 1 kg)", ok, "")


if __name__ == "__main__":
    results = [case_a(), case_b(), case_c(), case_d(), case_e(), case_f(), case_g(), case_config()]
    print(f"{sum(results)} of {len(results)} passed")
    sys.exit(0 if all(results) else 1)
