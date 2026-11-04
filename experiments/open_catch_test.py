"""Synthetic tests for the open catch, the source gate and the chunk player (decision M253) and the beaker gravity setting (M254).
No simulation is stepped: the tests drive the same objects the control loop uses (src/open_catch.py, src/chunk_player.py, src/holding.py,
with made-up data.

    python experiments/open_catch_test.py      (exit code 0 when every case passes)
"""

import os
import sys

import numpy as np

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, REPO)

from src import held_mass_ramp as hmr  # noqa: E402
from src.holding import HoldDetector  # noqa: E402
from src.open_catch import OpenCatch, SourceGate  # noqa: E402
from src.task_config import CHUNK_CFG, HOLD_STILL_CFG, OPEN_CATCH_CFG  # noqa: E402

DT = 1.0 / 150.0
OPEN_VALUE = 0.044
CLOSE = {"position": 0.0, "max_force": 8.0}
HELD_KG = 0.2245


class FakeModel:
    """Only a key for the held-mass records (held_mass_ramp keys them by id(model))."""


class FakeData:
    def __init__(self):
        self.time = 0.0
        self.qpos = np.zeros(4)
        self.qvel = np.zeros(4)
        self.actuator_force = np.zeros(2)


def arm_cmd(tag):
    return {"mode": "arms", "arm_targets": {"left": [float(tag)] * 7}, "gripper": {"left": CLOSE}}


def open_cmd(value=OPEN_VALUE):
    return {"mode": "arms", "arm_targets": {}, "gripper": {"left": value}}


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    return bool(ok)


class Harness:
    """One hand, driven the way the control loop drives it each tick: holding update, source gate, open filter, hold steps.
    force -16 N and finger 0.0375 m are a real hold; the holding detector is primed to ON before the first tick."""

    def __init__(self, items, force=-16.0, finger=0.0375):
        self.model = FakeModel()
        self.data = FakeData()
        self.hold = {"left": HoldDetector()}
        self.catch = {"left": OpenCatch(self.model, "left", 0, 1)}
        self.gate = SourceGate(max(1, int(round(150 / CHUNK_CFG["rate_hz"]))))
        self.log = []
        self.played = []          # (frame, command) of every command that reached the arm and gripper
        self.releases = []        # (frame, value) of each gripper open the hold sends
        self.ask_frames = []
        self.frame = 0
        self.items = items
        self.force = force
        self.finger = finger
        hmr.set_target(self.model, "left", HELD_KG, -1.0, 0)     # the held mass is HELD_KG from time 0 on (ramp finished)
        for _ in range(HOLD_STILL_CFG["holding_confirm_ticks"] + 1):
            self.hold["left"].update(self.force, self.finger)

    def ask(self):
        self.ask_frames.append(self.frame)
        return {"mode": "chunk", "commands": self.items} if self.items else None

    def run(self, n):
        for _ in range(n):
            self.tick()

    def tick(self):
        d = self.data
        d.time = self.frame * DT
        d.actuator_force[0] = self.force
        d.qpos[0] = self.finger
        self.hold["left"].update(float(d.actuator_force[0]), float(d.qpos[0]))
        out = self.gate.output("policy", self.ask, self.frame)
        if out is not None:
            cmds = self.gate.filter("policy", [out] if isinstance(out, dict) else out, self.catch, self.hold, d.time, self.frame, self.log)
            self.played += [(self.frame, c) for c in cmds]
        self.gate.hold_steps(self.catch, self.hold, d, d.time, self.frame, self.send_open, self.log)
        self.frame += 1

    def send_open(self, side, value):
        """What the loop does on release: the gripper opens, the squeeze drops and the fingers open (as in the sim)."""
        self.releases.append((self.frame, value))
        self.force = 0.04
        self.finger = OPEN_VALUE

    def arm_frames(self, lo=None, hi=None):
        return [f for f, c in self.played if c["mode"] == "arms" and c["arm_targets"] and (lo is None or f >= lo) and (hi is None or f < hi)]

    def open_frames(self):
        return [f for f, c in self.played if c.get("gripper", {}).get("left") == OPEN_VALUE]

    def kinds(self):
        return [e["kind"] for e in self.log]


def resume_tick(h):
    return next(e["tick"] for e in h.log if e["kind"] == "opened")


def case_a():
    h = Harness([arm_cmd(0), open_cmd()] + [arm_cmd(k) for k in range(2, 50)])
    h.run(300)
    caught = next(e for e in h.log if e["kind"] == "caught")
    resume = resume_tick(h)
    before = h.arm_frames(hi=caught["tick"])
    during = h.arm_frames(lo=caught["tick"], hi=resume)
    open_at_caught = [f for f in h.open_frames() if f >= caught["tick"] and f < resume]
    ok = (
        before == [0]                                       # the earlier command played
        and caught["tick"] == 5 and not open_at_caught      # the open at frame 5 did not reach the gripper
        and not during                                      # the rest of the chunk was discarded: nothing sent during the hold
        and len(h.releases) == 1 and h.releases[0][1] == OPEN_VALUE
        and h.ask_frames[0] == 0 and h.ask_frames[1] == resume + 1   # asked at the start, then the tick after the resume (sources step before the hold steps): not during the hold
        and h.catch["left"].state == "pass" and h.kinds()[-1] == "opened"
    )
    return check("(a) open mid-chunk: earlier command plays; open caught; rest discarded; source not asked during hold; asked again after the "
                 "opening", ok, f"earlier {before}, caught tick {caught['tick']}, resumed tick {resume}, sent during hold {during}, "
                 f"release {h.releases}, asks at {h.ask_frames[:3]}")


def case_b():
    h = Harness([arm_cmd(0), open_cmd(), arm_cmd(2)], force=0.0)     # not holding: the squeeze is off
    h.run(12)
    opens = h.open_frames()
    ok = opens == [5] and not h.releases and not h.log and h.catch["left"].state == "pass"
    return check("(b) open while NOT holding passes straight through, with no hold", ok,
                 f"open passed at frame {opens}, releases {h.releases}, log {h.log}")


def case_c():
    h = Harness([arm_cmd(0), open_cmd()] + [arm_cmd(k) for k in range(2, 50)])
    h.run(10)
    rec = hmr._record(h.model, "left")
    before = (rec["t0"], rec["target"])
    h.catch["left"].on_open(OPEN_VALUE, True, h.data.time, h.frame)
    rec = hmr._record(h.model, "left")
    repeats = [e for e in h.catch["left"].log if e["kind"] == "repeat_ignored"]
    ok = h.catch["left"].state == "ramp" and (rec["t0"], rec["target"]) == before and len(repeats) == 1
    return check("(c) the same open again during the hold is ignored and logged; the ramp is not restarted", ok,
                 f"ramp start and target unchanged {(rec['t0'], rec['target']) == before}, repeat logged {len(repeats)}")


def case_d():
    h = Harness([arm_cmd(0), open_cmd()] + [arm_cmd(k) for k in range(2, 50)])
    h.run(10)
    during = h.catch["left"].state == "ramp"
    h.force = 0.0                                              # the grasp fails during the hold
    fail_tick = h.frame
    h.run(4)
    failed = [e for e in h.log if e["kind"] == "failed_hold"]
    after = h.arm_frames(lo=fail_tick)
    ok = during and len(failed) == 1 and h.catch["left"].state == "pass" and not h.releases and after
    return check("(d) holding verdict off during the hold cancels it; commands pass through at once; the caught open is not sent", ok,
                 f"during hold {during}, failed_hold logged {len(failed)}, releases {h.releases}, commands after failure {len(after)}")


def case_e():
    h = Harness([arm_cmd(0), open_cmd(OPEN_VALUE), arm_cmd(2)])
    h.run(10)
    h.catch["left"].on_open(0.030, True, h.data.time, h.frame)     # a different open during the hold
    diff = [e for e in h.catch["left"].log if e["kind"] == "different_ignored"]
    ok = len(diff) == 1 and h.catch["left"].value == OPEN_VALUE and h.catch["left"].state == "ramp"
    return check("(e) a different second open during the hold: the first is kept; the difference is ignored and logged", ok,
                 f"kept value {h.catch['left'].value}, different logged {len(diff)}")


def case_f():
    items = [arm_cmd(k) for k in range(50)]
    asks = {"n": 0}

    def ask():
        asks["n"] += 1
        return {"mode": "chunk", "commands": items}

    gate = SourceGate(5)
    played = []
    for frame in range(60):
        out = gate.output("policy", ask, frame)
        if out is not None:
            played.append((frame, out["arm_targets"]["left"][0]))
    frames = [f for f, _ in played]
    n = CHUNK_CFG["play_count"]
    ok = frames[:n] == [5 * k for k in range(n)] and frames[n] == 5 * (n - 1) + 1 and asks["n"] == 2 and played[n - 1][1] == float(n - 1)
    return check("(f) chunk player plays play_count commands at the policy rate (every 5 ticks), then asks the source again", ok,
                 f"first {n} frames {frames[:n]}, next frame {frames[n]}, asks {asks['n']}, play_count {n}")


def case_g():
    single = arm_cmd(7)
    lst = [arm_cmd(1), arm_cmd(2)]
    gate = SourceGate(5)
    out1 = gate.output("policy", lambda: single, 0)
    out2 = gate.output("policy", lambda: lst, 1)
    catch = {"left": OpenCatch(FakeModel(), "left", 0, 1)}
    holds = {"left": HoldDetector()}
    kept1 = gate.filter("policy", [out1], catch, holds, 0.0, 0, [])
    kept2 = gate.filter("policy", out2, catch, holds, 0.0, 1, [])
    ok = out1 is single and out2 is lst and kept1 == [single] and kept2 == lst and not gate.chunk_of and not gate.paused_by
    return check("(g) a source giving one command or a list behaves as before: same objects, nothing paused, no chunk", ok,
                 f"single kept {kept1 == [single]}, list kept {kept2 == lst}, chunks {len(gate.chunk_of)}, paused {len(gate.paused_by)}")


def case_config():
    ok = OPEN_CATCH_CFG["open_command_min_m"] == 0.02 and CHUNK_CFG["play_count"] == 10 and HOLD_STILL_CFG["squeeze_threshold_N"] == 2.0
    return check("config: open threshold, chunk play count, squeeze threshold have their decided or placeholder values", ok,
                 f"open {OPEN_CATCH_CFG['open_command_min_m']}, play {CHUNK_CFG['play_count']}, squeeze {HOLD_STILL_CFG['squeeze_threshold_N']}")


if __name__ == "__main__":
    results = [case_a(), case_b(), case_c(), case_d(), case_e(), case_f(), case_g(), case_config()]
    print(f"{sum(results)} of {len(results)} passed")
    sys.exit(0 if all(results) else 1)
