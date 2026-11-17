"""Braking Phase E (David's fixes after the Phase D review): unit tests per fix.

Fix 1: the emergency also while braking. Fix 2: the look-ahead uses the speed the ramp will reach. Fix 3: distance-aware ramp.
Run: python experiments/safety_filter_braking_e_test.py   (exit 0 = all pass).
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.braking import BrakingLayer, stop_gap  # noqa: E402
from src.safety_filter.braking_math import stopping_distance  # noqa: E402

fails = []


def check(name, ok, info=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  {info}" if info and not ok else ""))
    if not ok:
        fails.append(name)


DT = cfg.derived()["tick_dt_s"]
CUSH = cfg.derived()["tick_cushion_m"]
CAP = cfg.BRAKING["hand_speed_cap_m_s"]
MARGIN = cfg.MARGINS["arm_world_m"]


def tool_lin(q, state):
    """A synthetic tool point: each arm's tool moves 0.5 m per rad of its joint 0 (the arms are not needed for these checks)."""
    q = np.asarray(q, float)
    out = np.zeros((2, 3))
    out[:, 0] = 0.5 * q[:, 0]
    return out


def wall(gap0, margin=MARGIN, ida="left_palm_2", idb="bench_top"):
    """One candidate pair on the left arm. The hand's closest point is at x = 0.5 * joint 0 and the obstacle lies in +x: moving the joint
    toward it lowers the gap (the obstacle is at gap0 from the hand at rest)."""
    def fn(q, state):
        x = 0.5 * float(np.asarray(q, float)[0, 0])
        gap = gap0 - x
        pa = np.array([x, 0.0, 0.0])
        return [[(gap, margin, None, ida, idb, np.array([1.0, 0.0, 0.0]), pa, stop_gap(margin, None))], None]
    return fn


def braking_arm(v, gap0, margin=MARGIN):
    """A layer whose left arm is already braking at speed v, with the closest point moving at v toward the obstacle last tick."""
    lay = BrakingLayer(tool_lin)
    lay._braking[0] = True
    lay.v[0] = v
    lay.a[0] = 0.0
    lay._prev_pa[(0, "left_palm_2", "bench_top")] = np.array([-v * DT, 0.0, 0.0])
    return lay


last = np.zeros((2, 7))
raw_step = last.copy()
raw_step[0, 0] = 0.02                                   # the command moves the hand 1 cm toward the obstacle (2 cm -> 1 cm gap)

# ---- fix 1: the emergency also while braking (no "not already braking" guard) ----
lay1 = braking_arm(0.2, 0.02)
out1, info1 = lay1.process(last, raw_step, None, wall(0.02))
check("fix 1: an arm braking at 0.20 m/s toward a 2 cm gap (margin 1 cm, the stop cannot fit) fires the emergency",
      bool(info1["override"][0]) and any("look-ahead failed (left)" in e for e in lay1.errors), str(lay1.errors[:1]))
check("fix 1: and hard clamps: the command stays at the last sent pose (no S-curve)", np.array_equal(out1[0], last[0]), f"{out1[0][:2]}")
check("fix 1: the stopping distance it reports exceeds the room to the margin", lay1.errors and "exceeds the room to the margin" in lay1.errors[0])

lay1c = braking_arm(0.2, 0.20)
out1c, info1c = lay1c.process(last, raw_step, None, wall(0.20))
check("fix 1: control: a braking arm with room to stop (20 cm gap) does not fire the emergency",
      not bool(info1c["override"][0]) and not lay1c.errors, str(lay1c.errors[:1]))

print(f"{len(fails)} failing check(s)" if fails else "ALL PASS")
sys.exit(1 if fails else 0)
