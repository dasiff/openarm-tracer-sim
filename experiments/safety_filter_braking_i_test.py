"""Braking Phase I (David's brief): release the stop latch on reversal (fix 1, SF-89); release exclusion of the just-released object (fix 2,
SF-90); the stop ramp from the profile speed (fix 3, SF-91). Kinematics only (no simulator).

Run: python experiments/safety_filter_braking_i_test.py   (exit 0 = all pass).
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.braking import BrakingLayer, stop_gap  # noqa: E402

fails = []


def check(name, ok, info=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  [{info}]" if info else ""))
    if not ok:
        fails.append(name)


def tool_lin(q, state):
    q = np.asarray(q, float)
    out = np.zeros((2, 3))
    out[:, 0] = 0.5 * q[:, 0]
    return out


MARG = cfg.MARGINS["arm_world_m"]
SG = stop_gap(MARG, None)
q0 = np.zeros((2, 7))


def fn_for(idb):
    """Two candidate pairs of the left arm, the source's joint 0 moves the tool 0.5 m/rad. Pair A: the left finger against idb, 3 cm of room
    (it latches the stop when the source closes on it). Pair B: the palm against a second obstacle with 20 cm of room, which closes when the
    source moves the other way (it never triggers a stop by itself, but it keeps the old release rule from releasing)."""
    def fn(q, state):
        x = 0.5 * float(np.asarray(q, float)[0, 0])
        pa = np.array([x, 0.0, 0.0])
        return [[(SG + 0.03 - x, MARG, None, "left_finger_left", idb, np.array([1.0, 0.0, 0.0]), pa, SG),
                 (SG + 0.20 + x, MARG, None, "left_palm_2", "beaker", np.array([1.0, 0.0, 0.0]), pa, SG)], None]
    return fn


def stops(dxs, idb="bench_top", v=0.15):
    """The stop flag of the left arm for successive ticks, the source's joint 0 at dxs (rad) each tick, the ramp at v m/s."""
    lay = BrakingLayer(tool_lin)
    lay.v[0] = v
    lay.a[0] = 0.0
    lay._prev_pa[(0, "left_finger_left", idb)] = np.zeros(3)
    fn = fn_for(idb)
    out = []
    for dx in dxs:
        qr = q0.copy()
        qr[0, 0] = dx
        stop, _limit, _info = lay.check(q0, qr, None, fn)
        out.append(bool(stop[0]))
    return out


# ---- fix 1 (SF-89): the stop latch releases on the tick the source reverses; a source that keeps closing keeps it ----
s_rev = stops([0.01, -0.01])
check("fix 1: a stop latched while closing on an obstacle releases within one tick when the source reverses",
      s_rev == [True, False], f"stop per tick {s_rev}")
s_keep = stops([0.01, 0.02])
check("fix 1: a source that keeps closing does NOT release the latch", s_keep == [True, True], f"stop per tick {s_keep}")
s_keep3 = stops([0.01, 0.015, 0.02])
check("fix 1: and it stays latched over several closing ticks", all(s_keep3), f"stop per tick {s_keep3}")
s_rev_then_close = stops([0.01, -0.01, 0.01])
check("fix 1: after the release, a new closing approach latches the stop again", s_rev_then_close == [True, False, True],
      f"stop per tick {s_rev_then_close}")

print(f"{len(fails)} failing check(s)" if fails else "ALL PASS")
sys.exit(1 if fails else 0)
