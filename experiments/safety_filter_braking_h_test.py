"""Braking Phase H fix 2 (David): the hard clamp holds the joints the bridge already has (zero step). An injected overspeed must give the
onset positions exactly, with zero joint change while the overspeed lasts. Kinematics only (no simulator).

Run: python experiments/safety_filter_braking_h_test.py   (exit 0 = all pass).
"""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.braking import BrakingLayer  # noqa: E402

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


CAP = float(cfg.BRAKING["hand_speed_cap_m_s"])
LIM = CAP * float(cfg.BRAKING["emergency_speed_multiplier"])
LAST = np.array([[-0.43, -0.25, 0.53, 0.75, -0.11, -0.10, 0.19], [0.40, 0.20, -0.30, 0.60, 0.10, 0.05, -0.20]])
MEASURED = LAST + np.array([[0.0, -0.14, 0.0, 0.76, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])   # the measured joints lag by 0.76 rad


def overspeed_layer(speed_left):
    lay = BrakingLayer(tool_lin)
    lay.last_measure = {"speed": np.array([speed_left, 0.0]), "accel": np.zeros(2), "vel": np.zeros((2, 3))}
    return lay


# 1. the clamp at onset records the joints the bridge holds (the last sent command), not the measured joints
lay = overspeed_layer(LIM * 1.5)
lay._hard_arms(SimpleNamespace(q_left=MEASURED[0], q_right=MEASURED[1]), LAST)
check("fix 2: the onset pose of an overspeed arm is the last sent command (exactly)", np.array_equal(lay._hard_pose[0], LAST[0]),
      f"onset {np.round(lay._hard_pose[0], 3)}")
check("fix 2: and not the measured joints (which were 0.76 rad away)", not np.allclose(lay._hard_pose[0], MEASURED[0]))

# 2. through the layer: an injected overspeed gives the onset positions, with zero joint change over the ticks it lasts
lay = overspeed_layer(LIM * 1.5)
raw = LAST.copy()
raw[0, 0] += 0.02                                  # the source asks for a move; the clamp must hold instead
outs = []
last = LAST.copy()
for _ in range(3):
    out, info = lay.process(last, raw, None, None, hold=False)
    outs.append(out.copy())
    last = out
check("fix 2: the clamp output is exactly the onset positions on the first tick (zero step from the last sent command)",
      np.array_equal(outs[0][0], LAST[0]), f"max step {float(np.abs(outs[0][0] - LAST[0]).max()):.6f} rad")
check("fix 2: over the ticks the overspeed lasts, the left joints do not change (zero joint change)",
      all(np.array_equal(o[0], LAST[0]) for o in outs), f"changes {[float(np.abs(o[0] - LAST[0]).max()) for o in outs]}")
check("fix 2: the clamp raises the override and an error is logged", bool(lay._hard_latch[0]) and any("hard clamp" in e for e in lay.errors))

# 3. the arm that is not overspeed is not clamped
check("fix 2: the arm below the limit is not clamped by the layer", not lay._hard_latch[1])

print(f"{len(fails)} failing check(s)" if fails else "ALL PASS")
sys.exit(1 if fails else 0)
