"""Braking Phase G fix 2 (David): the ramp start and the distance-aware cap take their acceleration from the profile, not from the measured
acceleration; the measured speed stays a bound on the speed. Kinematics only (no simulator).

Run: python experiments/safety_filter_braking_g_test.py   (exit 0 = all pass).
"""
import sys
from pathlib import Path

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


A_EFF = cfg.derived()["effective_accel_limit"]


def layer(v, a, measured_v=None, measured_a=0.0):
    lay = BrakingLayer(tool_lin)
    lay.v[0] = v
    lay.a[0] = a
    if measured_v is not None:
        lay.last_measure = {"speed": np.array([measured_v, 0.0]), "accel": np.array([measured_a, 0.0])}
    return lay


# 1. the ramp start acceleration does not depend on the measured acceleration (its sign, its size)
v_plus, a_plus = layer(0.02, 0.10, measured_v=0.20, measured_a=+0.5)._va(0)
v_minus, a_minus = layer(0.02, 0.10, measured_v=0.20, measured_a=-0.5)._va(0)
check("fix 2: the ramp start acceleration is the profile's, whatever the measured acceleration (+0.5 or -0.5)",
      a_plus == a_minus == 0.10, f"+0.5 -> {a_plus}, -0.5 -> {a_minus}")

# 2. the measured speed is still a bound on the speed when it is faster than the profile
check("fix 2: the measured speed still bounds the speed when it is faster than the profile", v_plus == 0.20 and v_minus == 0.20,
      f"v {v_plus}")
v_slow, _ = layer(0.10, 0.0, measured_v=0.05, measured_a=0.5)._va(0)
check("fix 2: a measured speed below the profile does not lower the start speed (as before)", v_slow == 0.10, f"v {v_slow}")

# 3. the distance-aware cap does not change when the measured acceleration reverses (the alternation's source)
caps = []
for ma in (+0.5, -0.5):
    lay = layer(0.12, 0.20, measured_v=0.12, measured_a=ma)
    caps.append(lay._stoppable_speed(0, 0.03))
check("fix 2: the distance-aware cap is the same when the measured acceleration reverses (+0.5 and -0.5)",
      abs(caps[0] - caps[1]) < 1e-12, f"{caps[0]:.6f} and {caps[1]:.6f} m/s")

# 4. the cap uses the profile's acceleration (the planned speed), clamped to [0, a_eff] as before
c_profile = layer(0.12, 0.20)._stoppable_speed(0, 0.03)
check("fix 2: the cap follows the profile's acceleration", c_profile == layer(0.12, 0.20, measured_v=0.12, measured_a=-0.5)._stoppable_speed(0, 0.03))
check("fix 2: the profile acceleration is clamped to the effective limit in the cap (a_eff)",
      abs(layer(0.12, 5.0)._stoppable_speed(0, 0.03) - layer(0.12, A_EFF)._stoppable_speed(0, 0.03)) < 1e-12)

print(f"{len(fails)} failing check(s)" if fails else "ALL PASS")
sys.exit(1 if fails else 0)
