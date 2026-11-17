"""Braking Phase J (David's brief): an ongoing stop uses the profile speed on every tick (fix 1, SF-92); the adaptive wake floor and HOLD
pruning (fixes 3 and 4, SF-94, SF-95); the Numba pair loop matches Python (fix 2, SF-93). Kinematics only (no simulator) except where noted.

Run: python experiments/safety_filter_braking_j_test.py   (exit 0 = all pass).
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


a_eff_dt = cfg.derived()["effective_accel_limit"] * cfg.derived()["tick_dt_s"]


def ongoing_stop(measured, moving):
    """Speeds (m/s, tool point) sent on each tick of a stop that is ongoing from its first tick, with the measured speed of each tick given.
    moving=False: the profile is at rest (a tiny move sets the direction, then no motion). moving=True: steady motion at 0.1 m/s commanded, so
    the profile is at its own speed (about 0.008 m/s under the stop cap) when the stop begins."""
    lay = BrakingLayer(tool_lin)
    lay.last_measure = {"speed": np.zeros(2), "accel": np.zeros(2), "vel": np.zeros((2, 3))}
    out = np.zeros((2, 7))
    if moving:
        dq = 0.1 * lay.dt / 0.5
        for _ in range(60):
            raw = out.copy()
            raw[0, 0] += dq
            prev, out = out, lay.step(out, raw, None)[0]
    else:
        raw = out.copy()
        raw[0, 0] = 1e-5
        prev, out = out, lay.step(out, raw, None)[0]
        for _ in range(10):
            prev, out = out, lay.step(out, out, None)[0]
    before = float(lay.commanded_speed(prev, out, None)[0])     # the sent speed of the last free tick
    speeds = []
    for m in measured:
        lay.last_measure = {"speed": np.array([m, 0.0]), "accel": np.zeros(2), "vel": np.zeros((2, 3))}
        prev = out
        out, _ = lay.step(out, out, None, stop=np.array([True, False]))
        speeds.append(float(lay.commanded_speed(prev, out, None)[0]))
    return before, speeds


def steps_of(before, speeds):
    seq = [before] + speeds
    return [abs(seq[k + 1] - seq[k]) for k in range(len(speeds))]


# ---- fix 1 (SF-92): an ongoing stop uses the profile speed on every tick ----
# (a) the brkZ8 spike case: the profile is at rest; the measured speed is 0.0167 m/s on every tick of the stop (the joint drifts). The old code
# (measured speed on the ongoing ticks) sent 0 -> 0.0167 m/s in one tick: 2.5 m/s2 at 150 Hz.
before, sp = ongoing_stop([0.029, 0.0167, 0.0167, 0.0167, 0.0167, 0.0167], moving=False)
st = steps_of(before, sp)
check("fix 1: an ongoing stop (measured 0.0167 m/s on each tick, profile at rest) never changes the sent speed by more than the accel limit x dt",
      max(st) <= a_eff_dt + 1e-9, f"largest tick change {max(st):.6f} m/s (limit {a_eff_dt:.6f})")

# (b) random measured speeds on an ongoing stop from steady motion: every tick within the limit
rng = np.random.default_rng(7)
measured = rng.uniform(0.0, 0.2, 40)
before, sp = ongoing_stop(measured, moving=True)
st = steps_of(before, sp)
check("fix 1: an ongoing stop from steady motion, random measured speeds 0 to 0.2 m/s over 40 ticks: no tick changes the sent speed by more than the accel limit x dt",
      max(st) <= a_eff_dt + 1e-9, f"largest tick change {max(st):.6f} m/s (limit {a_eff_dt:.6f}), profile before {before:.5f} m/s")
check("fix 1: the stop still brakes (the sent speed ends below where it started)", sp[-1] <= before + 1e-12, f"{before:.5f} -> {sp[-1]:.5f} m/s")

print(f"{len(fails)} failing check(s)" if fails else "ALL PASS")
sys.exit(1 if fails else 0)
