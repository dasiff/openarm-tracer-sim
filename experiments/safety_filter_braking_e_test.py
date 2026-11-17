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

# ---- fix 2: the look-ahead uses the speed the ramp will reach (its own speed, the closing share of its motion, plus the current acceleration) ----
def trigger_gap(v_meas, v_prof, a0, raw_step_rad=0.02, margin=MARGIN):
    """The largest gap (m) at which a fresh layer starts braking for the wall, scanned from 40 cm down in 0.5 mm steps. The hand's measured
    closing speed v_meas (m/s), the ramp's own speed v_prof (m/s), its acceleration a0 (m/s^2); the command closes on the wall (a full closing share)."""
    last_q = np.zeros((2, 7))
    raw_q = last_q.copy()
    raw_q[0, 0] = raw_step_rad
    for g in np.arange(0.40, 0.0, -0.0005):
        lay = BrakingLayer(tool_lin)
        lay.v[0] = v_prof
        lay.a[0] = a0
        lay._prev_pa[(0, "left_palm_2", "bench_top")] = np.array([-v_meas * DT, 0.0, 0.0])
        stop, _, _ = lay.check(last_q, raw_q, None, wall(g, margin))
        if stop[0]:
            return float(g)
    return float("nan")


g_const = trigger_gap(0.2, 0.2, 0.0)
g_acc = trigger_gap(0.2, 0.2, 0.4)
check("fix 2: an accelerating arm (0.4 m/s2) starts braking earlier than a constant-speed arm at the same speed and distance",
      g_acc > g_const + 0.002, f"constant {g_const * 1000:.1f} mm, accelerating {g_acc * 1000:.1f} mm")
g_lag = trigger_gap(0.02, 0.2, 0.0)
g_still = trigger_gap(0.02, 0.02, 0.0)
check("fix 2: the ramp's speed counts when the measured speed lags it: a ramp at 0.20 m/s with the arm measured at 0.02 m/s brakes earlier than a ramp at 0.02 m/s",
      g_lag > g_still + 0.005, f"ramp 0.20 m/s {g_lag * 1000:.1f} mm, ramp 0.02 m/s {g_still * 1000:.1f} mm")
last_q0 = np.zeros((2, 7))
lay_nc = BrakingLayer(tool_lin)
lay_nc._prev_pa[(0, "left_palm_2", "bench_top")] = np.array([-0.2 * DT, 0.0, 0.0])
lay_nc.v[0] = 0.2
stop_nc, _, _ = lay_nc.check(last_q0, last_q0.copy(), None, wall(0.05))
check("fix 2: unchanged rule: a pair counts only while the command closes on it (no commanded approach, no brake)",
      not bool(stop_nc[0]), f"stop {stop_nc[0]}")

# ---- fix 3: the distance-aware ramp: a free move's speed never exceeds what the formula allows to still stop at the target ----
A_EFF = cfg.derived()["effective_accel_limit"]
JL = cfg.BRAKING["jerk_limit_m_s3"]
CAP3 = cfg.BRAKING["hand_speed_cap_m_s"]


def stoppable(dist, a0):
    """The largest speed from which the two-step formula stops within dist, from acceleration a0 (no cushion): bisection."""
    a0 = min(max(a0, 0.0), A_EFF)
    lo, hi = 0.0, CAP3
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if stopping_distance(mid, a0, A_EFF, JL, 0.0) <= dist:
            lo = mid
        else:
            hi = mid
    return lo


def free_move(x_target, ticks):
    """A free move (no obstacle): the tool point (tool_lin, x = 0.5 * joint 0) driven to x_target by a command fixed at the target.
    Returns the tool positions, speeds and accelerations of the ramp, one per tick."""
    lay = BrakingLayer(tool_lin)
    last_q = np.zeros((2, 7))
    raw_q = last_q.copy()
    raw_q[0, 0] = 2.0 * x_target
    xs, vs_, as_ = [], [], []
    for _ in range(ticks):
        out, info = lay.process(last_q, raw_q, None, None)
        last_q = out
        xs.append(0.5 * float(out[0, 0]))
        vs_.append(float(info["speed"][0]))
        as_.append(float(info["accel"][0]))
    return np.array(xs), np.array(vs_), np.array(as_)


def envelope(x_target, xs, accs):
    """The stoppable speed of each tick, from the distance left and the acceleration before that tick."""
    x_prev = np.concatenate([[0.0], xs[:-1]])
    a_prev = np.concatenate([[0.0], accs[:-1]])
    return np.array([stoppable(x_target - xp, ap) for xp, ap in zip(x_prev, a_prev)])


xs5, sp5, ac5 = free_move(0.005, 400)
env5 = envelope(0.005, xs5, ac5)
check("fix 3: a target 5 mm away is never overshot (the tool point never passes it)", float(xs5.max()) <= 0.005 + 1e-9, f"max {xs5.max() * 1000:.4f} mm")
check("fix 3: and the move arrives at the target (within 0.1 mm)", abs(float(xs5[-1]) - 0.005) < 1e-4, f"final {xs5[-1] * 1000:.4f} mm")
excess5 = sp5 - env5
k_worst = int(np.argmax(excess5))
check("fix 3: the speed never exceeds what the formula allows at each tick (the stoppable speed for the distance left and the acceleration; "
      "the last sub-millimetre may exceed it by the final step's own speed, below 1e-4 m/s)",
      float(excess5.max()) <= 1e-4, f"worst excess {float(excess5.max()):.3e} m/s at tick {k_worst}, {(0.005 - xs5[k_worst]) * 1e6:.1f} um from the target")
k_peak = int(np.argmax(sp5))
check("fix 3: the ramp rides the allowance up to its peak (the speed at the peak is within 10 % of the allowance at that tick)",
      float(sp5[k_peak]) >= 0.9 * float(env5[k_peak]), f"peak {float(sp5[k_peak]):.5f} m/s vs allowance {float(env5[k_peak]):.5f} m/s")
xs50, sp50, ac50 = free_move(0.5, 300)
check("fix 3: a target 50 cm away ramps to the full hand speed (cap) normally",
      float(sp50.max()) >= 0.99 * CAP3, f"peak {float(sp50.max()):.4f} m/s vs cap {CAP3}")
check("fix 3: and the 50 cm move does not overshoot either", float(xs50.max()) <= 0.5 + 1e-9, f"max {xs50.max():.5f}")
print(f"{len(fails)} failing check(s)" if fails else "ALL PASS")
sys.exit(1 if fails else 0)
