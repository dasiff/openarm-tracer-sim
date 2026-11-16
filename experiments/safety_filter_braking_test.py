"""Braking layer (src/safety_filter/braking*.py, config BRAKING): unit tests, steps 1 to 6 of the Part 2 Phase A brief.
Run: python experiments/safety_filter_braking_test.py   (exit 0 = pass)"""
import copy
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.braking_math import stopping_distance  # noqa: E402

fails = []


def check(name, ok, info=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  {info}" if info and not ok else ""))
    if not ok:
        fails.append(name)


# ---- step 1: tunables and derived constants ----
d0 = cfg.derive()
check("derived: tick_dt is 1/tick_hz", math.isclose(d0["tick_dt_s"], 1 / 150.0))
check("derived: tick_cushion = cap x dt", math.isclose(d0["tick_cushion_m"], 0.20 / 150.0))
check("derived: effective accel = tunable (bridge can deliver more)", math.isclose(d0["effective_accel_limit"], 0.50))
cap, a, j = 0.20, 0.50, 2.00
v1 = cap
expect = v1 * v1 / (2 * a) + v1 * a / (2 * j) + cap / 150.0      # v1 >= a^2/j (0.125): the plateau branch; a0 = a would add step 1, see below
want = stopping_distance(cap, d0["effective_accel_limit"], d0["effective_accel_limit"], j, cap / 150.0)
check("derived: worst_stop_dist from the two-step formula at a0 = accel limit", math.isclose(d0["worst_stop_dist_m"], want))
check("derived: worst_stop_dist larger than braking from rest-acceleration alone", d0["worst_stop_dist_m"] > expect)
t2 = copy.deepcopy(cfg.BRAKING)
t2["hand_speed_cap_m_s"] = 0.40
d2 = cfg.derive(t2)
check("derived: doubling the cap raises worst_stop_dist and cushion", d2["worst_stop_dist_m"] > d0["worst_stop_dist_m"] and math.isclose(d2["tick_cushion_m"], 0.40 / 150.0))
t3 = copy.deepcopy(cfg.BRAKING)
t3["jerk_limit_m_s3"] = 1.0
check("derived: lower jerk limit lengthens the stop", cfg.derive(t3)["worst_stop_dist_m"] > d0["worst_stop_dist_m"])
t4 = copy.deepcopy(cfg.BRAKING)
t4["bridge_reach_m"] = 0.001                     # a bridge that can deliver almost nothing per tick
d4 = cfg.derive(t4)
check("derived: effective accel is capped by the bridge", d4["effective_accel_limit"] < 0.50 and math.isclose(d4["effective_accel_limit"], 0.5 * 0.001 * 150.0))
t5 = copy.deepcopy(cfg.BRAKING)
t5["tick_hz"] = 75.0
check("derived: tick rate change changes dt", math.isclose(cfg.derive(t5)["tick_dt_s"], 1 / 75.0))
check("derived: module-level constants equal derive()", cfg.BRAKING_DERIVED == cfg.derive())

# ---- step 2: hand speed ----
from src.safety_filter.braking import BrakingLayer, make_tool_fn  # noqa: E402

DT = 1 / 150.0
LIN = np.array([[0.5, 0.3, 0.2], [0.1, 0.4, 0.6]])


def fake_tool(q, state):                      # linear fake arm: tool point = LIN[side] * first three joints (exact displacement)
    return np.asarray(q)[:, :3] * LIN


L = BrakingLayer(fake_tool)
q0 = np.zeros((2, 7))
q1 = q0.copy()
q1[0, 0], q1[0, 1], q1[1, 2] = 0.010, -0.020, 0.030
L.measure(q0, None)
m = L.measure(q1, None)
exp_left = math.hypot(0.5 * 0.010, 0.3 * 0.020) / DT
exp_right = 0.6 * 0.030 / DT
check("hand speed: measured matches displacement / dt (fake arm)", np.allclose(m["speed"], [exp_left, exp_right]), str(m["speed"]))
check("hand speed: commanded matches displacement / dt", np.allclose(L.commanded_speed(q0, q1, None), [exp_left, exp_right]))
m2 = L.measure(q1, None)
check("hand speed: standing still gives speed 0 and acceleration -v/dt", np.allclose(m2["speed"], 0) and np.allclose(m2["accel"], -m["speed"] / DT))
check("hand speed: first call gives zeros", np.allclose(BrakingLayer(fake_tool).measure(q1, None)["speed"], 0))

# real model: joint 1 of the left arm turned by a known angle moves the tool point by r * angle, r = its distance from the joint axis
import mujoco  # noqa: E402
from src import task_config as tc  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes, RobotState  # noqa: E402

m_ = load_robot_only_model()
shapes = RobotShapes(m_, ROOT / cfg.SHAPES_FILE)
real = BrakingLayer(make_tool_fn(shapes))
home = np.array([tc.TASK_CFG["home_q_left"], tc.TASK_CFG["home_q_right"]], float)
st = RobotState(q_left=tuple(home[0]), q_right=tuple(home[1]))
qa, qb = home.copy(), home.copy()
ang = 0.002
qb[0, 0] += ang
real.measure(qa, st)
mr = real.measure(qb, st)
shapes.set_state(st)
jid = shapes._joint["left"][0]
axis, anchor = shapes.data.xaxis[jid].copy(), shapes.data.xanchor[jid].copy()
hand = shapes.hand_frame("left")[0]
rvec = hand - anchor
r = np.linalg.norm(rvec - axis * (rvec @ axis))
check("hand speed: real model, joint 1 turn gives r * angle / dt", abs(mr["speed"][0] - r * ang / DT) < 0.02 * r * ang / DT, f"{mr['speed'][0]:.4f} vs {r * ang / DT:.4f}")
check("hand speed: real model, the other arm did not move", mr["speed"][1] < 1e-9)
base_moved = RobotState(q_left=tuple(qb[0]), q_right=tuple(qb[1]), base_xyz=(0.003, 0.0, 0.0))
mb = real.measure(qb, base_moved)
check("hand speed: the base's motion is in the measured speed (world frame)", abs(mb["speed"][0] - 0.003 / DT) < 1e-6 and abs(mb["speed"][1] - 0.003 / DT) < 1e-6, str(mb["speed"]))
cs = real.commanded_speed(qa, qb, base_moved)
check("hand speed: the commanded speed has no base term", abs(cs[0] - r * ang / DT) < 0.02 * r * ang / DT and cs[1] < 1e-9, str(cs))

# ---- step 3: hand speed cap ----
CAPV = cfg.BRAKING["hand_speed_cap_m_s"]


def curved_tool(q, state):                    # nonlinear fake arm: the scale factor needs the correction loop
    q = np.asarray(q)
    return np.stack([np.sin(q[:, 0]) * 0.5, np.cos(q[:, 1]) * 0.3, q[:, 2] * 0.2], axis=1)


for name, tf in (("linear fake arm", fake_tool), ("nonlinear fake arm", curved_tool)):
    Lc = BrakingLayer(tf)
    last = np.zeros((2, 7))
    raw = last.copy()
    raw[0, :3] = [0.05, 0.05, 0.05]          # a fast jump: far above the cap
    raw[1, :3] = [0.0002, 0.0, 0.0]          # a slow one: below the cap
    out, capped = Lc.cap(last, raw, None)
    v_out = Lc.commanded_speed(last, out, None)
    check(f"speed cap ({name}): the fast arm ends at the cap", abs(v_out[0] - CAPV) <= 1e-4 * CAPV and v_out[0] <= CAPV * (1 + 1e-6), str(v_out))
    check(f"speed cap ({name}): the slow arm is untouched", np.array_equal(out[1], raw[1]) and not capped[1] and capped[0])
    ratio = out[0, :3] / raw[0, :3]
    check(f"speed cap ({name}): all joints of the capped arm scaled by one factor", np.allclose(ratio, ratio[0]) and 0 < ratio[0] < 1)

# real model: a fast joint 1 command is held at the cap
real2 = BrakingLayer(make_tool_fn(shapes))
fast = home.copy()
fast[0, 0] += 0.05
fast[0, 3] += 0.05
out, capped = real2.cap(home, fast, st)
v_real = real2.commanded_speed(home, out, st)
check("speed cap (real model): output tool speed at or below the cap", v_real[0] <= CAPV * (1 + 1e-6) and v_real[0] > 0.99 * CAPV and capped[0], str(v_real))
slow = home.copy()
slow[0, 0] += 0.0005
out, capped = real2.cap(home, slow, st)
check("speed cap (real model): a slow command passes unchanged", np.array_equal(out, slow) and not capped.any())

# ---- step 4: S-curve profile, jerk limit ----
A_LIM = cfg.BRAKING_DERIVED["effective_accel_limit"]
J_LIM = cfg.BRAKING["jerk_limit_m_s3"]
EPS = 1e-9


def run_profile(layer, ticks, stop_from=None):
    """Feeds a source that always wants far more than the cap (a 'step' to full speed), then optionally a stop; returns per-tick (v, a, pos)."""
    last = np.zeros((2, 7))
    rec = []
    for k in range(ticks):
        raw = last.copy()
        raw[0, 0] += 0.5                       # far ahead: the wanted speed is 0.5 * 0.5 / dt = way above the cap
        stop = np.array([stop_from is not None and k >= stop_from, False])
        out, info = layer.step(last, raw, None, stop=stop)
        rec.append((float(layer.v[0]), float(layer.a[0]), float(out[0, 0] * 0.5)))
        last = out
    return np.array(rec)


Lp = BrakingLayer(fake_tool)
rec = run_profile(Lp, 400)
v, a = rec[:, 0], rec[:, 1]
jerk = np.diff(np.concatenate([[0.0], a])) / DT
check("S-curve up: speed never above the cap and ends on it", v.max() <= CAPV + EPS and abs(v[-1] - CAPV) < 1e-9, f"{v.max()} {v[-1]}")
check("S-curve up: speed never decreases", np.all(np.diff(v) >= -EPS))
check("S-curve up: acceleration never above the limit", a.max() <= A_LIM + EPS and a.min() >= -EPS, f"{a.max()}")
check("S-curve up: jerk never above the limit", np.abs(jerk).max() <= J_LIM + 1e-6, f"{np.abs(jerk).max()}")
k_top = int(np.argmax(a >= A_LIM - EPS))
plateau = int(np.sum(a >= A_LIM - EPS))
check("S-curve up: acceleration ramps up, holds at the limit, ramps down (S shape)",
      a[0] < A_LIM / 2 and a[k_top - 1] < A_LIM and plateau >= 5 and a[-1] == 0.0 and np.all(np.diff(a[:k_top + 1]) >= -EPS) and np.all(np.diff(a[k_top + plateau:]) <= EPS),
      f"top {k_top} plateau {plateau}")
fd_acc = np.diff(np.concatenate([[0.0], v])) / DT
check("S-curve up: acceleration seen by finite difference of the speed also within the limit", fd_acc.max() <= A_LIM + 1e-6)
Lp2 = BrakingLayer(fake_tool)
rec2 = run_profile(Lp2, 600, stop_from=300)
v2, a2, pos2 = rec2[:, 0], rec2[:, 1], rec2[:, 2]
jerk2 = np.diff(np.concatenate([[0.0], a2])) / DT
check("S-curve down: reached the cap before the stop", abs(v2[299] - CAPV) < 1e-9)
check("S-curve down: speed ends at 0, never negative, never rises", v2[-1] == 0.0 and v2.min() >= 0.0 and np.all(np.diff(v2[300:]) <= EPS))
check("S-curve down: acceleration within the limit (braking side)", a2[300:].min() >= -A_LIM - EPS and a2[300:].max() <= EPS, f"{a2[300:].min()}")
check("S-curve down: jerk within the limit", np.abs(jerk2).max() <= J_LIM + 1e-6, f"{np.abs(jerk2).max()}")
kb = 300 + int(np.argmax(a2[300:] <= -A_LIM + EPS))
check("S-curve down: acceleration ramps to -limit, holds, ramps back to 0",
      np.sum(a2[300:] <= -A_LIM + EPS) >= 5 and a2[-1] == 0.0 and a2[kb - 1] > -A_LIM, f"plateau {np.sum(a2[300:] <= -A_LIM + EPS)}")
dist = pos2[-1] - pos2[299]
want = stopping_distance(CAPV, 0.0, A_LIM, J_LIM, CAPV * DT)
check("S-curve down: stopping distance within the formula's distance", dist <= want + 1e-9, f"{dist:.5f} vs {want:.5f}")

if __name__ == "__main__":
    print(f"\n{len(fails)} failed" if fails else "\nall passed")
    sys.exit(1 if fails else 0)
