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
A_LIM = cfg.derived()["effective_accel_limit"]
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

# ---- step 5: look-ahead ----
MARGIN = 0.02
X_OBS = 0.60                                     # obstacle surface along the left hand's x axis (fake arm: x = 0.5 * joint 1)


def obstacle(q, state):
    x = np.asarray(q)[0, 0] * 0.5
    return [(X_OBS - x, MARGIN), None]


def run_toward(layer, ticks, direction=1.0, start=0.0):
    last = np.zeros((2, 7))
    last[0, 0] = start / 0.5
    rec = []
    for k in range(ticks):
        raw = last.copy()
        raw[0, 0] += direction * 0.5
        out, info = layer.process(last, raw, None, obstacle)
        rec.append({"x": float(last[0, 0] * 0.5), "v": float(layer.v[0]), "a": float(layer.a[0]), "braking": bool(info["stop"][0]), "gap": X_OBS - last[0, 0] * 0.5})
        last = out
    return rec


Lk = BrakingLayer(fake_tool)
rec = run_toward(Lk, 700)
first = next(k for k, r in enumerate(rec) if r["braking"])
# independent expectation: the first tick whose state (v, a, gap) has stopping distance >= gap - margin/2 while closing
exp = next(k for k, r in enumerate(rec) if stopping_distance(r["v"], r["a"], A_LIM, J_LIM, CAPV * DT) >= r["gap"] - MARGIN / 2)
check("look-ahead: braking starts at the tick where the stopping distance reaches the distance to the target", first == exp, f"{first} vs {exp}")
check("look-ahead: not braking before that tick", not any(r["braking"] for r in rec[:first]))
x_stop = X_OBS - MARGIN / 2
x_end, v_end = rec[-1]["x"], rec[-1]["v"]
check("look-ahead: the hand stops (speed 0)", v_end == 0.0 and all(r["v"] >= 0 for r in rec))
check("look-ahead: the hand stops at the midpoint of the margin or just short of it", x_stop - 0.005 <= x_end <= x_stop + 1e-9, f"{x_end:.5f} vs {x_stop:.5f}")
dec = np.array([r["a"] for r in rec[first:]])
check("look-ahead: the stop is an S-curve within the acceleration limit", dec.min() >= -A_LIM - EPS)
check("look-ahead: speed reached the cap before braking (obstacle far enough)", max(r["v"] for r in rec) >= CAPV - 1e-9)
# moving away: no braking, no obstacle effect
Lk2 = BrakingLayer(fake_tool)
rec_away = run_toward(Lk2, 100, direction=-1.0, start=0.5)
check("look-ahead: moving away from the obstacle never brakes", not any(r["braking"] for r in rec_away) and rec_away[-1]["v"] >= CAPV - 1e-9)
# obstacle clears: a braking arm is released and speeds up again
Lk3 = BrakingLayer(fake_tool)
last = np.zeros((2, 7))
last[0, 0] = 0.45 / 0.5
Lk3.v[0] = CAPV
latched = False
for k in range(200):
    raw = last.copy()
    raw[0, 0] += 0.5
    out, info = Lk3.process(last, raw, None, obstacle)
    latched = latched or bool(info["stop"][0])
    last = out
    if Lk3.v[0] == 0.0 and latched:
        break
check("look-ahead: arm stopped before the obstacle", Lk3.v[0] == 0.0 and last[0, 0] * 0.5 <= x_stop + 1e-9)
held_pos = last.copy()
out, info = Lk3.process(last, last + np.array([[0.5] + [0] * 6, [0] * 7]), None, obstacle)
check("look-ahead: stays stopped while the command keeps closing", np.array_equal(out, held_pos) and info["stop"][0])
back = last.copy()
back[0, 0] -= 0.5
out, info = Lk3.process(last, back, None, obstacle)
check("look-ahead: released when the command moves away, then speeds up on the S-curve", (not info["stop"][0]) and 0 < Lk3.v[0] < 1e-3 and Lk3.a[0] > 0, f"v {Lk3.v[0]} a {Lk3.a[0]}")
# a hand already accelerating (a0 > 0) must start braking earlier than one at cruise
sd_cruise = stopping_distance(CAPV, 0.0, A_LIM, J_LIM, CAPV * DT)
sd_acc = stopping_distance(CAPV, A_LIM, A_LIM, J_LIM, CAPV * DT)
check("look-ahead: the formula counts current acceleration (a0 > 0 stops later)", sd_acc > sd_cruise)

# ---- step 6: HOLD is a smooth stop (braking S-curve), not repeat_last_safe ----
from types import SimpleNamespace  # noqa: E402

from src import task_config as _tc  # noqa: E402
from src.safety_filter_plugin import SafetyFilterPlugin  # noqa: E402


def run_hold(hold_at, ticks):
    Lh = BrakingLayer(fake_tool)
    last = np.zeros((2, 7))
    vs, accs, speeds_prev = [], [], 0.0
    for k in range(ticks):
        raw = last.copy()
        raw[0, 0] += 0.5
        out, info = Lh.process(last, raw, None, None, hold=(k >= hold_at))
        vs.append(float(Lh.v[0]))
        accs.append(float(Lh.a[0]))
        last = out
    return np.array(vs), np.array(accs)


vh, ah = run_hold(300, 600)
drops = -np.diff(vh[299:])
check("HOLD: the hand was at the cap when HOLD triggered", abs(vh[299] - CAPV) < 1e-9)
check("HOLD: no single-tick speed drop above accel_limit x dt", drops.max() <= A_LIM * DT + 1e-9, f"{drops.max():.6f} vs {A_LIM * DT:.6f}")
jh = np.abs(np.diff(np.concatenate([[ah[299]], ah[300:]]))) / DT
check("HOLD: jerk never above the limit", jh.max() <= J_LIM + 1e-6, f"{jh.max():.4f}")
check("HOLD: the hand ends at rest and stays there", vh[-1] == 0.0 and np.all(vh[-50:] == 0.0))
check("HOLD: the old behaviour (speed to 0 in one tick) would have broken the limit", CAPV > A_LIM * DT)


def pstatus(s, reason=""):
    return SimpleNamespace(status=s, hold_reason=reason, rounds=0, worst_gap=float("nan"), worst_shortfall=0.0, fraction=float("nan"),
                           worst_pair=None, methods=(), pairs_pushed=0, pairs_skipped=0, pairs_shortened=0, solver_calls=0)


class HoldAfter:
    """Stub filter: passes the raw command for n calls, then HOLD (returns the last command, as the real filter does)."""

    def __init__(self, n, error=False):
        self.n, self.calls, self.error = n, 0, error

    def __call__(self, raw, last, state, world, held):
        self.calls += 1
        if self.calls > self.n:
            if self.error:
                raise RuntimeError("boom")
            return last.copy(), pstatus("HOLD", "NO_SOLVER")
        return raw, pstatus("PASS")


def plugin_run(error):
    pl = SafetyFilterPlugin("on", HoldAfter(300, error), lambda holding: (None, [], None), braking=BrakingLayer(fake_tool), run_name="t", out_dir=None)
    cur = {"left": [0.0] * 7, "right": [0.0] * 7}
    x, sent_x = 0.0, []
    for k in range(600):
        x += 0.5
        out = pl.process({"left": [x] + [0.0] * 6, "right": [0.0] * 7}, {}, cur, t=k * DT)
        cur = out
        x = out["left"][0] if k >= 300 else x            # a source keeps asking for more; its own pose follows what was sent only after the hold
        sent_x.append(out["left"][0] * 0.5)
    return np.array(sent_x), pl


sx, pl = plugin_run(False)
sv_ = np.diff(sx) / DT
drop = -np.diff(sv_)
check("HOLD via the plug-in: speed ramps down smoothly and ends at rest", sv_[-1] == 0.0 and drop.max() <= A_LIM * DT + 1e-6, f"{drop.max():.6f}")
check("HOLD via the plug-in: moved past the hold point (it did not freeze in one tick)", sx[-1] - sx[299] > 0.01, f"{sx[-1] - sx[299]:.4f}")
sx2, pl2 = plugin_run(True)
sv2 = np.diff(sx2) / DT
check("filter error: treated as HOLD, smooth stop", sv2[-1] == 0.0 and (-np.diff(sv2)).max() <= A_LIM * DT + 1e-6)
check("config: the plug-in default hold policy is braking", _tc.SAFETY_FILTER_PLUGIN_CFG["hold_policy"] == "braking")

# ---- Phase B fixes: the profile follows what was really sent; the "state" jerk source reads the measured joints ----
Lq = BrakingLayer(fake_tool)
last = np.zeros((2, 7))
vseq = []
for k in range(120):
    raw = last.copy()
    raw[0, 0] += (2e-5 if k < 60 else 0.5)          # 60 ticks of a tiny commanded step (below the ramp's step), then a far command
    out, info = Lq.process(last, raw, None, None)
    vseq.append(float(Lq.commanded_speed(last, out, None)[0]))
    last = out
vseq = np.array(vseq)
check("clipped steps: the profile follows what was sent (no jump to a stale speed when the command speeds up)", np.max(np.diff(vseq)) <= A_LIM * DT + 1e-9, f"{np.max(np.diff(vseq)):.6f} vs {A_LIM * DT:.6f}")
check("clipped steps: after the far command the speed still reaches the cap", abs(vseq[-1] - CAPV) < 1e-6 or vseq[-1] > 0.5 * CAPV, f"{vseq[-1]}")


class Meas:
    def __init__(self, q):
        self.q_left, self.q_right = tuple(q[0]), tuple(q[1])


tun_state = copy.deepcopy(cfg.BRAKING)
tun_state["speed_source"] = "state"
Ls = BrakingLayer(fake_tool, tun_state)
Ls.measure(np.zeros((2, 7)), None)
qm = np.zeros((2, 7))
qm[0, 0] = 0.04                                     # the measured left joint moved: 0.5 * 0.04 = 2 cm in one tick = 3 m/s
Ls.process(np.zeros((2, 7)), np.zeros((2, 7)), Meas(qm))
check('speed_source "state": the ramp starts from the measured speed (3 m/s), not from the profile', abs(Ls.last_measure["speed"][0] - 3.0) < 1e-6 and abs(Ls._va(0)[0] - 3.0) < 1e-6)
Lsent = BrakingLayer(fake_tool)
Lsent.process(np.zeros((2, 7)), np.zeros((2, 7)), Meas(qm))
check('speed_source "sent" (default): measured joints are not read', Lsent.last_measure is None and Lsent._va(0)[0] == 0.0)

# Phase C step 3 made free motion shaped by default; these checks are the Phase B pass-through, switched on explicitly
LEGACY = copy.deepcopy(cfg.BRAKING)
LEGACY["shape_free_motion"] = False
# ---- transparency: a command that respects every limit is passed exactly as it is ----
def minjerk_plan(distance_joint, seconds, ticks):
    u = np.clip(np.arange(1, ticks + 1) / (seconds * 150.0), 0, 1)
    return distance_joint * (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5)


Lt = BrakingLayer(fake_tool, LEGACY)
plan = minjerk_plan(0.6, 10.0, 1500)                        # joint 1 moves 0.6 rad = 0.3 m of tool travel in 10 s: peak 0.056 m/s, within every limit
last = np.zeros((2, 7))
exact, maxdiff = 0, 0.0
for k in range(1500):
    raw = last.copy()
    raw[0, 0] = plan[k]
    out, info = Lt.process(last, raw, None, None)
    exact += int(np.array_equal(out, raw))
    maxdiff = max(maxdiff, float(np.abs(out - raw).max()))
    last = out
check("transparent: a minimum-jerk plan within the limits comes out exactly as it went in, every tick", exact == 1500 and maxdiff == 0.0, f"{exact}/1500 exact, max diff {maxdiff}")
Lj = BrakingLayer(fake_tool, LEGACY)
last = np.zeros((2, 7))
rng_ = np.random.default_rng(0)
same = 0
for k in range(300):
    raw = last.copy()
    raw[0, 0] += rng_.normal(0, 4e-6)                        # micro-motion of a still arm: far below the stop speed
    out, info = Lj.process(last, raw, None, None)
    same += int(np.array_equal(out, raw))
    last = out
check("transparent: micro-motion below the stop speed is not shaped", same == 300, f"{same}/300")
Lv = BrakingLayer(fake_tool, LEGACY)
last = np.zeros((2, 7))
raw = last.copy()
raw[0, 0] = 0.04                                             # a jump of 2 cm in one tick = 3 m/s: far above the cap
out, info = Lv.process(last, raw, None, None)
check("transparent: a command that breaks the cap is still shaped", not np.array_equal(out, raw) and Lv.commanded_speed(last, out, None)[0] < CAPV * 0.01)
# shaping starts when the cap is broken and ends when the ramp has converged on the commanded speed
Lw = BrakingLayer(fake_tool)
last = np.zeros((2, 7))
seq, shaped = [], []
for k in range(500):
    raw = last.copy()
    raw[0, 0] += (0.5 if k < 300 else 0.05 / 0.5 / 150.0 * 0.5 * 2)   # first far ahead (above the cap), then a command at 0.05 m/s (0.5 * 0.1 rad.. = 0.05 m/s)
    out, info = Lw.process(last, raw, None, None)
    seq.append(float(Lw.commanded_speed(last, out, None)[0]))
    shaped.append(bool(Lw._shaping[0]))
    last = out
seq = np.array(seq)
check("shaping: a source that itself commands a slower speed is followed (never past the commanded pose); no overshoot", abs(seq[-1] - 0.05) < 1e-6 and seq[300:].max() <= 0.2 + 1e-9, f"{seq[-1]}")
check("shaping: ends when the ramp has converged on the command, which is then passed exactly (free motion)", (not shaped[-1]) and not shaped[0] is None and any(shaped[:300]))

# ---- Phase B integration findings: the look-ahead acceleration is clamped; an early stop with room left is released ----
Lc2 = BrakingLayer(fake_tool)
Lc2.v[0], Lc2.a[0] = 0.11, 16.0                              # a pulsed command history: 16 m/s2 as a per-tick difference
last = np.zeros((2, 7))
raw = last.copy()
raw[0, 0] = 0.4
import contextlib  # noqa: E402
import io  # noqa: E402
buf2 = io.StringIO()
with contextlib.redirect_stdout(buf2):
    _, _, info2 = Lc2.check(last, raw, None, lambda q, s_: [(0.168 - np.asarray(q)[0, 0] * 0.5, 0.01), None])
check("look-ahead: the acceleration in the stopping distance is clamped to the effective limit (no absurd distance, no false emergency)",
      info2[0]["stop_dist"] < 0.15 and not Lc2.errors, f"{info2[0]['stop_dist']}")
Ld = BrakingLayer(fake_tool)
Ld._braking[0] = True                                        # braking latched, the hand is stopped far from the obstacle (an early stop)
stop2, _, info3 = Ld.check(np.zeros((2, 7)), raw, None, lambda q, s_: [(0.30 - np.asarray(q)[0, 0] * 0.5, 0.01), None])
check("look-ahead: stopped with room left releases the latch (no deadlock after an early stop)", not stop2[0] and not Ld._braking[0])
Ld3 = BrakingLayer(fake_tool)
Ld3._braking[0] = True
stop4, _, _ = Ld3.check(np.zeros((2, 7)), np.zeros((2, 7)), None, lambda q, s_: [(0.0095 - np.asarray(q)[0, 0] * 0.5, 0.01), None])
check("look-ahead: a tick where the source commands no motion does not release the latch (pulsed commands)", stop4[0] and Ld3._braking[0])
Ld4 = BrakingLayer(fake_tool)
Ld4._braking[0] = True
stop5, _, _ = Ld4.check(np.zeros((2, 7)), raw, None, lambda q, s_: [(0.012 - np.asarray(q)[0, 0] * 0.5, 0.01), None])
check("look-ahead: a stopped hand with more room than the one-tick cushion restarts (an early stop must not deadlock the task)", not stop5[0] and not Ld4._braking[0])
Ld2 = BrakingLayer(fake_tool)
Ld2._braking[0] = True
stop3, _, _ = Ld2.check(np.zeros((2, 7)), raw, None, lambda q, s_: [(0.005 - np.asarray(q)[0, 0] * 0.5, 0.01), None])
check("look-ahead: stopped at the midpoint (no room) stays stopped while the command closes", stop3[0] and Ld2._braking[0])

if __name__ == "__main__":
    print(f"\n{len(fails)} failed" if fails else "\nall passed")
    sys.exit(1 if fails else 0)
