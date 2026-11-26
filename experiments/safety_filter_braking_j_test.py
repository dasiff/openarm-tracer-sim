"""Braking Phase J (David's brief): an ongoing stop uses the profile speed on every tick (fix 1, SF-92); the adaptive wake floor and HOLD
pruning (fixes 3 and 4, SF-94, SF-95); the Numba pair loop matches Python (fix 2, SF-93). Kinematics only (no simulator) except where noted.

Run: python experiments/safety_filter_braking_j_test.py   (exit 0 = all pass).
"""
import math
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

# ---- fix 2 (SF-93): the compiled candidate loop equals the Python loop (1e-6), same candidates in the same order ----
from types import SimpleNamespace  # noqa: E402

from src.safety_filter.braking import ObstacleFeed  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes, RobotState  # noqa: E402
from src.safety_filter.touch import as_placed  # noqa: E402
from src.safety_filter.world import LabMapSource, catalog_object  # noqa: E402

rec = np.load(ROOT / "data" / "safety_filter" / "plugin_runs" / "brkZ8_phaseI_20261117_193334.npz", allow_pickle=True)
rs = RobotShapes(load_robot_only_model(), ROOT / "data" / "safety_filter" / "robot_shapes.json")
world = LabMapSource(str(ROOT / "data" / "safety_filter" / "lab_map.json")).objects()
poses = []
for i in range(2500, 5500, 150):
    b = rec["base"][i]
    st = RobotState(base_xyz=tuple(b[:3]), base_quat=tuple(b[3:7]), pedestal_m=float(rec["pedestal"][i]),
                    q_left=tuple(rec["q_meas"][i, 0]), q_right=tuple(rec["q_meas"][i, 1]),
                    finger_left=float(rec["fingers"][i, 0]), finger_right=float(rec["fingers"][i, 1]))
    poses.append((np.asarray(rec["q_meas"][i], float), st))
palm = next(p for p in rs.place(poses[0][1]) if p.part == "palm" and p.arm == "left")
beaker = catalog_object("beaker", np.asarray(palm.centre if palm.kind == "box" else (palm.a + palm.b) / 2, float))
held = SimpleNamespace(held={"left": {"kind": "object", "obj": beaker}},
                       placed=lambda side, pos, rot: as_placed(beaker) if side == "left" else None)


def pairs_both(q, st, wake, hold, use_held):
    """The candidates of the compiled loop and of the Python loop at one pose (same feed state, same wake)."""
    res = []
    for compiled in (True, False):
        cfg.GEOMETRY["compiled"] = compiled
        feed = ObstacleFeed(rs)
        feed.bind(world, held if use_held else None)
        feed.set_wake(wake, hold)
        res.append(feed.pairs(q, st))
    cfg.GEOMETRY["compiled"] = True
    return res


def max_diff(ck, cp):
    """Largest difference of two candidate lists (same count and ids), or None if they differ in count, order or ids."""
    if len(ck) != len(cp):
        return None
    worst = 0.0
    for x, y in zip(ck, cp):
        if (x[4], x[5]) != (y[4], y[5]) or (x[2] != y[2]) or ((x[3] is None) != (y[3] is None)):
            return None
        worst = max(worst, abs(x[0] - y[0]), abs(x[1] - y[1]), abs(x[8] - y[8]), float(np.max(np.abs(np.asarray(x[6], float) - np.asarray(y[6], float)))),
                    float(np.max(np.abs(np.asarray(x[7], float) - np.asarray(y[7], float)))))
    return worst


old_cfg = cfg.GEOMETRY["compiled"]
worst_all, n_cand, n_cyl, mismatches = 0.0, 0, 0, []
for use_held in (False, True):
    for wake, hold in ((None, np.zeros(2, bool)), (np.array([0.02, 0.10]), np.zeros(2, bool)), (np.array([0.16, 0.16]), np.array([True, False]))):
        for q, st in poses[::3]:
            (kl, kr), (pl, pr) = pairs_both(q, st, wake, hold, use_held)
            for arm_k, arm_p in ((kl, pl), (kr, pr)):
                d = max_diff(arm_k, arm_p)
                n_cand += len(arm_p)
                n_cyl += sum(1 for c in arm_p if c[5] == beaker.id)
                if d is None:
                    mismatches.append((use_held, None if wake is None else wake.tolist(), len(arm_k), len(arm_p)))
                else:
                    worst_all = max(worst_all, d)
cfg.GEOMETRY["compiled"] = old_cfg
check("fix 2: the compiled candidate loop lists the same candidates as the Python loop (count, order, ids), over poses, wake and HOLD settings, with and without the held beaker",
      not mismatches, f"{len(poses[::3]) * 6 * 2} pose-configs, {n_cand} candidates, mismatches {mismatches[:3]}")
check("fix 2: the largest difference of the candidates' numbers (gap, room, stop gap, direction, closest point) is below 1e-6",
      worst_all < 1e-6, f"largest difference {worst_all:.2e} over {n_cand} candidates; held-beaker candidates {n_cyl}")
check("fix 2: the held beaker (a cylinder) was among the candidates, so the cylinder kernels were compared", n_cyl > 0, f"{n_cyl} candidates against the beaker")

# the compiled stopping distance is the Python one (pure arithmetic): 1e-12 over a grid
from src.safety_filter.braking_math import stopping_distance as sd_jit  # noqa: E402


def sd_py(v, a0, a_lim, j_lim, cushion=0.0):
    v = max(0.0, float(v))
    a0 = max(0.0, float(a0))
    a, j = float(a_lim), float(j_lim)
    d1 = v * a0 / j + a0 ** 3 / (3.0 * j * j)
    v1 = v + a0 * a0 / (2.0 * j)
    d2 = v1 * v1 / (2.0 * a) + v1 * a / (2.0 * j) if v1 >= a * a / j else v1 ** 1.5 / math.sqrt(j)
    return d1 + d2 + float(cushion)


grid = [(v, a0, 0.5, 2.0, 0.00133) for v in np.linspace(0.0, 0.2, 21) for a0 in np.linspace(-0.5, 0.5, 11)]
worst_sd = max(abs(sd_jit(*g) - sd_py(*g)) for g in grid)
check("fix 2: the compiled stopping distance equals the Python formula (1e-12) over a grid of speeds and accelerations", worst_sd < 1e-12, f"largest difference {worst_sd:.2e}")

# ---- fix 3 (SF-94): the adaptive wake: the activation is the wake plus the stop gap, never wider than the static cutoff ----
from src import task_config as tc  # noqa: E402
from src.safety_filter.braking import BrakingLayer, make_tool_fn  # noqa: E402
from src.safety_filter.world import WorldObject  # noqa: E402

q_home = np.array([tc.TASK_CFG["home_q_left"], tc.TASK_CFG["home_q_right"]], float)
st_home = RobotState(q_left=tuple(q_home[0]), q_right=tuple(q_home[1]))
placed_home = rs.place(st_home)
palm_caps = [p for p in placed_home if p.arm == "left" and p.part == "palm"]
palm_low = min(min(p.a[2], p.b[2]) - p.r for p in palm_caps)
px_ = float(np.mean([p.a[0] for p in palm_caps]))
py_ = float(np.mean([p.a[1] for p in palm_caps]))


def bench_top(work=True):
    """The bench top's shape under the palm (as in the Phase D test): its top face 5 cm below the palm's lowest point at home."""
    zc = palm_low - 0.05 - 0.06
    return WorldObject(id="bench_top", shape="box", size=(0.5, 0.5, 0.06), centre=(px_, py_, zc), rot=tuple(np.eye(3).ravel()),
                       fixture=True, work_surface=work)


def palm_gap(q, feed):
    """The smallest gap of a left palm shape to the bench top at pose q, measured with the static cutoffs (the geometry)."""
    feed.set_wake(None, np.zeros(2, bool))
    cands = [c for c in feed.pairs(q, st_home)[0] if c[5] == "bench_top" and c[4].startswith("left_palm")]
    return min(c[1] for c in cands)


def descend_palm(adaptive, speed=0.04, ticks=900, hold=False):
    """The Phase D palm descent: a source drives the left palm down at `speed` through the braking layer (adaptive wake as given)."""
    cfg.BRAKING["adaptive_wake"] = adaptive
    feed = ObstacleFeed(rs)
    feed.bind([bench_top()], None)
    eps = 1e-4
    g0 = palm_gap(q_home, feed)
    best = None
    for jj in range(7):
        qp = q_home.copy()
        qp[0, jj] += eps
        dg = (palm_gap(qp, feed) - g0) / eps
        if best is None or dg < best[1]:
            best = (jj, dg)
    j, dg = best
    rate = speed / (abs(dg) * cfg.BRAKING["tick_hz"])
    lay = BrakingLayer(make_tool_fn(rs))
    last = q_home.copy()
    gaps = []
    for k in range(ticks):
        raw = q_home.copy()
        raw[0, j] += rate * (k + 1)
        out, _info = lay.process(last, raw, st_home, feed, hold=hold)
        gaps.append(palm_gap(out, feed))
        last = out
    return np.array(gaps), len(lay.errors)


g_on, err_on = descend_palm(True)
g_off, err_off = descend_palm(False)
cfg.BRAKING["adaptive_wake"] = True
check("fix 3: the palm stop distance is unchanged by the adaptive wake: final palm gap equal to adaptive off within 0.05 mm",
      abs(g_on[-1] - g_off[-1]) < 5e-5, f"adaptive on {g_on[-1] * 1000:.3f} mm, off {g_off[-1] * 1000:.3f} mm")
check("fix 3: with the adaptive wake on the fast palm stops in the Phase D window (14.5 to 17.5 mm)", 0.0145 <= g_on[-1] <= 0.0175,
      f"{g_on[-1] * 1000:.2f} mm")
check("fix 3: the palm's gap sequence is the same with the adaptive wake on and off (every tick within 0.05 mm)",
      float(np.max(np.abs(g_on - g_off))) < 5e-5, f"largest tick difference {float(np.max(np.abs(g_on - g_off))) * 1000:.3f} mm")
check("fix 3: no look-ahead error in either descent", err_on == 0 and err_off == 0, f"errors on {err_on}, off {err_off}")

# pairs awake at a given speed (the wake of the arm's current motion, on the recorded pose 10)
def static_count(q, st):
    feed = ObstacleFeed(rs)
    feed.bind(world, None)
    feed.set_wake(None, np.zeros(2, bool))
    n0 = feed.n_considered
    feed.pairs(q, st)
    return feed.n_considered - n0


best_pose = max(range(len(poses)), key=lambda k: static_count(*poses[k]))     # the recorded pose with the most pairs inside the static cutoff
q10, st10 = poses[best_pose]


def awake_at(speed, wake_override=None):
    feed = ObstacleFeed(rs)
    feed.bind(world, None)
    lay = BrakingLayer(make_tool_fn(rs))
    lay.v[0] = speed
    lay.a[0] = 0.0
    w = lay._wake_radii() if wake_override is None else wake_override
    feed.set_wake(w, np.zeros(2, bool))
    n0 = feed.n_considered
    feed.pairs(q10, st10)
    return feed.n_considered - n0, w[0]


n_static = None
feed_s = ObstacleFeed(rs)
feed_s.bind(world, None)
feed_s.set_wake(None, np.zeros(2, bool))
n0 = feed_s.n_considered
feed_s.pairs(q10, st10)
n_static = feed_s.n_considered - n0
n_002, w_002 = awake_at(0.02)
n_015, w_015 = awake_at(0.15)
n_floor, _ = awake_at(0.02, wake_override=np.zeros(2))
check("fix 3: at 0.02 m/s the awake pairs are the stop-gap zone plus a few mm of wake (between the floor and the static count)",
      n_floor <= n_002 < n_static, f"floor (wake 0) {n_floor}, at 0.02 m/s {n_002} (wake {w_002 * 1000:.1f} mm), static {n_static}")
check("fix 3: at 0.15 m/s more pairs are awake than at 0.02 m/s", n_015 > n_002, f"0.02 m/s {n_002}, 0.15 m/s {n_015} (wake {w_015 * 1000:.1f} mm)")
check("fix 3: adaptive wake is on by default (config)", cfg.BRAKING["adaptive_wake"] is True)

# ---- fix 4 (SF-95): during HOLD the wake is the remaining braking distance plus the margin; once stopped only the margin stays awake ----
def wake_at(speed, hold_on):
    lay = BrakingLayer(make_tool_fn(rs))
    lay.v[0] = speed
    lay.a[0] = 0.0
    lay._hold_arms = np.array([hold_on, False])
    return float(lay._wake_radii()[0])


def awake_hold(speed, hold_on_wake):
    """Awake pairs of the busiest pose in HOLD at a speed, with the wake of the old rule (hold_on_wake False) or of fix 4 (True)."""
    feed = ObstacleFeed(rs)
    feed.bind(world, None)
    w = wake_at(speed, hold_on_wake)
    wake = np.array([w, 0.0])
    feed.set_wake(wake, np.array([True, False]))
    n0 = feed.n_considered
    feed.pairs(q10, st10)
    return feed.n_considered - n0, w


n_hold_before, w_before = awake_hold(0.15, False)
n_hold_after, w_after = awake_hold(0.15, True)
check("fix 4: during HOLD at 0.15 m/s no more pairs are awake than before (the next-tick floor is gone): the awake count falls",
      n_hold_after <= n_hold_before, f"before {n_hold_before} (wake {w_before * 1000:.1f} mm), after {n_hold_after} (wake {w_after * 1000:.1f} mm)")
cush_m = float(cfg.derived()["tick_cushion_m"])
check("fix 4: once stopped, the HOLD wake is the tick cushion alone (the stopping distance at rest), so only the margin (plus one tick) stays awake; without HOLD the next-tick floor adds to it",
      abs(wake_at(0.0, True) - cush_m) < 1e-12 and wake_at(0.0, False) > wake_at(0.0, True),
      f"HOLD at rest {wake_at(0.0, True) * 1000:.2f} mm (cushion {cush_m * 1000:.2f} mm), no-hold at rest {wake_at(0.0, False) * 1000:.2f} mm")
gh_on, eh_on = descend_palm(True, hold=True)
gh_off, eh_off = descend_palm(False, hold=True)
cfg.BRAKING["adaptive_wake"] = True
check("fix 4: the palm in HOLD ends at the same gap with the adaptive wake on and off (within 0.05 mm)",
      abs(gh_on[-1] - gh_off[-1]) < 5e-5, f"on {gh_on[-1] * 1000:.3f} mm, off {gh_off[-1] * 1000:.3f} mm")
check("fix 4: the palm in HOLD stays where it was (the source's descent is held): gap at the end equals the gap at the start (adaptive on)",
      abs(float(gh_on[-1]) - float(gh_on[0])) < 5e-5, f"start {float(gh_on[0]) * 1000:.2f} mm, end {float(gh_on[-1]) * 1000:.2f} mm")
check("fix 4: no look-ahead error in the HOLD descents", eh_on == 0 and eh_off == 0, f"errors on {eh_on}, off {eh_off}")

print(f"{len(fails)} failing check(s)" if fails else "ALL PASS")
sys.exit(1 if fails else 0)
