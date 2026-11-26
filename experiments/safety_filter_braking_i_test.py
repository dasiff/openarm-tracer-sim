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

# ---- fix 2 (SF-90): the just-released object is excluded from the arm's look-ahead until the hand is held_world_margin_m away ----
from dataclasses import replace  # noqa: E402
from types import SimpleNamespace  # noqa: E402

from src.safety_filter.braking import HAND_SHAPE_PARTS, ObstacleFeed, _aabb  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes, RobotState  # noqa: E402
from src.safety_filter.touch import as_placed  # noqa: E402
from src.safety_filter.world import catalog_object  # noqa: E402

rs = RobotShapes(load_robot_only_model(), ROOT / "data" / "safety_filter" / "robot_shapes.json")
st = RobotState(pedestal_m=0.155, q_left=(0.394, -0.257, 0.271, 2.201, -0.065, 0.477, 0.227), q_right=(-0.394, 0.257, -0.271, 2.201, 0.065, -0.477, -0.227))
q = np.array([st.q_left, st.q_right], float)
placed_l = rs.place(st)
palm = next(p for p in placed_l if p.part == "palm" and p.arm == "left")
hand_hi_x = max(_aabb(p)[1][0] for p in placed_l if p.arm == "left" and p.part in HAND_SHAPE_PARTS)   # the furthest hand shape on +x
b0 = catalog_object("beaker", np.zeros(3))
lo_b, hi_b = _aabb(as_placed(b0))
lo_p, hi_p = _aabb(palm)
cb, cp = (lo_b + hi_b) / 2, (lo_p + hi_p) / 2
delta = np.array([(hand_hi_x + 0.005) - lo_b[0], cp[1] - cb[1], cp[2] - cb[2]])      # 5 mm beyond every hand shape on +x, centres aligned in y and z with the palm
beaker_near = replace(b0, centre=tuple(np.asarray(b0.centre, float) + delta))
beaker_far = replace(beaker_near, centre=tuple(np.asarray(beaker_near.centre, float) + np.array([0.015, 0.0, 0.0])))   # moved 1.5 cm away
nothing_held = SimpleNamespace(held={}, placed=lambda side, pos, rot: None)
held_beaker = SimpleNamespace(held={"left": {"kind": "object", "obj": beaker_near}}, placed=lambda side, pos, rot: None)


def beaker_pairs(feed):
    return [c for c in feed.pairs(q, st)[0] if c[5] == "beaker"]


control = ObstacleFeed(rs)
control.bind([beaker_near], nothing_held)                  # never held: the beaker is a normal obstacle
check("fix 2 control: a beaker 5 mm from the palm that was never held is checked (a candidate pair)", len(beaker_pairs(control)) > 0,
      f"{len(beaker_pairs(control))} pairs")

feed = ObstacleFeed(rs)
feed.bind([], held_beaker)                                  # the arm holds the beaker
feed.bind([beaker_near], nothing_held)                      # the gripper opens next to it (released this tick)
check("fix 2: open the gripper next to a beaker: no look-ahead pair against the beaker", len(beaker_pairs(feed)) == 0,
      f"{len(beaker_pairs(feed))} pairs")
feed.bind([beaker_far], nothing_held)                       # the hand is now 1.5 cm further away (the beaker moved 1.5 cm)
check("fix 2: after moving 1.5 cm away (more than held_world_margin_m), the beaker IS checked", len(beaker_pairs(feed)) > 0,
      f"{len(beaker_pairs(feed))} pairs")
check("fix 2: the margin is held_world_margin_m (1 cm placeholder, SF-90)", abs(cfg.BRAKING["held_world_margin_m"] - 0.01) < 1e-12)

# ---- fix 3 (SF-91): a stop that begins starts the ramp from the profile's planned speed, not the measured speed ----
def fix3_stop(moving, measured_speed):
    """Returns (sent speed on the last free tick, sent speed on the stop's first tick, profile speed before the stop, shaping before the stop).
    moving=False: the brkZ5 case: a tiny move sets the arm's direction, then no motion, so the profile settles at rest; the measured speed is
    0.029 m/s (the sag at the release). moving=True: steady motion at 0.1 m/s commanded (the profile settles at its speed under the stop cap);
    the measured speed is 0.13 m/s. Then a stop begins with no motion commanded."""
    lay = BrakingLayer(tool_lin)
    lay.last_measure = {"speed": np.zeros(2), "accel": np.zeros(2), "vel": np.zeros((2, 3))}
    out = np.zeros((2, 7))

    def advance(cur, raw):
        new, _ = lay.step(cur, raw, None)
        return new, float(lay.commanded_speed(cur, new, None)[0])

    s_before = 0.0
    if moving:
        dq = 0.1 * lay.dt / 0.5                    # the joint step per tick for 0.1 m/s at the tool (0.5 m/rad)
        for _ in range(60):
            raw = out.copy()
            raw[0, 0] += dq
            out, s_before = advance(out, raw)
    else:
        raw = out.copy()
        raw[0, 0] = 1e-5
        out, s_before = advance(out, raw)
        for _ in range(10):
            out, s_before = advance(out, out)
    v_before, shaping_before = float(lay.v[0]), bool(lay._shaping[0])
    lay.last_measure = {"speed": np.array([measured_speed, 0.0]), "accel": np.zeros(2), "vel": np.zeros((2, 3))}
    out2, _ = lay.step(out, out, None, stop=np.array([True, False]))
    s2 = float(lay.commanded_speed(out, out2, None)[0])
    return s_before, s2, v_before, shaping_before


a_eff_dt = cfg.derived()["effective_accel_limit"] * cfg.derived()["tick_dt_s"]
s_b, s_2, v_b, sh_b = fix3_stop(False, 0.029)
check("fix 3 precondition (brkZ5, at rest): the profile is at rest and not shaping before the stop", v_b == 0.0 and not sh_b,
      f"profile {v_b:.6f} m/s, shaping {sh_b}")
check("fix 3 (brkZ5, at rest): the first stop tick's speed change is within the jerk limit (no 0 -> 0.029 jump)",
      abs(s_2 - s_b) <= a_eff_dt + 1e-9, f"speed {s_b:.5f} -> {s_2:.5f} m/s (limit {a_eff_dt:.4f} m/s per tick)")
s_b, s_2, v_b, sh_b = fix3_stop(True, 0.13)
check("fix 3 precondition (moving): the profile is moving at its own speed before the stop", v_b > 1e-3, f"profile {v_b:.5f} m/s, shaping {sh_b}")
check("fix 3 (moving): a stop that begins while moving, measured 0.13 m/s: the first tick's speed change is within the jerk limit",
      abs(s_2 - s_b) <= a_eff_dt + 1e-9, f"speed {s_b:.5f} -> {s_2:.5f} m/s (limit {a_eff_dt:.4f} m/s per tick)")

print(f"{len(fails)} failing check(s)" if fails else "ALL PASS")
sys.exit(1 if fails else 0)
