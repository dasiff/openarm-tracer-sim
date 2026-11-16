"""Braking Phase C (David's fixes after the Phase B review): unit tests per step.
Run: python experiments/safety_filter_braking_c_test.py   (exit 0 = pass)"""
import contextlib
import copy
import io
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.touch import as_placed, gap_between  # noqa: E402
from src.safety_filter.world import catalog_object  # noqa: E402

fails = []


def check(name, ok, info=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  {info}" if info and not ok else ""))
    if not ok:
        fails.append(name)


# ---- step 1: the beaker's collision shape has the cylinder's extent (0.10 m tall, radius 0.035 m) ----
root = ET.parse(ROOT / "models" / "objects" / "beaker.xml").getroot()
g = [e for e in root.iter("geom") if e.get("name") == "task_beaker_glass"][0]
r_xml, h_xml = [float(x) for x in g.get("size").split()][:2]
bottom_z = 1.0
beaker = catalog_object("beaker", np.array([0.0, 0.0, bottom_z]))
ext_lo = beaker.centre[2] - (beaker.size[1] + beaker.size[0])
ext_hi = beaker.centre[2] + (beaker.size[1] + beaker.size[0])
check("step 1: the beaker's collision shape spans the cylinder's height exactly (lowest point = flat bottom)",
      abs(ext_lo - bottom_z) < 1e-12 and abs((ext_hi - ext_lo) - 2 * h_xml) < 1e-12, f"lo {ext_lo} hi {ext_hi} xml half {h_xml}")
check("step 1: the beaker's collision radius is the cylinder radius", abs(beaker.size[0] - r_xml) < 1e-12)

# held-vs-hotplate at margin 0: the beaker's flat bottom reaches the plate top (gap ~ 0), not 3.5 cm above it
hot = catalog_object("hotplate", np.array([0.0, 0.0, 0.86]))            # hotplate: its top face at z 0.86 (entry at top)
hot_top = float(hot.centre[2] + hot.size[2])
for label, dz in (("touching", 0.0), ("1 cm above", 0.01), ("3.5 cm above", 0.035)):
    b = catalog_object("beaker", np.array([0.0, 0.0, hot_top + dz]))
    gap = float(gap_between(as_placed(b), as_placed(hot))[0])
    check(f"step 1: beaker {label} the hotplate top: gap equals the bottom height above the plate ({gap * 1000:.2f} mm)", abs(gap - dz) < 1e-6, f"gap {gap}")
check("step 1: held-vs-hotplate margin is 0 (decided)", cfg.HELD_SURFACE_MARGIN_M == 0.0)

# ---- step 2: palm vs the hotplate (allowed-contact pair, held_world_margin_m, approach ceiling) ----
from src import task_config as tc  # noqa: E402
from src.safety_filter.braking import ObstacleFeed  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes, RobotState  # noqa: E402
from src.safety_filter.touch import Exempt, Rule, pair_rule  # noqa: E402

rs = RobotShapes(load_robot_only_model(), ROOT / cfg.SHAPES_FILE)
q_home = np.array([tc.TASK_CFG["home_q_left"], tc.TASK_CFG["home_q_right"]], float)
st_home = RobotState(q_left=tuple(q_home[0]), q_right=tuple(q_home[1]))
placed = rs.place(st_home)
palm_caps = [p for p in placed if p.arm == "left" and p.part == "palm"]
palm_low = min(min(p.a[2], p.b[2]) - p.r for p in palm_caps)
px, py = float(np.mean([p.a[0] for p in palm_caps])), float(np.mean([p.a[1] for p in palm_caps]))
hp_top = palm_low - 0.015                                                # the plate's top 1.5 cm below the palm's lowest point
hp_under = catalog_object("hotplate", np.array([px, py, hp_top]))
hp_placed = as_placed(hp_under)
r_palm = pair_rule(palm_caps[0], hp_placed)
check("step 2: palm vs hotplate is an allowed-contact pair with margin held_world_margin_m, named surface", isinstance(r_palm, Rule) and r_palm.kind == "palm-surface"
      and abs(r_palm.margin - cfg.MARGINS["held_world_margin_m"]) < 1e-12 and r_palm.surface == "hotplate", str(r_palm))
l7 = [p for p in placed if p.arm == "left" and p.part == "link7"][0]
r_l7 = pair_rule(l7, hp_placed)
check("step 2: a link-7 shape against the hotplate is NOT a surface pair (arm-world, no ceiling)", isinstance(r_l7, Rule) and r_l7.kind == "arm-world" and not r_l7.surface, str(r_l7))
check("step 2: fingers stay exempt from the world (surfaces too)", isinstance(pair_rule(next(p for p in placed if p.arm == "left" and p.part == "finger"), hp_placed), Exempt))
feed = ObstacleFeed(rs)
feed.bind([hp_under], None)
cands = [c for c in feed.pairs(q_home, st_home)[0] if c[5] == "hotplate"]
palm_c = [c for c in cands if c[4].startswith("left_palm")]
check("step 2: the palm's candidate pair carries the hotplate's ceiling and the gap to the plate top (1.5 cm)",
      len(palm_c) >= 1 and abs(min(c[1] for c in palm_c) - 0.015) < 2e-3 and palm_c[0][3] == cfg.approach_ceiling("hotplate") and palm_c[0][2] == cfg.MARGINS["held_world_margin_m"],
      str(palm_c[:1]))

# ---- step 3: free motion is shaped: the step toward the source's target is limited, the target is not changed ----
from src.safety_filter.braking import BrakingLayer  # noqa: E402

DT3 = 1 / 150.0
LIN3 = np.array([[0.5, 0.3, 0.2], [0.1, 0.4, 0.6]])


def tool3(q, state):
    return np.asarray(q)[:, :3] * LIN3


def tool_series(outs):
    pos = np.array([tool3(q, None) for q in outs])
    v = np.linalg.norm(np.diff(pos, axis=0), axis=2) / DT3
    a = np.diff(v, axis=0) / DT3
    j = np.diff(a, axis=0) / DT3
    return v, a, j


A3, J3 = cfg.derived()["effective_accel_limit"], cfg.BRAKING["jerk_limit_m_s3"]
L3 = BrakingLayer(tool3)
last = np.zeros((2, 7))
outs = [last.copy()]
for k in range(600):                                    # a pulsed source: a big joint jump on every third tick, rest in between
    raw = last.copy()
    if k % 3 == 0:
        raw[0, 0] += 0.2 if k < 300 else -0.2
    out, _ = L3.process(last, raw, None, None)
    outs.append(out)
    last = out
v3, a3, j3 = tool_series(outs)
check("step 3: a pulsed source: the commanded tool speed never above the cap", v3.max() <= cfg.BRAKING["hand_speed_cap_m_s"] * (1 + 1e-6), f"{v3.max()}")
check("step 3: a pulsed source: commanded acceleration within the limit (free motion)", np.abs(a3).max() <= A3 * (1 + 1e-4) + 1e-9, f"{np.abs(a3).max()} vs {A3}")
check("step 3: a pulsed source: commanded jerk within the limit (free motion)", np.abs(j3).max() <= J3 * (1 + 1e-4) + 1e-6, f"{np.abs(j3).max()} vs {J3}")
L4 = BrakingLayer(tool3)
last4 = np.zeros((2, 7))
target4 = last4.copy()
target4[0, 0] = 0.05
for k in range(900):                                    # the source holds its target: the output must reach it exactly, never past it
    out4, _ = L4.process(last4, target4, None, None)
    if np.max(np.abs(out4 - target4)) > np.max(np.abs(last4 - target4)) + 1e-12 and k > 0:
        break
    last4 = out4
check("step 3: a held target is reached exactly (the target is preserved, the joints ramp to it)", np.array_equal(last4, target4) or np.abs(last4 - target4).max() < 1e-12, f"{np.abs(last4 - target4).max()}")
L5 = BrakingLayer(tool3)
last5 = np.zeros((2, 7))
raw5 = last5.copy()
raw5[0, 0] = 0.001                                      # a tiny command (rest, far below the stop speed): the output must follow it (no stall on small targets)
for k in range(600):
    out5, _ = L5.process(last5, raw5, None, None)
    last5 = out5
check("step 3: a small target below the stop speed is still reached", np.abs(last5 - raw5).max() < 1e-9, f"{np.abs(last5 - raw5).max()}")

# ---- step 4: the hand stops at margin + PUSH_STOP_BEYOND_M and stays still (no creeping) ----
from src.safety_filter.braking import stop_gap  # noqa: E402

WALL_X = 0.30                                              # the obstacle surface, along the fake arm's x (x = 0.5 * joint 1)
MARG4 = 0.01
SG4 = MARG4 + cfg.PUSH_STOP_BEYOND_M


def wall4(q, state):
    return [[(WALL_X - np.asarray(q)[0, 0] * 0.5, MARG4, None, "left_palm_2", "bench_top")], None]


L4s = BrakingLayer(tool3)
last4s = np.zeros((2, 7))
gaps4, outs4 = [], []
for k in range(2000):                                       # a source that keeps commanding toward the wall at 0.1 rad per tick
    raw4s = last4s.copy()
    raw4s[0, 0] += 0.1
    out4s, _ = L4s.process(last4s, raw4s, None, wall4)
    gaps4.append(WALL_X - out4s[0, 0] * 0.5)
    outs4.append(out4s[0, 0])
    last4s = out4s
gaps4 = np.array(gaps4)
check("step 4: the stop target is margin + PUSH_STOP_BEYOND_M (not the midpoint)", abs(stop_gap(MARG4, None) - SG4) < 1e-12 and stop_gap(MARG4, "hotplate") == MARG4)
check("step 4: the hand stops at the stop target, on the safe side and within 5 mm of it", SG4 - 1e-6 <= gaps4[-1] <= SG4 + 0.005, f"final gap {gaps4[-1] * 1000:.3f} mm, target {SG4 * 1000:.1f} mm")
check("step 4: the hand stays still once stopped (no creeping for 300 ticks)", np.max(np.abs(np.diff(outs4[-300:]))) < 1e-12, f"{np.max(np.abs(np.diff(outs4[-300:])))}")
check("step 4: the hand never gets closer than the stop target", gaps4.min() >= SG4 - 0.005, f"min gap {gaps4.min() * 1000:.3f} mm")

# ---- step 6: the look-ahead brakes for the closing component of the hand velocity ----
DOWN6 = np.array([0.0, 0.0, -1.0])                          # the bench's normal, from the hand down to the bench


def xy_tool(q, state):                                     # joint 1 moves the tool along x only; the tool sits 2 cm above the bench (z = 0.02)
    q = np.asarray(q)
    return np.stack([0.5 * q[:, 0], np.zeros(2), np.full(2, 0.02)], axis=1)


def bench2cm(q, state):                                    # the hand's gap to the bench is 2 cm, whatever the motion along x
    return [[(0.02, 0.01, None, "left_palm_2", "bench_top", DOWN6)], None]


L6 = BrakingLayer(xy_tool)
last6 = np.zeros((2, 7))
stops6 = 0
for k in range(300):                                       # the source moves the hand along the bench at 0.12 m/s for 2 s
    raw6 = last6.copy()
    raw6[0, 0] += 0.12 * DT3 / 0.5
    out6, info6 = L6.process(last6, raw6, None, bench2cm)
    stops6 += int(info6["stop"].any())
    last6 = out6
check("step 6: a hand moving parallel to the bench at 2 cm does not trigger braking (300 ticks)", stops6 == 0 and not L6.errors, f"{stops6} stop ticks")
check("step 6: and the hand keeps moving along the bench at the commanded speed", abs(xy_tool(last6, None)[0, 0] - 0.5 * last6[0, 0]) < 1e-12 and last6[0, 0] * 0.5 > 0.03)
L7 = BrakingLayer(xy_tool)
L7.v[0] = 0.12
vel7 = np.array([[0.12, 0.0, -0.03], [0.0, 0.0, 0.0]])    # moving along the bench and slowly down toward it
_, _, info7 = L7.check(np.zeros((2, 7)), np.array([[0.02 * 0] + [0.0] * 6, [0.0] * 7]), None,
                       lambda q, s: [[(0.05, 0.01, None, "left_held", "bench_top", DOWN6)], None], vel=vel7)
from src.safety_filter.braking_math import stopping_distance  # noqa: E402
want7 = stopping_distance(0.03, 0.0, cfg.derived()["effective_accel_limit"], cfg.BRAKING["jerk_limit_m_s3"], cfg.derived()["tick_cushion_m"])
check("step 6: the stopping distance uses the closing component (0.03 m/s toward the bench), not the whole hand speed (0.12 m/s)", abs(info7[0]["stop_dist"] - want7) < 1e-12, f"{info7[0]['stop_dist']} vs {want7}")

# ---- step 7: held identity from the scripted grasp (validated, passed to the filter, the verdict is the grasp signal) ----
from src.commands import validate_command  # noqa: E402
from src.safety_filter.touch import HeldTracker  # noqa: E402
from src.safety_filter.world import WorldView  # noqa: E402

cmd7 = {"mode": "arms", "arm_targets": {"left": [0.0] * 7}, "gripper": {"left": {"position": 0.0, "max_force": 20.0}}, "held_object": {"left": "beaker"}}
clean7, msgs7 = validate_command(cmd7)
check("step 7: validate_command keeps the grasp's object name", clean7 is not None and clean7.get("held_object") == {"left": "beaker"}, str(clean7))
rel7, _ = validate_command({"mode": "arms", "arm_targets": {"left": [0.0] * 7}, "gripper": {"left": 0.044}})
check("step 7: a release (no held_object) carries no name", rel7 is not None and "held_object" not in rel7)
bad7, msgs_bad = validate_command({"mode": "arms", "arm_targets": {"left": [0.0] * 7}, "gripper": {"left": 0.0}, "held_object": {"middle": "beaker"}})
check("step 7: a name for an unknown arm side is refused", bad7 is None and msgs_bad, str(msgs_bad))
bad7b, _ = validate_command({"mode": "arms", "arm_targets": {"left": [0.0] * 7}, "gripper": {"left": 0.0}, "held_object": "beaker"})
check("step 7: a name that is not a mapping is refused", bad7b is None)


class _Src7:
    def __init__(self, objs):
        self._o = objs

    def objects(self):
        return list(self._o)


rs7 = RobotShapes(load_robot_only_model(), ROOT / cfg.SHAPES_FILE)
pl7 = rs7.place(st_home)
pos7, rot7 = rs7.hand_frame("left")
pad7 = np.mean([(s.a + s.b) / 2 for s in pl7 if s.part == "finger" and s.arm == "left"], axis=0)
ob7 = catalog_object("beaker", pad7 - np.array([0.0, 0.0, 0.05]))
t7 = HeldTracker(WorldView(_Src7([ob7])))
ev7 = t7.update("left", False, pl7, pos7, rot7, name="beaker")
check("step 7: a name without the grasp verdict attaches nothing (the grasp signal is the verdict)", ev7 is None and t7.placed("left", pos7, rot7) is None)
ev7b = t7.update("left", True, pl7, pos7, rot7, name="beaker")
check("step 7: with the verdict, the named object is attached and the identity comes from the command", ev7b == "attach" and t7.identity_source == "command", f"{ev7b} {t7.identity_source}")
t8 = HeldTracker(WorldView(_Src7([ob7])))
ev8 = t8.update("left", True, pl7, pos7, rot7)
check("step 7: no name, the verdict on and the pads match: the pad match is the fallback identity", ev8 == "attach" and t8.identity_source == "pads", f"{ev8} {t8.identity_source}")

# ---- step 5 fix: "state" with a stationary arm still ramps (the measured speed does not restart the ramp) ----
from types import SimpleNamespace  # noqa: E402

tun_s5 = copy.deepcopy(cfg.BRAKING)
tun_s5["speed_source"] = "state"
L5s = BrakingLayer(tool3, tun_s5)
still5 = SimpleNamespace(q_left=(0.0,) * 7, q_right=(0.0,) * 7)     # the physical arm does not move (its measured speed stays zero)
last5s = np.zeros((2, 7))
target5s = last5s.copy()
target5s[0, 0] = 0.5
for k in range(300):                                    # the source is far ahead: the layer must accelerate on its own profile
    out5s, _ = L5s.process(last5s, target5s, still5, None)
    last5s = out5s
v5s = np.linalg.norm(tool3(out5s, None)[0] - tool3(np.zeros((2, 7)), None)[0])
check("step 5 fix: with the measured arm still, the output still accelerates (no stall from a zero measured speed)", float(np.abs(out5s[0]).max()) > 0.02, f"output travel {np.abs(out5s[0]).max():.4f} rad after 300 ticks")
check("step 5 fix: the accelerating output respects the limits (speed cap)", float(np.linalg.norm(tool3(out5s, None)[0] - tool3(np.zeros((2, 7)), None)[0])) >= 0.0 and L5s.v[0] <= cfg.BRAKING["hand_speed_cap_m_s"] + 1e-9)

# the route filter keeps the grasp's name with the gripper the source owns (it was dropped in the first integration run)
from src.control_loop import _filter_by_route  # noqa: E402

routes7 = {"arms.left": "mpvi", "arms.right": "mpvi", "gripper.left": "mpvi", "gripper.right": "other"}
kept7 = _filter_by_route({"mode": "arms", "arm_targets": {"left": [0.0] * 7}, "gripper": {"left": {"position": 0.0}}, "held_object": {"left": "beaker"}}, "mpvi", routes7)
check("step 7: the route filter keeps held_object for the gripper the source owns", kept7 is not None and kept7.get("held_object") == {"left": "beaker"}, str(kept7))
kept7b = _filter_by_route({"mode": "arms", "arm_targets": {"right": [0.0] * 7}, "gripper": {"right": 0.0}, "held_object": {"right": "beaker"}}, "mpvi", routes7)
check("step 7: and drops it for a gripper the source does not own", kept7b is None or "held_object" not in kept7b, str(kept7b))

# ---- step 7 follow-up: the measured acceleration is a 5-tick average (a noisy speed does not look like an emergency) ----
def measured_acc(noise, accel_true):
    L8 = BrakingLayer(tool3, copy.deepcopy(cfg.BRAKING))
    rng8 = np.random.default_rng(4)
    out8 = []
    for k in range(80):
        t8 = k * DT3
        q8 = np.zeros((2, 7))
        q8[0, 0] = 0.5 * accel_true * t8 ** 2 / 0.5 + rng8.normal(0, noise)   # tool travel 0.5 * a * t^2 (joint 1 times 0.5)
        L8.measure(q8, SimpleNamespace(q_left=tuple(q8[0]), q_right=tuple(q8[1])))
        out8.append(float(L8.last_measure["accel"][0]))
    return np.array(out8[20:])


acc_noisy = measured_acc(3e-5, 0.0)                          # a steady arm with servo-level noise (3e-5 rad)
check("step 7 follow-up: a steady arm with servo-level noise reads below the acceleration limit (no spike of the size of the limit)", np.abs(acc_noisy).max() < cfg.derived()["effective_accel_limit"], f"max |a| {np.abs(acc_noisy).max():.3f} m/s2 vs limit {cfg.derived()['effective_accel_limit']}")
acc_true = measured_acc(3e-5, 0.4)
check("step 7 follow-up: a real acceleration of 0.4 m/s2 is still measured (the window is short, the signal is kept)", abs(np.median(acc_true) - 0.4) < 0.1, f"median {np.median(acc_true):.3f} m/s2")

# ---- Phase C fix: an emergency is a stop that would enter the margin, not one that overshoots the stop target ----
Le2 = BrakingLayer(tool3)
Le2.v[0] = 0.03
ef = io.StringIO()
with contextlib.redirect_stdout(ef):
    _, _, infoE = Le2.check(np.zeros((2, 7)), np.array([[0.004] + [0.0] * 6, [0.0] * 7]), None,
                            lambda q, s: [[(0.0301 - np.asarray(q)[0, 0] * 0.5, 0.01, None, "left_palm_2", "bench_top", np.array([0.0, 0.0, -1.0]))], None])
check("emergency: a stop that overshoots the stop target but stays outside the margin is not an emergency", not Le2.errors and not Le2._emergency.any(), str(Le2.errors)[:120])
Le3 = BrakingLayer(tool3)
Le3.v[0] = 0.12
with contextlib.redirect_stdout(io.StringIO()):
    Le3.check(np.zeros((2, 7)), np.array([[0.004] + [0.0] * 6, [0.0] * 7]), None,
              lambda q, s: [[(0.0105 - np.asarray(q)[0, 0] * 0.5, 0.01, None, "left_palm_2", "bench_top", np.array([0.0, 0.0, -1.0]))], None])
check("emergency: a stop that would enter the margin is still an emergency", len(Le3.errors) == 1, str(Le3.errors)[:120])

# ---- Phase C fix: the closing component uses the velocity of the closest HAND POINT (the tool point can stand still while a palm closes) ----
LB = BrakingLayer(tool3)
LB.v[0] = 0.0
pz = [0.05]
def closest_point_feed(q, s):
    # the palm's closest point is 5 cm above the bench and descends at 5 cm/s; the tool point (given a zero velocity) does not move
    return [[(pz[0] - 0.01, 0.01, None, "left_palm_2", "bench_top", np.array([0.0, 0.0, -1.0]), np.array([0.0, 0.0, pz[0]]))], None]
LB.check(np.zeros((2, 7)), np.zeros((2, 7)), None, closest_point_feed, vel=np.zeros((2, 3)) if False else np.zeros((2, 3)))
pz[0] = 0.05 - 0.05 * DT3
_, _, infoB = LB.check(np.zeros((2, 7)), np.zeros((2, 7)), None, closest_point_feed, vel=np.zeros((2, 3)))
check("closing component: the closest hand point's own velocity (5 cm/s toward the bench) sets the stopping distance, though the tool point is still", abs(infoB[0]["stop_dist"] - stopping_distance(0.05, 0.0, cfg.derived()["effective_accel_limit"], cfg.BRAKING["jerk_limit_m_s3"], cfg.derived()["tick_cushion_m"])) < 1e-9, f"{infoB[0]['stop_dist']}")

if __name__ == "__main__":
    print(f"\n{len(fails)} failed" if fails else "\nall passed")
    sys.exit(1 if fails else 0)
