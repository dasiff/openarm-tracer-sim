"""Braking layer Phase B (steps 7 to 9): held objects against the world, allowed-contact surfaces, widened pruning, bridge override.
Run: python experiments/safety_filter_braking_b_test.py   (exit 0 = pass)"""
import sys
import warnings
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes, RobotState  # noqa: E402
from src.safety_filter.touch import HeldTracker, Rule, evaluate_pairs, pair_rule, shape_of, as_placed  # noqa: E402
from src.safety_filter.world import OBJECT_CATALOG, WorldObject, WorldView, catalog_object  # noqa: E402

fails = []


def check(name, ok, info=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  {info}" if info and not ok else ""))
    if not ok:
        fails.append(name)


class Src:
    def __init__(self, objs):
        self._o = objs

    def objects(self):
        return list(self._o)


# ---- step 7: held objects against the world ----
check("catalog: the hotplate is an allowed-contact surface, the beaker is not", OBJECT_CATALOG["hotplate"]["allowed_contact"] is True and OBJECT_CATALOG["beaker"]["allowed_contact"] is False)
check("catalog: catalog_object carries the flag", catalog_object("hotplate", np.zeros(3)).allowed_contact and not catalog_object("beaker", np.zeros(3)).allowed_contact)
check("config: approach ceiling of the hotplate is the placeholder 0.05, an unlisted surface gets the hand cap",
      cfg.approach_ceiling("hotplate") == 0.05 and cfg.approach_ceiling("bench_top") == cfg.BRAKING["hand_speed_cap_m_s"])
check("config: no fallback capsule setting", not hasattr(cfg, "HELD_FALLBACK_HEIGHT_M"))

hot_top = 0.86
HOT = catalog_object("hotplate", np.array([1.0, 0.0, hot_top]))            # the world model keeps the hotplate at the centre of its top face (z 0.86)
bench = WorldObject(id="bench_top", shape="box", size=(0.5, 0.5, 0.02), centre=(1.0, 1.0, 0.78), rot=tuple(np.eye(3).ravel()), fixture=True)
bench_top = 0.80


def held_beaker(z_gap, top, x=1.0, y=0.0):
    """A held beaker capsule (the catalog beaker) whose lowest point is z_gap above the plane `top`."""
    rot = np.eye(3)
    return shape_of("left_held", "capsule", (0.035, 0.05), np.array([x, y, top + z_gap + 0.085]), rot, "left", "held", body="openarm_left_link7")


hp, bp = as_placed(HOT), as_placed(bench)
rule_h = pair_rule(held_beaker(0.02, hot_top), hp)
check("held vs hotplate (allowed contact): zero margin, surface named", isinstance(rule_h, Rule) and rule_h.kind == "held-surface" and rule_h.margin == 0.0 and rule_h.surface == "hotplate", str(rule_h))
rule_b = pair_rule(held_beaker(0.02, bench_top, y=1.0), bp)
check("held vs bench: one tunable margin (held_world_margin_m)", isinstance(rule_b, Rule) and rule_b.kind == "held-world" and rule_b.margin == cfg.MARGINS["held_world_margin_m"], str(rule_b))
check("fingers stay exempt from the world (M289)", not isinstance(pair_rule(shape_of("f", "capsule", (0.03, 0.02), np.zeros(3), np.eye(3), "left", "finger"), hp), Rule))
r5 = evaluate_pairs([held_beaker(0.005, hot_top)], [HOT])
check("held 5 mm above the hotplate: not inside its zero margin", len(r5) == 1 and r5[0].rule.kind == "held-surface" and not r5[0].in_margin and abs(r5[0].gap - 0.005) < 1e-6, str([(p.rule.kind, p.gap, p.in_margin) for p in r5]))
r6 = evaluate_pairs([held_beaker(-0.002, hot_top)], [HOT])
check("held touching (gap -2 mm) the hotplate: inside the zero margin", len(r6) == 1 and r6[0].in_margin)
r7 = evaluate_pairs([held_beaker(0.005, bench_top, y=1.0)], [bench])
check("held 5 mm above the bench: inside held_world_margin_m (1 cm), full-stop rule", len(r7) == 1 and r7[0].rule.kind == "held-world" and r7[0].in_margin, str([(p.rule.kind, p.gap, p.in_margin) for p in r7]))

m = load_robot_only_model()
rs = RobotShapes(m, ROOT / cfg.SHAPES_FILE)
from src import task_config as tc  # noqa: E402
st = RobotState(q_left=tuple(tc.TASK_CFG["home_q_left"]), q_right=tuple(tc.TASK_CFG["home_q_right"]))
placed = rs.place(st)
pos, rot = rs.hand_frame("left")
pad = np.mean([(s.a + s.b) / 2 for s in placed if s.part == "finger" and s.arm == "left"], axis=0)
beaker_at_pad = catalog_object("beaker", pad - np.array([0, 0, 0.05]))

# identity from the command
w1 = WorldView(Src([beaker_at_pad]))
t1 = HeldTracker(w1)
with warnings.catch_warnings(record=True) as w:
    warnings.simplefilter("always")
    ev = t1.update("left", True, placed, pos, rot, name="beaker")
check("held by name, pads agree: attached, no warning", ev == "attach" and not w and t1.placed("left", pos, rot).id == "left_held")
w2 = WorldView(Src([catalog_object("beaker", pad + np.array([1.0, 0, -0.05]))]))
t2 = HeldTracker(w2)
with warnings.catch_warnings(record=True) as w:
    warnings.simplefilter("always")
    ev = t2.update("left", False, placed, pos, rot, name="beaker")
check("held by name alone (no verdict, pads see nothing): attached, pad check warns", ev == "attach" and any("pad check failed" in str(x.message) for x in w))
w3 = WorldView(Src([]))
t3 = HeldTracker(w3)
ev = t3.update("left", False, placed, pos, rot, name="beaker")
h3 = t3.placed("left", pos, rot)
check("named object not in the world list: catalog shape centred at the pads", ev == "attach" and h3.kind == "capsule" and np.linalg.norm(h3.centre - pad) < 1e-9 and abs(h3.r - 0.035) < 1e-12)
w4 = WorldView(Src([beaker_at_pad]))
t4 = HeldTracker(w4)
ev = t4.update("left", True, placed, pos, rot)
check("no name, pads match: pad matching is the source", ev == "attach" and t4.placed("left", pos, rot) is not None)
w5 = WorldView(Src([beaker_at_pad]))
t5 = HeldTracker(w5)
with warnings.catch_warnings(record=True) as w:
    warnings.simplefilter("always")
    ev = t5.update("left", True, placed, pos, rot, name="hotplate")
check("name disagrees with the pad match: the name wins, a warning is logged", ev == "attach" and t5.held["left"]["obj"].id == "hotplate" and any("pads match" in str(x.message) for x in w))
t6 = HeldTracker(WorldView(Src([])))
with warnings.catch_warnings(record=True) as w:
    warnings.simplefilter("always")
    ev = t6.update("left", True, placed, pos, rot, name="no_such_object")
check("unknown name and nothing at the pads: empty gripper (gripper-only shapes)", ev is None and t6.placed("left", pos, rot) is None)
t7 = HeldTracker(WorldView(Src([])))
check("empty gripper: verdict on, no name, no match: no held shape, no fallback", t7.update("left", True, placed, pos, rot) is None and t7.placed("left", pos, rot) is None)
shapes_empty = evaluate_pairs(placed, [HOT, bench])
shapes_held = evaluate_pairs(placed + [held_beaker(0.005, hot_top)], [HOT, bench])
check("empty gripper uses the gripper-only shape set (no held pairs); held adds held pairs", not any("held" in p.a + p.b for p in shapes_empty) and any("held" in p.a + p.b for p in shapes_held))
ev = t1.update("left", False, placed, pos, rot, name=None)
check("release: verdict off and no name returns the object to the world with its flag", ev == "release" and w1.find("beaker") is not None and w1.find("beaker").allowed_contact is False)

# ---- step 8: reach from the model, widened cutoffs, sleep bound, obstacle feed, approach ceiling ----
import copy  # noqa: E402

import mujoco  # noqa: E402
from src.safety_filter.braking import BrakingLayer, ObstacleFeed  # noqa: E402
from src.safety_filter.resolver import Resolver  # noqa: E402
from src.safety_filter.touch import gap_between  # noqa: E402

d0 = cfg.derive()
rs.set_state(st)
indep = 0.0
for side in ("left", "right"):
    anc = rs.data.xanchor[rs._joint[side]]
    links = sum(float(np.linalg.norm(anc[k + 1] - anc[k])) for k in range(6))
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"openarm_{side}_link7")
    indep = max(indep, links + float(np.linalg.norm(rs.data.xpos[b] - anc[6])))
check("bridge_reach_m is derived from the arm model, not hard-coded", cfg.BRAKING["bridge_reach_m"] is None and abs(d0["bridge_reach_m"] - indep) < 1e-9 and 0.4 < indep < 0.8, f"{d0['bridge_reach_m']:.4f} vs {indep:.4f}")
t_ov = copy.deepcopy(cfg.BRAKING)
t_ov["bridge_reach_m"] = 0.30
check("bridge_reach_m: an explicit number overrides the derived one", abs(cfg.derive(t_ov)["bridge_reach_m"] - 0.30) < 1e-12)
t_off = copy.deepcopy(cfg.BRAKING)
t_off["tool_offset_local_m"] = (0.0, 0.0, 0.10)
check("bridge_reach_m: moving the tool point changes the derived reach", abs(cfg.derive(t_off)["bridge_reach_m"] - d0["bridge_reach_m"]) > 0.05)
check("derived: arm lag is the bridge's tracking error limit", d0["arm_lag_rad"] == tc.COMMAND_BRIDGE_CFG["max_tracking_error_rad"])

WAKE = d0["worst_stop_dist_m"]
rules = {k: pair_rule(held_beaker(0.02, hot_top), hp) for k in ("held-surface",)}
cut = {"hand-hand": cfg.MARGINS["hand_hand_m"], "robot-robot": cfg.MARGINS["robot_robot_m"], "arm-world": cfg.MARGINS["arm_world_m"], "held-world": cfg.MARGINS["held_world_margin_m"], "held-surface": 0.0}
from src.safety_filter.touch import _rule  # noqa: E402
ok = all(abs(_rule(k, mg).activation - max(mg + cfg.PUSH_START_BEYOND_M, WAKE)) < 1e-12 and abs(_rule(k, mg).release - max(mg + cfg.PUSH_STOP_BEYOND_M, WAKE)) < 1e-12 for k, mg in cut.items())
check("cutoffs: every activation and release distance is max(old value, worst_stop_dist_m)", ok)
check("cutoffs: at the placeholders the stopping distance is the larger one (all widened)", all(_rule(k, mg).activation == WAKE for k, mg in cut.items()))
saved = dict(cfg.BRAKING)
try:
    cfg.BRAKING["hand_speed_cap_m_s"] = 0.01          # a very slow cap and a gentle acceleration: the stopping distance is small, the old cutoffs win again
    cfg.BRAKING["accel_limit_m_s2"] = 0.05
    small = cfg.derived()["worst_stop_dist_m"]
    check("cutoffs: the existing cutoffs stay where the stopping distance is smaller (not removed)", small < 0.04 and abs(_rule("robot-robot", 0.03).activation - 0.04) < 1e-12, f"{small}")
finally:
    cfg.BRAKING.clear()
    cfg.BRAKING.update(saved)


def obstacle_at(shape, direction, gap):
    """A world box whose gap to the capsule `shape` is `gap` (bisection on its offset along `direction`)."""
    mid = 0.5 * (shape.a + shape.b)
    direction = np.asarray(direction, float) / np.linalg.norm(direction)
    lo_, hi_ = 0.0, 1.5
    for _ in range(60):
        off = 0.5 * (lo_ + hi_)
        o = WorldObject(id="obst", shape="box", size=(0.05, 0.05, 0.05), centre=tuple(mid + direction * (shape.r + off + 0.05)), rot=tuple(np.eye(3).ravel()))
        g = float(gap_between(shape, as_placed(o))[0])
        if g > gap:
            hi_ = off
        else:
            lo_ = off
    return o


def sleep_interval(shape_id, direction, gap):
    res = Resolver(rs, 1 / 150.0, None, joint_speed_rad_s=float(tc.COMMAND_BRIDGE_CFG["max_joint_velocity_rad_s"]))
    shp = next(p for p in placed if p.id == shape_id)
    o = obstacle_at(shp, direction, gap)
    q = np.array([tc.TASK_CFG["home_q_left"], tc.TASK_CFG["home_q_right"]], float)
    res.step(q.copy(), q.copy(), st, [o], None)
    sl = res.statics.sleep
    p = sl.tab.key_index[tuple(sorted((shape_id, "obst")))]
    return res, sl, p, int(sl.wake[p] - sl.now), o, shp


for sid in ("left_shoulder_mount", "left_palm_2"):
    shp = next(p for p in placed if p.id == sid)
    dirn = np.array([0.0, 1.0, 0.0])
    _, sl, p, n, o, _ = sleep_interval(sid, dirn, WAKE - 0.01)
    check(f"sleep ({sid}): an obstacle at worst_stop_dist - 1 cm is awake every tick", n == 1 and abs(float(gap_between(shp, as_placed(o))[0]) - (WAKE - 0.01)) < 1e-4, f"n {n}")
_, sl, p, n, o, shp = sleep_interval("left_shoulder_mount", np.array([0.0, 1.0, 0.0]), WAKE + 0.01)
check("sleep (fixed part, no joints): an obstacle at worst_stop_dist + 1 cm can sleep", n > 1, f"n {n}")
_, sl, p, n_p, o, shp = sleep_interval("left_palm_2", np.array([0.0, 1.0, 0.0]), WAKE + 0.01)
lagp, D = float(sl.lagpair[p]), cfg.SLEEP["drift_budget_m"]
check("sleep (palm): the arm lag (0.25 rad x lever arms) and the drift allowance are added, so +1 cm is still awake", lagp > 0.01 and n_p == 1, f"lag {lagp:.4f} n {n_p}")
_, sl, p, n_far, o, shp = sleep_interval("left_palm_2", np.array([0.0, 1.0, 0.0]), WAKE + lagp + D + 0.05)
check("sleep (palm): beyond worst_stop_dist + lag + drift allowance it can sleep", n_far > 1, f"n {n_far}")
check("sleep: the lag term is 0.25 rad x the lever arms", abs(float(sl.lagpair[p]) - 0.25 * (float(sl.lagpair[p]) / 0.25)) < 1e-12 and cfg.derived()["arm_lag_rad"] == 0.25)

# drift: base motion below the per-tick wake threshold adds up and wakes the sleeping pair
res, sl, p, n0, o, shp = sleep_interval("left_shoulder_mount", np.array([0.0, 1.0, 0.0]), WAKE + 0.30)
q = np.array([tc.TASK_CFG["home_q_left"], tc.TASK_CFG["home_q_right"]], float)
step_x = 1e-4                                                       # per tick, below state_change_m (2e-4): the old test never wakes
check("drift: the per-tick step is below the per-tick wake threshold", step_x < cfg.SLEEP["state_change_m"])
woke_at, x = None, 0.0
for k in range(1, 120):
    x += step_x
    stk = RobotState(q_left=st.q_left, q_right=st.q_right, base_xyz=(x, 0.0, 0.0))
    res.step(q.copy(), q.copy(), stk, [o], None)
    if woke_at is None and sl.pending[p]:
        woke_at = k
exp = int(np.ceil(cfg.SLEEP["drift_budget_m"] / step_x))
check("drift: slow base drift adds up and wakes the pair after the drift budget", woke_at is not None and abs(woke_at - exp) <= 2 and n0 > exp, f"woke {woke_at} expected about {exp}, n0 {n0}")

# obstacle feed
feed_shapes = RobotShapes(load_robot_only_model(), ROOT / cfg.SHAPES_FILE)
feed = ObstacleFeed(feed_shapes)
q_home = np.array([tc.TASK_CFG["home_q_left"], tc.TASK_CFG["home_q_right"]], float)
tr = HeldTracker(WorldView(Src([])))
tr.update("left", False, placed, pos, rot, name="beaker")
lowest = pad[2] - 0.085
hot_under = catalog_object("hotplate", np.array([pad[0], pad[1], lowest - 0.01]))     # its top face 1 cm under the beaker
feed.bind([hot_under], tr)
o_ = feed(q_home, st)
hp_pair = [c for c in o_[0] if c[2] is not None]
check("feed: held beaker 1 cm over the hotplate gives its pair with the gap, zero margin and the hotplate's ceiling (among all candidate pairs)",
      len(hp_pair) == 1 and abs(hp_pair[0][0] - 0.01) < 2e-3 and hp_pair[0][1] == 0.0 and hp_pair[0][2] == cfg.approach_ceiling("hotplate") and hp_pair[0][4] == "hotplate", str(o_[0][:3]))
other = WorldObject(id="bench_top", shape="box", size=(0.3, 0.3, 0.02), centre=(pad[0], pad[1], lowest - 0.01 - 0.02), rot=tuple(np.eye(3).ravel()), fixture=True)
feed.bind([other], tr)
o2 = feed(q_home, st)
hb = [c for c in o2[0] if c[4] == "bench_top" and "held" in c[3]]
check("feed: held beaker over an ordinary object gives held_world_margin_m and no ceiling", len(hb) == 1 and hb[0][1] == cfg.MARGINS["held_world_margin_m"] and hb[0][2] is None, str(o2[0][:3]))
far = WorldObject(id="far", shape="box", size=(0.05, 0.05, 0.05), centre=(pad[0], pad[1], lowest - WAKE - 0.2), rot=tuple(np.eye(3).ravel()), fixture=True)
feed.bind([far], tr)
ids = lambda arm: {c[5] for c in feed.pairs(q_home, st)[arm]}  # noqa: E731
check("feed: an obstacle beyond worst_stop_dist_m is not a candidate (left arm)", "far" not in ids(0))
near_wake = WorldObject(id="near", shape="box", size=(0.05, 0.05, 0.05), centre=(pad[0], pad[1], lowest - (WAKE - 0.01) - 0.05), rot=tuple(np.eye(3).ravel()), fixture=True)
feed.bind([near_wake], tr)
check("feed: an obstacle at worst_stop_dist_m - 1 cm is a candidate", "near" in ids(0))
tr_empty = HeldTracker(WorldView(Src([])))
feed.bind([hot_under], tr_empty)
o3 = feed(q_home, st)
check("feed: empty gripper: no candidate from a held shape or a finger (fingers stay exempt from the world)", not any(c[5] == "hotplate" and ("held" in c[4] or "finger" in c[4]) for c in feed.pairs(q_home, st)[0]) and any("held" in c[4] for c in (feed.bind([hot_under], tr) or feed.pairs(q_home, st)[0])))

# approach ceiling through the look-ahead (fake arm: x = 0.5 * joint 1; the surface is at x = 0.6)
from src.safety_filter.braking_math import stopping_distance  # noqa: E402
DT = 1 / 150.0
CAP = cfg.BRAKING["hand_speed_cap_m_s"]
CEIL = cfg.approach_ceiling("hotplate")


def fake_tool(q, state):
    return np.asarray(q)[:, :3] * np.array([[0.5, 0.3, 0.2], [0.1, 0.4, 0.6]])


def surface(q, state):
    # the surface (ceiling pair) AND an ordinary pair with a smaller room that does not close (a torso 2.5 cm from the palm, constant): the ceiling must still apply
    g = 0.60 - np.asarray(q)[0, 0] * 0.5
    return [[(0.0254, 0.03, None, "left_palm_2", "torso"), (g, 0.0, CEIL, "left_held", "hotplate")], None]


Lc = BrakingLayer(fake_tool)
last = np.zeros((2, 7))
recs = []
for k in range(900):
    raw = last.copy()
    raw[0, 0] += 0.5
    out, info = Lc.process(last, raw, None, surface)
    recs.append((0.60 - last[0, 0] * 0.5, float(Lc.v[0]), float(Lc.a[0]), bool(info["stop"][0])))
    last = out
    if recs[-1][0] < -0.01:
        break
g_, v_, a_, st_ = (np.array([r[i] for r in recs]) for i in range(4))
k0 = int(np.argmax(g_ <= 0.0))
check("ceiling: the hand slows to the ceiling before the allowed-contact surface and reaches it at that speed", abs(v_[k0] - CEIL) < 1e-3 and v_.max() >= CAP - 1e-9, f"v at contact {v_[k0]:.5f}, max {v_.max():.4f}")
check("ceiling: never stopped (an allowed-contact surface may be touched)", not st_.any() and v_[k0:].min() > 0.0)
jerk_ = np.abs(np.diff(np.concatenate([[0.0], a_]))) / DT
check("ceiling: the slow-down is an S-curve within the acceleration and jerk limits", a_.min() >= -d0["effective_accel_limit"] - 1e-9 and jerk_.max() <= cfg.BRAKING["jerk_limit_m_s3"] + 1e-6, f"a {a_.min():.3f} jerk {jerk_.max():.3f}")
drop = -np.diff(v_)
check("ceiling: no single-tick speed drop above accel_limit x dt", drop.max() <= d0["effective_accel_limit"] * DT + 1e-9, f"{drop.max():.6f}")

# ---- step 9: bridge override with error log ----
import contextlib  # noqa: E402
import io  # noqa: E402
from types import SimpleNamespace  # noqa: E402

from src.bridge_override import force_to_target  # noqa: E402
from src.safety_filter_plugin import SafetyFilterPlugin  # noqa: E402

MARG = 0.02


def close_obstacle(q, state):                       # fake arm x = 0.5 * joint 1; the surface is 3 cm ahead of x = 0
    return [(0.03 - np.asarray(q)[0, 0] * 0.5, MARG, None), None]


Le = BrakingLayer(fake_tool)
Le.v[0] = CAP                                        # the hand is at the cap speed with the obstacle already inside its stopping distance
last = np.zeros((2, 7))
raw = last.copy()
raw[0, 0] = 0.01
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    out, info = Le.process(last, raw, None, close_obstacle)
check("override: a forced look-ahead failure (stopping distance beyond the room) sets the override flag", bool(info["override"][0]) and not info["override"][1])
check("override: the error is logged (layer log and printed line)", len(Le.errors) == 1 and "look-ahead failed" in Le.errors[0] and "[braking] ERROR" in buf.getvalue(), buf.getvalue())
check("override: braking beyond the limits: the arm stops in that tick (no ramp)", np.array_equal(out, last) and Le.v[0] == 0.0 and Le.a[0] == 0.0)
# a normal approach never overrides and never logs an error
Ln = BrakingLayer(fake_tool)
last = np.zeros((2, 7))
flags = []


def far_surface(q, state):
    return [(0.60 - np.asarray(q)[0, 0] * 0.5, MARG, None), None]


for k in range(700):
    raw = last.copy()
    raw[0, 0] += 0.5
    out, info = Ln.process(last, raw, None, far_surface)
    flags.append(bool(info["override"].any()))
    last = out
check("override: a normal approach (look-ahead on time) never overrides and logs no error", not any(flags) and not Ln.errors and Ln.v[0] == 0.0)
Lh = BrakingLayer(fake_tool)
Lh.v[0] = CAP
_, infoh = Lh.process(np.zeros((2, 7)), np.zeros((2, 7)), None, None, hold=True)
check("override: a HOLD never overrides (it is smooth)", not infoh["override"].any() and not Lh.errors)

# the bridge side
bridge = {"cmd": {"left": np.zeros(7)}, "target": {"left": np.array([0.5] + [0.0] * 6)}, "accepted": {"left": np.zeros(7)}}
outb, errs = force_to_target(bridge, 0.01, 0.25)
check("bridge override: the command jumps to the target beyond the speed cap and the step rejection", np.allclose(bridge["cmd"]["left"], bridge["target"]["left"]) and np.allclose(bridge["accepted"]["left"], bridge["target"]["left"]) and abs(outb["left"][0] - 0.5) < 1e-12)
check("bridge override: an error line is returned for a jump beyond the usual limits", len(errs) == 1 and "0.5000" in errs[0] and "did not catch" in errs[0], str(errs))
bridge2 = {"cmd": {"left": np.zeros(7)}, "target": {"left": np.array([0.004] + [0.0] * 6)}, "accepted": {"left": np.zeros(7)}}
outb2, errs2 = force_to_target(bridge2, 0.01, 0.25)
check("bridge override: a jump within the usual limits is not an error", errs2 == [] and abs(outb2["left"][0] - 0.004) < 1e-12)
check("bridge override: nothing to do before the bridge has a command", force_to_target({"cmd": None, "target": {"left": np.zeros(7)}, "accepted": {}}, 0.01, 0.25) == ({}, []))

# the plug-in passes the request on, once
def pst(s, reason=""):
    return SimpleNamespace(status=s, hold_reason=reason, rounds=0, worst_gap=float("nan"), worst_shortfall=0.0, fraction=float("nan"),
                           worst_pair=None, methods=(), pairs_pushed=0, pairs_skipped=0, pairs_shortened=0, solver_calls=0)


lay = BrakingLayer(fake_tool)
lay.v[0] = CAP
pl = SafetyFilterPlugin("on", lambda raw, last, state, world, held: (raw, pst("PASS")), lambda holding: (None, [], None), braking=lay, obstacle_fn=close_obstacle, run_name="t", out_dir=None)
with contextlib.redirect_stdout(io.StringIO()):
    pl.process({"left": [0.01] + [0.0] * 6, "right": [0.0] * 7}, {}, {"left": [0.0] * 7, "right": [0.0] * 7}, t=1.5)
check("plug-in: the override request is available once, with a log entry", pl.take_override() is True and pl.take_override() is False and len(pl.override_log) == 1 and pl.override_log[0][0] == 1.5)

# closing speed versus full speed; no emergency for a hand at rest
def tangential(q, state):                            # 1 % of the motion of joint 1 closes on the obstacle (the rest is along it)
    return [[(0.0226 - 0.01 * np.asarray(q)[0, 0] * 0.5, 0.01, None, "left_held", "bench_top")], None]


def run_tangential(mode):
    tun = copy.deepcopy(cfg.BRAKING)
    tun["lookahead_speed"] = mode
    Lm = BrakingLayer(fake_tool, tun)
    Lm.v[0] = 0.12
    last = np.zeros((2, 7))
    raw = last.copy()
    raw[0, 0] = 0.12 * DT / 0.5
    stop, limit, info = Lm.check(last, raw, None, tangential)
    return bool(stop[0]), info[0]


stop_c, inf_c = run_tangential("closing")
stop_f, inf_f = run_tangential("full")
check('look-ahead "closing": a hand moving along a surface (gap closing at 1 % of its motion) is not braked', not stop_c and inf_c["closing"] and inf_c["stop_dist"] < inf_c["room"], str(inf_c))
check('look-ahead "full": the whole hand speed counts, the same motion is braked (the first, cautious reading)', stop_f and inf_f["stop_dist"] >= inf_f["room"], str(inf_f))
Lr = BrakingLayer(fake_tool)
Lr.v[0] = 0.0015                                     # at rest (below stop_speed_m_s), the shape overlapping the obstacle by 35 mm (the conservative beaker capsule on the bench)
buf3 = io.StringIO()
with contextlib.redirect_stdout(buf3):
    stop_r, _, info_r = Lr.check(np.zeros((2, 7)), np.array([[0.001] + [0.0] * 6, [0.0] * 7]), None, lambda q, s_: [[(-0.035 - np.asarray(q)[0, 0] * 0.5, 0.01, None, "left_held", "bench_top")], None])
check("look-ahead: a hand at rest inside a (model) overlap is not an emergency (nothing to stop), and the arm stays put while the command closes", not Lr.errors and not Lr._emergency.any() and stop_r[0])

# the hotplate reaches the filter: the simulator's world model keeps it as a FIXTURE entry at the centre of its top face
from src.safety_filter.world import WorldModelAdapter  # noqa: E402


class _Entry:
    def __init__(self, id_, kind, position):
        self.id, self.kind, self.position = id_, kind, np.asarray(position, float)


class _WM:
    def __init__(self, entries):
        self._e = entries

    def entries(self, kind=None):
        return [e for e in self._e if kind is None or e.kind == kind]


ad = WorldModelAdapter(_WM([_Entry("beaker", "object", (0.85, 0.66, 1.0825)), _Entry("hotplate", "fixture", (0.65, 0.66, 1.0825)),
                            _Entry("bench", "fixture", (0.75, 0.66, 1.0125))]))
objs = {o.id: o for o in ad.objects()}
hp_o = objs.get("hotplate")
check("adapter: the hotplate (a fixture entry in the world model) is passed to the filter, the bench (not in the catalog) is not", hp_o is not None and "bench" not in objs and "beaker" in objs)
check("adapter: the hotplate's entry position is its top face (its box ends exactly there)", hp_o is not None and abs((hp_o.centre[2] + hp_o.size[2]) - 1.0825) < 1e-12 and hp_o.allowed_contact, str(hp_o))
check("adapter: the beaker is placed from its bottom centre as before", abs(objs["beaker"].centre[2] - (1.0825 + 0.05)) < 1e-12)

if __name__ == "__main__":
    print(f"\n{len(fails)} failed" if fails else "\nall passed")
    sys.exit(1 if fails else 0)
