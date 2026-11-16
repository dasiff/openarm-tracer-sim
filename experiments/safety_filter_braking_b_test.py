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

HOT = catalog_object("hotplate", np.array([1.0, 0.0, 0.80]))               # bottom centre at z 0.80: top face at z 0.86
hot_top = 0.80 + 0.06
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

if __name__ == "__main__":
    print(f"\n{len(fails)} failed" if fails else "\nall passed")
    sys.exit(1 if fails else 0)
