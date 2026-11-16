"""Safety filter: never-touch objects (SF-65). Fingers and the held object ARE checked against a never-touch object, with its own margin
(config NEVER_TOUCH_MARGIN_M, 5 cm); an ordinary object is unchanged (fingers and held exempt, arm_world_m). No simulation stepping.
Run: python experiments/safety_filter_never_touch_test.py   (exit 0 = pass)"""
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes, RobotState  # noqa: E402
from src.safety_filter.touch import Exempt, HeldTracker, Rule, as_placed, evaluate_pairs, gap_between, pair_rule  # noqa: E402
from src.safety_filter.world import WorldObject, WorldView, catalog_object  # noqa: E402

fails = []


def check(name, ok, info=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  {info}" if info and not ok else ""))
    if not ok:
        fails.append(name)


rs = RobotShapes(load_robot_only_model(), ROOT / "data" / "safety_filter" / "robot_shapes.json")
st = RobotState(pedestal_m=0.155, q_left=(0.394, -0.257, 0.271, 2.201, -0.065, 0.477, 0.227), q_right=(-0.394, 0.257, -0.271, 2.201, 0.065, -0.477, -0.227))
placed = rs.place(st)
finger = next(p for p in placed if p.part == "finger" and p.arm == "left")
palm = next(p for p in placed if p.part == "palm" and p.arm == "left")


def cat_at(shape, gap):
    """A never-touch capsule whose surface is `gap` m from the shape: the cat's centre is moved along +x from the shape's centre (bisection)."""
    c = (shape.a + shape.b) / 2
    mk = lambda t: WorldObject(id="cat", shape="capsule", size=(0.10, 0.05), centre=tuple(c + np.array([t, 0, 0])), rot=tuple(np.eye(3).ravel()), never_touch=True)  # noqa: E731
    lo, hi = 0.0, 1.0
    for _ in range(60):
        mid = (lo + hi) / 2
        if gap_between(shape, as_placed(mk(mid)))[0] < gap:
            lo = mid
        else:
            hi = mid
    return mk((lo + hi) / 2)


check("config: never-touch margin is 0.05 m, arm_world stays 0.01 m", cfg.NEVER_TOUCH_MARGIN_M == 0.05 and cfg.MARGINS["arm_world_m"] == 0.01)
check("WorldObject default: may be touched", WorldObject("x", "box", (1, 1, 1), (0, 0, 0), tuple(np.eye(3).ravel())).never_touch is False)

cat = cat_at(finger, 0.04)
ordinary = replace(cat, never_touch=False, id="thing")
r_f = pair_rule(finger, as_placed(cat))
check("finger vs never-touch: checked (Rule) with margin 0.05", isinstance(r_f, Rule) and r_f.margin == 0.05, str(r_f))
check("finger vs ordinary object: still exempt", isinstance(pair_rule(finger, as_placed(ordinary)), Exempt))
r_p = pair_rule(palm, as_placed(cat))
check("palm vs never-touch: margin 0.05 (not 0.01)", isinstance(r_p, Rule) and r_p.margin == 0.05)
check("palm vs ordinary: margin 0.01 as before", pair_rule(palm, as_placed(ordinary)).margin == 0.01)

rep = [r for r in evaluate_pairs([finger], [cat]) if r.b == "cat" or r.a == "cat"]
check("evaluate_pairs: finger 4 cm from the cat is in margin", len(rep) == 1 and rep[0].in_margin and abs(rep[0].gap - 0.04) < 1e-6, str(rep))
check("evaluate_pairs: the same finger vs an ordinary object gives no pair", evaluate_pairs([finger], [ordinary]) == [])
wake = cfg.derived()["worst_stop_dist_m"]                      # Part 2: activation = max(6 cm, the worst stopping distance)
far = cat_at(finger, max(0.06, wake) + 0.015)
check("evaluate_pairs: a finger 1.5 cm beyond the (widened) activation distance is not an active pair", evaluate_pairs([finger], [far]) == [])
mid = cat_at(palm, 0.055)
rep = evaluate_pairs([palm], [mid])
check("evaluate_pairs: palm 5.5 cm away is an active pair but not in margin", len(rep) == 1 and not rep[0].in_margin)

# held object: the beaker at the pads, then a never-touch cat 3 cm from it
pad = np.mean([(p.a + p.b) / 2 for p in placed if p.part == "finger" and p.arm == "left"], axis=0)
beaker = catalog_object("beaker", pad - np.array([0, 0, 0.05]))


class Src:
    def objects(self):
        return [beaker]


view = WorldView(Src())
ht = HeldTracker(view)
pos, rot = rs.hand_frame("left")
ev = ht.update("left", True, placed, pos, rot)
held = ht.placed("left", pos, rot)
check("held beaker attached", ev == "attach" and held is not None)
cat_h = cat_at(held, 0.03)
r_h = pair_rule(held, as_placed(cat_h))
check("held beaker vs never-touch: checked with margin 0.05", isinstance(r_h, Rule) and r_h.margin == 0.05, str(r_h))
r_o = pair_rule(held, as_placed(replace(cat_h, never_touch=False)))
check("held beaker vs ordinary object: checked since Part 2 (held-world margin, not exempt)", isinstance(r_o, Rule) and r_o.kind == "held-world", str(r_o))
rep = evaluate_pairs([held], [cat_h])
check("evaluate_pairs: held beaker 3 cm from the cat is in margin", len(rep) == 1 and rep[0].in_margin)

print(f"\n{'ALL PASS' if not fails else 'FAILED: ' + ', '.join(fails)}")
sys.exit(1 if fails else 0)
