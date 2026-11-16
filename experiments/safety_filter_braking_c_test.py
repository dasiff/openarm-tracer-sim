"""Braking Phase C (David's fixes after the Phase B review): unit tests per step.
Run: python experiments/safety_filter_braking_c_test.py   (exit 0 = pass)"""
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

if __name__ == "__main__":
    print(f"\n{len(fails)} failed" if fails else "\nall passed")
    sys.exit(1 if fails else 0)
