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

if __name__ == "__main__":
    print(f"\n{len(fails)} failed" if fails else "\nall passed")
    sys.exit(1 if fails else 0)
