"""Braking Phase H fix 1 (SF-87): the BACK move after the release lifts first, then retreats level above the release height.
Geometry only (no simulator): mpvi_modules.place_back_waypoints. Run: python experiments/place_back_lift_test.py (exit 0 = all pass).
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.mpvi_modules import place_back_waypoints  # noqa: E402
from src.task_config import PICK_PLACE_CFG as CFG  # noqa: E402

fails = []


def check(name, ok, info=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  [{info}]" if info else ""))
    if not ok:
        fails.append(name)


down = np.array([0.30, -0.10, 0.25])               # the release point (pad centre over the plate, at the set-down height)
a = np.array([1.0, 0.0, 0.0])                       # the approach direction (unit, xy)
lift, back = place_back_waypoints(down, a, CFG)

check("fix 1: LIFT is straight up from the release point (same xy, same orientation step)",
      np.allclose(lift[:2], down[:2]) and abs(lift[2] - (down[2] + CFG["back_lift_m"])) < 1e-12, f"lift {np.round(lift, 4)}")
check("fix 1: the lift height is the configured back_lift_m (SF-87)", abs((lift[2] - down[2]) - CFG["back_lift_m"]) < 1e-12)
check("fix 1: BACK is level at the lifted height (no vertical component in the retreat)", abs(back[2] - lift[2]) < 1e-12, f"back z {back[2]:.4f}")
retreat = np.linalg.norm(back[:2] - lift[:2])
check("fix 1: BACK retreats the configured distance along the approach direction (away from the release point)",
      abs(retreat - CFG["retreat_m"]) < 1e-12 and np.dot(back[:2] - lift[:2], a[:2]) < 0, f"retreat {retreat:.4f} m")
check("fix 1: the palm never goes below its release height during BACK (lift and retreat are both at or above it)",
      min(lift[2], back[2]) >= down[2] - 1e-12)
check("fix 1: the lift is above the release point by a positive amount (the arm lifts before it retracts)",
      lift[2] > down[2] and CFG["back_lift_m"] > 0.0)

print(f"{len(fails)} failing check(s)" if fails else "ALL PASS")
sys.exit(1 if fails else 0)
