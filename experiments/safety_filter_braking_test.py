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

if __name__ == "__main__":
    print(f"\n{len(fails)} failed" if fails else "\nall passed")
    sys.exit(1 if fails else 0)
