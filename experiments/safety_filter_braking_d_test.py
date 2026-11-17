"""Braking Phase D (David's fixes after the Phase C review): unit tests per fix.

Fix 1: work surfaces (the bench top). Run: python experiments/safety_filter_braking_d_test.py  (exit 0 = all pass).
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src import task_config as tc  # noqa: E402
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.braking import BrakingLayer, ObstacleFeed, make_tool_fn, stop_gap  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes, RobotState  # noqa: E402
from src.safety_filter.touch import Exempt, Rule, as_placed, pair_rule, wake_dist  # noqa: E402
from src.safety_filter.world import LabMapSource, WorldObject  # noqa: E402

fails = []


def check(name, ok, info=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  {info}" if info and not ok else ""))
    if not ok:
        fails.append(name)


# ---- fix 1: work surfaces (David: the bench gets the flag; push-stop PUSH_STOP_BEYOND_M_WORK = 0.005 m; stop target 1.5 cm; margin unchanged) ----
check("fix 1: PUSH_STOP_BEYOND_M_WORK is 0.005 m", abs(cfg.PUSH_STOP_BEYOND_M_WORK - 0.005) < 1e-12)
check("fix 1: the arm-world margin is unchanged (1 cm)", abs(cfg.MARGINS["arm_world_m"] - 0.01) < 1e-12)
check("fix 1: OBJECT_CATALOG flags the bench top as a work surface", cfg.OBJECT_CATALOG["bench_top"].get("work_surface") is True)
check("fix 1: the hotplate and the beaker are not work surfaces",
      not cfg.OBJECT_CATALOG["hotplate"].get("work_surface", False) and not cfg.OBJECT_CATALOG["beaker"].get("work_surface", False))
fx = {o.id: o for o in LabMapSource(ROOT / "data/safety_filter/lab_map.json").objects()}
check("fix 1: the lab map's bench_top fixture carries the flag, and no other fixture does",
      fx["bench_top"].work_surface is True and not any(o.work_surface for k, o in fx.items() if k != "bench_top"))
check("fix 1: the catalog's bench half sizes equal the lab map's", tuple(cfg.OBJECT_CATALOG["bench_top"]["half_m"]) == tuple(fx["bench_top"].size), f"{fx['bench_top'].size}")
check("fix 1: a work surface's stop target is margin + 0.005 = 1.5 cm", abs(stop_gap(0.01, None, True) - 0.015) < 1e-12)
check("fix 1: a plain obstacle keeps margin + PUSH_STOP_BEYOND_M (3 cm)", abs(stop_gap(0.01, None) - 0.03) < 1e-12 and abs(stop_gap(0.01, None, False) - 0.03) < 1e-12)
check("fix 1: an allowed-contact surface keeps its margin boundary (the work flag does not change it)", stop_gap(0.0, "hotplate", True) == 0.0)

rs = RobotShapes(load_robot_only_model(), ROOT / cfg.SHAPES_FILE)
q_home = np.array([tc.TASK_CFG["home_q_left"], tc.TASK_CFG["home_q_right"]], float)
st_home = RobotState(q_left=tuple(q_home[0]), q_right=tuple(q_home[1]))
placed = rs.place(st_home)
palm_caps = [p for p in placed if p.arm == "left" and p.part == "palm"]
palm_low = min(min(p.a[2], p.b[2]) - p.r for p in palm_caps)              # the palm's lowest point at home
px = float(np.mean([p.a[0] for p in palm_caps]))
py = float(np.mean([p.a[1] for p in palm_caps]))
TOP_GAP0 = 0.05                                                            # m: the bench top starts 5 cm below the palm's lowest point


def bench(work):
    """A box (the bench top's shape) under the palm: its top face is TOP_GAP0 below the palm's lowest point."""
    zc = palm_low - TOP_GAP0 - 0.06
    return WorldObject(id="bench_top", shape="box", size=(0.5, 0.5, 0.06), centre=(px, py, zc), rot=tuple(np.eye(3).ravel()),
                       fixture=True, work_surface=work)


b_work = as_placed(bench(True))
r_palm = pair_rule(palm_caps[0], b_work)
check("fix 1: palm vs bench top is an arm-world pair with the 1 cm margin and the work flag",
      isinstance(r_palm, Rule) and r_palm.kind == "arm-world" and r_palm.work_surface and abs(r_palm.margin - 0.01) < 1e-12, str(r_palm))
check("fix 1: a finger is still exempt from the bench (the finger rule is unchanged)",
      isinstance(pair_rule(next(p for p in placed if p.arm == "left" and p.part == "finger"), b_work), Exempt))
feed0 = ObstacleFeed(rs)
feed0.bind([bench(True)], None)
pc = [c for c in feed0.pairs(q_home, st_home)[0] if c[5] == "bench_top" and c[4].startswith("left_palm")]
check("fix 1: the palm's candidate against the bench carries the 1.5 cm stop target (8th value, index 8 of pairs)",
      len(pc) >= 1 and abs(pc[0][8] - 0.015) < 1e-12, str(pc[:1]))
check("fix 1: at home the palm is 5 cm above the test bench top (geometry as built)", len(pc) >= 1 and abs(min(c[1] for c in pc) - TOP_GAP0) < 1e-4,
      f"{min(c[1] for c in pc) if pc else float('nan')}")


def palm_gap(q, feed):
    """The smallest gap of a left palm shape to the bench top at pose q (m)."""
    cands = [c for c in feed.pairs(q, st_home)[0] if c[5] == "bench_top" and c[4].startswith("left_palm")]
    return min(c[1] for c in cands)


def lowest_joint(feed):
    """The left joint whose + motion lowers the palm's gap to the bench fastest at home: (joint index, d gap / d rad)."""
    eps = 1e-4
    g0 = palm_gap(q_home, feed)
    best = None
    for j in range(7):
        qp = q_home.copy()
        qp[0, j] += eps
        d = (palm_gap(qp, feed) - g0) / eps
        if best is None or d < best[1]:
            best = (j, d)
    return best


def descend(work, speed, ticks=900):
    """A source that drives the left palm straight down toward the bench top at `speed` (m/s, at the palm) through the braking layer.
    The source keeps commanding past the target (a source that does not stop); the layer decides where the hand stops."""
    feed = ObstacleFeed(rs)
    feed.bind([bench(work)], None)
    j, dg = lowest_joint(feed)
    rate = speed / (abs(dg) * cfg.BRAKING["tick_hz"])                    # rad per tick of joint j, so the palm moves at about `speed`
    lay = BrakingLayer(make_tool_fn(rs))
    last = q_home.copy()
    gaps, brk, outs = [], [], []
    for k in range(ticks):
        raw = q_home.copy()
        raw[0, j] += rate * (k + 1)
        out, info = lay.process(last, raw, st_home, feed)
        gaps.append(palm_gap(out, feed))
        brk.append(bool(info["stop"][0]))                    # the left arm is held to rest by the look-ahead this tick
        outs.append(out.copy())
        last = out
    return {"gaps": np.array(gaps), "brk": np.array(brk), "outs": np.array(outs), "errors": list(lay.errors), "joint": j, "dg": dg}


R_fast = descend(True, 0.04)
check("fix 1: a fast palm (4 cm/s) stops at the 1.5 cm target: final palm gap in [14.5, 16.0] mm",
      0.0145 <= R_fast["gaps"][-1] <= 0.0160, f"final {R_fast['gaps'][-1] * 1000:.2f} mm (joint {R_fast['joint']}, d gap/d rad {R_fast['dg']:.3f})")
check("fix 1: the fast palm passes the 2.3 cm grasp pose on its way (it is not held at 3 cm)", R_fast["gaps"].min() <= 0.0235, f"min {R_fast['gaps'].min() * 1000:.2f} mm")
check("fix 1: once stopped the palm stays still (no creep over the last 200 ticks)", np.max(np.abs(np.diff(R_fast["outs"][-200:], axis=0))) < 1e-9,
      f"{np.max(np.abs(np.diff(R_fast['outs'][-200:], axis=0)))}")
check("fix 1: no look-ahead error in the fast work-surface descent", not R_fast["errors"], str(R_fast["errors"][:1]))

R_slow = descend(True, 0.01)
first = np.flatnonzero(R_slow["brk"])
first_gap = float(R_slow["gaps"][first[0]]) if first.size else float("nan")
# FINDING F-D1 (not asserted here, see docs/progress.md): the free-motion ramp takes its target speed from the joint lag (commanded minus last),
# so a slow approach overshoots the command when the output catches up with a lag (2.7x the commanded step), then the "never past the commanded
# pose" clip stops it abruptly (acceleration -1.4 m/s2, limit 0.5). The look-ahead then latches a brake at the first brake gap below, above the grasp pose.
print(f"INFO F-D1: slow palm (1 cm/s) first brake at {first_gap * 1000:.2f} mm (the grasp pose is 23 mm; not asserted until F-D1 is decided)")
check("fix 1: the slow palm reaches 2.3 cm and stops at 1.5 cm", R_slow["gaps"].min() <= 0.0235 and 0.0145 <= R_slow["gaps"][-1] <= 0.0160,
      f"min {R_slow['gaps'].min() * 1000:.2f} mm, final {R_slow['gaps'][-1] * 1000:.2f} mm")
check("fix 1: no look-ahead error in the slow work-surface descent", not R_slow["errors"], str(R_slow["errors"][:1]))

R_plain = descend(False, 0.04)
check("fix 1: control: the same fast descent with no work flag stops at 3 cm (the flag is what moves the target)",
      0.0290 <= R_plain["gaps"][-1] <= 0.0315, f"final {R_plain['gaps'][-1] * 1000:.2f} mm")
check("fix 1: control: that palm never reaches the 2.3 cm grasp pose", R_plain["gaps"].min() >= 0.0295, f"min {R_plain['gaps'].min() * 1000:.2f} mm")

print(f"{len(fails)} failing check(s)" if fails else "ALL PASS")
sys.exit(1 if fails else 0)
