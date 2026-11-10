"""SF-23 recheck: SF-16 rule check with and without the never-close list, and the left link 1 vs torso outcome.

Prints (kinematics only, nothing stepped):
  1. the flagged rows of the rule check (arm vs world M1-M9; robot vs robot P1, P3, P4, both pedestals) with the list OFF
     (every checked pair) and ON (listed pairs skipped): counts of MARGIN and BAND rows, and every MARGIN row left with the list on
  2. the never-close entries involving left link 1 and the torso, and the joint 1 values of the poses we use
Run: python experiments/safety_filter_never_close_check.py   (exit 0 always; read the output)
"""

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

from safety_filter_rule_check import evaluate, load_poses  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes  # noqa: E402

SHAPES = ROOT / "data" / "safety_filter" / "robot_shapes.json"


def main():
    poses = load_poses()
    m = load_robot_only_model()
    res = {}
    for label, use in (("list OFF", False), ("list ON", True)):
        rs = RobotShapes(m, SHAPES, use_never_close=use)
        v = evaluate(SHAPES, poses, rs=rs)
        res[label] = v
        mg = [r for r in v if r["flag"] == "MARGIN"]
        bd = [r for r in v if r["flag"] == "BAND"]
        print(f"{label}: {len(v)} flagged rows: MARGIN {len(mg)} (arm-world {sum(r['kind'] == 'arm-world' for r in mg)}, "
              f"robot-robot {sum(r['kind'] == 'robot-robot' for r in mg)}), BAND {len(bd)}", flush=True)
    print("\nMARGIN rows left with the list ON (checked pairs):")
    for r in sorted(res["list ON"], key=lambda r: r["gap"]):
        if r["flag"] == "MARGIN":
            print(f"  ped {r['ped']:.3f} {r['pose']:7s} {r['kind']:11s} {r['a']} vs {r['b']}  gap {r['gap'] * 1e3:+.1f} mm (margin {r['margin'] * 1e3:.0f})")
    print("\nrobot-vs-robot rows that the list removed (list OFF minus list ON), by pair:")
    on = {(r["pose"], r["ped"], r["a"], r["b"]) for r in res["list ON"]}
    gone = {}
    for r in res["list OFF"]:
        if r["kind"] == "robot-robot" and (r["pose"], r["ped"], r["a"], r["b"]) not in on:
            gone.setdefault((r["a"], r["b"], r["flag"]), []).append(r["gap"])
    for (a, b, f), g in sorted(gone.items()):
        print(f"  {f:6s} {a} vs {b}: {len(g)} rows, shape gap {min(g) * 1e3:+.1f} to {max(g) * 1e3:+.1f} mm")
    sw = json.loads(SHAPES.read_text(encoding="utf-8"))["never_close"]
    print("\nleft link 1 vs torso in the list:")
    for p in sw["pairs"]:
        if {p["a"], p["b"]} & {"left_link1_1", "left_link1_2", "left_link1_3"} and "torso" in (p["a"], p["b"]):
            print(f"  {p['a']} vs {p['b']}: variables {p['variables']}, step {p['steps']} {p['step_units']}, bound {p['bound_m'] * 1e3:.2f} mm, "
                  f"min real gap {p['min_gap_m'] * 1e3:.1f} mm at {p['min_pose']}")
    q1 = [p["q_left"][0] for p in poses]
    print(f"joint 1 (left) in the poses we use: min {min(q1):+.3f} rad, max {max(q1):+.3f} rad "
          f"(limits -3.491 to +1.396; the closest real gap is at +1.396 = 80 deg)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
