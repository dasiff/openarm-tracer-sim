"""Tests of the bridge's joint-buffer clamp (SF-122 stage 1; SF-125 range-based margins): every joint target is clamped into [hard lower + margin_j, hard upper - margin_j];
an inside target passes unchanged exactly; an outside target is clamped to the buffer edge; one clamped joint does not change the others.

    python experiments/bridge_clamp_test.py      (exit code 0 when every check passes)
"""

import os
import sys

import numpy as np

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, REPO)

from src.bridge_clamp import buffer_limits, buffer_margins, clamp_to_buffer  # noqa: E402
from src.task_config import BRIDGE_BUFFER_CFG, MINK_CFG  # noqa: E402

# hard joint ranges of the left arm (models/robot/combined_robot.xml)
HARD_LO = np.array([-3.490659, -3.316125, -1.570796, 0.0, -1.570796, -0.785398, -1.570796])
HARD_HI = np.array([1.396263, 0.174533, 1.570796, 2.443461, 1.570796, 0.785398, 1.570796])
FAILS = []


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}", flush=True)
    if not ok:
        FAILS.append(name)


def main():
    # SF-125 (David 2026-11-24): the margin of each joint is min(5 % of its range, 10 degrees) (BRIDGE_BUFFER_CFG mode "range"); these two checks
    # were the SF-122 fixed 10-degree checks and are replaced (listed in the report).
    lo, hi = buffer_limits(HARD_LO, HARD_HI)
    margins = buffer_margins(HARD_LO, HARD_HI)
    expected = np.minimum(0.05 * (HARD_HI - HARD_LO), np.radians(10.0))
    check("the margin of each joint is min(5 % of its range, 10 degrees) (SF-125)", np.allclose(margins, expected, atol=1e-12, rtol=0),
          f"margins (deg) {np.round(np.degrees(margins), 2).tolist()}")
    check("buffer edges are the hard limits moved in by each joint's margin",
          np.allclose(lo, HARD_LO + margins, atol=0, rtol=0) and np.allclose(hi, HARD_HI - margins, atol=0, rtol=0), "")
    saved = dict(BRIDGE_BUFFER_CFG)
    BRIDGE_BUFFER_CFG["mode"] = "fixed"
    fixed_m = buffer_margins(HARD_LO, HARD_HI)
    BRIDGE_BUFFER_CFG.update(saved)
    check("mode fixed gives the 10-degree margin (MINK_CFG joint_margin_rad) on every joint",
          np.allclose(fixed_m, float(MINK_CFG["joint_margin_rad"]), atol=1e-12, rtol=0) and abs(float(MINK_CFG["joint_margin_rad"]) - np.radians(10.0)) < 1e-12, "")

    inside = np.array([0.394, -0.1, 0.27, 1.0, -0.065, 0.477, 0.227])
    c, m = clamp_to_buffer(inside, lo, hi)
    check("a target inside every buffer passes unchanged exactly", np.array_equal(c, inside) and not m.any(), f"mask {m.astype(int).tolist()}")

    outside = inside.copy()
    outside[3] = 0.0                                    # joint 4 at its hard lower limit, below the buffer
    outside[1] = 0.5                                    # joint 2 above its hard upper limit
    c, m = clamp_to_buffer(outside, lo, hi)
    check("a target below the buffer is clamped to the lower buffer edge", c[3] == lo[3] and m[3], f"joint 4 -> {c[3]:.4f} (edge {lo[3]:.4f})")
    check("a target above the buffer is clamped to the upper buffer edge", c[1] == hi[1] and m[1], f"joint 2 -> {c[1]:.4f} (edge {hi[1]:.4f})")

    one = inside.copy()
    one[5] = 2.0                                        # joint 6 far above its buffer (only this joint is clamped)
    c, m = clamp_to_buffer(one, lo, hi)
    others = [j for j in range(7) if j != 5]
    check("one clamped joint does not change the others (exact)", np.array_equal(c[others], one[others]) and m.tolist() == [j == 5 for j in range(7)],
          f"clamped joints {[j + 1 for j in range(7) if m[j]]}")

    at_edge = lo.copy()
    c, m = clamp_to_buffer(at_edge, lo, hi)
    check("a target exactly at the buffer edge is not counted as clamped", np.array_equal(c, at_edge) and not m.any(), "")

    print(f"{len(FAILS)} failed of the checks above", flush=True)
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
