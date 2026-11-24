"""Tests of the source-side view (SF-125 decision 1, David 2026-11-24): the sources see exactly the arm targets they sent. The loop validates arm
commands without the hard joint-limit clip; the command bridge's buffer clamp is the one place that limits joint range. Pure functions, no simulator.

    python experiments/source_view_test.py      (exit code 0 when every check passes)
"""

import os
import sys

import numpy as np

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, REPO)

from src.bridge_clamp import buffer_limits, clamp_to_buffer  # noqa: E402
from src.commands import validate_command  # noqa: E402

HARD_LO = np.array([-3.490659, -3.316125, -1.570796, 0.0, -1.570796, -0.785398, -1.570796])
HARD_HI = np.array([1.396263, 0.174533, 1.570796, 2.443461, 1.570796, 0.785398, 1.570796])
FAILS = []


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}", flush=True)
    if not ok:
        FAILS.append(name)


def main():
    raw = [0.39, -0.1, 0.27, 0.0, -0.065, 0.477, 0.227]     # joint 4 at its hard lower limit (0): inside the hard range, below its buffer
    raw[4] = -2.0                                            # joint 5 below its hard lower limit (-1.57): the case the old clip changed
    raw[1] = 0.5                                             # joint 2 above its hard upper limit (0.1745)
    cmd = {"mode": "arms", "arm_targets": {"left": list(raw)}, "gripper": {"left": 0.044}}
    limits_src = {"gripper_force_ceiling": 60.0, "gripper": (0.0, 0.044), "pedestal": (0.0, 0.42)}   # no "arm": the loop's source-side limits
    clean, notes = validate_command(cmd, limits_src)
    check("the sources see exactly the arm targets they sent (no joint-limit clip in validation)",
          clean is not None and np.array_equal(np.asarray(clean["arm_targets"]["left"], float), np.asarray(raw, float)) and not any("clipped" in n for n in notes),
          f"notes {notes}")
    limits_old = dict(limits_src, arm={"left": (HARD_LO, HARD_HI)})
    clean_old, notes_old = validate_command(cmd, limits_old)
    check("the old validation (arm limits passed) would have clipped the same command (contrast)",
          any("clipped" in n for n in notes_old) and not np.array_equal(np.asarray(clean_old["arm_targets"]["left"], float), np.asarray(raw, float)),
          f"notes {notes_old}")
    lo, hi = buffer_limits(HARD_LO, HARD_HI)
    c, m = clamp_to_buffer(np.asarray(raw, float), lo, hi)
    check("the bridge's buffer clamp limits the out-of-range joints (joints 2 and 5 clamped into the buffer)",
          m[1] and m[4] and np.all(c >= lo - 1e-12) and np.all(c <= hi + 1e-12), f"clamped joints {[j + 1 for j in range(7) if m[j]]}")
    print(f"{len(FAILS)} failed of the checks above", flush=True)
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
