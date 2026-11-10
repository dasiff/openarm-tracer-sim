"""Tests of the trajectory dry-run (decisions-log M302; src/trajectory_dryrun.py). No physics is stepped.

    source .venv/bin/activate && MUJOCO_GL=egl python experiments/trajectory_dryrun_test.py     (exit 0 when every check passes)

What is checked:
  1. The staging stance that missed OVER in the run (seed 2680074893, draw 2; approach bin 10, the bin the OVER waypoint matched in the replay) fails the dry-run at
     pick OVER: the commanded pad leaves its reference by more than DRYRUN_CFG ref_tol_m. Its OVER duration is the planner's own (8.33 s; the run planned 8.3 s).
  2. A move to the pad's own pose passes (the pass path: reached, settled, within the reference tolerance).
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from src.full_check import FullCheck  # noqa: E402
from src.task_config import DRYRUN_CFG, PICK_PLACE_CFG, TASK_CFG  # noqa: E402
from src import trajectory_dryrun as td  # noqa: E402

FAILS = []


def check(name, ok, detail=""):
    if not ok:
        FAILS.append(name)
    print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}")


def main():
    fc = FullCheck(task_cfg=TASK_CFG, say=lambda *a, **k: None)
    stance = (0.9860, 1.0704, math.radians(-101.8), 0.1056)
    res = td.dry_run_stance(fc.chk, fc.model, stance, TASK_CFG["beaker_xy"], pick_k=10, place_theta_deg=0.0)
    over = res["subtasks"][0] if res["subtasks"] else {}
    check("the staging stance that missed OVER fails the dry-run at pick OVER", (not res["ok"]) and res["failed"] == "pick OVER" and "left its reference" in res["reason"],
          f"failed {res['failed']}: {res['reason']}")
    check("the OVER duration is the planner's (8.33 s; the run planned 8.3 s)", abs(over.get("T_s", 0) - 8.33) < 0.05, f"T {over.get('T_s', 0):.3f} s")
    check("the reference tolerance is the proposed 30 mm (DRYRUN_CFG)", DRYRUN_CFG["ref_tol_m"] == 0.03, "")

    # the pass path: a move to the pad's own pose (no move to make)
    arm_home = np.asarray(TASK_CFG["home_q_left"], float)
    chk = fc.chk
    chk.place(stance)
    d = chk.d
    mujoco.mj_kinematics(fc.model, d)
    from src.mink_arm import MinkArm, MINK_CFG  # noqa: E402
    arm = MinkArm(fc.model, d, "left", arm_home, MINK_CFG)
    arm.q_cmd = arm_home.copy()
    td._track(fc.model, d, arm)
    p0, R0 = td._commanded_pad(fc.model, d, arm, arm.q_cmd)
    rec, ok, why = td._move(fc.model, d, arm, "same pose", p0, R0, PICK_PLACE_CFG["slow_speed_m_s"], PICK_PLACE_CFG)
    check("a move to the pad's own pose passes (reached and settled)", ok and rec["final_mm"] < 3.0, f"ok {ok}, final {rec['final_mm']:.2f} mm, {rec['ticks']} ticks")
    print()
    print("FAILED: " + ", ".join(FAILS) if FAILS else "ALL PASS")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
