"""Analytic tests of the per-task approach-angle lists (David, 2026-11-08; decision M279). No simulator: the rules are checked on lists and points.

    source .venv/bin/activate && python experiments/approach_angle_list_test.py     (exit 0 when every check passes)

What is checked:
  1. A stance with two pick tasks that have different passing lists carries both lists, and passes when both have an angle.
  2. The choosing function returns the first angle of a list.
  3. A missing list and an empty list each give their own reason (approach_angle.ApproachAngleError), so the step does not run.
  4. Place recovery backs out along the place's own approach angle: the recovery and the place plan use one function (approach_angle.place_approach_xy).
  5. The check groups the points by task (full_check._task_of, _ordered_tasks): two pick steps and a place are three separate tasks.
Not checked here (needs the simulator): that a failed skill does not move the arm (mpvi_modules.start with an empty list); see the report.
"""
import math
import re
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np  # noqa: E402

from src import approach_angle as aa  # noqa: E402
from src.full_check import _ordered_tasks, _task_of  # noqa: E402

results = []


def check(name, ok, detail=""):
    results.append(bool(ok))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}", flush=True)


def main():
    # 1. two pick tasks with different lists: both carried, the stance passes when both have an angle
    lists = [{"task": 2, "kind": "pick", "angles": [3, 4]}, {"task": 5, "kind": "pick", "angles": [7]}]
    ok, failing = aa.combine_task_lists(lists)
    check("two pick tasks with different lists: the stance passes and carries both lists", ok and failing == [] and lists[0]["angles"] == [3, 4] and lists[1]["angles"] == [7],
          f"ok {ok}, failing {failing}")
    lists2 = [{"task": 2, "kind": "pick", "angles": [3]}, {"task": 5, "kind": "pick", "angles": []}]
    ok2, failing2 = aa.combine_task_lists(lists2)
    check("one task with an empty list fails the stance and names that task", (not ok2) and failing2 == [5], f"ok {ok2}, failing {failing2}")
    place_lists = [{"task": 9, "kind": "place", "angles": [0.0, 5.0]}, {"task": 2, "kind": "pick", "angles": [4]}]
    check("a place task with angles and a pick task with one angle: the stance passes", aa.combine_task_lists(place_lists)[0])

    # 2. the choosing function returns the first angle
    check("the choice from [7, 3] is 7 (the first passing angle)", aa.choose_approach_angle([7, 3]) == 7)
    check("the choice from [0.0, 5.0] is 0.0 (a place's first passing turn)", aa.choose_approach_angle([0.0, 5.0]) == 0.0)
    try:
        aa.choose_approach_angle([])
        check("choosing from an empty list raises", False)
    except aa.ApproachAngleError as e:
        check("choosing from an empty list raises with reason 'empty'", e.reason == aa.REASON_EMPTY, str(e))

    # 3. missing and empty lists give distinct reasons; a good list is returned as given
    try:
        aa.task_approach_angles({"primitive": "pick", "description": "Pick up the beaker"})
        check("a segment without a list raises", False)
    except aa.ApproachAngleError as e:
        check("a missing list gives reason 'missing'", e.reason == aa.REASON_MISSING and "no approach-angle list" in str(e), str(e))
    try:
        aa.task_approach_angles({"primitive": "place", "approach_angles": []})
        check("an empty list raises", False)
    except aa.ApproachAngleError as e:
        check("an empty list gives reason 'empty' (a different reason from 'missing')", e.reason == aa.REASON_EMPTY and "empty approach-angle list" in str(e), str(e))
    got = aa.task_approach_angles({"primitive": "pick", "approach_angles": [11, 0]})
    check("a good list is returned unchanged (no re-check, no fallback)", got == [11, 0], str(got))

    # 4. place recovery and the place plan turn the forward direction by the same function, by the place's own angle
    f = np.array([1.0, 0.0, 0.0])
    th = math.radians(30.0)
    direct = np.array([math.cos(th), math.sin(th), 0.0])
    check("the place direction is the forward turned by the place's angle (30 deg)", np.allclose(aa.place_approach_xy(f, th), direct), str(aa.place_approach_xy(f, th)))
    check("a place at 0 deg backs out along the forward direction", np.allclose(aa.place_approach_xy(f, 0.0), f))
    src = (PROJECT_ROOT / "src" / "mpvi_modules.py").read_text()
    uses = len(re.findall(r"place_approach_xy\(", src))
    check("the place plan and the place recovery both call place_approach_xy (one direction for both)", uses >= 2, f"calls in mpvi_modules: {uses}")

    # 5. the check groups its points by task
    pts = [{"p": [0, 0, 0], "pick_path": {"task": 2}}, {"p": [0, 0, 0], "pick_path": {"task": 2}}, {"p": [1, 0, 0], "pick_path": {"task": 5}},
           {"p": [2, 0, 0], "pose": "place", "task": 9}, {"p": [2, 0, 0], "pose": "place", "task": 9}]
    check("points of two pick steps and one place step form three tasks, in order", _ordered_tasks(pts) == [2, 5, 9], str(_ordered_tasks(pts)))
    check("a pick point's task is its pick_path's step, a place point's task is its own step", _task_of(pts[2]) == 5 and _task_of(pts[3]) == 9)

    print(f"\n{sum(results)}/{len(results)} checks passed")
    return all(results)


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
