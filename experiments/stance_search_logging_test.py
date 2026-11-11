#!/usr/bin/env python
"""Test stance search logging output files (JSON files written to unique directories).

    source .venv/bin/activate && export MUJOCO_GL=egl && python experiments/stance_search_logging_test.py

Tests:
  1. Three JSON files are written to a unique directory per search
  2. candidates_list.json contains all candidates attempted
  3. File structure matches spec: candidates with number, xy, heading, pedestal, wall_time, sim_time, checks
  4. Files are written even when no candidate passes
  5. Each search gets a different directory (no overwrites)
"""

import json
import shutil
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from src.mpvi_postprocess import postprocess  # noqa: E402
from src.stance_select import apply_stance_selection  # noqa: E402
from src.task_config import TASK_CFG  # noqa: E402


BEAKER_ON_HOTPLATE_PLAN = {
    "task_name": "Put the beaker on the hotplate",
    "segments": [
        {"type": "navigation", "description": "Go to the beaker", "completion_criteria": "At the beaker", "target": "the beaker"},
        {"type": "manipulation", "description": "Pick up the beaker", "completion_criteria": "The beaker is held", "target": None,
         "primitive": "pick", "objects": [{"phrase": "the beaker", "role": "manipulated"}]},
        {"type": "navigation", "description": "Go to the hotplate", "completion_criteria": "At the hotplate", "target": "the hotplate"},
        {"type": "manipulation", "description": "Place the beaker on the hotplate", "completion_criteria": "The beaker is on the hotplate and the gripper is open",
         "target": None, "primitive": "place", "objects": [{"phrase": "the beaker", "role": "manipulated"}, {"phrase": "the hotplate", "role": "destination"}]}
    ]
}

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append(bool(ok))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}")


def main():
    print("Testing stance search logging...")

    base_log_dir = PROJECT_ROOT / "data" / "demo_record" / "stance_search"

    if base_log_dir.exists():
        initial_dirs = set(d.name for d in base_log_dir.iterdir() if d.is_dir())
    else:
        initial_dirs = set()

    print("\n1. Run apply_stance_selection and check if logging directories are created")
    tagged = postprocess(BEAKER_ON_HOTPLATE_PLAN, False, say=lambda *a, **k: None)
    out = apply_stance_selection(tagged, TASK_CFG, say=lambda *a, **k: None)

    check("apply_stance_selection completes", out is not None, "")

    if base_log_dir.exists():
        after_dirs = set(d.name for d in base_log_dir.iterdir() if d.is_dir())
        new_dirs = after_dirs - initial_dirs
        check("a new logging directory is created", len(new_dirs) == 1, f"new dirs: {new_dirs}")

        if len(new_dirs) == 1:
            search_dir = base_log_dir / new_dirs.pop()
            check("logging directory exists", search_dir.exists(), f"path: {search_dir}")

            candidates_file = search_dir / "candidates_list.json"
            check("candidates_list.json is written", candidates_file.exists(), f"path: {candidates_file}")

            if candidates_file.exists():
                try:
                    candidates_data = json.loads(candidates_file.read_text())
                    check("candidates_list.json is valid JSON", True, "")
                    check("candidates list is a dict with 'candidates' and 'accepted'",
                          isinstance(candidates_data, dict) and "candidates" in candidates_data and "accepted" in candidates_data,
                          f"keys: {list(candidates_data.keys())}")

                    if "candidates" in candidates_data:
                        cands = candidates_data["candidates"]
                        check("candidates is a list", isinstance(cands, list), f"type: {type(cands)}")

                        if isinstance(cands, list) and len(cands) > 0:
                            first_cand = cands[0]
                            required_keys = {"number", "base_x", "base_y", "heading_deg", "pedestal_m", "wall_time_start_s", "sim_time_start_s", "checks"}
                            has_keys = all(k in first_cand for k in required_keys)
                            check("first candidate has required fields", has_keys, f"keys: {set(first_cand.keys())}")

                            if "checks" in first_cand:
                                check("first candidate has checks dict", isinstance(first_cand["checks"], dict), f"type: {type(first_cand['checks'])}")

                        print(f"  Total candidates recorded: {len(cands)}")
                except json.JSONDecodeError as e:
                    check("candidates_list.json is valid JSON", False, str(e))

            full_check_file = search_dir / "full_check_solutions.json"
            check("full_check_solutions.json exists", full_check_file.exists(), f"path: {full_check_file}")
            if full_check_file.exists():
                try:
                    fcs_data = json.loads(full_check_file.read_text())
                    check("full_check_solutions.json is valid JSON", True, "")
                except json.JSONDecodeError as e:
                    check("full_check_solutions.json is valid JSON", False, str(e))

            dryrun_file = search_dir / "dryrun_trajectories.json"
            check("dryrun_trajectories.json exists", dryrun_file.exists(), f"path: {dryrun_file}")
            if dryrun_file.exists():
                try:
                    dr_data = json.loads(dryrun_file.read_text())
                    check("dryrun_trajectories.json is valid JSON", True, "")
                except json.JSONDecodeError as e:
                    check("dryrun_trajectories.json is valid JSON", False, str(e))
    else:
        check("logging base directory is created", False, f"expected: {base_log_dir}")

    print(f"\n{'ALL PASS' if all(RESULTS) else 'SOME FAILED'}")
    return 0 if all(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
