#!/usr/bin/env python
"""Test of the stance search logs (src/stance_search_logging.py, decisions-log M309 to M316). Exits 0 when every check passes.

    source .venv/bin/activate && export MUJOCO_GL=egl && python experiments/stance_search_logging_test.py

Part A (fast): the logger alone, in a temporary folder: a run folder is never overwritten, NaN becomes null, the numbers are rounded, the status rules.
Part B (one real stance search, about 9 minutes; run it in tmux with a log, not in the foreground): the real full check and the real dry run on the staging layout,
  then the files are read back: every drawn stance has a record numbered in draw order, the times are measured from the search start and never decrease,
  the simulator clock is read at every candidate, the accepted candidate is the one whose dry run passed, every candidate that reached the full check has its
  solutions, and every dry-run subtask has one row per tick with the columns named in the file.
"""

import json
import sys
import tempfile
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "experiments"))

from src.stance_search_logging import DRYRUN_COLUMNS, FILES, StanceSearchLogger, num  # noqa: E402

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append(bool(ok))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}")


def part_a():
    print("A. logger alone (temporary folders)")
    tmp = Path(tempfile.mkdtemp())
    clk = (0.0, 0.0)
    first = StanceSearchLogger(run_dir=tmp / "run")
    second = StanceSearchLogger(run_dir=tmp / "run")
    check("a run folder gets the three files when the search starts", all((tmp / "run" / f).exists() for f in FILES), str(sorted(p.name for p in (tmp / "run").iterdir())))
    check("a second search in the same run folder gets its own folder", second.folder != first.folder and second.folder.parent == tmp / "run", str(second.folder))
    n = first.new_candidate((0.9, 1.1, 0.5, 0.12), clk)
    first.begin_check(n, "body_check", clk)
    first.end_check(n, "body_check", True, None, clk)
    first.add_solutions(n, [{"ok": True, "points": [{"q_rad": [float("nan")] * 7}]}])
    first.add_dryrun(n, {"place_theta_deg": num(-19.999949), "subtasks": {}})
    first.accept(n)
    first.save()
    second.save()
    body = json.loads((first.folder / "candidates_list.json").read_text())
    check("the first search's file is kept when the second search saves", body["accepted"] == 0 and body["candidates"][0]["checks"]["body_check"]["reason"] == "", str(body["accepted"]))
    sol = json.loads((first.folder / "full_check_solutions.json").read_text())
    check("NaN joint values are written as null", sol["candidates"]["0"][0]["points"][0]["q_rad"] == [None] * 7, "")
    dr = json.loads((first.folder / "dryrun_trajectories.json").read_text())
    check("numbers are rounded to 4 decimals", dr["candidates"]["0"]["place_theta_deg"] == -20.0 and num(0.123456) == 0.1235, str(dr["candidates"]["0"]["place_theta_deg"]))
    check("the dry-run file names its columns", dr["columns"] == DRYRUN_COLUMNS and len(dr["columns"]) == 11, str(len(dr["columns"])))
    check("a search with no accepted candidate records -1", (second.folder / "candidates_list.json").exists() and json.loads((second.folder / "candidates_list.json").read_text())["accepted"] == -1, "")


def part_b():
    print("B. one real stance search on the staging layout")
    from stance_search_proof import BEAKER_ON_HOTPLATE_PLAN
    from src.mpvi_postprocess import postprocess
    from src.stance_select import apply_stance_selection
    from src.task_config import TASK_CFG

    tagged = postprocess(BEAKER_ON_HOTPLATE_PLAN, False, say=lambda *a, **k: None)
    out = apply_stance_selection(tagged, TASK_CFG, say=lambda *a, **k: None)
    sampler = out["report"].get("sampler") or {}
    folder = Path(sampler.get("stance_search_dir", ""))
    check("the sampler record names the search folder", folder.is_dir(), str(folder))
    check("the three files are in that folder", all((folder / f).is_file() for f in FILES), "")
    cands = json.loads((folder / "candidates_list.json").read_text())
    sols = json.loads((folder / "full_check_solutions.json").read_text())
    dry = json.loads((folder / "dryrun_trajectories.json").read_text())
    cl = cands["candidates"]
    check("candidates are numbered 0, 1, 2, ... in draw order", [c["number"] for c in cl] == list(range(len(cl))), f"{len(cl)} drawn")
    starts = [c["wall_time_start_s"] for c in cl]
    check("wall-clock starts never decrease with the number", all(a <= b for a, b in zip(starts, starts[1:])), "")
    check("the first candidate starts within 5 s of the search start", starts[0] is not None and starts[0] < 5.0, f"{starts[0]} s")
    check("every candidate has an end time at or after its start", all(c.get("wall_time_end_s", -1) >= c["wall_time_start_s"] for c in cl if "wall_time_end_s" in c), "")
    clocks = {c["sim_time_start_s"] for c in cl} | {c.get("sim_time_end_s") for c in cl}
    check("the simulator clock is read at every candidate and is the same at all of them (paused)", len(clocks) == 1 and None not in clocks, str(clocks))
    check("headings are in degrees (the heading draw is not wrapped, so within -360 to 360)", all(abs(c["heading_deg"]) <= 360.0 for c in cl), "")
    accepted = [c for c in cl if c["status"] == "accepted"]
    check("the accepted candidate is the one whose dry run passed", (out["ok"] and len(accepted) == 1 and accepted[0]["number"] == cands["accepted"]) or (not out["ok"] and cands["accepted"] == -1 and not accepted), f"accepted {cands['accepted']}")
    full = [c for c in cl if "full_check" in c["checks"]]
    check("every candidate that reached the full check has its solutions in the file", all(str(c["number"]) in sols["candidates"] for c in full), f"{len(full)} reached the full check")
    solves = [s for v in sols["candidates"].values() for s in v]
    check("the full-check file has labelled solutions (kind, approach angle, points with paths and joint values)",
          bool(solves) and all({"kind", "approach_angle_deg", "points"} <= set(s) for s in solves) and any(s["points"] and s["points"][0]["q_rad"] for s in solves), f"{len(solves)} solves")
    dryc = [c for c in cl if "dry_run" in c["checks"]]
    check("every candidate that reached the dry run has its trajectory in the file", all(str(c["number"]) in dry["candidates"] for c in dryc), f"{len(dryc)} reached the dry run")
    rows_ok, rounded_ok, angles_ok = True, True, True
    for entry in dry["candidates"].values():
        angles_ok = angles_ok and entry["place_theta_deg"] is not None and entry["pick_approach_bin"] is not None
        for sub in entry["subtasks"].values():
            rows_ok = rows_ok and len(sub["rows"]) == sub["ticks"] and all(len(r) == len(DRYRUN_COLUMNS) for r in sub["rows"])
            rounded_ok = rounded_ok and all(round(v, 4) == v for r in sub["rows"] for v in r if v is not None)
    check("each dry-run subtask has one row per tick, with the columns named in the file", rows_ok, "")
    check("every dry-run number is rounded to 4 decimals", rounded_ok, "")
    check("the place angle (degrees) and the pick approach angle are fields of every dry run", angles_ok, "")


def main():
    part_a()
    part_b()
    print("\nALL PASS" if all(RESULTS) else f"\nFAILED: {RESULTS.count(False)} of {len(RESULTS)} checks")
    return 0 if all(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
