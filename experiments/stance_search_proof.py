#!/usr/bin/env python
"""ONE real stance search on the staging layout (TASK_CFG), called from a run folder: the real full check and the real dry run, no stub.

    source .venv/bin/activate && export MUJOCO_GL=egl && python experiments/stance_search_proof.py

The run folder data/demo_record/stance_search_proof_<stamp>/ gets the search's three files and run_info.json. The script then prints what the files hold
(sizes, one record from each file, every labelled arm solution of the accepted candidate, the first and last candidates' times, the simulator clock).
"""

import json
import sys
import time
from collections import Counter
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


def summarize(folder):
    cands = json.loads((folder / "candidates_list.json").read_text())
    sols = json.loads((folder / "full_check_solutions.json").read_text())
    dry = json.loads((folder / "dryrun_trajectories.json").read_text())
    print("\n=== FILES")
    for name in ("candidates_list.json", "full_check_solutions.json", "dryrun_trajectories.json"):
        print(f"  {folder / name}  {(folder / name).stat().st_size} bytes")
    cl = cands["candidates"]
    print(f"\n=== CANDIDATES: {len(cl)} drawn; accepted {cands['accepted']}; statuses {dict(Counter(c['status'] for c in cl))}")
    print("  simulator clock values over all candidates (start and end):",
          sorted({c["sim_time_start_s"] for c in cl} | {c.get("sim_time_end_s") for c in cl}, key=lambda v: (v is None, v)))
    print("  check clock values over all candidates (start and end):",
          sorted({c["check_clock_start_s"] for c in cl} | {c.get("check_clock_end_s") for c in cl}, key=lambda v: (v is None, v)))
    first, last = cl[0], cl[-1]
    for label, c in (("FIRST", first), ("LAST", last)):
        print(f"  {label} candidate {c['number']} ({c['status']}): wall start {c['wall_time_start_s']} s, wall end {c.get('wall_time_end_s')} s, "
              f"sim start {c['sim_time_start_s']} s, sim end {c.get('sim_time_end_s')} s")
    acc = next((c for c in cl if c["number"] == cands["accepted"]), None)
    print("\n=== ONE CANDIDATE RECORD (accepted, candidates_list.json):")
    print(json.dumps(acc, indent=1))
    print("\n=== ONE RECORD (full_check_solutions.json, accepted candidate, first solve):")
    acc_sols = sols["candidates"].get(str(cands["accepted"]), [])
    print(f"  {len(acc_sols)} solves for the accepted candidate")
    print(json.dumps(acc_sols[0] if acc_sols else None, indent=1)[:1500])
    print("\n=== LABELLED ARM SOLUTIONS, accepted candidate (every task point of every solve):")
    for s in acc_sols:
        print(f"  {s['kind']} approach {s['approach_angle_deg']} deg, ok {s['ok']}, failed point {s['failed_point']} ({s['failed_path']})")
        for p in s["points"]:
            print(f"    point {p['index']} {p['path']} ({p['pose']}) target {p['target_m']} q_rad {p['q_rad']}")
    print("\n=== ONE RECORD (dryrun_trajectories.json, accepted candidate):")
    d = dry["candidates"].get(str(cands["accepted"]))
    if d is None:
        print("  none for the accepted candidate")
    else:
        print(f"  pick approach bin {d['pick_approach_bin']} ({d['pick_approach_angle_deg']} deg), place turn {d['place_theta_deg']} deg, ok {d['ok']}")
        for name, sub in d["subtasks"].items():
            print(f"  {name}: ok {sub['ok']} ticks {sub['ticks']} rows {len(sub['rows'])} first {sub['rows'][0] if sub['rows'] else None} last {sub['rows'][-1] if sub['rows'] else None}")
    print(f"\n=== DRY-RUN candidates in the file: {sorted(int(k) for k in dry['candidates'])}; full-check candidates: {sorted(int(k) for k in sols['candidates'])}")


SAMPLER_SEED = 3397871272     # the seed of the smoke search that tried 28 candidates before its accepted one (data/logs/stance_search_smoke_20261112_020330.log)
FULLCHECK_SEED = 4242         # pins the full check's random starts for this process (FULLCHECK_CFG seed_pin); the search itself is unchanged


def main():
    stamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = PROJECT_ROOT / "data" / "demo_record" / f"stance_search_proof_{stamp}"
    run_dir.mkdir(parents=True)
    from src import edge_sampling
    from src.task_config import FULLCHECK_CFG
    FULLCHECK_CFG["seed_pin"] = FULLCHECK_SEED
    sampler = edge_sampling.iter_verified_stances

    def pinned_sampler(*a, **k):
        k["seed"] = SAMPLER_SEED
        return sampler(*a, **k)
    edge_sampling.iter_verified_stances = pinned_sampler
    tagged = postprocess(BEAKER_ON_HOTPLATE_PLAN, False, say=lambda *a, **k: None)
    t0 = time.perf_counter()
    sel = apply_stance_selection(tagged, TASK_CFG, run_dir=run_dir, say=print)
    seconds = time.perf_counter() - t0
    sampler = sel["report"].get("sampler") or {}
    folder = Path(sampler["stance_search_dir"])
    (run_dir / "run_info.json").write_text(json.dumps({"ok": sel["ok"], "stop": sel["stop"], "stance_search_dir": str(folder), "selection_seconds": round(seconds, 1),
                                                       "sampler_seed": SAMPLER_SEED, "fullcheck_seed": FULLCHECK_SEED}, indent=1))
    print(f"\nRESULT ok {sel['ok']} stop {sel['stop']} selection {seconds:.1f} s; run folder {run_dir}; search folder {folder}", flush=True)
    summarize(folder)
    return 0 if sel["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
