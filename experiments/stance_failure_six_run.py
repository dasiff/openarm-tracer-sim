"""The six-candidate run of M281 (David, 2026-11-09): the stance checks of the three overnight order-test flips and three other overnight failures, in one fresh process.

The stances are read from the overnight run's file (data/logs/overnight_20261109/B_given_noreseed.jsonl, the given order), by its row number. The points are the same as the overnight run's (PLAN4 through the stance selection), the maps are data/reach_map_before_bench.

usage: python experiments/stance_failure_six_run.py logged|control <answers.jsonl>
  logged:  logging on (checks.jsonl and failures.jsonl in data/logs/stance_failures/)
  control: logging off (no files); the same checks, to show the logging changes no answer
"""
import json
import math
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "experiments"))

from src import bench_stance as bs  # noqa: E402
from src import stance_failure_log as sfl  # noqa: E402
from src.full_check import FullCheck, SIDE  # noqa: E402
from src.mpvi_postprocess import postprocess  # noqa: E402
from src.stance_select import _manip_steps, make_context, step_target  # noqa: E402
from src.task_config import BENCH_STANCE_CFG, STANCE_LOG_CFG, TASK_CFG  # noqa: E402
from src.world_model import StaticTaskSource, WorldModel  # noqa: E402
from stance_select_test import PLAN4  # noqa: E402

OVERNIGHT = "data/logs/overnight_20261109/B_given_noreseed.jsonl"
MAPS = "data/reach_map_before_bench"
# (row in the overnight file, what it is)
ROWS = [(17, "flip: PASS in the given order"), (18, "flip: FAIL in the given order"), (21, "flip: PASS in the given order"),
        (24, "other failure: ik"), (37, "other failure: place_angle_window"), (45, "other failure: place_angle_window (5 deg window)")]


def build_points(fc):
    """The task points of PLAN4 (as the overnight run built them: make_context, step_target, the manipulation steps)."""
    ctx = make_context(fc.model, fc.scene_data, bs.load_maps(MAPS, sides=(SIDE,)), TASK_CFG, BENCH_STANCE_CFG, chk=fc.chk, say=lambda *_: None)
    world = WorldModel([StaticTaskSource(TASK_CFG)])
    world.refresh(0.0)
    tagged = postprocess(PLAN4, False, say=lambda *_: None)
    steps = _manip_steps(tagged)
    targets = {i: step_target(ctx, world, tagged["segments"][i], i, lambda *_: None) for i in steps}
    return [p for i in steps for p in targets[i]["points"]]


def main():
    mode, out = sys.argv[1], sys.argv[2]
    assert mode in ("logged", "control")
    rows = [json.loads(line) for line in open(OVERNIGHT)]
    cands = [(rows[i]["stance"], i, what) for i, what in ROWS]
    t0 = time.time()
    fc = FullCheck(maps=bs.load_maps(MAPS), task_cfg=TASK_CFG, say=lambda *_: None, maps_dir=MAPS,
                   failure_log_dir=(None if mode == "control" else STANCE_LOG_CFG["dir"]))
    pts = build_points(fc)
    sfl.append(out, {"event": "header", "mode": mode, "header": sfl.run_header(11), "commit": sfl.git_commit(), "points": len(pts)})
    print(f"{mode}: fresh FullCheck and {len(pts)} points in {time.time() - t0:.1f} s", flush=True)
    for stance, i, what in cands:
        x, y, hd, q = stance
        t1 = time.time()
        r = fc.check((x, y, math.radians(hd), q), pts)
        rec = {"event": "answer", "mode": mode, "overnight_row": i, "what": what, "stance": stance, "ok": r["ok"], "failed": r["failed"], "reason": r["reason"],
               "cached": r["cached"], "seconds": round(time.time() - t1, 1)}
        if fc.failure_log_dir is not None and not r["cached"]:          # the record to replay: failures.jsonl holds the failed ones (with each group run alone), checks.jsonl the others
            rec["record_id"] = f"{fc.run_id}-{fc.check_n}"
            rec["record_file"] = fc.failures_path if not r["ok"] else fc.checks_path
        sfl.append(out, rec)
        print(f"row {i} {stance} ok={r['ok']} failed={r['failed']} ({rec['seconds']} s)", flush=True)
    print(f"{mode}: done in {time.time() - t0:.1f} s", flush=True)


if __name__ == "__main__":
    main()
