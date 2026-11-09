"""Order test of the stance checks (M283): runs the given stances in one fresh process, each check with its own saved seed pinned, and prints each answer.

Run it once in the given order and once reversed; the answers must match row by row. Logging is off (no files written).
The points are the same as the six-candidate run's (build_points in experiments/stance_failure_six_run.py).

usage: python experiments/stance_failure_order_test.py <rows as a comma list, e.g. 17,18,21,45> <seeds json: {row: seed}> <answers.jsonl>
  rows: the overnight file's row numbers, in the order to run them
  seeds json: the seed of each row's check, from its saved record (checks.jsonl or failures.jsonl)
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
from src.full_check import FullCheck  # noqa: E402
from src.task_config import TASK_CFG  # noqa: E402
from stance_failure_six_run import MAPS, OVERNIGHT, build_points  # noqa: E402


def main():
    rows = [int(r) for r in sys.argv[1].split(",")]
    seeds = {int(k): int(v) for k, v in json.loads(sys.argv[2]).items()}
    out = sys.argv[3]
    overnight = [json.loads(line) for line in open(OVERNIGHT)]
    t0 = time.time()
    fc = FullCheck(maps=bs.load_maps(MAPS), task_cfg=TASK_CFG, say=lambda *_: None, maps_dir=MAPS, failure_log_dir=None)
    pts = build_points(fc)
    print(f"order {rows}: fresh FullCheck and {len(pts)} points in {time.time() - t0:.1f} s", flush=True)
    for i in rows:
        stance = overnight[i]["stance"]
        x, y, hd, q = stance
        t1 = time.time()
        r = fc.check((x, y, math.radians(hd), q), pts, use_cache=False, seed=seeds[i])
        rec = {"row": i, "seed": seeds[i], "stance": stance, "ok": r["ok"], "failed": r["failed"], "reason": r["reason"], "seconds": round(time.time() - t1, 1)}
        sfl.append(out, rec)
        print(f"row {i} seed {seeds[i]} ok={r['ok']} failed={r['failed']} ({rec['seconds']} s)", flush=True)
    print(f"order {rows}: done in {time.time() - t0:.1f} s", flush=True)


if __name__ == "__main__":
    main()
