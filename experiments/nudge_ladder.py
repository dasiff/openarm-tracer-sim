"""Nudge ladder of the stance selection (David 2026-11-07): the standard layout (TASK_CFG's beaker and hotplate) first as a control, then
layouts where each object moves independently by a random offset up to each nudge size in x and y (uniform in [-size, +size] per axis).

    source .venv/bin/activate && MUJOCO_GL=egl python experiments/nudge_ladder.py --per-size 20 --out docs/reports/nudge_ladder_20261107.json

Mode: current (the selection as it stands, stance_check_mode "current"), unfiltered maps (data/reach_map), the full check the pipeline builds.
Every layout is recorded: positions, offsets from the standard layout, pass or fail, the stop reason, and for full-check failures the
rejecting check of each candidate tried and the most common one. The JSON is rewritten (temporary file, then renamed) after every layout.
"""
import argparse
import collections
import copy
import json
import os
import random
import re
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "experiments"))
if not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")

from src import bench_stance as bs  # noqa: E402
from src.full_check import FullCheck  # noqa: E402
from src.mpvi_postprocess import postprocess  # noqa: E402
from src.stance_select import make_context, select_bench_stance  # noqa: E402
from src.task_beaker_hotplate import load_task  # noqa: E402
from src.task_config import BENCH_STANCE_CFG, TASK_CFG  # noqa: E402
from stance_select_test import PLAN4, world  # noqa: E402

SIZES = (0.02, 0.05, 0.10, 0.20, 0.40)


def reason_of(stop):
    if not stop:
        return "pass"
    if stop.startswith("no stance covers"):
        return "no stance covers any of the objects"
    if stop.startswith("no stance passed the full check"):
        return "no stance passed the full check"
    return "other: " + stop[:80]


def rejecting_checks(sel):
    """Per candidate tried by the full check: the failed check names; and the most common one."""
    per = []
    for t in sel["report"].get("tried", []):
        for c in t.get("full_check", []):
            per.append({"xy": [round(float(c["xy"][0]), 4), round(float(c["xy"][1]), 4)], "pedestal_q": round(float(c["q"]), 4), "failed": list(c.get("failed", []))})
    counts = collections.Counter(n for c in per for n in c["failed"])
    return per, (counts.most_common(1)[0][0] if counts else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--per-size", type=int, default=20)
    ap.add_argument("--seed", type=int, default=20261107)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    sim = load_task()
    m, d = sim.model, sim.data
    maps = bs.load_maps("data/reach_map", sides=("left",))
    ctx = make_context(m, d, maps)
    FULL = FullCheck(maps=bs.load_maps("data/reach_map"), task_cfg=TASK_CFG)
    tagged = postprocess(PLAN4, False, say=lambda *_: None)
    quiet = lambda *_: None
    cfg = copy.deepcopy(BENCH_STANCE_CFG)
    cfg["stance_check_mode"] = "current"
    B0, H0 = [float(v) for v in TASK_CFG["beaker_xy"]], [float(v) for v in TASK_CFG["hotplate_xy"]]
    rng = random.Random(args.seed)
    plan = [{"group": "control", "size_m": 0.0, "beaker_off": [0.0, 0.0], "hotplate_off": [0.0, 0.0]}]
    for s in SIZES:
        for _ in range(args.per_size):
            plan.append({"group": f"nudge_{int(round(100 * s))}cm", "size_m": s,
                         "beaker_off": [round(rng.uniform(-s, s), 4), round(rng.uniform(-s, s), 4)],
                         "hotplate_off": [round(rng.uniform(-s, s), 4), round(rng.uniform(-s, s), 4)]})
    out = {"mode": "current", "unfiltered_maps": "data/reach_map", "standard_layout": {"beaker_xy": B0, "hotplate_xy": H0}, "seed": args.seed,
           "per_size": args.per_size, "layouts": []}
    t_all = time.perf_counter()
    for i, p in enumerate(plan):
        b = [round(B0[0] + p["beaker_off"][0], 4), round(B0[1] + p["beaker_off"][1], 4)]
        h = [round(H0[0] + p["hotplate_off"][0], 4), round(H0[1] + p["hotplate_off"][1], 4)]
        t0 = time.perf_counter()
        w = world(beaker_xy=tuple(b), hotplate_xy=tuple(h))
        sel = select_bench_stance(tagged, w, TASK_CFG, ctx, cfg, full_check=FULL, say=quiet)
        ok = bool(sel["ok"])
        per, top = rejecting_checks(sel) if not ok else ([], None)
        row = {"i": i, "group": p["group"], "size_m": p["size_m"], "beaker_xy": b, "hotplate_xy": h, "beaker_offset": p["beaker_off"], "hotplate_offset": p["hotplate_off"],
               "ok": ok, "reason": reason_of(sel["stop"]), "stop": sel["stop"], "rejecting_checks_per_candidate": per, "most_common_rejecting_check": top,
               "stance": None if not ok else {"xy": [round(float(v), 4) for v in sel["report"]["runs"][0]["stance"]["xy"]], "q": float(sel["report"]["runs"][0]["stance"]["q"])},
               "seconds": round(time.perf_counter() - t0, 1)}
        out["layouts"].append(row)
        tmp = args.out + ".tmp"
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(out, f, indent=1)
        os.replace(tmp, args.out)
        print(f"[{i + 1}/{len(plan)}] {p['group']} B({b[0]:.3f},{b[1]:.3f}) H({h[0]:.3f},{h[1]:.3f}): {'PASS' if ok else 'FAIL ' + row['reason']} ({row['seconds']} s)", flush=True)
    out["total_seconds"] = round(time.perf_counter() - t_all, 1)
    with open(args.out + ".tmp", "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1)
    os.replace(args.out + ".tmp", args.out)
    print("NUDGE_DONE", flush=True)


if __name__ == "__main__":
    main()
