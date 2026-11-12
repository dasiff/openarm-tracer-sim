#!/usr/bin/env python
"""Stage 2c step 1: pair counts per tick for rows 4, 5 and 6 of the Stage 2b comparison (measurement only).

For each row the resolver runs the same rolling punch stream (first MAX_TICKS ticks); at every tick the pairs are counted at the
last filtered command (the pose the filter starts from) and at the raw command, by stage (all possible, after the never-close
list, after the broad phase, active) and by pair type. Mean and max per type are written to JSON and printed.
The comparison world has no objects, so the object, fixed-world and held counts are zero by construction.

Usage (venv active, repo root): bash experiments/run_capped.sh data/logs/<stamp>_pair_counts.log python scripts/safety_filter_pair_counts.py [--ticks N] [--rows 4,5,6] [--out PATH]
"""

import argparse
import datetime as dt
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import safety_filter_diag_common as dc  # noqa: E402
from src.safety_filter.arm_conversion import with_arms  # noqa: E402
from src.safety_filter.shapes import RobotState  # noqa: E402

STAGES = ("all_possible", "after_never_close", "after_broad_phase", "active")


def count_at(res, q):
    placed = res.shapes.place(with_arms(RobotState(), q))
    return dc.pair_stages(list(placed), [])[0]


def run_row(setup, row, n):
    series = {where: {t: {s: np.zeros(n) for s in STAGES} for t in dc.TYPES} for where in ("filtered", "raw")}
    with setup.row_settings(row):
        res = setup.make_resolver(row)
        last = np.array(setup.q0, float)
        base = RobotState()
        for k in range(n):
            for where, q in (("filtered", last), ("raw", setup.raw[k])):
                c = count_at(res, np.asarray(q, float))
                for t in dc.TYPES:
                    for s in STAGES:
                        series[where][t][s][k] = c[t][s]
            o, st = res.step(setup.raw[k].copy(), last.copy(), base)
            last = np.asarray(o, float).reshape(2, 7).copy()
            if k % 50 == 0:
                print(f"row {row} tick {k}/{n}", flush=True)
    summary = {}
    for where in series:
        summary[where] = {}
        for t in dc.TYPES:
            summary[where][t] = {s: {"mean": float(series[where][t][s].mean()), "max": int(series[where][t][s].max())} for s in STAGES}
        tot = {s: np.sum([series[where][t][s] for t in dc.TYPES], axis=0) for s in STAGES}
        summary[where]["total"] = {s: {"mean": float(tot[s].mean()), "max": int(tot[s].max())} for s in STAGES}
    return summary, series


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--ticks", type=int, default=dc.MAX_TICKS)
    ap.add_argument("--rows", default="4,5,6")
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)
    t0 = time.time()
    setup = dc.Setup(args.ticks)
    n = setup.n
    doc = {"ticks": n, "world_objects": 0, "rows": {}, "definitions": {
        "pose": "counts at the last filtered command (the pose the filter starts from) and at the raw command",
        "all_possible": "pairs not exempt for a structural reason, never-close list off",
        "after_never_close": "the same with the never-close list on (the list is built for our shapes; Kong-shape ids never match it, so row 5 loses none)",
        "after_broad_phase": "pairs touch.evaluate_pairs computes a gap for (box cut, then the pair's own activation distance)",
        "active": "of those, gap below the pair's activation distance (margin + 0.01 m) or closer"}}
    for row in [int(r) for r in args.rows.split(",")]:
        s, series = run_row(setup, row, n)
        doc["rows"][dc.ROWS[row][0]] = s
        print(f"== row {row} ({dc.ROWS[row][0]}) mean/max pairs per tick, at the filtered pose", flush=True)
        for t in list(dc.TYPES) + ["total"]:
            print(f"  {t:20s} " + "  ".join(f"{st}={s['filtered'][t][st]['mean']:.1f}/{s['filtered'][t][st]['max']}" for st in STAGES), flush=True)
    out = Path(args.out) if args.out else dc.ROOT / "data" / "safety_filter" / f"pair_counts_{dt.datetime.now():%Y%m%d_%H%M%S}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(doc, indent=1))
    json.loads(tmp.read_text())
    os.replace(tmp, out)
    print(f"wrote {out} (runtime {time.time() - t0:.1f} s)", flush=True)


if __name__ == "__main__":
    main()
