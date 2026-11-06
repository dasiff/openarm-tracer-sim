"""Layout sweep of the stance selection (David 2026-11-06): random beaker and hotplate positions on the usable bench surface, the selection run on each.

    source .venv/bin/activate && MUJOCO_GL=egl python experiments/layout_sweep.py --mode current --n 100 --out docs/reports/layout_sweep_current_20261106.json

Usable surface: the bench top's own extent in the scene (the geom "bench_top"), shrunk by BENCH_STANCE_CFG["bench_edge_margin"] (the objects'
distance from every bench edge, the layout search's rule). An object centre is uniform in that rectangle. A layout whose beaker and hotplate
overlap (centres closer than the sum of their radii, read from the world model's shapes) is redrawn. Nothing else is added or invented.
Each layout runs the selection on the unfiltered maps (data/reach_map, option A) with the full check the pipeline builds. Mode: --mode (current or two_pass).
Writes one JSON with every layout's positions, pass or fail, the chosen stance (if any) and the stop message.
"""
import argparse
import copy
import json
import os
import random
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "experiments"))
if not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402

from src import bench_stance as bs  # noqa: E402
from src.full_check import FullCheck  # noqa: E402
from src.mpvi_postprocess import postprocess  # noqa: E402
from src.stance_select import make_context, select_bench_stance  # noqa: E402
from src.task_beaker_hotplate import load_task  # noqa: E402
from src.task_config import BENCH_STANCE_CFG, LAYOUT_CFG, TASK_CFG  # noqa: E402
from stance_select_test import PLAN4, world  # noqa: E402


def radius_of(entry):
    sh = entry.properties.get("shape") or {}
    if sh.get("kind") == "cylinder":
        return float(sh["radius"])
    if sh.get("kind") == "box":
        return float(max(sh["half"][0], sh["half"][1]))
    raise ValueError(f"no radius for {entry.id}: shape {sh}")


def usable_rect(m, d, margin):
    mujoco.mj_kinematics(m, d)
    gid = m.geom("bench_top").id
    cx, cy = d.geom_xpos[gid][0], d.geom_xpos[gid][1]
    hx, hy = m.geom_size[gid][0], m.geom_size[gid][1]
    return (cx - hx + margin, cx + hx - margin), (cy - hy + margin, cy + hy - margin)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=("current", "two_pass"), default="current")
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--seed", type=int, default=20261106)
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
    cfg["stance_check_mode"] = args.mode
    base = world()
    r_beaker, r_hot = radius_of(base.get("beaker")), radius_of(base.get("hotplate"))
    (x0, x1), (y0, y1) = usable_rect(m, d, LAYOUT_CFG["bench_edge_margin"])
    rng = random.Random(args.seed)
    layouts = []
    while len(layouts) < args.n:
        b = (rng.uniform(x0, x1), rng.uniform(y0, y1))
        h = (rng.uniform(x0, x1), rng.uniform(y0, y1))
        if ((b[0] - h[0]) ** 2 + (b[1] - h[1]) ** 2) ** 0.5 < r_beaker + r_hot:
            continue
        layouts.append({"beaker_xy": [round(b[0], 4), round(b[1], 4)], "hotplate_xy": [round(h[0], 4), round(h[1], 4)]})
    print(f"usable rectangle x {x0:.3f}..{x1:.3f}, y {y0:.3f}..{y1:.3f}; radii beaker {r_beaker:.3f} hotplate {r_hot:.3f}; mode {args.mode}", flush=True)
    rows = []
    t_all = time.perf_counter()
    for i, L in enumerate(layouts):
        t0 = time.perf_counter()
        w = world(beaker_xy=tuple(L["beaker_xy"]), hotplate_xy=tuple(L["hotplate_xy"]))
        sel = select_bench_stance(tagged, w, TASK_CFG, ctx, cfg, full_check=FULL, say=quiet)
        ok = bool(sel["ok"])
        stance = sel["report"]["runs"][0]["stance"] if ok and sel["report"]["runs"] else None
        row = {"i": i, **L, "ok": ok, "stance": None if stance is None else {"xy": [round(float(v), 4) for v in stance["xy"]], "q": float(stance["q"])},
               "stop": sel["stop"], "seconds": round(time.perf_counter() - t0, 1)}
        rows.append(row)
        print(f"[{i + 1}/{len(layouts)}] beaker {L['beaker_xy']} hotplate {L['hotplate_xy']}: {'PASS' if ok else 'FAIL'} ({row['seconds']} s)", flush=True)
    npass = sum(r["ok"] for r in rows)
    out = {"mode": args.mode, "n": len(rows), "seed": args.seed, "pass": npass, "fail": len(rows) - npass, "pass_rate": round(npass / len(rows), 4),
           "usable_rectangle": {"x": [x0, x1], "y": [y0, y1]}, "radii": {"beaker": r_beaker, "hotplate": r_hot}, "bench_edge_margin": LAYOUT_CFG["bench_edge_margin"],
           "total_seconds": round(time.perf_counter() - t_all, 1), "layouts": rows}
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    tmp = args.out + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1)
    os.replace(tmp, args.out)
    print(f"SWEEP {args.mode}: {npass} of {len(rows)} pass ({100 * npass / len(rows):.1f} %)", flush=True)
    print("SWEEP_DONE", flush=True)


if __name__ == "__main__":
    main()
