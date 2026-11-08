"""Scene test of the analytic inverse (src/reach_model.py): the standard and the moved layout, left arm, the full check on the top candidates.

    source .venv/bin/activate && MUJOCO_GL=egl python experiments/reach_inverse_scene.py --scene standard|moved --out docs/reports/reach_inverse_<scene>.json
(run through bash experiments/run_capped.sh <log> ... : one MuJoCo process, memory capped.)

The analytic search gives the ideal spot of each object and the segment candidates; the full check (src/full_check.py FullCheck.check) decides each candidate.
Candidate format: stance = (x, y, heading, pedestal command), the same tuple FullCheck.check takes; points = src/task_points.task_points(model, data, layout cfg).
"""
import argparse
import json
import math
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
os.chdir(PROJECT_ROOT)

import numpy as np  # noqa: E402

from src import bench_stance as bs  # noqa: E402
from src import reach_model as rm  # noqa: E402
from src.full_check import FullCheck  # noqa: E402
from src.task_beaker_hotplate import bench_top_z  # noqa: E402
from src.task_config import BEAKER_CFG, BENCH_STANCE_CFG, PICK_PLACE_CFG, TASK_CFG  # noqa: E402
from src.task_points import task_points  # noqa: E402

MAPS = "data/reach_map_before_bench"              # the filtered maps (David)
ARM = "left"
PEDESTAL = BENCH_STANCE_CFG["pedestal_override_m"]        # 0.16 m, the staging pedestal (the pedestal the shoulder was read at)
HEADING = TASK_CFG["staging_yaw"]                 # -90 deg, facing the bench
LAYOUTS = {
    "standard": dict(TASK_CFG),
    "moved": {**TASK_CFG, "beaker_xy": (TASK_CFG["beaker_xy"][0] + TASK_CFG["moved_shift_x_m"], TASK_CFG["beaker_xy"][1]),
              "hotplate_xy": (TASK_CFG["hotplate_xy"][0] + TASK_CFG["moved_shift_x_m"], TASK_CFG["hotplate_xy"][1])},
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", choices=sorted(LAYOUTS), required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    cfg = LAYOUTS[a.scene]
    maps = bs.load_maps(MAPS, sides=(ARM,))
    fc = FullCheck(maps=maps, task_cfg=cfg, say=lambda *_: None)
    m, d = fc.model, fc.scene_data
    geom = {"shoulder": {arm: tuple(float(v) for v in fc.rg["shoulder"][arm]) for arm in ("left", "right")}, "torso_z0": float(fc.rg["torso_z0"]),
            "lift_factor": float(fc.rg["lift_factor"]), "base_z": float(fc.rg["base_z"])}
    frac = PICK_PLACE_CFG["grasp_height_frac"]
    top = bench_top_z(m)
    plate = m.geom("task_hotplate_top_plate")
    plate_top = float(d.geom_xpos[plate.id][2] + m.geom_size[plate.id][2])
    objects = [
        {"name": "beaker", "xy": [float(v) for v in cfg["beaker_xy"]], "z": float(top + frac * BEAKER_CFG["height"])},
        {"name": "hotplate", "xy": [float(v) for v in cfg["hotplate_xy"]], "z": float(plate_top + 0.002 + frac * BEAKER_CFG["height"])},
    ]
    res = rm.inverse_search(objects, ARM, geom)
    best = res["candidates"][0] if res["candidates"] else None
    ideal = {}
    if best is not None:
        for o in objects:
            ideal[o["name"]] = rm.ideal_spot(o["xy"], o["z"], ARM, math.radians(best["heading_deg"]), best["pedestal"], geom)
    points = task_points(m, d, cfg)
    rows = []
    for c in res["candidates"]:
        chk = fc.check(tuple(c["stance"]), points)
        rows.append({**c, "full_check_ok": bool(chk["ok"]), "failed": chk["failed"], "reason": chk["reason"]})
        print(f"{a.scene}: stance ({c['stance'][0]:.3f}, {c['stance'][1]:.3f}) heading {c['heading_deg']:.0f} pedestal {c['pedestal']:.2f} counts {c['counts']} -> ok {chk['ok']} failed {chk['failed']}", flush=True)
    out = {"scene": a.scene, "layout": {"beaker_xy": objects[0]["xy"], "hotplate_xy": objects[1]["xy"]}, "arm": ARM, "sweep": {"headings_deg": [0, 350, 10], "pedestals_m": [0.05, 0.25, 5], "segment_points": 50, "evaluations": res["evaluations"], "status_counts": res["status_counts"]}, "best_heading_deg": res["best_heading"], "best_pedestal_m": res["best_pedestal"],
           "geometry_shoulder_left": geom["shoulder"]["left"], "geometry_shoulder_right": geom["shoulder"]["right"],
           "objects": objects, "label": res["label"], "ideal_at_best": ideal, "split": res["split"], "single_stance_feasible": res["feasible_points"] > 0,
           "feasible_points": res["feasible_points"], "candidates": rows, "passing": [r["stance"] for r in rows if r["full_check_ok"]]}
    tmp = a.out + ".tmp"
    Path(tmp).write_text(json.dumps(out, indent=1, default=float))
    os.replace(tmp, a.out)
    print("RESULT", json.dumps({k: out[k] for k in ("scene", "label", "best_heading_deg", "best_pedestal_m", "single_stance_feasible", "feasible_points", "passing")}), flush=True)


if __name__ == "__main__":
    main()
