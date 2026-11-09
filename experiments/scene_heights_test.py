"""Test of the single source of heights (David 2026-11-08): the bench top, the hotplate's plate top, the beaker's z and the beaker's height read the same values at stance time (the stance-time world,
src/world_model.StaticTaskSource) and in the running sim (the simulator's world, SimGroundTruthSource, and the model itself), within 0.1 mm, and none of them reads 0.

Needs the model (no simulation step, no cameras). Exit 0 on pass.

    source .venv/bin/activate && MUJOCO_GL=egl python experiments/scene_heights_test.py
"""
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from src.task_beaker_hotplate import bench_top_z, load_task, plate_top_z, scene_heights  # noqa: E402
from src.task_config import BEAKER_CFG, TASK_CFG  # noqa: E402
from src.world_model import SimGroundTruthSource, StaticTaskSource, WorldModel  # noqa: E402

TOL_M = 0.0001                                   # 0.1 mm


results = []


def check(name, ok, detail=""):
    results.append(bool(ok))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}", flush=True)


def main():
    sim = load_task(TASK_CFG, cameras=False)
    m, d = sim.model, sim.data
    mujoco.mj_forward(m, d)

    live = WorldModel([SimGroundTruthSource(m, d, TASK_CFG)])
    live.refresh(d.time)
    static = WorldModel([StaticTaskSource(TASK_CFG)])
    static.refresh(0.0)

    stance_time = {
        "bench top": static.bench_top_z(),
        "bench position z": float(static.object_pose("bench")["position"][2]),
        "hotplate plate top": float(static.object_pose("hotplate")["position"][2]),
        "beaker z": float(static.object_pose("beaker")["position"][2]),
        "beaker height": float(static.object_height("beaker")),
    }
    sim_world = {
        "bench top": live.bench_top_z(),
        "bench position z": float(live.object_pose("bench")["position"][2]),
        "hotplate plate top": float(live.object_pose("hotplate")["position"][2]),
        "beaker z": float(live.object_pose("beaker")["position"][2]),
        "beaker height": float(live.object_height("beaker")),
    }
    sim_model = scene_heights(m, d, TASK_CFG, BEAKER_CFG)
    model_reads = {"bench top": sim_model["bench_top"], "hotplate plate top": plate_top_z(m, d), "beaker z": sim_model["beaker_z"], "beaker height": sim_model["beaker_height"]}
    site_top = float(d.site("hotplate_top").xpos[2])
    beaker_geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] == m.body("task_beaker").id and m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH]
    beaker_top_above_origin = max(float(d.geom_xpos[g][2] + m.geom_size[g][2]) for g in beaker_geoms) - float(d.body("task_beaker").xpos[2])

    print("stance-time world (StaticTaskSource):")
    for k, v in stance_time.items():
        print(f"    {k:20s} {v:.4f} m")
    print("running sim (SimGroundTruthSource and the model):")
    for k, v in sim_world.items():
        print(f"    {k:20s} {v:.4f} m")

    zeros = [k for k, v in stance_time.items() if abs(v) < 1e-9] + [k for k, v in sim_world.items() if abs(v) < 1e-9]
    check("no height reads 0 (stance-time world or running sim)", not zeros, f"zero: {zeros}" if zeros else "")

    for k in ("bench top", "hotplate plate top", "beaker z", "beaker height"):
        diff = abs(stance_time[k] - sim_world[k])
        check(f"{k}: stance-time world and running sim agree within 0.1 mm", diff <= TOL_M, f"stance {stance_time[k]:.4f}, sim {sim_world[k]:.4f}, diff {1000 * diff:.3f} mm")
    for k in ("bench top", "hotplate plate top", "beaker z", "beaker height"):
        diff = abs(model_reads[k] - sim_world[k])
        check(f"{k}: the model's own reading agrees with the sim world within 0.1 mm", diff <= TOL_M, f"model {model_reads[k]:.4f}, sim world {sim_world[k]:.4f}, diff {1000 * diff:.3f} mm")

    check("the hotplate's site 'hotplate_top' is at the plate top (within 0.1 mm)", abs(site_top - plate_top_z(m, d)) <= TOL_M, f"site {site_top:.4f}, plate {plate_top_z(m, d):.4f}")
    check("the beaker's geometry tops out at its configured height above its origin (within 0.1 mm)", abs(beaker_top_above_origin - BEAKER_CFG["height"]) <= TOL_M,
          f"geometry {beaker_top_above_origin:.4f} m, configured {BEAKER_CFG['height']:.4f} m")
    check("the bench top reads the value the scene file gives (0.960 m)", abs(stance_time["bench top"] - 0.960) <= TOL_M, f"{stance_time['bench top']:.4f}")

    print(f"\n{sum(results)}/{len(results)} checks passed")
    return all(results)


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
