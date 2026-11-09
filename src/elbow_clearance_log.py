"""Logging only (no control logic): per arm, the lowest elbow height over a run and when, and the bench contact of the elbow housing, the upper arm and the forearm.

The parts (MuJoCo bodies of each arm): the elbow housing is openarm_<side>_link4 (its elbow joint's origin is the elbow joint centre that the elbow guard
checks, src/elbow_guard.py), the upper arm is openarm_<side>_link3, the forearm is openarm_<side>_link5. "Bench" is every geom of the body "bench".

Per part it keeps the smallest gap to the bench (mm, negative = penetration; looked up to 0.3 m, every `every` ticks), the largest contact normal
force with a bench geom (N, from the contacts the physics reports), and the time of each. Nothing is written to the simulation.

    monitor = make_elbow_clearance_monitor(model, data)     # pass as an extra source of run_episode
    write_elbow_clearance(monitor, path)                     # after the run
"""
import json

import mujoco
import numpy as np

PARTS = {"elbow housing": "link4", "upper arm": "link3", "forearm": "link5"}
GAP_LIMIT_M = 0.3


def make_elbow_clearance_monitor(model, data, every=5):
    """Builds the logging source.

    Args:
        model (mujoco.MjModel): The compiled model.
        data (mujoco.MjData): The simulation data, read only.
        every (int): Measure the gaps on every this many calls. Defaults to 5.

    Returns:
        dict: {"name", "step": step(obs), "state"}; state["arms"][side] holds "min_elbow_z_m", "t_min_elbow_s", and "parts" (name -> min_gap_mm, t_min_gap_s,
            max_force_N, t_max_force_s, touched).
    """
    bench = [g for g in range(model.ngeom) if model.geom_bodyid[g] == model.body("bench").id and (model.geom_contype[g] or model.geom_conaffinity[g])]
    bench_set = set(bench)
    arms = {}
    for side in ("left", "right"):
        joint4 = model.joint(f"openarm_{side}_joint4").id
        geoms = {}
        for name, link in PARTS.items():
            body = model.body(f"openarm_{side}_{link}").id
            geoms[name] = [g for g in range(model.ngeom) if model.geom_bodyid[g] == body and (model.geom_contype[g] or model.geom_conaffinity[g])]
        arms[side] = {"joint4": joint4, "geoms": geoms, "min_elbow_z_m": None, "t_min_elbow_s": None,
                      "parts": {n: {"min_gap_mm": None, "t_min_gap_s": None, "max_force_N": 0.0, "t_max_force_s": None, "touched": False} for n in PARTS}}
    state = {"arms": arms, "n": 0}
    f6 = np.zeros(6)
    gap_fromto = np.zeros(6)

    def step(obs):
        t = float(obs["time"])
        state["n"] += 1
        for side, A in arms.items():
            z = float(data.xanchor[A["joint4"]][2])
            if A["min_elbow_z_m"] is None or z < A["min_elbow_z_m"]:
                A["min_elbow_z_m"], A["t_min_elbow_s"] = z, t
            for name, gs in A["geoms"].items():
                rec = A["parts"][name]
                for i in range(data.ncon):
                    c = data.contact[i]
                    if (c.geom1 in gs and c.geom2 in bench_set) or (c.geom2 in gs and c.geom1 in bench_set):
                        mujoco.mj_contactForce(model, data, i, f6)
                        rec["touched"] = True
                        if abs(f6[0]) > rec["max_force_N"]:
                            rec["max_force_N"], rec["t_max_force_s"] = abs(float(f6[0])), t
                if (state["n"] - 1) % every == 0:
                    for g in gs:
                        for b in bench:
                            dist = mujoco.mj_geomDistance(model, data, g, b, GAP_LIMIT_M, gap_fromto)
                            if dist < GAP_LIMIT_M and (rec["min_gap_mm"] is None or 1000 * dist < rec["min_gap_mm"]):
                                rec["min_gap_mm"], rec["t_min_gap_s"] = 1000 * float(dist), t

    return {"name": "elbow clearance log", "step": step, "state": state}


def write_elbow_clearance(monitor, path):
    """Writes the monitor's record as JSON (a temporary file, then renamed over the old one)."""
    s = monitor["state"]
    out = {"arms": {side: {"min_elbow_z_m": A["min_elbow_z_m"], "t_min_elbow_s": A["t_min_elbow_s"], "parts": A["parts"]} for side, A in s["arms"].items()},
           "ticks": s["n"]}
    tmp = f"{path}.tmp"
    with open(tmp, "w") as f:
        json.dump(out, f, indent=1, default=float)
    import os
    os.replace(tmp, path)
    return out
