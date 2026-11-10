"""Surveys the lab's fixed fixtures from the scene file and writes data/safety_filter/lab_map.json (M290).

Fixed fixtures ONLY: bench top and legs, the sink, the walls, the back counter, the fume hood, the storage
cabinet and the safety station. Not the hotplate, the beaker, the test-tube rack, the stools, the cart or
anything else that moves or is placed for a task: those come from perception (design plan 13).

Each collidable geom of a fixture body (contype > 0, the simulator's own collision set) becomes one entry in
the world frame: a box at its pose. Boxes are exact; a cylinder, capsule, ellipsoid or sphere becomes the box
that encloses it in its own frame. Geoms with contype 0 (visual only) and geoms not in the list are written
under "excluded" with the reason, so David can see what was left out.

The file is fingerprinted with the scene file's SHA-256. Written atomically (temp file, then rename).

Run:  python tools/survey_lab_map.py   (writes data/safety_filter/lab_map.json)
"""

import argparse
import hashlib
import json
import os
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.safety_filter.config import LAB_MAP_SCENE  # noqa: E402
from src.task_config import TASK_CFG  # noqa: E402  (only the bench_open setting is read; the scene is not built here)
from src.task_beaker_hotplate import BENCH_RAIL_GEOMS  # noqa: E402  (the rails the sim deletes when the bench is open)

DEFAULT_OUT = ROOT / "data" / "safety_filter" / "lab_map.json"

# (fixture name, body in the scene, geom-name test). The bench frame rails are dropped when bench_open is on
# (the same rule as the simulator: src/task_beaker_hotplate.remove_bench_rails); see survey() below.
FIXTURES = [
    ("bench", "bench", lambda n: n.startswith("bench_")),                       # bench_top, bench_frame_*, bench_leg_*
    ("bench_gas", "bench", lambda n: n == "gas_manifold" or n.startswith("gas_knob_")),  # the gas manifold and its knobs (fixed, on the bench top)
    ("sink", "bench", lambda n: n in ("sink_basin_outer", "faucet_post", "faucet_spout")),
    ("walls", "world", lambda n: n.startswith("wall_")),
    ("back_counter", "back_counter", lambda n: True),
    ("fume_hood", "fume_hood", lambda n: True),
    ("storage_cabinet", "storage_cabinet", lambda n: True),
    ("safety_station", "safety_station", lambda n: True),
]


def enclosing_half(model, g):
    """Half sizes of the box that encloses geom g in its own frame (exact for box and ellipsoid)."""
    t = model.geom_type[g]
    s = model.geom_size[g]
    if t == mujoco.mjtGeom.mjGEOM_BOX:
        return s.copy(), "box"
    if t == mujoco.mjtGeom.mjGEOM_SPHERE:
        return np.array([s[0], s[0], s[0]]), "sphere"
    if t == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
        return s.copy(), "ellipsoid"
    if t == mujoco.mjtGeom.mjGEOM_CYLINDER:
        return np.array([s[0], s[0], s[1]]), "cylinder"
    if t == mujoco.mjtGeom.mjGEOM_CAPSULE:
        return np.array([s[0], s[0], s[0] + s[1]]), "capsule"
    if t == mujoco.mjtGeom.mjGEOM_MESH:
        did = model.geom_dataid[g]
        adr, n = model.mesh_vertadr[did], model.mesh_vertnum[did]
        V = model.mesh_vert[adr:adr + n].astype(float)
        return (V.max(axis=0) - V.min(axis=0)) / 2, "mesh"
    raise ValueError(f"geom type {t} not handled")


def survey(scene_path, bench_open=None):
    """Fixtures and exclusions of the scene. bench_open (default: TASK_CFG bench_open): when True the four frame
    rails are not written, as the simulator deletes them. Returns (fixtures, excluded, scene sha256, bench_open)."""
    if bench_open is None:
        bench_open = bool(TASK_CFG["bench_open"])
    scene_path = Path(scene_path)
    model = mujoco.MjModel.from_xml_path(str(scene_path.resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    name = lambda kind, i: mujoco.mj_id2name(model, kind, i) or ""
    fixtures, excluded = [], []
    matched = set()
    listed_bodies = {body for _, body, _ in FIXTURES}
    for fixture, body_name, wanted in FIXTURES:
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body < 0:
            raise SystemExit(f"scene has no body named {body_name}")
        for g in range(model.ngeom):
            if model.geom_bodyid[g] != body:
                continue
            gname = name(mujoco.mjtObj.mjOBJ_GEOM, g)
            if not wanted(gname):
                continue
            if bench_open and gname in BENCH_RAIL_GEOMS:
                excluded.append({"fixture": None, "body": body_name, "geom": gname,
                                 "reason": "frame rail: deleted by the open-bench rule (bench_open True, TASK_CFG)"})
                matched.add(g)
                continue
            matched.add(g)
            if model.geom_contype[g] == 0:
                excluded.append({"fixture": fixture, "body": body_name, "geom": gname,
                                 "reason": "contype 0 (visual or hole only; no collision in the simulator)"})
                continue
            half, source = enclosing_half(model, g)
            R = data.geom_xmat[g].reshape(3, 3)
            quat = np.zeros(4)
            mujoco.mju_mat2Quat(quat, R.flatten())
            fixtures.append({
                "name": gname or f"{fixture}_{g}", "fixture": fixture, "body": body_name, "geom": gname,
                "shape": "box", "source": source,
                "center": [round(float(v), 6) for v in data.geom_xpos[g]],
                "quat": [round(float(v), 9) for v in quat],
                "half": [round(float(v), 6) for v in half],
            })
    # Geoms of the listed bodies that no fixture takes (gas manifold, knobs, ...): excluded, with the reason.
    for body_name in sorted(listed_bodies):
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        for g in range(model.ngeom):
            if model.geom_bodyid[g] == body and g not in matched and (model.geom_contype[g] > 0 or model.geom_conaffinity[g] > 0):
                excluded.append({"fixture": None, "body": body_name, "geom": name(mujoco.mjtObj.mjOBJ_GEOM, g),
                                 "reason": "collidable, not in the fixture list"})
    return fixtures, excluded, hashlib.sha256(scene_path.read_bytes()).hexdigest(), bench_open


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--scene", default=str(ROOT / LAB_MAP_SCENE))
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    args = ap.parse_args()
    fixtures, excluded, sha, bench_open = survey(args.scene)
    doc = {
        "schema": 2,
        "scene": str(Path(args.scene).resolve().relative_to(ROOT)),
        "scene_sha256": sha,
        "bench_open": bench_open,
        "bench_open_source": "src/task_config.py TASK_CFG['bench_open'] [M078]; the map is valid only for this setting",
        "frame": "scene world (the robot base at its zero pose)",
        "included_fixtures": sorted({f["fixture"] for f in fixtures}),
        "fixtures": fixtures,
        "excluded": excluded,
    }
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    os.replace(tmp, out)
    print(f"wrote {out}: {len(fixtures)} boxes, {len(excluded)} excluded geoms, scene sha256 {sha[:16]}")
    for f in fixtures:
        print(f"  {f['fixture']:16s} {f['name']:24s} half {f['half']} ({f['source']})")


if __name__ == "__main__":
    main()
