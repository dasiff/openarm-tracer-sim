"""Replays one saved stance check (M281): rebuilds the saved generator state and scene, reruns the check, then reruns each check group alone from the same start, and compares every answer with the saved one.

The record comes from data/logs/stance_failures/checks.jsonl (every computed check) or failures.jsonl (the failed ones, with each group run alone).
Nothing is logged by the replay. Exit code 0 when every saved field matches.

usage: python experiments/stance_failure_replay.py <checks.jsonl or failures.jsonl> <record id>
"""
import json
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src import bench_stance as bs
from src import stance_failure_log as sfl
from src.full_check import GROUPS, FullCheck
from src.task_config import TASK_CFG


def load_record(path, rid):
    for line in open(path):
        rec = json.loads(line)
        if rec.get("id") == rid:
            return rec
    raise SystemExit(f"no record with id {rid} in {path}")


def main():
    if len(sys.argv) != 3:
        raise SystemExit(__doc__.strip().splitlines()[-1])
    path, rid = sys.argv[1], sys.argv[2]
    t0 = time.time()
    rec = load_record(path, rid)
    stance = tuple(rec["stance_raw"])
    fc = FullCheck(maps=bs.load_maps(rec["maps_dir"]), task_cfg=TASK_CFG, say=lambda *_: None, maps_dir=rec["maps_dir"], failure_log_dir=None)
    if "seed" not in rec:                                         # M283: the check's own seed; records from before M283 have none and cannot be replayed exactly
        raise SystemExit(f"record {rid} has no seed (written before M283); it cannot be replayed with the per-check seed")
    seed = rec["seed"]
    # the saved state: the simulation's data and the object poses, as they were before the check; the check's own generator is built from its saved seed (M283)
    fc.rng = np.random.default_rng(seed)
    fc.chk.q_scene = np.asarray(rec["scene_before"]["q_scene"], float).copy()
    if "sim_state" in rec["scene_before"]:                        # M282: the whole saved simulation state, restored exactly (no forward step)
        saved = rec["scene_before"]["sim_state"]
        sfl_saved = {"physics": np.asarray(saved["physics"], float), "derived": {k: np.asarray(v, float) for k, v in saved["derived"].items()}}
        sfl.restore_sim_state(fc.model, fc.scene_data, sfl_saved)
        again = sfl.save_sim_state(fc.model, fc.scene_data)
        same = np.array_equal(again["physics"], sfl_saved["physics"]) and all(np.array_equal(again["derived"][k], sfl_saved["derived"][k]) for k in sfl_saved["derived"])
        print(f"sim state restored from the record; round trip exact: {same}", flush=True)
    else:                                                         # an older record: qpos and qvel only, then a forward step (the old restore)
        fc.scene_data.qpos[:] = np.asarray(rec["scene_before"]["qpos"], float)
        fc.scene_data.qvel[:] = np.asarray(rec["scene_before"]["qvel"], float)
        mujoco.mj_forward(fc.model, fc.scene_data)
        print("older record: qpos and qvel restored, then mj_forward", flush=True)
    start = fc._snapshot()
    print(f"loaded {rid} from {path} ({time.time() - t0:.1f} s); stance {stance}, arm {rec['arm']}, tasks {rec['tasks']}", flush=True)

    checks = []

    def compare(name, saved, got):
        ok = saved == got
        checks.append((name, ok))
        print(f"  {'MATCH' if ok else 'DIFFERENT'}  {name}: saved {str(saved)[:110]} | replayed {str(got)[:110]}", flush=True)

    t1 = time.time()
    r = fc.check(stance, rec["points"], rec["objects"], approach_bin=rec["approach_bin"], all_angles=rec["all_angles"], use_cache=False, seed=seed)
    print(f"whole check replayed in {time.time() - t1:.1f} s", flush=True)
    compare("ok", rec["result"]["ok"], r["ok"])
    compare("failed", rec["result"]["failed"], r["failed"])
    compare("reason", rec["result"]["reason"], r["reason"])
    compare("config in force", rec["config"], sfl.jsonable(fc._config_in_force()))
    if rec.get("independent"):
        for g in GROUPS:
            fc._restore(start)
            t1 = time.time()
            rg = fc.check(stance, rec["points"], rec["objects"], approach_bin=rec["approach_bin"], all_angles=rec["all_angles"], only=g, seed=seed)
            saved = rec["independent"][g]
            print(f"group {g} replayed in {time.time() - t1:.1f} s", flush=True)
            compare(f"{g}: ok", saved["ok"], rg["ok"])
            compare(f"{g}: failed", saved["failed"], rg["failed"])
            compare(f"{g}: reason", saved["reason"], rg["reason"])
            compare(f"{g}: measured", saved["measured"], sfl.jsonable(fc._measure.get(g)))
    bad = [n for n, ok in checks if not ok]
    print(f"RESULT {rid}: {len(checks) - len(bad)} of {len(checks)} saved fields reproduced" + ("" if not bad else f"; DIFFERENT: {bad}"), flush=True)
    print(f"total {time.time() - t0:.1f} s", flush=True)
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
