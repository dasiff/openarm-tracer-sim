"""Finds two extra left-arm poses for the gap split (SF-120 step 5). Each is at least 20 deg from every hard joint limit,
collision-free (no penetrating contact between a left-arm body and anything but its own arm), with the pad at z >= 1.1 m,
and within 0.6 rad (joint space) of the home pose. Seeded, so the same two poses come back. Measurement helper only:
it runs no simulation, only mj_forward.

    python experiments/gap_split_pose_search.py --out data/logs/gap_split_poses.json
"""

import argparse
import json
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "experiments"))
os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from src import sag_layer  # noqa: E402
from src.robot_helpers import arm_ids  # noqa: E402
from src.task_config import TASK_CFG  # noqa: E402
from experiments_loader import load  # noqa: E402

CLEAR_RAD = np.deg2rad(20.0)     # required distance from every hard joint limit
PAD_Z_MIN = 1.1                  # m, the pad height floor
NEAR_RAD = 0.6                   # rad, max joint-space distance from home


FIXED_TORSO = "openarm_body_link0"   # the torso link is fixed to the base: its contacts do not depend on the arm pose


def left_arm_contact(model, data):
    """True if a penetrating contact joins a left-arm body to a body outside the left arm (not the fixed torso link).
    Contacts between two left-arm bodies (the gripper's own fingers, the arm's own links) are not counted."""
    for i in range(data.ncon):
        c = data.contact[i]
        if c.dist >= 0:
            continue
        b1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[c.geom1]) or "world"
        b2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[c.geom2]) or "world"
        left1, left2 = b1.startswith("openarm_left"), b2.startswith("openarm_left")
        other = b2 if left1 else b1
        if left1 != left2 and other != FIXED_TORSO:
            return True
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seed", type=int, default=120)
    ap.add_argument("--tries", type=int, default=40000)
    a = ap.parse_args()

    model, data = load()
    arm = arm_ids(model, "left")
    jnt, qadr = np.asarray(arm["jnt"]), np.asarray(arm["qadr"])
    hard_lo = model.jnt_range[jnt, 0].copy()
    hard_hi = model.jnt_range[jnt, 1].copy()
    home = np.asarray(TASK_CFG["home_q_left"], float)
    probe = sag_layer.SagLayer(model, mujoco.MjData(model), "left", home)

    rng = np.random.default_rng(a.seed)
    lo_c = hard_lo + CLEAR_RAD
    hi_c = hard_hi - CLEAR_RAD
    found = []
    tried = 0
    saved_q = data.qpos.copy()
    for _ in range(a.tries):
        tried += 1
        q = home + rng.uniform(-NEAR_RAD, NEAR_RAD, size=7) / np.sqrt(7.0)
        if np.any(q < lo_c) or np.any(q > hi_c):
            continue
        if np.linalg.norm(q - home) > NEAR_RAD:
            continue
        data.qpos[:] = saved_q
        data.qpos[qadr] = q
        mujoco.mj_forward(model, data)
        p, _ = probe._pose(data.qpos, q)
        if p[2] < PAD_Z_MIN:
            continue
        if left_arm_contact(model, data):
            continue
        # keep two poses that differ from each other by at least 0.3 rad (joint space), so they are not near-duplicates
        if all(np.linalg.norm(q - f) > 0.3 for f in found):
            found.append(q.copy())
            print(f"candidate {len(found)} after {tried} tries: q={np.round(q, 3).tolist()} pad={np.round(p, 3).tolist()}", flush=True)
        if len(found) == 2:
            break
    data.qpos[:] = saved_q
    if len(found) < 2:
        raise SystemExit(f"only {len(found)} poses found in {tried} tries")

    poses = {"new1": found[0].round(4).tolist(), "new2": found[1].round(4).tolist()}
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(".tmp")
    tmp.write_text(json.dumps(poses, indent=1))
    tmp.replace(out)
    print("wrote", out, poses, flush=True)


if __name__ == "__main__":
    main()
