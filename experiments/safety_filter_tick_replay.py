"""Phase K replay benchmark: the filter tick (resolver step, then the braking layer) over the recorded approach poses of brkZ8 (rows 1 to N,
before the grasp, so no held object). Same objects as the plugin builds (solver, bridge speed bound, feed bound to the lab map). The world is
the lab map's fixtures only (the beaker is not in the recording), the same for every variant. Prints per-tick mean, p95, max (ms) for the
resolver, the braking layer and both together, and saves the commands sent (for a bit-for-bit comparison between variants).
Usage: python replay_k.py <out json> <out npy> [N]"""
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path("/home/vboxuser/projects/openarm-tracer-sim-sf-d")
sys.path.insert(0, str(ROOT))
OUT_JSON, OUT_NPY = sys.argv[1], sys.argv[2]
N = int(sys.argv[3]) if len(sys.argv) > 3 else 3200

from src import task_config as tc  # noqa: E402
from src.safety_filter import config as sfcfg  # noqa: E402
from src.safety_filter.arm_conversion import make_solver  # noqa: E402
from src.safety_filter.braking import BrakingLayer, ObstacleFeed, make_tool_fn  # noqa: E402
from src.safety_filter.resolver import Resolver  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes, RobotState  # noqa: E402
from src.safety_filter.world import LabMapSource  # noqa: E402

rec = np.load(ROOT / "data" / "safety_filter" / "plugin_runs" / "brkZ8_phaseI_20261117_193334.npz", allow_pickle=True)
bcfg = tc.COMMAND_BRIDGE_CFG
speed = float(bcfg["max_joint_velocity_rad_s"]) if bcfg["enabled"] else None
dt = 1.0 / 150.0
shapes = RobotShapes(load_robot_only_model(), ROOT / sfcfg.SHAPES_FILE)
resolver = Resolver(shapes, dt, make_solver(), joint_speed_rad_s=speed)
feed_shapes = RobotShapes(load_robot_only_model(), ROOT / sfcfg.SHAPES_FILE)
braking = BrakingLayer(make_tool_fn(feed_shapes))
feed = ObstacleFeed(feed_shapes)
world = LabMapSource(str(ROOT / sfcfg.LAB_MAP_FILE)).objects()
feed.bind(world, None)

t_res, t_brk, t_all = [], [], []
sent = np.zeros((N, 2, 7))
statuses = []
last = np.asarray(rec["sent"][0], float)
for k in range(1, N):
    b = rec["base"][k]
    state = RobotState(base_xyz=tuple(b[:3]), base_quat=tuple(b[3:7]), pedestal_m=float(rec["pedestal"][k]),
                       q_left=tuple(rec["q_meas"][k, 0]), q_right=tuple(rec["q_meas"][k, 1]),
                       finger_left=float(rec["fingers"][k, 0]), finger_right=float(rec["fingers"][k, 1]))
    raw = np.asarray(rec["raw"][k], float)
    t0 = time.process_time()
    filt, status = resolver.step(raw, last, state, world, None)
    t1 = time.process_time()
    filt = np.asarray(filt, float)
    out, _info = braking.process(last, filt, state, feed, hold=(status.status == "HOLD"), call_dt=dt)
    t2 = time.process_time()
    t_res.append(t1 - t0)
    t_brk.append(t2 - t1)
    t_all.append(t2 - t0)
    sent[k] = out
    statuses.append(status.status)
    last = np.asarray(out, float)

def stats(v):
    a = 1e3 * np.asarray(v)
    return {"n": int(len(a)), "mean": float(a.mean()), "p50": float(np.percentile(a, 50)), "p95": float(np.percentile(a, 95)), "max": float(a.max())}

res = {"resolver": stats(t_res), "braking": stats(t_brk), "tick": stats(t_all),
       "statuses": {s: statuses.count(s) for s in sorted(set(statuses))}}
np.save(OUT_NPY, sent)
json.dump(res, open(OUT_JSON, "w"), indent=1)
print("REPLAY", json.dumps(res))
