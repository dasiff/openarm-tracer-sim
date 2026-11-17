"""Look-ahead timing (Braking Phase F fixes 4 to 6): the brake layer with its obstacle feed, over fixed recorded poses, per configuration.

Inputs: the commands, measured joints and base/pedestal/finger states of the recording data/safety_filter/plugin_runs/<run>.npz (every
`step`-th call), the lab map's world objects (bench, hotplate; no held object: the held beaker adds cylinder pairs, see the integration
run). The same inputs for every configuration; each configuration starts from a fresh layer and feed.

Configurations (switches set at run time): compiled distances (cfg.GEOMETRY) on or off; adaptive wake (cfg.BRAKING) on or off; HOLD on
(every call is a HOLD trigger) or off. For each: mean, p95 and max time per call (ms) and the pairs whose gap is computed per call.

Run: python experiments/safety_filter_perf_bench.py <run name> [--step 30] [--rows START:END]
--rows START:END (Phase J): every recorded call in that row range, in order (consecutive calls, as the filter sees them); the poses of a
--step run are 30 calls apart, which makes every measured speed a jump and fills the log with emergencies: use --rows for timing.
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter import config as sfcfg  # noqa: E402
from src.safety_filter.braking import BrakingLayer, ObstacleFeed, make_tool_fn  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes, RobotState  # noqa: E402
from src.safety_filter.world import LabMapSource  # noqa: E402

ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
ap.add_argument("run")
ap.add_argument("--step", type=int, default=30)
ap.add_argument("--rows", default=None, help="START:END recorded rows, every call in order (Phase J)")
args = ap.parse_args()

d = np.load(ROOT / "data" / "safety_filter" / "plugin_runs" / f"{args.run}.npz", allow_pickle=True)
if args.rows:
    lo_r, hi_r = (int(x) for x in args.rows.split(":"))
    idx = np.arange(max(lo_r, 1), min(hi_r, len(d["raw"])), 1)
else:
    idx = np.arange(1, len(d["raw"]), args.step)
raw, sent, q_meas = d["raw"], d["sent"], d["q_meas"]     # loaded once
base, pedestal, fingers = d["base"], d["pedestal"], d["fingers"]     # loaded once (an npz member is read again on every access)
states = []
for i in idx:
    b = base[i]
    states.append(RobotState(base_xyz=tuple(b[:3]), base_quat=tuple(b[3:7]), pedestal_m=float(pedestal[i]),
                             q_left=tuple(q_meas[i, 0]), q_right=tuple(q_meas[i, 1]),
                             finger_left=float(fingers[i, 0]), finger_right=float(fingers[i, 1])))
samples = [(sent[i - 1], raw[i], states[k]) for k, i in enumerate(idx)]
print(f"{len(samples)} poses from {args.run} (every {args.step}th call)")

shapes = RobotShapes(load_robot_only_model(), ROOT / sfcfg.SHAPES_FILE)
world = LabMapSource(str(ROOT / "data" / "safety_filter" / "lab_map.json")).objects()


def run(compiled, adaptive, hold):
    """Times the whole look-ahead tick (BrakingLayer.process with the feed) over the samples; returns per-call times and pair counts."""
    cfg.GEOMETRY["compiled"] = compiled
    cfg.BRAKING["adaptive_wake"] = adaptive
    lay = BrakingLayer(make_tool_fn(shapes))
    feed = ObstacleFeed(shapes)
    feed.bind(world, None)
    lay.process(samples[0][0], samples[0][1], samples[0][2], feed, hold=hold)         # warm-up (compiles the kernels on the first call)
    feed.n_calls = feed.n_considered = 0
    times, pairs = [], []
    for last, cmd, st in samples:
        n0 = feed.n_considered
        t0 = time.perf_counter()
        lay.process(last, cmd, st, feed, hold=hold)
        times.append(time.perf_counter() - t0)
        pairs.append(feed.n_considered - n0)
    return np.array(times) * 1e3, np.array(pairs)


rows = [("before: Python distances, static cutoffs", False, False, False),
        ("compiled distances, static cutoffs", True, False, False),
        ("compiled distances, adaptive wake (fix 5)", True, True, False),
        ("compiled, adaptive, HOLD with the static cutoffs", True, False, True),
        ("compiled, adaptive, HOLD with the margin rule (fix 6)", True, True, True)]
saved = (cfg.GEOMETRY["compiled"], cfg.BRAKING["adaptive_wake"])
print(f"{'configuration':58s} {'mean ms':>8s} {'p95 ms':>8s} {'max ms':>8s} {'pairs/call':>10s} {'pairs p95':>10s}")
try:
    for name, comp, ad, hold in rows:
        t, p = run(comp, ad, hold)
        print(f"{name:58s} {t.mean():8.2f} {np.percentile(t, 95):8.2f} {t.max():8.2f} {p.mean():10.1f} {np.percentile(p, 95):10.1f}", flush=True)
finally:
    cfg.GEOMETRY["compiled"], cfg.BRAKING["adaptive_wake"] = saved
