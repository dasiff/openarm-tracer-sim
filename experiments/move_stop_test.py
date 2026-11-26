"""Controlled move-then-stop test (stage 2 teleop follow-up, David 2026-11-25, item 1c). The left arm moves from the home pose to a pose 0.3 rad away on
joints 1-4 together, at a set joint speed, stops, holds 3 s, moves back, stops, holds 3 s; repeated 4 times. The bridge is on. Run it under
experiments/climb_perturb_diag.py (which records every sag-layer call and applies the deliberate model error with --perturb, and the correction
switch --correction on|off):

    python experiments/climb_perturb_diag.py run --out DIR/calls.npz [--perturb] --correction on -- experiments/move_stop_test.py --speed 0.25 --out DIR

Writes DIR/move.npz (per tick: time, measured left joints, the hand error against the reference, the reference's pad position minus the measured
pad) and DIR/schedule.json (the stop times and the joint targets). Analysis only; the layer and the loop are not changed.
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

import numpy as np  # noqa: E402

from src import sag_layer  # noqa: E402
from src.control_loop import run_loop  # noqa: E402
from src.robot_helpers import arm_ids  # noqa: E402
from src.task_config import TASK_CFG  # noqa: E402
from experiments_loader import load, steps_per_frame, SPAWN  # noqa: E402

ROUTES = {k: "move" for k in ("base", "pedestal", "arms.left", "arms.right", "gripper.left", "gripper.right")}
TICK_HZ = 150.0
SOURCE_HZ = 30.0
HOME = np.asarray(TASK_CFG["home_q_left"], float)
DIRECTION = np.array([1.0, 1.0, 1.0, -1.0, 0.0, 0.0, 0.0])      # joints 1-4 together, each 0.3 rad (joint 4 down: +0.3 would pass its 2.44 rad limit from home)
AMPLITUDE = 0.3                                                 # rad
SETUP_S = 4.0                                                   # s: min-jerk from the rest pose to home
SETTLE_S = 3.0                                                  # s: hold at home before the first move
HOLD_S = 3.0                                                    # s: hold after each move (the stop)
CYCLES = 4


def min_jerk(u):
    u = min(max(u, 0.0), 1.0)
    return 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5


def build_schedule(speed, q_rest):
    """Returns (function t -> joint target, list of stops with their times and targets, total duration)."""
    out = HOME + AMPLITUDE * DIRECTION
    tm = AMPLITUDE / speed
    segs = []                                                   # (t_start, t_end, from, to, kind)
    t = 0.0
    segs.append((t, t + SETUP_S, q_rest, HOME, "setup")); t += SETUP_S
    segs.append((t, t + SETTLE_S, HOME, HOME, "hold")); t += SETTLE_S
    stops = []
    for c in range(CYCLES):
        segs.append((t, t + tm, HOME, out, "move")); t += tm
        stops.append({"t": t, "target": out.tolist(), "cycle": c, "leg": "out"})
        segs.append((t, t + HOLD_S, out, out, "hold")); t += HOLD_S
        segs.append((t, t + tm, out, HOME, "move")); t += tm
        stops.append({"t": t, "target": HOME.tolist(), "cycle": c, "leg": "back"})
        segs.append((t, t + HOLD_S, HOME, HOME, "hold")); t += HOLD_S

    def target(tt):
        for (a_, b_, p_, q_, kind) in segs:
            if a_ <= tt < b_:
                if kind == "setup":
                    return p_ + (q_ - p_) * min_jerk((tt - a_) / (b_ - a_))
                if kind == "move":
                    return p_ + (q_ - p_) * min(1.0, (tt - a_) / (b_ - a_))
                return np.asarray(q_, float)
        return np.asarray(segs[-1][3], float)
    return target, stops, t


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, required=True, help="joint speed of the moves (rad/s), e.g. 0.1, 0.25, 0.5")
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    model, data = load()
    spf = steps_per_frame(model)
    arm = arm_ids(model, "left")
    qadr = np.asarray(arm["qadr"])
    state = {"target": None, "q_rest": None, "t0": None}

    def step(obs):
        if state["q_rest"] is None:
            state["q_rest"] = np.asarray(obs["joint_angles"][qadr], float).copy()
            state["t0"] = float(obs["time"])                       # the schedule starts at the first call (the loop's clock is not zero here)
            state["target"], state["stops"], state["total"] = build_schedule(a.speed, state["q_rest"])
        q = state["target"](float(obs["time"]) - state["t0"])
        return {"mode": "arms", "arm_targets": {"left": [float(x) for x in q]}}

    src = {"name": "move", "step": step, "hz": SOURCE_HZ, "state": {}}
    rec = {"t": [], "q": [], "herr": [], "rerr": [], "ref_pad": [], "meas_pad": []}

    def hook(obs, loop):
        layer = sag_layer.SAG_LAYERS.get("left")
        if layer is None:
            return None
        rec["t"].append(float(obs["time"]) - (state["t0"] or 0.0))   # time from the start of the schedule
        rec["q"].append(np.asarray(data.qpos[qadr], float).copy())
        pad, Rp = layer.arm.pad_pose(data)
        if layer.refs:
            ref = layer.refs[-1][1]                              # the bridge's accepted target: output minus dq
            rpad, Rr = layer._pose(data.qpos, ref)
            rec["herr"].append(np.asarray(rpad - pad, float))
            rec["ref_pad"].append(np.asarray(rpad, float))
        else:
            rec["herr"].append(np.full(3, np.nan))
            rec["ref_pad"].append(np.full(3, np.nan))
        rec["meas_pad"].append(np.asarray(pad, float).copy())
        return None

    total_s = SETUP_S + SETTLE_S + CYCLES * 2 * (AMPLITUDE / a.speed + HOLD_S)
    cfg = {"source_hz": 1.0 / model.opt.timestep / spf, "spawn_pos": SPAWN, "cameras": False, "routes": ROUTES,
           "bench_frames": int(round((total_s + 0.5) * TICK_HZ)), "on_tick": hook}
    res = run_loop(model, data, {"move": src}, cfg, viewer=None)
    # the schedule, once the first call has set it
    stops = state.get("stops", [])
    lim = model.jnt_range[np.asarray(arm["jnt"]), :]
    out_pose = HOME + AMPLITUDE * DIRECTION
    inside = bool(np.all(out_pose >= lim[:, 0]) and np.all(out_pose <= lim[:, 1]))
    np.savez(out / "move.npz", **{k: np.array(v) for k, v in rec.items() if len(v)})
    (out / "schedule.json").write_text(json.dumps({"speed": a.speed, "stops": stops, "total_s": total_s, "home": HOME.tolist(),
                                                   "out_pose_inside_limits": inside, "setup_from": None if state["q_rest"] is None else state["q_rest"].tolist(),
                                                   "bridge_log": res.get("bridge_log", []), "stopped_by": res.get("stopped_by")}, indent=1, default=str))
    print(f"move_stop_test: speed {a.speed} rad/s, {len(stops)} stops, {len(rec['t'])} ticks, out pose inside limits {inside}, stopped_by {res.get('stopped_by')}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
