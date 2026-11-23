"""Why the four ARMS-stall tests do not reach home while the held run does (SF-120 trace, 2026-11-23). Read-only analysis.

Reads per-layer-call recordings (experiments/gap_split_instrument.py: left_t, left_q_src, left_q_sent, left_q_meas, and the right arm)
and writes, for each run: a table every 0.5 s (both arms: the commanded joints, the solver output, the measured joints, and which
solver constraint is active per joint), the start pose, the commanded home pose with its distance to each joint's margin and hard limit,
how long joint 2 is pinned at its margin, when the commanded path reaches home (the advance check), when the measured joints are within
arms_tol of home (what ARMS waits for), and the divergence time between runs.

    python experiments/arms_stall_trace.py --run LABEL=PATH.npz [--run ...] --out DIR [--collision SCENARIO_LABEL]

Constraint letters per joint at each sample: M = the solver output sits on its margin bound (within 1e-3 rad);
C = the solver output moved at its per-joint speed cap on the last tick (within 1%); X = neither, but the solver output differs from the
command by more than 0.02 rad (the other constraints: collision avoidance, posture, or the redundancy choice); . = none.

--collision LABEL: with MuJoCo (one process), sets the left arm to the solver output of that run at 0.5 s samples and reports the
distance from the left arm's links, hand and fingers to every non-arm geom (the bench, hotplate, beaker and so on) within 0.05 m, the
distance at which the solver's collision avoidance starts (MINK_CFG detect_distance). Uses the default scene from experiments_loader.load().
"""

import argparse
import csv
import json
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "experiments"))

import numpy as np  # noqa: E402

from src.task_config import MINK_CFG, STAGING_CFG, TASK_CFG  # noqa: E402

HZ = 150.0
CAP_PER_TICK = float(MINK_CFG["max_joint_speed"]) / HZ          # rad per tick at the solver's per-joint speed cap
MARGIN = float(MINK_CFG["joint_margin_rad"])
ARMS_TOL = float(STAGING_CFG["arms_tol"])
# hard joint limits per arm, from models/robot/combined_robot.xml (the right arm's joints 1 and 2 are mirrored)
LIM = {
    "left": np.array([[-3.490659, 1.396263], [-3.316125, 0.174533], [-1.570796, 1.570796], [0.0, 2.443461],
                      [-1.570796, 1.570796], [-0.785398, 0.785398], [-1.570796, 1.570796]]),
    "right": np.array([[-1.396263, 3.490659], [-0.174533, 3.316125], [-1.570796, 1.570796], [0.0, 2.443461],
                       [-1.570796, 1.570796], [-0.785398, 0.785398], [-1.570796, 1.570796]]),
}
HOME = {"left": np.asarray(TASK_CFG["home_q_left"], float), "right": np.asarray(TASK_CFG["home_q_right"], float)}
SIDES = ("left", "right")


def margin_bounds(side):
    return LIM[side][:, 0] + MARGIN, LIM[side][:, 1] - MARGIN


def load_run(path):
    d = np.load(path)
    run = {"t": d["left_t"]}
    for s in SIDES:
        run[s] = {k: d[f"{s}_{k}"] for k in ("q_src", "q_sent", "q_meas")}
    return run


def letters(src, sent, prev_sent, side):
    lo, hi = margin_bounds(side)
    out = []
    for j in range(7):
        on_margin = abs(sent[j] - hi[j]) < 1e-3 or abs(sent[j] - lo[j]) < 1e-3
        at_cap = prev_sent is not None and abs(sent[j] - prev_sent[j]) >= 0.99 * CAP_PER_TICK
        if on_margin:
            out.append("M")
        elif at_cap:
            out.append("C")
        elif abs(sent[j] - src[j]) > 0.02:
            out.append("X")
        else:
            out.append(".")
    return "".join(out)


def summarise(name, run):
    t = run["t"]
    res = {"run": name, "calls": int(len(t)), "t_first": round(float(t[0]), 3), "t_last": round(float(t[-1]), 3)}
    for s in SIDES:
        a = run[s]
        src, sent, meas = a["q_src"], a["q_sent"], a["q_meas"]
        home = HOME[s]
        lo, hi = margin_bounds(s)
        r = {}
        r["start_meas"] = np.round(meas[0], 3).tolist()
        r["home_cmd"] = np.round(home, 3).tolist()
        r["home_to_margin_lo_hi_rad"] = [np.round(home - lo, 3).tolist(), np.round(hi - home, 3).tolist()]
        r["home_to_hard_lo_hi_rad"] = [np.round(home - LIM[s][:, 0], 3).tolist(), np.round(LIM[s][:, 1] - home, 3).tolist()]
        # the commanded path: when the commanded joints first reach home (the advance check passes; the path is complete)
        cmd_home = np.where(np.max(np.abs(src - home), axis=1) < 1e-3)[0]
        r["cmd_reaches_home_t"] = round(float(t[cmd_home[0]]), 3) if len(cmd_home) else None
        # what ARMS waits for: every measured joint within arms_tol of home
        meas_home = np.where(np.max(np.abs(meas - home), axis=1) < ARMS_TOL)[0]
        r["meas_within_arms_tol_t"] = round(float(t[meas_home[0]]), 3) if len(meas_home) else None
        r["meas_max_dev_from_home_at_last"] = round(float(np.max(np.abs(meas[-1] - home))), 3)
        r["meas_dev_from_home_at_last"] = np.round(meas[-1] - home, 3).tolist()
        # joint pinned at its margin (per joint: ticks on the margin bound, first and last time)
        pinned = {}
        for j in range(7):
            on = np.where((np.abs(sent[:, j] - hi[j]) < 1e-3) | (np.abs(sent[:, j] - lo[j]) < 1e-3))[0]
            if len(on):
                pinned[f"j{j + 1}"] = {"ticks": int(len(on)), "fraction": round(len(on) / len(t), 3),
                                       "first_t": round(float(t[on[0]]), 3), "last_t": round(float(t[on[-1]]), 3)}
        r["margin_pinned"] = pinned
        cap_ticks = np.where(np.abs(np.diff(sent, axis=0)).max(axis=1) >= 0.99 * CAP_PER_TICK)[0]
        r["cap_ticks"] = int(len(cap_ticks))
        r["cap_first_t"] = round(float(t[cap_ticks[0]]), 3) if len(cap_ticks) else None
        # 0.5 s grid
        grid = []
        for k in range(len(t)):
            prev_sent = sent[k - 1] if k > 0 else None
            if k % int(round(0.5 * HZ)) == 0 or k == len(t) - 1:
                grid.append({"t": round(float(t[k]), 2), "cmd": np.round(src[k], 3).tolist(), "sent": np.round(sent[k], 3).tolist(),
                             "meas": np.round(meas[k], 3).tolist(),
                             "letters": letters(src[k], sent[k], prev_sent, s),
                             "max_dev_home": round(float(np.max(np.abs(meas[k] - home))), 3)})
        r["grid"] = grid
        res[s] = r
    return res


def divergence(a, b):
    """First time the solver outputs of two runs differ (left arm), on their common ticks."""
    n = min(len(a["t"]), len(b["t"]))
    diff = np.max(np.abs(a["left"]["q_sent"][:n] - b["left"]["q_sent"][:n]), axis=1)
    idx = np.where(diff > 1e-6)[0]
    return None if len(idx) == 0 else round(float(a["t"][idx[0]]), 3)


def collision_check(samples):
    """samples: {label: [(t, q_left_sent_7, q_right_sent_7), ...]}. For each sample, the nearest left-arm geom to (a) the right arm and
    (b) any other non-arm geom (bench, hotplate, beaker, ...), within 0.05 m. Scene: experiments_loader.load(). One MuJoCo process."""
    import mujoco
    from src.robot_helpers import arm_ids
    from experiments_loader import load  # noqa: E402
    model, data = load()
    qadr = {s: np.asarray(arm_ids(model, s)["qadr"]) for s in SIDES}
    body_name = lambda b: mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or "world"
    # the same sets as src/mink_arm.py _collision_limits (the solver's own collision pairs)
    col = [g for g in range(model.ngeom) if model.geom_contype[g] or model.geom_conaffinity[g]]
    bn = lambda g: body_name(model.geom_bodyid[g])
    left_geoms = [g for g in col if bn(g).startswith("openarm_left_") and not bn(g).endswith(("link0", "link1", "link2"))]
    right_geoms = [g for g in col if bn(g).startswith("openarm_right_") and not bn(g).endswith("link0")]
    furniture_geoms = [g for g in col if bn(g) in ("bench", "task_hotplate")]
    out = {}
    saved = data.qpos.copy()
    fromto = np.zeros(6)

    def nearest(geoms_b):
        best = (np.inf, None, None)
        for g in left_geoms:
            for og in geoms_b:
                d = mujoco.mj_geomDistance(model, data, g, og, 0.10, fromto)
                if d < best[0]:
                    best = (d, body_name(model.geom_bodyid[g]), body_name(model.geom_bodyid[og]))
        return best

    for label, pts in samples.items():
        rows = []
        for t, ql, qr in pts:
            data.qpos[:] = saved
            data.qpos[qadr["left"]] = ql
            data.qpos[qadr["right"]] = qr
            mujoco.mj_forward(model, data)
            dr = nearest(right_geoms)
            df = nearest(furniture_geoms)
            fmt = lambda d: None if not np.isfinite(d[0]) else round(float(d[0]), 4)
            rows.append({"t": round(float(t), 2), "left_to_right_arm_m": fmt(dr), "right_pair": [dr[1], dr[2]],
                         "left_to_furniture_m": fmt(df), "furniture_pair": [df[1], df[2]]})
        out[label] = rows
    data.qpos[:] = saved
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", action="append", required=True, help="LABEL=PATH.npz")
    ap.add_argument("--out", required=True)
    ap.add_argument("--collision", default=None, help="run label whose solver output is checked against the scene (MuJoCo)")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    runs = {}
    for spec in a.run:
        label, path = spec.split("=", 1)
        runs[label] = load_run(path)
    summary = {label: summarise(label, run) for label, run in runs.items()}
    labels = list(runs)
    div = {}
    for i in range(len(labels)):
        for j in range(i + 1, len(labels)):
            div[f"{labels[i]} vs {labels[j]}"] = divergence(runs[labels[i]], runs[labels[j]])
    summary["divergence_first_t_left_solver_output"] = div
    with open(out / "arms_stall_trace.json", "w") as f:
        json.dump(summary, f, indent=1)
    for label, s in summary.items():
        if label.startswith("divergence"):
            continue
        with open(out / f"{label}_grid.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "side"] + [f"cmd_q{j}" for j in range(1, 8)] + [f"sent_q{j}" for j in range(1, 8)]
                       + [f"meas_q{j}" for j in range(1, 8)] + ["constraints_j1_to_j7 (M margin, C cap, X other, . none)", "max_dev_home"])
            for side in SIDES:
                for row in s[side]["grid"]:
                    w.writerow([row["t"], side, *row["cmd"], *row["sent"], *row["meas"], row["letters"], row["max_dev_home"]])
    if a.collision:
        if a.collision not in runs:
            raise SystemExit(f"--collision: no run called {a.collision}")
        pts = {}
        for label, run in runs.items():
            ks = list(range(0, len(run["t"]), int(round(0.5 * HZ)))) + [len(run["t"]) - 1]
            pts[label] = [(float(run["t"][k]), run["left"]["q_sent"][k], run["right"]["q_sent"][k]) for k in ks]
        cc = collision_check(pts)
        with open(out / "collision_check.json", "w") as f:
            json.dump(cc, f, indent=1)
        print("collision check written:", out / "collision_check.json", flush=True)
    print("wrote", out / "arms_stall_trace.json", flush=True)


if __name__ == "__main__":
    main()
