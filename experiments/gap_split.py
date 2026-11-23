"""Splits the hand-to-target gap into a solver part and a servo part (SF-120 diagnosis, 2026-11-23). Measurement only.

    python experiments/gap_split.py --pose home|mid|near|new1|new2 [--bypass] [--only SEG,SEG|valid|all]
                                    [--valid-report lag_report.json] [--poses-file poses.json] [--out DIR]
    python experiments/gap_split.py --analyse DIR --pose NAME

Setup as in lag_measure.py: left arm, 150 Hz, sag trim FORCED TO ZERO (solver still running), bridge off, safety filter off, no
held object. --bypass switches the layer off (SAG_LAYER_CFG enabled False): the source's joint targets go straight to the arm.

Per tick three poses are logged: (i) the source's target (q_src, its forward kinematics "ref"), (ii) the joint targets the
layer sent on (q_sent: the solver's output, or q_src when bypassed or when the solver failed), (iii) the measured joints (q_meas,
the pad "pad_meas"). Solver part = ref - pad_sent; servo part = pad_sent - pad_meas; total = ref - pad_meas. Each part is reported
as a lag time (along the motion, minus that part's rest droop, divided by the commanded speed) and in mm.

Schedule, per pose: setup (min-jerk, 10 s), settle 2 s, rest 5 s (droop), then the selected ramps and steps, each followed by a
min-jerk return and a 1 s hold. Ramps: 0.5 s settle, 1.5 s ramp, measured over the last 1 s. Steps: 2 s response.

--only: a comma list of segment names, 'valid' (the segments marked valid in --valid-report for this pose), or 'all'.
A ramp truncated by a hard joint limit is recorded and skipped in the analysis (never worked around).
"""

import argparse
import csv
import json
import math
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "experiments"))
os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

from src import sag_layer  # noqa: E402
from src.control_loop import run_loop  # noqa: E402
from src.robot_helpers import arm_ids  # noqa: E402
from src.task_config import MINK_CFG, SAG_LAYER_CFG, TASK_CFG  # noqa: E402
from experiments_loader import load, steps_per_frame, SPAWN  # noqa: E402

HZ = 150.0
DT = 1.0 / HZ
HOME = np.asarray(TASK_CFG["home_q_left"], float)
ROUTES = {k: "measure" for k in ("base", "pedestal", "arms.left", "arms.right", "gripper.left", "gripper.right")}
TRANS_DIRS = {"up": np.array([0, 0, 1.0]), "down": np.array([0, 0, -1.0]), "side": np.array([0, 1.0, 0]), "fwd": np.array([1.0, 0, 0])}
TRANS_SPEEDS = [0.025, 0.05, 0.1, 0.2, 0.4]
ROT_AXES = {"x": np.array([1.0, 0, 0]), "y": np.array([0, 1.0, 0]), "z": np.array([0, 0, 1.0])}
ROT_RATES = [0.1, 0.25, 0.5, 1.0]
STEP_M = 0.02
RAMP_S, MEAS_FROM_S, SETTLE_S, REST_S, POST_S = 1.5, 0.5, 0.5, 5.0, 1.0
SETUP_PEAK, RETURN_PEAK = 0.4, 0.5
SPEED_CAP = float(MINK_CFG["max_joint_speed"])              # rad/s, the solver's per-joint velocity limit
MARGIN = float(MINK_CFG["joint_margin_rad"])                # rad, the solver's distance from every hard joint limit


def seg_specs():
    out = []
    for dn, dv in TRANS_DIRS.items():
        for v in TRANS_SPEEDS:
            out.append({"name": f"trans_{dn}_{int(round(v * 1000))}mms", "kind": "trans", "dir": dv, "value": v})
    for ax, av in ROT_AXES.items():
        for w in ROT_RATES:
            out.append({"name": f"rot_{ax}_{str(w).replace('.', 'p')}radps", "kind": "rot", "dir": av, "value": w})
    for dn, dv in TRANS_DIRS.items():
        out.append({"name": f"step_{dn}", "kind": "step", "dir": dv, "value": STEP_M})
    return out


def poses_for(name, pose_defs):
    if name == "home":
        return HOME.copy()
    if name == "mid":
        q = HOME.copy()
        q[0] += 0.4
        q[3] -= 0.3
        return q
    if name == "near":
        q = HOME.copy()
        q[1] = -0.20
        return q
    if name in pose_defs:
        return np.asarray(pose_defs[name], float)
    raise ValueError(f"unknown pose {name}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", required=True)
    ap.add_argument("--bypass", action="store_true")
    ap.add_argument("--only", default="all")
    ap.add_argument("--valid-report", default=None)
    ap.add_argument("--poses-file", default=None)
    ap.add_argument("--out", default=str(PROJECT_ROOT / "data" / "logs" / "gap_split"))
    ap.add_argument("--analyse", default=None)
    a = ap.parse_args()
    if a.analyse:
        return analyse(Path(a.analyse), a.pose)
    pose_defs = json.load(open(a.poses_file)) if a.poses_file else {}
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    tag = f"{a.pose}_{'bypass' if a.bypass else 'solver'}"

    # forced trim: the layer runs (solver included), its trim output is zeroed in this process only
    real_step = sag_layer._sag_trim_step

    def zeroed(C, cfg, model, side, t, p_err, r_err, dt):
        real_step(C, cfg, model, side, t, p_err, r_err, dt)
        C["p"][:] = 0.0
        C["r"][:] = 0.0

    sag_layer._sag_trim_step = zeroed
    if a.bypass:
        SAG_LAYER_CFG["enabled"] = False

    last = {"src": None, "out": None}
    real_correct = sag_layer.SagLayer.correct

    def capture(self, q_src, data, t, nominal_dt):
        out_q = real_correct(self, q_src, data, t, nominal_dt)
        if self.side == "left":
            last["src"] = np.array(q_src, float)
            last["out"] = np.array(out_q, float)
        return out_q

    sag_layer.SagLayer.correct = capture

    model, data = load()
    spf = steps_per_frame(model)
    arm = arm_ids(model, "left")
    jnt = np.asarray(arm["jnt"])
    qadr = np.asarray(arm["qadr"])
    hard_lo = model.jnt_range[jnt, 0].copy()
    hard_hi = model.jnt_range[jnt, 1].copy()
    probe = sag_layer.SagLayer(model, mujoco.MjData(model), "left", HOME)
    pose_fn = lambda q: probe._pose(data.qpos, q)

    def ik(q_init, p_t, R_t, iters=40):
        q = np.array(q_init, float)
        for _ in range(iters):
            p, R = pose_fn(q)
            e = np.concatenate([p_t - p, Rotation.from_matrix(R_t @ R.T).as_rotvec()])
            if np.max(np.abs(e)) < 1e-9:
                break
            J = np.zeros((6, 7))
            for i in range(7):
                q2 = q.copy()
                q2[i] += 1e-6
                p2, R2 = pose_fn(q2)
                J[:3, i] = (p2 - p) / 1e-6
                J[3:, i] = Rotation.from_matrix(R2 @ R.T).as_rotvec() / 1e-6
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-6 * np.eye(6), e)
            q = q + dq
        p, R = pose_fn(q)
        res = np.concatenate([p_t - p, Rotation.from_matrix(R_t @ R.T).as_rotvec()])
        return q, float(np.max(np.abs(res)))

    def minjerk(q_a, q_b, dur):
        n = max(1, int(round(dur * HZ)))
        s = np.array([10 * (k / n) ** 3 - 15 * (k / n) ** 4 + 6 * (k / n) ** 5 for k in range(1, n + 1)])
        return [q_a + (q_b - q_a) * si for si in s]

    def duration_for(q_a, q_b, peak):
        return max(1.0, 1.875 * float(np.max(np.abs(q_b - q_a))) / peak)

    state = {"i": 0, "built": False, "sched": None, "labels": None, "meta": {}}

    def build(q_rest):
        q_pose = poses_for(a.pose, pose_defs)
        p0, R0 = pose_fn(q_pose)
        Q, L = [], []

        def add(qs, lab):
            for q in qs:
                Q.append(np.asarray(q, float))
                L.append(lab)

        add(minjerk(q_rest, q_pose, duration_for(q_rest, q_pose, SETUP_PEAK)), {"phase": "setup", "seg": "setup"})
        add([q_pose] * int(round(2.0 * HZ)), {"phase": "settle", "seg": "settle"})
        add([q_pose] * int(round(REST_S * HZ)), {"phase": "rest", "seg": "rest"})
        wanted = None if a.only == "all" else set(a.only.split(","))
        valid = None
        if a.only == "valid":
            rep = json.load(open(a.valid_report))
            valid = {c["seg"] for P in rep["poses"] if P["pose"] == a.pose for c in P["cells"] if c.get("valid")}
            wanted = valid
        skipped = {}
        for spec in seg_specs():
            if wanted is not None and spec["name"] not in wanted:
                continue
            name = spec["name"]
            if spec["kind"] == "step":
                q_step, res = ik(q_pose, p0 + STEP_M * spec["dir"], R0)
                add([q_pose] * int(round(0.5 * HZ)), {"phase": "pre", "seg": name})
                add([q_step] * int(round(2.0 * HZ)), {"phase": "step", "seg": name})
                add(minjerk(q_step, q_pose, duration_for(q_step, q_pose, RETURN_PEAK)), {"phase": "return", "seg": name})
                add([q_pose] * int(round(POST_S * HZ)), {"phase": "post", "seg": name})
                continue
            n = int(round(RAMP_S * HZ))
            q_prev = q_pose.copy()
            qs = []
            limit_at = None
            worst = 0.0
            for k in range(1, n + 1):
                t = k * DT
                if spec["kind"] == "trans":
                    q, res = ik(q_prev, p0 + spec["value"] * t * spec["dir"], R0)
                else:
                    q, res = ik(q_prev, p0, Rotation.from_rotvec(spec["dir"] * spec["value"] * t).as_matrix() @ R0)
                worst = max(worst, res)
                if np.any(q < hard_lo) or np.any(q > hard_hi):
                    limit_at = t
                    break
                qs.append(q)
                q_prev = q
            if limit_at is not None:
                skipped[name] = {"limit_at_s": limit_at}
                continue                                  # a direction that reaches a joint limit within the window is skipped
            add([q_pose] * int(round(SETTLE_S * HZ)), {"phase": "pre", "seg": name})
            add(qs, {"phase": "ramp", "seg": name})
            add(minjerk(qs[-1], q_pose, duration_for(qs[-1], q_pose, RETURN_PEAK)), {"phase": "return", "seg": name})
            add([q_pose] * int(round(POST_S * HZ)), {"phase": "post", "seg": name})
            state["meta"].setdefault("ik_worst", {})[name] = worst
        state["sched"] = np.array(Q)
        state["labels"] = L
        state["meta"].update({"q_pose": q_pose.tolist(), "p0_pose_m": p0.tolist(), "skipped_limit": skipped, "n_ticks": len(Q)})
        state["built"] = True

    def step(obs):
        q_now = np.array(obs["joint_angles"][qadr], float)
        if not state["built"]:
            build(q_now)
        i = state["i"]
        if i >= len(state["sched"]):
            return None
        state["i"] = i + 1
        return {"mode": "arms", "arm_targets": {"left": [float(x) for x in state["sched"][i]]}}

    rows = []

    def hook(obs, loop):
        if state["built"] and state["i"] >= len(state["sched"]):
            state["after"] = state.get("after", 0) + 1
            if state["after"] >= 5:
                loop["stop"]("schedule done")
        if not state["built"] or state["i"] < 1 or last["out"] is None:
            return
        i = state["i"] - 1
        lab = state["labels"][i]
        q_src = state["sched"][i]
        q_sent = last["out"]
        q_meas = np.array(data.qpos[qadr], float)
        pad_m, _ = layer_pose_meas()
        ref, _ = pose_fn(q_src)
        pad_s, _ = pose_fn(q_sent)
        passthrough = int(np.array_equal(q_sent, q_src))
        rows.append((i, float(obs["time"]), lab["phase"], lab["seg"], q_src, q_sent, q_meas, pad_m, pad_s, ref, passthrough))

    def layer_pose_meas():
        layer = sag_layer.SAG_LAYERS["left"]
        return layer.arm.pad_pose(data)

    cfg = {"source_hz": 1.0 / model.opt.timestep / spf, "spawn_pos": SPAWN, "cameras": False, "routes": ROUTES,
           "bench_frames": 10 ** 6, "on_tick": hook, "command_bridge": {"enabled": False}}
    res = run_loop(model, data, {"measure": {"name": "measure", "step": step}}, cfg, viewer=None)

    cols = ["i", "t", "phase", "seg"] + [f"qsrc{k}" for k in range(7)] + [f"qsent{k}" for k in range(7)] + [f"qmeas{k}" for k in range(7)] \
        + ["pm_x", "pm_y", "pm_z", "ps_x", "ps_y", "ps_z", "ref_x", "ref_y", "ref_z", "passthrough"]
    with open(out / f"{tag}_ticks.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for i, t, ph, seg, qs_, qsent, qm, pm, ps, ref, pt in rows:
            w.writerow([i, f"{t:.6f}", ph, seg] + [f"{x:.7f}" for x in qs_] + [f"{x:.7f}" for x in qsent] + [f"{x:.7f}" for x in qm]
                       + [f"{x:.8f}" for x in pm] + [f"{x:.8f}" for x in ps] + [f"{x:.8f}" for x in ref] + [pt])
    meta = dict(state["meta"], pose=a.pose, bypass=bool(a.bypass), only=a.only, stopped_by=res.get("stopped_by"),
                hard_lo=hard_lo.tolist(), hard_hi=hard_hi.tolist(), margin_rad=MARGIN, speed_cap=SPEED_CAP, ticks=len(rows))
    (out / f"{tag}_meta.json").write_text(json.dumps(meta, indent=2))
    print(f"gap_split {tag}: {len(rows)} ticks, schedule {len(state['sched'])}, skipped {list(state['meta'].get('skipped_limit', {}))}, stopped_by {res.get('stopped_by')}", flush=True)
    return 0


def analyse(out, pose):
    """Per-cell lag of each part (minus that part's rest droop), joint-level servo fit, solver cap and margin fractions."""
    import glob
    res = {}
    for tag in sorted(glob.glob(str(out / f"{pose}_*_ticks.csv"))):
        name = Path(tag).name.replace("_ticks.csv", "")
        meta = json.load(open(out / f"{name}_meta.json"))
        bypass = meta["bypass"]
        rows = list(csv.DictReader(open(tag)))
        seg = np.array([r["seg"] for r in rows])
        ph = np.array([r["phase"] for r in rows])
        t = np.array([float(r["t"]) for r in rows])
        col = lambda pre: np.array([[float(r[f"{pre}{k}"]) for k in range(7)] for r in rows]) if pre.startswith("q") else \
            np.array([[float(r[f"{pre}_{c}"]) for c in "xyz"] for r in rows])
        qsrc, qsent, qmeas = col("qsrc"), col("qsent"), col("qmeas")
        pm, ps, ref = col("pm"), col("ps"), col("ref")
        rest = np.where(ph == "rest")[0][-int(HZ):]
        d_tot = (ref - pm)[rest].mean(axis=0)
        d_sol = (ref - ps)[rest].mean(axis=0)
        d_srv = (ps - pm)[rest].mean(axis=0)
        qm_rest = qmeas[rest].mean(axis=0)
        qs_rest = (qsent - qmeas)[rest].mean(axis=0)
        cells = []
        for sname in dict.fromkeys(seg):
            if sname in ("setup", "settle", "rest") or sname.startswith("step_"):
                continue
            idx = np.where((seg == sname) & (ph == "ramp"))[0]
            if len(idx) < int(RAMP_S * HZ) - 1:
                cells.append({"seg": sname, "valid": False, "why": "ramp truncated"})
                continue
            kind = "trans" if sname.startswith("trans") else "rot"
            spec = [s for s in seg_specs() if s["name"] == sname][0]
            d = np.asarray(spec["dir"], float)
            v = float(spec["value"])
            w = idx[int(MEAS_FROM_S * HZ):]
            if kind == "trans":
                tot = (ref - pm)[w].mean(axis=0) - d_tot
                sol = (ref - ps)[w].mean(axis=0) - d_sol
                srv = (ps - pm)[w].mean(axis=0) - d_srv
                al = lambda x: float(x @ d)
                cells.append({"seg": sname, "valid": True, "kind": "trans", "speed_mps": v,
                              "tot_mm": 1000 * al(tot), "sol_mm": 1000 * al(sol), "srv_mm": 1000 * al(srv),
                              "tot_ms": 1000 * al(tot) / v, "sol_ms": 1000 * al(sol) / v, "srv_ms": 1000 * al(srv) / v,
                              "perp_tot_mm": 1000 * float(np.linalg.norm(tot - al(tot) * d))})
            else:
                # rotation part: the pad orientation is not logged, so only the solver/servo split of the pose error is not available here
                cells.append({"seg": sname, "valid": True, "kind": "rot", "rate_radps": v, "note": "orientation split not logged"})
            # joint-level servo lag: (q_sent - q_meas) against the measured joint speed in the ramp window
            qd = np.gradient(qmeas[idx], DT, axis=0)[int(MEAS_FROM_S * HZ):]
            e = (qsent - qmeas)[w] - qs_rest
            if kind == "trans" or True:
                slope = [float(np.polyfit(qd[:, j], e[:, j], 1)[0]) if np.std(qd[:, j]) > 1e-6 else None for j in range(7)]
                cells[-1]["servo_joint_lag_s"] = slope
            # solver cap and margin fractions (solver output, ramp window)
            dq = np.abs(np.diff(qsent[idx], axis=0)) * HZ
            cap_frac = float(np.mean(np.any(dq >= 0.99 * SPEED_CAP, axis=1)))
            lo_m = np.asarray(meta["hard_lo"]) + MARGIN
            hi_m = np.asarray(meta["hard_hi"]) - MARGIN
            near_m = np.any((qsent[idx] <= lo_m + 2e-3) | (qsent[idx] >= hi_m - 2e-3), axis=1)
            cells[-1]["solver_cap_frac"] = cap_frac
            cells[-1]["margin_frac"] = float(np.mean(near_m))
        steps = {}
        for sname in dict.fromkeys(seg):
            if not sname.startswith("step_"):
                continue
            idx = np.where((seg == sname) & (ph == "step"))[0]
            if len(idx) < 10:
                continue
            dn = np.asarray([s for s in seg_specs() if s["name"] == sname][0]["dir"], float)
            p0m = pm[idx[0]]
            p0s = ps[idx[0]]
            dm = (pm[idx] - p0m) @ dn
            ds = (ps[idx] - p0s) @ dn
            def t63(x):
                h = np.where(x >= 0.63 * STEP_M)[0]
                return float(h[0] / HZ) if len(h) else None
            steps[sname] = {"t63_sent_ms": None if t63(ds) is None else 1000 * t63(ds), "t63_meas_ms": None if t63(dm) is None else 1000 * t63(dm),
                            "disp2s_meas_mm": 1000 * float(dm[-1])}
        res[name] = {"bypass": bypass, "cells": cells, "steps": steps, "droop_tot_mm": (1000 * d_tot).tolist(),
                     "droop_sol_mm": (1000 * d_sol).tolist(), "droop_srv_mm": (1000 * d_srv).tolist(), "skipped_limit": meta.get("skipped_limit", {})}
    out_path = out / f"{pose}_analysis.json"
    out_path.write_text(json.dumps(res, indent=2))
    print(f"wrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
