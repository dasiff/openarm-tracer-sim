"""Measures the lag in the hand-to-target gap (SF-120 pre-work, 2026-11-23). Measurement only: no production code is changed.

    python experiments/lag_measure.py --pose home|mid|near [--out DIR]

Setup: the real loop (src/control_loop.run_loop), 150 Hz, left arm only, the sag trim FORCED TO ZERO (the layer still runs its
solver; its trim output is zeroed by a monkeypatch in this process only), command bridge off, safety filter off, no held object.

The source sends joint targets, one per tick, from a precomputed schedule:
  setup     min-jerk move from the resting pose to the test pose (10 s or longer; peak joint speed 0.4 rad/s)
  settle    hold the pose 2 s
  rest      hold the pose 5 s          -> rest droop: the settled gap (last 1 s)
  ramp      hand (pad centre) at constant speed v (25, 50, 100, 200, 400 mm/s) in four directions (up +z, down -z, sideways +y,
            forward +x; world frame): 0.5 s to settle, 1.5 s ramp, the gap is averaged over the last 1 s of the ramp
  rotation  hand orientation about x, y, z (world) at 0.1, 0.25, 0.5 and 1 rad/s (position held)
  step      a 20 mm hand step in each of the four directions, 2 s response
  return    min-jerk back to the pose (duration from peak joint speed 0.5 rad/s), then a 1 s hold, after every ramp and step
The commanded joint targets of a ramp come from a damped-least-squares inverse kinematics of the hand pose (numeric Jacobian of
the same forward kinematics the layer uses). Ramp targets are exact constant-speed paths of the commanded hand pose, so the
commanded speed is known by construction; the measured speed is the pad's.

Gap = forward kinematics of the last commanded target minus the measured pad centre (position, metres); rotation gap = rotation
vector of R_ref R_pad^T (radians). Gap minus rest droop is the quantity the brief asks for.

Limits: a joint outside its hard joint range truncates the segment (recorded as LIMIT at that time). An IK residual above 1e-4
is recorded (IK_FAIL). Nothing is worked around; both are reported in the summary.

Writes <out>/<pose>_ticks.csv (one row per tick) and <pose>_segments.json (one entry per segment).
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
from src.task_config import TASK_CFG  # noqa: E402
from experiments_loader import load, steps_per_frame, SPAWN  # noqa: E402

HZ = 150.0
DT = 1.0 / HZ
HOME = np.asarray(TASK_CFG["home_q_left"], float)
ROUTES = {k: "measure" for k in ("base", "pedestal", "arms.left", "arms.right", "gripper.left", "gripper.right")}
TRANS_DIRS = {"up": np.array([0, 0, 1.0]), "down": np.array([0, 0, -1.0]), "side": np.array([0, 1.0, 0]), "fwd": np.array([1.0, 0, 0])}
TRANS_SPEEDS = [0.025, 0.05, 0.1, 0.2, 0.4]                                   # m/s
ROT_AXES = {"x": np.array([1.0, 0, 0]), "y": np.array([0, 1.0, 0]), "z": np.array([0, 0, 1.0])}
ROT_RATES = [0.1, 0.25, 0.5, 1.0]                                             # rad/s
STEP_M = 0.02
RAMP_S, MEAS_FROM_S, SETTLE_S, REST_S, POST_S = 1.5, 0.5, 0.5, 5.0, 1.0
SETUP_PEAK, RETURN_PEAK = 0.4, 0.5                                            # rad/s
Z_MIN = 1.10                                                                  # the hand must stay above this world height (bench top 0.96)


def poses_for(name, home):
    """Test poses (7 joint angles). home: the held run's start pose; mid: a reach; near: joint 2 close to its upper limit."""
    if name == "home":
        return home.copy()
    if name == "mid":
        q = home.copy()
        q[0] += 0.4
        q[3] -= 0.3
        return q
    if name == "near":
        q = home.copy()
        q[1] = -0.20
        return q
    raise ValueError(name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", required=True, choices=["home", "mid", "near"])
    ap.add_argument("--out", default=str(PROJECT_ROOT / "data" / "logs" / "lag_measure"))
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    # the trim is forced to zero in this process (the layer still runs, its solver included)
    real = sag_layer._sag_trim_step

    def zeroed(C, cfg, model, side, t, p_err, r_err, dt):
        real(C, cfg, model, side, t, p_err, r_err, dt)
        C["p"][:] = 0.0
        C["r"][:] = 0.0

    sag_layer._sag_trim_step = zeroed

    model, data = load()
    spf = steps_per_frame(model)
    arm = arm_ids(model, "left")
    lo_hard = model.jnt_range[np.asarray(arm["jnt"]), 0]
    hi_hard = model.jnt_range[np.asarray(arm["jnt"]), 1]
    probe = sag_layer.SagLayer(model, mujoco.MjData(model), "left", HOME)
    pose_fn = lambda q: probe._pose(data.qpos, q)                       # (position, rotation) of joint targets q

    def ik(q_init, p_t, R_t, use_pos, use_rot, iters=40):
        q = np.array(q_init, float)
        for _ in range(iters):
            p, R = pose_fn(q)
            e = []
            if use_pos:
                e.append(p_t - p)
            if use_rot:
                e.append(Rotation.from_matrix(R_t @ R.T).as_rotvec())
            e = np.concatenate(e)
            if np.max(np.abs(e)) < 1e-9:
                break
            J = np.zeros((len(e), 7))
            eps = 1e-6
            for i in range(7):
                q2 = q.copy()
                q2[i] += eps
                p2, R2 = pose_fn(q2)
                cols = []
                if use_pos:
                    cols.append((p2 - p) / eps)
                if use_rot:
                    cols.append(Rotation.from_matrix(R2 @ R.T).as_rotvec() / eps)
                J[:, i] = np.concatenate(cols)
            lam = 1e-3
            dq = J.T @ np.linalg.solve(J @ J.T + lam * lam * np.eye(len(e)), e)
            q = q + dq
        p, R = pose_fn(q)
        res = []
        if use_pos:
            res.append(p_t - p)
        if use_rot:
            res.append(Rotation.from_matrix(R_t @ R.T).as_rotvec())
        return q, float(np.max(np.abs(np.concatenate(res))))

    def minjerk_path(q_a, q_b, dur):
        n = max(1, int(round(dur * HZ)))
        s = np.array([10 * (k / n) ** 3 - 15 * (k / n) ** 4 + 6 * (k / n) ** 5 for k in range(1, n + 1)])
        return [q_a + (q_b - q_a) * si for si in s]

    def duration_for(q_a, q_b, peak):
        return max(1.0, 1.875 * float(np.max(np.abs(q_b - q_a))) / peak)

    state = {"i": 0, "q": None, "sched": None, "labels": None, "segments": [], "built": False}

    def build(q_rest):
        Q, L, segs = [], [], []

        def add(qs, label):
            for q in qs:
                Q.append(np.asarray(q, float))
                L.append(label)

        q_pose = poses_for(a.pose, HOME)
        p0, R0 = pose_fn(q_pose)
        segs.append({"name": "setup", "from": q_rest.tolist(), "to": q_pose.tolist()})
        dur = duration_for(q_rest, q_pose, SETUP_PEAK)
        add(minjerk_path(q_rest, q_pose, dur), {"phase": "setup"})
        add([q_pose] * int(round(2.0 * HZ)), {"phase": "settle"})
        add([q_pose] * int(round(REST_S * HZ)), {"phase": "rest", "seg": "rest"})
        segs.append({"name": "rest", "hand_pose_mm": (1000 * p0).round(2).tolist(), "pose_z_m": float(p0[2]), "z_ok": bool(p0[2] >= Z_MIN)})

        def ramp_segment(seg_name, kind, axis_or_dir, value):
            """kind 'trans' (direction vector, speed m/s) or 'rot' (axis vector, rate rad/s)."""
            n = int(round(RAMP_S * HZ))
            q_prev = q_pose.copy()
            limit_at = None
            ik_worst = 0.0
            qs = []
            for k in range(1, n + 1):
                t = k * DT
                if kind == "trans":
                    p_t = p0 + value * t * axis_or_dir
                    R_t = R0
                    q, res = ik(q_prev, p_t, R_t, True, True)
                else:
                    p_t = p0
                    R_t = Rotation.from_rotvec(axis_or_dir * value * t).as_matrix() @ R0
                    q, res = ik(q_prev, p_t, R_t, True, True)
                ik_worst = max(ik_worst, res)
                if np.any(q < lo_hard) or np.any(q > hi_hard):
                    limit_at = t
                    break
                qs.append(q)
                q_prev = q
            ok_len = len(qs)
            add([q_pose] * int(round(SETTLE_S * HZ)), {"phase": "pre", "seg": seg_name})
            add(qs, {"phase": "ramp", "seg": seg_name})
            end = qs[-1] if qs else q_pose
            d = duration_for(end, q_pose, RETURN_PEAK)
            add(minjerk_path(end, q_pose, d), {"phase": "return", "seg": seg_name})
            add([q_pose] * int(round(POST_S * HZ)), {"phase": "post", "seg": seg_name})
            segs.append({"name": seg_name, "kind": kind, "value": value, "ramp_ticks": ok_len,
                         "ramp_s_done": ok_len * DT, "limit_at_s": limit_at, "ik_worst_residual": ik_worst,
                         "ik_fail": bool(ik_worst > 1e-4), "dir_or_axis": (axis_or_dir.tolist() if kind == "rot" else axis_or_dir.tolist())})

        for dname, dvec in TRANS_DIRS.items():
            for v in TRANS_SPEEDS:
                ramp_segment(f"trans_{dname}_{int(v*1000)}mms", "trans", dvec, v)
        for ax, avec in ROT_AXES.items():
            for w in ROT_RATES:
                ramp_segment(f"rot_{ax}_{str(w).replace('.', 'p')}radps", "rot", avec, w)

        for dname, dvec in TRANS_DIRS.items():
            q_step, res = ik(q_pose, p0 + STEP_M * dvec, R0, True, True)
            add([q_pose] * int(round(0.5 * HZ)), {"phase": "pre", "seg": f"step_{dname}"})
            add([q_step] * int(round(2.0 * HZ)), {"phase": "step", "seg": f"step_{dname}"})
            segs.append({"name": f"step_{dname}", "kind": "step", "value": STEP_M, "ik_worst_residual": res,
                         "ik_fail": bool(res > 1e-4), "dir_or_axis": dvec.tolist()})
            d = duration_for(q_step, q_pose, RETURN_PEAK)
            add(minjerk_path(q_step, q_pose, d), {"phase": "return", "seg": f"step_{dname}"})
            add([q_pose] * int(round(POST_S * HZ)), {"phase": "post", "seg": f"step_{dname}"})
        state["sched"] = np.array(Q)
        state["labels"] = L
        state["segments"] = segs
        state["q_pose"] = q_pose
        state["p0"] = p0
        state["R0"] = R0
        state["built"] = True

    def step(obs):
        q_now = np.array(obs["joint_angles"][arm["qadr"]], float)
        if not state["built"]:
            build(q_now)
        i = state["i"]
        if i >= len(state["sched"]):
            return None
        q = state["sched"][i]
        state["q"] = q
        state["i"] = i + 1
        return {"mode": "arms", "arm_targets": {"left": [float(x) for x in q]}}

    rows = []

    def hook(obs, loop):
        if not state["built"]:
            return
        i = state["i"] - 1
        if i < 0:
            return
        layer = sag_layer.SAG_LAYERS["left"]
        pad, Rp = layer.arm.pad_pose(data)
        q = state["q"]
        ref, Rr = pose_fn(q)
        gap = ref - pad
        rgap = Rotation.from_matrix(Rr @ Rp.T).as_rotvec()
        lab = state["labels"][i]
        rows.append((i, float(obs["time"]), lab.get("phase"), lab.get("seg", ""), pad.copy(), ref.copy(), gap.copy(), rgap.copy()))

    cfg = {"source_hz": 1.0 / model.opt.timestep / spf, "spawn_pos": SPAWN, "cameras": False, "routes": ROUTES,
           "bench_frames": 10 ** 6, "on_tick": hook, "command_bridge": {"enabled": False}}

    class Stop(Exception):
        pass

    def stopper(obs, loop):
        # the source sends nothing after the schedule; stop five ticks after its last target
        if state["built"] and state["i"] >= len(state["sched"]):
            state["after"] = state.get("after", 0) + 1
            if state["after"] >= 5:
                loop["stop"]("schedule done")

    def hook_all(obs, loop):
        hook(obs, loop)
        stopper(obs, loop)

    cfg["on_tick"] = hook_all
    src = {"name": "measure", "step": step}
    res = run_loop(model, data, {"measure": src}, cfg, viewer=None)

    # ----- tick table
    with open(out / f"{a.pose}_ticks.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["i", "t", "phase", "seg", "pad_x", "pad_y", "pad_z", "ref_x", "ref_y", "ref_z", "gap_x", "gap_y", "gap_z", "rgap_x", "rgap_y", "rgap_z"])
        for i, t, ph, seg, pad, ref, gap, rgap in rows:
            w.writerow([i, f"{t:.6f}", ph, seg] + [f"{x:.9f}" for x in pad] + [f"{x:.9f}" for x in ref] + [f"{x:.9f}" for x in gap] + [f"{x:.9f}" for x in rgap])
    (out / f"{a.pose}_segments.json").write_text(json.dumps({"pose": a.pose, "q_pose": state["q_pose"].tolist(), "stopped_by": res.get("stopped_by"),
                                                           "bridge_log_events": len(res.get("bridge_log") or []), "segments": state["segments"]}, indent=2))
    print(f"lag_measure {a.pose}: {len(rows)} ticks; schedule {len(state['sched'])}; stopped_by {res.get('stopped_by')}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
