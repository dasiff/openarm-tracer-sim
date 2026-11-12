#!/usr/bin/env python
"""Stage 2 comparison: our safety-filter resolver against Kong's compiled SEW filter on a scripted two-arm rolling punch.

Four configurations run on the same raw command stream:
  1  no filter: output = raw
  2  Kong's compiled filter (SafetyFilter, collision_avoidance=True), streamed: first tick reset from HOME, then reset=False
  3  our resolver in "paper_plain" mode (gap rule off), arm-versus-arm pairs only, Kong's capsule radii, every cfg.MARGINS entry 0.0
  4  our resolver with the full gap rule ("ours"), our own shapes and margins (default), for information

The stream: each hand in turn moves in a straight line (min-jerk timing) from its HOME hand position to a point 5 cm past
the body midline (left hand y = -0.05 m, right hand y = +0.05 m, same x and z) in 0.5 s, and back in 0.5 s. The other hand
starts 0.5 s later. Period 1.0 s, REPEATS punches per hand, 150 ticks per second. The hand keeps its HOME orientation.
Raw commands are joint targets from our model's Jacobian: repeated single Jacobian steps per tick (up to IK_STEPS, from the
previous tick's joints).

Known problem (see the report): the straight line at constant x and z needs joint 2 beyond its upper limit at the midline.
  --dx, --dz shift the whole path (grid search with --scan); --clip final (default) clips the raw joints to the limits as the
  last resort. Both are reported in the output; nothing is hidden.

Metrics per configuration (all ticks): smallest arm-versus-arm gap with Kong's capsules and with our shapes, hold-still ticks,
hand position and orientation error of the output against the raw command, largest joint jump per tick, filter-call time,
status and hold-reason counts.

Run from the repository root, venv active. A full run takes more than a couple of minutes: run it in a detached tmux session:
  bash experiments/run_capped.sh data/logs/<timestamp>_kong_comparison.log python scripts/safety_filter_kong_comparison.py
Options: --ticks N (dry run: first N ticks, for timing), --dx M, --dz M, --clip final|none, --configs 1,2,3,4, --scan,
--repeats N, --out PATH. Output: JSON written to a temporary file and renamed (default data/safety_filter/kong_comparison_<stamp>.json).
"""

import argparse
import datetime as dt
import json
import os
import subprocess
import sys
import time
import traceback
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

import safety_filter_resolver_fixtures as fx  # noqa: E402  (HOME, DT, model_and_shapes, new_resolver, solver)
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.arm_conversion import ArmKinematics, rotation_error_rad  # noqa: E402
from src.safety_filter.geometry import capsule_capsule, capsule_capsule_batch  # noqa: E402
from src.safety_filter.resolver import Resolver  # noqa: E402
from src.safety_filter.shapes import PlacedShape, RobotShapes, RobotState  # noqa: E402

SIDES = ("left", "right")
FS = 150.0                                   # Hz: the control loop's tick (fixtures DT = 1/150)
REPEATS = 10                                 # count: punches per hand
PERIOD_S = 1.0                               # s: one out-and-back per hand
HALF_S = 0.5                                 # s: the out move, and the back move, of one punch
PUNCH_Y = {"left": -0.05, "right": 0.05}     # m: lateral end point of the punch, 5 cm past the body midline
IK_STEPS = 30                                # count: most Jacobian steps per arm per tick (ends earlier when a step moves no joint by IK_TOL)
IK_TOL = 1e-9                                # rad: a step that moves every joint of the arm less than this ends the tick's IK
KONG_RADII = {"upper": 0.05, "lower": 0.05, "hand": 0.08}   # m: Kong's capsule radii (docs/reports/safety_filter_kong_code_20261109.md section 5)
KONG_SEGMENT = {"upper": "upper_arm", "lower": "forearm", "hand": "hand"}   # the resolver's segment names (src/safety_filter/shapes.py)
KONG_BODY = {"upper": "openarm_{side}_link2", "lower": "openarm_{side}_link4", "hand": "openarm_{side}_link7"}   # MuJoCo body frame of each capsule
SCAN_DX = (-0.15, -0.10, -0.05, 0.0, 0.05, 0.10, 0.15, 0.20)   # m: forward offsets tried by --scan
SCAN_DZ = (-0.10, -0.05, 0.0, 0.05, 0.10)                        # m: height offsets tried by --scan


# ---------------------------------------------------------------- the stream

def min_jerk(tau):
    return 10.0 * tau ** 3 - 15.0 * tau ** 4 + 6.0 * tau ** 5


def excursion(t, start, repeats):
    """Fraction (0 to 1) of the way from the start point to the punch point at time t (s), for a hand whose first cycle begins at `start` (s)."""
    if t < start:
        return 0.0
    k = int(np.floor((t - start) / PERIOD_S + 1e-9))
    if k >= repeats:
        return 0.0
    u = max(t - start - k * PERIOD_S, 0.0)
    if u <= HALF_S:
        return float(min_jerk(u / HALF_S))
    return float(1.0 - min_jerk((u - HALF_S) / HALF_S))


def state_of(q):
    """RobotState with both arms' seven joints taken from q (2, 7); base and fingers at their defaults."""
    return RobotState(q_left=tuple(float(v) for v in q[0]), q_right=tuple(float(v) for v in q[1]))


def ik_tick(kin, cur, i, side, target, rot_target, steps):
    """Moves arm i of cur (2, 7) toward target (m) with repeated single Jacobian steps. Returns the number of steps used."""
    used = 0
    for _ in range(steps):
        new = kin.jacobian_step(state_of(cur), side, target, rot_target)
        used += 1
        moved = float(np.max(np.abs(new - cur[i])))
        cur[i] = new
        if moved < IK_TOL:
            break
    return used


def n_ticks(repeats):
    """Ticks from t = 0 to the end of the last return (repeats + 0.5 s), both ends included."""
    return int(round((repeats + HALF_S) * FS)) + 1


def build_stream(kin, dx=0.0, dz=0.0, clip="final", repeats=REPEATS):
    """The raw joint commands of the rolling punch, built once and shared by all configurations.

    dx, dz (m): shift of the whole path (start, punch point and turn-round point) in x (forward) and z (up).
    clip: "final" clips the raw joints to the joint limits (the IK trajectory is computed unclipped, then clipped);
          "none" keeps the unclipped trajectory.
    Returns a dict with raw (N, 2, 7), q_ik (N, 2, 7) unclipped, target (N, 2, 3) intended hand points, rot (2, 3, 3) held
    orientations, q0 (2, 7) the start pose, start (dict) start points, and stats.
    """
    if clip not in ("final", "none"):
        raise ValueError(f"clip must be 'final' or 'none', not {clip!r}")
    lo = {s: kin.limits[s][:, 0] for s in SIDES}
    hi = {s: kin.limits[s][:, 1] for s in SIDES}
    home = state_of(fx.HOME)
    start, end, rot = {}, {}, {}
    for side in SIDES:
        p, R = kin.hand_pose(home, side)
        start[side] = p + np.array([dx, 0.0, dz])
        end[side] = np.array([start[side][0], PUNCH_Y[side], start[side][2]])   # same x and z as the start
        rot[side] = R                                                             # the HOME orientation, held
    q0 = np.array(fx.HOME, dtype=float)                                           # the start pose (HOME when dx = dz = 0)
    for i, side in enumerate(SIDES):
        ik_tick(kin, q0, i, side, start[side], rot[side], steps=200)
    n = n_ticks(repeats)
    offset = {"left": 0.0, "right": HALF_S}
    cur = q0.copy()
    q_ik = np.zeros((n, 2, 7))
    target = np.zeros((n, 2, 3))
    used = np.zeros((n, 2), dtype=int)
    for k in range(n):
        t = k / FS
        for i, side in enumerate(SIDES):
            s = excursion(t, offset[side], repeats)
            target[k, i] = start[side] + s * (end[side] - start[side])
            used[k, i] = ik_tick(kin, cur, i, side, target[k, i], rot[side], IK_STEPS)
        q_ik[k] = cur
    raw = q_ik.copy()
    if clip == "final":
        for i, side in enumerate(SIDES):
            raw[:, i, :] = np.clip(q_ik[:, i, :], lo[side], hi[side])
    stats = stream_stats(kin, q_ik, raw, target, rot, q0, used, repeats)
    return {"raw": raw, "q_ik": q_ik, "target": target, "rot": rot, "q0": q0, "start": start, "stats": stats}


def stream_stats(kin, q_ik, raw, target, rot, q0, used, repeats):
    """Joint-limit and path statistics of the stream (before and after clipping)."""
    out = {"ticks": len(raw), "repeats": repeats,
           "ik_steps_mean": round(float(used.mean()), 3), "ik_steps_max": int(used.max()),
           "ik_ticks_at_step_cap": int((used >= IK_STEPS).sum()),
           "start_minus_home_max_rad": round(float(np.abs(q0 - np.array(fx.HOME)).max()), 4)}
    for name, arr in (("before_clip", q_ik), ("raw", raw)):
        bad = np.zeros((len(arr), 2), dtype=bool)
        worst = np.zeros((2, 7))
        pos, rotd = [], []
        for i, side in enumerate(SIDES):
            lo, hi = kin.limits[side][:, 0], kin.limits[side][:, 1]
            over = np.maximum(np.maximum(lo - arr[:, i, :], arr[:, i, :] - hi), 0.0)
            bad[:, i] = (over > 1e-9).any(axis=1)
            worst[i] = over.max(axis=0)
        for k in range(len(arr)):
            for i, side in enumerate(SIDES):
                p, R = kin.hand_pose(state_of(arr[k]), side)
                pos.append(float(np.linalg.norm(p - target[k, i])))
                rotd.append(rotation_error_rad(rot[side], R))
        pos = np.array(pos) * 1000.0
        out[name] = {"tick_arms_outside_limits": int(bad.sum()),
                     "max_outside_limits_rad": {"left": np.round(worst[0], 4).tolist(), "right": np.round(worst[1], 4).tolist()},
                     "line_error_pos_mean_mm": round(float(pos.mean()), 3),
                     "line_error_pos_max_mm": round(float(pos.max()), 3),
                     "line_error_rot_max_deg": round(float(np.degrees(max(rotd))), 3)}
    out["joint2_left_range_rad_before_clip"] = [round(float(q_ik[:, 0, 1].min()), 4), round(float(q_ik[:, 0, 1].max()), 4)]
    return out


# ---------------------------------------------------------------- Kong's capsules on our model

class KongShapes(RobotShapes):
    """Our robot model, but the shapes are Kong's six arm-versus-arm capsules (three per arm).

    Per arm: upper (S to E, radius 0.05 m), lower (E to W, 0.05 m), hand (W to T, 0.08 m). S, E and W are our model's points
    (ArmKinematics.sew_now); T = W + R7 @ tool_local, the tool point in the link-7 frame taken from Kong's solver at HOME.
    """

    def __init__(self, model, shapes_path, tool_local):
        super().__init__(model, shapes_path)
        self.tool_local = {s: np.asarray(tool_local[s], dtype=float) for s in SIDES}
        self._kin = ArmKinematics(self)
        self.largest_radius = max(KONG_RADII.values())   # the resolver's broad phase pads by this; Kong's hand capsule is the largest

    def place(self, state):
        super().place(state)           # sets the kinematics; our own shapes are not returned
        out = []
        for side in SIDES:
            S, E, W = self._kin.sew_now(side)
            _, R7 = self._kin.hand_pose_now(side)
            T = W + R7 @ self.tool_local[side]
            ends = {"upper": (S, E), "lower": (E, W), "hand": (W, T)}
            for part in ("upper", "lower", "hand"):
                a, b = ends[part]
                out.append(PlacedShape(id=f"{side}_kong_{part}", arm=side, part=f"kong_{part}", kind="capsule",
                                       body=KONG_BODY[part].format(side=side), a=np.array(a, float), b=np.array(b, float),
                                       r=KONG_RADII[part], geoms=(), segment=KONG_SEGMENT[part]))
        return out


def tool_local_at_home(kin, solver):
    """T - W in the link-7 frame of each arm, from Kong's solver at HOME (solver frame = world + a constant z offset, so the difference is frame-free)."""
    fk = solver.session.robot_fk_at(q_right=fx.HOME[1], q_left=fx.HOME[0])
    kin.set_state(state_of(fx.HOME))
    out = {}
    for side in SIDES:
        _, R7 = kin.hand_pose_now(side)
        d = np.asarray(fk[side]["T"], float) - np.asarray(fk[side]["W"], float)
        out[side] = R7.T @ d
    return out


# ---------------------------------------------------------------- the four configurations

def run_kong(raw, q_ref):
    from src.geo_kin_self_collision_filter import SafetyFilter
    sf = SafetyFilter(collision_avoidance=True)
    out = np.zeros_like(raw)
    times = np.zeros(len(raw))
    prev = np.array(q_ref, dtype=float)
    for k in range(len(raw)):
        t0 = time.perf_counter()
        ql, qr = sf.filter(raw[k, 0].copy(), raw[k, 1].copy(), q_current_left=prev[0].copy(), q_current_right=prev[1].copy(),
                           reset=(k == 0))
        times[k] = time.perf_counter() - t0
        out[k, 0] = ql
        out[k, 1] = qr
        prev = out[k].copy()
    return out, times, None


def run_resolver(res, raw, q_ref):
    out = np.zeros_like(raw)
    times = np.zeros(len(raw))
    statuses, reasons = [], []
    last = np.array(q_ref, dtype=float)
    base = RobotState()
    for k in range(len(raw)):
        t0 = time.perf_counter()
        o, st = res.step(raw[k].copy(), last.copy(), base)
        times[k] = time.perf_counter() - t0
        out[k] = np.asarray(o, dtype=float).reshape(2, 7)
        statuses.append(_label(st.status))
        reasons.append(_label(getattr(st, "hold_reason", None)))
        last = out[k].copy()
    return out, times, {"status": statuses, "hold_reason": reasons}


def run_paper_plain(m, path, tool_local, raw, q_ref):
    """Configuration 3: gap rule off, all margins 0.0, Kong's shapes. Restored in finally."""
    saved_mode = cfg.GAP_RULE["mode"]
    saved_margins = dict(cfg.MARGINS)
    try:
        cfg.GAP_RULE["mode"] = "paper_plain"
        for key in cfg.MARGINS:
            cfg.MARGINS[key] = 0.0
        shapes = KongShapes(m, path, tool_local)
        res = Resolver(shapes, fx.DT, fx.solver())
        return run_resolver(res, raw, q_ref)
    finally:
        cfg.GAP_RULE["mode"] = saved_mode
        cfg.MARGINS.clear()
        cfg.MARGINS.update(saved_margins)


def _label(x):
    if x is None:
        return "none"
    name = getattr(x, "name", None) or str(x)
    return str(name).split(".")[-1]


# ---------------------------------------------------------------- metrics

def gap_metrics(out, kong_metric, ours):
    """Smallest gaps over all ticks. Returns ((gap_kong, tick, pair), (gap_ours, tick, pair), (gap_ours_no_shoulder, tick, pair)).
    Kong: all left x right pairs of Kong's capsules (capsule_capsule). Ours: all left-arm x right-arm capsules of our shapes (batch, checked
    against capsule_capsule on the first tick); touch rules are ignored. The third value leaves out the left-right shoulder-mount pair,
    which sits 14 mm inside at HOME in our shapes (both arms mount on the torso) and so sets the minimum of every run."""
    best_k = (np.inf, -1, "")
    best_o = (np.inf, -1, "")
    best_s = (np.inf, -1, "")
    for k in range(len(out)):
        st = state_of(out[k])
        if kong_metric is not None:
            pl = kong_metric.place(st)
            L = [p for p in pl if p.arm == "left"]
            R = [p for p in pl if p.arm == "right"]
            for a in L:
                for b in R:
                    g = capsule_capsule(a.a, a.b, a.r, b.a, b.b, b.r).gap
                    if g < best_k[0]:
                        best_k = (float(g), k, f"{a.id} / {b.id}")
        pl = ours.place(st)
        L = [p for p in pl if p.arm == "left" and p.kind == "capsule"]
        R = [p for p in pl if p.arm == "right" and p.kind == "capsule"]
        A1 = np.array([p.a for p in L]); B1 = np.array([p.b for p in L]); r1 = np.array([p.r for p in L])
        A2 = np.array([p.a for p in R]); B2 = np.array([p.b for p in R]); r2 = np.array([p.r for p in R])
        ii = np.repeat(np.arange(len(L)), len(R))
        jj = np.tile(np.arange(len(R)), len(L))
        g = capsule_capsule_batch(A1[ii], B1[ii], r1[ii], A2[jj], B2[jj], r2[jj], gap_only=True)
        if k == 0:
            ref = np.array([capsule_capsule(L[i].a, L[i].b, L[i].r, R[j].a, R[j].b, R[j].r).gap for i, j in zip(ii, jj)])
            if not np.allclose(g, ref, atol=1e-12):
                raise RuntimeError("batch capsule gap differs from capsule_capsule")
        j_min = int(np.argmin(g))
        if g[j_min] < best_o[0]:
            best_o = (float(g[j_min]), k, f"{L[ii[j_min]].id} / {R[jj[j_min]].id}")
        keep = np.array([n2 for n2, (i, j) in enumerate(zip(ii, jj))
                         if L[i].part != "shoulder_mount" and R[j].part != "shoulder_mount"], dtype=int)
        if len(keep):
            s_min = int(keep[np.argmin(g[keep])])
            if g[s_min] < best_s[0]:
                best_s = (float(g[s_min]), k, f"{L[ii[s_min]].id} / {R[jj[s_min]].id}")
    return best_k, best_o, best_s


def hand_error(kin, raw, out):
    """Hand position (mean, max, mm) and orientation (max, deg) of the output against the raw command, both hands, all ticks."""
    pos, rot = [], []
    for k in range(len(raw)):
        for side in SIDES:
            p_o, R_o = kin.hand_pose(state_of(out[k]), side)
            p_r, R_r = kin.hand_pose(state_of(raw[k]), side)
            pos.append(float(np.linalg.norm(p_o - p_r)))
            rot.append(rotation_error_rad(R_r, R_o))
    pos = np.array(pos) * 1000.0
    return float(pos.mean()), float(pos.max()), float(np.degrees(max(rot)))


def max_jump(out, q_ref):
    seq = np.concatenate([np.asarray(q_ref, dtype=float)[None], out], axis=0)
    return float(np.abs(np.diff(seq, axis=0)).max())


def summarise(name, required, kin, raw, out, times, extra, q_ref, kong_metric, ours):
    (gk, kk, kpair), (go, ko, opair), (gs, ks, spair) = gap_metrics(out, kong_metric, ours)
    mean_pos, max_pos, max_rot = hand_error(kin, raw, out)
    row = {"name": name, "required_gap": required,
           "min_gap_kong_radii_mm": None if kong_metric is None else round(1000.0 * gk, 4),
           "min_gap_kong_radii_at": {"tick": kk, "pair": kpair},
           "min_gap_our_shapes_mm": round(1000.0 * go, 4),
           "min_gap_our_shapes_at": {"tick": ko, "pair": opair},
           "min_gap_our_shapes_no_shoulder_mount_mm": round(1000.0 * gs, 4),
           "min_gap_our_shapes_no_shoulder_mount_at": {"tick": ks, "pair": spair},
           "hold_ticks": None, "status_counts": None, "hold_reason_counts": None,
           "hand_pos_err_mean_mm": round(mean_pos, 4), "hand_pos_err_max_mm": round(max_pos, 4),
           "hand_rot_err_max_deg": round(max_rot, 4),
           "max_joint_jump_rad": round(max_jump(out, q_ref), 5),
           "time_ms_mean": None, "time_ms_max": None}
    if times is not None:
        row["time_ms_mean"] = round(1000.0 * float(np.mean(times)), 4)
        row["time_ms_max"] = round(1000.0 * float(np.max(times)), 4)
    if extra is not None:
        row["status_counts"] = count(extra["status"])
        row["hold_reason_counts"] = count(extra["hold_reason"])
        row["hold_ticks"] = int(sum(1 for s in extra["status"] if s == "HOLD"))
    return row


def count(seq):
    c = {}
    for s in seq:
        c[s] = c.get(s, 0) + 1
    return c


def guarded(name, fn):
    try:
        return fn()
    except Exception as exc:  # the compiled wheel or a config may fail: record it and run the rest
        print(f"{name} FAILED: {type(exc).__name__}: {exc}", flush=True)
        return {"name": name, "error": f"{type(exc).__name__}: {exc}", "traceback": traceback.format_exc()[-2000:]}


# ---------------------------------------------------------------- driver

def scan(kin, repeats):
    rows = []
    for dz in SCAN_DZ:
        for dx in SCAN_DX:
            t0 = time.time()
            st = build_stream(kin, dx, dz, clip="none", repeats=repeats)["stats"]
            b = st["before_clip"]
            row = {"dx": dx, "dz": dz, "tick_arms_outside_limits": b["tick_arms_outside_limits"],
                   "max_outside_limits_rad_left": b["max_outside_limits_rad"]["left"],
                   "joint2_left_range_rad": st["joint2_left_range_rad_before_clip"], "start_minus_home_max_rad": st["start_minus_home_max_rad"],
                   "ik_ticks_at_step_cap": st["ik_ticks_at_step_cap"], "seconds": round(time.time() - t0, 2)}
            rows.append(row)
            print(f"scan dx={dx:+.2f} dz={dz:+.2f}: tick-arms outside limits {row['tick_arms_outside_limits']}, "
                  f"left max outside (rad) {row['max_outside_limits_rad_left']}, joint 2 left range {row['joint2_left_range_rad']}, "
                  f"start-HOME {row['start_minus_home_max_rad']} rad, {row['seconds']} s", flush=True)
    return rows


def run_comparison(args, kin, m, sh):
    doc = {}
    path = ROOT / cfg.SHAPES_FILE
    stream = build_stream(kin, args.dx, args.dz, args.clip, args.repeats)
    doc["stream_stats"] = stream["stats"]
    print("stream:", json.dumps(stream["stats"]), flush=True)
    n_total = len(stream["raw"])
    n = n_total if args.ticks is None else min(args.ticks, n_total)
    raw = stream["raw"][:n]
    q_ref = stream["q0"]
    wanted = set(args.configs.split(","))

    tool_local, kong_err = None, None
    try:
        tool_local = tool_local_at_home(kin, fx.solver())
    except Exception as exc:
        kong_err = f"{type(exc).__name__}: {exc}"
        print(f"Kong's wheel did not load: {kong_err}", flush=True)
    doc["kong_import"] = "ok" if kong_err is None else kong_err
    doc["tool_local_at_home"] = None if tool_local is None else {s: np.round(tool_local[s], 6).tolist() for s in SIDES}
    kong_metric = KongShapes(m, path, tool_local) if tool_local is not None else None
    ours = sh
    rows = {}

    if "1" in wanted:
        print("config 1: no filter", flush=True)
        rows["1_no_filter"] = summarise("1_no_filter", "kong_radii", kin, raw, raw.copy(), None, None, q_ref, kong_metric, ours)
    if "2" in wanted:
        print("config 2: Kong compiled filter", flush=True)
        def c2():
            out, times, extra = run_kong(raw, q_ref)
            return summarise("2_kong_compiled", "kong_radii", kin, raw, out, times, extra, q_ref, kong_metric, ours)
        rows["2_kong_compiled"] = guarded("config 2", c2) if kong_err is None else {"name": "2_kong_compiled", "error": kong_err}
    if "3" in wanted:
        print("config 3: resolver paper_plain, Kong radii, margins 0", flush=True)
        def c3():
            out, times, extra = run_paper_plain(m, path, tool_local, raw, q_ref)
            return summarise("3_resolver_paper_plain_kong_radii", "kong_radii", kin, raw, out, times, extra, q_ref, kong_metric, ours)
        rows["3_resolver_paper_plain_kong_radii"] = guarded("config 3", c3) if kong_err is None else {"name": "3", "error": kong_err}
    if "4" in wanted:
        print("config 4: resolver full gap rule, our shapes and margins", flush=True)
        doc["config4_gap_rule_mode"] = cfg.GAP_RULE["mode"]
        doc["config4_margins"] = dict(cfg.MARGINS)
        def c4():
            res = fx.new_resolver()
            out, times, extra = run_resolver(res, raw, q_ref)
            return summarise("4_resolver_ours", "our_shapes", kin, raw, out, times, extra, q_ref, kong_metric, ours)
        rows["4_resolver_ours"] = guarded("config 4", c4) if kong_err is None else {"name": "4", "error": kong_err}

    doc["ticks_run"] = n
    doc["ticks_in_stream"] = n_total
    doc["configs"] = rows
    return doc


def git_head():
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip()
    except Exception:
        return "unknown"


def write_json_atomic(path, doc):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=2, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def print_table(rows):
    print("\n| configuration | required gap | min gap Kong radii (mm) | min gap our shapes (mm) | min gap our shapes w/o shoulder mounts (mm) | hold ticks | hand pos mean/max (mm) | hand rot max (deg) | max joint jump (rad) | time mean/max (ms) |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for name, r in rows.items():
        if "error" in r:
            print(f"| {name} | ERROR: {r['error']} |")
            continue
        f = lambda v: "n/a" if v is None else v
        tm = "n/a" if r["time_ms_mean"] is None else f"{r['time_ms_mean']} / {r['time_ms_max']}"
        print(f"| {name} | {r['required_gap']} | {f(r['min_gap_kong_radii_mm'])} | {r['min_gap_our_shapes_mm']} | "
              f"{r['min_gap_our_shapes_no_shoulder_mount_mm']} | {f(r['hold_ticks'])} | "
              f"{r['hand_pos_err_mean_mm']} / {r['hand_pos_err_max_mm']} | {r['hand_rot_err_max_deg']} | {r['max_joint_jump_rad']} | {tm} |", flush=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Kong's compiled filter vs our resolver on a scripted two-arm rolling punch.")
    ap.add_argument("--ticks", type=int, default=None, help="run only the first N ticks of the stream (dry run for timing)")
    ap.add_argument("--repeats", type=int, default=REPEATS, help="punches per hand (default 10)")
    ap.add_argument("--dx", type=float, default=0.0, help="forward shift of the whole path, m (default 0)")
    ap.add_argument("--dz", type=float, default=0.0, help="height shift of the whole path, m (default 0)")
    ap.add_argument("--clip", choices=("final", "none"), default="final", help="clip the raw joints to the limits (default final)")
    ap.add_argument("--configs", default="1,2,3,4", help="comma list of configurations to run (default 1,2,3,4)")
    ap.add_argument("--scan", action="store_true", help="only scan the offset grid (stream statistics, no filters)")
    ap.add_argument("--out", default=None, help="JSON output path (default data/safety_filter/kong_comparison_<stamp>.json)")
    args = ap.parse_args(argv)

    if abs(fx.DT - 1.0 / FS) > 1e-12:
        raise RuntimeError("fixtures DT does not match the 150 Hz tick")
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = Path(args.out) if args.out else ROOT / "data" / "safety_filter" / f"kong_comparison_{stamp}.json"
    t0 = time.time()
    m, sh = fx.model_and_shapes()
    kin = ArmKinematics(sh)
    doc = {"stamp": stamp, "git_head": git_head(), "args": vars(args),
           "params": {"fs_hz": FS, "repeats": args.repeats, "period_s": PERIOD_S, "half_s": HALF_S, "punch_y_m": PUNCH_Y,
                      "ik_steps": IK_STEPS, "ik_tol_rad": IK_TOL, "kong_radii_m": KONG_RADII, "kong_segment": KONG_SEGMENT,
                      "kong_body": KONG_BODY, "shapes_file": cfg.SHAPES_FILE, "margins_default": dict(cfg.MARGINS),
                      "gap_rule_mode_default": cfg.GAP_RULE["mode"]}}
    if args.scan:
        doc["offset_scan"] = scan(kin, args.repeats)
    else:
        doc.update(run_comparison(args, kin, m, sh))
        print_table(doc["configs"])
    doc["runtime_s"] = round(time.time() - t0, 1)
    write_json_atomic(out_path, doc)
    print(f"wrote {out_path} (runtime {doc['runtime_s']} s)", flush=True)
    return doc


if __name__ == "__main__":
    main()
