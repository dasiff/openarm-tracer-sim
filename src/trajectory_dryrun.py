"""Trajectory dry-run (decisions-log M302): the planner's pick and place moves run through the IK without physics, on the commanded pad.

The solver's own output is its feedback: `MinkArm.step` reads the arm joints from its own command `q_cmd` and never steps the physics (src/mink_arm.py). The dry run therefore
mirrors the skill's move sequence (src/mpvi_modules.py plan_pick and plan_place, move and step_inner) with these differences, all in the measurement and the stubs:
  - the start point and the settle test use the commanded pad (FK of q_cmd), not the measured pad;
  - before each tick the arm's joints are set to q_cmd (the "measured" state tracks the command) and the kinematics are updated (no mj_step);
  - CLOSE and OPEN take a fixed time with no physics (the gripper is not simulated); the held beaker keeps the offset the grasp gave the pad;
  - the blend handover (blend_radius_m) is not mirrored: every move settles before the next one starts, which is the stricter test.

Pass: every subtask reaches its settle condition (within tol_m and tol_rad, settled) before its timeout. Fail: the first subtask that times out, or the IK solver
diverges (a non-finite command, or no solution). There is no per-tick positional tolerance (changed 2026-11-10: the 30 mm reference check is removed).

    res = dry_run_stance(chk, model, stance, beaker_xy, pick_k, place_theta_deg)
    res["ok"], res["failed"], res["reason"], res["subtasks"], res["seconds"]
"""

import math
import time

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from src.approach_angle import place_approach_xy
from src.mink_arm import MinkArm, pad_rotation
from src.mpvi_modules import move_duration
from src.rate_limit import min_jerk
from src.task_beaker_hotplate import bench_top_z
from src.task_config import BEAKER_CFG, DRYRUN_CFG, MINK_CFG, PICK_PLACE_CFG, STANCE_SEARCH_LOG_CFG, TASK_CFG
from src.task_points import pick_waypoints, plate_top_z


SIDE = "left"
TICK_S = 1.0 / 150.0            # the control tick (src/control_loop: 150 Hz)


_SCRATCH = {}                   # one scratch state per model: a new MjData per call filled the memory cap (the run was killed, 2026-11-10)


def _commanded_pad(model, d, arm, q):
    """The pad centre and rotation with the arm at command q (FK on a scratch copy; the state d is not changed)."""
    d2 = _SCRATCH.get(id(model))
    if d2 is None:
        d2 = _SCRATCH[id(model)] = mujoco.MjData(model)
    d2.qpos[:] = d.qpos
    d2.qpos[arm.qadr] = q
    mujoco.mj_kinematics(model, d2)
    return arm.pad_pose(d2)


def _track(model, d, arm):
    """The measured state tracks the command: the arm's joints are set to q_cmd and the kinematics updated."""
    d.qpos[arm.qadr] = arm.q_cmd
    mujoco.mj_kinematics(model, d)


def _trace_row(t, arm, pad):
    """One tick of the trace (logging, M315): time since the subtask started, the commanded joints (rad) and the pad centre (m), rounded."""
    r = STANCE_SEARCH_LOG_CFG["round_decimals"]
    return [round(float(t), r)] + [round(float(v), r) for v in arm.q_cmd] + [round(float(v), r) for v in pad]


def _hold(model, d, arm, name, seconds, trace=False):
    """A stub subtask (CLOSE, OPEN): the arm holds its command for the given time; the gripper is not simulated."""
    n = int(round(seconds / TICK_S))
    rows = [] if trace else None
    for k in range(1, n + 1):
        _track(model, d, arm)
        if rows is not None:
            rows.append(_trace_row(k * TICK_S, arm, arm.pad_pose(d)[0]))
    return {"name": name, "T_s": float(seconds), "ticks": n, "ok": True, "max_ref_mm": 0.0, "final_mm": 0.0, "tick_log": [], "trace": rows}, True, None


def _move(model, d, arm, name, target, R, speed, cfg):
    """One waypoint move as the skill runs it (min-jerk reference, one IK step per tick, the settle test), on the commanded pad.

    Returns (record, ok, failure). The command is left at the end of the move (the arm is where the move ended).
    """
    p0, R0 = _commanded_pad(model, d, arm, arm.q_cmd)
    target = np.asarray(target, float)
    T, _ = move_duration(arm, d, p0, R0, target, R, speed, cfg, name)
    slerp = Slerp([0.0, 1.0], Rotation.from_matrix([R0, R]))
    timeout = cfg["timeout_factor"] * T + cfg["timeout_extra_s"]
    settle_t, log, max_ref, ticks = None, [], 0.0, 0
    rows = [] if cfg.get("_record_trace") else None
    q_prev = arm.q_cmd.copy()
    t = 0.0
    while True:
        t += TICK_S
        ticks += 1
        u = min(t / T, 1.0)
        s = min_jerk(u)
        p_ref = p0 + (target - p0) * s
        R_ref = slerp(s).as_matrix()
        _track(model, d, arm)
        try:
            arm.step(d, p_ref, R_ref)
        except Exception as e:                            # noqa: BLE001 - mink's NoSolutionFound and any solver failure end the subtask with its message
            why = f"{name}: no IK solution at {t:.2f} s ({type(e).__name__})"
            return {"name": name, "T_s": T, "ticks": ticks, "ok": False, "fail": why, "max_ref_mm": 1000 * max_ref, "final_mm": None, "tick_log": log, "trace": rows}, False, why
        _track(model, d, arm)
        p, Rp = arm.pad_pose(d)
        ref_err = float(np.linalg.norm(p - p_ref))
        max_ref = max(max_ref, ref_err)
        err = float(np.linalg.norm(p - target))
        ang = float(np.linalg.norm(Rotation.from_matrix(Rp.T @ R).as_rotvec()))
        speed_now = float(np.max(np.abs(arm.q_cmd - q_prev))) / TICK_S
        q_prev = arm.q_cmd.copy()
        if cfg.get("_record_ticks"):
            log.append([round(t, 4), round(1000 * err, 2), round(1000 * ref_err, 2)])
        if not np.all(np.isfinite(arm.q_cmd)):
            why = f"{name}: the IK diverged at {t:.2f} s (a non-finite command)"
            return {"name": name, "T_s": T, "ticks": ticks, "ok": False, "fail": why, "max_ref_mm": 1000 * max_ref, "final_mm": None, "tick_log": log, "trace": rows}, False, why
        if rows is not None:
            rows.append(_trace_row(t, arm, p))
        if u >= 1.0 and err < cfg["tol_m"] and ang < cfg["tol_rad"] and speed_now < cfg["settle_speed_rad_s"]:
            if settle_t is None:
                settle_t = t
            if t - settle_t >= cfg["settle_dwell_s"]:
                return {"name": name, "T_s": T, "ticks": ticks, "ok": True, "max_ref_mm": 1000 * max_ref, "final_mm": 1000 * err, "tick_log": log, "trace": rows}, True, None
        else:
            settle_t = None
        if t > timeout:
            why = f"{name}: not reached after {t:.1f} s ({1000 * err:.1f} mm from the waypoint)"
            return {"name": name, "T_s": T, "ticks": ticks, "ok": False, "fail": why, "max_ref_mm": 1000 * max_ref, "final_mm": 1000 * err, "tick_log": log, "trace": rows}, False, why


def _home(model, d, arm, name, q_home, cfg):
    """The move to the zombie pose's joints (the skill's HOME): min-jerk in joint space, the command set directly (no IK, as the skill's joint move)."""
    q0 = arm.q_cmd.copy()
    T = max(cfg["min_move_s"], float(np.max(np.abs(q_home - q0))) / 0.3)
    ticks = int(round(T / TICK_S))
    rows = [] if cfg.get("_record_trace") else None
    for k in range(1, ticks + 1):
        arm.q_cmd = q0 + (q_home - q0) * min_jerk(k / ticks)
        _track(model, d, arm)
        if rows is not None:
            rows.append(_trace_row(k * TICK_S, arm, arm.pad_pose(d)[0]))
    err = float(np.max(np.abs(arm.q_cmd - q_home)))
    ok = err < DRYRUN_CFG["home_tol_rad"]
    why = None if ok else f"{name}: {math.degrees(err):.1f} deg from the zombie pose"
    return {"name": name, "T_s": T, "ticks": ticks, "ok": ok, "max_ref_mm": 0.0, "final_mm": math.degrees(err), "tick_log": [], "trace": rows}, ok, why


def dry_run_stance(chk, model, stance, beaker_xy, pick_k, place_theta_deg, cfg=PICK_PLACE_CFG, mink_cfg=MINK_CFG, home_q=None, record=False, trace=False):
    """The dry run of one stance: pick (OVER, DOWN, IN, CLOSE, UP) then place (OVER, DOWN, OPEN, BACK, HOME), on the commanded pad.

    Args:
        chk (StanceChecks): The check's own state; place() sets its data to the stance (the arms at home).
        model (mujoco.MjModel): The task model.
        stance (tuple): (x, y, heading, pedestal command).
        beaker_xy (sequence of float): The beaker's xy (its bottom centre sits on the bench top).
        pick_k (int): The pick's approach bin (choose_approach_angle of the full check's list).
        place_theta_deg (float): The place's hand turn (choose_place_angle of the full check's list), degrees.
        cfg (dict): PICK_PLACE_CFG.
        mink_cfg (dict): MINK_CFG.
        home_q (sequence of 7 float or None): The zombie pose's joints. Defaults to TASK_CFG home_q_left.
        record (bool): Keep each tick of each move in tick_log: [t, error to the target mm, error to the reference mm].
        trace (bool): Keep each tick of each subtask as a row [t_s, the 7 commanded joints (rad), the pad centre x, y, z (m)], rounded (logging, M315).

    Returns:
        dict: "ok", "failed" (the subtask name or None), "reason", "subtasks" (one dict per subtask: name, T_s, ticks, ok, max_ref_mm, final_mm, fail), "seconds",
        "trace" (with trace=True: a list of {"name", "rows"} per subtask, else None; kept out of "subtasks" so the records that carry them do not grow).
    """
    t0 = time.perf_counter()
    home = np.asarray(TASK_CFG["home_q_left"] if home_q is None else home_q, float)
    chk.place(stance)
    d = chk.d
    mujoco.mj_kinematics(model, d)
    arm = MinkArm(model, d, SIDE, home, mink_cfg)
    arm.q_cmd = home.copy()
    _track(model, d, arm)
    cfg = dict(cfg, _record_ticks=record, _record_trace=trace)
    traces = []
    bx, by = float(beaker_xy[0]), float(beaker_xy[1])
    bench_top = float(bench_top_z(model))
    height = float(BEAKER_CFG["height"])
    hg = bench_top + cfg["grasp_height_frac"] * height
    h = float(stance[2])
    W = pick_waypoints((bx, by), hg, height, bench_top, float(plate_top_z(model, d)), h, pick_k, cfg)
    subtasks = []

    def finish(ok, failed, reason):
        return {"ok": bool(ok), "failed": failed, "reason": reason, "subtasks": subtasks, "seconds": time.perf_counter() - t0, "trace": traces if trace else None}

    def record_and_check(res):
        rec, ok, why = res
        rows = rec.pop("trace", None)
        if rows is not None:
            traces.append({"name": rec["name"], "rows": rows})
        subtasks.append({k: v for k, v in rec.items() if k != "tick_log" or record})
        return ok, why

    p0, R0 = _commanded_pad(model, d, arm, arm.q_cmd)
    R = pad_rotation(np.array([W["approach"][0], W["approach"][1], 0.0]), R0[:, 0])
    pick_seq = [("pick OVER", W["over"], cfg["speed_m_s"]), ("pick DOWN", W["standoff"], cfg["slow_speed_m_s"]), ("pick IN", (bx, by, hg), cfg["slow_speed_m_s"])]
    for name, target, speed in pick_seq:
        ok, why = record_and_check(_move(model, d, arm, name, target, R, speed, cfg))
        if not ok:
            return finish(False, name, why)
    ok, why = record_and_check(_hold(model, d, arm, "pick CLOSE", cfg["close_min_s"], trace=trace))
    ok, why = record_and_check(_move(model, d, arm, "pick UP", W["lift"], R, cfg["slow_speed_m_s"], cfg))
    if not ok:
        return finish(False, "pick UP", why)
    # the held beaker keeps the grasp's offset: the pad relative to the beaker's bottom centre at the pick
    pad_now, R_now = _commanded_pad(model, d, arm, arm.q_cmd)
    off = pad_now - np.array([bx, by, bench_top])
    hp = np.array(d.site("hotplate_top").xpos, float)
    th = math.radians(float(place_theta_deg))
    co, si = math.cos(th), math.sin(th)
    Rz = np.array([[co, -si, 0.0], [si, co, 0.0], [0.0, 0.0, 1.0]])
    fwd = np.array([math.cos(h), math.sin(h), 0.0])
    R_pl = Rz @ pad_rotation(fwd, R_now[:, 0])
    off_p = Rz @ off
    ap = place_approach_xy(fwd, th)
    over = np.array([hp[0] + off_p[0], hp[1] + off_p[1], pad_now[2]])
    down = np.array([hp[0] + off_p[0], hp[1] + off_p[1], hp[2] + off_p[2] + cfg["setdown_gap_m"]])
    back = down.copy()
    back[:2] -= ap[:2] * cfg["retreat_m"]
    place_seq = [("place OVER", over, cfg["speed_m_s"]), ("place DOWN", down, cfg["slow_speed_m_s"])]
    for name, target, speed in place_seq:
        ok, why = record_and_check(_move(model, d, arm, name, target, R_pl, speed, cfg))
        if not ok:
            return finish(False, name, why)
    ok, why = record_and_check(_hold(model, d, arm, "place OPEN", DRYRUN_CFG["open_s"], trace=trace))
    ok, why = record_and_check(_move(model, d, arm, "place BACK", back, R_pl, cfg["slow_speed_m_s"], cfg))
    if not ok:
        return finish(False, "place BACK", why)
    ok, why = record_and_check(_home(model, d, arm, "place HOME", home, cfg))
    return finish(ok, None if ok else "place HOME", why)
