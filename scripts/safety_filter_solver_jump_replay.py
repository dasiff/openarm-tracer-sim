"""Stage 2d step 3: replay the real solver-output jumps of Stage 2c (measurement only; nothing in src/ is changed).

The first N (default 20) BRANCH_GUARD holds whose jump was first seen at the solver output are taken from the Stage 2c traces
(data/safety_filter/hold_causes_row{4,6}.json: tick, arm and joint) for row 6 (kong_style, ours) and row 4 (layered, ours). The
resolver is run from tick 0 on the recorded raw stream exactly as in Stage 2c (its state, the active pair set, is carried by the
real run), and at each target tick res.kin.convert_directions is wrapped by a recorder (the wrapper only records) so the solver
inputs and output of the real call are captured. After the tick, three more solver calls are made on the same solver with the
same seed state: (U) the unpushed keypoints of the raw command, (C) the keypoints of the last filtered command (a round trip),
(C') the same seeded with the last filtered command. Nothing is fed back into the resolver.

Cause rules (first match; the numbers are in the table so the reader can disagree):
  (a) the round trip (C) already fails and the hand pose of its output is NOT the last pose -> solver and our arm disagree;
  (c) joint 6 within 0.15 rad of zero before or after (the wrist axes 5 and 7 line up);
  (d) an output joint sits at or past a joint limit;
  (b) another valid solution: joint 6 changes sign and joints 5 and 7 change by about pi (each within 0.3 rad of pi);
  (b2) the round trip (C) itself returns another solution with the same hand pose (other solution without any push);
  (e) the unpushed call does not jump but the pushed one does and no flip: sensitivity (ratio of joint change to push angle is given);
  other: anything else, one line.
The order is (a), (b2 / c), (d), (b), (e), other with the near-zero test (c) ahead of (b) and (d).

Usage: python scripts/safety_filter_solver_jump_replay.py [--n 20] [--rows 6 4] [--out data/safety_filter/solver_jump_replay.json]
"""

import argparse
import json
import os
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
import safety_filter_diag_common as dc  # noqa: E402
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.arm_conversion import rotation_error_rad, with_arms  # noqa: E402
from src.safety_filter.shapes import RobotState  # noqa: E402

kc = dc.kc
SIDES = dc.SIDES
IDX = {"left": 0, "right": 1}
JUMP_RAD = cfg.BRANCH_GUARD_RAD
NEAR_ZERO_J6 = 0.15      # rad: joint 6 closer to zero than this counts as near the wrist singularity (my choice, listed in the report)
FLIP_TOL = 0.3           # rad: "about pi" for joints 5 and 7
POS_TOL_M, ROT_TOL_DEG = 0.002, 0.5   # a hand pose "matches" within 2 mm and 0.5 deg (as in step 2)
LIMIT_TOL = 1e-6
SNAP_J4 = 0.242       # rad: the elbow bend below which the Oct 5 note says the solver snaps (value quoted by David; not found in this repo, tested here)
JOINT_JUMP = 0.2     # rad: joints 3 and 5 both count as jumping above this (my choice)
SUM_TOL = 0.1        # rad: q3+q5 "almost unchanged" when its change is below this (my choice)


class Recorder:
    """Wraps res.kin.convert_directions for one tick; the wrapper only records."""

    def __init__(self, kin):
        self.kin, self.calls, self.orig = kin, [], kin.convert_directions

    def __call__(self, state, side, u, l, rot):
        try:
            out = self.orig(state, side, u, l, rot)
        except RuntimeError as exc:
            self.calls.append({"state": state, "side": side, "u": np.array(u), "l": np.array(l), "rot": np.array(rot), "out": None, "err": str(exc)})
            raise
        self.calls.append({"state": state, "side": side, "u": np.array(u, float), "l": np.array(l, float), "rot": np.array(rot, float),
                           "out": np.array(out, float), "err": None})
        return out

    def install(self):
        self.kin.convert_directions = self

    def uninstall(self):
        if "convert_directions" in self.kin.__dict__:
            del self.kin.convert_directions


def wrap_pi(x):
    return (np.asarray(x) + np.pi) % (2 * np.pi) - np.pi


def pose_err(kin, state, side, q_side, target_pos, target_rot):
    q = np.array([state.q_left, state.q_right], float)
    q[IDX[side]] = q_side
    pos, rot = kin.hand_pose(with_arms(state, q), side)
    return float(np.linalg.norm(pos - target_pos) * 1000.0), float(np.degrees(rotation_error_rad(target_rot, rot)))


def describe(kin, orig, state, side, last_side, u, l, rot, tpos, trot, last_hand_pose=None):
    """Solver output for (u, l, rot) from the given state, with the numbers of the table."""
    out = np.array(orig(state, side, u, l, rot), float)
    return summarise(kin, state, side, out, last_side, tpos, trot)


def summarise(kin, state, side, out, last_side, tpos, trot):
    d = out - last_side
    lim = kin.limits[side]
    at_limit = [int(i + 1) for i in range(7) if out[i] <= lim[i, 0] + LIMIT_TOL or out[i] >= lim[i, 1] - LIMIT_TOL]
    pe, re_ = pose_err(kin, state, side, out, tpos, trot)
    return {"out": out.tolist(), "dq": d.tolist(), "max_dq": float(np.abs(d).max()), "max_dq_joint": int(np.abs(d).argmax() + 1),
            "j6_before": float(last_side[5]), "j6_after": float(out[5]),
            "j5_dq": float(d[4]), "j7_dq": float(d[6]), "at_limit_joints": at_limit,
            "hand_pos_err_mm": pe, "hand_rot_err_deg": re_}


def is_flip(s):
    """joint 6 sign flipped and joints 5 and 7 changed by about pi (wrapped distance to pi under FLIP_TOL)."""
    sign = s["j6_before"] * s["j6_after"] < 0
    near_pi = lambda x: abs(abs(float(wrap_pi(x))) - np.pi) < FLIP_TOL
    return bool(sign and near_pi(s["j5_dq"]) and near_pi(s["j7_dq"]))


def classify(P, U, C):
    """(cause label, one-line description) from the pushed (P), unpushed (U) and round-trip (C) summaries."""
    c_valid = C["hand_pos_err_mm"] <= POS_TOL_M * 1000 and C["hand_rot_err_deg"] <= ROT_TOL_DEG
    if C["max_dq"] >= JUMP_RAD:
        if not c_valid:
            return "(a)", "round trip from the last filtered keypoints returns a joint set %.3f rad away AND a different hand pose (%.1f mm)" % (C["max_dq"], C["hand_pos_err_mm"])
        if min(abs(C["j6_before"]), abs(C["j6_after"])) < NEAR_ZERO_J6:
            return "(c)", "round trip alone jumps %.3f rad with the same hand pose; joint 6 near zero (%.3f -> %.3f)" % (C["max_dq"], C["j6_before"], C["j6_after"])
        return "(b2)", "round trip alone returns another valid solution (max change %.3f rad, joint %d)" % (C["max_dq"], C["max_dq_joint"])
    if min(abs(P["j6_before"]), abs(P["j6_after"])) < NEAR_ZERO_J6:
        return "(c)", "joint 6 near zero (%.3f -> %.3f)" % (P["j6_before"], P["j6_after"])
    if P["at_limit_joints"]:
        return "(d)", "output joint(s) %s at or past a limit" % P["at_limit_joints"]
    if is_flip(P):
        return "(b)", "joint 6 sign flipped, joints 5 and 7 changed by about pi (%.2f, %.2f)" % (P["j5_dq"], P["j7_dq"])
    if U["max_dq"] < JUMP_RAD:
        return "(e)", "unpushed output stays within %.3f rad; pushed output moves joint %d by %.3f rad (no flip, no limit)" % (U["max_dq"], P["max_dq_joint"], P["max_dq"])
    return "other", "unpushed raw call also jumps %.3f rad (joint %d), no flip, no limit, joint 6 not near zero" % (U["max_dq"], U["max_dq_joint"])


def targets_of(row, n, joint=None):
    d = json.load(open(ROOT / f"data/safety_filter/hold_causes_row{row}.json"))
    out = []
    for c in d["all_hold_causes_short"]:
        m = re.match(r"joint (\d+) of the (left|right) arm jumped ([0-9.]+) rad; first seen at solver output", c["cause"])
        if c["reason"] == "BRANCH_GUARD" and m and (joint is None or int(m.group(1)) == joint):
            out.append({"tick": c["tick"], "joint": int(m.group(1)), "side": m.group(2), "jump": float(m.group(3))})
        if len(out) >= n:
            break
    return out


def replay_row(setup, row, n, joint=None):
    targets = targets_of(row, n, joint)
    by_tick = {t["tick"]: t for t in targets}
    last_tick = max(by_tick)
    first = min(by_tick)
    trace_ticks = {first - 3, first - 2, first - 1}      # the passing ticks before the first hold: every solver call of the tick is listed
    trace = []
    raw = setup.raw
    npz = np.load(ROOT / f"data/safety_filter/hold_causes_row{row}.npz")
    cases = []
    with setup.row_settings(row):
        res = setup.make_resolver(row)
        kin = res.kin
        orig = kin.convert_directions
        last, base = np.array(setup.q0, float), RobotState()
        for k in range(last_tick + 1):
            rec = Recorder(kin) if (k in by_tick or k in trace_ticks) else None
            if rec:
                rec.install()
            try:
                o, st = res.step(raw[k].copy(), last.copy(), base)
            finally:
                if rec:
                    rec.uninstall()
            o = np.asarray(o, float).reshape(2, 7)
            if not np.array_equal(o, npz["out"][k]):
                raise RuntimeError(f"row {row} tick {k}: replay output differs from the Stage 2c trace")
            if rec and k in trace_ticks:
                trace.append(trace_of(kin, rec, k, last, o, raw[k], kc._label(st.status)))
            if rec and k in by_tick:
                cases.append(analyse_case(kin, orig, rec, by_tick[k], last, raw[k], kc._label(st.status),
                                          kc._label(getattr(st, "hold_reason", None))))
            last = o.copy()
            if k % 20 == 0:
                print(f"row {row} tick {k}/{last_tick}", flush=True)
    return cases, trace


def trace_of(kin, rec, k, last, out, raw_k, status):
    """Every solver call of one tick (left arm): output change from the last filtered command, joints 5 and 6, and the final output."""
    calls = []
    for c in rec.calls:
        si = IDX[c['side']]
        if c['out'] is None:
            continue
        d = c['out'] - last[si]
        calls.append({'side': c['side'], 'max_change_from_last': float(np.abs(d).max()), 'joint': int(np.abs(d).argmax() + 1),
                      'j5_out': float(c['out'][4]), 'j6_out': float(c['out'][5]), 'j4_out': float(c['out'][3])})
    return {'tick': k, 'status': status, 'n_calls': len(calls), 'calls': calls,
            'final_left_j4_j5_j6': [float(x) for x in out[0][3:6]], 'last_left_j4_j5_j6': [float(x) for x in last[0][3:6]],
            'raw_left_j4_j5_j6': [float(x) for x in raw_k[0][3:6]]}


def analyse_case(kin, orig, rec, tgt, last, raw_k, status, reason):
    side = tgt["side"]
    si = IDX[side]
    # the first recorded call of this arm whose output is more than JUMP_RAD from the last filtered command
    first = next((c for c in rec.calls if c["side"] == side and c["out"] is not None and np.abs(c["out"] - last[si]).max() > JUMP_RAD), None)
    case = {"tick": tgt["tick"], "side": side, "stage2c_joint": tgt["joint"], "stage2c_jump": tgt["jump"], "status": status, "hold_reason": reason,
            "n_solver_calls_this_tick": len(rec.calls)}
    if first is None:
        case.update({"cause": "other", "description": "no solver call of this arm jumped over %.2f rad when replayed" % JUMP_RAD})
        return case
    state = first["state"]
    S, _, _ = kin.sew(state, side)
    # unpushed keypoints of the raw command and of the last filtered command (our forward kinematics)
    Sr, Er, Wr = kin.sew(kc.state_of(raw_k), side)
    Hr, Rr = kin.hand_pose(kc.state_of(raw_k), side)
    Sl, El, Wl = kin.sew(kc.state_of(last), side)
    Hl, Rl = kin.hand_pose(kc.state_of(last), side)
    ur, lr = (Er - Sr) / np.linalg.norm(Er - Sr), (Wr - Er) / np.linalg.norm(Wr - Er)
    ul, ll = (El - Sl) / np.linalg.norm(El - Sl), (Wl - El) / np.linalg.norm(Wl - El)
    u, l = first["u"], first["l"]
    ang = lambda a, b: float(np.degrees(np.arccos(np.clip(a @ b, -1, 1))))
    # implied pushed keypoints with the raw link lengths (the solver reads directions only)
    E2 = Sr + np.linalg.norm(Er - Sr) * u
    W2 = E2 + np.linalg.norm(Wr - Er) * l
    H2 = W2 + (Hr - Wr)
    pushed_in = {"elbow_push_mm": float(np.linalg.norm(E2 - Er) * 1000), "wrist_push_mm": float(np.linalg.norm(W2 - Wr) * 1000),
                 "hand_push_mm": float(np.linalg.norm(H2 - Hr) * 1000), "upper_dir_change_deg": ang(u, ur), "fore_dir_change_deg": ang(l, lr),
                 "hand_rot_change_deg": float(np.degrees(rotation_error_rad(first["rot"], Rr)))}
    P = summarise(kin, state, side, first["out"], last[si], H2, first["rot"])
    P["hand_err_vs_raw_pose_mm"], P["hand_err_vs_raw_pose_deg"] = pose_err(kin, state, side, first["out"], Hr, Rr)
    U = describe(kin, orig, state, side, last[si], ur, lr, Rr, Hr, Rr)
    C = describe(kin, orig, state, side, last[si], ul, ll, Rl, Hl, Rl)
    state_last = with_arms(state, np.array([last[0], last[1]]))
    C2 = describe(kin, orig, state_last, side, last[si], ul, ll, Rl, Hl, Rl)
    for X in (U, C, C2):
        X["j6_before"] = float(last[si][5])
    # the raw command itself against the last filtered one (how far the stream moves per tick)
    cause, why = classify(P, U, C)
    sens = P["max_dq"] / max(1e-9, np.radians(max(pushed_in["upper_dir_change_deg"], pushed_in["fore_dir_change_deg"])))
    case.update({"solver_inputs": pushed_in, "pushed": P, "unpushed": U, "roundtrip_seeded_current": C, "roundtrip_seeded_last": C2,
                 "raw_vs_last_max_dq": float(np.abs(raw_k[si] - last[si]).max()), "sensitivity_rad_per_rad": float(sens),
                 "flip_pattern": is_flip(P), "cause": cause, "description": why})
    # elbow bend (joint 4 as the angle between the solver's upper-arm and forearm directions), joints 3 and 5 sums, wrist error
    bend_in = float(np.arccos(np.clip(u @ l, -1, 1)))
    qs = np.array([state.q_left, state.q_right], float)
    qs[si] = first["out"]
    W_out = kin.sew(with_arms(state, qs), side)[2]
    out_v, last_v = np.asarray(first["out"], float), np.asarray(last[si], float)
    s_before, s_after = float(last_v[2] + last_v[4]), float(out_v[2] + out_v[4])
    d3, d5 = float(out_v[2] - last_v[2]), float(out_v[4] - last_v[4])
    sum_unchanged = bool(abs(s_after - s_before) < SUM_TOL and abs(d3) > JOINT_JUMP and abs(d5) > JOINT_JUMP)
    case["elbow"] = {"bend_input_rad": bend_in, "bend_input_below_snap": bool(bend_in < SNAP_J4),
                     "joint4_last_rad": float(last_v[3]), "joint4_last_below_snap": bool(last_v[3] < SNAP_J4),
                     "bend_last_from_directions_rad": float(np.arccos(np.clip(ul @ ll, -1, 1))),
                     "q3_plus_q5_before": s_before, "q3_plus_q5_after": s_after, "d_q3": d3, "d_q5": d5,
                     "sum_change": s_after - s_before, "sum_almost_unchanged_joints_3_5_jump": sum_unchanged,
                     "wrist_err_mm": float(np.linalg.norm(W_out - W2) * 1000)}
    return case


def elbow_table(cases):
    L = ["| tick | arm | bend of solver input (rad) | joint 4 of last filtered (rad) | bend < 0.242? | q3+q5 before > after (rad) | d q3 / d q5 (rad) | sum almost unchanged, joints 3 and 5 jump? | wrist error of solver output (mm) | cause |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    for c in cases:
        e = c.get("elbow")
        if e is None:
            L.append(f"| {c['tick']} | {c['side']} | - | - | - | - | - | - | - | {c['cause']} |")
            continue
        L.append(f"| {c['tick']} | {c['side']} | {e['bend_input_rad']:.3f} | {e['joint4_last_rad']:.3f} | {'yes' if e['bend_input_below_snap'] else 'no'} | "
                 f"{e['q3_plus_q5_before']:.3f} > {e['q3_plus_q5_after']:.3f} | {e['d_q3']:.2f} / {e['d_q5']:.2f} | "
                 f"{'yes' if e['sum_almost_unchanged_joints_3_5_jump'] else 'no'} | {e['wrist_err_mm']:.1f} | {c['cause']} |")
    return "\n".join(L)


def split_summary(cases):
    """Cause counts split by elbow bend below the snap value (yes/no); within yes, the sum-unchanged count."""
    yes = [c for c in cases if "elbow" in c and c["elbow"]["bend_input_below_snap"]]
    no = [c for c in cases if "elbow" in c and not c["elbow"]["bend_input_below_snap"]]
    def counts(xs):
        d = {}
        for c in xs:
            d[c["cause"]] = d.get(c["cause"], 0) + 1
        return d
    return {"bend_below_snap_yes": {"n": len(yes), "causes": counts(yes),
                                    "joints_3_5_jump_with_sum_almost_unchanged": sum(1 for c in yes if c["elbow"]["sum_almost_unchanged_joints_3_5_jump"])},
            "bend_below_snap_no": {"n": len(no), "causes": counts(no),
                                   "joints_3_5_jump_with_sum_almost_unchanged": sum(1 for c in no if c["elbow"]["sum_almost_unchanged_joints_3_5_jump"])}}


def holds_joint4_full_stream(row):
    """All holds of the full Stage 2c stream (no replay): joint 4 of the last filtered command of each hold, either arm."""
    z = np.load(ROOT / f"data/safety_filter/hold_causes_row{row}.npz")
    out, status = z["out"], z["status"]
    ks = [k for k in range(1, len(status)) if status[k] == "HOLD"]
    j4 = np.array([[out[k - 1][0][3], out[k - 1][1][3]] for k in ks])
    below = np.any(j4 < SNAP_J4, axis=1)
    return {"holds": len(ks), "holds_with_an_arm_joint4_below_snap": int(below.sum()),
            "min_joint4_last_filtered": float(j4.min()) if len(ks) else None,
            "max_joint4_last_filtered": float(j4.max()) if len(ks) else None}


def md_table(cases):
    L = ["| tick | arm | max change (rad, joint) | joint 6 before > after | elbow / wrist push (mm) | dir change upper / fore (deg) | raw command vs last, max (rad) | unpushed: max change (rad, joint) | round trip: max change (rad, joint) | j5 / j7 change (rad) | limit | hand error vs raw (mm / deg) | cause |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for c in cases:
        if "pushed" not in c:
            L.append(f"| {c['tick']} | {c['side']} | - | - | - | - | - | - | - | - | - | - | {c['cause']}: {c['description']} |")
            continue
        P, U, C, I = c["pushed"], c["unpushed"], c["roundtrip_seeded_current"], c["solver_inputs"]
        L.append(f"| {c['tick']} | {c['side']} | {P['max_dq']:.3f} (j{P['max_dq_joint']}) | {P['j6_before']:.3f} > {P['j6_after']:.3f} | "
                 f"{I['elbow_push_mm']:.1f} / {I['wrist_push_mm']:.1f} | {I['upper_dir_change_deg']:.2f} / {I['fore_dir_change_deg']:.2f} | "
                 f"{c['raw_vs_last_max_dq']:.3f} | {U['max_dq']:.3f} (j{U['max_dq_joint']}) | {C['max_dq']:.3f} (j{C['max_dq_joint']}) | {P['j5_dq']:.2f} / {P['j7_dq']:.2f} | "
                 f"{P['at_limit_joints'] or '-'} | {P['hand_err_vs_raw_pose_mm']:.1f} / {P['hand_err_vs_raw_pose_deg']:.2f} | {c['cause']} |")
    return "\n".join(L)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--rows", type=int, nargs="+", default=[6, 4])
    ap.add_argument("--out", default="data/safety_filter/solver_jump_replay.json")
    ap.add_argument("--joint", type=int, default=None, help="only holds whose Stage 2c cause names this joint (e.g. 6); default: the first holds of any joint")
    args = ap.parse_args(argv)
    setup = dc.Setup(full_stream=True)
    result = {"jump_rad": JUMP_RAD, "near_zero_j6_rad": NEAR_ZERO_J6, "flip_tol_rad": FLIP_TOL, "rows": {}}
    for row in args.rows:
        cases, trace = replay_row(setup, row, args.n, args.joint)
        if not cases:
            print(f"row {row}: no holds naming joint {args.joint}")
        counts = {}
        for c in cases:
            counts[c["cause"]] = counts.get(c["cause"], 0) + 1
        result["rows"][str(row)] = {"cases": cases, "cause_counts": counts, "pre_hold_trace": trace}
        print(f"\n### row {row}: {dc.ROWS[row][0]}\n")
        print(md_table(cases))
        print("\ncause counts:", counts)
        print("\nElbow bend and joints 3 and 5 (per case)\n")
        print(elbow_table(cases))
        split = split_summary(cases)
        result.setdefault("elbow_split", {})[str(row)] = split
        print("\nSplit by elbow bend below %.3f rad:" % SNAP_J4, json.dumps(split))
        result.setdefault("holds_full_stream", {})[str(row)] = holds_joint4_full_stream(row)
        print("Holds of the full stream, joint 4 of the last filtered command:", result["holds_full_stream"][str(row)])
        print("\nSolver calls of the ticks before the first hold (left arm j4, j5, j6; last = last filtered command)")
        for tr in trace:
            print(f"tick {tr['tick']} {tr['status']}: last {np.round(tr['last_left_j4_j5_j6'], 3).tolist()} raw {np.round(tr['raw_left_j4_j5_j6'], 3).tolist()} final {np.round(tr['final_left_j4_j5_j6'], 3).tolist()}")
            for c in tr["calls"]:
                print(f"   call {c['side']}: max change from last {c['max_change_from_last']:.3f} (j{c['joint']}); j4 {c['j4_out']:.3f} j5 {c['j5_out']:.3f} j6 {c['j6_out']:.3f}")
        seen = set()
        for c in cases:
            if c["cause"] not in seen:
                seen.add(c["cause"])
                print(f"example {c['cause']}: tick {c['tick']} {c['side']}: {c['description']}")
    tmp = str(args.out) + ".tmp"
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    with open(tmp, "w") as f:
        json.dump(result, f, indent=1)
    os.replace(tmp, args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
