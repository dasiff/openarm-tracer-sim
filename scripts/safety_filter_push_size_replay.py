"""Stage 2f: how big are the pushes behind the branch-guard holds that name joint 6 or joint 7? (measurement only)

Nothing in src/ or config is changed. The resolver is run from tick 0 on the recorded raw stream exactly as in Stage 2c, with its
functions wrapped from outside. The wrappers only record; the replayed output is compared with the Stage 2c trace at every tick
(a mismatch stops the run). Nothing is fed back into the resolver.

Holds replayed (all BRANCH_GUARD holds whose jump was first seen at the solver output, from data/safety_filter/hold_causes_row{4,6}.json):
  row 6 (kong_style):  naming joint 6 (both arms)
  row 4 (layered):     naming joint 6 (10) or joint 7 (241, both arms)
--sample N keeps only the first N holds per row (tick order) for a quick check.

Per hold (record keys in build_hold, all lengths in mm, angles in rad):
  named joint: last filtered value and distance to its nearest limit; the last solver output of the holding arm for that joint and
  how far beyond its limit it is (positive = beyond); the final pushed value; the solver's bend (joint 4 of the solver input).
  push_mm: displacement of the elbow (E), wrist (W) and hand reference point (H) from the final pushed pose back to three starts:
     raw   = the stream's raw command keypoints,
     ii    = the pose the pushes start from (kong_style: after the jump-through guard; layered: the first round's pose, which is
             the raw command because the layered resolver has no guard and starts its rounds from it),
     last  = the last filtered command keypoints.
  rounds: one per kong_style sweep or layered round: its pairs (names, categories, type, gap now / at the raw command / at last,
     margin, shortfall, required gap), per point the summed and largest single contribution (before summing), tau and its factor,
     pre-restore vs post-restore displacement (link-length restore), plan re-check, planned vs achieved after conversion,
     achieved and cumulative displacement from ii (both arms).
  guard: whether the jump-through guard shortened the step (kong_style).
  across_rounds: summed achieved displacement over rounds vs the largest single round (per point, holding arm).
  scale: the holding arm's displacement from ii scaled by 1.0, 0.5, 0.25, 0.1, 0.05, converted the way the resolver converts
     (restore_lengths, Kong's solver, one Jacobian step); joint limits, largest joint change from last, branch-guard pass.

JSON layout (read by scripts/safety_filter_push_size_tables.py):
  {"census": {"6": {...}, "4": {...}}, "rows": {"6": {"name", "mode", "ticks_run", "targets", "replayed", "holds": [hold, ...]},
   "4": {...}}, "branch_guard_rad", "scales", "near_limit_rad", "sample", "runtime_s"}
  hold = {"row", "tick", "arm", "named_joint", "cause", "jump_rad", "replay_reason", "replay_ok", "mode",
          "last": {"j4", "j6", "j7", "named", "j6_dist_limit", "j7_dist_limit", "named_dist_limit"},
          "solver": {"n_calls_arm", "bend_rad", "named_out", "named_change", "named_beyond_rad", "max_change_rad",
                     "max_change_joint", "j6_out", "j6_beyond_rad"} or None,
          "final": {"named", "named_change", "named_beyond_rad", "max_change_rad", "max_change_joint"},
          "push_mm": {"raw": {"E", "W", "H"}, "ii": {...}, "last": {...}},
          "guard": {"shortened", "samples", "fraction"} or None, "shorten_events": [...],
          "rounds": [round, ...], "across_rounds": {"E"/"W"/"H": {...}}, "largest_round": {"index", "pairs"},
          "cum_from_ii_mm": [{"E","W","H"} per round, holding arm], "sweep1_over_final": {"E","W","H"},
          "scale": {"by_scale": {"1.0": {...}, ...}, "largest_pass_scale", "largest_pass_mm"}, "scale1_vs_final_rad"}
Usage:
  python scripts/safety_filter_push_size_replay.py --sample 20 --out data/safety_filter/push_size_replay_sample.json
  python scripts/safety_filter_push_size_replay.py --out data/safety_filter/push_size_replay.json
"""

import argparse
import json
import math
import os
import re
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
import safety_filter_diag_common as dc  # noqa: E402
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter import kong_style as KS  # noqa: E402
from src.safety_filter import resolver as R  # noqa: E402
from src.safety_filter.arm_conversion import rotation_error_rad  # noqa: E402
from src.safety_filter.gap_rule import required_gap_next  # noqa: E402
from src.safety_filter.link_rules import FOREARM, HAND, UPPER_ARM, category_of  # noqa: E402
from src.safety_filter.push_direction import contact_fraction, pair_push  # noqa: E402
from src.safety_filter.shapes import RobotState, SIDES  # noqa: E402

kc = dc.kc
SCALES = (1.0, 0.5, 0.25, 0.1, 0.05)           # step 3 scale factors
NEAR_LIMIT_RAD = 0.05                          # rad: "within 0.05 rad of its limit" (task text)
TARGET_JOINTS = {6: (6,), 4: (6, 7)}           # 1-based joints whose solver-output holds are replayed, per row
EXPECTED_TARGETS = {6: 1461, 4: 251}           # 1461 joint 6 (row 6); 10 joint 6 + 241 joint 7 (row 4)
CAUSE_RE = re.compile(r"joint (\d+) of the (left|right) arm jumped ([0-9.]+) rad; first seen at (\w+) output")
IDX = {"left": 0, "right": 1}
PTS = ("E", "W", "H")


# ---------------------------------------------------------------- small helpers
def mm(x):
    x = float(x)
    return x * 1000.0 if math.isfinite(x) else None


def norm(v):
    return float(np.linalg.norm(np.asarray(v, float)))


def sum_vec(vs):
    return np.sum([np.asarray(v, float) for v in vs], axis=0) if vs else np.zeros(3)


def beyond(x, lo, hi):
    """Rad past the nearer limit: positive = beyond a limit, negative = inside by that much."""
    return float(max(x - hi, lo - x))


def point_stats(vs):
    """Summed vs largest single contribution to one point (nonzero contributions only)."""
    vs = [np.asarray(v, float) for v in vs if norm(v) > 1e-12]
    if not vs:
        return {"n": 0, "sum_mm": 0.0, "max_single_mm": 0.0, "ratio_vec": None, "ratio_mag": None}
    s = np.sum(vs, axis=0)
    mags = [norm(v) for v in vs]
    mx = max(mags)
    return {"n": len(vs), "sum_mm": mm(norm(s)), "max_single_mm": mm(mx), "ratio_vec": norm(s) / mx, "ratio_mag": sum(mags) / mx}


def clean(x):
    if isinstance(x, dict):
        return {str(k): clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [clean(v) for v in x]
    if isinstance(x, np.ndarray):
        return clean(x.tolist())
    if isinstance(x, np.bool_):
        return bool(x)
    if isinstance(x, np.integer):
        return int(x)
    if isinstance(x, (float, np.floating)):
        return float(x) if math.isfinite(float(x)) else None
    return x


# ---------------------------------------------------------------- the holds (Stage 2c traces)
def census(row):
    """Every hold of the Stage 2c trace: reason, and for BRANCH_GUARD the stage, arm and named joint."""
    d = json.load(open(ROOT / f"data/safety_filter/hold_causes_row{row}.json"))
    L = d["all_hold_causes_short"]
    bg = Counter()
    for c in L:
        if c["reason"] != "BRANCH_GUARD":
            continue
        m = CAUSE_RE.match(c["cause"])
        bg[f"{m.group(4)}|{m.group(2)}|joint {m.group(1)}" if m else "unparsed"] += 1
    return {"holds_total": len(L), "by_reason": dict(Counter(c["reason"] for c in L)), "branch_guard": dict(bg)}


def targets_of(row, sample):
    d = json.load(open(ROOT / f"data/safety_filter/hold_causes_row{row}.json"))
    out = []
    for c in d["all_hold_causes_short"]:
        m = CAUSE_RE.match(c["cause"])
        if c["reason"] == "BRANCH_GUARD" and m and m.group(4) == "solver" and int(m.group(1)) in TARGET_JOINTS[row]:
            out.append({"tick": int(c["tick"]), "arm": m.group(2), "joint": int(m.group(1)), "jump": float(m.group(3)), "cause": c["cause"]})
    out.sort(key=lambda x: x["tick"])
    ticks = [x["tick"] for x in out]
    if len(set(ticks)) != len(ticks):
        raise RuntimeError(f"row {row}: two holds on one tick")
    return out[:sample] if sample else out


# ---------------------------------------------------------------- recording (wrappers only record)
class Rec:
    def __init__(self):
        self.on = False
        self.reset()

    def reset(self):
        self.t = None
        self.raw_k = None
        self.calls = []             # solver calls of the tick: {side, u, l, out, err}
        self.snaps = []             # (2,7) pose at the entry of each round and at the checks
        self.rounds = []
        self.start_q = None         # pose ii
        self.guard = None
        self.final_q = None
        self.raw_post = None
        self.shorten_events = []

    def begin(self, raw_k):
        self.reset()
        self.raw_k = np.array(raw_k, float)
        self.on = True

    def note_entry(self, q):
        self.snaps.append(np.array(q, float))
        if self.start_q is None:
            self.start_q = np.array(q, float)


REC = Rec()
ORIG = {}


def install_convert(kin):
    orig = kin.convert_directions

    def conv(state, side, u, l, rot):
        try:
            out = orig(state, side, u, l, rot)
        except RuntimeError as exc:
            if REC.on:
                REC.calls.append({"side": side, "u": np.array(u, float), "l": np.array(l, float), "out": None, "err": str(exc)})
            raise
        if REC.on:
            REC.calls.append({"side": side, "u": np.array(u, float), "l": np.array(l, float), "out": np.array(out, float), "err": None})
        return out

    kin.convert_directions = conv
    return lambda: kin.__dict__.pop("convert_directions", None)


def pair_meta(t, res, key, short, q, raw_k):
    a, b = t.shape(q, key[0]), t.shape(q, key[1])

    def catname(s):
        return category_of(s) if s.arm in SIDES else "body"

    gap_now = t.gap(q, key)
    g0 = t.g0[key]
    margin = t.rules[key].margin
    return {"pair": [key[0], key[1]], "cats": [catname(a), catname(b)], "type": dc.pair_type(a, b, {}),
            "gap_now_mm": mm(gap_now), "gap_raw_mm": mm(t.gap(raw_k, key)), "g0_mm": mm(g0), "margin_mm": mm(margin),
            "required_gap_mm": mm(required_gap_next(g0, margin, res.dt)), "shortfall_mm": mm(short),
            "margin_minus_gap_mm": mm(margin - gap_now)}


def pair_rows(res, t, q, key, short, kp, raw_k, kong):
    """One row per closing arm of a pair: what the pair asks of the arm. kong: the displacement of E, W, H exactly as plan_sweep
    computes it (same functions); layered: the planned push goes to the elbow (or hand), recorded but not summed here."""
    arms = t.closing(key)
    out = []
    if not arms:
        return out
    share = short / len(arms)
    a, b = t.shape(q, key[0]), t.shape(q, key[1])
    pp = pair_push(a, b)
    meta = pair_meta(t, res, key, short, q, raw_k)
    for side in arms:
        mine, direction, point = (a, pp.dir_a, pp.point_a) if a.arm == side else (b, pp.dir_b, pp.point_b)
        c = category_of(mine)
        S, E, W, H = kp[side]
        pv = np.asarray(direction, float) * share
        row = dict(meta, side=side, cat=c, push_mm=mm(share), tau=None, tau_factor=None, shorten=False,
                   lever_E=None, lever_W=None, contrib_mm=None)
        vec = {"E": np.zeros(3), "W": np.zeros(3), "H": np.zeros(3)}
        if c == UPPER_ARM:
            row["tau"] = contact_fraction(point, S, E)
        elif c == FOREARM:
            row["tau"] = contact_fraction(point, E, W)
        if not kong:                                  # layered: the planned push is recorded only (its own tools move the arm)
            out.append((row, side, vec))
            continue
        if c == HAND:
            vec["W"], vec["H"] = KS.hand_displacement(pv)
        elif c == UPPER_ARM:
            if row["tau"] < cfg.RESOLVER["min_lever_fraction"]:
                row["shorten"] = True
            else:
                vec["E"] = KS.upper_arm_displacement(pv, row["tau"])
                row["tau_factor"] = 1.0 / row["tau"]
        elif c == FOREARM:
            vec["E"], vec["W"] = KS.forearm_displacement(pv, row["tau"])
            row["lever_E"] = norm(vec["E"]) / share
            row["lever_W"] = norm(vec["W"]) / share
        else:
            row["shorten"] = True
        if row["shorten"]:
            vec = {"E": np.zeros(3), "W": np.zeros(3), "H": np.zeros(3)}
        row["contrib_mm"] = {P: mm(norm(vec[P])) for P in PTS}
        out.append((row, side, vec))
    return out


def plan_wrap(res, t, q, pairs, cons):
    if not REC.on:
        return ORIG["plan"](res, t, q, pairs, cons)
    REC.t = t
    REC.note_entry(q)
    kp = KS._keypoints(res, t, q)
    rows, contrib = [], {s: {P: [] for P in PTS} for s in SIDES}
    for key, short in pairs:
        for row, side, vec in pair_rows(res, t, q, key, short, kp, REC.raw_k, kong=True):
            rows.append(row)
            if not row["shorten"]:
                for P in PTS:
                    contrib[side][P].append(vec[P])
    plan = ORIG["plan"](res, t, q, pairs, cons)            # the real call
    rnd = {"index": len(REC.rounds), "kind": "sweep", "tools": ["kong_sweep"], "pairs": rows, "cats": list(plan.cats),
           "points": {s: {P: point_stats(contrib[s][P]) for P in PTS} for s in SIDES},
           "tau_effect": {}, "pre_restore_mm": {}, "post_restore_mm": {}, "plan_recheck_mm": {},
           "pushed_sides": sorted(plan.pushed), "shorten_sides": sorted(plan.shorten), "planned": {}}
    for s in SIDES:
        up = [r for r in rows if r["side"] == s and r["cat"] == UPPER_ARM and not r["shorten"]]
        und = sum(r["push_mm"] for r in up)
        div = sum(r["contrib_mm"]["E"] for r in up)
        rnd["tau_effect"][s] = {"n_upper": len(up), "sum_undivided_mm": und, "sum_divided_mm": div, "ratio": (div / und if und else None)}
    for s in plan.pushed:
        S, E, W, H = kp[s]
        lengths, offset = cons[s]
        Ep = E + sum_vec(contrib[s]["E"])
        Wp = W + sum_vec(contrib[s]["W"])
        Hp = Wp + offset
        E2, W2, H2 = plan.pushed[s]
        rnd["pre_restore_mm"][s] = {"E": mm(norm(Ep - E)), "W": mm(norm(Wp - W)), "H": mm(norm(Hp - H))}
        rnd["post_restore_mm"][s] = {"E": mm(norm(E2 - E)), "W": mm(norm(W2 - W)), "H": mm(norm(H2 - H))}
        Er, Wr, Hr = KS.restore_lengths(S, Ep, Wp, lengths, offset)
        rnd["plan_recheck_mm"][s] = mm(max(norm(Er - E2), norm(Wr - W2), norm(Hr - H2)))
        rnd["planned"][s] = (E2, W2, H2)
    REC.rounds.append(rnd)
    return plan


def fix_wrap(self, t, q, worst, methods, targets, st):
    if not REC.on:
        return ORIG["fix"](self, t, q, worst, methods, targets, st)
    REC.t = t
    REC.note_entry(q)
    kp = KS._keypoints(self, t, q)
    key, short = worst
    rows = [row for row, _, _ in pair_rows(self, t, q, key, short, kp, REC.raw_k, kong=False)]
    n0 = len(methods)
    ORIG["fix"](self, t, q, worst, methods, targets, st)  # the real call
    REC.rounds.append({"index": len(REC.rounds), "kind": "round", "tools": list(methods[n0:]), "pairs": rows, "cats": [],
                       "points": {}, "tau_effect": {}, "pre_restore_mm": {}, "post_restore_mm": {}, "plan_recheck_mm": {},
                       "pushed_sides": [], "shorten_sides": [], "planned": {}})


def guard_wrap(res, t, methods, targets, st):
    if not REC.on:
        return ORIG["guard"](res, t, methods, targets, st)
    REC.t = t
    n = KS._path_samples(res, t)
    shortened = ORIG["guard"](res, t, methods, targets, st)   # the real call
    REC.guard = {"shortened": bool(shortened), "samples": int(n), "fraction": (float(st.fraction) if shortened else None)}
    REC.start_q = np.array(t.raw, float)                     # pose ii for kong_style: the command after the guard
    return shortened


def shorten_wrap(self, t, q, sides, methods, targets, st, passes=None):
    if not REC.on:
        return ORIG["shorten"](self, t, q, sides, methods, targets, st, passes=passes)
    ev = {"sides": sorted(sides), "after_round": len(REC.rounds), "fraction": None}
    REC.shorten_events.append(ev)
    try:
        return ORIG["shorten"](self, t, q, sides, methods, targets, st, passes=passes)   # the real call
    finally:
        f = float(st.fraction)
        ev["fraction"] = f if math.isfinite(f) else None


def checks_wrap(self, t, q, targets):
    if not REC.on:
        return ORIG["checks"](self, t, q, targets)
    REC.t = t
    REC.snaps.append(np.array(q, float))
    REC.final_q = np.array(q, float)
    REC.raw_post = np.array(t.raw, float)
    if REC.start_q is None:
        REC.start_q = np.array(q, float)
    return ORIG["checks"](self, t, q, targets)               # the real call


def install_wrappers():
    ORIG.update(plan=KS.plan_sweep, guard=KS._guard, fix=R.Resolver._fix, shorten=R.Resolver._shorten, checks=R.Resolver._checks)
    KS.plan_sweep, KS._guard = plan_wrap, guard_wrap
    R.Resolver._fix, R.Resolver._shorten, R.Resolver._checks = fix_wrap, shorten_wrap, checks_wrap


def uninstall_wrappers():
    KS.plan_sweep, KS._guard = ORIG["plan"], ORIG["guard"]
    R.Resolver._fix, R.Resolver._shorten, R.Resolver._checks = ORIG["fix"], ORIG["shorten"], ORIG["checks"]


# ---------------------------------------------------------------- per-hold analysis
def finalize_rounds(res, t, side, kp_ii):
    """Achieved displacement per round (both arms), cumulative from ii, planned vs achieved, and the holding arm's totals."""
    kps = [KS._keypoints(res, t, s) for s in REC.snaps]
    out, across = [], {P: [] for P in PTS}
    for r, rnd in enumerate(REC.rounds):
        if r + 1 >= len(kps):
            break
        rnd = dict(rnd)
        ach, cum, pva = {}, {}, {}
        for s in SIDES:
            S0, E0, W0, H0 = kps[r][s]
            S1, E1, W1, H1 = kps[r + 1][s]
            vecs = {"E": E1 - E0, "W": W1 - W0, "H": H1 - H0}
            ach[s] = {P: mm(norm(vecs[P])) for P in PTS}
            Sc, Ec, Wc, Hc = kp_ii[s]
            cum[s] = {"E": mm(norm(E1 - Ec)), "W": mm(norm(W1 - Wc)), "H": mm(norm(H1 - Hc))}
            if s in rnd["planned"]:
                E2, W2, H2 = rnd["planned"][s]
                pva[s] = mm(max(norm(E1 - E2), norm(W1 - W2), norm(H1 - H2)))
            if s == side:
                for P in PTS:
                    across[P].append(vecs[P])
        rnd.pop("planned")
        rnd["achieved_mm"] = ach
        rnd["cum_from_ii_mm"] = cum
        rnd["planned_vs_achieved_mm"] = pva
        out.append(rnd)
    totals = {}
    for P in PTS:
        vs = [v for v in across[P] if norm(v) > 1e-12]
        if vs:
            s_, mx = norm(np.sum(vs, axis=0)), max(norm(v) for v in vs)
            totals[P] = {"sum_mm": mm(s_), "max_single_mm": mm(mx), "ratio_vec": s_ / mx, "ratio_mag": sum(norm(v) for v in vs) / mx}
        else:
            totals[P] = {"sum_mm": 0.0, "max_single_mm": 0.0, "ratio_vec": None, "ratio_mag": None}
    return out, totals


def largest_round(rounds, side):
    best = None
    for rnd in rounds:
        m = max((rnd["achieved_mm"][side][P] or 0.0) for P in PTS)
        if best is None or m > best[1]:
            best = (rnd["index"], m)
    if best is None or best[1] <= 0.0:
        return None
    rnd = rounds[best[0]]
    rows = [{"side": r["side"], "cat": r["cat"], "pair": r["pair"], "type": r["type"], "push_mm": r["push_mm"]}
            for r in rnd["pairs"] if "side" in r]
    return {"index": best[0], "max_achieved_mm": best[1], "tools": rnd["tools"], "pairs": rows}


def scale_test(res, t, side, last_arm, q_ii, kp_ii, kp_f, final_arm, r_cmd):
    """Step 3: the holding arm's total push (from ii to the final pose) scaled by SCALES, converted as the resolver converts."""
    S, E0, W0, H0 = kp_ii[side]
    lengths = (norm(E0 - S), norm(W0 - E0))
    offset = H0 - W0
    DE = kp_f[side][1] - E0
    DW = kp_f[side][2] - W0
    lim = res.kin.limits[side]
    gj = list(cfg.RESOLVER["branch_guard_joints"])
    by = {}
    for s in (0.0,) + SCALES:                            # 0.0 is a control (no push): the solver's own error; not in the pass test
        E_s = E0 + s * DE
        W_s = W0 + s * DW
        E2, W2, H2 = KS.restore_lengths(S, E_s, W_s, lengths, offset)
        try:
            qa = np.asarray(KS._convert(res, t, q_ii, side, S, E2, W2, H2, r_cmd), float)
        except R._Hold as h:
            by[str(s)] = {"solver_hold": str(h.reason), "pass": False}
            continue
        all_within = bool(np.all((qa >= lim[:, 0]) & (qa <= lim[:, 1])))
        dq = np.abs(qa - last_arm)
        guard_pass = bool(np.max(dq[gj]) <= cfg.BRANCH_GUARD_RAD)
        q2 = np.array(q_ii, float)
        q2[IDX[side]] = qa
        kp2 = KS._keypoints(res, t, q2)[side]
        by[str(s)] = {"j6_out": float(qa[5]), "j6_within": bool(lim[5, 0] <= qa[5] <= lim[5, 1]), "all_within": all_within,
                      "max_change_rad": float(dq.max()), "max_change_joint": int(dq.argmax() + 1), "guard_pass": guard_pass,
                      "pass": bool(all_within and guard_pass), "E_mm": mm(norm(E2 - E0)), "W_mm": mm(norm(W2 - W0)),
                      "H_mm": mm(norm(H2 - H0)), "scaled_minus_final_max_rad": float(np.max(np.abs(qa - final_arm))),
                      "kp_err_mm": {"E": mm(norm(kp2[1] - E2)), "W": mm(norm(kp2[2] - W2)), "H": mm(norm(kp2[3] - H2))}}
    passing = [s for s in SCALES if by[str(s)].get("pass")]
    largest = max(passing) if passing else None
    lp = None
    if largest is not None:
        d = by[str(largest)]
        lp = {"scale": largest, "E": d["E_mm"], "W": d["W_mm"], "H": d["H_mm"]}
    return {"by_scale": by, "largest_pass_scale": largest, "largest_pass_mm": lp}


def roundtrip(res, t, q, side):
    """Solver alone on the keypoints of pose q (no push, no restore): joint error, keypoint error, hand orientation error."""
    S, E, W, H = KS._keypoints(res, t, q)[side]
    state = res._state(t, q)
    r = res.kin.hand_pose(state, side)[1]
    try:
        qc = np.asarray(KS.solve_arm(res, state, side, S, E, W, r), float)
    except RuntimeError as exc:
        return {"solver_error": str(exc)}
    dq = np.abs(qc - q[IDX[side]])
    q2 = np.array(q, float)
    q2[IDX[side]] = qc
    kp2 = KS._keypoints(res, t, q2)[side]
    rot2 = res.kin.hand_pose(res._state(t, q2), side)[1]
    return {"max_joint_rad": float(dq.max()), "max_joint_index": int(dq.argmax() + 1), "E_mm": mm(norm(kp2[1] - E)),
            "W_mm": mm(norm(kp2[2] - W)), "rot_deg": float(np.degrees(rotation_error_rad(r, rot2)))}


def analyse(res, row, tg, last, raw_k, st):
    REC.on = False
    side, ai, j = tg["arm"], IDX[tg["arm"]], tg["joint"]
    t = REC.t
    if t is None or REC.final_q is None:
        raise RuntimeError(f"row {row} tick {tg['tick']}: no checks recorded")
    lim = res.kin.limits[side]
    lo, hi = lim[:, 0], lim[:, 1]
    last_arm = np.asarray(last[ai], float)
    final_arm = np.asarray(REC.final_q[ai], float)
    k = j - 1
    reason = kc._label(st.hold_reason)
    status = kc._label(st.status)
    last_info = {"j4": float(last_arm[3]), "j6": float(last_arm[5]), "j7": float(last_arm[6]),
                 "named": float(last_arm[k]), "j6_dist_limit": float(min(last_arm[5] - lo[5], hi[5] - last_arm[5])),
                 "j7_dist_limit": float(min(last_arm[6] - lo[6], hi[6] - last_arm[6])),
                 "named_dist_limit": float(min(last_arm[k] - lo[k], hi[k] - last_arm[k]))}
    calls_arm = [c for c in REC.calls if c["side"] == side and c["out"] is not None]
    solver = None
    if calls_arm:
        c = calls_arm[-1]
        out = c["out"]
        dq = np.abs(out - last_arm)
        uu = c["u"] / np.linalg.norm(c["u"])
        ll = c["l"] / np.linalg.norm(c["l"])
        solver = {"n_calls_arm": len(calls_arm), "bend_rad": float(np.arccos(np.clip(np.dot(uu, ll), -1.0, 1.0))),
                  "named_out": float(out[k]), "named_change": float(out[k] - last_arm[k]),
                  "named_beyond_rad": beyond(out[k], lo[k], hi[k]), "max_change_rad": float(dq.max()),
                  "max_change_joint": int(dq.argmax() + 1), "j6_out": float(out[5]), "j6_beyond_rad": beyond(out[5], lo[5], hi[5])}
    dqf = np.abs(final_arm - last_arm)
    final = {"named": float(final_arm[k]), "named_change": float(final_arm[k] - last_arm[k]),
             "named_beyond_rad": beyond(final_arm[k], lo[k], hi[k]), "max_change_rad": float(dqf.max()),
             "max_change_joint": int(dqf.argmax() + 1)}
    kp_i = KS._keypoints(res, t, raw_k)
    kp_ii = KS._keypoints(res, t, REC.start_q)
    kp_l = KS._keypoints(res, t, last)
    kp_f = KS._keypoints(res, t, REC.final_q)
    push = {}
    for name, kp0 in (("raw", kp_i), ("ii", kp_ii), ("last", kp_l)):
        S0, E0, W0, H0 = kp0[side]
        S1, E1, W1, H1 = kp_f[side]
        push[name] = {"E": mm(norm(E1 - E0)), "W": mm(norm(W1 - W0)), "H": mm(norm(H1 - H0))}
    hand_minus_wrist = {name: mm(norm(kp0[side][3] - kp0[side][2])) for name, kp0 in
                        (("raw", kp_i), ("ii", kp_ii), ("last", kp_l), ("final", kp_f))}
    rounds, across = finalize_rounds(res, t, side, kp_ii)
    cum_hold = [r["cum_from_ii_mm"][side] for r in rounds]
    sweep1 = {}
    for P in PTS:
        if cum_hold and cum_hold[0][P] and cum_hold[-1][P]:
            sweep1[P] = cum_hold[0][P] / cum_hold[-1][P]
        else:
            sweep1[P] = None
    r_cmd = res.kin.hand_pose(res._state(t, REC.raw_post), side)[1]
    scale = scale_test(res, t, side, last_arm, REC.start_q, kp_ii, kp_f, final_arm, r_cmd)
    scale1 = scale["by_scale"]["1.0"]
    hold = {"row": row, "tick": tg["tick"], "arm": side, "named_joint": j, "cause": tg["cause"], "jump_rad": tg["jump"],
            "replay_reason": reason, "replay_status": status, "replay_ok": bool(status == "HOLD" and reason == "BRANCH_GUARD"),
            "mode": dc.ROWS[row][1], "last": last_info, "solver": solver, "final": final,
            "solver_max_change_is_named": (bool(solver["max_change_joint"] == j) if solver else None),
            "push_mm": push, "hand_minus_wrist_mm": hand_minus_wrist,
            "roundtrip": {"ii": roundtrip(res, t, REC.start_q, side), "raw": roundtrip(res, t, raw_k, side),
                          "last": roundtrip(res, t, last, side)},
            "guard": REC.guard, "shorten_events": REC.shorten_events, "rounds": rounds,
            "across_rounds": across, "largest_round": largest_round(rounds, side),
            "cum_from_ii_mm": cum_hold, "sweep1_over_final": sweep1,
            "scale": scale, "scale1_vs_final_rad": scale1.get("scaled_minus_final_max_rad"),
            "ii_vs_raw_same": bool(np.array_equal(REC.start_q, raw_k)),
            "last_j6_within_0p05_of_limit": bool(last_info["j6_dist_limit"] < NEAR_LIMIT_RAD)}
    return hold


# ---------------------------------------------------------------- the replay of one row
def replay_row(setup, row, targets):
    by_tick = {tg["tick"]: tg for tg in targets}
    last_tick = max(by_tick) if by_tick else -1
    npz = np.load(ROOT / f"data/safety_filter/hold_causes_row{row}.npz")["out"]
    holds = []
    with setup.row_settings(row):
        res = setup.make_resolver(row)
        restore_conv = install_convert(res.kin)
        install_wrappers()
        try:
            last = np.array(setup.q0, float)
            base = RobotState()
            for k in range(last_tick + 1):
                tg = by_tick.get(k)
                if tg is not None:
                    REC.begin(setup.raw[k])
                try:
                    o, st = res.step(setup.raw[k].copy(), last.copy(), base)
                finally:
                    REC.on = False
                o = np.asarray(o, float).reshape(2, 7)
                if not np.array_equal(o, npz[k]):
                    raise RuntimeError(f"row {row} tick {k}: replay output differs from the Stage 2c trace")
                if tg is not None:
                    holds.append(analyse(res, row, tg, last, setup.raw[k], st))
                    h = holds[-1]
                    print(f"row {row} tick {k} {h['arm']} j{h['named_joint']} {h['replay_reason']} "
                          f"push_ii E {h['push_mm']['ii']['E']:.1f} W {h['push_mm']['ii']['W']:.1f} mm, "
                          f"largest pass {h['scale']['largest_pass_scale']}", flush=True)
                last = o.copy()
                if k % 100 == 0:
                    print(f"row {row} tick {k}/{last_tick} holds {len(holds)}", flush=True)
        finally:
            uninstall_wrappers()
            restore_conv()
    return holds, last_tick


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--rows", type=int, nargs="+", default=[6, 4])
    ap.add_argument("--sample", type=int, default=0, help="first N holds per row (0 = all)")
    ap.add_argument("--out", default="data/safety_filter/push_size_replay.json")
    args = ap.parse_args(argv)
    t0 = time.time()
    setup = dc.Setup(full_stream=True)
    doc = {"branch_guard_rad": cfg.BRANCH_GUARD_RAD, "scales": list(SCALES), "near_limit_rad": NEAR_LIMIT_RAD,
           "sample": args.sample, "census": {}, "rows": {}}
    for row in args.rows:
        doc["census"][str(row)] = census(row)
        targets = targets_of(row, args.sample)
        if not args.sample and len(targets) != EXPECTED_TARGETS[row]:
            raise RuntimeError(f"row {row}: {len(targets)} targets, expected {EXPECTED_TARGETS[row]}")
        print(f"### row {row} ({dc.ROWS[row][0]}): {len(targets)} targets", flush=True)
        holds, last_tick = replay_row(setup, row, targets)
        doc["rows"][str(row)] = {"name": dc.ROWS[row][0], "mode": dc.ROWS[row][1], "ticks_run": last_tick + 1,
                                 "targets": len(targets), "replayed": len(holds),
                                 "replay_check": "identical at every tick run", "holds": holds}
        print(f"row {row}: replayed {len(holds)} of {len(targets)}; replay_ok {sum(h['replay_ok'] for h in holds)}", flush=True)
    doc["runtime_s"] = round(time.time() - t0, 1)
    out = ROOT / args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = str(out) + ".tmp"
    with open(tmp, "w") as f:
        json.dump(clean(doc), f)
    os.replace(tmp, out)
    print(f"wrote {out} in {doc['runtime_s']} s", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
