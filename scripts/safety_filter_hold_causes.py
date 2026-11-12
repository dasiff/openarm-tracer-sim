#!/usr/bin/env python
"""Stage 2c step 4: why the resolvers hold (measurement only; no filter code is changed).

One row (4, 5 or 6 of the Stage 2b comparison) runs the full rolling punch stream (1576 ticks). Pure recording wrappers (they only read
and pass the real results through) are put around the resolver's own functions from outside (Tracer), so every tick leaves a trace:
the guard samples, the shortening tries (fraction, pass or fail, the pair that failed), every solver output and Jacobian step with its
joint jump and hand error, and the four final checks. From the trace each HOLD gets a CAUSE (not only a reason code).
The first 3 holds of each reason are also REPLAYED: a second resolver gets the recorded last filtered command, raw command and the
recorded active-pair set, steps once with the tracer, and must give the same answer as the run (checked); the worked example (worst
pair, start-pose pairs below the guard gap, the stage of the jump, pairs below 5 cm) is written from that replay.
Lag test: of the no-contact ticks (raw smallest gap above 5 cm, same measure as Stage 2b) with hand error above 10 mm, the share that
lies within 20 ticks after a HOLD.

Usage (venv active, repo root): bash experiments/run_capped.sh data/logs/<stamp>_hold_row<N>.log python scripts/safety_filter_hold_causes.py --row 4 [--ticks N] [--out PATH]
"""

import argparse
import datetime as dt
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import safety_filter_diag_common as dc  # noqa: E402
from src.safety_filter import kong_style as ks  # noqa: E402
from src.safety_filter import resolver as R  # noqa: E402
from src.safety_filter.arm_conversion import ArmKinematics, rotation_error_rad, with_arms  # noqa: E402
from src.safety_filter.link_rules import category_of  # noqa: E402
from src.safety_filter.shapes import RobotState  # noqa: E402
from src.safety_filter.touch import Exempt, gap_between, pair_rule  # noqa: E402

cfg, kc, fx, SIDES = dc.cfg, dc.kc, dc.fx, dc.SIDES
REASONS = ("SHORTEN_ZERO", "BRANCH_GUARD", "ACCURACY", "JOINT_LIMITS")
EXAMPLES_PER_REASON = 3        # count: holds per reason that are replayed in detail (task text)
CLOSE_LIST_M = 0.05            # m: pairs below this gap at the last filtered command are listed in a worked example
LAG_WINDOW_TICKS = 20          # count: "within the 20 ticks after a hold"
LAG_ERROR_MM = 10.0            # mm: a hand error above this counts as lag
IDX = {"left": 0, "right": 1}
RAD_JOINT = lambda q: [round(float(v), 4) for v in q]


def names(key):
    return f"{key[0]} | {key[1]}"


class Tracer:
    """Recording wrappers around the resolver's own functions. Nothing is changed in the results; everything is restored in uninstall()."""

    def __init__(self, res):
        self.res = res
        self.kin2 = ArmKinematics(res.shapes)          # separate kinematics for the error numbers (does not touch res.kin's state)
        self.saved = []
        self.reset()

    def reset(self):
        self.tick = None
        self.last_gap = None
        self.last_worst = None
        self.unit = ("-", 0)               # ("round" | "sweep", index)
        self.events, self.shortens, self.checks = [], [], None
        self.guard = {"n_samples": None, "ran": False}
        self.in_shorten = None
        self.disc_count = 0

    def _patch(self, obj, name, new):
        self.saved.append((obj, name, obj.__dict__.get(name, None), name in obj.__dict__))
        setattr(obj, name, new)

    def install(self):
        tr, res = self, self.res
        o_init, o_gap, o_disc, o_worst = R._Tick.__init__, R._Tick.gap, R._Tick.discover, R._Tick.worst
        o_fix, o_short, o_checks = R.Resolver._fix, R.Resolver._shorten, R.Resolver._checks
        o_plan, o_guard, o_paths = ks.plan_sweep, ks._guard, ks._path_samples

        def init(t, *a, **k):
            o_init(t, *a, **k)
            tr.tick = t

        def gap(t, q, key):
            g = o_gap(t, q, key)
            tr.last_gap = (key, g)
            return g

        def disc(t, q):
            tr.disc_count += 1
            return o_disc(t, q)

        def worst(t, q, discover=False):
            w = o_worst(t, q, discover)
            tr.last_worst = w
            return w

        def fix(rs, t, q, w, methods, targets, st):
            tr.unit = ("round", tr.unit[1] + 1)
            return o_fix(rs, t, q, w, methods, targets, st)

        def plan(rs, t, q, pairs, cons):
            tr.unit = ("sweep", tr.unit[1] + 1)
            return o_plan(rs, t, q, pairs, cons)

        def paths(rs, t):
            n = o_paths(rs, t)
            tr.guard["n_samples"] = n
            return n

        def guard(rs, t, methods, targets, st):
            tr.guard["ran"] = True
            return o_guard(rs, t, methods, targets, st)

        def short(rs, t, q, sides, methods, targets, st, passes=None):
            rec = {"sides": list(sides), "kind": "guard" if passes is not None else "gap_rule", "calls": [], "fraction": None,
                   "raised": None, "unit": tr.unit, "q_in": np.array(q, float).copy()}
            tr.shortens.append(rec)
            tr.in_shorten = rec
            p2 = None
            if passes is not None:
                def p2(f):
                    tr.disc_count = 0
                    tr.last_gap = None
                    r = passes(f)
                    rec["calls"].append({"f": float(f), "passed": bool(r), "sample": tr.disc_count,
                                         "pair": None if (r or tr.last_gap is None) else tr.last_gap[0],
                                         "gap": None if (r or tr.last_gap is None) else float(tr.last_gap[1])})
                    return r
            else:
                orig_w = R._Tick.worst_value_discover

                def wvd(t2, q2):
                    v = orig_w(t2, q2)
                    w = tr.last_worst
                    rec["calls"].append({"f": None, "passed": bool(v < cfg.RESOLVER["stop_m"]),
                                         "pair": None if w is None else w[0], "shortfall": None if w is None else float(w[1])})
                    return v
                tr._patch(R._Tick, "worst_value_discover", wvd)
            try:
                return o_short(rs, t, q, sides, methods, targets, st, p2) if passes is not None else o_short(rs, t, q, sides, methods, targets, st)
            except R._Hold as h:
                rec["raised"] = h.reason
                raise
            finally:
                rec["fraction"] = None if st.fraction != st.fraction else float(st.fraction)
                tr.in_shorten = None
                if passes is None:
                    tr._unpatch_last()

        def checks(rs, t, q, targets):
            out = o_checks(rs, t, q, targets)
            tr.checks = {"reason": out, "q": np.array(q, float).copy(), "targets": {s: (np.array(v[0]), np.array(v[1])) for s, v in targets.items()},
                         "raw": np.array(t.raw, float).copy()}
            return out

        kin = res.kin
        o_conv, o_jac = kin.convert_directions, kin.jacobian_step

        def event(kind, state, side, qnew, tgt=None):
            i = IDX[side]
            last_arm = np.array(tr.tick.last[i], float)
            d = np.abs(np.asarray(qnew, float) - last_arm)
            lim = res.kin.limits[side]
            over_lim = [(j, float(lim[j, 0] - qnew[j]) if qnew[j] < lim[j, 0] else float(qnew[j] - lim[j, 1]))
                        for j in range(7) if qnew[j] < lim[j, 0] or qnew[j] > lim[j, 1]]
            ev = {"kind": kind, "unit": tr.unit, "side": side, "max_jump": float(d.max()), "joint": int(d.argmax()),
                  "over_guard": [(j, float(d[j])) for j in range(7) if d[j] > cfg.BRANCH_GUARD_RAD], "over_limit": over_lim,
                  "pos_mm": None, "rot_deg": None}
            if tgt is not None:
                q_all = np.array([state.q_left, state.q_right], float)
                q_all[i] = qnew
                p, Rm = tr.kin2.hand_pose(with_arms(state, q_all), side)
                ev["pos_mm"] = 1000.0 * float(np.linalg.norm(p - tgt[0]))
                ev["rot_deg"] = float(np.degrees(rotation_error_rad(tgt[1], Rm)))
            tr.events.append(ev)

        def conv(state, side, u, l, hand_rot):
            q = o_conv(state, side, u, l, hand_rot)
            event("solver", state, side, q)
            return q

        def jac(state, side, tp, tr_):
            q = o_jac(state, side, tp, tr_)
            event("jacobian", state, side, q, (np.asarray(tp, float), np.asarray(tr_, float)))
            return q

        self._patch(R._Tick, "__init__", init)
        self._patch(R._Tick, "gap", gap)
        self._patch(R._Tick, "discover", disc)
        self._patch(R._Tick, "worst", worst)
        self._patch(R.Resolver, "_fix", fix)
        self._patch(R.Resolver, "_shorten", short)
        self._patch(R.Resolver, "_checks", checks)
        self._patch(ks, "plan_sweep", plan)
        self._patch(ks, "_guard", guard)
        self._patch(ks, "_path_samples", paths)
        self.saved.append((kin, "convert_directions", kin.__dict__.get("convert_directions"), "convert_directions" in kin.__dict__))
        self.saved.append((kin, "jacobian_step", kin.__dict__.get("jacobian_step"), "jacobian_step" in kin.__dict__))
        kin.convert_directions, kin.jacobian_step = conv, jac

    def _unpatch_last(self):
        obj, name, old, had = self.saved.pop()
        if had:
            setattr(obj, name, old)
        else:
            delattr(obj, name)

    def uninstall(self):
        while self.saved:
            self._unpatch_last()


# ---------------------------------------------------------------- analysis of one traced tick
def describe_pair(res, t, key, q):
    """Names, categories, gap at q and margin of a pair (using the tick's shapes)."""
    a, b = t.shape(q, key[0]), t.shape(q, key[1])
    rule = t.rules.get(key)
    return {"pair": names(key), "categories": [category_of(a) if a.arm in SIDES else a.part, category_of(b) if b.arm in SIDES else b.part],
            "gap_mm": round(1000.0 * t.gap(q, key), 3), "margin_mm": None if rule is None else round(1000.0 * rule.margin, 3)}


def analyse(res, tr, st, out, last, raw):
    """The trace of one tick -> dict with the hold cause and its numbers. Only reads."""
    t = tr.tick
    rec = {"reason": st.hold_reason, "status": kc._label(st.status)}
    min_gap = cfg.RESOLVER["tunnel_guard_min_gap_m"]
    start_below = sorted(k for k in t.keys if t.g0[k] < min_gap)
    rec["start_pairs_below_guard_gap"] = [names(k) for k in start_below]
    rec["skip_already_below"] = bool(cfg.RESOLVER["tunnel_guard_skip_already_below"])
    if st.worst_pair is not None:
        rec["worst_pair"] = describe_pair(res, t, tuple(st.worst_pair), last)
        rec["worst_pair"]["gap_at_raw_mm"] = round(1000.0 * t.gap(raw, tuple(st.worst_pair)), 3)
        rec["worst_pair"]["shortfall_mm"] = round(1000.0 * st.worst_shortfall, 3)
    reason = st.hold_reason
    if reason == "SHORTEN_ZERO":
        sh = [s for s in tr.shortens if s["raised"] == "SHORTEN_ZERO"]
        s = sh[-1] if sh else None
        rec["path"] = None if s is None else s["kind"]
        if s is None:
            rec["cause"] = "SHORTEN_ZERO without a recorded shortening call"
            return rec
        calls = s["calls"]
        fails = [c for c in calls if not c["passed"]]
        rec["n_guard_samples"] = tr.guard["n_samples"]
        rec["fraction_found"] = s["fraction"]
        rec["first_failing_try"] = None if not fails else {k: (names(v) if k == "pair" and v else v) for k, v in fails[0].items()}
        rec["tries"] = len(calls)
        fin = fails[-1] if fails else None
        pair = None if fin is None else fin.get("pair")
        rec["final_failing_pair"] = None if pair is None else names(pair)
        if s["kind"] == "guard":
            first_try = next((c for c in calls), None)
            if pair is not None:
                below_at_start = pair in start_below
                rec["final_failing_pair_start_gap_mm"] = round(1000.0 * t.g0[pair], 3)
                rec["cause"] = ("guard: start pose already violates the guard gap for " if below_at_start
                                else "guard: path crosses the guard gap (start pose clear) for ") + names(pair)
                rec["cause_class"] = "guard, start pose already below guard gap" if below_at_start else "guard, path crosses (start clear)"
            else:
                rec["cause"], rec["cause_class"] = "guard: no failing pair recorded", "guard, unknown pair"
        else:
            if pair is not None:
                sf = t.shortfall(last, pair)
                rec["final_failing_pair_shortfall_at_start_mm"] = round(1000.0 * sf, 3)
                already = sf > cfg.RESOLVER["stop_m"]
                rec["cause"] = ("gap-rule shortening: start pose already short of the gap rule for " if already
                                else "gap-rule shortening: fails only after moving, pair ") + names(pair)
                rec["cause_class"] = ("shortening (gap rule), start pose already short" if already else "shortening (gap rule), fails after moving")
            else:
                rec["cause"], rec["cause_class"] = "gap-rule shortening: no failing pair recorded", "shortening (gap rule), unknown pair"
        if s["kind"] == "guard" and s["calls"]:
            rec["halving_log"] = [{"f": c["f"], "passed": c["passed"], "sample": c["sample"], "pair": None if c.get("pair") is None else names(c["pair"])}
                                  for c in calls[:16]]
        return rec
    ck = tr.checks
    if ck is None:
        rec["cause"], rec["cause_class"] = f"{reason} without a recorded final check", f"{reason}, no trace"
        return rec
    q = ck["q"]
    raw = ck["raw"]                                    # the command the checks compared against (the shortened one after a guard shortening)
    changed = [s for s in SIDES if not np.array_equal(q[IDX[s]], raw[IDX[s]])]
    if reason == "BRANCH_GUARD":
        joints = list(cfg.RESOLVER["branch_guard_joints"])
        worst = None
        for s in changed:
            d = np.abs(q[IDX[s]][joints] - last[IDX[s]][joints])
            j = int(d.argmax())
            if worst is None or d[j] > worst[2]:
                worst = (s, joints[j], float(d[j]))
        side, joint, amount = worst
        rec["joint"] = {"arm": side, "joint": joint + 1, "jump_rad": round(amount, 4), "over_by_rad": round(amount - cfg.BRANCH_GUARD_RAD, 4)}
        first = next((e for e in tr.events if e["side"] == side and any(j == joint for j, _ in e["over_guard"])), None)
        rec["first_jump_stage"] = None if first is None else {"stage": first["kind"], "unit": list(first["unit"]),
                                                              "jump_rad": round(dict(first["over_guard"])[joint], 4)}
        rec["events_of_arm"] = [{"stage": e["kind"], "unit": list(e["unit"]), "max_jump": round(e["max_jump"], 4), "joint": e["joint"] + 1}
                                for e in tr.events if e["side"] == side][:12]
        stage = "no solver/jacobian event over the limit (final pose differs)" if first is None else f"{first['kind']} output"
        rec["cause"] = f"joint {joint + 1} of the {side} arm jumped {amount:.2f} rad; first seen at {stage}"
        rec["cause_class"] = f"branch jump at {'none' if first is None else first['kind']} (joint {joint + 1})"
        return rec
    if reason == "JOINT_LIMITS":
        found = None
        for s in changed:
            lim = res.kin.limits[s]
            for j in range(7):
                v = q[IDX[s]][j]
                if v < lim[j, 0] or v > lim[j, 1]:
                    amt = float(lim[j, 0] - v) if v < lim[j, 0] else float(v - lim[j, 1])
                    side_lim = "lower" if v < lim[j, 0] else "upper"
                    if found is None or amt > found[3]:
                        found = (s, j, side_lim, amt)
        if found is None:
            rec["cause"], rec["cause_class"] = "JOINT_LIMITS but no joint over its limit in the final pose", "joint limits, not reproduced"
            return rec
        s, j, side_lim, amt = found
        first = next((e for e in tr.events if e["side"] == s and any(jj == j for jj, _ in e["over_limit"])), None)
        rec["joint"] = {"arm": s, "joint": j + 1, "limit": side_lim, "over_by_rad": round(amt, 5)}
        rec["first_over_limit_stage"] = None if first is None else {"stage": first["kind"], "unit": list(first["unit"])}
        # does the raw command itself sit on this limit?
        lim = res.kin.limits[s]
        rec["raw_joint_at_limit"] = bool(abs(raw[IDX[s]][j] - lim[j, 0]) < 1e-9 or abs(raw[IDX[s]][j] - lim[j, 1]) < 1e-9)
        rec["cause"] = f"joint {j + 1} of the {s} arm {amt * 1000:.1f} mrad past its {side_lim} limit (first at {'?' if first is None else first['kind']})"
        rec["cause_class"] = f"joint {j + 1} {side_lim} limit, {'?' if first is None else first['kind']}"
        return rec
    if reason == "ACCURACY":
        worst = None
        for s in changed:
            tgt = ck["targets"].get(s)
            kind = "push target"
            if tgt is None:
                kind = "raw commanded pose"
                tgt = res.kin.hand_pose(res._state(tr.tick, tr.tick.raw), s)
            pos, rot = res.kin.hand_pose(res._state(tr.tick, q), s)
            ep = 1000.0 * float(np.linalg.norm(pos - tgt[0]))
            er = float(np.degrees(rotation_error_rad(tgt[1], rot)))
            bad = ep > 1000 * cfg.RESOLVER["accuracy_pos_m"] or er > cfg.RESOLVER["accuracy_deg"]
            score = max(ep / (1000 * cfg.RESOLVER["accuracy_pos_m"]), er / cfg.RESOLVER["accuracy_deg"])
            if bad and (worst is None or score > worst[4]):
                worst = (s, kind, ep, er, score)
        if worst is None:
            rec["cause"], rec["cause_class"] = "ACCURACY not reproduced from the final pose", "accuracy, not reproduced"
            return rec
        s, kind, ep, er, _ = worst
        first = next((e for e in tr.events if e["side"] == s and e["kind"] == "jacobian"
                      and ((e["pos_mm"] or 0) > 1000 * cfg.RESOLVER["accuracy_pos_m"] or (e["rot_deg"] or 0) > cfg.RESOLVER["accuracy_deg"])), None)
        which = "position" if ep > 1000 * cfg.RESOLVER["accuracy_pos_m"] and er <= cfg.RESOLVER["accuracy_deg"] else \
                "orientation" if er > cfg.RESOLVER["accuracy_deg"] and ep <= 1000 * cfg.RESOLVER["accuracy_pos_m"] else "both"
        rec["accuracy"] = {"arm": s, "target": kind, "pos_err_mm": round(ep, 3), "rot_err_deg": round(er, 3), "exceeded": which,
                           "first_over_in": None if first is None else {"unit": list(first["unit"]), "pos_mm": round(first["pos_mm"], 3),
                                                                        "rot_deg": round(first["rot_deg"], 3)}}
        rec["cause"] = f"hand error {ep:.1f} mm / {er:.2f} deg against the {kind} ({which})"
        rec["cause_class"] = f"accuracy vs {kind}, {which} exceeded, first over in {'?' if first is None else first['unit'][0] + ' ' + str(first['unit'][1])}"
        return rec
    rec["cause"], rec["cause_class"] = reason, reason
    return rec


def close_pairs(res, last):
    """All non-exempt pairs below CLOSE_LIST_M at the last filtered command (touch rules, never-close list on)."""
    placed = res.shapes.place(with_arms(RobotState(), last))
    out = []
    for i in range(len(placed)):
        for j in range(i + 1, len(placed)):
            a, b = placed[i], placed[j]
            if a.kind == "box" and b.kind == "box":
                continue
            rule = pair_rule(a, b)
            if isinstance(rule, Exempt):
                continue
            g = float(gap_between(a, b)[0])
            if g < CLOSE_LIST_M:
                out.append({"pair": f"{a.id} | {b.id}", "gap_mm": round(1000 * g, 2), "margin_mm": round(1000 * rule.margin, 2),
                            "categories": [category_of(a) if a.arm in SIDES else a.part, category_of(b) if b.arm in SIDES else b.part]})
    return sorted(out, key=lambda r: r["gap_mm"])


# ---------------------------------------------------------------- run
def run(setup, row, n, solver):
    raw = setup.raw[:n]
    out = np.zeros_like(raw)
    status, reasons, times = [], [], np.zeros(n)
    causes, detail = [], {r: [] for r in REASONS}
    seen = {r: 0 for r in REASONS}
    replay_ok = {"checked": 0, "identical": 0}
    with setup.row_settings(row):
        res = setup.make_resolver(row)
        tr = Tracer(res)
        tr.install()
        try:
            last = np.array(setup.q0, float)
            base = RobotState()
            for k in range(n):
                tr.reset()
                active_before = set(res._active)
                t0 = time.perf_counter()
                o, st = res.step(raw[k].copy(), last.copy(), base)
                times[k] = time.perf_counter() - t0
                o = np.asarray(o, float).reshape(2, 7)
                out[k] = o
                lab, rs = kc._label(st.status), kc._label(getattr(st, "hold_reason", None))
                status.append(lab)
                reasons.append(rs)
                if lab == "HOLD":
                    a = analyse(res, tr, st, o, last, raw[k])
                    a["tick"], a["mode"] = k, cfg.RESOLVER["mode"]
                    causes.append(a)
                    if rs in seen and seen[rs] < EXAMPLES_PER_REASON:
                        seen[rs] += 1
                        res2 = type(res)(res.shapes, fx.DT, solver)
                        res2._active = set(active_before)
                        tr2 = Tracer(res2)
                        tr.uninstall()
                        tr2.install()
                        try:
                            o2, st2 = res2.step(raw[k].copy(), last.copy(), base)
                            a2 = analyse(res2, tr2, st2, np.asarray(o2, float).reshape(2, 7), last, raw[k])
                        finally:
                            tr2.uninstall()
                            tr.install()
                        same = bool(np.array_equal(np.asarray(o2, float).reshape(2, 7), o)) and st2.hold_reason == st.hold_reason
                        replay_ok["checked"] += 1
                        replay_ok["identical"] += int(same)
                        if not same:
                            raise RuntimeError(f"replay of tick {k} differs from the run (row {row})")
                        a2.update({"tick": k, "mode": cfg.RESOLVER["mode"], "close_pairs_at_last": close_pairs(res, last),
                                   "last_filtered": [RAD_JOINT(last[0]), RAD_JOINT(last[1])], "raw": [RAD_JOINT(raw[k][0]), RAD_JOINT(raw[k][1])]})
                        detail[rs].append(a2)
                last = o.copy()
                if k % 100 == 0:
                    print(f"row {row} tick {k}/{n}  holds so far {sum(1 for s in status if s == 'HOLD')}", flush=True)
        finally:
            tr.uninstall()
    return out, status, reasons, times, causes, detail, replay_ok


def lag_test(setup, row, out, status, kong_metric):
    kin = setup.kin
    raw = setup.raw[:len(out)]
    series, _ = kc.gap_series(raw, kong_metric, setup.ours)
    mask = (series["kong"] if ROW_MEASURE[row] == "kong" else series["ours_ns"]) > kc.NO_CONTACT_GAP_M
    pos, _ = kc.hand_error_series(kin, raw, out)
    err = pos.max(axis=1)
    hold = np.array([s == "HOLD" for s in status])
    near_before = np.zeros(len(out), bool)      # a hold within the previous 20 ticks (not the tick itself)
    for k in range(len(out)):
        near_before[k] = hold[max(0, k - LAG_WINDOW_TICKS):k].any()
    near_incl = near_before | hold
    big = mask & (err > LAG_ERROR_MM)
    return {"measure": "Kong radii gap" if ROW_MEASURE[row] == "kong" else "our gap, shoulder-mount pair left out",
            "no_contact_ticks": int(mask.sum()), "no_contact_ticks_with_error_over_10mm": int(big.sum()),
            "of_those_within_20_ticks_after_a_hold_(hold_before_tick)": int((big & near_before).sum()),
            "of_those_within_20_ticks_incl_a_hold_on_the_tick_itself": int((big & near_incl).sum()),
            "share_before": None if not big.sum() else round(float((big & near_before).sum() / big.sum()), 4),
            "share_incl": None if not big.sum() else round(float((big & near_incl).sum() / big.sum()), 4),
            "context_share_of_all_no_contact_ticks_within_20_after_hold": None if not mask.sum() else round(float((mask & near_before).sum() / mask.sum()), 4),
            "no_contact_ticks_that_are_themselves_holds": int((mask & hold).sum()),
            "tick_list_big_error_no_contact": [int(i) for i in np.flatnonzero(big)][:200]}


ROW_MEASURE = {4: "ours", 5: "kong", 6: "ours"}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--row", type=int, required=True, choices=(4, 5, 6))
    ap.add_argument("--ticks", type=int, default=None, help="first N ticks only (default: the full stream)")
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)
    t0 = time.time()
    setup = dc.Setup(full_stream=True)
    n = setup.n if args.ticks is None else min(args.ticks, setup.n)
    solver = fx.solver()
    out, status, reasons, times, causes, detail, replay_ok = run(setup, args.row, n, solver)
    kong_metric = kc.KongShapes(setup.model, setup.path, setup.tool_local) if ROW_MEASURE[args.row] == "kong" else None
    lag = lag_test(setup, args.row, out, status, kong_metric)
    reason_counts, cause_counts = {}, {}
    for a in causes:
        reason_counts[a["reason"]] = reason_counts.get(a["reason"], 0) + 1
        key = f"{a['reason']}: {a.get('cause_class', a.get('cause'))}"
        cause_counts[key] = cause_counts.get(key, 0) + 1
    doc = {"row": args.row, "name": dc.ROWS[args.row][0], "ticks": n, "hold_ticks": len(causes), "hold_reason_counts": reason_counts,
           "cause_counts": dict(sorted(cause_counts.items(), key=lambda kv: -kv[1])), "replay_check": replay_ok,
           "examples": detail, "lag_test": lag, "time_ms_mean": round(1000 * float(times.mean()), 3),
           "all_hold_causes_short": [{"tick": a["tick"], "reason": a["reason"], "cause": a.get("cause")} for a in causes],
           "config": {"mode": dc.ROWS[args.row][1], "gap_rule": dc.ROWS[args.row][2], "tunnel_guard_min_gap_m": cfg.RESOLVER["tunnel_guard_min_gap_m"],
                      "tunnel_guard_skip_already_below": cfg.RESOLVER["tunnel_guard_skip_already_below"]},
           "runtime_s": round(time.time() - t0, 1)}
    print(json.dumps({k: doc[k] for k in ("hold_ticks", "hold_reason_counts", "cause_counts", "replay_check", "lag_test")}, indent=1)[:6000], flush=True)
    path = Path(args.out) if args.out else dc.ROOT / "data" / "safety_filter" / f"hold_causes_row{args.row}_{dt.datetime.now():%Y%m%d_%H%M%S}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(doc, indent=1, default=str))
    json.loads(tmp.read_text())
    os.replace(tmp, path)
    np.savez_compressed(path.with_suffix(".npz"), out=out, status=np.array(status), reasons=np.array(reasons))
    print(f"wrote {path} (runtime {doc['runtime_s']} s)", flush=True)


if __name__ == "__main__":
    main()
