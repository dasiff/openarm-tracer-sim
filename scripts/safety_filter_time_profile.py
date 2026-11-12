#!/usr/bin/env python
"""Stage 2c steps 2 and 3: where the time goes in the safety-filter resolver (measurement only; no algorithm change).

Step 2, time profile. Rows 4, 5 and 6 of the Stage 2b comparison run on the first MAX_TICKS ticks of the rolling punch stream
(scripts/safety_filter_diag_common.py). Timing wrappers are put around the existing functions FROM OUTSIDE: the module attributes
that resolver.py and kong_style.py look up are replaced inside this process and put back in a finally block. Per tick and per
stage the call count and the wall time (time.perf_counter) are kept. Stages nest (the gap checks happen inside the jump-through
guard and inside step shortening), so the shares are inclusive and do not add to 100 percent.

Identity check. Each row also runs with the timers off on the same ticks (the honest speed). Outputs, status, hold reason,
fraction, rounds and methods must be identical on every tick; otherwise the script stops with a RuntimeError naming the row and
the first differing tick.

Step 3, advisory only. For each pair type: the pairs of the after-never-close set (dc.pair_stages) and their smallest gap over
the ticks, at the filtered command and at the raw command. One stream only: not a proof, and must not be used to exclude any pair.

Usage (venv active, repo root):
    bash experiments/run_capped.sh data/logs/<stamp>_time_profile.log python scripts/safety_filter_time_profile.py [--ticks N] [--rows 4,5,6] [--out PATH]
"""

import argparse
import contextlib
import datetime as dt
import functools
import json
import math
import os
import sys
import time
from collections import Counter, namedtuple
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import safety_filter_diag_common as dc  # noqa: E402

from src.safety_filter import arm_conversion as AC  # noqa: E402
from src.safety_filter import gap_rule as GR  # noqa: E402
from src.safety_filter import kong_style as KS  # noqa: E402
from src.safety_filter import push_direction as PD  # noqa: E402
from src.safety_filter import resolver as RS  # noqa: E402
from src.safety_filter import touch as TO  # noqa: E402
from src.safety_filter.arm_conversion import with_arms  # noqa: E402
from src.safety_filter.shapes import RobotState  # noqa: E402

MAX_TICKS = dc.MAX_TICKS          # count: ticks per row (400: the first ticks of the rolling punch stream, as in the other Stage 2c scripts)
ROWS_DEFAULT = "4,5,6"            # the Stage 2b rows this profile covers
PROGRESS_EVERY = 50               # ticks between progress lines (flushed)
BROAD = "broad_phase"             # stage: touch.evaluate_pairs, called by the resolver
PAIR_TOTAL = "pair_distance"      # stage: all resolver gap_between calls (summed from the per-type stages)
PAIR_PREFIX = "pair_distance:"    # per-type stage name = PAIR_PREFIX + pair type
ITEM_KIND = {BROAD: "pairs", "path_samples": "samples", "shorten": "evals"}   # what the items of these stages count
ADVISORY_SKIP_M = 0.25            # m: a pair whose boxes are further apart than this at a pose is not measured there; it counts as "gap above 0.25 m" for that tick (the boxes never come closer than the true gap, so the skip cannot hide a close pair)
ADV_HI_M = 0.20                   # m: advisory level "stays above 20 cm"
ADV_LO_M = 0.10                   # m: advisory level "stays above 10 cm"
WHERES = ("filtered", "raw")      # advisory poses: the filtered command of the tick, and the raw command

TickOut = namedtuple("TickOut", "out status hold_reason fraction rounds methods")

STAGE_ROWS = (
    [(BROAD, "broad phase: evaluate_pairs (items = pairs)"),
     (PAIR_TOTAL, "pair distance checks, all types (gap_between)")]
    + [(PAIR_PREFIX + ty, "  " + ty) for ty in dc.TYPES]
    + [("gap_rule", "gap rule: gap_rule.shortfall"),
       ("push_direction", "push direction: pair_push"),
       ("pair_push_gap", "  gap_between inside pair_push"),
       ("guard", "jump-through guard: _guard"),
       ("path_samples", "  guard samples: _path_samples (items)"),
       ("sweeps", "sweeps: _sweep (calls = sweeps)"),
       ("restore_lengths", "link length restore: restore_lengths"),
       ("kong_solver", "Kong solver: solve_arm"),
       ("jacobian", "Jacobian step: ArmKinematics.jacobian_step"),
       ("checks", "four final checks: Resolver._checks"),
       ("shorten", "step shortening: _shorten (items = evals)"),
       ("hand_slide", "layered hand slide: _hand_slide"),
       ("elbow_swing", "layered elbow swing: _elbow_swing")]
)

STAGE_NOTES = [
    "Shares are inclusive: stages nest (gap checks inside the guard and inside shortening; pass tests inside shortening).",
    "pair_distance: calls of resolver.gap_between (cache misses of the _Tick gap cache). One call = one pair check. Type from dc.pair_type with no world objects (the comparison world has none).",
    "broad_phase: calls of resolver.evaluate_pairs; items = pairs in its batch. Those gaps are batched inside touch.evaluate_pairs, so they are not gap_between calls.",
    "pair_push_gap: gap_between calls made inside push_direction.pair_push (kong_style and layered). A pair check, not in pair_distance; its time is inside push_direction.",
    "restore_lengths: one call per pushed arm. sweeps: one _sweep call per sweep.",
    "path_samples: kong_style._path_samples; items = its return value (the samples of the jump-through guard).",
    "shorten: Resolver._shorten; items = pass-test evaluations (full step first, then halvings). Default test: _Tick.worst_value_discover calls made while _shorten runs.",
    "Timers: time.perf_counter around each call. The timers-off run gives the honest speed; the timers add their own overhead.",
]
NOT_AVAILABLE = [
    ["recording (src/safety_filter/recording.py)",
     "not available (not called by the tick): resolver.step and kong_style.step never write a recording, and the stream loop does not call it"],
    ["_Tick bookkeeping (discover, closing, worst, add_known), keypoints, state helpers, _update_active",
     "not timed: not in the stage list; their time is in the whole-tick figure only"],
]


# ---------------------------------------------------------------- timing wrappers (installed from outside, removed in finally)
class Profiler:
    """Timing wrappers for the stages above. install() replaces the functions; uninstall() puts every original back.
    While a tick runs, the wrappers add (calls, seconds, items) to self.cur; end_tick() files the tick in self.ticks."""

    def __init__(self):
        self.cur = None           # the tick in progress: stage -> [calls, seconds, items]
        self.ticks = []           # one dict per tick, same shape as cur
        self.call_items = {}      # stage -> items of every call (only for the stages in ITEM_KIND)
        self.shorten_frames = []  # one frame per _shorten call in progress: [pass-test evaluations, default test in use]
        self._saved = []          # (owner, name, original) for uninstall

    def begin_tick(self):
        self.cur = {}

    def end_tick(self):
        self.ticks.append(self.cur)
        self.cur = None

    def _add(self, stage, secs, items=None):
        if self.cur is None:
            return
        c = self.cur.setdefault(stage, [0, 0.0, 0])
        c[0] += 1
        c[1] += secs
        if items is not None:
            c[2] += items
            if stage in ITEM_KIND:
                self.call_items.setdefault(stage, []).append(items)

    def _timed(self, stage, fn, items_of=None):
        prof = self

        @functools.wraps(fn)
        def timed(*args, **kwargs):
            t0 = time.perf_counter()
            items = None
            try:
                out = fn(*args, **kwargs)
                if items_of is not None:
                    items = items_of(out)
                return out
            finally:
                prof._add(stage, time.perf_counter() - t0, items)

        return timed

    def _gap_wrapper(self, orig):
        prof = self

        def gap_between(a, b):
            stage = PAIR_PREFIX + dc.pair_type(a, b, {})     # classified before the timer starts
            t0 = time.perf_counter()
            try:
                return orig(a, b)
            finally:
                prof._add(stage, time.perf_counter() - t0)

        return gap_between

    def _shorten_wrapper(self, orig):
        prof = self

        def shorten(res, t, q, sides, methods, targets, st, passes=None):
            frame = [0, passes is None]                   # [pass-test evaluations, default test in use]
            kw = {}
            if passes is not None:                        # the jump-through guard's own pass test
                inner = passes

                def counted(f):
                    frame[0] += 1
                    return inner(f)

                kw["passes"] = counted
            prof.shorten_frames.append(frame)
            t0 = time.perf_counter()
            try:
                return orig(res, t, q, sides, methods, targets, st, **kw)
            finally:
                prof.shorten_frames.pop()
                prof._add("shorten", time.perf_counter() - t0, frame[0])

        return shorten

    def _wvd_wrapper(self, orig):
        prof = self

        def worst_value_discover(t, q):                   # the default pass test calls this once per evaluation
            if prof.shorten_frames and prof.shorten_frames[-1][1]:
                prof.shorten_frames[-1][0] += 1
            return orig(t, q)

        return worst_value_discover

    def _patch(self, owner, name, make):
        orig = getattr(owner, name)
        self._saved.append((owner, name, orig))
        setattr(owner, name, make(orig))

    def install(self):
        self._patch(RS, "gap_between", self._gap_wrapper)                   # resolver's own name (_Tick.gap)
        self._patch(RS, "evaluate_pairs", lambda f: self._timed(BROAD, f, items_of=len))
        self._patch(RS, "pair_push", lambda f: self._timed("push_direction", f))
        self._patch(KS, "pair_push", lambda f: self._timed("push_direction", f))
        self._patch(PD, "gap_between", lambda f: self._timed("pair_push_gap", f))
        self._patch(GR, "shortfall", lambda f: self._timed("gap_rule", f))
        self._patch(KS, "_guard", lambda f: self._timed("guard", f))
        self._patch(KS, "_path_samples", lambda f: self._timed("path_samples", f, items_of=lambda r: r))
        self._patch(KS, "_sweep", lambda f: self._timed("sweeps", f))
        self._patch(KS, "restore_lengths", lambda f: self._timed("restore_lengths", f))
        self._patch(KS, "solve_arm", lambda f: self._timed("kong_solver", f))
        self._patch(AC.ArmKinematics, "jacobian_step", lambda f: self._timed("jacobian", f))
        self._patch(RS.Resolver, "_checks", lambda f: self._timed("checks", f))
        self._patch(RS.Resolver, "_shorten", self._shorten_wrapper)
        self._patch(RS._Tick, "worst_value_discover", self._wvd_wrapper)
        self._patch(RS.Resolver, "_hand_slide", lambda f: self._timed("hand_slide", f))
        self._patch(RS.Resolver, "_elbow_swing", lambda f: self._timed("elbow_swing", f))

    def uninstall(self):
        for owner, name, orig in reversed(self._saved):
            setattr(owner, name, orig)
        self._saved = []


@contextlib.contextmanager
def installed(prof):
    """The wrappers are in place inside the block and removed in a finally block, also when the block raises."""
    try:
        prof.install()
        yield prof
    finally:
        prof.uninstall()


# ---------------------------------------------------------------- one pass over a row's ticks
def run_row(setup, row, prof):
    """One pass over the row's ticks. prof: a Profiler whose wrappers the caller installed, or None (timers off).
    Returns (list of TickOut, numpy array of the wall time of res.step per tick, s)."""
    n = setup.n
    tag = "timers on" if prof is not None else "timers off"
    outs, tick_s = [], np.zeros(n)
    with setup.row_settings(row):
        res = setup.make_resolver(row)
        last = np.array(setup.q0, float)
        base = RobotState()
        for k in range(n):
            if prof is not None:
                prof.begin_tick()
            t0 = time.perf_counter()
            o, st = res.step(setup.raw[k].copy(), last.copy(), base)
            tick_s[k] = time.perf_counter() - t0
            if prof is not None:
                prof.end_tick()
            out = np.asarray(o, float).reshape(2, 7).copy()
            outs.append(TickOut(out, st.status, st.hold_reason, float(st.fraction), int(st.rounds), tuple(st.methods)))
            last = out.copy()
            if k % PROGRESS_EVERY == 0:
                print(f"row {row} {tag} tick {k}/{n}", flush=True)
    return outs, tick_s


def same_float(a, b):
    return a == b or (math.isnan(a) and math.isnan(b))


def check_identity(name, on, off):
    """Raises RuntimeError at the first tick where the timers-on and timers-off runs differ."""
    if len(on) != len(off):
        raise RuntimeError(f"identity check failed, row {name}: {len(on)} ticks with timers on, {len(off)} with timers off")
    for k, (x, y) in enumerate(zip(on, off)):
        same = (np.array_equal(x.out, y.out) and x.status == y.status and x.hold_reason == y.hold_reason
                and same_float(x.fraction, y.fraction) and x.rounds == y.rounds and x.methods == y.methods)
        if not same:
            raise RuntimeError(f"identity check failed, row {name}: timers-on and timers-off runs differ at tick {k} "
                               f"(on: {x.status} {x.hold_reason!r} {x.methods}; off: {y.status} {y.hold_reason!r} {y.methods}; "
                               f"outputs equal: {np.array_equal(x.out, y.out)})")


# ---------------------------------------------------------------- summaries
def per_tick_arrays(ticks, key):
    """(calls, seconds, items) per tick for a stage. PAIR_TOTAL sums the per-type stages."""
    n = len(ticks)
    calls, secs, items = np.zeros(n), np.zeros(n), np.zeros(n)
    for k, rec in enumerate(ticks):
        for name, (c, s, it) in rec.items():
            if name == key or (key == PAIR_TOTAL and name.startswith(PAIR_PREFIX)):
                calls[k] += c
                secs[k] += s
                items[k] += it
    return calls, secs, items


def stage_stats(prof, tick_s, key):
    calls, secs, items = per_tick_arrays(prof.ticks, key)
    tot = float(tick_s.sum())
    d = {"calls": int(calls.sum()), "ms_per_tick_mean": 1e3 * float(secs.mean()), "ms_per_tick_max": 1e3 * float(secs.max()),
         "share_of_tick_time": float(secs.sum()) / tot, "calls_per_tick_mean": float(calls.mean()),
         "calls_per_tick_max": int(calls.max())}
    if key in ITEM_KIND:
        arr = prof.call_items.get(key, [])
        d.update(items_kind=ITEM_KIND[key], items_total=int(items.sum()), items_per_tick_mean=float(items.mean()),
                 items_per_tick_max=int(items.max()), items_per_call_mean=float(np.mean(arr)) if arr else None,
                 items_per_call_max=int(max(arr)) if arr else None)
    if key == BROAD and items.sum() > 0:
        d["us_per_unit"] = 1e6 * float(secs.sum()) / float(items.sum())          # per pair in the batch
    elif (key == PAIR_TOTAL or key.startswith(PAIR_PREFIX)) and calls.sum() > 0:
        d["us_per_unit"] = 1e6 * float(secs.sum()) / float(calls.sum())          # per pair check
    return d


def summarise(prof, tick_s):
    stages, per_tick = {}, {"tick_ms": [round(1e3 * float(x), 4) for x in tick_s]}
    for key, _ in STAGE_ROWS:
        stages[key] = stage_stats(prof, tick_s, key)
        calls, secs, _items = per_tick_arrays(prof.ticks, key)
        per_tick[key] = {"calls": calls.astype(int).tolist(), "ms": (1e3 * secs).round(4).tolist()}
    return {"tick_total_s": float(tick_s.sum()), "tick_ms_mean": 1e3 * float(tick_s.mean()),
            "tick_ms_max": 1e3 * float(tick_s.max()), "stages": stages, "per_tick": per_tick}


def print_summary(name, n, s):
    over = 100.0 * (s["tick_total_s"] / max(s["off_total_s"], 1e-12) - 1.0)
    print(f"-- {name}: {n} ticks; timers on {s['tick_total_s']:.3f} s, timers off {s['off_total_s']:.3f} s "
          f"(timers add {over:+.1f} %)", flush=True)
    print(f"   status {s['status_counts']}  hold reasons {s['hold_counts']}")
    print(f"   {'stage':<46}{'calls':>8}{'ms/tick':>10}{'max ms':>9}{'share':>8}{'calls/t':>9}{'max':>5}{'items/t':>9}{'max':>5}{'us/unit':>9}")
    print(f"   {'whole tick (res.step wall time)':<46}{n:>8d}{s['tick_ms_mean']:>10.3f}{s['tick_ms_max']:>9.3f}{'100%':>8}")
    for key, label in STAGE_ROWS:
        d = s["stages"][key]
        items = f"{d['items_per_tick_mean']:>9.2f}{d['items_per_tick_max']:>5d}" if "items_kind" in d else " " * 14
        us = f"{d['us_per_unit']:>9.2f}" if "us_per_unit" in d else ""
        print(f"   {label:<46}{d['calls']:>8d}{d['ms_per_tick_mean']:>10.3f}{d['ms_per_tick_max']:>9.3f}"
              f"{d['share_of_tick_time']:>8.1%}{d['calls_per_tick_mean']:>9.2f}{d['calls_per_tick_max']:>5d}{items}{us}",
              flush=True)


# ---------------------------------------------------------------- step 3: advisory pair gaps
def advisory_label(n):
    return (f"ADVISORY ONLY: from one stream (the rolling punch, first {n} ticks), not a proof, "
            f"must not be used to exclude any pair")


def never_close_set(placed):
    """(i, j, type) of the pairs of dc.pair_stages' after-never-close set, i < j: not box-box, not exempt by the touch rules,
    and not exempt by the never-close list (the same tests as dc.pair_stages)."""
    out = []
    for i in range(len(placed)):
        for j in range(i + 1, len(placed)):
            a, b = placed[i], placed[j]
            if a.kind == "box" and b.kind == "box":
                continue
            if isinstance(TO.pair_rule(a, b, use_never_close=False), TO.Exempt):
                continue
            if isinstance(TO.pair_rule(a, b, use_never_close=True), TO.Exempt):
                continue
            out.append((i, j, dc.pair_type(a, b, {})))
    return out


def check_against_pair_stages(placed, pairs, name, where, k):
    counts = dc.pair_stages(placed, [])[0]
    for ty in dc.TYPES:
        mine = sum(1 for _, _, t in pairs if t == ty)
        if mine != counts[ty]["after_never_close"]:
            raise RuntimeError(f"row {name} {where} tick {k}: {ty} after-never-close pairs {mine} "
                               f"!= dc.pair_stages {counts[ty]['after_never_close']}")


def advisory_stats(mins):
    """mins: {sorted ids: [smallest gap over the ticks (m; inf = skipped by the boxes on every tick), pair type]}.
    Per type and in total: pairs, share whose smallest gap is above ADV_HI_M and ADV_LO_M, smallest finite smallest gap."""
    def blank():
        return {"pairs": 0, "above_20cm": 0, "above_10cm": 0, "smallest_min_m": None}

    per_type = {ty: blank() for ty in dc.TYPES}
    total = blank()
    for g, ty in mins.values():
        for d in (per_type[ty], total):
            d["pairs"] += 1
            d["above_20cm"] += int(g > ADV_HI_M)
            d["above_10cm"] += int(g > ADV_LO_M)
            if math.isfinite(g) and (d["smallest_min_m"] is None or g < d["smallest_min_m"]):
                d["smallest_min_m"] = g
    for d in list(per_type.values()) + [total]:
        for lvl in ("20cm", "10cm"):
            d[f"share_above_{lvl}"] = d[f"above_{lvl}"] / d["pairs"] if d["pairs"] else None
    return per_type, total


def advisory_row(setup, row, outs, name):
    """Step 3 for one row. Per pose (filtered = the timers-off output of each tick; raw = the raw command): the pairs of the
    after-never-close set and their smallest gap over the ticks, from touch.gap_between (timers are off here)."""
    n = setup.n
    result = {}
    with setup.row_settings(row):
        res = setup.make_resolver(row)
        for where in WHERES:
            mins, measured, pair_ticks = {}, 0, 0
            for k in range(n):
                q = outs[k].out if where == "filtered" else setup.raw[k]
                placed = list(res.shapes.place(with_arms(RobotState(), np.asarray(q, float))))
                pairs = never_close_set(placed)
                if k % PROGRESS_EVERY == 0:
                    check_against_pair_stages(placed, pairs, name, where, k)
                    print(f"row {row} advisory {where} tick {k}/{n}", flush=True)
                lo, hi = TO._aabb_arrays(placed)
                for i, j, ty in pairs:
                    key = tuple(sorted((placed[i].id, placed[j].id)))
                    pair_ticks += 1
                    sep = float(np.max(np.maximum(lo[i] - hi[j], lo[j] - hi[i])))
                    if sep > ADVISORY_SKIP_M:
                        g = math.inf                              # boxes apart by more than the skip distance: gap above it
                    else:
                        g = float(TO.gap_between(placed[i], placed[j])[0])
                        measured += 1
                    rec = mins.setdefault(key, [math.inf, ty])
                    rec[0] = min(rec[0], g)
            per_type, total = advisory_stats(mins)
            pairs_out = [[a, b, ty, g if math.isfinite(g) else None] for (a, b), (g, ty) in sorted(mins.items())]
            result[where] = {"pair_ticks": pair_ticks, "gap_measured": measured, "per_type": per_type, "total": total,
                             "pairs": pairs_out}
    return result


def _pct(x):
    return "-" if x is None else f"{x:.1%}"


def print_advisory(n, adv):
    print(advisory_label(n), flush=True)
    for where in WHERES:
        a = adv[where]
        print(f"   {where} command: gap measured on {a['gap_measured']} of {a['pair_ticks']} pair-ticks "
              f"(the rest: boxes more than {ADVISORY_SKIP_M} m apart, counted as gap above {ADVISORY_SKIP_M} m)")
        print(f"     {'pair type':<22}{'pairs':>7}{'min > 20 cm':>13}{'min > 10 cm':>13}{'smallest min, mm':>18}")
        for ty, d in list(a["per_type"].items()) + [("all types", a["total"])]:
            small = "-" if d["smallest_min_m"] is None else f"{1e3 * d['smallest_min_m']:.1f}"
            print(f"     {ty:<22}{d['pairs']:>7d}{_pct(d['share_above_20cm']):>13}{_pct(d['share_above_10cm']):>13}{small:>18}",
                  flush=True)


# ---------------------------------------------------------------- output and main
def check_row_names(rows):
    bad = [r for r in rows if r not in dc.ROWS]
    if bad:
        raise SystemExit(f"unknown rows {bad}; the Stage 2b rows are {sorted(dc.ROWS)}")


def write_json_atomic(path, doc):
    """Temp file in the same folder, checked with json.loads, then renamed over the target (an interrupted run leaves the old file)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(doc, indent=1, default=lambda o: o.tolist()))
    json.loads(tmp.read_text())
    os.replace(tmp, path)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Resolver time profile (Stage 2c step 2) and advisory pair gaps (step 3). Measurement only.")
    ap.add_argument("--ticks", type=int, default=MAX_TICKS, help="ticks per row (default %(default)s)")
    ap.add_argument("--rows", default=ROWS_DEFAULT, help="Stage 2b rows, comma-separated (default %(default)s)")
    ap.add_argument("--out", default=None, help="JSON output path (default data/safety_filter/time_profile_<stamp>.json)")
    args = ap.parse_args(argv)
    rows = [int(r) for r in args.rows.split(",")]
    check_row_names(rows)
    if args.ticks < 1:
        ap.error("--ticks must be at least 1")
    t_start = time.time()
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    setup = dc.Setup(args.ticks)
    n = setup.n
    print(f"time profile: rows {rows}, {n} ticks each. Shares are inclusive (stages nest) and do not add to 100 percent.",
          flush=True)
    doc = {"stamp": stamp, "ticks": n, "advisory_label": advisory_label(n), "stage_notes": STAGE_NOTES,
           "not_available": NOT_AVAILABLE, "rows": {}}
    for row in rows:
        name = dc.ROWS[row][0]
        print(f"\n== row {row} ({name})", flush=True)
        prof = Profiler()
        with installed(prof):
            on_outs, on_s = run_row(setup, row, prof)
        off_outs, off_s = run_row(setup, row, None)
        check_identity(name, on_outs, off_outs)
        summary = summarise(prof, on_s)
        summary.update(off_total_s=float(off_s.sum()), status_counts=dict(Counter(o.status for o in on_outs)),
                       hold_counts=dict(Counter(o.hold_reason for o in on_outs if o.hold_reason)),
                       identity=(f"timers-on and timers-off identical on all {n} ticks "
                                 f"(outputs, status, hold reason, fraction, rounds, methods)"))
        print_summary(name, n, summary)
        print(f"   identity: {summary['identity']}", flush=True)
        adv = advisory_row(setup, row, off_outs, name)
        print_advisory(n, adv)
        doc["rows"][name] = {"time_profile": summary, "advisory": adv}
    print("\nnot available / not timed:")
    for what, why in NOT_AVAILABLE:
        print(f"  {what}: {why}")
    out = Path(args.out) if args.out else dc.ROOT / "data" / "safety_filter" / f"time_profile_{stamp}.json"
    write_json_atomic(out, doc)
    print(f"wrote {out} (runtime {time.time() - t_start:.1f} s)", flush=True)


if __name__ == "__main__":
    main()
