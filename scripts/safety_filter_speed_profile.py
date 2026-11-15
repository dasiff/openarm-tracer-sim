#!/usr/bin/env python
"""Where the safety filter's time goes, on the golden recordings (Stage 5 speed, steps 1, 3 and 4; measurement only).

Each recording is replayed in order with a fresh resolver of the working tree (the gate's part A replay), with timers put around the
filter's functions FROM OUTSIDE (attributes replaced in this process, put back at the end). Per stage: calls per tick, inclusive time
(with everything it calls) and own time (without the timed stages it calls), as milliseconds per tick and as a share of the tick,
separately for pass-through ticks and for ticks where the filter acted. Own times add up to the tick; inclusive times nest.
Pair distance checks are counted by shape kind and by pair type. The timers cost time themselves (about a microsecond per timed call),
so the honest time per call is the one of the gate's part A run, not this one; the shares are what this script is for.

A stage whose function no longer exists (after a speed change) is skipped and listed.

Usage: python scripts/safety_filter_speed_profile.py --label <name> [--only punch_original_kong_style,...] [--ticks N]
Output: data/safety_filter/speed_profile/<label>.json and a printed table.
"""

import argparse
import importlib
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import safety_filter_speed_common as sc  # noqa: E402
import safety_filter_speed_gate as gate  # noqa: E402

ROOT = sc.ROOT
OUT_DIR = ROOT / "data" / "safety_filter" / "speed_profile"
DEFAULT_TICKS = {"punch_original": 400, "punch_reactive": 400, "shadow_pp": 0, "cat": 0}   # ticks replayed per stream (0 = all); the punch streams repeat every 150 ticks

# (module, attribute path, stage label). A dotted attribute path is Class.method. The same function reached through another
# module's imported name is listed again with the same label.
TARGETS = [
    ("src.safety_filter.shapes", "RobotShapes.set_state", "forward kinematics (set_state: joints in, mj_kinematics)"),
    ("src.safety_filter.shapes", "RobotShapes.place", "shape placement (place: all shapes of one pose)"),
    ("src.safety_filter.arm_conversion", "with_arms", "per-pose setup (with_arms: state object)"),
    ("src.safety_filter.resolver", "with_arms", "per-pose setup (with_arms: state object)"),
    ("src.safety_filter.resolver", "_Tick.__init__", "tick start-up (world objects to shapes)"),
    ("src.safety_filter.resolver", "_Tick.pose", "pose lookup (cache, placement, held shapes)"),
    ("src.safety_filter.resolver", "_Tick.discover", "pair listing (discover)"),
    ("src.safety_filter.resolver", "evaluate_pairs", "broad phase (evaluate_pairs)"),
    ("src.safety_filter.touch", "as_placed", "world object conversion (as_placed)"),
    ("src.safety_filter.resolver", "as_placed", "world object conversion (as_placed)"),
    ("src.safety_filter.touch", "_aabb_arrays", "broad phase: shape boxes (_aabb_arrays)"),
    ("src.safety_filter.touch", "pair_rule", "broad phase: touch rule per pair (pair_rule)"),
    ("src.safety_filter.resolver", "pair_rule", "broad phase: touch rule per pair (pair_rule)"),
    ("src.safety_filter.touch", "capsule_capsule_batch", "broad phase: batched capsule-capsule gaps"),
    ("src.safety_filter.touch", "capsule_box_batch", "broad phase: batched capsule-box gaps"),
    ("src.safety_filter.resolver", "gap_between", "pair distance check (gap_between, one pair)"),
    ("src.safety_filter.push_direction", "gap_between", "pair distance check (gap_between, one pair)"),
    ("src.safety_filter.resolver", "_Tick.gap", "gap lookup (cache)"),
    ("src.safety_filter.resolver", "_Tick.closing", "closing-arm decision"),
    ("src.safety_filter.resolver", "_Tick.worst", "worst pair search"),
    ("src.safety_filter.kong_style", "list_pairs", "pair list of a sweep (list_pairs)"),
    ("src.safety_filter.kong_style", "_guard", "jump-through guard (_guard)"),
    ("src.safety_filter.kong_style", "_path_samples", "guard: number of samples (_path_samples)"),
    ("src.safety_filter.resolver", "Resolver._shorten", "step shortening (_shorten)"),
    ("src.safety_filter.kong_style", "_sweep", "sweeps (_sweep)"),
    ("src.safety_filter.kong_style", "_push_pair", "push of one pair (_push_pair)"),
    ("src.safety_filter.arm_conversion", "ArmKinematics.convert_directions", "Kong solver (convert_directions)"),
    ("src.safety_filter.arm_conversion", "ArmKinematics.jacobian_step", "Jacobian step"),
    ("src.safety_filter.arm_conversion", "ArmKinematics.hand_pose", "hand pose (forward kinematics)"),
    ("src.safety_filter.arm_conversion", "ArmKinematics.sew", "shoulder-elbow-wrist points (forward kinematics)"),
    ("src.safety_filter.resolver", "Resolver._checks", "four final checks (_checks)"),
    ("src.safety_filter.resolver", "Resolver._update_active", "active set update (_update_active)"),
    ("src.safety_filter.resolver", "Resolver._hand_slide", "layered hand slide"),
    ("src.safety_filter.resolver", "Resolver._elbow_swing", "layered elbow swing"),
]
TOTAL = "whole tick (Resolver.step)"
PAIR_KINDS = ("capsule-capsule", "capsule-box")
PAIR_TYPES = ("arm_vs_arm", "arm_vs_body", "arm_vs_world", "held_object")


class Timers:
    def __init__(self):
        self.stack = []
        self.tick = defaultdict(lambda: [0, 0.0, 0.0])     # label -> [calls, inclusive s, own s] of the current tick
        self.pairs = defaultdict(int)                      # (kind, type) -> count of the current tick
        self.saved, self.missing = [], []

    def timed(self, fn, label, pair_hook=False):
        stack, tick, pairs = self.stack, self.tick, self.pairs

        def wrapper(*a, **k):
            if pair_hook:
                x, y = a[0], a[1]
                kind = "capsule-capsule" if x.kind == "capsule" and y.kind == "capsule" else "capsule-box"
                if x.part == "held" or y.part == "held":
                    ty = "held_object"
                elif x.arm == "world" or y.arm == "world":
                    ty = "arm_vs_world"
                elif x.arm in sc.SIDES and y.arm in sc.SIDES:
                    ty = "arm_vs_arm"
                else:
                    ty = "arm_vs_body"
                pairs[(kind, ty)] += 1
            frame = [0.0]
            stack.append(frame)
            t0 = time.perf_counter()
            try:
                return fn(*a, **k)
            finally:
                dt = time.perf_counter() - t0
                stack.pop()
                if stack:
                    stack[-1][0] += dt
                rec = tick[label]
                rec[0] += 1
                rec[1] += dt
                rec[2] += dt - frame[0]
        wrapper.__wrapped__ = fn
        return wrapper

    def install(self):
        seen = {}
        for mod_name, path, label in TARGETS:
            try:
                mod = importlib.import_module(mod_name)
                owner = mod
                parts = path.split(".")
                for p in parts[:-1]:
                    owner = getattr(owner, p)
                fn = getattr(owner, parts[-1])
            except (ImportError, AttributeError):
                self.missing.append(f"{mod_name}.{path}")
                continue
            fn = getattr(fn, "__wrapped__", fn)
            key = (id(fn), label)
            if key not in seen:
                seen[key] = self.timed(fn, label, pair_hook=(parts[-1] == "gap_between"))
            self.saved.append((owner, parts[-1], getattr(owner, parts[-1])))
            setattr(owner, parts[-1], seen[key])

    def remove(self):
        for owner, name, old in reversed(self.saved):
            setattr(owner, name, old)
        self.saved = []


def profile(rec, ticks, say=print):
    tm = Timers()
    tm.install()
    n = rec.n if not ticks else min(ticks, rec.n)
    classes = {"pass": defaultdict(lambda: [0, 0.0, 0.0]), "acted": defaultdict(lambda: [0, 0.0, 0.0])}
    pair_counts = {"pass": defaultdict(int), "acted": defaultdict(int)}
    n_class = {"pass": 0, "acted": 0}
    try:
        with sc.resolver_mode(rec.mode):
            res = gate.new_resolver(rec.dt)
            step = tm.timed(res.step, TOTAL)
            for k in range(n):
                tm.tick.clear()
                tm.pairs.clear()
                raw, last, state, world, held = rec.inputs(k)
                _, st = step(raw, last, state, world, held)
                c = "pass" if st.status == "PASS" else "acted"
                n_class[c] += 1
                for label, v in tm.tick.items():
                    agg = classes[c][label]
                    agg[0] += v[0]; agg[1] += v[1]; agg[2] += v[2]
                for key, v in tm.pairs.items():
                    pair_counts[c][key] += v
                if (k + 1) % 500 == 0:
                    say(f"{rec.path.stem}: profiled tick {k + 1}/{n}")
    finally:
        tm.remove()
    out = {"recording": rec.path.name, "ticks": n, "ticks_pass": n_class["pass"], "ticks_acted": n_class["acted"], "missing_stages": tm.missing, "classes": {}}
    for c in ("pass", "acted"):
        m = n_class[c]
        if not m:
            continue
        total = classes[c][TOTAL][1]
        rows = []
        for label, v in classes[c].items():
            rows.append({"stage": label, "calls_per_tick": round(v[0] / m, 2), "incl_ms_per_tick": round(v[1] / m * 1e3, 4), "incl_share": round(v[1] / total, 4),
                         "own_ms_per_tick": round(v[2] / m * 1e3, 4), "own_share": round(v[2] / total, 4)})
        rows.sort(key=lambda r: -r["own_ms_per_tick"])
        out["classes"][c] = {"ticks": m, "tick_ms_mean_with_timers": round(total / m * 1e3, 4), "stages": rows,
                             "pair_checks_per_tick": {f"{kd} {ty}": round(pair_counts[c].get((kd, ty), 0) / m, 2) for kd in PAIR_KINDS for ty in PAIR_TYPES
                                                      if pair_counts[c].get((kd, ty), 0)}}
    return out


def print_table(r):
    for c, d in r["classes"].items():
        print(f"\n== {r['recording']}  {c} ticks: {d['ticks']}  mean {d['tick_ms_mean_with_timers']} ms per tick (with timers)")
        print(f"  {'stage':58s} {'calls':>9s} {'incl ms':>10s} {'incl %':>7s} {'own ms':>10s} {'own %':>7s}")
        for s in d["stages"]:
            print(f"  {s['stage'][:58]:58s} {s['calls_per_tick']:9.1f} {s['incl_ms_per_tick']:10.3f} {100 * s['incl_share']:7.1f} {s['own_ms_per_tick']:10.3f} {100 * s['own_share']:7.1f}")
        print(f"  pair distance checks per tick: {d['pair_checks_per_tick']}")
    if r["missing_stages"]:
        print(f"  stages not found (skipped): {r['missing_stages']}")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", required=True)
    ap.add_argument("--only", default="")
    ap.add_argument("--ticks", type=int, default=-1, help="ticks per recording (-1: the default of each stream; 0: all)")
    ap.add_argument("--src-root", default="", help="another source tree to profile (default the working tree)")
    args = ap.parse_args(argv)
    gate.use_source_tree(args.src_root or ROOT)
    names = [(s, m) for s, m in sc.GOLDEN_SET if not args.only or sc.name_of(s, m) in args.only.split(",")]
    path = OUT_DIR / f"{args.label}.json"
    import json
    doc = json.loads(path.read_text()) if path.exists() else {"label": args.label, "head": gate.git_head(), "recordings": {}}
    for stream, mode in names:
        name = sc.name_of(stream, mode)
        rec = sc.Recording(sc.GOLDEN_DIR / f"{name}.npz")
        r = profile(rec, DEFAULT_TICKS[stream] if args.ticks < 0 else args.ticks)
        doc["recordings"][name] = r
        sc.write_json_atomic(path, doc)
        print_table(r)
    print(f"\nwrote {path}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
