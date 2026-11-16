"""Torso-guard self-check for the divide-and-conquer never-close labels.

Check A tests the torso labels in the DC file against sampled arm poses:
  A1  every torso-dependent pair stays within the torso margin plus a loose extra on poses drawn from its certifying boxes;
  A2  every other pair stays clear of the torso margin on poses drawn from its torso-free box.

Check B replays the golden command streams through the resolver with the merged never-close list and the torso guard:
  - a tick away from the torso consults the full never-close list and gives the golden output;
  - a torso-proximate tick consults no torso-dependent pair, and its output still equals the golden output
    (the golden recordings were made with every such pair checked).

The script never resolves a disagreement. At the first one it prints the details, writes the result JSON with "stopped" set,
and exits with code 1.

Run from the repo root with the venv active. This is a heavy job: run it detached with a log under data/logs/:
    python scripts/safety_filter_dc_torso_check.py [--dc PATH] [--out PATH] [--only A|B]
"""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
import safety_filter_speed_common as sc
from src.safety_filter import config as cfg
from src.safety_filter import shapes as shapes_mod
from src.safety_filter.arm_conversion import make_solver
from src.safety_filter.resolver import Resolver
from src.safety_filter.shapes import RobotShapes
from tools.never_close_dc import DC, DC_FILE, Pair, leaves_of, setup, state_of, write_json

EXIT_PASS = 0  # process exit code when both checks passed
EXIT_FAIL = 1  # process exit code when a check failed or stopped at a disagreement
MM_PER_M = 1000.0  # millimetres per metre: gaps are stored and printed in mm
MM_DIGITS = 2  # decimal places for a gap printed in mm
S_DIGITS = 1  # decimal places for seconds printed
PROGRESS_EVERY = 2000  # check B prints a progress line every this many ticks of a recording
OUT_SHAPE = (2, 7)  # one command array: 2 arms by 7 joints
DEFAULT_OUT = ROOT / "data" / "safety_filter" / "never_close_dc_torso_check.json"  # result JSON path when --out is not given


class Disagreement(Exception):
    """A check found a disagreement. Nothing resolves it: the run stops and the details go to the result JSON."""

    def __init__(self, check, info):
        super().__init__(info["rule"])
        self.check = check  # "A" or "B"
        self.info = dict(info, check=check)  # plain values only: rule, the pair or tick, the pose or outputs


def to_mm(metres):
    """A gap in metres as millimetres (None stays None)."""
    return None if metres is None else float(metres) * MM_PER_M


def fmt_mm(mm):
    """A gap in millimetres as text."""
    return "n/a" if mm is None else f"{mm:.{MM_DIGITS}f}"


def widen(old, new, pick):
    """Running max or min (pick) over a group that starts empty (None); a None new value changes nothing."""
    if new is None:
        return old
    return new if old is None else pick(old, new)


def load_entries(path, fp):
    """The DC file's entries. Stops if its fingerprint is not this tree's, or a listed entry has no torso_dependent key."""
    doc = json.loads(Path(path).read_text())
    if doc.get("fingerprint") != fp:
        raise SystemExit(f"{path}: fingerprint {doc.get('fingerprint')} differs from this tree's {fp}")
    for entry in doc["entries"]:
        if entry.get("listed", False) and "torso_dependent" not in entry:
            raise SystemExit(f"{path}: listed entry {entry.get('shape_pairs')} has no torso_dependent key")
    return doc["entries"]


def listed_pairs(entries):
    """(entry, id_a, id_b) for every shape pair of every listed entry, in file order."""
    return [(entry, ida, idb) for entry in entries if entry.get("listed", False) for ida, idb in entry["shape_pairs"]]


def full_pose(rng, vm, all_names, own_names, own_values):
    """Every filter variable: the pair's own variables at own_values, every other one uniform in its limits."""
    own = dict(zip(own_names, own_values))
    values = []
    for name in all_names:
        if name in own:
            values.append(float(own[name]))
        else:
            lo, hi = vm.range(name)
            values.append(float(lo + rng.random() * (hi - lo)))
    return values


def pose_dict(names, values):
    """A pose as {variable name: value}."""
    return {name: float(v) for name, v in zip(names, values)}


def torso_gap_at(rs, m, vm, all_names, values, sides):
    """Smallest arm-to-torso gap in metres, for the arms in sides, at this pose."""
    rs.set_state(state_of(all_names, values, m, vm))
    return float(rs.torso_gap(sides))


def check_a(entries, m, vm, kins, rs, stats):
    """Check A: the torso labels against sampled poses. Raises Disagreement at the first sample that breaks its rule."""
    all_names = sorted(vm.joints)
    margin = DC["torso_proximity_margin_m"]
    dep_limit = margin + DC["torso_check_loose_extra_m"]
    pairs = listed_pairs(entries)
    for index, (entry, ida, idb) in enumerate(pairs):
        t0 = time.perf_counter()
        dependent = bool(entry["torso_dependent"])
        label = "torso-dependent" if dependent else "not torso-dependent"
        pair = Pair(vm, kins[ida], kins[idb], entry["margin_m"])
        sides = None  # every arm shape of both arms, as the run-time guard and the label count them
        rng = np.random.default_rng([DC["seed"], index])
        info = {"pair": [ida, idb], "label": label, "pair_margin_m": entry["margin_m"]}
        if dependent:
            lo, hi, _lb, _depth, _r = leaves_of(pair)
            n_boxes = len(lo)
            n_samples = DC["torso_check_dependent_poses"]
            if n_boxes == 0:
                raise Disagreement("A", dict(info, rule="torso-dependent pair has no certifying box to sample"))
            worst = None  # largest gap over this pair's samples, metres
            for _ in range(n_samples):
                b = int(rng.integers(n_boxes))
                own = lo[b] + rng.random(len(pair.names)) * (hi[b] - lo[b])
                values = full_pose(rng, vm, all_names, pair.names, own)
                g = torso_gap_at(rs, m, vm, all_names, values, sides)
                if g > dep_limit:
                    rule = "A1: torso-dependent pair is farther from the torso than the limit"
                    pose = pose_dict(all_names, values)
                    raise Disagreement("A", dict(info, rule=rule, pose=pose, torso_gap_mm=to_mm(g), limit_mm=to_mm(dep_limit)))
                worst = widen(worst, g, max)
            stats["n_dependent"] += 1
            stats["samples_dependent"] += n_samples
            stats["largest_gap_dependent_mm"] = widen(stats["largest_gap_dependent_mm"], to_mm(worst), max)
            extreme, which = worst, "largest gap"
        else:
            box = (entry.get("torso_label") or {}).get("torso_free_box")
            if box is None:
                raise Disagreement("A", dict(info, rule="A2: torso_free_box is null for a pair labelled not torso-dependent"))
            missing = [n for n in pair.names if n not in box["lo"] or n not in box["hi"]]
            if missing:
                raise SystemExit(f"torso_free_box of pair {ida}+{idb} has no limits for {missing}")
            lo = np.array([box["lo"][n] for n in pair.names], dtype=float)
            hi = np.array([box["hi"][n] for n in pair.names], dtype=float)
            n_samples = DC["torso_check_free_poses"]
            best = None  # smallest gap over this pair's samples, metres
            for _ in range(n_samples):
                own = lo + rng.random(len(pair.names)) * (hi - lo)
                values = full_pose(rng, vm, all_names, pair.names, own)
                g = torso_gap_at(rs, m, vm, all_names, values, sides)
                if g <= margin:
                    rule = "A2: pair inside its torso-free box comes within the torso margin"
                    pose = pose_dict(all_names, values)
                    raise Disagreement("A", dict(info, rule=rule, pose=pose, torso_gap_mm=to_mm(g), margin_mm=to_mm(margin)))
                best = widen(best, g, min)
            stats["n_not_dependent"] += 1
            stats["samples_not_dependent"] += n_samples
            stats["smallest_gap_not_dependent_mm"] = widen(stats["smallest_gap_not_dependent_mm"], to_mm(best), min)
            extreme, which = best, "smallest gap"
        secs = time.perf_counter() - t0
        print(f"A {index + 1}/{len(pairs)} pair {ida}+{idb} {label}: {n_samples} samples, "
              f"{which} {fmt_mm(to_mm(extreme))} mm, {secs:.{S_DIGITS}f} s", flush=True)


def tick_rule(proximate, same, active, shapes):
    """The rule this tick breaks, or None. same: the output equals the golden output."""
    if not proximate:
        if not same:
            return "not torso-proximate: output differs from the golden output"
        if active != shapes.never_close:
            return "not torso-proximate: the filter did not consult the full never-close list"
        return None
    if active & shapes.torso_dependent:
        return "torso-proximate: the filter consulted a torso-dependent pair"
    if not same:
        return "torso-proximate: output differs from the golden output (less conservative than the unguarded filter)"
    return None


def check_b(m, stats):
    """Check B: the guard on the golden command streams. Raises Disagreement at the first tick that breaks a rule."""
    shapes = RobotShapes(m, ROOT / cfg.SHAPES_FILE)  # built once, reused for every recording
    solver = make_solver()  # built once, reused for every recording
    if not (isinstance(shapes.torso_dependent, frozenset) and isinstance(shapes.never_close, frozenset)):
        raise Disagreement("B", {"rule": "shapes.torso_dependent or shapes.never_close is not a frozenset"})
    stats["torso_dependent_count"] = len(shapes.torso_dependent)
    stats["never_close_count"] = len(shapes.never_close)
    for stream, mode in sc.GOLDEN_SET:
        name = sc.name_of(stream, mode)
        rec = sc.Recording(sc.GOLDEN_DIR / f"{name}.npz")
        n_ticks = int(rec.n)
        row = {"name": name, "ticks": n_ticks, "torso_proximate_ticks": 0, "disagreements": 0,
               "min_torso_gap_mm": None, "max_torso_gap_mm": None, "seconds": None}
        t0 = time.perf_counter()
        with sc.resolver_mode(rec.mode):
            res = Resolver(shapes, rec.dt, solver)
            res.joint_speed_rad_s = sc.bridge_speed()
            for k in range(n_ticks):
                raw, last, state, world, held = rec.inputs(k)
                out, st = res.step(raw, last, state, world, held)
                proximate = bool(res.torso_proximate)
                gap_mm = to_mm(res.torso_gap_m)
                gold_cmd = np.asarray(rec.a["filtered"][k], dtype=float).reshape(OUT_SHAPE)
                gold_status = str(rec.a["status"][k])
                gold_reason = str(rec.a["reason"][k])
                new_cmd = np.asarray(out, dtype=float).reshape(OUT_SHAPE)
                new_status = st.status
                new_reason = st.hold_reason or ""
                same = bool(np.array_equal(gold_cmd, new_cmd)) and gold_status == new_status and gold_reason == new_reason
                active = frozenset(shapes_mod.never_close_pairs())
                rule = tick_rule(proximate, same, active, shapes)
                if rule is not None:
                    consulted = [sorted(map(str, p)) for p in active & shapes.torso_dependent]
                    raise Disagreement("B", {
                        "rule": rule, "recording": name, "tick": k, "torso_proximate": proximate,
                        "max_abs_command_diff": float(np.max(np.abs(gold_cmd - new_cmd))),
                        "golden_status": gold_status, "new_status": str(new_status),
                        "golden_hold_reason": gold_reason, "new_hold_reason": str(new_reason),
                        "active_list_size": len(active), "torso_dependent_consulted": consulted,
                    })
                if proximate:
                    row["torso_proximate_ticks"] += 1
                row["min_torso_gap_mm"] = widen(row["min_torso_gap_mm"], gap_mm, min)
                row["max_torso_gap_mm"] = widen(row["max_torso_gap_mm"], gap_mm, max)
                if k and k % PROGRESS_EVERY == 0:
                    print(f"  {name}: tick {k}/{n_ticks}, {time.perf_counter() - t0:.{S_DIGITS}f} s", flush=True)
        row["seconds"] = time.perf_counter() - t0
        stats["recordings"].append(row)
        stats["total_ticks"] += n_ticks
        stats["torso_proximate_ticks"] += row["torso_proximate_ticks"]
        stats["disagreements"] += row["disagreements"]
        print(f"{name}: {n_ticks} ticks, {row['torso_proximate_ticks']} torso-proximate, {row['disagreements']} disagreements, "
              f"torso gap {fmt_mm(row['min_torso_gap_mm'])}..{fmt_mm(row['max_torso_gap_mm'])} mm, "
              f"{row['seconds']:.{S_DIGITS}f} s", flush=True)


def summary_a(s):
    """The final Check A line."""
    return (f"Check A: PASS — {s['n_dependent']} torso-dependent pairs, {s['samples_dependent']} samples, "
            f"largest arm-to-torso gap {fmt_mm(s['largest_gap_dependent_mm'])} mm "
            f"(limit {fmt_mm(s['limit_dependent_mm'])} mm); "
            f"{s['n_not_dependent']} other pairs, {s['samples_not_dependent']} samples, "
            f"smallest gap {fmt_mm(s['smallest_gap_not_dependent_mm'])} mm "
            f"(must exceed {fmt_mm(s['margin_mm'])} mm)")


def summary_b(s):
    """The final Check B line."""
    return (f"Check B: PASS — {s['total_ticks']} ticks on {len(s['recordings'])} recordings, "
            f"{s['torso_proximate_ticks']} torso-proximate, {s['disagreements']} disagreements")


def save(path, obj):
    """Write the result JSON, creating its folder if needed."""
    path.parent.mkdir(parents=True, exist_ok=True)
    write_json(path, obj)


def main():
    ap = argparse.ArgumentParser(description="Torso-guard self-check: check A (torso labels), check B (guard on golden streams).")
    ap.add_argument("--dc", type=Path, default=DC_FILE, help="DC file with the labelled entries")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT, help="where the result JSON is written")
    ap.add_argument("--only", choices=("A", "B"), help="run just this check")
    args = ap.parse_args()

    m, fp, doc, vm, kins, rs, listed, checks = setup()
    entries = load_entries(args.dc, fp)
    margin = DC["torso_proximity_margin_m"]
    run_a = args.only in (None, "A")
    run_b = args.only in (None, "B")
    stats_a = {"n_dependent": 0, "n_not_dependent": 0, "samples_dependent": 0, "samples_not_dependent": 0,
               "largest_gap_dependent_mm": None, "smallest_gap_not_dependent_mm": None,
               "limit_dependent_mm": to_mm(margin + DC["torso_check_loose_extra_m"]), "margin_mm": to_mm(margin),
               "passed": False}
    stats_b = {"recordings": [], "total_ticks": 0, "torso_proximate_ticks": 0, "disagreements": 0,
               "torso_dependent_count": None, "never_close_count": None, "passed": False}
    result = {"fingerprint": fp, "margin_m": margin, "check_a": None, "check_b": None, "stopped": None}
    try:
        if run_a:
            result["check_a"] = stats_a
            check_a(entries, m, vm, kins, rs, stats_a)
            stats_a["passed"] = True
        if run_b:
            result["check_b"] = stats_b
            check_b(m, stats_b)
            stats_b["passed"] = True
    except Disagreement as d:
        print(json.dumps(d.info, indent=2, default=str), flush=True)
        print(f"Check {d.check}: FAIL — {d.info['rule']}", flush=True)
        result["stopped"] = d.info
        save(args.out, result)
        return EXIT_FAIL
    save(args.out, result)
    if run_a:
        print(summary_a(stats_a), flush=True)
    if run_b:
        print(summary_b(stats_b), flush=True)
    return EXIT_PASS


if __name__ == "__main__":
    sys.exit(main())
