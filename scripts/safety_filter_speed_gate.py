#!/usr/bin/env python
"""The equivalence gate of the safety filter speed work (Stage 5 speed, step 2): a changed filter must decide what the tagged filter decided.

Part A, per tick. The code under test (the working tree, or --src-root) is run over the recorded inputs of every golden recording, in
order, with a fresh resolver; each call gets the recorded last filtered command, so whatever the filter remembers between ticks
evolves from the same history. A tick passes when the filtered command is within TOL_CMD_RAD on every joint, the status class is the
same and the hold reason is the same. Every failing tick is listed with its numbers. Also reported (not part of pass or fail): ticks
whose exact status text, active pair set after the tick (the only between-tick memory that can be read from outside), sweeps, pairs
pushed / skipped / shortened, solver calls or step fraction differ. The run's time per call is the timing of the code under test.

Part B, closed loop. The code under test runs the punch (original and reactive) and the cat scene closed loop; the summaries
(measured by the TAG's code, scripts/safety_filter_speed_metrics.py) are compared with the golden summaries: every status count and
the hold count within TOL_COUNT_SHARE of the golden count, the smallest true gap (overall and per pair type) within TOL_GAP_MM, the
hand position error mean and max within TOL_HAND_MM.

The thresholds below were chosen by the assistant from the task text; David to confirm.

Usage (venv active, repo root, through scripts/safety_filter_speed_run.sh):
  python scripts/safety_filter_speed_gate.py --part A --label <name> [--only punch_original_kong_style,...] [--ticks N]
  python scripts/safety_filter_speed_gate.py --part B --label <name> [--only cat_kong_style]
  [--src-root <tree>] runs another source tree (a scratch copy) instead of the working tree.
Output: data/safety_filter/speed_gate/<label>/gate_A.json, gate_B.json (+ the closed-loop recordings). Exit code 0 = pass.
"""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import safety_filter_speed_common as sc  # noqa: E402

ROOT = sc.ROOT
GATE_DIR = ROOT / "data" / "safety_filter" / "speed_gate"

# ---- thresholds (assistant's choice from the task text; David to confirm) ----
TOL_CMD_RAD = 1e-3        # rad: part A, largest allowed difference of the filtered command on any joint
STATUS_CLASSES = {"PASS": "pass", "HOLD": "hold", "KONG_PUSH": "changed", "STEP_SHORTENED": "changed", "ELBOW_SWING": "changed",
                  "HAND_SLIDE": "changed"}   # part A "same status class": passed through, held still, or changed the command
TOL_COUNT_SHARE = 0.01    # share: part B, a status or hold count may differ by this share of its golden count (rounded down; a golden count of 0 allows 0)
TOL_GAP_MM = 1.0          # mm: part B, smallest true gap (overall and per pair type)
TOL_HAND_MM = 0.5         # mm: part B, hand position error mean and max
MAX_LISTED = 300          # count: failing ticks written out per recording (all are counted)
INFO_FIELDS = ("status", "active", "rounds", "pairs_pushed", "pairs_skipped", "pairs_shortened", "solver_calls", "fraction")


def use_source_tree(root):
    """Puts a source tree first on sys.path and makes sure `src` is imported from it."""
    root = str(Path(root).resolve())
    sys.path.insert(0, root)
    import src
    if not str(Path(src.__file__).resolve()).startswith(root):
        raise RuntimeError(f"src was imported from {src.__file__}, not from {root}")
    return root


def new_resolver(dt):
    """A fresh resolver of the code under test, on the working tree's shapes file."""
    from src.safety_filter import config as sfcfg
    from src.safety_filter.arm_conversion import make_solver
    from src.safety_filter.resolver import Resolver
    from src.safety_filter.robot_model import load_robot_only_model
    from src.safety_filter.shapes import RobotShapes
    if "solver" not in _CACHE:
        _CACHE["solver"] = make_solver()
        _CACHE["model"] = load_robot_only_model()
    return Resolver(RobotShapes(_CACHE["model"], ROOT / sfcfg.SHAPES_FILE), dt, _CACHE["solver"])


_CACHE = {}


def replay(rec, ticks=0, say=print):
    """Part A on one recording. Returns the result dict."""
    a = rec.a
    n = rec.n if not ticks else min(ticks, rec.n)
    fails, info = [], {k: 0 for k in INFO_FIELDS}
    max_diff, max_diff_tick = 0.0, -1
    times = np.zeros(n)
    with sc.resolver_mode(rec.mode):
        res = new_resolver(rec.dt)
        for k in range(n):
            raw, last, state, world, held = rec.inputs(k)
            t0 = time.perf_counter()
            out, st = res.step(raw, last, state, world, held)
            times[k] = time.perf_counter() - t0
            out = np.asarray(out, float).reshape(2, 7)
            d = float(np.abs(out - a["filtered"][k]).max())
            if d > max_diff:
                max_diff, max_diff_tick = d, k
            g_status, g_reason = str(a["status"][k]), str(a["reason"][k])
            reason = st.hold_reason or ""
            ok = d <= TOL_CMD_RAD and STATUS_CLASSES.get(st.status, st.status) == STATUS_CLASSES.get(g_status, g_status) and reason == g_reason
            if not ok:
                if len(fails) < MAX_LISTED:
                    j = np.unravel_index(int(np.argmax(np.abs(out - a["filtered"][k]))), (2, 7))
                    fails.append({"tick": k, "max_cmd_diff_rad": d, "arm": sc.SIDES[j[0]], "joint": int(j[1]) + 1, "status": [g_status, st.status],
                                  "hold_reason": [g_reason, reason], "sweeps": [int(a["rounds"][k]), int(st.rounds)],
                                  "pairs_pushed": [int(a["pairs_pushed"][k]), int(st.pairs_pushed)],
                                  "fraction": [float(a["fraction"][k]), float(st.fraction)], "worst_pair": [str(a["worst_pair"][k]), "|".join(st.worst_pair) if st.worst_pair else ""]})
                else:
                    fails.append(k)
            new = {"status": st.status, "active": ";".join(sorted("|".join(x) for x in res._active)), "rounds": int(st.rounds),
                   "pairs_pushed": int(st.pairs_pushed), "pairs_skipped": int(st.pairs_skipped), "pairs_shortened": int(st.pairs_shortened),
                   "solver_calls": int(st.solver_calls), "fraction": float(st.fraction)}
            for f in INFO_FIELDS:
                g = a[f][k]
                same = (np.isnan(g) and np.isnan(new[f])) or g == new[f] if f == "fraction" else str(g) == str(new[f])
                if not same:
                    info[f] += 1
            if (k + 1) % 500 == 0:
                say(f"{rec.path.stem}: tick {k + 1}/{n}, failing so far {len(fails)}, largest difference {max_diff:.3g} rad")
    gold_t = np.asarray(a["filter_time"][:n], float)
    not_pass = a["status"][:n] != "PASS"
    return {"recording": rec.path.name, "mode": rec.mode, "ticks": n, "failing_ticks": len(fails), "pass": len(fails) == 0,
            "largest_cmd_diff_rad": max_diff, "largest_cmd_diff_tick": max_diff_tick,
            "ticks_with_different": {("exact_status" if f == "status" else "active_set_after_tick" if f == "active" else f): v for f, v in info.items()},
            "time_ms": sc.time_stats_ms(times), "time_ms_golden": sc.time_stats_ms(gold_t),
            "time_ms_not_pass": sc.time_stats_ms(times[not_pass]), "time_ms_pass": sc.time_stats_ms(times[~not_pass]),
            "failing": fails[:MAX_LISTED]}


def part_a(args, out_dir):
    names = [sc.name_of(s, m) for s, m in sc.GOLDEN_SET]
    if args.only:
        names = [n for n in names if n in args.only.split(",")]
    path = out_dir / ("gate_A.json" if not args.ticks else f"gate_A_first{args.ticks}.json")
    doc = json.loads(path.read_text()) if path.exists() else {}
    doc.update(label=args.label, part="A", tol_cmd_rad=TOL_CMD_RAD, src_root=args.src_root or str(ROOT), head=git_head())
    doc.setdefault("recordings", {})
    for name in names:
        rec = sc.Recording(sc.GOLDEN_DIR / f"{name}.npz")
        t0 = time.time()
        r = replay(rec, args.ticks)
        r["runtime_s"] = round(time.time() - t0, 1)
        doc["recordings"][name] = r
        doc["pass"] = all(x["pass"] for x in doc["recordings"].values())
        sc.write_json_atomic(path, doc)                      # one recording at a time: a killed run keeps what is done
        t = r["time_ms"]
        print(f"A {name:28s} ticks {r['ticks']:5d} failing {r['failing_ticks']:4d} largest diff {r['largest_cmd_diff_rad']:.3g} rad  "
              f"time mean {t['mean']} median {t['median']} p95 {t['p95']} max {t['max']} ms  (golden mean {r['time_ms_golden']['mean']})  "
              f"different: {r['ticks_with_different']}", flush=True)
    print(f"PART A {'PASS' if doc['pass'] else 'FAIL'} ({len(doc['recordings'])} recordings) -> {path}")
    return doc["pass"]


def compare_summaries(gold, new):
    """Part B comparison of two summaries. Returns (ok, list of rows (name, golden, new, allowed, ok))."""
    rows = []

    def count_row(name, g, v):
        allowed = int(TOL_COUNT_SHARE * g)
        rows.append((name, g, v, f"+-{allowed}", abs(v - g) <= allowed))

    for st in sorted(set(gold["status_counts"]) | set(new["status_counts"])):
        count_row(f"status {st}", int(gold["status_counts"].get(st, 0)), int(new["status_counts"].get(st, 0)))
    count_row("hold ticks", int(gold["hold_ticks"]), int(new["hold_ticks"]))
    for r in sorted(set(gold["hold_reason_counts"]) | set(new["hold_reason_counts"])):
        count_row(f"hold reason {r}", int(gold["hold_reason_counts"].get(r, 0)), int(new["hold_reason_counts"].get(r, 0)))

    def gap_row(name, g, v):
        if g is None or v is None:
            rows.append((name, g, v, "both none", g is None and v is None))
        else:
            rows.append((name, g, v, f"+-{TOL_GAP_MM} mm", abs(v - g) <= TOL_GAP_MM))

    gap_row("min true gap mm", gold["min_true_gap_mm"], new["min_true_gap_mm"])
    for ty in gold["min_true_gap_by_type"]:
        g, v = gold["min_true_gap_by_type"][ty], new["min_true_gap_by_type"].get(ty)
        gap_row(f"min true gap {ty} mm", None if g is None else g["gap_mm"], None if v is None else v["gap_mm"])
    for key in ("hand_pos_err_mean_mm", "hand_pos_err_max_mm"):
        rows.append((key, gold[key], new[key], f"+-{TOL_HAND_MM} mm", abs(new[key] - gold[key]) <= TOL_HAND_MM))
    rows.append(("ticks (not gated)", gold["ticks"], new["ticks"], "-", True))
    rows.append(("largest joint change per tick rad (not gated)", gold["max_joint_change_per_tick_rad"], new["max_joint_change_per_tick_rad"], "-", True))
    return all(r[4] for r in rows), rows


def part_b(args, out_dir):
    names = [(s, m) for s, m in sc.CLOSED_LOOP_SET if not args.only or sc.name_of(s, m) in args.only.split(",")]
    path = out_dir / "gate_B.json"
    doc = json.loads(path.read_text()) if path.exists() else {}
    doc.update(label=args.label, part="B", src_root=args.src_root or str(ROOT), head=git_head(),
               tolerances={"count_share": TOL_COUNT_SHARE, "gap_mm": TOL_GAP_MM, "hand_mm": TOL_HAND_MM})
    doc.setdefault("streams", {})
    for stream, mode in names:
        name = sc.name_of(stream, mode)
        gold = json.loads((sc.GOLDEN_DIR / f"{name}_summary.json").read_text())
        rec, info = sc.run_stream(stream, mode, out_dir / "work", ticks=args.ticks)
        rpath = rec.save(out_dir / f"{name}.npz")
        spath = out_dir / f"{name}_summary.json"
        subprocess.run([sys.executable, str(ROOT / "scripts" / "safety_filter_speed_metrics.py"), "--root", str(sc.REF_TREE), str(rpath), "--out", str(spath)],
                       cwd=ROOT, check=True)
        new = json.loads(spath.read_text())
        ok, rows = compare_summaries(gold, new)
        g = sc.Recording(sc.GOLDEN_DIR / f"{name}.npz")
        n = min(g.n, rec.arrays()["filtered"].shape[0])
        same_cmd = bool(n == g.n == len(rec) and np.array_equal(g.a["filtered"], rec.arrays()["filtered"]) and np.array_equal(g.a["status"], rec.arrays()["status"]))
        first_diff = int(np.argmax(np.abs(g.a["filtered"][:n] - rec.arrays()["filtered"][:n]).max(axis=(1, 2)) > 0)) if not same_cmd and n else None
        doc["streams"][name] = {"pass": ok, "rows": [list(r) for r in rows], "run_info": info, "time_ms": new["time_ms"], "time_ms_golden": gold["time_ms"],
                                "identical_to_golden_commands_and_status": same_cmd, "first_tick_with_a_different_command": first_diff}
        doc["pass"] = all(x["pass"] for x in doc["streams"].values())
        sc.write_json_atomic(path, doc)
        print(f"B {name}: {'PASS' if ok else 'FAIL'}; identical to golden: {same_cmd}; time mean {new['time_ms'].get('mean')} ms (golden {gold['time_ms'].get('mean')})", flush=True)
        for r in rows:
            print(f"    {'ok  ' if r[4] else 'FAIL'} {r[0]:45s} golden {r[1]}  new {r[2]}  allowed {r[3]}", flush=True)
    print(f"PART B {'PASS' if doc['pass'] else 'FAIL'} ({len(doc['streams'])} streams) -> {path}")
    return doc["pass"]


def git_head():
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    except OSError:
        return ""


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--part", choices=("A", "B"), required=True)
    ap.add_argument("--label", required=True, help="name of this gate run (folder under data/safety_filter/speed_gate)")
    ap.add_argument("--only", default="", help="comma-separated <stream>_<mode> names (default: all)")
    ap.add_argument("--ticks", type=int, default=0, help="first N ticks only (a quick check, not the gate)")
    ap.add_argument("--src-root", default="", help="another source tree to test (a scratch copy); default the working tree")
    args = ap.parse_args(argv)
    use_source_tree(args.src_root or ROOT)
    out_dir = GATE_DIR / args.label
    out_dir.mkdir(parents=True, exist_ok=True)
    ok = part_a(args, out_dir) if args.part == "A" else part_b(args, out_dir)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
