"""Falsification search for the never-close claims of tools/never_close_dc.py.

Each pair the divide-and-conquer tool declared never-close (entries with "listed": true) is attacked: many random-restart
local minimisations (scipy L-BFGS-B) of the EXACT gap function over the joint limits look for the closest pose. A declared
pair passes when the smallest gap found is at least margin + PUSH_START_BEYOND_M (1 cm). Any pose below that breaks the claim.

Control: CONTROL_PAIRS pairs that were NOT declared go through the same search. A control pair passes when the search finds a
gap within CONTROL_NEAR_M of the threshold or below it, which shows the search can find close poses when they exist.

Progress is saved after every pair in <out>.work.json. A run stopped by --budget-min continues from that file on the next run.

Run from the project root, inside the activated venv:
    python scripts/safety_filter_dc_falsify.py [--dc FILE] [--out FILE] [--budget-min M] [--control-status S]

Exit codes: 0 all declared and control pairs passed; 1 a check failed; 3 stopped by the time budget (rerun to continue).
"""

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.touch import gap_between  # noqa: E402
from tools.never_close_dc import setup, state_of  # noqa: E402

# ---- Settings. Every number of this script is here, each with a plain-word comment: what it is, its unit, why this value. ----

SEED = 0                    # integer: base seed of every random generator; fixed so a run repeats exactly
RESTARTS = 200              # count: random starting poses per pair, each followed by one local minimisation; set for this search
CONTROL_PAIRS = 5           # count: unlisted pairs run as the control; a few pairs, not one, so the control is not a single case
CONTROL_NEAR_M = 0.01       # m (1 cm): a control pair passes when its smallest gap is at most this above the threshold (or below it)
OPTIMIZER = "L-BFGS-B"      # text: scipy local method run from each starting pose; it keeps the joint limits as bounds
DEFAULT_DC = ROOT / "data" / "safety_filter" / "never_close_dc.json"              # the file written by tools/never_close_dc.py
DEFAULT_OUT = ROOT / "data" / "safety_filter" / "never_close_dc_falsify.json"    # the result, written when every pair is done
WORK_SUFFIX = ".work.json"  # text: appended to the output name to make the checkpoint file name
TMP_SUFFIX = ".tmp"         # text: appended to a file name while it is written, before it is renamed into place
JSON_INDENT = 2             # count: spaces per level in the written JSON files (readable diffs)
SECONDS_PER_MINUTE = 60.0   # s per min: converts --budget-min to seconds
MM_PER_M = 1000.0           # mm per m: gaps and thresholds are printed in millimetres
GAP_DECIMALS = 3            # count: digits after the point for a printed gap in mm (1 micrometre)
SECONDS_DECIMALS = 1        # count: digits after the point for printed seconds
EXIT_OK = 0                 # exit code: every declared pair and every control pair passed
EXIT_FAIL = 1               # exit code: a check failed
EXIT_BUDGET = 3             # exit code: stopped by --budget-min; rerun to continue
ROLES = ("declared", "control")  # the two groups of rows in the checkpoint and in the result


def gap_fn(names, ida, idb, m, vm, rs):
    """The exact gap (m) between shapes ida and idb at the pose where the variables `names` take the values x."""
    def gap(x):
        placed = {p.id: p for p in rs.place(state_of(names, [float(v) for v in x], m, vm)) if p.id in (ida, idb)}
        return float(gap_between(placed[ida], placed[idb])[0])
    return gap


def min_gap_search(names, bounds, pair_no, gap):
    """Random-restart L-BFGS-B search for the smallest gap. Returns (smallest gap m, pose, evaluations, restarts run)."""
    if not names:
        return gap([]), {}, 1, 0
    lo = np.array([b[0] for b in bounds])
    hi = np.array([b[1] for b in bounds])
    best = {"gap": math.inf, "x": None, "evals": 0}

    def f(x):
        x = np.clip(np.asarray(x, dtype=float), lo, hi)      # the optimiser may step outside the limits: clip before evaluating
        g = gap(x)
        best["evals"] += 1
        if g < best["gap"]:
            best["gap"], best["x"] = g, x.copy()
        return g

    for k in range(RESTARTS):
        x0 = np.random.default_rng([SEED, pair_no, k]).uniform(lo, hi)
        f(x0)
        minimize(f, x0, method=OPTIMIZER, bounds=bounds)
    pose = {n: float(v) for n, v in zip(names, best["x"])}
    return best["gap"], pose, best["evals"], RESTARTS


def run_pair(role, entry, pair_no, m, vm, rs):
    """Search one pair and return its result row. pair_no numbers the pairs in run order (declared first, then control)."""
    ida, idb = entry["shape_pairs"][0]
    names = list(entry["variables"])
    if len(names) != entry["n_vars"]:
        raise SystemExit(f"{ida} vs {idb}: {len(names)} variables listed, but n_vars says {entry['n_vars']}")
    bounds = [(float(vm.range(n)[0]), float(vm.range(n)[1])) for n in names]
    thr = float(entry["margin_m"]) + cfg.PUSH_START_BEYOND_M
    t0 = time.time()
    gap, pose, evals, restarts_run = min_gap_search(names, bounds, pair_no, gap_fn(names, ida, idb, m, vm, rs))
    if role == "declared":
        passed = gap >= thr
    else:
        passed = gap <= thr + CONTROL_NEAR_M
    return {"pair": [ida, idb], "pair_type": entry["pair_type"], "n_vars": len(names), "status": entry["status"],
            "threshold_m": thr, "min_gap_m": gap, "min_pose": pose, "evaluations": evals, "restarts": restarts_run,
            "seconds": time.time() - t0, "passed": bool(passed)}


def progress_line(number, total, role, row):
    ida, idb = row["pair"]
    verdict = "PASS" if row["passed"] else "FAIL"
    return (f"[{number}/{total}] {ida} vs {idb}: {row['n_vars']} variables, {role}; smallest gap "
            f"{row['min_gap_m'] * MM_PER_M:.{GAP_DECIMALS}f} mm, threshold {row['threshold_m'] * MM_PER_M:.{GAP_DECIMALS}f} mm, "
            f"{verdict}; {row['seconds']:.{SECONDS_DECIMALS}f} s, {row['evaluations']} gap evaluations")


def key_of(role, entry):
    return (role, tuple(entry["shape_pairs"][0]))


def pick_control(entries, status):
    """CONTROL_PAIRS unlisted pairs, drawn without replacement with a fixed seed from the unlisted pairs sorted by pair id."""
    pool = [e for e in entries if not e["listed"] and (status is None or e["status"] == status)]
    pool.sort(key=lambda e: tuple(e["shape_pairs"][0]))
    if len(pool) < CONTROL_PAIRS:
        raise SystemExit(f"only {len(pool)} unlisted pairs to choose the control from (status {status}); {CONTROL_PAIRS} needed")
    picks = np.random.default_rng(SEED).choice(len(pool), size=CONTROL_PAIRS, replace=False)
    return [pool[int(k)] for k in picks]


def write_json_atomic(path, obj):
    """Write obj as JSON to a temporary file, read it back to check it is complete, then rename it over path.
    An interrupted write leaves the old file in place."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + TMP_SUFFIX)
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=JSON_INDENT)
        f.flush()
        os.fsync(f.fileno())
    with open(tmp, encoding="utf-8") as f:
        json.load(f)
    os.replace(tmp, path)


def load_work(path, fp):
    """Rows finished by an earlier run, keyed by (role, pair). Empty if there is no checkpoint or it is for another run."""
    done = {}
    if not path.exists():
        return done
    w = json.loads(path.read_text(encoding="utf-8"))
    if (w.get("fingerprint"), w.get("restarts"), w.get("seed")) != (fp, RESTARTS, SEED):
        print(f"checkpoint {path.name} is for another run (fingerprint, restarts or seed differ): starting again", flush=True)
        return done
    for role in ROLES:
        for row in w.get(role, []):
            done[(role, tuple(row["pair"]))] = row
    print(f"continuing: {len(done)} pairs already in {path.name}", flush=True)
    return done


def save_work(path, fp, done):
    rows = {role: [row for (r, _), row in done.items() if r == role] for role in ROLES}
    write_json_atomic(path, {"fingerprint": fp, "restarts": RESTARTS, "seed": SEED, **rows})


def main():
    ap = argparse.ArgumentParser(description="Falsification search for the never-close claims (see the module docstring).")
    ap.add_argument("--dc", type=Path, default=DEFAULT_DC, help="never-close file from tools/never_close_dc.py")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT, help="result file, written when every pair is done")
    ap.add_argument("--budget-min", type=float, default=None,
                    help="stop cleanly after this many minutes (exit 3); the checkpoint keeps the finished pairs")
    ap.add_argument("--control-status", default=None,
                    help="choose the control only among unlisted pairs with this status, e.g. reachable or 'not certified'")
    args = ap.parse_args()
    t_start = time.time()

    if not args.dc.exists():
        raise SystemExit(f"input file not found: {args.dc}")
    dc = json.loads(args.dc.read_text(encoding="utf-8"))
    m, fp, doc, vm, kins, rs, listed, checks = setup()
    if dc["fingerprint"] != fp:
        raise SystemExit("the never-close file is for another robot model: rebuild it first")

    entries = dc["entries"]
    declared = [e for e in entries if e["listed"]]
    control = pick_control(entries, args.control_status)
    plan = [("declared", e) for e in declared] + [("control", e) for e in control]
    total = len(plan)
    work_path = args.out.with_name(args.out.name + WORK_SUFFIX)
    done = load_work(work_path, fp)
    print(f"{len(declared)} declared pairs and {len(control)} control pairs, {RESTARTS} restarts each", flush=True)

    for pos, (role, entry) in enumerate(plan):
        ida, idb = entry["shape_pairs"][0]
        key = key_of(role, entry)
        if key in done:
            print(f"[{pos + 1}/{total}] {ida} vs {idb}: {role}, already in the checkpoint", flush=True)
            continue
        if args.budget_min is not None and time.time() - t_start >= args.budget_min * SECONDS_PER_MINUTE:
            n_done = sum(1 for r, e in plan if key_of(r, e) in done)
            print(f"time budget used up with {n_done} of {total} pairs done: rerun the same command to continue", flush=True)
            return EXIT_BUDGET
        row = run_pair(role, entry, pos, m, vm, rs)
        done[key] = row
        save_work(work_path, fp, done)
        print(progress_line(pos + 1, total, role, row), flush=True)

    declared_rows = [done[key_of("declared", e)] for e in declared]
    control_rows = [done[key_of("control", e)] for e in control]
    all_rows = declared_rows + control_rows
    margins = [r["min_gap_m"] - r["threshold_m"] for r in declared_rows]
    result = {
        "fingerprint": fp, "restarts": RESTARTS, "seed": SEED, "method": OPTIMIZER,
        "declared": declared_rows, "control": control_rows,
        "n_declared": len(declared_rows),
        "n_declared_passed": sum(r["passed"] for r in declared_rows),
        "declared_failed": [r["pair"] for r in declared_rows if not r["passed"]],
        "smallest_margin_over_threshold_m": min(margins) if margins else None,
        "n_control": len(control_rows),
        "n_control_passed": sum(r["passed"] for r in control_rows),
        "seconds_total": sum(r["seconds"] for r in all_rows),
    }
    write_json_atomic(args.out, result)
    all_ok = result["n_declared_passed"] == result["n_declared"] and result["n_control_passed"] == result["n_control"]
    smallest = "n/a" if not margins else f"{min(margins) * MM_PER_M:.{GAP_DECIMALS}f} mm"
    print(f"SUMMARY: declared {result['n_declared_passed']} of {result['n_declared']} passed; control "
          f"{result['n_control_passed']} of {result['n_control']} passed; smallest declared margin over threshold {smallest}; "
          f"{result['seconds_total']:.{SECONDS_DECIMALS}f} s; wrote {args.out}", flush=True)
    return EXIT_OK if all_ok else EXIT_FAIL


if __name__ == "__main__":
    sys.exit(main())
