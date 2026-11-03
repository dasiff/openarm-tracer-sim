"""
Compare a CC_BENCH_LOG CSV with the CURRENT baseline of a scenario: max absolute difference per column.

    python experiments/cc_bench_compare.py <manual|teleop|policy> new.csv [--index PATH]

The baseline is the CURRENT row of experiments/baselines/INDEX.csv for the scenario (see experiments/baselines/README.md).
If the index is missing, or the scenario has no CURRENT baseline (NO_CURRENT), this stops with a message and exit code 2 or 3.
It never falls back to a retired file.

Base position in mm, yaw and joints in degrees, pedestal in mm, fingers in mm. Rows are matched by index (both logs
are written every 75 physics steps), and the row counts and sim times must agree.
"""

import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cc_bench_index  # noqa: E402  (the index reader, next to this file)


def load(path):
    with open(path) as f:
        r = csv.reader(f)
        head = next(r)
        return head, np.array([[float(v) for v in row] for row in r])


def current_baseline(scenario, index_path):
    """The path of the CURRENT baseline file for a scenario. Stops with a message and exit code 2 (no index or no row) or 3 (NO_CURRENT)."""
    def stop(code, text):
        print(f"cc_bench_compare: {text}\nNo comparison was made.", file=sys.stderr)
        sys.exit(code)
    try:
        rel = cc_bench_index.current(scenario, index_path)
    except cc_bench_index.IndexMissing as e:
        stop(2, f"baseline index missing or unreadable: {e}")
    except cc_bench_index.NoCurrent as e:
        stop(3, f"{e}. Make a baseline first (experiments/cc_bench_regen.sh {scenario}).")
    except cc_bench_index.NoRow as e:
        stop(2, str(e))
    path = os.path.join(os.path.dirname(index_path), rel)
    if not os.path.isfile(path):
        stop(2, f"the CURRENT file {path} does not exist")
    return path


def main(a_path, b_path):
    ha, A = load(a_path)
    hb, B = load(b_path)
    common = [h for h in ha if h in hb]                     # a log with extra columns (the neck's) is compared on the shared ones
    A = A[:, [ha.index(h) for h in common]]
    B = B[:, [hb.index(h) for h in common]]
    ha = common
    extra = [h for h in hb if h not in common]
    if extra:
        print(f"  (columns only in {b_path}, not compared: {extra})")
    n = min(len(A), len(B))
    print(f"{a_path}: {len(A)} rows; {b_path}: {len(B)} rows; comparing {n}")
    if len(A) != len(B):
        print("  NOTE: row counts differ")
    d = np.abs(A[:n] - B[:n])
    print(f"  max sim-time difference {d[:, 0].max():.2e} s")
    worst = 0.0
    for i, name in enumerate(ha):
        if i == 0:
            continue
        unit, scale = ("mm", 1000.0) if name in ("base_x", "base_y", "pedestal_m") or "finger" in name else ("deg", 1.0) if name == "base_yaw_deg" else ("deg", 180 / np.pi)
        v = d[:, i].max() * scale
        row = int(d[:, i].argmax())
        print(f"  {name:14s} max diff {v:10.6f} {unit}  (row {row}, t={A[row, 0]:.2f} s)")
        worst = max(worst, v if unit == "mm" else v)
    return 0


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Compare a bench CSV with the CURRENT baseline of its scenario (from INDEX.csv).")
    ap.add_argument("scenario", choices=["manual", "teleop", "policy"])
    ap.add_argument("new_csv", help="the CSV to check, e.g. CC_BENCH_LOG output")
    ap.add_argument("--index", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "baselines", "INDEX.csv"),
                    help="the baseline index (default experiments/baselines/INDEX.csv)")
    a = ap.parse_args()
    if not os.path.isfile(a.new_csv):
        print(f"cc_bench_compare: {a.new_csv} does not exist", file=sys.stderr)
        sys.exit(2)
    sys.exit(main(current_baseline(a.scenario, a.index), a.new_csv))
