"""Rest times and the comparison table (2026-11-25): for each run of the comparison (experiments/policy_stream_test.py, perturb on, bridge on,
safety filter off), from the per-tick CSVs (<name>.csv: hand error, correction, pad) and joint CSVs (<name>_joints.csv: reference joints, measured
joints, joint velocities):

  * t_ref_stop: the last time after the stream ends (the hold start) at which the reference joints (bridge output minus dq) change at more than
    THR (1e-4 rad/s on any joint). After it the reference is constant.
  * t_joint_stop: the last time after the hold start at which any joint's velocity exceeds THR.
  * a. the hand position error (mm) and rotation error (mrad) 1 s and 2 s after t_ref_stop (and, for comparison, 1 s after the hold start).
  * b. the largest position error over the run (mm) and when; the largest rotation error over the run (mrad).
  * c. the largest correction (hand displacement |C_p|, mm); clip ticks from the summary.

Runs that did not settle before the end of the run are flagged (a value after the run end is "n/a").

    python experiments/policy_rest_table.py --root data/logs/policy_compare_20261125/h6 --out <table.md>
"""

import argparse
import csv
import glob
import json
import math
import os
from pathlib import Path

import numpy as np

THR = 1e-4          # rad/s on any joint (rest threshold)
STREAM_S = 30.0     # sim time at which the stream ends and the hold starts (the source's trigger)


def load(folder):
    main = glob.glob(os.path.join(folder, "*.csv"))
    main = [m for m in main if not m.endswith("_joints.csv")]
    if not main:
        return None
    base = main[0][:-4]
    M = list(csv.DictReader(open(main[0])))
    J = list(csv.DictReader(open(base + "_joints.csv")))
    S = json.load(open(base + ".json"))
    t = np.array([float(r["t"]) for r in M])
    herr = np.array([[float(r[k]) for k in ("herr_x", "herr_y", "herr_z")] for r in M])
    rerr = np.array([[float(r[k]) for k in ("rerr_x", "rerr_y", "rerr_z")] for r in M])
    cp = np.array([[float(r[k]) for k in ("Cp_x", "Cp_y", "Cp_z")] for r in M])
    tj = np.array([float(r["t"]) for r in J])
    ref = np.array([[float(r[f"ref{k}"]) for k in range(7)] for r in J])
    qv = np.array([[float(r[f"v{k}"]) for k in range(7)] for r in J])
    return {"name": os.path.basename(base), "t": t, "herr": herr, "rerr": rerr, "cp": cp, "tj": tj, "ref": ref, "qv": qv, "summary": S}


def last_time_above(t, sig, start_t, thr):
    """The last time at or after start_t at which sig exceeds thr (None if never; start_t if only before it)."""
    sel = np.where((t >= start_t) & (sig > thr))[0]
    return float(t[sel[-1]]) if len(sel) else None


def at(t, arr, when):
    if when is None or when > t[-1]:
        return np.nan
    i = int(np.argmin(np.abs(t - when)))
    return arr[i]


def metrics(d):
    t, tj, ref, qv = d["t"], d["tj"], d["ref"], d["qv"]
    t_hold = float(t[np.searchsorted(t, STREAM_S)]) if np.any(t >= STREAM_S) else None
    if t_hold is None:
        return None
    # reference change rate per joint (rad/s), at the joint log's sample times
    dt = np.diff(tj)
    rate = np.max(np.abs(np.diff(ref, axis=0)) / np.maximum(dt, 1e-12)[:, None], axis=1)
    tr_last = last_time_above(tj[1:], rate, t_hold, THR)
    t_ref_stop = tr_last if tr_last is not None else t_hold
    vmax = np.max(np.abs(qv), axis=1)
    tv_last = last_time_above(tj, vmax, t_hold, THR)
    t_joint_stop = tv_last if tv_last is not None else t_hold
    herr = np.linalg.norm(d["herr"], axis=1) * 1000
    rerr = np.linalg.norm(d["rerr"], axis=1) * 1000
    cp = np.linalg.norm(d["cp"], axis=1) * 1000
    ok = ~np.isnan(herr)
    bmax_i = int(np.nanargmax(herr))
    run_end = float(t[-1])
    flag = []
    if run_end - t_ref_stop < 2.0:
        flag.append("ref stop too late for +2 s")
    if run_end - t_joint_stop < 0.5:
        flag.append("joints not settled before run end")
    S = d["summary"]
    return {
        "name": d["name"], "t_hold": t_hold, "run_end": run_end,
        "t_ref_stop_rel": t_ref_stop - t_hold, "t_joint_stop_rel": t_joint_stop - t_hold,
        "a1_pos": at(t, herr, t_ref_stop + 1.0), "a1_rot": at(t, rerr, t_ref_stop + 1.0),
        "a2_pos": at(t, herr, t_ref_stop + 2.0), "a2_rot": at(t, rerr, t_ref_stop + 2.0),
        "a_hold1_pos": at(t, herr, t_hold + 1.0), "a_hold1_rot": at(t, rerr, t_hold + 1.0),
        "b_pos": float(herr[bmax_i]), "b_t": float(t[bmax_i]), "b_phase": "hold" if t[bmax_i] >= t_hold else "motion",
        "b_rot": float(np.nanmax(rerr)), "c_disp": float(np.nanmax(cp)),
        "c_clip_pos": S["position"]["ticks_at_clip"], "c_clip_rot": S["rotation"]["ticks_at_clip"],
        "flags": flag,
    }


def fmt(x, nd=1):
    return "n/a" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:.{nd}f}"


def cell(vals, nd=1):
    vals = [v for v in vals if v is not None and not (isinstance(v, float) and math.isnan(v))]
    if not vals:
        return "n/a"
    m = sum(vals) / len(vals)
    if len(vals) == 1:
        return fmt(m, nd)
    return f"{fmt(m, nd)} ({fmt(min(vals), nd)} to {fmt(max(vals), nd)})"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    cases = ["off", "nudge", "measure"]
    streams = [("A", [1]), ("B", [1, 2, 3]), ("C", [1, 2, 3]), ("D", [1, 2, 3])]
    R = {}
    for case in cases:
        for st, seeds in streams:
            for s in seeds:
                f = glob.glob(os.path.join(a.root, case, f"{st}_s{s}_on"))
                R[(case, st, s)] = metrics(load(f[0])) if f else None
        f = glob.glob(os.path.join(a.root, f"ref_{case}", "A_s1_*"))
        R[(f"ref_{case}", "A", 1)] = metrics(load(f[0])) if f else None
    L = []
    L.append("## Rest times (sim time, measured from the hold start; threshold 1e-4 rad/s on any joint)\n")
    L.append("| case | stream | seed | reference stops changing (s after hold) | joints stop moving (s after hold) | flags |")
    L.append("|---|---|---|---|---|---|")
    for key, r in R.items():
        if r is None:
            L.append(f"| {key[0]} | {key[1]} | {key[2]} | missing | | |")
            continue
        L.append(f"| {key[0]} | {key[1]} | {key[2]} | {r['t_ref_stop_rel']:.2f} | {r['t_joint_stop_rel']:.2f} | {'; '.join(r['flags'])} |")
    L.append("\n## Per run: a, b, c\n")
    L.append("| case | stream | seed | a1 hand pos mm (1 s after ref stop) | a1 rot mrad | a2 pos mm (2 s after ref stop) | a2 rot mrad | a_hold1 pos mm (1 s after hold start) | b max pos mm | b when | b max rot mrad | c max |C_p| mm | clip pos | clip rot |")
    L.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for key, r in R.items():
        if r is None:
            continue
        L.append(f"| {key[0]} | {key[1]} | {key[2]} | {fmt(r['a1_pos'])} | {fmt(r['a1_rot'])} | {fmt(r['a2_pos'])} | {fmt(r['a2_rot'])} | {fmt(r['a_hold1_pos'])} | "
                 f"{fmt(r['b_pos'])} | {r['b_phase']} at {r['b_t']:.2f} s | {fmt(r['b_rot'])} | {fmt(r['c_disp'])} | {r['c_clip_pos']} | {r['c_clip_rot']} |")

    def table(title, key, nd=1):
        out = [f"\n### {title}\n", "| stream | off (no correction) | nudge (8e45fac, test only) | measure (production) |", "|---|---|---|---|"]
        for st, seeds in streams:
            cells = [cell([R[(c, st, s)][key] if R[(c, st, s)] else None for s in seeds], nd) for c in cases]
            out.append(f"| {st} | " + " | ".join(cells) + " |")
        out.append("| A, normal physics (reference) | " + " | ".join(cell([R[(f"ref_{c}", "A", 1)][key] if R[(f"ref_{c}", "A", 1)] else None], nd) for c in cases) + " |")
        return out
    L.append("\n## Table (B, C, D: mean (range over seeds))")
    L += table("a. hand position error 1 s after the reference stops changing (mm)", "a1_pos")
    L += table("a. hand rotation error 1 s after the reference stops changing (mrad)", "a1_rot")
    L += table("a. hand position error 2 s after the reference stops changing (mm)", "a2_pos")
    L += table("a. hand rotation error 2 s after the reference stops changing (mrad)", "a2_rot")
    L += table("a (old definition, 1 s after the hold starts): hand position error (mm)", "a_hold1_pos")
    L += table("b. largest position error over the run (mm)", "b_pos")
    L += table("b. largest rotation error over the run (mrad)", "b_rot")
    L += table("c. largest correction, hand displacement |C_p| (mm)", "c_disp")
    text = "\n".join(L)
    print(text)
    if a.out:
        Path(a.out).write_text(text + "\n")


if __name__ == "__main__":
    main()
