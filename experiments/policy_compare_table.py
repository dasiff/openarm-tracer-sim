"""Consistent comparison table (2026-11-25): reads the per-run CSVs and summaries written by experiments/policy_stream_test.py for the
comparison queue (data/logs/policy_compare_20261125/<case>/<stream>_s<seed>_<perturb>/) and prints the three measurements per run and the table.

Measurements (positions in mm, rotations in mrad; reference = the bridge-accepted target minus the correction, as the stream test defines it):
  a. position and rotation error 1 s after the final hold starts (hold start = first tick at or after --stream-s).
  b. largest position error over the run, when it occurred (motion or hold), and the largest rotation error over the run.
  c. largest correction (hand displacement |C_p|, mm) and ticks at the clip (position and rotation, from the summary; zero for no correction).

    python experiments/policy_compare_table.py --root data/logs/policy_compare_20261125 --out data/logs/policy_compare_20261125/table.md
"""

import argparse
import csv
import glob
import json
import math
import os
from pathlib import Path


def read_run(folder, stream_s):
    csvs = sorted(glob.glob(os.path.join(folder, "*.csv")))
    if not csvs:
        return None
    base = csvs[0][:-4]
    rows = list(csv.DictReader(open(csvs[0])))
    summ = json.load(open(base + ".json"))
    t = [float(r["t"]) for r in rows]
    herr = [math.sqrt(float(r["herr_x"]) ** 2 + float(r["herr_y"]) ** 2 + float(r["herr_z"]) ** 2) * 1000 for r in rows]
    rerr = [math.sqrt(float(r["rerr_x"]) ** 2 + float(r["rerr_y"]) ** 2 + float(r["rerr_z"]) ** 2) * 1000 for r in rows]
    cp = [math.sqrt(float(r["Cp_x"]) ** 2 + float(r["Cp_y"]) ** 2 + float(r["Cp_z"]) ** 2) * 1000 for r in rows]
    hold_i = next((i for i, x in enumerate(t) if x >= stream_s), None)
    if hold_i is None:
        return None
    t_hold = t[hold_i]
    a_i = min(range(len(t)), key=lambda i: abs(t[i] - (t_hold + 1.0)))
    finite = [(herr[i], i) for i in range(len(t)) if not math.isnan(herr[i])]
    bmax, bi = max(finite)
    cfin = [(cp[i], i) for i in range(len(t)) if not math.isnan(cp[i])]
    cmax, _ = max(cfin)
    rfin = [rerr[i] for i in range(len(t)) if not math.isnan(rerr[i])]
    return {
        "folder": folder, "name": os.path.basename(base), "t_hold": round(t_hold, 3), "t_a": round(t[a_i], 3),
        "a_pos_mm": herr[a_i], "a_rot_mrad": rerr[a_i],
        "b_pos_mm": bmax, "b_t": round(t[bi], 3), "b_phase": "hold" if t[bi] >= t_hold else "motion", "b_rot_mrad": max(rfin),
        "c_disp_mm": cmax, "c_clip_pos": summ["position"]["ticks_at_clip"], "c_clip_rot": summ["rotation"]["ticks_at_clip"],
        "hold_s": summ.get("hold_s"), "rule": summ.get("rule"), "perturb": summ.get("perturb"), "correction": summ.get("correction"),
        "stopped_by": summ.get("stopped_by"),
    }


def fmt(x, nd=1):
    return "n/a" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:.{nd}f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="data/logs/policy_compare_20261125")
    ap.add_argument("--stream-s", type=float, default=30.0)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    cases = ["off", "nudge", "measure"]
    streams = [("A", [1]), ("B", [1, 2, 3]), ("C", [1, 2, 3]), ("D", [1, 2, 3])]
    runs = {}
    for case in cases:
        for st, seeds in streams:
            for s in seeds:
                folder = glob.glob(os.path.join(a.root, case, f"{st}_s{s}_on"))
                runs[(case, st, s)] = read_run(folder[0], a.stream_s) if folder else None
        ref = glob.glob(os.path.join(a.root, f"ref_{case}", "A_s1_*"))
        runs[(f"ref_{case}", "A", 1)] = read_run(ref[0], a.stream_s) if ref else None
    lines = []
    lines.append("## Per run\n")
    lines.append("| case | stream | seed | a: pos mm @1 s | a: rot mrad | b: max pos mm | b: when | b: rot max mrad | c: max corr mm | c: clip pos ticks | c: clip rot ticks |")
    lines.append("|---|---|---|---|---|---|---|---|---|---|---|")
    for key, r in runs.items():
        if r is None:
            lines.append(f"| {key[0]} | {key[1]} | {key[2]} | missing | | | | | | | |")
            continue
        lines.append(f"| {key[0]} | {key[1]} | {key[2]} | {fmt(r['a_pos_mm'])} | {fmt(r['a_rot_mrad'])} | {fmt(r['b_pos_mm'])} | "
                     f"{r['b_phase']} at {r['b_t']} s | {fmt(r['b_rot_mrad'])} | {fmt(r['c_disp_mm'])} | {r['c_clip_pos']} | {r['c_clip_rot']} |")
    lines.append("\n## Table: rows = streams; columns = cases. B, C, D: mean (range over seeds)\n")

    def cell(vals, nd=1):
        vals = [v for v in vals if v is not None and not (isinstance(v, float) and math.isnan(v))]
        if not vals:
            return "n/a"
        m = sum(vals) / len(vals)
        if len(vals) == 1:
            return fmt(m, nd)
        return f"{fmt(m, nd)} ({fmt(min(vals), nd)} to {fmt(max(vals), nd)})"
    measures = [("a. position error 1 s after the final hold starts (mm)", "a_pos_mm", 1), ("a. rotation error 1 s after the final hold starts (mrad)", "a_rot_mrad", 1),
                ("b. largest position error over the run (mm)", "b_pos_mm", 1), ("b. largest rotation error over the run (mrad)", "b_rot_mrad", 1),
                ("c. largest correction, hand displacement (mm)", "c_disp_mm", 1), ("c. ticks at the position clip", "c_clip_pos", 0), ("c. ticks at the rotation clip", "c_clip_rot", 0)]
    for title, key, nd in measures:
        lines.append(f"### {title}\n")
        lines.append("| stream | off (no correction) | nudge (8e45fac rule, test only) | measure (production) |")
        lines.append("|---|---|---|---|")
        for st, seeds in streams:
            cells = []
            for case in cases:
                vals = [runs[(case, st, s)][key] if runs[(case, st, s)] else None for s in seeds]
                cells.append(cell(vals, nd))
            lines.append(f"| {st} | " + " | ".join(cells) + " |")
        lines.append(f"| A, normal physics (reference) | " + " | ".join(
            cell([runs[(f"ref_{c}", "A", 1)][key] if runs[(f"ref_{c}", "A", 1)] else None], nd) for c in cases) + " |")
        lines.append("")
    text = "\n".join(lines)
    print(text)
    if a.out:
        Path(a.out).write_text(text + "\n")


if __name__ == "__main__":
    main()
