"""Rescore of the smoothing time (SF-126, David d0bc 2026-11-24, decision 4): for production and each T, the hand error at rest 1 s after each
move ends, the offset's distance from the at-rest droop at the moment motion stops, and the peak offset and ticks at the clip. Analysis only; loads
MuJoCo for the pad forward kinematics, so run it with no other simulation.

    python experiments/rule_rescore.py --root data/logs/stage2m --out data/logs/stage2m/rescore.md

Moves and stops: policy B (bridge off): a stop is the first tick after a move (the source target changing faster than 0.05 rad/s) at which the
source target stays still for 0.5 s; the hand error is the pad of the source target minus the pad of the measured joints, 1 s after the stop.
Teleop pauses (bridge on): the accepted reference (REF = bridge output minus dq) stays unchanged for at least 1 s; the pause start is the stop.
Five poses: the static holds' own numbers (hand error 1 s after each stop; offset at the stop against the droop of the correction-off hold).
"""

import argparse
import re
import sys
from pathlib import Path

import numpy as np
import mujoco

PROJECT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(PROJECT / "experiments"))

NOMINAL = 1.0 / 150.0
RULES = [("integ", "production"), ("meas_T0.25", "measure T=0.25 s"), ("meas_T0.5", "measure T=0.5 s"),
         ("meas_T1", "measure T=1 s"), ("meas_T2", "measure T=2 s")]


def stops_from_source(t, src, min_move=0.05, still_s=0.5):
    """Indices of the stops: the first tick of each still period (>= still_s) that follows a move."""
    dt = np.gradient(t)
    sp = np.max(np.abs(np.gradient(src, t, axis=0)), axis=1)
    moving = sp > min_move
    out = []
    still_run = 0
    was_moving = False
    for k in range(len(t)):
        if moving[k]:
            was_moving = True
            still_run = 0
        else:
            still_run += 1
            if was_moving and still_run * np.median(dt) >= still_s:
                out.append(k - still_run + 1)
                was_moving = False
    return out


def pauses_from_reference(t, ref, still_s=1.0):
    """Indices where a pause starts: REF unchanged from this tick for at least still_s, after a change."""
    changed = np.r_[True, np.any(np.abs(np.diff(ref, axis=0)) > 0, axis=1)]
    out = []
    k = 1
    n = len(t)
    while k < n:
        if changed[k - 1] and not changed[k]:
            j = k
            while j < n and not changed[j]:
                j += 1
            if t[min(j, n - 1)] - t[k] >= still_s:
                out.append(k)
            k = j
        else:
            k += 1
    return out


def longest_run_s(t, flag):
    """Longest run (s) of consecutive ticks where flag is True."""
    best, start = 0.0, None
    for k, f in enumerate(flag):
        if f and start is None:
            start = k
        if start is not None and (not f or k == len(flag) - 1):
            end = k if f else k - 1
            best = max(best, float(t[end] - t[start]))
            start = None
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    root = Path(a.root)
    from experiments_loader import load
    from src.sag_layer import _PadGeometry
    from src.robot_helpers import arm_ids
    model, data = load()
    geo = _PadGeometry(model, "left")
    qadr = np.asarray(arm_ids(model, "left")["qadr"])

    def pad(q7):
        data.qpos[qadr] = q7
        mujoco.mj_forward(model, data)
        return geo.pad_pose(data)[0].copy()

    rows = {}
    for tag, label in RULES:
        r = {"five_1s": [], "five_dist": [], "five_peak": 0.0, "five_clip": 0, "B_1s": [], "B_dist": [], "B_peak": 0.0, "B_clip": 0,
             "T_1s": [], "T_dist": [], "T_peak": 0.0, "T_clip": 0, "nB": 0, "nT": 0}
        # five poses (static holds): the md lines
        md = root / f"static_{tag}.md"
        if md.exists():
            for line in md.read_text().splitlines():
                m = re.search(r"distance ([\d.]+) mm\); hand error 1 s after the stop ([\d.]+) mm; \|C_p\| at the end of the hold ([\d.]+) mm", line)
                if m:
                    r["five_dist"].append(float(m.group(1)))
                    r["five_1s"].append(float(m.group(2)))
            m2 = re.search(r"Peak \|C_p\| over the five holds: ([\d.]+) mm; ticks with \|C_p\| at the 40 mm clip: (\d+)", md.read_text())
            if m2:
                r["five_peak"] = float(m2.group(1))
                r["five_clip"] = int(m2.group(2))
        # policy B stops
        pb = root / f"policyB_{tag}" / "run.npz"
        if pb.exists():
            z = np.load(pb)
            t = z["left_t"]
            Q = z["left_q"][:, qadr]
            SRC = z["left_src"][:, :7]
            C = z["left_C_p"]
            scale = z["left_scale"]
            r["B_peak"] = 1e3 * float(np.max(np.linalg.norm(C, axis=1)))
            r["B_clip"] = int(np.sum((np.linalg.norm(C, axis=1) >= 0.04) | (scale < 1.0)))
            # stop check: the longest hold of the source (speed at or below the 0.05 rad/s move threshold); a stop needs 0.5 s
            sp = np.max(np.abs(np.gradient(SRC, t, axis=0)), axis=1)
            r["B_hold_s"] = longest_run_s(t, sp <= 0.05)
            for k in stops_from_source(t, SRC):
                k1 = min(k + int(round(1.0 / NOMINAL)), len(t) - 1)
                if k1 <= k or k1 - k < int(round(0.9 / NOMINAL)):
                    continue
                pt, pm = pad(SRC[k1]), pad(Q[k1])
                droop = pt - pm
                r["B_1s"].append(1e3 * float(np.linalg.norm(droop)))
                r["B_dist"].append(1e3 * float(np.linalg.norm(C[k] - droop)))
                r["nB"] += 1
        # teleop pauses (bridge on)
        tp = root / f"teleop_on_{tag}" / "run.npz"
        if tp.exists():
            z = np.load(tp)
            t = z["left_t"]
            Q = z["left_q"][:, qadr]
            REF = z["left_ref"][:, :7]
            C = z["left_C_p"]
            r["T_peak"] = 1e3 * float(np.max(np.linalg.norm(C, axis=1)))
            r["T_clip"] = int(np.sum(np.linalg.norm(C, axis=1) >= 0.04))
            # stop check: the longest unchanged stretch of the accepted reference (a pause needs 1 s)
            ch = np.r_[True, np.any(np.abs(np.diff(REF, axis=0)) > 0, axis=1)]
            r["T_hold_s"] = longest_run_s(t, ~ch)
            for k in pauses_from_reference(t, REF):
                # the 1 s sample must be a logged tick within 50 ms of 1 s after the stop; the teleop log has gaps (the fake operator loses
                # tracking and the arms go to manual, nothing is logged), so a pause with no such tick gets no hand error
                target = t[k] + 1.0
                k1 = int(np.searchsorted(t, target))
                if k1 >= len(t) or t[k1] - target > 0.05:
                    r["T_nosample"] = r.get("T_nosample", 0) + 1
                    continue
                pt, pm = pad(REF[k1]), pad(Q[k1])
                droop = pt - pm
                r["T_1s"].append(1e3 * float(np.linalg.norm(droop)))
                r["T_dist"].append(1e3 * float(np.linalg.norm(C[k] - droop)))
                r["nT"] += 1
        rows[label] = r

    def mean(x):
        return float(np.mean(x)) if x else float("nan")

    def cell(vals, n):
        """A mean with its event count; no events is written as none, not as nan."""
        return f"{mean(vals):.2f} ({n})" if n else "none (0)"

    def tcell(r, key):
        """The teleop cell: a pause that has no logged tick 1 s after it is not a measurement, so it is written as such."""
        if r["nT"] == 0 and r.get("T_nosample", 0):
            return f"no 1 s sample ({r['T_nosample']} pause)"
        return cell(r[key], r["nT"])

    lines = ["# Smoothing-time rescore (SF-126 decision 4): one table, production and each T (generated by experiments/rule_rescore.py)", ""]
    lines.append("| rule | a. hand error 1 s after the stop, five poses (mm) | a. policy B stops (mm, n) | a. teleop pauses (mm, n) | "
                 "b. offset vs at-rest droop at the stop, five poses (mm) | b. policy B stops (mm) | b. teleop pauses (mm) | "
                 "c. peak offset, policy B (mm) | c. ticks at clip, policy B | c. peak offset, teleop (mm) | c. ticks at clip, teleop | c. peak offset, five poses (mm) | c. ticks at clip, five poses |")
    lines.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for tag, label in RULES:
        r = rows[label]
        lines.append(f"| {label} | {mean(r['five_1s']):.2f} | {cell(r['B_1s'], r['nB'])} | {tcell(r, 'T_1s')} | "
                     f"{mean(r['five_dist']):.2f} | {cell(r['B_dist'], r['nB'])} | {tcell(r, 'T_dist')} | {r['B_peak']:.1f} | {r['B_clip']} | "
                     f"{r['T_peak']:.1f} | {r['T_clip']} | {r['five_peak']:.1f} | {r['five_clip']} |")
    lines += ["", "Stop check (why the policy B columns say none, and what the teleop pause is):", "",
              "| rule | policy B: longest hold of the source (s; a stop needs 0.5 s) | policy B stops | teleop: longest unchanged reference (s) | "
              "teleop pauses (>= 1 s) | of which with no logged tick 1 s after the stop |",
              "|---|---|---|---|---|---|"]
    for tag, label in RULES:
        r = rows[label]
        bh = r.get("B_hold_s", float("nan"))
        th = r.get("T_hold_s", float("nan"))
        lines.append(f"| {label} | {bh:.2f} | {r['nB']} | {th:.2f} | {r['nT'] + r.get('T_nosample', 0)} | {r.get('T_nosample', 0)} |")
    lines += ["", "Policy B's source is a continuous path (experiments/policy_stream_source.py, variant B): it never holds for 0.5 s, so it has no stops "
              "by construction and the B columns of (a) and (b) have no events.",
              "The teleop run's one pause (about 5.4 s) is the fake operator losing tracking; the arms hold, then the controller goes to manual and the "
              "log has no ticks for about 2.6 s. No logged tick falls 1 s after the stop, so the teleop (a) and (b) columns have no sample."]
    Path(a.out).write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
