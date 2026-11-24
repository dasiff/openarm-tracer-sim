"""Tables for the stage 2 report (David d0bc 2026-11-24): the policy A-D runs (bridge off, correction on and off), the teleop runs logged on every
tick, the five static holds, the held runs and the timing. Analysis only: reads the saved folders, loads no simulator.

    python experiments/stage2_tables.py --root data/logs/rebuild_s2 --out data/logs/rebuild_s2/analysis/stage2_tables.md

The clip is the production hand clip: 40 mm position, 100 mrad rotation (PICK_PLACE_CFG sag_max_m, sag_max_rad). A tick is at the clip when the
hand move of the offset reaches it (sag_timing_probe and policy_stream_test count those ticks).
"""

import argparse
import json
import re
from pathlib import Path

import numpy as np


def policy_rows(root):
    rows = []
    for v in "ABCD":
        for s in (1, 2, 3):
            for c in ("on", "off"):
                jf = root / "policy" / f"{v}_s{s}_{c}_nobridge.json"
                tf = root / "policy" / f"{v}_s{s}_{c}_bridgeoff_timing.json"
                if not jf.exists():
                    rows.append((v, s, c, None, None))
                    continue
                d = json.loads(jf.read_text())
                t = json.loads(tf.read_text()) if tf.exists() else None
                rows.append((v, s, c, d, t))
    return rows


def policy_table(root):
    lines = ["## Policy A-D, bridge off, correction on and off (30 s, 3 seeds)", "",
             "| variant | seed | correction | offset peak (mm) | offset peak rot (mrad) | ticks at position clip | ticks at rotation clip | "
             "hand error mean (mm) | hand error mean, last 2 s (mm) | rot error mean (mrad) | max commanded joint speed (rad/s) | exit |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for v, s, c, d, _ in policy_rows(root):
        if d is None:
            lines.append(f"| {v} | {s} | {c} | not run | | | | | | | | |")
            continue
        lines.append(f"| {v} | {s} | {c} | {1000 * d['position']['peak_m']:.2f} | {1000 * d['rotation']['peak_rad']:.2f} | "
                     f"{d['position']['ticks_at_clip']} | {d['rotation']['ticks_at_clip']} | {1000 * d['hand_error_m']['mean']:.2f} | "
                     f"{1000 * d['hand_error_m']['mean_last_2s']:.2f} | {1000 * d['hand_error_m']['mean_rot_rad']:.2f} | "
                     f"{d.get('max_commanded_joint_speed_rad_s', float('nan')):.2f} | {d.get('stopped_by')} |")
    lines.append("")
    lines += ["### Timing per call (ms), policy runs", "", "| run | sag layer mean | sag layer p99 | bridge mean | bridge p99 | bridge p99 sum |",
              "|---|---|---|---|---|---|"]
    for v, s, c, d, t in policy_rows(root):
        if t is None:
            continue
        sg, br = t["sag_layer_call_ms"], t["bridge_call_ms"]
        lines.append(f"| {v} s{s} {c} | {sg['mean']:.3f} | {sg['p99']:.3f} | {br['mean']:.3f} | {br['p99']:.3f} | {sg['p99'] + br['p99']:.3f} |")
    lines.append("")
    return lines


def teleop_table(root):
    lines = ["## Teleop, bridge on, correction on and off (every tick logged, through the tracking loss)", ""]
    for c in ("on", "off"):
        f = root / "teleop" / c / "run.npz"
        if not f.exists():
            lines.append(f"- correction {c}: not run")
            continue
        z = np.load(f, allow_pickle=True)
        t = z["t"]
        Cp = z["C_p"]
        Cr = z["C_r"]
        err = z["err_m"]
        ref = z["ref"]
        cp_norm = np.linalg.norm(Cp, axis=1)
        cr_norm = np.linalg.norm(Cr, axis=1)
        lines.append(f"### correction {c}: {len(t)} ticks, {t[0]:.2f} s to {t[-1]:.2f} s")
        lines.append("")
        lines.append(f"- offset peak |C_p| {1000 * cp_norm.max():.2f} mm; ticks at the 40 mm clip {int(np.sum(cp_norm >= 0.04 * (1 - 1e-9)))}; "
                     f"peak |C_r| {1000 * cr_norm.max():.2f} mrad; ticks at the 100 mrad clip {int(np.sum(cr_norm >= 0.1 * (1 - 1e-9)))}")
        # pauses: the bridge-accepted reference (REF) unchanged for at least 1 s on consecutive logged ticks
        # a tick with no bridge reference (NaN: no arm command has gone through the bridge yet) is not a pause: NaN compares as "unchanged" otherwise
        finite = np.all(np.isfinite(ref), axis=1)
        changed = np.r_[True, np.any(np.abs(np.diff(ref, axis=0)) > 0, axis=1)] | ~finite
        pauses = []
        i = 1
        while i < len(t):
            if changed[i - 1] and not changed[i]:
                j = i
                while j < len(t) and not changed[j]:
                    j += 1
                if t[min(j, len(t) - 1)] - t[i] >= 1.0 and finite[i]:
                    pauses.append(i)
                i = j
            else:
                i += 1
        lines.append(f"- pauses of at least 1 s (REF unchanged): {len(pauses)}")
        for k in pauses:
            k1 = int(np.searchsorted(t, t[k] + 1.0))
            k1 = min(k1, len(t) - 1)
            e1 = np.linalg.norm(err[k1]) if np.all(np.isfinite(err[k1])) else float("nan")
            lines.append(f"  - pause from {t[k]:.2f} s: |C_p| at the stop {1000 * cp_norm[k]:.2f} mm; hand error 1 s after the stop "
                         f"(logged tick at {t[k1]:.2f} s) {1000 * e1:.2f} mm")
        lines.append("")
    return lines


def static_table(root):
    lines = ["## Five static holds, deliberate model error, correction on (home reported separately)", ""]
    md = root / "static" / "perturbed.md"
    if not md.exists():
        return lines + ["not run", ""]
    text = md.read_text()
    stop_lines = [ln for ln in text.splitlines() if ln.startswith("- ") and "hand error 1 s after the stop" in ln]
    lines.append("| pose | offset vs droop at the stop (mm) | hand error 1 s after the stop (mm) | |C_p| at the end of the hold (mm) |")
    lines.append("|---|---|---|---|")
    for ln in stop_lines:
        name = ln[2:].split(":")[0]
        m = re.search(r"distance ([\d.]+) mm\); hand error 1 s after the stop ([\d.]+) mm; \|C_p\| at the end of the hold ([\d.]+) mm", ln)
        if m:
            lines.append(f"| {name} | {m.group(1)} | {m.group(2)} | {m.group(3)} |")
    m2 = re.search(r"Peak \|C_p\| over the five holds: ([\d.]+) mm; ticks with \|C_p\| at the 40 mm clip: (\d+)", text)
    if m2:
        lines.append("")
        lines.append(f"Peak |C_p| over the five holds {m2.group(1)} mm; ticks at the 40 mm clip {m2.group(2)}.")
    lines.append("")
    return lines


def held_table(root):
    lines = ["## Held run (seed 4294874296, --liquid 0.100), production layer", "", "| run | outcome | sag mean (ms) | sag p99 (ms) | bridge p99 (ms) |",
             "|---|---|---|---|---|"]
    for name, log, tim in (("nominal", root / "final" / "held_nominal.log", root / "final" / "held_nominal" / "held_timing.json"),
                           ("perturbed", root / "final" / "held_perturbed.log", None)):
        out = "not run"
        if log.exists():
            m = re.search(r"outcome (.*)", log.read_text())
            out = m.group(1).strip() if m else "no outcome line"
        sg = br = "-"
        if tim and tim.exists():
            t = json.loads(tim.read_text())
            sg = f"{t['sag_layer_call_ms']['mean']:.3f} | {t['sag_layer_call_ms']['p99']:.3f} | {t['bridge_call_ms']['p99']:.3f}"
        else:
            sg = "- | - | -"
        lines.append(f"| {name} | {out} | {sg} |")
    lines.append("")
    return lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    root = Path(a.root)
    lines = ["# Stage 2 tables (generated by experiments/stage2_tables.py)", ""]
    lines += policy_table(root)
    lines += teleop_table(root)
    lines += static_table(root)
    lines += held_table(root)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text("\n".join(lines) + "\n")
    print(f"stage2_tables: wrote {a.out}")


if __name__ == "__main__":
    main()
