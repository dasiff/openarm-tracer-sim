"""Bounce metrics for the three runs of the held-mass ramp experiment (decisions-log M215 to M223). Read only.

    python experiments/bounce_ramp_metrics.py <out.md>

Definitions (approved by David):
  baseline = hand height (left pad centre, mm) on the tick before the held mass first becomes nonzero (T0 - 1)
  rise     = highest height above the baseline, over the window T0 .. T0 + 200 ticks (about 1.3 s)
  drop     = lowest height relative to the baseline, over the same window
  swing    = rise minus drop
  Each run is aligned on its own T0.

Also reported: the pad height at the end of DOWN (relative to the baseline), where the beaker ends up (the final episode row), and the place time
(from the start of PLACE: OVER to the start of PLACE: BACK, from the episode log).

Runs: the two existing runs read from their per-tick extracts (height_20261101), and the new run from its live log (bounce_ramp_run.py).
The extracts are replays of the same scenario; their held mass is the old one-step value (held_mass_ticks).
"""

import csv
import sys

import numpy as np
import pandas as pd

WINDOW = 200
RUNS = [
    ("sag on (existing)", "extract", "data/diag/height_20261101/extract_on.csv",
     "data/closed_loop/on_20261029_1453/episode/episode_log.csv"),
    ("sag off in place only (existing)", "extract", "data/diag/height_20261101/extract_nosag.csv",
     "data/diag/nosag_20261101/nosag_20261101_173149/episode/episode_log.csv"),
    ("new: ramp, sag off everywhere", "new", "data/bounce_ramp_20261102/run_new/bounce_ticks.csv",
     "data/bounce_ramp_20261102/run_new/episode/episode_log.csv"),
]


def load_track(kind, path):
    """(z in mm, held mass in kg, target in kg or None, phase or None), each indexed by tick."""
    df = pd.read_csv(path, index_col="tick")
    if kind == "extract":
        return df["padz_mm_rel"], df["held_mass_ticks"].fillna(0.0), None, df["phase"]
    return 1000.0 * df["pad_z_m"], df["mass_kg"], df["target_kg"], None


def place_time(path):
    """Seconds from the first PLACE: OVER row to the first PLACE: BACK row of the episode log; None if the place is not in the log."""
    rows = list(csv.DictReader(open(path)))
    t_over = next((float(r["sim_time"]) for r in rows if "PLACE: OVER" in r["phase"]), None)
    t_back = next((float(r["sim_time"]) for r in rows if "PLACE: BACK" in r["phase"]), None)
    return (t_back - t_over) if t_over is not None and t_back is not None else None, rows


def main():
    out = sys.argv[1] if len(sys.argv) == 2 else None
    lines = []
    results = []
    for name, kind, track_path, episode_path in RUNS:
        z, mass, target, phase = load_track(kind, track_path)
        nz = mass[mass > 0]
        if nz.empty:
            print(f"{name}: no held mass in the log (STOP for this run)")
            continue
        T0 = int(nz.index[0])
        base = float(z.loc[T0 - 1])
        win = z.loc[T0:T0 + WINDOW] - base
        rise, drop = float(win.max()), float(win.min())
        if kind == "extract":
            # the old runs: the end of DOWN is the last tick labelled DOWN in the window before OPEN
            labels = phase.loc[T0:T0 + WINDOW]
            print(f"{name}: phase labels in window: {sorted(set(labels.dropna()))}")
            down = [t for t in labels.index if "DOWN" in str(labels.loc[t])]
            t_down = max(down) if down else None
        else:
            # the new run: the target goes to 0 at the end of DOWN (M221), the first such tick after T0
            after = target.loc[T0 + 1:]
            zero = after[after == 0.0]
            t_down = int(zero.index[0]) if len(zero) else None
        down_mm = float(z.loc[t_down] - base) if t_down is not None else float("nan")
        ptime, rows = place_time(episode_path)
        final = rows[-1]
        results.append(dict(run=name, T0=T0, baseline_mm=base, rise_mm=rise, drop_mm=drop, swing_mm=rise - drop, t_down=t_down,
                            pad_at_down_end_mm=down_mm, place_s=ptime, beaker=(float(final["beaker_x"]), float(final["beaker_y"]), float(final["beaker_z"]))))

    lines.append("| run | first held tick T0 | baseline (mm) | rise (mm) | drop (mm) | swing (mm) | pad at end of DOWN vs baseline (mm) | place time (s) | final beaker x, y, z (m) |")
    lines.append("|---|---|---|---|---|---|---|---|---|")
    for r in results:
        bx, by, bz = r["beaker"]
        lines.append(f"| {r['run']} | {r['T0']} | {r['baseline_mm']:.2f} | {r['rise_mm']:+.2f} | {r['drop_mm']:+.2f} | {r['swing_mm']:.2f} | "
                     f"{r['pad_at_down_end_mm']:+.2f} | {r['place_s']:.2f} | {bx:.4f}, {by:.4f}, {bz:.4f} |")
    text = "\n".join(lines)
    print(text)
    if out:
        with open(out, "w") as fh:
            fh.write(text + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
