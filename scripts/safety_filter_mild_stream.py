#!/usr/bin/env python
"""Stage 2c step 5: a milder command stream, rows 4, 5 and 6 of the Stage 2b comparison on it (measurement only).

The stream: each hand in turn moves in a straight line toward the OTHER hand's home position until the true gap between the arms
(our shapes, all left-arm x right-arm capsules, the left-right shoulder-mount pair left out as in Stage 2b's "ours_ns") is
STOP_GAP_M (4 cm) with the other arm at home, pauses PAUSE_S, and retreats to its start in BACK_S. The other hand starts half a period
later (period = out + pause + back). REPEATS cycles per hand. Raw commands: our own model's Jacobian (repeated single steps per
tick, as in Stage 2b), clipped to the joint limits. The script FAILS LOUDLY if the raw command's smallest arm-arm gap (our shapes) is
not above 0 on every tick. (Kong's capsules are bigger than our shapes, so the Kong-radii gap can still be negative; it is reported.)

Metrics per row as in Stage 2b: smallest true gap (Kong radii, our shapes), hold ticks and reasons, hand error mean and max, largest
joint jump per tick, filter time per tick, and whether any hold occurs.

Usage (venv active, repo root): bash experiments/run_capped.sh data/logs/<stamp>_mild_stream.log python scripts/safety_filter_mild_stream.py [--ticks N] [--rows 4,5,6] [--out PATH]
"""

import argparse
import datetime as dt
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import safety_filter_diag_common as dc  # noqa: E402

kc, cfg, fx, SIDES = dc.kc, dc.cfg, dc.fx, dc.SIDES
STOP_GAP_M = 0.04      # m: a hand moves toward the other until the gap between the arms (our shapes) is this (task text: 4 cm)
OUT_S = 0.5            # s: duration of the move out (min-jerk timing, as in Stage 2b); the task gives no number for it, same as Stage 2b's half period
PAUSE_S = 0.3          # s: pause at the end of the move (task text)
BACK_S = 0.5           # s: duration of the retreat (task text)
PERIOD_S = OUT_S + PAUSE_S + BACK_S   # s: one cycle of one hand; the other hand starts half of it later
REPEATS = 5            # count: cycles per hand (task text)
FS = kc.FS
S_GRID = 200           # count: grid steps along the line in the search for the 4 cm point
BISECT_STEPS = 24      # count: bisection steps that refine the 4 cm point


def gap_of(q):
    """Smallest left-right arm gap (m) of our shapes at one joint pose q (2, 7), shoulder-mount pair left out."""
    series, _ = kc.gap_series(q[None], None, SH)
    return float(series["ours_ns"][0])


def find_end(kin, q0, side, start, other_home, rot):
    """The point on the straight line start -> other_home where the arm gap first falls to STOP_GAP_M (other arm at home). Returns (s_end, point)."""
    i = SIDES.index(side)
    cur = q0.copy()
    prev_q, prev_s = cur.copy(), 0.0
    for n in range(1, S_GRID + 1):
        s = n / S_GRID
        kc.ik_tick(kin, cur, i, side, start + s * (other_home - start), rot, 200)
        g = gap_of(cur)
        if g <= STOP_GAP_M:
            lo, hi, q_lo = prev_s, s, prev_q.copy()
            for _ in range(BISECT_STEPS):
                mid = 0.5 * (lo + hi)
                c = q_lo.copy()
                kc.ik_tick(kin, c, i, side, start + mid * (other_home - start), rot, 200)
                if gap_of(c) > STOP_GAP_M:
                    lo, q_lo = mid, c
                else:
                    hi = mid
            return lo, start + lo * (other_home - start)
        prev_q, prev_s = cur.copy(), s
    raise RuntimeError(f"the {side} hand never reaches a {STOP_GAP_M} m gap on the line to the other hand")


def fraction(t, start):
    """0..1 of the way along the line at time t (s) for a hand whose first cycle begins at `start`: out, pause, back."""
    if t < start:
        return 0.0
    k = int(np.floor((t - start) / PERIOD_S + 1e-9))
    if k >= REPEATS:
        return 0.0
    u = t - start - k * PERIOD_S
    if u <= OUT_S:
        return float(kc.min_jerk(u / OUT_S))
    if u <= OUT_S + PAUSE_S:
        return 1.0
    return float(1.0 - kc.min_jerk((u - OUT_S - PAUSE_S) / BACK_S))


def build(setup):
    kin = setup.kin
    lo = {s: kin.limits[s][:, 0] for s in SIDES}
    hi = {s: kin.limits[s][:, 1] for s in SIDES}
    q0 = np.array(fx.HOME, float)
    home = kc.state_of(q0)
    start, rot = {}, {}
    for side in SIDES:
        start[side], rot[side] = kin.hand_pose(home, side)
    end, s_end = {}, {}
    for side, other in (("left", "right"), ("right", "left")):
        s_end[side], end[side] = find_end(kin, q0, side, start[side], start[other], rot[side])
        print(f"{side}: 4 cm point at {s_end[side]:.4f} of the way to the other hand's home position "
              f"({np.linalg.norm(end[side] - start[side]) * 1000:.1f} mm travel)", flush=True)
    n = int(round((REPEATS * PERIOD_S + PERIOD_S / 2) * FS)) + 1
    offset = {"left": 0.0, "right": PERIOD_S / 2}
    cur = q0.copy()
    q_ik = np.zeros((n, 2, 7))
    for k in range(n):
        t = k / FS
        for i, side in enumerate(SIDES):
            tgt = start[side] + fraction(t, offset[side]) * (end[side] - start[side])
            kc.ik_tick(kin, cur, i, side, tgt, rot[side], kc.IK_STEPS)
        q_ik[k] = cur
    raw = q_ik.copy()
    for i, side in enumerate(SIDES):
        raw[:, i, :] = np.clip(q_ik[:, i, :], lo[side], hi[side])
    return {"raw": raw, "q0": q0, "clipped_ticks": int((np.abs(raw - q_ik) > 0).any(axis=(1, 2)).sum()), "s_end": s_end,
            "travel_mm": {s: round(1000 * float(np.linalg.norm(end[s] - start[s])), 1) for s in SIDES}}


SH = None


def main(argv=None):
    global SH
    ap = argparse.ArgumentParser()
    ap.add_argument("--ticks", type=int, default=None)
    ap.add_argument("--rows", default="4,5,6")
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)
    t0 = time.time()
    setup = dc.Setup()
    SH = setup.ours
    stream = build(setup)
    raw_all = stream["raw"]
    n = len(raw_all) if args.ticks is None else min(args.ticks, len(raw_all))
    raw = raw_all[:n]
    kong_metric = kc.KongShapes(setup.model, setup.path, setup.tool_local)
    series, best = kc.gap_series(raw, kong_metric, setup.ours)
    print(f"raw command: smallest arm gap our shapes (no shoulder pair) {1000 * series['ours_ns'].min():.2f} mm at tick "
          f"{int(series['ours_ns'].argmin())}; Kong radii {1000 * series['kong'].min():.2f} mm; ticks {n}; clipped ticks {stream['clipped_ticks']}", flush=True)
    if not series["ours_ns"].min() > 0.0:
        raise RuntimeError(f"the raw command collides (our shapes): smallest gap {1000 * series['ours_ns'].min():.2f} mm at tick "
                           f"{int(series['ours_ns'].argmin())}")
    masks = {"kong_radii": series["kong"] > kc.NO_CONTACT_GAP_M, "our_shapes": series["ours_ns"] > kc.NO_CONTACT_GAP_M}
    doc = {"ticks": n, "params": {"stop_gap_m": STOP_GAP_M, "out_s": OUT_S, "pause_s": PAUSE_S, "back_s": BACK_S, "period_s": PERIOD_S,
                                  "repeats": REPEATS}, "stream": {k: v for k, v in stream.items() if k not in ("raw", "q0")},
           "raw_min_gap_our_shapes_no_shoulder_mm": round(1000 * float(series["ours_ns"].min()), 3),
           "raw_min_gap_kong_radii_mm": round(1000 * float(series["kong"].min()), 3), "rows": {}}
    for row in [int(r) for r in args.rows.split(",")]:
        name, mode, gap_mode, marg, shp = dc.ROWS[row]
        required = "kong_radii" if shp == "kong" else "our_shapes"
        measure = "kong_radii" if required == "kong_radii" else "our_shapes_no_shoulder_mount"
        settings = {"resolver_mode": mode, "gap_rule_mode": gap_mode, "shapes": required}
        print(f"row {row}: {name}", flush=True)
        with setup.row_settings(row):
            out, times, extra = kc.run_resolver(setup.make_resolver(row), raw, stream["q0"])
        r = kc.summarise(name, required, setup.kin, raw, out, times, extra, stream["q0"], kong_metric, setup.ours, settings, (measure, masks[required]))
        doc["rows"][name] = r
        print(json.dumps({k: r[k] for k in ("hold_ticks", "status_counts", "hold_reason_counts", "min_gap_kong_radii_mm", "min_gap_our_shapes_no_shoulder_mount_mm",
                                            "hand_pos_err_mean_mm", "hand_pos_err_max_mm", "max_joint_jump_rad", "time_ms_mean", "time_ms_max")}), flush=True)
    doc["runtime_s"] = round(time.time() - t0, 1)
    path = Path(args.out) if args.out else dc.ROOT / "data" / "safety_filter" / f"mild_stream_{dt.datetime.now():%Y%m%d_%H%M%S}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(doc, indent=1, default=str))
    json.loads(tmp.read_text())
    os.replace(tmp, path)
    print(f"wrote {path} (runtime {doc['runtime_s']} s)", flush=True)


if __name__ == "__main__":
    main()
