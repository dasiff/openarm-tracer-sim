"""Stage 2g steps 5 and 6: a reactive test stream, and a runner that measures rows 5 and 6 on the original and on the reactive stream.

TEST ONLY (decision row SF-57): this is not part of the filter and the filter never sees it.

The original stream (scripts/safety_filter_kong_comparison.py build_stream) is open loop: when the filter holds an arm still, the raw
command keeps running away from it (the "lag" of Stage 2c). The reactive stream closes the loop: the same scripted rolling punch path
gives the TARGET pose of each tick (the original stream's raw command of that tick, with its joint-limit clipping), and the raw command
sent to the filter is the arm's ACTUAL previous filtered command moved toward that target by at most REACTIVE_MAX_STEP_RAD per joint per
tick, then clipped to the joint limits as the original stream is. A held arm therefore does not fall behind a runaway command.

Rows (Stage 2b comparison, scripts/safety_filter_diag_common.py ROWS):
  5  kong_style, gap_rule paper_plain, margins zero, Kong radii arm-vs-arm only
  6  kong_style, full gap rule, our shapes and margins
Metrics per run: hold ticks and share, hold reasons, hand position error (mean, max), largest joint jump per tick, smallest true gap
(Kong radii for row 5, our shapes for row 6), filter time per tick, solver calls per tick, pairs pushed and skipped per tick, joint-limit
overshoot at intermediate poses (new filter only), and the status counts. The solver calls are counted from outside by wrapping
kin.convert_directions, so the same script runs on the old (summing) code for the before numbers.

Usage (one MuJoCo process at a time; send output to a log):
  python scripts/safety_filter_reactive_stream.py --row 6 --stream reactive --ticks 400
  python scripts/safety_filter_reactive_stream.py --row 5 --stream original            (full 1576 ticks)
Output: data/safety_filter/seqpush_row<row>_<stream>_<ticks>.json (+ .npz with the per-tick arrays), written through a temporary file.
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
import safety_filter_diag_common as dc  # noqa: E402
from src.safety_filter.shapes import RobotState  # noqa: E402

kc = dc.kc
cfg = dc.cfg
SIDES = dc.SIDES
REACTIVE_MAX_STEP_RAD = 0.018    # rad per joint per tick: the original stream's largest per-tick joint step (printed by the run as a check) [SF-57]


def reactive_command(target, prev_out, lo, hi, max_step=REACTIVE_MAX_STEP_RAD):
    """The raw command of one tick: prev_out (the arm's actual previous filtered command, (2, 7)) moved toward target (2, 7) by at most
    max_step per joint, clipped to the joint limits lo, hi ((2, 7))."""
    step = np.clip(np.asarray(target, float) - np.asarray(prev_out, float), -max_step, max_step)
    return np.clip(np.asarray(prev_out, float) + step, lo, hi)


def limits_arrays(kin):
    lo = np.array([kin.limits[s][:, 0] for s in SIDES])
    hi = np.array([kin.limits[s][:, 1] for s in SIDES])
    return lo, hi


def run_stream(setup, row, kind, n):
    """Runs rows 5 or 6 on the 'original' or 'reactive' stream for n ticks. Returns (summary dict, arrays dict)."""
    kin = setup.kin
    target = np.asarray(setup.raw[:n], float)
    q0 = np.asarray(setup.q0, float)
    lo, hi = limits_arrays(kin)
    steps = np.abs(np.diff(np.concatenate([q0[None], target], axis=0), axis=0))
    print(f"original stream: largest per-tick joint step {steps.max():.5f} rad (reactive limit {REACTIVE_MAX_STEP_RAD})", flush=True)
    out = np.zeros_like(target)
    raw = np.zeros_like(target)
    times = np.zeros(n)
    calls = np.zeros(n, dtype=int)
    pushed = np.zeros(n, dtype=int)
    skipped = np.zeros(n, dtype=int)
    sweeps = np.zeros(n, dtype=int)
    overshoot = np.zeros(n)
    jchange = np.zeros(n)
    statuses, reasons = [], []
    base = RobotState()
    with setup.row_settings(row):
        res = setup.make_resolver(row)
        counter = [0]
        orig = res.kin.convert_directions

        def counted(*a, **k):
            counter[0] += 1
            return orig(*a, **k)

        res.kin.convert_directions = counted
        try:
            last = q0.copy()
            for k in range(n):
                raw[k] = target[k] if kind == "original" else reactive_command(target[k], last, lo, hi)
                counter[0] = 0
                t0 = time.perf_counter()
                o, st = res.step(raw[k].copy(), last.copy(), base)
                times[k] = time.perf_counter() - t0
                out[k] = np.asarray(o, float).reshape(2, 7)
                calls[k] = counter[0]
                pushed[k] = getattr(st, "pairs_pushed", 0)
                skipped[k] = getattr(st, "pairs_skipped", 0)
                sweeps[k] = st.rounds
                overshoot[k] = getattr(st, "max_limit_overshoot_rad", 0.0)
                jchange[k] = getattr(st, "max_joint_change_rad", 0.0)
                statuses.append(kc._label(st.status))
                reasons.append(kc._label(getattr(st, "hold_reason", None)))
                last = out[k].copy()
                if k % 100 == 0:
                    print(f"row {row} {kind} tick {k}/{n} hold so far {sum(s == 'HOLD' for s in statuses)}", flush=True)
        finally:
            res.kin.__dict__.pop("convert_directions", None)
    required = "kong_radii" if dc.ROWS[row][4] == "kong" else "our_shapes"
    kong_metric = kc.KongShapes(setup.model, setup.path, setup.tool_local)   # the smallest Kong-radii gap is reported for both rows
    series, _ = kc.gap_series(raw, kong_metric, setup.ours)
    mask = series["kong"] > kc.NO_CONTACT_GAP_M if required == "kong_radii" else series["ours_ns"] > kc.NO_CONTACT_GAP_M
    measure = "kong_radii" if required == "kong_radii" else "our_shapes_no_shoulder_mount"
    settings = {"row": row, "mode": dc.ROWS[row][1], "gap_rule": dc.ROWS[row][2], "pair_order": cfg.RESOLVER.get("pair_order"),
                "stream": kind, "ticks": n, "reactive_max_step_rad": REACTIVE_MAX_STEP_RAD if kind == "reactive" else None}
    row_doc = kc.summarise(dc.ROWS[row][0], required, kin, raw, out, times, {"status": statuses, "hold_reason": reasons}, q0,
                           kong_metric, setup.ours, settings, (measure, mask))
    hold = np.array([s == "HOLD" for s in statuses])
    pushed_ticks = pushed > 0
    row_doc.update({
        "hold_share": round(float(hold.mean()), 5),
        "solver_calls_mean": round(float(calls.mean()), 3), "solver_calls_max": int(calls.max()),
        "pairs_pushed_mean": round(float(pushed.mean()), 3), "pairs_pushed_max": int(pushed.max()),
        "pairs_skipped_mean": round(float(skipped.mean()), 3), "pairs_skipped_max": int(skipped.max()),
        "sweeps_mean": round(float(sweeps.mean()), 3), "sweeps_max": int(sweeps.max()),
        "overshoot_ticks": int((overshoot > 0).sum()), "overshoot_max_rad": round(float(overshoot.max()), 5),
        "intermediate_joint_change_max_rad": round(float(jchange.max()), 5),
        "ticks_with_a_push": int(pushed_ticks.sum()),
        "first_hold_tick": (int(np.argmax(hold)) if hold.any() else None),
        "raw_vs_out_joint_gap_max_rad": round(float(np.abs(raw - out).max()), 5)})
    arrays = {"out": out, "raw": raw, "times": times, "calls": calls, "pushed": pushed, "skipped": skipped, "sweeps": sweeps,
              "overshoot": overshoot, "status": np.array(statuses), "reason": np.array(reasons)}
    return row_doc, arrays


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--row", type=int, choices=(5, 6), required=True)
    ap.add_argument("--stream", choices=("original", "reactive"), required=True)
    ap.add_argument("--ticks", type=int, default=0, help="first N ticks (0 = the whole stream, 1576)")
    ap.add_argument("--tag", default="", help="text added to the output file name (for example 'old' for the summing code)")
    args = ap.parse_args(argv)
    setup = dc.Setup(max_ticks=args.ticks or 10 ** 6, full_stream=not args.ticks)
    n = setup.n
    t0 = time.time()
    doc, arrays = run_stream(setup, args.row, args.stream, n)
    doc["runtime_s"] = round(time.time() - t0, 1)
    base = ROOT / "data" / "safety_filter" / f"seqpush{args.tag}_row{args.row}_{args.stream}_{n}"
    base.parent.mkdir(parents=True, exist_ok=True)
    tmp = str(base) + ".json.tmp"
    with open(tmp, "w") as f:
        json.dump(doc, f, indent=1)
    os.replace(tmp, str(base) + ".json")
    tmpz = str(base) + ".tmp.npz"
    np.savez(tmpz, **arrays)
    os.replace(tmpz, str(base) + ".npz")
    keys = ("hold_ticks", "hold_share", "hold_reason_counts", "hand_pos_err_mean_mm", "hand_pos_err_max_mm", "max_joint_jump_rad",
            "min_gap_kong_radii_mm", "min_gap_our_shapes_mm", "time_ms_mean", "time_ms_max", "solver_calls_mean", "solver_calls_max",
            "pairs_pushed_mean", "pairs_skipped_mean", "sweeps_max", "overshoot_ticks", "overshoot_max_rad", "status_counts")
    print(json.dumps({k: doc[k] for k in keys}, indent=1))
    print(f"wrote {base}.json (runtime {doc['runtime_s']} s)", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
