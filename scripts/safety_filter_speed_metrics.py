#!/usr/bin/env python
"""Closed-loop summary of a speed-work recording, measured with the code of a GIVEN source tree (Stage 5 speed; measurement only).

The equivalence gate runs this with --root pointing at the tagged reference tree (the code before any speed change), so the numbers
that judge a changed filter are never computed by the changed code itself.

Summary: status and hold-reason counts, the smallest true gap at the filtered command by pair type (never-close list OFF, so every
pair the touch rules check is measured), the hand position error between the raw and the filtered command (mean and max over ticks
and arms), the largest joint change per tick (filtered command against the last filtered command), and the time per call.

Usage: python scripts/safety_filter_speed_metrics.py --root <source tree> <recording.npz> --out <summary.json>
"""

import argparse
import sys
from collections import Counter
from pathlib import Path

import numpy as np

PAIR_TYPES = ("arm_vs_arm", "arm_vs_body", "arm_vs_world", "arm_vs_never_touch", "held_object")


def pair_type(a, b):
    if a.part == "held" or b.part == "held":
        return "held_object"
    if a.arm == "world" or b.arm == "world":
        return "arm_vs_never_touch" if (a.never_touch or b.never_touch) else "arm_vs_world"
    if a.arm in ("left", "right") and b.arm in ("left", "right"):
        return "arm_vs_arm"
    return "arm_vs_body"


def summarise(rec, root, say=print, every=1000):
    from src.safety_filter import config as sfcfg
    from src.safety_filter.robot_model import load_robot_only_model
    from src.safety_filter.shapes import RobotShapes
    from src.safety_filter.touch import as_placed, evaluate_pairs
    shapes = RobotShapes(load_robot_only_model(), Path(root) / sfcfg.SHAPES_FILE, use_never_close=False)
    a = rec.a
    n = rec.n
    best = {t: [np.inf, -1, ""] for t in PAIR_TYPES}
    hand_err = np.zeros((n, 2))
    for k in range(n):
        world = rec.world(k)
        held = rec.held(k)
        shapes.set_state(rec.state(k, a["raw"][k]))
        p_raw = [shapes.hand_frame(s)[0] for s in ("left", "right")]
        placed = list(shapes.place(rec.state(k, a["filtered"][k])))
        for i, s in enumerate(("left", "right")):
            pos, rot = shapes.hand_frame(s)
            hand_err[k, i] = float(np.linalg.norm(pos - p_raw[i]))
            if held is not None:
                hs = held.placed(s, pos, rot)
                if hs is not None:
                    placed.append(hs)
        by_id = {s.id: s for s in placed}
        by_id.update({o.id: as_placed(o) for o in world})
        for rep in evaluate_pairs(placed, world):
            ty = pair_type(by_id[rep.a], by_id[rep.b])
            if rep.gap < best[ty][0]:
                best[ty] = [float(rep.gap), k, f"{rep.a}|{rep.b}"]
        if every and (k + 1) % every == 0:
            say(f"metrics tick {k + 1}/{n}")
    t = np.asarray(a["filter_time"], float) * 1e3
    status = Counter(a["status"].tolist())
    reasons = Counter(r for r in a["reason"].tolist() if r)
    gaps = {ty: (None if not np.isfinite(v[0]) else {"gap_mm": round(v[0] * 1e3, 4), "tick": v[1], "pair": v[2]}) for ty, v in best.items()}
    finite = [v[0] for v in best.values() if np.isfinite(v[0])]
    return {
        "stream": rec.stream, "mode": rec.mode, "ticks": n, "dt": rec.dt,
        "status_counts": dict(sorted(status.items())), "hold_ticks": int(status.get("HOLD", 0)), "hold_reason_counts": dict(sorted(reasons.items())),
        "min_true_gap_mm": (round(min(finite) * 1e3, 4) if finite else None), "min_true_gap_by_type": gaps,
        "hand_pos_err_mean_mm": round(float(hand_err.mean()) * 1e3, 5) if n else 0.0,
        "hand_pos_err_max_mm": round(float(hand_err.max()) * 1e3, 5) if n else 0.0,
        "max_joint_change_per_tick_rad": round(float(np.abs(a["filtered"] - a["last"]).max()), 7) if n else 0.0,
        "time_ms": ({"mean": round(float(t.mean()), 3), "median": round(float(np.median(t)), 3), "p95": round(float(np.percentile(t, 95)), 3),
                     "max": round(float(t.max()), 3)} if n else {}),
        "time_ms_not_pass": ({"mean": round(float(t[a["status"] != "PASS"].mean()), 3), "n": int((a["status"] != "PASS").sum())}
                             if n and (a["status"] != "PASS").any() else {}),
        "time_ms_pass": ({"mean": round(float(t[a["status"] == "PASS"].mean()), 3), "n": int((a["status"] == "PASS").sum())}
                         if n and (a["status"] == "PASS").any() else {}),
        "pairs_pushed_total": int(a["pairs_pushed"].sum()), "pairs_skipped_total": int(a["pairs_skipped"].sum()),
        "pairs_shortened_total": int(a["pairs_shortened"].sum()), "solver_calls_total": int(a["solver_calls"].sum()),
        "measured_with_source_tree": str(root),
    }


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("recording")
    ap.add_argument("--root", required=True, help="the source tree whose src/ does the measuring (the tagged reference tree for the gate)")
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    root = str(Path(args.root).resolve())
    sys.path.insert(0, root)                       # before the working tree: src is imported from here
    import safety_filter_speed_util as sc         # from scripts/ (the script's own folder); imports nothing from src by itself
    import src
    if not str(Path(src.__file__).resolve()).startswith(root):
        raise RuntimeError(f"src was imported from {src.__file__}, not from {root}")
    doc = summarise(sc.Recording(args.recording), root)
    sc.write_json_atomic(args.out, doc)
    print(f"wrote {args.out}: status {doc['status_counts']}, min true gap {doc['min_true_gap_mm']} mm, hand error mean {doc['hand_pos_err_mean_mm']} max {doc['hand_pos_err_max_mm']} mm", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
