"""Stance selection by weighted random sampling from the A8 Gaussian reach model (decided by David, 2026-11-10; decisions-log M299).

What the pipeline runs (src/stance_select.apply_stance_selection calls only this module):
  1. Draw a batch of N stances, uniformly and continuously: x, y in the rectangle around the task's objects grown by the A8 margin; the heading uniformly within
     sampler_heading_half_width_deg of the direction from the stance's (x, y) to the midpoint of the task points (Revision 2, 2026-11-10: the robot faces the objects from
     wherever it stands); the pedestal in the pedestal range. While sampler_heading_held is True the heading is held at TASK_CFG's staging_yaw (-90 deg) instead, because
     the staging and navigation are not checked at other headings (report: docs/reports/gaussian_sampling_selection_20261110.md).
  2. Score each stance with the A8 count (src/reach_model.predict_count_asym, the same formula the fit used). The shoulder position comes from the stance; each
     task point (the pick's grasp point, the place's set-down point) gives one count; the stance's score is the LOWER of its counts (the worst point).
  3. Weight = score ** weight power. Draw one stance with probability proportional to its weight.
  4. Run the full check on that stance with its values unchanged. Stop at the first pass. A failed stance is not put back; at most `sampler_max_draws` checks run.
  5. The sampling seed comes from the clock and is saved in the record, so the batch and every draw can be reproduced from it.

The arm used for scoring is BENCH_STANCE_CFG["arm"] (left). The right arm would use the mirrored model (reach_model.arm_params).
"""
import math
import time

import numpy as np

from src.reach_model import arm_params, shoulder_height
from src.reach_model import predict_count_asym, A8


def margin_m(cfg_sigmas):
    """The sampling rectangle's growth on every side: the A8 radial centre plus `cfg_sigmas` radial widths (r0 + k * sr), metres."""
    return A8["r0"] + cfg_sigmas * A8["sr"]


def _turn(h, x, y):
    """The vector (x, y) turned by the angle h (radians), elementwise for arrays."""
    c, s = np.cos(h), np.sin(h)
    return c * x - s * y, s * x + c * y


def a8_scores(xs, ys, qs, heading, points, geom, arm):
    """The A8 score of each stance: the lower, over the task points, of the A8 count of that point.

    Args:
        xs, ys, qs (numpy.ndarray): The stances' base x, y (m) and pedestal commands (m), same length N.
        heading (float or numpy.ndarray): The torso's +x direction in the world (radians): one value, or one per stance (N,).
        points (list of (x, y, z)): The task points in the world (m): each object's grasp point (pick) or set-down point (place).
        geom (dict): robot_geometry(model) (src/stance_finder.py): "shoulder", "torso_z0", "lift_factor", "base_z".
        arm (str): "left" or "right".

    Returns:
        numpy.ndarray: (N,) the score of each stance, in [0, 12] (approach bins).
    """
    p, z_cut = arm_params(arm)
    sx, sy, _ = geom["shoulder"][arm]
    heading = np.broadcast_to(np.asarray(heading, float), np.shape(xs))
    ox, oy = _turn(heading, sx, sy)
    Sx, Sy = xs + ox, ys + oy                                    # the shoulder's world x, y, per stance
    Sz = shoulder_height(geom, arm, qs)                         # the shoulder's world height, per stance
    counts = []
    for px, py, pz in points:
        xl, yl = _turn(-heading, px - Sx, py - Sy)               # the point in the shoulder frame (x forward, y left)
        z = pz - Sz
        r = np.hypot(np.hypot(xl, yl), z)                        # 3D distance from the shoulder
        counts.append(predict_count_asym(r, np.arctan2(yl, xl), z, p, z_cut))
    return np.min(np.stack(counts), axis=0)


def task_points(targets):
    """The (x, y, z) points the score uses, one per manipulation target: the mean height of its grasp points (pick) or of its set-down points (place).

    Args:
        targets (list of dict): The overlay-style objects of stance_select.step_target ("xy", "points" with "path" and "p"), in step order.

    Returns:
        list of (x, y, z): floats, metres.
    """
    out = []
    for ob in targets:
        pts = ob["points"]
        band = [p["p"] for p in pts if p.get("path") == "grasp"] or [p["p"] for p in pts if p.get("path") == "setdown"] or [pts[0]["p"]]
        z = float(np.mean([b[2] for b in band]))
        out.append((float(ob["xy"][0]), float(ob["xy"][1]), z))
    return out


def facing_heading(xs, ys, points):
    """The direction (radians, world) from each stance's floor position (xs, ys) to the midpoint of the task points (their mean x, y)."""
    mx = float(np.mean([p[0] for p in points]))
    my = float(np.mean([p[1] for p in points]))
    return np.arctan2(my - ys, mx - xs)


def draw_batch(points, cfg, seed, held_heading=None):
    """The whole batch, drawn from one seed: x, y, pedestal, then the headings (or the held heading for all).

    Args:
        points (list of (x, y, z)): The task points.
        cfg (dict): BENCH_STANCE_CFG (sampler_batch, sampler_margin_sigmas, pedestal_range, sampler_heading_half_width_deg).
        seed (int): The sampling seed.
        held_heading (float or None): When not None (radians), every stance gets this heading and no heading is drawn.

    Returns:
        dict: "rng", "xs", "ys", "qs", "headings" (numpy arrays of length N), "facing" (the facing direction of each stance), "box" ((x_lo, x_hi, y_lo, y_hi)).
    """
    rng = np.random.default_rng(seed)
    n = int(cfg["sampler_batch"])
    grow = margin_m(cfg["sampler_margin_sigmas"])
    px = np.array([p[0] for p in points])
    py = np.array([p[1] for p in points])
    box = (float(px.min() - grow), float(px.max() + grow), float(py.min() - grow), float(py.max() + grow))
    q_lo, q_hi = cfg["pedestal_range"]
    xs = rng.uniform(box[0], box[1], n)
    ys = rng.uniform(box[2], box[3], n)
    qs = rng.uniform(q_lo, q_hi, n)
    facing = facing_heading(xs, ys, points)
    if held_heading is not None:
        headings = np.full(n, float(held_heading))
    else:
        half = math.radians(float(cfg["sampler_heading_half_width_deg"]))
        headings = facing + rng.uniform(-half, half, n)
    return {"rng": rng, "xs": xs, "ys": ys, "qs": qs, "headings": headings, "facing": facing, "box": box}


def select_stance(points, geom, arm, held_heading, cfg, check, seed=None, say=print):
    """The whole selection: draw, score, weight, draw again, check.

    Args:
        points (list of (x, y, z)): task_points(...) of the plan's manipulation steps.
        geom (dict): robot_geometry(model).
        arm (str): The arm used for scoring.
        held_heading (float or None): The heading held for every stance (radians), or None to draw each stance's heading (see draw_batch).
        cfg (dict): BENCH_STANCE_CFG (uses sampler_batch, sampler_weight_power, sampler_max_draws, sampler_margin_sigmas, pedestal_range).
        check (callable): check(stance) -> dict with "ok" (bool), "failed" (list), "reason" (str), "seconds" (float or None), "seed" (the full check's seed or None).
            stance is (x, y, heading, q) with float values, unchanged.
        seed (int or None): The sampling seed. None: taken from the clock (time.time_ns()).
        say (callable): Progress lines.

    Returns:
        dict: "ok", "stop" (the reason when not ok), "seed", "arm", "heading", "box" ((x_lo, x_hi, y_lo, y_hi)), "batch" (N), "score" (min, median, max, fraction at zero),
        "draws" (one dict per full check: "draw", "x", "y", "heading", "pedestal", "beaker_score"/"hotplate_score" (per point), "score", "weight", "ok", "failed", "reason", "seconds", "seed"),
        "chosen" (the draw dict that passed, or None).
    """
    if seed is None:
        seed = time.time_ns() % (2 ** 32)
    b = draw_batch(points, cfg, seed, held_heading)
    rng, n, box = b["rng"], int(cfg["sampler_batch"]), b["box"]
    xs, ys, qs, headings = b["xs"], b["ys"], b["qs"], b["headings"]
    q_lo, q_hi = cfg["pedestal_range"]
    scores = a8_scores(xs, ys, qs, headings, points, geom, arm)
    weights = scores ** float(cfg["sampler_weight_power"])
    rec = {"seed": int(seed), "arm": arm, "held_heading": None if held_heading is None else float(held_heading), "batch": n, "box": box, "pedestal_range": [float(q_lo), float(q_hi)], "margin_m": margin_m(cfg["sampler_margin_sigmas"]),
           "score": {"min": float(scores.min()), "median": float(np.median(scores)), "max": float(scores.max()), "zero_fraction": float(np.mean(scores <= 0.0))},
           "draws": [], "chosen": None, "ok": False, "stop": None}
    say(f"[sampler] seed {seed}: {n} stances, score min {rec['score']['min']:.3f} median {rec['score']['median']:.3f} max {rec['score']['max']:.3f}, zero fraction {rec['score']['zero_fraction']:.3f}")
    if weights.sum() <= 0.0:
        rec["stop"] = "every stance in the batch has a zero weight (A8 score zero): no stance to draw"
        return rec
    alive = np.ones(n, bool)
    for k in range(int(cfg["sampler_max_draws"])):
        w = np.where(alive, weights, 0.0)
        if w.sum() <= 0.0:
            rec["stop"] = f"no stance left with a positive weight after {k} draws"
            return rec
        i = int(rng.choice(n, p=w / w.sum()))
        alive[i] = False
        stance = (float(xs[i]), float(ys[i]), float(headings[i]), float(qs[i]))
        r = check(stance)
        row = {"draw": k + 1, "index": i, "x": stance[0], "y": stance[1], "heading": stance[2], "facing": float(b["facing"][i]), "pedestal": stance[3],
               "beaker_score": None, "hotplate_score": None, "score": float(scores[i]), "weight": float(weights[i]),
               "ok": bool(r["ok"]), "failed": list(r.get("failed", [])), "reason": str(r.get("reason", ""))[:300], "seconds": r.get("seconds"), "seed": r.get("seed")}
        for j, pnt in enumerate(points):                        # the per-point counts (the score is the lower one)
            row[f"point_{j}_score"] = float(a8_scores(np.array([stance[0]]), np.array([stance[1]]), np.array([stance[3]]), np.array([stance[2]]), [pnt], geom, arm)[0])
        rec["draws"].append(row)
        say(f"[sampler] draw {k + 1}: x {stance[0]:.4f} y {stance[1]:.4f} pedestal {stance[3]:.4f} score {row['score']:.3f}: full check {'PASS' if row['ok'] else 'FAIL ' + str(row['failed'])}")
        if row["ok"]:
            rec.update(ok=True, chosen=row)
            return rec
    rec["stop"] = f"all {int(cfg['sampler_max_draws'])} draws failed the full check"
    return rec
