"""Stance selection by weighted random sampling from the A8 Gaussian reach model (decided by David, 2026-11-10; decisions-log M299, Revisions 1 to 3).

What the pipeline runs (src/stance_select.apply_stance_selection calls only this module):
  1. Draw a batch of N stances in polar coordinates around the midpoint of the task points (Revision 3): the pedestal, then the
     left shoulder's floor position at a distance chosen so that the shoulder-to-centre distance lies between the inner limit r_in
     (measured from the reach maps) and the outer limit r_out (A8: r0 + 2 sr), with equal floor areas equally likely; the base is placed
     so that the shoulder is there, with a heading within sampler_heading_half_width_deg of facing the centre from the shoulder.
     A stance whose task point is closer than r_in to the shoulder is dropped and drawn again. While sampler_heading_held is True the
     heading is held at TASK_CFG's staging_yaw (-90 deg) instead, because the staging and navigation are not checked at other headings.
  2. Score each stance with the A8 count (src/reach_model.predict_count_asym). The shoulder position comes from the stance; each task
     point (the pick's grasp point, the place's set-down point) gives one count; the stance's score is the LOWER of its counts.
  3. Weight = score ** weight power. Draw one stance with probability proportional to its weight, without putting failed stances back.
  4. Run the full check on that stance with its values unchanged. Stop at the first pass; at most `sampler_max_draws` checks run.
  5. The sampling seed comes from the clock and is saved in the record, so the batch and every draw can be reproduced from it.

The arm used for scoring is BENCH_STANCE_CFG["arm"] (left). The right arm would use the mirrored model (reach_model.arm_params).
"""
import math
import time

import numpy as np

from src.reach_model import arm_params, shoulder_height
from src.reach_model import predict_count_asym, A8


def outer_limit_m(sigmas):
    """r_out: the A8 radial centre plus `sigmas` radial widths (r0 + k * sr), metres (about 0.89 m for k = 2)."""
    return A8["r0"] + sigmas * A8["sr"]


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


def draw_batch(points, geom, arm, cfg, seed, held_heading=None):
    """The batch, drawn from one seed in polar coordinates around the centre of the task points (Revision 3).

    Each attempt: the pedestal q (uniform in pedestal_range); the shoulder's height gives dz to the centre; if r_out <= |dz| the pedestal is drawn again (the
    attempt is spent). Otherwise the floor distance d of the shoulder from the centre is d = sqrt(U (d_max^2 - d_min^2) + d_min^2), with
    d_min = sqrt(max(r_in^2 - dz^2, 0)) and d_max = sqrt(r_out^2 - dz^2) (equal floor areas equally likely); the direction phi is uniform in 0..360 deg;
    the shoulder's floor position is centre + d (cos phi, sin phi). The heading is the facing direction from that position to the centre plus a uniform
    offset in +- sampler_heading_half_width_deg (or the held heading). The base origin is the shoulder's floor position minus the shoulder's horizontal
    offset (geom shoulder x, y) turned by the heading. A stance with a task point closer than r_in to the shoulder (3D) is dropped and drawn again.
    Attempts are capped at sampler_redraw_factor * N; if the cap is hit the stances kept so far are the batch.

    Args:
        points (list of (x, y, z)): The task points.
        geom (dict): robot_geometry(model).
        arm (str): "left" (the scoring arm).
        cfg (dict): BENCH_STANCE_CFG (sampler_batch, sampler_r_in_m, sampler_outer_sigmas, sampler_redraw_factor, sampler_heading_half_width_deg, pedestal_range).
        seed (int): The sampling seed.
        held_heading (float or None): When not None (radians), every stance gets this heading and no heading is drawn.

    Returns:
        dict: "rng" (the generator, to continue with the draws), "xs", "ys" (base origin), "qs", "headings", "facing" (from the shoulder to the centre), "shoulder" ((N, 3)
        shoulder world position), "dz", "d" (floor distance shoulder to centre), "phi", "attempts" (attempts used to fill the batch), "drops" (task point
        closer than r_in), "pedestal_redraws" (r_out <= |dz|), "cap_hit" (bool), "r_in", "r_out", "offset" ((sx, sy) torso-frame shoulder offset from the base centre).
    """
    rng = np.random.default_rng(seed)
    n = int(cfg["sampler_batch"])
    r_in = float(cfg["sampler_r_in_m"])
    r_out = outer_limit_m(cfg["sampler_outer_sigmas"])
    cap = int(cfg["sampler_redraw_factor"]) * n
    q_lo, q_hi = cfg["pedestal_range"]
    P = np.array(points, float)
    cx, cy, cz = P.mean(axis=0)                                  # the centre: the midpoint of the task points
    sx, sy, sz = geom["shoulder"][arm]
    half = math.radians(float(cfg["sampler_heading_half_width_deg"]))
    out = {k: [] for k in ("xs", "ys", "qs", "headings", "facing", "shoulder", "dz", "d", "phi")}
    kept, attempts, drops, redraws, cap_hit = 0, 0, 0, 0, False
    while kept < n:
        if attempts >= cap:
            cap_hit = True
            break
        m = max(64, 2 * (n - kept))
        q = rng.uniform(q_lo, q_hi, m)
        Sz = shoulder_height(geom, arm, q)
        dz = Sz - cz
        ok_q = r_out > np.abs(dz)                                 # the pedestal must allow an outer limit beyond the height gap
        dmin = np.sqrt(np.maximum(r_in ** 2 - dz ** 2, 0.0))
        dmax = np.sqrt(np.maximum(r_out ** 2 - dz ** 2, 0.0))
        U = rng.uniform(0.0, 1.0, m)
        d = np.sqrt(U * (dmax ** 2 - dmin ** 2) + dmin ** 2)
        phi = rng.uniform(0.0, 2 * math.pi, m)
        Sx, Sy = cx + d * np.cos(phi), cy + d * np.sin(phi)      # the shoulder's floor position
        facing = np.arctan2(cy - Sy, cx - Sx)
        if held_heading is not None:
            heading = np.full(m, float(held_heading))
        else:
            heading = facing + rng.uniform(-half, half, m)
        ox, oy = _turn(heading, sx, sy)
        xs, ys = Sx - ox, Sy - oy                                 # the base origin: the shoulder minus its offset turned by the heading
        dist = np.min(np.stack([np.sqrt((Sx - px) ** 2 + (Sy - py) ** 2 + (Sz - pz) ** 2) for px, py, pz in P]), axis=0)
        inner = dist >= r_in
        good = ok_q & inner
        # attempts are used in order; the batch is filled from the first good ones
        idx = np.nonzero(good)[0]
        take = idx[: n - kept]
        if len(take) == n - kept:
            used = int(take[-1]) + 1
        else:
            used = m
        attempts += used
        seen = slice(0, used)
        redraws += int((~ok_q[seen]).sum())
        drops += int((ok_q[seen] & ~inner[seen]).sum())
        for k, v in (("xs", xs), ("ys", ys), ("qs", q), ("headings", heading), ("facing", facing), ("dz", dz), ("d", d), ("phi", phi)):
            out[k].append(np.asarray(v)[take])
        out["shoulder"].append(np.stack([Sx[take], Sy[take], Sz[take]], axis=1))
        kept += len(take)
    cat = {k: (np.concatenate(v) if v and len(v[0]) else np.zeros(0)) for k, v in out.items() if k != "shoulder"}
    cat["shoulder"] = np.concatenate(out["shoulder"]) if out["shoulder"] else np.zeros((0, 3))
    cat.update(rng=rng, attempts=attempts, drops=drops, pedestal_redraws=redraws, cap_hit=cap_hit, r_in=r_in, r_out=r_out, offset=(float(sx), float(sy)))
    return cat


def select_stance(points, geom, arm, held_heading, cfg, check, seed=None, say=print):
    """The whole selection: draw, score, weight, draw again, check.

    Args:
        points (list of (x, y, z)): task_points(...) of the plan's manipulation steps.
        geom (dict): robot_geometry(model).
        arm (str): The arm used for scoring.
        held_heading (float or None): The heading held for every stance (radians), or None to draw each stance's heading (see draw_batch).
        cfg (dict): BENCH_STANCE_CFG (uses sampler_batch, sampler_weight_power, sampler_max_draws, sampler_r_in_m, sampler_outer_sigmas, sampler_redraw_factor,
            sampler_heading_half_width_deg, pedestal_range).
        check (callable): check(stance) -> dict with "ok" (bool), "failed" (list), "reason" (str), "seconds" (float or None), "seed" (the full check's seed or None).
            stance is (x, y, heading, q) with float values, unchanged.
        seed (int or None): The sampling seed. None: taken from the clock (time.time_ns()).
        say (callable): Progress lines.

    Returns:
        dict: "ok", "stop" (the reason when not ok), "seed", "arm", "held_heading", "batch" (stances kept), "attempts", "drops", "pedestal_redraws", "cap_hit",
        "r_in", "r_out", "offset", "score" (min, median, max, fraction at zero), "draws" (one dict per full check: "draw", "index", "x", "y", "heading", "facing",
        "pedestal", "shoulder" (x, y, z), "dz", "d", "score", "weight", per point "point_j_score", "ok", "failed", "reason", "seconds", "seed"), "chosen" (the draw that passed, or None).
    """
    if seed is None:
        seed = time.time_ns() % (2 ** 32)
    b = draw_batch(points, geom, arm, cfg, seed, held_heading)
    rng = b["rng"]
    xs, ys, qs, headings = b["xs"], b["ys"], b["qs"], b["headings"]
    n = len(xs)
    rec = {"seed": int(seed), "arm": arm, "held_heading": None if held_heading is None else float(held_heading), "batch": n, "attempts": int(b["attempts"]),
           "drops": int(b["drops"]), "pedestal_redraws": int(b["pedestal_redraws"]), "cap_hit": bool(b["cap_hit"]), "r_in": float(b["r_in"]), "r_out": float(b["r_out"]),
           "offset": [float(v) for v in b["offset"]], "pedestal_range": [float(v) for v in cfg["pedestal_range"]],
           "draws": [], "chosen": None, "ok": False, "stop": None, "score": None}
    if n == 0:
        rec["stop"] = f"no stance kept: the attempt cap ({int(cfg['sampler_redraw_factor']) * int(cfg['sampler_batch'])}) was reached first"
        return rec
    scores = a8_scores(xs, ys, qs, headings, points, geom, arm)
    weights = scores ** float(cfg["sampler_weight_power"])
    rec["score"] = {"min": float(scores.min()), "median": float(np.median(scores)), "max": float(scores.max()), "zero_fraction": float(np.mean(scores <= 0.0))}
    say(f"[sampler] seed {seed}: {n} stances kept in {b['attempts']} attempts ({b['drops']} dropped inside r_in, {b['pedestal_redraws']} pedestal redraws, cap hit {b['cap_hit']}); "
        f"score min {rec['score']['min']:.3f} median {rec['score']['median']:.3f} max {rec['score']['max']:.3f}, zero fraction {rec['score']['zero_fraction']:.3f}")
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
               "shoulder": [float(v) for v in b["shoulder"][i]], "dz": float(b["dz"][i]), "d": float(b["d"][i]),
               "score": float(scores[i]), "weight": float(weights[i]),
               "ok": bool(r["ok"]), "failed": list(r.get("failed", [])), "reason": str(r.get("reason", ""))[:300], "seconds": r.get("seconds"), "seed": r.get("seed")}
        for j, pnt in enumerate(points):                        # the per-point counts (the score is the lower one)
            row[f"point_{j}_score"] = float(a8_scores(np.array([stance[0]]), np.array([stance[1]]), np.array([stance[3]]), np.array([stance[2]]), [pnt], geom, arm)[0])
        rec["draws"].append(row)
        say(f"[sampler] draw {k + 1}: x {stance[0]:.4f} y {stance[1]:.4f} heading {math.degrees(stance[2]):.1f} pedestal {stance[3]:.4f} score {row['score']:.3f}: "
            f"full check {'PASS' if row['ok'] else 'FAIL ' + str(row['failed'])}")
        if row["ok"]:
            rec.update(ok=True, chosen=row)
            return rec
    rec["stop"] = f"all {int(cfg['sampler_max_draws'])} draws failed the full check"
    return rec
