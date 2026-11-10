"""The edge sampler: the stances drawn in front of the usable edges of the furniture under the objects (David, 2026-11-10; decisions-log M300).

It replaces the polar draw of M299 Revision 3 (src/stance_sampling.py, now uncalled by the pipeline). For each object (1 or more):
  - the object's region is the set of base centres (x, y) that are, for one of the object's usable edges (src/usable_edges.py):
      (a) at signed distance from the edge line of at least the margin (outward), and within the edge part's extent; and
      (b) within the reach limit: the 3D distance from the left shoulder to each of the object's task points is at most r_max;
  - the overlap of all the objects' regions is the set of stances; its rectangles ("pieces", one per combination of edges) are the draw's boxes;
  - a stance is: pedestal uniform in the pedestal range; base x, y uniform in a piece's rectangle (the piece chosen with probability proportional to its area), kept only inside
    the overlap; heading = the exact direction from the base centre to the centroid of the objects' midpoints (the midpoint of the pick grasp point and the place point of each object).

Score: the A9 count (src/reach_model.predict_count_a9) of each task point, the lowest over the points; the weight is the score to sampler_weight_power. The draw goes by weight,
without putting failed stances back. Each drawn stance first goes through the body-check gate (no full check), then the full check (up to sampler_max_draws); the first pass stops.

    objects = [{"name", "points": [provider, ...], "edges": [segment, ...], "pick": (x, y, z) or None, "place": (x, y, z) or None, "mid": (x, y, z)}, ...]
    provider: {"kind": "fixed", "p": (x, y, z)} or {"kind": "back", "pivot": (x, y), "z": z, "rt": metres}  (the back-off point turns with the heading)
    segment: a usable edge (src/usable_edges.py): "p0", "p1", "n", "length", "furniture", "side".

Contents:
    a9_params(cfg, arm), provider_xyz, shoulder, headings_for, region, pieces_for, draw_stances, a9_score, select_edge_stance.
"""

import itertools
import math
import time

import numpy as np

from src.reach_model import predict_count_a9, shoulder_height
from src.stance_sampling import write_record


GRID_STEP_M = 0.005          # m: the grid of the rectangle search (the pieces' bounding boxes) [implementation detail, not a decided value]
SLACK_M = 0.05               # m: the shoulder's offset from the base centre (horizontal), the most it can add to a reach test on the base centre [robot_geometry: 0.031 m, rounded up]
PEDESTAL_STEPS = 9           # count: the pedestal heights (evenly across pedestal_range) at which the overlap is tested; the rectangle is the bounding box of the overlap over all of them [implementation detail]


def a9_params(cfg, arm):
    """The A9 parameters of an arm, as predict_count_a9 takes them: ((C, r0, sig_near, sig_far, s_neg0, s_neg1, s_pos0, s_pos1), z_cut).

    The right arm is the left arm mirrored: the narrow side (theta < 0 for the left arm) moves to theta >= 0.
    """
    a = cfg["sampler_a9"]
    if arm == "left":
        sn, sp = a["neg"], a["pos"]
    elif arm == "right":
        sn, sp = a["pos"], a["neg"]
    else:
        raise ValueError(f"arm must be 'left' or 'right', got {arm!r}")
    return (a["C"], a["r0"], a["sigma_near"], a["sigma_far"], *sn, *sp), a["z_cut"]


def provider_xyz(prov, headings):
    """The x, y, z (each (N,)) of a task point for N stances with these headings."""
    n = len(headings)
    if prov["kind"] == "fixed":
        x, y, z = prov["p"]
        return np.full(n, float(x)), np.full(n, float(y)), np.full(n, float(z))
    px, py = prov["pivot"]
    c, s = np.cos(headings), np.sin(headings)
    return px - prov["rt"] * c, py - prov["rt"] * s, np.full(n, float(prov["z"]))


def shoulder(geom, arm, xs, ys, headings, qs):
    """The arm's shoulder (x, y, z), each (N,), for stances at base (xs, ys), headings and pedestal commands qs."""
    sx, sy, _ = geom["shoulder"][arm]
    c, s = np.cos(headings), np.sin(headings)
    return xs + c * sx - s * sy, ys + s * sx + c * sy, shoulder_height(geom, arm, qs)


def headings_for(xs, ys, objects):
    """The heading of each stance: the exact direction from its base centre to the centroid of the objects' midpoints (radians)."""
    cx = float(np.mean([o["mid"][0] for o in objects]))
    cy = float(np.mean([o["mid"][1] for o in objects]))
    return np.arctan2(cy - ys, cx - xs)


def _edge_test(seg, xs, ys, margin):
    """(ok, s): ok where the base centre is at least `margin` out from the edge's line and within the edge part; s the signed distance (outward positive)."""
    p0 = np.asarray(seg["p0"], float)
    p1 = np.asarray(seg["p1"], float)
    n = np.asarray(seg["n"], float)
    L = float(seg["length"])
    u = (p1 - p0) / np.hypot(*(p1 - p0))
    dx, dy = xs - p0[0], ys - p0[1]
    s = dx * n[0] + dy * n[1]
    t = dx * u[0] + dy * u[1]
    return (s >= margin) & (t >= 0.0) & (t <= L), s


def region(xs, ys, qs, objects, geom, arm, cfg):
    """Which stances are in the overlap of the objects' regions.

    Returns:
        tuple: (ok (N,) bool, headings (N,), info): info is one dict per object with "edge" (N,) int (the first of the object's usable edges the stance is in, -1 none),
            "signed" (N,) float (its signed distance from that edge, NaN none), "reach" (N,) bool (every task point of the object within r_max).
    """
    margin, rmax = float(cfg["sampler_margin_m"]), float(cfg["sampler_r_max_m"])
    heads = headings_for(xs, ys, objects)
    sx, sy, sz = shoulder(geom, arm, xs, ys, heads, qs)
    ok = np.ones(len(xs), bool)
    info = []
    for o in objects:
        reach = np.ones(len(xs), bool)
        for prov in o["points"]:
            px, py, pz = provider_xyz(prov, heads)
            reach &= np.sqrt((sx - px) ** 2 + (sy - py) ** 2 + (sz - pz) ** 2) <= rmax
        edge = np.full(len(xs), -1)
        signed = np.full(len(xs), np.nan)
        any_edge = np.zeros(len(xs), bool)
        for k, seg in enumerate(o["edges"]):
            e_ok, s = _edge_test(seg, xs, ys, margin)
            new = e_ok & ~any_edge
            edge[new] = k
            signed[new] = s[new]
            any_edge |= e_ok
        ok &= reach & any_edge
        info.append({"edge": edge, "signed": signed, "reach": reach})
    return ok, heads, info


def pieces_for(objects, geom, arm, cfg, step=GRID_STEP_M):
    """The overlap split by the edges used: one piece per combination of one usable edge per object.

    A piece is a rectangle: the bounding box of the base centres that are in the overlap at some pedestal height in pedestal_range (tested at PEDESTAL_STEPS heights). The
    candidates before that test are the superset on the edges' outward sides within the reach limit plus the shoulder's offset, so the test is exact and the box is tight.

    Returns:
        list of dict: {"edges": [(object index, edge index), ...], "rect": (xmin, xmax, ymin, ymax), "area": m^2, "candidates": int, "feasible": bool, "feasible_points": int}.
    """
    margin, rmax = float(cfg["sampler_margin_m"]), float(cfg["sampler_r_max_m"])
    if any(not o["edges"] for o in objects):
        return []
    rt_max = max([p.get("rt", 0.0) for o in objects for p in o["points"]] + [0.0])
    R = rmax + SLACK_M + rt_max
    pedestals = np.linspace(float(cfg["pedestal_range"][0]), float(cfg["pedestal_range"][1]), PEDESTAL_STEPS)
    out = []
    for combo in itertools.product(*[range(len(o["edges"])) for o in objects]):
        segs = [objects[i]["edges"][k] for i, k in enumerate(combo)]
        s0 = segs[0]
        p0 = np.asarray(s0["p0"], float)
        n0 = np.asarray(s0["n"], float)
        u0 = (np.asarray(s0["p1"], float) - p0) / float(s0["length"])
        T = np.arange(0.0, float(s0["length"]) + step / 2, step)
        S = np.arange(margin, margin + R + step / 2, step)
        TT, SS = np.meshgrid(T, S)
        X = (p0[0] + TT * u0[0] + SS * n0[0]).ravel()
        Y = (p0[1] + TT * u0[1] + SS * n0[1]).ravel()
        keep = np.ones(len(X), bool)
        for i, seg in enumerate(segs):
            keep &= _edge_test(seg, X, Y, margin)[0]
        for o in objects:
            for prov in o["points"]:
                cx, cy = (prov["pivot"][0], prov["pivot"][1]) if prov["kind"] == "back" else (prov["p"][0], prov["p"][1])
                slack = SLACK_M + (prov.get("rt", 0.0) if prov["kind"] == "back" else 0.0)
                keep &= np.hypot(X - cx, Y - cy) <= rmax + slack
        X, Y = X[keep], Y[keep]
        piece = {"edges": [(i, k) for i, k in enumerate(combo)], "candidates": int(len(X)), "rect": None, "area": 0.0, "feasible": False, "feasible_points": 0}
        if len(X) >= 2:
            inside = np.zeros(len(X), bool)
            for q in pedestals:
                inside |= region(X, Y, np.full(len(X), q), objects, geom, arm, cfg)[0]
            X, Y = X[inside], Y[inside]
            piece["feasible_points"] = int(len(X))
            piece["feasible"] = len(X) > 0
            if len(X) >= 2:
                rect = (float(X.min()), float(X.max()), float(Y.min()), float(Y.max()))
                piece["rect"] = rect
                piece["area"] = (rect[1] - rect[0]) * (rect[3] - rect[2])
        out.append(piece)
    return out


def draw_stances(pieces, objects, geom, arm, cfg, seed, chunk=2000):
    """The stances kept by the weighted-area draw: pedestal uniform, x and y uniform in a feasible piece's rectangle (the piece chosen with probability proportional to its area),
    kept only inside the overlap. Attempts stop at sampler_redraw_factor x sampler_batch.

    Returns:
        dict: xs, ys, qs, headings (each (N,)), info (per object, the edge and the signed distance of each kept stance), attempts, cap_hit, rng.
    """
    rng = np.random.default_rng(seed)
    feas = [p for p in pieces if p["feasible"] and p["area"] > 0]
    batch = int(cfg["sampler_batch"])
    cap = int(cfg["sampler_redraw_factor"]) * batch
    xs, ys, qs, heads = [], [], [], []
    infos = [{"edge": [], "signed": [], "reach": []} for _ in objects]
    kept, attempts = 0, 0
    if not feas:
        return {"xs": np.zeros(0), "ys": np.zeros(0), "qs": np.zeros(0), "headings": np.zeros(0), "info": infos, "attempts": 0, "cap_hit": False, "rng": rng}
    area = np.array([p["area"] for p in feas])
    prob = area / area.sum()
    rects = np.array([p["rect"] for p in feas])
    pmin, pmax = cfg["pedestal_range"]
    while kept < batch and attempts < cap:
        m = min(chunk, cap - attempts)
        pick = rng.choice(len(feas), size=m, p=prob)
        r = rects[pick]
        x = r[:, 0] + (r[:, 1] - r[:, 0]) * rng.random(m)
        y = r[:, 2] + (r[:, 3] - r[:, 2]) * rng.random(m)
        q = rng.uniform(pmin, pmax, m)
        ok, h, info = region(x, y, q, objects, geom, arm, cfg)
        good = np.flatnonzero(ok)
        need = batch - kept
        if len(good) >= need:                                   # the batch fills inside this chunk: the attempts count only up to the last stance kept
            good = good[:need]
            attempts += int(good[-1]) + 1
        else:
            attempts += m
        xs.append(x[good]); ys.append(y[good]); qs.append(q[good]); heads.append(h[good])
        for j, inf in enumerate(info):
            infos[j]["edge"].append(inf["edge"][good]); infos[j]["signed"].append(inf["signed"][good]); infos[j]["reach"].append(inf["reach"][good])
        kept += len(good)
    cat = lambda v: np.concatenate(v) if v else np.zeros(0)
    infos = [{k: cat(v) for k, v in inf.items()} for inf in infos]
    return {"xs": cat(xs), "ys": cat(ys), "qs": cat(qs), "headings": cat(heads), "info": infos, "attempts": attempts, "cap_hit": kept < batch, "rng": rng}


def a9_score(xs, ys, qs, heads, objects, geom, arm, cfg):
    """The A9 score of each stance: the lower, over all the objects' task points, of the A9 count of that point.

    Returns:
        tuple: (score (N,), counts: list of (N,) per task point, in the order of the objects' points).
    """
    p9, z_cut = a9_params(cfg, arm)
    sx, sy, sz = shoulder(geom, arm, xs, ys, heads, qs)
    c, s = np.cos(heads), np.sin(heads)
    counts = []
    for o in objects:
        for prov in o["points"]:
            px, py, pz = provider_xyz(prov, heads)
            dx, dy, dz = px - sx, py - sy, pz - sz
            xl = c * dx + s * dy                                  # the point in the shoulder frame: x forward, y left
            yl = -s * dx + c * dy
            r = np.sqrt(xl ** 2 + yl ** 2 + dz ** 2)
            counts.append(predict_count_a9(r, np.arctan2(yl, xl), dz, p9, z_cut))
    return np.min(np.stack(counts), axis=0), counts


def select_edge_stance(objects, pieces, geom, arm, cfg, check, gate=None, seed=None, say=print, record_dir=None, base_metric=None, point_labels=None):
    """The whole selection: draw, score, weight, draw again, gate, check.

    Args:
        objects (list of dict): See the module docstring.
        pieces (list of dict): pieces_for(objects, ...); only the feasible ones are drawn from.
        geom (dict): robot_geometry(model).
        arm (str): The arm used for the reach.
        cfg (dict): BENCH_STANCE_CFG (sampler_batch, sampler_weight_power, sampler_max_draws, sampler_gate_max_failures, sampler_redraw_factor, sampler_margin_m, sampler_r_max_m,
            sampler_a9, pedestal_range).
        check (callable): check(stance) -> dict with "ok", "failed", "reason", "seconds", "seed". stance is (x, y, heading, q) with float values.
        gate (callable or None): gate(stance) -> (ok, pair, distance): the full check's body check, before the full check. A failing stance is discarded (not put back) and
            does not count toward sampler_max_draws; more than sampler_gate_max_failures of them stops the selection.
        seed (int or None): The sampling seed. None: from the clock.
        say (callable): Progress lines.
        record_dir (str, Path or None): Where the selection record is written (JSON, atomically). None: not written.
        base_metric (callable or None): base_metric(x, y) -> the base's distance to the bench top's edge (logging only).
        point_labels (list of str or None): One label per task point, in the order of the objects' points (logging).

    Returns:
        dict: "ok", "stop", "seed", "arm", "batch", "attempts", "cap_hit", "pieces" (the summary), "fraction_kept", "score" (min, median, max, zero_fraction), "gate_failures",
        "gate_pairs", "gate_rows", "draws" (one dict per full check), "chosen" (the draw that passed, or None), "record_path", "seconds".
    """
    t0 = time.perf_counter()
    if seed is None:
        seed = time.time_ns() % (2 ** 32)
    feas = [p for p in pieces if p["feasible"] and p["area"] > 0]
    rec = {"seed": int(seed), "arm": arm, "batch": 0, "attempts": 0, "cap_hit": False, "ok": False, "stop": None, "chosen": None, "draws": [], "score": None,
           "gate_failures": 0, "gate_pairs": {}, "gate_rows": [], "fraction_kept": None, "record_path": None,
           "pieces": [{"edges": [{"furniture": objects[i]["edges"][k]["furniture"], "side": objects[i]["edges"][k]["side"], "edge": int(objects[i]["edges"][k]["edge"]),
                                  "p0": [float(v) for v in objects[i]["edges"][k]["p0"]], "p1": [float(v) for v in objects[i]["edges"][k]["p1"]], "object": objects[i]["name"]}
                                 for i, k in p["edges"]],
                       "rect": None if p["rect"] is None else [float(v) for v in p["rect"]], "area_m2": float(p["area"]), "feasible": bool(p["feasible"]),
                       "feasible_points": int(p["feasible_points"])} for p in pieces]}
    if not feas:
        rec["stop"] = "needs travel between stances: the objects' regions do not overlap"
        return _finish(rec, record_dir, t0)
    d = draw_stances(pieces, objects, geom, arm, cfg, seed)
    rng, xs, ys, qs, heads = d["rng"], d["xs"], d["ys"], d["qs"], d["headings"]
    n = len(xs)
    rec.update(batch=n, attempts=int(d["attempts"]), cap_hit=bool(d["cap_hit"]), pedestal_range=[float(v) for v in cfg["pedestal_range"]])
    rec["fraction_kept"] = float(n / d["attempts"]) if d["attempts"] else None
    if n == 0:
        rec["stop"] = f"no stance kept: the attempt cap ({int(cfg['sampler_redraw_factor']) * int(cfg['sampler_batch'])}) was reached first"
        return _finish(rec, record_dir, t0)
    scores, counts = a9_score(xs, ys, qs, heads, objects, geom, arm, cfg)
    weights = scores ** float(cfg["sampler_weight_power"])
    rec["score"] = {"min": float(scores.min()), "median": float(np.median(scores)), "max": float(scores.max()), "zero_fraction": float(np.mean(scores <= 0.0))}
    say(f"[sampler] seed {seed}: {n} stances kept in {d['attempts']} attempts (fraction kept {rec['fraction_kept']:.3f}, cap hit {d['cap_hit']}); "
        f"A9 score min {rec['score']['min']:.3f} median {rec['score']['median']:.3f} max {rec['score']['max']:.3f}, zero fraction {rec['score']['zero_fraction']:.3f}")
    if weights.sum() <= 0.0:
        rec["stop"] = "every stance in the batch has a zero weight (A9 score zero): no stance to draw"
        return _finish(rec, record_dir, t0)
    cx = float(np.mean([o["mid"][0] for o in objects]))
    cy = float(np.mean([o["mid"][1] for o in objects]))
    alive = np.ones(n, bool)
    k = 0
    cap_gate = int(cfg["sampler_gate_max_failures"])
    while k < int(cfg["sampler_max_draws"]):
        w = np.where(alive, weights, 0.0)
        if w.sum() <= 0.0:
            rec["stop"] = f"no stance left with a positive weight after {k} full-check draws"
            return _finish(rec, record_dir, t0)
        i = int(rng.choice(n, p=w / w.sum()))
        alive[i] = False
        stance = (float(xs[i]), float(ys[i]), float(heads[i]), float(qs[i]))
        if gate is not None:
            g_ok, g_pair, g_dist = gate(stance)
            if not g_ok:
                rec["gate_failures"] += 1
                rec["gate_pairs"][g_pair] = rec["gate_pairs"].get(g_pair, 0) + 1
                rec["gate_rows"].append({"n": rec["gate_failures"], "x": stance[0], "y": stance[1], "heading": stance[2], "pedestal": stance[3], "pair": g_pair, "distance_m": float(g_dist)})
                say(f"[gate] failure {rec['gate_failures']}: x {stance[0]:.4f} y {stance[1]:.4f} pedestal {stance[3]:.4f}: touches {g_pair} ({1000 * float(g_dist):.1f} mm)")
                if rec["gate_failures"] >= cap_gate:
                    rec["stop"] = f"the body-check gate failed {rec['gate_failures']} times (cap {cap_gate}): selection stopped"
                    return _finish(rec, record_dir, t0)
                continue
        k += 1
        r = check(stance)
        sh = [float(v[0]) for v in shoulder(geom, arm, xs[i:i + 1], ys[i:i + 1], heads[i:i + 1], qs[i:i + 1])]
        row = {"draw": k, "index": i, "x": stance[0], "y": stance[1], "heading": stance[2], "pedestal": stance[3], "shoulder": sh,
               "edges": [], "signed_distance_m": [], "score": float(scores[i]), "weight": float(weights[i]),
               "ok": bool(r["ok"]), "failed": list(r.get("failed", [])), "reason": str(r.get("reason", "")), "seconds": r.get("seconds"), "seed": r.get("seed")}
        for j, o in enumerate(objects):
            e = int(d["info"][j]["edge"][i])
            seg = o["edges"][e] if e >= 0 else None
            row["edges"].append(None if seg is None else {"object": o["name"], "furniture": seg["furniture"], "side": seg["side"], "edge": int(seg["edge"]),
                                                          "p0": [float(v) for v in seg["p0"]], "p1": [float(v) for v in seg["p1"]]})
            row["signed_distance_m"].append(float(d["info"][j]["signed"][i]))
        flat = [p for o in objects for p in o["points"]]
        dd = []
        for p in flat:
            px, py, pz = provider_xyz(p, heads[i:i + 1])
            dd.append(float(np.sqrt((sh[0] - px[0]) ** 2 + (sh[1] - py[0]) ** 2 + (sh[2] - pz[0]) ** 2)))
        row["shoulder_to_point_m"] = dd
        row["point_scores"] = [float(c[i]) for c in counts]
        row["point_labels"] = list(point_labels) if point_labels is not None else None
        row["shoulder_to_pick_m"] = [None if o["pick"] is None else _dist3(sh, o["pick"]) for o in objects]
        row["shoulder_to_place_m"] = [None if o["place"] is None else _dist3(sh, o["place"]) for o in objects]
        row["base_to_bench_edge_m"] = None if base_metric is None else float(base_metric(stance[0], stance[1]))
        rec["draws"].append(row)
        say(f"[sampler] draw {k}: x {stance[0]:.4f} y {stance[1]:.4f} heading {math.degrees(stance[2]):.1f} pedestal {stance[3]:.4f} score {row['score']:.3f}: "
            f"full check {'PASS' if row['ok'] else 'FAIL'} ({row['seconds'] if row['seconds'] is not None else float('nan'):.2f} s)"
            + ("" if row["ok"] else f" reason: {row['reason']}"))
        if row["ok"]:
            rec.update(ok=True, chosen=row)
            return _finish(rec, record_dir, t0)
    rec["stop"] = f"all {int(cfg['sampler_max_draws'])} draws failed the full check"
    return _finish(rec, record_dir, t0)


def _dist3(a, b):
    return float(math.sqrt(sum((float(a[j]) - float(b[j])) ** 2 for j in range(3))))


def _finish(rec, record_dir, t0):
    """Writes the record (when a folder is given) and returns it."""
    rec["seconds"] = time.perf_counter() - t0
    if record_dir is not None:
        rec["record_path"] = write_record(rec, record_dir / f"sampler_{rec['seed']}.json")
    return rec
