"""A small Gaussian model of the reach maps (left arm, filtered maps): how many of the 12 approach bins reach a grasp point.

Standalone: not used by the pipeline or the stance selection.

The only inputs are the 3D offset from the shoulder to the grasp point, in the shoulder frame of the arm (x forward along the heading, y left, z up):
    d     horizontal distance (m)
    theta bearing relative to the heading (rad, turning left; atan2(y, x))
    z     vertical offset (m, negative = below the shoulder)
A heading change rotates the offset, which shifts theta and leaves d and z alone, so the model is heading-independent once the caller turns the world offset into the shoulder frame.

    count(d, theta, z) = C(z) * exp(-((d - d0(z)) / sd(z))^2) * exp(-((theta - th0) / st(z))^2),   clipped to [0, 12]
    C = a0 + a1 z,  d0 = b0 + b1 z,  sd = c0 + c1 z,  st = e0 + e1 z      (th0 = 0 in the specified form; `th0` is the one added parameter of the alternative)
"""
import math

import numpy as np

N_BINS = 12
PARAM_NAMES = ("a0", "a1", "b0", "b1", "c0", "c1", "e0", "e1")
MIN_WIDTH = 1e-3                                   # a width never goes below this (m or rad), so the formula stays defined


def predict_count(d, theta, z, p, th0=0.0):
    """Predicted number of approach bins that reach the offset (d, theta, z).

    Args:
        d (array-like): Horizontal distance from the shoulder, metres.
        theta (array-like): Bearing relative to the heading, radians, in (-pi, pi].
        z (array-like): Vertical offset, metres.
        p (sequence of float): The 8 parameters a0, a1, b0, b1, c0, c1, e0, e1.
        th0 (float): Angular centre, radians. Defaults to 0 (the specified model).

    Returns:
        numpy.ndarray: The count, in [0, N_BINS].
    """
    d, theta, z = np.broadcast_arrays(np.asarray(d, float), np.asarray(theta, float), np.asarray(z, float))
    a0, a1, b0, b1, c0, c1, e0, e1 = p
    C = a0 + a1 * z
    d0 = b0 + b1 * z
    sd = np.maximum(c0 + c1 * z, MIN_WIDTH)
    st = np.maximum(e0 + e1 * z, MIN_WIDTH)
    dth = (theta - th0 + np.pi) % (2 * np.pi) - np.pi            # wrapped difference
    return np.clip(C * np.exp(-((d - d0) / sd) ** 2) * np.exp(-(dth / st) ** 2), 0.0, N_BINS)


def predict_count_asym(r, theta, z, p, z_cut=None):
    """Gaussian in the 3D distance r and in the bearing, with a different angular width on each side of the heading, zero below z_cut.

    count = C exp(-((r - r0) / sr)^2) exp(-(theta / sigma(theta))^2); the peak is at r = r0, theta = 0 whatever the widths.
    sigma(theta) = s_neg if theta < 0 else s_pos, each linear in z (s = s0 + s1 z).

    Bearing sign: theta = atan2(y, x) in the shoulder frame (x forward, y left), so theta > 0 is to the robot's LEFT and theta < 0 to its right.

    Args:
        r (array-like): 3D distance from the shoulder, metres.
        theta (array-like): Bearing relative to the heading, radians, in (-pi, pi].
        z (array-like): Vertical offset, metres.
        p (sequence of float): (C, r0, sr, s_neg0, s_neg1, s_pos0, s_pos1): peak count (bins), radial centre (m), radial width (m), angular width for theta < 0 at z = 0 (rad) and its change per metre
            of z (rad/m), the same for theta >= 0.
        z_cut (float or None): Below this vertical offset the count is 0.

    Returns:
        numpy.ndarray: The count, in [0, N_BINS].
    """
    r, theta, z = np.broadcast_arrays(np.asarray(r, float), np.asarray(theta, float), np.asarray(z, float))
    C, r0, sr, sn0, sn1, sp0, sp1 = p
    sig = np.maximum(np.where(theta < 0, sn0 + sn1 * z, sp0 + sp1 * z), MIN_WIDTH)
    out = np.clip(C * np.exp(-((r - r0) / max(sr, MIN_WIDTH)) ** 2) * np.exp(-(theta / sig) ** 2), 0.0, N_BINS)
    return out if z_cut is None else np.where(z >= z_cut, out, 0.0)


# ---------------------------------------------------------------- A8 model: both arms and the analytic inverse
# The A8 fit (docs/reports/reach_model_fit_asym.json, "A8 ..."; left arm, filtered maps). Not refitted here.
# theta = atan2(y, x) in the shoulder frame: theta > 0 is the robot's left. For the left arm the theta < 0 side is the narrow one.
A8 = {"C": 3.7362697343695577, "r0": 0.4526881363136531, "sr": 0.21861105895402475, "z_cut": -0.375,
      "narrow": (0.27163635870503666, -0.5478728062183895),     # (sigma0, sigma1) of the narrow side, sigma = sigma0 + sigma1 z
      "wide": (50.682374812360926, -117.56740471140535)}       # the wide side


def arm_params(arm):
    """The A8 parameters of one arm: ((C, r0, sr, s_neg0, s_neg1, s_pos0, s_pos1), z_cut), s_neg for theta < 0, s_pos for theta >= 0.

    The right arm is the left arm mirrored: theta flips sign, so the narrow side moves to theta >= 0 and the wide side to theta < 0.
    """
    if arm == "left":
        sn, sp = A8["narrow"], A8["wide"]
    elif arm == "right":
        sn, sp = A8["wide"], A8["narrow"]
    else:
        raise ValueError(f"arm must be 'left' or 'right', got {arm!r}")
    return (A8["C"], A8["r0"], A8["sr"], *sn, *sp), A8["z_cut"]


def shoulder_height(geom, arm, q):
    """The arm's shoulder height above the floor at pedestal command q (the base origin is base_z above the floor)."""
    return geom["base_z"] + geom["torso_z0"] + geom["lift_factor"] * q + geom["shoulder"][arm][2]


def _rot(h, x, y):
    c, s = math.cos(h), math.sin(h)
    return c * x - s * y, s * x + c * y


def counts_at(base_xy, arm, heading, q, geom, points):
    """The A8 count of each world point, for a base at base_xy facing heading, pedestal q, one arm.

    Args:
        base_xy (sequence of float): The base origin (world x, y), m.
        arm (str): "left" or "right".
        heading (float): The torso's +x direction in the world, radians.
        q (float): The pedestal command, m.
        geom (dict): "shoulder" ({arm: (sx, sy, sz)} torso frame), "torso_z0", "lift_factor", "base_z" (robot_geometry's values).
        points (list of (x, y, z)): World points (the grasp or set-down point of each object), m.

    Returns:
        list of float: The predicted count of each point, in [0, 12].
    """
    p, z_cut = arm_params(arm)
    sx, sy, _ = geom["shoulder"][arm]
    ox, oy = _rot(heading, sx, sy)
    Sx, Sy = base_xy[0] + ox, base_xy[1] + oy
    Sz = shoulder_height(geom, arm, q)
    out = []
    for x, y, zw in points:
        xs, ys = _rot(-heading, x - Sx, y - Sy)
        z = zw - Sz
        r = math.hypot(math.hypot(xs, ys), z)
        out.append(float(predict_count_asym(r, math.atan2(ys, xs), z, p, z_cut)))
    return out


def ideal_spot(obj_xy, obj_z, arm, heading, q, geom):
    """The analytic single-object spot: the base origin at which the object sits straight ahead (theta = 0) at the peak distance r = r0.

    The peak is at the 3D distance r0, so the horizontal distance is d = sqrt(r0^2 - z^2) with z the object's height relative to the shoulder.
    The base is then the shoulder minus the sideways shoulder offset for this arm (turned by the heading).

    Returns:
        dict: "xy" (base origin, world), "d_horizontal" (m), "z_rel" (m), or None if the object is below the cutoff or outside the peak sphere (|z| >= r0).
    """
    p, z_cut = arm_params(arm)
    r0 = p[1]
    z = obj_z - shoulder_height(geom, arm, q)
    if z < z_cut or abs(z) >= r0:
        return None
    d = math.sqrt(r0 * r0 - z * z)
    c, s = math.cos(heading), math.sin(heading)
    sx, sy, _ = geom["shoulder"][arm]
    ox, oy = _rot(heading, sx, sy)
    return {"xy": [obj_xy[0] - d * c - ox, obj_xy[1] - d * s - oy], "d_horizontal": d, "z_rel": z}


def inverse_stances(objects, arm, heading, q, geom, n_segment=50, top=8):
    """Ideal spots for each object, then the best shared stances on the segment between the first two.

    Args:
        objects (list of dict): {"name", "xy" (world), "z" (the grasp or set-down point's world height)}.
        arm (str), heading (rad), q (m), geom: as in counts_at.
        n_segment (int): Points sampled on the segment between the first two ideal spots. Defaults to 50.
        top (int): Candidates returned. Defaults to 8.

    Returns:
        dict: "ideal" (per object: its spot, or None), "stop" (None, or "no shared stance, repositioning needed", "no feasible shared stance on the segment", or the reason an object has no spot),
        "single_stance_feasible" (bool: some segment point has every object's count >= 1), "feasible_points" (int), "candidates" (list of dict: "stance" (x, y, heading, q), "counts" ({name: count}), "min_count", "total_count").
    """
    ideal = {o["name"]: ideal_spot(o["xy"], o["z"], arm, heading, q, geom) for o in objects}
    out = {"ideal": ideal, "stop": None, "single_stance_feasible": False, "feasible_points": 0, "candidates": []}
    missing = [n for n, s in ideal.items() if s is None]
    if missing:
        out["stop"] = "no ideal spot (outside the cutoff or the peak sphere): " + ", ".join(missing)
        return out
    A, B = ideal[objects[0]["name"]]["xy"], ideal[objects[1]["name"]]["xy"]
    if math.hypot(B[0] - A[0], B[1] - A[1]) > 2 * arm_params(arm)[0][1]:
        out["stop"] = "no shared stance, repositioning needed"
        return out
    names = [o["name"] for o in objects]
    pts = [(o["xy"][0], o["xy"][1], o["z"]) for o in objects]
    scored = []
    for t in np.linspace(0.0, 1.0, n_segment):
        base = (A[0] + t * (B[0] - A[0]), A[1] + t * (B[1] - A[1]))
        counts = [counts_at(base, arm, heading, q, geom, [p])[0] for p in pts]
        if min(counts) >= 1.0:
            scored.append((min(counts), sum(counts), base, counts))
    out["feasible_points"] = len(scored)
    if not scored:
        out["stop"] = "no feasible shared stance on the segment"
        return out
    out["single_stance_feasible"] = True
    scored.sort(key=lambda s: (-s[0], -s[1]))
    for mn, tot, base, counts in scored[:top]:
        out["candidates"].append({"stance": [float(base[0]), float(base[1]), float(heading), float(q)], "counts": dict(zip(names, counts)), "min_count": mn, "total_count": tot})
    return out


# ---------------------------------------------------------------- the sweep: heading and pedestal are search variables (mid-course change)
HEADINGS = tuple(math.radians(d) for d in range(0, 360, 10))       # 36 headings, 0 to 350 deg
PEDESTALS = tuple(float(v) for v in np.linspace(0.05, 0.25, 5))    # 5 pedestal commands, m
N_SEGMENT_SWEEP = 50


def _scan(objects, arm, heading, q, geom, n_segment):
    """The segment points of one (heading, pedestal) combination: (scored, status). scored = [(min, total, base, counts)] for feasible points."""
    ideal = [ideal_spot(o["xy"], o["z"], arm, heading, q, geom) for o in objects]
    if any(s is None for s in ideal):
        return [], "no ideal spot"
    A, B = ideal[0]["xy"], ideal[1]["xy"]
    if math.hypot(B[0] - A[0], B[1] - A[1]) > 2 * arm_params(arm)[0][1]:
        return [], "ideal spots more than 2 r0 apart"
    pts = [(o["xy"][0], o["xy"][1], o["z"]) for o in objects]
    scored = []
    for t in np.linspace(0.0, 1.0, n_segment):
        base = (A[0] + t * (B[0] - A[0]), A[1] + t * (B[1] - A[1]))
        counts = [counts_at(base, arm, heading, q, geom, [p])[0] for p in pts]
        if min(counts) >= 1.0:
            scored.append((min(counts), sum(counts), base, counts))
    return scored, "ok" if scored else "no feasible point on the segment"


def single_object_stance(obj_xy, obj_z, arm, geom, heading=None, pedestal=None):
    """Part C2 with the sweep: the ideal spot of one object, optimized over heading and pedestal when they are not given.

    The peak count is C at every (heading, pedestal) where the spot exists, so the choice is by margin: the larger of (r0 - |z|) and (z - z_cut), in metres,
    the distance of the object from the peak sphere's edge and from the cutoff. Ties go to the lower pedestal, then the earlier heading in the grid.

    Returns:
        dict: "heading", "pedestal", "spot" (the ideal spot dict of single-object ideal_spot), "margin_m"; or None if no combination gives a spot.
    """
    headings = HEADINGS if heading is None else (heading,)
    pedestals = PEDESTALS if pedestal is None else (pedestal,)
    p, z_cut = arm_params(arm)
    r0 = p[1]
    best = None
    for h in headings:
        for q in pedestals:
            s = ideal_spot(obj_xy, obj_z, arm, h, q, geom)
            if s is None:
                continue
            z = s["z_rel"]
            margin = min(r0 - abs(z), z - z_cut)
            key = (margin, -q)
            if best is None or key > best[0]:
                best = (key, {"heading": float(h), "pedestal": float(q), "spot": s, "margin_m": float(margin)})
    return None if best is None else best[1]


def inverse_search(objects, arm, geom, n_segment=N_SEGMENT_SWEEP, top=8):
    """The sweep: every (heading, pedestal) combination, its segment between the two objects' ideal spots, ranked together.

    The ranking is by the worst object's count (largest first), then the total count. If no combination gives a feasible point (or no combination has both spots),
    the result is "repositioning needed": the best single-object stance of each object, each with its own heading and pedestal.

    Returns:
        dict: "label" ("shared" or "repositioning needed"), "evaluations" (segment points evaluated), "feasible_points", "best_heading", "best_pedestal" (of the first candidate,
        or None), "candidates" (top, each {"stance": (x, y, heading, q), "heading_deg", "pedestal", "counts", "min_count", "total_count"}), "split" ({name: single-object stance} or None),
        "status_counts" (how many combinations ended with each status).
    """
    names = [o["name"] for o in objects]
    scored_all = []
    evaluations = 0
    status_counts = {}
    for h in HEADINGS:
        for q in PEDESTALS:
            scored, status = _scan(objects, arm, h, q, geom, n_segment)
            status_counts[status] = status_counts.get(status, 0) + 1
            evaluations += n_segment if status != "no ideal spot" else 0
            for mn, tot, base, counts in scored:
                scored_all.append((mn, tot, base, counts, h, q))
    out = {"label": "shared", "evaluations": evaluations, "feasible_points": len(scored_all), "best_heading": None, "best_pedestal": None,
           "candidates": [], "split": None, "status_counts": status_counts}
    if not scored_all:
        out["label"] = "repositioning needed"
        out["split"] = {}
        for o in objects:
            best = single_object_stance(o["xy"], o["z"], arm, geom)
            out["split"][o["name"]] = best
        return out
    scored_all.sort(key=lambda s: (-s[0], -s[1]))
    for mn, tot, base, counts, h, q in scored_all[:top]:
        out["candidates"].append({"stance": (float(base[0]), float(base[1]), float(h), float(q)), "heading_deg": math.degrees(h), "pedestal": float(q),
                                  "counts": dict(zip(names, counts)), "min_count": mn, "total_count": tot})
    out["best_heading"] = out["candidates"][0]["heading_deg"]
    out["best_pedestal"] = out["candidates"][0]["pedestal"]
    return out
