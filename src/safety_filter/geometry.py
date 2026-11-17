"""Distances between capsules and boxes (numbers in, numbers out; no simulator).

Conventions
  gap      signed distance between the two shapes minus nothing: negative when they overlap.
           For capsules: segment distance minus the two radii. For capsule and box: the distance from the
           segment to the box minus the radius (negative inside: depth of the shortest way out, approximated
           by sampling the segment; see tests for the measured error against MuJoCo).
  push     unit vector pointing from the second shape to the first at their closest points: moving the
           first shape along it by -gap separates them. Deep overlap: for two capsules whose lines cross,
           the cross product of their directions (perpendicular to both lines); for a capsule inside a
           box, the normal of the box face the capsule is nearest to (shortest way out).
  points   the closest point on the first shape and on the second shape.

Box: centre c (3,), rotation R (3x3, columns are the box axes in the world), half sizes h (3,).
"""

from dataclasses import dataclass

import numpy as np

from src.safety_filter import config as cfg   # Phase F fix 4 (SF-81): the public contacts run compiled when cfg.GEOMETRY["compiled"]


@dataclass(frozen=True)
class Contact:
    gap: float          # metres; negative when overlapping
    point_a: np.ndarray  # closest point on the first shape (world)
    point_b: np.ndarray  # closest point on the second shape (world)
    push: np.ndarray     # unit vector: push the first shape along it to separate the pair


def _unit(v, fallback):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else fallback


def closest_points_segments(p1, q1, p2, q2):
    """Closest points between segments [p1, q1] and [p2, q2] (Ericson's method, clamped)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e = d1 @ d1, d2 @ d2
    f = d2 @ r
    eps = 1e-14
    if a <= eps and e <= eps:
        s = t = 0.0
    elif a <= eps:
        s, t = 0.0, np.clip(f / e, 0.0, 1.0)
    else:
        c = d1 @ r
        if e <= eps:
            t, s = 0.0, np.clip(-c / a, 0.0, 1.0)
        else:
            b = d1 @ d2
            denom = a * e - b * b
            s = np.clip((b * f - c * e) / denom, 0.0, 1.0) if denom > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, np.clip(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t, s = 1.0, np.clip((b - c) / a, 0.0, 1.0)
    return p1 + s * d1, p2 + t * d2


def _py_capsule_capsule(A1, B1, r1, A2, B2, r2):
    """Python reference of capsule_capsule (Phase F: the compiled one is geometry_jit.cc_contact; this one stays the reference)."""
    A1, B1, A2, B2 = (np.asarray(x, float) for x in (A1, B1, A2, B2))
    p1, p2 = closest_points_segments(A1, B1, A2, B2)
    d = p1 - p2
    dist = float(np.linalg.norm(d))
    gap = dist - (r1 + r2)
    u, v = B1 - A1, B2 - A2
    if dist > 1e-12:
        push = d / dist
    else:
        # the lines meet: push perpendicular to both lines (the shortest way out for crossing lines)
        n = np.cross(u, v)
        if np.linalg.norm(n) < 1e-9 * max(np.linalg.norm(u) * np.linalg.norm(v), 1e-12):
            # parallel lines: any direction perpendicular to the first capsule's line
            ax = np.eye(3)[int(np.argmin(np.abs(u / max(np.linalg.norm(u), 1e-12))))]
            n = np.cross(u, ax)
        push = _unit(n, np.array([0.0, 0.0, 1.0]))
    return Contact(gap=gap, point_a=p1, point_b=p2, push=push)


def _to_box(c, R, p):
    return R.T @ (np.asarray(p, float) - c)


def _sd_box_local(p, h):
    """Signed distance from local point p to the box [-h, h]: positive outside, negative inside."""
    q = np.abs(p) - h
    outside = np.linalg.norm(np.maximum(q, 0.0))
    inside = min(max(q[0], q[1], q[2]), 0.0)
    return outside + inside


def _py_capsule_box(A, B, r, c, R, h):
    """Python reference of capsule_box (Phase F: the compiled one is geometry_jit.cb_contact; this one stays the reference)."""
    A, B = np.asarray(A, float), np.asarray(B, float)
    c, h = np.asarray(c, float), np.asarray(h, float)
    R = np.asarray(R, float)
    a, b = _to_box(c, R, A), _to_box(c, R, B)
    # the signed distance along the segment, on a fine grid (vectorised; 4001 samples = 0.075 mm steps on a
    # 0.3 m segment; the tests compare it with a 4001-sample brute force and with MuJoCo)
    ts = np.linspace(0.0, 1.0, 4001)
    pts = a[None, :] + ts[:, None] * (b - a)[None, :]
    q = np.abs(pts) - h[None, :]
    sd = np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)
    t = ts[int(np.argmin(sd))]
    p = a + t * (b - a)               # closest segment point, box frame
    sd_p = _sd_box_local(p, h)
    if sd_p > 0.0:
        q = np.clip(p, -h, h)         # closest box point
        n_local = _unit(p - q, np.array([0.0, 0.0, 1.0]))
    else:
        # inside: the face the point is nearest to is the shortest way out
        depth = h - np.abs(p)
        i = int(np.argmin(depth))
        q = p.copy()
        q[i] = np.sign(p[i]) * h[i] if p[i] != 0.0 else h[i]
        n_local = np.zeros(3)
        n_local[i] = 1.0 if p[i] >= 0.0 else -1.0
    gap = sd_p - r
    point_a = A + t * (B - A)
    point_b = c + R @ q
    return Contact(gap=float(gap), point_a=point_a, point_b=point_b, push=R @ n_local)


# ---------- batch versions (vectorised over N pairs; the same results as the scalar functions) ----------

def _unit_rows(v, fallback):
    n = np.linalg.norm(v, axis=1)
    out = np.where(n[:, None] > 1e-12, v / np.maximum(n, 1e-300)[:, None], fallback)
    return out


def capsule_capsule_batch(A1, B1, r1, A2, B2, r2, gap_only=False):
    """Batch of capsule_capsule. Inputs: (N,3) ends and (N,) radii. Returns gap (N,), point_a (N,3), point_b (N,3), push (N,3)."""
    A1, B1, A2, B2 = (np.asarray(x, float) for x in (A1, B1, A2, B2))
    r1, r2 = np.asarray(r1, float), np.asarray(r2, float)
    d1, d2, r = B1 - A1, B2 - A2, A1 - A2
    a = np.einsum("ij,ij->i", d1, d1)
    e = np.einsum("ij,ij->i", d2, d2)
    f = np.einsum("ij,ij->i", d2, r)
    c = np.einsum("ij,ij->i", d1, r)
    b = np.einsum("ij,ij->i", d1, d2)
    eps = 1e-14
    a_s, e_s = np.maximum(a, eps), np.maximum(e, eps)
    denom = a_s * e_s - b * b
    s = np.where(denom > eps, np.clip((b * f - c * e_s) / np.maximum(denom, eps), 0.0, 1.0), 0.0)
    t = (b * s + f) / e_s
    # clamp t to [0, 1] and recompute s for the clamped end
    s_for_t0 = np.clip(-c / a_s, 0.0, 1.0)
    s_for_t1 = np.clip((b - c) / a_s, 0.0, 1.0)
    lo, hi = t < 0.0, t > 1.0
    t = np.where(lo, 0.0, np.where(hi, 1.0, t))
    s = np.where(lo, s_for_t0, np.where(hi, s_for_t1, s))
    # degenerate second segment (a point at A2): the closest point on the first segment, t = 0
    s = np.where(e <= eps, s_for_t0, s)
    t = np.where(e <= eps, 0.0, t)
    p1 = A1 + s[:, None] * d1
    p2 = A2 + t[:, None] * d2
    diff = p1 - p2
    dist = np.linalg.norm(diff, axis=1)
    gap = dist - (r1 + r2)
    if gap_only:
        return gap
    cross = np.cross(d1, d2)
    fallback = np.tile(np.array([0.0, 0.0, 1.0]), (len(A1), 1))
    perp = _unit_rows(cross, fallback)
    push = np.where((dist > 1e-12)[:, None], diff / np.maximum(dist, 1e-300)[:, None], perp)
    return gap, p1, p2, push


def _sd_rows(p, h):
    q = np.abs(p) - h
    return np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)


def capsule_box_batch(A, B, r, c, R, h, grid=33, refine=14, gap_only=False):
    """Batch of capsule_box. A, B (N,3) capsule ends; r (N,); c (N,3) box centres; R (N,3,3) box rotations; h (N,3) half sizes.

    The signed distance along the segment is evaluated on a grid of `grid` points, then golden-section refined
    (`refine` steps) inside the bracket around the best grid point. Returns gap, point_a, point_b, push as capsule_box;
    with gap_only=True returns the gaps only (fewer numpy calls)."""
    A, B, c, h = (np.asarray(x, float) for x in (A, B, c, h))
    R = np.asarray(R, float)
    r = np.asarray(r, float)
    Rt = np.transpose(R, (0, 2, 1))
    a = np.einsum("nij,nj->ni", Rt, A - c)
    b = np.einsum("nij,nj->ni", Rt, B - c)
    ts = np.linspace(0.0, 1.0, grid)
    seg = b - a
    pts = a[:, None, :] + ts[None, :, None] * seg[:, None, :]          # (N, grid, 3)
    q = np.abs(pts) - h[:, None, :]
    sd = np.linalg.norm(np.maximum(q, 0.0), axis=2) + np.minimum(q.max(axis=2), 0.0)  # (N, grid)
    best = np.argmin(sd, axis=1)
    rows = np.arange(len(a))
    lo = ts[np.maximum(best - 1, 0)]
    hi = ts[np.minimum(best + 1, grid - 1)]
    gr = 0.5 * (np.sqrt(5.0) - 1.0)
    n = len(a)
    h2 = np.concatenate([h, h])
    for _ in range(refine):  # golden-section on the bracket, both probes in one stacked call
        m1 = hi - gr * (hi - lo)
        m2 = lo + gr * (hi - lo)
        both = _sd_rows(np.concatenate([a + m1[:, None] * seg, a + m2[:, None] * seg]), h2)
        left = both[:n] < both[n:]
        hi = np.where(left, m2, hi)
        lo = np.where(left, lo, m1)
    t = 0.5 * (lo + hi)
    keep_grid = _sd_rows(a + t[:, None] * seg, h) > sd[rows, best]
    t = np.where(keep_grid, ts[best], t)
    if gap_only:
        return _sd_rows(a + t[:, None] * seg, h) - r
    p = a + t[:, None] * seg                                            # closest segment point, box frame
    sd_p = _sd_rows(p, h)
    outside = sd_p > 0.0
    # outside: closest box point is the clamp; push = normalised (p - clamp), in the box frame
    qo = np.clip(p, -h, h)
    no = p - qo
    no_norm = np.linalg.norm(no, axis=1)
    n_out = np.where((no_norm > 1e-12)[:, None], no / np.maximum(no_norm, 1e-300)[:, None], np.array([0.0, 0.0, 1.0]))
    # inside: the face the point is nearest to (shortest way out)
    depth = h - np.abs(p)
    i = np.argmin(depth, axis=1)
    rows = np.arange(len(a))
    sgn = np.where(p[rows, i] >= 0.0, 1.0, -1.0)
    qi = p.copy()
    qi[rows, i] = sgn * h[rows, i]
    n_in = np.zeros_like(p)
    n_in[rows, i] = sgn
    q_box = np.where(outside[:, None], qo, qi)
    n_local = np.where(outside[:, None], n_out, n_in)
    gap = sd_p - r
    point_a = A + t[:, None] * (B - A)
    point_b = c + np.einsum("nij,nj->ni", R, q_box)
    push = np.einsum("nij,nj->ni", R, n_local)
    return gap, point_a, point_b, push


# ---------- flat-ended (solid) cylinder: centre c, axis rot[:, 2], radius rc, half height hc (Phase D fix 4, David) ----------
# The beaker's collision shape is its true cylinder: the bottom and top are flat, so a flat contact is exact. Conventions as above:
# gap negative when overlapping; push moves the SECOND shape away from the first; point_a / point_b on the first / second shape.
# Cylinder and capsule: the least signed distance along the segment (dense grid, then vectorised refinement). Cylinder and box: GJK
# on the Minkowski difference (exact to GJK_TOL_M); a deep overlap, or a touching contact on a face of the box, is handled apart.

CYL_GRID = 401              # count: samples along a segment for the cylinder-capsule minimum (the refinement finds the rest)
CYL_REFINE = 201            # count: samples per refinement round (the bracket around the best sample shrinks ~100x per round)
CYL_REFINE_ROUNDS = 5       # count: refinement rounds (5 rounds: the segment parameter to about 1e-15 of the segment)
GJK_TOL_M = 1.0e-9          # m: GJK stops when the distance is within this of its support bound (about 8 iterations)
GJK_MAX_IT = 500            # count: a GJK run that does not converge in this many steps raises (never seen in the tests)
OVERLAP_DIRS = 200          # count: fixed random directions tried for a deep overlap, after the box faces and the axis


def cyl_signed_distance(p, c, R, rc, hc):
    """Signed distance from the point p to the solid cylinder (positive outside, negative inside). Exact (closed form)."""
    q = R.T @ (np.asarray(p, float) - c)
    rho = float(np.hypot(q[0], q[1]))
    dr, dz = rho - rc, abs(float(q[2])) - hc
    return float(np.hypot(max(dr, 0.0), max(dz, 0.0)) + min(max(dr, dz), 0.0))


def cyl_sd_rows(P, c, R, rc, hc):
    """cyl_signed_distance for each row of P (N, 3)."""
    Q = (np.asarray(P, float) - c[None, :]) @ R
    rho = np.hypot(Q[:, 0], Q[:, 1])
    dr, dz = rho - rc, np.abs(Q[:, 2]) - hc
    return np.hypot(np.maximum(dr, 0.0), np.maximum(dz, 0.0)) + np.minimum(np.maximum(dr, dz), 0.0)


def nearest_cyl_point(p, c, R, rc, hc):
    """The point of the solid cylinder nearest to p (for a point outside it: the point on its surface)."""
    q = R.T @ (np.asarray(p, float) - c)
    rho = float(np.hypot(q[0], q[1]))
    if rho > rc:
        x, y = q[0] * rc / rho, q[1] * rc / rho
    elif rho > 1e-300:
        x, y = q[0], q[1]
    else:
        x, y = rc, 0.0
    return c + R @ np.array([x, y, float(np.clip(q[2], -hc, hc))])


def _seg_min_cyl(A, B, c, R, rc, hc):
    """(least signed distance from the segment A-B to the cylinder, the segment parameter t where it is)."""
    ts = np.linspace(0.0, 1.0, CYL_GRID)
    vals = cyl_sd_rows(A[None, :] + ts[:, None] * (B - A)[None, :], c, R, rc, hc)
    k = int(np.argmin(vals))
    best_v, best_t = float(vals[k]), float(ts[k])
    lo, hi = ts[max(k - 1, 0)], ts[min(k + 1, CYL_GRID - 1)]
    for _ in range(CYL_REFINE_ROUNDS):
        tt = np.linspace(lo, hi, CYL_REFINE)
        vv = cyl_sd_rows(A[None, :] + tt[:, None] * (B - A)[None, :], c, R, rc, hc)
        j = int(np.argmin(vv))
        if vv[j] <= best_v:
            best_v, best_t = float(vv[j]), float(tt[j])
        lo, hi = tt[max(j - 1, 0)], tt[min(j + 1, CYL_REFINE - 1)]
    return best_v, best_t


def _py_cylinder_capsule(c, R, rc, hc, A, B, r):
    """Python reference of cylinder_capsule (Phase F: the compiled one is geometry_jit.cyl_cap_contact; this one stays the reference).
    Gap between the solid cylinder (first shape) and the capsule (second: segment A-B, radius r): the least signed distance from the segment
    to the cylinder minus r. Returns Contact(gap, point_a on the cylinder, point_b on the segment, push: moves the capsule away from the cylinder).
    Overlapping: the gap is negative, the depth at the segment's deepest point."""
    c, R = np.asarray(c, float), np.asarray(R, float)
    A, B = np.asarray(A, float), np.asarray(B, float)
    if np.linalg.norm(B - A) < 1e-15:
        d, t = cyl_signed_distance(A, c, R, rc, hc), 0.0
    else:
        d, t = _seg_min_cyl(A, B, c, R, rc, hc)
    p = A + t * (B - A)
    if d > 0.0:
        cp = nearest_cyl_point(p, c, R, rc, hc)
        v = p - cp
        n = float(np.linalg.norm(v))
        push = v / n if n > 1e-15 else R[:, 2]
        return Contact(gap=float(d - r), point_a=cp, point_b=p, push=push)
    q = R.T @ (p - c)
    rho = float(np.hypot(q[0], q[1]))
    radial_depth, axial_depth = rc - rho, hc - abs(float(q[2]))
    if radial_depth < axial_depth:                      # the radially nearest face: its outward normal
        n_loc = np.array([q[0], q[1], 0.0]) / rho if rho > 1e-300 else np.array([1.0, 0.0, 0.0])
        face = radial_depth
    else:                                               # the top or bottom face
        n_loc = np.array([0.0, 0.0, 1.0 if q[2] >= 0.0 else -1.0])
        face = axial_depth
    push = R @ n_loc
    return Contact(gap=float(d - r), point_a=p + face * push, point_b=p, push=push)


def _sup_cyl(c, R, rc, hc, d):
    """The point of the solid cylinder furthest along the world direction d."""
    dl = R.T @ d
    rho = float(np.hypot(dl[0], dl[1]))
    x, y = (dl[0] * rc / rho, dl[1] * rc / rho) if rho > 1e-300 else (rc, 0.0)
    return c + R @ np.array([x, y, hc if dl[2] >= 0.0 else -hc])


def _sup_box(c, R, h, d):
    """The point of the solid box furthest along the world direction d."""
    dl = R.T @ d
    return c + R @ np.where(dl >= 0.0, h, -h)


def _closest_on_hull(P, newest):
    """The point of the convex hull of the rows of P (at most 4) nearest the origin, over the subsets that contain row `newest` (all subsets
    when None; the GJK rule: the new closest point lies in a subset with the newest support point). Returns (|v|^2, v, subset, weights)."""
    others = [i for i in range(len(P)) if i != newest]
    base = [newest] if newest is not None else []
    best = None
    for mask in range(1 << len(others)):
        idx = base + [others[j] for j in range(len(others)) if mask >> j & 1]
        if not idx:
            continue
        if len(idx) == 1:
            lam, v = np.array([1.0]), P[idx[0]].copy()
        elif len(idx) == 2:
            a, b = P[idx[0]], P[idx[1]]
            e = b - a
            ee = float(e @ e)
            t = 0.0 if ee < 1e-300 else min(max(-float(a @ e) / ee, 0.0), 1.0)
            lam, v = np.array([1.0 - t, t]), a + t * e
        else:
            Q = P[idx]
            p0 = Q[0]
            mu = np.linalg.lstsq((Q[1:] - p0).T, -p0, rcond=1e-12)[0]     # least squares on the differences (the Gram matrix would square the condition number)
            lam = np.concatenate([[1.0 - mu.sum()], mu])
            if np.any(lam < -1e-12):
                continue
            lam = np.clip(lam, 0.0, None)
            lam = lam / lam.sum()
            v = Q.T @ lam
        n2 = float(v @ v)
        if best is None or n2 < best[0] - 1e-30:
            best = (n2, v, idx, lam)
    return best


def _gjk_distance(sup, start):
    """Distance between two convex bodies from sup(d) -> (a on the first body, b on the second) with a - b the support of their difference along d.
    Returns (distance, point on the first, point on the second), or None when the origin lies in the difference (touching or overlapping)."""
    a, b = sup(start)
    P = np.array([a - b])
    AA, BB = [a], [b]
    newest = None
    for _ in range(GJK_MAX_IT):
        n2, v, idx, lam = _closest_on_hull(P, newest)
        P = P[idx]
        AA = [AA[i] for i in idx]
        BB = [BB[i] for i in idx]
        nv = float(np.sqrt(n2))
        if nv < 1e-12:
            return None
        a, b = sup(-v)
        w = a - b
        if n2 - float(v @ w) <= GJK_TOL_M * nv:        # no point of the difference is nearer than |v| - GJK_TOL_M
            pa = sum(float(x) * y for x, y in zip(lam, AA))
            pb = sum(float(x) * y for x, y in zip(lam, BB))
            return nv, pa, pb
        P = np.vstack([P, w[None, :]])
        AA.append(a)
        BB.append(b)
        newest = len(P) - 1
    raise RuntimeError("GJK did not converge")


def _py_cylinder_box(c, R, rc, hc, cb, Rb, hb):
    """Python reference of cylinder_box (Phase F: the compiled one is geometry_jit.cyl_box_contact; this one stays the reference).
    Gap between the solid cylinder (first shape) and the solid box (second: centre cb, rotation Rb, half sizes hb). Returns Contact(gap,
    point_a on the cylinder, point_b on the box, push: moves the box away from the cylinder). Separated: the distance (GJK, exact to GJK_TOL_M).
    Touching on a box face or the cylinder axis: gap 0. Deep overlap: minus the penetration depth, the least support of the difference over
    the box faces, the axis and OVERLAP_DIRS fixed directions, then a local search (approximate; not used in normal operation)."""
    c, R, cb, Rb, hb = (np.asarray(x, float) for x in (c, R, cb, Rb, hb))
    sup = lambda d: (_sup_cyl(c, R, rc, hc, d), _sup_box(cb, Rb, hb, -d))
    start = cb - c if float(np.linalg.norm(cb - c)) > 1e-12 else np.array([1.0, 0.0, 0.0])
    res = _gjk_distance(sup, start)
    if res is not None:
        dist, pa, pb = res
        v = pb - pa
        n = float(np.linalg.norm(v))
        return Contact(gap=float(dist), point_a=pa, point_b=pb, push=v / n if n > 1e-15 else Rb[:, 2])
    # the origin lies in the difference: a face or the axis touching, or a real overlap
    def h_diff(u):
        a, b = sup(u)
        return float((a - b) @ u)
    cands = [Rb[:, k] * s for k in range(3) for s in (1.0, -1.0)] + [R[:, 2], -R[:, 2]]
    vals = [h_diff(u) for u in cands]
    j = int(np.argmin(vals))
    if vals[j] <= 1e-10:
        a, b = sup(cands[j])
        return Contact(gap=-max(vals[j], 0.0), point_a=a, point_b=b, push=cands[j])
    g = np.random.default_rng(0).normal(size=(OVERLAP_DIRS, 3))
    cands += list(g / np.linalg.norm(g, axis=1)[:, None])
    vals = [h_diff(u) for u in cands]
    j = int(np.argmin(vals))
    u, best = cands[j], vals[j]
    rng = np.random.default_rng(1)
    step = 0.2
    for _ in range(60):
        improved = False
        for _ in range(12):
            t = u + step * rng.normal(size=3)
            t = t / np.linalg.norm(t)
            ht = h_diff(t)
            if ht < best:
                u, best, improved = t, ht, True
        if not improved:
            step *= 0.6
    a, b = sup(u)
    return Contact(gap=-float(best), point_a=a, point_b=b, push=u)


# ---------- public contacts (Phase F fix 4, SF-81): compiled when cfg.GEOMETRY["compiled"], else the Python reference above ----------

_JIT = None


def _jit():
    """geometry_jit, imported on first use (numba is slow to import, and the Python path does not need it)."""
    global _JIT
    if _JIT is None:
        from src.safety_filter import geometry_jit
        _JIT = geometry_jit
    return _JIT


def capsule_capsule(A1, B1, r1, A2, B2, r2):
    """Gap, closest points and push direction between two capsules (ends A, B, radius r)."""
    if not cfg.GEOMETRY["compiled"]:
        return _py_capsule_capsule(A1, B1, r1, A2, B2, r2)
    g, pa, pb, push = _jit().cc_contact(np.asarray(A1, float), np.asarray(B1, float), float(r1),
                                        np.asarray(A2, float), np.asarray(B2, float), float(r2))
    return Contact(gap=float(g), point_a=pa, point_b=pb, push=push)


def capsule_box(A, B, r, c, R, h):
    """Gap, closest points and push direction between a capsule (ends A, B, radius r) and a box."""
    if not cfg.GEOMETRY["compiled"]:
        return _py_capsule_box(A, B, r, c, R, h)
    g, pa, pb, push = _jit().cb_contact(np.asarray(A, float), np.asarray(B, float), float(r), np.asarray(c, float),
                                        np.asarray(R, float), np.asarray(h, float))
    return Contact(gap=float(g), point_a=pa, point_b=pb, push=push)


def cylinder_capsule(c, R, rc, hc, A, B, r):
    """Gap between the solid cylinder (first shape) and a capsule (see _py_cylinder_capsule)."""
    if not cfg.GEOMETRY["compiled"]:
        return _py_cylinder_capsule(c, R, rc, hc, A, B, r)
    g, pa, pb, push = _jit().cyl_cap_contact(np.asarray(c, float), np.asarray(R, float), float(rc), float(hc),
                                             np.asarray(A, float), np.asarray(B, float), float(r))
    return Contact(gap=float(g), point_a=pa, point_b=pb, push=push)


def cylinder_box(c, R, rc, hc, cb, Rb, hb):
    """Gap between the solid cylinder (first shape) and a solid box (see _py_cylinder_box)."""
    if not cfg.GEOMETRY["compiled"]:
        return _py_cylinder_box(c, R, rc, hc, cb, Rb, hb)
    j = _jit()
    g, pa, pb, push = j.cyl_box_contact(np.asarray(c, float), np.asarray(R, float), float(rc), float(hc), np.asarray(cb, float),
                                        np.asarray(Rb, float), np.asarray(hb, float), j.DIRS, j.LS_NORMALS)
    return Contact(gap=float(g), point_a=pa, point_b=pb, push=push)
