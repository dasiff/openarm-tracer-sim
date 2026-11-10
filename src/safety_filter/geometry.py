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


def capsule_capsule(A1, B1, r1, A2, B2, r2):
    """Gap, closest points and push direction between two capsules (ends A, B, radius r)."""
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


def capsule_box(A, B, r, c, R, h):
    """Gap, closest points and push direction between a capsule (ends A, B, radius r) and a box."""
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
