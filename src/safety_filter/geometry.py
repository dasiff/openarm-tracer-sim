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
