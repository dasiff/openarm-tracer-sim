"""Compiled gap-only batch distances for the tick's approximate gaps (Braking Phase K fix 2, SF-97, David).

tick_arrays.approx_gaps asks for the gaps of many capsule-capsule and capsule-box pairs at once. geometry.capsule_capsule_batch and
geometry.capsule_box_batch do that with dozens of numpy calls per call (a 33-point grid, then golden-section steps). These are the same
computations as loops over the pairs (the same formulas, the same grid, the same golden-section steps and tie rules), compiled with Numba.
The numpy versions stay as the reference: tests compare the two on random and recorded poses.
"""
import numpy as np
from numba import njit

from src.safety_filter.pair_jit import pair_geometry


@njit(cache=True)
def capsule_capsule_gap(A1, B1, r1, A2, B2, r2):
    """Gap of each pair of capsules (ends A, B, radius r): the same as geometry.capsule_capsule_batch(..., gap_only=True)."""
    n = A1.shape[0]
    out = np.empty(n)
    eps = 1e-14
    for k in range(n):
        d1x, d1y, d1z = B1[k, 0] - A1[k, 0], B1[k, 1] - A1[k, 1], B1[k, 2] - A1[k, 2]
        d2x, d2y, d2z = B2[k, 0] - A2[k, 0], B2[k, 1] - A2[k, 1], B2[k, 2] - A2[k, 2]
        rx, ry, rz = A1[k, 0] - A2[k, 0], A1[k, 1] - A2[k, 1], A1[k, 2] - A2[k, 2]
        a = d1x * d1x + d1y * d1y + d1z * d1z
        e = d2x * d2x + d2y * d2y + d2z * d2z
        f = d2x * rx + d2y * ry + d2z * rz
        c = d1x * rx + d1y * ry + d1z * rz
        b = d1x * d2x + d1y * d2y + d1z * d2z
        a_s = max(a, eps)
        e_s = max(e, eps)
        denom = a_s * e_s - b * b
        if denom > eps:
            s = min(max((b * f - c * e_s) / max(denom, eps), 0.0), 1.0)
        else:
            s = 0.0
        t = (b * s + f) / e_s
        s_t0 = min(max(-c / a_s, 0.0), 1.0)
        s_t1 = min(max((b - c) / a_s, 0.0), 1.0)
        if t < 0.0:
            t = 0.0
            s = s_t0
        elif t > 1.0:
            t = 1.0
            s = s_t1
        if e <= eps:
            s = s_t0
            t = 0.0
        p1x, p1y, p1z = A1[k, 0] + s * d1x, A1[k, 1] + s * d1y, A1[k, 2] + s * d1z
        p2x, p2y, p2z = A2[k, 0] + t * d2x, A2[k, 1] + t * d2y, A2[k, 2] + t * d2z
        dx, dy, dz = p1x - p2x, p1y - p2y, p1z - p2z
        dist = np.sqrt(dx * dx + dy * dy + dz * dz)
        out[k] = dist - (r1[k] + r2[k])
    return out


@njit(cache=True)
def _sd_point(px, py, pz, hx, hy, hz):
    qx, qy, qz = abs(px) - hx, abs(py) - hy, abs(pz) - hz
    mx, my, mz = max(qx, 0.0), max(qy, 0.0), max(qz, 0.0)
    qmax = max(qx, max(qy, qz))
    return np.sqrt(mx * mx + my * my + mz * mz) + min(qmax, 0.0)


@njit(cache=True)
def capsule_box_gap(A, B, r, c, R, h, ts, refine):
    """Gap of each capsule-box pair (box centre c, rotation R, half sizes h): the same as geometry.capsule_box_batch(..., gap_only=True).
    ts: the grid of the segment parameter (np.linspace(0, 1, grid), computed by the caller, the same grid as the numpy version)."""
    n = A.shape[0]
    grid = ts.shape[0]
    out = np.empty(n)
    gr = 0.5 * (np.sqrt(5.0) - 1.0)
    for k in range(n):
        dx, dy, dz = A[k, 0] - c[k, 0], A[k, 1] - c[k, 1], A[k, 2] - c[k, 2]
        ex, ey, ez = B[k, 0] - c[k, 0], B[k, 1] - c[k, 1], B[k, 2] - c[k, 2]
        # a = R^T (A - c), b = R^T (B - c): the box frame of the two segment ends
        ax = R[k, 0, 0] * dx + R[k, 1, 0] * dy + R[k, 2, 0] * dz
        ay = R[k, 0, 1] * dx + R[k, 1, 1] * dy + R[k, 2, 1] * dz
        az = R[k, 0, 2] * dx + R[k, 1, 2] * dy + R[k, 2, 2] * dz
        bx = R[k, 0, 0] * ex + R[k, 1, 0] * ey + R[k, 2, 0] * ez
        by = R[k, 0, 1] * ex + R[k, 1, 1] * ey + R[k, 2, 1] * ez
        bz = R[k, 0, 2] * ex + R[k, 1, 2] * ey + R[k, 2, 2] * ez
        sx, sy, sz = bx - ax, by - ay, bz - az                              # the segment in the box frame
        hx, hy, hz = h[k, 0], h[k, 1], h[k, 2]
        best = 0
        best_sd = np.inf
        for g in range(grid):                                               # the grid: first minimum, as np.argmin
            t = ts[g]
            sd = _sd_point(ax + t * sx, ay + t * sy, az + t * sz, hx, hy, hz)
            if sd < best_sd:
                best_sd = sd
                best = g
        lo = ts[max(best - 1, 0)]
        hi = ts[min(best + 1, grid - 1)]
        for _ in range(refine):                                             # golden section on the bracket
            m1 = hi - gr * (hi - lo)
            m2 = lo + gr * (hi - lo)
            sd1 = _sd_point(ax + m1 * sx, ay + m1 * sy, az + m1 * sz, hx, hy, hz)
            sd2 = _sd_point(ax + m2 * sx, ay + m2 * sy, az + m2 * sz, hx, hy, hz)
            if sd1 < sd2:
                hi = m2
            else:
                lo = m1
        t = 0.5 * (lo + hi)
        sd_t = _sd_point(ax + t * sx, ay + t * sy, az + t * sz, hx, hy, hz)
        if sd_t > best_sd:                                                  # keep the grid point when the refinement did not beat it
            t = ts[best]
            sd_t = _sd_point(ax + t * sx, ay + t * sy, az + t * sz, hx, hy, hz)
        out[k] = sd_t - r[k]
    return out


def capsule_capsule_gap_batch(A1, B1, r1, A2, B2, r2):
    """Python entry of capsule_capsule_gap (float arrays, C-contiguous)."""
    f = lambda x: np.ascontiguousarray(x, dtype=float)                       # noqa: E731
    return capsule_capsule_gap(f(A1), f(B1), f(r1), f(A2), f(B2), f(r2))


def capsule_box_gap_batch(A, B, r, c, R, h, grid=33, refine=14):
    """Python entry of capsule_box_gap (the grid of the numpy version: linspace(0, 1, grid), refine golden-section steps)."""
    f = lambda x: np.ascontiguousarray(x, dtype=float)                       # noqa: E731
    return capsule_box_gap(f(A), f(B), f(r), f(c), f(R), f(h), np.linspace(0.0, 1.0, grid), refine)


@njit(cache=True)
def pair_gap_at(a, b, kind, A, B, C, R, radius, half):
    """Phase L fix 2 (SF-100): the exact gap of the pair of table items a and b at a pose (kind codes, capsule ends A and B, centres C,
    rotations R, radii, half sizes: the tick's item arrays). The same contact functions as geometry.capsule_capsule, capsule_box,
    cylinder_capsule and cylinder_box (through pair_jit.pair_geometry), so the same numbers."""
    g, p1, p2, pu = pair_geometry(kind[a], A[a], B[a], radius[a], C[a], R[a], half[a], kind[b], A[b], B[b], radius[b], C[b], R[b], half[b])
    return g
