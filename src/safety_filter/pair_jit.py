"""The look-ahead's candidate loop as one compiled function (Braking Phase J fix 2, SF-93, David).

ObstacleFeed.pairs used to run this loop in Python: the bounding box of every shape, then for every hand shape and every other shape the
separation, the threshold, the exclusions, the pair rule, the activation and the distance. Here the shapes are arrays and the candidate
loop is one Numba function: for every hand row and every other shape the separation, the threshold, the exclusion, the activation (from
the arm's wake radius) and, for the candidates, the distance with the compiled contacts of geometry_jit. The Python side builds the arrays
(once per call, from the placed shapes) and turns the few candidates into the tuples the layer uses.

Kinds: 0 capsule, 1 box, 2 cylinder (the held object: centre C, rotation R, radius r, half-length h[2]).
Arms: 0 left, 1 right, -1 any other shape (world objects). The candidates come back in the Python loop's order within each arm.
"""
import numpy as np
from numba import njit

from src.safety_filter import geometry_jit as _gj

KIND_CAPSULE, KIND_BOX, KIND_CYLINDER = 0, 1, 2
_DIRS = _gj.DIRS
_LS = _gj.LS_NORMALS


@njit(cache=True)
def aabb_of(kind, A, B, r, C, R, h):
    """Axis-aligned box (lo, hi) of one shape: the same numbers as PlacedShape.aabb (capsule, flat-ended cylinder, box)."""
    lo = np.empty(3)
    hi = np.empty(3)
    if kind == KIND_CAPSULE:
        for k in range(3):
            lo[k] = min(A[k], B[k]) - r
            hi[k] = max(A[k], B[k]) + r
    elif kind == KIND_CYLINDER:
        for k in range(3):
            e = R[k, 2]
            ext = r * np.sqrt(max(1.0 - e * e, 0.0)) + h[2] * abs(e)
            lo[k] = C[k] - ext
            hi[k] = C[k] + ext
    else:
        for k in range(3):
            ext = 0.0
            for j in range(3):
                ext += abs(R[k, j]) * h[j]
            lo[k] = C[k] - ext
            hi[k] = C[k] + ext
    return lo, hi


@njit(cache=True)
def aabb_all(kind, A, B, r, C, R, h):
    """(lo, hi), each (n, 3): the bounding boxes of all shapes."""
    n = kind.shape[0]
    lo = np.empty((n, 3))
    hi = np.empty((n, 3))
    for s in range(n):
        lo[s], hi[s] = aabb_of(kind[s], A[s], B[s], r[s], C[s], R[s], h[s])
    return lo, hi


@njit(cache=True)
def pair_geometry(ka, Aa, Ba, ra, Ca, Ra, ha, kb, Ab, Bb, rb, Cb, Rb, hb):
    """(gap, point on a, point on b, push) of a pair, with the conventions of touch.gap_between and touch._cylinder_gap. A box against a
    box is never a checked pair (the loop skips it)."""
    if ka == KIND_CYLINDER or kb == KIND_CYLINDER:
        if ka == KIND_CYLINDER:
            if kb == KIND_CAPSULE:
                g, p1, p2, pu = _gj.cyl_cap_contact(Ca, Ra, ra, ha[2], Ab, Bb, rb)
            else:
                g, p1, p2, pu = _gj.cyl_box_contact(Ca, Ra, ra, ha[2], Cb, Rb, hb, _DIRS, _LS)
            return g, p1, p2, -pu
        if ka == KIND_CAPSULE:
            g, p1, p2, pu = _gj.cyl_cap_contact(Cb, Rb, rb, hb[2], Aa, Ba, ra)
        else:
            g, p1, p2, pu = _gj.cyl_box_contact(Cb, Rb, rb, hb[2], Ca, Ra, ha, _DIRS, _LS)
        return g, p2, p1, pu
    if ka == KIND_CAPSULE and kb == KIND_CAPSULE:
        return _gj.cc_contact(Aa, Ba, ra, Ab, Bb, rb)
    if ka == KIND_CAPSULE and kb == KIND_BOX:
        return _gj.cb_contact(Aa, Ba, ra, Cb, Rb, hb)
    g, p1, p2, pu = _gj.cb_contact(Ab, Bb, rb, Ca, Ra, ha)      # box a, capsule b
    return g, p2, p1, -pu


@njit(cache=True)
def look_kernel(rows, rows_arm, kind, arm, A, B, r, C, R, h, lo, hi, excl, margin_hn, act_hn, exempt_hn,
                wake_on, wake, hold, t_static, t_push, t_margin, push_start):
    """The candidate loop. rows: the hand shapes (both arms), rows_arm: their arm (0 left, 1 right). excl: (2, n) bool, the released
    objects excluded per arm. margin_hn, act_hn, exempt_hn: (rows, n) rule values of each hand row against each shape (margin, static
    activation cutoff, exempt). wake_on and wake (2,): the adaptive wake radius per arm (None when off). hold (2,): the HOLD flag per arm.
    t_static, t_push, t_margin: the threshold constants (largest cutoff, push start beyond the largest margin, largest margin).
    Returns (count, hand row, other shape, arm, separation, gap, point on a, point on b, push, pairs whose gap was computed)."""
    n = kind.shape[0]
    H = rows.shape[0]
    thr = np.empty(2)
    for i in range(2):
        if wake_on:
            thr[i] = min(t_static, max(t_push, wake[i] + t_margin))
        else:
            thr[i] = t_static
    cap = H * n
    ro_o = np.empty(cap, np.int64)
    ib_o = np.empty(cap, np.int64)
    arm_o = np.empty(cap, np.int64)
    sep_o = np.empty(cap)
    gap_o = np.empty(cap)
    pa_o = np.empty((cap, 3))
    pb_o = np.empty((cap, 3))
    push_o = np.empty((cap, 3))
    cnt = 0
    considered = 0
    for ro in range(H):
        ia = rows[ro]
        i = rows_arm[ro]
        for ib in range(n):
            if ib == ia or arm[ib] == i:
                continue
            if kind[ia] == KIND_BOX and kind[ib] == KIND_BOX:
                continue
            sep = -1e300
            for k in range(3):
                s1 = lo[ia, k] - hi[ib, k]
                s2 = lo[ib, k] - hi[ia, k]
                if s1 > sep:
                    sep = s1
                if s2 > sep:
                    sep = s2
            if sep > thr[i]:
                continue
            if excl[i, ib] or exempt_hn[ro, ib]:
                continue
            act = act_hn[ro, ib]
            if wake_on:
                extra = margin_hn[ro, ib] if hold[i] else 0.0
                act = min(act, max(margin_hn[ro, ib] + push_start, wake[i] + extra))
            if sep > act:
                continue
            considered += 1
            g, pa, pb, pu = pair_geometry(kind[ia], A[ia], B[ia], r[ia], C[ia], R[ia], h[ia],
                                          kind[ib], A[ib], B[ib], r[ib], C[ib], R[ib], h[ib])
            ro_o[cnt] = ro
            ib_o[cnt] = ib
            arm_o[cnt] = i
            sep_o[cnt] = sep
            gap_o[cnt] = g
            pa_o[cnt] = pa
            pb_o[cnt] = pb
            push_o[cnt] = pu
            cnt += 1
    return cnt, ro_o, ib_o, arm_o, sep_o, gap_o, pa_o, pb_o, push_o, considered
