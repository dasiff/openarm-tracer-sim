"""Compiled (numba) contacts of geometry.py (Braking Phase F fix 4): gap, closest points and push direction, from shape parameters.

The same algorithms and constants as the Python functions in geometry.py (which stay the reference: _py_capsule_capsule and the others;
experiments/safety_filter_jit_test.py compares every compiled value with them to 1e-6 m). Inputs are arrays and floats; outputs are
tuples of floats and arrays. No Python objects inside the compiled code.

Compiled: capsule-capsule (closed form), capsule-box (4001-sample minimum), cylinder-capsule (sampled minimum and refinement),
cylinder-box (GJK on the Minkowski difference, and the overlap search). The overlap search's random directions and draws are
precomputed here with the same seeds and order as geometry.py, so the compiled code needs no random generator.

The constants below equal geometry.py's (the test checks them).
"""

import numpy as np
from numba import njit

CAP_BOX_SAMPLES = 4001          # count: samples along the segment for capsule-box
GJK_TOL_M = 1.0e-9              # m: GJK stop tolerance
GJK_MAX_IT = 500                # count: GJK iteration limit
CYL_GRID = 401                  # count: cylinder-capsule grid samples along the segment
CYL_REFINE = 201                # count: cylinder-capsule refinement samples per round
CYL_REFINE_ROUNDS = 5           # count: refinement rounds
OVERLAP_DIRS = 200              # count: fixed random directions for a deep overlap
LS_OUTER, LS_INNER = 60, 12     # count: the overlap local search: 60 rounds of 12 trial directions

# the overlap search's directions and draws, as geometry.cylinder_box draws them (seed 0 for the directions, seed 1 for the local search)
_DIRS_RAW = np.random.default_rng(0).normal(size=(OVERLAP_DIRS, 3))
DIRS = np.ascontiguousarray(_DIRS_RAW / np.linalg.norm(_DIRS_RAW, axis=1)[:, None])
LS_NORMALS = np.ascontiguousarray(np.random.default_rng(1).normal(size=(LS_OUTER * LS_INNER, 3)))
_Z = np.array([0.0, 0.0, 1.0])
_X = np.array([1.0, 0.0, 0.0])


@njit(cache=True)
def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


@njit(cache=True)
def _norm(a):
    return np.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


@njit(cache=True)
def _cross(a, b):
    out = np.empty(3)
    out[0] = a[1] * b[2] - a[2] * b[1]
    out[1] = a[2] * b[0] - a[0] * b[2]
    out[2] = a[0] * b[1] - a[1] * b[0]
    return out


@njit(cache=True)
def _matvec(R, v):
    out = np.empty(3)
    for i in range(3):
        out[i] = R[i, 0] * v[0] + R[i, 1] * v[1] + R[i, 2] * v[2]
    return out


@njit(cache=True)
def _mattvec(R, v):
    out = np.empty(3)
    for i in range(3):
        out[i] = R[0, i] * v[0] + R[1, i] * v[1] + R[2, i] * v[2]
    return out


@njit(cache=True)
def _unit(v, fallback):
    n = _norm(v)
    if n > 1e-12:
        return v / n
    return fallback.copy()


@njit(cache=True)
def _clip01(x):
    if x < 0.0:
        return 0.0
    if x > 1.0:
        return 1.0
    return x


# ---------- capsule against capsule ----------

@njit(cache=True)
def _closest_seg(p1, q1, p2, q2):
    """Closest points of the segments [p1, q1] and [p2, q2] (geometry.closest_points_segments)."""
    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2
    a = _dot(d1, d1)
    e = _dot(d2, d2)
    f = _dot(d2, r)
    eps = 1e-14
    s = 0.0
    t = 0.0
    if a <= eps and e <= eps:
        s = 0.0
        t = 0.0
    elif a <= eps:
        s = 0.0
        t = _clip01(f / e)
    else:
        c = _dot(d1, r)
        if e <= eps:
            t = 0.0
            s = _clip01(-c / a)
        else:
            b = _dot(d1, d2)
            denom = a * e - b * b
            s = _clip01((b * f - c * e) / denom) if denom > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = _clip01(-c / a)
            elif t > 1.0:
                t = 1.0
                s = _clip01((b - c) / a)
    return p1 + s * d1, p2 + t * d2


@njit(cache=True)
def cc_contact(A1, B1, r1, A2, B2, r2):
    """(gap, point on the first capsule, point on the second, push) for two capsules (geometry.capsule_capsule)."""
    p1, p2 = _closest_seg(A1, B1, A2, B2)
    d = p1 - p2
    dist = _norm(d)
    gap = dist - (r1 + r2)
    u = B1 - A1
    v = B2 - A2
    if dist > 1e-12:
        push = d / dist
    else:
        n = _cross(u, v)
        nu = _norm(u)
        nw = _norm(v)
        if _norm(n) < 1e-9 * max(nu * nw, 1e-12):
            # parallel lines: a direction perpendicular to the first capsule's line (the axis it is least along)
            k = 0
            best = abs(u[0]) / max(nu, 1e-12)
            for i in range(1, 3):
                val = abs(u[i]) / max(nu, 1e-12)
                if val < best:
                    best = val
                    k = i
            ax = np.zeros(3)
            ax[k] = 1.0
            n = _cross(u, ax)
        push = _unit(n, _Z)
    return gap, p1, p2, push


# ---------- capsule against box ----------

@njit(cache=True)
def _sd_box_local(p, h):
    """Signed distance from the local point p to the box [-h, h] (positive outside, negative inside)."""
    outside = 0.0
    inside = -1e300
    for i in range(3):
        q = abs(p[i]) - h[i]
        if q > 0.0:
            outside += q * q
        if q > inside:
            inside = q
    return np.sqrt(outside) + min(inside, 0.0)


@njit(cache=True)
def cb_contact(A, B, r, c, R, h):
    """(gap, point on the capsule, point on the box, push) for a capsule and a box (geometry.capsule_box)."""
    a = _mattvec(R, A - c)
    b = _mattvec(R, B - c)
    seg = b - a
    best = np.inf
    kbest = 0
    for k in range(CAP_BOX_SAMPLES):
        t = k / (CAP_BOX_SAMPLES - 1)
        sd = _sd_box_local(a + t * seg, h)
        if sd < best:
            best = sd
            kbest = k
    t = kbest / (CAP_BOX_SAMPLES - 1)
    p = a + t * seg
    sd_p = _sd_box_local(p, h)
    q = np.empty(3)
    n_local = np.zeros(3)
    if sd_p > 0.0:
        for i in range(3):
            q[i] = min(max(p[i], -h[i]), h[i])
        n_local = _unit(p - q, _Z)
    else:
        # inside: the face the point is nearest to is the shortest way out
        ib = 0
        dbest = h[0] - abs(p[0])
        for i in range(1, 3):
            di = h[i] - abs(p[i])
            if di < dbest:
                dbest = di
                ib = i
        q[:] = p
        if p[ib] != 0.0:
            q[ib] = (1.0 if p[ib] > 0.0 else -1.0) * h[ib]
        else:
            q[ib] = h[ib]
        n_local[ib] = 1.0 if p[ib] >= 0.0 else -1.0
    gap = sd_p - r
    pa = A + t * (B - A)
    pb = c + _matvec(R, q)
    push = _matvec(R, n_local)
    return gap, pa, pb, push


# ---------- solid cylinder (centre c, axis R[:, 2], radius rc, half height hc) ----------

@njit(cache=True)
def _cyl_sd(p, c, R, rc, hc):
    """Signed distance from the point p to the solid cylinder (positive outside, negative inside; closed form)."""
    q = _mattvec(R, p - c)
    rho = np.hypot(q[0], q[1])
    dr = rho - rc
    dz = abs(q[2]) - hc
    return np.hypot(max(dr, 0.0), max(dz, 0.0)) + min(max(dr, dz), 0.0)


@njit(cache=True)
def _nearest_cyl(p, c, R, rc, hc):
    """The point of the solid cylinder nearest to p (for a point outside it: on its surface)."""
    q = _mattvec(R, p - c)
    rho = np.hypot(q[0], q[1])
    if rho > rc:
        x = q[0] * rc / rho
        y = q[1] * rc / rho
    elif rho > 1e-300:
        x = q[0]
        y = q[1]
    else:
        x = rc
        y = 0.0
    loc = np.empty(3)
    loc[0] = x
    loc[1] = y
    loc[2] = min(max(q[2], -hc), hc)
    return c + _matvec(R, loc)


@njit(cache=True)
def _seg_min_cyl(A, B, c, R, rc, hc):
    """(least signed distance from the segment A-B to the cylinder, the segment parameter t where it is): geometry._seg_min_cyl."""
    best_v = np.inf
    best_t = 0.0
    k = 0
    for i in range(CYL_GRID):
        t = i / (CYL_GRID - 1)
        v = _cyl_sd(A + t * (B - A), c, R, rc, hc)
        if v < best_v:
            best_v = v
            best_t = t
            k = i
    lo = max(k - 1, 0) / (CYL_GRID - 1)
    hi = min(k + 1, CYL_GRID - 1) / (CYL_GRID - 1)
    for _ in range(CYL_REFINE_ROUNDS):
        jbest = 0
        vbest = np.inf
        for i in range(CYL_REFINE):
            t = lo + (hi - lo) * i / (CYL_REFINE - 1)
            v = _cyl_sd(A + t * (B - A), c, R, rc, hc)
            if v < vbest:
                vbest = v
                jbest = i
        if vbest <= best_v:
            best_v = vbest
            best_t = lo + (hi - lo) * jbest / (CYL_REFINE - 1)
        old_lo = lo
        old_hi = hi
        lo = old_lo + (old_hi - old_lo) * max(jbest - 1, 0) / (CYL_REFINE - 1)
        hi = old_lo + (old_hi - old_lo) * min(jbest + 1, CYL_REFINE - 1) / (CYL_REFINE - 1)
    return best_v, best_t


@njit(cache=True)
def cyl_cap_contact(c, R, rc, hc, A, B, r):
    """(gap, point on the cylinder, point on the capsule, push) for a solid cylinder and a capsule (geometry.cylinder_capsule)."""
    if _norm(B - A) < 1e-15:
        d = _cyl_sd(A, c, R, rc, hc)
        t = 0.0
    else:
        d, t = _seg_min_cyl(A, B, c, R, rc, hc)
    p = A + t * (B - A)
    if d > 0.0:
        cp = _nearest_cyl(p, c, R, rc, hc)
        v = p - cp
        n = _norm(v)
        if n > 1e-15:
            push = v / n
        else:
            push = R[:, 2].copy()
        return d - r, cp, p, push
    q = _mattvec(R, p - c)
    rho = np.hypot(q[0], q[1])
    radial_depth = rc - rho
    axial_depth = hc - abs(q[2])
    n_loc = np.zeros(3)
    if radial_depth < axial_depth:
        if rho > 1e-300:
            n_loc[0] = q[0] / rho
            n_loc[1] = q[1] / rho
        else:
            n_loc[0] = 1.0
        face = radial_depth
    else:
        n_loc[2] = 1.0 if q[2] >= 0.0 else -1.0
        face = axial_depth
    push = _matvec(R, n_loc)
    return d - r, p + face * push, p, push


# ---------- solid cylinder against solid box: GJK on the Minkowski difference ----------

@njit(cache=True)
def _sup_cyl(c, R, rc, hc, d):
    """The point of the solid cylinder furthest along the world direction d (geometry._sup_cyl)."""
    dl = _mattvec(R, d)
    rho = np.hypot(dl[0], dl[1])
    loc = np.empty(3)
    if rho > 1e-300:
        loc[0] = dl[0] * rc / rho
        loc[1] = dl[1] * rc / rho
    else:
        loc[0] = rc
        loc[1] = 0.0
    loc[2] = hc if dl[2] >= 0.0 else -hc
    return c + _matvec(R, loc)


@njit(cache=True)
def _sup_box(c, R, h, d):
    """The point of the solid box furthest along the world direction d (geometry._sup_box)."""
    dl = _mattvec(R, d)
    loc = np.empty(3)
    for i in range(3):
        loc[i] = h[i] if dl[i] >= 0.0 else -h[i]
    return c + _matvec(R, loc)


@njit(cache=True)
def _solve_small(M, rhs):
    """Gauss elimination with partial pivoting for a small system. Returns (ok, x); ok is False when the system is singular to working
    precision: the caller skips that subset (a degenerate subset has the same closest point as a proper subset of it)."""
    n = rhs.shape[0]
    A = M.copy()
    b = rhs.copy()
    scale = 0.0
    for i in range(n):
        for j in range(n):
            scale = max(scale, abs(A[i, j]))
    x = np.zeros(n)
    if scale <= 0.0:
        return False, x
    for col in range(n):
        piv = col
        for i in range(col + 1, n):
            if abs(A[i, col]) > abs(A[piv, col]):
                piv = i
        if abs(A[piv, col]) <= 1e-12 * scale:
            return False, x
        if piv != col:
            for j in range(n):
                tmp = A[col, j]
                A[col, j] = A[piv, j]
                A[piv, j] = tmp
            tmp = b[col]
            b[col] = b[piv]
            b[piv] = tmp
        for i in range(col + 1, n):
            f = A[i, col] / A[col, col]
            for j in range(col, n):
                A[i, j] -= f * A[col, j]
            b[i] -= f * b[col]
    for i in range(n - 1, -1, -1):
        s = b[i]
        for j in range(i + 1, n):
            s -= A[i, j] * x[j]
        x[i] = s / A[i, i]
    return True, x


@njit(cache=True)
def _hull_point(P, idx, m, lam, v):
    """The point of the convex hull of the m points P[idx] nearest the origin (geometry._closest_on_hull for one subset). Fills lam (the
    weights of idx, sum 1) and v. Returns False when the subset gives no point (degenerate, or the point lies outside it)."""
    if m == 1:
        lam[0] = 1.0
        v[:] = P[idx[0]]
        return True
    if m == 2:
        a = P[idx[0]]
        e = P[idx[1]] - a
        ee = _dot(e, e)
        t = 0.0 if ee < 1e-300 else _clip01(-_dot(a, e) / ee)
        lam[0] = 1.0 - t
        lam[1] = t
        v[:] = a + t * e
        return True
    p0 = P[idx[0]]
    nm = m - 1
    D = np.empty((nm, 3))
    for i in range(nm):
        D[i] = P[idx[i + 1]] - p0
    Gm = np.empty((nm, nm))
    rhs = np.empty(nm)
    for i in range(nm):
        rhs[i] = -_dot(D[i], p0)
        for j in range(nm):
            Gm[i, j] = _dot(D[i], D[j])
    ok, mu = _solve_small(Gm, rhs)
    if not ok:
        return False
    total = 1.0
    for i in range(nm):
        total -= mu[i]
    lam[0] = total
    for i in range(nm):
        lam[i + 1] = mu[i]
    for i in range(m):
        if lam[i] < -1e-12:
            return False
    s = 0.0
    for i in range(m):
        lam[i] = max(lam[i], 0.0)
        s += lam[i]
    v[:] = 0.0
    for i in range(m):
        lam[i] /= s
        v += lam[i] * P[idx[i]]
    return True


@njit(cache=True)
def _gjk_contact(c, R, rc, hc, cb, Rb, hb):
    """GJK distance between the cylinder and the box (geometry._gjk_distance). Returns (True, distance, point on the cylinder, point on the
    box), or (False, 0, zeros, zeros) when the origin lies in the difference (touching or overlapping)."""
    d0 = cb - c
    if _norm(d0) <= 1e-12:
        d0 = _X.copy()
    P = np.zeros((6, 3))
    AA = np.zeros((6, 3))
    BB = np.zeros((6, 3))
    a0 = _sup_cyl(c, R, rc, hc, d0)
    b0 = _sup_box(cb, Rb, hb, -d0)
    P[0] = a0 - b0
    AA[0] = a0
    BB[0] = b0
    k = 1
    newest = -1
    for _ in range(GJK_MAX_IT):
        others = np.empty(k, np.int64)
        no = 0
        for i in range(k):
            if i != newest:
                others[no] = i
                no += 1
        best_found = False
        best_n2 = 0.0
        best_v = np.zeros(3)
        best_idx = np.zeros(4, np.int64)
        best_lam = np.zeros(4)
        best_m = 0
        for mask in range(1 << no):
            idx = np.zeros(4, np.int64)
            m = 0
            if newest >= 0:
                idx[0] = newest
                m = 1
            for j in range(no):
                if (mask >> j) & 1:
                    idx[m] = others[j]
                    m += 1
            if m == 0:
                continue
            lam = np.zeros(4)
            v = np.zeros(3)
            if not _hull_point(P, idx, m, lam, v):
                continue
            n2 = _dot(v, v)
            if (not best_found) or n2 < best_n2 - 1e-30:
                best_found = True
                best_n2 = n2
                best_v = v
                best_idx = idx
                best_lam = lam
                best_m = m
        if not best_found:
            return False, 0.0, np.zeros(3), np.zeros(3)
        newP = np.zeros((6, 3))
        newA = np.zeros((6, 3))
        newB = np.zeros((6, 3))
        for t in range(best_m):
            newP[t] = P[best_idx[t]]
            newA[t] = AA[best_idx[t]]
            newB[t] = BB[best_idx[t]]
        P = newP
        AA = newA
        BB = newB
        k = best_m
        nv = np.sqrt(best_n2)
        if nv < 1e-12:
            return False, 0.0, np.zeros(3), np.zeros(3)
        pa = np.zeros(3)
        pb = np.zeros(3)
        for t in range(best_m):
            pa += best_lam[t] * AA[t]
            pb += best_lam[t] * BB[t]
        a = _sup_cyl(c, R, rc, hc, -best_v)
        b = _sup_box(cb, Rb, hb, best_v)
        w = a - b
        if best_n2 - _dot(best_v, w) <= GJK_TOL_M * nv:
            return True, nv, pa, pb
        P[k] = w
        AA[k] = a
        BB[k] = b
        k += 1
        newest = k - 1
    raise RuntimeError("GJK did not converge")


@njit(cache=True)
def _h_diff(c, R, rc, hc, cb, Rb, hb, u):
    """The support of the difference along u: (sup_cylinder(u) - sup_box(-u)) . u (geometry.cylinder_box's h_diff)."""
    return _dot(_sup_cyl(c, R, rc, hc, u) - _sup_box(cb, Rb, hb, -u), u)


@njit(cache=True)
def cyl_box_contact(c, R, rc, hc, cb, Rb, hb, dirs, ls):
    """(gap, point on the cylinder, point on the box, push) for a solid cylinder and a solid box (geometry.cylinder_box). Separated: the
    GJK distance. Touching or overlapping: minus the least support of the difference over the box faces, the cylinder axis and the fixed
    directions `dirs`, then a local search with the draws `ls` (approximate, as in geometry.py)."""
    found, dist, pa, pb = _gjk_contact(c, R, rc, hc, cb, Rb, hb)
    if found:
        v = pb - pa
        n = _norm(v)
        if n > 1e-15:
            push = v / n
        else:
            push = Rb[:, 2].copy()
        return dist, pa, pb, push
    nd = dirs.shape[0]
    cands = np.empty((8 + nd, 3))
    for k in range(3):
        cands[2 * k] = Rb[:, k]
        cands[2 * k + 1] = -Rb[:, k]
    cands[6] = R[:, 2]
    cands[7] = -R[:, 2]
    vals = np.empty(8 + nd)
    for i in range(8):
        vals[i] = _h_diff(c, R, rc, hc, cb, Rb, hb, cands[i])
    j = 0
    for i in range(8):
        if vals[i] < vals[j]:
            j = i
    if vals[j] <= 1e-10:
        a = _sup_cyl(c, R, rc, hc, cands[j])
        b = _sup_box(cb, Rb, hb, -cands[j])
        return -max(vals[j], 0.0), a, b, cands[j].copy()
    for i in range(nd):
        cands[8 + i] = dirs[i]
        vals[8 + i] = _h_diff(c, R, rc, hc, cb, Rb, hb, cands[8 + i])
    j = 0
    for i in range(8 + nd):
        if vals[i] < vals[j]:
            j = i
    u = cands[j].copy()
    best = vals[j]
    step = 0.2
    for outer in range(LS_OUTER):
        improved = False
        for inner in range(LS_INNER):
            tr = u + step * ls[outer * LS_INNER + inner]
            tr = tr / _norm(tr)
            ht = _h_diff(c, R, rc, hc, cb, Rb, hb, tr)
            if ht < best:
                u = tr
                best = ht
                improved = True
        if not improved:
            step *= 0.6
    a = _sup_cyl(c, R, rc, hc, u)
    b = _sup_box(cb, Rb, hb, -u)
    return -best, a, b, u
