"""Compiled distance functions (Braking Phase F fix 4, SF-81): every compiled contact (gap, both closest points, push direction)
against the Python reference (geometry._py_*), to 1e-6 m, and the public functions with the switch on and off. Kinematics only.

Run: python experiments/safety_filter_jit_test.py   (exit 0 = all pass).
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter import geometry as G  # noqa: E402
from src.safety_filter import geometry_jit as J  # noqa: E402

fails = []
TOL = 1e-6          # m: the tolerance the brief asks for (compiled against Python)


def check(name, ok, info=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  [{info}]" if info else ""))
    if not ok:
        fails.append(name)


rng = np.random.default_rng(7)


def pt(scale):
    return rng.uniform(-scale, scale, 3)


def rot():
    q, r = np.linalg.qr(rng.normal(size=(3, 3)))
    q = q * np.sign(np.diag(r))
    if np.linalg.det(q) < 0:
        q[:, 0] = -q[:, 0]
    return q


def diff(ref, got):
    """Largest difference of the four contact fields (gap, point a, point b, push)."""
    return max(abs(ref.gap - got[0]), float(np.max(np.abs(ref.point_a - got[1]))),
               float(np.max(np.abs(ref.point_b - got[2]))), float(np.max(np.abs(ref.push - got[3]))))


# ---- the constants and the overlap search's draws equal geometry's ----
check("constants equal geometry.py (GJK tolerance, iteration limit, cylinder grids, overlap directions)",
      (J.GJK_TOL_M, J.GJK_MAX_IT, J.CYL_GRID, J.CYL_REFINE, J.CYL_REFINE_ROUNDS, J.OVERLAP_DIRS)
      == (G.GJK_TOL_M, G.GJK_MAX_IT, G.CYL_GRID, G.CYL_REFINE, G.CYL_REFINE_ROUNDS, G.OVERLAP_DIRS))
_g = np.random.default_rng(0).normal(size=(G.OVERLAP_DIRS, 3))
check("the overlap search's directions are the ones geometry.cylinder_box draws (seed 0)",
      np.array_equal(J.DIRS, _g / np.linalg.norm(_g, axis=1)[:, None]))

# ---- capsule against capsule: random, zero-length, parallel and crossing segments ----
worst = 0.0
for k in range(2000):
    A1, B1, A2, B2 = pt(0.3), pt(0.3), pt(0.3), pt(0.3)
    r1, r2 = rng.uniform(0.005, 0.05, 2)
    if k % 10 == 0:
        B1 = A1.copy()                                  # zero-length first capsule
    if k % 10 == 1:
        B2 = A2 + 2.0 * (B1 - A1)                       # parallel segments
    if k % 10 == 2:
        A2 = A1 + 0.5 * (B1 - A1) + 0.01 * pt(1.0)      # crossing near the middle of the first
        B2 = A2 + pt(0.3)
    ref = G._py_capsule_capsule(A1, B1, r1, A2, B2, r2)
    worst = max(worst, diff(ref, J.cc_contact(A1, B1, r1, A2, B2, r2)))
check("capsule-capsule: 2000 random, zero-length, parallel and crossing pairs match the reference to 1e-6 m", worst <= TOL, f"worst {worst:.2e} m")

# ---- capsule against box ----
worst, n_over = 0.0, 0
for k in range(300):
    A, B = pt(0.3), pt(0.3)
    r = rng.uniform(0.005, 0.03)
    c, R, h = pt(0.2), rot(), rng.uniform(0.01, 0.12, 3)
    ref = G._py_capsule_box(A, B, r, c, R, h)
    n_over += ref.gap < 0
    worst = max(worst, diff(ref, J.cb_contact(A, B, r, c, R, h)))
check("capsule-box: 300 random pairs (separated and overlapping) match the reference to 1e-6 m", worst <= TOL,
      f"worst {worst:.2e} m, {n_over} overlapping")

# ---- solid cylinder against capsule ----
worst, n_over = 0.0, 0
for k in range(300):
    c, R = pt(0.2), rot()
    rc, hc = rng.uniform(0.01, 0.05), rng.uniform(0.01, 0.1)
    A, B = pt(0.3), pt(0.3)
    r = rng.uniform(0.005, 0.03)
    ref = G._py_cylinder_capsule(c, R, rc, hc, A, B, r)
    n_over += ref.gap < 0
    worst = max(worst, diff(ref, J.cyl_cap_contact(c, R, rc, hc, A, B, r)))
check("cylinder-capsule: 300 random pairs (separated and overlapping) match the reference to 1e-6 m", worst <= TOL,
      f"worst {worst:.2e} m, {n_over} overlapping")

# ---- solid cylinder against solid box (GJK; overlap search) ----
# The cylinder-box gap matches to 1e-6 m. The closest PAIR of two flat-faced solids need not be unique (a face against a face, or an
# edge along a face), so the two implementations may pick different witness points at the same distance. The compiled witness is checked
# for validity instead: |pb - pa| = gap, pa on the cylinder surface, pb on the box surface (separated pairs). The witness difference is reported.
worst_gap, worst_valid, worst_wit, n_over, n_sep = 0.0, 0.0, 0.0, 0, 0
for k in range(400):
    c, R = pt(0.1), rot()
    rc, hc = rng.uniform(0.01, 0.05), rng.uniform(0.01, 0.1)
    cb, Rb, hb = pt(0.12), rot(), rng.uniform(0.01, 0.1, 3)
    ref = G._py_cylinder_box(c, R, rc, hc, cb, Rb, hb)
    gap, pa, pb, push = J.cyl_box_contact(c, R, rc, hc, cb, Rb, hb, J.DIRS, J.LS_NORMALS)
    n_over += ref.gap < 0
    n_sep += ref.gap > 0
    worst_gap = max(worst_gap, abs(ref.gap - gap))
    if ref.gap > 0:
        q = Rb.T @ (pb - cb)
        sd_box = np.linalg.norm(np.maximum(np.abs(q) - hb, 0.0)) + min(np.max(np.abs(q) - hb), 0.0)
        worst_valid = max(worst_valid, abs(np.linalg.norm(pb - pa) - gap), abs(G.cyl_signed_distance(pa, c, R, rc, hc)), abs(sd_box))
        worst_wit = max(worst_wit, float(np.max(np.abs(ref.point_a - pa))), float(np.max(np.abs(ref.point_b - pb))))
check("cylinder-box: 400 random pairs (separated, touching and overlapping): gap matches geometry.cylinder_box to 1e-6 m",
      worst_gap <= TOL, f"worst gap diff {worst_gap:.2e} m, {n_sep} separated, {n_over} overlapping")
check("cylinder-box: the compiled closest pair is valid for every separated pair (distance = gap, on both surfaces) to 1e-6 m",
      worst_valid <= TOL, f"worst {worst_valid:.2e} m")
print(f"       (info) cylinder-box witness points differ from the reference by up to {worst_wit:.2e} m (a non-unique closest pair)")

# ---- the public functions give the same answer with the switch on and off ----
saved = cfg.GEOMETRY["compiled"]
worst = 0.0
try:
    for k in range(60):
        A, B, A2, B2 = pt(0.3), pt(0.3), pt(0.3), pt(0.3)
        r, r2 = rng.uniform(0.005, 0.03, 2)
        c, R, h, Rb = pt(0.15), rot(), rng.uniform(0.01, 0.1, 3), rot()
        rc, hc = rng.uniform(0.01, 0.05), rng.uniform(0.01, 0.1)
        cases = [(G.capsule_capsule, (A, B, r, A2, B2, r2)), (G.capsule_box, (A, B, r, c, R, h)),
                 (G.cylinder_capsule, (c, R, rc, hc, A, B, r)), (G.cylinder_box, (c, R, rc, hc, c + pt(0.1), Rb, h))]
        for fn, args in cases:
            cfg.GEOMETRY["compiled"] = True
            on = fn(*args)
            cfg.GEOMETRY["compiled"] = False
            off = fn(*args)
            if fn is G.cylinder_box:          # the gap only (the closest pair need not be unique; see the cylinder-box checks above)
                worst = max(worst, abs(off.gap - on.gap))
            else:
                worst = max(worst, diff(off, (on.gap, on.point_a, on.point_b, on.push)))
finally:
    cfg.GEOMETRY["compiled"] = saved
check("public functions: compiled and Python switch give the same contact (240 calls, four kinds) to 1e-6 m", worst <= TOL,
      f"worst {worst:.2e} m")

print(f"{len(fails)} failing check(s)" if fails else "ALL PASS")
sys.exit(1 if fails else 0)
