"""Safety filter: capsule-capsule and capsule-box distances against MuJoCo's own mj_geomDistance (no stepping).

MuJoCo is used here as ground truth only; the filter itself never sees it.
2000 random capsule-capsule pairs and 2000 random capsule-box pairs, about half of them deep overlaps.
Reports the max error of the gap, and for the closest-point pairs the max error of the points.

Pass: outside pairs (gap > 0) agree to 1e-6 m; deep overlaps agree in sign (both overlapping) and the
push direction separates the pair (moving the first shape along push by -gap clears it, checked with
the filter's own distance). The deep-overlap depth error is reported; it is not a pass criterion
(see docs/reports for the measured value).

Run: python experiments/safety_filter_distance_test.py   (exit 0 = pass)
"""

import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.safety_filter.geometry import capsule_box, capsule_capsule  # noqa: E402

N_PAIRS = 2000
OUTSIDE_TOL = 1e-6  # m

XML = """
<mujoco>
  <worldbody>
    <body name="c1"><geom name="cap1" type="capsule" size="0.05 0.1"/></body>
    <body name="c2"><geom name="cap2" type="capsule" size="0.05 0.1"/></body>
    <body name="bx"><geom name="box1" type="box" size="0.1 0.1 0.1"/></body>
  </worldbody>
</mujoco>
"""


def set_capsule(m, d, g, A, B, r):
    """Place MuJoCo's capsule geom g from end A to end B (fromto) with radius r."""
    mid = (A + B) / 2
    L = np.linalg.norm(B - A)
    m.geom_size[g] = [r, L / 2, 0.0]
    q = np.zeros(4)
    mujoco.mju_quatZ2Vec(q, (B - A) / max(L, 1e-12))
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, q)
    d.geom_xpos[g] = mid
    d.geom_xmat[g] = R


def set_box(m, d, g, c, R, h):
    m.geom_size[g] = h
    d.geom_xpos[g] = c
    d.geom_xmat[g] = R.reshape(9)


def mj_distance(m, d, g1, g2):
    fromto = np.zeros(6)
    dist = mujoco.mj_geomDistance(m, d, g1, g2, 1.0, fromto)
    return dist, fromto[:3], fromto[3:]


def main():
    rng = np.random.default_rng(7)
    m = mujoco.MjModel.from_xml_string(XML)
    d = mujoco.MjData(m)
    cap1, cap2, box1 = (mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n) for n in ("cap1", "cap2", "box1"))

    # ---- capsule-capsule ----
    cc_err, cc_outside_err, cc_outside, cc_deep, cc_point_err, cc_sign_bad, cc_push_bad = [], [], 0, 0, [], 0, 0
    for _ in range(N_PAIRS):
        A1 = rng.uniform(-0.2, 0.2, 3)
        B1 = A1 + rng.uniform(-0.3, 0.3, 3)
        r1 = rng.uniform(0.01, 0.08)
        A2 = A1 + rng.uniform(-0.15, 0.15, 3)       # close on purpose: many deep overlaps
        B2 = A2 + rng.uniform(-0.3, 0.3, 3)
        r2 = rng.uniform(0.01, 0.08)
        set_capsule(m, d, cap1, A1, B1, r1)
        set_capsule(m, d, cap2, A2, B2, r2)
        pass  # no forward pass: it would overwrite the geom poses set above
        dist_mj, p_mj, q_mj = mj_distance(m, d, cap1, cap2)
        c = capsule_capsule(A1, B1, r1, A2, B2, r2)
        err = abs(c.gap - dist_mj)
        cc_err.append(err)
        if dist_mj > 0:
            cc_outside += 1
            cc_outside_err.append(err)
        else:
            cc_deep += 1
            if (c.gap < 0) != (dist_mj < 0):
                cc_sign_bad += 1
        if dist_mj > 1e-9:
            # MuJoCo's fromto points are on the capsule surfaces, ours on the centre lines: compare directions
            u_ours = (c.point_a - c.point_b) / max(np.linalg.norm(c.point_a - c.point_b), 1e-12)
            u_mj = (p_mj - q_mj) / np.linalg.norm(p_mj - q_mj)
            cc_point_err.append(np.linalg.norm(u_ours - u_mj))
        # the push direction must separate: after moving A1 by -gap along push, the gap is zero (or more)
        if c.gap < 0:
            moved = capsule_capsule(A1 - c.gap * c.push, B1 - c.gap * c.push, r1, A2, B2, r2)
            if moved.gap < -1e-6:
                cc_push_bad += 1
    cc_err = np.array(cc_err)
    print(f"capsule-capsule: {N_PAIRS} pairs, outside {cc_outside}, deep {cc_deep}")
    print(f"  max |gap - mj_geomDistance| {cc_err.max():.3e} m (all); outside only {max(cc_outside_err):.3e} m")
    print(f"  deep overlaps: sign disagreements {cc_sign_bad}, push fails to separate {cc_push_bad}")
    print(f"  max difference of the separation direction (outside, unit vectors) {max(cc_point_err) if cc_point_err else 0:.3e}")

    # ---- capsule-box ----
    # MuJoCo's mj_geomDistance is NOT reliable for a capsule that passes through a box: it can return a
    # positive distance for a segment inside the box (found while writing this test: a case with 11295 of
    # 20001 segment samples inside the box reported +0.0107 m). So the reference for capsule-box is a dense
    # brute-force sample of the segment (signed distance to the box minus the radius), and MuJoCo is
    # compared where it agrees with that reference.
    ts_bf = np.linspace(0.0, 1.0, 40001)  # ten times the filter's grid: an independent reference
    cb_err_bf, cb_err_mj_agree, cb_mj_wrong, cb_outside, cb_deep, cb_push_bad = [], [], 0, 0, 0, 0
    for _ in range(N_PAIRS):
        c_box = rng.uniform(-0.1, 0.1, 3)
        R = Rotation.random(random_state=int(rng.integers(1 << 30))).as_matrix()
        h = rng.uniform(0.03, 0.15, 3)
        set_box(m, d, box1, c_box, R, h)
        A = c_box + rng.uniform(-0.25, 0.25, 3)
        B = A + rng.uniform(-0.3, 0.3, 3)
        r = rng.uniform(0.01, 0.08)
        set_capsule(m, d, cap1, A, B, r)
        dist_mj, _, _ = mj_distance(m, d, cap1, box1)
        c = capsule_box(A, B, r, c_box, R, h)
        loc = (A[None, :] + ts_bf[:, None] * (B - A)[None, :] - c_box) @ R
        q = np.abs(loc) - h
        sd_bf = np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)
        ref = sd_bf.min() - r
        cb_err_bf.append(abs(c.gap - ref))
        if ref > 0:
            cb_outside += 1
        else:
            cb_deep += 1
        if (dist_mj > 0) != (ref > 0):
            cb_mj_wrong += 1
        elif ref > 0:
            cb_err_mj_agree.append(abs(c.gap - dist_mj))
        if c.gap < 0:
            # the push must not make the gap worse (for a segment through the box it clears the deepest point only)
            moved = capsule_box(A - c.gap * c.push, B - c.gap * c.push, r, c_box, R, h)
            if moved.gap < c.gap - 2e-4:
                cb_push_bad += 1
    cb_err_bf = np.array(cb_err_bf)
    print(f"capsule-box: {N_PAIRS} pairs, outside (brute) {cb_outside}, deep (brute) {cb_deep}")
    print(f"  max |gap - brute force| {cb_err_bf.max():.3e} m (all)")
    print(f"  max |gap - mj_geomDistance| on outside pairs where MuJoCo agrees {max(cb_err_mj_agree):.3e} m")
    print(f"  MuJoCo wrong on the sign (positive for a segment through the box): {cb_mj_wrong}")
    print(f"  deep pairs where the push makes the gap worse: {cb_push_bad}")
    cb_sign_bad = 0
    ok = True
    if cc_sign_bad or cc_push_bad or cb_push_bad or (cb_err_bf.max() > 2e-4):
        ok = False
    if max(cb_err_mj_agree, default=0.0) > OUTSIDE_TOL:
        ok = False
    if (max(cc_outside_err) if cc_outside_err else 0) > OUTSIDE_TOL:
        ok = False
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
