"""Forearm capsules on the elbow-wrist line: 1, 2 and 3 capsules, split points chosen to minimise total volume (kinematics only).

Forearm = links 4-6 collision geoms; the line is the joint 4 anchor (elbow) to the joint 7 anchor (wrist), per pose.
Sampling as the build tool: joints 4, 5 and 6 with their limit corners (8) and random poses within the limits
(2000 in the fit set). The other joints are at zero; the forearm's geometry relative to its line does not depend on
joints 1-3 or 7 (the line's ends are on the joint axes of their own bodies).

For a split s (fractions of the line, 0 = elbow, 1 = wrist) each piece covers the vertices whose position along the
line lies in its range; a piece's radius is the largest distance from its vertices to its own segment, over all
poses. Volume = sum over pieces of pi r^2 (L + 4/3 r). The best split for each count minimises that volume.

Check: a second, independent set of 500 random poses (seed 99) gives the stick-out of every piece (must be <= 0).

Writes nothing to data/. Prints the result as JSON (last line).
"""

import itertools
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402

N_FIT = 2000
N_CHECK = 500
STEP = {1: None, 2: 0.005, 3: 0.02}   # split grid step per count (fraction of the line)


def sample_joint_sets(m, joints, rng, n):
    lim = np.array([m.jnt_range[j] for j in joints], float)
    corners = np.array(list(itertools.product([0, 1], repeat=len(joints))))
    corners = lim[np.arange(len(joints)), corners]  # shape (8, k)? fixed below
    corners = np.array([[lim[i, b] for i, b in enumerate(bits)] for bits in itertools.product([0, 1], repeat=len(joints))])
    rnd = rng.uniform(lim[:, 0], lim[:, 1], size=(n, len(joints)))
    return np.vstack([corners, rnd])


def local_hull(m, g):
    """The convex hull vertices of a mesh geom in its own frame (MuJoCo collides with the hull; the hull is what matters)."""
    from scipy.spatial import ConvexHull
    did = m.geom_dataid[g]
    V = m.mesh_vert[m.mesh_vertadr[did]:m.mesh_vertadr[did] + m.mesh_vertnum[did]].astype(float)
    return V[ConvexHull(V).vertices]


def geometry_per_pose(m, d, joints_idx, samples, geoms, j4, j7):
    """For each pose: the line (A, B) and the forearm hull vertices (world)."""
    hulls = [local_hull(m, g) for g in geoms]
    out = []
    for q in samples:
        d.qpos[:] = m.qpos0
        d.qpos[[m.jnt_qposadr[j] for j in joints_idx]] = q
        mujoco.mj_kinematics(m, d)
        A, B = d.xanchor[j4].copy(), d.xanchor[j7].copy()
        Vs = [H @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g] for H, g in zip(hulls, geoms)]
        out.append((A, B, np.vstack(Vs)))
    return out


def per_vertex(pose_list):
    """Per pose: unit axis, length, t (position along the line, unclipped) and the vertices."""
    rows = []
    for A, B, V in pose_list:
        AB = B - A
        L = float(np.linalg.norm(AB))
        u = AB / L
        rel = V - A
        t = rel @ u / L
        perp = np.linalg.norm(rel - np.outer(rel @ u, u), axis=1)
        rows.append((A, u, L, t, perp, rel))
    return rows


def piece_radius(rows, t0, t1, is_first, is_last):
    """Largest distance to the piece segment [t0, t1] (fractions) over the vertices assigned to it, all poses."""
    r = 0.0
    for A, u, L, t, perp, rel in rows:
        lo = -np.inf if is_first else t0
        hi = np.inf if is_last else t1
        sel = (t >= lo) & (t < hi) if not is_last else (t >= lo)
        if not sel.any():
            continue
        tc = np.clip(t[sel], t0, t1)
        dist = np.sqrt(perp[sel] ** 2 + ((t[sel] - tc) * L) ** 2)
        r = max(r, float(dist.max()))
    return r


def volume(radii, lengths):
    return sum(np.pi * r ** 2 * (Lk + 4.0 / 3.0 * r) for r, Lk in zip(radii, lengths))


def best_split(rows, count, Lmean):
    if count == 1:
        r = piece_radius(rows, 0.0, 1.0, True, True)
        return {"splits": [], "radii": [r], "volume": volume([r], [Lmean]), "ranges": [[0.0, 1.0]]}
    grid = np.round(np.arange(STEP[count], 1.0, STEP[count]), 6)
    best = None
    combos = [(s,) for s in grid] if count == 2 else [(a, b) for a, b in itertools.combinations(grid, 2)]
    cache = {}
    for splits in combos:
        bounds = [0.0] + list(splits) + [1.0]
        radii = []
        for k in range(count):
            key = (bounds[k], bounds[k + 1], k == 0, k == count - 1)
            if key not in cache:
                cache[key] = piece_radius(rows, bounds[k], bounds[k + 1], k == 0, k == count - 1)
            radii.append(cache[key])
        lengths = [(bounds[k + 1] - bounds[k]) * Lmean for k in range(count)]
        v = volume(radii, lengths)
        if best is None or v < best["volume"]:
            best = {"splits": list(map(float, splits)), "radii": list(map(float, radii)), "volume": float(v),
                    "ranges": [[bounds[k], bounds[k + 1]] for k in range(count)]}
    return best


def stick_out(rows_check, fit):
    """Largest stick-out over the independent check poses: for each vertex the best (min over pieces) of dist - r, max over vertices."""
    worst = -np.inf
    for A, u, L, t, perp, rel in rows_check:
        best_vertex = np.full(len(t), np.inf)
        for (t0, t1), r in zip(fit["ranges"], fit["radii"]):
            tc = np.clip(t, t0, t1)
            dist = np.sqrt(perp ** 2 + ((t - tc) * L) ** 2)
            best_vertex = np.minimum(best_vertex, dist - r)
        worst = max(worst, float(best_vertex.max()))
    return worst


def main():
    m = load_robot_only_model()
    d = mujoco.MjData(m)
    j4 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "openarm_left_joint4")
    j5 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "openarm_left_joint5")
    j6 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "openarm_left_joint6")
    j7 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "openarm_left_joint7")
    geoms = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"openarm_left_link{k}_collision") for k in (4, 5, 6)]
    joints = [j4, j5, j6]
    rng = np.random.default_rng(0)
    fit_samples = sample_joint_sets(m, joints, rng, N_FIT - 8)
    rng2 = np.random.default_rng(99)
    check_samples = sample_joint_sets(m, joints, rng2, N_CHECK)[8:]
    check_samples = np.vstack([check_samples])
    fit_geo = geometry_per_pose(m, d, joints, fit_samples, geoms, j4, j7)
    check_geo = geometry_per_pose(m, d, joints, check_samples, geoms, j4, j7)
    rows = per_vertex(fit_geo)
    rows_check = per_vertex(check_geo)
    Lmean = float(np.mean([r[2] for r in rows]))
    print(f"fit poses {len(rows)} (8 corners + {N_FIT - 8} random), check poses {len(rows_check)}; mean line length {Lmean * 100:.2f} cm")
    out = {}
    for count in (1, 2, 3):
        fit = best_split(rows, count, Lmean)
        so = stick_out(rows_check, fit)
        out[count] = {**fit, "check_stick_out_m": so}
        print(f"{count} capsule(s): splits {[round(s, 3) for s in fit['splits']]}, ranges {[[round(a, 3), round(b, 3)] for a, b in fit['ranges']]}, "
              f"radii cm {[round(r * 100, 3) for r in fit['radii']]}, volume {fit['volume'] * 1e6:.1f} cm^3, "
              f"independent-check stick-out {so * 1e3:+.3f} mm")
    print("JSON " + json.dumps({"Lmean": Lmean, "counts": out}, default=float))
    return 0


if __name__ == "__main__":
    sys.exit(main())
