"""Stage 2d step 1: what the solver is told about our arm (measurement only; nothing in src/ is changed).

Compares Kong's solver model with our MuJoCo model, both arms:
  * the solver's own spec (openarm_teleop/assets/specs/openarm_v1.npz: joint axes h, offsets p, joint limits) against our model's
    joint limits, and the link lengths (shoulder-elbow, elbow-wrist, wrist-hand reference point) at the zero pose;
  * the solver's forward kinematics (session.robot_fk_at: S, E, W, R_0_7) against our forward kinematics (ArmKinematics.sew, link 7
    pose), at the zero pose, with each joint moved alone (axis direction, sign, zero position), and at random poses;
  * the hand frame offset: C_HAND (our link 7 frame -> the solver's hand frame) and the residual rotation between R_0_7 and our link 7.
The two models use different world frames, so one rigid transform (Kabsch fit over all points) is fitted and the RESIDUALS are the
differences. Every difference above tolerance is flagged with its size. What cannot be read from outside is listed at the end.

Usage: python scripts/safety_filter_solver_model_check.py [--out data/safety_filter/solver_model_check.json]
"""

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import safety_filter_kong_comparison as kc  # noqa: E402

fx = kc.fx
SIDES = ("left", "right")
TOL_POS_MM = 0.1          # mm: a position difference above this is flagged
TOL_ROT_DEG = 0.01        # degrees
TOL_LIM_RAD = 1e-6        # rad
SPEC = Path("/home/vboxuser/projects/openarm_teleop/openarm_teleop/assets/specs/openarm_v1.npz")


def kabsch(P, Q):
    """Rigid transform (R, t) with Q ~ R P + t (rows are points)."""
    pc, qc = P.mean(0), Q.mean(0)
    H = (P - pc).T @ (Q - qc)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    return R, qc - R @ pc


def rot_angle_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="data/safety_filter/solver_model_check.json")
    ap.add_argument("--n-random", type=int, default=200)
    args = ap.parse_args(argv)

    model, shapes = fx.model_and_shapes()
    kin = kc.ArmKinematics(shapes)
    solver = fx.solver()
    spec = np.load(SPEC, allow_pickle=True)
    rng = np.random.default_rng(0)
    flags, rows = [], []

    def flag(what, size, unit):
        flags.append({"what": what, "size": float(size), "unit": unit})

    # ---- samples: zero pose, one joint at a time (+-0.3 rad), random poses (home of the other arm = zero here)
    poses = [np.zeros((2, 7))]
    for i in range(7):
        for dq in (0.3, -0.3):
            q = np.zeros((2, 7))
            q[:, i] = dq
            poses.append(q)
    for _ in range(args.n_random):
        q = np.zeros((2, 7))
        for si, s in enumerate(SIDES):
            lim = kin.limits[s]
            q[si] = rng.uniform(lim[:, 0], lim[:, 1])
        poses.append(q)
    ours, theirs = {s: [] for s in SIDES}, {s: [] for s in SIDES}
    for q in poses:
        st = kc.state_of(q)
        fk = solver.session.robot_fk_at(q_right=q[1].copy(), q_left=q[0].copy())
        for si, s in enumerate(SIDES):
            S, E, W = kin.sew(st, s)
            H, Rh = kin.hand_pose_now(s)
            ours[s].append((S, E, W, H, Rh))
            theirs[s].append((np.asarray(fk[s]["S"], float), np.asarray(fk[s]["E"], float), np.asarray(fk[s]["W"], float),
                              np.asarray(fk[s]["R_0_7"], float)))

    out = {"tolerances": {"pos_mm": TOL_POS_MM, "rot_deg": TOL_ROT_DEG, "limit_rad": TOL_LIM_RAD}, "arms": {}}
    for si, s in enumerate(SIDES):
        P = np.array([p for o in ours[s] for p in o[:3]])
        Q = np.array([p for t in theirs[s] for p in t[:3]])
        R, t = kabsch(P, Q)
        arm = {"frame_fit": {"rotation_angle_deg": rot_angle_deg(R), "translation_m": t.tolist()}}
        # position residuals per point type
        for k, name in enumerate(("S", "E", "W")):
            res = np.array([np.linalg.norm(R @ o[k] + t - th[k]) for o, th in zip(ours[s], theirs[s])]) * 1000
            arm[f"{name}_residual_mm"] = {"max": float(res.max()), "mean": float(res.mean()), "zero_pose": float(res[0])}
            if res.max() > TOL_POS_MM:
                flag(f"{s}: forward kinematics point {name} differs (max over {len(res)} poses)", res.max(), "mm")
        # hand frame: relative rotation between the solver's R_0_7 (in its frame) and ours (fitted frame)
        rel = [th[3].T @ (R @ o[4]) for o, th in zip(ours[s], theirs[s])]
        mean_rel = rel[0]
        spread = max(rot_angle_deg(r.T @ mean_rel) for r in rel)
        arm["hand_frame_offset"] = {"R_solver_to_ours_at_zero": mean_rel.tolist(), "angle_deg": rot_angle_deg(mean_rel),
                                    "spread_over_poses_deg": spread}
        if rot_angle_deg(mean_rel) > TOL_ROT_DEG:
            flag(f"{s}: link 7 orientation of the solver (R_0_7) differs from ours by a constant rotation", rot_angle_deg(mean_rel), "deg")
        if spread > TOL_ROT_DEG:
            flag(f"{s}: link 7 orientation differs and the difference changes with the pose (axis/sign/zero mismatch)", spread, "deg")
        # joint by joint (axis direction, sign, zero position): poses 1..14 are single-joint moves of +-0.3 rad
        per_joint = []
        for i in range(7):
            e = []
            for k in (1 + 2 * i, 2 + 2 * i):
                o, th = ours[s][k], theirs[s][k]
                dw = np.linalg.norm(R @ o[2] + t - th[2]) * 1000
                de = np.linalg.norm(R @ o[1] + t - th[1]) * 1000
                dr = rot_angle_deg(th[3].T @ (R @ o[4]) @ mean_rel.T)
                e.append((dw, de, dr))
            per_joint.append({"joint": i + 1, "W_mm_max": max(x[0] for x in e), "E_mm_max": max(x[1] for x in e),
                              "link7_rot_deg_max": max(x[2] for x in e)})
            if max(x[0] for x in e) > TOL_POS_MM or max(x[2] for x in e) > TOL_ROT_DEG:
                flag(f"{s}: moving joint {i + 1} alone gives different wrist position/orientation", max(max(x[0] for x in e), max(x[2] for x in e)),
                     "mm or deg")
        arm["single_joint_moves"] = per_joint
        # link lengths at the zero pose (solver FK vs ours), and wrist to hand reference point (ours: link 7 body origin - joint 7 anchor)
        S0o, E0o, W0o, H0o, _ = ours[s][0]
        S0t, E0t, W0t, _ = theirs[s][0]
        L = {"shoulder_elbow_m": (float(np.linalg.norm(E0o - S0o)), float(np.linalg.norm(E0t - S0t))),
             "elbow_wrist_m": (float(np.linalg.norm(W0o - E0o)), float(np.linalg.norm(W0t - E0t)))}
        L["wrist_to_hand_reference_point_m"] = (float(np.linalg.norm(H0o - W0o)), None)
        arm["link_lengths_zero_pose (ours, solver)"] = L
        for k in ("shoulder_elbow_m", "elbow_wrist_m"):
            d = abs(L[k][0] - L[k][1]) * 1000
            if d > TOL_POS_MM:
                flag(f"{s}: {k} differs", d, "mm")
        # lengths over the random poses (ours must stay constant; the solver's too)
        for name, a, b in (("shoulder_elbow", 0, 1), ("elbow_wrist", 1, 2)):
            lo = np.array([np.linalg.norm(o[b] - o[a]) for o in ours[s]])
            lt = np.array([np.linalg.norm(th[b] - th[a]) for th in theirs[s]])
            arm[f"{name}_length_range_mm (ours min,max; solver min,max)"] = [float(lo.min() * 1000), float(lo.max() * 1000),
                                                                           float(lt.min() * 1000), float(lt.max() * 1000)]
        # joint limits: solver spec vs our model
        lo_s, up_s = spec[f"{s}_arm/joint_lower"], spec[f"{s}_arm/joint_upper"]
        lim = kin.limits[s]
        dl = np.abs(lim[:, 0] - lo_s)
        du = np.abs(lim[:, 1] - up_s)
        arm["joint_limits"] = {"ours_lower": lim[:, 0].tolist(), "solver_lower": lo_s.tolist(), "ours_upper": lim[:, 1].tolist(),
                               "solver_upper": up_s.tolist(), "max_difference_rad": float(max(dl.max(), du.max()))}
        for i in range(7):
            for nm, d in (("lower", dl[i]), ("upper", du[i])):
                if d > TOL_LIM_RAD:
                    flag(f"{s}: joint {i + 1} {nm} limit differs (ours vs solver spec)", d, "rad")
        # axes: solver spec h (in its own link frames) vs our model's axes at the zero pose in the fitted frame: lengths of the spec offsets
        arm["spec_offset_norms_m"] = [float(np.linalg.norm(v)) for v in spec[f"{s}_arm/p"]]
        arm["spec_axes_h"] = spec[f"{s}_arm/h"].tolist()
        # our axes at the zero pose, mapped into the solver frame (to be compared by eye with h through R_base_0)
        kin.set_state(kc.state_of(np.zeros((2, 7))))
        arm["our_axes_zero_pose_in_solver_frame"] = [(R @ kin.data.xaxis[kin.jids[s][i]]).tolist() for i in range(7)]
        out["arms"][s] = arm

    # C_HAND sanity
    from src.geo_kin_self_collision_filter import C_HAND
    out["C_HAND"] = {"orthonormal_error": float(np.abs(C_HAND @ C_HAND.T - np.eye(3)).max()), "det": float(np.linalg.det(C_HAND))}
    out["flags"] = flags
    out["not_checkable_from_outside"] = [
        "the solver is a compiled wheel: its internal IK model, damping and iteration limits cannot be read; only its forward kinematics "
        "(robot_fk_at), its spec file and its outputs can be compared",
        "joint limits inside the solver: the spec file holds them but the solver's own option 'limited' is off (module docstring of "
        "src/geo_kin_self_collision_filter.py), so whether they are applied can only be seen from the outputs (step 2 and 3 record limit hits)",
    ]

    tmp = str(args.out) + ".tmp"
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    with open(tmp, "w") as f:
        json.dump(out, f, indent=1)
    os.replace(tmp, args.out)

    # ---- printed tables
    print("## Solver model vs our model")
    for s in SIDES:
        a = out["arms"][s]
        print(f"\n### {s} arm  (frame fit rotation {a['frame_fit']['rotation_angle_deg']:.3f} deg)")
        print("| item | ours | solver | difference |")
        print("|---|---|---|---|")
        L = a["link_lengths_zero_pose (ours, solver)"]
        for k in ("shoulder_elbow_m", "elbow_wrist_m"):
            print(f"| {k} (zero pose) | {L[k][0]:.6f} | {L[k][1]:.6f} | {abs(L[k][0] - L[k][1]) * 1000:.4f} mm |")
        print(f"| wrist_to_hand_reference_point_m | {L['wrist_to_hand_reference_point_m'][0]:.6f} | (not in the solver: it reads directions) | n/a |")
        for n in ("S", "E", "W"):
            r = a[f"{n}_residual_mm"]
            print(f"| FK point {n}, over {len(poses)} poses | | | max {r['max']:.4f} mm, zero pose {r['zero_pose']:.4f} mm |")
        h = a["hand_frame_offset"]
        print(f"| link 7 orientation offset | | | constant {h['angle_deg']:.4f} deg, spread {h['spread_over_poses_deg']:.4f} deg |")
        for pj in a["single_joint_moves"]:
            print(f"| joint {pj['joint']} moved alone +-0.3 rad | | | wrist {pj['W_mm_max']:.4f} mm, elbow {pj['E_mm_max']:.4f} mm, link7 {pj['link7_rot_deg_max']:.4f} deg |")
        print(f"| joint limits | see json | | max difference {a['joint_limits']['max_difference_rad']:.2e} rad |")
    print(f"\nC_HAND orthonormal error {out['C_HAND']['orthonormal_error']:.1e}, det {out['C_HAND']['det']:.3f}")
    print("\nFLAGS (difference above tolerance):")
    for f in flags:
        print(f"  - {f['what']}: {f['size']:.6g} {f['unit']}")
    if not flags:
        print("  none")
    print("\nNOT CHECKABLE FROM OUTSIDE:")
    for n in out["not_checkable_from_outside"]:
        print("  -", n)
    return 0


if __name__ == "__main__":
    sys.exit(main())
