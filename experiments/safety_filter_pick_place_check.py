"""Safety filter measurement: can the filter's shapes pick and place? (report only; no shape or config change)

Kinematics only: the scene is loaded and its kinematics set with mj_kinematics; nothing is stepped.
Scene: the task scene (TASK_CFG), default layout, the staging stance (STAGING_CFG staging_xy, staging_yaw,
pedestal_q). STANCE SELECTION IS NOT RUN here: the staging stance is used as the base pose.

Moments (left arm, the scripted pick-and-place plan of src/task_points.py / src/mink_arm.py):
  M1 approach     the standoff point of the pick (pad at the grasp height, 12 cm back along the approach), fingers open
  M2 grasp 45%   pad centre at 45 % of the beaker height, fingers open
  M3 grasp 60%   pad centre at 60 %, fingers open
  M4 lift start  pad at the grasp height (52.5 %), fingers closed on the beaker
  M5 carry       pad over the midpoint of beaker and hotplate, carry height, held
  M6 over        pad over the hotplate at carry height, held
  M7 place down  beaker resting on the hotplate (pad at the set-down height), held
  M8 release     the same pose, fingers open, the beaker stays on the hotplate
  M9 recovery    the place back-off point (retreat along the approach, pad at the set-down height), fingers open

The arm is solved with MinkArm.step (the scripted skill's IK), iterated to convergence on a fixed scene state.
The approach bin k is the one whose approach direction best matches the direction from the stance to the target.

For each moment and filter shape of the left arm: the gap of the SHAPE to the bench top, the hotplate and the
target beaker, and the gap of the REAL collision geometry (MuJoCo mj_geomDistance, scoring only), with a flag:
MARGIN if the shape is inside the arm-vs-world margin (1 cm), BAND if inside the push-start band (2 cm).

Run: python experiments/safety_filter_pick_place_check.py   (prints the tables; writes nothing)
"""

import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.task_config import BEAKER_CFG, PICK_PLACE_CFG, TASK_CFG  # noqa: E402
from src.task_beaker_hotplate import apply_task_poses, bench_top_z, load_task, plate_top_z  # noqa: E402
from src.mink_arm import MinkArm, pad_rotation  # noqa: E402
from src.task_points import approach_unit  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes, RobotState  # noqa: E402
from src.safety_filter.geometry import capsule_box, capsule_capsule  # noqa: E402
from src.safety_filter.world import LabMapSource, catalog_object  # noqa: E402
from src.safety_filter.touch import as_placed, gap_between  # noqa: E402
from src.safety_filter.shapes import PlacedShape  # noqa: E402

SHAPES = ROOT / "data" / "safety_filter" / "robot_shapes.json"
LAB_MAP = ROOT / "data" / "safety_filter" / "lab_map.json"
MARGIN, BAND = 0.01, 0.02
FINGER_OPEN, FINGER_HELD = 0.044, 0.037   # finger travel (m) open, and closed on the 70 mm beaker (7 mm less per finger; see the report)
ARM_PARTS = {
    "upper arm": ["openarm_left_link1_collision", "openarm_left_link2_collision", "openarm_left_link3_collision"],
    "forearm": ["openarm_left_link4_collision", "openarm_left_link5_collision", "openarm_left_link6_collision"],
    "link 7": ["openarm_left_link7_collision"],
    "palm": ["openarm_left_hand_collision"],
    "fingers": ["openarm_left_right_finger_collision", "openarm_left_left_finger_collision"],
}


def geom_id(m, name):
    g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
    if g < 0:
        raise SystemExit(f"scene has no geom {name}")
    return g


def real_gap(m, d, geoms_a, geoms_b):
    """Smallest MuJoCo distance between any geom of A and any geom of B (scoring only; not the filter's method)."""
    best = np.inf
    fromto = np.zeros(6)
    for a in geoms_a:
        for b in geoms_b:
            dist = mujoco.mj_geomDistance(m, d, a, b, 0.5, fromto)
            best = min(best, dist)
    return float(best)


def shape_gap(shape, target):
    """Gap from one placed shape to a target box or capsule (PlacedShape). Negative = overlap."""
    return gap_between(shape, target)[0]


def main():
    sim = load_task(TASK_CFG, cameras=False)
    m, d = sim.model, sim.data
    apply_task_poses(m, d, TASK_CFG)
    # staging stance (reference stance; the stance selection is not run here)
    base = m.joint("base_float").qposadr[0]
    yaw = TASK_CFG["staging_yaw"]
    sx, sy = TASK_CFG["staging_xy"]
    d.qpos[base:base + 7] = [sx, sy, 0.0, math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
    ped = m.joint("pedestal_joint").qposadr[0]
    d.qpos[ped] = TASK_CFG["pedestal_q"]
    home_r = np.array(TASK_CFG["home_q_right"])
    d.qpos[[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_right_joint{k}")] for k in range(1, 8)]] = home_r
    d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "openarm_left_finger_joint1")]] = FINGER_OPEN
    d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "openarm_left_finger_joint2")]] = FINGER_OPEN
    mujoco.mj_forward(m, d)

    top = bench_top_z(m)
    plate_top = plate_top_z(m, d)
    h = BEAKER_CFG["height"]
    bx, by = TASK_CFG["beaker_xy"]
    hx, hy = TASK_CFG["hotplate_xy"]
    bottom = top
    f_mid = PICK_PLACE_CFG["grasp_height_frac"]
    z_carry_ref = plate_top
    z_carry = max(z_carry_ref + PICK_PLACE_CFG["grasp_height_frac"] * h + PICK_PLACE_CFG["carry_gap_m"],
                  bottom + f_mid * h + 0.05)
    z_place = plate_top + PICK_PLACE_CFG["setdown_gap_m"] + f_mid * h

    heading = yaw

    def bins_by_closeness(target_xy):
        want = math.atan2(target_xy[1] - sy, target_xy[0] - sx)
        order = []
        for k in range(12):
            ux, uy = approach_unit(heading, k)
            diff = abs((math.atan2(uy, ux) - want + math.pi) % (2 * math.pi) - math.pi)
            order.append((diff, k, (ux, uy)))
        return [(k, u) for _, k, u in sorted(order)]

    # The approach bin is chosen by the solver: the first bin, nearest to the direction of the target, at which the
    # grasp (45 %) and the place (set-down) poses both converge (set in choose_bins below).
    print(f"staging stance (x, y, heading) = ({sx:.3f}, {sy:.3f}, {math.degrees(yaw):.0f} deg); pedestal {TASK_CFG['pedestal_q']:.4f} m")

    mk = MinkArm(m, d, "left", np.array(TASK_CFG["home_q_left"]))
    qadr = mk.qadr
    d.qpos[qadr] = np.array(TASK_CFG["home_q_left"])
    mujoco.mj_forward(m, d)
    mk.reset(d)
    p0, R0 = mk.pad_pose(d)
    closing = R0[:, 0]

    def solve(pos, approach_xy, tries=6000):
        """Returns (joints or None, status). Converged: position within 1 mm and the pad rotation within 0.05 (Frobenius).
        Not converged but within 5 mm: the joints are kept and the residual is reported with the row."""
        ax, ay = approach_xy
        R = pad_rotation(np.array([ax, ay, 0.0]), closing)
        mk.reset(d)
        q = None
        last = None
        for it in range(tries):
            try:
                q = mk.step(d, pos, R)
            except Exception as e:  # mink.NoSolutionFound
                return None, f"no solution: {e}"
            d.qpos[qadr] = q
            mujoco.mj_kinematics(m, d)
            p, Rc = mk.pad_pose(d)
            last = (np.linalg.norm(p - pos), np.linalg.norm(R - Rc))
            if last[0] < 1e-3 and last[1] < 0.05:
                return q.copy(), f"ok (residual {last[0] * 1e3:.2f} mm, rot {last[1]:.3f}, {it} steps)"
        if last[0] < 5e-3:
            return q.copy(), f"residual {last[0] * 1e3:.2f} mm, rot {last[1]:.3f} (kept)"
        return None, f"not converged: pos error {last[0] * 1e3:.2f} mm, rot {last[1]:.3f}"

    def pose_ok(pos, approach_xy):
        return solve(np.array(pos), approach_xy)[0] is not None

    def choose_bins():
        for k, (cx, cy) in bins_by_closeness((bx, by)):
            if pose_ok((bx, by, bottom + 0.45 * h), (cx, cy)):
                kp, pu = k, (cx, cy)
                break
        else:
            raise SystemExit("no pick approach bin converges at the grasp point")
        for k, (cx, cy) in bins_by_closeness((hx, hy)):
            if pose_ok((hx, hy, z_place), (cx, cy)):
                return kp, pu, k, (cx, cy)
        raise SystemExit("no place approach bin converges at the set-down point")

    k_pick, (ux, uy), k_place, (vx, vy) = choose_bins()
    print(f"bins chosen by convergence: pick {k_pick}, place {k_place}")
    stand_xy = (bx - ux * PICK_PLACE_CFG["standoff_m"], by - uy * PICK_PLACE_CFG["standoff_m"])
    back_xy = (hx - vx * PICK_PLACE_CFG["retreat_m"], hy - vy * PICK_PLACE_CFG["retreat_m"])
    mid_xy = ((bx + hx) / 2, (by + hy) / 2)
    z_grasp = bottom + f_mid * h
    moments = [
        ("M1 approach", (stand_xy[0], stand_xy[1], z_grasp), (ux, uy), False, "bench"),
        ("M2 grasp 45%", (bx, by, bottom + 0.45 * h), (ux, uy), False, "bench"),
        ("M3 grasp 60%", (bx, by, bottom + 0.60 * h), (ux, uy), False, "bench"),
        ("M4 lift start", (bx, by, z_grasp), (ux, uy), True, "bench"),
        ("M5 carry", (mid_xy[0], mid_xy[1], z_carry), (vx, vy), True, "carry"),
        ("M6 over plate", (hx, hy, z_carry), (vx, vy), True, "plate"),
        ("M7 place down", (hx, hy, z_place), (vx, vy), True, "plate"),
        ("M8 release", (hx, hy, z_place), (vx, vy), False, "plate"),
        ("M9 recovery", (back_xy[0], back_xy[1], z_place), (vx, vy), False, "plate"),
    ]

    # filter shapes and the world (lab map: bench top; the hotplate from the task scene)
    rs = RobotShapes(load_robot_only_model(), SHAPES)
    lab = LabMapSource(LAB_MAP)
    bench = next(as_placed(o) for o in lab.objects() if o.id == "bench_top")
    rot_id = np.eye(3)
    hp_base = PlacedShape(id="hotplate_base", arm="world", part="world", kind="box", body="world",
                          centre=np.array([hx, hy, top + 0.03]), rot=rot_id, half=np.array([0.09, 0.09, 0.03]))
    hp_plate = PlacedShape(id="hotplate_plate", arm="world", part="world", kind="box", body="world",
                           centre=np.array([hx, hy, top + 0.064]), rot=rot_id, half=np.array([0.075, 0.075, 0.004]))
    hot = [hp_base, hp_plate]
    real_bench = [geom_id(m, "bench_top")]
    real_hot = [geom_id(m, "task_hotplate_base"), geom_id(m, "task_hotplate_top_plate")]
    beaker_body = m.body(BEAKER_CFG["body"]).id
    real_beaker_all = [g for g in range(m.ngeom) if m.geom_bodyid[g] == beaker_body and m.geom_contype[g] > 0]
    beaker_free = m.joint("task_beaker_free").qposadr[0]

    rows = []        # per moment
    poses = {}
    for name, pos, (ax, ay), held, phase in moments:
        q, status = solve(np.array(pos), (ax, ay))
        if q is None:
            print(f"  {name}: {status}")
            rows.append({"moment": name, "status": status})
            continue
        print(f"  {name}: {status}")
        poses[name] = q
        # beaker pose for this moment
        if held:
            bottom_now = pos[2] - f_mid * h
            bxy = (pos[0], pos[1])
        elif phase == "plate":
            bottom_now = plate_top + PICK_PLACE_CFG["setdown_gap_m"]
            bxy = (hx, hy)
        else:
            bottom_now = bottom
            bxy = (bx, by)
        # the real scene at this moment (kinematics only)
        d.qpos[qadr] = q
        d.qpos[beaker_free:beaker_free + 7] = [bxy[0], bxy[1], bottom_now, 1, 0, 0, 0]
        d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "openarm_left_finger_joint1")]] = FINGER_HELD if held else FINGER_OPEN
        d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "openarm_left_finger_joint2")]] = FINGER_HELD if held else FINGER_OPEN
        mujoco.mj_kinematics(m, d)
        # the filter's shapes for the same state
        state = RobotState(base_xyz=(sx, sy, 0.0), base_quat=(math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)),
                           pedestal_m=TASK_CFG["pedestal_q"],
                           q_left=tuple(q), q_right=tuple(home_r),
                           finger_left=FINGER_HELD if held else FINGER_OPEN, finger_right=FINGER_HELD if held else FINGER_OPEN)
        placed = {p.id: p for p in rs.place(state)}
        # the beaker: shape (catalog capsule) and its real geoms
        beaker_shape = as_placed(catalog_object("beaker", (bxy[0], bxy[1], bottom_now)))
        held_shape = as_placed(catalog_object("beaker", (bxy[0], bxy[1], bottom_now))) if held else None
        shapes = {
            "upper arm": [placed["left_upper_arm"]],
            "forearm": [placed["left_forearm"]],
            "link 7": [placed["left_link7"]],
            "palm": [placed["left_palm_1"], placed["left_palm_2"]],
            "fingers": [placed["left_finger_right"], placed["left_finger_left"]],
        }
        if held:
            shapes["held beaker"] = [held_shape]
        row = {"moment": name, "status": status, "pos": pos, "held": held, "phase": phase, "items": {}}
        for part, sh in shapes.items():
            geoms = ARM_PARTS.get(part, [])
            if part == "held beaker":
                real_a = real_beaker_all
            else:
                real_a = [geom_id(m, g) for g in geoms]
            s_bench = min(shape_gap(s, bench) for s in sh)
            s_hot = min(shape_gap(s, t) for s in sh for t in hot)
            if held and part == "held beaker":
                s_beaker = None
            else:
                s_beaker = min(shape_gap(s, beaker_shape) for s in sh)
            r_bench = real_gap(m, d, real_a, real_bench)
            r_hot = real_gap(m, d, real_a, real_hot)
            if held and part == "held beaker":
                r_beaker = None
            else:
                r_beaker = real_gap(m, d, real_a, real_beaker_all)
            row["items"][part] = {"shape": {"bench": s_bench, "hotplate": s_hot, "beaker": s_beaker},
                                  "real": {"bench": r_bench, "hotplate": r_hot, "beaker": r_beaker}}
        rows.append(row)

    def flag(v):
        if v is None:
            return "-"
        if v < MARGIN:
            return "MARGIN"
        if v < BAND:
            return "BAND"
        return ""

    print("\nmoment | shape | shape gap bench / hotplate / beaker (mm) | real gap bench / hotplate / beaker (mm) | flags (shape)")
    for row in rows:
        if "items" not in row:
            print(f"{row['moment']} | {row['status']}")
            continue
        for part, it in row["items"].items():
            sg, rg = it["shape"], it["real"]
            fmt = lambda v: "-" if v is None else f"{v * 1e3:+.1f}"
            flags = [f"{flag(v)}({nm})" for nm, v in (("bench", sg["bench"]), ("plate", sg["hotplate"]), ("beaker", sg["beaker"])) if v is not None and flag(v)]
            print(f"{row['moment']} | {part} | {fmt(sg['bench'])} / {fmt(sg['hotplate'])} / {fmt(sg['beaker'])} | "
                  f"{fmt(rg['bench'])} / {fmt(rg['hotplate'])} / {fmt(rg['beaker'])} | {','.join(flags)}")

    # ---- forearm looseness profile (10 points from elbow to wrist, max over the poses) and the two-capsule forearm ----
    j4 = m.joint("openarm_left_joint4").id
    j7 = m.joint("openarm_left_joint7").id
    fore_geoms = [geom_id(m, g) for g in ARM_PARTS["forearm"]]
    prof = np.full(10, -np.inf)
    prof_min = np.full(10, np.inf)
    per_pose = []   # (A, B, verts in world)
    for name, q in poses.items():
        d.qpos[qadr] = q
        mujoco.mj_kinematics(m, d)
        A, B = d.xanchor[j4].copy(), d.xanchor[j7].copy()
        # real vertices (mesh) in world
        Vw = []
        for g in fore_geoms:
            did = m.geom_dataid[g]
            Vm = m.mesh_vert[m.mesh_vertadr[did]:m.mesh_vertadr[did] + m.mesh_vertnum[did]].astype(float)
            Vw.append(Vm @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])
        Vw = np.vstack(Vw)
        per_pose.append((name, A, B, Vw))
    r_single = {}
    for name, A, B, Vw in per_pose:
        AB = B - A
        L = np.linalg.norm(AB)
        u = AB / L
        t = (Vw - A) @ u / L
        perp = np.linalg.norm((Vw - A) - np.outer((Vw - A) @ u, u), axis=1)
        # the current forearm capsule's radius at this pose (from the filter's placement)
        r_cap = next(sp["radius"] for sp in rs.specs if sp["id"] == "left_forearm")
        for i in range(10):
            s_i = i / 9.0
            sel = np.abs(t - s_i) <= 1.0 / 18.0
            ext = perp[sel].max() if sel.any() else 0.0
            prof[i] = max(prof[i], r_cap - ext)
            prof_min[i] = min(prof_min[i], r_cap - ext)
        r_single[name] = float(perp.max())
    lengths = [np.linalg.norm(B - A) for _, A, B, _ in per_pose]
    Lmean = float(np.mean(lengths))
    # two capsules on the elbow-wrist line, each with its own radius (max over poses of the vertices it covers)
    best = None
    for s_star in np.linspace(0.05, 0.95, 91):
        r1 = r2 = 0.0
        for name, A, B, Vw in per_pose:
            AB = B - A
            L = np.linalg.norm(AB)
            u = AB / L
            t = (Vw - A) @ u / L
            perp = np.linalg.norm((Vw - A) - np.outer((Vw - A) @ u, u), axis=1)
            r1 = max(r1, perp[t <= s_star].max() if (t <= s_star).any() else 0.0)
            r2 = max(r2, perp[t > s_star].max() if (t > s_star).any() else 0.0)
        L1, L2 = s_star * Lmean, (1 - s_star) * Lmean
        vol = math.pi * r1 ** 2 * (L1 + 4 / 3 * r1) + math.pi * r2 ** 2 * (L2 + 4 / 3 * r2)
        if best is None or vol < best[0]:
            best = (vol, s_star, r1, r2)
    vol_s, s_star, r1, r2 = best
    if not per_pose:
        raise SystemExit("no pose converged: nothing to measure")
    r_one = max(r_single.values())
    vol_one = math.pi * r_one ** 2 * (Lmean + 4 / 3 * r_one)
    print("\nforearm looseness: gap between the capsule surface and the real forearm geometry (mm), 10 points elbow -> wrist, max over the poses")
    print("  loosest (max over poses): " + "  ".join(f"{i / 9:.2f}:{v * 1e3:+.1f}" for i, v in enumerate(prof)))
    print("  tightest (min over poses): " + "  ".join(f"{i / 9:.2f}:{v * 1e3:+.1f}" for i, v in enumerate(prof_min)))
    print(f"two-capsule forearm: split at {s_star:.2f} of the elbow-wrist line (length {Lmean * 100:.1f} cm), radii {r1 * 100:.2f} cm (elbow) and {r2 * 100:.2f} cm (wrist); "
          f"volume {vol_s * 1e6:.1f} cm^3 vs one capsule {vol_one * 1e6:.1f} cm^3 (radius {r_one * 100:.2f} cm)")

    # two-capsule forearm gaps at the same moments
    print("\ntwo-capsule forearm gaps (mm): moment | capsule elbow / wrist | shape to bench / hotplate / beaker (min of the two) | flags")
    for row in rows:
        if "items" not in row:
            continue
        q = poses[row["moment"]]
        d.qpos[qadr] = q
        mujoco.mj_kinematics(m, d)
        A, B = d.xanchor[j4].copy(), d.xanchor[j7].copy()
        AB = B - A
        L = np.linalg.norm(AB)
        u = AB / L
        P = A + s_star * AB
        caps = [PlacedShape(id="fore_elbow", arm="left", part="forearm", kind="capsule", body="", a=A, b=P, r=r1),
                PlacedShape(id="fore_wrist", arm="left", part="forearm", kind="capsule", body="", a=P, b=B, r=r2)]
        held = row["held"]
        bxy = (row["pos"][0], row["pos"][1]) if held else ((hx, hy) if row["phase"] == "plate" else (bx, by))
        bz = (row["pos"][2] - f_mid * h) if held else (plate_top + PICK_PLACE_CFG["setdown_gap_m"] if row["phase"] == "plate" else bottom)
        bshape = as_placed(catalog_object("beaker", (bxy[0], bxy[1], bz)))
        sb = min(shape_gap(c, bench) for c in caps)
        sh = min(shape_gap(c, t) for c in caps for t in hot)
        sbe = min(shape_gap(c, bshape) for c in caps)
        fl = [f"{flag(v)}({nm})" for nm, v in (("bench", sb), ("plate", sh), ("beaker", sbe)) if flag(v)]
        ge, gw = shape_gap(caps[0], bench), shape_gap(caps[1], bench)
        print(f"{row['moment']} | {ge * 1e3:+.1f} / {gw * 1e3:+.1f} | {sb * 1e3:+.1f} / {sh * 1e3:+.1f} / {sbe * 1e3:+.1f} | {','.join(fl)}")

    out = {"k_pick": k_pick, "k_place": k_place, "forearm_profile_max_mm": (prof * 1e3).tolist(), "forearm_profile_min_mm": (prof_min * 1e3).tolist(),
           "two_capsule": {"split": s_star, "r_elbow": r1, "r_wrist": r2, "length_m": Lmean},
           "one_capsule_radius": r_one}
    print("\nJSON " + json.dumps(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
