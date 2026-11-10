"""Safety filter: robot-vs-robot pairs at the poses we use (read-only; kinematics only, nothing is stepped).

Poses, at pedestal 0.155 m (the staging stance's height) and 0.12 m:
  P1  both arms at the zombie pose (= the home pose, TASK_CFG home_q_left / home_q_right; the same pose in this code)
  P2  both arms at the home pose (identical to P1; listed to show it)
  P3  left arm at each pick-and-place moment M1 to M9 (the IK of experiments/safety_filter_pick_scene.py), right arm at zombie
  P4  the close-in carry with the held beaker: M5 (carry) and M6 (over the plate) with the left arm; right at zombie
Fingers: open (0.044 m) at P1, P2 and M1-M3, M8-M9; closed on the beaker (0.037 m) at M4-M7 (held).

For every checked robot-vs-robot pair (the touch rules of src/safety_filter/touch.py: arm vs other arm, arm vs torso and
body, fingers vs robot, held object vs other arm and body; the held object's own arm is exempt):
  shape gap     the filter's shapes (touch.evaluate_pairs)
  real gap      MuJoCo mj_geomDistance between the real collision geoms of the two shapes (scoring only)
  margin        hand vs hand 1 cm; any other robot vs robot pair 3 cm
  MARGIN        shape gap below the margin
  BAND          shape gap below margin + 1 cm (the push-start band)
  REAL-INSIDE   real gap below the margin (no honest shape could pass; the margin would have to change)
Exempt pairs (touch rules) are listed separately with the shape gap and the real gap.

Writes nothing to data/ or the shapes file. Prints the tables.
"""

import math
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from safety_filter_pick_scene import setup, FINGER_OPEN, FINGER_HELD, _jadr  # noqa: E402
from src.task_config import TASK_CFG, BEAKER_CFG  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes, RobotState  # noqa: E402
from src.safety_filter.world import catalog_object  # noqa: E402
from src.safety_filter.touch import as_placed, evaluate_pairs, pair_rule, shape_of, Exempt, Rule, gap_between  # noqa: E402

SHAPES = ROOT / "data" / "safety_filter" / "robot_shapes.json"
PEDESTALS = (0.155, 0.12)
HOME_L = np.array(TASK_CFG["home_q_left"], float)
HOME_R = np.array(TASK_CFG["home_q_right"], float)


def scene_geoms(m, names):
    out = []
    for n in names:
        g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
        if g >= 0:
            out.append(g)
    return out


def set_scene(ctx, q_l, q_r, f_l, f_r, held_xyz=None):
    """Puts the scene in one pose (kinematics only). held_xyz: beaker bottom-centre (x, y, z) when held, else the beaker stays on the bench."""
    m, d = ctx["m"], ctx["d"]
    d.qpos[ctx["qadr"]] = q_l
    for k, v in enumerate(q_r, start=1):
        d.qpos[_jadr(m, f"openarm_right_joint{k}")] = v
    for side, f in (("left", f_l), ("right", f_r)):
        d.qpos[_jadr(m, f"openarm_{side}_finger_joint1")] = f
        d.qpos[_jadr(m, f"openarm_{side}_finger_joint2")] = f
    if held_xyz is not None:
        free = m.joint("task_beaker_free").qposadr[0]
        d.qpos[free:free + 7] = [held_xyz[0], held_xyz[1], held_xyz[2], 1, 0, 0, 0]
    mujoco.mj_kinematics(m, d)


def real_geom_names(shape_geoms):
    return list(shape_geoms)


def pose_rows(ctx, rs, name, q_l, q_r, f_l, f_r, held_info):
    """Checked pairs and exempt pairs for one pose. held_info: None, or (bottom xy, bottom z) of the held beaker."""
    m, d = ctx["m"], ctx["d"]
    sx, sy, yaw = ctx["sx"], ctx["sy"], ctx["yaw"]
    ped = ctx["pedestal"]
    state = RobotState(base_xyz=(sx, sy, 0.0), base_quat=(math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)),
                       pedestal_m=ped, q_left=tuple(q_l), q_right=tuple(q_r), finger_left=f_l, finger_right=f_r)
    placed = rs.place(state)
    beaker_geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] == m.body(BEAKER_CFG["body"]).id and m.geom_contype[g] > 0]
    held = None
    if held_info is not None:
        (bx, by), bz = held_info
        bshape = catalog_object("beaker", (bx, by, bz))
        held = shape_of("left_held", bshape.shape, bshape.size, bshape.centre, bshape.rot_matrix(), "left", "held", body="openarm_left_link7")
    set_scene(ctx, q_l, q_r, f_l, f_r, None if held_info is None else (held_info[0][0], held_info[0][1], held_info[1]))
    all_shapes = list(placed) + ([held] if held is not None else [])
    geoms_of = {p.id: list(p.geoms) for p in placed}
    if held is not None:
        geoms_of["left_held"] = ["__beaker__"]
    by_id = {p.id: p for p in all_shapes}

    def real_min(a_id, b_id):
        A = beaker_geoms if a_id == "left_held" else scene_geoms(m, geoms_of[a_id])
        B = beaker_geoms if b_id == "left_held" else scene_geoms(m, geoms_of[b_id])
        best, pair, fromto = np.inf, None, np.zeros(6)
        for ga_ in A:
            for gb_ in B:
                dist = mujoco.mj_geomDistance(m, d, ga_, gb_, 0.5, fromto)
                if dist < best:
                    best, pair = dist, (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, ga_), mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, gb_))
        return best, pair

    rows, exempt = [], []
    # every pair of the robot's shapes (held included): checked pairs and exempt pairs
    for i in range(len(all_shapes)):
        for j in range(i + 1, len(all_shapes)):
            a, b = all_shapes[i], all_shapes[j]
            if a.kind == "box" and b.kind == "box":
                continue
            r = pair_rule(a, b)
            gap = gap_between(a, b)[0]
            if isinstance(r, Exempt):
                if gap < 0.05:
                    rg, _ = real_min(a.id, b.id)
                    exempt.append((a.id, b.id, r.reason, gap, rg))
                continue
            if gap > r.activation + 0.02 and {a.id, b.id} != {"right_palm_1", "torso"}:
                continue
            rg, pair = real_min(a.id, b.id)
            flags = []
            if gap < r.margin:
                flags.append("MARGIN")
            elif gap < r.margin + 0.01:
                flags.append("BAND")
            if rg < r.margin:
                flags.append("REAL-INSIDE")
            rows.append((a.id, b.id, r.kind, r.margin, gap, rg, pair, flags))
    return rows, exempt


def fmt(v):
    return f"{v * 1e3:+.1f}"


def print_rows(title, rows, exempt):
    print(f"\n### {title}")
    print("pair | margin (mm) | shape gap | real gap | flags | real geoms (closest)")
    for a, b, kind, mg, gap, rg, pair, flags in rows:
        print(f"{a} vs {b} [{kind}] | {mg * 1e3:.0f} | {fmt(gap)} | {fmt(rg)} | {','.join(flags) or '-'} | {pair}")
    flagged = sum(1 for r in rows if r[7])
    held_gaps = [r[4] for r in rows if "left_held" in (r[0], r[1]) or "right_held" in (r[0], r[1])]
    print(f"held object: checked pairs within margin + 2 cm: {len(held_gaps)}" + (f"; smallest shape gap {min(held_gaps) * 1e3:+.1f} mm" if held_gaps else ""))
    print(f"checked pairs within margin + 2 cm: {len(rows)}; flagged {flagged}")
    same = [e for e in exempt if e[2].startswith("same arm")]
    other = [e for e in exempt if not e[2].startswith("same arm")]
    print(f"exempt pairs with shape gap < 50 mm: {len(exempt)} ({len(same)} same arm, not listed)")
    for a, b, reason, gap, rg in other:
        print(f"  EXEMPT {a} vs {b} | shape {fmt(gap)} | real {fmt(rg)} | {reason}")


def main():
    summary = []
    for ped in PEDESTALS:
        ctx = setup(verbose=True, pedestal=ped)
        rs = RobotShapes(load_robot_only_model(), SHAPES)
        fo, fh = FINGER_OPEN, FINGER_OPEN
        print(f"\n==================== pedestal {ped:.3f} m ====================")
        print("solved moments:", {k: v for k, v in ctx["status"].items()})
        # P1 zombie (= home) pose for both arms, grippers open
        rows, ex = pose_rows(ctx, rs, "P1", HOME_L, HOME_R, fo, fo, None)
        print_rows(f"P1 zombie pose, both arms (pedestal {ped:.3f})", rows, ex)
        summary.append((ped, "P1", rows))
        # P2 home pose: identical to P1 in this code (zombie = home); recomputed to show it
        rows2, _ = pose_rows(ctx, rs, "P2", HOME_L, HOME_R, fo, fo, None)
        same = all(abs(a[4] - b[4]) < 1e-9 for a, b in zip(rows, rows2)) and len(rows) == len(rows2)
        print(f"P2 home pose: identical to P1 = {same}")
        # P3: left at M1..M9, right at zombie; P4 = the M5/M6 carry rows (held)
        for mname, pos, a_, held, phase in ctx["moments"]:
            if mname not in ctx["poses"]:
                print(f"P3 {mname}: not solved ({ctx['status'][mname]})")
                continue
            q_l = ctx["poses"][mname]
            if held:
                f_l = FINGER_HELD
                held_info = ((pos[0], pos[1]), pos[2] - ctx["f_mid"] * ctx["h"])
            else:
                f_l = FINGER_OPEN
                if phase == "plate":
                    held_info = None
                    bxy = (ctx["hx"], ctx["hy"])
                else:
                    held_info = None
            label = "P4" if mname in ("M5", "M6") else "P3"
            rows3, ex3 = pose_rows(ctx, rs, mname, q_l, HOME_R, f_l, fo, held_info)
            print_rows(f"{label} {mname} (left arm at {mname}, right at zombie; pedestal {ped:.3f}; left fingers {f_l:.3f})", rows3, ex3)
            summary.append((ped, f"{label} {mname}", rows3))
    # ---- summary of MARGIN flags ----
    print("\n==================== SUMMARY: every MARGIN flag ====================")
    for ped, label, rows in summary:
        for a, b, kind, mg, gap, rg, pair, flags in rows:
            if "MARGIN" in flags:
                kindflag = "REAL-INSIDE" if "REAL-INSIDE" in flags else ("loose" if rg > mg else "close")
                print(f"ped {ped:.3f} {label}: {a} vs {b} [{kind}] shape {fmt(gap)} real {fmt(rg)} margin {mg * 1e3:.0f} -> {kindflag}; real geoms {pair}")
    print("\nSUMMARY of right palm 1 vs torso:")
    for ped, label, rows in summary:
        for a, b, kind, mg, gap, rg, pair, flags in rows:
            if {a, b} == {"right_palm_1", "torso"}:
                print(f"ped {ped:.3f} {label}: shape {fmt(gap)} real {fmt(rg)} margin {mg * 1e3:.0f} flags {flags} real geoms {pair}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
