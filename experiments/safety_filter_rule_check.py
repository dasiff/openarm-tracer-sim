"""SF-16 rule check and the real-geometry torso test (kinematics only; nothing stepped).

SF-16 (extended to robot vs robot by SF-18), at the poses we use:
  arm vs world (1 cm margin):     the left arm's checked shapes at M1 to M9 (pedestals 0.155 and 0.120 m)
  robot vs robot (hand vs hand 1 cm, other 3 cm): all checked pairs at P1 (zombie = home, both arms), P3 (left at M1 to M9,
                                  right at home) and P4 (the carry rows M5, M6 of P3), pedestals 0.155 and 0.120 m
  MARGIN = shape gap below the margin (the rule forbids it); BAND = below margin + 1 cm (allowed).

Poses are solved once with the scene's IK (experiments/safety_filter_pick_scene.setup) and cached in data/logs/rule_poses.json.
Real gaps (MuJoCo on the real geoms) are reported for diagnosis only.
"""

import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.task_config import TASK_CFG, BEAKER_CFG  # noqa: E402
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import RobotShapes, RobotState  # noqa: E402
from src.safety_filter.touch import as_placed, pair_rule, Exempt, gap_between, shape_of  # noqa: E402
from src.safety_filter.world import LabMapSource, catalog_object  # noqa: E402
from src.safety_filter.shapes import PlacedShape  # noqa: E402

POSE_CACHE = ROOT / "data" / "logs" / "rule_poses.json"
LAB_MAP = ROOT / "data" / "safety_filter" / "lab_map.json"
PEDESTALS = (0.155, 0.12)
HOME_L = list(TASK_CFG["home_q_left"])
HOME_R = list(TASK_CFG["home_q_right"])
FO, FH = 0.044, 0.037
STAGING = (TASK_CFG["staging_xy"][0], TASK_CFG["staging_xy"][1], TASK_CFG["staging_yaw"])


def build_pose_cache(path=POSE_CACHE):
    """Solves the poses with the scene (IK) and writes the cache. Takes a few minutes (two scene solves)."""
    from safety_filter_pick_scene import setup
    poses = []
    for ped in PEDESTALS:
        ctx = setup(verbose=True, pedestal=ped)
        common = dict(ped=ped, hot=[TASK_CFG["hotplate_xy"][0], TASK_CFG["hotplate_xy"][1], ctx["top"]],
                      plate_top=ctx["plate_top"], bench_top=ctx["top"], h=ctx["h"], f_mid=ctx["f_mid"],
                      bottom=ctx["bottom"], bx=ctx["bx"], by=ctx["by"])
        poses.append(dict(name="P1", group="P1", moment=None, q_left=HOME_L, q_right=HOME_R, f_left=FO, f_right=FO,
                          held=False, held_xy=None, held_z=None, **common))
        for name, pos, a_, held, phase in ctx["moments"]:
            if name not in ctx["poses"]:
                continue
            if held:
                held_xy, held_z = [pos[0], pos[1]], pos[2] - ctx["f_mid"] * ctx["h"]
            else:
                held_xy, held_z = None, None
            poses.append(dict(name=f"P3 {name}", group="P3", moment=name, q_left=[float(v) for v in ctx["poses"][name]],
                              q_right=HOME_R, f_left=FH if held else FO, f_right=FO, held=bool(held),
                              held_xy=held_xy, held_z=held_z, phase=phase, **common))
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(poses), encoding="utf-8")
    import os
    os.replace(tmp, path)
    return poses


def load_poses(path=POSE_CACHE):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def state_of(p):
    return RobotState(base_xyz=(STAGING[0], STAGING[1], 0.0),
                      base_quat=(math.cos(STAGING[2] / 2), 0, 0, math.sin(STAGING[2] / 2)),
                      pedestal_m=p["ped"], q_left=tuple(p["q_left"]), q_right=tuple(p["q_right"]),
                      finger_left=p["f_left"], finger_right=p["f_right"])


def placed_with_held(rs, p):
    placed = rs.place(state_of(p))
    held = None
    if p["held"]:
        bs = catalog_object("beaker", (p["held_xy"][0], p["held_xy"][1], p["held_z"]))
        held = shape_of("left_held", bs.shape, bs.size, bs.centre, bs.rot_matrix(), "left", "held", body="openarm_left_link7")
    return placed, held


def world_shapes(p, lab_objs):
    """World shapes for the arm-vs-world check at a moment: lab fixtures, the hotplate (base and plate), the beaker (if on a surface)."""
    out = [as_placed(o) for o in lab_objs]
    hx, hy, top = p["hot"]
    out.append(PlacedShape(id="hotplate_base", arm="world", part="world", kind="box", body="world",
                           centre=np.array([hx, hy, top + 0.03]), rot=np.eye(3), half=np.array([0.09, 0.09, 0.03])))
    out.append(PlacedShape(id="hotplate_plate", arm="world", part="world", kind="box", body="world",
                           centre=np.array([hx, hy, top + 0.064]), rot=np.eye(3), half=np.array([0.075, 0.075, 0.004])))
    if not p["held"]:
        # the beaker stands on the bench before the pick and on the plate after the place
        if p["moment"] in ("M8", "M9"):
            bx, by, bz = hx, hy, p["plate_top"] + cfg_setdown()
        else:
            bx, by, bz = p["bx"], p["by"], p["bottom"]
        bs = catalog_object("beaker", (bx, by, bz))
        out.append(as_placed(bs))
    return out


def cfg_setdown():
    from src.task_config import PICK_PLACE_CFG
    return PICK_PLACE_CFG["setdown_gap_m"]


def evaluate(shapes_path, poses, rs=None, only_ids=None):
    """All SF-16 flags at the given poses. Returns a list of dicts (MARGIN and BAND only).
    only_ids: if given, only pairs with at least one shape of these ids are evaluated."""
    if rs is None:
        rs = RobotShapes(load_robot_only_model(), shapes_path)
    lab_objs = LabMapSource(LAB_MAP).objects()
    out = []
    margin_arm_world = cfg.MARGINS["arm_world_m"]
    for p in poses:
        placed, held = placed_with_held(rs, p)
        robot = placed + ([held] if held is not None else [])
        if p["moment"] is not None:  # arm vs world: the left arm's checked shapes (not fingers, not held)
            world = world_shapes(p, lab_objs)
            for s in placed:
                if s.arm != "left" or s.part in ("finger",):
                    continue
                if only_ids is not None and s.id not in only_ids:
                    continue
                for w in world:
                    g = gap_between(s, w)[0]
                    if g < margin_arm_world + 0.01:
                        out.append(dict(pose=p["name"], ped=p["ped"], kind="arm-world", a=s.id, b=w.id,
                                        gap=float(g), margin=margin_arm_world,
                                        flag="MARGIN" if g < margin_arm_world else "BAND"))
        for i in range(len(robot)):
            for j in range(i + 1, len(robot)):
                a, b = robot[i], robot[j]
                if a.kind == "box" and b.kind == "box":
                    continue
                if only_ids is not None and a.id not in only_ids and b.id not in only_ids:
                    continue
                r = pair_rule(a, b)
                if isinstance(r, Exempt):
                    continue
                g = gap_between(a, b)[0]
                if g < r.margin + 0.01:
                    out.append(dict(pose=p["name"], ped=p["ped"], kind="robot-robot", a=a.id, b=b.id,
                                    gap=float(g), margin=r.margin, flag="MARGIN" if g < r.margin else "BAND"))
    return out


def real_home_torso(shapes_path=None):
    """Real-geometry torso test at home (SF-19): for each arm shape, the MuJoCo gap between its link's real collision
    geoms and the real torso mesh, at the home pose (pedestal 0.155, fingers open). Robot-only model (no scene needed)."""
    rs = RobotShapes(load_robot_only_model(), shapes_path or (ROOT / "data" / "safety_filter" / "robot_shapes.json"))
    p = dict(ped=0.155, q_left=HOME_L, q_right=HOME_R, f_left=FO, f_right=FO)
    placed = rs.place(state_of(p))          # also sets the robot-only model's kinematics for this pose
    m, d = rs.model, rs.data
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "openarm_body_link0_collision")
    fr = np.zeros(6)
    rows = []
    for s in placed:
        if s.arm not in ("left", "right") or s.part == "torso":
            continue
        gs = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, g) for g in s.geoms]
        real = min(mujoco.mj_geomDistance(m, d, g, torso, 0.5, fr) for g in gs if g >= 0)
        rows.append((s.id, s.part, s.segment, float(gap_between(s, placed_torso(placed))[0]), float(real)))
    return rows


def placed_torso(placed):
    return next(p for p in placed if p.id == "torso")


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--build-poses", action="store_true")
    ap.add_argument("--torso", action="store_true")
    ap.add_argument("--shapes", default=str(ROOT / "data" / "safety_filter" / "robot_shapes.json"))
    a = ap.parse_args()
    if a.build_poses:
        build_pose_cache()
        print("pose cache written", POSE_CACHE)
    if a.torso:
        for row in real_home_torso(a.shapes):
            print(f"{row[0]:24s} {row[1]:14s} {row[2]:9s} shape {row[3] * 1e3:+8.1f}  real {row[4] * 1e3:+8.1f} mm")
    if not a.build_poses and not a.torso:
        poses = load_poses()
        v = evaluate(a.shapes, poses)
        print(f"{len(v)} flagged rows")
        for r in v:
            print(f"{r['flag']:6s} ped {r['ped']:.3f} {r['pose']:8s} {r['kind']:10s} {r['a']} vs {r['b']} gap {r['gap'] * 1e3:+.1f} margin {r['margin'] * 1e3:.0f}")
