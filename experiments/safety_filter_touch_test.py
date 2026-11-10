"""Safety filter: touch rules table, held objects (attach, release, fallback), broad phase, catalog and config.

No simulation stepping. MuJoCo is used only to read the robot model's kinematics and the object model files
(the object sizes are read from the XML text, the filter itself reads nothing of the sort).

Run: python experiments/safety_filter_touch_test.py   (exit 0 = pass)
"""

import sys
import warnings
from dataclasses import replace
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.robot_model import load_robot_only_model  # noqa: E402
from src.safety_filter.shapes import PlacedShape, RobotShapes, RobotState  # noqa: E402
from src.safety_filter.touch import (Exempt, HeldTracker, Rule, broad_phase, evaluate_pairs,  # noqa: E402
                                     pair_rule)
from src.safety_filter.world import LabMapSource, WorldModelAdapter, WorldObject, WorldView, catalog_object  # noqa: E402

SHAPES = ROOT / "data" / "safety_filter" / "robot_shapes.json"
LAB_MAP = ROOT / "data" / "safety_filter" / "lab_map.json"
FAILS = []


def check(name, cond, detail=""):
    print(f"  {'ok  ' if cond else 'FAIL'} {name} {detail}")
    if not cond:
        FAILS.append(name)


def cap(sid, arm, part, body="x"):
    return PlacedShape(id=sid, arm=arm, part=part, kind="capsule", body=body,
                       a=np.zeros(3), b=np.array([0.0, 0.0, 0.1]), r=0.01)


def box(sid, arm, part):
    return PlacedShape(id=sid, arm=arm, part=part, kind="box", body=arm, centre=np.zeros(3),
                       rot=np.eye(3), half=np.full(3, 0.01))


def table():
    print("touch rule table (one case per pair kind)")
    cases = [
        ("arm-world: left link 7 vs bench", cap("l7", "left", "link7"), box("bench", "world", "world"), "arm-world", cfg.MARGINS["arm_world_m"]),
        ("finger vs world: exempt", cap("lf", "left", "finger"), box("bench", "world", "world"), None, None),
        ("held object vs world: exempt", cap("lh", "left", "held"), box("bench", "world", "world"), None, None),
        ("same arm: link 7 vs own palm: exempt", cap("l7", "left", "link7"), cap("lp", "left", "palm"), None, None),
        ("held vs own hand: exempt (SF-4)", cap("lh", "left", "held"), cap("lp", "left", "palm"), None, None),
        ("torso vs the left upper arm (touches by construction): exempt", cap("t", "common", "torso"), cap("left_upper_arm", "left", "upper_arm"), None, None),
        ("torso vs the left shoulder mount: exempt", cap("t", "common", "torso"), cap("left_shoulder_mount", "left", "shoulder_mount"), None, None),
        ("torso vs the right upper arm: checked (it is +18 mm at the zero pose)", cap("t", "common", "torso"), cap("right_upper_arm", "right", "upper_arm"), "robot-robot", cfg.MARGINS["robot_robot_m"]),
        ("torso vs the left forearm: checked", cap("t", "common", "torso"), cap("left_forearm", "left", "forearm"), "robot-robot", cfg.MARGINS["robot_robot_m"]),
        ("torso vs a held object: checked", cap("t", "common", "torso"), cap("left_held", "left", "held"), "robot-robot", cfg.MARGINS["robot_robot_m"]),
        ("robot body vs robot body: exempt", cap("eb", "common", "pedestal_base"), box("pt", "common", "pedestal_top"), None, None),
        ("shoulder mount vs the other mount: exempt", cap("lm", "left", "shoulder_mount"), cap("rm", "right", "shoulder_mount"), None, None),
        ("hand-hand: left palm vs right palm", cap("lp", "left", "palm"), cap("rp", "right", "palm"), "hand-hand", cfg.MARGINS["hand_hand_m"]),
        ("held (left) vs right palm: hand-hand", cap("lh", "left", "held"), cap("rp", "right", "palm"), "hand-hand", cfg.MARGINS["hand_hand_m"]),
        ("robot-robot: left upper arm vs right forearm", cap("lu", "left", "upper_arm"), cap("rf", "right", "forearm"), "robot-robot", cfg.MARGINS["robot_robot_m"]),
        ("robot-robot: left upper arm vs pedestal base", cap("lu", "left", "upper_arm"), box("pb", "common", "pedestal_base"), "robot-robot", cfg.MARGINS["robot_robot_m"]),
    ]
    for name, a, b, kind, margin in cases:
        r = pair_rule(a, b)
        if kind is None:
            check(name, isinstance(r, Exempt), f"-> exempt ({getattr(r, 'reason', '')})")
        else:
            ok = isinstance(r, Rule) and r.kind == kind and abs(r.margin - margin) < 1e-12
            ok = ok and abs(r.activation - (margin + cfg.PUSH_START_BEYOND_M)) < 1e-12
            ok = ok and abs(r.release - (margin + cfg.PUSH_STOP_BEYOND_M)) < 1e-12
            check(name, ok, f"-> {getattr(r, 'kind', r)} margin {getattr(r, 'margin', 0) * 100:.0f} cm")


def held_and_broad():
    print("held objects and the broad phase (real robot shapes, no stepping)")
    rs = RobotShapes(load_robot_only_model(), SHAPES)
    st = RobotState(pedestal_m=0.15, q_left=(0, 0, 0, 0.5, 0, 0, 0), q_right=(0, 0, 0, 0.5, 0, 0, 0), finger_left=0.02, finger_right=0.02)
    placed = rs.place(st)
    fingers = [p for p in placed if p.part == "finger" and p.arm == "left"]
    pad = np.mean([(f.a + f.b) / 2 for f in fingers], axis=0)
    pos, rot = rs.hand_frame("left")

    beaker = catalog_object("beaker", pad - np.array([0, 0, 0.05]))  # centre at the pad: bottom centre = pad - 0.05
    far_beaker = catalog_object("beaker", pad + np.array([1.5, 0, 0]) - np.array([0, 0, 0.05]))
    far_beaker = replace(far_beaker, id="beaker_far")  # ids must be unique in the world
    world = WorldView(_Src([beaker, far_beaker]))
    ht = HeldTracker(world)
    ev = ht.update("left", True, placed, pos, rot)
    check("attach: the beaker at the pads is matched", ev == "attach", f"-> {ev}")
    ids = [o.id for o in world.objects()]
    check("attach: the held beaker left the world (the far one stays)", ids == ["beaker_far"], f"-> {ids}")
    held0 = ht.placed("left", pos, rot)
    check("attach: held shape exists and is a capsule (the beaker is a capsule, SF-8)", held0 is not None and held0.kind == "capsule")

    # move the arm: the held shape must keep its offset in the hand frame
    st2 = RobotState(pedestal_m=0.15, q_left=(0.3, -0.2, 0.1, 0.9, 0.2, 0.1, 0.0), q_right=st.q_right,
                     finger_left=0.02, finger_right=0.02)
    placed2 = rs.place(st2)
    pos2, rot2 = rs.hand_frame("left")
    held1 = ht.placed("left", pos2, rot2)
    off_before = rot.T @ (held0.centre - pos)
    off_after = rot2.T @ (held1.centre - pos2)
    check("held follows the hand: fixed offset in the hand frame", np.allclose(off_before, off_after, atol=1e-9),
          f"(max diff {np.max(np.abs(off_before - off_after)):.2e} m)")
    pairs = evaluate_pairs(placed2 + [held1], [])
    check("held vs own hand is not a checked pair", not any(
        p.a == "left_held" and p.b.startswith("left_palm") or p.b == "left_held" and p.a.startswith("left_palm") for p in pairs))

    # release: the object returns at the pose the hand left it
    centre_before = np.asarray(held1.centre)
    ev = ht.update("left", False, placed2, pos2, rot2)
    back = world.find("beaker")
    check("release: the beaker returns to the world", ev == "release" and back is not None, f"-> {ev}")
    check("release: at the pose the hand left it", back is not None and np.allclose(back.centre, centre_before, atol=1e-9))

    # fallback: nothing graspable at the pads: a capsule from the finger gap, with a warning
    world2 = WorldView(_Src([catalog_object("beaker", pad + np.array([0.5, 0, 0]) - np.array([0, 0, 0.05]))]))
    ht2 = HeldTracker(world2)
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        ev = ht2.update("left", True, placed, pos, rot)
    check("fallback: no object at the pads gives the fallback event", ev == "fallback", f"-> {ev}")
    check("fallback: a warning is logged", any("fallback" in str(x.message) for x in w))
    fb = ht2.placed("left", pos, rot)
    check("fallback: the held shape is a capsule of the default height",
          fb is not None and fb.kind == "capsule" and abs(np.linalg.norm(fb.b - fb.a) - cfg.HELD_FALLBACK_HEIGHT_M) < 1e-9)
    check("fallback: radius from the finger gap (half the gap)", fb is not None and fb.r > 0.005)
    ev = ht2.update("left", False, placed, pos, rot)
    check("fallback release: nothing returns to the world", ev == "release" and world2.find("beaker") is not None
          and sum(1 for o in world2.objects() if o.id == "beaker") == 1)

    # broad phase: near objects are candidates; far ones are not; pairs being pushed stay until released
    far = catalog_object("beaker", np.array([3.0, 0.0, 0.0]))
    near = catalog_object("beaker", pad - np.array([0, 0, 0.05]) + np.array([0.02, 0, 0]))
    cands = [o.id + str(i) for i, o in enumerate(broad_phase(placed, placed, [near, far], 0.01, 0.03))]
    check("broad phase: a near object is a candidate", any(c.startswith("beaker0") for c in cands))
    check("broad phase: a far object is not", not any(c.startswith("beaker1") for c in cands))
    kept = broad_phase(placed, placed, [near, far], 0.01, 0.03, keep={"beaker"})
    check("broad phase: a pair being pushed is kept while far (until released)", len(kept) == 2)
    dropped = broad_phase(placed, placed, [near, far], 0.01, 0.03, keep=())
    check("broad phase: released (not kept) and far: dropped", all(o is not far for o in dropped))


class _Src:
    def __init__(self, objs):
        self._objs = objs

    def objects(self):
        return list(self._objs)


def catalog_and_config():
    print("object catalog against the object model files, lab map, adapter")
    root = ET.parse(ROOT / "models" / "objects" / "beaker.xml").getroot()
    g = [e for e in root.iter("geom") if e.get("name") == "task_beaker_glass"][0]
    size = [float(x) for x in g.get("size").split()]
    pos = [float(x) for x in g.get("pos").split()]
    cat = cfg.OBJECT_CATALOG["beaker"]
    check("catalog: the beaker is a capsule (round, M287, SF-8)", cat["shape"] == "capsule" and cat["round"] is True)
    check("catalog: beaker radius matches models/objects/beaker.xml", abs(cat["radius_m"] - size[0]) < 1e-12, f"({size[0]})")
    check("catalog: beaker half height matches", abs(cat["half_height_m"] - size[1]) < 1e-12, f"({size[1]})")
    check("catalog: beaker centre above its bottom matches", abs(cat["centre_above_bottom_m"] - pos[2]) < 1e-12, f"({pos[2]})")
    hp = ET.parse(ROOT / "models" / "objects" / "hotplate.xml").getroot()
    g = [e for e in hp.iter("geom") if e.get("name") == "task_hotplate_base"][0]
    hs = tuple(float(x) for x in g.get("size").split())
    hz = float(g.get("pos").split()[2])
    check("catalog: hotplate half sizes match models/objects/hotplate.xml", tuple(cfg.OBJECT_CATALOG["hotplate"]["half_m"]) == hs, f"({hs})")
    check("catalog: hotplate centre above its bottom matches", abs(cfg.OBJECT_CATALOG["hotplate"]["centre_above_bottom_m"] - hz) < 1e-12)

    lab = LabMapSource(LAB_MAP)
    import hashlib
    scene = ROOT / "models" / "scenes" / "chemistry_lab_beaker_hotplate.xml"
    sha = hashlib.sha256(scene.read_bytes()).hexdigest()
    check("lab map: fingerprint equals the scene file's SHA-256", sha == lab.scene_sha256)
    names = {o.id for o in lab.objects()}
    check("lab map: no hotplate, beaker or other movable object", not any(n.startswith(("task_", "beaker", "hotplate")) for n in names))
    check("lab map: bench top and walls present", "bench_top" in names and "wall_n" in names)
    check("lab map: bench built for bench_open True (the rails are not written)", lab.bench_open is True
          and not any(n.startswith("bench_frame") for n in names))
    check("lab map: gas manifold and knobs are fixtures", {"gas_manifold", "gas_knob_1", "gas_knob_2", "gas_knob_3"} <= names)
    from src.safety_filter.world import LabMapMismatch
    refused = False
    try:
        LabMapSource(LAB_MAP, expected_bench_open=False)
    except LabMapMismatch:
        refused = True
    check("lab map: a filter expecting the other bench_open setting refuses the map", refused)

    class FakeEntry:
        def __init__(self, id_, position):
            self.id, self.position, self.kind = id_, position, "object"

    class FakeWorld:
        def entries(self, kind=None):
            return [FakeEntry("beaker", np.array([0.1, 0.2, 0.9])), FakeEntry("unknown_thing", np.zeros(3))]

    ad = WorldModelAdapter(FakeWorld())
    objs = ad.objects()
    check("adapter: beaker from the world model, centre above its bottom",
          len(objs) == 1 and np.allclose(objs[0].centre, [0.1, 0.2, 0.95]))
    check("adapter: unknown entries are skipped and listed", ad.skipped == ["unknown_thing"])

    print("torso exemption (SF-9, re-checked after the forearm and link 7 pieces: docs/reports/20261110_1359_safety_filter_shape_refinement.md)")
    from src.safety_filter.config import TORSO_TOUCHING_IDS
    check("torso list is the decided three", tuple(sorted(TORSO_TOUCHING_IDS)) == ("left_shoulder_mount", "left_upper_arm", "right_shoulder_mount"))
    check("no forearm or link 7 piece is exempt from the torso", not any(("forearm" in i or "link7" in i) for i in TORSO_TOUCHING_IDS))

    print("margins and settings (config)")
    check("margins: hand-hand 1 cm, robot-robot 3 cm, arm-world 1 cm",
          (cfg.MARGINS["hand_hand_m"], cfg.MARGINS["robot_robot_m"], cfg.MARGINS["arm_world_m"]) == (0.01, 0.03, 0.01))
    check("push: starts 1 cm and stops 2 cm beyond the margin", (cfg.PUSH_START_BEYOND_M, cfg.PUSH_STOP_BEYOND_M) == (0.01, 0.02))


def main():
    table()
    held_and_broad()
    catalog_and_config()
    print("PASS" if not FAILS else f"FAIL: {len(FAILS)} check(s): {FAILS}")
    return 0 if not FAILS else 1


if __name__ == "__main__":
    sys.exit(main())
