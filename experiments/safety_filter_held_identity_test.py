"""Safety filter Phase S, S1 and S2 (SF-112, SF-113): held-object identity. No physics: the held tracker on a synthetic hand and scene.

S1 (SF-113, unidentified): nothing names the object and the pads match nothing -> the generic box (square section of the finger gap, from the
work surface under the pads up to half the table-to-gripper height above the gripper). The box is exempt from the surface it rests on and
checked against everything else. The detector is asked (display label "identifying object" while it runs); a name swaps the box for that
object's real shape (logged); no name keeps the box. The old finger-gap capsule is gone. The episode annotation lists the events.
S2 (SF-112, pad match): only objects overlapping the space between the pads are candidates; the nearest to the palm, measured forward along
the fingers, wins; an object behind the hand never counts; a width mismatch is a warning and never rejects a match.

    python experiments/safety_filter_held_identity_test.py      (exit code 0 when every check passes)
"""

import dataclasses
import os
import sys
import warnings

import numpy as np

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, REPO)

from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.object_detector import SimObjectDetector  # noqa: E402
from src.safety_filter.shapes import PlacedShape  # noqa: E402
from src.safety_filter.touch import Exempt, HeldTracker, Rule, as_placed, pair_rule, shape_of  # noqa: E402
from src.safety_filter.world import WorldObject, WorldView, catalog_object  # noqa: E402

FAILS = []
PALM = np.array([0.30, 0.0, 1.10])        # the palm centre (m); the fingers point along +x from it
EYE = np.eye(3)
TOP = 0.96                                 # the bench top's surface (m)


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    if not ok:
        FAILS.append(name)


class Src:
    def __init__(self, objs):
        self._objs = list(objs)

    def objects(self):
        return list(self._objs)


def bench():
    return WorldObject(id="bench_top", shape="box", size=(0.6, 0.4, 0.06), centre=(0.5, 0.0, TOP - 0.06), rot=tuple(EYE.ravel()),
                       fixture=True, work_surface=True)


def hand(side, gap, x_pads=0.40):
    """Placed shapes of one hand: a palm capsule at PALM, two finger capsules at the pads (x_pads, y = +-gap/2, z = PALM z), radius 5 mm."""
    palm = shape_of(f"{side}_palm", "capsule", (0.02, 0.02), tuple(PALM), EYE, side, "palm", body=f"openarm_{side}_link7")
    f1 = shape_of(f"{side}_finger1", "capsule", (0.005, 0.01), (x_pads, +gap / 2, PALM[2]), EYE, side, "finger", body=f"openarm_{side}_link7")
    f2 = shape_of(f"{side}_finger2", "capsule", (0.005, 0.01), (x_pads, -gap / 2, PALM[2]), EYE, side, "finger", body=f"openarm_{side}_link7")
    return [palm, f1, f2]


def beaker(x, y=0.0, id_="beaker"):
    return dataclasses.replace(catalog_object("beaker", np.zeros(3)), centre=(x, y, PALM[2]), id=id_)


class Stub:
    """A detector stand-in (the interface): records its calls and the display label seen during the call."""

    def __init__(self, answer):
        self.answer, self.calls, self.label_seen = answer, [], None
        self.tracker = None

    def identify(self, arm, finger_state, world):
        self.calls.append((arm, float(finger_state.gap)))
        if self.tracker is not None:
            self.label_seen = self.tracker.display_label(arm)
        return self.answer


def run_update(world, placed, name=None, detector=None):
    tr = HeldTracker(world, detector=detector)
    if detector is not None and hasattr(detector, "tracker"):
        detector.tracker = tr
    ev = tr.update("left", True, placed, PALM, EYE, name=name)
    return tr, ev


def s1_checks():
    gap = 0.05
    placed = hand("left", gap)
    world = WorldView(Src([bench()]))
    stub = Stub(None)
    tr, ev = run_update(world, placed, None, stub)
    check("S1 (a) no name and no pad match: the object is held as 'unidentified'", ev == "attach" and tr.identity_of("left") == "unidentified",
          f"event {ev}, identity {tr.identity_of('left')}")
    hp = tr.placed("left", PALM, EYE)
    d_top = PALM[2] + (PALM[2] - TOP) / 2          # the top: half the table-to-gripper height above the gripper
    ok_box = hp is not None and hp.kind == "box" and np.allclose(hp.half, [gap / 2, gap / 2, (d_top - TOP) / 2], atol=1e-9) \
        and np.allclose(hp.centre, [PALM[0] + 0.10, 0.0, (TOP + d_top) / 2], atol=1e-9) and hp.support == "bench_top"
    check("S1 (a) the generic box records the bench top as its support", hp is not None and hp.support == "bench_top", f"support {None if hp is None else hp.support!r}")
    check("S1 (a) the generic box: finger gap square section, bottom on the bench top, top half the table-to-gripper height above the gripper",
          ok_box, f"half {None if hp is None else np.round(hp.half, 4)}, centre {None if hp is None else np.round(hp.centre, 4)}, top {d_top:.3f} m")
    check("S1 (a) the detector was asked once, with the finger gap", stub.calls == [("left", gap)], f"calls {stub.calls}")
    check("S1 (a) no finger-gap capsule is held (the box replaces it)", hp is not None and hp.kind != "capsule")
    check("S1 (f) the display label reads 'identifying object' during the identification, and is clear after",
          stub.label_seen == "identifying object" and tr.display_label("left") is None, f"during {stub.label_seen!r}, after {tr.display_label('left')!r}")
    evs = [e["event"] for e in tr.events]
    unid = [e for e in tr.events if e["event"] == "unidentified"]
    check("S1 (c) the episode record has the 'unidentified' event with its support surface", evs == ["unidentified", "attach"] and unid and unid[0]["support"] == "bench_top",
          f"events {evs}")

    # S1 (b): exemption from the support surface; checked against everything else
    box = hp
    pb = as_placed(bench())
    check("S1 (b) the generic box is EXEMPT from the bench top it sits on", isinstance(pair_rule(box, pb), Exempt))
    leg = WorldObject(id="leg_front", shape="box", size=(0.03, 0.03, 0.4), centre=(0.1, 0.3, 0.4), rot=tuple(EYE.ravel()), fixture=True)
    check("S1 (b) the generic box is checked against a bench leg / wall (other fixtures)", isinstance(pair_rule(box, as_placed(leg)), Rule))
    hot = catalog_object("hotplate", np.array([0.6, 0.0, TOP]))
    check("S1 (b) the generic box is checked against the hotplate (allowed-contact surface, held-surface rule)",
          isinstance(pair_rule(box, as_placed(hot)), Rule) and pair_rule(box, as_placed(hot)).kind == "held-surface")
    other = beaker(1.0, id_="beaker_far")
    check("S1 (b) the generic box is checked against another object", isinstance(pair_rule(box, as_placed(other)), Rule))
    rpalm = shape_of("right_palm", "capsule", (0.02, 0.02), (0.3, -0.4, 1.1), EYE, "right", "palm", body="openarm_right_link7")
    check("S1 (b) the generic box is checked against the other hand (hand-hand)", isinstance(pair_rule(box, rpalm), Rule))
    torso = shape_of("torso_probe", "box", (0.1, 0.1, 0.1), (0.0, 0.0, 0.6), EYE, "common", "torso", body="torso")
    check("S1 (b) the generic box is checked against the body (torso)", isinstance(pair_rule(box, torso), Rule))

    # S1 (i): release of an unidentified box: an ordinary world object afterwards
    tr.update("left", False, placed, PALM, EYE)
    rel = [e for e in tr.events if e["event"] == "release"]
    back = [o for o in world.objects() if o.id.startswith("unidentified_left")]
    check("S1 (i) released: the generic box is an ordinary, graspable world object; the release is logged", len(rel) == 1 and len(back) == 1 and back[0].graspable,
          f"release events {len(rel)}, world objects {[o.id for o in back]}")

    # S1 (d): the detector names the object: the box is swapped for its real shape; the swap is logged
    stub2 = Stub("beaker")
    world2 = WorldView(Src([bench(), beaker(0.455, 0.0)]))      # 1.5 cm off the pads: outside the pad match, inside the detector radius
    tr2, ev2 = run_update(world2, hand("left", gap), None, stub2)
    hp2 = tr2.placed("left", PALM, EYE)
    sw = [e for e in tr2.events if e["event"] == "swap"]
    check("S1 (d) the detector names an object: the generic box is swapped for its real shape (catalog cylinder), logged",
          ev2 == "attach" and tr2.identity_of("left") == "detector" and hp2 is not None and hp2.kind == "cylinder" and np.isclose(hp2.r, 0.035)
          and len(sw) == 1 and sw[0]["to"] == "beaker", f"identity {tr2.identity_of('left')}, shape {None if hp2 is None else hp2.kind}, swap {sw}")
    check("S1 (d) the named object leaves the world list while held", world2.find("beaker") is None)

    # S1 (e): the sim stand-in itself (SimObjectDetector) is what the config selects, and it returns None here: the box stays
    tr3 = HeldTracker(WorldView(Src([bench()])))
    check("S1 (e) the default detector is the configured implementation", isinstance(tr3.detector, SimObjectDetector))
    tr3.update("left", True, hand("left", gap), PALM, EYE, name=None)
    check("S1 (e) the sim stand-in finds nothing near the pads (no object): the box stays", tr3.identity_of("left") == "unidentified")

    # S1 (h): a name from the task plan is the primary source, even when the pads match another object
    tr4, _ = run_update(WorldView(Src([bench(), beaker(PALM[0] + 0.10, 0.0, "beaker")])), hand("left", gap), "beaker", None)
    check("S1 (h) the policy command's name is the identity (command), not the pad match", tr4.identity_of("left") == "command")


def s2_checks():
    # two graspable objects overlap the space between the pads: the one nearer the palm wins (the old centre-to-pad rule would take the far one)
    gap = 0.05
    near = beaker(0.365, 0.0, "beaker_near")
    far = beaker(0.43, 0.0, "beaker_far")
    world = WorldView(Src([far, near]))
    tr, ev = run_update(world, hand("left", gap), None, Stub(None))
    check("S2 (a) two objects overlap the pads: the one nearer the palm (forward along the fingers) wins", tr.identity_of("left") == "pads"
          and tr.held["left"]["obj"].id == "beaker_near", f"held {tr.held['left']['obj'].id if 'left' in tr.held else None}")

    # behind the hand: an object that overlaps the pads but whose centre is behind the palm never counts
    behind = WorldObject(id="behind_box", shape="box", size=(0.20, 0.10, 0.10), centre=(0.20, 0.0, PALM[2]), rot=tuple(EYE.ravel()), graspable=True)
    tr2, _ = run_update(WorldView(Src([behind])), hand("left", gap), None, Stub(None))
    check("S2 (b) an object behind the hand never counts: no match, so unidentified", tr2.identity_of("left") == "unidentified",
          f"identity {tr2.identity_of('left')}")
    ahead = dataclasses.replace(behind, id="ahead_box", centre=(0.50, 0.0, PALM[2]))
    tr3, _ = run_update(WorldView(Src([ahead])), hand("left", gap), None, Stub(None))
    check("S2 (b) the same box ahead of the palm does match (the rule is the direction, not the object)", tr3.identity_of("left") == "pads")

    # width mismatch: a warning, the match stands
    gap_w = 0.03
    wide = beaker(0.40, 0.0, "beaker")                          # 7 cm across, fingers 3 cm: 4 cm off
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        tr4, _ = run_update(WorldView(Src([wide])), hand("left", gap_w), None, Stub(None))
    warned = any("across the pads" in str(w.message) for w in caught) or any("across the pads" in m for m in tr4.warnings)
    check("S2 (c) a width mismatch is logged as a warning and does not reject the match", tr4.identity_of("left") == "pads" and warned,
          f"identity {tr4.identity_of('left')}, warning logged {warned}")
    tr5, _ = run_update(WorldView(Src([beaker(0.40, 0.0, "beaker")])), hand("left", 0.07), None, Stub(None))
    check("S2 (c) a width that agrees with the finger gap gives no warning", not any("across the pads" in m for m in tr5.warnings))


def main():
    s1_checks()
    s2_checks()
    print("FAILED: " + ", ".join(FAILS) if FAILS else "ALL PASS")
    return 0 if not FAILS else 1


if __name__ == "__main__":
    sys.exit(main())
