"""Touch rules, held objects and the broad phase (M288, M289, M291, SF-3, SF-4). Stage 1: shared layer only.

No resolver here (Stage 2). This module answers: which pairs are checked, with which margin, what gap
they have, which world objects are near an arm, and which object a hand holds.

Pair rules (first match wins):
  world      an arm part vs a world object: checked with arm_world_m; fingers and held objects are exempt (M289).
  same arm   any two shapes of one arm (including the arm's held object against its own hand, SF-4): exempt.
  torso      an arm or held object vs the torso: exempt while EXEMPT_TORSO_PAIRS (the mounts touch it by construction).
  body-body  two robot body shapes: exempt (rigidly connected robot body); both shoulder mounts: exempt (fixed to the torso).
  hand-hand  palm, fingers or held object of one arm vs the same kinds of the other arm: hand_hand_m.
  robot-robot every other pair between the two arms, or an arm part vs another robot body part: robot_robot_m.
  Robot body vs world objects is not in the touch rules (no margin set): exempt, and flagged in the report.
"""

import warnings
from dataclasses import dataclass

import numpy as np

from src.safety_filter import config as cfg
from src.safety_filter.geometry import capsule_box, capsule_capsule
from src.safety_filter.shapes import PlacedShape
from src.safety_filter.world import WorldObject

HAND_PARTS = ("palm", "finger", "held")
SIDES = ("left", "right")


@dataclass(frozen=True)
class Rule:
    kind: str          # "world", "hand-hand", "robot-robot", "arm-world"
    margin: float      # m
    activation: float  # m: a pair becomes active this far inside its margin distance (M288: 1 cm beyond)
    release: float     # m: a pair is released 2 cm beyond it (M288)


@dataclass(frozen=True)
class Exempt:
    reason: str


def _rule(kind, margin):
    return Rule(kind=kind, margin=margin, activation=margin + cfg.PUSH_START_BEYOND_M,
                release=margin + cfg.PUSH_STOP_BEYOND_M)


def pair_rule(a: PlacedShape, b: PlacedShape):
    """The touch rule of a pair of placed shapes: a Rule (checked) or an Exempt (not checked, with the reason)."""
    if a.arm == "world" and b.arm == "world":
        return Exempt("world against world is not a robot pair")
    if a.arm == "world" or b.arm == "world":
        robot = b if a.arm == "world" else a
        if robot.part == "finger":
            return Exempt("fingers are never checked against the world (M289)")
        if robot.part == "held":
            return Exempt("held objects are exempt from the world (M289)")
        if robot.arm == "common":
            return Exempt("robot body against the world is not in the touch rules")
        return _rule("arm-world", cfg.MARGINS["arm_world_m"])
    if a.part == "shoulder_mount" and b.part == "shoulder_mount":
        return Exempt("both shoulder mounts are fixed to the torso: they never move relative to each other")
    if a.arm == b.arm and a.arm in SIDES:
        return Exempt("same arm (held object vs its own hand: SF-4)")
    if "common" in (a.arm, b.arm):
        robot = b if a.arm == "common" else a
        body = a if a.arm == "common" else b
        if body.part == "torso" and cfg.EXEMPT_TORSO_PAIRS:
            return Exempt("torso is the parent of both shoulder mounts")
        if robot.arm == "common":
            return Exempt("robot body against itself")
        return _rule("robot-robot", cfg.MARGINS["robot_robot_m"])
    a_hand, b_hand = a.part in HAND_PARTS, b.part in HAND_PARTS
    if a_hand and b_hand:
        return _rule("hand-hand", cfg.MARGINS["hand_hand_m"])
    return _rule("robot-robot", cfg.MARGINS["robot_robot_m"])


def as_placed(obj: WorldObject) -> PlacedShape:
    """A world object as a placed box (a cylinder is checked as its enclosing box: conservative, Stage 1)."""
    if obj.shape == "cylinder":
        r, h = obj.size
        half = np.array([r, r, h])
    else:
        half = np.asarray(obj.size, float)
    return PlacedShape(id=obj.id, arm="world", part="world", kind="box", body="world",
                       centre=np.asarray(obj.centre, float), rot=obj.rot_matrix(), half=half)


def gap_between(a: PlacedShape, b: PlacedShape):
    """(gap, closest point on a, closest point on b, push direction from b to a), for a capsule-capsule or capsule-box pair."""
    if a.kind == "capsule" and b.kind == "capsule":
        c = capsule_capsule(a.a, a.b, a.r, b.a, b.b, b.r)
    elif a.kind == "capsule" and b.kind == "box":
        c = capsule_box(a.a, a.b, a.r, b.centre, b.rot, b.half)
    elif a.kind == "box" and b.kind == "capsule":
        c0 = capsule_box(b.a, b.b, b.r, a.centre, a.rot, a.half)
        return c0.gap, c0.point_b, c0.point_a, -c0.push  # push from the capsule (b) to the box (a)
    else:
        raise ValueError("box against box is not a checked pair")
    return c.gap, c.point_a, c.point_b, c.push


@dataclass
class PairReport:
    a: str
    b: str
    rule: object          # Rule or Exempt
    gap: float            # m; negative when overlapping
    in_margin: bool       # gap below the margin


def _aabb_gap(a, b):
    """A lower bound of the distance between two placed shapes: the gap between their axis-aligned boxes."""
    la, ha = a.aabb()
    lb, hb = b.aabb()
    return float(np.max(np.maximum(la - hb, lb - ha)))


def evaluate_pairs(robot_shapes, world_objects):
    """Gaps of the checked pairs between the robot's placed shapes and the world objects.

    Pairs whose boxes are further apart than their activation distance are skipped first (the true gap is
    at least the box gap, so such a pair cannot be within its margin or active). Exempt pairs are left out.
    robot_shapes: list of PlacedShape (both arms, held objects, robot body). world_objects: list of WorldObject.
    Returns a list of PairReport.
    """
    items = list(robot_shapes) + [as_placed(o) for o in world_objects]
    out = []
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            a, b = items[i], items[j]
            if a.kind == "box" and b.kind == "box":
                continue
            rule = pair_rule(a, b)
            if isinstance(rule, Exempt):
                continue
            if _aabb_gap(a, b) > rule.activation:
                continue
            gap, _, _, _ = gap_between(a, b)
            out.append(PairReport(a=a.id, b=b.id, rule=rule, gap=gap, in_margin=gap < rule.margin))
    return out


def broad_phase(robot_shapes, desired_shapes, world_objects, world_margin, release, keep=()):
    """Candidate world objects for this tick (M291).

    The arm's box at the current pose and at the desired pose, padded by the largest capsule radius plus the
    activation distance plus the world margin (and at least the release distance). Objects whose box overlaps
    it are candidates; ids in `keep` (pairs being pushed) are always candidates until released (not in keep)."""
    pad = max(s.r for s in robot_shapes if s.kind == "capsule") if any(s.kind == "capsule" for s in robot_shapes) else 0.0
    pad = max(pad + cfg.PUSH_START_BEYOND_M + world_margin, release)
    los, his = [], []
    for s in list(robot_shapes) + list(desired_shapes):
        lo, hi = s.aabb()
        los.append(lo)
        his.append(hi)
    lo = np.min(los, axis=0) - pad
    hi = np.max(his, axis=0) + pad
    out = []
    for o in world_objects:
        if o.id in keep:
            out.append(o)
            continue
        ext = np.abs(o.rot_matrix()) @ np.asarray(o.size if o.shape == "box" else (o.size[0], o.size[0], o.size[1]), float)
        c = np.asarray(o.centre, float)
        if np.all(c + ext >= lo) and np.all(c - ext <= hi):
            out.append(o)
    return out


class HeldTracker:
    """Held objects (M289, SF-4): attaches the matched object to the hand, places it each tick, releases it.

    update() is called every tick with each arm's holding verdict. Attach happens when the verdict turns on,
    release when it turns off. Nothing else is decided here."""

    def __init__(self, world_view):
        self.world = world_view
        self.held = {}          # side -> dict (the attached object, or the fallback capsule)
        self.warnings = []

    def _pad(self, placed, side):
        centres = []
        for s in placed:
            if s.part == "finger" and s.arm == side:
                centres.append((s.a + s.b) / 2)
        if len(centres) != 2:
            raise ValueError(f"{side}: expected two finger capsules")
        return np.mean(centres, axis=0), float(np.linalg.norm(centres[0] - centres[1]))

    def update(self, side, holding, placed, hand_pos, hand_rot):
        """holding: this tick's verdict for the side. placed: the robot's placed shapes this tick.
        hand_pos, hand_rot: link 7 pose (world). Returns the event for this tick: None, 'attach', 'fallback' or 'release'."""
        if holding and side not in self.held:
            pad, gap = self._pad(placed, side)
            best, best_d = None, None
            for o in self.world.objects():
                if o.fixture or not o.graspable:
                    continue
                d = float(np.linalg.norm(np.asarray(o.centre) - pad))
                if d - o.bound_radius() <= cfg.HELD_MATCH_EXTRA_M and (best is None or d < best_d):
                    best, best_d = o, d
            if best is not None:
                off_pos = hand_rot.T @ (np.asarray(best.centre) - hand_pos)
                off_rot = hand_rot.T @ best.rot_matrix()
                self.held[side] = {"kind": "object", "obj": best, "off_pos": off_pos, "off_rot": off_rot}
                self.world.remove(best.id)
                return "attach"
            # fallback: a capsule at the pad, sized from the finger gap, default height (M289)
            msg = f"{side}: no graspable object at the finger pads; fallback capsule used"
            warnings.warn(msg)
            self.warnings.append(msg)
            self.held[side] = {"kind": "fallback", "pad_local": hand_rot.T @ (pad - hand_pos),
                               "radius": max(gap / 2.0, 0.005)}
            return "fallback"
        if not holding and side in self.held:
            rec = self.held.pop(side)
            if rec["kind"] == "object":
                obj = rec["obj"]
                pos = hand_pos + hand_rot @ rec["off_pos"]
                rot = hand_rot @ rec["off_rot"]
                self.world.add(WorldObject(id=obj.id, shape=obj.shape, size=obj.size, centre=tuple(pos),
                                           rot=tuple(rot.ravel()), graspable=obj.graspable, round=obj.round))
                return "release"
            return "release"
        return None

    def placed(self, side, hand_pos, hand_rot):
        """The held shape of one arm this tick, placed from the hand frame (None if nothing is held)."""
        rec = self.held.get(side)
        if rec is None:
            return None
        if rec["kind"] == "object":
            obj = rec["obj"]
            half = np.array([obj.size[0], obj.size[0], obj.size[1]]) if obj.shape == "cylinder" else np.asarray(obj.size, float)
            return PlacedShape(id=f"{side}_held", arm=side, part="held", kind="box", body=f"openarm_{side}_link7",
                               centre=hand_pos + hand_rot @ rec["off_pos"], rot=hand_rot @ rec["off_rot"], half=half)
        c = hand_pos + hand_rot @ rec["pad_local"]
        ax = hand_rot[:, 2]  # the hand's z axis (the gripper axis)
        h = cfg.HELD_FALLBACK_HEIGHT_M / 2.0
        return PlacedShape(id=f"{side}_held", arm=side, part="held", kind="capsule", body=f"openarm_{side}_link7",
                           a=c - h * ax, b=c + h * ax, r=rec["radius"])
