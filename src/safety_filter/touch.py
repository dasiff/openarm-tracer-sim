"""Touch rules, held objects and the broad phase (M288, M289, M291, SF-3, SF-4). Stage 1: shared layer only.

No resolver here (Stage 2). This module answers: which pairs are checked, with which margin, what gap
they have, which world objects are near an arm, and which object a hand holds.

Pair rules (first match wins):
  world      an arm part vs a world object: checked with arm_world_m; fingers are exempt (M289), except against a never-touch object (SF-65: nothing exempt, margin NEVER_TOUCH_MARGIN_M).
             A held object (Part 2) is checked against the world: an allowed-contact surface (hotplate): zero margin and the surface's approach ceiling; every other world object: held_world_margin_m.
  same arm   any two shapes of one arm (including the arm's held object against its own hand, SF-4): exempt.
  torso      only the parts in TORSO_EXEMPT_PARTS (real geometry within 1 mm of the torso at home, SF-19) are exempt;
             every other arm part and held object is checked against the torso.
  body-body  two robot body shapes: exempt (rigidly connected robot body); both shoulder mounts: exempt (fixed to the torso).
  hand-hand  palm, fingers or held object of one arm vs the same kinds of the other arm: hand_hand_m.
  never-close a robot-vs-robot pair on the list in robot_shapes.json (SF-23) is exempt (checked after the rules above).
  robot-robot every other pair between the two arms, or an arm part vs another robot body part: robot_robot_m.
  Robot body vs world objects is not in the touch rules (no margin set): exempt, and flagged in the report.
"""

import warnings
from dataclasses import dataclass, replace

import numpy as np

from src.safety_filter import config as cfg
from src.safety_filter.geometry import (capsule_box, capsule_box_batch, capsule_capsule, capsule_capsule_batch, cylinder_box, cylinder_capsule)
from src.safety_filter.shapes import PlacedShape, never_close_pairs
from src.safety_filter.config import OBJECT_CATALOG
from src.safety_filter.object_detector import FingerState, make_object_detector, pad_space
from src.safety_filter.world import WorldObject, catalog_object

HAND_PARTS = ("palm", "finger", "held")
SIDES = ("left", "right")


@dataclass(frozen=True)
class Rule:
    kind: str          # "world", "hand-hand", "robot-robot", "arm-world"
    margin: float      # m
    activation: float  # m: a pair becomes active this far inside its margin distance (M288: 1 cm beyond)
    release: float     # m: a pair is released 2 cm beyond it (M288)
    work_surface: bool = False  # Phase D fix 1: the other shape is a work surface: the push-stop is PUSH_STOP_BEYOND_M_WORK beyond the margin
    surface: str = ""  # Part 2: for a held object against an allowed-contact surface, the surface's name (object id); its approach ceiling applies


@dataclass(frozen=True)
class Exempt:
    reason: str


def wake_dist():
    """Part 2: worst_stop_dist_m, the derived stopping distance from the hand speed cap (config.derive). Every activation cutoff and
    release (sleep wake) distance is at least this, so a pair within the stopping distance is awake and checked."""
    return cfg.derived()["worst_stop_dist_m"]


def _rule(kind, margin, surface="", work_surface=False):
    w = wake_dist()
    beyond = cfg.PUSH_STOP_BEYOND_M_WORK if work_surface else cfg.PUSH_STOP_BEYOND_M     # Phase D fix 1: a work surface has the tighter push zone
    return Rule(kind=kind, margin=margin, activation=max(margin + cfg.PUSH_START_BEYOND_M, w),
                release=max(margin + beyond, w), surface=surface, work_surface=work_surface)


def pair_rule(a: PlacedShape, b: PlacedShape, use_never_close=True):
    """The touch rule of a pair of placed shapes: a Rule (checked) or an Exempt (not checked, with the reason).
    A robot-vs-robot pair on the never-close list (SF-23) is exempt: its real geometry cannot come within margin + 1 cm."""
    r = _pair_rule(a, b)
    if use_never_close and isinstance(r, Rule) and frozenset((a.id, b.id)) in never_close_pairs():
        return Exempt("never-close pair: swept over the full joint limits on the real geometry (SF-23)")
    return r


def _pair_rule(a: PlacedShape, b: PlacedShape):
    if a.arm == "world" and b.arm == "world":
        return Exempt("world against world is not a robot pair")
    if a.arm == "world" or b.arm == "world":
        robot = b if a.arm == "world" else a
        if robot.support and (a if a.arm == "world" else b).id == robot.support:     # S1 (SF-113): an unidentified held box is exempt from the surface it rests on
            return Exempt("the unidentified held box rests on its support surface (SF-113)")
        if (a if a.arm == "world" else b).never_touch:          # SF-65: no finger or held-object exemption, own margin
            if robot.arm == "common":
                return Exempt("robot body against the world is not in the touch rules")
            return _rule("arm-world", cfg.NEVER_TOUCH_MARGIN_M)
        if robot.part == "finger":
            return Exempt("fingers are never checked against the world (M289)")
        world = a if a.arm == "world" else b
        if robot.part == "held":                                # Part 2: held objects are checked against the world
            if world.allowed_contact:
                return _rule("held-surface", cfg.HELD_SURFACE_MARGIN_M, surface=world.id)   # zero margin, approach ceiling speed
            return _rule("held-world", cfg.MARGINS["held_world_margin_m"], work_surface=world.work_surface)               # one margin, full stop at its midpoint
        if robot.part == "palm" and world.allowed_contact:  # Phase C step 2 (David): the palm may approach an allowed-contact surface at its approach ceiling, one margin off it
            return _rule("palm-surface", cfg.MARGINS["held_world_margin_m"], surface=world.id)
        if robot.arm == "common":
            return Exempt("robot body against the world is not in the touch rules")
        return _rule("arm-world", cfg.MARGINS["arm_world_m"], work_surface=world.work_surface)
    if a.part == "shoulder_mount" and b.part == "shoulder_mount":
        return Exempt("both shoulder mounts are fixed to the torso: they never move relative to each other")
    if a.arm == b.arm and a.arm in SIDES:
        return Exempt("same arm (SF-4 and SF-10 for the held object; no pair check inside one arm)")
    if "common" in (a.arm, b.arm):
        robot = b if a.arm == "common" else a
        body = a if a.arm == "common" else b
        if body.part == "torso" and robot.part in cfg.TORSO_EXEMPT_PARTS:
            return Exempt("touches the torso by construction (real geometry within 1 mm of the torso at home, SF-19)")
        if robot.arm == "common":
            return Exempt("robot body against itself")
        return _rule("robot-robot", cfg.MARGINS["robot_robot_m"])
    a_hand, b_hand = a.part in HAND_PARTS, b.part in HAND_PARTS
    if a_hand and b_hand:
        return _rule("hand-hand", cfg.MARGINS["hand_hand_m"])
    return _rule("robot-robot", cfg.MARGINS["robot_robot_m"])


def shape_of(obj_id, shape, size, centre, rot, arm, part, body="world", never_touch=False, allowed_contact=False, work_surface=False, support=""):
    """A placed shape from a world-style description: a box (half sizes) or a capsule (radius, half length along
    the rotation's z column). Both keep their centre."""
    centre, rot = np.asarray(centre, float), np.asarray(rot, float)
    if shape == "cylinder":                             # Phase D fix 4: a flat-ended solid; half = (radius, radius, half height), the box that holds it
        r, hh = size
        return PlacedShape(id=obj_id, arm=arm, part=part, kind="cylinder", body=body, centre=centre, rot=rot, r=float(r),
                           half=np.array([float(r), float(r), float(hh)]), never_touch=never_touch, allowed_contact=allowed_contact,
                           work_surface=work_surface, support=support)
    if shape == "capsule":
        r, h = size
        ax = rot[:, 2]
        return PlacedShape(id=obj_id, arm=arm, part=part, kind="capsule", body=body,
                           a=centre - h * ax, b=centre + h * ax, r=float(r), centre=centre, rot=rot, never_touch=never_touch,
                           allowed_contact=allowed_contact, work_surface=work_surface, support=support)
    return PlacedShape(id=obj_id, arm=arm, part=part, kind="box", body=body, centre=centre, rot=rot,
                       half=np.asarray(size, float), never_touch=never_touch, allowed_contact=allowed_contact, work_surface=work_surface, support=support)


def as_placed(obj: WorldObject) -> PlacedShape:
    """A world object as a placed shape (capsule or box)."""
    return shape_of(obj.id, obj.shape, obj.size, obj.centre, obj.rot_matrix(), "world", "world", never_touch=obj.never_touch,
                    allowed_contact=obj.allowed_contact, work_surface=obj.work_surface)


def _cylinder_gap(a: PlacedShape, b: PlacedShape):
    """Phase D fix 4: a flat-ended cylinder against a capsule or a box: (gap, point on a, point on b, push: moves a away from b)."""
    if a.kind == "cylinder" and b.kind == "cylinder":
        raise ValueError("cylinder against cylinder is not a checked pair (one cylinder object in the world)")
    cyl, oth = (a, b) if a.kind == "cylinder" else (b, a)
    if oth.kind == "capsule":
        c = cylinder_capsule(cyl.centre, cyl.rot, cyl.r, cyl.half[2], oth.a, oth.b, oth.r)     # push: moves the capsule away
        if cyl is a:
            return c.gap, c.point_a, c.point_b, -c.push
        return c.gap, c.point_b, c.point_a, c.push
    c = cylinder_box(cyl.centre, cyl.rot, cyl.r, cyl.half[2], oth.centre, oth.rot, oth.half)  # push: moves the box away
    if cyl is a:
        return c.gap, c.point_a, c.point_b, -c.push
    return c.gap, c.point_b, c.point_a, c.push


def gap_between(a: PlacedShape, b: PlacedShape):
    """(gap, closest point on a, closest point on b, push direction from b to a), for a capsule-capsule, capsule-box or cylinder pair."""
    if a.kind == "cylinder" or b.kind == "cylinder":
        return _cylinder_gap(a, b)
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


def _aabb_arrays(shapes):
    """(lo, hi) arrays (N,3) of the shapes' axis-aligned boxes (capsules and boxes, batched by kind)."""
    n = len(shapes)
    los = np.empty((n, 3))
    his = np.empty((n, 3))
    caps = [k for k, s in enumerate(shapes) if s.kind == "capsule"]
    boxes = [k for k, s in enumerate(shapes) if s.kind == "box"]
    for k, s in enumerate(shapes):                  # Phase D fix 4: a cylinder's own axis-aligned box
        if s.kind == "cylinder":
            los[k], his[k] = s.aabb()
    if caps:
        A = np.array([shapes[k].a for k in caps])
        B = np.array([shapes[k].b for k in caps])
        r = np.array([shapes[k].r for k in caps])[:, None]
        los[caps] = np.minimum(A, B) - r
        his[caps] = np.maximum(A, B) + r
    if boxes:
        C = np.array([shapes[k].centre for k in boxes])
        R = np.array([shapes[k].rot for k in boxes])
        H = np.array([shapes[k].half for k in boxes])
        ext = np.abs(R) @ H[:, :, None]
        ext = ext[:, :, 0]
        los[boxes] = C - ext
        his[boxes] = C + ext
    return los, his


def evaluate_pairs(robot_shapes, world_objects):
    """Gaps of the checked pairs between the robot's placed shapes and the world objects (batched).

    Stage 1 order: all pairs are first cut by their boxes (a pair whose boxes are further apart than the largest
    activation distance cannot be within its margin); the surviving pairs are classified by the touch rules, cut
    again by their own activation distance, and their gaps computed in two batches (capsule-capsule and capsule-box).
    Exempt pairs and box-box pairs are left out.
    Returns a list of PairReport.
    """
    items = list(robot_shapes) + [as_placed(o) for o in world_objects]
    n = len(items)
    if n < 2:
        return []
    lo, hi = _aabb_arrays(items)
    max_act = max(max(max(cfg.MARGINS.values()), cfg.NEVER_TOUCH_MARGIN_M if any(o.never_touch for o in world_objects) else 0.0) + cfg.PUSH_START_BEYOND_M, wake_dist())
    I, J = np.triu_indices(n, k=1)
    sep = np.maximum(lo[I] - hi[J], lo[J] - hi[I]).max(axis=1)
    keep = sep <= max_act
    cand = []
    for k in np.flatnonzero(keep):
        a, b = items[I[k]], items[J[k]]
        if a.kind == "box" and b.kind == "box":
            continue
        rule = pair_rule(a, b)
        if isinstance(rule, Exempt):
            continue
        if sep[k] > rule.activation:
            continue
        cand.append((a, b, rule))
    cc = [(a, b, r) for a, b, r in cand if a.kind == "capsule" and b.kind == "capsule"]
    cb = []
    for a, b, r in cand:
        if a.kind == "capsule" and b.kind == "box":
            cb.append((a, b, r, 1.0))
        elif a.kind == "box" and b.kind == "capsule":
            cb.append((b, a, r, -1.0))      # computed as (capsule, box); the push is flipped back below
    out = []
    if cc:
        A1 = np.array([a.a for a, _, _ in cc]); B1 = np.array([a.b for a, _, _ in cc]); r1 = np.array([a.r for a, _, _ in cc])
        A2 = np.array([b.a for _, b, _ in cc]); B2 = np.array([b.b for _, b, _ in cc]); r2 = np.array([b.r for _, b, _ in cc])
        gaps = capsule_capsule_batch(A1, B1, r1, A2, B2, r2, gap_only=True)
        for (a, b, r), g in zip(cc, gaps):
            out.append(PairReport(a=a.id, b=b.id, rule=r, gap=float(g), in_margin=bool(g < r.margin)))
    if cb:
        A = np.array([a.a for a, _, _, _ in cb]); B = np.array([a.b for a, _, _, _ in cb]); rr = np.array([a.r for a, _, _, _ in cb])
        C = np.array([b.centre for _, b, _, _ in cb]); R = np.array([b.rot for _, b, _, _ in cb]); H = np.array([b.half for _, b, _, _ in cb])
        gaps = capsule_box_batch(A, B, rr, C, R, H, gap_only=True)
        for (a, b, r, _), g in zip(cb, gaps):
            out.append(PairReport(a=a.id, b=b.id, rule=r, gap=float(g), in_margin=bool(g < r.margin)))
    for a, b, r in cand:                                 # Phase D fix 4: a cylinder pair is not batched (its own exact distance)
        if a.kind == "cylinder" or b.kind == "cylinder":
            g = float(gap_between(a, b)[0])
            out.append(PairReport(a=a.id, b=b.id, rule=r, gap=g, in_margin=bool(g < r.margin)))
    return out


def broad_phase(robot_shapes, desired_shapes, world_objects, world_margin, release, keep=()):
    """Candidate world objects for this tick (M291).

    The arm's box at the current pose and at the desired pose, padded by the largest capsule radius plus the
    activation distance plus the world margin (and at least the release distance). Objects whose box overlaps
    it are candidates; ids in `keep` (pairs being pushed) are always candidates until released (not in keep)."""
    pad = max(s.r for s in robot_shapes if s.kind == "capsule") if any(s.kind == "capsule" for s in robot_shapes) else 0.0
    pad = max(pad + cfg.PUSH_START_BEYOND_M + world_margin, release)
    arm_lo, arm_hi = _aabb_arrays(list(robot_shapes) + list(desired_shapes))
    lo = arm_lo.min(axis=0) - pad
    hi = arm_hi.max(axis=0) + pad
    if not world_objects:
        return []
    olo, ohi = _aabb_arrays([as_placed(o) for o in world_objects])
    hit = np.all(ohi >= lo, axis=1) & np.all(olo <= hi, axis=1)
    out = []
    for k, o in enumerate(world_objects):
        if o.id in keep or hit[k]:
            out.append(o)
    return out


def _width_across(shape, u):
    """Phase S, S2 (SF-112): the extent of a placed world shape along the unit vector u (m): the width across the finger pads."""
    if shape.kind == "capsule":
        return 2.0 * float(shape.r) + abs(float(np.dot(np.asarray(shape.b) - np.asarray(shape.a), u)))
    if shape.kind == "cylinder":
        c = abs(float(np.dot(np.asarray(shape.rot)[:, 2], u)))
        return 2.0 * (float(shape.r) * float(np.sqrt(max(0.0, 1.0 - c * c))) + float(shape.half[2]) * c)
    return 2.0 * float(np.sum(np.abs(np.asarray(shape.rot).T @ u) * np.asarray(shape.half, float)))


class HeldTracker:
    """Held objects (M289, SF-4, Part 2; Phase S S1, S2): attaches the held object to the hand, places it each tick, releases it.

    The grasp signal is the holding verdict (the squeeze check). The identity chain (SF-113), first answer wins:
      1. the policy command's name ("you are holding object X": the `name` argument, looked up in OBJECT_CATALOG);
      2. the pad match (SF-112): the graspable world object whose shape overlaps the space between the finger pads, forward of the palm,
         the one nearest the palm (measured forward along the fingers); an object behind the hand never counts;
      3. "unidentified" (SF-113): a generic box (square section of the finger gap, from the work surface under the pads up to half the
         table-to-gripper height above the gripper). The ObjectDetector (src/safety_filter/object_detector.py) is asked; a name it returns swaps
         the box for that object's real shape.
    A name that disagrees with the pad match is a warning; a width mismatch is a warning and never rejects a match (SF-112).
    self.events collects the episode annotation: attach (with its identity), unidentified, swap and release events."""

    def __init__(self, world_view, detector=None):
        self.world = world_view
        self.held = {}                  # side -> dict (the attached object, its hand offset, its identity and support surface)
        self.identity = {}              # side -> identity of the object held now: "command", "pads", "detector" or "unidentified" (None: nothing held)
        self.identity_source = None     # identity of the last attached object (kept for the older callers)
        self.labels = {}                # side -> display label while an identification runs ("identifying object"); none otherwise
        self.events = []                # the episode annotation: dicts with "side", "event" and what it says
        self.warnings = []
        self.detector = detector if detector is not None else make_object_detector()
        self._n_unidentified = 0

    def _pad(self, placed, side):
        """The finger pads of one arm: (midpoint, finger gap, pad a, pad b, finger radius)."""
        fingers = [s for s in placed if s.part == "finger" and s.arm == side]
        if len(fingers) != 2:
            raise ValueError(f"{side}: expected two finger capsules")
        centres = [(s.a + s.b) / 2 for s in fingers]
        a, b = centres[0], centres[1]
        return (a + b) / 2, float(np.linalg.norm(a - b)), a, b, float(fingers[0].r)

    def _palm_centre(self, placed, side, hand_pos):
        """The palm: the mean centre of this arm's palm capsules (the hand position if there are none)."""
        centres = [(s.a + s.b) / 2 for s in placed if s.part == "palm" and s.arm == side]
        return np.mean(centres, axis=0) if centres else np.asarray(hand_pos, float)

    def _warn(self, msg):
        warnings.warn(msg)
        self.warnings.append(msg)

    def _pad_match(self, placed, side, hand_pos, mid, gap, a, b, radius):
        """SF-112: the graspable object the pads hold: overlapping the space between the pads (within HELD_MATCH_EXTRA_M), forward of the palm
        (its centre ahead of the palm along the fingers), the nearest to the palm. A width mismatch is a warning only."""
        pads = pad_space(side, FingerState(a, b, radius))
        palm = self._palm_centre(placed, side, hand_pos)
        d = np.asarray(mid, float) - palm
        nd = float(np.linalg.norm(d))
        fwd = d / nd if nd > 1e-9 else None           # the direction the fingers point, from the palm to the pads
        best, best_s = None, None
        for o in self.world.objects():
            if o.fixture or not o.graspable:
                continue
            if float(gap_between(pads, as_placed(o))[0]) > cfg.HELD_MATCH_EXTRA_M:
                continue                              # the object's shape must overlap the space between the pads (within the tolerance)
            s = float(np.dot(np.asarray(o.centre, float) - palm, fwd)) if fwd is not None else 0.0
            if s < 0.0:
                continue                              # behind the hand: never counts
            if best_s is None or s < best_s:
                best, best_s = o, s
        if best is not None and gap > 1e-9:
            u = (np.asarray(b, float) - np.asarray(a, float)) / gap
            width = _width_across(as_placed(best), u)
            if abs(width - gap) > cfg.HELD_WIDTH_TOL_M:
                self._warn(f"{side}: the pad match '{best.id}' is {width * 100:.1f} cm across the pads, the fingers measure {gap * 100:.1f} cm (width check; the match stands)")
        return best

    def _support_under(self, mid):
        """The work surface the held object would sit on: the highest work-surface top below the pads whose footprint holds the pad midpoint
        (None if there is none; the caller then uses the floor)."""
        best, best_top = None, None
        for o in self.world.objects():
            if not o.work_surface:
                continue
            R = o.rot_matrix()
            local = R.T @ (np.asarray(mid, float) - np.asarray(o.centre, float))
            half = np.asarray(o.size, float)
            if abs(local[0]) > half[0] or abs(local[1]) > half[1]:
                continue
            top = float(o.centre[2]) + float(np.sum(np.abs(R[2, :]) * half))
            if top <= float(mid[2]) and (best_top is None or top > best_top):
                best, best_top = o, top
        return best, best_top

    def _attach(self, side, obj, identity, hand_pos, hand_rot, support="", in_world=True):
        """Holds obj in the hand: its pose is kept relative to the hand frame; it leaves the world list (when it is in it)."""
        off_pos = hand_rot.T @ (np.asarray(obj.centre) - hand_pos)
        off_rot = hand_rot.T @ obj.rot_matrix()
        self.held[side] = {"kind": "object", "obj": obj, "off_pos": off_pos, "off_rot": off_rot, "identity": identity, "support": support}
        self.identity[side] = identity
        self.identity_source = identity
        if in_world:
            self.world.remove(obj.id)
        self.events.append({"side": side, "event": "attach", "object": obj.id, "identity": identity})

    def _unidentified(self, side, mid, gap, a, b, radius, hand_pos, hand_rot):
        """SF-113: the generic box for a held object nothing names, then the detector's answer (a name swaps the box for its real shape)."""
        support, top_s = self._support_under(mid)
        z_low = top_s if top_s is not None else 0.0
        if top_s is None:
            self._warn(f"{side}: no work surface under the pads; the generic box stands on the floor (z = 0)")
        z_grip = float(mid[2])
        z_top = z_grip + max(z_grip - z_low, 0.0) / 2.0          # the top: half the table-to-gripper distance above the gripper
        if z_top - z_low < 1e-6:
            z_top = z_low + gap                                  # the gripper is at the table: a box of the gap's height, so it has a size
            self._warn(f"{side}: the gripper is at the surface; the generic box gets the finger gap as its height")
        self._n_unidentified += 1
        box_id = f"unidentified_{side}_{self._n_unidentified}"
        half = (gap / 2.0, gap / 2.0, (z_top - z_low) / 2.0)
        centre = (float(mid[0]), float(mid[1]), (z_low + z_top) / 2.0)
        box = WorldObject(id=box_id, shape="box", size=tuple(half), centre=centre, rot=tuple(np.eye(3).ravel()), graspable=False)
        support_id = support.id if support is not None else ""
        self.events.append({"side": side, "event": "unidentified", "object": box_id, "support": support_id, "box_half_m": [float(x) for x in half],
                            "box_centre_m": [float(x) for x in centre], "pad_gap_m": gap})
        self.labels[side] = "identifying object"
        try:
            name = self.detector.identify(side, FingerState(a, b, radius), self.world)
        finally:
            self.labels.pop(side, None)
        if name is not None:
            if name in OBJECT_CATALOG:
                found = self.world.find(name)                # the object's world shape if the filter has it, else the catalog's; centred at the pads
                real = replace(found if found is not None else catalog_object(name, np.zeros(3)), centre=tuple(mid), id=name)
                self.events.append({"side": side, "event": "swap", "from": box_id, "to": name, "shape": real.shape})
                self._attach(side, real, "detector", hand_pos, hand_rot)
                return
            self._warn(f"{side}: the detector named '{name}', which is not in the object catalog; the generic box stays")
        self._attach(side, box, "unidentified", hand_pos, hand_rot, support=support_id, in_world=False)

    def update(self, side, holding, placed, hand_pos, hand_rot, name=None):
        """holding: this tick's holding verdict for the side. name: the object the policy command says is held (or None).
        placed: the robot's placed shapes this tick. hand_pos, hand_rot: link 7 pose (world).
        Returns the event for this tick: None, 'attach' or 'release'."""
        want = bool(holding)
        if want and side not in self.held:
            mid, gap, a, b, radius = self._pad(placed, side)
            by_pad = self._pad_match(placed, side, hand_pos, mid, gap, a, b, radius)
            obj, identity = None, None
            if name:
                if name not in OBJECT_CATALOG:
                    self._warn(f"{side}: the command names '{name}', which is not in the object catalog; using the pad match")
                else:
                    obj = self.world.find(name)
                    if obj is None:                     # named but not in the world list: the catalog shape, centred at the pads
                        obj = replace(catalog_object(name, np.zeros(3)), centre=tuple(mid), id=name)
                    identity = "command"
                    if by_pad is not None and by_pad.id != name:
                        self._warn(f"{side}: the command says '{name}' is held but the pads match '{by_pad.id}'; using '{name}'")
                    elif by_pad is None:
                        self._warn(f"{side}: the command says '{name}' is held but no graspable object is at the pads (pad check failed)")
            if obj is None and by_pad is not None:
                obj, identity = by_pad, "pads"
            if obj is not None:
                self._attach(side, obj, identity, hand_pos, hand_rot)
                return "attach"
            self._unidentified(side, mid, gap, a, b, radius, hand_pos, hand_rot)     # SF-113: nothing names it and the pads match nothing
            return "attach"
        if not want and side in self.held:
            rec = self.held.pop(side)
            obj = rec["obj"]
            pos = hand_pos + hand_rot @ rec["off_pos"]
            rot = hand_rot @ rec["off_rot"]
            self.world.add(WorldObject(id=obj.id, shape=obj.shape, size=obj.size, centre=tuple(pos),
                                       rot=tuple(rot.ravel()), graspable=obj.graspable or rec.get("identity") == "unidentified",
                                       round=obj.round, allowed_contact=obj.allowed_contact, work_surface=obj.work_surface))
            self.identity.pop(side, None)
            self.events.append({"side": side, "event": "release", "object": obj.id, "identity": rec.get("identity")})
            return "release"
        return None

    def identity_of(self, side):
        """The identity of the object held by one side now ("command", "pads", "detector", "unidentified"), or None."""
        return self.identity.get(side)

    def display_label(self, side):
        """"identifying object" while the detector runs for this side (the sim stand-in answers in the same tick), otherwise None."""
        return self.labels.get(side)

    def placed(self, side, hand_pos, hand_rot):
        """The held shape of one arm this tick, placed from the hand frame (None if nothing is held: the gripper-only shapes are used)."""
        rec = self.held.get(side)
        if rec is None or rec.get("kind") != "object":
            return None
        obj = rec["obj"]
        return shape_of(f"{side}_held", obj.shape, obj.size, hand_pos + hand_rot @ rec["off_pos"],
                        hand_rot @ rec["off_rot"], side, "held", body=f"openarm_{side}_link7", support=rec.get("support", ""))
