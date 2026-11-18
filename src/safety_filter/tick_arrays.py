"""Array form of one tick's shapes and pairs (Stage 5 speed; decision rows SF-69 to SF-75): how the resolver computes, not what it decides.

The object form (touch.evaluate_pairs, touch.gap_between on PlacedShape objects) stays the definition. This module gives the resolver
the same pairs, rules and gaps from arrays:

  PairTable    the checked pairs of one set of items (robot shapes, held shapes, world objects), built once per set with touch.pair_rule
               and kept between ticks: index arrays, margins, activation and release distances, rules and pair names.
  PoseArrays   every item's capsule ends or box, and its axis-aligned box, at one pose (the numbers of RobotShapes.place and
               touch.as_placed, without building one object per shape).
  box_separation   the broad phase of touch.evaluate_pairs for all checked pairs at once: the separation of the two axis-aligned boxes.
  exact_gap    one pair's gap by the scalar functions of geometry.py, with the arguments in the order touch.gap_between uses: the
               same number, bit for bit.
  approx_gaps  the gaps of many pairs in one batched call, as an interval [low, high] that is guaranteed to contain the exact gap.

The intervals let the resolver decide most threshold questions (is this pair's shortfall above the trigger, is its gap below the
release distance, did the path pass below the guard gap) without the exact gap. Whenever an interval does not decide a question, and
wherever a gap's VALUE is used (the shortfall that is pushed, the order of the pairs, the status record), the exact gap is computed
as before. So every decision and every number that leaves the resolver is the one the object form gives.

Why the intervals are safe:
  box separation <= gap      a capsule lies inside its axis-aligned box, a box inside its own; the separation of two axis-aligned boxes
                             (largest per-axis distance, negative when they overlap) is never larger than the distance of what is inside
                             them. Overlapping: a capsule pair's gap is segment distance minus the radii and the boxes are the segments'
                             boxes grown by the radii; a point inside a box is never deeper in the box than in the box around it.
  capsule against capsule    the batched function is the scalar function's formula; the interval is the value +- cfg.ARRAY_FORM['cc_eps_m'].
  capsule against box        both the scalar function (4001 samples along the segment) and the batched one (33 samples, then 14
                             golden-section steps) return the box distance at ONE point of the segment, so each is at least the true
                             minimum. The distance along the segment is convex (signed distance to a convex box, along a line), so the
                             4001-sample minimum is within segment length / 8000 of the true minimum, and the batched value within
                             segment length x 3.7e-5 (half of the last bracket, (2 / 32) x 0.618^14). cfg.ARRAY_FORM['cb_low_per_m'] and cfg.ARRAY_FORM['cb_high_per_m'] are those
                             two numbers with a fifth added.
  travel                     a point of an arm shape moves, between the last command and any pose q, by at most the sum over the arm's
                             joints of (distance of the point from the joint's anchor AT THE LAST COMMAND) x |joint change|: turn the
                             joints one at a time from the shoulder outwards; while joint i turns, the joints after it still have their
                             last-command values, so the point's distance from joint i's axis is the one at the last command. A capsule's
                             gap to anything changes by at most the larger move of its two ends.
"""

import numpy as np

from src.safety_filter import config as cfg
from src.safety_filter.geometry import capsule_box, capsule_box_batch, capsule_capsule, capsule_capsule_batch, cylinder_box, cylinder_capsule
from src.safety_filter.batch_jit import capsule_box_gap_batch, capsule_capsule_gap_batch, pair_gap_at, pair_separation_kernel    # Phase K fix 2 (SF-97) and Phase L fix 2 (SF-100): compiled
from src.safety_filter.touch import Exempt, pair_rule

SIDE_INDEX = {"left": 0, "right": 1}
KIND_CODE = {"capsule": 0, "box": 1, "cylinder": 2}   # Phase D fix 4: the kinds of the items in the pair table


class PairTable:
    """The checked pairs of an item list. items: PlacedShape-like objects in the order robot shapes, held shapes, world objects
    (only id, arm, part, kind, never_touch and the sizes are read). n_robot: how many of them are robot or held shapes."""

    def __init__(self, items, n_robot):
        n = len(items)
        self.n, self.n_robot = n, n_robot
        self.st_n_spec = 0                                  # set by Statics.table: how many items are the robot's own shapes (the rest of the robot items are held shapes)
        self.ids = [s.id for s in items]
        self.item_index = {s.id: k for k, s in enumerate(items)}
        self.item_side = np.array([SIDE_INDEX.get(s.arm, -1) for s in items], dtype=np.int64)
        self.is_capsule = np.array([s.kind == "capsule" for s in items], dtype=bool)
        self.radius = np.array([float(s.r) if s.kind in ("capsule", "cylinder") else 0.0 for s in items])   # Phase D fix 4: a cylinder's radius too
        self.kind_code = np.array([KIND_CODE[s.kind] for s in items], dtype=np.int64)
        self.is_cyl_item = self.kind_code == KIND_CODE["cylinder"]
        self.half = np.array([np.asarray(s.half, float) if s.kind != "capsule" else np.zeros(3) for s in items]).reshape(n, 3)
        I, J, ka, kb, rules, keys = [], [], [], [], [], []
        for i in range(n_robot):
            a = items[i]
            for j in range(i + 1, n):
                b = items[j]
                if a.kind == "box" and b.kind == "box":
                    continue
                rule = pair_rule(a, b)
                if isinstance(rule, Exempt):
                    continue
                key = tuple(sorted((a.id, b.id)))
                I.append(i); J.append(j); rules.append(rule); keys.append(key)
                first = i if key[0] == a.id else j                  # touch.gap_between gets the shapes in pair-name order
                ka.append(first); kb.append(i + j - first)
        self.I, self.J = np.array(I, dtype=np.int64), np.array(J, dtype=np.int64)
        self.ka, self.kb = np.array(ka, dtype=np.int64), np.array(kb, dtype=np.int64)
        self.rules, self.keys = rules, keys
        self.key_index = {k: p for p, k in enumerate(keys)}
        self.margin = np.array([r.margin for r in rules], dtype=float)
        self.act = np.array([r.activation for r in rules], dtype=float)
        self.release = np.array([r.release for r in rules], dtype=float)
        cap_i, cap_j = self.is_capsule[self.I] if len(I) else np.zeros(0, bool), self.is_capsule[self.J] if len(I) else np.zeros(0, bool)
        self.is_cc = cap_i & cap_j
        # Phase D fix 4: a pair with a cylinder is exact-only (no batched bound of its own): its lower bound is the boxes' separation, its upper bound the centre distance
        self.is_cy = (self.is_cyl_item[self.I] | self.is_cyl_item[self.J]) if len(I) else np.zeros(0, bool)
        self.cb_cap = np.where(cap_i, self.I, self.J)               # capsule and box item of a capsule-box pair (unused for capsule pairs)
        self.cb_box = np.where(cap_i, self.J, self.I)
        side_i, side_j = (self.item_side[self.I], self.item_side[self.J]) if len(I) else (np.zeros(0, int), np.zeros(0, int))
        self.uses_side = np.stack([(side_i == s) | (side_j == s) for s in (0, 1)]) if len(I) else np.zeros((2, 0), bool)

    def __len__(self):
        return len(self.keys)


class PoseArrays:
    """All items at one pose: capsule ends A, B and box centre C, rotation R (rows of non-matching kind are unused), and the
    axis-aligned boxes lo, hi. sep, glo and ghi are filled when first asked for."""
    __slots__ = ("A", "B", "C", "R", "lo", "hi", "sep", "glo", "ghi", "held", "objs", "travel", "anchors")

    def __init__(self, A, B, C, R, lo, hi, held):
        self.A, self.B, self.C, self.R, self.lo, self.hi, self.held = A, B, C, R, lo, hi, held
        self.sep = self.glo = self.ghi = self.travel = None
        self.objs = {}


def box_separation(tab, P):
    """Separation of the two axis-aligned boxes of every checked pair at the pose (negative when they overlap): touch.evaluate_pairs' cut.
    Phase M fix 1 (SF-102): compiled (batch_jit.pair_separation_kernel); _box_separation_numpy is the reference."""
    if P.sep is None:
        P.sep = pair_separation_kernel(P.lo, P.hi, np.asarray(tab.I, np.int64), np.asarray(tab.J, np.int64)) if len(tab) else np.zeros(0)
    return P.sep


def _box_separation_numpy(tab, P):
    """The reference of box_separation (numpy)."""
    return np.maximum(P.lo[tab.I] - P.hi[tab.J], P.lo[tab.J] - P.hi[tab.I]).max(axis=1) if len(tab) else np.zeros(0)


def _cylinder_exact(tab, P, p):
    """The gap of a cylinder pair at the pose (geometry.cylinder_capsule or cylinder_box, the same function the object form uses)."""
    a, b = tab.ka[p], tab.kb[p]
    cyl, oth = (a, b) if tab.kind_code[a] == KIND_CODE["cylinder"] else (b, a)
    rc, hc = float(tab.radius[cyl]), float(tab.half[cyl][2])
    if tab.kind_code[oth] == KIND_CODE["capsule"]:
        return float(cylinder_capsule(P.C[cyl], P.R[cyl], rc, hc, P.A[oth], P.B[oth], float(tab.radius[oth])).gap)
    return float(cylinder_box(P.C[cyl], P.R[cyl], rc, hc, P.C[oth], P.R[oth], tab.half[oth]).gap)


def _cylinder_upper(tab, P, p):
    """An upper bound of the gap of the cylinder pair p: the distance from the cylinder's centre to the other shape (the centre is inside
    the cylinder, so the cylinder is never farther than its centre; overlapping pairs have a gap of at most 0)."""
    a, b = tab.ka[p], tab.kb[p]
    cyl, oth = (a, b) if tab.kind_code[a] == KIND_CODE["cylinder"] else (b, a)
    c = P.C[cyl]
    if tab.kind_code[oth] == KIND_CODE["capsule"]:
        A, B = P.A[oth], P.B[oth]
        seg = B - A
        ll = float(seg @ seg)
        t = 0.0 if ll < 1e-300 else min(max(float((c - A) @ seg) / ll, 0.0), 1.0)
        return float(np.linalg.norm(c - (A + t * seg))) - float(tab.radius[oth])
    q = P.R[oth].T @ (c - P.C[oth])
    sd = float(np.linalg.norm(np.maximum(np.abs(q) - tab.half[oth], 0.0)) + min(np.max(np.abs(q) - tab.half[oth]), 0.0))
    return max(sd, 0.0)


def exact_gap(tab, P, p):
    """The gap of pair p at the pose: touch.gap_between's number (same function, same argument order). Phase L fix 2 (SF-100): compiled
    (batch_jit.pair_gap_at, the same contact functions) when cfg.GEOMETRY is compiled; _exact_gap_python is the reference."""
    if not cfg.GEOMETRY["compiled"]:
        return _exact_gap_python(tab, P, p)
    if tab.is_cy[p] or tab.is_cc[p]:
        a, b = tab.ka[p], tab.kb[p]
    else:
        a, b = tab.cb_cap[p], tab.cb_box[p]
    return float(pair_gap_at(a, b, tab.kind_code, P.A, P.B, P.C, P.R, tab.radius, tab.half))


def _exact_gap_python(tab, P, p):
    """The reference of exact_gap (the Python path of the contact functions)."""
    if tab.is_cy[p]:
        return _cylinder_exact(tab, P, p)
    if tab.is_cc[p]:
        a, b = tab.ka[p], tab.kb[p]
        return float(capsule_capsule(P.A[a], P.B[a], float(tab.radius[a]), P.A[b], P.B[b], float(tab.radius[b])).gap)
    c, b = tab.cb_cap[p], tab.cb_box[p]
    return float(capsule_box(P.A[c], P.B[c], float(tab.radius[c]), P.C[b], P.R[b], tab.half[b]).gap)


def approx_gaps(tab, P, idx):
    """(low, high) arrays for the pairs idx at the pose: the exact gap of each lies inside (see the module text). Kept per pose."""
    if P.glo is None:
        P.glo = np.full(len(tab), np.nan)
        P.ghi = np.full(len(tab), np.nan)
    need = idx[np.isnan(P.glo[idx])]
    if len(need):
        cc = need[tab.is_cc[need]]
        if len(cc):
            a, b = tab.ka[cc], tab.kb[cc]
            g = capsule_capsule_gap_batch(P.A[a], P.B[a], tab.radius[a], P.A[b], P.B[b], tab.radius[b])
            P.glo[cc], P.ghi[cc] = g - cfg.ARRAY_FORM['cc_eps_m'], g + cfg.ARRAY_FORM['cc_eps_m']
        cy = need[tab.is_cy[need]]                  # Phase D fix 4: cylinder pairs: lower bound = the boxes' separation (never above the gap), upper bound = the centre distance
        if len(cy):
            sep = box_separation(tab, P)
            P.glo[cy] = sep[cy]
            P.ghi[cy] = [_cylinder_upper(tab, P, p) for p in cy]
        cb = need[~tab.is_cc[need] & ~tab.is_cy[need]]
        if len(cb):
            c, b = tab.cb_cap[cb], tab.cb_box[cb]
            v = capsule_box_gap_batch(P.A[c], P.B[c], tab.radius[c], P.C[b], P.R[b], tab.half[b])
            length = np.linalg.norm(P.B[c] - P.A[c], axis=1)
            P.glo[cb] = v - cfg.ARRAY_FORM['cb_low_per_m'] * length - cfg.ARRAY_FORM['bound_slack_m']
            P.ghi[cb] = v + cfg.ARRAY_FORM['cb_high_per_m'] * length + cfg.ARRAY_FORM['bound_slack_m']
    return P.glo[idx], P.ghi[idx]
