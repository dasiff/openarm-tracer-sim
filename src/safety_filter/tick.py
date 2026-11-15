"""The state of one resolver tick, in array form, and the pair sleep timers (Stage 5 speed; decision rows SF-69 to SF-75).

_Tick answers the same questions as before (candidate pairs, gaps, closing arms, shortfalls, the worst pair) with the same answers.
What changed is how: the shapes of a pose are arrays (tick_arrays.PoseArrays), the pair list of a pose is one array comparison over a
pair table kept between ticks, and threshold questions are first put to cheap bounds (box separation, travel, batched gap intervals);
the exact scalar gap (touch.gap_between's number) is computed only when the bounds do not decide, or when the value itself is used.

SleepTimers (candidate 1 of the speed plan): a reference pose with a lower bound of every pair's gap there; in later ticks a pair is
asleep (not looked at) while the travel of its two parts since the reference cannot have brought it inside its release distance and
cannot make the gap rule act. A tick with every pair asleep is a pass-through with no kinematics at all.
"""

import mujoco
import numpy as np

from src.safety_filter import config as cfg
from src.safety_filter import gap_rule
from src.safety_filter import tick_arrays as ta
from src.safety_filter.arm_conversion import with_arms
from src.safety_filter.link_rules import category_of_part
from src.safety_filter.shapes import ARM_JOINTS, SIDES, PlacedShape
from src.safety_filter.touch import Exempt, _aabb_arrays, as_placed, gap_between, pair_rule

_ARM_INDEX = {"left": 0, "right": 1}
POSE_CACHE = 64          # count: poses kept per tick before the cache is emptied (as before the speed work: keeps a long shortening search's memory flat)
TABLE_CACHE = 12         # count: pair tables kept per resolver (one per set of world objects and held shapes seen)
SLEEP = {
    "enabled": True,          # on/off [SF-72]: pair sleep timers
    "reref_extra_awake": 6,   # count [SF-72]: a new reference pose is taken when this many more pairs are awake than right after the last reference
    "base_reach_extra_m": 0.05,  # m [SF-72]: added to the largest distance of any robot shape from the base origin when a base rotation is turned into travel
}


class _LazyGaps(dict):
    """Gap of each candidate pair at the last filtered command, computed when first read (same number as before)."""

    def __init__(self, tick):
        super().__init__()
        self._tick = tick

    def __missing__(self, key):
        v = self[key] = self._tick.gap(self._tick.last, key)
        return v


class Statics:
    """What a resolver keeps between ticks for the array form: the robot shapes' fixed data, the world arrays of the last world
    list, the pair tables, and the sleep timers."""

    def __init__(self, shapes):
        self.shapes = shapes
        m = shapes.model
        specs = shapes.specs
        self.n_spec = len(specs)
        self.spec_items = [PlacedShape(id=s["id"], arm=s["arm"], part=s["part"], kind=s["kind"], body=s["frame"], r=float(s.get("radius", 0.0)),
                                       half=np.asarray(s.get("half", (0.0, 0.0, 0.0)), float), geoms=tuple(s["geoms"]), segment=s.get("segment", ""))
                           for s in specs]
        self.cap_items, self.box_items = shapes._cap_idx, shapes._box_idx
        self.cap_r = shapes._cap_r[:, None] if len(self.cap_items) else np.zeros((0, 1))
        self.box_h = shapes._box_h if len(self.box_items) else np.zeros((0, 3))
        # per robot shape: how many arm joints lie between the torso and the body (or bodies) the shape is fixed to
        jid = {s: list(shapes._joint[s]) for s in SIDES}
        self.jids = jid

        def n_joints(body_name, side):
            b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
            count = 0
            while b > 0:
                count += sum(1 for j in jid[side] if m.jnt_bodyid[j] == b)
                b = m.body_parentid[b]
            return count

        self.spec_link = np.zeros(self.n_spec, dtype=np.int64)
        for k, s in enumerate(specs):
            if s["arm"] in SIDES:
                bodies = list(s.get("ends_body", [])) + [s["frame"]]
                self.spec_link[k] = max(n_joints(b, s["arm"]) for b in bodies)
        self._tables, self._world = {}, None
        self.sleep = SleepTimers(self)

    # ---- world list -> arrays (kept while the list does not change) ----
    def world(self, objects):
        w = self._world
        if w is not None and len(w["objs"]) == len(objects) and all(a is b or a == b for a, b in zip(w["objs"], objects)):
            return w
        placed = [as_placed(o) for o in objects]
        n = len(placed)
        w = {"objs": list(objects), "placed": placed, "by_id": {s.id: s for s in placed},
             "sig": tuple((s.id, s.kind, bool(s.never_touch)) for s in placed),
             "A": np.zeros((n, 3)), "B": np.zeros((n, 3)), "C": np.zeros((n, 3)), "R": np.zeros((n, 3, 3))}
        for k, s in enumerate(placed):
            if s.kind == "capsule":
                w["A"][k], w["B"][k] = s.a, s.b
            else:
                w["C"][k], w["R"][k] = s.centre, s.rot
        w["lo"], w["hi"] = _aabb_arrays(placed) if n else (np.zeros((0, 3)), np.zeros((0, 3)))
        self._world = w
        return w

    def table(self, held_shapes, w):
        """The pair table for these held shapes and this world list under the present settings (built once, kept)."""
        from src.safety_filter.shapes import never_close_pairs
        sig = (tuple((h.id, h.kind, h.arm) for h in held_shapes), w["sig"], tuple(sorted(cfg.MARGINS.items())), cfg.NEVER_TOUCH_MARGIN_M,
               cfg.PUSH_START_BEYOND_M, cfg.PUSH_STOP_BEYOND_M, tuple(cfg.TORSO_EXEMPT_PARTS), id(never_close_pairs()))
        tab = self._tables.get(sig)
        if tab is None:
            if len(self._tables) >= TABLE_CACHE:
                self._tables.clear()
            items = self.spec_items + list(held_shapes) + w["placed"]
            tab = self._tables[sig] = ta.PairTable(items, self.n_spec + len(held_shapes))
            tab.never_close = never_close_pairs()           # kept alive so that its id stays unique while the table lives
            tab.link = np.concatenate([self.spec_link, np.full(len(held_shapes), ARM_JOINTS, dtype=np.int64), np.zeros(len(w["placed"]), dtype=np.int64)])
            rad = {}
            for k in range(tab.n_robot):
                s = items[k]
                if s.arm in SIDES and s.kind == "capsule" and s.r > 0.0:
                    c = category_of_part(s.part)[0]
                    rad[c] = min(rad.get(c, np.inf), s.r)
            tab.capsule_radii = rad
            tab.items = items
        return tab


class _Tick:
    """The state of one tick: poses (arrays), candidate pairs, gaps and closing flags, cached."""

    def __init__(self, res, raw, last, state, world, held, awake=None):
        self.res, self.raw, self.last, self.state, self.held = res, raw, last, state, held
        self.world_objects = list(world)
        st = self.st = res.statics
        self._w = st.world(self.world_objects)
        self.world = self._w["by_id"]
        self._poses, self._gaps = {}, {}
        self.rules, self.g0, self._closing = {}, _LazyGaps(self), {}
        self.keys = set()
        self._kidx, self._karr = [], None
        self._bounds_ok = gap_rule.bounds_valid(res.dt)
        self.tab = None
        P0 = self._place(np.asarray(last, float))             # the pose of the last command: fixes the held shapes, hence the pair table
        self._in = np.zeros(len(self.tab), dtype=bool)
        self._awake = awake                                    # None: every pair; else a bool mask over the table's pairs (sleep timers)
        self._lever = None
        self._P0 = P0

    # ---- poses ----
    def _place(self, q):
        k = q.tobytes()
        P = self._poses.get(k)
        if P is None:
            if len(self._poses) > POSE_CACHE:
                self._poses.clear()
                self._gaps.clear()
            P = self._poses[k] = self._build(q)
        return P

    def _build(self, q):
        st, sh, w = self.st, self.res.shapes, self._w
        sh.set_state(with_arms(self.state, q))
        cA, cB, bC, bR = sh.place_arrays()
        held = []
        if self.held is not None:
            for side in SIDES:
                pos, rot = sh.hand_frame(side)
                hs = self.held.placed(side, pos, rot)
                if hs is not None:
                    held.append(hs)
        if self.tab is None:
            self.tab = st.table(held, w)
        tab = self.tab
        n, ns, nr = tab.n, st.n_spec, tab.n_robot
        if nr - ns != len(held) or any(tab.ids[ns + m] != h.id or tab.is_capsule[ns + m] != (h.kind == "capsule") for m, h in enumerate(held)):
            raise RuntimeError("the held shapes changed inside one tick")
        A, B, C, R = np.zeros((n, 3)), np.zeros((n, 3)), np.zeros((n, 3)), np.zeros((n, 3, 3))
        lo, hi = np.empty((n, 3)), np.empty((n, 3))
        if len(st.cap_items):
            A[st.cap_items], B[st.cap_items] = cA, cB
            lo[st.cap_items] = np.minimum(cA, cB) - st.cap_r
            hi[st.cap_items] = np.maximum(cA, cB) + st.cap_r
        if len(st.box_items):
            C[st.box_items], R[st.box_items] = bC, bR
            ext = (np.abs(bR) @ st.box_h[:, :, None])[:, :, 0]
            lo[st.box_items], hi[st.box_items] = bC - ext, bC + ext
        for m, h in enumerate(held):
            k = ns + m
            if h.kind == "capsule":
                A[k], B[k] = h.a, h.b
            else:
                C[k], R[k] = h.centre, h.rot
            lo[k], hi[k] = h.aabb()
        if n > nr:
            A[nr:], B[nr:], C[nr:], R[nr:], lo[nr:], hi[nr:] = w["A"], w["B"], w["C"], w["R"], w["lo"], w["hi"]
        return ta.PoseArrays(A, B, C, R, lo, hi, held)

    def pose(self, q):
        """(shapes by id, list of shapes) of the robot and held shapes at the joint angles q (2, 7): the object form, built on request."""
        allp = [self.shape(q, sid) for sid in self.tab.ids[:self.tab.n_robot]]
        return {s.id: s for s in allp}, allp

    def shape(self, q, sid):
        P = self._place(q)
        s = P.objs.get(sid)
        if s is None:
            k = self.tab.item_index.get(sid)
            if k is None:
                return None
            ns = self.st.n_spec
            if k >= self.tab.n_robot:
                s = self.world[sid]
            elif k >= ns:
                s = P.held[k - ns]
            else:
                it = self.st.spec_items[k]
                if it.kind == "capsule":
                    s = PlacedShape(id=it.id, arm=it.arm, part=it.part, kind="capsule", body=it.body, a=P.A[k], b=P.B[k], r=it.r, geoms=it.geoms, segment=it.segment)
                else:
                    s = PlacedShape(id=it.id, arm=it.arm, part=it.part, kind="box", body=it.body, centre=P.C[k], rot=P.R[k], half=it.half, geoms=it.geoms, segment=it.segment)
            P.objs[sid] = s
        return s

    def gap(self, q, key):
        """Scalar gap of a pair at the pose q (the shared layer's exact distance); +inf if a shape is missing."""
        ck = (q.tobytes(), key)
        g = self._gaps.get(ck)
        if g is None:
            p = self.tab.key_index.get(key)
            if p is not None:
                g = ta.exact_gap(self.tab, self._place(q), p)
            else:                                              # a pair the touch rules do not check: the object form
                a, b = self.shape(q, key[0]), self.shape(q, key[1])
                g = np.inf if a is None or b is None else float(gap_between(a, b)[0])
            self._gaps[ck] = g
        return g

    # ---- candidate pairs ----
    def discover(self, q):
        """Adds the pairs near at the pose q to the candidate set (touch.evaluate_pairs' list: box separation within the pair's activation distance)."""
        tab = self.tab
        near = (ta.box_separation(tab, self._place(q)) <= tab.act) & ~self._in
        if self._awake is not None:
            near &= self._awake
        for p in np.flatnonzero(near):
            self._add_index(int(p))

    def _add_index(self, p):
        self._in[p] = True
        self._kidx.append(p)
        self._karr = None
        key = self.tab.keys[p]
        self.keys.add(key)
        self.rules[key] = self.tab.rules[p]

    def _add(self, key, rule):
        if key not in self.keys:
            p = self.tab.key_index.get(key)
            if p is not None:
                self._add_index(p)
            else:
                self.keys.add(key)
            self.rules[key] = rule

    def add_known(self, key):
        if key not in self.keys:
            p = self.tab.key_index.get(key)
            if p is not None and (self._awake is None or self._awake[p]):
                self._add_index(p)

    def wake_all(self):
        """Drops the sleep mask (a tick that acts looks at every pair, as before the sleep timers)."""
        self._awake = None

    def _indices(self):
        if self._karr is None:
            self._karr = np.array(self._kidx, dtype=np.int64)
        return self._karr

    def arms_of(self, key):
        out = []
        for sid in key:
            k = self.tab.item_index.get(sid)
            if k is not None and self.tab.item_side[k] >= 0 and SIDES[self.tab.item_side[k]] not in out:
                out.append(SIDES[self.tab.item_side[k]])
        return out

    def closing(self, key):
        """The arms of the pair that are closing: moving only that arm to its raw command makes the gap smaller (SF-30)."""
        c = self._closing.get(key)
        if c is None:
            c = []
            for side in self.arms_of(key):
                i = _ARM_INDEX[side]
                if np.array_equal(self.raw[i], self.last[i]):   # the arm does not move: the gap is the one at the last command
                    continue
                q = self.last.copy()
                q[i] = self.raw[i]
                if self.gap(q, key) < self.g0[key] - cfg.RESOLVER["closing_eps_m"]:
                    c.append(side)
            self._closing[key] = c
        return c

    def capsule_radii(self):
        """Smallest capsule radius per link category over the arm shapes (and held shapes) of this tick."""
        return self.tab.capsule_radii

    # ---- bounds ----
    def _travel(self, P, q):
        """Per item: how far any point of it can have moved between the last command and the pose q (m); see tick_arrays."""
        if P.travel is None:
            tab, st = self.tab, self.st
            if self._lever is None:
                P0 = self._P0
                self.res.shapes.set_state(with_arms(self.state, self.last))
                d = self.res.shapes.data
                lever = np.zeros((tab.n, ARM_JOINTS))
                for s, side in enumerate(SIDES):
                    anc = d.xanchor[st.jids[side]]                                  # (7, 3) at the last command
                    rows = np.flatnonzero(tab.item_side == s)
                    if not len(rows):
                        continue
                    cap = tab.is_capsule[rows]
                    dist = np.where(cap[:, None],
                                    np.maximum(np.linalg.norm(P0.A[rows][:, None, :] - anc[None], axis=2), np.linalg.norm(P0.B[rows][:, None, :] - anc[None], axis=2)),
                                    np.linalg.norm(P0.C[rows][:, None, :] - anc[None], axis=2) + np.linalg.norm(tab.half[rows], axis=1)[:, None])
                    lever[rows] = np.where(np.arange(ARM_JOINTS)[None, :] < tab.link[rows][:, None], dist, 0.0)
                self._lever = lever
            dq = np.abs(q - self.last)
            side = np.maximum(tab.item_side, 0)
            P.travel = np.einsum("ki,ki->k", self._lever, dq[side])
        return P.travel

    def _moving(self):
        """Per pair: one of its arms moves between the last and the raw command (otherwise no arm of it can be closing)."""
        ml, mr = (not np.array_equal(self.raw[0], self.last[0])), (not np.array_equal(self.raw[1], self.last[1]))
        u = self.tab.uses_side
        return (u[0] & ml) | (u[1] & mr)

    def _maybe_short(self, q, thr):
        """Indices of the candidate pairs whose shortfall at the pose q may be at or above thr and that may have a closing arm
        (a superset; the caller measures them exactly)."""
        idx = self._indices()
        if not len(idx):
            return idx
        idx = idx[self._moving()[idx]]
        if not len(idx) or not self._bounds_ok:
            return idx
        tab, dt, slack = self.tab, self.res.dt, ta.BOUND_SLACK_M
        P0, P = self._place(self.last), self._place(q)
        lb0 = ta.box_separation(tab, P0)[idx]
        tr = self._travel(P, q)
        ub = gap_rule.required_gap_next_array(lb0, tab.margin[idx], dt) - lb0 + tr[tab.I[idx]] + tr[tab.J[idx]]
        idx = idx[ub >= thr - slack]
        if not len(idx):
            return idx
        _, hi0 = ta.approx_gaps(tab, P0, idx)
        lo, _ = ta.approx_gaps(tab, P, idx)
        ub = gap_rule.required_gap_next_array(hi0, tab.margin[idx], dt) - lo
        return idx[ub >= thr - slack]

    def maybe_short(self, q, thr):
        """The candidate pairs (names) whose shortfall at q may reach thr: every pair with shortfall >= thr and a closing arm is in it."""
        return [self.tab.keys[p] for p in self._maybe_short(q, thr)] + [k for k in self.keys if k not in self.tab.key_index]

    def any_gap_below(self, q, limit, skip_start_below):
        """True when a candidate pair's gap at the pose q is below limit (after adding the pairs near at q); with skip_start_below a
        pair already below limit at the last command does not count (the jump-through guard's test, SF-46)."""
        self.discover(q)
        idx = self._indices()
        tab, slack = self.tab, ta.BOUND_SLACK_M
        if len(idx) and self._bounds_ok:
            P = self._place(q)
            idx = idx[ta.box_separation(tab, P)[idx] < limit + slack]
            if len(idx):
                lo, _ = ta.approx_gaps(tab, P, idx)
                idx = idx[lo < limit + slack]
            if len(idx) and skip_start_below:
                _, hi0 = ta.approx_gaps(tab, self._place(self.last), idx)
                idx = idx[~(hi0 < limit - slack)]
        for key in [tab.keys[p] for p in idx] + [k for k in self.keys if k not in tab.key_index]:
            if self.gap(q, key) < limit and not (skip_start_below and self.g0[key] < limit):
                return True
        return False

    def active_keys(self, q):
        """The candidate pairs inside their release distance at the pose q or at the last command (what the resolver keeps for the next tick)."""
        idx = self._indices()
        tab, slack = self.tab, ta.BOUND_SLACK_M
        out = set()
        if len(idx) and self._bounds_ok:
            P0, P = self._place(self.last), self._place(q)
            rel = tab.release[idx]
            keep = np.minimum(ta.box_separation(tab, P)[idx], ta.box_separation(tab, P0)[idx]) < rel + slack
            idx, rel = idx[keep], rel[keep]
            if len(idx):
                lo0, hi0 = ta.approx_gaps(tab, P0, idx)
                lo, hi = ta.approx_gaps(tab, P, idx)
                sure = np.minimum(hi0, hi) < rel - slack
                out.update(tab.keys[p] for p in idx[sure])
                idx = idx[~sure & (np.minimum(lo0, lo) < rel + slack)]
        for key in [tab.keys[p] for p in idx] + [k for k in self.keys if k not in tab.key_index]:
            if min(self.gap(q, key), self.g0[key]) < self.rules[key].release:
                out.add(key)
        return out

    # ---- shortfall ----
    def shortfall(self, q, key):
        return gap_rule.shortfall(self.g0[key], self.gap(q, key), self.rules[key].margin, self.res.dt)

    def worst(self, q, discover=False):
        """(key, shortfall) of the worst pair that needs action at the pose q, or None."""
        if discover:
            self.discover(q)
        best = None
        trig = cfg.RESOLVER["trigger_m"]
        for key in self.maybe_short(q, trig):
            s = self.shortfall(q, key)
            if s > trig and self.closing(key) and (best is None or s > best[1]):
                best = (key, s)
        return best

    def worst_value_discover(self, q):
        """The worst shortfall at the pose q (0.0 if no pair needs action), after adding the pairs near at q."""
        w = self.worst(q, discover=True)
        return 0.0 if w is None else w[1]


class SleepTimers:
    """Pair sleep timers (see the module text). Kept by the resolver between ticks; any change of the pair table (held shapes,
    settings) or of the world list starts again from nothing."""

    def __init__(self, statics):
        self.st = statics
        self.reset()

    def reset(self):
        self.tab = self.w = None

    def _static_lever(self, t):
        """Per item and arm joint: an upper bound, for EVERY pose, of the distance of any point of the item from the joint's anchor:
        the lengths of the links between the joint and the item's link (fixed) plus the item's reach from its own link's last joint."""
        tab, st, P = t.tab, self.st, t._P0
        t.res.shapes.set_state(with_arms(t.state, t.last))
        d = t.res.shapes.data
        lever = np.zeros((tab.n, ARM_JOINTS))
        reach = 0.0
        for s, side in enumerate(SIDES):
            anc = d.xanchor[st.jids[side]].copy()
            seg = np.linalg.norm(np.diff(anc, axis=0), axis=1)                      # fixed distances between consecutive joint anchors
            reach = max(reach, float(seg.sum()))
            for k in np.flatnonzero(tab.item_side == s):
                L = int(tab.link[k])
                if L == 0:
                    continue
                a = anc[L - 1]
                e = max(np.linalg.norm(P.A[k] - a), np.linalg.norm(P.B[k] - a)) if tab.is_capsule[k] else np.linalg.norm(P.C[k] - a) + np.linalg.norm(tab.half[k])
                for i in range(L):
                    lever[k, i] = seg[i:L - 1].sum() + e
        nr = tab.n_robot
        base = np.asarray(t.state.base_xyz, float)
        far = max([np.linalg.norm(P.A[:nr] - base, axis=1).max(), np.linalg.norm(P.B[:nr][tab.is_capsule[:nr]] - base, axis=1).max()] if nr else [0.0])
        return lever, float(far) + 2.0 * reach + SLEEP["base_reach_extra_m"]

    def reference(self, t):
        """Takes the tick's last command as the reference pose: a lower bound of every pair's gap there (box separation, or the batched
        gap interval's low end for the pairs whose boxes are inside the release distance)."""
        tab = t.tab
        P0 = t._place(t.last)
        lb = ta.box_separation(tab, P0).copy()
        near = np.flatnonzero(lb < tab.release + ta.BOUND_SLACK_M)
        if len(near):
            lo, _ = ta.approx_gaps(tab, P0, near)
            lb[near] = np.maximum(lb[near], lo)
        self.tab, self.w, self.lb = tab, t._w, lb
        self.q = np.array(t.last, float)
        s = t.state
        self.base = np.asarray(s.base_xyz, float)
        self.quat = np.asarray(s.base_quat, float)
        self.other = np.array([s.pedestal_m, s.finger_left, s.finger_right], float)
        self.held_sig = self._held_sig(t.held)
        self.lever, self.base_reach = self._static_lever(t)
        self.side = np.maximum(tab.item_side, 0)
        self.awake0 = int((~self._asleep(self.q, self.q, s)).sum())

    @staticmethod
    def _held_sig(held):
        if held is None or not hasattr(held, "held"):
            return None if held is None else id(held)
        return tuple(sorted((side, id(rec), rec["kind"]) for side, rec in held.held.items()))

    def _asleep(self, raw, last, state):
        tab, dt = self.tab, self._dt
        dq = np.maximum(np.abs(raw - self.q), np.abs(last - self.q))
        tr = np.einsum("ki,ki->k", self.lever, dq[self.side])
        quat = np.asarray(state.base_quat, float)
        turn = 2.0 * np.arccos(min(1.0, abs(float(quat @ self.quat))))
        common = (float(np.linalg.norm(np.asarray(state.base_xyz, float) - self.base)) + turn * self.base_reach
                  + float(np.abs(np.array([state.pedestal_m, state.finger_left, state.finger_right]) - self.other).sum()))
        lb = self.lb - tr[tab.I] - tr[tab.J] - common
        step = np.einsum("ki,ki->k", self.lever, np.abs(raw - last)[self.side])
        ub = gap_rule.required_gap_next_array(lb, tab.margin, dt) - lb + step[tab.I] + step[tab.J]
        return (lb >= tab.release + ta.BOUND_SLACK_M) & (ub <= cfg.RESOLVER["trigger_m"] - ta.BOUND_SLACK_M)

    def awake(self, res, raw, last, state, world, held):
        """The mask of the pairs that must be looked at this tick, or None when there is no valid reference (look at every pair).
        An all-False mask means every pair is asleep."""
        if not SLEEP["enabled"] or self.tab is None or not gap_rule.bounds_valid(res.dt):
            return None
        w = self.w
        if len(w["objs"]) != len(world) or not all(a is b or a == b for a, b in zip(w["objs"], world)) or self._held_sig(held) != self.held_sig:
            self.reset()
            return None
        if self._settings != self._settings_now():
            self.reset()
            return None
        self._dt = res.dt
        mask = ~self._asleep(np.asarray(raw, float), np.asarray(last, float), state)
        self.n_awake = int(mask.sum())
        return mask

    def _settings_now(self):
        from src.safety_filter.shapes import never_close_pairs
        return (tuple(sorted(cfg.MARGINS.items())), cfg.NEVER_TOUCH_MARGIN_M, cfg.PUSH_START_BEYOND_M, cfg.PUSH_STOP_BEYOND_M, tuple(cfg.TORSO_EXEMPT_PARTS),
                id(never_close_pairs()), cfg.GAP_RULE["mode"], cfg.GAP_RULE["approach_per_s"], cfg.GAP_RULE["max_grow_speed_m_s"], cfg.RESOLVER["trigger_m"])

    def after_tick(self, t):
        """Called at the end of a tick that built its poses: takes a new reference when there is none or when too many pairs have woken."""
        if not SLEEP["enabled"] or not t._bounds_ok:
            return
        if self.tab is None or self.tab is not t.tab or self.n_awake - self.awake0 >= SLEEP["reref_extra_awake"]:
            self._dt = t.res.dt
            self._settings = self._settings_now()
            self.reference(t)
            self.n_awake = self.awake0
