"""The state of one resolver tick, in array form, and the pair sleep timers (Stage 5 speed; decision rows SF-69 to SF-75).

_Tick answers the same questions as before (candidate pairs, gaps, closing arms, shortfalls, the worst pair) with the same answers.
What changed is how: the shapes of a pose are arrays (tick_arrays.PoseArrays), the pair list of a pose is one array comparison over a
pair table kept between ticks, and threshold questions are first put to cheap bounds (box separation, travel, batched gap intervals);
the exact scalar gap (touch.gap_between's number) is computed only when the bounds do not decide, or when the value itself is used.

SleepTimers (candidate 1 of the speed plan): a reference pose with a lower bound of every pair's gap there; in later ticks a pair is
asleep (not looked at) while the travel of its two parts since the reference cannot have brought it inside its release distance and
cannot make the gap rule act. A tick with every pair asleep is a pass-through with no kinematics at all.
"""

import heapq

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
            if len(self._tables) >= cfg.ARRAY_FORM['table_cache']:
                self._tables.clear()
            items = self.spec_items + list(held_shapes) + w["placed"]
            tab = self._tables[sig] = ta.PairTable(items, self.n_spec + len(held_shapes))
            tab.st_n_spec = self.n_spec
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
        self._awake = self.awake0 = awake                      # None: every pair; else a bool mask over the table's pairs (sleep timers)
        self.woke_all = False
        self._lever = None
        self._P0 = P0

    # ---- poses ----
    def _place(self, q):
        k = q.tobytes()
        P = self._poses.get(k)
        if P is None:
            if len(self._poses) > cfg.ARRAY_FORM['pose_cache']:
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
        self.woke_all = True

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
        tab, dt, slack = self.tab, self.res.dt, cfg.ARRAY_FORM['bound_slack_m']
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
        tab, slack = self.tab, cfg.ARRAY_FORM['bound_slack_m']
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
        tab, slack = self.tab, cfg.ARRAY_FORM['bound_slack_m']
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
    """Priority-queue pair scheduler (SF-72). A min-heap keyed on wakeup_tick, the tick at which a pair must next be looked at.
    Each tick every pair with wakeup_tick <= now is popped and is the only pair the broad phase and the resolver see; after the tick
    each popped pair gets wakeup_tick = now + max(1, floor((G - release) / (v x dt x (1 + safety_buffer)))), G a lower bound of its gap at the
    pose the filter returns, release = margin + slowing band, v the fastest the pair's two parts can approach each other (the sum over each
    part's arm joints of the part's largest distance from the joint, for any pose, x the speed bound the caller gave the resolver (Resolver.joint_speed_rad_s, the command bridge limit)).
    A pair at or inside its release distance gets now + 1 (looked at every tick). A tick with no pair popped is a pass-through
    without any kinematics.

    Woken at once (every pair, or the affected pairs): the world list has other objects, the held objects change, the pair table or the
    settings change, the last command is not the pose the filter returned the tick before (all pairs); an object of the world list moved
    (the pairs of that object); a command step faster than the speed bound allows (the pairs of that arm, this tick); the base, pedestal
    or fingers moved by more than SLEEP["state_change_m"] since the tick before (all pairs, this tick).
    A tick that acts looks at every pair and reschedules every pair."""

    def __init__(self, statics):
        self.st = statics
        self.hist = None
        self.n_ticks = self.n_checked = 0          # statistics for the profile; kept over resets
        self.reset()

    def reset(self):
        self.tab = self.w_objs = self.q_ref = None
        self.now = getattr(self, "now", 0)
        self.heap = []

    @staticmethod
    def _held_sig(held):
        if held is None or not hasattr(held, "held"):
            return None if held is None else id(held)
        return tuple(sorted((side, id(rec), rec["kind"]) for side, rec in held.held.items()))

    def _settings_now(self):
        from src.safety_filter.shapes import never_close_pairs
        return (tuple(sorted(cfg.MARGINS.items())), cfg.NEVER_TOUCH_MARGIN_M, cfg.PUSH_START_BEYOND_M, cfg.PUSH_STOP_BEYOND_M, tuple(cfg.TORSO_EXEMPT_PARTS),
                id(never_close_pairs()), tuple(sorted(cfg.GAP_RULE.items())), cfg.RESOLVER["trigger_m"], tuple(sorted(cfg.SLEEP.items())), self._speed)

    def _static_lever(self, t):
        """Per item and arm joint: an upper bound, for EVERY pose, of the distance of any point of the item from the joint's anchor:
        the lengths of the links between the joint and the item's link (fixed) plus the item's reach from its own link's last joint."""
        tab, st, P = t.tab, self.st, t._P0
        t.res.shapes.set_state(with_arms(t.state, t.last))
        d = t.res.shapes.data
        lever = np.zeros((tab.n, ARM_JOINTS))
        for s, side in enumerate(SIDES):
            anc = d.xanchor[st.jids[side]].copy()
            seg = np.linalg.norm(np.diff(anc, axis=0), axis=1)                      # fixed distances between consecutive joint anchors
            for k in np.flatnonzero(tab.item_side == s):
                L = int(tab.link[k])
                if L == 0:
                    continue
                a = anc[L - 1]
                e = max(np.linalg.norm(P.A[k] - a), np.linalg.norm(P.B[k] - a)) if tab.is_capsule[k] else np.linalg.norm(P.C[k] - a) + np.linalg.norm(tab.half[k])
                for i in range(L):
                    lever[k, i] = seg[i:L - 1].sum() + e
        return lever

    @staticmethod
    def _state_vec(state):
        return np.concatenate([np.asarray(state.base_xyz, float), np.asarray(state.base_quat, float), [state.pedestal_m, state.finger_left, state.finger_right]])

    def _push(self, idx, ticks):
        self.wake[idx] = ticks
        for p, w in zip(idx.tolist(), np.broadcast_to(ticks, idx.shape).tolist()):
            heapq.heappush(self.heap, (w, p))

    def awake(self, res, raw, last, state, world, held):
        """The mask of the pairs that woke this tick, or None (look at every pair: no valid schedule). All False = pass-through."""
        self.now += 1
        self._speed = res.joint_speed_rad_s
        if not cfg.SLEEP["enabled"] or res.joint_speed_rad_s is None or self.tab is None or not gap_rule.bounds_valid(res.dt):
            return None
        objs = self.w_objs
        same_ids = len(objs) == len(world) and all(a.id == b.id for a, b in zip(objs, world))
        if (not same_ids or self._held_sig(held) != self.held_sig or self._settings != self._settings_now()
                or not np.allclose(last, self.q_ref, rtol=0.0, atol=cfg.SLEEP['same_pose_atol_rad'])):
            self.reset()
            return None
        tab, now = self.tab, self.now
        moved = [i for i, (a, b) in enumerate(zip(objs, world)) if a != b]
        if moved:                                           # an object moved: its pairs wake now
            hit = np.isin(tab.I, [tab.n_robot + i for i in moved]) | np.isin(tab.J, [tab.n_robot + i for i in moved])
            self._push(np.flatnonzero(hit), now)
            self.w_objs = list(world)
        mask = np.zeros(len(tab), dtype=bool)
        while self.heap and self.heap[0][0] <= now:
            w, p = heapq.heappop(self.heap)
            if self.wake[p] == w:
                mask[p] = True
        step_cap = res.joint_speed_rad_s * res.dt * (1.0 + cfg.SLEEP["safety_buffer"]) + cfg.SLEEP["command_step_slack_rad"]
        fast = np.abs(np.asarray(raw, float) - np.asarray(last, float)).max(axis=1) > step_cap       # per arm: the command step is faster than the bound
        mask |= (tab.uses_side[0] & fast[0]) | (tab.uses_side[1] & fast[1])
        sv = self._state_vec(state)
        if float(np.abs(sv - self.state_vec).sum()) > cfg.SLEEP["state_change_m"]:
            mask[:] = True
        self.state_vec = sv
        self.q_ref = np.array(raw, float)                  # the next tick's last command if the filter passes the raw command on (after_tick sets it otherwise)
        self.n_ticks += 1
        self.n_checked += int(mask.sum())
        return mask

    def after_tick(self, t, q):
        """Reschedules the pairs looked at this tick (all of them when the tick looked at every pair), from a lower bound of each pair's
        gap at the pose q the filter returns."""
        if not cfg.SLEEP["enabled"] or t.res.joint_speed_rad_s is None or not t._bounds_ok:
            return
        tab = t.tab
        fresh = self.tab is not tab
        if fresh:
            self.tab, self.w_objs, self.held_sig, self._settings = tab, list(t.world_objects), self._held_sig(t.held), self._settings_now()
            vitem = self._static_lever(t).sum(axis=1) * t.res.joint_speed_rad_s
            self.vpair = vitem[tab.I] + vitem[tab.J]
            self.wake = np.zeros(len(tab), dtype=np.int64)
            self.heap = []
            self.n_ticks += 1
            self.n_checked += len(tab)
            held_pair = ((tab.I >= tab.st_n_spec) & (tab.I < tab.n_robot)) | ((tab.J >= tab.st_n_spec) & (tab.J < tab.n_robot))
            self.ptype = np.where(held_pair, 3, np.where((tab.item_side[tab.I] >= 0) & (tab.item_side[tab.J] >= 0), 0, np.where(tab.J >= tab.n_robot, 2, 1)))
            if self.hist is None:
                self.hist = np.zeros((4, cfg.SLEEP["stats_cap_ticks"] + 1), dtype=np.int64)
        redo = np.ones(len(tab), dtype=bool) if (fresh or t.woke_all or t.awake0 is None) else t.awake0
        self.state_vec = self._state_vec(t.state)
        self.q_ref = np.array(q, float)
        P = t._place(np.asarray(q, float))
        idx = np.flatnonzero(redo)
        lb = ta.box_separation(tab, P)[idx].copy()
        near = lb < tab.release[idx] + cfg.ARRAY_FORM['bound_slack_m']
        if near.any():
            lo, _ = ta.approx_gaps(tab, P, idx[near])
            lb[near] = np.maximum(lb[near], lo)
        rem = lb - tab.release[idx]
        cost = np.maximum(self.vpair[idx] * t.res.dt * (1.0 + cfg.SLEEP["safety_buffer"]), cfg.SLEEP["min_cost_m"])
        n = np.where(rem > 0.0, np.minimum(np.floor(rem / cost), cfg.SLEEP["stats_cap_ticks"]), 0).astype(np.int64)
        n = np.maximum(n, 1)                                 # a pair inside its release distance, or with no whole tick to spare, is looked at every tick
        self._push(idx, self.now + n)
        np.add.at(self.hist, (self.ptype[idx], n), 1)

    def report(self):
        """The distribution of the wakeup intervals (ticks) per pair type and the mean number of pairs looked at per tick."""
        out = {"mean_pairs_checked_per_tick": (self.n_checked / self.n_ticks if self.n_ticks else None), "pairs": None if self.tab is None else len(self.tab)}
        if self.hist is None:
            return out
        for k, name in enumerate(("arm_vs_arm", "arm_vs_body", "arm_vs_world", "held_object")):
            h = self.hist[k]
            tot = int(h.sum())
            if tot:
                cum = np.cumsum(h)
                out[name] = {"n": tot, "mean": float((h * np.arange(len(h))).sum() / tot), "median": int(np.searchsorted(cum, tot / 2.0)), "max": int(np.flatnonzero(h)[-1])}
        return out
