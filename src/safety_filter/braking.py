"""The braking layer (Part 2, David's braking brief): sits ABOVE the resolver; the gap rule inside the resolver is unchanged.

Works on (2, 7) joint arrays (left arm, right arm, rad) like the plug-in. Everything is measured at the tool point.
  measure()          measured speed and acceleration: finite difference of the tool point on consecutive ticks (world frame, so the base is in it)
  commanded_speed()  speed of a commanded joint change: forward kinematics of the change at the tool point (base contributes zero)
"""
import math
from collections import deque
from dataclasses import replace

import numpy as np

from src.safety_filter import config as cfg
from src.safety_filter.braking_math import JerkRamp, slowing_distance, stopping_distance
from src.safety_filter.touch import Exempt, as_placed, gap_between, pair_rule, wake_dist
from src.safety_filter.pair_jit import KIND_BOX, KIND_CAPSULE, KIND_CYLINDER, aabb_all, look_kernel    # Phase J fix 2 (SF-93): the compiled candidate loop

SIDES = ("left", "right")
ACCEL_WINDOW_TICKS = 5    # ticks: the measured acceleration is the average over this many ticks (Phase C fix; PLACEHOLDER — tuned by test)
HAND_SHAPE_PARTS = ("palm", "finger", "link7", "held")      # the shapes of one arm the look-ahead brakes for (the hand and what it holds)


def make_tool_fn(shapes, offset=None):
    """Tool point function of the real model: tool_fn(q (2, 7), state) -> (2, 3) world positions. state (RobotState) carries the base,
    pedestal and fingers; its own arm joints are replaced by q. Tool point = link 7 origin + R7 @ offset (config tool_offset_local_m)."""
    off = np.asarray(cfg.BRAKING["tool_offset_local_m"] if offset is None else offset, float)

    def tool_fn(q, state):
        shapes.set_arms(q, state)                    # Phase K fix 3 (SF-98): the same as set_state(replace(state, q_left=..., q_right=...))
        out = np.zeros((2, 3))
        for i, side in enumerate(SIDES):
            pos, rot = shapes.hand_frame(side)
            out[i] = pos + rot @ off
        return out
    return tool_fn


_KIND_CODE = {"capsule": KIND_CAPSULE, "box": KIND_BOX, "cylinder": KIND_CYLINDER}


def _stop_of(rule):
    """The stop gap of a pair rule (m): the distance from the margin boundary a stop lands at (stop_gap of the rule's surface)."""
    ceiling = cfg.approach_ceiling(rule.surface) if rule.surface else None
    return stop_gap(rule.margin, ceiling, rule.work_surface)


def _shape_arrays(allp):
    """Phase J fix 2 (SF-93): the placed shapes as arrays for the compiled loop: kind, arm (0 left, 1 right, -1 other), capsule ends A, B and
    radius r, box or cylinder centre C, rotation R and half sizes h (a cylinder's radius is r)."""
    n = len(allp)
    kind = np.empty(n, np.int64)
    armc = np.full(n, -1, np.int64)
    A, B, C = np.zeros((n, 3)), np.zeros((n, 3)), np.zeros((n, 3))
    r = np.zeros(n)
    R, h = np.zeros((n, 3, 3)), np.zeros((n, 3))
    for s, sh in enumerate(allp):
        kind[s] = _KIND_CODE[sh.kind]
        if sh.arm in SIDES:
            armc[s] = SIDES.index(sh.arm)
        if sh.kind == "capsule":
            A[s], B[s], r[s] = sh.a, sh.b, sh.r
        else:
            C[s], R[s], h[s] = sh.centre, sh.rot, sh.half
            if sh.kind == "cylinder":
                r[s] = sh.r
    return kind, armc, A, B, r, C, R, h


class ObstacleFeed:
    """The look-ahead's obstacle query. feed(q, state) -> [None or (gap_m, margin_m, ceiling_m_s or None)] for the two arms: the pair of the
    arm's hand shapes (palm, fingers, link 7, the held object) with any other shape that has the smallest room (gap - margin/2) at pose q.
    Every pair within worst_stop_dist_m is considered (the cutoff of each pair rule is widened to at least that); the pair rules are the
    filter's own (touch.pair_rule), so exemptions, margins and allowed-contact surfaces are the same. The ceiling is set for a held object
    against an allowed-contact surface (its approach ceiling, config.approach_ceiling)."""

    def __init__(self, shapes):
        self.shapes = shapes                  # its own RobotShapes (the resolver's kinematics are not shared)
        self.world, self.held = [], None
        self._wkey, self._wplaced = None, []
        self.wake, self.hold_arms = None, np.zeros(2, bool)    # Phase F fix 5/6 (SF-82, SF-83): per arm, the wake radius (m) or None (every pair within the static cutoff)
        self.n_calls, self.n_considered = 0, 0                 # run diagnostics: calls of pairs(), and pairs that passed the cutoff (their gap is computed)
        self._rules = {}                                       # Phase F (a cache, no decision): touch.pair_rule per pair of shape attributes (it reads only these)
        self._excluded = {"left": set(), "right": set()}       # Phase I fix 2 (SF-90): per arm, the released objects excluded from its look-ahead
        self._prev_held = {}                                   # Phase I fix 2: per arm, the object held on the previous tick
        self._rule_cache = {}                                  # Phase J fix 2 (a cache, no decision): the rule arrays of the compiled loop, per shape list
        self._max_margin = max(list(cfg.MARGINS.values()) + [cfg.NEVER_TOUCH_MARGIN_M])

    def _rule_of(self, a, b):
        """touch.pair_rule, cached per pair of shape attributes (the rule reads the id, arm, part, kind and the three flags of each shape)."""
        ka = (a.id, a.arm, a.part, a.kind, a.allowed_contact, a.never_touch, a.work_surface, a.support)
        kb = (b.id, b.arm, b.part, b.kind, b.allowed_contact, b.never_touch, b.work_surface, b.support)
        key = (ka, kb)
        rule = self._rules.get(key)
        if rule is None:
            rule = self._rules[key] = pair_rule(a, b)
        return rule

    def _threshold(self, i):
        """The largest box separation (m) a pair of arm i can have and still be a candidate: the largest cutoff any pair rule gives
        (margin + push start, or the worst stopping distance), or with the wake radius of arm i (plus the largest stop gap, Phase J fix 3)."""
        t_static = max(self._max_margin + cfg.PUSH_START_BEYOND_M, wake_dist())
        if self.wake is None:
            return t_static
        return min(t_static, max(self._max_margin + cfg.PUSH_START_BEYOND_M,
                                 float(self.wake[i]) + self._max_margin + cfg.PUSH_STOP_BEYOND_M))

    def set_wake(self, wake, hold_arms):
        """Sets this tick's wake radii (m, per arm; None = the static cutoffs) and the held arms (HOLD: the margin is added to the radius)."""
        self.wake = None if wake is None else np.asarray(wake, float)
        self.hold_arms = np.asarray(hold_arms, bool)

    @staticmethod
    def _activation(rule, wake, hold, reach):
        """The cutoff of a pair (m) with the wake radius (Phase J fix 3, SF-94): the wake (the stopping distance now or on the next tick) plus
        the pair's stop gap (reach: the distance a stop triggers at), or plus the margin in HOLD (SF-83); never wider than the static cutoff.
        At rest the cutoff is the stop gap, so a pair that a stop must catch is always awake."""
        return min(rule.activation, wake + (rule.margin if hold else reach))

    def bind(self, world_objects, held):
        """Sets this tick's world objects and held-object tracker."""
        key = tuple(id(o) for o in world_objects)
        if key != self._wkey:
            self._wkey, self._wplaced = key, [as_placed(o) for o in world_objects]
        self.world, self.held = world_objects, held
        # Phase I fix 2 (SF-90): an object that was held by an arm and is no longer held (the gripper opened) is excluded from that arm's
        # look-ahead until the hand is held_world_margin_m away from it; an object picked up again is not excluded
        cur = {}
        if held is not None and hasattr(held, "held"):
            cur = {side: rec["obj"].id for side, rec in held.held.items() if isinstance(rec, dict) and "obj" in rec}
        for side, oid in self._prev_held.items():
            if cur.get(side) != oid:
                self._excluded[side].add(oid)
        for side, oid in cur.items():
            self._excluded[side].discard(oid)
        self._prev_held = cur

    def pairs(self, q, state):
        """For each arm, every candidate pair within its (widened) cutoff at pose q: a list of (room, gap, margin, ceiling, hand shape id,
        other shape id)."""
        q = np.asarray(q, float)
        self.n_calls += 1
        items = list(self.shapes.place(replace(state, q_left=tuple(q[0]), q_right=tuple(q[1]))))
        held = []
        if self.held is not None:
            for side in SIDES:
                pos, rot = self.shapes.hand_frame(side)
                hs = self.held.placed(side, pos, rot)
                if hs is not None:
                    held.append(hs)
        allp = items + held + list(self._wplaced)
        if not allp:
            return [[], []]
        compiled = bool(cfg.GEOMETRY["compiled"])
        if compiled:
            arr = _shape_arrays(allp)                  # Phase J fix 2 (SF-93): the shapes as arrays for the compiled candidate loop
            lo, hi = aabb_all(arr[0], arr[2], arr[3], arr[4], arr[5], arr[6], arr[7])
        else:
            lo = np.array([_aabb(x)[0] for x in allp])
            hi = np.array([_aabb(x)[1] for x in allp])
        for side in SIDES:                              # Phase I fix 2 (SF-90): a released object becomes a normal obstacle once every hand shape of the arm is held_world_margin_m away
            excl = self._excluded[side]
            if not excl:
                continue
            hand_idx = [ia for ia, a in enumerate(allp) if a.arm == side and a.part in HAND_SHAPE_PARTS]
            for ib, b in enumerate(allp):
                if b.id in excl and hand_idx:
                    d = min(float(np.maximum(lo[ia] - hi[ib], lo[ib] - hi[ia]).max()) for ia in hand_idx)
                    if d > float(cfg.BRAKING["held_world_margin_m"]):
                        excl.discard(b.id)
        if compiled:
            return self._pairs_compiled(allp, arr, lo, hi)
        out = [[], []]
        for i, side in enumerate(SIDES):
            thr = self._threshold(i)
            for ia, a in enumerate(allp):
                if a.arm != side or a.part not in HAND_SHAPE_PARTS:
                    continue
                # Phase F (exact; a speed change, no decision): the box separation of this hand shape with every shape at once; a pair beyond every cutoff the feed can
                # use is skipped before its rule is looked up (the same numbers as the pair-by-pair separation)
                sep_row = np.maximum(lo[ia] - hi, lo - hi[ia]).max(axis=1)
                for ib, b in enumerate(allp):
                    if ib == ia or (b.arm == side):
                        continue
                    if a.kind == "box" and b.kind == "box":
                        continue
                    if sep_row[ib] > thr:
                        continue
                    if b.id in self._excluded[side]:          # Phase I fix 2: the just-released object, until the hand is held_world_margin_m away
                        continue
                    rule = self._rule_of(a, b)
                    if isinstance(rule, Exempt):
                        continue
                    sep = float(sep_row[ib])
                    act = rule.activation if self.wake is None else self._activation(rule, float(self.wake[i]), bool(self.hold_arms[i]), _stop_of(rule))
                    if sep > act:
                        continue
                    self.n_considered += 1
                    gap, pa, pb, push = gap_between(a, b)
                    gap = float(gap)
                    d_ = np.asarray(pb, float) - np.asarray(pa, float)               # Phase C step 6: the direction from the hand toward the obstacle
                    if np.linalg.norm(d_) > 1e-9:
                        nrm = d_ / np.linalg.norm(d_)
                    else:
                        nrm = -np.asarray(push, float) / max(np.linalg.norm(push), 1e-12)   # overlapping: the direction back out of the obstacle, reversed
                    ceiling = cfg.approach_ceiling(rule.surface) if rule.surface else None      # any allowed-contact surface pair (held beaker or palm)
                    stop = stop_gap(rule.margin, ceiling, rule.work_surface)     # Phase D fix 1: a work surface stops the hand closer to it
                    out[i].append((gap - stop, gap, rule.margin, ceiling, a.id, b.id, nrm, np.asarray(pa, float), stop))
        return out

    def _rules_for(self, allp):
        """Phase J fix 2 (a cache, no decision): the rule of every (hand row, other shape) pair of this shape list, as arrays for the compiled
        loop. Keyed by the rule attributes of the shapes (touch.pair_rule reads only these), so it is rebuilt only when the shape list changes."""
        key = tuple((a.id, a.arm, a.part, a.kind, a.allowed_contact, a.never_touch, a.work_surface, a.support) for a in allp)
        hit = self._rule_cache.get(key)
        if hit is not None:
            return hit
        hand = [s for s, a in enumerate(allp) if a.arm in SIDES and a.part in HAND_SHAPE_PARTS]
        H, n = len(hand), len(allp)
        margin_hn = np.zeros((H, n))
        act_hn = np.zeros((H, n))
        sg_hn = np.zeros((H, n))
        exempt_hn = np.ones((H, n), bool)
        rule_objs = [[None] * n for _ in range(H)]
        for ro, ia in enumerate(hand):
            a = allp[ia]
            for ib, b in enumerate(allp):
                if ib == ia or b.arm == a.arm or (a.kind == "box" and b.kind == "box"):
                    continue
                rule = self._rule_of(a, b)
                if isinstance(rule, Exempt):
                    continue
                exempt_hn[ro, ib] = False
                margin_hn[ro, ib] = rule.margin
                act_hn[ro, ib] = rule.activation
                sg_hn[ro, ib] = _stop_of(rule)
                rule_objs[ro][ib] = rule
        built = (np.array(hand, int), np.array([SIDES.index(allp[s].arm) for s in hand], int), margin_hn, act_hn, sg_hn, exempt_hn, rule_objs)
        if len(self._rule_cache) > 16:
            self._rule_cache.clear()
        self._rule_cache[key] = built
        return built

    def _pairs_compiled(self, allp, arr, lo, hi):
        """Phase J fix 2 (SF-93): the candidate loop as one compiled function (pair_jit.look_kernel). The candidates it returns are turned
        into the same tuples as the Python loop, in the same order within each arm."""
        kind, armc, A, B, r, C, R, h = arr
        rows, rows_arm, margin_hn, act_hn, sg_hn, exempt_hn, rule_objs = self._rules_for(allp)
        n = len(allp)
        excl = np.zeros((2, n), bool)
        for i, side in enumerate(SIDES):
            if self._excluded[side]:
                for sidx, a in enumerate(allp):
                    if a.id in self._excluded[side]:
                        excl[i, sidx] = True
        wake_on = self.wake is not None
        wake = np.asarray(self.wake if wake_on else np.zeros(2), float)
        maxm = float(self._max_margin)
        t_static = max(maxm + cfg.PUSH_START_BEYOND_M, float(wake_dist()))
        (cnt, ro_o, ib_o, arm_o, _sep, gap_o, pa_o, pb_o, push_o, considered) = look_kernel(
            rows, rows_arm, kind, armc, A, B, r, C, R, h, lo, hi, excl, margin_hn, act_hn, sg_hn, exempt_hn,
            wake_on, wake, np.asarray(self.hold_arms, bool), float(t_static), maxm + float(cfg.PUSH_START_BEYOND_M),
            maxm + float(cfg.PUSH_STOP_BEYOND_M))
        self.n_considered += int(considered)
        out = [[], []]
        for k in range(int(cnt)):
            ro, ib, i = int(ro_o[k]), int(ib_o[k]), int(arm_o[k])
            a = allp[int(rows[ro])]
            b = allp[ib]
            rule = rule_objs[ro][ib]
            gap = float(gap_o[k])
            pa, pb, push = pa_o[k], pb_o[k], push_o[k]
            d_ = np.asarray(pb, float) - np.asarray(pa, float)               # Phase C step 6: the direction from the hand toward the obstacle
            if np.linalg.norm(d_) > 1e-9:
                nrm = d_ / np.linalg.norm(d_)
            else:
                nrm = -np.asarray(push, float) / max(np.linalg.norm(push), 1e-12)
            ceiling = cfg.approach_ceiling(rule.surface) if rule.surface else None
            stop = stop_gap(rule.margin, ceiling, rule.work_surface)
            out[i].append((gap - stop, gap, rule.margin, ceiling, a.id, b.id, nrm, np.asarray(pa, float), stop))
        return out

    def __call__(self, q, state):
        """For each arm, every candidate pair at pose q as (gap, margin, ceiling or None, hand shape id, other shape id, direction, closest hand point, stop gap), nearest room first."""
        out = [[], []]
        for i, cands in enumerate(self.pairs(q, state)):
            out[i] = [(c[1], c[2], c[3], c[4], c[5], c[6], c[7], c[8]) for c in sorted(cands, key=lambda c: c[0])]
        return out


def stop_gap(margin, ceiling, work_surface=False):
    """Phase C step 4 (David): the gap at which the hand stops, measured from the modelled surface. Plain obstacle pairs stop just outside the
    resolver's push-out zone (margin + PUSH_STOP_BEYOND_M: the gap rule leaves a pair alone beyond its release distance); an allowed-contact
    surface (a ceiling pair) is approached up to its margin boundary, which the palm and the held beaker both reach by design."""
    if ceiling is not None:
        return margin
    # Phase D fix 1 (David): a work surface (the bench top) has the tighter push-stop PUSH_STOP_BEYOND_M_WORK (stop target 1.5 cm)
    return margin + (cfg.PUSH_STOP_BEYOND_M_WORK if work_surface else cfg.PUSH_STOP_BEYOND_M)


def _aabb(sh):
    return sh.aabb()


class BrakingLayer:
    def __init__(self, tool_fn, tun=None, bridge=None):
        """tool_fn(q (2, 7), state) -> (2, 3). tun: tunables (default config.BRAKING); the derived constants are computed once here."""
        self.tool_fn = tool_fn
        self.tun = cfg.BRAKING if tun is None else tun
        self.d = cfg.derive(self.tun, bridge)
        self.dt = self.d["tick_dt_s"]
        self._prev_pos = None          # (2, 3) tool points of the previous measure() call
        self._prev_speed = None        # (2,) speeds of the previous measure() call
        self.last_measure = None
        self.ramp = JerkRamp(self.d["effective_accel_limit"], float(self.tun["jerk_limit_m_s3"]), self.dt)
        self.v = np.zeros(2)           # profile speed of each arm (m/s), the speed of the last command this layer produced
        self.a = np.zeros(2)           # profile acceleration of each arm (m/s^2)
        self._braking = np.zeros(2, bool)   # look-ahead latch: this arm is braking for an obstacle
        self._emergency = np.zeros(2, bool)
        self._latch_key = [None, None]         # Phase I fix 1 (SF-89): per arm, the (hand shape, other shape) pair that latched the stop
        self._hold_arms = np.zeros(2, bool)    # Phase F fix 6 (SF-83): the arms of this tick that are held (the HOLD trigger of process)
        self._hard_latch = np.zeros(2, bool)   # Phase D fix 3: an arm already hard-clamped (its error is logged once, when the overspeed starts)
        self._hard_pose = np.zeros((2, 7))     # Phase D fix 3: the pose each hard-clamped arm holds (the one it had when the overspeed started)
        self.errors = []                    # error log: every override (the look-ahead failed to catch an obstacle early enough: a bug to investigate)
        self._ceiling = np.zeros(2, bool)   # look-ahead latch: this arm is held to an allowed-contact surface's approach ceiling
        self._prev_pa = {}                  # (arm, hand shape, other shape) -> the closest hand point of the previous tick (Phase C closing component)
        self._speed_hist = deque(maxlen=ACCEL_WINDOW_TICKS + 1)   # measured speeds of the last ticks (the acceleration window)
        self._tool_prev = None              # tool points of the last sent pose (the hand velocity when the jerk source is "sent")
        self._shaping = np.zeros(2, bool)   # this arm's command is going through the S-curve (it stays so until the ramp has converged on the commanded speed)
        self._dir = np.zeros((2, 7))   # unit joint direction of each arm's last step (the way it was going; used when stopping)
        self._src_ref = [None, None]   # S3 (SF-111, Phase S): per arm, the last DIFFERENT source command (7,), None before the first
        self._src_step = np.zeros((2, 7))   # S3: per arm, the step from the last different command to the one before it (the arm's motion)
        self._rev_pending = np.zeros(2, bool)   # S3: per arm, a source reversal seen while braking; the stop releases once no closing pair needs it

    # ---- step 2: hand speed ----
    def measure(self, q, state, call_dt=None):
        """Measured hand speed: |tool point now - tool point at the previous call| / dt, per arm (world frame: a moving base counts, as decided
        in Phase A). call_dt (s, the time since the previous call, None = unknown): a gap of more than 1.5 ticks starts a new history, because
        the displacement over a gap is not a speed (integration run brkV: the base drove 2.3 m during a 38 s gap between two filter calls; the
        old measure divided that by one tick and read 395 m/s, and the hard clamp fired on it). Acceleration: least-squares slope of the speed
        over ACCEL_WINDOW_TICKS ticks. Returns {"pos", "vel", "speed", "accel"}; the first call (or the first after a gap) gives zeros."""
        if call_dt is not None and float(call_dt) > 1.5 * self.dt:
            self._prev_pos = None
        pos = np.array(self.tool_fn(np.asarray(q, float), state), float)
        if self._prev_pos is None:
            vel = np.zeros((2, 3))
            speed = np.zeros(2)
            acc = np.zeros(2)
        else:
            vel = (pos - self._prev_pos) / self.dt
            speed = np.linalg.norm(vel, axis=1)
            # Phase C fix: the acceleration is the average over the last ACCEL_WINDOW_TICKS ticks, not the difference of two neighbouring
            # speeds: the measured speed is noisy at the tick scale, and a noise spike fed the cautious stopping formula as a large
            # acceleration (false look-ahead emergencies in run brkR_plain)
            self._speed_hist.append(speed.copy())
            n = len(self._speed_hist)
            if n > 2:                                   # the least-squares slope of the speed over the window (noise averages out)
                tt = (np.arange(n) - (n - 1) / 2.0) * self.dt
                hist = np.array(self._speed_hist)
                acc = (tt[:, None] * (hist - hist.mean(axis=0))).sum(axis=0) / (tt @ tt)
            else:
                acc = (speed - self._prev_speed) / self.dt
        if self._prev_pos is None:
            self._speed_hist.clear()
            self._speed_hist.append(speed.copy())
        self._prev_pos, self._prev_speed = pos, speed
        self.last_measure = {"pos": pos, "vel": vel, "speed": speed, "accel": acc}
        return self.last_measure

    def commanded_speed(self, last, cmd, state):
        """Speed of the commanded change last -> cmd at the tool point, per arm (2,): |FK(cmd) - FK(last)| / dt. The base is not in it:
        arms and base never move at the same time in an episode (exclusivity on)."""
        a = np.asarray(self.tool_fn(np.asarray(last, float), state), float)
        b = np.asarray(self.tool_fn(np.asarray(cmd, float), state), float)
        return np.linalg.norm(b - a, axis=1) / self.dt

    def _hand_velocity(self, last, state):
        """Velocity of the tool point, (2, 3) m/s, world frame: from the measured joints when the jerk source is "state" (the measure() of this
        tick), else from the sent command (the tool point of the last sent pose against the one before). First call: zeros."""
        if self.tun["speed_source"] == "state" and self.last_measure is not None:
            return np.array(self.last_measure["vel"], float)
        pos = np.asarray(self.tool_fn(np.asarray(last, float), state), float)
        vel = np.zeros((2, 3)) if self._tool_prev is None else (pos - self._tool_prev) / self.dt
        self._tool_prev = pos
        return vel

    def reset(self):
        self._prev_pos = self._prev_speed = self.last_measure = None
        self._tool_prev = None
        self.v[:] = 0.0
        self.a[:] = 0.0
        self._dir[:] = 0.0
        self._braking[:] = False
        self._latch_key = [None, None]
        self._ceiling[:] = False
        self._prev_pa = {}
        self._shaping[:] = False
        self._emergency = np.zeros(2, bool)
        self._hard_latch[:] = False
        self._hard_pose[:] = 0.0
        self._src_ref = [None, None]
        self._src_step[:] = 0.0
        self._rev_pending[:] = False

    # ---- step 3: hand speed cap ----
    def scale_step(self, i, q_from, step, speed, state):
        """Scales the joint step of arm i (7,) by one factor so that the tool point moves at `speed` m/s over this tick (speed 0 -> no
        motion). Forward kinematics is nonlinear, so the factor is corrected a few times; it never ends above the wanted speed.
        Returns the scaled step (7,)."""
        step = np.asarray(step, float)
        if speed <= 0.0 or not np.any(step):
            return np.zeros(7)
        base = np.array(q_from, float)

        def moved(s):
            q = base.copy()
            q[i] = base[i] + s * step
            a = self.tool_fn(base, state)[i]
            return float(np.linalg.norm(self.tool_fn(q, state)[i] - a)) / self.dt

        s = 1.0
        for _ in range(8):
            v = moved(s)
            if v < 1e-12:
                break
            if abs(v - speed) <= 1e-6 * speed:
                break
            s *= speed / v
        while moved(s) > speed * (1.0 + 1e-6) and s > 1e-12:    # never above the wanted speed
            s *= 0.999
        return s * step

    def cap(self, last, raw, state):
        """The hand speed cap: for each arm whose commanded tool speed exceeds hand_speed_cap_m_s, all its joint changes are scaled back
        proportionally so the tool point moves at the cap. Returns (command (2, 7), capped (2,) bool)."""
        last, raw = np.asarray(last, float), np.asarray(raw, float)
        out = raw.copy()
        capped = np.zeros(2, bool)
        vs = self.commanded_speed(last, raw, state)
        capv = float(self.tun["hand_speed_cap_m_s"])
        for i in range(2):
            if vs[i] > capv:
                out[i] = last[i] + self.scale_step(i, last, raw[i] - last[i], capv, state)
                capped[i] = True
        return out, capped

    # ---- step 4: S-curve profile with the jerk limit ----
    def _va(self, i):
        """Speed and acceleration the ramp starts from. The profile (the layer's own speed and acceleration) always carries the ramp; with
        speed_source "state" the measured tool speed is used when it is FASTER than the profile (the arm is moving faster than the layer
        thinks: a safety fact, taken at its word). Phase G fix 2 (David): the acceleration is always the profile's, never the measured one. Phase C fix: a measured speed that is simply
        zero (the arm has not moved yet) used to restart the ramp every tick, so a stationary arm could never accelerate (replay of run
        brkP_plain: the output stayed still while the source moved 2 rad away)."""
        v, a = float(self.v[i]), float(self.a[i])
        if self.tun["speed_source"] == "state" and self.last_measure is not None:
            mv = float(self.last_measure["speed"][i])
            if mv > v:
                v = mv                                    # the measured speed is a bound on the speed only (Phase G fix 2, David)
        return v, a                                       # the acceleration is always the profile's: the measured acceleration is noisy and
                                                          # reverses when the arm does, which made the distance-aware cap alternate (the 68 s chatter)

    def _accel_of(self, i, vs):
        """Acceleration of arm i if its speed goes from the last sent speed to vs (m/s) in one tick, on the speed ABOVE the at-rest level
        (stop_speed_m_s): micro-motion below it is not counted, so leaving rest is not a jump in acceleration."""
        rest = float(self.tun["stop_speed_m_s"])
        return (max(vs - rest, 0.0) - max(float(self.v[i]) - rest, 0.0)) / self.dt

    def _needs_shaping(self, i, vs, stop, speed_limit):
        """True when arm i's command must go through the S-curve this tick: a stop (HOLD or look-ahead), the layer is still shaping (it started
        and has not converged on the commanded speed yet), or the commanded tool speed vs is above the hand speed cap or the look-ahead's
        speed limit. Anything else is free motion under the cap and passes untouched: sources that plan their own motion (the staging move)
        and need their command applied as given keep working, and the bridge keeps interpolating them."""
        lim = float(self.tun["hand_speed_cap_m_s"])
        if speed_limit is not None:
            lim = min(lim, float(speed_limit[i]))
        return bool(stop[i]) or bool(self._shaping[i]) or vs > lim * (1.0 + 1e-9)

    def _hard_arms(self, state, last):
        """Phase D fix 3 (David): the arms whose MEASURED hand speed is above hand_speed_cap_m_s x emergency_speed_multiplier: the last resort,
        when the S-curve and the look-ahead have not held the arm. Needs the measured joints (speed_source "state"). An error is logged when
        an arm's overspeed starts, not on every tick it lasts. Phase H fix 2 (David): the clamp holds the joints the bridge already has (the last
        sent command at onset), so it sends zero step; the measured joints lag the command and were up to 0.76 rad away (brkZ6: the step)."""
        hard = np.zeros(2, bool)
        if self.tun["speed_source"] != "state" or self.last_measure is None:
            return hard
        mult = float(self.tun["emergency_speed_multiplier"])
        lim = float(self.tun["hand_speed_cap_m_s"]) * mult
        for i in range(2):
            v = float(self.last_measure["speed"][i])
            if v > lim:
                hard[i] = True
                if not self._hard_latch[i]:
                    self._hard_pose[i] = np.asarray(last, float)[i].copy()      # the joints the bridge holds now (the last sent command)
                    msg = (f"hard clamp ({SIDES[i]}): measured hand speed {v:.3f} m/s is above {lim:.3f} m/s (cap x {mult:g}); "
                           "the command is set to the measured joints, no S-curve")
                    self.errors.append(msg)
                    print(f"[braking] ERROR {msg}")
        self._hard_latch = hard.copy()
        return hard

    def _wake_radii(self):
        """Phase F fix 5 (SF-82, David's brief): per arm, the look-ahead's wake radius (m). The current stopping distance from the speed and
        acceleration the ramp starts from (_va), but never below the floor: the stopping distance from the highest speed the ramp can send at
        the next tick (the sent speed rises by at most the effective acceleration times the tick). A pair beyond the radius cannot be reached
        before the arm stops, so the look-ahead does not check it; the release distances and the resolver keep the worst-case cutoffs."""
        a_eff, jl, cush = self.d["effective_accel_limit"], float(self.tun["jerk_limit_m_s3"]), self.d["tick_cushion_m"]
        capv = float(self.tun["hand_speed_cap_m_s"])
        out = np.zeros(2)
        for i in range(2):
            v, a = self._va(i)
            now = stopping_distance(v, min(max(a, 0.0), a_eff), a_eff, jl, cush)
            if self._hold_arms[i]:
                # Phase J fix 4 (SF-95, David): during HOLD the wake is the remaining braking distance (plus the margin, added by the feed):
                # no next-tick floor, so the wake shrinks as the arm decelerates and, once stopped, only the margin stays awake
                out[i] = now
                continue
            v_next = min(capv, max(float(self.v[i]), 0.0) + a_eff * self.dt)
            floor = stopping_distance(v_next, 0.0, a_eff, jl, cush)
            out[i] = max(now, floor)
        return out

    def _stoppable_speed(self, i, dist):
        """Phase E fix 3 (David): the largest speed (m/s, at most the hand cap) from which the ramp can still stop within dist (m) at the
        target: the two-step stopping formula from the arm's current acceleration (eased to zero first). Same math as the obstacle look-ahead,
        applied to the command target. No tick cushion: the stop lands on the target."""
        if dist <= 0.0:
            return 0.0
        capv = float(self.tun["hand_speed_cap_m_s"])
        a_eff, jl = self.d["effective_accel_limit"], float(self.tun["jerk_limit_m_s3"])
        a0 = min(max(float(self.a[i]), 0.0), a_eff)
        if stopping_distance(capv, a0, a_eff, jl, 0.0) <= dist:
            return capv
        lo, hi = 0.0, capv
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if stopping_distance(mid, a0, a_eff, jl, 0.0) <= dist:
                lo = mid
            else:
                hi = mid
        return lo

    def step(self, last, raw, state, stop=None, speed_limit=None):
        """One tick of the profile. Each arm's speed moves toward its target through the jerk-limited ramp (S-shaped both ways); the joint
        step is the commanded direction scaled to the ramp's speed.
        stop: (2,) bool or None; True = target speed 0 along the way the arm was going (a stop; the command is not followed).
        speed_limit: (2,) extra speed limit per arm (m/s) or None (used by the look-ahead for allowed-contact surfaces).
        Otherwise the target is min(commanded speed, hand speed cap); the arm never goes past the commanded pose. Returns (command (2,7), info)."""
        last, raw = np.asarray(last, float), np.asarray(raw, float)
        out = last.copy()
        stop = np.zeros(2, bool) if stop is None else np.asarray(stop, bool)
        vs = self.commanded_speed(last, raw, state)
        capv = float(self.tun["hand_speed_cap_m_s"])
        for i in range(2):
            v0, a0 = self._va(i)
            if stop[i]:
                # Phase J fix 1 (SF-92, David): every tick of a stop (the start and the ongoing stop) starts the ramp from the profile's planned speed,
                # not the measured speed. Phase I fix 3 (SF-91) did this on the start tick only; the ongoing stop still used the measured speed, which
                # gave the 2.47 and 1.35 m/s2 sent-speed spikes of brkZ8 (the measured joint drifts while the profile is near rest).
                v0, a0 = float(self.v[i]), float(self.a[i])
            delta = raw[i] - last[i]
            if not self.tun.get("shape_free_motion", True) and not self._needs_shaping(i, float(vs[i]), stop, speed_limit):
                out[i] = raw[i]                           # free motion under the cap: sent as it is, to the last digit
                self.a[i] = self._accel_of(i, float(vs[i]))
                self.v[i] = float(vs[i])
                n = np.linalg.norm(delta)
                if n > 0:
                    self._dir[i] = delta / n
                continue
            stop_cap = None
            if stop[i] or not np.any(delta):
                direction = self._dir[i]
                target = 0.0 if stop[i] else 0.0
            else:
                direction = delta
                target = min(float(vs[i]), capv)
                stop_cap = self._stoppable_speed(i, float(vs[i]) * self.dt)     # Phase E fix 3: never faster than can still stop at the target (vs * dt is the tool distance left)
                target = min(target, stop_cap)
            if speed_limit is not None:
                target = min(target, float(speed_limit[i]))
            v1, a1 = self.ramp.step(v0, a0, target)
            # Phase D fix 2 (David): the hand speed cap holds on every tick, on the step itself, not only on the target. The ramp can only brake
            # at the acceleration limit, so a start above the cap (the measured speed of a fast arm) was sent at the ramp's speed: run brkU sent
            # 0.688 m/s with the cap at 0.20. A step faster than the cap is scaled back to it; the profile follows. (The approach ceilings of
            # allowed-contact surfaces are NOT clamped here: they are reached on the S-curve, as before.)
            if v1 > capv:
                v1 = capv
                a1 = float(np.clip((v1 - v0) / self.dt, -self.d["effective_accel_limit"], self.d["effective_accel_limit"]))
            if stop_cap is not None and v1 > stop_cap:          # Phase E fix 3 (David): the ramp's speed capped to the stoppable speed of this tick (the ramp follows a falling target one tick late)
                v1 = stop_cap
                a1 = float(np.clip((v1 - v0) / self.dt, -self.d["effective_accel_limit"], self.d["effective_accel_limit"]))
            if not np.any(direction):
                v1, a1 = 0.0, 0.0
            step = self.scale_step(i, last, direction, v1, state) if v1 > 0.0 else np.zeros(7)
            if not stop[i] and np.any(step) and np.linalg.norm(step) > np.linalg.norm(delta):   # never past the commanded pose
                step = delta
            capped_target = target < float(vs[i]) * (1.0 - 1e-12)
            self._shaping[i] = bool(stop[i] or capped_target or abs(v1 - target) > 1e-12 or abs(a1) > 1e-12)   # converged on the commanded speed: free motion again
            clipped = (not stop[i]) and np.any(delta) and v1 > 0.0 and np.array_equal(step, delta)
            out[i] = last[i] + step
            n = np.linalg.norm(step)
            if n > 0:
                self._dir[i] = step / n
            if clipped:
                # the step was cut to the commanded pose: the profile must follow what was really sent, or the next tick could jump to the old
                # profile speed in one step (breaking the acceleration limit)
                q_new = last.copy()
                q_new[i] = out[i]
                v_act = float(self.commanded_speed(last, q_new, state)[i])
                # Phase F (chatter diagnosis): the acceleration is limited like every other branch (it was unlimited: a step cut to the pose could set
                # an acceleration far above the limit)
                v1, a1 = v_act, float(np.clip((v_act - v0) / self.dt, -self.d["effective_accel_limit"], self.d["effective_accel_limit"]))
                self._shaping[i] = False                  # the ramp allowed more than the command asked for: the command goes through as it is
            self.v[i], self.a[i] = v1, a1
        return out, {"speed": self.v.copy(), "accel": self.a.copy()}

    # ---- step 5: look-ahead ----
    @staticmethod
    def _pairs_of(entry):
        """An obstacle_fn entry for one arm as a list of (gap, margin, ceiling, id_a, id_b, direction toward the obstacle or None): None -> [],
        one tuple -> [tuple], a list stays. Entries of 5 values (no direction) are accepted: the whole hand speed is then used."""
        if entry is None:
            return []
        items = [entry] if (len(entry) and not isinstance(entry[0], (tuple, list))) else list(entry)
        out = []
        for e in items:
            e = tuple(e)
            nrm = np.asarray(e[5], float) if len(e) > 5 and e[5] is not None else None
            pa = np.asarray(e[6], float) if len(e) > 6 and e[6] is not None else None
            ceil_e = e[2] if len(e) > 2 else None
            stop = float(e[7]) if len(e) > 7 and e[7] is not None else stop_gap(float(e[1]), ceil_e)   # the stop gap (8th value); a 7-value entry gets the default
            out.append((float(e[0]), float(e[1]), ceil_e, e[3] if len(e) > 3 else "", e[4] if len(e) > 4 else "", nrm, pa, stop))
        return out

    def _track_source(self, raw):
        """Phase S, S3 (SF-111): per arm, this tick's source command against the last DIFFERENT source command (some joint more than
        BRAKING src_diff_tol_rad away from it), not the previous tick's. A repeated target between chunk updates is not a new command, so it
        cannot hide a reversal or delay the release. Returns (2,) bool: this tick's new command reverses the arm's motion (the step to it
        opposes the step that brought the arm to the last different command)."""
        tol = float(self.tun["src_diff_tol_rad"])
        rev = np.zeros(2, bool)
        for i in range(2):
            cur = np.asarray(raw[i], float)
            ref = self._src_ref[i]
            if ref is None:
                self._src_ref[i] = cur.copy()
                continue
            step = cur - ref
            if float(np.max(np.abs(step))) <= tol:                  # a repeat of the last different command: not a new command
                continue
            rev[i] = bool(np.dot(step, self._src_step[i]) < 0.0)
            self._src_step[i] = step
            self._src_ref[i] = cur.copy()
        return rev

    def check(self, last, raw, state, obstacle_fn, vel=None):
        """The look-ahead, part of the safety check: for each arm and each obstacle pair, can the hand still brake from its current speed and
        acceleration to the stop target (stop_gap: margin + PUSH_STOP_BEYOND_M from the modelled surface, margin + PUSH_STOP_BEYOND_M_WORK on a work surface, or the margin for an allowed-contact surface) within the gap?
        obstacle_fn(q (2,7), state) -> per arm a list of (gap_m, margin_m, ceiling_m_s or None, hand shape id, other shape id) (ObstacleFeed; a
        single tuple or None is accepted): every pair within worst_stop_dist_m is in it. A ceiling marks an allowed-contact surface (zero margin).
        A pair counts when the wanted command is closing on it (its gap at the command is smaller than at the last command). The speed used for a
        pair is the hand speed (config lookahead_speed "full") or its closing component, the rate the gap shrinks at the wanted command, never
        more than the hand speed ("closing"). An arm brakes when, for any closing pair, the stopping distance (config two-step formula + tick
        cushion) reaches the distance to the target. A braking arm stays braking while a closing pair is left, and is released when none closes
        any more, or when it has stopped with room left (an early stop). A hand that goes faster than its stopping distance allows while it is
        really moving (above stop_speed_m_s) has failed the look-ahead: that is the emergency (the caller overrides the bridge and logs an error).
        For an allowed-contact surface the hand is not stopped: when the distance needed to slow from the current speed to the ceiling reaches
        the distance to the surface, the speed target becomes the ceiling (through the same jerk-limited ramp).
        Returns (stop (2,) bool, limit (2,) speed limit in m/s (inf = none), info list of dicts)."""
        last, raw = np.asarray(last, float), np.asarray(raw, float)
        rev = self._track_source(raw)                  # S3 (SF-111): the source's reversals, against its last different command
        if hasattr(obstacle_fn, "set_wake"):          # Phase F fix 5/6 (SF-82, SF-83): this tick's wake radii and HOLD arms (the feed's pairs)
            wake = self._wake_radii() if self.tun.get("adaptive_wake", True) else None
            obstacle_fn.set_wake(wake, self._hold_arms)
        at_last = obstacle_fn(last, state)
        at_raw = obstacle_fn(raw, state)
        eps = float(self.tun["closing_eps_m"])
        rest = float(self.tun["stop_speed_m_s"])
        mode = self.tun["lookahead_speed"]
        a_eff, jl, cush = self.d["effective_accel_limit"], float(self.tun["jerk_limit_m_s3"]), self.d["tick_cushion_m"]
        capv_hand = float(self.tun["hand_speed_cap_m_s"])
        vs_all = self.commanded_speed(last, raw, state)     # Phase E fix 2: the commanded tool speed of each arm (the closing share of the ramp's motion)
        stop = np.zeros(2, bool)
        limit = np.full(2, np.inf)
        emergency = np.zeros(2, bool)
        self._emergency = emergency
        info = [None, None]
        for i in range(2):
            pl = self._pairs_of(at_last[i])
            if not pl:
                self._braking[i] = self._ceiling[i] = False
                self._latch_key[i] = None
                self._rev_pending[i] = False
                continue
            raw_by = {(p[3], p[4]): p for p in self._pairs_of(at_raw[i])}
            v, a = self._va(i)
            a = min(max(a, 0.0), a_eff)             # the acceleration of the formula never exceeds the effective limit (the worst case the derived constants assume); a per-tick difference of pulsed commands can be far larger
            need_brake, brake_pairs, ceil_pairs, need_ceiling, worst = False, [], [], False, None
            closing_now, trig_key = {}, None         # Phase I fix 1: closing of every pair this tick; the pair that latched the stop
            for gap, margin, ceiling, ida, idb, nrm, pa, sg in pl:      # sg: this pair's stop gap (the array 'stop' above is the per-arm stop flag)
                pr = raw_by.get((ida, idb))
                closing = pr is not None and pr[0] < gap - eps
                closing_now[(ida, idb)] = closing
                room = gap - sg     # Phase C step 4 (David): the stop target is just outside the resolver's push-out zone, not the midpoint (0 for the decided zero margin)
                prev_pa = self._prev_pa.get((i, ida, idb))
                if mode == "full" or nrm is None:
                    vp = v                                 # the whole hand speed (the cautious reading, and the only one without a direction)
                elif pa is not None and prev_pa is not None:
                    # Phase C fix (David's closing component, at the point of the hand nearest this obstacle): the velocity of that point is
                    # the finite difference of its position over the last tick. The tool point's velocity underestimates the approach of
                    # other hand points (the palm met the bench faster than the tool point moved: false emergencies in run brkT)
                    vp = max(0.0, float(np.dot((pa - prev_pa) / self.dt, nrm)))
                elif vel is not None:
                    vp = max(0.0, float(np.dot(vel[i], nrm)))   # the tool point's velocity (the first tick of a pair)
                else:
                    vp = v
                # Phase E fix 2 (David): the look-ahead uses the speed the ramp WILL reach, not the measured closing speed alone. The ramp's own
                # speed v (the profile, or the measured speed when that is faster) leads a lagging arm; its share along the pair's closing
                # direction is the commanded closing rate over the commanded speed. The two-step formula then eases the current acceleration to
                # zero before braking (it adds a^2/(2 j) to that speed). The measured closing speed still counts when it is higher.
                closing_rate = max(0.0, gap - pr[0]) / self.dt if pr is not None else 0.0
                vs_i = float(vs_all[i])
                share = min(1.0, closing_rate / vs_i) if vs_i > 1e-9 else 1.0
                # Phase G fix 3 (David): an arm-to-arm pair (the other arm's shapes) is reached by the closing component only, not by the whole
                # ramp speed scaled by the closing share (the share saturates at 1 when the commanded closing rate of the pair reaches the commanded
                # tool speed, which is how the whole speed kept the left arm held during BACK). The other arm and the held beaker stay obstacles.
                arm_to_arm = str(idb).startswith("right_" if i == 0 else "left_")
                v_reach = max(vp, v * share) if (closing and not arm_to_arm) else vp
                sd = stopping_distance(v_reach, a, a_eff, jl, cush)
                if worst is None or room < worst["room"]:
                    worst = {"gap": gap, "margin": margin, "ceiling": ceiling, "stop_dist": sd, "room": room, "closing": closing, "pair": f"{ida}|{idb}", "speed": v}
                if not closing:
                    continue
                if ceiling is None:
                    brake_pairs.append(room)
                    if sd >= room:
                        need_brake = True
                        if trig_key is None:
                            trig_key = (ida, idb)
                        hard_room = gap - margin     # Phase C fix: an emergency is a stop that would ENTER the margin, not one that overshoots the stop target
                        finger_pair = "finger" in str(ida) or "finger" in str(idb)     # Phase F fix 1 (David): finger motion is gripper actuation, not arm approach: no emergency from a finger pair
                        if sd - hard_room > float(self.tun["emergency_overshoot_ticks"]) * cush and vp > rest and not finger_pair:    # Phase E fix 1 (David): also while already braking: a stop that cannot fit in the margin is an emergency
                            emergency[i] = True          # even the S-curve cannot stop in time: braking must exceed its limits
                            self.errors.append(f"look-ahead failed ({SIDES[i]}): stopping distance {sd:.4f} m exceeds the room to the margin {hard_room:.4f} m "
                                               f"(speed {vp:.3f} m/s, gap {gap:.4f} m, margin {margin:.4f} m, pair {ida}|{idb})")
                            print(f"[braking] ERROR {self.errors[-1]}: braking beyond the limits, overriding the bridge")
                else:
                    ceil_pairs.append(float(ceiling))
                    if v_reach > float(ceiling) and slowing_distance(v_reach, a, float(ceiling), a_eff, jl, cush) >= room:     # Phase E fix 2: the reachable speed
                        need_ceiling = True
            commanded_motion = bool(np.any(raw[i] != last[i]))     # a tick where the source commands no motion says nothing about whether the obstacle cleared
            latched = self._latch_key[i]                 # Phase I fix 1 (David): the pair that latched the stop
            latch_closing = closing_now.get(latched, False) if latched is not None else False
            if rev[i] and self._braking[i]:              # S3 (SF-111): the source reversed while this arm is braking: the stop releases as soon as no closing pair needs it
                self._rev_pending[i] = True
            if need_brake:
                self._braking[i] = True
                if latched is None or not latch_closing:
                    self._latch_key[i] = trig_key
            elif self._braking[i] and self._rev_pending[i] and not latch_closing:
                self._braking[i] = False                 # S3 (SF-111): a reversal of the source releases the stop once the latched pair no longer closes on the command (no delay for repeated targets)
                self._latch_key[i] = None
                self._rev_pending[i] = False
            elif self._braking[i] and latched is not None and not latch_closing and commanded_motion:
                self._braking[i] = False                 # Phase I fix 1 (David): the source now moves away from the obstacle that latched the stop: release on this tick
                self._latch_key[i] = None
            elif self._braking[i] and ((not brake_pairs and commanded_motion) or (v <= rest and brake_pairs and all(r_ > cush for r_ in brake_pairs))):
                self._braking[i] = False                 # nothing closes any more, or stopped with room left (an early stop): speed up again on the S-curve
                self._latch_key[i] = None
            if not self._braking[i]:
                self._rev_pending[i] = False
            if need_ceiling:
                self._ceiling[i] = True
            elif self._ceiling[i] and not ceil_pairs:
                self._ceiling[i] = False
            if self._ceiling[i] and ceil_pairs:
                limit[i] = min(ceil_pairs)
            stop[i] = self._braking[i]
            info[i] = {**worst, "braking": bool(self._braking[i]), "ceiling_active": bool(self._ceiling[i]),
                       "surface_gap": min([p[0] for p in pl if p[2] is not None], default=float("nan"))}
        self._prev_pa = {(i, ida, idb): pa for i in range(2) for (_g, _m, _c, ida, idb, _n, pa, _s) in self._pairs_of(at_last[i]) if pa is not None}
        return stop, limit, info

    def process(self, last, raw, state, obstacle_fn=None, hold=False, call_dt=None):
        """The whole layer for one tick: look-ahead, hand speed cap and S-curve. hold: True (or (2,) bool) = a HOLD trigger (collision,
        timeout, error): both arms follow the S-curve to rest instead of following the command. Returns (command (2,7), info dict)."""
        if self.tun["speed_source"] == "state" and hasattr(state, "q_left"):
            self.measure(np.array([state.q_left, state.q_right], float), state, call_dt=call_dt)      # the measured joints feed the jerk limiting and the hard clamp
        hard = self._hard_arms(state, last)               # Phase D fix 3: measured overspeed (the last resort); Phase H fix 2: holds the last sent joints
        stop = np.zeros(2, bool)
        limit = None
        info = [None, None]
        vel = self._hand_velocity(last, state)
        self._hold_arms = np.broadcast_to(np.asarray(hold, bool), (2,)).copy()      # Phase F fix 6 (SF-83): the held arms of this tick
        if obstacle_fn is not None:
            stop, limit, info = self.check(last, raw, state, obstacle_fn, vel=vel)
        else:
            self._track_source(np.asarray(raw, float))         # S3: keep the last different command current without a look-ahead
        stop = stop | np.asarray(hold, bool)
        out, prof = self.step(last, raw, state, stop=stop, speed_limit=limit)
        override = np.zeros(2, bool)
        if obstacle_fn is not None:
            override = self._emergency.copy()
            for i in np.flatnonzero(override):               # Part 2: braking beyond the limits: stop now (no ramp), the loop overrides the bridge
                out[i] = np.asarray(last, float)[i]
                self.v[i] = self.a[i] = 0.0
                prof = {"speed": self.v.copy(), "accel": self.a.copy()}
        for i in np.flatnonzero(hard):                   # Phase D fix 3: the measured joints, at once, no S-curve; the loop overrides the bridge
            out[i] = self._hard_pose[i]                   # the pose at the onset of the overspeed (no tracking of the moving arm: it is a stop)
            self.v[i] = self.a[i] = 0.0
            self._braking[i] = self._shaping[i] = False
            override[i] = True
            prof = {"speed": self.v.copy(), "accel": self.a.copy()}
        return out, {"stop": stop, "limit": limit, "lookahead": info, "override": override, **prof}
