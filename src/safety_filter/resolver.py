"""The resolver (Stage 2; M293 to M295, SF-28 to SF-39): from a raw arm command to the filtered command, per control tick.

Inputs per tick: raw command and last filtered command (each a (2, 7) array: left arm, right arm, rad), the robot state
for everything but the arm joints (base, pedestal, fingers), the world objects the world model gives, and the held
objects. The tick length dt is given at construction (the caller reads it from the control config, 1 / source_hz).
Output: (filtered command, FilterStatus).

1. Pass-through: no pair needs action -> the raw array itself comes back, untouched (status PASS).
2. A pair needs action when its gap falls short of what the gap rule allows for this tick (gap_rule.shortfall) by more than
   trigger_m AND an arm of the pair is closing (moving only that arm to its raw command makes the gap smaller).
   An arm that is not closing is never acted on (a parked arm stays parked).
3. Rounds (up to RESOLVER["rounds"]): each round fixes only the single worst pair, then every pair is checked again; the
   rounds stop when the worst shortfall is under stop_m. Per closing arm of the worst pair, by the category of its contact
   shape (link_rules): hand -> hand slide; upper arm / forearm -> elbow swing; shoulder, or a point an elbow swing cannot
   move -> step shortening. Both arms closing: each takes half of the push.
4. Checks after any push (branch guard, shape re-check, joint limits, accuracy); a failure holds still: the last filtered
   command comes back with status HOLD and a reason code.

Robot state may come from the simulator; the resolver never reads world data from it (it takes world objects as numbers).
"""

from dataclasses import dataclass, field

import numpy as np

from src.safety_filter import config as cfg
from src.safety_filter import gap_rule
from src.safety_filter.arm_conversion import ArmKinematics, rotation_error_rad, with_arms
from src.safety_filter.link_rules import FOREARM, HAND, UPPER_ARM, category_of
from src.safety_filter.push_direction import contact_fraction, elbow_displacement, pair_push
from src.safety_filter.shapes import SIDES, RobotState
from src.safety_filter.touch import Exempt, as_placed, evaluate_pairs, gap_between, pair_rule

PASS, ELBOW_SWING, HAND_SLIDE, STEP_SHORTENED, HOLD, KONG_PUSH = "PASS", "ELBOW_SWING", "HAND_SLIDE", "STEP_SHORTENED", "HOLD", "KONG_PUSH"
STATUSES = (PASS, ELBOW_SWING, HAND_SLIDE, STEP_SHORTENED, HOLD, KONG_PUSH)   # KONG_PUSH last: older recordings keep their indices
MODES = ("layered", "kong_style", "cbf")
HOLD_REASONS = ("BRANCH_GUARD", "SHAPE_RECHECK", "JOINT_LIMITS", "ACCURACY", "SHORTEN_ZERO", "NO_SOLVER", "SOLVER_FAILED", "NOT_FINITE")
_ARM_INDEX = {"left": 0, "right": 1}


@dataclass
class FilterStatus:
    """The status record of one tick. worst_pair: the pair that was worst at the raw command (ids), worst_gap (m) its gap
    there, worst_shortfall (m) how far short of the gap rule it was. rounds: rounds used. hold_reason: "" unless HOLD.
    methods: the moves used. fraction: the step fraction of a step shortening (nan if none)."""
    status: str = PASS
    rounds: int = 0
    worst_pair: tuple = None
    worst_gap: float = float("nan")
    worst_shortfall: float = 0.0
    hold_reason: str = ""
    methods: tuple = ()
    fraction: float = float("nan")
    hand_targets: dict = field(default_factory=dict)   # side -> (position (3,), rotation (3,3)): the pose the resolver aimed the hand at (empty on PASS, HOLD and for shortened arms)


class _Hold(Exception):
    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


class _Tick:
    """The state of one tick: poses (placed shapes), gaps and closing flags, cached."""

    def __init__(self, res, raw, last, state, world, held):
        self.res, self.raw, self.last, self.state, self.held = res, raw, last, state, held
        self.world = {o.id: as_placed(o) for o in world}
        self.world_objects = list(world)
        self._poses, self._gaps = {}, {}
        self.rules, self.g0, self._closing = {}, {}, {}
        self.keys = set()

    # ---- poses ----
    def pose(self, q):
        """(robot shapes by id incl. the held shapes) at the joint angles q (2,7); cached by the bytes of q."""
        k = q.tobytes()
        p = self._poses.get(k)
        if p is None:
            sh = self.res.shapes
            placed = sh.place(with_arms(self.state, q))
            allp = list(placed)
            if self.held is not None:
                for side in SIDES:
                    pos, rot = sh.hand_frame(side)
                    hs = self.held.placed(side, pos, rot)
                    if hs is not None:
                        allp.append(hs)
            p = ({s.id: s for s in allp}, allp)
            if len(self._poses) > 64:
                self._poses.clear()
                self._gaps.clear()
            self._poses[k] = p
        return p

    def shape(self, q, sid):
        by_id, _ = self.pose(q)
        return by_id.get(sid) or self.world.get(sid)

    def gap(self, q, key):
        """Scalar gap of a pair at the pose q (the shared layer's exact distance); +inf if a shape is missing."""
        ck = (q.tobytes(), key)
        g = self._gaps.get(ck)
        if g is None:
            a, b = self.shape(q, key[0]), self.shape(q, key[1])
            g = np.inf if a is None or b is None else float(gap_between(a, b)[0])
            self._gaps[ck] = g
        return g

    # ---- candidate pairs ----
    def discover(self, q):
        """Adds the pairs near at the pose q to the candidate set (with their rules and gaps at the last command)."""
        _, allp = self.pose(q)
        for rep in evaluate_pairs(allp, self.world_objects):
            self._add(tuple(sorted((rep.a, rep.b))), rep.rule)

    def _add(self, key, rule):
        if key not in self.keys:
            self.keys.add(key)
            self.rules[key] = rule
            self.g0[key] = self.gap(self.last, key)

    def add_known(self, key):
        if key not in self.keys:
            a, b = self.shape(self.last, key[0]), self.shape(self.last, key[1])
            if a is None or b is None:
                return
            rule = pair_rule(a, b)
            if not isinstance(rule, Exempt):
                self._add(key, rule)

    def arms_of(self, key):
        out = []
        for sid in key:
            s = self.shape(self.last, sid)
            if s is not None and s.arm in SIDES and s.arm not in out:
                out.append(s.arm)
        return out

    def closing(self, key):
        """The arms of the pair that are closing: moving only that arm to its raw command makes the gap smaller (SF-30)."""
        c = self._closing.get(key)
        if c is None:
            c = []
            for side in self.arms_of(key):
                q = self.last.copy()
                i = _ARM_INDEX[side]
                q[i] = self.raw[i]
                if self.gap(q, key) < self.g0[key] - cfg.RESOLVER["closing_eps_m"]:
                    c.append(side)
            self._closing[key] = c
        return c

    # ---- shortfall ----
    def shortfall(self, q, key):
        return gap_rule.shortfall(self.g0[key], self.gap(q, key), self.rules[key].margin, self.res.dt)

    def worst(self, q, discover=False):
        """(key, shortfall) of the worst pair that needs action at the pose q, or None."""
        if discover:
            self.discover(q)
        best = None
        for key in self.keys:
            s = self.shortfall(q, key)
            if s > cfg.RESOLVER["trigger_m"] and self.closing(key) and (best is None or s > best[1]):
                best = (key, s)
        return best

    def worst_value_discover(self, q):
        """The worst shortfall at the pose q (0.0 if no pair needs action), after adding the pairs near at q."""
        w = self.worst(q, discover=True)
        return 0.0 if w is None else w[1]


class Resolver:
    """The push-apart resolver. Holds the pairs being acted on between ticks (the release band) and nothing else."""

    def __init__(self, shapes, dt, solver=None):
        """Args: shapes (RobotShapes): the robot shapes (the robot-only model; its kinematics are ours).
        dt (float): tick length, s (1 / the control loop's source_hz). solver: the converter for the elbow swing (arm_conversion.make_solver()),
        or None (an elbow swing then holds still with reason NO_SOLVER)."""
        self.shapes, self.dt = shapes, float(dt)
        self.kin = ArmKinematics(shapes, solver)
        self._active = set()

    def reset(self):
        self._active = set()

    # ------------------------------------------------------------------
    def step(self, raw, last, state: RobotState, world_objects=(), held=None):
        """One control tick. raw, last: (2, 7) arrays (rad). Returns (filtered, FilterStatus).
        On PASS the filtered command is the raw array object itself (not a copy).
        Dispatches on config RESOLVER["mode"] (SF-40): "layered" (this file), "kong_style" (kong_style.py), "cbf" (not built)."""
        mode = cfg.RESOLVER["mode"]
        if mode not in MODES:
            raise ValueError(f"RESOLVER['mode'] must be one of {MODES}, not {mode!r}")
        if mode == "cbf":
            raise NotImplementedError("CBF resolver not built")
        if mode == "kong_style":
            from src.safety_filter import kong_style
            return kong_style.step(self, raw, last, state, world_objects, held)
        return self._step_layered(raw, last, state, world_objects, held)

    def _step_layered(self, raw, last, state, world_objects, held):
        last = np.asarray(last, float)
        if not np.isfinite(np.asarray(raw, float)).all():
            return last.copy(), FilterStatus(status=HOLD, hold_reason="NOT_FINITE")
        t = _Tick(self, np.asarray(raw), last, state, world_objects, held)
        t.discover(last)
        t.discover(t.raw)
        for key in self._active:
            t.add_known(key)
        first = t.worst(t.raw)
        if first is None:
            self._update_active(t, t.raw)
            return raw, FilterStatus()
        key0, s0 = first
        st = FilterStatus(worst_pair=key0, worst_gap=t.gap(t.raw, key0), worst_shortfall=s0)
        q = np.array(t.raw, float)
        methods, targets = [], {}
        try:
            rounds = 0
            while rounds < cfg.RESOLVER["rounds"]:
                w = t.worst(q)
                if w is None or (rounds > 0 and w[1] < cfg.RESOLVER["stop_m"]):
                    break
                self._fix(t, q, w, methods, targets, st)
                rounds += 1
            st.rounds = rounds
            if (cfg.RESOLVER["shorten_after_failed_push"] and STEP_SHORTENED not in methods
                    and t.worst_value_discover(q) >= cfg.RESOLVER["stop_m"]):
                changed = [s for s in SIDES if not np.array_equal(q[_ARM_INDEX[s]], t.raw[_ARM_INDEX[s]])]
                if changed:
                    self._shorten(t, q, changed, methods, targets, st)
            reason = self._checks(t, q, targets)
            if reason:
                raise _Hold(reason)
        except _Hold as h:
            st.status, st.hold_reason = HOLD, h.reason
            st.methods = tuple(methods)
            self._update_active(t, last)
            return last.copy(), st
        st.methods = tuple(dict.fromkeys(methods))
        st.hand_targets = {s: (np.array(v[0]), np.array(v[1])) for s, v in targets.items()}
        st.status = (STEP_SHORTENED if STEP_SHORTENED in methods else ELBOW_SWING if ELBOW_SWING in methods
                     else HAND_SLIDE if HAND_SLIDE in methods else PASS)
        self._update_active(t, q)
        return q, st

    # ------------------------------------------------------------------
    def _update_active(self, t, q):
        self._active = {k for k in t.keys if min(t.gap(q, k), t.g0[k]) < t.rules[k].release}

    def _state(self, t, q):
        return with_arms(t.state, q)

    def _fix(self, t, q, worst, methods, targets, st):
        """One round: fix the worst pair for each of its closing arms (the push shared equally)."""
        key, short = worst
        arms = t.closing(key)
        share = short / len(arms)
        shorten = []
        a, b = t.shape(q, key[0]), t.shape(q, key[1])
        pp = pair_push(a, b)             # one snapshot of the pair for every closing arm, so the arms take equal shares
        for side in arms:
            mine, direction, point = (a, pp.dir_a, pp.point_a) if a.arm == side else (b, pp.dir_b, pp.point_b)
            cat = category_of(mine)
            if cat == HAND:
                self._hand_slide(t, q, side, direction, methods, targets)
            elif cat in (UPPER_ARM, FOREARM):
                if not self._elbow_swing(t, q, side, cat, point, direction * share, methods, targets):
                    shorten.append(side)
            else:
                shorten.append(side)
        if shorten:
            self._shorten(t, q, shorten, methods, targets, st)

    # ---- hand slide ----
    def _hand_slide(self, t, q, side, direction, methods, targets):
        i = _ARM_INDEX[side]
        p_last, _ = self.kin.hand_pose(self._state(t, t.last), side)
        p_now, _ = self.kin.hand_pose(self._state(t, q), side)
        _, r_raw = self.kin.hand_pose(self._state(t, t.raw), side)
        into = -np.asarray(direction, float)               # unit vector into the obstacle
        v = p_now - p_last
        along = float(v @ into)
        if along > 0.0:
            v = v - along * into                           # remove only the part pointing into the obstacle
        target = p_last + v
        q[i] = self.kin.jacobian_step(self._state(t, q), side, target, r_raw)
        methods.append(HAND_SLIDE)
        targets[side] = (target, r_raw)

    # ---- elbow swing ----
    def _elbow_swing(self, t, q, side, cat, point, push_vec, methods, targets):
        """Returns False when the swing cannot move this contact (the caller then shortens the step)."""
        i = _ARM_INDEX[side]
        S, E, W = self.kin.sew(self._state(t, q), side)
        axis = W - S
        la = float(np.linalg.norm(axis))
        if la < 1e-9:
            return False
        axis = axis / la
        if np.linalg.norm(np.cross(point - S, axis)) < cfg.RESOLVER["swing_unmovable_m"]:
            return False
        radial = (E - S) - ((E - S) @ axis) * axis
        rho = float(np.linalg.norm(radial))
        if rho < cfg.RESOLVER["swing_straight_m"]:
            return False
        tfrac = contact_fraction(point, S, E) if cat == UPPER_ARM else contact_fraction(point, E, W)
        disp = elbow_displacement(push_vec, tfrac, cat == UPPER_ARM)
        e_t = (E + disp) - S
        r2 = e_t - (e_t @ axis) * axis
        n2 = float(np.linalg.norm(r2))
        if n2 < 1e-9:
            return False
        e_new = S + ((E - S) @ axis) * axis + rho * r2 / n2     # a swing about the S-W line keeps both arm lengths
        u = (e_new - S) / np.linalg.norm(e_new - S)
        l = (W - e_new) / np.linalg.norm(W - e_new)
        p_raw, r_raw = self._swing_hand_target(t, side, targets)
        try:
            qc = self.kin.convert_directions(self._state(t, q), side, u, l, r_raw)
        except RuntimeError as exc:
            raise _Hold("NO_SOLVER" if str(exc).startswith("NO_SOLVER") else "SOLVER_FAILED")
        q2 = q.copy()
        q2[i] = qc
        q[i] = self.kin.jacobian_step(self._state(t, q2), side, p_raw, r_raw)
        methods.append(ELBOW_SWING)
        targets[side] = (p_raw, r_raw)
        return True

    def _swing_hand_target(self, t, side, targets):
        """The hand pose an elbow swing aims at (SF-49): "slid" = the target set earlier in this tick (a hand slide's), else the raw commanded pose."""
        m = cfg.RESOLVER["elbow_swing_hand_target"]
        if m not in ("slid", "raw"):
            raise ValueError(f"RESOLVER['elbow_swing_hand_target'] must be 'slid' or 'raw', not {m!r}")
        if m == "slid" and side in targets:
            return targets[side]
        return self.kin.hand_pose(self._state(t, t.raw), side)

    # ---- step shortening ----
    def _shorten(self, t, q, sides, methods, targets, st, passes=None):
        """Scale the joint-space step (last -> raw) of the given arms by the largest fraction at which every pair meets
        the gap rule (halving search); below shorten_min_fraction: hold still."""
        idx = [_ARM_INDEX[s] for s in sides]

        def trial(f):
            qq = q.copy()
            for i in idx:
                qq[i] = t.last[i] + f * (t.raw[i] - t.last[i])
            return qq

        if passes is None:                       # default pass test: every pair meets the gap rule; kong_style's jump-through guard passes its own
            def passes(f):
                return t.worst_value_discover(trial(f)) < cfg.RESOLVER["stop_m"]

        if passes(1.0):
            f = 1.0
        else:
            lo, hi = 0.0, 1.0
            for _ in range(cfg.RESOLVER["shorten_halvings"]):
                mid = 0.5 * (lo + hi)
                if passes(mid):
                    lo = mid
                else:
                    hi = mid
            f = lo
        if f < cfg.RESOLVER["shorten_min_fraction"]:
            st.fraction = f
            raise _Hold("SHORTEN_ZERO")
        q[:] = trial(f)
        st.fraction = f
        methods.append(STEP_SHORTENED)
        for s in sides:
            targets.pop(s, None)

    # ---- checks ----
    def _checks(self, t, q, targets):
        """The four checks after any push, on the final joint angles. Returns the reason code of the first failure or ''."""
        if not np.isfinite(q).all():
            return "NOT_FINITE"
        changed = [s for s in SIDES if not np.array_equal(q[_ARM_INDEX[s]], t.raw[_ARM_INDEX[s]])]
        joints = list(cfg.RESOLVER["branch_guard_joints"])
        for s in changed:                                                       # a. branch guard
            i = _ARM_INDEX[s]
            if np.max(np.abs(q[i][joints] - t.last[i][joints])) > cfg.BRANCH_GUARD_RAD:
                return "BRANCH_GUARD"
        if t.worst_value_discover(q) >= cfg.RESOLVER["stop_m"]:               # b. shape re-check
            return "SHAPE_RECHECK"
        for s in changed:                                                       # c. joint limits
            lim = self.kin.limits[s]
            if np.any(q[_ARM_INDEX[s]] < lim[:, 0]) or np.any(q[_ARM_INDEX[s]] > lim[:, 1]):
                return "JOINT_LIMITS"
        for s in changed:                                                       # d. accuracy
            tgt = targets.get(s)
            if tgt is None:
                if cfg.RESOLVER["shortening_accuracy_target"] == "skip":
                    continue
                tgt = self.kin.hand_pose(self._state(t, t.raw), s)
            pos, rot = self.kin.hand_pose(self._state(t, q), s)
            if (np.linalg.norm(pos - tgt[0]) > cfg.RESOLVER["accuracy_pos_m"]
                    or np.degrees(rotation_error_rad(tgt[1], rot)) > cfg.RESOLVER["accuracy_deg"]):
                return "ACCURACY"
        return ""

