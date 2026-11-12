"""The kong_style resolver (Stage 2b; SF-40 to SF-50): every pair that needs a push is shared among four points per arm
(shoulder, elbow, wrist, hand reference point), the pushes are ADDED per point, the link lengths are restored, and the points
are converted to joint angles in one go (Kong's solver, then exactly one Jacobian step on our own model).

What is new here is only how a pair's push is shared among the points. Everything else is the layered resolver's: the link
categories (link_rules), the shared layer's shapes and gaps, the gap rule (gap_rule: how MUCH each pair must move), the push
direction (push_direction.pair_push), the closing-arm rule, step shortening, the four checks, hold-still and the status record.

Per tick (resolver.Resolver.step calls step() below when RESOLVER["mode"] is "kong_style"):
 1. Pass-through: no pair needs a push or a speed limit -> the raw array itself comes back (status PASS).
 2. Jump-through guard: the straight joint-space path last -> raw is sampled; if a sample has a pair below tunnel_guard_min_gap_m
    the step is shortened (the layered resolver's step shortening, with this pass test); the result is the raw command from here on.
 3. Sweeps (up to RESOLVER["rounds"], stop when the worst shortfall is under stop_m): per category the pushes are summed per point
    (see the *_displacement functions), lengths restored (restore_lengths), points converted (solve_arm).
 4. The four checks (resolver.Resolver._checks); the accuracy target is the PUSHED hand pose (SF-47). Any failure: hold-still.
 5. Status KONG_PUSH: sweeps used (rounds), worst pair, gap, shortfall, and the categories pushed (methods).
"""

import math

import numpy as np

from src.safety_filter import config as cfg
from src.safety_filter.link_rules import FOREARM, HAND, UPPER_ARM, category_of
from src.safety_filter.push_direction import contact_fraction, pair_push
from src.safety_filter.shapes import SIDES

_IDX = {"left": 0, "right": 1}


# ---------------------------------------------------------------- the sharing of one pair's push (pure functions)
def upper_arm_displacement(push_vec, tau):
    """Elbow displacement for an upper arm contact (SF-44): the elbow alone moves, by push / tau, where tau is the fraction of the
    way from the shoulder (0) to the elbow (1) where the closest point lies. The caller handles tau < min_lever_fraction."""
    return np.asarray(push_vec, float) / tau


def forearm_displacement(push_vec, tau, w_elbow=None, w_wrist=None):
    """(elbow displacement, wrist displacement) for a forearm contact (SF-43). tau: fraction from the elbow (0) to the wrist (1).
    With weights wE, wW: dE = wE (1-tau) push / D, dW = wW tau push / D, D = wE (1-tau)^2 + wW tau^2.
    The contact point then moves by exactly the push: (1-tau) dE + tau dW = push."""
    wE = cfg.RESOLVER["lever_weight_elbow"] if w_elbow is None else w_elbow
    wW = cfg.RESOLVER["lever_weight_wrist"] if w_wrist is None else w_wrist
    push_vec = np.asarray(push_vec, float)
    d = wE * (1.0 - tau) ** 2 + wW * tau ** 2
    return wE * (1.0 - tau) * push_vec / d, wW * tau * push_vec / d


def hand_displacement(push_vec):
    """(wrist displacement, hand reference point displacement) for a hand contact (SF-42): both move by the full push (rigid, no tilt)."""
    push_vec = np.asarray(push_vec, float)
    return push_vec.copy(), push_vec.copy()


def restore_lengths(S, E, W, lengths, hand_offset):
    """Put the pushed elbow and wrist back at the original link lengths (SF-45). S fixed. lengths = (|E-S|, |W-E|) as they were;
    hand_offset = the original vector from the wrist to the hand reference point (world, = the commanded hand orientation applied
    to the link 7 offset, so the hand never tilts). Elbow: moved along the shoulder-to-elbow direction to the original distance
    from the shoulder; wrist: the same from the elbow. Returns (E, W, H)."""
    S, E, W = (np.asarray(v, float) for v in (S, E, W))
    d = E - S
    n = float(np.linalg.norm(d))
    if n < 1e-9:
        raise ValueError("elbow on the shoulder")
    E2 = S + lengths[0] * d / n
    d = W - E2
    n = float(np.linalg.norm(d))
    if n < 1e-9:
        raise ValueError("wrist on the elbow")
    W2 = E2 + lengths[1] * d / n
    return E2, W2, W2 + np.asarray(hand_offset, float)


# ---------------------------------------------------------------- the one place that calls Kong's solver
def solve_arm(res, state, side, S, E, W, hand_rot):
    """Joint angles (7,) of one arm for the restored points: Kong's solver (collision avoidance off) gets the shoulder-to-elbow and
    elbow-to-wrist directions and the commanded hand orientation (SF-32; it reads directions only, so the lengths are ours).
    Raises RuntimeError("NO_SOLVER..." / "SOLVER_FAILED...") as the layered resolver's converter does. This is the only place
    kong_style calls the solver; a stub can replace it in tests."""
    u = (E - S) / np.linalg.norm(E - S)
    l = (W - E) / np.linalg.norm(W - E)
    return res.kin.convert_directions(state, side, u, l, hand_rot)


class SweepPlan:
    """What a sweep wants before conversion. pushed: {side: (E, W, H)} the points after summing and length restore;
    shorten: set of arms whose step must be shortened instead; cats: categories pushed (in order)."""

    def __init__(self):
        self.pushed, self.shorten, self.cats = {}, set(), []


def _keypoints(res, t, q):
    out = {}
    for side in SIDES:
        S, E, W = res.kin.sew(res._state(t, q), side)
        H, _ = res.kin.hand_pose_now(side)
        out[side] = (S, E, W, H)
    return out


def plan_sweep(res, t, q, pairs, cons):
    """Sum the pushes of all pairs per point and restore the link lengths. pairs: [(key, shortfall)] of the pairs needing a push;
    cons: {side: (lengths, hand_offset)}. Returns a SweepPlan (nothing is converted here)."""
    kp = _keypoints(res, t, q)
    disp = {s: {"E": np.zeros(3), "W": np.zeros(3), "H": np.zeros(3)} for s in SIDES}
    plan = SweepPlan()
    pushed_arm = set()
    for key, short in pairs:
        arms = t.closing(key)
        if not arms:
            continue
        share = short / len(arms)                       # both arms closing: each takes half; world and one closing arm: the arm takes it all
        a, b = t.shape(q, key[0]), t.shape(q, key[1])
        pp = pair_push(a, b)
        for side in arms:
            mine, direction, point = (a, pp.dir_a, pp.point_a) if a.arm == side else (b, pp.dir_b, pp.point_b)
            S, E, W, H = kp[side]
            pv = np.asarray(direction, float) * share
            cat = category_of(mine)
            if cat == HAND:
                dw, dh = hand_displacement(pv)
                disp[side]["W"] += dw
                disp[side]["H"] += dh
            elif cat == UPPER_ARM:
                tau = contact_fraction(point, S, E)
                if tau < cfg.RESOLVER["min_lever_fraction"]:
                    plan.shorten.add(side)
                    continue
                disp[side]["E"] += upper_arm_displacement(pv, tau)
            elif cat == FOREARM:
                tau = contact_fraction(point, E, W)
                de, dw = forearm_displacement(pv, tau)
                disp[side]["E"] += de
                disp[side]["W"] += dw
            else:                                       # shoulder (and unknown parts): step shortening
                plan.shorten.add(side)
                continue
            pushed_arm.add(side)
            if cat not in plan.cats:
                plan.cats.append(cat)
    for side in pushed_arm - plan.shorten:
        S, E, W, H = kp[side]
        lengths, offset = cons[side]
        plan.pushed[side] = restore_lengths(S, E + disp[side]["E"], W + disp[side]["W"], lengths, offset)
    return plan


# ---------------------------------------------------------------- the jump-through guard
def _capsule_radii(t, q):
    rad = {}
    for s in t.pose(q)[1]:
        if getattr(s, "arm", None) in SIDES and s.kind == "capsule" and s.r > 0.0:
            c = category_of(s)
            rad[c] = min(rad.get(c, np.inf), s.r)
    return rad


def _path_samples(res, t):
    """Number of samples along the straight joint-space path last -> raw: ceil(largest keypoint displacement / its capsule radius), 1 to max."""
    rad = _capsule_radii(t, t.last)
    a, b = _keypoints(res, t, t.last), _keypoints(res, t, t.raw)
    worst = 0.0
    for side in SIDES:
        for idx, cat in ((1, UPPER_ARM), (2, FOREARM), (3, HAND)):
            r = rad.get(cat)
            if r:
                worst = max(worst, float(np.linalg.norm(b[side][idx] - a[side][idx])) / r)
    return int(min(max(1, math.ceil(worst)), cfg.RESOLVER["tunnel_guard_max_samples"]))


def _guard(res, t, methods, targets, st):
    """Shorten the step when the path last -> raw passes a pair closer than tunnel_guard_min_gap_m (SF-46). Returns True if shortened."""
    n = _path_samples(res, t)
    min_gap = cfg.RESOLVER["tunnel_guard_min_gap_m"]
    skip_below = cfg.RESOLVER["tunnel_guard_skip_already_below"]

    def violates(qs):
        t.discover(qs)
        for key in t.keys:
            g = t.gap(qs, key)
            if g < min_gap and not (skip_below and t.g0[key] < min_gap):
                return True
        return False

    def passes(f):                                      # no sample from 0 to the fraction f violates
        for j in range(1, n + 1):
            if violates(t.last + (f * j / n) * (t.raw - t.last)):
                return False
        return True

    if passes(1.0):
        return False
    q = np.array(t.raw, float)
    res._shorten(t, q, list(SIDES), methods, targets, st, passes=passes)
    t.raw = q                                           # the shortened command is the raw command for the rest of the tick
    t._closing = {}
    return True


# ---------------------------------------------------------------- the tick
def step(res, raw, last, state, world_objects=(), held=None):
    """One kong_style control tick. Same inputs and outputs as Resolver.step."""
    from src.safety_filter import resolver as R
    last = np.asarray(last, float)
    if not np.isfinite(np.asarray(raw, float)).all():
        return last.copy(), R.FilterStatus(status=R.HOLD, hold_reason="NOT_FINITE")
    t = R._Tick(res, np.asarray(raw), last, state, world_objects, held)
    t.discover(last)
    t.discover(t.raw)
    for key in res._active:
        t.add_known(key)
    first = t.worst(t.raw)
    if first is None:                                   # 1. pass-through
        res._update_active(t, t.raw)
        return raw, R.FilterStatus()
    key0, s0 = first
    st = R.FilterStatus(worst_pair=key0, worst_gap=t.gap(t.raw, key0), worst_shortfall=s0)
    methods, targets, cats = [], {}, []
    q = np.array(t.raw, float)
    try:
        shortened = _guard(res, t, methods, targets, st)          # 2. jump-through guard
        q = np.array(t.raw, float)
        # 3. keypoints: the lengths and the hand offset are those of the (possibly shortened) commanded pose
        cons, r_cmd = {}, {}
        for side in SIDES:
            S, E, W, H = _keypoints(res, t, t.raw)[side]
            cons[side] = ((float(np.linalg.norm(E - S)), float(np.linalg.norm(W - E))), H - W)
            r_cmd[side] = res.kin.hand_pose(res._state(t, t.raw), side)[1]
        sweeps, pushed_any = 0, False
        while sweeps < cfg.RESOLVER["rounds"]:
            w = t.worst(q, discover=True)
            if w is None or (sweeps > 0 and w[1] < cfg.RESOLVER["stop_m"]):
                break
            pairs = [(k, t.shortfall(q, k)) for k in sorted(t.keys)
                     if t.shortfall(q, k) > cfg.RESOLVER["trigger_m"] and t.closing(k)]
            plan = plan_sweep(res, t, q, pairs, cons)
            for side, (E2, W2, H2) in plan.pushed.items():
                S = res.kin.sew(res._state(t, q), side)[0]
                q[_IDX[side]] = _convert(res, t, q, side, S, E2, W2, H2, r_cmd[side])
                targets[side] = (H2, r_cmd[side])
                pushed_any = True
            for c in plan.cats:
                if c not in cats:
                    cats.append(c)
            sweeps += 1
            if plan.shorten:                            # a pair that cannot be pushed: step shortening of that arm
                res._shorten(t, q, sorted(plan.shorten), methods, targets, st)
                shortened = True
                for side in plan.shorten:
                    if cfg.RESOLVER["kong_shortened_accuracy"] == "own_pose":
                        targets[side] = res.kin.hand_pose(res._state(t, q), side)
                break
        st.rounds = sweeps
        reason = res._checks(t, q, targets)
        if reason:
            raise R._Hold(reason)
    except R._Hold as h:
        st.status, st.hold_reason = R.HOLD, h.reason
        st.methods = tuple(dict.fromkeys(cats + methods))
        res._update_active(t, last)
        return last.copy(), st
    st.methods = tuple(dict.fromkeys(cats + methods))
    st.hand_targets = {s: (np.array(v[0]), np.array(v[1])) for s, v in targets.items()}
    st.status = R.KONG_PUSH if pushed_any else R.STEP_SHORTENED if R.STEP_SHORTENED in methods else R.PASS
    res._update_active(t, q)
    return q, st


def _convert(res, t, q, side, S, E2, W2, H2, r_cmd):
    """Points -> joint angles of one arm (SF-32): solve_arm, then one Jacobian step. Solver failure holds still."""
    from src.safety_filter import resolver as R
    state = res._state(t, q)
    try:
        qc = solve_arm(res, state, side, S, E2, W2, r_cmd)
    except RuntimeError as exc:
        raise R._Hold("NO_SOLVER" if str(exc).startswith("NO_SOLVER") else "SOLVER_FAILED")
    q2 = q.copy()
    q2[_IDX[side]] = qc
    return res.kin.jacobian_step(res._state(t, q2), side, H2, r_cmd)
