"""Safety filter Stage 2b (kong_style): the resolver on fixed poses and scripted command streams (no physics).

Covers: the lever split of a forearm contact and the upper arm push (pure; the minimum lever fraction on a real sweep, with
the lever input forced to tau 0.1); the hand displacement; link length restore (pure, and in a real sweep); the hand
orientation after conversion; arm against arm (only the closing arm moves; both closing: equal shares); a hand against a world
box (only the arm takes the push); pass-through; the rounds limit and the stop; determinism; the gap rule switch near the
margin; the four checks forced to fail; the layered resolver with elbow_swing_hand_target "raw" (old results); mode "cbf"
not built.
Written from the spec and the kong_style docstrings, not from its code. Fixtures: experiments/safety_filter_resolver_fixtures.py.

Run: source .venv/bin/activate; python experiments/safety_filter_kong_style_test.py   (exit 0 = pass)
"""

import sys
from pathlib import Path
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from safety_filter_resolver_fixtures import HOME, MIRROR, hand_hand_gap, ik, left_hand_at_y, new_resolver, state_of  # noqa: E402

from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter import kong_style as K  # noqa: E402
from src.safety_filter.arm_conversion import rotation_error_rad  # noqa: E402
from src.safety_filter.push_direction import contact_fraction, pair_push  # noqa: E402
from src.safety_filter.resolver import _Tick  # noqa: E402
from src.safety_filter.shapes import RobotState  # noqa: E402
from src.safety_filter.world import WorldObject  # noqa: E402

FAILS = []
RESULTS = {"run": 0, "ok": 0}
BASE = RobotState()
SIDES = ("left", "right")
PALM_PAIR = ("left_palm_2", "right_palm_2")   # hand against hand: the two palm capsules (the approach pair of the fixtures)


def check(name, cond, detail=""):
    RESULTS["run"] += 1
    RESULTS["ok"] += bool(cond)
    print(f"  {'ok  ' if cond else 'FAIL'} {name} {detail}")
    if not cond:
        FAILS.append(name)


class patched:
    """Temporarily sets entries of a config dict (or a module's __dict__); restored on exit, also after an error."""

    def __init__(self, d, **kv):
        self.d, self.kv, self.old = d, kv, {}

    def __enter__(self):
        for k, v in self.kv.items():
            self.old[k] = self.d[k]
            self.d[k] = v

    def __exit__(self, *exc):
        self.d.update(self.old)


def in_mode(mode):
    """Runs a section with cfg.RESOLVER["mode"] set to mode (restored afterwards)."""
    def wrap(fn):
        def run():
            with patched(cfg.RESOLVER, mode=mode):
                fn()
        run.__name__ = fn.__name__
        return run
    return wrap


def tick(res, last, raw, world=(), extra=()):
    """The candidate pairs of one tick as the step builds them (pairs near last and raw); extra pairs are added as the
    step adds the pairs still active from the last tick."""
    t = _Tick(res, np.asarray(raw, float), np.asarray(last, float), BASE, list(world), None)
    t.discover(t.last)
    t.discover(t.raw)
    for key in extra:
        t.add_known(key)
    return t


def kin_points(res, q):
    """Per arm: ((|E - S|, |W - E|), hand offset W -> H) at the joint angles q. The constraints the step restores to."""
    out = {}
    for side in SIDES:
        S, E, W = res.kin.sew(state_of(q), side)
        H, _ = res.kin.hand_pose(state_of(q), side)
        out[side] = ((float(np.linalg.norm(E - S)), float(np.linalg.norm(W - E))), H - W)
    return out


class _Plan:
    """What one sweep of the given pairs did: pushed {side: (E2, W2, H2)} the restored points of each pushed arm (last push),
    shorten: arms handed to step shortening, q: the joint angles afterwards, st: the status counters."""

    def __init__(self):
        self.pushed, self.shorten, self.q, self.st = {}, set(), None, None


def sweep(res, t, pairs, q=None):
    """One sweep (kong_style._sweep: the pairs pushed one at a time, each converted at once) of the given pairs at the pose q (default:
    the raw command). The shortfall of each pair is the one GIVEN (the tick's shortfall is pinned to it), so a test can force it."""
    from src.safety_filter.resolver import FilterStatus
    q = np.array(t.raw if q is None else q, float)
    cons = kin_points(res, q)
    r_cmd = {s: res.kin.hand_pose(state_of(q), s)[1] for s in SIDES}
    plan, targets, calls = _Plan(), {}, []
    orig = K.restore_lengths

    def spy(*a):
        r = orig(*a)
        calls.append(r)
        return r

    given = dict(pairs)
    real = t.shortfall
    plan.st = FilterStatus()
    with mock.patch.object(K, "restore_lengths", spy), mock.patch.object(t, "shortfall", lambda qq, k: given.get(k, real(qq, k))):
        K._sweep(res, t, q, K.order_pairs(list(pairs)), cons, r_cmd, targets, plan.shorten, plan.st)
    for side, (H2, _) in targets.items():
        for E2, W2, H in calls:
            if np.array_equal(H, H2):
                plan.pushed[side] = (E2, W2, H2)
    plan.q = q
    return plan


def hand_pos(res, q, side):
    return res.kin.hand_pose(state_of(q), side)[0]


def head_on_stream(res, n=60):
    """Commands of a head-on stream: both hands converge from the zombie pose to 5 cm past the midline (the raw commands)."""
    p_l, r_l = res.kin.hand_pose(state_of(HOME), "left")
    p_r, r_r = res.kin.hand_pose(state_of(HOME), "right")
    tgt = ik(res, HOME, "left", np.array([p_l[0] + 0.05, -0.05, p_l[2]]), r_l)
    tgt = ik(res, tgt, "right", np.array([p_r[0] + 0.05, 0.05, p_r[2]]), r_r)
    return [HOME + (tgt - HOME) * k / n for k in range(1, n + 1)]


def run_stream(res, stream):
    """Feeds the stream through one resolver. Rows: (raw, last, filtered, status)."""
    last, rows = HOME.copy(), []
    for raw in stream:
        out, st = res.step(raw, last, BASE)
        rows.append((raw, last.copy(), out, st))
        last = out
    return rows


def worst_after(res, row):
    """The worst shortfall of the filtered command of one tick, against the gap rule of that tick (as the resolver's own test)."""
    raw, last, out, _ = row
    return tick(res, last, raw).worst_value_discover(np.asarray(out, float))


def left_closing(res):
    """Hand-hand approach: left hand from lateral y 0.034 to 0.026 (gap 15.1 mm -> 7.1 mm), right arm at the zombie pose."""
    return left_hand_at_y(res, 0.034), left_hand_at_y(res, 0.026)


# ---------------------------------------------------------------- 1. lever split and upper arm push (pure)
def sec_lever_split():
    print("1. forearm lever split and upper arm push (pure functions)")
    push = np.array([0.003, -0.002, 0.001])
    de, dw = K.forearm_displacement(push, 0.5, 1.0, 1.0)
    check("forearm, tau 0.5, equal weights: elbow and wrist each move the full push", np.allclose(de, push) and np.allclose(dw, push))
    de, dw = K.forearm_displacement(push, 0.0, 1.0, 1.0)
    check("forearm, tau 0 (contact at the elbow): elbow moves the full push, wrist 0", np.allclose(de, push) and np.allclose(dw, 0.0))
    de, dw = K.forearm_displacement(push, 1.0, 1.0, 1.0)
    check("forearm, tau 1 (contact at the wrist): wrist moves the full push, elbow 0", np.allclose(dw, push) and np.allclose(de, 0.0))
    de, dw = K.forearm_displacement(push, 0.5, 2.0, 1.0)
    fe, fw = float(de @ push) / float(push @ push), float(dw @ push) / float(push @ push)
    check("forearm, tau 0.5, elbow weight 2, wrist weight 1: elbow 1.333 x push, wrist 0.667 x push",
          abs(fe - 4 / 3) < 1e-9 and abs(fw - 2 / 3) < 1e-9 and np.allclose(de, fe * push), f"(elbow {fe:.4f} x, wrist {fw:.4f} x)")
    worst = 0.0
    for tau, wE, wW in ((0.5, 1.0, 1.0), (0.0, 1.0, 1.0), (1.0, 1.0, 1.0), (0.5, 2.0, 1.0), (0.3, 2.0, 1.0), (0.8, 1.0, 3.0)):
        de, dw = K.forearm_displacement(push, tau, wE, wW)
        worst = max(worst, float(np.abs((1 - tau) * de + tau * dw - push).max()))
    check("in every case (1 - tau) dE + tau dW equals the push", worst < 1e-12, f"(worst difference {worst:.1e})")
    check("upper arm: elbow displacement = push / tau (tau 0.5 gives 2 x push, tau 0.25 gives 4 x push)",
          np.allclose(K.upper_arm_displacement(push, 0.5), 2 * push) and np.allclose(K.upper_arm_displacement(push, 0.25), 4 * push))


# ---------------------------------------------------------------- 2. hand contact (pure)
def sec_hand_displacement():
    print("2. hand contact (pure)")
    push = np.array([0.001, 0.002, -0.0005])
    w, h = K.hand_displacement(push)
    check("wrist and hand reference point move by the same full push", np.array_equal(w, push) and np.array_equal(h, push))


# ---------------------------------------------------------------- 3. link length restore
@in_mode("kong_style")
def sec_length_restore():
    print("3. link length restore")
    rng = np.random.default_rng(7)
    S = rng.normal(size=3) * 0.05
    E = S + np.array([0.0, 0.0, -0.2]) + rng.normal(size=3) * 0.01
    W = E + np.array([0.15, -0.02, 0.05]) + rng.normal(size=3) * 0.01
    off = np.array([0.1, 0.0, 0.0]) + rng.normal(size=3) * 0.01
    L = (float(np.linalg.norm(E - S)), float(np.linalg.norm(W - E)))
    pe, pw = rng.normal(size=3) * 0.004, rng.normal(size=3) * 0.004
    E2, W2, H = K.restore_lengths(S, E + pe, W + pw, L, off)
    err = (abs(float(np.linalg.norm(E2 - S)) - L[0]), abs(float(np.linalg.norm(W2 - E2)) - L[1]), float(np.abs((H - W2) - off).max()))
    check("pure: a pushed elbow and wrist are restored; upper arm, forearm and wrist-to-hand vector keep their lengths (1e-9)",
          max(err) < 1e-9 and float(np.linalg.norm(E2 - E)) > 1e-4, f"(worst error {max(err):.1e} m)")

    res = new_resolver()
    last, raw = left_closing(res)
    t = tick(res, last, raw, extra=[PALM_PAIR])
    plan = sweep(res, t, [(PALM_PAIR, t.shortfall(raw, PALM_PAIR))])
    pushed = plan.pushed.get("left")
    if pushed is None:
        check("real sweep (hand-hand, left closing): the left arm is pushed", False, f"(pushed {sorted(plan.pushed)})")
        return
    E2, W2, H2 = pushed
    (L1, L2), off = kin_points(res, raw)["left"]
    S_r, E_r, W_r = res.kin.sew(state_of(raw), "left")
    moved = float(np.linalg.norm(W2 - W_r)) * 1000
    err = (abs(float(np.linalg.norm(E2 - S_r)) - L1), abs(float(np.linalg.norm(W2 - E2)) - L2), float(np.abs((H2 - W2) - off).max()))
    check("real sweep (hand-hand, left closing): the pushed points keep the commanded link lengths and hand offset (1e-9)",
          max(err) < 1e-9, f"(worst error {max(err):.1e} m; wrist moved {moved:.2f} mm)")


# ---------------------------------------------------------------- 4. upper arm contact and the minimum lever fraction
@in_mode("kong_style")
def sec_upper_arm():
    print("4. upper arm contact and the minimum lever fraction")
    res = new_resolver()
    S, E, _ = res.kin.sew(state_of(HOME), "left")
    # a small box 3.5 cm off the left upper arm (+y side), at the point 10% of the way from the shoulder to the elbow
    box = WorldObject(id="ul_box", shape="box", size=(0.01, 0.01, 0.01), centre=tuple(S + 0.1 * (E - S) + np.array([0.0, 0.035, 0.0])),
                      rot=tuple(np.eye(3).ravel()))
    raw = HOME.copy()
    raw[0, 0] += 0.05                                   # joint 0 turned 0.05 rad: the upper arm closes on the box
    key = ("left_link3_2", "ul_box")
    t = tick(res, HOME, raw, world=[box])
    short = t.shortfall(raw, key)
    S_r, E_r, _ = res.kin.sew(state_of(raw), "left")
    pp = pair_push(t.shape(raw, key[0]), t.shape(raw, key[1]))
    tau_real = contact_fraction(pp.point_a, S_r, E_r)
    check("real geometry: the upper-arm pair closes and is short of the gap rule", t.closing(key) == ["left"] and short > cfg.RESOLVER["trigger_m"],
          f"(short {short * 1000:.2f} mm, closing {t.closing(key)})")
    # the real geometry is short by 0.44 mm, under stop_m (0.5 mm), so the sweep would skip the pair (Stage 2g); the lever rule is tested
    # with a given shortfall of 2 mm
    plan = sweep(res, t, [(key, 0.002)])
    check("real contact above min_lever_fraction: the elbow is pushed, nothing is shortened",
          tau_real > cfg.RESOLVER["min_lever_fraction"] and "left" in plan.pushed and not plan.shorten,
          f"(tau {tau_real:.3f}, pushed {sorted(plan.pushed)}, shorten {sorted(plan.shorten)})")
    # the lever input forced to tau 0.1: the zombie-pose link 3 contacts all project at tau about 0.3 or more (see the report)
    with mock.patch.object(K, "contact_fraction", lambda point, end_a, end_b: 0.1):
        plan = sweep(res, t, [(key, 0.002)])
    check("tau 0.1 (below min_lever_fraction): the pair is not pushed; step shortening is used for that arm",
          0.1 < cfg.RESOLVER["min_lever_fraction"] and plan.shorten == {"left"} and "left" not in plan.pushed,
          f"(shorten {sorted(plan.shorten)}, pushed {sorted(plan.pushed)}, min_lever_fraction {cfg.RESOLVER['min_lever_fraction']})")


# ---------------------------------------------------------------- 5. hand push: orientation after conversion
@in_mode("kong_style")
def sec_hand_push():
    print("5. hand push: orientation after conversion")
    res = new_resolver()
    last, raw = left_closing(res)
    out, st = res.step(raw, last, BASE)
    check("hand against hand (left closing): status KONG_PUSH", st.status == "KONG_PUSH", f"({st.status} {st.hold_reason})")
    r_cmd = res.kin.hand_pose(state_of(raw), "left")[1]
    r_out = res.kin.hand_pose(state_of(out), "left")[1]
    deg = float(np.degrees(rotation_error_rad(r_cmd, r_out)))
    check("realized hand orientation after conversion within cfg accuracy_deg of the commanded one (not 1e-6)",
          deg <= cfg.RESOLVER["accuracy_deg"], f"({deg:.4f} deg, limit {cfg.RESOLVER['accuracy_deg']} deg)")


# ---------------------------------------------------------------- 6. arm against arm
@in_mode("kong_style")
def sec_arm_against_arm():
    print("6. arm against arm")
    res = new_resolver()
    last, raw = left_closing(res)
    out, st = res.step(raw, last, BASE)
    check("one closing arm (left): the parked right arm is exactly as it was, the left arm moved",
          np.array_equal(out[1], last[1]) and not np.array_equal(out[0], last[0]), f"({st.status})")
    pr, rr = res.kin.hand_pose(state_of(HOME), "right")
    last_r = ik(res, HOME, "right", np.array([pr[0], -0.034, pr[2]]), rr)
    raw_r = ik(res, HOME, "right", np.array([pr[0], -0.026, pr[2]]), rr)
    out_r, st_r = new_resolver().step(raw_r, last_r, BASE)
    check("one closing arm (right, the closing-arm case mirrored): only the right arm moves",
          np.array_equal(out_r[0], last_r[0]) and not np.array_equal(out_r[1], last_r[1]), f"({st_r.status} {st_r.hold_reason})")

    # both arms closing (symmetric, not overlapping): one pair, same geometry; one-arm variant has the right arm parked
    l_last, l_raw = left_hand_at_y(res, 0.10)[0], left_hand_at_y(res, 0.094)[0]
    last_b = np.array([l_last, l_last * MIRROR])
    raw_b = np.array([l_raw, l_raw * MIRROR])
    one_raw = np.array([raw_b[0], last_b[1]])
    t_both = tick(res, last_b, raw_b, extra=[PALM_PAIR])
    t_one = tick(res, last_b, one_raw, extra=[PALM_PAIR])
    short = 0.004
    plan_both = sweep(res, t_both, [(PALM_PAIR, short)], q=raw_b)
    plan_one = sweep(res, t_one, [(PALM_PAIR, short)], q=raw_b)
    h0 = {s: hand_pos(res, raw_b, s) for s in SIDES}
    d_both = {s: float(np.linalg.norm(plan_both.pushed[s][2] - h0[s])) for s in plan_both.pushed}
    d_one = {s: float(np.linalg.norm(plan_one.pushed[s][2] - h0[s])) for s in plan_one.pushed}
    check("the plan pushes only the closing arms: both when both close, the left alone when the right is parked",
          set(plan_both.pushed) == {"left", "right"} and set(plan_one.pushed) == {"left"} and not plan_both.shorten and not plan_one.shorten,
          f"(both {sorted(plan_both.pushed)}, one {sorted(plan_one.pushed)})")
    ratio = d_both.get("left", 0.0) / max(d_one.get("left", 0.0), 1e-12)
    check("both closing: each arm takes half the push (left displacement with both / alone = 0.5 within 0.02)",
          abs(ratio - 0.5) < 0.02, f"(ratio {ratio:.4f}; alone {d_one.get('left', 0.0) * 1000:.3f} mm)")
    diff = abs(d_both.get("left", 0.0) - d_both.get("right", 0.0))
    check("both closing: the two arms take equal shares (within 0.2 mm; the palm capsules are not exact mirrors)", diff < 0.2e-3,
          f"(left {d_both.get('left', 0.0) * 1000:.3f} mm, right {d_both.get('right', 0.0) * 1000:.3f} mm)")


# ---------------------------------------------------------------- 7. world contact
@in_mode("kong_style")
def sec_world_contact():
    print("7. hand against a world object")
    res = new_resolver()
    p, r = res.kin.hand_pose(state_of(HOME), "left")
    raw = ik(res, HOME, "left", p - np.array([0.0, 0.0, 0.008]), r)            # the left hand 8 mm down, orientation kept
    # under the left palm (its centre at the zombie pose is (0.2154, 0.1247, 0.8404)): palm gap 11.8 mm at the zombie pose
    # (inside the 2 cm band, outside the 1 cm margin), 3.8 mm after the drop
    box = WorldObject(id="test_box", shape="box", size=(0.03, 0.03, 0.03), centre=(0.2154, 0.1247, 0.770), rot=tuple(np.eye(3).ravel()))
    key = ("left_palm_2", "test_box")
    t = tick(res, HOME, raw, world=[box])
    short = t.shortfall(raw, key)
    check("scene: the palm-box pair is closing and short of the gap rule", t.closing(key) == ["left"] and short > 1e-3,
          f"(gap {t.gap(HOME, key) * 1000:.1f} -> {t.gap(raw, key) * 1000:.1f} mm, short {short * 1000:.2f} mm)")
    plan = sweep(res, t, [(key, short)])
    d = float(np.linalg.norm(plan.pushed["left"][2] - hand_pos(res, raw, "left"))) if "left" in plan.pushed else 0.0
    check("world and one closing arm: only the arm is pushed, and it takes the whole shortfall (within 10%)",
          set(plan.pushed) == {"left"} and abs(d - short) < 0.1 * short, f"(hand moved {d * 1000:.2f} mm, short {short * 1000:.2f} mm)")
    out, st = new_resolver().step(raw, HOME, BASE, world_objects=[box])
    check("full step: status KONG_PUSH, the right arm exactly as it was, the left arm moved",
          st.status == "KONG_PUSH" and np.array_equal(out[1], HOME[1]) and not np.array_equal(out[0], HOME[0]), f"({st.status} {st.hold_reason})")


# ---------------------------------------------------------------- 8. pass-through
@in_mode("kong_style")
def sec_pass_through():
    print("8. pass-through")
    res = new_resolver()
    out, st = res.step(HOME, HOME, BASE)
    check("nothing near: the raw array object itself comes back, status PASS, 0 rounds", out is HOME and st.status == "PASS" and st.rounds == 0)
    far_raw = HOME.copy()
    far_raw[0, 0] += 0.01
    out, st = new_resolver().step(far_raw, HOME.copy(), BASE)
    check("a small move far from everything: the identical array object, status PASS", out is far_raw and st.status == "PASS")
    raw_in = left_hand_at_y(res, 0.026)
    out, st = new_resolver().step(raw_in, raw_in.copy(), BASE)
    check("a parked pair inside its margin (nobody closing): the identical array object, status PASS", out is raw_in and st.status == "PASS")


# ---------------------------------------------------------------- 9. rounds, stop, determinism
@in_mode("kong_style")
def sec_rounds_and_determinism():
    print("9. rounds, stop and determinism")
    stream = head_on_stream(new_resolver())
    rows = run_stream(new_resolver(), stream)
    print("  statuses seen:", sorted({f"{r[3].status}/{r[3].hold_reason or '-'}" for r in rows}), " rounds seen:", sorted({r[3].rounds for r in rows}))
    max_rounds = max(r[3].rounds for r in rows)
    check("rounds never exceed 10 (nor the rounds setting)", max_rounds <= min(10, cfg.RESOLVER["rounds"]), f"(max {max_rounds})")
    res = new_resolver()
    pushed = [r for r in rows if r[3].status == "KONG_PUSH"]
    ends = [worst_after(res, r) for r in pushed]
    check("every KONG_PUSH tick stopped with its worst shortfall under stop_m (0.5 mm)",
          len(pushed) > 0 and all(e < cfg.RESOLVER["stop_m"] for e in ends),
          f"({len(pushed)} KONG_PUSH ticks, worst end shortfall {max(ends, default=0.0) * 1000:.3f} mm)")
    again = run_stream(new_resolver(), stream[:12])
    same = all(np.array_equal(a[2], b[2]) and a[3].status == b[3].status and a[3].hold_reason == b[3].hold_reason
               for a, b in zip(rows[:12], again))
    check("the same inputs give the same outputs and statuses twice (fresh resolvers, 12 ticks of the stream, bit for bit)", same)
    last, raw = left_closing(res)
    o1, s1 = new_resolver().step(raw, last, BASE)
    o2, s2 = new_resolver().step(raw, last, BASE)
    check("hand-hand step twice on fresh resolvers: identical array and status", np.array_equal(o1, o2) and s1.status == s2.status and s1.hold_reason == s2.hold_reason)


# ---------------------------------------------------------------- 10. gap rule mode near the margin
@in_mode("kong_style")
def sec_gap_rule_switch():
    print("10. gap rule mode near the margin")
    res = new_resolver()
    last, raw = left_hand_at_y(res, 0.0296), left_hand_at_y(res, 0.0292)   # palm-palm gap 10.74 mm -> 10.34 mm; margin 10 mm
    gap_raw = hand_hand_gap(raw)[0]
    with patched(cfg.GAP_RULE, mode="ours"):
        t = tick(res, last, raw, extra=[PALM_PAIR])
        short_ours = t.shortfall(raw, PALM_PAIR)
        out_o, st_o = new_resolver().step(raw, last, BASE)
    with patched(cfg.GAP_RULE, mode="paper_plain"):
        t = tick(res, last, raw, extra=[PALM_PAIR])
        short_plain = t.shortfall(raw, PALM_PAIR)
        out_p, st_p = new_resolver().step(raw, last, BASE)
    check("hand-hand pair just above the margin (raw gap %.2f mm): ours limits the approach, paper_plain does not push" % (gap_raw * 1000),
          short_ours > 1e-6 and short_plain == 0.0, f"(shortfall ours {short_ours * 1000:.3f} mm, paper_plain {short_plain * 1000:.3f} mm)")
    check("the mode switch changes the result of the same tick", not np.array_equal(out_o, out_p) or (st_o.status, st_o.hold_reason) != (st_p.status, st_p.hold_reason),
          f"(ours {st_o.status} {st_o.hold_reason or '-'}, paper_plain {st_p.status} {st_p.hold_reason or '-'})")


# ---------------------------------------------------------------- 11. the four checks forced to fail
@in_mode("kong_style")
def sec_forced_checks():
    print("11. each check forced to fail gives hold-still with its reason")
    last, raw = left_closing(new_resolver())
    forced = {}
    with patched(cfg.__dict__, BRANCH_GUARD_RAD=1e-9):
        forced["BRANCH_GUARD"] = new_resolver().step(raw, last, BASE)
    with patched(cfg.RESOLVER, stop_m=-1.0):
        forced["SHAPE_RECHECK"] = new_resolver().step(raw, last, BASE)
    r = new_resolver()
    r.kin.limits["left"] = np.tile([[-1e-3, 1e-3]], (7, 1))
    forced["JOINT_LIMITS"] = r.step(raw, last, BASE)
    with patched(cfg.RESOLVER, accuracy_pos_m=0.0):
        forced["ACCURACY"] = new_resolver().step(raw, last, BASE)
    for reason, (out, st) in forced.items():
        check(f"{reason}: status HOLD with that reason; the last filtered command comes back",
              st.status == "HOLD" and st.hold_reason == reason and np.array_equal(out, last) and out is not last, f"({st.status} {st.hold_reason})")


# ---------------------------------------------------------------- 12. layered resolver, old results; cbf not built
def sec_layered_old():
    print("12. layered resolver with elbow_swing_hand_target raw (old results); cbf")
    with patched(cfg.RESOLVER, mode="layered", elbow_swing_hand_target="raw"):
        res = new_resolver()
        last, raw = left_closing(res)
        out, st = new_resolver().step(raw, last, BASE)
        check("layered/raw, hand-hand left closing: HAND_SLIDE in 1 round", st.status == "HAND_SLIDE" and st.rounds == 1, f"({st.status} {st.rounds} rounds)")
        pr, rr = res.kin.hand_pose(state_of(HOME), "right")
        last_r = ik(res, HOME, "right", np.array([pr[0], -0.034, pr[2]]), rr)
        raw_r = ik(res, HOME, "right", np.array([pr[0], -0.026, pr[2]]), rr)
        out_r, st_r = new_resolver().step(raw_r, last_r, BASE)
        check("layered/raw, right-closing mirror case: HOLD with reason ACCURACY after 10 rounds",
              st_r.status == "HOLD" and st_r.hold_reason == "ACCURACY" and st_r.rounds == 10, f"({st_r.status} {st_r.hold_reason} {st_r.rounds} rounds)")
    with patched(cfg.RESOLVER, mode="cbf"):
        try:
            new_resolver().step(raw, last, BASE)
            message = None
        except NotImplementedError as exc:
            message = str(exc)
    check("mode cbf raises NotImplementedError 'CBF resolver not built'", message == "CBF resolver not built", f"({message!r})")


def main():
    sec_lever_split()
    sec_hand_displacement()
    sec_length_restore()
    sec_upper_arm()
    sec_hand_push()
    sec_arm_against_arm()
    sec_world_contact()
    sec_pass_through()
    sec_rounds_and_determinism()
    sec_gap_rule_switch()
    sec_forced_checks()
    sec_layered_old()
    print(f"{RESULTS['ok']} of {RESULTS['run']} checks ok")
    print("PASS" if not FAILS else f"FAIL: {FAILS}")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
