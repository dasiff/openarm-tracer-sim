"""Safety filter Stage 2: the resolver on fixed poses and scripted command streams (no physics).

Covers: pass-through is the identical array; hand slide (target orientation, into-motion removed, along-surface motion kept);
elbow swing accuracy; step shortening (largest passing fraction, category dispatch); each of the four checks forced to fail gives
hold-still with its reason; arm against arm (only the closing arm moves, equal shares, a parked arm is not touched); rounds limit
and stop; determinism; the paper_plain switch; the SEW points against Kong's own forward kinematics; the recording.

Run: python experiments/safety_filter_resolver_test.py   (exit 0 = pass). Needs Kong's licensed wheel (Linux) for the elbow swing.
"""

import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from safety_filter_resolver_fixtures import (DT, HOME, MIRROR, hand_hand_gap, ik, left_hand_at_y, model_and_shapes, new_resolver,  # noqa: E402
                                              solver, state_of)

from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter import gap_rule, link_rules  # noqa: E402
from src.safety_filter.arm_conversion import rotation_error_rad  # noqa: E402
from src.safety_filter.push_direction import pair_push  # noqa: E402
from src.safety_filter.recording import FilterRecorder, load_filter_recording  # noqa: E402
from src.safety_filter.resolver import _Tick  # noqa: E402
from src.safety_filter.shapes import RobotState  # noqa: E402

FAILS = []
BASE = RobotState()


def check(name, cond, detail=""):
    print(f"  {'ok  ' if cond else 'FAIL'} {name} {detail}")
    if not cond:
        FAILS.append(name)


class patched:
    """Temporarily sets entries of a config dict."""

    def __init__(self, d, **kv):
        self.d, self.kv, self.old = d, kv, {}

    def __enter__(self):
        for k, v in self.kv.items():
            self.old[k] = self.d[k]
            self.d[k] = v

    def __exit__(self, *a):
        self.d.update(self.old)


def convergence_stream(res, n=60):
    """Both hands converge from the zombie pose to 5 cm past the midline (a head-on stream): the raw commands."""
    p_l, r_l = res.kin.hand_pose(state_of(HOME), "left")
    p_r, r_r = res.kin.hand_pose(state_of(HOME), "right")
    tgt = ik(res, HOME, "left", np.array([p_l[0] + 0.05, -0.05, p_l[2]]), r_l)
    tgt = ik(res, tgt, "right", np.array([p_r[0] + 0.05, 0.05, p_r[2]]), r_r)
    return [HOME + (tgt - HOME) * k / n for k in range(1, n + 1)]


def run_stream(res, stream, base=BASE):
    last, out_all = HOME.copy(), []
    for raw in stream:
        out, st = res.step(raw, last, base)
        out_all.append((raw, last.copy(), out, st))
        last = out
    return out_all


def main():
    res = new_resolver()
    kin = res.kin
    _, sh = model_and_shapes()
    margin_hh = cfg.MARGINS["hand_hand_m"]
    # a hand-hand approach: left hand last at gap 1.5 cm (slowing band), raw at 0.7 cm (inside the 1 cm margin)
    last_hh, raw_hh = left_hand_at_y(res, 0.034), left_hand_at_y(res, 0.026)
    g_last, g_raw = hand_hand_gap(last_hh)[0], hand_hand_gap(raw_hh)[0]
    print(f"setup: hand-hand gap last {g_last*1000:.1f} mm, raw {g_raw*1000:.1f} mm, margin {margin_hh*1000:.0f} mm")

    print("pass-through")
    out, st = res.step(HOME, HOME, BASE)
    check("nothing near: the raw array object comes back, status PASS", out is HOME and st.status == "PASS" and st.rounds == 0)
    far_last, far_raw = HOME.copy(), HOME.copy()
    far_raw[0, 0] += 0.01
    out, st = res.step(far_raw, far_last, BASE)
    check("a small move far from everything: identical array (same object, bit for bit)", out is far_raw and st.status == "PASS")
    last_in, raw_in = raw_hh.copy(), raw_hh.copy()
    out, st = res.step(raw_in, last_in, BASE)
    check("a parked pair inside its margin (nobody closing) is not acted on", out is raw_in and st.status == "PASS")

    print("hand slide")
    out, st = res.step(raw_hh, last_hh, BASE)
    check("hand contact gives HAND_SLIDE", st.status == "HAND_SLIDE" and st.worst_pair is not None, f"({st.status} {st.hold_reason} {st.worst_pair})")
    p_raw, r_raw = kin.hand_pose(state_of(raw_hh), "left")
    p_last, _ = kin.hand_pose(state_of(last_hh), "left")
    tp, tr = st.hand_targets["left"]
    check("the hand orientation target is the commanded one (to 1e-6 rad)", rotation_error_rad(r_raw, tr) < 1e-6, f"({rotation_error_rad(r_raw, tr):.2e})")
    p_out, r_out = kin.hand_pose(state_of(out), "left")
    ang = rotation_error_rad(r_raw, r_out)
    check("after the single Jacobian step the achieved orientation is within 0.5 deg", np.degrees(ang) < 0.5, f"(achieved {ang:.2e} rad)")
    check("hand position within 2 mm of the pushed-back target", np.linalg.norm(p_out - tp) < 0.002, f"({np.linalg.norm(p_out - tp)*1000:.2f} mm)")
    sa, sb = sh.place(state_of(last_hh)), None
    ids = st.worst_pair
    shp = {s.id: s for s in sa}
    pp = pair_push(shp[ids[0]], shp[ids[1]])
    into = -(pp.dir_a if shp[ids[0]].arm == "left" else pp.dir_b)
    v_raw, v_out = p_raw - p_last, p_out - p_last
    check("the hand's motion into the obstacle is removed", float(v_out @ into) < 0.002 and float(v_raw @ into) > 0.005,
          f"(raw {float(v_raw @ into)*1000:.1f} mm, out {float(v_out @ into)*1000:.2f} mm)")
    check("the right (parked) arm is untouched", np.array_equal(out[1], last_hh[1]))
    check("the gap did not close below what the rule allows", hand_hand_gap(out)[0] >= g_last - 0.0015, f"({hand_hand_gap(out)[0]*1000:.2f} mm)")
    # motion along the surface is kept: add a sideways (z) move to the raw command
    p_l0, r_l0 = kin.hand_pose(state_of(HOME), "left")
    raw_side = ik(res, raw_hh, "left", np.array([p_raw[0], p_raw[1], p_raw[2] + 0.004]), r_l0)
    out2, st2 = res.step(raw_side, last_hh, BASE)
    p_raw2, _ = kin.hand_pose(state_of(raw_side), "left")
    p_out2, _ = kin.hand_pose(state_of(out2), "left")
    t_raw = (p_raw2 - p_last) - float((p_raw2 - p_last) @ into) * into
    t_out = (p_out2 - p_last) - float((p_out2 - p_last) @ into) * into
    check("a sideways move along the surface is kept (within 2 mm)", st2.status == "HAND_SLIDE" and np.linalg.norm(t_raw - t_out) < 0.002,
          f"({st2.status}, {np.linalg.norm(t_raw - t_out)*1000:.2f} mm)")

    print("elbow swing")
    ticks = run_stream(res, convergence_stream(res))
    swings = [t for t in ticks if t[3].status == "ELBOW_SWING"]
    check("the head-on stream produces elbow swings", len(swings) > 0, f"({len(swings)} of {len(ticks)} ticks)")
    worst_pos, worst_ang = 0.0, 0.0
    for raw, last, out, st in swings:
        for side in ("left", "right"):
            i = 0 if side == "left" else 1
            if not np.array_equal(out[i], raw[i]):
                tp, tr = st.hand_targets[side]
                p_o, r_o = kin.hand_pose(state_of(out), side)
                worst_pos = max(worst_pos, float(np.linalg.norm(p_o - tp)))
                worst_ang = max(worst_ang, float(np.degrees(rotation_error_rad(tr, r_o))))
    check("after the Jacobian step the hand is within 2 mm and 0.5 deg of the raw commanded pose",
          worst_pos <= cfg.RESOLVER["accuracy_pos_m"] and worst_ang <= cfg.RESOLVER["accuracy_deg"], f"(worst {worst_pos*1000:.2f} mm, {worst_ang:.3f} deg)")
    st_names = {t[3].status for t in ticks}
    print("  statuses seen in the stream:", sorted(st_names), "hold reasons:", sorted({t[3].hold_reason for t in ticks if t[3].hold_reason}))

    print("step shortening")
    # (a) the search itself: the largest passing fraction (within the search resolution)
    t = _Tick(res, np.array(raw_hh), np.array(last_hh), BASE, [], None)
    t.discover(t.last)
    t.discover(t.raw)
    from src.safety_filter.resolver import FilterStatus
    st_s, methods = FilterStatus(), []
    q = np.array(raw_hh)
    with patched(cfg.RESOLVER, shortening_accuracy_target="skip"):
        res._shorten(t, q, ["left"], methods, {}, st_s)
    f = st_s.fraction
    def ok_at(fr):
        qq = np.array(raw_hh)
        qq[0] = last_hh[0] + fr * (raw_hh[0] - last_hh[0])
        return t.worst_value_discover(qq) < cfg.RESOLVER["stop_m"]
    res_ = 1.0 / 2 ** cfg.RESOLVER["shorten_halvings"]
    check("the chosen fraction passes the gap rule", ok_at(f), f"(fraction {f:.5f})")
    check("a fraction one search step larger fails (largest passing within the resolution)", f >= 1.0 or not ok_at(min(1.0, f + 1.5 * res_)))
    check("the fraction is between 0 and 1 and the arm moved by exactly that share",
          0.0 < f < 1.0 and np.allclose(q[0], last_hh[0] + f * (raw_hh[0] - last_hh[0])))
    # (b) the dispatch: a hand contact treated as a shoulder contact is shortened end to end
    old = link_rules.PART_CATEGORY["palm"], link_rules.PART_CATEGORY["link7"], link_rules.PART_CATEGORY["finger"]
    try:
        for part in ("palm", "link7", "finger"):
            link_rules.PART_CATEGORY[part] = link_rules.SHOULDER
        with patched(cfg.RESOLVER, shortening_accuracy_target="skip"):
            out, st = res.step(raw_hh, last_hh, BASE)
        check("a shoulder-category contact is handled by step shortening (status STEP_SHORTENED)", st.status == "STEP_SHORTENED" and 0 < st.fraction < 1,
              f"({st.status} {st.hold_reason} fraction {st.fraction:.4f})")
        out_lit, st_lit = res.step(raw_hh, last_hh, BASE)
        check("with the literal accuracy target (raw pose) the shortened step is refused when it slows the hand by more than 2 mm",
              st_lit.status == "HOLD" and st_lit.hold_reason == "ACCURACY", f"({st_lit.status} {st_lit.hold_reason})")
    finally:
        link_rules.PART_CATEGORY["palm"], link_rules.PART_CATEGORY["link7"], link_rules.PART_CATEGORY["finger"] = old
    # (c) the largest passing fraction below the minimum: hold-still (SHORTEN_ZERO)
    try:
        for part in ("palm", "link7", "finger"):
            link_rules.PART_CATEGORY[part] = link_rules.SHOULDER
        with patched(cfg.RESOLVER, shortening_accuracy_target="skip", shorten_min_fraction=0.5):
            out, st = res.step(raw_hh, last_hh, BASE)
        check("when the largest passing fraction is below the minimum the result is hold-still (SHORTEN_ZERO)",
              st.status == "HOLD" and st.hold_reason == "SHORTEN_ZERO" and np.array_equal(out, last_hh), f"({st.status} {st.hold_reason})")
    finally:
        link_rules.PART_CATEGORY["palm"], link_rules.PART_CATEGORY["link7"], link_rules.PART_CATEGORY["finger"] = old

    print("forced failure of each check gives hold-still with the right reason")
    forced = {}
    with patched(cfg.__dict__, BRANCH_GUARD_RAD=1e-9):
        forced["BRANCH_GUARD"] = res.step(raw_hh, last_hh, BASE)
    with patched(cfg.RESOLVER, stop_m=-1.0, shorten_after_failed_push=False):
        forced["SHAPE_RECHECK"] = res.step(raw_hh, last_hh, BASE)
    saved = kin.limits["left"].copy()
    kin.limits["left"] = np.tile([[-1e-3, 1e-3]], (7, 1))
    try:
        forced["JOINT_LIMITS"] = res.step(raw_hh, last_hh, BASE)
    finally:
        kin.limits["left"] = saved
    with patched(cfg.RESOLVER, accuracy_pos_m=0.0):
        forced["ACCURACY"] = res.step(raw_hh, last_hh, BASE)
    for reason, (out, st) in forced.items():
        check(f"{reason}: HOLD with that reason, the last filtered command comes back",
              st.status == "HOLD" and st.hold_reason == reason and np.array_equal(out, last_hh) and out is not last_hh, f"({st.status} {st.hold_reason})")

    print("arm against arm")
    out, st = res.step(raw_hh, last_hh, BASE)
    check("only the closing (left) arm moves, the right arm is exactly as it was", not np.array_equal(out[0], raw_hh[0]) and np.array_equal(out[1], last_hh[1]))
    l_last = left_hand_at_y(res, 0.07)[0]
    mirror_last = np.array([l_last, l_last * MIRROR])
    l_raw = left_hand_at_y(res, 0.064)[0]
    mirror_raw = np.array([l_raw, l_raw * MIRROR])
    g_m = hand_hand_gap(mirror_last)[0]
    with patched(cfg.RESOLVER, rounds=1, shorten_after_failed_push=False, accuracy_pos_m=0.01, stop_m=0.1):   # one round: the worst pair is the mirrored palm pair, so the shares are visible
        out_b, st_b = res.step(mirror_raw, mirror_last, BASE)
    # mirrored arms: right = MIRROR x left holds for the zombie pose; the pose built this way is the mirror image of the left arm at the same y
    pl, _ = kin.hand_pose(state_of(mirror_raw), "left")
    pr, _ = kin.hand_pose(state_of(mirror_raw), "right")
    check("mirror setup: the right hand is the mirror image of the left hand", np.allclose(pl * np.array([1, -1, 1]), pr, atol=1e-6), f"({np.abs(pl * np.array([1, -1, 1]) - pr).max():.1e})")
    check("both arms closing: both are acted on", st_b.status in ("HAND_SLIDE", "ELBOW_SWING") and not np.array_equal(out_b[0], mirror_raw[0])
          and not np.array_equal(out_b[1], mirror_raw[1]), f"({st_b.status} {st_b.hold_reason})")
    t_l, t_r = st_b.hand_targets["left"][0], st_b.hand_targets["right"][0]
    d_l = np.linalg.norm(t_l - kin.hand_pose(state_of(mirror_last), "left")[0])
    d_r = np.linalg.norm(t_r - kin.hand_pose(state_of(mirror_last), "right")[0])
    check("both arms closing: equal shares (the two hands' pushed-back targets are mirror images within 1 mm; the fitted palm capsules of the two arms are not exact mirrors)",
          np.linalg.norm(t_l - t_r * np.array([1, -1, 1])) < 0.001 and abs(d_l - d_r) < 0.001,
          f"(target mirror error {np.linalg.norm(t_l - t_r * np.array([1, -1, 1]))*1000:.2f} mm, step {d_l*1000:.2f} / {d_r*1000:.2f} mm)")
    out_p, st_p = res.step(raw_hh, raw_hh, BASE)
    check("a parked arm pair (raw equals last) is not touched even deep in the band", out_p is raw_hh and st_p.status == "PASS")

    print("rounds")
    rounds = [t[3].rounds for t in ticks]
    check("rounds never exceed 10", max(rounds) <= 10, f"(max {max(rounds)})")
    with patched(cfg.RESOLVER, rounds=3):
        r3 = [t[3].rounds for t in run_stream(res, convergence_stream(res)[30:50])]
    check("the rounds setting is the limit", max(r3) <= 3, f"(max {max(r3)})")
    ok_stop = [t for t in ticks if t[3].status in ("ELBOW_SWING", "HAND_SLIDE") and t[3].rounds < 10]
    check("a tick that stopped early ended with the worst shortfall under 0.5 mm", len(ok_stop) > 0 and all(
        _final_short(res, t) < cfg.RESOLVER["stop_m"] for t in ok_stop[:20]), f"({len(ok_stop)} early stops)")

    print("determinism")
    stream = convergence_stream(res)
    a = run_stream(new_resolver(), stream)
    b = run_stream(new_resolver(), stream)
    check("the same inputs give the same outputs twice (bit for bit)", all(np.array_equal(x[2], y[2]) and x[3].status == y[3].status and x[3].hold_reason == y[3].hold_reason
                                                                            for x, y in zip(a, b)))

    print("paper_plain switch")
    last_band, raw_band = left_hand_at_y(res, 0.040), left_hand_at_y(res, 0.034)      # closing inside the band, still above the margin
    gb = hand_hand_gap(raw_band)[0]
    out_o, st_o = res.step(raw_band, last_band, BASE)
    with patched(cfg.GAP_RULE, mode="paper_plain"):
        out_pl, st_pl = res.step(raw_band, last_band, BASE)
    check("the switch changes the result of the same tick (closing inside the band: ours limits the speed; plain waits for the margin of every pair)",
          (st_o.status, st_o.hold_reason) != (st_pl.status, st_pl.hold_reason),
          f"(hand gap {gb*1000:.1f} mm; ours {st_o.status} {st_o.hold_reason}, plain {st_pl.status} {st_pl.hold_reason})")

    print("SEW points and the converter against Kong's own forward kinematics")
    rng = np.random.default_rng(1)
    offs = []
    lim = kin.limits
    for _ in range(5):
        q = np.array([rng.uniform(lim[s][:, 0], lim[s][:, 1]) * 0.5 for s in ("left", "right")])
        fk = solver().session.robot_fk_at(q_right=q[1], q_left=q[0])
        for side in ("left", "right"):
            S, E, W = kin.sew(state_of(q), side)
            f = fk[side]
            offs.append(np.concatenate([S - f["S"], E - f["E"], W - f["W"]]))
    offs = np.array(offs)
    check("S, E, W of our model equal the solver's up to one constant offset (spread < 1e-6 m)", float(np.abs(offs - offs.mean(axis=0)).max()) < 1e-6,
          f"({np.abs(offs - offs.mean(axis=0)).max():.1e})")
    q0 = HOME.copy()
    u = kin.data.xaxis[kin.jids["left"][2]].copy()
    kin.set_state(state_of(q0))
    u, l = kin.data.xaxis[kin.jids["left"][2]].copy(), kin.data.xaxis[kin.jids["left"][4]].copy()
    _, r7 = kin.hand_pose(state_of(q0), "left")
    qc = kin.convert_directions(state_of(q0), "left", u, l, r7)
    check("the converter given the pose's own directions returns the same joints (round trip, 1e-4 rad)", np.abs(qc - q0[0]).max() < 1e-4, f"({np.abs(qc - q0[0]).max():.1e})")
    out, st = new_resolver(with_solver=False).step(*_swing_inputs(res, ticks))
    check("an elbow swing without a converter holds still (NO_SOLVER)", st.status == "HOLD" and st.hold_reason == "NO_SOLVER", f"({st.status} {st.hold_reason})")

    print("recording")
    with tempfile.TemporaryDirectory() as d:
        rec = FilterRecorder(Path(d) / "rec", DT, chunk_ticks=20)
        for raw, last, out, st in ticks:
            rec.tick(raw, out, st)
        rec.close()
        r = load_filter_recording(Path(d) / "rec")
        check("recording: raw, filtered and status of every tick come back", len(r["status"]) == len(ticks) and r["meta"]["complete"]
              and np.array_equal(r["raw"][5], ticks[5][0]) and np.array_equal(r["filtered"][5], ticks[5][2]) and r["status"][5] == ticks[5][3].status)
    print("PASS" if not FAILS else f"FAIL: {FAILS}")
    return 1 if FAILS else 0


def _final_short(res, tick):
    raw, last, out, st = tick
    t = _Tick(res, np.array(raw), np.array(last), BASE, [], None)
    t.discover(t.last)
    t.discover(t.raw)
    return t.worst_value_discover(np.array(out))


def _swing_inputs(res, ticks):
    for raw, last, out, st in ticks:
        if st.status == "ELBOW_SWING":
            return raw, last, BASE
    raise RuntimeError("no elbow swing in the stream")


if __name__ == "__main__":
    sys.exit(main())
