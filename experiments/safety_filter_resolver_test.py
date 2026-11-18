"""Safety filter Stage 2: the resolver on fixed poses and scripted command streams (no physics).

kong_style is the only resolver. The layered resolver (hand slide, elbow swing) was removed (SF-108), so its checks went with it.
This file keeps the checks that hold for the resolver as a whole: pass-through is the identical array; step shortening (largest
passing fraction, category dispatch, the SHORTEN_ZERO hold); each of the four checks forced to fail gives hold-still with its
reason; arm against arm (only the closing arm moves, equal shares, a parked arm is not touched); rounds limit and stop;
determinism; the paper_plain switch; the SEW points against Kong's own forward kinematics; the converter round trip; the recording.

Run: python experiments/safety_filter_resolver_test.py   (exit 0 = pass). Needs Kong's licensed wheel (Linux) for the elbow swing.
"""

import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from safety_filter_resolver_fixtures import (DT, HOME, MIRROR, hand_hand_gap, ik, left_hand_at_y, new_resolver,  # noqa: E402
                                              solver, state_of)

from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter import link_rules  # noqa: E402
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
    margin_hh = cfg.MARGINS["hand_hand_m"]
    # a hand-hand approach: left hand last at gap 1.5 cm (slowing band), raw at 0.7 cm (inside the 1 cm margin)
    last_hh, raw_hh = left_hand_at_y(res, 0.034), left_hand_at_y(res, 0.026)
    g_last, g_raw = hand_hand_gap(last_hh)[0], hand_hand_gap(raw_hh)[0]
    print(f"setup: hand-hand gap last {g_last*1000:.1f} mm, raw {g_raw*1000:.1f} mm, margin {margin_hh*1000:.0f} mm")

    print("pass-through")
    out, st = res.step(HOME, HOME, BASE)
    check("nothing near: the raw array object comes back, status PASS", out is HOME and st.status == "PASS" and st.rounds == 0)
    far_last, far_raw = HOME.copy(), HOME.copy()
    far_raw[1, 0] += 0.01       # right arm: at HOME the left link 1 joint 3 is 2.9 mm from the torso (checked since Phase F fix 3), so a left move is not far
    out, st = res.step(far_raw, far_last, BASE)
    check("a small move far from everything: identical array (same object, bit for bit)", out is far_raw and st.status == "PASS")
    last_in, raw_in = raw_hh.copy(), raw_hh.copy()
    out, st = res.step(raw_in, last_in, BASE)
    check("a parked pair inside its margin (nobody closing) is not acted on", out is raw_in and st.status == "PASS")

    print("convergence stream (kong_style)")
    ticks = run_stream(res, convergence_stream(res))
    print("  statuses seen in the stream:", sorted({t[3].status for t in ticks}),
          "hold reasons:", sorted({t[3].hold_reason for t in ticks if t[3].hold_reason}))

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
        d_b = raw_hh[0] - last_hh[0]
        f_line = float(np.dot(out[0] - last_hh[0], d_b) / np.dot(d_b, d_b))
        resid = float(np.linalg.norm(out[0] - last_hh[0] - f_line * d_b))
        k_grid = st.fraction * 4096
        check("the shortened step lies on the raw step (joint space)", resid <= 1e-9, f"(residual {resid:.1e})")
        check("the shortening fraction is on the 1/4096 grid", abs(k_grid - round(k_grid)) <= 1e-6, f"(fraction {round(k_grid)}/4096)")
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
    # stop_m -1 makes every active pair short. The braking shrink keeps the candidate pairs to the old cutoffs (the same
    # forcing as safety_filter_kong_style_test.py), so the shape re-check is the reason, not shortening to zero
    with patched(cfg.RESOLVER, stop_m=-1.0), patched(cfg.BRAKING, hand_speed_cap_m_s=0.01, accel_limit_m_s2=0.05):
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
    with patched(cfg.RESOLVER, rounds=1, accuracy_pos_m=0.01):   # one sweep: the mirrored palm pair is pushed first, so the shares are visible
        out_b, st_b = res.step(mirror_raw, mirror_last, BASE)
    # mirrored arms: right = MIRROR x left holds for the zombie pose; the pose built this way is the mirror image of the left arm at the same y
    pl, _ = kin.hand_pose(state_of(mirror_raw), "left")
    pr, _ = kin.hand_pose(state_of(mirror_raw), "right")
    check("mirror setup: the right hand is the mirror image of the left hand", np.allclose(pl * np.array([1, -1, 1]), pr, atol=1e-6), f"({np.abs(pl * np.array([1, -1, 1]) - pr).max():.1e})")
    check("both arms closing: both are acted on", st_b.status == "KONG_PUSH" and not np.array_equal(out_b[0], mirror_raw[0])
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
    ok_stop = [t for t in ticks if t[3].status == "KONG_PUSH" and t[3].rounds < 10]
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
    out, st = new_resolver(with_solver=False).step(raw_hh, last_hh, BASE)
    check("a push that needs the converter holds still without one (NO_SOLVER)", st.status == "HOLD" and st.hold_reason == "NO_SOLVER", f"({st.status} {st.hold_reason})")

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


if __name__ == "__main__":
    sys.exit(main())
