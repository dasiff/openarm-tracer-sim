"""Safety filter Stage 2: the resolver against the spec (gap rule numbers, pass-through, hand slide, elbow swing,
step shortening, forced holds, arm against arm, round limit, determinism) and the Stage 1 tests as subprocesses.
Written from the spec, not from the resolver code. Fixtures: experiments/safety_filter_resolver_fixtures.py. No physics.

Run: source .venv/bin/activate; python experiments/safety_filter_spec_test.py   (exit 0 = pass)
"""

import copy
import subprocess
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "experiments"))

import safety_filter_resolver_fixtures as fx  # noqa: E402
from src.safety_filter import config as cfg  # noqa: E402
cfg.RESOLVER["mode"] = "layered"            # these tests are about the layered resolver (SF-40); kong_style is tested in safety_filter_kong_style_test.py
cfg.RESOLVER["elbow_swing_hand_target"] = "raw"  # the old behaviour these tests were written for (SF-49)
from src.safety_filter import gap_rule  # noqa: E402
from src.safety_filter.shapes import RobotState  # noqa: E402

FAILS = []
RESULTS = {"run": 0, "ok": 0}
STAGE1_TESTS = (
    "safety_filter_structure_test.py",
    "safety_filter_distance_test.py",
    "safety_filter_shapes_test.py",
    "safety_filter_touch_test.py",
)


def check(name, cond, detail=""):
    RESULTS["run"] += 1
    RESULTS["ok"] += bool(cond)
    print(f"  {'ok  ' if cond else 'FAIL'} {name} {detail}")
    if not cond:
        FAILS.append(name)


def note(text):
    print(f"  NOTE {text}")


def near(a, b, tol=1e-9):
    return abs(a - b) <= tol


def rot_angle(R1, R2):
    """Angle in rad between two rotation matrices."""
    return float(np.linalg.norm(Rotation.from_matrix(np.asarray(R1) @ np.asarray(R2).T).as_rotvec()))


def hand(res, q, side):
    """Hand position and rotation of one arm at the joint pair q."""
    return res.kin.hand_pose(fx.state_of(q), side)


def hand_hand_scenario(res):
    """Spec scenario: left hand closing from lateral y 0.034 to 0.026 (hand-hand gap about 15 mm to 7 mm), right arm at HOME."""
    return fx.left_hand_at_y(res, 0.034), fx.left_hand_at_y(res, 0.026)


def head_on_targets(res, y_start=0.13, y_stop=0.09, step=0.001):
    """Commands of a head-on stream: the left hand moves from lateral y_start toward the midline, 1 mm a tick, joints by
    Jacobian IK with warm start; the right arm is the mirror image of the left arm (fx.MIRROR)."""
    p_home, r_home = hand(res, fx.HOME, "left")
    prev = fx.HOME.copy()
    targets = []
    for k in range(int(round((y_start - y_stop) / step)) + 1):
        y = y_start - k * step
        q = fx.ik(res, prev, "left", np.array([p_home[0], y, p_home[2]]), r_home)
        raw = np.array([q[0], q[0] * fx.MIRROR])
        targets.append((y, raw))
        prev = raw
    return targets


def run_stream(res, targets):
    """Each tick's output becomes the next tick's last command (the filter's own state)."""
    last = targets[0][1].copy()
    ticks = []
    for y, raw in targets[1:]:
        out, st = res.step(raw, last, RobotState())
        ticks.append({"y": y, "last": last, "raw": raw, "out": out, "st": st})
        last = np.array(out, dtype=float, copy=True)
    return ticks


def set_module(name, value):
    """Apply: change one config value; returns the restore function."""
    def apply(res):
        saved = getattr(cfg, name)
        setattr(cfg, name, value)
        return lambda: setattr(cfg, name, saved)
    return apply


def set_resolver(**changes):
    """Apply: change some cfg.RESOLVER values; returns the restore function."""
    def apply(res):
        saved = {k: cfg.RESOLVER[k] for k in changes}
        cfg.RESOLVER.update(changes)
        return lambda: cfg.RESOLVER.update(saved)
    return apply


def joint_limits(res):
    """Apply: left-arm joint limits of +-1 mrad; returns the restore function."""
    saved = res.kin.limits["left"]
    res.kin.limits["left"] = np.tile([[-1e-3, 1e-3]], (7, 1))

    def restore():
        res.kin.limits["left"] = saved
    return restore


def hold_case(label, code, apply):
    """Forced failure: the scenario must hold still (output equal to last) with the given reason code."""
    res = fx.new_resolver()
    last, raw = hand_hand_scenario(res)
    restore = apply(res)
    try:
        out, st = res.step(raw, last, RobotState())
    finally:
        restore()
    ok = st.status == "HOLD" and st.hold_reason == code and np.array_equal(out, last)
    check(f"forced {label}: hold still with reason {code}", ok,
          f"(status {st.status}, reason {st.hold_reason!r}, rounds {st.rounds})")


def sec_pass_through():
    print("1. pass-through")
    res = fx.new_resolver()
    raw = fx.HOME.copy()
    last = fx.HOME.copy()
    out, st = res.step(raw, last, RobotState())
    check("raw equal to last at HOME: the same array object comes back, status PASS",
          out is raw and st.status == "PASS", f"(status {st.status})")


def sec_gap_rule():
    print("2. gap rule numbers (margin 0.03 m, one tick 1/150 s)")
    margin, dt = 0.03, 1 / 150
    saved = cfg.GAP_RULE["mode"]
    try:
        cfg.GAP_RULE["mode"] = "ours"
        check("a gap 2 cm above the margin allows 0.6 m/s",
              near(gap_rule.allowed_approach_speed(margin + 0.02, margin), 0.6))
        check("a gap 1 mm inside the margin pushes at 0.03 m/s",
              near(gap_rule.required_grow_speed(margin - 0.001, margin), 0.03))
        check("a gap 5 cm inside the margin pushes at the 0.05 m/s cap",
              near(gap_rule.required_grow_speed(margin - 0.05, margin), 0.05))
        ours = gap_rule.shortfall(margin + 0.015, margin + 0.009, margin, dt)
        cfg.GAP_RULE["mode"] = "paper_plain"
        plain = gap_rule.shortfall(margin + 0.015, margin + 0.009, margin, dt)
        check("mode paper_plain changes behaviour: a pair closing above its margin is short under ours, not under paper_plain",
              ours > 0.0 and plain == 0.0, f"(ours {ours * 1000:.2f} mm, paper_plain {plain * 1000:.2f} mm)")
    finally:
        cfg.GAP_RULE["mode"] = saved


def sec_hand_slide():
    print("3. hand slide (hand against hand)")
    res = fx.new_resolver()
    last, raw = hand_hand_scenario(res)
    g_last = fx.hand_hand_gap(last)[0]
    g_raw = fx.hand_hand_gap(raw)[0]
    check("scenario: hand-hand gap about 15 mm at last and about 7 mm at raw (inside the 10 mm margin)",
          abs(g_last - 0.015) <= 0.001 and abs(g_raw - 0.007) <= 0.001 and g_raw < 0.010,
          f"(last {g_last * 1000:.2f} mm, raw {g_raw * 1000:.2f} mm)")
    out, st = res.step(raw, last, RobotState())
    check("status is HAND_SLIDE", st.status == "HAND_SLIDE", f"(status {st.status})")

    raw_rot = hand(res, raw, "left")[1]
    e_target = rot_angle(st.hand_targets["left"][1], raw_rot)
    check("hand orientation target equals the commanded (raw) orientation within 1e-6 rad",
          e_target <= 1e-6, f"(error {e_target:.1e} rad)")
    e_real = rot_angle(hand(res, out, "left")[1], raw_rot)
    check("realized hand orientation (forward kinematics of the output) within 1e-6 rad of the commanded one",
          e_real <= 1e-6, f"(error {e_real:.2e} rad = {np.degrees(e_real):.4f} deg)")

    p_last = hand(res, last, "left")[0]
    p_out = hand(res, out, "left")[0]
    p_raw = hand(res, raw, "left")[0]
    n = p_last - hand(res, last, "right")[0]
    n = n / np.linalg.norm(n)   # unit vector from the right hand to the left hand (the line between the hands)
    closing = -float(np.dot(p_out - p_last, n))      # m, positive = toward the other hand
    closing_raw = -float(np.dot(p_raw - p_last, n))
    check("motion into the obstacle is removed: the output closes on the other hand by at most 1 mm",
          closing <= 0.001, f"(output {closing * 1000:.3f} mm, raw command {closing_raw * 1000:.2f} mm)")

    def tangential(v):
        return v - np.dot(v, n) * n
    surf = float(np.linalg.norm(tangential(p_out - p_last) - tangential(p_raw - p_last)))
    check("motion along the surface is kept within 2 mm (spec scenario)", surf <= 0.002, f"(difference {surf * 1000:.3f} mm)")

    check("the other arm is untouched: its output equals its command and its last command",
          np.array_equal(out[1], raw[1]) and np.array_equal(out[1], last[1]))

    # same push with 10 mm of motion along the surface (z) added to the command
    p_home, r_home = hand(res, fx.HOME, "left")
    raw_t = fx.ik(res, fx.HOME, "left", np.array([p_home[0], 0.026, p_home[2] + 0.010]), r_home)
    out_t, st_t = fx.new_resolver().step(raw_t, last, RobotState())
    surf_t = float(np.linalg.norm(
        tangential(hand(res, out_t, "left")[0] - p_last) - tangential(hand(res, raw_t, "left")[0] - p_last)))
    check("motion along the surface is kept within 2 mm (command with 10 mm along the surface)",
          st_t.status == "HAND_SLIDE" and surf_t <= 0.002,
          f"(status {st_t.status}, difference {surf_t * 1000:.3f} mm)")


def sec_elbow_and_shortening(ticks):
    print("4. elbow swing and 5. step shortening (head-on stream, mirrored arms)")
    res = fx.new_resolver()
    el = next((t for t in ticks if t["st"].status == "ELBOW_SWING"), None)
    check("the head-on stream has an ELBOW_SWING tick", el is not None,
          f"(first at y {el['y']:.3f} m)" if el is not None else "")
    if el is not None:
        for i, side in enumerate(("left", "right")):
            if np.array_equal(el["out"][i], el["last"][i]):
                continue
            p_o, R_o = hand(res, el["out"], side)
            p_r, R_r = hand(res, el["raw"], side)
            pos_err = float(np.linalg.norm(p_o - p_r))
            rot_err = float(np.degrees(rot_angle(R_o, R_r)))
            check(f"elbow swing at y {el['y']:.3f} m ({side} arm): hand within 2 mm and 0.5 deg of the raw commanded pose",
                  pos_err <= 0.002 and rot_err <= 0.5, f"(position {pos_err * 1000:.4f} mm, orientation {rot_err:.4f} deg)")

    st_tick = next((t for t in ticks if t["st"].status == "STEP_SHORTENED"), None)
    check("the head-on stream has a STEP_SHORTENED tick", st_tick is not None,
          f"(first at y {st_tick['y']:.3f} m)" if st_tick is not None else "")
    if st_tick is not None:
        for i, side in enumerate(("left", "right")):
            d = st_tick["raw"][i] - st_tick["last"][i]
            e = st_tick["out"][i] - st_tick["last"][i]
            if not np.any(d):
                continue
            f = float(np.dot(e, d) / np.dot(d, d))
            resid = float(np.linalg.norm(e - f * d))
            k = f * 4096
            check(f"step shortening ({side}): the shortened step lies on the raw step (joint space)",
                  resid <= 1e-9, f"(residual {resid:.1e})")
            check(f"step shortening ({side}): the fraction is on the 1/4096 grid and strictly between 0 and 1",
                  abs(k - round(k)) <= 1e-6 and 0.0 < f < 1.0, f"(fraction {round(k)}/4096 = {f:.6f})")
    note("step shortening: that the chosen fraction is the LARGEST passing one cannot be checked from outside the "
         "resolver (its search is internal); only the grid and the straight step are checked")


def sec_forced_holds():
    print("6. forced failures: hold still with the reason code")
    hold_case("BRANCH_GUARD", "BRANCH_GUARD", set_module("BRANCH_GUARD_RAD", 1e-6))
    hold_case("JOINT_LIMITS", "JOINT_LIMITS", joint_limits)
    hold_case("ACCURACY", "ACCURACY", set_resolver(accuracy_pos_m=0.0))
    hold_case("SHAPE_RECHECK", "SHAPE_RECHECK", set_resolver(stop_m=-1.0, shorten_after_failed_push=False))


def sec_arms():
    print("7. arm against arm")
    res = fx.new_resolver()
    last, raw = hand_hand_scenario(res)
    out, st = res.step(raw, last, RobotState())
    check("only the closing arm moves: the parked right arm's output exactly equals its last command",
          np.array_equal(out[1], last[1]), f"(status {st.status})")
    check("the closing left arm moves", not np.array_equal(out[0], last[0]))

    # mirror case: the right arm closes, the left arm is parked at HOME
    p_r, r_r = hand(res, fx.HOME, "right")
    res_m = fx.new_resolver()
    last_r = fx.ik(res_m, fx.HOME, "right", np.array([p_r[0], -0.034, p_r[2]]), r_r)
    raw_r = fx.ik(res_m, fx.HOME, "right", np.array([p_r[0], -0.026, p_r[2]]), r_r)
    out_r, st_r = fx.new_resolver().step(raw_r, last_r, RobotState())
    check("parked arm not touched: the left arm (raw equal to last) keeps exactly its command while the right arm closes",
          np.array_equal(raw_r[0], last_r[0]) and np.array_equal(out_r[0], last_r[0]), f"(status {st_r.status})")
    check("the closing right arm moves", not np.array_equal(out_r[1], last_r[1]))

    # both closing: mirrored commands
    a = fx.left_hand_at_y(res, 0.07)[0]
    b = fx.left_hand_at_y(res, 0.064)[0]
    last_m = np.array([a, a * fx.MIRROR])
    raw_m = np.array([b, b * fx.MIRROR])
    res_o = fx.new_resolver()
    out_m, st_m = res_o.step(raw_m, last_m, RobotState())
    both = (not np.array_equal(out_m[0], last_m[0])) and (not np.array_equal(out_m[1], last_m[1]))
    mirror_err = float(np.linalg.norm(
        hand(res, out_m, "right")[0] - np.diag([1.0, -1.0, 1.0]) @ hand(res, out_m, "left")[0]))
    check("both closing (mirrored): both arms move and the two outputs are mirror images at the hand within 1 mm",
          both and mirror_err <= 0.001, f"(status {st_m.status}, mirror error {mirror_err * 1000:.3f} mm)")
    start_gap = fx.hand_hand_gap(last_m)[0]
    note(f"the spec's mirrored start is already {start_gap * 1000:.1f} mm (negative = overlapping) at the hand-hand pair, "
         "so this case checks the shares of an overlapping start, not a clean approach")


def sec_rounds(targets, ticks):
    print("8. rounds")
    rmax = max(t["st"].rounds for t in ticks)
    check("rounds never exceed 10 in the head-on stream", rmax <= 10, f"(max {rmax})")
    saved = cfg.RESOLVER["rounds"]
    try:
        cfg.RESOLVER["rounds"] = 2
        low = run_stream(fx.new_resolver(), targets)
    finally:
        cfg.RESOLVER["rounds"] = saved
    lmax = max(t["st"].rounds for t in low)
    check("a lowered cfg.RESOLVER['rounds'] (2) is the limit in the same stream", lmax <= 2, f"(max {lmax})")


def sec_determinism(targets, ticks):
    print("9. determinism")
    targets_b = head_on_targets(fx.new_resolver())
    ticks_b = run_stream(fx.new_resolver(), targets_b)
    same_cmd = len(targets) == len(targets_b) and all(np.array_equal(a[1], b[1]) for a, b in zip(targets, targets_b))
    same = len(ticks) == len(ticks_b) and all(
        np.array_equal(a["out"], b["out"]) and a["st"].status == b["st"].status and a["st"].rounds == b["st"].rounds
        for a, b in zip(ticks, ticks_b))
    check("head-on stream: two fresh runs give identical commands, outputs, statuses and round counts at every tick",
          same_cmd and same, f"({len(ticks)} ticks)")

    last, raw = hand_hand_scenario(fx.new_resolver())
    out1, st1 = fx.new_resolver().step(raw, last, RobotState())
    out2, st2 = fx.new_resolver().step(raw, last, RobotState())
    check("hand slide scenario: two fresh resolvers give identical outputs and status",
          np.array_equal(out1, out2) and st1.status == st2.status, f"(status {st1.status})")


def sec_stage1():
    print("10. Stage 1 tests (subprocess, exit code 0)")
    for name in STAGE1_TESTS:
        r = subprocess.run([sys.executable, str(ROOT / "experiments" / name)], cwd=str(ROOT),
                           capture_output=True, text=True)
        check(f"{name} exits 0", r.returncode == 0, f"(exit {r.returncode})")
        if r.returncode != 0:
            for line in (r.stdout + r.stderr).strip().splitlines()[-5:]:
                print(f"      {line}")


def main():
    snapshot = (copy.deepcopy(cfg.GAP_RULE), copy.deepcopy(cfg.RESOLVER), cfg.BRANCH_GUARD_RAD)
    sec_pass_through()
    sec_gap_rule()
    sec_hand_slide()
    targets = head_on_targets(fx.new_resolver())
    ticks = run_stream(fx.new_resolver(), targets)
    sec_elbow_and_shortening(ticks)
    sec_forced_holds()
    sec_arms()
    sec_rounds(targets, ticks)
    sec_determinism(targets, ticks)
    check("every config setting is back to its value at the start",
          (cfg.GAP_RULE, cfg.RESOLVER, cfg.BRANCH_GUARD_RAD) == snapshot)
    sec_stage1()
    print(f"{RESULTS['ok']} of {RESULTS['run']} checks passed")
    print("PASS" if not FAILS else f"FAIL: {FAILS}")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
