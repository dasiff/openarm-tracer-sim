"""Analytic tests of the Gaussian stance sampler and the place-angle middle rule (David, 2026-11-10; decision M299). No simulator.

    source .venv/bin/activate && python experiments/gaussian_sampling_test.py     (exit 0 when every check passes)

What is checked:
  1. The A8 score at hand-computed points (src/reach_model A8 values, written out here): on the axis at the peak distance, on the narrow side, and z below the cutoff (0).
  2. The vector score equals reach_model.counts_at for the same stance (the same formula, a synthetic robot geometry with a shoulder offset and a pedestal height).
  3. The score of a stance is the LOWER of its task points' counts (the min combination).
  4. Weighting: every draw of a batch has a positive score (a zero-score stance is never drawn); a batch with no positive score stops with a reason.
  5. Replay: the same seed gives the same batch and the same draws.
  6. The 10-draw cap: a check that always fails is called exactly sampler_max_draws times, each stance at most once, and the selection reports failure.
  7. The place-angle middle rule: odd window, even window (the tie takes the counter-clockwise one), a window that wraps past 0 deg; the full check's window and the robot's angle agree.
"""
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np  # noqa: E402

from src import reach_model as rm  # noqa: E402
from src.approach_angle import choose_place_angle  # noqa: E402
from src.full_check import choose_place_window  # noqa: E402
from src.stance_sampling import a8_scores, select_stance  # noqa: E402

results = []


def check(name, ok, detail=""):
    results.append(bool(ok))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}")


CFG_HEAD = {"sampler_heading_half_width_deg": 45.0, "sampler_heading_held": True}
CFG = {"sampler_batch": 400, "sampler_weight_power": 2, "sampler_max_draws": 10, "sampler_margin_sigmas": 2.0, "pedestal_range": (0.05, 0.25)}
# a synthetic robot: the left shoulder 0.1 m forward and 0.2 m left of the torso origin, 0.3 m up; the torso 0.5 m above the base origin at pedestal 0
GEOM = {"shoulder": {"left": (0.1, 0.2, 0.3), "right": (0.1, -0.2, 0.3)}, "torso_z0": 0.5, "lift_factor": 1.0, "base_z": 0.15}


def a8_count_by_hand(r, theta, z, arm_sigmas):
    """The A8 count written out from the fitted numbers (not from the module's function)."""
    C, r0, sr, z_cut = rm.A8["C"], rm.A8["r0"], rm.A8["sr"], rm.A8["z_cut"]
    if z < z_cut:
        return 0.0
    (n0, n1), (w0, w1) = rm.A8["narrow"], rm.A8["wide"]
    sig = (n0 + n1 * z) if theta < 0 else (w0 + w1 * z)
    return C * math.exp(-((r - r0) / sr) ** 2) * math.exp(-(theta / sig) ** 2)


def main():
    print("1. A8 score at hand-computed points (left arm; the shoulder at the origin, pedestal 0, heading 0)")
    geom0 = {"shoulder": {"left": (0.0, 0.0, 0.0)}, "torso_z0": 0.0, "lift_factor": 1.0, "base_z": 0.0}
    r0 = rm.A8["r0"]
    s_peak = a8_scores(np.array([0.0]), np.array([0.0]), np.array([0.0]), 0.0, [(r0, 0.0, 0.0)], geom0, "left")[0]
    check("on the axis at the peak distance: count = C", abs(s_peak - rm.A8["C"]) < 1e-9, f"{s_peak:.6f} vs C {rm.A8['C']:.6f}")
    th = -0.2                                                      # theta < 0 is the narrow side for the left arm
    p = (r0 * math.cos(th), r0 * math.sin(th), 0.0)
    s_narrow = a8_scores(np.array([0.0]), np.array([0.0]), np.array([0.0]), 0.0, [p], geom0, "left")[0]
    exp_narrow = a8_count_by_hand(r0, th, 0.0, None)
    check("narrow side (theta -0.2 rad at the peak distance, z 0)", abs(s_narrow - exp_narrow) < 1e-9, f"{s_narrow:.6f} vs hand {exp_narrow:.6f}")
    z = -0.3                                                       # above the cutoff (-0.375)
    p2 = (r0 * math.cos(0.0), 0.0, z)
    d = math.hypot(r0, z)
    s_z = a8_scores(np.array([0.0]), np.array([0.0]), np.array([0.0]), 0.0, [p2], geom0, "left")[0]
    exp_z = a8_count_by_hand(d, 0.0, z, None)
    check("z above the cutoff (-0.30 m, on the axis)", abs(s_z - exp_z) < 1e-9 and s_z > 0, f"{s_z:.6f} vs hand {exp_z:.6f}")
    p3 = (r0, 0.0, -0.4)                                           # below the cutoff (-0.375): zero
    s_low = a8_scores(np.array([0.0]), np.array([0.0]), np.array([0.0]), 0.0, [p3], geom0, "left")[0]
    check("z below the cutoff (-0.40 m) gives 0", s_low == 0.0, f"{s_low}")

    print("2. the vector score equals reach_model.counts_at (synthetic robot, a stance with heading and pedestal)")
    rng = np.random.default_rng(5)
    xs, ys, qs = rng.uniform(0.2, 1.4, 30), rng.uniform(0.2, 1.4, 30), rng.uniform(0.05, 0.25, 30)
    heading = math.radians(-90.0)
    pts = [(0.9, 1.0, 1.05), (1.1, 0.7, 1.02)]
    vec = a8_scores(xs, ys, qs, heading, pts, GEOM, "left")
    ref = np.array([min(rm.counts_at((xs[i], ys[i]), "left", heading, qs[i], GEOM, pts)) for i in range(len(xs))])
    check("all 30 stances agree with counts_at (the lower of the two points)", np.allclose(vec, ref, atol=1e-9), f"max |diff| {np.max(np.abs(vec - ref)):.2e}")
    check("some stances score above zero in this set", float(np.max(vec)) > 0.0, f"max {np.max(vec):.3f}")

    print("3. the stance's score is the lower of its task points' counts")
    one = a8_scores(np.array([0.5]), np.array([0.6]), np.array([0.12]), heading, pts, GEOM, "left")[0]
    each = [a8_scores(np.array([0.5]), np.array([0.6]), np.array([0.12]), heading, [q], GEOM, "left")[0] for q in pts]
    check("min over the two points", abs(one - min(each)) < 1e-12, f"score {one:.4f}, points {each[0]:.4f} and {each[1]:.4f}")

    print("4. weighting: a zero-score stance is never drawn; no positive score stops with a reason")
    far_pts = [(0.9, 1.0, 1.05)]
    seen = []

    def always_pass(stance):
        seen.append(stance)
        return {"ok": True, "failed": [], "reason": "", "seconds": 0.0, "seed": 1}

    rec = select_stance(far_pts, GEOM, "left", heading, CFG, always_pass, seed=7, say=lambda s: None)
    drawn_scores = [row["score"] for row in rec["draws"]]
    check("every drawn stance had a positive score", rec["ok"] and all(s > 0.0 for s in drawn_scores), f"scores {np.round(drawn_scores, 3).tolist()}")
    far = select_stance([(0.9, 1.0, 7.0)], GEOM, "left", heading, CFG, always_pass, seed=7, say=lambda s: None)      # 7 m above the shoulder: every count underflows to 0
    check("a batch with no positive score stops with the zero-weight reason", (not far["ok"]) and "zero weight" in str(far["stop"]), f"stop: {far['stop']}")

    print("5. replay: the same seed gives the same batch and the same draws")

    def fail_then_pass(stance):
        return {"ok": len(seen_b) >= 3, "failed": [] if len(seen_b) >= 3 else ["x"], "reason": "", "seconds": 0.0, "seed": 1}

    seen_b = []

    def check_b(stance):
        seen_b.append(stance)
        return fail_then_pass(stance)

    a = select_stance(far_pts, GEOM, "left", heading, CFG, check_b, seed=12345, say=lambda s: None)
    seen_b.clear()
    b = select_stance(far_pts, GEOM, "left", heading, CFG, check_b, seed=12345, say=lambda s: None)
    same = [(r["index"], r["x"], r["y"], r["pedestal"]) for r in a["draws"]] == [(r["index"], r["x"], r["y"], r["pedestal"]) for r in b["draws"]]
    check("same seed: same draws (index, x, y, pedestal)", same and a["ok"] == b["ok"], f"{len(a['draws'])} draws")
    c = select_stance(far_pts, GEOM, "left", heading, CFG, always_pass, seed=99, say=lambda s: None)
    check("a different seed gives different draws", [r["index"] for r in c["draws"]] != [r["index"] for r in rec["draws"]] or c["draws"][0]["x"] != rec["draws"][0]["x"], "")

    print("6. the 10-draw cap: a check that always fails is called exactly 10 times, no stance twice")
    calls = []

    def always_fail(stance):
        calls.append(stance)
        return {"ok": False, "failed": ["body"], "reason": "test", "seconds": 0.0, "seed": 1}

    capped = select_stance(far_pts, GEOM, "left", heading, {**CFG, "sampler_max_draws": 10}, always_fail, seed=3, say=lambda s: None)
    idx = [r["index"] for r in capped["draws"]]
    check("exactly 10 checks, selection fails", len(calls) == 10 and not capped["ok"] and capped["chosen"] is None and "10 draws" in str(capped["stop"]), f"calls {len(calls)}, stop: {capped['stop']}")
    check("no stance is drawn twice", len(set(idx)) == len(idx), f"indices {idx}")

    print("7. the place-angle middle rule (5 deg turns)")
    check("odd window 0..10: the middle turn 5", choose_place_angle([0, 5, 10]) == 5)
    check("odd window -10..10: the middle turn 0", choose_place_angle([-10, -5, 0, 5, 10]) == 0)
    got = choose_place_angle([0, 5, 10, 15])
    check("even window 0..15: exactly the later of the two middle turns (10)", got == 10, f"got {got}")
    w = choose_place_window([0, 5, 10, 15], 5, 10)
    check("the full check's window 0..15 gives theta 10 and the same window list", w["theta_deg"] == 10 and w["window_turns"] == [0, 5, 10, 15], f"theta {w['theta_deg']}, turns {w['window_turns']}")
    wrap = choose_place_window([345, 350, 355, 0, 5], 5, 10)
    check("a window across 0 (345..5, one window): theta -5 (midpoint 355)", wrap["theta_deg"] == -5 and wrap["window_turns"] == [-15, -10, -5, 0, 5], f"theta {wrap['theta_deg']}, turns {wrap['window_turns']}")
    check("the robot's angle is the check's angle (wrapped window)", choose_place_angle(wrap["window_turns"]) == wrap["theta_deg"], "")

    print("8. the heading rule (Revision 2): every stance in a drawn batch faces the task midpoint within 45 deg; the held heading is one value")
    from src.stance_sampling import draw_batch, facing_heading
    half = CFG_HEAD["sampler_heading_half_width_deg"]
    b = draw_batch(far_pts + [(0.5, 0.9, 0.1)], {**CFG, "sampler_batch": 10000, **CFG_HEAD}, seed=21)
    mid = np.mean([[p[0], p[1]] for p in far_pts + [(0.5, 0.9, 0.1)]], axis=0)
    facing = np.arctan2(mid[1] - b["ys"], mid[0] - b["xs"])
    dev = np.degrees(np.abs((b["headings"] - facing + np.pi) % (2 * np.pi) - np.pi))
    check("all 10000 stances: |heading - facing| <= 45 deg", float(dev.max()) <= half + 1e-9, f"max deviation {dev.max():.6f} deg")
    check("the facing direction is the one from each stance to the midpoint", np.allclose(b["facing"], facing, atol=1e-12), "")
    check("the headings are spread across the 90 deg window (not constant)", float(np.std(b["headings"])) > 0.1, f"std {np.std(b['headings']):.3f} rad")
    held = draw_batch(far_pts, {**CFG, **CFG_HEAD}, seed=21, held_heading=math.radians(-90.0))
    check("held heading: every stance has the held value, no heading drawn", np.all(held["headings"] == math.radians(-90.0)), "")

    print("9. replay with drawn headings: the same seed gives the same stances and draws")
    d1 = select_stance(far_pts, GEOM, "left", None, {**CFG, **CFG_HEAD}, always_pass, seed=77, say=lambda s: None)
    d2 = select_stance(far_pts, GEOM, "left", None, {**CFG, **CFG_HEAD}, always_pass, seed=77, say=lambda s: None)
    check("same seed, drawn headings: same x, y, heading, pedestal in the draws", [(r["x"], r["y"], r["heading"], r["pedestal"]) for r in d1["draws"]] == [(r["x"], r["y"], r["heading"], r["pedestal"]) for r in d2["draws"]] and len(d1["draws"]) > 0, "")

    print(f"\n{sum(results)}/{len(results)} checks passed")
    return all(results)


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
