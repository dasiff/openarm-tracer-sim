"""Analytic tests of the Gaussian stance sampler (Revision 3: polar draw) and the place-angle middle rule (David, 2026-11-10; decision M299). No simulator.

    source .venv/bin/activate && python experiments/gaussian_sampling_test.py     (exit 0 when every check passes)

What is checked:
  1. The A8 score at hand-computed points (src/reach_model A8 values, written out here): on the axis at the peak distance, on the narrow side, and z below the cutoff (0).
  2. The vector score equals reach_model.counts_at for the same stance (the same formula, a synthetic robot geometry with a shoulder offset and a pedestal height).
  3. The score of a stance is the LOWER of its task points' counts (the min combination).
  4. Weighting: every draw of a batch has a positive score (a zero-score stance is never drawn); a batch with no positive score stops with a reason.
  5. Replay: the same seed gives the same batch and the same draws.
  6. The 10-draw cap: a check that always fails is called exactly sampler_max_draws times, each stance at most once, and the selection reports failure.
  7. The place-angle middle rule: odd window, even window (the tie takes the counter-clockwise one), a window that wraps past 0 deg; the full check's window and the robot's angle agree.
  8. The polar draw (Revision 3), for every stance of a 10000 batch: the shoulder-to-centre 3D distance lies in [r_in, r_out]; both task points are at least r_in from the shoulder;
     the heading is within 45 deg of facing the centre from the shoulder's floor position; the base places the shoulder where the draw says.
  9. Equal areas: at a fixed pedestal, about half of the floor distances lie below the area midpoint sqrt((d_min^2 + d_max^2) / 2).
 10. Replay of the polar batch: the same seed gives the same batch.
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
from src.stance_sampling import a8_scores, draw_batch, outer_limit_m, select_stance  # noqa: E402

results = []


def check(name, ok, detail=""):
    results.append(bool(ok))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}")


# a synthetic robot: the left shoulder 0.1 m forward and 0.2 m left of the base centre, 0.3 m up in the torso frame; the torso 0.5 m above the base origin at pedestal 0
GEOM = {"shoulder": {"left": (0.1, 0.2, 0.3), "right": (0.1, -0.2, 0.3)}, "torso_z0": 0.5, "lift_factor": 1.0, "base_z": 0.15}
R_IN = 0.0497                                                    # the measured inner limit (src/task_config.py sampler_r_in_m)
CFG = {"sampler_batch": 400, "sampler_weight_power": 2, "sampler_max_draws": 10, "sampler_r_in_m": R_IN, "sampler_outer_sigmas": 2.0,
       "sampler_redraw_factor": 20, "sampler_heading_half_width_deg": 45.0, "pedestal_range": (0.05, 0.25), "sampler_gate_max_failures": 500}


def a8_count_by_hand(r, theta, z):
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
    exp_narrow = a8_count_by_hand(r0, th, 0.0)
    check("narrow side (theta -0.2 rad at the peak distance, z 0)", abs(s_narrow - exp_narrow) < 1e-9, f"{s_narrow:.6f} vs hand {exp_narrow:.6f}")
    z = -0.3                                                       # above the cutoff (-0.375)
    p2 = (r0, 0.0, z)
    s_z = a8_scores(np.array([0.0]), np.array([0.0]), np.array([0.0]), 0.0, [p2], geom0, "left")[0]
    exp_z = a8_count_by_hand(math.hypot(r0, z), 0.0, z)
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

    def always_pass(stance):
        return {"ok": True, "failed": [], "reason": "", "seconds": 0.0, "seed": 1}

    rec = select_stance(pts, GEOM, "left", math.radians(-90.0), CFG, always_pass, seed=7, say=lambda s: None)
    drawn_scores = [row["score"] for row in rec["draws"]]
    check("every drawn stance had a positive score", rec["ok"] and all(s > 0.0 for s in drawn_scores), f"scores {np.round(drawn_scores, 3).tolist()}")
    far = select_stance([(0.9, 1.0, 7.0)], GEOM, "left", math.radians(-90.0), CFG, always_pass, seed=7, say=lambda s: None)
    check("a centre 7 m above the shoulder: every pedestal fails the outer limit, no stance is kept, the cap stops it with a reason", (not far["ok"]) and far["batch"] == 0 and "attempt cap" in str(far["stop"]), f"stop: {far['stop']}")

    print("5. replay: the same seed gives the same batch and the same draws")

    seen_b = []

    def fail_then_pass(stance):
        seen_b.append(stance)
        ok = len(seen_b) >= 3
        return {"ok": ok, "failed": [] if ok else ["x"], "reason": "", "seconds": 0.0, "seed": 1}

    a = select_stance(pts, GEOM, "left", math.radians(-90.0), CFG, fail_then_pass, seed=12345, say=lambda s: None)
    seen_b.clear()
    b = select_stance(pts, GEOM, "left", math.radians(-90.0), CFG, fail_then_pass, seed=12345, say=lambda s: None)
    same = [(r["index"], r["x"], r["y"], r["pedestal"]) for r in a["draws"]] == [(r["index"], r["x"], r["y"], r["pedestal"]) for r in b["draws"]]
    check("same seed: same draws (index, x, y, pedestal)", same and a["ok"] == b["ok"] and len(a["draws"]) > 0, f"{len(a['draws'])} draws")
    c = select_stance(pts, GEOM, "left", math.radians(-90.0), CFG, always_pass, seed=99, say=lambda s: None)
    check("a different seed gives different draws", c["draws"][0]["x"] != rec["draws"][0]["x"], "")

    print("6. the 10-draw cap: a check that always fails is called exactly 10 times, no stance twice")
    calls = []

    def always_fail(stance):
        calls.append(stance)
        return {"ok": False, "failed": ["body"], "reason": "test", "seconds": 0.0, "seed": 1}

    capped = select_stance(pts, GEOM, "left", math.radians(-90.0), {**CFG, "sampler_max_draws": 10}, always_fail, seed=3, say=lambda s: None)
    idx = [r["index"] for r in capped["draws"]]
    check("exactly 10 checks, selection fails", len(calls) == 10 and not capped["ok"] and capped["chosen"] is None and "10 draws" in str(capped["stop"]), f"calls {len(calls)}, stop: {capped['stop']}")
    check("no stance is drawn twice", len(set(idx)) == len(idx), f"indices {idx}")

    print("7. the place-angle middle rule (5 deg turns)")
    check("odd window 0..10: the middle turn 5", choose_place_angle([0, 5, 10]) == 5)
    check("odd window -10..10: the middle turn 0", choose_place_angle([-10, -5, 0, 5, 10]) == 0)
    got = choose_place_angle([0, 5, 10, 15])
    check("even window 0..15 (tie): exactly the later of the two middle turns (10)", got == 10, f"got {got}")
    w = choose_place_window([0, 5, 10, 15], 5, 10)
    check("the full check's window 0..15 gives theta 10 and the same window list", w["theta_deg"] == 10 and w["window_turns"] == [0, 5, 10, 15], f"theta {w['theta_deg']}, turns {w['window_turns']}")
    wrap = choose_place_window([345, 350, 355, 0, 5], 5, 10)
    check("a window across 0 (345..5, one window): theta -5 (midpoint 355)", wrap["theta_deg"] == -5 and wrap["window_turns"] == [-15, -10, -5, 0, 5], f"theta {wrap['theta_deg']}, turns {wrap['window_turns']}")
    check("the robot's angle is the check's angle (wrapped window)", choose_place_angle(wrap["window_turns"]) == wrap["theta_deg"], "")

    print("8. the polar draw: every stance's shoulder-to-centre distance in [r_in, r_out]; task points at least r_in away; heading within 45 deg of facing")
    big = {**CFG, "sampler_batch": 10000}
    b8 = draw_batch(pts, GEOM, "left", big, seed=21)
    centre = np.mean(np.array(pts, float), axis=0)
    S = b8["shoulder"]
    dc = np.linalg.norm(S - centre, axis=1)
    check("shoulder-to-centre 3D distance within [r_in, r_out] for all 10000 stances", float(dc.min()) >= R_IN - 1e-9 and float(dc.max()) <= outer_limit_m(2.0) + 1e-9,
          f"min {dc.min():.4f}, max {dc.max():.4f}, r_in {R_IN}, r_out {outer_limit_m(2.0):.4f}")
    dp = np.min(np.stack([np.linalg.norm(S - np.array(p), axis=1) for p in pts]), axis=0)
    check("both task points at least r_in from the shoulder, for all stances", float(dp.min()) >= R_IN - 1e-9, f"min {dp.min():.4f}")
    floor_facing = np.arctan2(centre[1] - S[:, 1], centre[0] - S[:, 0])
    dev = np.degrees(np.abs((b8["headings"] - floor_facing + np.pi) % (2 * np.pi) - np.pi))
    check("every heading within 45 deg of facing the centre from the shoulder's floor position", float(dev.max()) <= 45.0 + 1e-9, f"max deviation {dev.max():.6f} deg")
    sx, sy, _ = GEOM["shoulder"]["left"]
    hx = b8["xs"] + np.cos(b8["headings"]) * sx - np.sin(b8["headings"]) * sy
    hy = b8["ys"] + np.sin(b8["headings"]) * sx + np.cos(b8["headings"]) * sy
    check("the base places the shoulder where the draw says (base + offset turned by the heading)", np.allclose(np.stack([hx, hy], 1), S[:, :2], atol=1e-9), "")
    check("the batch is full (10000 stances) and the cap was not hit", len(b8["xs"]) == 10000 and not b8["cap_hit"], f"attempts {b8['attempts']}, drops {b8['drops']}, pedestal redraws {b8['pedestal_redraws']}")

    print("9. equal areas: at a fixed pedestal about half the floor distances lie below the area midpoint")
    fixed = {**CFG, "sampler_batch": 10000, "pedestal_range": (0.12, 0.12)}
    b9 = draw_batch(pts, GEOM, "left", fixed, seed=8)
    dz = b9["dz"]
    dmin = math.sqrt(max(R_IN ** 2 - dz[0] ** 2, 0.0))
    dmax = math.sqrt(outer_limit_m(2.0) ** 2 - dz[0] ** 2)
    mid = math.sqrt((dmin ** 2 + dmax ** 2) / 2)
    frac = float(np.mean(b9["d"] < mid))
    check("fraction below the area midpoint is about one half", abs(frac - 0.5) < 0.02, f"fraction {frac:.4f} (midpoint {mid:.4f} m, d_min {dmin:.4f}, d_max {dmax:.4f})")

    print("10. replay of the polar batch: the same seed gives the same batch")
    r1 = draw_batch(pts, GEOM, "left", CFG, seed=55)
    r2 = draw_batch(pts, GEOM, "left", CFG, seed=55)
    check("same seed: same x, y, pedestal, heading", np.array_equal(r1["xs"], r2["xs"]) and np.array_equal(r1["ys"], r2["ys"]) and np.array_equal(r1["qs"], r2["qs"]) and np.array_equal(r1["headings"], r2["headings"]), "")

    print("11. the body-check gate: every stance passed to the full check has passed the body check; gate failures do not count toward the 10 draws")

    def body_ok(stance):                                         # a stand-in body check: the stances with x above 0.75 "touch" the scene
        ok = stance[0] <= 0.75
        return ok, "" if ok else "fake_arm vs fake_bench", 0.0 if ok else -0.01

    gate_calls = []

    def gate_fn(stance):
        r = body_ok(stance)
        gate_calls.append((stance, r[0]))
        return r

    checked = []

    def pass_all(stance):
        checked.append(stance)
        return {"ok": False, "failed": ["body"], "reason": "t", "seconds": 0.0, "seed": 1}

    g = select_stance(pts, GEOM, "left", None, {**CFG, "sampler_batch": 2000, "sampler_max_draws": 10}, pass_all, seed=41, say=lambda s: None, gate=gate_fn)      # drawn headings: the held heading changes the scores
    check("every stance the full check saw had passed the body check", len(checked) == 10 and all(body_ok(st)[0] for st in checked), f"checks {len(checked)}")
    check("gate failures are counted, not drawn from the 10 full-check draws", g["gate_failures"] > 0 and len(checked) == 10 and sum(1 for _, ok in gate_calls if not ok) == g["gate_failures"],
          f"gate failures {g['gate_failures']}, gate calls {len(gate_calls)}")
    check("gate failures are not put back: no stance is seen twice by the gate", len({tuple(st) for st, _ in gate_calls}) == len(gate_calls), "")
    cap_g = select_stance(pts, GEOM, "left", None, {**CFG, "sampler_batch": 2000, "sampler_gate_max_failures": 50}, pass_all, seed=42, say=lambda s: None,
                          gate=lambda st: (False, "fake_arm vs fake_bench", -0.02))
    check("the gate cap (50 here) stops the selection with a report", (not cap_g["ok"]) and cap_g["gate_failures"] == 50 and "gate failed 50 times" in str(cap_g["stop"]), f"stop: {cap_g['stop']}")

    print(f"\n{sum(results)}/{len(results)} checks passed")
    return all(results)


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
