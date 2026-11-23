# Pre-push verification: full tests, baselines, policy-style command stream (2026-11-23)

Branch `safety-filter`, HEAD `42371fb` (code of SF-119 in `a980ce2`). Comparison commits: `0c96d55` (before the phase), `e5ab377` (before SF-119), `c2fec3d` (S5), `7290d3c` (pre-S5), `394567c` (SF-116), `8da6c0b` (SF-117). No production code changed. No test changed, so no decision-log row. New: `experiments/policy_stream_source.py`, `experiments/policy_stream_test.py`. Not pushed.

## Part 1: tests

### Full suite (`experiments/run_tests.sh`, tag `pp_head_20261122_1412`)

71 entries: 49 pass, 22 fail (exit code 1). The safety-filter regression (all `safety_filter_*` entries) passes.

Also run separately at HEAD: `sag_layer_test` 17/17, `sag_solver_failure_test` ALL PASS, `held_mass_ramp_test` 40 PASS, `open_catch_test` 12/12, `sag_warm_start_reset_test` 7/7, `sag_feedback_test` 8/8, `command_bridge_test` 14/14 (in the suite too). `episode_test_sources.py` is a helper module with no `__main__`, not a test; its importers are in the suite.

### The 22 failures, classified against 0c96d55 (same commands, same data)

**Fail on both (pre-existing).** Exit 1 on both sides:

| Test | HEAD | 0c96d55 | Note |
|---|---|---|---|
| search_timeline_test | missing data file | same | environment |
| search_display_isolation_test | missing data file | same | environment |
| elbow_guard_test | FAILED clamp | same | |
| face_test | crashes (TypeError) | 24/27 | HEAD crashes in the cheat episode; base runs to the end |
| gripper_force_test | 12/15 | 27/29 | different failing checks |
| target_mode_test | 9/16 | 9/16 | same checks |
| cc_loop_tests | EGL error, missing `data/cc_baseline/manual_headless_cams.csv` | same | environment |
| cc_mode_tests | crashes (TypeError) | 33/39 | HEAD crashes early |
| full_check_test | 28/31 | 28/31 | same checks |
| nav_pipeline_follow_test | 4/14 | 10/14 | different checks |
| stance_select_test | IndexError | same | |
| place_theta_logic_test | 2 fails | same 2 | same checks |
| stance_select_pipeline_test | 3/7 | 3/7 | same checks |
| stance_select_moved_test | IndexError | same | |
| transit_clearance_test | IndexError | same | |
| bench_stance_test | EGL error | same | |
| mpvi_pipeline_test | crashes (StanceSelectionError) | 10/14 | HEAD crashes in the stance sampler |
| mpvi_text_test | 7/11 | 5/11 | both fail; different checks |

**New (pass on 0c96d55, fail on HEAD):** four, all the same cause.

| Test | 0c96d55 | e5ab377 (pre-SF-119) | HEAD |
|---|---|---|---|
| home_pose_test | pass 34/34 | fail | fail (crash) |
| grasp_lift_test | pass 4/4 | fail | fail |
| beaker_hotplate_correction_test | pass 9/9 | fail 5/9 | fail 5/9 |
| nav_follow_sim_test | pass 5/5 | fail 0/5 | fail 0/5 |

All four fail with "ARMS timed out after 30 s" in the staging step (the staging step's arm move to home). Attribution, by rerunning the four tests at each commit (`c2fec3d` S5 passes, `394567c` SF-116 passes, `8da6c0b` SF-117 fails):

- SF-119 (the loop cut) is **not** the cause; `e5ab377` already fails all four.
- S5 (`c2fec3d`) is **not** the cause; it passes all four.
- SF-117 (`8da6c0b`, warm-start reset after a failed solve) **is** the cause. Before SF-117 the sag solver stayed in its failed state in these scenarios, so the correction never ran (pass-through). After SF-117 the solver recovers and the correction acts during the staging move, which then misses the 30 s deadline. The same mechanism drove the held run before SF-119.

**Classification of the four.** The tests assert the staging move finishes within its 30 s deadline with the correction now active, which is the behaviour before SF-117 only by accident. I did not change the tests. **Question for David:** should the sag correction act during the staging move at all (every source, per S5 and SF-117)? Or should the staging phase be exempt, or the deadline changed? (Test-only fixes: none.)

## Part 2: baselines

Scenario runs with `CC_HEADLESS=1 bash experiments/cc_bench_scenarios.sh`. Compared by row index, sim time agrees for all.

- **Determinism.** HEAD twice, manual, teleop and policy: all three byte-identical between the two runs.
- **Manual vs CURRENT (`manual_reference_20261103.csv`).** 25 of 84 rows differ. Largest: `left_joint3` 15.02° at 8.20 s (median of changed rows 0.54°); `right_joint1` 4.95° at 9.20 s; `right_joint4` 0.63° at 9.20 s; `right_joint7` 0.03°; fingers and base < 0.02. Most of these are at 8 to 10 s, when the staging/solver activity starts.
- **Teleop, HEAD vs 7290d3c (no CURRENT baseline).** 58 of 78 rows differ. Largest: `left_joint5` 16.0° (9.20 s), `left_joint6` 13.2° (9.20 s), `left_joint1` 13.1° (8.60 s), `left_joint7` 12.1° (9.20 s), `left_joint4` 11.5° (8.60 s), `left_joint2` 5.9° (8.60 s), `right_joint2` 4.1° (3.70 s); base up to 0.02 mm; typical (median of changed rows) 0.4 to 2.4° on the left arm.
- **Policy, HEAD vs 7290d3c.** 31 of 84 rows differ. Largest: `left_joint3` 0.70° (6.90 s), median 0.57°; all other joints < 0.001°; base and pedestal ≤ 0.0000 mm.

These differences are expected from SF-117, S5 and SF-119 (the correction now runs on the sources' commands; the loop cut changes what sources see). I did not judge or tune them.

## Part 3: policy-style command stream

**Setup.** Real loop, MuJoCo arm physics, safety filter off, no held object, left arm only, fixed seeds, 30 s sim time per run. Each target is held for 5 ticks (30 Hz), per `experiments/policy_stream_source.py` (A hold; B smooth chunks of 15 targets, hand 50 mm per 0.5 s = 100 mm/s; C as B with every third chunk followed by a 1 s stall; D random reachable poses every 0.5 s, hand speed 0.2 to 1.0 m/s, abrupt direction changes, 3 seeds). The arm first moves from its resting pose to the home pose (min-jerk, 10.3 s), and each variant starts there; statistics cover the 19.7 s after setup.

**Deviations (please review):**
1. The **bridge is off** for these runs. With the bridge on, the stream freezes the arm even with the correction off (see below), so the correction's effect cannot be read. Two bridge-on runs are reported to show the freeze.
2. The 10.3 s setup move is not in the brief. Without it the arm starts below the bench top and the first targets are not reachable.
3. Clip of the trim: position 40 mm, rotation 0.1 rad (`PICK_PLACE_CFG`).

**Results (bridge off, correction on; correction-off references where run).** Position in mm, rotation in rad.

| Variant | Peak pos (time, hand speed at peak) | Final pos | Settle (pos) | Ticks at pos clip | Peak rot | Final rot | Settle (rot) | Ticks at rot clip | Solver ok / fail | Hand error mean / last 2 s (on; off) |
|---|---|---|---|---|---|---|---|---|---|---|
| A hold (seed 1) | 1.51 (0.00 s, hold) | 0.88 | 6.6 s | 0 | 0.0124 | 0.0101 | 5.4 s | 0 | 4443 / 81 | 0.1 / 0.1 mm; off: 1.1 / 1.0 mm |
| B smooth (seed 1) | 40.0 (4.45 s, 23 mm/s) | 40.0 | 11.4 s | 2767 | 0.1000 (0.46 s, 395 mm/s) | 0.1000 | 16.5 s | 2737 | 4443 / 81 | 104 / 126 mm; off: 40 / 48 mm |
| C stalls (seed 1) | 40.0 (13.65 s, 488 mm/s) | 39.1 | 17.9 s | 2507 | 0.1000 (0.68 s, 337 mm/s) | 0.069 | 19.8 s | 1871 | 4443 / 81 | 69 / 25 mm; off: 32 / 31 mm |
| D random (seed 1) | 40.0 (0.01 s, 217 mm/s) | 39.4 | 18.3 s | 2540 | 0.1000 (0.17 s, 371 mm/s) | 0.097 | 18.4 s | 2771 | 4443 / 81 | 251 / 256 mm; off: 240 / 225 mm |
| D random (seed 2) | 40.0 (1.09 s, 608 mm/s) | 39.2 | 19.3 s | 2690 | 0.1000 (0.77 s, 467 mm/s) | 0.099 | 19.3 s | 2785 | 4443 / 81 | 245 / 225 mm; off: 234 / 249 mm |
| D random (seed 3) | 40.0 (10.69 s, 677 mm/s) | 38.9 | 19.8 s | 2728 | 0.1000 (0.19 s, 461 mm/s) | 0.0995 | 18.6 s | 2847 | 4443 / 81 | 262 / 182 mm; off: 229 / 191 mm |

Notes on the table: the solver counts cover the whole 30 s run, setup included (the solver is called on every tick), so they are identical in every correction-on run; the "final" is the mean of the last 2 s; the settle time is the last time the trim is more than 10 % from its final value (time from the start of the measurement); the hand speed at peak is a 0.1 s average.

**Variant A, steady droop.** With the correction off, the steady hand error at the held pose is (ref − pad) = (−0.2, 1.0, 0.0) mm (norm 1.0 mm), and the rotation error is (−0.0076, 0.0005, 0.0073) rad (norm 0.0105 rad). The final trim (correction on) is (−0.57, 0.14, 0.65) mm (norm 0.88 mm) and (−0.0091, 0.0010, 0.0043) rad (norm 0.0101 rad). So the rotation trim matches the droop (cosine 0.93, norm ratio 0.96), and the position trim matches in size (ratio 0.88) but not in direction (cosine 0.28). I did not decide what this means.

**Variant D, largest hand-position error vs the commanded target (ref = forward kinematics of the commanded targets).**
- Correction on: max 704 mm (seed 1), 586 mm (seed 2), 686 mm (seed 3); mean over the run 251, 245, 262 mm.
- Correction off: max 639, 695, 584 mm; mean 240, 234, 229 mm.
So over the three seeds the correction makes the mean error about 19 mm larger and the maximum about the same. It does not help during the fast moves.

**Bridge-on runs (B, seed 1), to show the freeze.**
- Correction on: 1 suspension; hand error mean 464 mm; trim pinned at the clip for the whole run.
- Correction off: 17 "target step too large" rejections; hand error mean 759 mm, max 1047 mm; the arm stops following the stream.
With the bridge's 0.5 rad/s per-joint limit, and rejection of steps over 0.25 rad, a policy-like stream at these speeds is not followed.

**Other observations.**
- The commanded joint speeds reach 4 to 5 rad/s in D and about 1.8 rad/s in B (the maximum over the whole run includes the setup-to-variant handoff; I did not isolate the 1.8 rad/s value). That is far above the bridge's 0.5 rad/s limit, so with the bridge on, D would be rejected. With the bridge off the arm cannot follow it either: the hand error is 0.2 to 0.25 m even with the correction off.
- The trim saturates at its clip in B, C and D, within about 0.5 s of the start in B and D.
- No collision stops or limit stops occurred. The source checked only that hand targets stay above z = 1.10 m (above the bench top) and within 0.6 m of home.

## Questions (not built)

1. The four new failures (Part 1): should the sag correction run during the staging move (every source), and does the 30 s deadline stay?
2. Should a policy stream be allowed to run through the bridge, given that its limits reject steps over 0.25 rad and cap joint speed at 0.5 rad/s? With the bridge on the arm freezes even with no correction.
3. The trim drives the hand error to the clip during fast moves, and does not reduce the error in D. Should the correction's limits or its gain change? (Not tuned here.)
4. Variant A's trim matches the steady droop in size and in rotation, but not in position direction. Expected?

## Not run

- Full test list for the `applied` readers beyond the suite (`cc_loop_tests` and `cc_mode_tests` are in the suite, and both fail on both sides as above).
- The baselines for `held_200g` (the held baseline needs its own regeneration, per the baseline README).
- Policy-style runs with the bridge on for variants A, C and D (only B was run with the bridge on).
