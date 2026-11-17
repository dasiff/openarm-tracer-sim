# Braking policy, Phase J (ongoing stop, compiled look-ahead, adaptive wake, HOLD pruning, integration)

Branch safety-filter (worktree ~/projects/openarm-tracer-sim-sf-d). NOT pushed.
Written for: David (project lead), who reviews the braking policy and decides the open items in section 8.

## 1. Result in one paragraph

Four fixes are committed and tested. The integration run brkJ1 completes the episode (SUCCESS, pick, carry, place, LIFT, BACK, HOME), the speed cap holds on every tick, there are no overrides and no emergency statuses, and BACK runs with no stop against the released beaker. The brief's performance goal is not met: the filter mean in the integration is 14.6 ms (brkZ8: 13.4 ms), not under 6 ms. The look-ahead itself dropped from 6.8 ms to 3.5 ms mean (static to adaptive, on the same poses without the held object), and adaptive wake now checks 1.8 pairs per call instead of 80. Braking did not stay the same as Phase I: the stop sequence changed (six stop starts against four), the closest held-beaker gap moved by 0.49 mm, and one filter error (GJK did not converge at 65.5 s) appeared. The contact criterion still fails, by 0.027 mm (held beaker against bench top). The Phase I 2.47 m/s2 spike is gone, but a new 2.86 m/s2 event at 73.28 s remains. The palm-descent stop distance is unchanged by adaptive wake (16.27 mm, every tick). One decision differs from the brief's literal wording (the wake floor, SF-94), and it is flagged for David.

## 2. Commits (branch safety-filter, not pushed)

| Commit | Content |
|---|---|
| b49da4f | Phase J plan in docs/progress.md |
| 8462c8e | Fix 1: every tick of a stop starts its ramp from the profile speed (SF-92) |
| 49e64dc | Fix 2: the look-ahead's candidate loop as one Numba function over shape arrays; stopping distance compiled (SF-93) |
| ef93e59 | Fix 3: adaptive wake on by default; activation = wake plus the stop gap (SF-94) |
| 3a6294a | Fix 4: HOLD wake = remaining braking distance plus the margin (SF-95) |
| 3a1ed11 | Progress note (regression and integration started) |
| f3b4521 | Phase F test expectations follow the Phase J decisions (found by the regression) |
| (this report) | Phase J report, progress, bench columns |

## 3. Files

- src/safety_filter/pair_jit.py (new): aabb_all, pair_geometry, look_kernel (the compiled candidate loop).
- src/safety_filter/braking.py: ObstacleFeed.pairs (compiled path and Python reference), _rules_for (rule cache), _pairs_compiled, _shape_arrays, _stop_of; _activation (wake plus stop gap; HOLD: plus margin); _threshold; _wake_radii (HOLD: no next-tick floor); step (every stop tick from the profile speed; _stop_prev removed).
- src/safety_filter/braking_math.py: stopping_distance compiled (njit).
- src/safety_filter/config.py: BRAKING adaptive_wake True.
- experiments/safety_filter_braking_j_test.py (new): fix 1, 2, 3, 4 checks.
- experiments/safety_filter_braking_f_test.py: expectations updated to SF-94 and SF-95 (16 of 16).
- experiments/safety_filter_perf_bench.py: --rows START:END (consecutive calls; the step-30 mode is not usable for timing), recording loaded once, p95 pairs column.
- docs/mpvi_decisions.md: SF-92, SF-93, SF-94, SF-95. docs/progress.md.

## 4. Test results

| Test | Result |
|---|---|
| safety_filter_braking_j_test (final) | ALL PASS, 19 checks (fix 1: 3, fix 2: 4, fix 3: 7, fix 4: 5) |
| Fix 1 checks on the committed code (before 8462c8e) | FAIL: 0.0167 m/s step, the 2.47 m/s2 spike reproduced |
| Fix 2 equivalence | candidates identical in count, order and ids over 84 pose-configs, with and without the held beaker (1296 candidates, 56 against the beaker); largest difference 0.0; stopping distance within 1e-12 |
| Fix 3 palm stop distance | adaptive on and off: 16.273 mm, every tick within 0.000 mm |
| Regression (21 files, run j1reg_20261117_211343) | 20 of 21 exit 0; safety_filter_braking_f_test exit 1 with the old expectations; after the expectation update (f3b4521) it passes 16 of 16 |
| safety_filter_braking_i_test | 12 of 12 |
| safety_filter_braking_d_test, braking_test | 47 of 47, 75 of 75 |

## 5. Benchmarks, before and after each change

Look-ahead tick (ObstacleFeed plus BrakingLayer.process), consecutive calls, mean and p95 in ms; pairs awake = candidates whose gap is computed per call (mean, p95). Run-to-run noise on the same code is 10 to 15 percent (static rows 7.01 and 6.16 for the same path after fix 3 and fix 4).

| Change | Python distances, static | Compiled, static | Compiled, adaptive | Compiled, HOLD static | Compiled, adaptive HOLD |
|---|---|---|---|---|---|
| Phase I code (baseline, rows 2500:5500) | 17.90 / 29.49, 79.1 pairs | 9.36 / 15.58, 79.1 | 5.00 / 8.30, 0.9 (floor too narrow) | 8.68 / 15.20, 79.1 | 4.48 / 7.19, 0.9 |
| After fix 2 (49e64dc) | 16.19 / 28.48 | 7.22 / 13.26, 79.1 | 2.97 / 4.97, 0.9 (old floor) | 5.77 / 9.95, 79.1 | 1.90 / 3.19, 0.9 |
| After fix 3 (ef93e59) | 17.34 / 26.27 | 7.01 / 12.05, 79.1 | 2.79 / 4.44, 2.5 | 4.90 / 7.67, 79.1 | 2.18 / 4.18, 0.6 |
| After fix 4 (3a6294a) | 16.60 / 28.29 | 6.16 / 11.99, 79.1 | 3.20 / 5.68, 2.5 | 5.61 / 9.92, 79.1 | 1.63 / 2.97, 0.6 |
| Integration poses (brkJ1 recording, all 5949 calls, no held object) | 17.06 / 31.36, 79.9 pairs (p95 170) | 6.75 / 12.14, 79.9 (p95 170) | 3.48 / 6.07, 1.8 pairs (p95 8) | 5.18 / 9.49, 79.9 | 1.98 / 3.37, 0.5 (p95 4) |

The fix-1 change has no measurable time cost; it changes numbers, not work.

Pairs awake with adaptive wake (brief fix 3 tests, recorded pose with the most static pairs, 82):

| Speed | Awake pairs | Wake (mm) |
|---|---|---|
| 0.02 m/s | 4 (the stop-gap floor alone: 4) | 3.9 |
| 0.15 m/s | 8 | 44.0 |
| Static (adaptive off) | 82 | - |

Palm stop distance (fast descent, 4 cm/s, work surface), final palm gap:

| Wake rule | Final gap | Largest tick difference to adaptive off |
|---|---|---|
| Adaptive off (static cutoffs) | 16.273 mm | - |
| Committed (wake plus stop gap, SF-94) | 16.273 mm | 0.000 mm |
| Brief's literal floor (pre-braking cutoff 2 cm, the Phase F formula; scratch experiment) | 14.014 mm | 2.26 mm |

The last row reproduces the Phase F defect (14.01 mm). So the brief's literal floor is the cause, not a fix: see decision 1.

HOLD pairs awake (fix 4), speed 0.15 m/s: 6 before, 6 after (wake 44.0 mm before, 42.6 mm after). At rest the HOLD wake is the tick cushion (1.33 mm), no-HOLD is 1.47 mm.

## 6. Integration: brkJ1 against brkZ8

Run: tmux j1run; command `bash experiments/run_capped.sh data/logs/brkJ1_phaseJ_20261117_211343.log python experiments/safety_filter_braking_episode.py --no-shelf --name brkJ1_phaseJ_20261117_211343` (scratch runner j1run.sh). Log data/logs/brkJ1_phaseJ_20261117_211343.log (exit 0, JOB_DONE). Recording data/safety_filter/plugin_runs/brkJ1_phaseJ_20261117_211343.npz. Analysis data/braking_episode/brkJ1_phaseJ_20261117_211343/analysis.json; compare data/logs/brkJ1_compare_*.log.

| Measure | brkZ8 (Phase I, 06840b3) | brkJ1 (Phase J, 3a6294a) |
|---|---|---|
| Outcome | SUCCESS; plan finished 84.23 s | SUCCESS; plan finished about 84.8 s |
| Calls / sim s / script wall | 5828 / 84.8 / 317 s (time: 383 s) | 5950 / 83.4 / 265 s |
| Filter ms mean / median / p95 / max | 13.4 / 11.8 / 29.5 / 94.0 | 14.6 / 11.1 / 29.3 / 392.0 |
| Stop ticks (left) | 176 | 256 |
| Stop starts (s) | 60.87, 65.71, 72.89, 73.26 | 60.87, 61.13, 65.69, 73.23, 73.54, 75.39 |
| KONG_PUSH / STEP_SHORTENED / HOLD statuses | 585 / 0 / 0 | 670 / 52 / 2 |
| Overrides (bridge) | 0 | 0 |
| Emergency statuses | 0 | 0 |
| Filter errors (logged) | 0 | 1: "GJK did not converge" (braking), at about 65.5 s |
| Sent speed peak / ticks over 0.20 | 0.1405 / 0 | 0.1379 / 0 |
| Sent accel peak (m/s2) | 2.47 at 61.44 s (0 to 0.0167 m/s in one tick) | 2.86 at 73.28 s (0 to 0.019 m/s in one tick) |
| Free-motion accel peak (m/s2) | 0.49 | 0.71 |
| Sent jerk peak (m/s3) | 678 (braking) | 465 |
| Closest any-obstacle gap | -0.517 mm (held beaker vs bench_top, 65.74 s) | -0.027 mm (held beaker vs bench_top, 65.60 s) |
| Hotplate surface gap (tool) | +6.3 mm (73.36 s) | not reported by the tool for this run |
| Look-ahead ticks with a hotplate pair | 1195 | 159 |
| Left stop ticks against the hotplate | 85 | 88 |
| Peak RSS | 4.1 GB | 4.1 GB |

Stop ticks by phase (left arm):

| Phase | brkZ8 | brkJ1 |
|---|---|---|
| Pick approach | (not split) | 82 ticks, palm vs beaker |
| Grasp and UP | 11 ticks (palm vs beaker, held vs bench) | 86 ticks (29 palm vs beaker, 57 held vs bench_top) |
| Carry (to hotplate) | (not split) | 75 ticks, link 7 vs hotplate |
| LIFT (after OPEN) | 27 ticks, link 7 vs hotplate | 13 ticks, link 7 vs hotplate |
| BACK | 0 | 0 |
| HOME | 0 | 0 |

Pass criteria (brief, integration):

| Criterion | Result | Numbers |
|---|---|---|
| Episode completes (pick, carry, place, LIFT, BACK, HOME) | PASS | OVER 50.11, DOWN 54.47, IN 58.22, CLOSE 61.59, UP 65.91, OVER 69.14, DOWN 71.73, OPEN 73.99, LIFT 75.39, BACK 77.84, HOME 79.15, HOME joints 83.57 s |
| Hand speed at or below the cap on every tick | PASS | sent peak 0.1379 m/s against 0.20; 0 ticks over |
| No false emergencies | PASS (with one logged filter error) | 0 overrides, 0 emergency statuses; one GJK non-convergence at about 65.5 s, two HOLD ticks and 52 STEP_SHORTENED ticks follow it |
| No penetrations (all gaps at least 0, excluding never-close pairs) | FAIL by 0.027 mm | closest pair held beaker vs bench_top, -0.027 mm at 65.60 s (was -0.52 mm); the robot-only minimum is not split out by the tool |
| BACK completes without holds against the released beaker | PASS | BACK 77.84 to 79.15 s: 197 ticks, 0 stop ticks; no beaker pair in any stop after OPEN |
| Filter time mean under 6 ms (fix 5) | FAIL | 14.6 ms mean (median 11.1, p95 29.3) |
| Adaptive wake on (fix 5) | PASS | config adaptive_wake True |
| Same closest gaps and stop distances as Phase I within 0.5 mm (fix 5) | NOT MET | closest held-beaker gap moved 0.49 mm (inside 0.5 mm); the stop sequence changed: six stop starts against four, 256 stop ticks against 176, 86 grasp-to-UP stop ticks against 11 |
| Peak accel within the 0.5 m/s2 limit | FAIL | 2.86 m/s2 at 73.28 s (Phase I 2.47 spike at 61.44 s is gone) |

## 7. Not done

- Filter mean under 6 ms: not reached (14.6 ms in the integration). The look-ahead alone is 3.5 ms mean on the integration poses without the held object; the integration filter includes the held beaker (cylinder pairs with the GJK contact), which the bench does not exercise. I did not measure the held-object cost separately, and the integration's filter mean rose while the median fell by 0.7 ms.
- Remaining look-ahead costs (not in the brief's list, proposed rather than built): the scale_step bisection calls forward kinematics about 14 times per tick (about 1.7 ms per tick in the profile), place() builds about 136 Python shape objects per call (about 0.4 ms), and the candidate post-processing (about 1 ms per call with 40 candidates).
- The 2.86 m/s2 event at 73.28 s: the speed goes from 0 to 0.019 m/s on the tick after a stop releases (73.26 to 73.28 s). Cause not isolated.
- The held beaker's contact with bench_top: -0.027 mm (criterion fails by that much). Not changed.
- The GJK non-convergence in the cylinder-box contact (65.5 s): the filter caught it; the handling is the existing one. Not changed.
- The stop sequence differs from Phase I (the grasp and lift are held longer). Not investigated beyond the phase counts.
- Deferred and untouched: kong guard cost, per-pair torso margins, 15 mm gripper dip, spec_test, sequential_push_test, gripper-stop bug, beaker 0.52 mm shape tolerance.
- Push: not done.

## 8. Decisions for David

1. Wake floor (SF-94). The brief says the floor is the pair's static cutoff from before the braking work, margin plus 1 cm. That is the Phase F formula, and it gives a palm stop at 14.01 mm where adaptive off gives 16.27 mm (reproduced). I used the stop gap (margin plus 2 cm, 1.5 cm on a work surface) as the floor, plus the wake: the palm is then identical on every tick. Confirm this floor, or say which other rule you want.
2. "At 0.02 m/s the pairs awake equal the old static count" (brief fix 3). Read literally, the static count is 82, so adaptive wake would do nothing. I read it as "equal the stop-gap floor" (4 pairs). Confirm.
3. HOLD wake (SF-95). Once stopped, the wake is the tick cushion (1.3 mm), so only pairs within margin plus one tick stay awake. Confirm the cushion is kept.
4. Stop-start ramp on every tick (SF-92). The ongoing stop now takes the profile speed too. This removed the 2.47 spike. Confirm.
5. Next performance step. The filter misses 6 ms. The candidates are: the held-object contact cost (measure first), scale_step's forward-kinematics bisection, and place() objects. Each is outside the brief's list, so I propose them rather than build them. Which should I take?
6. The stop sequence change (grasp and lift held longer, six stop starts against four). Accept the new sequence, or should I trace it to adaptive wake's activation for the held beaker and fix that?
7. The 2.86 m/s2 event at 73.28 s and the GJK non-convergence at 65.5 s. Both need a decision on whether to investigate now.

## 9. Files, logs, data

- Integration: data/logs/brkJ1_phaseJ_20261117_211343.log; recording data/safety_filter/plugin_runs/brkJ1_phaseJ_20261117_211343.npz; analysis data/braking_episode/brkJ1_phaseJ_20261117_211343/analysis.json.
- Regression: data/logs/j1reg_20261117_211343.* (21 files; exit codes in the "=== exit" lines; summary j1reg_20261117_211343.summary).
- Benchmarks: data/logs/jbench0c_*.log (baseline), jbench2_20261117_205947.log (fix 2), jbench3_20261117_210859.log (fix 3), jbench4_20261117_211130.log (fix 4), jbench5_20261117_212412.log (integration poses).
- Scratch (not in the repo): floor experiment (palm with the brief's floor) and the profile script, in the session scratchpad.
