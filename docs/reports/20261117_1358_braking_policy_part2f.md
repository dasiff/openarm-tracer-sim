# Braking policy, Phase F (fixes from the Phase E review, performance, integration)

Branch safety-filter (worktree ~/projects/openarm-tracer-sim-sf-d). NOT pushed. Built on b387013 (Phase F fix 1).

## 1. Result in one paragraph

Fixes 1 to 6 are built, tested and committed; the integration pass (fix 7) does NOT pass. The committed state (cfe877c) keeps the decided torso change (fix 3), and with it the pick fails: the left arm cannot reach the beaker (IN not reached, 451 mm), the bridge is overridden 7 times at 64 s by look-ahead errors on left_link7_2 against the hotplate, and the sent acceleration and jerk reach 30 m/s2 and 9000 m/s3. With the torso pairs restored (diagnostic run only, not committed) the pick succeeds and the place reaches the release, then the BACK move fails (389 mm from its waypoint, 10 s); the acceleration and jerk chatter of Phase E (16 m/s2, 4686 m/s3 at 68 s) is still there, so the mechanical clip (fix 2) does not remove it. The filter is still far from the 6 ms target: 75 ms mean per call in the run, 11 to 24 ms in the look-ahead benchmark. The compiled distances (fix 4) do not give a measurable speed-up on their own, because the time is not in the distance maths. Adaptive wake and HOLD pruning (fixes 5 and 6) cut the pairs checked by 42 percent, but with the wake on the decided fast-palm stop moves from 14.5 to 17.5 mm to 14.0 mm, so the switch is off by default until David decides the floor. The fix 3 pair test is satisfied: both torso pairs are awake and checked in the pick-and-place run (27 of 3000 calls each).

## 2. Commits (branch safety-filter, not pushed)

| Commit | Content |
|---|---|
| b387013 | Fix 1: finger pairs exempt from the emergency closing-speed test |
| 127b047 | Fix 2: chatter diagnosis (root cause in the message); the clipped-branch acceleration bound (mechanical); the algorithm part is a decision |
| 94f85eb | Fix 3: the two left link 1 torso pairs off the never-close list (config NEVER_CLOSE_REMOVED); list rebuilt 366 to 364 pairs; decision row SF-80 |
| 44c8bac | Fix 3 check: per-pair look counter (run diagnostic, no effect on results); written to <run>.looks.json |
| e544a8d | Fix 4: the four distance functions compiled with numba (geometry_jit.py), Python kept as reference (geometry._py_*); decision row SF-81; numba 0.68.1 in requirements.txt |
| 9563b29 | Fixes 5 and 6: adaptive wake radius (SF-82) and HOLD pruning (SF-83) behind cfg.BRAKING["adaptive_wake"]; tests; benchmark script |
| cfe877c | Performance (exact): look-ahead candidate prefilter and pair-rule cache; adaptive wake switched off by default (SF-82 pending David) |

Files: src/safety_filter/braking.py, geometry.py, geometry_jit.py (new), config.py, tick.py, src/safety_filter_plugin.py, tools/build_never_close.py, data/safety_filter/robot_shapes.json (tracked list), experiments/safety_filter_braking_e_test.py, _d_test.py (palm measurement with static cutoffs), _f_test.py (new), jit_test.py (new), perf_bench.py (new), kong_style_test.py, resolver_test.py, touch_test.py (test expectations for the removed pairs and the far-move premise), docs/mpvi_decisions.md (SF-80 to SF-83), docs/progress.md, requirements.txt.

## 3. Test results

Regression of the 17 safety-filter files on cfe877c (run f6reg_20261117_125627, real exit codes read from each log): 17 of 17 exit 0, 0 FAIL lines.
Notes: the summary file's exit codes are always 0 (run_capped.sh ends with echo); the real codes are in the "=== exit" line of each log.

New or changed checks:
- experiments/safety_filter_jit_test.py: ALL PASS. Constants equal; capsule-capsule 2000 random, zero-length, parallel and crossing pairs: exact; capsule-box 300: 1.2e-15 m; cylinder-capsule 300: 4e-8 m; cylinder-box 400: gap 7e-10 m, every compiled closest pair valid to 6e-10 m; the cylinder-box witness points differ from the reference by up to 1.3e-5 m where the closest pair is not unique (flat faces); public functions with the switch on and off: 2e-8 m.
- experiments/safety_filter_braking_e_test.py: fix 3 checks (both torso pairs checked, not on the list, in the pair table, other torso pairs still listed, 364 pairs) and the fix 1 and fix 2 checks: ALL PASS.
- experiments/safety_filter_braking_f_test.py: fixes 5 and 6 unit checks and the wiring: ALL PASS.
- experiments/safety_filter_braking_d_test.py: 3 FAILs when the wake is on (fast palm stops at 14.01 mm, window 14.5 to 17.5 mm); ALL PASS with the wake off (bisection: wake off with compiled on passes; wake on with compiled off still fails). An earlier note in progress.md said the palm test passed after the measurement fix; that was wrong (the rerun's exit code was masked by grep | head).
- experiments/safety_filter_touch_test.py: the list check drops the configured removals (the sweep record keeps them).
- experiments/safety_filter_resolver_test.py, _kong_style_test.py: the far move uses the right arm (at the zombie pose the left link 1 joint 3 is 2.9 mm from the torso, so a left move is no longer far).

## 4. Chatter diagnosis (fix 2)

Root cause (unchanged by this phase's runs): the distance-aware cap (braking._stoppable_speed) depends on the current acceleration a0, clamped to [0, a_eff]. a0 comes from the measured arm when that is faster than the profile. When the physical arm reverses, a0 changes sign, the cap collapses (0.099 to 0.0033 m/s for 0.5 mm of distance change), the step clamp writes a = -0.5 m/s2, and the loop runs with period 2. With a0 = 0 in the cap the dip is gone.

Mechanical fix (committed 127b047): the clipped branch of step() was not acceleration-limited (-13.5 m/s2 in the test scenario); it is now clipped to effective_accel_limit (test: fails without it).

Not fixed: the cap algorithm. Still a decision for David (section 9, item 4). The integration runs of this phase show the 68 s chatter again: sent acceleration 16.0 m/s2 and jerk 4686 m/s3 (limits 0.5 and 2.0), the same as Phase E.

## 5. Benchmarks

Look-ahead benchmark (experiments/safety_filter_perf_bench.py on 100 poses from brkZ1, every 30th call; look-ahead only, no held beaker; the held beaker adds cylinder pairs and makes the real filter slower):

Final code (bench2, after the prefilter and the rule cache):

| Configuration | mean ms | p95 ms | max ms | pairs with a gap computed per call |
|---|---|---|---|---|
| before: Python distances, static cutoffs | 24.40 | 40.96 | 55.97 | 65.0 |
| compiled distances, static cutoffs | 18.76 | 33.74 | 40.48 | 65.0 |
| compiled, adaptive wake (fix 5) | 15.40 | 26.09 | 34.27 | 37.9 |
| compiled, HOLD with the static cutoffs | 11.40 | 19.36 | 23.05 | 65.0 |
| compiled, adaptive, HOLD with the margin rule (fix 6) | 11.94 | 19.88 | 28.88 | 39.2 |

Before the prefilter and rule cache (bench1, same poses): before 44.90 / 64.17 / 78.43 ms (65.0 pairs); compiled static 52.06 / 70.29 / 166.80 (65.0); compiled adaptive 51.35 / 68.06 / 72.58 (37.9); HOLD static 46.15 / 66.21 / 90.36 (65.0); HOLD margin 44.65 / 60.82 / 74.39 (39.2).

Reading:
- The prefilter and rule cache (exact: the pairs with a gap computed are unchanged, 65.0, 37.9 and 39.2 in both runs) roughly halve the look-ahead time.
- Compiled distances before/after: 24.4 to 18.8 ms in bench2 (-23%), 44.9 to 52.1 ms in bench1 (+16%). No consistent gain; the profile shows the time is in the Python pair loop and rule lookups, not the distances (ObstacleFeed.pairs, pair_rule about 618 calls per tick at 33 us, wake_dist/derive in each call). The compiled kernels are kept because they are exact to 1e-6 m and remove the distance cost from the resolver's exact gaps.
- Pairs awake before and after adaptive pruning: 65.0 to 37.9 per call (-42 percent) with the wake on; HOLD margin rule 39.2.

Filter time in the integration runs (per call, the plug-in's measurement): see section 6.

## 6. Integration (fix 7): not passed

Runs (all on the committed code, except where noted):

| Run | Code | Outcome | Calls, sim s, wall | Filter ms mean / median / p95 / max |
|---|---|---|---|---|
| brkY (Phase E, reference) | ac1e592 | PLAN_FAILED at BACK (303 mm) | 6225, 84.9, 1039 s | 73 / - / 252.6 / 578.2 |
| brkZ1 (fix 3 on) | 94f85eb | killed by me at 12:15 (the run was slow, see below) | 3000 flushed | 30 / - / 101.6 / 777 (first 3000 calls) |
| brkZ2 (all fixes, fix 3 on) | cfe877c | killed by me at 13:10 (false stall; the run was still progressing) | not complete | not complete |
| brkZ3 (diagnostic, fix 3 on, faulthandler) | cfe877c | PLAN_FAILED: IN not reached (451 mm, 58 to 73 s); recovery; second IN not reached (3.9 mm, 14.9 s) | 11210, 113.1, 2407 s | 153 / 32.6 / 938.7 / 4028.8 |
| brkZ4 (diagnostic, fix 3 reverted in the list file only) | cfe877c | PLAN_FAILED at BACK (389 mm, 10 s after 76.2 s) | 6226, 84.9, 1046 s | 74.8 / 32.7 / 255.0 / 1107.5 |

Checklist (brkZ4 with the torso pairs restored, and brkY for comparison; brkZ3 for the fix 3 state):

| Item | brkZ4 | brkY (Phase E) | brkZ3 (fix 3 on) | Verdict |
|---|---|---|---|---|
| Sent hand speed, every tick (cap 0.20) | peak 0.2000; 0 ticks over | peak 0.164; 0 over | peak 0.2000; 0 over | pass |
| Sent accel, free motion (limit 0.5) | 16.04 m/s2 | 16.05 | 30.0 | fail (chatter not fixed) |
| Sent jerk, free motion (limit 2.0) | 4686 m/s3 | 4682 | 9000 | fail |
| Sent accel / jerk, braking | 0.90 / 167 | 0.92 / 169 | 30.0 / 9000 | fail |
| Measured speed peak | 0.183 | 0.163 | 0.293 | pass (below the cap) |
| Bridge overrides (false emergencies) | 0 | 1 (false, 76 s) | 7 ticks at 64 s (left_link7_2 vs hotplate, look-ahead) | brkZ4 pass; brkZ3 fail |
| Pick: grasp, lift | pass (CLOSE 62.3 s, UP 66.0 s) | pass | fail (IN not reached) | |
| Beaker reaches the hotplate | pass (surface gap 0.34 mm) | pass (0.41 mm) | not reached (min surface gap 21.7 mm) | |
| BACK move completes after release | fail (389 mm, 10 s) | fail (303 mm) | not reached | fail |
| Filter time mean / p95 / max | 74.8 / 255.0 / 1107.5 ms | 73 / 252.6 / 578.2 | 153 / 938.7 / 4028.8 | target mean < 6 ms: fail |
| Arm-arm, arm-torso overlaps | not measured in this pass | measured in Phase E (scratch scripts) | not measured | not checked |

What the runs show:
- The torso pair change (fix 3, SF-80) is the cause of the failed pick in brkZ3. At the zombie pose the left link 1 joint 3 is 2.9 mm from the torso (inside its 30 mm margin, now checked), and the look-ahead then most likely holds the left arm on the way to the beaker (IN); the logs show the IN failure and the look-ahead errors, not the full chain. The overrides are look-ahead errors on left_link7_2 against the hotplate during that hold. Restoring the two pairs (brkZ4) lets the pick succeed.
- The BACK failure (brkZ4) is a look-ahead hold, not an override: during the recovery the stop flag is on for pairs that involve the torso and the right arm (left_finger_left vs torso, right_link7_2 vs torso, right_finger_right vs left_link5_1, left_palm_2 vs beaker). It is the same kind of hold as the Phase E finger false emergency, but through the look-ahead, not the emergency. That needs a decision (section 9).
- The slow filter is the kong-style jump-through guard: its passes and violates loops evaluate the active pairs at every path sample, and each new pose recomputes the cylinder bounds in a Python loop (kong_style._guard -> guard_fraction / passes -> tick.any_gap_below -> tick_arrays.approx_gaps -> _cylinder_upper). Single calls take up to 4 s in brkZ3 and 1.1 s in brkZ4; runs of such calls make a run crawl (brkZ3 took 40 min of wall time). A faulthandler stack dump from brkZ3 shows this path. It is a cost problem, not a hang; it is a decision for David (section 9, item 5).
- Earlier "stall" reports (brkZ1, brkZ2) were this slowness, not a deadlock: brkZ2 was stopped by me on a silence rule of 6 minutes, which was too short; brkZ3 and brkZ4 both completed.

## 7. Torso check (fix 3) and the pair test

- The decided removal: left_link1_3 vs torso and left_link1_2 vs torso are checked (pair_rule is a Rule; both in the pair table; both absent from the never-close list).
- Awake and checked in a pick-and-place run: brkZ1 (fix 3 on, 3000 calls flushed): left_link1_3|torso and left_link1_2|torso each looked at on 27 of 3000 calls (the scheduler's awake-and-checked count); left_link1_1|torso (still exempt) none.
- Consequence: section 6 (failed pick in brkZ3).

## 8. Not done

- Integration pass (fix 7): not passed. Pick fails with the torso pairs on (brkZ3); BACK fails with them off (brkZ4).
- Filter mean under 6 ms: not met (75 ms in the integration run, 11 to 24 ms in the look-ahead benchmark; the held beaker's cylinder pairs are not in the benchmark).
- Chatter algorithm fix: not built (decision).
- Adaptive wake and HOLD pruning: built, tested, switched off by default (the palm stop changes with them on).
- Arm overlaps in the integration run: not measured in this pass.
- Push: not done (standing instruction).
- Worktree kept (its data/ holds the brkY, brkZ and f-series recordings and logs).

## 9. Decisions for David

1. Torso pairs (SF-80, fix 3). As decided, the pick fails (brkZ3): the look-ahead holds the left arm from home (the zombie pose is 2.9 mm from the torso, inside the 30 mm robot-robot margin). Options: keep the pairs checked and accept the hold (the pick needs a different approach or a smaller margin for this pair); restore the two pairs; or re-sweep the pose. The committed list keeps fix 3 as decided.
2. Wake floor (SF-82). The brief's "worst-case radius as a floor" gives no pruning. The next-tick floor I built cuts the palm's stop to 14.0 mm (the look-ahead starts braking 20 mm from the bench). Which floor? Until decided the switch is off (my default; revert by setting cfg.BRAKING["adaptive_wake"] = True).
3. HOLD pruning (SF-83). Interpreted as the remaining braking distance plus the pair's margin, capped at the static cutoff. Confirm.
4. Chatter (fix 2, algorithm): (a) take the ramp's start acceleration from the profile, the measured acceleration only as a speed bound (recommended); (b) a0 = 0 in the cap (about 10 mm of easing margin lost at 0.5 m/s2); (c) limit the cap's change per tick. The 68 s chatter (16 m/s2, 4686 m/s3) is still in every integration run.
5. Kong-style tunnel guard cost. Calls take up to 4 s and runs crawl (40 min for one run). Options: cache the cylinder bounds per pose; cap the passes per tick; vectorise the cylinder bounds. Each changes how much work a tick may do; a decision on the budget.
6. BACK after the release (look-ahead). During the recovery the left arm is held for pairs with the torso, the right arm's fingers and links, and the held beaker. Should the look-ahead treat the other arm's parts and the torso as obstacles for the recovery? (Same question as the Phase E finger motion.)
7. Numba (SF-81): numba 0.68.1 (llvmlite 0.50.0) added to requirements.txt; compiles on the first call per process. Confirm the dependency.
8. Never-close source (SF-80): the DC file was missing from this worktree and was copied from ~/projects/openarm-tracer-sim-safety-filter. Same 219 listed pairs and model fingerprint; the file bytes differ (sha 2b89cf... recorded before, 1c7819... now). The list is rebuilt from that file. Confirm the provenance.
9. Joint-2 torso pair. left_link1_2|torso was not measured in contact (minimum 11.7 mm, brkW); it was removed on the brief's list. Confirm.

## 10. Files, logs, data

- Logs: data/logs/f3reg_20261117_050920.*, f3fix_20261117_115533.*, f4reg_20261117_121529.*, f5reg_20261117_122346.*, f6reg_20261117_125627.*; benchmark data/logs/bench_20261117_122110.log and bench2_20261117_122315.log; integration data/logs/brkZ1_fix3_20261117_115950.log, brkZ2_all_20261117_125950.log, brkZ3_diag_20261117_131025.log, brkZ4_nofix3_20261117_133246.log.
- Recordings: data/safety_filter/plugin_runs/brkZ1_fix3_20261117_115950.npz (3000 calls), brkZ3_diag_20261117_131025.npz, brkZ4_nofix3_20261117_133246.npz (with .looks.json).
- The list before fix 3 (the diagnostic file for brkZ4): scratchpad f3/robot_shapes_before.json (not committed).
