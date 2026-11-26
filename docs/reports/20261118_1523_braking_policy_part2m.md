# Braking policy, Phase M (candidate machinery, gap batching, re-measure, arm kinematics decision)

Branch safety-filter (worktree ~/projects/openarm-tracer-sim-sf-d). NOT pushed.
Written for: David (project lead). Decision row: SF-102 (implementation, no change of decision).

## 1. Result in one paragraph

One fix is committed and tested: the pair box separations and the lever arms of the resolver's tick are compiled (6a5110c). In a back-to-back replay of the approach ticks the resolver's median falls from 3.2 to 2.4 ms, and the commands sent are identical. Gap batching (step 2) was measured and is not worth building: each exact pair gap costs about 15 µs including its wrapper, and the compiled call itself 1.6 µs, so batching ten calls per tick saves about 0.15 ms. The integration run brkM1 completes the episode with behaviour identical to brkL1 (same stop starts, same closest gap, same cap check, same status counts). Its recorded full call averages 32.9 ms, but the VM was about 2.5 times slower during that run (wall 536 s, against 217 s for brkL1), so that number cannot be compared with brkL1's 11.5 ms. The goal (full call under 6 ms) is not shown to be met. Arm kinematics (step 4) is not built: the estimate below says it would save about 3 ms and would not reach 6 ms on its own.

## 2. Commits (branch safety-filter, not pushed)

| Commit | Content |
|---|---|
| 6a5110c | Fix 1: pair box separations and lever arms compiled (SF-102) |
| (this report) | Phase M report and progress |

## 3. Files

- src/safety_filter/batch_jit.py: pair_separation_kernel, lever_kernel.
- src/safety_filter/tick_arrays.py: box_separation uses the compiled kernel; _box_separation_numpy is the reference.
- src/safety_filter/tick.py: lever_at uses lever_kernel; _lever_at_numpy is the reference.
- experiments/safety_filter_braking_m_test.py (new): fix 1 checks.
- docs/mpvi_decisions.md: SF-102. docs/progress.md.

## 4. Test results

| Test | Result |
|---|---|
| safety_filter_braking_m_test | ALL PASS: box separations compiled vs numpy, largest difference 0.0 over 3864 pairs at 4 poses; lever arms compiled vs numpy, 0.0 |
| Regression, 24 files (run m2reg_20261118_151041) | 24 of 24 exit 0 |
| Commands sent, replay A/B | identical (largest difference 0.0) |

## 5. Benchmarks

### 5a. Back-to-back replay, approach ticks (3200), CPU time

| Change | Measure | Before | After |
|---|---|---|---|
| Fix 1 (SF-102) | resolver median / mean | 3.21 / 6.82 ms | 2.37 / 3.43 ms |
| Fix 1 (SF-102) | full tick median / mean | 7.16 / 10.92 ms | 5.64 / 7.09 ms |

The committed run has one 7.9 s stall, which distorts its mean; the medians are the comparison.

### 5b. Gap batching (step 2): measured, not built

| Measure | Value |
|---|---|
| Exact pair gap, Python wrapper included (one call) | 14.8 µs |
| Compiled pair gap call alone | 1.6 µs |
| Tick.gap calls per tick in the integration | 10.1, all on the table path (no fallback to the object form) |
| Saving if all ten were batched | about 0.15 ms per tick |

The inclusive cost of Tick.gap in the integration profile (3.4 ms per tick, wrapper-inflated) is mostly the pose builds that gap() triggers through _place when the pose is new, not the gap calls themselves. Batching the gap calls would not change that.

### 5c. Integration profile, instrumented, during the slow VM run (brkM0; wrapper-inflated; ranking only)

| Rank | Item | ms per tick | Calls per tick |
|---|---|---|---|
| 1 | Tick.gap (includes the pose builds it triggers) | 3.24 | 10.1 |
| 2 | Shapes.set_arms | 2.71 | 25.1 |
| 3 | Shapes.place_arrays | 1.43 | 6.6 |
| 4 | Tick._build | 1.31 | 3.6 |
| 5 | Shapes.hand_frame | 1.23 | 54.4 |
| 6 | Plugin.process (own) | 1.20 | 1.0 |
| 7 | Tick._maybe_short | 1.17 | 1.4 |

The pose-build group (set_arms, place_arrays, _build, hand_frame, Shapes.place, set_state) is about 7 ms inflated. The candidate group (_maybe_short, discover, active_keys, guard_fraction) is about 2.5 ms inflated.

## 6. Integration brkM1 against brkL1

| Measure | brkL1 (Phase L, bd5ec05) | brkM1 (Phase M, 6a5110c) |
|---|---|---|
| Outcome | SUCCESS (wall 217 s) | SUCCESS (wall 536 s, slow VM) |
| Full call mean / median / p95 / max (ms) | 11.52 / 9.17 / 24.33 / 171.4 | 32.94 / 21.67 / 88.81 / 405.7 |
| Resolver mean / braking mean (ms) | 7.77 / 3.76 | 21.00 / 11.94 |
| Stop starts (s) | 60.87, 61.13, 65.69, 73.23, 73.54, 75.39 | identical |
| Stop ticks (left) | 256 | 256 |
| Sent speed peak, ticks over cap | 0.1379, 0 | 0.1379, 0 |
| Sent accel / jerk peak | 2.862 / 465 | 2.862 / 465 |
| Closest any-obstacle gap | -0.0273 mm | identical |
| Statuses (HOLD, STEP_SHORTENED, KONG_PUSH, PASS) | 2, 52, 670, 5226 | identical |
| Filter errors (GJK) | 1 | 1 |

The full-call numbers are not comparable: the same code ran about 2.5 times slower in wall time on this VM during brkM1. The replay shows the Phase M change itself (section 5a).

## 7. Step 4: arm kinematics, not built

The brief asks for a compiled arm-chain kinematics if the full call stays above 6 ms. Two findings make me stop short of building it now.

- The pose builds (about 7 ms inflated in 5c) are mostly Python around the forward kinematics, not the MuJoCo call itself (mj_kinematics is about 8 µs). A compiled kernel would have to return the placed shape arrays directly, not only the body frames, to save the Python work. My estimate: about 3 ms saved on the integration.
- The forward kinematics also feeds the elbow solver (arm_conversion reads xaxis and xanchor) and the tool point (config). The replacement must produce all of xpos, xmat, xanchor and xaxis, and be validated against MuJoCo to 1e-4 m.

Estimated full call after step 4: about 7 to 8 ms (from about 11 ms on the fast VM state). The candidate machinery (discover, active_keys, _maybe_short, guard_fraction) would still be about 1.5 ms, and it works on Python sets and dicts, so compiling it needs a restructure of the candidate data flow. Step 4 alone would not reach 6 ms.

## 8. Not done, and decisions for David

Not done:
- The full call under 6 ms (not shown; estimate about 7 to 8 ms after step 4 and the candidate restructure).
- Step 2 (gap batching), measured and not built (section 5b).
- Step 4 (arm kinematics), estimated and not built (section 7).
- The 2.86 m/s2 event and the GJK error (deferred, as confirmed).
- Push.

Decisions for David:
1. **Arm kinematics and the candidate data flow (about 3 ms plus about 1.5 ms).** This is the architecture change the brief describes. Expected result: about 7 to 8 ms, not yet under 6. Do you want both done, knowing the 6 ms goal needs them and maybe more?
2. **The benchmark on this VM.** Wall-clock timings move by 2 to 3 times between hours (brkL1 at 217 s, brkM1 at 536 s for the same behaviour). For the final comparison, should the full call be measured in the same window as a baseline run (both runs back to back), or as replay numbers only?
3. **Gap batching (step 2).** Measured at 0.15 ms per tick: not built. Confirm you agree.
