# Braking policy, Phase K (profile the remaining cost, compiled gap batches, arm kinematics from arrays, integration)

Branch safety-filter (worktree ~/projects/openarm-tracer-sim-sf-d). NOT pushed.
Written for: David (project lead). Decisions are SF-96, SF-97, SF-98 (implementation rows, no change of decision); David's Phase J confirmations (SF-92, SF-94, SF-95) are recorded in the decision log.

## 1. Result in one paragraph

Three fixes are committed and tested, and the integration run brkK1 completes the episode with behaviour identical to brkJ1 (same stop starts, same closest gap, same peak speed and accel). The resolver's recorded wall-clock time fell from 14.6 ms to 8.6 ms mean (median 11.1 to 6.4 ms). The full filter call is still above the 6 ms goal: the resolver alone is 8.6 ms, and the braking layer adds about 3 to 4 ms (not recorded by the plugin; estimated from the profile). So the estimated full call is about 12 ms, not under 6 ms. The remaining cost is spread over per-pair exact gaps, arm kinematics, and the sleep timers' lever (section 5). The 2.86 m/s2 event and the GJK error are deferred, as David decided.

## 2. Commits (branch safety-filter, not pushed)

| Commit | Content |
|---|---|
| b1c471f | Phase K plan and profile result |
| cea5152 | Fix 1: static lever of the pair sleep timers as arrays (SF-96) |
| 1a7d43b | Fix 2: compiled gap-only batch distances for the approximate gaps (SF-97) |
| 28c29d1 | Fix 3: arm forward kinematics from an array, RobotShapes.set_arms (SF-98) |
| be1e470 | Progress (fix results, regression and integration started) |
| 67e4b94 | Decision log: David's Phase J confirmations (SF-92, SF-94, SF-95) |
| (this report) | Phase K report and progress |

## 3. Files

- src/safety_filter/tick.py: _static_lever (arrays) with _static_lever_loop (reference); _build and _static_lever use set_arms.
- src/safety_filter/batch_jit.py (new): capsule_capsule_gap, capsule_box_gap (compiled gap-only batches) and Python entries.
- src/safety_filter/tick_arrays.py: approx_gaps uses the compiled gap-only batches.
- src/safety_filter/shapes.py: RobotShapes.set_arms(q, state).
- src/safety_filter/braking.py: tool point uses set_arms.
- experiments/safety_filter_braking_k_test.py (new): fix 1, 2 and 3 checks.
- docs/mpvi_decisions.md: SF-96, SF-97, SF-98. docs/progress.md.

## 4. Test results

| Test | Result |
|---|---|
| safety_filter_braking_k_test (final) | ALL PASS: fix 1 (static lever, largest difference 2.2e-16 over 10 poses); fix 2 (capsule-capsule 2.2e-16, capsule-box 0.0 over 600 random pairs each); fix 3 (set_arms exactly equal to set_state(with_arms), 0.0 over 4 states) |
| Regression, 22 files (run k1reg_20261118_022412) | 22 of 22 exit 0 (includes braking_f, updated in Phase J) |
| Commands sent, every replay A/B | identical (largest difference 0.0) |

## 5. Profiles and benchmarks

### 5a. Profile, before (integration brkK0, wall clock, wrappers included; ms per tick)

| Block | ms/tick |
|---|---|
| Plugin.process (whole tick) | 22.1 |
| Resolver.step | 15.7 |
| Braking.process | 4.8 |
| Resolver tick machinery (Tick, Statics) | 10.8 (exclusive) |
| Shape placement and FK (Shapes) | 3.8 |
| Wake/sleep bookkeeping (Sleep) | 3.2 |
| Braking layer own | 1.6 |
| Look-ahead feed and kernel | 1.3 |

Top 5 before: Tick.gap 2.42 (10.1 calls per tick); Tick.active_keys 2.31; Tick._maybe_short 2.29; Sleep._static_lever 2.17 (1.9 calls); Shapes.set_state 1.93 (29.6 calls).

### 5b. Profile, after (integration brkK2, same harness, same episode; ms per tick)

| Metric | before (brkK0) | after (brkK2) |
|---|---|---|
| Plugin.process | 22.1 | 14.8 |
| Resolver.step | 15.7 | 9.2 |
| Braking.process | 4.8 | 4.2 |

Top 5 after: Tick.gap 2.04 (10.1 calls; includes the exact pair gap, ta.exact_gap, which the harness does not wrap); Shapes.set_arms 1.38 (25.1 calls; replaces set_state at 1.93 before); Sleep._static_lever 0.96 (1.9 calls; 2.17 before, still a numpy call per item and joint); Shapes.place_arrays 0.72 (6.6 calls); Tick.guard_fraction 0.64 (0.1 calls; 1.42 before).

Fix 2 removed approx_gaps from the top list: in the replay profile it was 6.1 ms per tick (5 calls) before and 0.3 ms per tick after. The integration harness does not wrap tick_arrays, so it is not in brkK2's table.

### 5c. Back-to-back replays (3200 approach ticks, recorded poses, CPU time; the same objects as the plugin; no held object; the lab-map fixtures only)

| Change | Measure | Before | After |
|---|---|---|---|
| Fix 1 (SF-96) | resolver median / mean | 15.5 / 17.0 ms | 11.9 / 14.0 ms |
| Fix 2 (SF-97) | resolver mean / median | 6.7 / 5.8 ms | 4.0 / 3.1 ms |
| Fix 2 (SF-97) | full tick mean / median | 10.3 / 9.1 ms | 7.3 / 6.2 ms |
| Fix 3 (SF-98) | full tick mean / median | 8.0 / 6.6 ms | 6.6 / 5.6 ms |
| Fix 3 (SF-98) | braking mean | 3.7 ms | 2.8 ms |

The replay runs in the same process window for A and B, so drift affects both. Absolute replay numbers are not comparable across hours (the same code measured 15 ms and 22 ms for the resolver in different hours); wall-clock timings on this VM also stall (up to 14 s in one run), so the replay uses CPU time.

### 5d. Integration brkK1 against brkJ1 and brkZ8 (recorded by the plugin, wall clock, resolver filter_time)

| Measure | brkZ8 (Phase I) | brkJ1 (Phase J) | brkK1 (Phase K) |
|---|---|---|---|
| Resolver mean / median / p95 / max (ms) | 13.35 / 11.78 / 29.54 / 94.0 | 14.59 / 11.10 / 29.28 / 392.0 | 8.58 / 6.40 / 20.41 / 151.8 |
| Outcome | SUCCESS | SUCCESS | SUCCESS (wall 272 s) |
| Stop starts (s) | 60.87, 65.71, 72.89, 73.26 | 60.87, 61.13, 65.69, 73.23, 73.54, 75.39 | identical to brkJ1 |
| Stop ticks (left) | 176 | 256 | 256 |
| Overrides / emergency statuses | 0 / 0 | 0 / 0 | 0 / 0 |
| Statuses (HOLD, STEP_SHORTENED, KONG_PUSH) | 0, 0, 585 | 2, 52, 670 | 2, 52, 670 |
| Sent speed peak (m/s), ticks over 0.20 | 0.1405, 0 | 0.1379, 0 | 0.1379, 0 |
| Sent accel peak (m/s2) | 2.47 | 2.86 | 2.86 (identical) |
| Closest any-obstacle gap | -0.52 mm | -0.027 mm | -0.027 mm (identical) |
| Filter errors logged | 0 | 1 (GJK) | 1 (GJK, identical) |

The plugin records only the resolver's time (`filter_time`), so the braking layer's time per tick is not in these numbers. The profile puts it at about 3 to 4 ms (section 5b), so the full call is estimated at about 12 ms.

Note: bench_k.py times with CPU time, and the episode uses about 270% CPU across threads, so its absolute numbers (plugin 47 ms, resolver 31 ms, braking 12 ms) are inflated and are not used here.

## 6. Pass criteria (Phase K brief)

| Criterion | Result | Numbers |
|---|---|---|
| Episode completes | PASS | SUCCESS; plan finished about 84 s |
| Hand speed at or below the cap | PASS | peak 0.1379 m/s against 0.20; 0 ticks over |
| No false emergencies | PASS (one logged GJK error, as in brkJ1) | 0 overrides, 0 emergency statuses |
| Filter mean under 6 ms | FAIL | resolver 8.6 ms recorded; full call about 12 ms estimated |
| Adaptive wake on | PASS | config adaptive_wake True |

## 7. Not done

- The full filter mean under 6 ms. The estimate is about 12 ms (8.6 resolver plus about 3.4 braking).
- Exact pair gaps (Tick.gap, 2.0 ms per tick): Python per call, 10 calls per tick. The obvious next fix is a batched compiled exact gap, as done for the approximate gaps.
- Arm kinematics: set_arms and place_arrays (about 2 ms per tick together, wrapper-inflated), hand_frame (54 calls per tick).
- Sleep._static_lever (0.96 ms per tick): still numpy-heavy per item and joint.
- Tick.guard_fraction and _maybe_short (about 1.2 ms per tick together).
- The braking layer's own time is not recorded by the plugin. Recording it would need a small change to the recorder (a decision for David).
- The 2.86 m/s2 event at 73.28 s and the GJK non-convergence at 65.5 s: deferred to after the performance work (David).
- Deferred and untouched: kong guard cost, per-pair torso margins, 15 mm gripper dip, spec_test, sequential_push_test, gripper-stop bug, beaker 0.52 mm shape tolerance.
- Push: not done.

## 8. Decisions for David

1. The 6 ms goal. The full call is about 12 ms estimated. Options: (a) compile the exact pair gaps and the sleep lever, then re-measure; (b) restructure the resolver's per-tick candidate machinery into arrays and compile it (a larger change, several steps). Which do you want?
2. Recording the braking layer's time per tick in the plugin (a small recorder change), so the integration reports the full call directly. Yes or no?
3. The 2.86 m/s2 event and the GJK error: after the performance work, as decided. Confirm the order.
4. The benchmark on this VM: wall-clock numbers drift by up to 2 to 3 times between hours, and the episode uses multiple threads. For the next comparisons, should we rely on back-to-back replays (as here) plus the recorded wall-clock resolver time within one integration?
