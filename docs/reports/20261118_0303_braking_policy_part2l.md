# Braking policy, Phase L (record the braking time, compile the exact pair gaps and the sleep lever, re-profile, integration)

Branch safety-filter (worktree ~/projects/openarm-tracer-sim-sf-d). NOT pushed.
Written for: David (project lead). Decisions are SF-99, SF-100, SF-101 (implementation rows, no change of decision).

## 1. Result in one paragraph

Three fixes are committed and tested. The plugin now records the braking layer's time per call, so the full call (resolver plus braking) is in every recording. The integration run brkL1 completes the episode, the speed cap holds, there are no emergencies, and the behaviour is identical to brkK1 (same six stop starts, same closest gap -0.0273 mm, same peak speed and accel, same status counts). The goal is not met: the full call averages 11.5 ms in the recording (median 9.2 ms, p95 24.3 ms), against the 6 ms target. The remaining cost is in arm kinematics (about 4 ms per tick in the profile, wrapper-inflated) and the candidate machinery (about 2.5 ms). Both need decisions from David before they can be changed (section 8).

## 2. Commits (branch safety-filter, not pushed)

| Commit | Content |
|---|---|
| f8dbcee | Fix 1: the recorder holds the braking time (SF-99) |
| 2b7c3d9 | Fix 2: the exact pair gap is one compiled call (SF-100) |
| bef86a2 | Fix 3: the sleep timers' static lever is one compiled function (SF-101) |
| (this report) | Phase L report and progress |

## 3. Files

- src/safety_filter_plugin.py: process times the braking call (dt_braking); _record stores braking_time next to filter_time.
- src/safety_filter/batch_jit.py: pair_gap_at (compiled exact pair gap, through pair_jit.pair_geometry); static_lever_kernel (compiled static lever).
- src/safety_filter/tick_arrays.py: exact_gap compiled when cfg.GEOMETRY is compiled; _exact_gap_python is the reference.
- src/safety_filter/tick.py: _static_lever uses static_lever_kernel; _static_lever_vector and _static_lever_loop are the references.
- experiments/safety_filter_braking_l_test.py (new): fix 1 (recorder), fix 2 (exact gap), fix 3 (static lever).
- docs/mpvi_decisions.md: SF-99, SF-100, SF-101. docs/progress.md.

## 4. Test results

| Test | Result |
|---|---|
| safety_filter_braking_l_test (final) | ALL PASS: recorder (5 ms resolver and 10 ms braking stubs recorded separately; full 18.6 ms; zero without braking); exact gap compiled vs Python, largest difference 0.0 over 600 pair evaluations at 4 poses; static lever compiled vs loop, largest difference 1.1e-16 at 4 poses |
| Regression, 23 files (run l1reg_20261118_024911) | 23 of 23 exit 0 |
| Commands sent, every replay A/B | identical (largest difference 0.0) |

## 5. Benchmarks and profiles

### 5a. Back-to-back replays (3200 approach ticks, CPU time)

| Change | Measure | Before | After |
|---|---|---|---|
| Fix 2 (SF-100) | resolver mean / median | 4.2 / 2.9 ms | 4.0 / 2.9 ms (no change: the approach ticks have few exact gaps; the integration has about 10 per tick) |
| Fix 3 (SF-101) | resolver mean / median | 4.7 / 3.2 ms | 3.1 / 2.3 ms |
| Fix 3 (SF-101) | full tick mean / median | 8.0 / 6.2 ms | 6.0 / 5.0 ms |

### 5b. Integration brkL1 (the plugin's own full-call recorder, wall clock)

| Measure | Resolver (filter_time) | Braking (braking_time) | Full call (sum) |
|---|---|---|---|
| Mean | 7.77 ms | 3.76 ms | 11.52 ms |
| Median | - | - | 9.17 ms |
| p95 | - | - | 24.33 ms |
| Max | - | - | 171.4 ms |

### 5c. Profile before and after (integration, wall clock, wrappers included; ms per tick)

Before (brkK0, after Phase K fix 0): plugin 22.1, resolver 15.7, braking 4.8. Top 5 exclusive: Tick.gap 2.42; Tick.active_keys 2.31; Tick._maybe_short 2.29; Sleep._static_lever 2.17; Shapes.set_state 1.93.

After Phase K (brkK2): plugin 14.8, resolver 9.2, braking 4.2. Top 5: Tick.gap 2.04; Shapes.set_arms 1.38; Sleep._static_lever 0.96; Shapes.place_arrays 0.72; Tick.guard_fraction 0.64.

After Phase L (brkL2p): plugin 13.4, resolver 8.1, braking 4.0. Top 5: Tick.gap 1.86; Shapes.set_arms 1.33; Shapes.place_arrays 0.70; Tick.guard_fraction 0.63; Plugin.process (own, including the new timing) 0.63.

Next items after the top 5 (Phase L profile): Tick._build 0.59; Shapes.hand_frame 0.58 (54 calls per tick); Tick._maybe_short 0.57; Tick.discover 0.48; Shapes.place 0.44; Tick.lever_at 0.44; Braking.tool_fn 0.42; Tick.active_keys 0.40.

Note: the profile harness wraps the methods, so the numbers add up to more than the plugin tick; they rank the costs, they are not exact milliseconds.

## 6. Integration brkL1 against brkK1

| Measure | brkK1 (Phase K) | brkL1 (Phase L) |
|---|---|---|
| Outcome | SUCCESS | SUCCESS (wall 217 s) |
| Full call mean / median / p95 / max | not recorded (resolver only: 8.58 / 6.40 / 20.41 / 151.8) | 11.52 / 9.17 / 24.33 / 171.4 |
| Stop starts (s) | 60.87, 61.13, 65.69, 73.23, 73.54, 75.39 | identical |
| Stop ticks (left) | 256 | 256 |
| Sent speed peak (m/s), ticks over 0.20 | 0.1379, 0 | 0.1379, 0 |
| Sent accel peak (m/s2) / jerk peak | 2.862 / 465 | 2.862 / 465 (identical) |
| Closest any-obstacle gap | -0.0273 mm (held beaker vs bench_top, 65.60 s) | identical |
| Overrides / emergency statuses | 0 / 0 | 0 / 0 |
| Statuses (HOLD, STEP_SHORTENED, KONG_PUSH, PASS) | 2, 52, 670, 5226 | identical |
| Filter errors logged | 1 (GJK, identical) | 1 (GJK, identical) |

Pairs awake: not measured in Phase L. The candidate loop of the look-ahead was not changed in this phase, and the integration behaviour is identical to brkK1, so the pair counts are the brkK1 ones (about 1.8 mean and 8 at p95 for the adaptive look-ahead on the integration poses, Phase J bench).

## 7. Pass criteria (Phase L brief)

| Criterion | Result | Numbers |
|---|---|---|
| Episode completes | PASS | SUCCESS |
| Hand speed at or below the cap | PASS | peak 0.1379 m/s against 0.20; 0 ticks over |
| No false emergencies | PASS (one logged GJK error, as before) | 0 overrides, 0 emergency statuses |
| Full call mean under 6 ms | FAIL | 11.52 ms mean recorded |
| Behaviour identical to brkK1 (stop starts; closest gaps within 0.1 mm) | PASS | identical stop starts; closest gap identical |

## 8. Not done, and decisions for David

Not done: the full call under 6 ms (11.5 ms recorded, about 5.5 ms to remove). Step 5 of the brief (restructure the next biggest cost) is not built, because the two largest remaining costs need a decision first.

1. **Arm kinematics (about 4 ms per tick, wrapper-inflated; Shapes.set_arms 1.33, place_arrays 0.70, hand_frame 0.58, Tick._build 0.59, Shapes.place 0.44, set_state 0.36).** Each new pose runs MuJoCo's forward kinematics and then builds the shape arrays in Python. Getting the next large saving means replacing the MuJoCo pose computation for the arm shapes with a compiled arm-chain kinematics. That changes the architecture (the robot model's kinematics would no longer be MuJoCo's for the filter's shapes), so it needs your approval. Do you want it?
2. **Candidate machinery (about 2.5 ms per tick: Tick.guard_fraction 0.63, _maybe_short 0.57, discover 0.48, lever_at 0.44, active_keys 0.40).** These are array routines and can be compiled in the same way as the gap and lever fixes. Expected saving about 1.5 to 2 ms. Shall I compile them next?
3. **Tick.gap (1.86 ms per tick, 10 calls).** Each call still pays a Python-to-compiled dispatch. Batching the pairs per pose (one call per pose instead of one per pair) would remove most of it, but it changes the call pattern of the resolver. Approve?
4. The 2.86 m/s2 event and the GJK error: deferred, as confirmed.
5. Push: not done.

## 9. Files, logs, data

- Integration brkL1: data/logs/brkL1_phaseL_20261118_024911.log; recording data/safety_filter/plugin_runs/brkL1_phaseL_20261118_024911.npz (filter_time and braking_time); analysis data/braking_episode/brkL1_phaseL_20261118_024911/analysis.json.
- Profile episode brkL2p: data/logs/brkL2p_profile_20261118_024911.log and .stats.json.
- Regression: data/logs/l1reg_20261118_024911.* (23 files).
- Replays: session scratchpad (A/B logs replayA_l2a, replayB_l2a, replayA_l3a, replayB_l3a, and the earlier t3, f2a, f3a).
