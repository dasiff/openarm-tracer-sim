# Braking policy, Phase H (BACK path, hard clamp, traces, integration)

Branch safety-filter (worktree ~/projects/openarm-tracer-sim-sf-d). NOT pushed. Built on bfe59c1 (Phase G report).

## 1. Result in one paragraph

The integration pass did NOT pass, and the BACK move did not complete. Two fixes are committed. Fix 1 (a LIFT waypoint straight up from the release point, then the level retreat) is in the code and its geometry test passes, but in the integration run the LIFT was not executed: the raw source command dips about 15 mm as the gripper opens, the look-ahead latched a stop on the just-released beaker, and the stop then carried the hand further down into the plate. The hand penetrated the hotplate and the bench in both the old BACK run (brkZ5) and the new LIFT run (brkZ7), with the same minimum of −35.8 mm. Fix 2 (the hard clamp holds the joints the bridge already has) is committed and tested; the clamp did not trigger in this run, so its effect on the run is not measured. The hand-speed criterion holds in both runs (0 ticks over the cap). The traces answer the two questions: the bench penetration is the same descent during the release, and the 4.36 m/s2 free-motion spike is the start of the stop ramp from the measured speed.

## 2. Commits (branch safety-filter, not pushed)

| Commit | Content |
|---|---|
| 697cc34 | Fix 1: LIFT waypoint (PICK_PLACE_CFG back_lift_m 0.10 m) then the BACK retreat at the lifted height; place_back_waypoints helper; test; SF-87 |
| cd93262 | Fix 2: the hard clamp holds the last sent joints (onset recorded from the command, not the measured joints); tests; SF-88 |
| (this report) | Phase H report and progress |

## 3. Files

src/mpvi_modules.py (place_back_waypoints, plan_place: LIFT then BACK), src/task_config.py (PICK_PLACE_CFG back_lift_m), src/safety_filter/braking.py (_hard_arms(state, last), process), experiments/place_back_lift_test.py (new), experiments/safety_filter_braking_h_test.py (new), experiments/safety_filter_braking_d_test.py (hard clamp expectation updated), docs/mpvi_decisions.md (SF-87, SF-88), docs/progress.md, this report.

## 4. Test results

- experiments/place_back_lift_test.py: ALL PASS (6 checks: LIFT straight up by back_lift_m; BACK level at the lifted height; retreat retreat_m along the approach; palm never below the release height during BACK).
- experiments/safety_filter_braking_h_test.py: ALL PASS (injected overspeed: onset pose equals the last sent command exactly, not the measured joints; clamp output equals the onset positions with zero joint change over three ticks; override and error raised; the arm below the limit not clamped). On the committed code before fix 2 the test cannot run (old signature).
- experiments/safety_filter_braking_d_test.py: ALL PASS after the hard-clamp expectation changed to the onset joints (it had asserted the measured onset, the bug).
- Regression of the 19 safety-filter files (run h2reg_20261117_155456): 19 of 19 exit 0, 0 FAIL lines.

## 5. Bench penetration trace (task 3, read-only, brkZ5)

Trace of the left arm's worst look-ahead pairs (the recording's brake_gap and brake_pair):

| Time (s) | Event | Left worst pair | Gap (mm) | Status |
|---|---|---|---|---|
| 75.07 | OPEN (release) | - | - | PASS |
| 76.47 | BACK starts (retreat at release height, brkZ5) | palm vs beaker | +23 | PASS, stop latched at 76.48 |
| 77.65 | link 7 gap crosses zero | link 7 vs hotplate | -1.8 | HOLD |
| 78.07 | palm inside the hotplate | palm vs hotplate | -35 | HOLD |
| 78.467 | FIRST palm vs bench_top below 0 (tick 5107) | palm vs bench_top | -12.6 | HOLD |
| 86.8 | deepest bench penetration | palm vs bench_top | -57.1 | (after the failure) |

Answer: the palm first goes below zero against the bench at 78.467 s, but the palm was already inside the hotplate from about 78.0 s, so the bench crossing is the same descent through the plate. The descent happens during the retreat and the stop that follows it, so it is caused by the BACK motion (the first penetration is link 7 against the hotplate at 77.65 s). Fix 1 is aimed at that, but the integration run with fix 1 (brkZ7) shows the same penetration (minimum −35.8 mm).

Tool-height trace of brkZ7 (forward kinematics of the recorded joints, tool point; 75.8 to 79.0 s):

| Time (s) | Raw command z (mm) | Sent z (mm) | Measured z (mm) |
|---|---|---|---|
| 76.89 | 1077 | 1083 | 1079 |
| 77.42 | 1092 | 1077 | 1080 |
| 78.01 | 1124 | 1049 | 1097 |
| 78.97 | 1176 | 1003 | 1151 |

The release height is about 1089 mm. The raw command rises by about 100 mm as intended, but the sent command falls by about 80 mm. The filter is not following the raw command: the stop engaged at 76.48 s brakes the arm along the direction it was already moving (down, from the DOWN and OPEN moves), so the hand keeps going down into the plate.

## 6. Accel spike trace (task 4, read-only, brkZ5)

Peak of the tool-point acceleration on free motion (sent series): 4.357 m/s2 at 76.473 s (tick 4808 in brkZ5). The sample just before (76.467 s) is at rest (speed 0, accel 0.004); the sample after (76.480 s) is at speed 0.029 m/s with stop = 1 (the look-ahead's stop). So the sent tool speed goes from 0 to 0.029 m/s in one tick, with jerk -668 m/s3. Cause: at the free-to-stop transition the ramp starts from max(profile speed, measured speed); the measured arm was already moving at about 0.03 m/s (the sag of the release), so the stop ramp starts at the measured speed in one tick instead of from rest. This is the speed bound of the Phase C rule (the measured speed is a safety fact when faster than the profile) meeting the first tick of the stop. It is not a one-line fix; it is listed as decision 4.

## 7. Integration numbers (fix 5): not passed

| Run | Code | Outcome | Calls / sim s / wall | Filter ms mean / median / p95 / max | Free accel (m/s2) / jerk | Braking accel / jerk | Overrides | Surface gap min (mm) | Sent speed peak / ticks over cap |
|---|---|---|---|---|---|---|---|---|---|
| brkY (Phase E) | ac1e592 | BACK fails at 303 mm | 6225 / 84.9 / 1039 s | 73 / - / 253 / 578 | 16.05 / 4682 | 0.92 / 169 | 1 (false, 76 s) | +0.41 | 0.164 / 0 |
| brkZ4 (torso pairs restored; no fixes 2 and 3) | dcce75e | BACK fails at 389 mm | 6226 / 84.9 / 1046 s | 75 / 33 / 255 / 1108 | 16.04 / 4686 | 0.90 / 167 | 0 | +0.34 | 0.200 / 0 |
| brkZ5 (fixes 1 to 3 of Phase G) | 0f10477 | BACK fails at 61.7 mm | 6360 / 85.4 / 523 s | 42 / 12 / 157 / 555 | 4.36 / 653 | 14.06 / 4217 | 12 (real, penetration) | -35.8 | 0.141 / 0 |
| brkZ7 (Phase H: fixes 1 and 2) | cd93262 | LIFT fails at 168 mm, 34 deg (16 s) | 7207 / 91.1 / 798 s | 64 / 18 / 210 / 1377 | 4.36 / 653 | 13.75 / 4125 | 12 (real, penetration) | -35.8 | 0.141 / 0 |

Pass criteria, measured on brkZ7:

| Criterion | Result | Numbers |
|---|---|---|
| Episode completes (pick, carry, place, BACK) | FAIL at LIFT (the first move after the release) | pick passes (CLOSE 61.4 s, UP 64.8 s); OPEN 75.1 s; LIFT not reached after 16.0 s: 168 mm, 34 deg |
| Hand speed <= cap every tick | PASS | sent peak 0.1408 m/s; 0 ticks over 0.20 |
| No false emergencies | PASS before the release; the 12 override ticks (78.35 s on) are real contacts with the hotplate and bench | 0 overrides before 78 s; 216 look-ahead error lines, all after the stop at 76.48 s |
| No penetrations (all gaps >= 0) | FAIL | surface gap min -35.8 mm (hotplate/bench, 78 to 80 s); closest any-obstacle -57.8 mm (palm vs bench, 92.5 s) |

Comparison with brkZ5: the pick and the release are identical in both runs; the BACK/LIFT stop occurs at the same tick (76.48 s) in both; the penetration minimum is identical (-35.8 mm). The filter is slower in brkZ7 because the stop runs longer (2497 stop ticks against 1650) and the kong-style guard runs more often (2767 KONG_PUSH against 1864).

## 8. Not done

- The integration pass (fix 5): not passed. LIFT and BACK do not complete; penetration remains.
- Fix 1's test "the palm never goes below its release height during BACK": not met in the run (the palm sinks about 80 mm during the stop that follows the release).
- Fix 2's effect in the run: not measured (the clamp did not trigger in brkZ7; the unit test covers it).
- Free-motion spike at 76.47 s (4.36 m/s2): traced, not changed (decision 4).
- Arm-to-arm and arm-to-torso closest-distance measurements: not repeated.
- Filter mean under 6 ms: not met (64 ms mean in brkZ7).
- Deferred items untouched: adaptive wake floor, kong guard cost, per-pair torso margins, cylinder optimisation, spec_test, sequential_push_test, gripper-stop bug.
- Push: not done.

## 9. Decisions for David

1. HOLD direction. A stop brakes the arm along the direction it was already moving (the stop carries the hand down into the plate after the release). Should a stop hold the position (zero motion) when the source's raw command reverses, instead of continuing the previous direction?
2. Released beaker. The palm-to-beaker pair latches a stop at the release (23 mm, closing on the raw dip). Should the beaker just released be excluded from the look-ahead for the hand that released it until the hand has lifted?
3. Raw dip at the gripper opening. The source command goes about 15 mm down as the fingers open before it rises (LIFT starts at the gripper-open event at 76.46 s). Is that the manipulation's IK / sag layer, and should it be fixed in manipulation?
4. Stop ramp start (the 4.36 m/s2 spike). At the free-to-stop transition the ramp starts from the measured speed (about 0.03 m/s) in one tick. Should the ramp start from the profile speed when the arm is in free motion?
5. Hotplate contact during the release. Should the arm stop its descent at the hotplate top (approach ceiling) for the release, and the look-ahead's ceiling branch stop the arm rather than only limit its speed (carried over from Phase G)?
6. Fix 1 value. back_lift_m = 0.10 m is my value for the first run; confirm or change it.

## 10. Files, logs, data

- Integration: data/logs/brkZ7_phaseH_20261117_155928.log; recording data/safety_filter/plugin_runs/brkZ7_phaseH_20261117_155928.npz; analysis data/braking_episode/brkZ7_phaseH_20261117_155928/analysis.json.
- Regression: data/logs/h2reg_20261117_155456.* (19 files, exit codes in the "=== exit" lines).
- Traces: brkZ5 recording (bench and spike traces) and brkZ7 (tool height trace) as above.
