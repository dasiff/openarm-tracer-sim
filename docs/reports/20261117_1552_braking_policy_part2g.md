# Braking policy, Phase G (stabilize for the integration pass)

Branch safety-filter (worktree ~/projects/openarm-tracer-sim-sf-d). NOT pushed. Built on dcce75e (Phase F report).

## 1. Result in one paragraph

The integration pass did NOT pass. The three fixes are committed and each does what it was meant to do: fix 1 (both torso pairs back on the never-close list) lets the arm reach the beaker and lift it; fix 2 (the ramp and the distance-aware cap take the profile's acceleration) removes the 68 s chatter, with the joint acceleration peak in that window going from 7.1 to 0.7 rad/s2; fix 3 (arm-to-arm pairs use the closing component only) removes every arm-to-arm hold during the BACK move (150 ticks to 0). The episode still fails at BACK after the release: the left link 7 and palm move into the hotplate and the bench under the look-ahead's hold, which engages about 8 mm too late, and the bridge is overridden 12 times. The look-ahead's stopping estimate reads the same acceleration as the ramp, so the profile acceleration (fix 2 as briefed) is what brakes too late; with the measured acceleration back in the look-ahead (tested, brkZ6, reverted) the look-ahead raises 202 emergencies at low speed and the hard clamp then sends a 0.76 rad step (22.9 m/s tool speed for two ticks). Fix 4 is confirmed: the adaptive wake is off by default and stays off. The state reported here is HEAD 0f10477, the code that ran as brkZ5.

## 2. Commits (branch safety-filter, not pushed)

| Commit | Content |
|---|---|
| ef32ca3 | Fix 1: both torso pairs back on the never-close list (NEVER_CLOSE_REMOVED empty; list rebuilt to the Phase D 366 pairs, identical set and stamp); E test checks; SF-84; SF-80 withdrawn |
| 2203060 | Fix 2: the ramp start and the distance-aware cap take the profile's acceleration; the measured speed stays a bound on the speed; G test; SF-85 |
| 0f10477 | Fix 3: arm-to-arm pairs reached by the closing component only (no whole ramp speed through the closing share); G test; SF-86 |
| (none) | Fix 4: adaptive_wake default False confirmed (config value unchanged) |
| (this report) | Phase G report and progress |

Progress and report entries carry the run names and logs.

## 3. Test results

- Regression of the 18 safety-filter files on 0f10477 (run g3reg_20261117_151039): 18 of 18 exit 0, 0 FAIL lines.
- experiments/safety_filter_braking_g_test.py: ALL PASS (fix 2: 6 checks; fix 3: 2 checks).
- experiments/safety_filter_braking_e_test.py (fix 1 checks): ALL PASS; experiments/safety_filter_touch_test.py: exit 0.
- Note on the Phase G regression logs: exit codes are read from the "=== exit" lines (the summary file is always 0).

## 4. Chatter (fix 2): root cause confirmed

Mechanism (unchanged): the ramp start took the measured acceleration whenever the measured speed was above the profile; the ramp's output became the profile acceleration; the distance-aware cap reads that acceleration, so when the arm reverses the cap collapses and the sent command alternates.

Confirmation from the runs (window 67.5 to 69.5 s, left arm):

| Run | Code | Joint accel peak (rad/s2) | p99 (rad/s2) |
|---|---|---|---|
| brkZ4 | before fix 2 (torso pairs restored) | 7.1 (at 68.37 s) | 9.9 |
| brkZ5 | fix 2 on | 0.7 (at 69.69 s) | 0.2 |

The alternation is gone. The look-ahead also read the measured acceleration (the same function), and the fix changed its stopping estimate, which is the cause of the BACK collision (section 6).

## 5. BACK move diagnosis (fix 3)

The question: does arm-to-arm use the closing component or the whole speed during BACK? Code: in state mode every pair uses the closing component (the hand point's velocity along the pair direction; the tool-velocity fallback is also a closing component). The whole ramp speed entered through the closing share (reach = max(closing component, ramp speed x share), share capped at 1). For arm-to-arm pairs the share saturates at 1 whenever the commanded closing rate of the pair reaches the commanded tool speed, so the whole ramp speed counted.

Evidence (left arm, stopped ticks 76.2 to 80 s):

| Run | Left stop ticks | Left pairs against the right arm | Top left pairs |
|---|---|---|---|
| brkZ4 (before fix 3) | 566 | 150 (left_finger_left vs right_finger_right 82, vs right_link1_3 68) | palm vs beaker 204; finger vs torso 185 |
| brkZ5 (fix 3 on) | 528 | 0 | palm vs bench_top 230; link7 vs hotplate 182; palm vs hotplate 66; palm vs beaker 50 |

Fix 3 removes the arm-to-arm holds. The remaining BACK holds are arm-to-world and held-object pairs, which the brief keeps as they are.

## 6. Integration (fix 5): not passed

Runs compared (all on the same episode and recording conventions):

| Run | Code | Outcome | Calls / sim s / wall | Filter ms mean / median / p95 / max |
|---|---|---|---|---|
| brkY (Phase E) | ac1e592 | PLAN_FAILED at BACK (303 mm) | 6225 / 84.9 / 1039 s | 73 / - / 252.6 / 578.2 |
| brkZ4 (torso pairs restored by the shape file; fixes 2 and 3 not in) | dcce75e | PLAN_FAILED at BACK (389 mm) | 6226 / 84.9 / 1046 s | 74.8 / 32.7 / 255.0 / 1107.5 |
| brkZ5 (fixes 1 to 3; HEAD 0f10477) | 0f10477 | PLAN_FAILED at BACK (61.7 mm, 34 deg) | 6360 / 85.4 / 523 s | 42.0 / 12.4 / 157.3 / 555.4 |
| brkZ6 (look-ahead reads measured acceleration; reverted) | experiment | PLAN_FAILED at BACK (771 mm, 147 deg) | 6362 / 85.4 / 727 s | 57.4 / 19.5 / 222.8 / 1470.2 |

Pass criteria, measured on brkZ5:

| Criterion | Result | Numbers |
|---|---|---|
| Episode completes (pick, carry, place, back) | FAIL at BACK | pick: CLOSE 61.4 s, UP 64.8 s (beaker lifted); place: OPEN 75.1 s; BACK not reached (61.7 mm, 34 deg) |
| Hand speed at most the cap every tick | PASS | sent tool speed peak 0.1408 m/s, 0 ticks over 0.20; measured peak 0.146 m/s |
| No false emergencies | PASS before BACK; the BACK overrides are real contacts | 0 overrides before 76 s; 12 override ticks at 78.7 s (the palm and link 7 inside the hotplate and bench) |
| Beaker reaches the hotplate | Placed (OPEN at 75.1 s after the DOWN move); beaker surface gap not separated in this pass | hotplate pair surface gap minimum -35.8 mm (arm, not the beaker) |
| BACK completes | FAIL | as above |

Accel and jerk (commanded tool series, the analysis tool's checklist; limits 0.5 m/s2, 2.0 m/s3):

| Run | Free peak accel (m/s2) | Free peak jerk (m/s3) | Braking peak accel | Braking peak jerk |
|---|---|---|---|---|
| brkY | 16.05 | 4682 | 0.92 | 169 |
| brkZ4 | 16.04 | 4686 | 0.90 | 167 |
| brkZ5 | 4.36 | 653 | 14.06 | 4217 |

The 68 s alternation is gone (section 4). The tool-level free peak of 4.36 m/s2 in brkZ5 is not yet traced to its tick; it is not the 68 s window (where the joint acceleration is 0.7 rad/s2).

Closest distances (look-ahead gaps, from the recordings):

| Run | Left arm closest gap | Pair | Right arm closest gap | Pair |
|---|---|---|---|---|
| brkZ4 | -80.7 mm at 84.95 s | left_palm_2 vs bench_top | 24.6 mm at 1.41 s | right_palm_2 vs torso |
| brkZ5 | -57.1 mm at 86.81 s | left_palm_2 vs bench_top | 24.6 mm at 1.41 s | right_palm_2 vs torso |

Arm-to-arm and arm-to-torso measurements of Phase E (scratch scripts) were not repeated in this pass.

Overrides and errors (brkZ5): 12 override ticks at 78.68 s onward; the errors are look-ahead failures on left_link7_1 vs bench_top and left_palm_1 vs bench_top with negative gaps (penetration).

Why BACK fails (brkZ5): during BACK (76.5 s) the left palm and link 7 move toward the hotplate at about 0.1 m/s. The look-ahead holds the arm from 76.5 s, but the link 7 vs hotplate gap is already 8 mm (room 8 mm less than the stopping distance of about 27 mm), so the arm reaches the hotplate (gap -16 mm at 78 s, -35 mm for the palm). The hotplate is an allowed-contact surface (approach ceiling 0.05 m/s), so its pairs go to the ceiling branch, which does not stop the arm; the ceiling limit was active for 0 ticks in brkZ5. The emergency that would stop it fires only after penetration.

## 7. Fix 2 scope test (reverted)

The look-ahead's stopping estimate also reads the ramp's acceleration (the same function). Two variants:

- Fix 2 for the ramp and cap only, the look-ahead on the profile acceleration (committed, brkZ5): the look-ahead brakes too late in BACK; penetration of the hotplate and bench; 12 overrides; BACK 61.7 mm short.
- Fix 2 for the ramp and cap, the look-ahead on the clipped measured acceleration (brkZ6, not committed): 202 look-ahead emergencies at low speed (about 0.02 m/s, stop distance 18 mm against room 11 mm); a hard clamp then sends a 0.76 rad step (22.9 m/s tool speed on two ticks, the 83.6 s hold); BACK 771 mm short.

Both fail. The committed state is the first variant. The choice between them is a design decision (section 9).

## 8. Not done

- Integration pass (fix 5): not passed (BACK fails; penetration of the hotplate and bench during BACK and the hold after it).
- Hand-speed criterion: passes in brkZ5 (0 ticks over the cap), fails in brkZ6 (hard-clamp jump).
- Free-motion tool-level accel of 4.36 m/s2 in brkZ5: not traced to its tick.
- Arm-to-arm and arm-to-torso closest-distance measurement (Phase E scratch scripts): not repeated.
- Filter mean under 6 ms: not met (42 ms mean in brkZ5, 12 ms median).
- Deferred items untouched: adaptive wake floor design, kong guard cost (4 s calls), per-pair torso margins, cylinder distance optimization, spec_test, sequential_push_test, gripper-stop bug.
- Push: not done (standing instruction).

## 9. Decisions for David

1. BACK and the hotplate. Under HOLD the look-ahead engages too late for the hotplate pairs (allowed-contact surface, ceiling branch, no stop). Options: (a) the look-ahead's stopping estimate keeps the measured acceleration (brkZ6: no penetration path, but 202 low-speed emergencies and the hard-clamp jump); (b) the ceiling branch of the look-ahead stops the arm, not only limits its speed, when the stopping distance exceeds the room; (c) the BACK path is changed so the palm and link 7 leave the hotplate upward first (manipulation, outside the filter).
2. Hard clamp in HOLD (Phase D decision 5). The clamp sets the command to the measured pose, which made a 0.76 rad step (22.9 m/s tool speed) on two ticks during the hold. The hand-speed criterion fails in that run. Options: rate-limit the clamp's step, or keep the clamp and accept the step.
3. Palm below the bench in HOLD (-57 to -81 mm, both brkZ4 and brkZ5). The HOLD stop does not keep the palm above the bench top after the BACK failure. Needs a look at the bench model and the hold path.
4. Look-ahead emergency at low speed (brkZ6: 202 emergencies at about 0.02 m/s with room 11 mm). Is an emergency at that speed intended?
5. Free-motion tool accel (brkZ5: 4.36 m/s2 against 0.5): does the criterion count the tool-level series, and where does that peak come from?
6. Fix 2 scope: the look-ahead on the profile acceleration (committed) or on the measured acceleration (brkZ6). Confirm which.
7. Adaptive wake (SF-82): off by default, confirmed; the floor design is deferred.

## 10. Files, logs, data

- Logs: data/logs/g2reg_20261117_150455.* and g3reg_20261117_151039.* (regressions); data/logs/brkZ5_phaseG_20261117_151439.log (integration, committed state); data/logs/brkZ6_phaseG_20261117_152626.log (experiment, reverted).
- Recordings: data/safety_filter/plugin_runs/brkZ5_phaseG_20261117_151439.npz (+ .looks.json), brkZ6_phaseG_20261117_152626.npz.
- Analysis: experiments/safety_filter_braking_analyze.py on the same run names.
