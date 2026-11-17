# Braking policy, Phase I (release fix, stop ramp, integration pass)

Branch safety-filter (worktree ~/projects/openarm-tracer-sim-sf-d). NOT pushed. Built on e974211 (Phase H report).
Written for: David (project lead), who reviews the braking policy and decides the open items below.

## 1. Result in one paragraph

The three fixes are committed and tested. The integration run brkZ8 completed the whole pick-and-place episode (pick, carry, place, LIFT, BACK, HOME; plan finished at 84.2 s), which brkZ7 did not (it failed at LIFT). Four of the five pass criteria hold: the hand speed never exceeds the cap (sent peak 0.1405 m/s against 0.20), there are no overrides, emergencies or hard clamps, and BACK runs with no stop against the released beaker. The fifth criterion, "all gaps >= 0", fails by 0.52 mm: the held beaker's shape is 0.52 mm into bench_top for 36 ticks during the first lift after the grasp (65.7 s). The hotplate gap never drops below +6.3 mm. The filter is faster (mean 13.4 ms against 64 ms in brkZ7). Two sent-accel events remain (2.47 m/s2 at 61.44 s and 1.35 m/s2 at 72.90 s, against the 0.5 limit). They come from the measured-speed bound in an ongoing stop (SF-85), which fix 3 does not cover because David decided it for stop starts only.

## 2. Commits (branch safety-filter, not pushed)

| Commit | Content |
|---|---|
| dc350ec | Fix 1: the stop latch releases on the tick the source reverses; the latch remembers the pair that set it (SF-89) |
| 0b17002 | Fix 2: the just-released object is excluded from that arm's look-ahead until every hand shape is held_world_margin_m (1 cm) away (SF-90) |
| 8026131 | Fix 3: a stop that begins starts its ramp from the profile's planned speed, not the measured speed (SF-91) |
| 1d9adbd | Progress note (regression result, brkZ8 start) |
| (this report) | Phase I report and progress |

## 3. Files

- src/safety_filter/braking.py: fix 1 (latch key, release on reversal), fix 2 (ObstacleFeed bind/pairs: exclusion sets per arm), fix 3 (step: stop-start ramp from the profile; _stop_prev).
- src/safety_filter/config.py: BRAKING held_world_margin_m 0.01 (SF-90, placeholder).
- experiments/safety_filter_braking_i_test.py: new; fix 1 (4 checks), fix 2 (4 checks), fix 3 (4 checks).
- docs/mpvi_decisions.md: SF-90, SF-91 rows (SF-89 from fix 1).
- docs/progress.md.

## 4. Test results

| Test | Result |
|---|---|
| experiments/safety_filter_braking_i_test.py (final) | ALL PASS, 12 of 12 (log data/logs/brkI_final_20261117_192940.log) |
| Fix 2 part, staged version (fix 1 and 2 checks, fix 2 code without fix 3) | ALL PASS, 8 of 8 (checked in the worktree before commit 0b17002) |
| Fix 3 checks with the fix off (the stop-start branch disabled in memory) | 2 of 4 fail: at rest 0 to 0.02896 m/s in one tick; moving (measured 0.13 m/s) 0.00795 to 0.13003 m/s |
| Regression: 20 safety-filter test files (the 19 of Phase H plus braking_i) | 20 of 20 exit 0, 0 FAIL lines, 0 tracebacks (data/logs/i1reg_20261117_193007.*.log, summary i1reg_20261117_193007.summary) |

Fix 3 test, what it checks: (a) at rest, as in brkZ5: a tiny move sets the arm's direction, the profile settles to rest, the measured speed is 0.029 m/s, then a stop begins; the first tick's speed change must be within the accel limit x dt (0.0033 m/s). (b) moving: the profile settles at its own speed (0.008 m/s under the stop cap), the measured speed is 0.13 m/s, then a stop begins. The first tick's speed change is within the same bound.

## 5. Integration: brkZ8 against brkZ7

Run: tmux brkZ8; command `bash experiments/run_capped.sh data/logs/brkZ8_phaseI_20261117_193334.log python experiments/safety_filter_braking_episode.py --no-shelf --name brkZ8_phaseI_20261117_193334`. Log data/logs/brkZ8_phaseI_20261117_193334.log (exit 0, JOB_DONE). Recording data/safety_filter/plugin_runs/brkZ8_phaseI_20261117_193334.npz. Analysis data/braking_episode/brkZ8_phaseI_20261117_193334/analysis.json; compare output data/logs/brkZ8_compare_20261117_193930.log.

| Measure | brkZ7 (Phase H, cd93262) | brkZ8 (Phase I, 8026131) |
|---|---|---|
| Outcome | LIFT fails at 168 mm, 34 deg (16 s) | SUCCESS; plan finished at 84.23 s |
| Calls / sim s / wall | 7207 / 91.1 / 798 s | 5828 / 84.8 / 383 s (the script's own wall: 317 s) |
| Filter ms mean / median / p95 / max | 64 / 18 / 210 / 1377 | 13.4 / 11.8 / 29.5 / 94.0 |
| Stop ticks (left) | 2497 | 176 |
| KONG_PUSH statuses | 2767 | 585 |
| Overrides (bridge) | 12 (real, penetration) | 0 |
| Emergency statuses | not counted in brkZ7 report | 0 |
| Look-ahead error lines | 216 (all after 76.48 s) | 0 |
| Hard clamp | not triggered | not triggered |
| Sent speed peak / ticks over 0.20 | 0.1408 / 0 | 0.1405 / 0 |
| Sent accel peak, free / braking (m/s2) | 4.36 / 13.75 | 0.49 / 2.47 |
| Sent jerk peak, free / braking (m/s3) | 653 / 4125 | 300 / 678 |
| Closest any-obstacle gap | -57.8 mm (palm vs bench_top, 92.45 s) | -0.52 mm (held beaker vs bench_top, 65.74 s) |
| Hotplate gap min | -35.8 mm (78 to 80 s) | +6.3 mm (73.36 s) |
| Peak RSS (time output) | not recorded | 4.1 GB (cap 6 GB) |

Stop starts (brkZ8): 60.87, 65.71, 72.89, 73.26 s.

## 6. Pass criteria (brief, fix 4)

| Criterion | Result | Numbers |
|---|---|---|
| Episode completes (pick, carry, place, LIFT, BACK, HOME) | PASS | CLOSE 61.43 s; UP 65.71; OPEN 73.37; LIFT 74.77; BACK 77.25; HOME 78.57; joint HOME 83.03; plan finished 84.23 (largest joint error 0.002 rad) |
| Hand speed <= cap every tick | PASS | sent peak 0.1405 m/s against 0.20; 0 ticks over |
| No false emergencies | PASS | 0 override ticks, 0 emergency statuses, 0 hard clamps, 0 look-ahead error lines; statuses PASS 5243, KONG_PUSH 585 |
| No penetrations (all gaps >= 0, excluding never-close pairs) | FAIL by 0.52 mm | Closest pair held beaker vs bench_top, -0.517 mm at 65.74 s; 36 left-arm ticks negative during UP. Hotplate minimum +6.29 mm at 73.36 s. Robot-only bench minimum not isolated by the tool |
| BACK completes without holds against the released beaker | PASS | BACK 77.25 to 78.57 s: 198 ticks, 0 stop ticks, 0 HOLD statuses. No beaker pair appears in any stop after OPEN |

## 7. Notes from the traces

- Accel events (sent tool point). At 61.44 s the sent speed goes from 0.0002 to 0.0167 m/s (2.47 m/s2), then back to 0.0030 m/s on the next tick (-2.06 m/s2). At 72.90 s it goes from 0.0164 to 0.0254 m/s (1.35 m/s2). Both happen inside an ongoing stop: the first stop began at 60.87 s; the second began at 72.89 s, and fix 3 handled that start tick from the profile, so the rise came one tick later. On these ticks the recorded profile speed jumps to the measured speed, because SF-85 takes the measured speed as a bound whenever it is faster. At 61.4 s the measured left joint drifts about 0.00018 rad per tick (about 0.015 m/s at the tool) while the profile is at 0.0002 m/s; the cause of the drift is not isolated (probably the gripper closing, not verified). Fix 3 covers only the tick on which a stop begins, as decided (SF-91), so these events are outside its scope.
- LIFT. 27 left-arm stop ticks (74.77 to 77.25 s) against left_link7_1 vs hotplate. The closest hotplate gap over the episode was +6.3 mm (73.36 s). The lift was slowed but completed.
- Released beaker. No beaker pair in any stop after the release (73.37 s). The exclusion (SF-90) is checked by the unit test; the run does not show it directly.
- Carry slice. The monitor logged a forearm 169 mm outside the carry slice (margin 120 mm) at 71.44 s. It is logged only and not a pass criterion.
- Pick-time gap. The held beaker's bottom reads -0.52 mm against bench_top for 36 ticks around 65.7 s (UP starts at 65.71 s). This is a grasp-geometry question for the held-object model, not a robot shape (see decisions).

## 8. Not done

- The no-penetration criterion (strict): fails by 0.52 mm (held beaker vs bench_top, decision 3 below). Not changed.
- The two accel blips (2.47 and 1.35 m/s2): not fixed. Decision 2.
- Robot-only gap minimum to bench_top: not isolated (the tool reports the closest pair overall).
- Exclusion effect (SF-90) in a run: not shown by the trace; covered by the unit test only.
- Deferred and untouched: adaptive wake, kong guard cost, per-pair torso margins, cylinder optimization, 15 mm gripper dip, spec_test, sequential_push_test, gripper-stop bug.
- Push: not done (the feature is not yet complete under the push rule, since the integration pass does not pass).

## 9. Decisions for David

1. Fix 3 scope (SF-91, my reading). The rule applies to every stop start (no stop on the previous tick), not only to stops from rest. A check that the profile is not on the S-curve left the fix to at-rest stops only, and a moving arm is almost always on the S-curve (moving test: 0.00795 to 0.13003 m/s in one tick without the fix). Confirm this reading, or say that "in free motion" meant unshaped motion only.
2. Measured speed in an ongoing stop (SF-85). The blips at 61.44 s and 72.90 s come from the measured speed as the ramp's start while a stop is on. Should an ongoing stop also start from the profile speed, or should the measured bound stay? This is the largest remaining accel violation (2.47 m/s2 against 0.5).
3. Held beaker against bench_top, -0.52 mm for 36 ticks at the first lift. Is this held-object geometry (possibly the held shape sits about 0.5 mm low in the held model; not checked) to be fixed there, or does the criterion count a held object against the bench? As written, the criterion fails by 0.5 mm.
4. Look-ahead stop during LIFT (27 ticks, link 7 vs hotplate). The approach-ceiling question from Phase H decision 5 still applies: is a stop at the hotplate during LIFT acceptable, or should the ceiling branch limit the speed only?
5. Margin of the release exclusion (SF-90). The exclusion ends by axis-aligned bounding-box separation (a lower bound on the true gap, so the release is never early). Keep it, or measure the true distance?

## 10. Files, logs, data

- Integration: data/logs/brkZ8_phaseI_20261117_193334.log; recording data/safety_filter/plugin_runs/brkZ8_phaseI_20261117_193334.npz; analysis data/braking_episode/brkZ8_phaseI_20261117_193334/analysis.json; compare data/logs/brkZ8_compare_20261117_193930.log; analyze log data/logs/brkZ8_analyze_20261117_193900.log.
- Regression: data/logs/i1reg_20261117_193007.* (20 files; exit codes in the "=== exit" lines; summary i1reg_20261117_193007.summary).
- Phase I test log: data/logs/brkI_final_20261117_192940.log.
- brkZ7 (comparison): data/logs/brkZ7_phaseH_20261117_155928.log, data/braking_episode/brkZ7_phaseH_20261117_155928/analysis.json.
