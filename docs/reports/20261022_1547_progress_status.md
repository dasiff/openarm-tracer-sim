# Overnight queue: progress status (2026-10-22, 15:50)

Written for: David.

## Stage table
| Stage | State | Last commit | Tests |
|---|---|---|---|
| 1 Full check speedups | DONE (R2j skipped: 27 s warm, under 30 s; kept as LATER) | 861c33d | full_check_test 25/25; suite 33/33 modules at the time |
| 2 Navigation core | DONE | 0b664ec | nav_core_test 31/31 with `--sim` (27/27 without); report 20261022_1252_queue_stage2.md |
| 3 Follower + integration (`NAV_CFG["follow"]`, default False) | DONE, with questions | 469e256 | nav_follow_test 10/10 (kinematic); nav_follow_sim_test 5/5 (arrival 3.8 mm / 0.15 deg); full pipeline with it on: SUCCESS 13/13, replay bit-identical; setting off: staging summary identical to the old code (only wall time differs); lean sweep 14 profiles, all under 0.02 deg (the sim's base cannot lean: caveat in the report) |
| 4 Observation slots | DEFERRED by you (M152) | 6ae1908 | n/a |
| 5 Bench rectangle (replaces old 5) | BUILT; BLOCKED on a decision | 5b01efe | stance_select_test 23/23; live test passes only with a hand-set rectangle whose centred spot is TASK_CFG's own (SUCCESS, one stance, null second navigation step, replay bit-identical) |
| 6 New neck framing rule (`FRAMING_CFG["rule"]`, default "aim_centre") | DONE | e9d8bd7 | neck_rule_test 10/10: movement 0.80 rad vs 0.99, margin 10 %, repeatability 0.0028 rad over 2 draws |

## The one thing that needs you (Stage 5)
With the measured rectangle (left arm: forward 0.15 to 0.34 m, sideways -0.05 to +0.41 m) the centred spot and all four shifted spots fail the full check on `placed_beaker`: the arms at home touch a beaker on the hotplate (the largest rectangle sits near the shoulder, the home pose needs the targets about 0.38 m ahead). Per your rule the selection stops and reports. A hand-set rectangle 14 mm from TASK_CFG's stance passes the full check but the real place then fails 8.4 mm short. Details and options: docs/reports/20261022_1416_queue_stage5.md.

## Test suite state
- Final full suite 20261022_141600: 29 modules passed, none failed, then `full_check_test` was OOM-killed in section 7 at 15:29 (the machine's idle memory use had risen from 2.2 GB to 3.4 GB; the test needs about 4.5 GB). I halved that section's headings (every 10 degrees; the angle tolerance became one 10-degree step) and restarted the rest.
- That restart (TAG 20261022_153117) died at 15:31 when this session ended (tmux server gone); no result from it. Not yet run in the final suite: full_check_test, nav_core_test, nav_follow_test, nav_follow_sim_test, nav_pipeline_follow_test, stance_select_test, stance_select_pipeline_test (the `--plumbing-same` variant), neck_rule_test, mpvi_pipeline_test, mpvi_text_test, safety_filter_model_check; face_test alone at the end. Each of these passed individually earlier today.
- A runner edited while running broke the earlier subset run's tail once (bash reads scripts as it runs): I no longer edit it during a run.

## Still to do
Rerun the remaining tests (restarting now), face_test alone, then docs/reports/<ts>_overnight_summary.md (stage table, all questions in one list, which settings to flip), then stop.

## Questions so far (full lists in the stage reports)
Stage 2: footprint 0.60 m wide at the zombie pose; obstacle source. Stage 3: follower starting values (M148/M149); keep gentle limits as lean cannot be measured in this sim; DRIVE takes 34.5 s instead of 20.2 s. Stage 5: five questions (stand-off, work heights, left arm only, full check margin vs real execution, timing). Stage 6: aim error 3.3 degrees acceptable?
