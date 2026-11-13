# Failed runs study, 2026-11-09 (read only; untracked)

Read-only study of saved logs, saved results and code. No code changes, no simulator runs, no commits, no push. HEAD at start and end: dbca9e6. Helper scripts are outside the repo in /home/vboxuser/projects/oc_scripts/ (fr_*.py). The only file written in the repo is this report.

Question: does a later subtask fail because of the pose the earlier subtask left the arm in? David's hypothesis: when the arm holds the beaker between the beaker and the hotplate, the next subtask must either twist the hand a long way or go over the hotplate, and that causes many failures.

Short answer from the saved data: there is no handoff jump. The place's first move starts from the pose the pick left (the largest joint change at the pick-to-place boundary is 0.010 rad). The large motion is inside the place OVER move: its largest joint change is 0.74 to 1.05 rad at a place hand turn of -55 deg, and 0.27 to 0.31 rad at -30 deg. The hand always passes over the hotplate in the place OVER move, by design. Joint 3 changes most in every place OVER move, not the wrist. So David's hypothesis is partly supported (the large move and the over-the-hotplate path occur together in the -55 deg runs), but the saved data show no handoff jump. Most failures (50 of 60 logs) are pick failures with no earlier subtask at the same stance. Group (c) fails at the place's own carry stop over the hotplate, which the check tests without the pick's end pose.

## 1. What was done and not done

- Done: listed every saved failure in three groups (section 2); per-check results where the saved record has them (section 3); joint jumps at each pick-to-place boundary and inside the place OVER move from the saved joint traces (section 4); hotplate and bench clearance from saved beaker and pad traces (section 5); code reading with file and line (section 6); the 51-stance pair table (section 7); shared numbers (section 8).
- Not done: no simulator run, no check run, no code change, no test, no commit, no push, no edit to docs/progress.md (the task allowed only this file). Recordings of the D cases went to /tmp and are gone, so those joints cannot be read.

## 2. Failure counts by group

| group | what | count | notes |
|---|---|---|---|
| (a) | stance candidates that failed the full check | 47 distinct stances fail in at least one of 4 saved 51-stance runs (28, 29, 30, 30 per run; 9 fail in all four) | the production check stops at its first failing check, so most records hold one check only; see section 3 |
| (a) | M281 logged runs | 4 of 6 logged stances fail (n=2, 4, 5, 6) | n=6 is in checks.jsonl only (see section 9) |
| (a) | spread-candidate sheet, Nov 8 | 10 unique stances, all fail | the moved sheet repeats the standard sheet exactly (same geometry shifted 0.15 m) |
| (a) | analytic-inverse candidates, Nov 8 | 8 per scene in each of 3 saved runs (sweep, fixed heading, 0% occlusion) | first failing check only |
| (a) | layout sweep, Nov 6 | 98 candidate entries in 31 failing layouts; 69 layouts had no stance covering the objects (no check ran) | first failing check per candidate |
| (b) | pipeline and mock logs with a step failure, Nov 5 to 9 | 60 logs: 50 pick failures, 10 place failures | many logs repeat one run (section 2b) |
| (b) | excluded: pipeline or mock logs that crashed or aborted before a step | 10 | StanceSelectionError (7 Nov 6 moved tests, 1 Nov 8), ABORTED (2 Nov 5) |
| (c) | pick passed, no place angle passed (2026-11-08 stances) | 12 of the 51 stances (B indices 37 to 48) | 11 have no passing place turn; 1 (index 45) has two passing turns (-70, -65 deg), a 5 deg window, under the 10 deg minimum |

### 2a. Group (a): per-check results

The per-check table counts each check's own result where the saved record has one. "Not run" means the saved record stopped before that check.

| source | stances | ik (pick grasp IK) | place_angle_window | home_clearance | placed_beaker | view | occlusion beaker | occlusion hotplate | body | how the check result was saved |
|---|---|---|---|---|---|---|---|---|---|---|
| 51-stance, given order, no reseed (B_given_noreseed) | 28 failing | 16 | 12 | not run | not run | not run | not run | not run | not run | first failing check only (production stops early) |
| 51-stance, reverse, no reseed | 29 failing | 17 | 12 | not run | not run | not run | not run | not run | not run | first failing check only |
| 51-stance, given, reseed | 30 failing | 18 | 12 | not run | not run | not run | not run | not run | not run | first failing check only |
| 51-stance, reverse, reseed | 30 failing | 18 | 12 | not run | not run | not run | not run | not run | not run | first failing check only |
| M281 logged, per-check (failures.jsonl) | 3 failing (n=2, 4, 5) | 2 | 1 | 0 | 0 | 0 | 0 | 0 | 0 | every check run on its own (logging mode) |
| M281 logged, n=6 (checks.jsonl only) | 1 failing | not saved | 1 | not saved | not saved | not saved | not saved | not saved | not saved | first failing check only |
| spread sheet, standard = moved (Nov 8), top 10 | 10 unique | 1 fail, 1 pass, 8 not run | 0 fail | 8 | 0 | 2 | 5 | 0 | 0 | margin per check, negative = fail |
| analytic inverse, sweep, standard (Nov 8) | 8 | not saved | not saved | 4 | 4 | not saved | not saved | not saved | not saved | first failing check only |
| analytic inverse, sweep, moved (Nov 8) | 8 | not saved | not saved | 8 | 0 | not saved | not saved | not saved | not saved | first failing check only |
| analytic inverse, fixed heading, both scenes (Nov 8) | 8 + 8 | not saved | not saved | not saved | not saved | not saved | 16 (occlusion) | not saved | not saved | first failing check only |
| layout sweep, Nov 6 (98 entries in 31 failing layouts) | 98 | 58 | not saved | 19 | 5 | 16 | not saved | not saved | not saved | first failing check per candidate |

The M281 n=2 and n=4 records fail only at the pick (ik). The M281 n=5 record fails only at the place (place_angle_window). The other checks pass in those three records.

### 2b. Group (b): pipeline and mock step failures, by step

The failing subtask is the segment where the orchestrator reports FAILED. A pick and a place at the same stance are one stance run (no navigation between them), so a place failure is a later subtask at the same stance as the pick.

| failing subtask and first failing step | count | logs |
|---|---|---|
| pick, OVER | 30 | held200_mock_20261105_024640; suite and parent copies of ped016 (Nov 5, 10 logs); Nov 6 optA, ab_bench (2), cur_bench, cur2, step4 current and two-pass pipeline, twopass and twopass2 (18 logs); head_mpvi_pipeline_test_20261108 |
| pick, IN | 14 | held200 mock and scripted pairs (Nov 5, 11 logs, incl. head_191821 and after_guard); m275_.log, m275_stance_select_moved_test, m275_mpvi_pipeline_test (Nov 8, 3 logs) |
| pick, DOWN | 3 | heights_stance_select_moved_test, heights_mpvi_pipeline_test, in_probe_20261108 (Nov 8) |
| pick, CLOSE | 3 | guardoff_stance_select_moved_test and its rerun (Nov 8); pt/stance_select_moved_test (Nov 9) |
| place, OVER | 2 | overnight A_mock_oc_base (Nov 9); pt/mock.log (Nov 9, same run) |
| place, DOWN then HOME recovery | 4 | overnight A_pipeline_oc_base; pt/mpvi_pipeline_test (Nov 9); overnight D1_pipeline_park012; D2_pipeline_both016 |
| place, HOME (first failing step) | 4 | overnight A_pipeline_oc_a2 and a3 (both place at -30 deg); guardoff_mpvi_pipeline_test and guardoff_probe (Nov 8) |

Totals by step over the 60 logs: OVER 32 (30 pick, 2 place); IN 14; DOWN as the first failing step 7 (3 pick, 4 place); HOME as the first failing step 4 (place); CLOSE 3. HOME appears in 8 logs (4 first failing, plus the 4 recoveries after DOWN). By place hand turn: the 6 place failures at -55 deg are 2 at OVER and 4 at DOWN then HOME; the 4 at -30 deg fail at HOME first (the -30 turn for the guard-off runs is inferred, see section 11).

Duplicates: A_pipeline_oc_a2 and a3 have identical traces (same -30 deg place). A_mock_oc_base and pt/mock have identical traces. guardoff_mpvi and guardoff_probe have identical traces. Many suite and parent copies repeat one stance run. Distinct runs are fewer than 60.

### 2c. Group (c): pick passed, no place angle passed

Stances (B index; per-task table row = index + 1): 37 (1.140, 0.914, -130, 0.20), 38 (1.188, 0.947, -130, 0.15), 39 (1.163, 0.907, -130, 0.20), 40 (1.166, 0.906, -130, 0.20), 41 (1.138, 1.011, -120, 0.10), 42 (1.124, 0.980, -120, 0.15), 43 (1.059, 1.002, -110, 0.15), 44 (1.063, 1.001, -110, 0.15), 45 (0.996, 1.049, -100, 0.10), 46 (1.070, 1.000, -110, 0.15), 47 (1.082, 1.033, -110, 0.10), 48 (1.008, 1.047, -100, 0.10).

- Pick angles passing: 1 to 3 bins (indices 0, 10, 11 in B). The pick passes at all error stances.
- Place angles: 11 stances have no passing turn at all. Index 45 has two (-70, -65 deg). The place window is 5 deg, under the 10 deg minimum (B reason text).
- Failing stop: 11 of 12 B reasons name the nominal carry stop (the stop over the hotplate) as the most common failure. The M281 n=5 record (B index 37) fails the nominal carry stop at 63 of 72 turns and the setdown stop at 9.
- Joint trace: none saved (the check is solve-only). See section 4.

## 3. Group (a) per-check detail (M281 logged records)

| stance (x, y, heading, ped) | result | ik | place_angle_window | home_clearance | placed_beaker | view | occlusion beaker | occlusion hotplate | body | clearance pair |
|---|---|---|---|---|---|---|---|---|---|---|
| (0.6655, 0.9837, -70, 0.20) n=2 | fail | FAIL (task 1, nominal, grasp point 2 at z 1.005) | ok | ok | ok | ok | ok | ok | ok | not failed |
| (0.6037, 0.9805, -60, 0.15) n=4 | fail | FAIL (task 1, heading+1, grasp point 2) | ok | ok | ok | ok | ok | ok | ok | not failed |
| (1.1404, 0.9145, -130, 0.20) n=5 | fail | ok | FAIL (carry stop) | ok | ok | ok | ok | ok | ok | not failed |

## 4. Joint jumps: pick to place at the same stance

Method (saved traces only): the pick's last row (UP, or DONE in the mock) and the place's first row (OVER). "Boundary" = that pair of rows. "Demanded" = the place's first commanded joint target minus the pick's end pose (episode log: cmd_left_joint; probe: tgt). "Place OVER move" = joint change from the first to the last OVER row (the move's own change). Joints are the left arm, 1 to 7; joint 7 is the last joint. Angles in rad. Threshold 0.25 rad.

| run | place turn | later failure | boundary largest joint (actual / demanded) | place OVER largest joint | place OVER joints over 0.25 rad | largest-change joint order in OVER |
|---|---|---|---|---|---|---|
| A_pipe_oc_base (pipeline) | -55 | DOWN then HOME | j4 0.002 / j5 0.006 | j3 +1.045 | 1,2,3,4,5,6,7 | j3, j1, j5, j4, j6, j7 |
| A_pipe_oc_a2 | -30 | HOME | j4 0.002 / j5 0.006 | j3 +0.314 | 3 | j3, j6, j2, j1 |
| A_pipe_oc_a3 (identical to a2) | -30 | HOME | j4 0.002 / j5 0.006 | j3 +0.314 | 3 | j3, j6, j2, j1 |
| A_mock_oc_base (probe) | -55 | OVER (no DOWN reached) | j3 0.000 / j5 0.010 | j3 +0.741 | 1,3,4,5,6,7 | j3, j6, j1, j4 |
| pt/mock (Nov 9; identical to A_mock_oc_base) | -55 | OVER | j3 0.000 / j5 0.010 | j3 +0.741 | 1,3,4,5,6,7 | j3, j6, j1, j4 |
| guardoff_mpvi_run (Nov 8 pipeline) | -30 (by inference) | HOME | j4 0.002 / j5 0.006 | j3 +0.314 | 3 | j3, j6, j2, j1 |
| guardoff_probe (Nov 8, identical to guardoff_mpvi) | -30 (by inference) | HOME | j4 0.002 / j5 0.006 | j3 +0.314 | 3 | j3, j6, j2, j1 |
| control: A_mock_oc_a2 (passed) | -30 | none | j3 0.000 / j5 0.010 | j3 +0.272 | 3 | j3, j6, j2, j1 |
| control: A_mock_oc_a3 (passed, identical) | -30 | none | j3 0.000 / j5 0.010 | j3 +0.272 | 3 | j3, j6, j2, j1 |
| D1_pipeline_park012 | -55 | DOWN then HOME | not saved (recording in /tmp, now gone) | not saved | not saved | not saved |
| D2_pipeline_both016 | -55 | DOWN then HOME | not saved (recording in /tmp, now gone) | not saved | not saved | not saved |
| pt/mpvi_pipeline_test (Nov 9) | -55 (by inference) | DOWN then HOME | not saved (no episode file in data/logs) | not saved | not saved | not saved |

Pick failures (50 logs) have no earlier subtask at the same stance (the pick is the first subtask at the staging stance; navigation runs at another stance), so they are not in this table.

Group (c): no joint trace is saved for any of the 12 stances. Not measurable from saved data.

Which joint dominates. In every place OVER move, joint 3 has the largest change. The wrist (joint 6) is second in the -55 deg mock (-0.66 rad) and second or third in the others. Joint 7 (last wrist joint) changes -0.39 to +0.03 rad. So the wrist twist does not dominate the place OVER; joint 3 does. In the HOME failure, the largest error from the Nov 8 report is joint 6 (see section 6, HOME).

## 5. Hotplate and bench clearance along the place OVER move

The footprint is the hotplate at (0.80, 0.66) with half extent 0.065 m (the beaker footprint check). Heights: hotplate top 1.028 m; bench top 0.960 m. beaker_z in the logs is the beaker bottom (it reads 0.960 at rest on the bench).

| run | beaker over hotplate footprint (frames, time) | min beaker height above hotplate top while over footprint | min pad height above hotplate top while over footprint | min beaker height above bench, OVER window |
|---|---|---|---|---|
| A_pipe_oc_base | 17 frames, 84.90 to 86.50 s | 0.027 m | not saved (no pad column) | 0.095 m |
| A_pipe_oc_a2 | 9 frames, 83.80 to 84.60 s | 0.029 m | not saved | 0.097 m |
| guardoff_mpvi (Nov 8) | 9 frames, 83.80 to 84.60 s | 0.029 m | not saved | 0.097 m |
| A_mock_oc_base | 1988 frames, 87.79 to 101.04 s (stalled over the plate) | 0.035 m | 0.090 m | 0.103 m |
| pt/mock (Nov 9) | 1988 frames, 87.79 to 101.04 s | 0.035 m | 0.090 m | 0.103 m |
| control: A_mock_oc_a2 | 260 frames, 87.05 to 88.78 s | 0.038 m | 0.093 m | 0.106 m |
| control: A_mock_oc_a3 | 260 frames, 87.05 to 88.78 s | 0.038 m | 0.093 m | 0.106 m |

- The pick UP and all pick phases do not enter the hotplate footprint. In both probe runs, 0 of 2,994 pick frames have the pad inside the footprint (pad x 0.928 to 1.003 m; footprint x 0.735 to 0.865 m).
- The hand and beaker enter the hotplate footprint only in the place OVER move. This is by design: the place stop is over the hotplate.
- The mock base stalls at the place OVER stop (3.4 mm short, 15 s) with the hand 0.090 m above the plate top, over the plate, for 13 s.
- The pipeline runs (A_pipe, guardoff) have no pad column; the beaker columns are the only height data.

## 6. Code findings (read only; file and line in src/)

(i) Pose left at the end of a pick and of a place; pose a skill starts from.
- Pick: the steps are OVER, DOWN, IN, CLOSE, UP (src/mpvi_modules.py:246-247). UP ends at the lift point with the beaker held, not at home. If the stance selection set end_at_zombie, BACK and HOME are appended (src/mpvi_modules.py:248-251); the home pad pose is at :191-197. end_at_zombie is set when the next navigation step moves (src/stance_select.py:626-629; flag read at src/mpvi_modules.py:285).
- Place: OVER, DOWN, OPEN, BACK, HOME, then joints to home (src/mpvi_modules.py:274-275). A place always ends at home (the zombie pose), with the hand open.
- Start of a skill: plan_pick reads the current pad pose and the grounded object (src/mpvi_modules.py:234-236). The place's OVER height is the current pad height p[2], and its xy offset is the pad-to-beaker offset measured now (src/mpvi_modules.py:261, 269). start() resets the arm to the measured state (arm.reset at src/mpvi_modules.py:297), or continues from the commanded state when restart_from_commanded is set (:293).
- So the place starts from where the pick left the arm, and its OVER target is computed from that pose. That is why the boundary jump is near zero.

(ii) Does the stance check test each subtask's approach angles on their own, or a pair?
- Each pick task is solved on its own: the loop over pick tasks solves only that task's points (src/full_check.py:744-773; per-task points at :748-749).
- Each place task is solved on its own by _place_window (src/full_check.py:543-581; called per place task at :790-798). Its warm start is the nominal solution of the previous passing turn of the same window (:558-572), not the pick's end pose.
- Nothing carries the earlier subtask's end pose into the later subtask's start. The check tests no pair.
- Production stops at the first failing check (early = only is None, src/full_check.py:677; signature src/full_check.py:645). This is why most saved records hold one check. The M281 per-check records run each group on its own (src/full_check.py:634 calls check with only=g).

(iii) Is the arm held close (home or tucked) while the base moves between stances?
- Yes. The staging rule: the arms stay at home (zombie pose) except during manipulation (src/staging_source.py:23-25). Phases: ARMS moves the arms to home at the spawn, then PEDESTAL, DRIVE (arms at home), CREEP_IN (arms at home) (src/staging_source.py:26-50; DRIVE entered at :708; CREEP_IN at :715-750; PEDESTAL entered at :818).
- A pick that is followed by a moving navigation step returns to the zombie pose holding the beaker (end_at_zombie, src/mpvi_modules.py:248-251, src/stance_select.py:626-629).
- The code for the held-close pose during base motion is in src/staging_source.py (make_staging_source) and the end_at_zombie flag in src/stance_select.py and src/mpvi_modules.py.

(iv) How many subtasks per stance does the code support?
- The check loops over any number of pick tasks and place tasks at one stance (pick_tasks and place_tasks built by _ordered_tasks at src/full_check.py:215, used at :745 and :790). A stance run holds a list of steps (src/stance_select.py:516, 600).
- The executed staging parks one stance per run: a second stance is reported and dropped (src/stance_select.py:22 and :619-620). Several tasks at one stance are supported; several stances are not.

HOME stall (existing reports, not re-run). The Nov 8 guard-off report gives the place HOME failure: largest joint error 0.267 rad after 9 s, joint 6 at 12.0 deg with home at 27.3 deg (docs/reports/guard_off_20261108.md:49). The Nov 8 replay report finds joint 6 frozen at 11.4 deg from 93.8 s, and attributes it to the sag layer, which does not add the trim to the source's joint targets (docs/reports/replay_home_close_20261108.md:71-74). So the HOME stall is a wrist (joint 6) hold, not a handoff jump. Only one run is saved with a per-joint HOME result.

## 7. The 51 overnight stances: pick and place, tested together?

Saved data: data/logs/overnight_20261109/B_given_noreseed.jsonl (the table is for this run; the reseed columns come from B_given_reseed.jsonl). B index = per-task table row minus 1 (docs/reports/per_task_angle_lists_20261109.md). "Pick and place tested together?" is No for every row: each task is solved on its own (section 6 ii). Pick and place are checked in the same stance call, not as a pair.

| B index (table row) | stance (x, y, heading, ped) | pick angles passing (bin index) | place angles passing (deg) | pick and place tested together? | first failed check, given no reseed | first failed check, given reseed | pass: given no reseed | pass: given reseed | pass: reverse no reseed | pass: reverse reseed |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 (1) | (0.870, 1.072, -100, 0.10) | 11 | 11 turns, -60 to -10 | No (separate per-task solves) | - | - | yes | yes | yes | yes |
| 1 (2) | (0.842, 1.077, -100, 0.10) | 11 | 12 turns, -60 to -5 | No (separate per-task solves) | - | - | yes | yes | yes | yes |
| 2 (3) | (0.806, 1.072, -90, 0.10) | 11 | 11 turns, -60 to -10 | No (separate per-task solves) | - | - | yes | yes | no | no |
| 3 (4) | (0.793, 1.074, -90, 0.10) | 11 | 12 turns, -60 to -5 | No (separate per-task solves) | - | - | yes | yes | no | no |
| 4 (5) | (0.781, 1.076, -90, 0.10) | 11 | 12 turns, -60 to -5 | No (separate per-task solves) | - | - | yes | yes | no | no |
| 5 (6) | (0.781, 1.051, -90, 0.15) | 0,11 | 12 turns, -60 to -5 | No (separate per-task solves) | - | - | yes | yes | no | no |
| 6 (7) | (0.748, 1.032, -80, 0.15) | 0,11 | 10 turns, -65 to -20 | No (separate per-task solves) | - | - | yes | yes | no | no |
| 7 (8) | (0.773, 1.053, -90, 0.15) | 0,11 | 12 turns, -60 to -5 | No (separate per-task solves) | - | - | yes | yes | no | no |
| 8 (9) | (0.822, 1.069, -90, 0.10) | 11 | 10 turns, -65 to -20 | No (separate per-task solves) | - | - | yes | yes | no | no |
| 9 (10) | (0.731, 1.061, -80, 0.10) | 0,11 | 11 turns, -60 to -10 | No (separate per-task solves) | - | - | yes | yes | no | no |
| 10 (11) | (0.718, 1.063, -80, 0.10) | 11 | 12 turns, -60 to -5 | No (separate per-task solves) | - | - | yes | yes | no | no |
| 11 (12) | (0.710, 1.064, -80, 0.10) | 11 | 12 turns, -60 to -5 | No (separate per-task solves) | - | - | yes | yes | no | no |
| 12 (13) | (0.671, 1.012, -70, 0.15) | 0,11 | 11 turns, -60 to -10 | No (separate per-task solves) | - | - | yes | yes | no | no |
| 13 (14) | (0.658, 1.038, -70, 0.10) | 0,11 | 12 turns, -60 to -5 | No (separate per-task solves) | - | - | yes | yes | no | no |
| 14 (15) | (0.654, 1.038, -70, 0.10) | 0,11 | 12 turns, -60 to -5 | No (separate per-task solves) | - | - | yes | yes | no | no |
| 15 (16) | (0.613, 1.000, -60, 0.10) | 11 | 11 turns, -60 to -10 | No (separate per-task solves) | - | - | yes | yes | no | no |
| 16 (17) | (0.658, 1.015, -70, 0.15) | 11 | 12 turns, -60 to -5 | No (separate per-task solves) | - | - | yes | yes | no | no |
| 17 (18) | (0.604, 1.001, -60, 0.10) | 11 | 11 turns, -60 to -10 | No (separate per-task solves) | - | - | yes | yes | no | no |
| 18 (19) | (0.665, 0.984, -70, 0.20) | - | - | No (separate per-task solves) | ik | ik | no | no | no | no |
| 19 (20) | (0.661, 0.985, -70, 0.20) | - | - | No (separate per-task solves) | ik | ik | no | no | no | no |
| 20 (21) | (0.645, 1.018, -70, 0.15) | 11 | 12 turns, -60 to -5 | No (separate per-task solves) | - | - | yes | yes | no | no |
| 21 (22) | (0.595, 1.002, -60, 0.10) | 11 | 12 turns, -60 to -5 | No (separate per-task solves) | - | ik | yes | no | no | no |
| 22 (23) | (0.656, 0.986, -70, 0.20) | - | - | No (separate per-task solves) | ik | ik | no | no | no | no |
| 23 (24) | (0.591, 1.003, -60, 0.10) | 11 | 12 turns, -60 to -5 | No (separate per-task solves) | - | ik | yes | no | no | no |
| 24 (25) | (0.604, 0.981, -60, 0.15) | - | - | No (separate per-task solves) | ik | ik | no | no | no | no |
| 25 (26) | (0.561, 0.936, -50, 0.15) | - | - | No (separate per-task solves) | ik | ik | no | no | no | no |
| 26 (27) | (0.590, 0.983, -60, 0.15) | - | - | No (separate per-task solves) | ik | ik | no | no | no | no |
| 27 (28) | (0.539, 0.957, -50, 0.10) | - | - | No (separate per-task solves) | ik | ik | no | no | yes | no |
| 28 (29) | (0.556, 0.936, -50, 0.15) | - | - | No (separate per-task solves) | ik | ik | no | no | no | no |
| 29 (30) | (0.530, 0.958, -50, 0.10) | - | - | No (separate per-task solves) | ik | ik | no | no | no | no |
| 30 (31) | (0.547, 0.938, -50, 0.15) | - | - | No (separate per-task solves) | ik | ik | no | no | yes | yes |
| 31 (32) | (0.526, 0.958, -50, 0.10) | - | - | No (separate per-task solves) | ik | ik | no | no | no | no |
| 32 (33) | (0.542, 0.939, -50, 0.15) | - | - | No (separate per-task solves) | ik | ik | no | no | yes | no |
| 33 (34) | (0.521, 0.959, -50, 0.10) | - | - | No (separate per-task solves) | ik | ik | no | no | no | yes |
| 34 (35) | (0.537, 0.940, -50, 0.15) | - | - | No (separate per-task solves) | ik | ik | no | no | yes | yes |
| 35 (36) | (0.516, 0.959, -50, 0.10) | - | - | No (separate per-task solves) | ik | ik | no | no | yes | yes |
| 36 (37) | (0.512, 0.960, -50, 0.10) | - | - | No (separate per-task solves) | ik | ik | no | no | yes | yes |
| 37 (38) | (1.140, 0.914, -130, 0.20) | 0,10,11 | - | No (separate per-task solves) | place_angle_window | place_angle_window | no | no | yes | yes |
| 38 (39) | (1.188, 0.947, -130, 0.15) | 10 | - | No (separate per-task solves) | place_angle_window | place_angle_window | no | no | yes | yes |
| 39 (40) | (1.163, 0.907, -130, 0.20) | 10,11 | - | No (separate per-task solves) | place_angle_window | place_angle_window | no | no | yes | yes |
| 40 (41) | (1.166, 0.906, -130, 0.20) | 10,11 | - | No (separate per-task solves) | place_angle_window | place_angle_window | no | no | yes | yes |
| 41 (42) | (1.138, 1.011, -120, 0.10) | 10 | - | No (separate per-task solves) | place_angle_window | place_angle_window | no | no | yes | yes |
| 42 (43) | (1.124, 0.980, -120, 0.15) | 10,11 | - | No (separate per-task solves) | place_angle_window | place_angle_window | no | no | yes | yes |
| 43 (44) | (1.059, 1.002, -110, 0.15) | 10 | - | No (separate per-task solves) | place_angle_window | place_angle_window | no | no | yes | yes |
| 44 (45) | (1.063, 1.001, -110, 0.15) | 10,11 | - | No (separate per-task solves) | place_angle_window | place_angle_window | no | no | yes | yes |
| 45 (46) | (0.996, 1.049, -100, 0.10) | 10 | 2 turns, -70 to -65 | No (separate per-task solves) | place_angle_window | place_angle_window | no | no | yes | yes |
| 46 (47) | (1.070, 1.000, -110, 0.15) | 10 | - | No (separate per-task solves) | place_angle_window | place_angle_window | no | no | yes | yes |
| 47 (48) | (1.082, 1.033, -110, 0.10) | 10 | - | No (separate per-task solves) | place_angle_window | place_angle_window | no | no | yes | yes |
| 48 (49) | (1.008, 1.047, -100, 0.10) | 10,11 | - | No (separate per-task solves) | place_angle_window | place_angle_window | no | no | yes | yes |
| 49 (50) | (0.775, 1.105, -90, 0.12) | 0,11 | 11 turns, -55 to -5 | No (separate per-task solves) | - | - | yes | yes | yes | yes |
| 50 (51) | (0.870, 1.072, -100, 0.10) | 11 | 11 turns, -60 to -10 | No (separate per-task solves) | - | - | yes | yes | yes | yes |

Summary: 23 pass in this run (given, no reseed); 16 fail ik; 12 fail place_angle_window. The 12 place failures are the 12 stances of section 2c.

## 8. Shared numbers (group b and c failures, and the saved traces)

- Pick-to-place boundary jump over 0.25 rad: 0 of 7 saved failing place runs (largest 0.010 rad, demanded).
- Place OVER move with a joint over 0.25 rad: 7 of 7 saved failing place runs, and 2 of 2 passing controls. The 0.25 threshold does not separate them.
- Place OVER largest joint change: 1.045 rad (pipeline, -55), 0.741 rad (mock, -55), 0.314 rad (pipeline, -30), 0.272 rad (mock control, -30).
- Joint 3 is the largest joint change in all 4 distinct place OVER moves above. Joint 6 (wrist) is 0.66 rad in the -55 mock, 0.45 rad in the -55 pipeline, and 0.21 to 0.24 rad in the -30 runs.
- Share of the 10 place failures with the hand or beaker over the hotplate footprint during the place OVER: 7 of 7 saved (100%); the other 3 (D1, D2, pt/mpvi) have no trace, and their OVER completed before the DOWN failure.
- Share of the 50 pick failures with the pad or beaker over the hotplate footprint: 0 of 2 saved pick traces (0 of 2,994 frames each). The other pick failures have no saved trace.
- Smallest height above the hotplate top while over the footprint (place OVER): 0.027 to 0.038 m (beaker bottom); pad 0.090 to 0.093 m.
- Smallest height above the bench top during the place OVER: 0.095 to 0.106 m.
- Share of the 60 step-failure logs with a later subtask at the same stance (place reached): 10 of 60 (17%).
- Share of the 16 ik failures (51-stance, given, no reseed) whose first failing point is the pick's grasp point: 16 of 16. None has an earlier subtask at the same stance.
- Share of the 12 group (c) stances whose reason names the nominal carry stop as the most common failing stop: 11 of 12 (the 12th has two passing turns and fails on width).
- Share of the 12 group (c) stances with a joint trace: 0 of 12.
- Number of failing place stops at the nominal carry (M281 n=5 record): 63 of 72 turns (88%).

## 9. Inconsistencies found (not resolved)

- data/logs/stance_failures/checks.jsonl has 6 logged stances (n=1 to 6), 4 failing (n=2, 4, 5, 6). failures.jsonl has 3 records (n=2, 4, 5). The n=6 failure (1.0 0.9963, 1.0488, -100, 0.10) is absent from failures.jsonl; its per-check detail is not saved. This stance is B index 45 (group c).
- The B records hold one failing check per stance, consistent with the early stop. The M281 record for the same stance (n=5, B index 37) shows the carry stop as the first failing stop.

## 10. Needs a run to answer

- Per-check results for all 51 stances in logging mode (every check on its own). Saved data has first failing check only.
- Joint traces for the pick end and place start on the 12 group (c) stances: the check is solve-only, so no joints are saved. A run that solves or plays the pick and then the place from the pick end pose would give this.
- Whether the place carry-stop failure (group c) depends on the pick's end pose: a check that starts the place from the pick's end pose would test it. Not run.
- Pad positions and joints for the pipeline runs (A_pipe, guardoff): the episode log has the beaker but no pad column, and no per-tick joint command beyond the first move.
- D1 and D2 joint traces: the recordings went to /tmp and are gone; a re-run would be needed.
- Whether the mock OVER stall at 3.4 mm (A_mock_oc_base, pt/mock) comes from joint 3 or the wrist: the trace shows joint 3 and 6 both moving; a run with per-tick command data would settle it.
- Which joint holds the HOME error (joint 6 in one run) across the other HOME failures: per-joint HOME error is saved for one run only.

## 11. Not found

- A Nov 6 log with 8 candidates. The 8-candidate sets are Nov 8 (analytic inverse: sweep, fixed heading, 0% occlusion).
- A dedicated 2026-11-08 report for the 12 stances. The 12 are listed in docs/reports/per_task_angle_lists_20261109.md (section 5, rows 38 to 49, dated Nov 9).
- Joint traces for the Nov 6 pipeline runs, the Nov 8 heights, head and m275 pipeline runs, and pt/mpvi_pipeline_test (no episode files in data/logs for them).
- Recordings of D1 and D2 (written to /tmp, now absent).
- Per-check results (other than first failing) for the 51-stance runs, the layout sweep, and the Nov 8 analytic-inverse candidates.
- The place turn of the guardoff run (Nov 8): not logged; -30 deg is inferred from identical traces to the -30 deg runs.

## 12. Files

- This report: docs/reports/failed_runs_study_20261109.md (untracked).
- Helpers (outside the repo): /home/vboxuser/projects/oc_scripts/fr_summary.py, fr_scan_logs.py, fr_jumps.py, fr_moves.py, fr_heights.py, fr_steps.py, fr_table51.py, fr_table51_rows.md.
- Source data: data/logs/overnight_20261109/ (B_*.jsonl, A_*, D*), data/logs/stance_failures/ (checks.jsonl, failures.jsonl), docs/reports/per_task_angle_lists_20261109.md, docs/reports/layout_sweep_current_20261106.json, docs/reports/reach_inverse_*.json, docs/reports/reach_inverse_spread_candidates.xlsx, docs/reports/guard_off_20261108.md, docs/reports/replay_home_close_20261108.md.
