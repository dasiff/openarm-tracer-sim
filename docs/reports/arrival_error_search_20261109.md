# Arrival error and stance tolerance: what the repo already holds, 2026-11-09

Written for: David (deciding whether the middle place angle is safe against the real base arrival error).
Read-only search. No file in src/, config, tests or docs was changed, no simulator was run, no commit, no push. This report is untracked. HEAD at the time of the search: 543de46.

Search scope: docs/reports/, docs/progress.md, docs/mpvi_decisions.md, docs/design_plan.md, docs/TODO.md, experiments/ (docstrings and code), data/logs (summary and log lines only), and `git log --all` messages for arrival, stopping error, stop error, tolerance, pose error, creep, nav_follow, approach spot, repeatability, accuracy.

## 1. Summary

- Base arrival error with the follower on or off was measured in **single runs** in the staging. The saved data holds 70 arrival lines in data/logs, but they carry **10 distinct values**. No file computes a standard deviation, percentile or worst case over repeated arrivals of a randomized setup.
- Real-robot base stopping accuracy has never been measured. The 1 cm / 1 degree value is a **stand-in** taken from the sim's own pose-correction thresholds. Design plan §11 still lists the real value as open (owner: Michael).
- The stance tolerance numbers (xy 7.1 cm, heading ±8 deg, pedestal ±4.2 cm) come from the **2026-10-19 finder run** (report dated 20261019), and were adopted in the staging-stance decision of 2026-10-20. They are not a repeated measurement.
- No study measured the arrival error with the arms out or with a beaker held. Every saved arrival has the arms at the zombie pose during the DRIVE.

## 2. Table of studies and measurements found

Position error is the distance from the base origin to the target (the approach spot, or the handoff pose). Heading error is the yaw difference. "Trials" counts saved runs.

| # | Path | Date | What it measured | Trials | Position error (mean / worst) | Heading error (mean / worst) | Conditions |
|---|---|---|---|---|---|---|---|
| A1 | docs/reports/20261022_1319_queue_stage3.md (code: commits 8c6bbbc, c0d20d7) | 2026-10-22 | Staging arrival at the approach spot and handoff pose error, follower off vs on | 1 run each | Follower off: arrival 3.1 mm, handoff 3.9 mm. Follower on: arrival 3.8 mm, handoff 4.0 mm | Off: 0.19 deg arrival, 0.20 deg handoff. On: 0.15 deg arrival, 0.16 deg handoff | Full staging in the sim (ARMS, PEDESTAL, DRIVE, CREEP_IN), zombie pose, no beaker held at the drive. DRIVE 20.2 s (off) and 34.5 s (on). Follower on: route 2.11 m, 0 replans, cross-track 2.9 mm, peak lean 0.00 deg |
| A2 | docs/reports/20261022_1547_progress_status.md line 10 | 2026-10-22 | Same as A1, follower on (nav_follow_sim_test 5/5) | 1 run | 3.8 mm | 0.15 deg | As A1 |
| A3 | docs/reports/20261022_1319_queue_stage3.md, Tests section (kinematic, no simulator) | 2026-10-22 | Follower on a kinematic unicycle at 150 Hz (nav_follow_test, 10/10) | 10 tests, one arrival value saved | 3.7 mm | 0.00 deg | No simulator. Not a sim or hardware arrival |
| A4 | docs/reports/20261023_1520_h_h2_c3.md line 24 | 2026-10-23 | Arrival at the chosen stance in the pipeline test | 1 run | 3.1 mm | 0.19 deg | Pipeline SUCCESS, beaker on the hotplate. Follower status not stated in the report (the 3.1 / 0.19 pattern matches follower-off runs) |
| A5 | docs/reports/20261025_0500_held_contacts_moved_layout.md line 34 | 2026-10-25 | Arrival in the moved layout (beaker and hotplate 15 cm shifted along the bench) | 1 run | 3.1 mm (handoff 3.9 mm) | 0.19 deg | Moved layout. The report does not record the follower state |
| A6 | data/logs, 70 arrival lines (listed in section 3) | 2026-10-22 to 2026-11-08 | Arrival at the approach spot in the saved suite and pipeline tests | 70 log lines, **10 distinct values** | 3.14 to 3.88 mm (mean 3.35 mm over the 70 lines; computed by me from the logged values, not stored in any file) | 0.01 to 0.34 deg (mean 0.17 deg over the 70 lines; same caveat) | Staging sim tests; the follower on in the nav_follow_sim_test logs, and the stance-select tests (follower state per log not checked) |
| A7 | data/logs, handoff lines | 2026-10-22 to 2026-11-08 | Handoff pose error, same logs | 5 distinct values | 3.8 to 4.0 mm | 0.01 to 0.20 deg | As A6 |
| A8 | docs/reports/overnight_measurements_20261109.md | 2026-11-09 | No new arrival measurement. It runs the mock place and pipeline at the stances. Its place-DOWN errors (4.6 to 7.2 mm, 0.5 to 2.1 deg) are arm/pad errors at the place, not base arrival | — | — | — | Not an arrival study |
| A9 | docs/TODO.md line 44 and lines 49-52 | undated (about 2026-10-22) | "Staging drift": base moves about 3 mm / 0.9 deg during PEDESTAL and 6 mm during ARMS before the corrections; handoff error 8.5 mm / 0.36 deg | not found in any saved log or report | 8.5 mm (TODO only) | 0.36 deg (TODO only) | Origin of these figures not found in data/ or docs/reports. Only the TODO carries them |

Not arrival studies, listed because they show up in the search: the N9 lean sweep (experiments/nav_lean_sweep.py: peak tilt only); experiments/arm_hold_drive_test.py (arm swing during a base drive, not base position; TODO line 36 reports arms swinging about 20 cm in fast turns at 2 rad/s and ending about 2 cm off after stopping, with the old combined controller); bench_stance_validate.py and the stance-select tests (they check the stance, not the drive).

## 3. The 70 saved arrival lines, grouped by distinct value

Counts are the number of log lines carrying that value. Scans by me over data/logs; the values are exact as logged.

| Arrival (mm, deg) | Lines | Example logs |
|---|---|---|
| 3.74, 0.01 | 11 | data/logs/suite_20261026_171509_nav_follow_sim_test.log; data/logs/suite_20261106_twopass_nav_follow_sim_test.log |
| 3.80, 0.15 | 2 | data/logs/nav_follow_sim_20261022_130524.log; data/logs/suite_20261022_131836_nav_follow_sim_test.log |
| 3.86, 0.19 | 2 | data/logs/nav_follow_sim_20261022_125601.log; data/logs/nav_follow_sim_20261022_130026.log |
| 3.88, 0.15 | 5 | data/logs/suite_p3_20261025_220800_nav_follow_sim_test.log; data/logs/suite_j2_20261025_060041_nav_follow_sim_test.log |
| 3.14, 0.19 | 6 | data/logs/stance_select_moved_20261025_220439.log; data/logs/stance_select_moved_20261026_022730.log |
| 3.15, 0.19 | 10 | data/logs/suite_j2_20261025_060041_stance_select_pipeline_test.log; data/logs/suite_i2_20261023_201334_stance_select_pipeline_test.log |
| 3.18, 0.19 | 5 | data/logs/guardoff_stance_select_moved_test_20261108.log; data/logs/m275_stance_select_moved_test_20261108_143741.log |
| 3.18, 0.34 | 7 | data/logs/suite_20261106_twopass_stance_select_pipeline_test.log; data/logs/cur2_stance_select_pipeline_test.log |
| 3.19, 0.19 | 20 | data/logs/suite_20261027_171543_stance_select_pipeline_test.log; data/logs/parent_stance_select_pipeline_test.log |
| 3.20, 0.19 | 2 | data/logs/ab_bench_043631_stance_select_moved_test.log; data/logs/suite_20261105_ped016_stance_select_moved_test.log |

What this does and does not show: the same value repeats across many log files, which fits repeated runs of one deterministic setup (the stance-select and nav tests replay the same scene). These are not independent draws of the arrival error. I did not check the random seed or the variation of each log. The only spread across distinct values is what the table shows: position 3.14 to 3.88 mm, heading 0.01 to 0.34 deg.

## 4. Origin of the 1 cm and 1 degree stand-in

- Config: `src/task_config.py` lines 220-222, comment: "Stand-in for the base's stopping error (the real value is a hardware question, design plan §11): the sim's own pose-correction thresholds". Values `stop_error_xy` 0.01 m and `stop_error_yaw_deg` 1.0 deg, row M085.
- Decision row: docs/mpvi_decisions.md line 97, M085, dated 2026-10-22. It is a **backfill** (the row says "BACKFILL (standing rule, design plan 0)"). Its source column reads "David (as recorded in the cited place)". The cited place is design plan §3.1 f, dated 2026-10-16 and 2026-10-19.
- Design plan §3.1 f (docs/design_plan.md line 89): "Margins: erode horizontally by the base's stopping error, plus a 1 cm margin ... Stand-in for the stopping error: 1 cm and 1 deg (the sim's own pose-correction thresholds); the real value is a hardware question (§11)."
- Measurement: none. The value was not taken from a measurement of the Tracer's stopping accuracy. Nothing in data/ or docs/reports measures the real base's stopping error.
- Other uses of the same 1 cm / 1 deg: the full check's error budget (±1 cm x, ±1 cm y, ±1 deg heading, pedestal ±2 cm, design plan §3.4, row M132); `heading_robust_deg` 1.0 (row M086); the staging's `drive_pos_tol` 0.01 m (row M109, marked PENDING, no decision recorded).
- Open question in the design plan: §11 line 424, "Base stopping accuracy, position and heading (Michael)". §12 line 451-455 says the sim gives the base pose as ground truth, and that stance margins must cover the stopping error plus the localization error.

## 5. Tolerance studies for the stance

### 5a. The 2026-10-19 finder run (the source of 7.1 cm / 8 deg / 4.2 cm)

- Report: docs/reports/20261019_2245_finder_widest_range_rank1.md. Earlier context: docs/reports/20261019_1820_stance_requirements_and_rank1_finder.md. Adopted as the staging stance in the design plan §3.4 on 2026-10-20 (DECIDED, David). So the date in your brief (2026-10-20) is the decision date. The tolerance numbers come from 2026-10-19.
- What was measured: the finder's reach tolerance, meaning the distance from the stance to the edge of the valid reach region (the score the finder ranks by), plus the IK verification's tolerances.
- Result (chosen stance x 0.765, y 1.085, heading -90 deg, pedestal 0.155 m): finder tolerance 7.1 cm. IK verification: xy tolerance 7 cm, heading ±8 deg, pedestal ±4.2 cm, joint margin 30 deg, all 12 points solve. Pedestal passing range 0.10 to 0.21 m (11 cm, 1 cm grid). xy region around the stance: 39 of 49 points of a ±6 cm grid (2 cm step) pass everything, largest passing radius 5.7 cm.
- Step sizes and ranges: the tolerance is read off the region score at the grasp-map grid (1 cm cubes; design plan §3.1). The region-score code (src/stance_finder.py `region_scores`) was not read at the step level in this search. The xy grid in the experiment is ±6 cm at 2 cm (experiments/stance_from_map.py). The pedestal passing range is stepped at 1 cm (`STANCE_CFG["pedestal_step"]` is 0.04 m for the searched heights; the bench-stance pedestal grid in `BENCH_STANCE_CFG` is 0.01 m, row M170). The stance search uses headings every 5 deg (row M086).
- Runs: one finder run (the widest-range rule, 23 minutes per the report). The report's comparison table lists the other candidates' tolerances (14.9 cm, 15.0 cm, 0.0 cm, 2.2 cm for the other columns of the 20261019_1820 table).
- "±4.2 cm" for the pedestal is recorded as is. Its derivation is not spelled out in the saved report. The 2026-10-22 profile below gives 4 cm for the same ladder.

### 5b. The 2026-10-22 stance check profile (ladders around the same stance)

- Report: docs/reports/20261022_0344_stance_check_profile.md. Script: experiments/stance_check_profile.py (committed).
- What was walked: the IK verification of stance A (the TASK_CFG stance) with tolerance ladders. Base shift: radii 1 to 8 cm, 8 directions each (64 stances). Heading: 2, 4, 6, 8 deg, both signs (8 stances). Pedestal: half and one step, both signs (4 stances). Joint margin: 10, 14, 18, 22, 26, 30 deg (6 stances). Total 83 stances, 286,848 IK starts.
- Result: every ladder ran to its end, so A passes the widest rung of each ladder ("A's tolerances (7 cm, 8 degrees, 4 cm) are wide"). Those are the ladder limits, not a measured breaking point.

### 5c. The 2026-10-22 full check error budget (queue stage 1)

- Report: docs/reports/20261022_0548_queue_stage1.md. Test: experiments/full_check_test.py, 25/25.
- What was checked: 9 stances, the nominal stance and each error alone: x ±1 cm, y ±1 cm, heading ±1 deg, pedestal ±2 cm. The stance passes all 9.
- Not run: all 81 combinations of the error budget (design plan §9 build order, line 341, M132). The report says this combined check is not done.

### 5d. The 2026-11-09 tolerance walk (place and pick angles)

- Report: docs/reports/middle_vs_tolerant_20261109.md (untracked; this session's earlier work). Saved data: the tolerance file it reads.
- What was walked: for each of 8 directions (x+, x-, y+, y-, heading+, heading-, pedestal+, pedestal-), the largest passing offset: x and y 5 mm to 60 mm, heading 1 deg to 10 deg, pedestal 5 mm to 50 mm, each step reruns the task's path. T = the minimum over the 8 directions as a fraction of its full range.
- Scope: 23 stances, pick and place, 295 entries in the tolerance file. Its arrival multiples use the single 3.8 mm / 0.15 deg value (A1). It is a path-solve tolerance, not a base arrival study.

## 6. Did any study measure arrival with the arms moving or with a beaker held?

- **Arms moving during the drive:** No arrival measurement. The staging order puts the arms at the zombie pose before any driving (design plan §1, the arms rule, 2026-10-19). The DRIVE and CREEP_IN in A1 to A7 all have the arms at home. The arm-swing study (TODO line 36, experiments/arm_hold_drive_test.py) measured arm displacement, not base arrival.
- **Beaker held during the drive:** No study found. The held-beaker sweep (commit daec93b, J2) is a WIP commit and its test (`transit_clearance_test`) is "written, not run". The overnight 200 g and mock runs (A8) start after the drive, at the stance.
- **Creep:** The CREEP_IN phase is inside the saved arrivals (the handoff is measured after it). No separate creep-error study was found. The TODO's "5 mm post-ARMS correction trigger" is not enabled (docs/TODO.md lines 49-52).

## 7. What was not found

- A repeated base arrival study with a spread (standard deviation, percentile or worst case over trials). The saved arrivals are single runs or repeats of one value.
- Any measurement of the real Tracer's stopping accuracy (design plan §11, owner Michael, still open).
- The 8.5 mm / 0.36 deg handoff figure and the 3 mm / 0.9 deg and 6 mm drift figures (TODO only, no log or report).
- Any git commit with a message about repeatability of the base. The grep hits were for the neck (e9d8bd7, f3a31bd) and for a log of repeated commands (44961c8), not the base.
- A pedestal-height arrival or a pedestal stopping error. The design plan notes it is not defined.
- The derivation of the ±4.2 cm pedestal tolerance.

## 8. Commit and push state

- This task made no commits and no pushes. The only new file is this report, untracked.
- HEAD: 543de46c16809421e4fa9d051ca7a2932544a794.
- Pre-existing untracked files (not touched): .claude/, compare_nosag.py, docs/reports/middle_vs_tolerant_20261109.md, extract_height_data.py, measstart, scratch/. The git status also shows deps/AgileX_Tracer as modified; not touched.
