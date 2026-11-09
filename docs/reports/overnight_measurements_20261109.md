# Overnight measurements, 2026-11-09 (for David's decisions; not committed)

Measurements only. No change was made in the working copy, no commit, no push. Base code: HEAD 3e93f73 (the working tree was clean apart from untracked scratch files).

## Setup

- Copies (outside the repo, each with its own data/ holding the maps the run reads):
  - `/home/vboxuser/projects/oc_base`: unchanged code (Part A variant 1, Part B "no reseed", Part C, Part D "as now").
  - `/home/vboxuser/projects/oc_a2`: `approach_angle.choose_approach_angle`: picks first passing; places the middle of the widest run of passing turns (variant 2).
  - `/home/vboxuser/projects/oc_a3`: same; places the passing turn nearest the middle of the widest run, walking outward (ties: lower turn) (variant 3).
  - `/home/vboxuser/projects/oc_breseed`: `FullCheck.check` restarts the random generator (seed 11) at every call (Part B "reseed").
  - `/home/vboxuser/projects/oc_d1`: `pedestal_override_m` 0.16 -> 0.12 (park at 0.12; the check already runs at the candidate's 0.12).
  - `/home/vboxuser/projects/oc_d2`: the full check of each candidate runs at 0.16 (`stance_select.py` line 585), the park is 0.16 (unchanged).
- Scripts: `/home/vboxuser/projects/oc_scripts/` (`ov_lib.py`, `run51.py`, `part_c.py`, `analyze_B.py`, `chain_AB.sh`, `chain_CD.sh`).
- Logs and results: `data/logs/overnight_20261109/` in the working copy (git-ignored); progress file `data/logs/overnight_20261109/progress.txt`.
- One MuJoCo process at a time: Parts A and B in one tmux chain (`ov_AB`), then Parts C and D in a second chain (`ov_CD`) that waits for the first.
- Timing sample: one stance check 34 to 44 s (passing stances); the 51-stance pass is about 30 min.

## Status

All parts ran to completion (see Coverage). Measurements only.

## Part A: place angle choice (copies oc_base, oc_a2, oc_a3)

Place list at the staging stance (same in all three runs): -55, -50, ..., -5 (11 turns). Chosen place turn, derived from the list and the choice function (the summary does not store the chosen value): base (first passing) -55 deg; oc_a2 (middle of the widest run) -30 deg; oc_a3 (nearest the middle) -30 deg.

| variant | mpvi_pipeline_test | place turn | first failing step (pipeline) | mock 200 g outcome | mock: place OVER / DOWN / OPEN / BACK end error (mm, position) |
|---|---|---|---|---|---|
| 1 base (first passing) | 6/14 (exit 1) | -55 | DOWN: 7.2 mm, 2.1 deg (attempt 1); retry recovery then HOME: largest joint error 0.267 rad | PLAN_FAILED at place OVER after 15.0 s: 3.4 mm, 0.2 deg (t=101.0 s) | OVER 3.4 (failed); not reached DOWN/OPEN/BACK |
| 2 window middle | 10/14 (exit 1) | -30 | HOME: did not settle after 9.0 s, largest joint error 0.267 rad | SUCCESS | OVER 1.2 / DOWN 1.4 / BACK 1.0 |
| 3 nearest middle outward | 10/14 (exit 1) | -30 | HOME: same as variant 2 | SUCCESS | OVER 1.2 / DOWN 1.4 / BACK 1.0 |

Notes: degrees were not logged for steps that passed (only the failure messages carry degrees), so the degree columns are not filled for passing steps. The mock errors are the pad-to-plan position at the last row of each move (probe CSV), not the controller's own reached check. The pipeline test's own check of the place angle output is in data/logs/overnight_20261109/A_pipeline_oc_*.log.

## Part B: order independence (51 stances, copies oc_base and oc_breseed)

Given order = the spreadsheet order (49 candidates, staging, point 4). Reverse = the same list reversed. Each run is a fresh process. Full per-stance table: `analyze_B` output in `data/logs/overnight_20261109/B_*.jsonl` (one line per stance).

| run | PASS of 51 |
|---|---|
| given, no reseed (current code) | 23 |
| reverse, no reseed (current code) | 22 |
| given, reseed per check | 21 |
| reverse, reseed per check | 21 |

Flips (pass in one order, fail in the other):
- Without reseeding: 3 stances flip (given vs reverse): candidate (0.604, 1.001, -60 deg, pedestal 0.10): PASS given, FAIL reverse; candidate (0.595, 1.002, -60 deg, 0.10): PASS given, FAIL reverse; candidate (0.665, 0.984, -70 deg, 0.20): FAIL given, PASS reverse.
- With reseeding: 0 flips.

Reseeding also changes some results against the fresh-process no-reseed runs (23 vs 21 in the given order): the reseeded generator is a different draw, not the one the batch used.

## Part C: comfort of each passing approach angle (oc_base, the current code)

Stances: the 21 passing candidates, staging and point 4 (23 stances, 46 stance-tasks, 295 angle entries). All 295 entries reproduced the check's path (seeded solve). Full table below and in `data/logs/overnight_20261109/analyze_C.txt`.

Measures (per entry, minimum over the solved path):
1. smin of the position part (3x7), smin of the orientation part (3x7), and Yoshikawa sqrt(det(J J^T)) of the 6x7 hand Jacobian at the pad point (`batch_fk_jac`, from the solved configurations of the nominal stance).
2. Hardware margin: smallest distance from any arm joint to its model range limit (degrees), with the joint.
3. Tolerance T: for each of the 8 directions (x, y, pedestal ±, heading ±), the largest passing offset (the walk stops at the first failing step: x/y 5 mm to 60 mm, heading 1 deg to 10 deg, pedestal 5 mm to 50 mm), divided by its full range; T = the minimum of the 8. Higher = more tolerant. Each offset reruns the task's whole path (the pick path re-expanded for the offset heading).

Pick angles are bin indices (x 30 deg). Place angles are degrees.

Per stance and task (best angle per measure; window middle; distance from the middle to each best in degrees, in the order pos / rot / Yoshikawa / margin / T):

| stance (x, y, heading, ped) | task | passing angles | best smin pos | best smin rot | best Yoshikawa | best margin | best T | window middle | middle to best (deg) pos/rot/yo/margin/T |
|---|---|---|---|---|---|---|---|---|---|
| 0.870,1.072,-100,0.10 candidate | 3 place | 11 | -60.0 | -10.0 | -30.0 | -40.0 | -50.0 | -35.0 | 25.0/25.0/5.0/5.0/15.0 |
| 0.842,1.077,-100,0.10 candidate | 1 pick | 1 | 11 | 11 | 11 | 11 | 11 | 11.0 | 0.0/0.0/0.0/0.0/0.0 |
| 0.842,1.077,-100,0.10 candidate | 3 place | 12 | -60.0 | -10.0 | -30.0 | -35.0 | -50.0 | -32.5 | 27.5/22.5/2.5/2.5/17.5 |
| 0.806,1.072,-90,0.10 candidate | 1 pick | 1 | 11 | 11 | 11 | 11 | 11 | 11.0 | 0.0/0.0/0.0/0.0/0.0 |
| 0.806,1.072,-90,0.10 candidate | 3 place | 11 | -60.0 | -10.0 | -35.0 | -40.0 | -50.0 | -35.0 | 25.0/25.0/0.0/5.0/15.0 |
| 0.793,1.074,-90,0.10 candidate | 1 pick | 1 | 11 | 11 | 11 | 11 | 11 | 11.0 | 0.0/0.0/0.0/0.0/0.0 |
| 0.793,1.074,-90,0.10 candidate | 3 place | 12 | -60.0 | -10.0 | -30.0 | -40.0 | -50.0 | -32.5 | 27.5/22.5/2.5/7.5/17.5 |
| 0.781,1.076,-90,0.10 candidate | 1 pick | 1 | 11 | 11 | 11 | 11 | 11 | 11.0 | 0.0/0.0/0.0/0.0/0.0 |
| 0.781,1.076,-90,0.10 candidate | 3 place | 12 | -60.0 | -10.0 | -30.0 | -40.0 | -50.0 | -32.5 | 27.5/22.5/2.5/7.5/17.5 |
| 0.781,1.051,-90,0.15 candidate | 1 pick | 2 | 11 | 0 | 11 | 11 | 11 | 0.0 | 30.0/0.0/30.0/30.0/30.0 |
| 0.781,1.051,-90,0.15 candidate | 3 place | 12 | -60.0 | -10.0 | -25.0 | -40.0 | -50.0 | -32.5 | 27.5/22.5/7.5/7.5/17.5 |
| 0.748,1.032,-80,0.15 candidate | 1 pick | 2 | 0 | 11 | 11 | 11 | 11 | 0.0 | 0.0/30.0/30.0/30.0/30.0 |
| 0.748,1.032,-80,0.15 candidate | 3 place | 10 | -65.0 | -20.0 | -30.0 | -40.0 | -50.0 | -42.5 | 22.5/22.5/12.5/2.5/7.5 |
| 0.773,1.053,-90,0.15 candidate | 1 pick | 2 | 11 | 0 | 11 | 11 | 11 | 0.0 | 30.0/0.0/30.0/30.0/30.0 |
| 0.773,1.053,-90,0.15 candidate | 3 place | 12 | -60.0 | -5.0 | -25.0 | -35.0 | -50.0 | -32.5 | 27.5/27.5/7.5/2.5/17.5 |
| 0.822,1.069,-90,0.10 candidate | 1 pick | 1 | 11 | 11 | 11 | 11 | 11 | 11.0 | 0.0/0.0/0.0/0.0/0.0 |
| 0.822,1.069,-90,0.10 candidate | 3 place | 10 | -65.0 | -20.0 | -35.0 | -45.0 | -50.0 | -42.5 | 22.5/22.5/7.5/2.5/7.5 |
| 0.731,1.061,-80,0.10 candidate | 1 pick | 2 | 11 | 11 | 11 | 11 | 11 | 0.0 | 30.0/30.0/30.0/30.0/30.0 |
| 0.731,1.061,-80,0.10 candidate | 3 place | 11 | -60.0 | -10.0 | -35.0 | -40.0 | -50.0 | -35.0 | 25.0/25.0/0.0/5.0/15.0 |
| 0.718,1.063,-80,0.10 candidate | 1 pick | 1 | 11 | 11 | 11 | 11 | 11 | 11.0 | 0.0/0.0/0.0/0.0/0.0 |
| 0.718,1.063,-80,0.10 candidate | 3 place | 12 | -60.0 | -10.0 | -30.0 | -40.0 | -50.0 | -32.5 | 27.5/22.5/2.5/7.5/17.5 |
| 0.710,1.064,-80,0.10 candidate | 1 pick | 1 | 11 | 11 | 11 | 11 | 11 | 11.0 | 0.0/0.0/0.0/0.0/0.0 |
| 0.710,1.064,-80,0.10 candidate | 3 place | 12 | -60.0 | -10.0 | -30.0 | -35.0 | -50.0 | -32.5 | 27.5/22.5/2.5/2.5/17.5 |
| 0.671,1.012,-70,0.15 candidate | 1 pick | 2 | 11 | 11 | 11 | 11 | 11 | 0.0 | 30.0/30.0/30.0/30.0/30.0 |
| 0.671,1.012,-70,0.15 candidate | 3 place | 11 | -60.0 | -10.0 | -30.0 | -40.0 | -50.0 | -35.0 | 25.0/25.0/5.0/5.0/15.0 |
| 0.658,1.038,-70,0.10 candidate | 1 pick | 2 | 11 | 11 | 11 | 11 | 11 | 0.0 | 30.0/30.0/30.0/30.0/30.0 |
| 0.658,1.038,-70,0.10 candidate | 3 place | 12 | -60.0 | -10.0 | -35.0 | -40.0 | -50.0 | -32.5 | 27.5/22.5/2.5/7.5/17.5 |
| 0.654,1.038,-70,0.10 candidate | 1 pick | 2 | 11 | 11 | 11 | 11 | 11 | 0.0 | 30.0/30.0/30.0/30.0/30.0 |
| 0.654,1.038,-70,0.10 candidate | 3 place | 12 | -60.0 | -10.0 | -35.0 | -40.0 | -50.0 | -32.5 | 27.5/22.5/2.5/7.5/17.5 |
| 0.613,1.000,-60,0.10 candidate | 1 pick | 1 | 11 | 11 | 11 | 11 | 11 | 11.0 | 0.0/0.0/0.0/0.0/0.0 |
| 0.613,1.000,-60,0.10 candidate | 3 place | 11 | -60.0 | -10.0 | -35.0 | -40.0 | -50.0 | -35.0 | 25.0/25.0/0.0/5.0/15.0 |
| 0.658,1.015,-70,0.15 candidate | 1 pick | 1 | 11 | 11 | 11 | 11 | 11 | 11.0 | 0.0/0.0/0.0/0.0/0.0 |
| 0.658,1.015,-70,0.15 candidate | 3 place | 12 | -60.0 | -10.0 | -25.0 | -40.0 | -50.0 | -32.5 | 27.5/22.5/7.5/7.5/17.5 |
| 0.604,1.001,-60,0.10 candidate | 1 pick | 1 | 11 | 11 | 11 | 11 | 11 | 11.0 | 0.0/0.0/0.0/0.0/0.0 |
| 0.604,1.001,-60,0.10 candidate | 3 place | 11 | -60.0 | -10.0 | -35.0 | -40.0 | -50.0 | -35.0 | 25.0/25.0/0.0/5.0/15.0 |
| 0.645,1.018,-70,0.15 candidate | 1 pick | 1 | 11 | 11 | 11 | 11 | 11 | 11.0 | 0.0/0.0/0.0/0.0/0.0 |
| 0.645,1.018,-70,0.15 candidate | 3 place | 12 | -60.0 | -5.0 | -25.0 | -35.0 | -50.0 | -32.5 | 27.5/27.5/7.5/2.5/17.5 |
| 0.595,1.002,-60,0.10 candidate | 1 pick | 1 | 11 | 11 | 11 | 11 | 11 | 11.0 | 0.0/0.0/0.0/0.0/0.0 |
| 0.595,1.002,-60,0.10 candidate | 3 place | 12 | -60.0 | -10.0 | -35.0 | -40.0 | -50.0 | -32.5 | 27.5/22.5/2.5/7.5/17.5 |
| 0.591,1.003,-60,0.10 candidate | 1 pick | 1 | 11 | 11 | 11 | 11 | 11 | 11.0 | 0.0/0.0/0.0/0.0/0.0 |
| 0.591,1.003,-60,0.10 candidate | 3 place | 12 | -60.0 | -5.0 | -35.0 | -40.0 | -50.0 | -32.5 | 27.5/27.5/2.5/7.5/17.5 |
| 0.775,1.105,-90,0.12 staging | 1 pick | 2 | 0 | 11 | 11 | 11 | 0 | 0.0 | 0.0/30.0/30.0/30.0/0.0 |
| 0.775,1.105,-90,0.12 staging | 3 place | 11 | -55.0 | -10.0 | -25.0 | -35.0 | -45.0 | -30.0 | 25.0/20.0/5.0/5.0/15.0 |
| 0.870,1.072,-100,0.10 point 4 | 1 pick | 1 | 11 | 11 | 11 | 11 | 11 | 11.0 | 0.0/0.0/0.0/0.0/0.0 |
| 0.870,1.072,-100,0.10 point 4 | 3 place | 11 | -60.0 | -10.0 | -30.0 | -40.0 | -55.0 | -35.0 | 25.0/25.0/5.0/5.0/20.0 |

Correlations (Spearman, n = 295 entries):
- measure 1, smin position vs margin: rho -0.138
- measure 1, smin orientation vs margin: rho 0.104
- measure 1, Yoshikawa vs margin: rho 0.697
- measure 3, tolerance T vs margin: rho 0.706

Margin under 15 degrees with measures 1 and 3 at or above the median: 0 cases. (Of 295 entries, 156 have a margin under 15 degrees.)

Limits of this part: the pick window middle for a two-bin run that wraps past bin 11 (bins 0 and 11) is reported as bin 0, not 11.5, because the middle picks the first of equal-length runs. The pick distance columns are in degrees (30 per bin). Smin and Yoshikawa units: position part m per rad, orientation part per rad, Yoshikawa dimensionless per rad. The tolerance walk is the path solve only (the body, view and occlusion checks are not rerun).

## Part D: pedestal mismatch at the staging stance (copies oc_d1, oc_d2; "as now" = oc_base Part A run)

| case | check pedestal | park pedestal | mpvi_pipeline_test | first failing step | later failure |
|---|---|---|---|---|---|
| 1 both at 0.12 | 0.12 | 0.12 | 6/14 | DOWN: 4.6 mm, 0.5 deg (attempt 1) | HOME: largest joint error 0.266 rad |
| 2 both at 0.16 | 0.16 | 0.16 | 6/14 | DOWN: 7.2 mm, 2.1 deg (attempt 1) | HOME: largest joint error 0.267 rad |
| 3 as now | 0.12 | 0.16 | 6/14 | DOWN: 7.2 mm, 2.1 deg (attempt 1) | HOME: largest joint error 0.267 rad |

Place turn in all three is the first passing (-55 deg). In case 2 every candidate's check runs at 0.16 (the copy's change).

## Coverage

- Part A: all 3 variants, pipeline test and mock 200 g.
- Part B: all 4 runs, all 51 stances.
- Part C: measures 1 and 2 for all 295 entries; tolerance (measure 3) for all 295 entries (all 23 stances, both tasks, every passing angle). Nothing was cut.
- Part D: all 3 cases (case 3 is the existing base run).

## Not done

- Nothing in the listed parts was left out. The run took about 8 hours after the chain setup.
- The place angle is derived from the list and the choice function, not read from a log, because the choice is not logged (degrees for passing place steps are not logged either). Reason: no logging change allowed in the copies.
- Pick window middle for a wrapping run (bins 0 and 11): reported as bin 0, not 11.5 (see Part C limits).

## Decisions for David (from these measurements)

1. Place angle: the window middle (or nearest middle) gives mock success and 10 of 14 pipeline checks; first passing gives 6 of 14 and the mock fails at place OVER. The choice is David's.
2. Random generator: reseeding per check removes all flips (0 vs 3 without). With reseeding the 51-stance count is 21; without it 22 or 23 depending on order.
3. Pedestal at staging: the park height does not change the pipeline result (6/14 in all three); the first failing step is DOWN in all cases. The check height (0.12 or 0.16) matters only for which candidate passes.
4. The place DOWN failure (4.6 to 7.2 mm) and the HOME failure after it are the same in all three cases. The fix is not in these copies.
