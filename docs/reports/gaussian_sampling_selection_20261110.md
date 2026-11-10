# Stance selection by weighted random sampling from the A8 model (2026-11-10)

Written for: David. Decision M299 (`docs/mpvi_decisions.md`, with Revisions 1 to 3). The pipeline's stance search is now the sampler; the overlay is still in the repo, not called.

## Where things stand

- The sampler is built as **Revision 3** (polar draw around the task midpoint, inner limit from the reach maps, outer limit from A8, heading within 45° of facing the objects). Unit tests: 26 of 26 pass.
- **No end-to-end run has been done with Revision 3** (David's instruction: he will decide on the run after the home_clearance pictures). This is listed under NOT DONE.
- Two end-to-end runs were done with earlier versions (rectangle draw; then Revision 2 with the held heading). **Both stopped at stance selection**: all 10 drawn stances failed the full check.
- The pipeline tests `stance_select_moved_test`, `mpvi_pipeline_test` and `stance_select_pipeline_test` hit the 5-minute cap (not finished).

## Commits (local, not pushed)

| hash | message |
|---|---|
| 885926a | Place angle: the robot uses the middle of the window the check chose (M299) |
| bef613e | Stance selection by weighted random sampling from the A8 model (M299) |
| 21d2105 | Decision M299: Gaussian sampling stance selection; design plan and suite list (M299) |
| 4ed28ff | Sampler heading rule (Revision 2, M299); the step context gets the bench's near edge |
| eb4a913 | Decision M299: heading rule (Revision 2) |
| 090a33c | Sampler Revision 3: polar draw around the task midpoint, inner and outer reach limits (M299) |
| 72ae86d | Decision M299 Revision 3: polar draw around the task midpoint |
| 9511363 | Report: what fails home_clearance, with pictures (read-only) |

Decision row: **M299** (Revisions 1 to 3 in the row).

## What the sampler does (Revision 3), in order

Module `src/stance_sampling.py`; wired by `src/stance_select.py` `apply_stance_selection` (which calls only `select_stance`).

1. **Centre**: the 3D midpoint of the task points (`task_points`: each object's grasp points for a pick, set-down points for a place; their mean height).
2. **Batch** (`draw_batch`), one seed, attempts in order. Each attempt:
   - Pedestal q uniform in `pedestal_range` (0.05 to 0.25 m).
   - dz = left shoulder height at q minus the centre's height. If r_out ≤ |dz| the pedestal is drawn again (counted as a pedestal redraw).
   - d_min = √max(r_in² − dz², 0), d_max = √(r_out² − dz²); d = √(U(d_max² − d_min²) + d_min²), U uniform: equal floor areas equally likely.
   - Direction φ uniform in 0–360°. The shoulder's floor position = centre + d(cos φ, sin φ).
   - Heading = facing direction (shoulder floor position → centre) + uniform offset in ±45° (`sampler_heading_half_width_deg`). While `sampler_heading_held` is True the heading is held at TASK_CFG `staging_yaw` (−90°) instead.
   - Base origin = shoulder floor position − the shoulder's horizontal offset turned by the heading. The offset (torso frame) is (0.000, 0.031) m for the left arm.
   - Inner check: if either task point is closer than r_in to the shoulder (3D), the stance is dropped and drawn again (counted as a drop).
   - Attempts are capped at `sampler_redraw_factor` × N; if the cap is hit, the stances kept are the batch.
3. **Score** (`a8_scores`): the shoulder from the stance (base + offset turned by the heading, height from the pedestal); for each task point, the A8 count (`reach_model.predict_count_asym`, z cutoff −0.375 m). The stance's score is the **lower** of its points' counts. Checked against `reach_model.counts_at` (4e-16).
4. **Weight** = score² (`sampler_weight_power`). If the weights sum to zero, the selection stops with a reason.
5. **Draws** (`select_stance`): one stance drawn with probability ∝ weight among the stances not yet drawn (a failed stance is not put back), up to `sampler_max_draws` = 10 full checks. The full check gets the stance's floats unchanged. Stops at the first pass.
6. **Seed**: from the clock (`time.time_ns()`), saved as `seed` in the selection record. The batch and the draws replay from it.
7. **Result**: `task_cfg` gets the chosen stance's x, y, heading (as the staging yaw) and pedestal (M284: the robot parks at the checked pedestal).

Arm used for scoring: **left** (the task uses the left arm). The right arm would use the mirrored model (`reach_model.arm_params`); not exercised.

### Settings (`BENCH_STANCE_CFG`, `src/task_config.py`)

| key | value | plain words |
|---|---|---|
| `sampler_batch` | 10000 | stances kept in one selection |
| `sampler_weight_power` | 2 | weight = A8 score to this power (2: score squared) |
| `sampler_max_draws` | 10 | the most full checks in one selection; a stance that fails is not drawn again |
| `sampler_r_in_m` | 0.0497 | **inner limit**: smallest 3D shoulder-to-cell distance of the filtered reach maps (`data/reach_map_before_bench`, left arm, all 12 bins, measured 2026-11-10). See the histogram below |
| `sampler_outer_sigmas` | 2.0 | **outer limit** r_out = A8 r0 + 2 sr = 0.4527 + 2 × 0.2186 = **0.8899 m** |
| `sampler_redraw_factor` | 20 | attempts capped at 20 × the batch (pedestal and inner-limit redraws included) |
| `sampler_heading_half_width_deg` | 45.0 | heading drawn within this many degrees of facing the objects from the shoulder |
| `sampler_heading_held` | True | heading held at −90° (the staging is not checked at other headings; see Heading) |
| `arm` | "left" | the arm used for scoring (existing key) |
| `pedestal_range` | (0.05, 0.25) | existing key, the pedestal range |

Removed: the Revision 1 rectangle margin (`sampler_margin_sigmas`). The Revision 1 fixed heading range was never built.

### Inner reach limit r_in (measured 2026-11-10)

From `data/reach_map_before_bench`, left arm, all 12 bins, cell centres in the shoulder frame (`scratch/polar_draw/inner_limit.py`; full output `docs/reports/home_clearance_images_20261110/r_in_histogram.json`):
- **r_in = 0.0497 m**: one cell at (0.025, −0.035, 0.025) m. 35 cells lie under 0.10 m.
- Quantiles: 0.01 % = 0.0513 m, 0.1 % = 0.0585 m, 1 % = 0.0923 m.
- Reachable cells by 5 cm distance from the shoulder (635093 cells): 0.00–0.05: 1; 0.05–0.10: 696; 0.10–0.15: 4775; 0.15–0.20: 12790; 0.20–0.25: 24853; 0.25–0.30: 37891; 0.30–0.35: 52351; 0.35–0.40: 66785; 0.40–0.45: 79137; 0.45–0.50: 81733; 0.50–0.55: 82483; 0.55–0.60: 76765; 0.60–0.65: 62732; 0.65–0.70: 39898; 0.70–0.75: 12203.

## Batch for the Revision 3 draw (seed 20261110; no full check run)

`scratch/polar_draw/batch_stats.py` (stats in `docs/reports/home_clearance_images_20261110/revision3_batch_stats.json`):

| item | value |
|---|---|
| batch | 10000 stances kept in 10000 attempts |
| drops (task point inside r_in) | 0 |
| pedestal redraws (r_out ≤ \|dz\|) | 0 |
| cap hit | no |
| shoulder offset used | (0.000, 0.031) m, torso frame |
| dz range | 0.158 to 0.358 m |
| score min / median / max | 0.000 / 0.171 / 3.733 |
| fraction scoring zero | 0.096 |
| base origin over the bench top | **0.607** (0.645 within 5 cm of it) |

Top five by weight (no checks):

| x | y | heading (°) | pedestal | score | base distance to bench top (m; negative: over it) |
|---|---|---|---|---|---|
| 1.0100 | 0.2681 | 58.4 | 0.0984 | 3.733 | −0.532 |
| 0.9531 | 0.9988 | −118.3 | 0.1849 | 3.730 | +0.199 |
| 1.0248 | 0.2548 | 68.1 | 0.0753 | 3.726 | −0.545 |
| 0.8986 | 1.0918 | −125.0 | 0.0568 | 3.722 | +0.292 |
| 0.9676 | 1.0611 | −115.5 | 0.0646 | 3.720 | +0.261 |

Most of the high-weight stances put the base over the bench top, the case that failed home_clearance in the earlier runs (see `docs/reports/home_clearance_images_20261110.md`).

## Body-check gate at draw time (addition, 2026-11-10)

- After a weighted draw and before the full check, the sampler runs the full check's own body check on the stance (`FullCheck.clear.body_distance`, the same robot pose, the same limit `FULLCHECK_CFG["body_clearance"]` = 0 m). A stance that fails is discarded (not put back) and does not count toward the 10 full-check draws. More than `sampler_gate_max_failures` (500) gate failures stop the selection and report it. The full check keeps its own body check. Bases under the bench top stay allowed (no footprint filter). Commit `7d63150`.
- Time per body check (`scratch/polar_draw/gate_stats.py`, 200 timed stances from the seed-20261110 batch): **mean 37 ms, max 73 ms**.
- Gate failures over 2000 drawn stances: **1322 (66 %)**. Colliding pairs: `openarm_body_link0_collision vs bench_top` 1276; `base_link_geom vs bench_leg_fr` 29; `battery_geom vs stool_base_4` 9; finger vs `task_hotplate_top_plate` 2 each; a few others (1 each). Minimum distance over the 2000: −0.435 m. Data: `docs/reports/home_clearance_images_20261110/gate_stats.json`.
- Selection-only demo at the staging layout (`scratch/polar_draw/select_demo.py`; NOT an episode): seed **381989416**, batch 10000 stances in 10000 attempts (0 dropped inside r_in, 0 pedestal redraws). Score min 0.000, median 0.015, max 3.611, zero fraction 0.083. **Gate failures: 16** (15 `openarm_body_link0_collision vs bench_top`, 1 finger vs `task_hotplate_top_plate`). Full checks: 10 draws, all failed; selection stopped: "all 10 draws failed the full check". Heading held at −90° (the config default).

| draw | x | y | heading (°) | pedestal | score | weight | full check | reason |
|---|---|---|---|---|---|---|---|---|
| 1 | 0.8909 | 1.2108 | −90 | 0.1203 | 1.739 | 3.03 | FAIL | ik |
| 2 | 0.6420 | 1.0181 | −90 | 0.1491 | 2.854 | 8.15 | FAIL | placed_beaker |
| 3 | 0.8321 | 0.8874 | −90 | 0.2208 | 2.239 | 5.01 | FAIL | placed_beaker |
| 4 | 0.5910 | 1.1467 | −90 | 0.1014 | 1.481 | 2.19 | FAIL | ik |
| 5 | 0.6388 | 0.9608 | −90 | 0.1528 | 3.234 | 10.46 | FAIL | placed_beaker |
| 6 | 0.8318 | 1.0847 | −90 | 0.0723 | 3.137 | 9.84 | FAIL | home_clearance |
| 7 | 0.5782 | 0.9289 | −90 | 0.1020 | 3.254 | 10.59 | FAIL | home_clearance |
| 8 | 0.5597 | 0.9051 | −90 | 0.1517 | 2.926 | 8.56 | FAIL | placed_beaker |
| 9 | 0.8641 | 1.3458 | −90 | 0.0582 | 0.816 | 0.67 | FAIL | ik |
| 10 | 0.8901 | 0.8900 | −90 | 0.2066 | 0.916 | 0.84 | FAIL | placed_beaker |

Check times per full check (from the record): 0.0–2.3 s each. Approach angles: none chosen (no stance passed). Picture of the last stance tried (draw 10; selection failed): `docs/reports/home_clearance_images_20261110/selection_demo_top.png` (top view, same size and camera as the home_clearance pictures; the robot is drawn from the check's own data, so the scene's shading differs a little from the earlier pictures).

Reading: the gate removes the stances whose base is over the bench top (the body_link0 vs bench_top contacts), but the stances that pass the gate still fail the full check on `placed_beaker`, `ik` and `home_clearance`. The gate does not fix the selection by itself.

## Heading

- Revision 2's rule (facing the objects within ±45°) is in the code. It is **held** at −90° by default (`sampler_heading_held` True).
- Reason for holding (the check the brief asks for): the staging's DRIVE is a straight line from where the base starts to the goal, with clearance checks tuned to the current layout, and no path planning exists. The staging code turns to any `staging_yaw` (no literal −90° there), but a drawn heading would drive the base through the bench for many stances. So holding is the safe choice until the staging and navigation are tested at other headings.
- Places that assume −90°: `src/task_config.py:85` (`staging_yaw`, the held value), `:35` and `:74` (documentation text), `src/stance_sampling.py` (module docstring) and `src/stance_select.py` `apply_stance_selection` (`yaw = float(task_cfg["staging_yaw"])`, the held heading). Also `src/stance_select.py:74` (`StanceContext.yaw`, old overlay path, not called) and `src/world_model.py:41`, `src/mpvi_modules.py:111` (stance record, follow staging_yaw).
- Not handled with a drawn heading: `step_target` (`src/stance_select.py`) computes the place's back-off direction from the context yaw (the held yaw). With a drawn heading that direction would not match the stance. Only matters when the heading is not held.

## Arm and place-angle rule

- Place angle: the robot takes the middle of the chosen window (`choose_place_window` returns `window_turns`; the robot uses `choose_place_angle`). Even window: the counter-clockwise (later) middle turn. Checked by unit tests (odd, even tie, wrap past 0).
- Pick angle: unchanged (first passing).

## Unit tests (`experiments/gaussian_sampling_test.py`; added to `experiments/run_tests.sh`)

26 of 26 checks pass (exit 0, 1 s): the A8 score at hand-computed points; the vector score equals `counts_at`; min over points; weighting (positive-score draws; the cap path for a centre 7 m above the shoulder); replay; the 10-draw cap; the place-angle middle rule; the polar draw (every stance's shoulder-to-centre distance in [r_in, r_out], task points ≥ r_in away, headings within 45° of facing, the base places the shoulder, equal area: fraction below the midpoint 0.497, replay).

## Existing stance-selection tests (after M299, Revision 3 code)

Each run with `timeout 300`, one at a time. "Before" = the parent suite (`b62488a`) unless noted.

| test | before | after | secs | note |
|---|---|---|---|---|
| gaussian_sampling_test | (new) | 26/26 pass, exit 0 | 1 | new |
| approach_angle_list_test | exit 0, 14/14 | exit 0, 14/14 | 0 | unchanged |
| place_theta_logic_test | exit 0 | exit 0 | 40 | unchanged |
| bench_stance_test | exit 0 | exit 0, 13/13 | 123 | the overlay's own test |
| full_check_test | exit 1, 30/31 | exit 1, 30/31 | 130 | the same check fails as before: "a candidate that fails the cheap checks fails fast with the reason" (arms at home 29 mm from openarm_left_hand_collision at pedestal 0.120) |
| stance_select_test | exit 1 (EGL crash) | exit 124 at 300 s | 300 | run before the `near` fix; not finished |
| stance_select_pipeline_test | exit 1, 3/7 | exit 124 at 301 s | 301 | over the 5-minute cap: not finished (the sampler's draws take longer than 5 min) |
| stance_select_moved_test | exit 1, 5/9 | exit 124 at 301 s | 301 | over the cap: not finished |
| mpvi_pipeline_test | exit 1, 6/14 | exit 124 at 300 s | 300 | over the cap: not finished |

No test failed because it assumed the overlay's numbers. The pipeline-based tests need a stance that passes the full check; with the sampler's draws they do not finish within 5 minutes (the first run showed all 10 draws failing).

## End-to-end runs (200 g, staging layout), with earlier versions

Command: `scratch/e2e_pedestal/run_staging.py` (the `demo_video` pipeline with its layout name set to `TASK_CFG`), `python -u`, under `run_capped.sh`.

**Run A** (rectangle draw, before Revision 2). Log `data/logs/e2e_gauss_20261110_024154.log`. Seed 2938876846. Score min 0.000, median 0.000, max 3.574, zero fraction 0.084.

| draw | x | y | pedestal | score | failing check |
|---|---|---|---|---|---|
| 1 | 0.7564 | 0.2214 | 0.1396 | 2.888 | home_clearance |
| 2 | 0.7245 | 0.3021 | 0.1691 | 3.250 | home_clearance |
| 3 | 0.4965 | 0.6935 | 0.1328 | 2.905 | home_clearance |
| 4 | 0.5419 | 0.6670 | 0.2192 | 2.936 | placed_beaker |
| 5 | 0.6512 | 0.5398 | 0.1695 | 2.222 | home_clearance |
| 6 | 0.6052 | 0.4812 | 0.0873 | 2.155 | home_clearance |
| 7 | 0.7503 | 0.3126 | 0.0894 | 3.378 | home_clearance |
| 8 | 0.6551 | 0.6096 | 0.1446 | 1.590 | home_clearance |
| 9 | 0.5605 | 0.9335 | 0.2242 | 2.173 | placed_beaker |
| 10 | 0.4414 | 0.6883 | 0.1571 | 2.260 | body |

Result: all 10 failed; `StanceSelectionError: stance sampler: all 10 draws failed the full check`. The robot did not move.

**Run B** (Revision 2 heading rule, heading held at −90°). Log `data/logs/e2e_gauss3_20261110_025000.log`. Seed 3623316766. Score min 0.000, median 0.000, max 3.559, zero fraction 0.087.

| draw | x | y | pedestal | score | failing check |
|---|---|---|---|---|---|
| 1 | 0.7485 | 0.3786 | 0.1241 | 2.915 | home_clearance |
| 2 | 0.7489 | 0.9041 | 0.1427 | 2.675 | placed_beaker |
| 3 | 0.7337 | 0.2486 | 0.1315 | 3.090 | home_clearance |
| 4 | 0.6412 | 0.2306 | 0.0758 | 2.618 | home_clearance |
| 5 | 0.5106 | 0.8578 | 0.1948 | 2.331 | ik |
| 6 | 0.8798 | 1.0484 | 0.1726 | 2.320 | occlusion |
| 7 | 0.4743 | 0.7053 | 0.0506 | 2.624 | home_clearance |
| 8 | 0.5686 | 0.9078 | 0.0608 | 2.939 | home_clearance |
| 9 | 0.7049 | 0.3217 | 0.1999 | 3.071 | home_clearance |
| 10 | 0.3388 | 0.7804 | 0.1367 | 1.039 | body |

Result: all 10 failed (home_clearance ×6, placed_beaker, ik, occlusion, body); the robot did not move. Draws 3 and 9 (the two highest-scoring home_clearance failures) were analysed with pictures and numbers in `docs/reports/home_clearance_images_20261110.md`: both have the base over the bench top, with the arm 44 to 47 mm inside the microscope.

Other partial output: `data/e2e_gauss2_20261110_024923` (a run killed at start, when it overlapped a test; no results).

Run times: about 4 min 52 s for run A (including setup); per-draw check times were not logged.

## Old search code left in the repo, not called by the pipeline

- `src/stance_select.py`: `select_bench_stance` (436), `_two_pass` (358), `_overlay` (655), `_best_stance` (662), `make_context` (115), `StanceContext` (51), the old run loop.
- `src/bench_stance.py`: `overlay` (669), `get_region` (239), `build_region` (163).
- `src/stance_finder.py`: `find_stances` (286), `find_stances_windowed` (368). Still used by experiments (`full_check_test`, `stance_selection_timing`, `stance_from_map`).
- `src/reach_model.py`: `inverse_search` (246; used by `experiments/reach_inverse_scene.py`), `single_object_stance` (219), `ideal_spot` (132), `inverse_stances` (153, no callers), `_scan` (201, no callers).

## NOT DONE

1. **End-to-end run with the Revision 3 polar draw** (per David: he will decide after the home_clearance pictures). Listed here, not run.
2. Full checks of the Revision 3 draws: none were run (the batch statistics above are from the sampler alone).
3. The three pipeline tests did not finish within 5 minutes (sampler draws).
4. The Revision 1 fixed heading range was never built (superseded by Revision 2 and the held heading).
5. The sampler record is not written to disk when a run stops (only printed, and held in memory); the seed is in the log line.
6. The sampler's log prints only the failing check's name, not its reason (the reason is kept in memory, cut to 300 characters).
7. With a drawn heading (not held), the place's back-off direction uses the held yaw (see Heading).
8. `docs/progress.md` has uncommitted lines (mine and the other session's); not committed.
9. Plans with more than one stance run are not handled: one stance serves every manipulation step of the plan.

## DECISIONS NEEDED

1. **Bench**: about 61 % of the Revision 3 batch has its base over the bench top, and the top-weighted stances are mostly there. The A8 model has no bench term and the sampler has no filter by design. Should the sampler reject stances whose base is over the bench top before the full check?
2. **r_in**: the literal minimum (0.0497 m) is one cell 25 mm from the joint; it constrains little. Keep the literal value, or use a quantile (for example the 0.1 % point, 0.0585 m)?
3. **Heading**: keep held at −90° until the staging and navigation are tested at other headings, or test them now?
4. **Logging**: print the failing check's reason and the per-draw check time in the sampler log, and write the sampler record into the run record (for replay without the log)?
5. **End-to-end run with Revision 3**: go or no-go after the home_clearance pictures?
6. **Old search code**: delete the overlay and the old reach-model functions now, or after a working run?
