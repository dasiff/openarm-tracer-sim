# Edge stance selection (M300), 2026-11-10

Written for: David (decision review). Pictures: `docs/reports/edge_stance_selection_20261110/staging_selected_top.png`, `moved_selected_top.png`; margin: `margin.json`. Commits 9bcfe84 (usable edges, margin, A9 count), 5476e64 (edge sampler, wiring, tests, decision row M300, design plan). Nothing pushed.

## Result in one paragraph
Both layouts got a selected stance from the edge sampler and both e2e runs stopped at the pick's OVER move (not reached: 51 mm and 47 mm short). The stance selection itself worked in both layouts (draw 2 staging, draw 3 moved). The runs were not repeated and not fixed.

## Usable edges (found by `usable_edges.py`)
The scene's floor-standing tops (4 edges each, usable part and outward direction):
- bench_top (world entry bench, the support of the beaker and the hotplate): -y edge (back, y = -0.8), whole, 4.40 m, outward -y. +x edge (x = 2.2), whole, 1.60 m. -x edge (x = -2.2), whole, 1.60 m. +y edge (front, y = +0.8, the staging side) in 3 parts: x 0.198 to 2.200 (2.00 m), x -1.510 to -0.196 (1.31 m), x -2.200 to -1.888 (0.31 m). The stool (x -0.2..0.2) and the bin (x -1.88..-1.51) block the middle parts.
- Also found, not in the world model and not used: back_counter_top (4 edges), hood_worktop (4 edges), cart_top (4 edges).
Base length for the strip: 0.704 m (`base_length`, chassis geoms).

## Margin (MEASURED, experiments/edge_margin_study.py)
Robot square to the bench's +y edge, both arms at home, minimum over bench geoms only:

| pedestal | x = -1.0 | x = 0.0 | x = +1.0 | x = +1.6 |
|---|---|---|---|---|
| 0.05 m | touches at 30.00 mm, clear at 30.08 mm (openarm_body_link0 vs bench_top) | same | same | same |
| 0.25 m | same | same | same | same |

Stored: `sampler_margin_m = 0.0301` m (first clear distance, rounded up to 0.1 mm; no safety margin). Objects on the bench are not in this number: at x = +1.0 the right arm touches the hotplate (full body check -21 mm at 30 mm out). The gate handles those.

## r_max and A9
- r_max = A9 r0 + sigma_far = 0.5423 + 0.1231 = 0.6654 m (3D from the left shoulder).
- A9 (commit 05cc4b2): C 3.5141, r0 0.54235, sigma_near 0.41238, sigma_far 0.12310, bearing neg (0.25164, -0.50723) rad, pos (54.569, -126.61) rad, z cut -0.395 m. Stored in `sampler_a9`.

## Heading
Free (`sampler_heading_held = False`). The heading is the exact direction from the base to the task centroid.
- Fix: `points_at_heading` (src/stance_select.py, the back-off point of a place is rebuilt for each stance's heading; used by the full check and the score). Before, the BACK point used the staging yaw (-90 deg), which was NOT DONE item 7 of the M299 report.
- The place back-off motion (src/mpvi_modules.py, lines ~262-272) already uses the live forward direction, so no change there.
- Staging drive and creep-in read the yaw from task_cfg (src/staging_source.py, lines 367-383, 436-437): no constant -90 found, so no redesign.
- Evidence: the staging run reached manipulation at its chosen heading -101.8 deg; the moved run at -112.1 deg. Heading -118.7 deg (staging draw 1) failed in the full check (place angle window), not in the staging.

## Selection (both layouts)
Only one piece is feasible in each layout: the front (+y) edge, part x 0.39 to 1.43 (staging) and 0.24 to 1.28 (moved), y 0.83 to 1.29. The overlap is one edge part, so no "needs travel" case. Rectangle area 0.476 m2 in both.

| | staging | moved |
|---|---|---|
| seed | 2680074893 | 3912061599 |
| fraction of rectangle points kept | 0.603 (16586 draws) | 0.601 (16642 draws) |
| A9 score min / median / max | 0.000 / 1.450 / 2.885 | 0.000 / 1.445 / 2.905 |
| zero-score fraction | 0.024 | 0.020 |
| gate failures | 0 | 0 |
| selection time (record) | 120.6 s | 90.5 s |
| result | draw 2 PASS | draw 3 PASS |

Full draw table (from the records; heading in degrees from the log):

| layout | draw | x | y | heading | pedestal | edge | signed dist (m) | shoulder to pick (m) | shoulder to place (m) | A9 score | weight | result | reason | check time |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| staging | 1 | 1.0948 | 1.0160 | -118.7 | 0.1711 | +y | 0.216 | 0.479 | 0.529 | 1.925 | 3.704 | FAIL | task 3 (place): no place angle window | 40.3 s |
| staging | 2 | 0.9860 | 1.0704 | -101.8 | 0.1056 | +y | 0.270 | 0.475 | 0.492 | 1.388 | 1.927 | PASS | | 40.6 s |
| moved | 1 | 0.4863 | 1.0154 | -53.4 | 0.2354 | +y | 0.215 | 0.631 | 0.504 | 1.812 | 3.283 | FAIL | task 1 (pick): nominal grasp point 2 has no collision-free solution | 1.5 s |
| moved | 2 | 0.3439 | 0.9872 | -38.9 | 0.0549 | +y | 0.187 | 0.632 | 0.471 | 1.868 | 3.490 | FAIL | arms at home 37 mm from openarm_right_link4 vs bench_top (limit 40 mm) | 0.0 s |
| moved | 3 | 0.9103 | 1.0538 | -112.1 | 0.1032 | +y | 0.254 | 0.463 | 0.510 | 1.631 | 2.661 | PASS | | 31.2 s |

The gate never fired in these runs; a 2-stance stub check (not part of the runs) found the non-square touch at -3.9 mm and -0.6 mm, so the gate is needed.

## End-to-end runs (one each, not repeated)
- **Staging** (log `data/logs/edge_staging_20261110_111521.log`, record `data/demo_record/edge_staging_20261110_111521`): stance draw 2 (x 0.986, y 1.070, heading -101.8 deg, pedestal 0.1056 m). Staging completed; manipulation began at t = 68.5 s with OVER. OVER not reached after 30.8 s: 51.4 mm and 4.1 deg from the waypoint. Recovery (OPEN, BACK, HOME): BACK not reached after 19.3 s, 23.6 mm off. PLAN_FAILED at segment 2/4 (pick). Recording 17739 ticks, 118.3 s sim. Not measured: commanded versus settled pedestal (the summary was not read), place DOWN, release, HOME, joints 2 and 6, bench contact.
- **Moved** (log `data/logs/edge_moved_20261110_112032.log`, record `data/demo_record/edge_moved_20261110_112032`): stance draw 3 (x 0.910, y 1.054, heading -112.1 deg, pedestal 0.1032 m). OVER not reached after 31.5 s and again after the retry (47.5 mm, 3.7 deg). PLAN_FAILED at segment 2/4 (pick; retry_max 1 used). Recording 21728 ticks, 144.9 s sim. Not measured: as above.

## Pictures
Same camera and size as the earlier pictures (640 x 480, free top camera, azimuth 90, elevation -89, distance 2.6 m, 0.25 m ahead of the stance). Cyan: usable edges with outward arrows. Green: the overlap. Yellow: the piece box. Magenta: pick and place midpoints. Red: the chosen base and heading.
- `docs/reports/edge_stance_selection_20261110/staging_selected_top.png`
- `docs/reports/edge_stance_selection_20261110/moved_selected_top.png`

## Tests
- `experiments/edge_sampling_test.py`: ALL PASS (usable edges: free table 4, cabinet against wall 1, post splits edge to 2 parts; support chain beaker to bench; region margin and r_max; foot-centred; overlap; needs travel; heading to centroid; fraction kept; replay; A9 hand values; back-off point; real scene 6 bench parts, pieces on bench, margin and r_max agree). Added to `run_tests.sh`.
- `experiments/gaussian_sampling_test.py` (old polar sampler, unchanged code): 33/33.
- Stance-selection pipeline test files (stance_select_test, stance_select_moved_test, stance_select_pipeline_test, mpvi_pipeline_test): NOT RUN this session. They hit the 5-minute cap in the M299 work because each full check is about 40 s.

## Old search code left uncalled (David decides on deletion)
- `src/stance_sampling.py`: `draw_batch`, `a8_scores`, `select_stance`, `_select_stance`, `outer_limit_m`, `_turn`, `task_points` (the polar draw and the A8 sampler; `write_record` is still used).
- `src/stance_select.py`: `select_bench_stance`, `_two_pass`, `_log_two_pass_failure`, `_overlay`, `_best_stance`, `make_context` (only referenced within the module).
- `src/reach_model.py`: `inverse_search`, `inverse_stances`, `single_object_stance`, `ideal_spot`, `counts_at`, `_scan` (A8 predict functions still used by the A9 fit script and tests).
- `src/stance_finder.py`: `find_stances`, `find_stances_windowed`, `region_scores`, `arm_for` (only `robot_geometry` is still used).

## NOT DONE
1. Pedestal commanded versus settled, place DOWN, release on the hotplate, HOME, joints 2 and 6, bench contact: not measured for either run (the runs stopped at the pick).
2. Stance-selection pipeline test files: not run (time).
3. Elbow height and bench distance during the runs: not measured.
4. Progress log (`docs/progress.md`) has 20 uncommitted lines from another session; my entries are on disk and not committed.

## DECISIONS NEEDED
1. **Margin**: 30.1 mm is the measured bench contact at a square heading. Objects are tighter (the hotplate at x = +1.0 m touches the right arm about 21 mm before the bench does at 30 mm). Keep 30.1 mm and rely on the gate, or set a larger margin?
2. **OVER**: the full check passed draw 2 (staging) and draw 3 (moved), but the run missed OVER by 51 mm and 47 mm. Either the full check's OVER model disagrees with the execution, or the stance is too close to the bench for the pick move. Which should I investigate first? (Not fixed here.)
3. **Gate**: not triggered in the two runs. Keep it as designed?
4. **Heading**: free works through the staging and creep at -101.8 and -112.1 deg; -118.7 deg failed at the place angle. Accept free heading as the default?
