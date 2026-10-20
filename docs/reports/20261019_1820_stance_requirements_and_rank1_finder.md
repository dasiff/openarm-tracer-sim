# Report 2026-10-19 18:20: one set of stance requirements, finder rerun for rank 1, pose comparison

Written for: David (the home pose / stance work). Nothing was applied: TASK_CFG, the staging stance and the home pose are unchanged.

## 1. Backup and log
- `main` pushed to `origin wip/new-stance` (4d05ec3..d91eba9), not to main. Commits made after that push (the code below, the comparison script, the log) are local only; I have not pushed them (WIP, rule 2).
- `docs/progress.md`: the IK-target tests entry is corrected (both finished and pass: grasp_lift_test 4/4, gripper_force_test 29/29, log `data/logs/tests_ik_targets_20261019_161815.log`).
- CLAUDE.md rule 6 (reports survive crashes) added; this file is the first one.

## 2. One set of requirements (done)
The limits live in one place, `STANCE_REQUIREMENTS` in `src/task_config.py`: arms at home at least 40 mm from everything, neck spread at most 0.06 rad, framing slack at least 10 % (both grippers in the frame), occlusion by the pixel-area measure: beaker 0 % hidden, hotplate at most 20 % hidden.
- `src/stance_checks.py`: new `OcclusionMeter` (segmentation render of the eagle camera with the robot and with the robot invisible; hidden share = 1 - visible / free pixels), and `StanceChecks.framing_slack`, `.occlusion`, `.requirements` (cheap checks first, later ones skipped when an earlier one fails). The 18-point ray test is gone from `neck_spread`.
- `src/neck_framing.py`: `best_margin` (the framing slack) moved here from `home_pose_test.py` (still importable from there).
- Used by: the finder's selection and xy region (`experiments/stance_from_map.py`), the home pose search (`home_pose_search.py`: two occlusion margins, beaker -hidden/5 % or +cap when 0 px hidden, hotplate (20 % - hidden)/20 %), `home_pose_candidates.py`, `home_pose_test.py` (pixel occlusion and the 10 % framing slack as checks).
- The new search writes `evals_v3_pixel.jsonl`; `evals.jsonl` (v2, ray occlusion) is untouched and still the source of ranks 1 to 3. The search was NOT rerun.
- Cost: a first occlusion call takes about 22 s (renderers, model copy), later ones 0.3 s; framing slack 1.3 s; the neck spread (9.7 s) still dominates. The finder run now takes 47 min (it took 25).
- home_pose_test with the new checks: 30/32 (log `data/logs/home_pose_test_pixel_20261019_172037.log`). The two failures are pose A, as measured: hotplate 26.4 % hidden (limit 20 %), nominal framing slack 9.9 % (limit 10 %, beaker_corner7). Everything else passes.

## 3. Finder rerun for rank 1 under the new checks
Folder `data/home_pose_search/search_rank1_pixel_framing/` (the old `search_rank1/` is kept), log `data/logs/home_search_finder1_pixel_framing_20261019_172037.log`.

Chosen stance: x 0.705, y 1.005, heading -90 deg, pedestal 0.110 m. IK verification passes (12/12 points, joint margin 22 deg).

| requirement | value | limit | margin |
|---|---|---|---|
| beaker hidden (pixels) | 0.0 % (5223 px visible) | 0 % | met |
| hotplate hidden | 4.6 % | 20 % | +15.4 points |
| framing slack, grippers in frame | 12.7 % (limit: right_gripper_tip1) | 10 % | +2.7 points |
| arms at home to everything | 50.3 mm (left finger vs hotplate top plate) | 40 mm | +10.3 mm |
| neck spread | 0.040 rad | 0.06 | +0.020 rad |
| joint margin | 13.9 deg | 10 deg | +3.9 deg |
| finder tolerance (xy) | 15.0 cm | - | - |
| pedestal passing range | 0.11 to 0.11 (one 1 cm grid height) | - | - |
| xy region around the stance | 20 of 49 grid points pass, largest passing radius 2.0 cm | - | - |
| arms to a beaker placed on the hotplate | **-3.0 mm** | 40 mm in the search; information only in the finder | - |

Things to decide:
1. **The pedestal range is a single height.** The finder ranks by tolerance at the chosen height, so it took candidate 1 (tolerance 15.0 cm) whose passing run is 0.11 only. Other candidates are much more forgiving in height: candidate 2 (0.655, 0.995) passes 0.12 to 0.23 (tolerance 9.8 cm, home clearance 110 mm, hotplate 11 % hidden, framing 12.7 %); candidate 4 (0.735, 1.045) passes 0.10 to 0.19 (tolerance 10.8 cm, clearance 95 mm, hotplate 2.4 %, framing 12.7 %); candidate 12 (0.765, 1.085) passes 0.10 to 0.21 but tolerance 7.1 cm. The old stance for this pose (0.615, 0.985) now has no passing height (framing slack 4.8 % there).
2. **Placed-beaker clearance.** Your original list had finger clearance of 40 mm to "a beaker placed on the hotplate"; the search enforces it, but the finder only reports it. At the chosen stance it is -3 mm (pose A at its stance: -2 mm). If it is a requirement for the finder too, it needs adding to `STANCE_REQUIREMENTS` and the finder would have to be rerun (47 min).
3. xy region is small (radius 2 cm, the passing points form a diagonal strip): the failures are the neck spread (10 points), occlusion (8), home clearance (8), framing (3).

## 4. Comparison of A and ranks 1 to 3
Image: `data/home_pose_search/compare/comparison.png` (4 columns; rows: front view, left side view, eagle camera; green = requirement met, red = not). Numbers: `comparison.json`, log `data/logs/home_compare2_20261019_181754.log`. Each pose is drawn at its own stance: A at the current one; rank 1 at the new finder stance; ranks 2 and 3 at the stance the search evaluated them at (they have not been through the finder).

| | A (current) | rank 1 (finder stance) | rank 2 | rank 3 |
|---|---|---|---|---|
| stance x, y, pedestal | 0.655, 0.995, 0.11 | 0.705, 1.005, 0.11 | 0.660, 1.031, 0.24 | 0.665, 1.046, 0.20 |
| beaker hidden (0 %) | 0.0 % | 0.0 % | **5.1 %** | **3.8 %** |
| hotplate hidden (<= 20 %) | **26.8 %** | 4.6 % | 8.1 % | 6.0 % |
| framing slack (>= 10 %) | **9.8 %** | 12.7 % | 13.1 % | 11.6 % |
| arms / fingers to hotplate (>= 40 mm) | 63 / 63 | 50 / 50 | 184 / 184 | 156 / 156 |
| arms to placed beaker (>= 40 mm) | **-2** | **-3** | 102 | 81 |
| fingers to bench (>= 40 mm) | 131 | 109 | 243 | 214 |
| neck spread (<= 0.06) | 0.014 | 0.040 | 0.005 | 0.004 |
| joint margin (>= 10 deg) | 10 | 14 | 14 | 12 |
| finder tolerance | 14.9 cm | 15.0 cm | **0.0 cm** | **2.2 cm** |

Reading it: under the new requirements A fails hotplate occlusion and framing; ranks 2 and 3 fail the beaker's 0 % (a finger hides a corner), and at their search stances the finder's reach tolerance is about zero (pedestal 0.24 and 0.20 are at or past the edge of the valid heights there). Rank 1 at the finder's stance meets every finder requirement and is the only column that meets occlusion and framing and is also reachable (tolerance 15 cm); its one red entry is the placed-beaker clearance (-3 mm), as is A's.
Caveats: the front view is small (the robot is far away at the fixed distance) and the "front" / "left side" row labels are white on a white background, so hard to read; rows are in the order front, left side, eagle.

## 5. Open
- Whether to add the placed-beaker clearance to the finder's requirements, and whether to prefer a candidate with a wide pedestal range (2 or 4) over candidate 1.
- Whether to rerun the search with the pixel margins (about 40 min; code ready, writes `evals_v3_pixel.jsonl`), since ranks 2 and 3 fail under the new occlusion limit.
- Whether to push the local commits to `wip/new-stance`.
