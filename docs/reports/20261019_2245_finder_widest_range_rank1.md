# Report 2026-10-19 22:45: placed-beaker requirement, widest-range ranking, finder rerun for rank 1

Written for: David. Nothing was applied: TASK_CFG, the staging stance and the home pose are unchanged.

## Result: the finder finds a stance for the rank 1 pose
Chosen: **x 0.765, y 1.085, heading -90 deg, pedestal 0.155 m**, passing range **0.10 to 0.21 m (11 cm wide)**. IK verification passes (12/12 points, whole arm clear, joint margin 30 deg, xy tolerance 7 cm, heading +-8 deg, pedestal +-4.2 cm).

| requirement | value | limit | margin |
|---|---|---|---|
| beaker hidden (pixels, eagle camera) | 0.0 % (4773 px visible) | 0 % | met |
| hotplate hidden | 0.1 % | 20 % | +19.9 points |
| framing slack, grippers in frame | 12.7 % (limit: right_gripper_tip1) | 10 % | +2.7 points |
| arms at home to everything | 116.0 mm (left finger vs hotplate top plate) | 40 mm | +76.0 mm |
| arms to a beaker placed on the hotplate | 97.0 mm | 40 mm | +57.0 mm |
| neck spread | 0.007 rad | 0.06 | +0.053 rad |
| fingers to bench | 163 mm | 40 mm | +123 mm |
| joint margin (home pose) | 13.9 deg | 10 deg | +3.9 deg |
| pedestal passing range | 0.10 to 0.21 (width 11 cm) | - | - |
| finder tolerance (xy) | 7.1 cm | - | - |
| xy region around the stance (pedestal 0.155, grid +-6 cm, 2 cm) | 39 of 49 points pass, largest passing radius 5.7 cm | - | - |

Notes:
- Tolerance is the price of the wide range: the old choice had 15.0 cm but one passing height. Candidate 4 (0.735, 1.045) has a 9-step range (0.10 to 0.19) with tolerance 10.8 cm; the rule (widest range first) takes candidate 12 (11 steps). Only candidates 2, 4, 6, 11, 12 had a passing range (widths 4, 9, 2, 3, 11 steps); the other seven had none.
- The stance is 11 cm further out in x and 9 cm further back in y than the current staging stance, so the beaker is 42.5 cm forward of the torso (34.5 now). The chassis is 5.6 cm under the bench top, the torso front 25.5 cm behind its edge.
- The 4 failing xy grid points fail on the placed beaker (4), the neck spread (4), occlusion (1); one is out of reach.
- The framing slack margin (+2.7 points) is the thinnest one; the right gripper's tip is what limits it.
- The grasp tests and the IK targets were built for the old stance; if this stance is applied, grasp_lift_test and gripper_force_test need a rerun there.

## What changed
- `STANCE_REQUIREMENTS` (`src/task_config.py`) has `placed_beaker_min_clearance` 0.040 m. `StanceChecks.requirements` checks it (cheap, right after the home clearance); it is a finder requirement (selection and xy region), a search margin and a `home_pose_test` check (new check: nominal home at the staging stance).
- Ranking (`experiments/stance_from_map.py`, design plan §3.4): among candidates with a passing range, the widest range first, the middle of that range as the height (the nearest passing grid height if the middle falls between two and fails), ties by tolerance, then the IK verification. Runs are split where a grid height is skipped. Recorded in §3.4.
- Not rerun with the new requirement: `home_pose_test` (pose A is at -2 mm to the placed beaker, so it will fail the new check, as the comparison shows; it also still fails the occlusion and framing checks) and the home pose search (ranks still come from `evals.jsonl`, the v2 search).
- Cost: finder run 23 min (1404 s, fewer candidates reach the slow checks); log `data/logs/home_search_finder1_widest_range_20261019_221323.log`; folder `data/home_pose_search/search_rank1_widest_range/` (summary.json, open_b45_selected.json, images at_chosen_stance/).

## Comparison images (where to find them)
- Folder: `data/home_pose_search/compare_widest_range/` (`comparison.png`, `comparison.json`); the image is also copied to `docs/reports/20261019_2245_comparison_widest_range.png` (committed). The older set, with the unreadable labels and the old finder stance, is in `data/home_pose_search/compare/`.
- Labels are white on dark boxes now. Rows: front, left side, eagle camera; one column per pose: A, rank 1 (at the new finder stance), ranks 2 and 3 (at the stance the search evaluated them at; not through the finder).

| | A (current) | rank 1 (finder stance) | rank 2 | rank 3 |
|---|---|---|---|---|
| stance x, y, pedestal | 0.655, 0.995, 0.11 | 0.765, 1.085, 0.155 | 0.660, 1.031, 0.24 | 0.665, 1.046, 0.20 |
| beaker hidden (0 %) | 0.0 % | 0.0 % | **5.1 %** | **3.8 %** |
| hotplate hidden (<= 20 %) | **26.8 %** | 0.1 % | 8.1 % | 6.0 % |
| framing slack (>= 10 %) | **9.8 %** | 12.7 % | 13.1 % | 11.6 % |
| arms / fingers to hotplate (>= 40 mm) | 63 / 63 | 116 / 116 | 184 / 184 | 156 / 156 |
| arms to placed beaker (>= 40 mm) | **-2** | 97 | 102 | 81 |
| fingers to bench | 131 | 163 | 243 | 214 |
| neck spread (<= 0.06) | 0.014 | 0.007 | 0.005 | 0.004 |
| joint margin (>= 10 deg) | **10.0** (just under) | 13.9 | 14.0 | 11.5 |
| finder tolerance | 14.9 cm | 7.1 cm | 0.0 cm | 2.2 cm |

## Open
- Apply rank 1 at the finder's stance? (Not done; it needs TASK_CFG, the staging check, home_pose_test and the grasp tests at the new stance.)
- Push the local commits to `wip/new-stance` (unpushed since 62f4c1d).
