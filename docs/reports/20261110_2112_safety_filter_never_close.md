# Safety filter never-close pair list (2026-11-10)

Kinematics only. Branch safety-filter, not pushed. Written for David.

## Outcome
Pairs that can never come within their margin on the REAL geometry are now skipped by the touch rules (SF-23). The list is in `data/safety_filter/robot_shapes.json` (`never_close`, under the fingerprint). With it, **0 MARGIN rows remain on checked pairs** (arm vs world at M1-M9; robot vs robot at P1, P3, P4; pedestals 0.155 and 0.120 m); before it there were 40, all left link 1 vs torso. Left link 1 vs torso is on the list. Shapes are unchanged (byte-identical apart from the new key); no refit, no margin change.

## Method (as asked)
- Variables of a pair: the filter's state variables (7 joints per arm, one finger value per hand, pedestal height) on the joint-tree path between the two parts. Base pose and wheels are fixed. The torso, eagle camera, shoulder mounts and pedestal top ride the pedestal with the arm; the base boxes, wheels, battery and pedestal base do not, so arm vs those adds the pedestal.
- 3 or fewer: sweep on the real geoms with mj_geomDistance: 1 variable 0.5 deg (pedestal 2 mm); 2 variables 1 deg grid; 3 variables 3 deg grid + 20000 random samples (seed 0). Full joint limits.
- Step-size bound = sum over swept variables of lever arm x half step. The lever arm is a pose-independent upper bound on any vertex's distance from the joint axis (first link offset perpendicular to the axis + rigid offsets down the chain + the vertex's distance from its body origin). It is rigorous but loose. For 1 and 2 variables the step was halved until the bound is 3 mm or less (the starting steps already met it: largest 1.73 mm).
- Listed only if minimum real gap - bound is at least margin + 1 cm (hand-hand 2 cm, other 4 cm). More than 3 variables: "not swept" (stay checked).
- Tool: `tools/never_close_sweep.py` (3 min; log `data/logs/nc_sweep_20261110_205804.log`; result `data/safety_filter/never_close_sweep.json`, keyed by real geoms; `tools/build_filter_shapes.py --merge-never-close` copies it into the shapes file).

## Counts
- 665 checked shape pairs on 382 distinct real-geom pairs.
- Swept: 85 geom pairs (153 shape pairs). Listed: 81 (147 shape pairs). Swept, not listed: 4 geom pairs (6 shape pairs): link 3 vs torso and link 3 vs the eagle camera, both arms (minimum real gap 42.0 mm and 27.0 mm; the 3 deg grid bound is 15.4 mm).
- Not swept: 297 geom pairs (512 shape pairs): 4 variables 29, 5: 30, 6: 31, 7: 40, 8: 61, 9: 48, 10-16: 58.
- A gap shown as >=500 means the search distance (0.5 m) was never reached: all samples are at least that far.

## Left link 1 vs torso
Listed. Variable: left joint 1 only. Range -3.491 to +1.396 rad, 0.5 deg steps (562 points). Minimum real gap 64.0 mm at joint 1 = +1.396 rad (80 deg, the upper limit). Step-size bound 0.26 mm. Required 40 mm. The poses we use have joint 1 between -0.754 and +0.471 rad. The right arm is the mirror: 64.0 mm at -1.396 rad.

## Sweep table (swept pairs, all rows)
| pair (real geoms) | variables swept | step | bound (mm) | min real gap (mm) | at | listed | shape pairs |
|---|---|---|---|---|---|---|---|
| eagle_cam_body_box vs left_link0 | - | - | 0.00 | 127.1 | - | yes | 1 |
| eagle_cam_body_box vs right_link0 | - | - | 0.00 | 127.1 | - | yes | 1 |
| left_link0 vs pedestal_top | - | - | 0.00 | >=500 | - | yes | 1 |
| right_link0 vs pedestal_top | - | - | 0.00 | >=500 | - | yes | 1 |
| base_link_geom vs left_link0 | pedestal | pedestal 2 mm | 1.00 | >=500 | pedestal +0.000 | yes | 1 |
| base_link_geom vs right_link0 | pedestal | pedestal 2 mm | 1.00 | >=500 | pedestal +0.000 | yes | 1 |
| battery_geom vs left_link0 | pedestal | pedestal 2 mm | 1.00 | >=500 | pedestal +0.000 | yes | 1 |
| battery_geom vs right_link0 | pedestal | pedestal 2 mm | 1.00 | >=500 | pedestal +0.000 | yes | 1 |
| body_link0 vs left_link1 | left_joint1 | left_joint1 0.5 deg | 0.26 | 64.0 | left_joint1 +1.396 | yes | 3 |
| body_link0 vs right_link1 | right_joint1 | right_joint1 0.5 deg | 0.26 | 64.0 | right_joint1 -1.396 | yes | 3 |
| eagle_cam_body_box vs left_link1 | left_joint1 | left_joint1 0.5 deg | 0.26 | 151.7 | left_joint1 -2.539 | yes | 3 |
| eagle_cam_body_box vs right_link1 | right_joint1 | right_joint1 0.5 deg | 0.26 | 151.7 | right_joint1 +2.539 | yes | 3 |
| left_Link_geom vs left_link0 | pedestal | pedestal 2 mm | 1.00 | >=500 | pedestal +0.000 | yes | 1 |
| left_Link_geom vs right_link0 | pedestal | pedestal 2 mm | 1.00 | >=500 | pedestal +0.000 | yes | 1 |
| left_front_link_geom vs left_link0 | pedestal | pedestal 2 mm | 1.00 | >=500 | pedestal +0.000 | yes | 1 |
| left_front_link_geom vs right_link0 | pedestal | pedestal 2 mm | 1.00 | >=500 | pedestal +0.000 | yes | 1 |
| left_link0 vs pedestal_base | pedestal | pedestal 2 mm | 1.00 | >=500 | pedestal +0.000 | yes | 1 |
| left_link0 vs right_front_link_geom | pedestal | pedestal 2 mm | 1.00 | >=500 | pedestal +0.000 | yes | 1 |
| left_link0 vs right_link1 | right_joint1 | right_joint1 0.5 deg | 0.26 | 125.0 | right_joint1 -1.396 | yes | 3 |
| left_link0 vs right_link_geom | pedestal | pedestal 2 mm | 1.00 | >=500 | pedestal +0.000 | yes | 1 |
| left_link0 vs right_rear_link_geom | pedestal | pedestal 2 mm | 1.00 | >=500 | pedestal +0.000 | yes | 1 |
| left_link1 vs pedestal_top | left_joint1 | left_joint1 0.5 deg | 0.26 | >=500 | left_joint1 -3.491 | yes | 3 |
| left_link1 vs right_link0 | left_joint1 | left_joint1 0.5 deg | 0.26 | 125.0 | left_joint1 +1.396 | yes | 3 |
| left_rear_link_geom vs left_link0 | pedestal | pedestal 2 mm | 1.00 | >=500 | pedestal +0.000 | yes | 1 |
| left_rear_link_geom vs right_link0 | pedestal | pedestal 2 mm | 1.00 | >=500 | pedestal +0.000 | yes | 1 |
| right_link0 vs pedestal_base | pedestal | pedestal 2 mm | 1.00 | >=500 | pedestal +0.000 | yes | 1 |
| right_link0 vs right_front_link_geom | pedestal | pedestal 2 mm | 1.00 | >=500 | pedestal +0.000 | yes | 1 |
| right_link0 vs right_link_geom | pedestal | pedestal 2 mm | 1.00 | >=500 | pedestal +0.000 | yes | 1 |
| right_link0 vs right_rear_link_geom | pedestal | pedestal 2 mm | 1.00 | >=500 | pedestal +0.000 | yes | 1 |
| right_link1 vs pedestal_top | right_joint1 | right_joint1 0.5 deg | 0.26 | >=500 | right_joint1 -1.396 | yes | 3 |
| base_link_geom vs left_link1 | left_joint1, pedestal | left_joint1 1 deg, pedestal 2 mm | 1.52 | >=500 | left_joint1 -3.491, pedestal +0.000 | yes | 3 |
| base_link_geom vs right_link1 | pedestal, right_joint1 | pedestal 2 mm, right_joint1 1 deg | 1.52 | >=500 | pedestal +0.000, right_joint1 -1.396 | yes | 3 |
| battery_geom vs left_link1 | left_joint1, pedestal | left_joint1 1 deg, pedestal 2 mm | 1.52 | >=500 | left_joint1 -3.491, pedestal +0.000 | yes | 3 |
| battery_geom vs right_link1 | pedestal, right_joint1 | pedestal 2 mm, right_joint1 1 deg | 1.52 | >=500 | pedestal +0.000, right_joint1 -1.396 | yes | 3 |
| body_link0 vs left_link2 | left_joint1, left_joint2 | left_joint1 1 deg, left_joint2 1 deg | 1.73 | 74.1 | left_joint1 -1.571, left_joint2 +0.105 | yes | 1 |
| body_link0 vs right_link2 | right_joint1, right_joint2 | right_joint1 1 deg, right_joint2 1 deg | 1.73 | 74.1 | right_joint1 +1.571, right_joint2 -0.105 | yes | 1 |
| eagle_cam_body_box vs left_link2 | left_joint1, left_joint2 | left_joint1 1 deg, left_joint2 1 deg | 1.73 | 139.1 | left_joint1 -3.491, left_joint2 +0.175 | yes | 1 |
| eagle_cam_body_box vs right_link2 | right_joint1, right_joint2 | right_joint1 1 deg, right_joint2 1 deg | 1.73 | 139.1 | right_joint1 +3.491, right_joint2 -0.175 | yes | 1 |
| left_Link_geom vs left_link1 | left_joint1, pedestal | left_joint1 1 deg, pedestal 2 mm | 1.52 | >=500 | left_joint1 -3.491, pedestal +0.000 | yes | 3 |
| left_Link_geom vs right_link1 | pedestal, right_joint1 | pedestal 2 mm, right_joint1 1 deg | 1.52 | >=500 | pedestal +0.000, right_joint1 -1.396 | yes | 3 |
| left_front_link_geom vs left_link1 | left_joint1, pedestal | left_joint1 1 deg, pedestal 2 mm | 1.52 | >=500 | left_joint1 -3.491, pedestal +0.000 | yes | 3 |
| left_front_link_geom vs right_link1 | pedestal, right_joint1 | pedestal 2 mm, right_joint1 1 deg | 1.52 | >=500 | pedestal +0.000, right_joint1 -1.396 | yes | 3 |
| left_link0 vs right_link2 | right_joint1, right_joint2 | right_joint1 1 deg, right_joint2 1 deg | 1.73 | 135.1 | right_joint1 +1.606, right_joint2 -0.105 | yes | 1 |
| left_link1 vs pedestal_base | left_joint1, pedestal | left_joint1 1 deg, pedestal 2 mm | 1.52 | >=500 | left_joint1 -3.491, pedestal +0.000 | yes | 3 |
| left_link1 vs right_front_link_geom | left_joint1, pedestal | left_joint1 1 deg, pedestal 2 mm | 1.52 | >=500 | left_joint1 -3.491, pedestal +0.000 | yes | 3 |
| left_link1 vs right_link1 | left_joint1, right_joint1 | left_joint1 1 deg, right_joint1 1 deg | 1.04 | 188.0 | left_joint1 +1.396, right_joint1 -1.396 | yes | 9 |
| left_link1 vs right_link_geom | left_joint1, pedestal | left_joint1 1 deg, pedestal 2 mm | 1.52 | >=500 | left_joint1 -3.491, pedestal +0.000 | yes | 3 |
| left_link1 vs right_rear_link_geom | left_joint1, pedestal | left_joint1 1 deg, pedestal 2 mm | 1.52 | >=500 | left_joint1 -3.491, pedestal +0.000 | yes | 3 |
| left_link2 vs pedestal_top | left_joint1, left_joint2 | left_joint1 1 deg, left_joint2 1 deg | 1.73 | >=500 | left_joint1 -3.491, left_joint2 -3.316 | yes | 1 |
| left_link2 vs right_link0 | left_joint1, left_joint2 | left_joint1 1 deg, left_joint2 1 deg | 1.73 | 135.1 | left_joint1 -1.606, left_joint2 +0.105 | yes | 1 |
| left_rear_link_geom vs left_link1 | left_joint1, pedestal | left_joint1 1 deg, pedestal 2 mm | 1.52 | >=500 | left_joint1 -3.491, pedestal +0.000 | yes | 3 |
| left_rear_link_geom vs right_link1 | pedestal, right_joint1 | pedestal 2 mm, right_joint1 1 deg | 1.52 | >=500 | pedestal +0.000, right_joint1 -1.396 | yes | 3 |
| right_link1 vs pedestal_base | pedestal, right_joint1 | pedestal 2 mm, right_joint1 1 deg | 1.52 | >=500 | pedestal +0.000, right_joint1 -1.396 | yes | 3 |
| right_link1 vs right_front_link_geom | pedestal, right_joint1 | pedestal 2 mm, right_joint1 1 deg | 1.52 | >=500 | pedestal +0.000, right_joint1 -1.396 | yes | 3 |
| right_link1 vs right_link_geom | pedestal, right_joint1 | pedestal 2 mm, right_joint1 1 deg | 1.52 | >=500 | pedestal +0.000, right_joint1 -1.396 | yes | 3 |
| right_link1 vs right_rear_link_geom | pedestal, right_joint1 | pedestal 2 mm, right_joint1 1 deg | 1.52 | >=500 | pedestal +0.000, right_joint1 -1.396 | yes | 3 |
| right_link2 vs pedestal_top | right_joint1, right_joint2 | right_joint1 1 deg, right_joint2 1 deg | 1.73 | >=500 | right_joint1 -1.396, right_joint2 -0.175 | yes | 1 |
| base_link_geom vs left_link2 | left_joint1, left_joint2, pedestal | left_joint1 3 deg, left_joint2 3 deg, pedestal 2 mm | 6.19 | >=500 | left_joint1 -3.491, left_joint2 -3.316, pedestal +0.000 | yes | 1 |
| base_link_geom vs right_link2 | pedestal, right_joint1, right_joint2 | pedestal 2 mm, right_joint1 3 deg, right_joint2 3 deg | 6.19 | >=500 | pedestal +0.000, right_joint1 -1.396, right_joint2 -0.175 | yes | 1 |
| battery_geom vs left_link2 | left_joint1, left_joint2, pedestal | left_joint1 3 deg, left_joint2 3 deg, pedestal 2 mm | 6.19 | >=500 | left_joint1 -3.491, left_joint2 -3.316, pedestal +0.000 | yes | 1 |
| battery_geom vs right_link2 | pedestal, right_joint1, right_joint2 | pedestal 2 mm, right_joint1 3 deg, right_joint2 3 deg | 6.19 | >=500 | pedestal +0.000, right_joint1 -1.396, right_joint2 -0.175 | yes | 1 |
| body_link0 vs left_link3 | left_joint1, left_joint2, left_joint3 | left_joint1 3 deg, left_joint2 3 deg, left_joint3 3 deg | 15.37 | 42.0 | left_joint1 -3.124, left_joint2 -3.316, left_joint3 +0.733 | NO | 2 |
| body_link0 vs right_link3 | right_joint1, right_joint2, right_joint3 | right_joint1 3 deg, right_joint2 3 deg, right_joint3 3 deg | 15.37 | 42.0 | right_joint1 +0.017, right_joint2 -0.175, right_joint3 +0.733 | NO | 1 |
| eagle_cam_body_box vs left_link3 | left_joint1, left_joint2, left_joint3 | left_joint1 3 deg, left_joint2 3 deg, left_joint3 3 deg | 15.37 | 27.0 | left_joint1 +0.175, left_joint2 -3.316, left_joint3 +0.733 | NO | 2 |
| eagle_cam_body_box vs right_link3 | right_joint1, right_joint2, right_joint3 | right_joint1 3 deg, right_joint2 3 deg, right_joint3 3 deg | 15.37 | 27.0 | right_joint1 +2.950, right_joint2 -0.175, right_joint3 +0.733 | NO | 1 |
| left_Link_geom vs left_link2 | left_joint1, left_joint2, pedestal | left_joint1 3 deg, left_joint2 3 deg, pedestal 2 mm | 6.19 | >=500 | left_joint1 -3.491, left_joint2 -3.316, pedestal +0.000 | yes | 1 |
| left_Link_geom vs right_link2 | pedestal, right_joint1, right_joint2 | pedestal 2 mm, right_joint1 3 deg, right_joint2 3 deg | 6.19 | >=500 | pedestal +0.000, right_joint1 -1.396, right_joint2 -0.175 | yes | 1 |
| left_front_link_geom vs left_link2 | left_joint1, left_joint2, pedestal | left_joint1 3 deg, left_joint2 3 deg, pedestal 2 mm | 6.19 | >=500 | left_joint1 -3.491, left_joint2 -3.316, pedestal +0.000 | yes | 1 |
| left_front_link_geom vs right_link2 | pedestal, right_joint1, right_joint2 | pedestal 2 mm, right_joint1 3 deg, right_joint2 3 deg | 6.19 | >=500 | pedestal +0.000, right_joint1 -1.396, right_joint2 -0.175 | yes | 1 |
| left_link0 vs right_link3 | right_joint1, right_joint2, right_joint3 | right_joint1 3 deg, right_joint2 3 deg, right_joint3 3 deg | 15.37 | 127.3 | right_joint1 -0.401, right_joint2 -0.175, right_joint3 +0.995 | yes | 1 |
| left_link1 vs right_link2 | left_joint1, right_joint1, right_joint2 | left_joint1 3 deg, right_joint1 3 deg, right_joint2 3 deg | 6.76 | 198.1 | left_joint1 +0.726, right_joint1 +1.639, right_joint2 -0.103 | yes | 3 |
| left_link2 vs pedestal_base | left_joint1, left_joint2, pedestal | left_joint1 3 deg, left_joint2 3 deg, pedestal 2 mm | 6.19 | >=500 | left_joint1 -3.491, left_joint2 -3.316, pedestal +0.000 | yes | 1 |
| left_link2 vs right_front_link_geom | left_joint1, left_joint2, pedestal | left_joint1 3 deg, left_joint2 3 deg, pedestal 2 mm | 6.19 | >=500 | left_joint1 -3.491, left_joint2 -3.316, pedestal +0.000 | yes | 1 |
| left_link2 vs right_link1 | left_joint1, left_joint2, right_joint1 | left_joint1 3 deg, left_joint2 3 deg, right_joint1 3 deg | 6.76 | 198.1 | left_joint1 -1.675, left_joint2 +0.102, right_joint1 -1.122 | yes | 3 |
| left_link2 vs right_link_geom | left_joint1, left_joint2, pedestal | left_joint1 3 deg, left_joint2 3 deg, pedestal 2 mm | 6.19 | >=500 | left_joint1 -3.491, left_joint2 -3.316, pedestal +0.000 | yes | 1 |
| left_link2 vs right_rear_link_geom | left_joint1, left_joint2, pedestal | left_joint1 3 deg, left_joint2 3 deg, pedestal 2 mm | 6.19 | >=500 | left_joint1 -3.491, left_joint2 -3.316, pedestal +0.000 | yes | 1 |
| left_link3 vs pedestal_top | left_joint1, left_joint2, left_joint3 | left_joint1 3 deg, left_joint2 3 deg, left_joint3 3 deg | 15.37 | 452.3 | left_joint1 -3.019, left_joint2 -3.316, left_joint3 +1.152 | yes | 2 |
| left_link3 vs right_link0 | left_joint1, left_joint2, left_joint3 | left_joint1 3 deg, left_joint2 3 deg, left_joint3 3 deg | 15.37 | 127.3 | left_joint1 -0.401, left_joint2 -3.316, left_joint3 +0.995 | yes | 2 |
| left_rear_link_geom vs left_link2 | left_joint1, left_joint2, pedestal | left_joint1 3 deg, left_joint2 3 deg, pedestal 2 mm | 6.19 | >=500 | left_joint1 -3.491, left_joint2 -3.316, pedestal +0.000 | yes | 1 |
| left_rear_link_geom vs right_link2 | pedestal, right_joint1, right_joint2 | pedestal 2 mm, right_joint1 3 deg, right_joint2 3 deg | 6.19 | >=500 | pedestal +0.000, right_joint1 -1.396, right_joint2 -0.175 | yes | 1 |
| right_link2 vs pedestal_base | pedestal, right_joint1, right_joint2 | pedestal 2 mm, right_joint1 3 deg, right_joint2 3 deg | 6.19 | >=500 | pedestal +0.000, right_joint1 -1.396, right_joint2 -0.175 | yes | 1 |
| right_link2 vs right_front_link_geom | pedestal, right_joint1, right_joint2 | pedestal 2 mm, right_joint1 3 deg, right_joint2 3 deg | 6.19 | >=500 | pedestal +0.000, right_joint1 -1.396, right_joint2 -0.175 | yes | 1 |
| right_link2 vs right_link_geom | pedestal, right_joint1, right_joint2 | pedestal 2 mm, right_joint1 3 deg, right_joint2 3 deg | 6.19 | >=500 | pedestal +0.000, right_joint1 -1.396, right_joint2 -0.175 | yes | 1 |
| right_link2 vs right_rear_link_geom | pedestal, right_joint1, right_joint2 | pedestal 2 mm, right_joint1 3 deg, right_joint2 3 deg | 6.19 | >=500 | pedestal +0.000, right_joint1 -1.396, right_joint2 -0.175 | yes | 1 |
| right_link3 vs pedestal_top | right_joint1, right_joint2, right_joint3 | right_joint1 3 deg, right_joint2 3 deg, right_joint3 3 deg | 15.37 | 452.3 | right_joint1 +0.122, right_joint2 -0.175, right_joint3 +1.152 | yes | 1 |

## Recheck
- Rule check (list OFF -> ON): 98 flagged rows (40 MARGIN, 58 BAND) -> 8 rows (0 MARGIN, 8 BAND). The list removed: left_link1_3 vs torso (20 MARGIN rows, shape +2.5 to +4.6 mm), left_link1_2 vs torso (20 MARGIN, +11.7 to +13.4 mm), and 50 BAND rows (right_link1_2/_3 vs torso 40, left_link1_1 vs torso 9, left_link2 vs torso at M1 ped 0.12: 1).
- Remaining BAND rows (allowed): left link 7 pieces and palm 2 vs bench at M2 (shape +14.6 to +18.7 mm, margin 10), and left_link3_2 vs torso at M9 (+37.0 mm at 0.155 m, +38.6 mm at 0.120 m, margin 30; link 3 vs torso is not on the list).
- Tests: shapes PASS, touch PASS (new section: list entries obey the rule, touch rules exempt every listed pair, 300 random joint states keep every listed pair's real gap at least margin + 1 cm; smallest margin left +24.0 mm), distance PASS, structure PASS (its parameter check now allows `use_never_close`).
- Timing, same 200 random states, median ms (3 alternating runs; VM load average 1.8 to 5.0, from my sweep and test jobs and other sessions): list OFF total 3.05 (3.03, 3.60, 3.05), list ON 2.77 (2.55, 2.77, 3.04). Checked pairs median 22 -> 16. The 1 ms pair and 2 ms tick targets are not met at this load.

## Housekeeping
Deleted `experiments/safety_filter_torso_evidence.py`, `safety_filter_robot_pairs_check.py`, `safety_filter_forearm_fit.py` (not in run_tests.sh; two older reports now say "deleted"). `set_scene` moved to `experiments/safety_filter_pick_scene.py` (the torso-share script used it). The link-count search prints a progress line per link, per count and every 10 candidates.

## Not done
- Pairs with more than 3 variables stay checked (512 shape pairs), including all palm, finger and held pairs.
- Lever arms are loose upper bounds, so link 3 vs torso (min 42.0 mm) fails the listing although its real clearance may be larger than the bound allows; a tighter bound or a finer grid could list it.
- The sweep trusts mj_geomDistance for the minimum; it is not cross-checked by a second method (the random-state test is an independent sampling check only).
