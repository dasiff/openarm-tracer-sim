# Middle rule versus most tolerant angle, 2026-11-09 (read-only, saved data only)

Written for David's decision on the place approach-angle rule. Numbers only; no rule is recommended here. No simulation was run and no file in src/, config, tests or docs was changed. This report is untracked.

## Method

- Rule's pick: oc_a3's `choose_approach_angle` for places (the passing turn nearest the middle of the widest run of 5-degree-step passing turns, walking outward; ties to the lower turn). For picks the same rule on the 12-bin circle (a run may wrap past bin 11; middle 11.5 for bins 11 and 0; tie to the lower bin). oc_a3 itself keeps picks first-passing; the first-passing bin and the rule's pick agree in every pick row here (see the table).
- Most tolerant: T = the minimum over the 8 directions of (largest passing offset / full range), with full ranges x and y 60 mm, heading 10 deg, pedestal 50 mm. Ties go to the first angle in the list.
- Real units come from the tolerance file (no conversion needed). Walk step: x and y 5 mm, heading 1 deg, pedestal 5 mm (part_c.py AXES). A value of 0 means the first step already failed.
- Stance of each tolerance line: rebuilt from the loop order and checked line by line against (label, task, kind, angle) (all 295 matched).
- Arrival error: 3.8 mm and 0.15 deg, from docs/reports/20261022_1319_queue_stage3.md (table row 'arrival at the approach spot', follow on) and docs/reports/20261022_1547_progress_status.md line 10 (nav_follow_sim_test, arrival 3.8 mm / 0.15 deg). It is a single staging arrival with the follower on, not a distribution; the config holds only the waypoint tolerances (tol_m 3 mm, tol_rad 2 deg, PICK_PLACE_CFG).
- Multiples: weakest of x+, x-, y+, y- divided by 3.8 mm, and of heading+, heading- divided by 0.15 deg; the row's weakest multiple is the smaller of the two. Pedestal is raw only (no arrival error is defined for it).

## Table: 46 rows (23 stances x pick and place)

| # | stance (x, y, heading, ped) | task | list | rule's pick | tie | tolerant angle | apart (deg) | angles at max T (n) | max T | x/y mm at pick (+,-) | heading deg at pick (+,-) | pedestal mm at pick (+,-) | x/y mm at tolerant (+,-) | heading deg at tolerant (+,-) | pedestal mm at tolerant (+,-) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 0.775, 1.105, -90, 0.12 (staging) | 1 pick | 0, 11 | bin 0 | tie (lower) | bin 0 | 0 | 0, 11 (2) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 2 | 0.775, 1.105, -90, 0.12 (staging) | 3 place | -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5 | -30 deg | no | -45 deg | 15 | -45, -40, -35, -30, -25, -20, -15 (7) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 3 | 0.870, 1.072, -100, 0.10 (point 4) | 1 pick | 11 | bin 11 | no | bin 11 | 0 | 11 (1) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 4 | 0.870, 1.072, -100, 0.10 (point 4) | 3 place | -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10 | -35 deg | no | -55 deg | 20 | -55, -50, -45, -40, -35 (5) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 5 | 0.870, 1.072, -100, 0.10 (candidate) | 1 pick | 11 | bin 11 | no | bin 11 | 0 | 11 (1) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 6 | 0.870, 1.072, -100, 0.10 (candidate) | 3 place | -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10 | -35 deg | no | -55 deg | 20 | -55, -50, -45, -40, -35 (5) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 7 | 0.842, 1.077, -100, 0.10 (candidate) | 1 pick | 11 | bin 11 | no | bin 11 | 0 | 11 (1) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 8 | 0.842, 1.077, -100, 0.10 (candidate) | 3 place | -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5 | -35 deg | tie (lower) | -50 deg | 15 | -50, -45, -40, -35, -30, -25, -20 (7) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 9 | 0.806, 1.072, -90, 0.10 (candidate) | 1 pick | 11 | bin 11 | no | bin 11 | 0 | 11 (1) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 10 | 0.806, 1.072, -90, 0.10 (candidate) | 3 place | -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10 | -35 deg | no | -55 deg | 20 | -55, -50, -45, -40 (4) | 1.000 | 60/60, 60/60 | 8/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 11 | 0.793, 1.074, -90, 0.10 (candidate) | 1 pick | 11 | bin 11 | no | bin 11 | 0 | 11 (1) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 12 | 0.793, 1.074, -90, 0.10 (candidate) | 3 place | -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5 | -35 deg | tie (lower) | -55 deg | 20 | -55, -50, -45, -40, -35 (5) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 13 | 0.781, 1.076, -90, 0.10 (candidate) | 1 pick | 11 | bin 11 | no | bin 11 | 0 | 11 (1) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 14 | 0.781, 1.076, -90, 0.10 (candidate) | 3 place | -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5 | -35 deg | tie (lower) | -55 deg | 20 | -55, -50, -45, -40, -35, -30, -25 (7) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 15 | 0.781, 1.051, -90, 0.15 (candidate) | 1 pick | 0, 11 | bin 0 | tie (lower) | bin 11 | 30 | 11 (1) | 1.000 | 40/60, 60/55 | 7/10 | 50/35 | 60/60, 60/60 | 10/10 | 50/50 |
| 16 | 0.781, 1.051, -90, 0.15 (candidate) | 3 place | -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5 | -35 deg | tie (lower) | -55 deg | 20 | -55, -50, -45, -40, -35, -30 (6) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 17 | 0.748, 1.032, -80, 0.15 (candidate) | 1 pick | 0, 11 | bin 0 | tie (lower) | bin 11 | 30 | 11 (1) | 1.000 | 5/60, 60/60 | 2/10 | 50/25 | 60/60, 60/60 | 10/10 | 50/50 |
| 18 | 0.748, 1.032, -80, 0.15 (candidate) | 3 place | -65, -60, -55, -50, -45, -40, -35, -30, -25, -20 | -45 deg | tie (lower) | -60 deg | 15 | -60, -55 (2) | 1.000 | 50/60, 60/60 | 8/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 19 | 0.773, 1.053, -90, 0.15 (candidate) | 1 pick | 0, 11 | bin 0 | tie (lower) | bin 11 | 30 | 11 (1) | 1.000 | 45/60, 60/60 | 10/10 | 50/40 | 60/60, 60/60 | 10/10 | 50/50 |
| 20 | 0.773, 1.053, -90, 0.15 (candidate) | 3 place | -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5 | -35 deg | tie (lower) | -55 deg | 20 | -55, -50, -45, -40, -35, -30, -25 (7) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 21 | 0.822, 1.069, -90, 0.10 (candidate) | 1 pick | 11 | bin 11 | no | bin 11 | 0 | 11 (1) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 22 | 0.822, 1.069, -90, 0.10 (candidate) | 3 place | -65, -60, -55, -50, -45, -40, -35, -30, -25, -20 | -45 deg | tie (lower) | -55 deg | 10 | -55, -50 (2) | 1.000 | 60/60, 60/60 | 8/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 23 | 0.731, 1.061, -80, 0.10 (candidate) | 1 pick | 0, 11 | bin 0 | tie (lower) | bin 11 | 30 | 11 (1) | 1.000 | 15/60, 60/15 | 2/10 | 50/15 | 60/60, 60/60 | 10/10 | 50/50 |
| 24 | 0.731, 1.061, -80, 0.10 (candidate) | 3 place | -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10 | -35 deg | no | -55 deg | 20 | -55, -50, -45, -40 (4) | 1.000 | 60/60, 60/60 | 9/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 25 | 0.718, 1.063, -80, 0.10 (candidate) | 1 pick | 11 | bin 11 | no | bin 11 | 0 | 11 (1) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 26 | 0.718, 1.063, -80, 0.10 (candidate) | 3 place | -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5 | -35 deg | tie (lower) | -55 deg | 20 | -55, -50, -45, -40, -35, -30 (6) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 27 | 0.710, 1.064, -80, 0.10 (candidate) | 1 pick | 11 | bin 11 | no | bin 11 | 0 | 11 (1) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 28 | 0.710, 1.064, -80, 0.10 (candidate) | 3 place | -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5 | -35 deg | tie (lower) | -55 deg | 20 | -55, -50, -45, -40, -35, -30, -25 (7) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 29 | 0.671, 1.012, -70, 0.15 (candidate) | 1 pick | 0, 11 | bin 0 | tie (lower) | bin 11 | 30 | 11 (1) | 0.900 | 60/60, 60/60 | 4/10 | 50/50 | 60/60, 60/60 | 10/10 | 45/50 |
| 30 | 0.671, 1.012, -70, 0.15 (candidate) | 3 place | -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10 | -35 deg | no | -55 deg | 20 | -55, -50, -45 (3) | 1.000 | 45/60, 60/60 | 7/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 31 | 0.658, 1.038, -70, 0.10 (candidate) | 1 pick | 0, 11 | bin 0 | tie (lower) | bin 11 | 30 | 11 (1) | 1.000 | 50/60, 60/60 | 4/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 32 | 0.658, 1.038, -70, 0.10 (candidate) | 3 place | -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5 | -35 deg | tie (lower) | -55 deg | 20 | -55, -50, -45, -40 (4) | 1.000 | 60/60, 60/60 | 9/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 33 | 0.654, 1.038, -70, 0.10 (candidate) | 1 pick | 0, 11 | bin 0 | tie (lower) | bin 11 | 30 | 11 (1) | 1.000 | 55/60, 60/60 | 4/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 34 | 0.654, 1.038, -70, 0.10 (candidate) | 3 place | -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5 | -35 deg | tie (lower) | -55 deg | 20 | -55, -50, -45, -40, -35 (5) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 35 | 0.613, 1.000, -60, 0.10 (candidate) | 1 pick | 11 | bin 11 | no | bin 11 | 0 | 11 (1) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 36 | 0.613, 1.000, -60, 0.10 (candidate) | 3 place | -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10 | -35 deg | no | -55 deg | 20 | -55, -50 (2) | 1.000 | 45/60, 60/60 | 7/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 37 | 0.658, 1.015, -70, 0.15 (candidate) | 1 pick | 11 | bin 11 | no | bin 11 | 0 | 11 (1) | 0.700 | 60/50, 45/60 | 10/10 | 35/50 | 60/50, 45/60 | 10/10 | 35/50 |
| 38 | 0.658, 1.015, -70, 0.15 (candidate) | 3 place | -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5 | -35 deg | tie (lower) | -55 deg | 20 | -55, -50, -45, -40, -35 (5) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 39 | 0.604, 1.001, -60, 0.10 (candidate) | 1 pick | 11 | bin 11 | no | bin 11 | 0 | 11 (1) | 0.833 | 60/55, 50/60 | 10/10 | 50/50 | 60/55, 50/60 | 10/10 | 50/50 |
| 40 | 0.604, 1.001, -60, 0.10 (candidate) | 3 place | -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10 | -35 deg | no | -55 deg | 20 | -55, -50, -45 (3) | 1.000 | 50/60, 60/60 | 8/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 41 | 0.645, 1.018, -70, 0.15 (candidate) | 1 pick | 11 | bin 11 | no | bin 11 | 0 | 11 (1) | 0.500 | 60/35, 35/60 | 10/10 | 25/50 | 60/35, 35/60 | 10/10 | 25/50 |
| 42 | 0.645, 1.018, -70, 0.15 (candidate) | 3 place | -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5 | -35 deg | tie (lower) | -50 deg | 15 | -50, -45, -40, -35, -30 (5) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 43 | 0.595, 1.002, -60, 0.10 (candidate) | 1 pick | 11 | bin 11 | no | bin 11 | 0 | 11 (1) | 0.750 | 60/45, 45/60 | 10/10 | 45/50 | 60/45, 45/60 | 10/10 | 45/50 |
| 44 | 0.595, 1.002, -60, 0.10 (candidate) | 3 place | -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5 | -35 deg | tie (lower) | -50 deg | 15 | -50, -45, -40, -35 (4) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |
| 45 | 0.591, 1.003, -60, 0.10 (candidate) | 1 pick | 11 | bin 11 | no | bin 11 | 0 | 11 (1) | 0.667 | 60/40, 40/60 | 9/10 | 40/50 | 60/40, 40/60 | 9/10 | 40/50 |
| 46 | 0.591, 1.003, -60, 0.10 (candidate) | 3 place | -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5 | -35 deg | tie (lower) | -50 deg | 15 | -50, -45, -40, -35 (4) | 1.000 | 60/60, 60/60 | 10/10 | 50/50 | 60/60, 60/60 | 10/10 | 50/50 |

## Weakest multiple of the arrival error: counts (strict below)

| rows | angle | < 2x | < 5x | < 10x | < 20x |
|---|---|---|---|---|---|
| all 46 | rule's pick | 1 | 2 | 3 | 46 |
| all 46 | most tolerant | 0 | 0 | 1 | 46 |
| place 23 | rule's pick | 0 | 0 | 0 | 23 |
| place 23 | most tolerant | 0 | 0 | 0 | 23 |
| pick 23 | rule's pick | 1 | 2 | 3 | 23 |
| pick 23 | most tolerant | 0 | 0 | 1 | 23 |

Rows where the most tolerant angle is a tie (more than one angle at the maximum T): 24 of 46. Rows where at least one angle reaches the full range in all 8 directions: 40 of 46. Where several angles are tied, 'most tolerant' above is the first of them in list order, which is not a real ranking.

Angles at the full range in all 8 directions (60 mm x and y, 10 deg heading, 50 mm pedestal), per row:

- staging pick (stance 0.775, 1.105, -90, 0.12): 2 of 2 angles at the full range in all 8 directions
- staging place (stance 0.775, 1.105, -90, 0.12): 7 of 11 angles at the full range in all 8 directions
- point 4 pick (stance 0.870, 1.072, -100, 0.10): 1 of 1 angles at the full range in all 8 directions
- point 4 place (stance 0.870, 1.072, -100, 0.10): 5 of 11 angles at the full range in all 8 directions
- candidate pick (stance 0.870, 1.072, -100, 0.10): 1 of 1 angles at the full range in all 8 directions
- candidate place (stance 0.870, 1.072, -100, 0.10): 5 of 11 angles at the full range in all 8 directions
- candidate pick (stance 0.842, 1.077, -100, 0.10): 1 of 1 angles at the full range in all 8 directions
- candidate place (stance 0.842, 1.077, -100, 0.10): 7 of 12 angles at the full range in all 8 directions
- candidate pick (stance 0.806, 1.072, -90, 0.10): 1 of 1 angles at the full range in all 8 directions
- candidate place (stance 0.806, 1.072, -90, 0.10): 4 of 11 angles at the full range in all 8 directions
- candidate pick (stance 0.793, 1.074, -90, 0.10): 1 of 1 angles at the full range in all 8 directions
- candidate place (stance 0.793, 1.074, -90, 0.10): 5 of 12 angles at the full range in all 8 directions
- candidate pick (stance 0.781, 1.076, -90, 0.10): 1 of 1 angles at the full range in all 8 directions
- candidate place (stance 0.781, 1.076, -90, 0.10): 7 of 12 angles at the full range in all 8 directions
- candidate pick (stance 0.781, 1.051, -90, 0.15): 1 of 2 angles at the full range in all 8 directions
- candidate place (stance 0.781, 1.051, -90, 0.15): 6 of 12 angles at the full range in all 8 directions
- candidate pick (stance 0.748, 1.032, -80, 0.15): 1 of 2 angles at the full range in all 8 directions
- candidate place (stance 0.748, 1.032, -80, 0.15): 2 of 10 angles at the full range in all 8 directions
- candidate pick (stance 0.773, 1.053, -90, 0.15): 1 of 2 angles at the full range in all 8 directions
- candidate place (stance 0.773, 1.053, -90, 0.15): 7 of 12 angles at the full range in all 8 directions
- candidate pick (stance 0.822, 1.069, -90, 0.10): 1 of 1 angles at the full range in all 8 directions
- candidate place (stance 0.822, 1.069, -90, 0.10): 2 of 10 angles at the full range in all 8 directions
- candidate pick (stance 0.731, 1.061, -80, 0.10): 1 of 2 angles at the full range in all 8 directions
- candidate place (stance 0.731, 1.061, -80, 0.10): 4 of 11 angles at the full range in all 8 directions
- candidate pick (stance 0.718, 1.063, -80, 0.10): 1 of 1 angles at the full range in all 8 directions
- candidate place (stance 0.718, 1.063, -80, 0.10): 6 of 12 angles at the full range in all 8 directions
- candidate pick (stance 0.710, 1.064, -80, 0.10): 1 of 1 angles at the full range in all 8 directions
- candidate place (stance 0.710, 1.064, -80, 0.10): 7 of 12 angles at the full range in all 8 directions
- candidate pick (stance 0.671, 1.012, -70, 0.15): 0 of 2 angles at the full range in all 8 directions
- candidate place (stance 0.671, 1.012, -70, 0.15): 3 of 11 angles at the full range in all 8 directions
- candidate pick (stance 0.658, 1.038, -70, 0.10): 1 of 2 angles at the full range in all 8 directions
- candidate place (stance 0.658, 1.038, -70, 0.10): 4 of 12 angles at the full range in all 8 directions
- candidate pick (stance 0.654, 1.038, -70, 0.10): 1 of 2 angles at the full range in all 8 directions
- candidate place (stance 0.654, 1.038, -70, 0.10): 5 of 12 angles at the full range in all 8 directions
- candidate pick (stance 0.613, 1.000, -60, 0.10): 1 of 1 angles at the full range in all 8 directions
- candidate place (stance 0.613, 1.000, -60, 0.10): 2 of 11 angles at the full range in all 8 directions
- candidate pick (stance 0.658, 1.015, -70, 0.15): 0 of 1 angles at the full range in all 8 directions
- candidate place (stance 0.658, 1.015, -70, 0.15): 5 of 12 angles at the full range in all 8 directions
- candidate pick (stance 0.604, 1.001, -60, 0.10): 0 of 1 angles at the full range in all 8 directions
- candidate place (stance 0.604, 1.001, -60, 0.10): 3 of 11 angles at the full range in all 8 directions
- candidate pick (stance 0.645, 1.018, -70, 0.15): 0 of 1 angles at the full range in all 8 directions
- candidate place (stance 0.645, 1.018, -70, 0.15): 5 of 12 angles at the full range in all 8 directions
- candidate pick (stance 0.595, 1.002, -60, 0.10): 0 of 1 angles at the full range in all 8 directions
- candidate place (stance 0.595, 1.002, -60, 0.10): 4 of 12 angles at the full range in all 8 directions
- candidate pick (stance 0.591, 1.003, -60, 0.10): 0 of 1 angles at the full range in all 8 directions
- candidate place (stance 0.591, 1.003, -60, 0.10): 4 of 12 angles at the full range in all 8 directions

Rows below 5x at the rule's pick:

- candidate pick (stance 0.748, 1.032, -80, 0.15), weakest 1.32x on x/y
- candidate pick (stance 0.731, 1.061, -80, 0.10), weakest 3.95x on x/y

## Wrapping-pick rows (bins 0 and 11 reported as bin 0 in the overnight file)

- staging stance (0.775, 1.105, -90, 0.12), list [0, 11]: stored window middle 0.0 (the bug); corrected middle 11.5; tie between bins 11 and 0 at 15 deg each; rule picks bin 0 (lower value).
- candidate stance (0.781, 1.051, -90, 0.15), list [0, 11]: stored window middle 0.0 (the bug); corrected middle 11.5; tie between bins 11 and 0 at 15 deg each; rule picks bin 0 (lower value).
- candidate stance (0.748, 1.032, -80, 0.15), list [0, 11]: stored window middle 0.0 (the bug); corrected middle 11.5; tie between bins 11 and 0 at 15 deg each; rule picks bin 0 (lower value).
- candidate stance (0.773, 1.053, -90, 0.15), list [0, 11]: stored window middle 0.0 (the bug); corrected middle 11.5; tie between bins 11 and 0 at 15 deg each; rule picks bin 0 (lower value).
- candidate stance (0.731, 1.061, -80, 0.10), list [0, 11]: stored window middle 0.0 (the bug); corrected middle 11.5; tie between bins 11 and 0 at 15 deg each; rule picks bin 0 (lower value).
- candidate stance (0.671, 1.012, -70, 0.15), list [0, 11]: stored window middle 0.0 (the bug); corrected middle 11.5; tie between bins 11 and 0 at 15 deg each; rule picks bin 0 (lower value).
- candidate stance (0.658, 1.038, -70, 0.10), list [0, 11]: stored window middle 0.0 (the bug); corrected middle 11.5; tie between bins 11 and 0 at 15 deg each; rule picks bin 0 (lower value).
- candidate stance (0.654, 1.038, -70, 0.10), list [0, 11]: stored window middle 0.0 (the bug); corrected middle 11.5; tie between bins 11 and 0 at 15 deg each; rule picks bin 0 (lower value).

## What ended the walks

Not saved. The tolerance record stores only the first failing offset for each direction (first_fail_mm / first_fail_deg), not which check failed (collision, no path found, joint limit). part_c.py kept only the success flag of `_solve_stance`. Not rerun, as asked.

## Correction to the overnight report

In analyze_C.py the tolerance lines were keyed by (label, task, kind, angle) with no stance. All 21 candidate stances share the label 'candidate', so their tolerance values collided and each candidate row took the value of one stance. The overnight report's 'best T' column, its 'middle to best' T distances and its T correlation are therefore wrong. With the stances rebuilt here, the Spearman correlation of T with the margin is 0.457 (n = 295), against 0.706 in the report. The walk values themselves are correct (per line, in the file). The window middle of the wrapping pick rows (0.0 in the report) should be 11.5.

## Not done

- Walk-end reasons: not saved (see above).
- The rule's pick is computed from the saved list; the list is the check's output, not rerun.
- The overnight report itself was not edited (the rules allow only this one new file); its T-based numbers need the correction above.

