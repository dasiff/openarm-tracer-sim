# Part 4: carry angle measurement, carry slice `<=`, navigation row, VLM calls, return to the zombie pose (2026-10-26)

Written for: David (decisions at the end). Nothing is pushed or squashed (last push 43dc54e).

## Summary
- **Passed:** Step 1 measured (nothing changed in src/); Step 2 done, `transit_clearance_test` 27/27, the zero-margin case blocks; Step 3 done (new row M182); Step 5 answered from code and logs; Step 6 added to the design plan.
- **Failed / stopped:** Step 4: the FIRST live VLM call returned 504 then 503 UNAVAILABLE (the client's six own retries used up). As instructed I stopped: no second call, no cache-only passes.
- **New:** M182 (navigation on by default); M180 revised (`<=`); design-plan entry "Safe return to the zombie pose" (NOT DECIDED); measurement script `experiments/place_over_angle_measure.py`.
- **Main finding of Step 1:** the full check accepts a hand turned about 11 to 12 degrees clockwise (seen from above) with a 3 to 4 degree tilt at the OVER stop, but the skill commands exactly level and straight forward. Turned clockwise by 5 to 50 degrees (moved layout) or 0 to 45 degrees (normal layout) the arm reaches the stop within the episode's tolerance and passes the full check's collision tests. At the skill's own angle the moved layout is 8.1 mm short (tolerance 3 mm); the normal layout is 1.2 mm.
- **Main finding of Step 5:** the path home is never checked in advance; the recovery's last move (joints) has no collision avoidance; in the moved-layout run the hand still held the beaker and joint 7 stayed 0.030 rad off the home command.

## Step 1: the place's OVER stop and the hand's angle (measurement only)
Script `experiments/place_over_angle_measure.py`; results `docs/reports/logs/place_over_angle_moved.json` and `..._normal.json`; logs `data/logs/place_over_angle_{moved,normal}_20261026_014314.log`. Moved layout: chosen stance (0.635, 1.135), pedestal 0.10, episode PLAN_FAILED. Normal layout: (0.755, 1.145), pedestal 0.10, episode SUCCESS. Same plan (PLAN4) for both.
How the turn sweep was done: the skill's own solver (MinkArm, its settings, collision avoidance and joint margin on) is started from the arm's state at the first tick of the OVER, with the hand's rotation turned by 0, 5, ... 355 degrees about the vertical axis through the beaker's centre (the hotplate's centre; the pad-centre target turns about the same axis, the hand stays level so the beaker stays upright) and run up to 1500 ticks (10 s of arm time). Positive turns are counter-clockwise seen from above. For each turn the final pose is checked with the full check's collision tests (arm against the robot, hand clearance at least 10 mm, arm clearance above 0 to the bench, hotplate and objects) at the chosen stance.

**a. What freedom the full check allows today (from the code, `make_constraint` in src/grasp_rules.py, `_solve_stance` in src/full_check.py).** The position of the pad centre must match within 0.5 mm. The hand's approach axis must be within 5 degrees of horizontal and the finger-closing axis within 5 degrees of horizontal (`level_deg` 5). The approach direction (azimuth about the vertical) must lie within 15 degrees either side of the centre of one of twelve 30 degree bins (`bin_half_deg` 15); a point with no required bin tries the bins in the order of the reach maps (closest to the shoulder-to-target direction first), so the OVER stop is solved in bin 0 first (it was). So today: turning about the vertical is allowed (plus or minus 15 degrees in the bin tried, any azimuth over the bins) and a tilt of up to 5 degrees of the approach axis and 5 degrees of the closing axis. The roll about the approach axis is fixed by the closing axis being horizontal (a 180 degree flip is the only other choice). It does not require the skill's angle (approach exactly along the robot's forward direction, hand exactly level).

**b. Hand angle the full check's solver found at the OVER stop vs the skill's commanded angle** (angles in the torso frame; azimuth from forward towards the left):
| | moved layout | normal layout |
|---|---|---|
| skill (the hold's pad rotation): azimuth / approach elevation / closing-axis elevation | -0.2 / 0.0 / 0.0 deg | 0.2 / 0.0 / 0.0 deg |
| full check's solution (carry point) | -10.7 / 2.7 / 4.0 deg | -12.0 / -4.0 / -1.7 deg |
| rotation between the two | 11.7 deg | 13.0 deg |
(The skill's azimuth is not exactly zero because the robot's heading is -90.2 / -89.8 degrees, not -90.) The full check's solution is turned clockwise by about 11 to 12 degrees and tilted by up to 4 degrees.

**c. Reachability at the skill's fixed angle (turn 0).** Moved layout: best position error 8.11 mm, rotation error 0.63 degrees, against the episode's waypoint tolerance 3 mm and 2 degrees: NOT within tolerance (the position is). The episode itself ended 12.2 mm and 0.8 degrees short (the simulated arm also sags). Normal layout: 1.16 mm, 0.03 degrees: within tolerance.

**d. Turning the hand about the vertical through the beaker's centre in 5 degree steps (hand level, beaker upright).** Summary: moved layout: the turns within tolerance AND passing the full check's collision tests are -50 to -5 degrees (310 to 355 in the table), all of them; the one closest to the skill's angle is -5 degrees (1.15 mm, 0.03 degrees). The skill's 0 degrees is just outside (8.1 mm). Normal layout: -45 to 0 degrees; the closest is 0 degrees itself. Every turn counter-clockwise of the skill's angle fails at once (5 degrees: 29 mm in the moved layout, 12.5 mm in the normal one) and everything beyond about +5 degrees up to -45 degrees is out by 20 to 700 mm. In the reachable turns the hand clears the furniture and objects by 62 mm or more and the arm by 64 mm or more. Full tables (turns shown as -180 to +175):

Moved layout:
| turn | solver position error (mm) | solver rotation error (deg) | within tolerance (3 mm, 2 deg) | full check's collision tests |
|---:|---:|---:|:--:|:--:|
| -175 | 424.72 | 52.44 | no | pass |
| -170 | 423.07 | 47.82 | no | pass |
| -165 | 423.30 | 42.87 | no | pass |
| -160 | 422.37 | 38.12 | no | pass |
| -155 | 417.96 | 33.92 | no | pass |
| -150 | 414.25 | 29.61 | no | pass |
| -145 | 390.97 | 28.73 | no | pass |
| -140 | 367.65 | 27.76 | no | pass |
| -135 | 344.67 | 26.65 | no | pass |
| -130 | 321.28 | 25.53 | no | pass |
| -125 | 297.29 | 24.38 | no | pass |
| -120 | 274.59 | 22.91 | no | pass |
| -115 | 252.91 | 21.24 | no | pass |
| -110 | 230.33 | 19.66 | no | pass |
| -105 | 203.70 | 18.58 | no | pass |
| -100 | 178.63 | 17.00 | no | pass |
| -95 | 159.32 | 14.64 | no | pass |
| -90 | 138.17 | 12.62 | no | pass |
| -85 | 116.82 | 10.63 | no | pass |
| -80 | 95.62 | 8.64 | no | pass |
| -75 | 74.55 | 6.65 | no | pass |
| -70 | 53.57 | 4.69 | no | pass |
| -65 | 32.67 | 2.78 | no | pass |
| -60 | 11.85 | 0.94 | no | pass |
| -55 | 4.26 | 0.32 | no | pass |
| -50 | 2.11 | 0.14 | yes | pass |
| -45 | 1.46 | 0.08 | yes | pass |
| -40 | 1.28 | 0.06 | yes | pass |
| -35 | 1.14 | 0.05 | yes | pass |
| -30 | 1.03 | 0.04 | yes | pass |
| -25 | 0.96 | 0.03 | yes | pass |
| -20 | 0.91 | 0.02 | yes | pass |
| -15 | 0.88 | 0.01 | yes | pass |
| -10 | 0.97 | 0.01 | yes | pass |
| -5 | 1.15 | 0.03 | yes | pass |
| +0 | 8.11 | 0.63 | no | pass |
| +5 | 28.99 | 2.39 | no | pass |
| +10 | 49.87 | 4.18 | no | pass |
| +15 | 70.71 | 5.97 | no | pass |
| +20 | 91.53 | 7.76 | no | pass |
| +25 | 112.34 | 9.50 | no | pass |
| +30 | 133.17 | 11.15 | no | pass |
| +35 | 154.19 | 12.56 | no | pass |
| +40 | 176.33 | 13.28 | no | pass |
| +45 | 195.19 | 13.51 | no | pass |
| +50 | 209.84 | 14.54 | no | pass |
| +55 | 225.15 | 15.57 | no | pass |
| +60 | 241.01 | 16.59 | no | pass |
| +65 | 257.32 | 17.58 | no | pass |
| +70 | 274.00 | 18.54 | no | pass |
| +75 | 290.99 | 19.43 | no | pass |
| +80 | 308.22 | 20.26 | no | pass |
| +85 | 325.65 | 21.01 | no | pass |
| +90 | 388.71 | 20.60 | no | pass |
| +95 | 397.33 | 21.77 | no | pass |
| +100 | 404.74 | 22.66 | no | pass |
| +105 | 410.21 | 24.12 | no | pass |
| +110 | 415.29 | 25.95 | no | pass |
| +115 | 417.74 | 28.54 | no | FAIL |
| +120 | 425.40 | 33.14 | no | pass |
| +125 | 443.30 | 35.06 | no | pass |
| +130 | 466.82 | 36.09 | no | pass |
| +135 | 499.47 | 35.38 | no | pass |
| +140 | 531.91 | 34.27 | no | pass |
| +145 | 563.86 | 32.74 | no | pass |
| +150 | 595.83 | 30.53 | no | pass |
| +155 | 627.42 | 27.47 | no | pass |
| +160 | 658.86 | 22.94 | no | pass |
| +165 | 691.81 | 12.91 | no | pass |
| +170 | 694.85 | 15.12 | no | pass |
| +175 | 694.53 | 19.49 | no | pass |
| +180 | 694.50 | 23.92 | no | pass |

Normal layout:
| turn | solver position error (mm) | solver rotation error (deg) | within tolerance (3 mm, 2 deg) | full check's collision tests |
|---:|---:|---:|:--:|:--:|
| -175 | 444.62 | 52.40 | no | pass |
| -170 | 442.79 | 47.81 | no | pass |
| -165 | 440.82 | 43.25 | no | pass |
| -160 | 439.94 | 38.48 | no | pass |
| -155 | 435.06 | 34.37 | no | pass |
| -150 | 431.15 | 30.12 | no | pass |
| -145 | 407.30 | 29.41 | no | pass |
| -140 | 383.77 | 28.53 | no | pass |
| -135 | 360.47 | 27.52 | no | pass |
| -130 | 336.95 | 26.45 | no | pass |
| -125 | 312.67 | 25.38 | no | pass |
| -120 | 289.17 | 24.05 | no | pass |
| -115 | 267.23 | 22.42 | no | pass |
| -110 | 243.62 | 20.99 | no | pass |
| -105 | 215.65 | 20.03 | no | pass |
| -100 | 193.77 | 18.04 | no | pass |
| -95 | 173.81 | 15.85 | no | pass |
| -90 | 152.22 | 13.89 | no | pass |
| -85 | 130.85 | 11.91 | no | pass |
| -80 | 109.64 | 9.91 | no | pass |
| -75 | 88.55 | 7.92 | no | pass |
| -70 | 67.56 | 5.95 | no | pass |
| -65 | 46.65 | 4.02 | no | pass |
| -60 | 25.83 | 2.14 | no | pass |
| -55 | 8.12 | 0.62 | no | pass |
| -50 | 3.35 | 0.24 | no | pass |
| -45 | 1.78 | 0.10 | yes | pass |
| -40 | 1.43 | 0.07 | yes | pass |
| -35 | 1.26 | 0.06 | yes | pass |
| -30 | 1.13 | 0.05 | yes | pass |
| -25 | 1.03 | 0.04 | yes | pass |
| -20 | 0.97 | 0.03 | yes | pass |
| -15 | 0.92 | 0.01 | yes | pass |
| -10 | 0.91 | 0.00 | yes | pass |
| -5 | 0.99 | 0.01 | yes | pass |
| +0 | 1.16 | 0.03 | yes | pass |
| +5 | 12.54 | 1.03 | no | pass |
| +10 | 33.58 | 2.83 | no | pass |
| +15 | 54.58 | 4.64 | no | pass |
| +20 | 75.56 | 6.46 | no | pass |
| +25 | 96.50 | 8.24 | no | pass |
| +30 | 117.42 | 9.96 | no | pass |
| +35 | 138.37 | 11.53 | no | pass |
| +40 | 159.56 | 12.79 | no | pass |
| +45 | 182.81 | 12.95 | no | pass |
| +50 | 198.01 | 13.73 | no | pass |
| +55 | 212.85 | 14.74 | no | pass |
| +60 | 228.33 | 15.76 | no | pass |
| +65 | 244.33 | 16.75 | no | pass |
| +70 | 260.75 | 17.72 | no | pass |
| +75 | 277.53 | 18.63 | no | pass |
| +80 | 342.12 | 18.87 | no | pass |
| +85 | 355.60 | 19.78 | no | pass |
| +90 | 369.11 | 20.35 | no | pass |
| +95 | 379.53 | 21.23 | no | pass |
| +100 | 388.09 | 22.02 | no | pass |
| +105 | 394.32 | 23.47 | no | pass |
| +110 | 399.08 | 25.53 | no | FAIL |
| +115 | 400.78 | 28.39 | no | FAIL |
| +120 | 401.90 | 31.42 | no | FAIL |
| +125 | 552.38 | 25.15 | no | pass |
| +130 | 451.86 | 35.98 | no | pass |
| +135 | 483.46 | 35.64 | no | pass |
| +140 | 515.13 | 34.89 | no | pass |
| +145 | 546.97 | 33.67 | no | pass |
| +150 | 578.93 | 31.86 | no | pass |
| +155 | 611.04 | 29.23 | no | pass |
| +160 | 643.66 | 25.23 | no | pass |
| +165 | 682.68 | 14.87 | no | pass |
| +170 | 689.94 | 15.43 | no | pass |
| +175 | 690.25 | 19.50 | no | pass |
| +180 | 690.21 | 23.95 | no | pass |

**e. From the existing failed run (no new episode).** From data/moved_place_split_diag/episode (the same selection and plan; its OVER failure matches the test log to the digit: 12.2 mm, 0.76 degrees) and docs/reports/logs/moved_place_split_diag.json: the OVER target was (0.650, 0.669, 1.120) m. The commanded joints (radians) at the stop: joint 2 = -0.0000 and joint 6 = 0.6109, both exactly on the solver's own bound (joint limit minus the 10 degree margin: joint 2 upper bound -0.0000003, joint 6 upper bound 0.6109); the measured joints followed within 0.04 rad; the pad ended 12.2 mm and 0.76 degrees from the target after 11 s, at rest. The same solver without the 10 degree margin reaches 0.95 mm (0.02 degrees) on that target; without hotplate/right-arm avoidance it still stops at 8.11 mm. My sweep above reproduces 8.11 mm at turn 0.

**f. Interpretation (mine, not a measured fact).** The full check and the skill disagree about the hand's angle at the OVER stop. The check found a solution because it may turn the hand about 11 degrees clockwise and tilt it up to 4 degrees; the skill insists on exactly level and straight forward, where the arm sits on its joint-margin limits (joints 2 and 6) in the moved layout and just inside the tolerance in the normal layout. The reachable turns form a one-sided sliver (clockwise), so a small clockwise turn (5 degrees or more, the step of this sweep; the true edge lies between 0 and -5 degrees in the moved layout) would make both layouts reach the stop, with 1 to 1.2 mm of error and clear collision margins. The sweep keeps the beaker upright (only a turn about the vertical); the check's own solution also tilts, which this sweep does not use. Limits of the measurement: only the OVER stop (the later waypoints DOWN, BACK and HOME keep the same hand rotation in the skill and were not swept); the solver's 10 s run from one start may stop in a local minimum for the far-away turns (the numbers beyond 30 degrees counter-clockwise should be read as unreachable by this solver, not as proven impossible); the simulated arm sags a few millimetres more than the solver predicts. Nothing in the checker or the skill was changed.

## Step 2: carry slice margin test `<=`
`carry_blocked` and `carry_blocked_mask` (src/stance_select.py) now block when the gap is `<=` the margin (was `<`): with the old test a zero margin never blocked, because the gap of an overlapping obstacle is clamped to 0 and `0 < 0` is false. The test case that I had moved to a 1 mm margin is back at zero margin and blocks (log line: obstacle "bulge", distance 0.0, limit 0.0). `transit_clearance_test` 27/27, log data/logs/transit_clearance_test_20261026_015513.log. No effect on the real 120 mm margin. Noted in M180.

## Step 3: navigation decision row
New row M182 (below). The tests and scripts that use navigation off: `experiments/stance_select_test.py` section 6 (`cfg_off`), and `experiments/held_beaker_contacts.py` with `--at-reference-stance`. Nothing else sets it (the other "navigation" entries in the tests are the orchestrator's module key). The setting's comment in src/task_config.py now points to M182 (and M175 for what navigation on and off do in stance selection).

## Step 4: live VLM calls
Command: `python experiments/mpvi_text_test.py --mode cache_or_live --checker vlm` (log data/logs/mpvi_text_vlm_live_20261026_015655.log). The planner reply came from the cache. The FIRST VLM call (the grip check, eagle camera, at the gripper_closed event, sim time 57.54 s) failed: a 504 first, then 503 UNAVAILABLE ("This model is currently experiencing high demand"), six retries of the client's own (4.9 s up to 65 s), "gave up after 7 tries". The log ended at 2026-10-26 02:05:21. The orchestrator counted it as not complete (oracle said PASS) and the plan failed at segment 2/4 (the test's own checks: 6 of 11). As instructed: no retry, no second call, no cache-only passes (there is no reply to replay). This is the 503 pattern of 2026-10-22 and 2026-10-25, not a quota error.

## Step 5: how the arm returns to the zombie pose today (read only)
Code: src/mpvi_modules.py (`plan_pick`, `plan_place`, `recover`, `step`), src/mink_arm.py, MINK_CFG and PICK_PLACE_CFG in src/task_config.py, src/control_loop.py; logs: the moved-layout run (data/logs/stance_select_moved_20261025_220439.log and data/moved_place_split_diag/episode/episode_log.csv).
1. **Where it starts.** After a scripted pick that is followed by a navigation step that moves, the last moves are UP (to the carry height, beaker held), then BACK (5 cm back along the robot's forward direction at the carry height, 0.04 m/s), then HOME. After a place: OPEN, BACK (5 cm back at the set-down height), HOME. In a recovery (a failed step): the command is reset to the measured joints, the fingers open only if the world model says the hand holds nothing, then BACK (5 cm back from wherever the hand is, hand level), then HOME.
2. **How it travels.** Three pieces in a row. (i) BACK: a straight pad-centre line at 0.04 m/s, smooth (minimum-jerk), at least 1 s. (ii) HOME (move): a straight pad-centre line to the pad pose the zombie pose has, at 0.12 m/s with the hand turning at most 0.5 rad/s, smooth, executed tick by tick through mink's inverse kinematics (the posture task pulls the joints towards the zombie pose); joint speed limit 0.5 rad/s, joints kept 10 degrees from their limits. (iii) HOME (joints): a smooth joint-space interpolation straight from the current command to the zombie pose, lasting at least 1 s and at most the largest joint distance divided by 0.3 rad/s; it ends when every joint is within 0.02 rad and slower than 0.02 rad/s for 0.2 s; timeout 3 times the planned time plus 6 s (9.0 s for a 1 s move). So: a waypoint series ending in one joint move.
3. **Checked before moving.** Nothing about the path. The zombie pose itself is checked when a stance is chosen (home clearance of the arms at home against the furniture; the full check), not on the way. In particular BACK, the HOME move and the joint interpolation are not checked in advance.
4. **Checked during the move.** Steps (i) and (ii) run through mink with: joint limits with the 10 degree margin, the joint speed limit, and collision avoidance of the left arm's links 3 to 7, hand and fingers against the bench and hotplate (kept 8 mm away) and the right arm (kept 40 mm away), detection distance 5 cm, gain 0.85. When such a limit blocks, mink just allows no motion towards it (the arm slows or stops or slides); if no solution at all exists it raises and the step fails ("mink found no solution"); if the waypoint is not reached in time it fails ("not reached after ..."). A failed recovery is not retried and there is no operator, so the plan stops. Step (iii), the joint move, has no collision avoidance, no limit and no speed check beyond its own 0.3 rad/s interpolation. The geo_kin safety filter (src/safety_filter.py) is not on this path: only the teleop code and experiments use it. The control loop only clips the targets to the actuators' range.
5. **Held object.** Not accounted for in any of these checks: the beaker is not among the geometries mink avoids and it is not in the full check. The recovery keeps the grip when the hand holds something (it does not open) and carries the object home; the Cartesian steps correct the arm's sag under its weight (an integral of the measured pad error), the joint step (iii) does not.
6. **The moved-layout recovery: what stopped it.** Not the safety filter (not on the path), not mink (the last step is the plain joint interpolation) and not a joint limit (joint 7 was at 0.258 rad, far from its limits). The command had reached the zombie pose exactly (cmd = home on all joints); the measured joints stopped at rest with joint 7 0.030 rad (1.7 degrees) off its command and the other joints within 0.012 rad, and the settle test needs 0.02 rad, so it timed out after 9.0 s. At that moment the hand still held the beaker (command: gripper closed, fingers 37.5 mm apart, beaker at (0.783, 0.828, 1.021) near the zombie pose). My reading (an interpretation): the simulator's gravity compensation does not carry the beaker's weight, so the wrist joint sags a steady 0.03 rad under it, and the joint-move step has no sag correction. I did not test this; one run with the same recovery after the beaker is released would tell.

## Step 6: design plan
Added under the "Before B4" list in docs/design_plan.md: "Safe return to the zombie pose [NOT DECIDED, for discussion with David; added 2026-10-26]" with the problem and your sketch word for word, plus the related failure above.

## Decisions log rows (new or changed this part, pasted word for word)
Changed (Step 2 added the "Revised 2026-10-26" sentences at the end of M180):

| M180 | 2026-10-25 | The carry check is a pie slice, not a line (David, Oct 25, replaces the line check of M176). Shape: seen from above, a circular sector with its apex at the carrying arm's shoulder, its two straight edges toward the hand at the two grasp points, and its far edge an arc around the shoulder with the larger shoulder-to-hand distance as radius (`slice_distance` in `src/stance_select.py`). Margin: its own setting `BENCH_STANCE_CFG["carry_slice_margin_m"]` = 120 mm; `obstacle_margin_m` and `arm_radius_m` are unchanged and not used by the slice. Height rule: a cylinder blocks the carry only when its top is above the carried beaker's bottom (`carry_bottom_z`: the higher surface under the grasp points plus `carry_gap_m`, and at least 5 cm above the lower one); the two steps' own targets are not obstacles. Filter before ranking: for a run of k >= 2 the slice check is applied to every candidate spot of the overlay (`carry_blocked_mask`, vectorized, like the other quick filters), blocked spots are removed, the rest are ranked with the usual spacing rule (`rank_keep`, `rank_min_distance_m`); k drops to k-1 only if no spot survives. The same check limits the steps a current stance serves with no navigation. Time added: 0.5 ms for 41800 spots (transit_clearance_test). Measurements it rests on (carry_slice_check.py replaying the two recorded place episodes of data/held_beaker, window pick UP to place OVER; largest distance outside the slice without margin; TASK_CFG stance / overlay's pick): hand and beaker centre 10 mm or less, beaker bottom rim 45 mm, forearm (elbow to wrist line) 105 mm / 87 mm; the hand path is a straight line (within 14 mm of the chord), so the arc does not bulge to it. 120 mm = the forearm's 105 mm plus a buffer; the arm's body is thicker than its centre line. Logging: every episode that carries an object logs the largest distance outside the slice (no margin) for the hand, the beaker's bottom rim and the forearm (`make_carry_slice_monitor`, `src/mpvi_monitors.py`) and warns when the forearm exceeds 120 mm. Navigation stays ON by default (`BENCH_STANCE_CFG["navigation"]`); off is kept for tests (stance_select_test section 6, the held-beaker script's --at-reference-stance). Revised 2026-10-26 (David, Part 4 step 2): the margin test is "gap <= margin" (was "<"), in `carry_blocked` and `carry_blocked_mask`, so an obstacle that overlaps the slice (gap 0) blocks even with a zero margin; with "<" a zero margin never blocked anything. No effect on the real 120 mm margin. The zero-margin test case of transit_clearance_test is restored (27/27, log data/logs/transit_clearance_test_20261026_015513.log). | David 2026-10-25 |

New (Step 3):

| M182 | 2026-10-26 | Navigation is ON by default, and the navigation-off option is kept for future tests (David, Oct 25; row written 2026-10-26, the decision had no row of its own before: it was only the last sentence of M180). The setting is `BENCH_STANCE_CFG["navigation"]` (src/task_config.py), default True: the robot may be driven to the stance the algorithm chooses, and what that means in stance selection is M175. Off (False): the robot stays at TASK_CFG's reference stance; a plan that stance does not serve stops with "needs navigation on" and names both stances (src/stance_select.py). Nothing in the pipeline's default path uses off. The only places that set it off today: `experiments/stance_select_test.py` section 6 (`cfg_off`, the "needs navigation on" stop) and `experiments/held_beaker_contacts.py` with `--at-reference-stance` (the held-beaker contact run at TASK_CFG's stance; it sets `BENCH_STANCE_CFG["navigation"] = False`). No other test or script sets it; the test suite's other `navigation` entries are the module key of the orchestrator, not this setting. | David 2026-10-25 |
