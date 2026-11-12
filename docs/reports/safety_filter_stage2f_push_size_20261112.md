# Stage 2f: how big are the pushes behind the joint 6 and joint 7 holds, and why (diagnosis only)

Date 2026-11-12. Branch safety-filter. Measurement only: nothing in src/safety_filter/ and no config changed, no fix chosen, nothing pushed.
Scripts: scripts/safety_filter_push_size_replay.py (replay, anatomy per sweep or round, scale test; step 1 to 3), scripts/safety_filter_push_size_tables.py (step 4 tables; written by a subagent from the step 4 spec, reviewed and run by the main session), scripts/safety_filter_push_size_evidence.py (the numbers behind H1 to H6 and the worked examples, added so they can be reproduced).
Results (git-ignored): data/safety_filter/push_size_replay.json (118 MB), data/safety_filter/push_size_tables.json. Logs: data/logs/s2f_full_20261112_183731.log (replay), data/logs/s2f_tables_full_20261112_190229.log (tables), data/logs/s2f_evidence4_20261112_190547.log (evidence).

## The answer in one paragraph

Every solver-output branch-guard hold that names joint 6 (row 6: 1461; row 4: 10) or joint 7 (row 4: 241) was replayed, and every replay matched the Stage 2c trace at every tick. The pushes are large: in row 6 the wrist is pushed a median 424 mm from the guard-shortened pose (510 mm from the raw command), and the elbow 141 mm (193 mm from raw). Two things make them large. First, the push sums the contributions of many contacts (median 55 per sweep, aligned, so they add): the summed wrist contribution is 19.5 times the largest single one. Second, and new in this stage, the held pose is already 0.28 rad (row 6) to 0.62 rad (row 4) away from the commanded pose before any push: the branch guard (0.25 rad from the last filtered command) would reject the unpushed guard-shortened command in 1426 of 1461 row 6 holds and in all 241 row 4 joint 7 holds. The arm is held in 97% of the stream, so each hold is a continuation of the one before. The pushes then add to this: in row 6 they raise the joint 6 change from 0.28 to 1.52 rad (median) and overshoot the joint 6 limit by 0.25 rad; in row 4 they raise the joint 7 change from 0.62 to 2.51 rad and overshoot joint 7's limit by 2.1 rad. Row 6 passes the branch guard only at 25% or 10% of its push (none at 50% or more), and row 4 at any tested scale in only 4 of 251 holds. The 195 mm / 480 mm example of tick 48 holds in part: it reproduces from the raw command, but from the guard-shortened pose the push is 145 mm (elbow) and 404 mm (wrist).

## 1. Hold counts (table a)

Solver-output branch-guard holds in the full Stage 2c stream (1576 ticks), by row, arm and named joint. Replayed = all of them.

| row (mode) | arm | named joint | target | holds at solver output | replayed |
|---|---|---|---|---|---|
| 6 (kong_style) | left | 6 | yes | 797 | 797 |
| 6 (kong_style) | right | 6 | yes | 664 | 664 |
| 6 | all | other joints (4, 5 left; 3, 4, 5 right) | no | 69 | 0 (not targets) |
| 4 (layered) | left | 6 | yes | 7 | 7 |
| 4 | right | 6 | yes | 3 | 3 |
| 4 | left | 7 | yes | 4 | 4 |
| 4 | right | 7 | yes | 237 | 237 |
| 4 | all | other joints (1, 3, 5) | no | 232 | 0 (not targets) |

Row 6 has 1461 targets (joint 6) and all 1461 were replayed. Row 4 has 251 targets (10 joint 6 and 241 joint 7) and all 251 were replayed. Row 4 also has 232 further solver-output holds (joints 1, 3, 5), 133 branch-guard holds at the jacobian output, and 17 joint-limit holds (616 branch-guard holds in total); these are outside this stage's scope and were not replayed.

The 1461 row 6 holds start at tick 25 and run to the end of the stream: 1530 holds in 1576 ticks (97.1%). Row 4: 1529 holds in 1576 ticks (97.0%). Each hold is therefore a continuation of an episode that began early in the stream.

## 2. Push sizes from the three starting points (table b)

Displacement (mm) of the elbow (E), wrist (W) and hand reference point (H) from the final pushed pose back to each start. Start raw = the stream's raw command keypoints; ii = the pose the pushes start from (row 6: after the jump-through guard; row 4: the raw command, because the layered resolver has no guard and starts its rounds from it, so ii equals raw); last = the last filtered command keypoints.

| group | start | point | n | median | p10 | p90 | max |
|---|---|---|---|---|---|---|---|
| row 6 joint 6 | raw | E | 1461 | 192.9 | 161.8 | 215.6 | 224.0 |
| row 6 joint 6 | raw | W = H | 1461 | 510.3 | 494.4 | 534.3 | 542.0 |
| row 6 joint 6 | ii | E | 1461 | 141.1 | 122.8 | 181.4 | 190.4 |
| row 6 joint 6 | ii | W = H | 1461 | 423.8 | 404.0 | 483.8 | 490.0 |
| row 6 joint 6 | last | E | 1461 | 102.1 | 95.4 | 140.1 | 147.9 |
| row 6 joint 6 | last | W = H | 1461 | 363.7 | 340.8 | 427.1 | 442.6 |
| row 4 joint 6 | raw = ii | E | 10 | 157.1 | 67.4 | 171.0 | 172.5 |
| row 4 joint 6 | raw = ii | W = H | 10 | 85.2 | 35.1 | 91.0 | 91.8 |
| row 4 joint 6 | last | E | 10 | 70.7 | 57.5 | 85.4 | 86.4 |
| row 4 joint 6 | last | W = H | 10 | 7.2 | 4.3 | 28.6 | 31.4 |
| row 4 joint 7 | raw = ii | E | 241 | 287.6 | 286.8 | 288.5 | 327.9 |
| row 4 joint 7 | raw = ii | W = H | 241 | 166.3 | 164.0 | 170.5 | 223.2 |
| row 4 joint 7 | last | E | 241 | 261.0 | 257.8 | 263.4 | 289.3 |
| row 4 joint 7 | last | W = H | 241 | 65.3 | 63.4 | 66.3 | 92.4 |

What it means: the push is smaller from the guard-shortened pose than from the raw command (row 6 wrist 424 vs 510 mm median), because the jump-through guard had already shortened the step; and smaller again from the last filtered command, the start the branch guard compares against. The hand reference point and the wrist point are the same point in our keypoints: the hand-to-wrist distance is 0.00 mm in all 1712 holds (the link 7 body origin and the joint 7 anchor coincide to that precision), so the H columns are copies of W. The solver is given directions only, so this point is not separately visible to it.

## 3. Ratio tables (table c)

(c1) Row 6 (kong_style), per sweep and point, holding arm: summed contribution before restore divided by the largest single contribution.

| point | quantity | n | median | p10 | p90 | max |
|---|---|---|---|---|---|---|
| E | sum / largest (vector) | 1461 | 3.47 | 2.46 | 5.97 | 7.54 |
| W | sum / largest (vector) | 1461 | 19.52 | 13.35 | 32.86 | 40.41 |
| H | sum / largest (vector) | 1461 | 15.11 | 9.59 | 22.08 | 22.79 |

Summed and largest contributions in mm (medians): elbow sum 191.1 (largest single 58.3); wrist sum 1071.2 (largest single 51.5); hand 561.8 (38.1). The wrist sum has a median of 55 contributions per sweep (p10 45, p90 62).

What it means: contributions add up, and they mostly point the same way (vector sum divided by magnitude sum is 0.91, median). The wrist receives about 20 times the push of its largest single contact.

(c2) Row 4 (layered): sum over rounds divided by the largest single round, holding arm.

| point | quantity | n | median | p10 | p90 | max |
|---|---|---|---|---|---|---|
| E | sum / largest (vector) | 251 | 2.44 | 2.29 | 2.54 | 4.06 |
| W | sum / largest (vector) | 251 | 1.53 | 1.49 | 1.54 | 1.67 |

(c3) Tau effect, row 6: the upper-arm contributions with tau divided out (as the resolver does) against undivided.

| quantity | n | median | p10 | p90 | max |
|---|---|---|---|---|---|
| sum divided / sum undivided, per sweep | 1461 | 1.57 | 1.33 | 1.67 | 1.73 |
| factor 1/tau, per upper-arm contribution | 6160 | 1.41 | 1.33 | 3.40 | 3.40 |
| tau, per upper-arm contribution | 6160 | 0.71 | 0.30 | 0.75 | 0.76 |

What it means: the lever division multiplies upper-arm contributions by 1.3 to 3.4. About a third of them (2020 of 6160) sit at tau 0.295 and carry the maximum factor of 3.4. No contact reaches the 5-fold bound the minimum lever fraction (0.2) would allow, because none is closer to the shoulder than 0.295 of the upper arm in these holds.

(c4) Sweep 1 versus final (displacement after the first sweep or round divided by the final displacement).

| row | point | n | median | p10 | p90 | min | max |
|---|---|---|---|---|---|---|---|
| 6 | E, W, H | 1461 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |
| 4 | E | 251 | 0.377 | 0.374 | 0.379 | 0.246 | 0.563 |
| 4 | W, H | 251 | 0.000 | 0.000 | 0.000 | 0.000 | 0.784 |

What it means: every row 6 hold ends after one sweep, so the first sweep is the whole push. In row 4 most of the push comes after the first round (the elbow moves 38% in round 1, the wrist almost none).

## 4. Last joint 6 within 0.05 rad of its limit (table d)

| group | holds | within 0.05 rad | share |
|---|---|---|---|
| row 6, joint 6 holds | 1461 | 0 | 0.0% |
| row 4, all holds | 251 | 11 | 4.4% |
| row 4, joint 6 holds | 10 | 7 | 70% |
| row 4, joint 7 holds (joint 7 distance) | 241 | 0 | 0.0% |

The stored flag agrees with the recomputed distance in every hold (0 disagreements). Row 4's joint 7 holds: joint 7 is 1.24 to 1.34 rad from its limit at the start. The 4 joint 7 holds that start with joint 6 at its limit are not near the joint 7 limit.

What it means: the row 6 joint 6 holds start 0.23 to 0.31 rad inside the limit, so they are not at the limit when they start; the push takes them over. In row 4, 7 of the 10 joint 6 holds start at the limit.

## 5. Largest passing push (table e)

The largest tested scale (1.0, 0.5, 0.25, 0.1 or 0.05) at which the holding arm's output is inside all its joint limits and the branch guard passes; displacement from ii at that scale. The scale 0.0 control is not part of the pass test.

| group | holds | with a passing scale | none | E median (p10, p90, max) mm | W median (p10, p90, max) mm |
|---|---|---|---|---|---|
| row 6 (all) | 1461 | 1461 | 0 | 33.5 (12.0, 41.6, 46.5) | 98.8 (35.6, 110.1, 114.3) |
| row 4 (all) | 251 | 4 | 247 | 49.5 (35.2, 103.5, 121.6) (n = 4) | 25.9 (18.0, 94.3, 120.7) (n = 4) |

Passing scale counts: row 6: 0.25 in 823 holds, 0.10 in 638; never 1.0 or 0.5 (every row 6 hold fails at full push, 1430 of them with joint 6 past its limit). Row 4: 1.0 in 1 hold, 0.5 in 3, none below.

What it means: the largest push a row 6 hold can take is a quarter of its full push (about 33 mm of elbow and 99 mm of wrist, median), and that is only because the push pulls the joint 6 output back toward the last value. Row 4 has almost no passing push at all.

## 6. Pairs behind the largest pushes (table f)

Top decile of holds by their ii push (row 6: wrist at or above 483.8 mm; row 4: 288.5 mm). Pair counts are per closing arm (each pair appears once for each arm that closes on it), so the holds column is the number to read.

Row 6, the 10 most frequent pairs (all 147 top-decile holds contain each of them):

| rank | pair | categories | pair type |
|---|---|---|---|
| 1 | left_finger_left + right_finger_left | hand, hand | arm vs arm |
| 2 | left_finger_left + right_finger_right | hand, hand | arm vs arm |
| 3 | left_finger_left + right_link5_2 | hand, forearm | arm vs arm |
| 4 | left_finger_left + right_link6 | hand, forearm | arm vs arm |
| 5 | left_finger_left + right_link7_1 | hand, hand | arm vs arm |
| 6 | left_finger_left + right_link7_2 | hand, hand | arm vs arm |
| 7 | left_finger_left + right_palm_2 | hand, hand | arm vs arm |
| 8 | left_finger_right + right_finger_right | hand, hand | arm vs arm |
| 9 | left_finger_right + right_link5_2 | hand, forearm | arm vs arm |
| 10 | left_finger_right + right_link6 | hand, forearm | arm vs arm |

Row 4, top decile (26 holds): left_palm_2 + right_finger_right (23 holds, hand, hand) and left_palm_2 + right_palm_2 (3 holds, hand, hand).

What it means: the largest row 6 pushes are the left hand against the right arm's fingers, palm and lower links, in the same tick. Every one of them is a hand-to-hand or hand-to-forearm contact between the two arms; the torso and the upper arm are not among the top pairs. In row 4 the palm of the left hand is pushed against the right arm's fingers and palm.

## 7. Why the holds happen: the push or the held pose (not in the brief; added because the zero-push control showed it)

Control (scale 0.0): the solver alone on the post-guard pose (row 6) or the raw command (row 4), no push. The round trip is exact (joint error 0.0000 rad, keypoint error 0.00 mm at ii, raw and last in all holds), so the control is the start pose itself. Values are the largest joint change from the last filtered command, in rad.

| row, named joint | holds | zero push, median | zero push fails the 0.25 rad guard | full push (scale 1.0), median | full push output past the named joint's limit, median | scale 0.25, median |
|---|---|---|---|---|---|---|
| 6, joint 6 | 1461 | 0.284 | 1426 | 1.524 | 0.251 | 0.239 |
| 4, joint 6 | 10 | 0.455 | 7 | 0.312 | 0.311 | 0.291 |
| 4, joint 7 | 241 | 0.621 | 241 | 2.510 | 2.120 | 0.670 |

What it means: most row 6 holds are already over the guard before any push, because the held pose and the guard-shortened command are 0.28 rad apart (the arm has been held for a long time while the command moved on). The full push then multiplies the joint change by about 5 and overshoots the joint 6 limit. Row 4's joint 7 holds are different: the raw command is 0.62 rad from the held pose, and the full push flips joint 7 by 2.5 rad. Row 4's joint 6 holds are dominated by the held-pose distance (0.46 rad, against 0.31 at full push).

## 8. Hypotheses

Each hypothesis from the brief, with the evidence. "Supported", "contradicted" or "cannot tell".

- **H1 (pushes from several pairs add up; the sum is much larger than the largest single push): supported.** Row 6 wrist: the summed contribution per sweep is 19.5 times the largest single one (median; p10 13.4, p90 32.9, max 40.4) with 55 contributions per sweep, and the contributions are aligned (0.91). After the length restore the wrist still moves 424 mm (ii median), 8 times the largest single contribution (51.5 mm). Row 4: summed over rounds 1.5 times the largest round for the wrist, 2.4 for the elbow.
- **H2 (upper-arm pushes are divided by tau, up to 5 times): partly supported.** The division is real: upper-arm contributions are multiplied by 1.3 to 3.4 (median 1.41), and a sweep's upper-arm sum is 1.57 times the undivided sum (median, max 1.73). The 5-fold bound is never reached (max 3.4). The division applies only to the elbow; the wrist push, which is the large one, comes from forearm and hand contacts that are not divided.
- **H3 (the push is large because the pair is deeply violated at the raw command): contradicted in row 6, supported in row 4.** Row 6: at the raw command the first-sweep pairs have a median gap of +39 mm; only 12108 of 85373 pair rows (14%) are below zero, and the correlation between the deepest gap and the wrist push is +0.33 (deeper gaps go with smaller pushes). Row 4: the raw command is 55 mm inside the torso pair in 248 of 251 holds (median gap −55 mm, shortfall 105 mm), and the correlation is −0.63 (deeper goes with larger push). So the raw stream does collide by construction in row 4, but not in row 6.
- **H4 (pushes grow from sweep to sweep instead of settling): contradicted in row 6, partly supported in row 4.** Row 6 never runs a second sweep (1461 of 1461 holds end after one sweep), so there is no growth to see. Row 4 does not settle quickly: the worst shortfall at round entry stays about 105 mm for rounds 0 to 2 (medians 105.2, 103.8, 105.8 mm), then falls to 95, 88, 74 and 72 mm by round 6. Six holds reach the 10-round cap. The per-round elbow motion does not grow; it changes tool from round to round.
- **H5 (the start is at or near joint 6's limit): contradicted in row 6, supported in row 4 joint 6, contradicted for joint 7.** Row 6: the last joint 6 is 0.23 to 0.31 rad from its limit in all 1461 holds (none within 0.05), and the full push takes the output past the limit (1430 holds). Row 4 joint 6: 7 of 10 holds start within 0.05 rad (median 0.001 rad). Row 4 joint 7: 1.24 to 1.34 rad from its limit; the joint 7 start is not near the limit.
- **H6 (the push is measured differently from what the solver receives, for example counted after the link-length restore): supported.** The restore changes the push the solver receives: the elbow is 1.35 times smaller after the restore (median; p10 1.22, p90 1.91) and the wrist 2.47 times smaller (median; p10 1.68, p90 3.66). The solver output then reaches the restored points exactly (planned against achieved 0.000 mm, median and max), and our replication of the plan matches it to 0.000 mm. So the plan's own sums (592 mm of wrist push at tick 48) are not the displacement that reaches the solver (404 mm). The hand point adds no separate measurement: it is the wrist point.

Two things the brief did not list. (1) The held pose lags the command (section 7): this is why most holds happen at all. (2) The hand reference point equals the wrist point in our keypoints.

## 9. Tick 48, the 195 mm / 480 mm example

Row 6, tick 48, left arm, joint 6 (last filtered 0.553 rad, 0.233 rad from the limit):

| start | elbow (mm) | wrist (mm) |
|---|---|---|
| raw command (as Stage 2d, 2e) | 194.5 | 479.7 |
| guard-shortened pose (ii; the pose the pushes start from; the guard had kept 46% of the step) | 145.0 | 403.7 |
| last filtered command | 103.1 | 347.4 |

The solver output for joint 6 is −0.822 rad, 0.036 rad past its limit (0.785), so the joint 6 result of Stage 2d and 2e reproduces. Measured from the guard-shortened pose the push is 25% smaller at the elbow and 16% smaller at the wrist. The solver receives the restored points: 145 mm (elbow) and 404 mm (wrist) from ii. The summed contributions before the restore were 200 mm (elbow) and 592 mm (wrist). The "longer than the arm" argument does not show an error on its own: the arm's two links are 218 mm each, and a point can move up to twice that (across the shoulder), so a 404 to 480 mm wrist displacement is possible geometrically. At 50% of the push joint 6 is −0.067 rad and the guard fails; at 25% it is 0.331 rad and the guard passes.

## 10. Three worked examples (row 6, typical holds)

- **Small push: row 6, tick 1527, right arm, joint 6** (smallest wrist push from ii, 389 mm). Last joint 6 −0.477 rad (0.309 rad from its limit). Pushes: elbow 158 / 127 / 99 mm and wrist 457 / 389 / 323 mm (raw / ii / last). Zero push already moves 0.283 rad (guard fails). Full push: joint 6 0.848 rad (0.063 past the limit). Scale 0.25 passes (joint 6 −0.312 rad). The largest contributions: right_link5_1 vs torso (gap at raw −44.8 mm, shortfall 50.2 mm, wrist contribution 57.0 mm), left_palm_2 vs right_palm_2 (gap −23.2 mm, shortfall 44.6 mm, 44.6 mm), left_palm_2 vs right_finger_right (gap −0.6 mm, shortfall 41.6 mm, 41.6 mm).
- **Median push: row 6, tick 807, left arm, joint 6** (wrist from ii 424 mm, the median). Last joint 6 0.553 rad (0.233 from the limit). Pushes: elbow 194 / 142 / 103 mm, wrist 514 / 424 / 365 mm (raw / ii / last). Zero push 0.277 rad (guard fails). Full push: joint 6 −0.954 rad (0.169 past the limit). Scale 0.25 passes (0.337 rad). Largest contributions: left_link5_1 vs torso (gap at raw −66.5 mm, shortfall 47.5 mm), left_link5_2 vs torso (−30.5 mm, 39.1 mm), left_link7_2 vs torso (+12.9 mm, 33.7 mm).
- **Largest push: row 6, tick 1161, left arm, joint 6** (wrist from ii 490 mm, the maximum). Last joint 6 0.553 rad. Pushes: elbow 211 / 187 / 143 mm, wrist 523 / 490 / 434 mm. Zero push 0.287 rad (guard fails). Full push: joint 6 −1.242 rad (0.456 past the limit). Scale 0.25 fails (joint 6 0.308 rad), scale 0.1 passes (0.516 rad). Largest contributions: left_link5_1 vs torso (gap at raw −30.4 mm, shortfall 47.6 mm), left_palm_2 vs right_palm_2 (gap −56.7 mm, shortfall 90.4 mm).

## 11. Options for David (none implemented)

1. Compare the branch guard with the commanded step (the change from the raw command or per tick) instead of the held pose: stops most holds, which are caused by the held pose lagging the command; costs weakening the check that keeps the arm near its last pose, a safety-relevant decision.
2. Let the jump-through guard also shorten the step until the joint change from the last filtered command is 0.25 rad or less: the shortened command then passes the branch guard without any push; costs slower tracking, since the arm already lags the stream by 0.3 to 0.6 rad.
3. Take the largest contribution per point instead of the sum over pairs (H1): cuts the wrist push about 20-fold per sweep; costs the gap rule's guarantee, since a pair may stay penetrating, and it needs a decision on what the push means.
4. Compute the push after the length restore (or restore each pair's share): removes the 1.35 times (elbow) and 2.5 times (wrist) overstatement of H6; costs a rewrite of the sharing step and a re-check that the restored push still meets the gap rule.
5. Cap the lever factor 1/tau (for example at 2), or raise min_lever_fraction: limits the elbow amplification of H2 (factor up to 3.4); costs more step shortening on contacts near the shoulder.
6. Clip the solver's joint 6 and joint 7 output to the joint limits before the branch guard: stops the output overshoot (median 0.25 rad for joint 6, 2.1 rad for joint 7 at full push); costs a decision that clipping is acceptable (the solver's own limit option is off).
7. Give the hand reference point its own position, or drop it from the keypoints: the current hand point is the wrist point, so hand contacts are counted at one place; costs a decision on what the hand point is for (the accuracy check uses it).
8. Reject or shorten a raw command that already penetrates a pair before pushing (row 4's torso pair is 55 mm inside in 248 of 251 holds): removes the deep-penetration push of H3 in row 4; costs a command-side rule and possibly a stopped stream.
9. After a hold, aim the raw command at the held pose (Stage 2c option 9): breaks the self-sustaining hold of section 7; costs an interface change to the command stream.
10. Block layered elbow swings that would move joint 7 past its limit (row 4 joint 7 output 2.1 rad past the limit at full push): costs a rule and a test in the layered resolver.

## 12. Not built, or built only in part

- The scale test is one conversion of the scaled push (restore, one solver call, one Jacobian step), not a re-run of the resolver. For row 6 the resolver's sweep structure is therefore not repeated at each scale; for row 4 the elbow swings and hand slides are not re-run either.
- Five scales only (1.0, 0.5, 0.25, 0.1, 0.05) and the zero control. No finer search; the largest passing push is the largest of these five.
- Per-tool sizes in row 4 are recorded per round (elbow swing, hand slide, step shortening), but a tool's effect on joints separately from the others was not measured.
- Pair types: the replay has no world objects, so only arm-vs-arm and arm-vs-body pairs appear. Held-object and world pairs are not in this replay.
- Row 4's 17 joint-limit holds and 233 jacobian-output branch-guard holds were not replayed (outside the brief).
- Only hold ticks are analysed. The replay runs every tick from tick 0 and matches the Stage 2c trace at every tick, but pushes at non-hold ticks are not recorded in the JSON.
- The tables (scripts/safety_filter_push_size_tables.py) were written by a subagent from the step 4 spec and reviewed by the main session. The main session recomputed table (b) medians, (c1) ratios, (d) counts and (e) counts from the raw JSON and they agree. Pair counts in (f) count each pair once per closing arm (294 = 2 × 147 holds), so the holds column is the one to read.

## 13. Decisions needed (David)

- Should the branch guard compare with the held pose (as now), or with the commanded step? This decides whether most holds happen at all (option 1, 2).
- Should the push sum the contributions of all pairs, or take the largest per point? This changes the push size about 20-fold (option 3).
- Should pushes be computed before or after the length restore? This changes the elbow by 1.35 times and the wrist by 2.5 times (option 4).
- Should joint limits be enforced by clipping the solver output (option 6), and what should happen to the joint 7 flips in row 4 (option 10)?
- Is the hand reference point meant to be separate from the wrist point (option 7)?
- Should the raw stream be allowed to penetrate pairs (row 4, option 8)? This is a stream design question.

## Progress

Progress entry added to docs/progress.md at the end of the task (step, commits, and the replay and table log paths).
