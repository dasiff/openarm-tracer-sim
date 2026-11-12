# Stage 2c: where the time goes and why the resolvers hold (diagnosis only)

Date 2026-11-12. Branch safety-filter. Measurement only: nothing in src/safety_filter/ was changed, no fix chosen, nothing pushed.
Rows 4, 5, 6 are those of the Stage 2b report (docs/reports/safety_filter_stage2b_comparison_20261112.md); same rolling punch stream, same configs.
**The comparison world contains no objects**, so the object, fixed-world and held-object counts are zero by construction.
Scripts: scripts/safety_filter_diag_common.py, safety_filter_pair_counts.py, safety_filter_time_profile.py (written by a subagent), safety_filter_hold_causes.py, safety_filter_mild_stream.py.
Results (git-ignored): data/safety_filter/pair_counts_s2c.json, time_profile_s2c.json, hold_causes_row{4,5,6}.json, mild_stream_s2c.json. Logs: data/logs/20261112_151845_s2c_*.log.
Check against Stage 2b: the full-stream hold counts and reason counts of the three rows came out identical to the Stage 2b report (row 4: 1529 = ACCURACY 651, BRANCH_GUARD 616, SHORTEN_ZERO 245, JOINT_LIMITS 17; row 5: 1487 SHORTEN_ZERO; row 6: 1530 BRANCH_GUARD), and with the timers on every output matched the timers-off output on all 400 ticks of all rows.

## 1. Pair counts (first 400 ticks)

Definitions: all possible = pairs the touch rules do not exempt for a structural reason, never-close list off; after never-close = same with the list on (the list is built for our shapes, so row 5's Kong capsules lose none); after broad phase = pairs the resolver's evaluate_pairs computes a gap for; active = of those, gap inside the pair's activation distance (margin + 1 cm) or closer. "Active" is my reading of "inside the slowing band" (the gap rule slows every pair above its margin, so there is no other cut-off).

**Counted at the filtered command** (mean / max per tick)

| row | pair type | all possible | after never-close | after broad phase | active |
|---|---|---|---|---|---|
| 4 layered, ours | arm vs arm | 271.0 / 271 | 245.0 / 245 | 1.9 / 2 | 0.0 / 0 |
| 4 layered, ours | arm vs body | 394.0 / 394 | 273.0 / 273 | 4.8 / 5 | 1.0 / 1 |
| 4 layered, ours | total | 665.0 / 665 | 518.0 / 518 | 6.6 / 7 | 1.0 / 1 |
| 5 kong_style, Kong radii | arm vs arm | 9.0 / 9 | 9.0 / 9 | 0.9 / 1 | 0.9 / 1 |
| 5 kong_style, Kong radii | arm vs body | 0.0 / 0 | 0.0 / 0 | 0.0 / 0 | 0.0 / 0 |
| 5 kong_style, Kong radii | total | 9.0 / 9 | 9.0 / 9 | 0.9 / 1 | 0.9 / 1 |
| 6 kong_style, ours | arm vs arm | 271.0 / 271 | 245.0 / 245 | 0.0 / 0 | 0.0 / 0 |
| 6 kong_style, ours | arm vs body | 394.0 / 394 | 273.0 / 273 | 0.1 / 3 | 0.0 / 1 |
| 6 kong_style, ours | total | 665.0 / 665 | 518.0 / 518 | 0.1 / 3 | 0.0 / 1 |

**Counted at the raw command** (mean / max per tick)

| row | pair type | all possible | after never-close | after broad phase | active |
|---|---|---|---|---|---|
| 4 layered, ours | arm vs arm | 271.0 / 271 | 245.0 / 245 | 46.1 / 65 | 18.3 / 31 |
| 4 layered, ours | arm vs body | 394.0 / 394 | 273.0 / 273 | 8.2 / 12 | 6.4 / 10 |
| 4 layered, ours | total | 665.0 / 665 | 518.0 / 518 | 54.3 / 76 | 24.7 / 39 |
| 5 kong_style, Kong radii | arm vs arm | 9.0 / 9 | 9.0 / 9 | 4.1 / 5 | 3.5 / 4 |
| 5 kong_style, Kong radii | arm vs body | 0.0 / 0 | 0.0 / 0 | 0.0 / 0 | 0.0 / 0 |
| 5 kong_style, Kong radii | total | 9.0 / 9 | 9.0 / 9 | 4.1 / 5 | 3.5 / 4 |
| 6 kong_style, ours | arm vs arm | 271.0 / 271 | 245.0 / 245 | 46.1 / 65 | 18.3 / 31 |
| 6 kong_style, ours | arm vs body | 394.0 / 394 | 273.0 / 273 | 8.2 / 12 | 6.4 / 10 |
| 6 kong_style, ours | total | 665.0 / 665 | 518.0 / 518 | 54.3 / 76 | 24.7 / 39 |

What it means: of 518 pairs that survive the never-close list, the broad phase passes only about 54 (at most 76) at the raw command and about 25 are active. At the filtered command there are far fewer (7 or fewer in row 4, almost none in row 6), because the filter holds the arms near the zombie pose. The never-close list is therefore not what keeps the checked set small; the broad phase already does that. Rows 4 and 6 have identical counts at the raw command (same shapes, same stream).

## 2. Time profile (400 ticks, timers on; shares are inclusive, stages nest, so they do not add to 100%)

| stage | row 4 layered, ours: ms/tick, share | row 5 kong_style, Kong radii: ms/tick, share | row 6 kong_style, ours: ms/tick, share |
|---|---|---|---|
| whole tick (timers on) | 99.0 (max 298) | 23.0 (max 70) | 761.8 (max 1652) |
| whole run, timers off, 400 ticks | 35.3 s | 8.9 s | 272.9 s |
| broad phase (evaluate_pairs) | 9.9, 10.0% (10.1 calls/tick) | 5.1, 22.3% (14.6 calls) | 157.0, 20.6% (196.5 calls) |
| pair distance checks, all | 61.1, 61.7% | 2.1, 9.0% | 474.2, 62.2% |
|   arm vs arm | 52.5, 53.0% | 2.1, 9.0% | 411.3, 54.0% |
|   arm vs body | 8.6, 8.7% | 0 | 62.8, 8.2% |
| gap rule | 1.9, 1.9% | 0.01, 0.0% | 0.9, 0.1% |
| push direction | 0.44, 0.4% | 0 | 3.1, 0.4% |
| jump-through guard (incl. samples) | not used | 17.9, 77.7% (0.9 calls, 2.2 samples/call) | 695.5, 91.3% (0.96 calls, 14.6 samples/call) |
| sweeps (plan_sweep) | not used | 0 | 5.8, 0.8% (0.96 sweeps/tick) |
| link length restore | not used | 0 | 0.03, 0.0% |
| Kong solver call | not used | 0 | 0.33, 0.0% (1.1 calls) |
| Jacobian step | 3.4, 3.5% | 0 | 0.46, 0.1% |
| four final checks | 0.71, 0.7% | 0.001, 0.0% | 0.10, 0.0% |
| step shortening (incl. halvings) | 39.4, 39.8% (0.52 calls, 6.8 pass-test evaluations per call) | 15.4, 67.0% (11.7 evaluations per call) | 691.0, 90.7% (17.5 evaluations per call) |
| hand slide / elbow swing (layered only) | 2.2 (2.2%) / 7.6 (7.6%) | 0 | 0 |
| recording | not available (the tick does not call it) | | |

Pair checks (calls of gap_between): row 4 1060 per tick mean (max 2050), 57.6 us each; row 5 33 per tick (max 55), 62.2 us each; row 6 9478 per tick (max 13745), 50.0 us each. Not timed: the _Tick bookkeeping (discover, closing, worst) and keypoint helpers; they only show in the whole-tick figure. Timers add 12% (row 4), 4% (row 5), 12% (row 6) to the run.

What it means: the time is pair distance checks, 62% of every tick in rows 4 and 6, and almost all of it sits inside step shortening (in kong_style, the jump-through guard). The solver, Jacobian step, link restore and final checks together are under 1% in row 6. The broad phase passes about 54 pairs, but shortening re-checks every pair found so far (the candidate set only grows during a tick) at every sampled pose of the path (about 15 samples in row 6) at every halving try (about 17): roughly 50 pairs x 15 samples x 17 tries x 50 us, about the 0.5 s seen in row 6. The cost per check is steady at 50 to 62 us, so the time can be predicted as (pair checks per tick) x 55 us once a never-close test changes the pair count; since the broad phase already keeps about 54 candidates, a never-close test would mainly cut the broad phase itself (10 to 21% of the tick; it is called 196 times per tick in row 6, one all-pairs box cut each time), not the checks.

## 3. Advisory: which pairs never come close (ADVISORY ONLY)

**ADVISORY ONLY: from one stream (the rolling punch, first 400 ticks), not a proof, must not be used to exclude any pair.** Pairs are those after the never-close list; a pair whose boxes were more than 25 cm apart at a tick counts as "gap above 25 cm" there. Share of pairs whose smallest gap over the 400 ticks stays above the distance. Raw command is the stream itself (it penetrates by construction); filtered is what the resolver let through.

| row | pose | pair type | pairs | smallest gap stays > 20 cm | stays > 10 cm | smallest minimum (mm) |
|---|---|---|---|---|---|---|
| 4 | filtered | arm vs arm | 245 | 57.1% | 98.0% | 63.4 |
| 4 | filtered | arm vs body | 273 | 91.9% | 95.2% | 28.6 |
| 4 | filtered | all | 518 | 75.5% | 96.5% | 28.6 |
| 4 | raw | arm vs arm | 245 | 13.9% | 62.9% | -56.9 |
| 4 | raw | arm vs body | 273 | 91.6% | 94.5% | -69.8 |
| 5 | filtered | arm vs arm (Kong, 9 pairs) | 9 | 22.2% | 55.6% | 0.0 |
| 5 | raw | arm vs arm (Kong, 9 pairs) | 9 | 0.0% | 33.3% | -78.2 |
| 6 | filtered | arm vs arm | 245 | 67.8% | 99.6% | 94.1 |
| 6 | filtered | arm vs body | 273 | 92.3% | 95.2% | 31.3 |
| 6 | filtered | all | 518 | 80.7% | 97.3% | 31.3 |
| 6 | raw | all | same stream and shapes as row 4 raw | | | |

What it means (rough idea only): on this stream about 75 to 80% of the pairs the filter lets through never come within 20 cm, and 96 to 97% never within 10 cm; against the raw command only 14% of arm-vs-arm pairs stay beyond 20 cm. A reachability test removes by geometry what this stream only suggests; one stream cannot stand in for it. Objects and held objects: none in this world, so no number.

## 4. Why the resolvers hold (full stream, 1576 ticks)

Every hold was classified from a trace of the real functions (wrappers that only record). The first 3 holds of each reason were replayed from the recorded last filtered command, raw command and active-pair set on a second resolver; all replays (12 in row 4, 3 in row 5, 3 in row 6) gave the identical output and reason as the run. Full per-hold detail is in the JSON (examples, halving logs, pairs below 5 cm).

**Expectation tested: most SHORTEN_ZERO holds come from the start pose already violating the guard's gap. It is false.** In none of the replayed SHORTEN_ZERO holds does any pair sit below the guard gap at the start pose (start_pairs_below_guard_gap is empty; tunnel_guard_skip_already_below is true, so such pairs would not count anyway).

Holds by cause, row 4 (layered, ours; 1529 holds):

| reason | cause found | holds |
|---|---|---|
| SHORTEN_ZERO | gap-rule shortening fails at every fraction down to 1/4096; failing pair at the smallest fraction is right_link5_1 \| torso; its shortfall at the start pose is 0.0 mm, so the start pose does not fail the rule but every move does | 245 |
| BRANCH_GUARD | joint jump found at the solver output: joint 7 241, joint 3 175, joint 5 55, joint 6 10, joint 1 2 | 483 |
| BRANCH_GUARD | joint jump found at the Jacobian step: joint 7 97, joint 4 31, joint 1 5 | 133 |
| ACCURACY | error against the raw commanded pose, position over 2 mm only (first over in round 1 for 318 of 329) | 329 |
| ACCURACY | same, position and orientation both over (first over in round 1: 182, round 4: 79, round 2: 5, round 5: 4, not found in a Jacobian step: 50) | 320 |
| ACCURACY | error against a push target (not raw), both over | 2 |
| JOINT_LIMITS | joint 6 over its upper limit, first at the solver output | 17 |

Row 5 (kong_style, Kong radii, margins 0; 1487 holds): all 1487 are SHORTEN_ZERO of one cause: the jump-through guard fails at its first sample for the left hand capsule vs the right hand capsule, for every fraction down to 1/4096. The start pose is clear (the pair sits at exactly 0.0 mm, touching, from the previous push; the guard gap is 0.0 so it is not "below"), and any step toward the other hand goes negative at once.

Row 6 (kong_style, ours; 1530 holds): all BRANCH_GUARD, all first seen at the solver output (Kong's solver): joint 6 1461, joint 4 28, joint 3 24, joint 5 17. The jump is over 0.25 rad from the very first sweep.

Worked examples (3 per reason; ticks of the full stream; details in the JSON):

| row | reason | ticks | worst pair (gap at last / at raw, margin) | what the replay found |
|---|---|---|---|---|
| 5 | SHORTEN_ZERO | 42, 43, 44 | left_kong_hand \| right_kong_hand (hand, hand), 0.0 mm, raw -7.5, -11.9, -16.2 mm, margin 0 | guard has 1 sample; fails at f=1 and at every one of 13 tries; start gap 0.0 mm; start pose not below the guard gap; fraction 0.0 |
| 4 | SHORTEN_ZERO | 101, 102, 103 | left_finger_left \| right_palm_2 (hand, hand), 84.5 mm, raw -44.8 mm, margin 10 mm; only pair below 5 cm at last: left_link5_1 \| torso, 28.6 mm (margin 30 mm) | 13 pass-tests all fail; the failing pair at the smallest fraction is right_link5_1 \| torso; fraction 0.0 |
| 6 | BRANCH_GUARD | 25, 26, 27 | left_link5_1 \| torso (forearm, torso), 52.0 mm, raw 15.1, 12.2, 9.1 mm, margin 30 mm; no pair below 5 cm at last | left joint 5 jumps 0.31, 0.37, 0.43 rad at the first solver output of sweep 1 (limit 0.25) |
| 4 | BRANCH_GUARD | 46, 47, 48 | left_link5_1 \| torso, 28.6 mm, raw -55.6 to -60.6 mm, margin 30 mm | left joint 6 jumps 0.263, 0.299, 0.336 rad; first over 0.25 at the solver output of round 1 (0.256, 0.279, 0.301) |
| 4 | ACCURACY | 55, 67, 68 | left_link5_1 \| torso, 28.6 mm, raw -66 to -70 mm | hand error 101.5 / 114.1 / 114.4 mm and 4.0 / 7.7 / 7.8 deg against the raw commanded pose; first over 2 mm / 0.5 deg in round 2 (tick 55); not seen in a Jacobian step (ticks 67, 68) |
| 4 | JOINT_LIMITS | 29, 30, 31 | left_link5_1 \| torso, 28.6 mm, raw 2.7, -0.7, -4.1 mm | left joint 6 over its upper limit by 35.7, 73.3, 78.2 mrad; first at the solver output (rounds 4, 3, 2); the raw joint is not on the limit |

What it means: the holds are not rare near-collisions that need careful stepping; they are structural, and the same forearm-torso pair (left_link5_1 and right_link5_1 against the torso) shows up in all of them. In row 4 the left forearm sits 28.6 mm from the torso at the zombie pose, 1.4 mm inside its 30 mm margin, and it is the only pair below 5 cm at the last command in all row 4 examples. The raw command pushes the forearms far into the torso (raw gaps of -50 to -70 mm), the push needed is large, and the converted pose is more than 0.25 rad away on one joint (joints 3 to 7), or off the raw hand pose by 2 mm or more, or past a joint limit; the tick is held. Once held, the next tick starts from the same pose while the raw command has moved further, so the failure repeats until the stream turns back. Row 6 has the same pair at 52 mm (outside its margin) and the solver jump (joint 6 in 95% of its holds) is the single cause. In row 5 the loop comes from the guard, which runs before any push: when the last push left two hand capsules touching at 0.0 mm, the guard rejects every non-zero step, so the hands never slide past each other.

## 5. Lag test

Ticks the Stage 2b report counts as no-contact (raw smallest gap above 5 cm; same measure per row), with hand position error above 10 mm, and how many lie within the 20 ticks after a hold (a hold on one of the 20 preceding ticks; the next column also counts a hold on the tick itself):

| row | no-contact ticks | of those with hand error > 10 mm | within 20 ticks after a hold | including the tick itself | for context: all no-contact ticks within 20 after a hold |
|---|---|---|---|---|---|
| 4 | 64 | 15 | 15 (100%) | 15 (100%) | 53% |
| 5 | 60 | 0 | n/a | n/a | 5% |
| 6 | 64 | 28 | 25 (89%) | 26 (93%) | 59% |

What it means: the lag idea holds. Every large hand error on a no-contact tick in row 4 and 25 of 28 in row 6 follows a hold within 20 ticks, more than the 53 to 59% any no-contact tick has anyway. These "no-contact" errors are the filter still catching up after an earlier hold (the hand is behind the raw command), not a response to a nearby pair. Row 5 has no such ticks.

## 6. Milder stream, rows 4 to 6

Stream (scripts/safety_filter_mild_stream.py): each hand moves in a straight line toward the other hand's home position to the point where the arm gap (our shapes, shoulder-mount pair left out) is 4 cm with the other arm at home (73.0 mm of travel for each hand, found by search), pauses 0.3 s, retreats in 0.5 s; out move 0.5 s (my choice, the task gives no number); period 1.3 s; the other hand starts 0.65 s later; 5 cycles per hand; 1073 ticks; joint targets from our Jacobian, clipped (no tick needed clipping). The script checks the raw command is collision free by our shapes (it would stop otherwise): smallest raw gap 1.37 mm, so it passes, but because the two hands overlap in time the arms come closer than 4 cm (see Decisions). Kong's capsules are bigger: the raw command goes to -7.9 mm against them.

| row | smallest gap Kong radii (mm) | smallest gap our shapes, no shoulder pair (mm) | hold ticks (of 1073) | hold reasons | hand error mean / max (mm) | max joint jump (rad) | time per tick mean / max (ms) |
|---|---|---|---|---|---|---|---|
| 4 layered, ours | 20.6 | 29.7 | 901 (84%) | BRANCH_GUARD 374, JOINT_LIMITS 327, ACCURACY 196, SHORTEN_ZERO 4 | 27.6 / 69.8 | 0.322 | 17.5 / 86.5 |
| 5 kong_style, Kong radii | 0.0 | 9.3 | 201 (19%) | SHORTEN_ZERO 201 | 3.5 / 34.4 | 0.080 | 5.8 / 38.0 |
| 6 kong_style, ours | 83.1 | 92.5 | 1008 (94%) | BRANCH_GUARD 1008 | 59.9 / 111.0 | 0.272 | 172.6 / 415.4 |

What it means: holds occur on the milder stream too, on every row: 84%, 19% and 94% of the ticks. Row 5 improves a lot (19% against 94% on the punch). Rows 4 and 6 hold as much as on the punch although the raw gap never goes below zero by our shapes, which fits section 4: the holds come from the solver and branch, accuracy and limit checks on the forearm-torso pair, not from the arms nearly colliding. I did not classify the holds of this stream by cause (not asked).

## Options for David (none implemented)

1. Run the jump-through guard only against pairs that are active or near (not the whole accumulated candidate set): cuts the dominant 62% of tick time, costs one more rule to specify and test.
2. Evaluate the guard and shortening tests with one batched geometry call per sampled pose (as evaluate_pairs does): reduces the 50 us per pair check, costs new batch code in the resolver.
3. Lower the number of guard samples and halving tries (max_samples 50, 12 halvings): cuts time roughly in proportion, costs coarser step sizes.
4. Call the broad phase once per sampled pose set instead of at every discover (196 calls per tick in row 6): removes up to 20% of the tick, costs a changed candidate-set rule.
5. Build the planned never-close (reachability) test: by section 2 it would mainly cut the broad phase (10 to 21%), not the pair checks.
6. Move the left forearm out of its margin at the zombie pose (a smaller margin for the forearm-torso pair or a shape fix): the pair would no longer start inside its margin; costs a decision on margins or shapes (M288) and less safety allowance.
7. Run the Kong guard after the push, or let it slide the hands as the layered hand slide does: touching hand capsules would no longer freeze the whole tick; costs a design change to the order of steps.
8. Raise the 0.25 rad branch guard, or make the solver call pick the solution nearest the last pose: stops the solver-output jumps, costs weakening or changing a safety check.
9. After a hold, aim the next raw command at the held pose (command-side catch-up) so the lag after a hold (section 5) does not grow: costs an interface change between the filter and its caller.
10. Add a collision-free but close stream with non-overlapping hands as a regression test: costs one new test.

## Decisions needed / deviations

- Mild stream timing: the period is out + pause + back = 1.3 s and the out move 0.5 s, so the second hand starts 0.65 s later while the first is still retreating; the raw arms then come to 1.4 mm apart, not 4 cm. For a true 4 cm minimum the hands must not overlap in time, or the stop point must be searched with both hands moving. David to decide.
- "Active" (slowing band) was taken as gap below margin + 1 cm (PUSH_START_BEYOND_M).
- Pair counts are taken at the last filtered and at the raw command.
- Not built: timing of recording (the tick never calls it); the _Tick bookkeeping; a cause classification of the mild stream's holds; shortening sample numbers for the layered default test (from outside only the count of pass tests is available).
