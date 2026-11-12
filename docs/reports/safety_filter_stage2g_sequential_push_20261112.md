# Stage 2g: the kong_style resolver pushes one pair at a time

Date 2026-11-12. Branch safety-filter, not pushed. Decision rows SF-52 to SF-58 (docs/mpvi_decisions.md, docs/design_plan.md section 16 "Stage 2g").
Change: src/safety_filter/kong_style.py (summing removed, sequential sweep), config RESOLVER["pair_order"], status counters in src/safety_filter/resolver.py (FilterStatus). The layered resolver is unchanged.
Scripts: scripts/safety_filter_kong_single_pair_fixtures.py, scripts/safety_filter_reactive_stream.py (new); scripts/safety_filter_hold_causes.py and scripts/safety_filter_time_profile.py (one line each: they wrapped the deleted plan_sweep, now _sweep).
Tests: experiments/safety_filter_sequential_push_test.py (new, 43 checks, written by a subagent from the step 4 spec, reviewed), experiments/safety_filter_kong_style_test.py (helper rewritten, 38 checks).
Results (git-ignored): data/safety_filter/seqpush_*.json and .npz, hold_causes_s2g_row6.json, kong_single_pair_fixtures.json. Logs: data/logs/s2g_*.log.

## What changed, in plain words

Before: one sweep measured every colliding pair at the same starting pose, shared each pair's push among the arm's points and ADDED them (Stage 2f: about 55 contributions per sweep, the wrist sum about 20 times the largest single one). Now: a sweep lists the pairs with a shortfall of 0.5 mm or more, orders them by shortfall (largest first, ties by name), and for each one measures it again at the current pose, skips it if under 0.5 mm, otherwise pushes it (existing amount, direction, lever split, closing-arm rule), restores the link lengths and converts to joint angles at once (Kong's solver, one Jacobian step). Nothing is clipped and no guard or limit check runs between pairs. The summing code is deleted. The leftover search found nothing else in src/safety_filter/ or the tests that refers to summing (only unrelated "added"); the old plan_sweep users (two scripts) were updated.

## 1. Single-pair regression (step 1 fixtures against the new code)

| case | result |
|---|---|
| upper_arm_vs_box | same status; joints differ by 3.7e-4 rad (explained below) |
| forearm_vs_torso | identical (within 1e-9 rad) |
| hand_vs_box_link7 (ends in HOLD SHAPE_RECHECK after 10 sweeps) | identical |
| hand_link7_vs_box_converges | identical |
| arm_vs_arm_left_closing | identical |
| arm_vs_arm_both_closing | identical |
| shoulder_vs_box_shortening | identical (STEP_SHORTENED) |
| upper_arm_link3_1_vs_box | differs: KONG_PUSH before, PASS now (shortfall 0.43 mm) |

Six of eight cases are identical. The two differences have the same cause. The old code pushed every pair above 1 micrometre; the new rule (spec) lists only pairs at or above the stop value 0.5 mm. In upper_arm_vs_box the second sweep also holds a pair with a shortfall of 0.029 mm, which the old code pushed together with the forearm pair; the new code skips it. The fixtures were not changed. This is listed under decisions.

## 2. Before and after, rows 5 and 6, full stream (1576 ticks)

"Before" for the original stream is the saved Stage 2b run (kong_comparison_20261112_140129.json); for the reactive stream the same runner was run on the old code (worktree at commit 3be78ca, same shapes file). Smallest gap: "ours" = our shapes without the shoulder-mount pair (which sits 14 mm inside at the zombie pose and sets the plain minimum), "Kong" = Kong's capsule radii.

Original stream:

| | row 5 before | row 5 after | row 6 before | row 6 after |
|---|---|---|---|---|
| hold ticks (share) | 1487 (94.4%) | 1487 (94.4%) | 1530 (97.1%) | 0 (0%) |
| hold reasons | SHORTEN_ZERO 1487 | SHORTEN_ZERO 1487 | BRANCH_GUARD 1530 | none |
| hand position error mean / max (mm) | 69.3 / 165.9 | 69.3 / 165.9 | 83.2 / 165.9 | 60.0 / 124.2 |
| largest joint jump per tick (rad) | 0.145 | 0.145 | 0.245 | 0.070 |
| smallest gap, Kong radii (mm) | 0.0 | 0.0 | 84.7 | -10.9 |
| smallest gap, ours (mm) | 9.3 | 9.3 | 94.1 | 24.0 |
| time per tick mean / max (ms) | 49.7 / 122 | 35.7 / 158 | 1145 / 3262 | 586 / 1637 |
| solver calls per tick mean / max | not measured | 0 / 0 | not measured | 3.9 / 11 |
| pairs pushed / skipped per tick (mean) | n/a | 0 / 0 | n/a | 2.7 / 49.9 (max skipped 70) |
| sweeps per tick (max) | n/a | 0 | n/a | 2 |
| intermediate joint-limit overshoot | n/a | none (0 ticks) | n/a | none (0 ticks) |
| largest joint change between intermediate poses (rad) | n/a | 0 | n/a | 0.195 |

Reactive stream (the raw command follows the arm's actual previous output, at most 0.018 rad per joint per tick):

| | row 5 before | row 5 after | row 6 before | row 6 after |
|---|---|---|---|---|
| hold ticks (share) | 1458 (92.5%) | 1458 (92.5%) | 7 (0.4%) | 0 (0%) |
| hold reasons | SHORTEN_ZERO 1458 | SHORTEN_ZERO 1458 | JOINT_LIMITS 7 | none |
| hand position error mean / max (mm) | 5.7 / 8.6 | 5.7 / 8.6 | 4.2 / 9.2 | 3.7 / 9.5 |
| largest joint jump per tick (rad) | 0.018 | 0.018 | 0.0199 | 0.0286 |
| smallest gap, Kong radii (mm) | 0.0 | 0.0 | 20.5 | 10.7 |
| smallest gap, ours (mm) | 9.3 | 9.3 | 37.2 | 24.8 |
| time per tick mean / max (ms) | 25.0 / 49 | 29.4 / 126 | 15.3 / 31 | 14.6 / 38 |
| solver calls per tick mean / max | 0 / 0 | 0 / 0 | 1.38 / 2 | 1.71 / 7 |
| pairs pushed / skipped per tick (mean) | n/a | 0 / 0 | n/a | 1.6 / 2.2 (max skipped 6) |
| sweeps per tick (max) | n/a | 0 | n/a | 2 |
| intermediate joint-limit overshoot | n/a | none | n/a | none (0 ticks) |
| largest joint change between intermediate poses (rad) | n/a | 0 | n/a | 0.029 |

First sample (400 ticks) gave the same pattern (row 6: 0 holds on both streams; row 5: 358 and 340 SHORTEN_ZERO holds). Timings are from different runs and machine loads, so the time columns are only a rough comparison.

What it means. Row 6 on the original stream is the clear gain: the 1530 branch-guard holds (97% of the stream) are gone, the largest joint jump per tick falls from 0.245 to 0.070 rad (the guard limit is 0.25), the hand follows the command better (mean error 83 to 60 mm, max 166 to 124 mm) and the filter takes about half the time per tick. The arm now moves, so it also comes closer to the other arm and the torso: the smallest gap fell from 94 mm (an arm stuck in place) to 24 mm in our shapes, still positive. By Kong's larger radii the smallest gap is -10.9 mm; row 6 enforces our own tighter shapes and margins, so this is not a violation of its own rule, and the old value of 84.7 mm only reflected the arm being stuck. On the reactive stream the old code already held only 7 ticks (joint limits) and the new code holds none; the numbers are otherwise close to equal, with slightly more solver calls (1.7 against 1.4 per tick). Row 5 did not change at all: its hold is the jump-through guard shortening the step to zero (SHORTEN_ZERO) before any push, so the sequential sweep never runs there (0 solver calls).

## 3. What the one-pair-at-a-time method did to the pushes

| measure (row 6) | original stream | reactive stream |
|---|---|---|
| ticks with at least one push | 1534 of 1576 | 1533 of 1576 |
| pairs listed that an earlier push had already fixed (skipped), per tick | 49.9 (max 70) | 2.2 (max 6) |
| pairs pushed per tick | 2.7 | 1.6 |
| intermediate joint-limit overshoot | none | none |
| holds that name joint 6 | 0 | 0 |

The replay of Stage 2f (scripts/safety_filter_push_size_replay.py) was not run, because no hold remains in row 6 (hold_causes on the original stream: 0 holds in 1576 ticks), so there is no joint 6 hold to replay. The push-size numbers of Stage 2f (wrist 424 mm from the guard pose) cannot be compared directly: this stage does not record per-tick push displacement. The indirect evidence is the hand error against the raw command (largest 124 mm, against 166 mm before) and the largest joint change between intermediate poses (0.195 rad on the original stream, 0.029 rad on the reactive one).

What it means. The method removes the large pushes of Stage 2f: of the pairs listed in a sweep on the original stream about 50 per tick are already fixed by an earlier push and are skipped, which is exactly the double counting that added up in the old code. No hold remains in row 6 on either stream, and no intermediate pose went past a joint limit, so the open decisions on clipping and on a new guard rule are not needed by these streams (they may be needed by other scenes).

## 4. Tests

Existing: distance, gap_rule, link_rules, shapes, structure, touch, resolver: all pass. kong_style test: 38 of 38. New sequential-push test: 43 of 43. Spec test: one failing check, "the closing right arm moves" (HOLD ACCURACY in the right-arm mirrored case); the same check fails in the Stage 2b logs, so it is not caused by this change.
Test code changed (all in experiments/safety_filter_kong_style_test.py): the helper sweep() now runs the new sequential sweep with a given shortfall and returns the restored points through a spy (it used the deleted plan_sweep); the two checks in section 4 (upper arm contact above and below min_lever_fraction) use a given shortfall of 2 mm, because their real geometry is 0.44 mm short, under the new 0.5 mm stop value. No test was weakened; none asserted a summed total.

## 5. Not built or only in part

- Per-tick push displacement is not recorded in the status (only counts and the intermediate joint change); the push-size replay was therefore not repeated.
- Before numbers for solver calls and pushes on the original stream do not exist (the Stage 2b run did not record them).
- The test that compares a full step with the largest single pair (item 6 of the tests) uses a small scene: full wrist move 6.5 mm against 4.7 mm for the largest single pair (ratio 1.39, limit 1.5). The about 20-fold case of Stage 2f is not reproduced in a test.
- Only the rolling punch was run; grasping or other scenes were not.
- Step shortening inside a sweep is kept as it was: an arm flagged for shortening is shortened at the end of the sweep and the sweeps stop (pushes of that arm earlier in the sweep are overwritten, later pushes of that arm are skipped); other arms' pushes stay.

## Options for David (none implemented)

1. List pairs from the old threshold (1 micrometre) instead of 0.5 mm: restores the old single-pair results exactly, but costs extra solver calls for pushes smaller than the final check tolerance.
2. Let rows with Kong radii (row 5) skip or relax the jump-through guard when a push is possible: would let the sequential sweep act there; costs deciding what the guard is for.
3. Convert once per arm per sweep instead of after every pair: would cut solver calls (3.9 per tick on the original stream) and time; costs the exactness of each next measurement that David asked for.
4. Add a per-tick push displacement to the status record: makes the push size measurable in every run; costs one more field and a small amount of time.
5. Make the reactive row 6 run a regular test (1576 ticks takes about half a minute): catches regressions in holds and overshoot; costs test time.
6. Try other scenes (carrying a beaker, arm near the bench) before closing the joint limit and guard decisions: the rolling punch shows no overshoot, other scenes may.

## Decisions needed

- Listing threshold: stop value 0.5 mm as in the spec (default now) or the old 1 micrometre (option 1); it changes two single-pair fixtures.
- Shortening inside a sweep: kept as is; to be confirmed.
- The reactive stream limit 0.018 rad per joint per tick is a script constant (SF-57), test only.
- Row 5 stays at 94% hold (jump-through guard); no change made.
- Not decided here, still open: joint-limit clipping and any new guard rule (not added).
