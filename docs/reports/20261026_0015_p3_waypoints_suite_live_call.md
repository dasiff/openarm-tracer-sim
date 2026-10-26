# P3: waypoints in the full check, moved layout, suite, live planner call (2026-10-26)

Written for: David (decisions needed at the end). Nothing is pushed (the last push is 43dc54e, P2).

## Summary
- Step 1 and 2 done: the waypoint table (`docs/reports/20261025_p3_waypoint_table.md`) and the skill's untested waypoints are now in the overlay's full-check points (M181).
- Adding them exposed a latent bug in the full check (`body_distance` measured the left arm left posed by the IK path check). Fixed; row M181.
- Step 3 NOT met: the moved-layout episode still fails at place OVER (12.2 mm and 0.8 deg short after 11 s), although the full check now passes the first ranked stance.
- Step 4 done: full suite exit 0 for 40 tests, exit 1 only for `stance_select_moved_test` (expected, 6/9). `face_test` alone 27/27.
- Step 5 done: one live planner call (39.7 s, 1 try, no 503): SUCCESS; cache-only rerun 9/9, replay bit-identical.

## Step 2: points added (per target, in the forward frame of the robot)
Pick: "over" (standoff back, bench top + 0.20 m), "standoff" (band middle height), "lift" (band middle + 0.10 m), "back" (0.05 m back at the lift height). Place: "carry" (over the hotplate centre, beaker bottom 0.04 m above the plate) and "back" (0.05 m back at the set-down height). The true UP height depends on the destination, so "lift" uses the region's definition (about 1 cm under the real carry height in the test scene).

## The bug the points exposed
`PathClearance.arm_distances` leaves the left arm at the last joint vector it tried; `body_distance` (documented as "both arms at home") never reset it. With the old last point (above the hotplate) the posed hand was clear; with the new last point (place BACK, next to the plate) the pedestal-1 error stance (2 cm lower) put the finger into the plate, so EVERY ranked stance failed "body", also in the default layout. Control: new points reverted, `mpvi_pipeline_test` 14/14; new points and fix, 14/14. `full_check_test` 31/31 (new check 3b tests the arm is back home). Not a loosened limit.

## Step 3: moved layout (log data/logs/stance_select_moved_20261025_220439.log)
- Chosen stance (0.635, 1.135) pedestal 0.10, full check PASS on the first ranked stance, one run for both steps, second navigation step null.
- Episode: grasp OK, then place OVER "not reached after 11.0 s: 12.2 mm and 0.8 deg from the waypoint", recovery HOME did not settle (0.030 rad). Outcome PLAN_FAILED; beaker not on the hotplate. Same failure as before the new points.
- So the full check, with the place OVER point ("carry"), still says reachable what the skill cannot reach. My hypothesis (NOT verified): the check lets every point use any approach direction (`bin None`), the skill always uses bin 0 (forward, hand level), and the held beaker's offset from the pad moves the real OVER target. A measure-only run with bin 0 on these points would test it.

### The addition: did the backup stances fail for the same reason as the first?
No backups were needed after the fix (the first ranked stance passed in both layouts). The only evidence is the earlier run, before the fix, moved layout, three ranked stances, three different checks: (0.635, 1.135) "body" (the bug above, so an artefact), (0.735, 1.105) "ik" at the error stance y-1 on point 17 (place BACK, no collision-free solution), (0.555, 1.065) "home_clearance" (arms 32 mm from the hotplate base at pedestal 0.070). Different check, different place each time. Weak evidence (the first was the bug); I did not build anything.

## Step 4: suite (summary data/logs/suite_p3_20261025_220800_summary.txt)
40 tests exit 0, `stance_select_moved_test` exit 1 (6/9: the episode fails at place OVER; criterion not met). Skipped on purpose: `mpvi_text_test` (live; run separately below), `face_test` (alone: 27/27, data/logs/face_test_p3_20261026_000136.log). Memory fine in the exempt scope (no oomd kill).

## Step 5: live planner call (log data/logs/mpvi_text_oracle_live_20261026_000421.log)
One call to gemini-3.8-flash, 39.7 s, 1 try; episode SUCCESS; 8/9 because `mpvi_text_test` counted the carry slice monitor's entries as drops (stale check from P2); fixed to count only drop-monitor entries. Cache-only rerun 9/9 (data/logs/mpvi_text_oracle_cache_20261026_000730.log), the reply is stored in experiments/llm_cache. No VLM calls made (not part of P3).

## Carry slice monitor, every run with a carry (largest distance outside the slice, no margin, margin 120 mm)
| Run | hand | beaker rim | forearm | warning |
|---|---|---|---|---|
| mpvi_pipeline_test (default layout, p3b) | 4.1 mm | 44.9 mm | 86.6 mm | none |
| mpvi_text_test, live and cache-only | 4.1 mm | 44.9 mm | 86.6 mm | none |
| stance_select_moved_test | not logged: the window ends at the place's first waypoint (OVER), which was not reached | | | |
| suite's other pipeline runs | in their logs (data/logs/suite_p3_20261025_220800_*.log) | | | |

## Questions for David
1. Should the full check require the skill's own approach (bin 0, hand level) for the skill's waypoints? That is likely why the moved layout passes the check and fails the episode. It would be a change to the check; I did not do it.
2. The carry check's margin test is `distance < margin`, so a zero margin never blocks an overlap (unchanged; 120 mm in use). Use `<=`?
3. The navigation-on-by-default decision has no row of its own (only the last sentence of M180). Move it into M175?
4. Moved-layout failure: want the measure-only run of question 1?
