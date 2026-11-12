# Stage 2b: six-row comparison on the rolling punch (layered vs kong_style vs Kong's filter)

Run: 2026-11-12, started 14:01:28 (tmux `s2b_cmp`), full stream of 1576 ticks (10 punches per hand). Stamp of the JSON: 20261112_140129.
Commit: `406330d` (`git rev-parse --short HEAD` at the run). The script changes for this run are committed in the step 5 commit of branch safety-filter; nothing was pushed.

Files:
- Script: `scripts/safety_filter_kong_comparison.py` (extended to six rows)
- Raw results: `data/safety_filter/kong_comparison_20261112_140129.json`
- Log: `data/logs/20261112_140128_s2b_comparison.log` (exit 0; wall 37 min 35 s; script runtime 2254.3 s; peak RSS 2.19 GB)
- Dry run (60 ticks, all six rows ran): `data/logs/20261112_140039_s2b_dry60.log` (exit 0; its JSON went to the scratch folder, not to `data/safety_filter/`)
- Earlier four-row report: `docs/reports/safety_filter_stage2_kong_comparison_20261111.md`

## Result

Required gap per row: rows 1 to 3 and 5 use Kong's capsule radii (kong_radii), rows 4 and 6 use our shapes. Distances in mm, angles in degrees, filter time per tick in ms. Tick numbers of each smallest gap are in the JSON.

| configuration | required gap | min gap Kong radii (mm) | min gap our shapes (mm) | min gap our shapes, shoulder-mount pair left out (mm) | hold ticks (of 1576) | hand position error vs raw, mean / max (mm) | hand orientation error vs raw, max (deg) | max joint jump per tick (rad) | filter time per tick, mean / max (ms) | no-contact ticks | no-contact hand position error, mean / max (mm) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1. no filter (output = raw) | Kong radii | -78.16 | -56.95 | -56.95 | n/a | 0.0 / 0.0 | 0.0 | 0.018 | n/a | 60 | 0.0 / 0.0 (by definition) |
| 2. Kong compiled filter, streamed | Kong radii | +3.41 | -14.22 | -7.20 | n/a | 50.0 / 104.4 | 43.2 | 0.201 | 0.29 / 2.69 | 60 | 1.8 / 22.6 |
| 3. layered, paper_plain (gap rule off), Kong radii, margins 0 | Kong radii | -0.50 | -14.22 | +8.53 | 1387 (88%) | 69.7 / 165.7 | 8.1 | 0.418 | 16.8 / 119.3 | 60 | 0.0 / 0.0 |
| 4. layered, full gap rule ("ours"), our shapes and margins | our shapes | +54.14 | -14.22 | +63.42 | 1529 (97%) | 72.2 / 165.9 | 8.1 | 0.326 | 174.0 / 527.9 | 64 | 9.7 / 62.0 |
| 5. kong_style, paper_plain, Kong radii, margins 0 | Kong radii | +0.00 | -14.22 | +9.30 | 1487 (94%) | 69.3 / 165.9 | 8.1 | 0.145 | 49.7 / 122.5 | 60 | 0.0 / 0.0 |
| 6. kong_style, full gap rule ("ours"), our shapes and margins | our shapes | +84.75 | -14.22 | +94.05 | 1530 (97%) | 83.2 / 165.9 | 8.1 | 0.245 | 1145.3 / 3261.7 | 64 | 13.8 / 91.8 |

No-contact measure: rows 2, 3 and 5 use Kong's gap; rows 4 and 6 use our gap with the shoulder-mount pair left out (see open choice 1). Ticks counted are the raw command's ticks above 0.05 m: 60 with Kong's gap, 64 with our gap (no shoulder pair), 0 with our gap including the shoulder pair.

Status and hold-reason counts (ticks; a blank reason is a tick that is not a hold, not shown):

| configuration | PASS | ELBOW_SWING | HAND_SLIDE | STEP_SHORTENED | KONG_PUSH | HOLD | hold reasons |
|---|---|---|---|---|---|---|---|
| 3. layered, paper_plain, Kong radii | 82 | 0 | 107 | 0 | 0 | 1387 | ACCURACY 1033, BRANCH_GUARD 330, SHORTEN_ZERO 24 |
| 4. layered, full gap rule, own shapes | 33 | 13 | 0 | 1 | 0 | 1529 | ACCURACY 651, BRANCH_GUARD 616, SHORTEN_ZERO 245, JOINT_LIMITS 17 |
| 5. kong_style, paper_plain, Kong radii | 82 | 0 | 0 | 7 | 0 | 1487 | SHORTEN_ZERO 1487 |
| 6. kong_style, full gap rule, own shapes | 29 | 0 | 0 | 0 | 17 | 1530 | BRANCH_GUARD 1530 |

Rows 1 and 2 have no statuses (n/a). Each status row sums to 1576.

## What it means

On this stream the raw command is in collision on most ticks, so every resolver holds most of the time. Kong's compiled filter never holds: it keeps its own capsules apart (smallest gap +3.4 mm) by moving the hands 50 mm on average (104 mm max) from the raw command, with joint jumps up to 0.20 rad and 0.3 ms per tick. Our layered resolver with the gap rule off holds on 1387 ticks (88%), lags the command by 70 mm on average, goes 0.50 mm into Kong's capsules (inside its own 0.5 mm stop tolerance), has the largest one-tick joint jumps of the six rows (0.42 rad) and takes 16.8 ms per tick on average. The kong_style resolver with the same gap-rule-off setup holds on 1487 ticks (94%, all as SHORTEN_ZERO), lags by the same 69 mm, never goes below 0 mm (smallest gap 0.00 mm), has smaller joint jumps (0.15 rad) and takes 49.7 ms per tick, about three times the layered time. So kong_style is worse than layered on holds and speed, equal on lag, and better on penetration and joint jumps; it is not better overall on this stream. With the full gap rule and our own shapes, layered holds on 1529 ticks (97%), lags 72 mm, jumps 0.33 rad and takes 174 ms per tick; kong_style holds on 1530 ticks, lags more (83 mm), jumps less (0.25 rad), keeps more clearance (+85 mm to Kong's capsules against +54 mm) and takes 1145 ms per tick on average (3.3 s max), about 170 times the 6.7 ms tick budget and 6.6 times the layered time. Both gap-rule rows change the hand on ticks whose raw command is more than 5 cm from contact (9.7 mm mean for layered, 13.8 mm for kong_style, against 1.8 mm for Kong's filter on its own no-contact ticks). These numbers describe how each filter responds to commands that are infeasible under this stream, not how it would do on a real punch.

## Deviations and open choices

1. No-contact measure for the our-shapes rows (4 and 6): I left out the shoulder-mount pair, because with it the raw gap is below 0.05 m on all 1576 ticks (JSON `no_contact.raw_ticks_above_threshold.our_shapes_all_pairs` = 0), which would leave no no-contact ticks. This is my reading of "the row's own gap measure". David to confirm.
2. `NO_CONTACT_GAP_M = 0.05` m is my choice, as specified. Where in the stream the 60 and 64 no-contact ticks fall was not checked.
3. Row 4's hold-reason split differs from the earlier report (hold total is the same, 1529): earlier ACCURACY 1002, BRANCH_GUARD 326, SHORTEN_ZERO 136, JOINT_LIMITS 65. The earlier run was at commit 87d1a3c, which has no `elbow_swing_hand_target` setting, so it ran the old behaviour. Row 3 (no elbow swings) matches the earlier run exactly. I did not re-run row 4 with the old setting, so the cause is likely but not confirmed.
4. Timing: rows 2 to 4 are 1.5 to 1.9 times the earlier run's means (0.29, 16.8 and 174.0 ms against 0.20, 8.75 and 96.7 ms). The code path for these rows did not change. The VM had 3.4 GB of swap in use before and after the run (3.8 GB in the earlier run). Not investigated. Row 6 took 1145 ms mean and 3262 ms max per tick, so the full run took 37 min, not the hour the brief expected. No optimisation was done, as instructed.
5. Row 5: all 1487 holds are SHORTEN_ZERO. Row 6: all 1530 holds are BRANCH_GUARD. Not investigated.
6. Row 5: the smallest Kong-radii gap is exactly 0.00 mm (at tick 1531), i.e. touching, not negative. Not investigated whether this is the push landing on the margin.
7. Rows 4 and 6 are now run even if Kong's wheel fails to load (the earlier script returned an error row for rows 3 and 4 in that case). Not triggered in this run (Kong's import was ok). A small change made for this run.
8. The stream is in collision by construction, as in the earlier report: the raw Kong-radii gap is negative on 1494 of 1576 ticks (earlier report, same stream, not re-measured here). Joint limits are handled as in the earlier report (`--clip final`, clipped trajectory, offsets 0). Unchanged from the earlier run.
9. Nothing committed or pushed. The script and this report are uncommitted. `docs/progress.md` was not updated, because the brief allowed edits to only two files.
