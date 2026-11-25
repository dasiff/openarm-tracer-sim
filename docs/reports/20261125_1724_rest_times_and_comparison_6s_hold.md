# Comparison at rest: when the reference and the joints settle, and the (a) measurement 1 s and 2 s after the reference settles (2026-11-25)

Branch `safety-filter`, HEAD be75e33 before this round. One MuJoCo process at a time. Runs in `data/logs/policy_compare_20261125/h6/` (git-ignored; 33 runs, all exit 0). Nothing pushed. No production change.

**Correction to the earlier report** (`20261125_1659_harness_fix_and_consistent_comparison.md`): it said 34 runs, a 30 s stream and a 3 s hold. The runs were 33. The loop starts at sim time 1.4 s, so the 30 s stream ended at 30.0 s sim time, and the "3 s hold" actually ran from 30.0 s to 34.55 s (4.55 s). The stream therefore lasted 28.6 s, and the setup move (about 10.3 s) is part of it. The 1 s-after-hold-start numbers of that report are correct for the runs they describe. The table below is from a rerun with a 6.15 s hold (30.0 s to 36.15 s).

Written for: David (lab notebook: the comparison at rest).

## Plain-language answer

1. **The reference settles early; the joints almost never fall below the threshold.** In every run the reference (bridge output minus dq) stops changing between 0.0 and 2.3 s after the hold starts. The joints do not all drop below 1e-4 rad/s within the hold in most runs. Even policy A with no correction creeps at about 7e-5 rad/s on one joint, so the joint criterion is at the arm's creep level and is not a useful stop test here.
2. **Measurement (a), 1 s and 2 s after the reference stops, is much smaller than before.** For B, C and D, no correction gives 2.8 to 5.6 mm position error 1 s after the reference settles, against 7.6 to 12.2 mm 1 s after the hold starts. The production rule gives 1.1 to 1.9 mm, and the nudging rule 0.3 to 1.0 mm. So the earlier numbers mostly reflected the arm still moving toward the reference, as you suspected.
3. **Rotation does not settle the same way.** The production rule leaves a steady rotation error of 4 to 7 mrad 1 to 2 s after the reference settles, against 1 to 4 mrad for the nudging rule and 12 to 21 mrad with no correction.
4. **The 3 s hold was long enough for (a).** The latest reference stop is 2.27 s after the hold start, so two seconds later is 4.27 s, which is inside the earlier 4.55 s hold. I reran all 33 runs with a 6.15 s hold anyway, and the table below is from that rerun.
5. **Policy A's largest error is from the initial move, not from the hold.** Its largest position error in all three cases is at 1.81 s, about 0.4 s after the loop's start, during the setup move toward the first target. Its error in the hold is 0.0 to 0.1 mm (nudge and production) and 1.5 mm with no correction.

## Rest times (sim time after the hold starts; threshold 1e-4 rad/s on any joint)

"Joints stop moving" is the last time after the hold start at which any joint's velocity exceeds the threshold. "not reached" means it was still above the threshold at the end of the run (6.15 s after the hold start).

| case | stream | seed | reference stops changing (s) | joints stop moving (s) |
|---|---|---|---|---|
| off | A | 1 | 0.00 | 0.00 |
| off | B | 1 | 1.43 | not reached |
| off | B | 2 | 1.47 | not reached |
| off | B | 3 | 2.21 | 5.47 |
| off | C | 1 | 0.05 | not reached |
| off | C | 2 | 1.01 | 1.99 |
| off | C | 3 | 1.21 | not reached |
| off | D | 1 | 0.81 | not reached |
| off | D | 2 | 1.39 | not reached |
| off | D | 3 | 0.77 | not reached |
| nudge | A | 1 | 0.00 | 0.84 |
| nudge | B | 1 | 1.45 | not reached |
| nudge | B | 2 | 1.47 | not reached |
| nudge | B | 3 | 2.27 | 5.61 |
| nudge | C | 1 | 0.05 | 5.01 |
| nudge | C | 2 | 1.07 | not reached |
| nudge | C | 3 | 1.23 | not reached |
| nudge | D | 1 | 0.83 | 5.59 |
| nudge | D | 2 | 1.41 | 3.42 |
| nudge | D | 3 | 0.79 | not reached |
| measure | A | 1 | 0.00 | 0.00 |
| measure | B | 1 | 1.43 | not reached |
| measure | B | 2 | 1.45 | not reached |
| measure | B | 3 | 2.23 | not reached |
| measure | C | 1 | 0.05 | not reached |
| measure | C | 2 | 1.03 | not reached |
| measure | C | 3 | 1.21 | not reached |
| measure | D | 1 | 0.81 | not reached |
| measure | D | 2 | 1.41 | not reached |
| measure | D | 3 | 0.77 | not reached |
| reference, normal physics, off | A | 1 | 0.00 | 0.00 |
| reference, normal physics, nudge | A | 1 | 0.00 | 2.85 |
| reference, normal physics, measure | A | 1 | 0.00 | 0.00 |

Reading: the "0.00" for policy A means the arm was already still at the hold start. For the reference rows of policy A with the nudging rule, the joints settle at 2.85 s. For the rest, the joints creep above 1e-4 rad/s to the end of the run (the 6.15 s hold). The creep is the correction's slow relaxation (nudge and measure), and in the no-correction runs the arm's own slow approach to its droop.

## The table: (a) at 1 s and 2 s after the reference stops changing; (b) and (c) as before

Hand position error (mm) and rotation error (mrad). B, C, D: mean (range over three seeds). The normal-physics reference row is policy A at its own rest.

### a. hand position error 1 s after the reference stops changing (mm)

| stream | off (no correction) | nudge (8e45fac, test only) | measure (production) |
|---|---|---|---|
| A | 1.5 | 0.1 | 0.0 |
| B | 5.6 (5.0 to 6.7) | 0.3 (0.2 to 0.5) | 1.9 (1.3 to 2.4) |
| C | 2.8 (1.3 to 4.0) | 1.0 (0.5 to 1.6) | 1.1 (0.7 to 1.8) |
| D | 3.0 (1.4 to 4.5) | 0.7 (0.5 to 1.0) | 1.1 (0.9 to 1.2) |
| A, normal physics (reference) | 1.9 | 0.0 | 0.0 |

### a. hand rotation error 1 s after the reference stops changing (mrad)

| stream | off (no correction) | nudge (8e45fac, test only) | measure (production) |
|---|---|---|---|
| A | 14.8 | 0.7 | 0.2 |
| B | 20.7 (19.8 to 22.1) | 1.0 (0.9 to 1.2) | 7.0 (5.7 to 7.9) |
| C | 12.3 (9.8 to 16.2) | 4.2 (2.8 to 5.0) | 6.8 (3.7 to 9.6) |
| D | 16.2 (11.7 to 19.3) | 3.7 (1.3 to 6.4) | 6.8 (5.6 to 7.5) |
| A, normal physics (reference) | 16.9 | 0.6 | 0.2 |

### a. hand position error 2 s after the reference stops changing (mm)

| stream | off (no correction) | nudge (8e45fac, test only) | measure (production) |
|---|---|---|---|
| A | 1.5 | 0.1 | 0.0 |
| B | 5.5 (4.9 to 6.6) | 0.3 (0.2 to 0.4) | 1.2 (0.8 to 1.5) |
| C | 2.8 (1.3 to 4.0) | 0.4 (0.2 to 0.6) | 0.7 (0.3 to 1.3) |
| D | 3.0 (1.4 to 4.4) | 0.4 (0.3 to 0.8) | 0.7 (0.6 to 0.8) |
| A, normal physics (reference) | 1.9 | 0.0 | 0.0 |

### a. hand rotation error 2 s after the reference stops changing (mrad)

| stream | off (no correction) | nudge (8e45fac, test only) | measure (production) |
|---|---|---|---|
| A | 14.8 | 0.7 | 0.3 |
| B | 20.6 (19.7 to 22.0) | 1.5 (0.6 to 2.8) | 4.4 (3.6 to 4.8) |
| C | 12.1 (9.7 to 16.0) | 2.3 (1.0 to 2.9) | 4.2 (2.0 to 6.6) |
| D | 16.0 (11.5 to 19.4) | 2.8 (1.2 to 4.0) | 4.3 (3.6 to 4.7) |
| A, normal physics (reference) | 16.9 | 0.6 | 0.2 |

### a (old definition, kept for comparison): hand position error 1 s after the hold starts (mm)

| stream | off (no correction) | nudge (8e45fac, test only) | measure (production) |
|---|---|---|---|
| A | 1.5 | 0.1 | 0.0 |
| B | 12.2 (10.6 to 14.5) | 7.1 (6.0 to 9.0) | 10.1 (9.3 to 11.4) |
| C | 7.6 (4.0 to 10.1) | 5.3 (1.5 to 8.1) | 7.4 (1.8 to 11.1) |
| D | 10.6 (7.5 to 15.6) | 9.4 (6.2 to 12.0) | 10.4 (5.6 to 15.7) |
| A, normal physics (reference) | 1.9 | 0.0 | 0.0 |

(The old definition is from the earlier 4.55 s hold; the 6.15 s runs give the same values to 0.1 mm.)

### b. largest position error over the run (mm)

| stream | off (no correction) | nudge (8e45fac, test only) | measure (production) |
|---|---|---|---|
| A | 26.3 | 29.5 | 26.8 |
| B | 35.0 (29.2 to 38.1) | 29.5 (29.5 to 29.5) | 32.2 (26.8 to 36.3) |
| C | 37.3 (36.2 to 39.3) | 32.7 (29.5 to 36.3) | 35.9 (33.1 to 37.5) |
| D | 50.1 (45.6 to 52.8) | 44.0 (39.6 to 48.6) | 46.8 (41.4 to 50.6) |
| A, normal physics (reference) | 26.6 | 29.7 | 27.0 |

### b. largest rotation error over the run (mrad)

| stream | off (no correction) | nudge (8e45fac, test only) | measure (production) |
|---|---|---|---|
| A | 62.6 | 65.7 | 63.3 |
| B | 132.0 (128.2 to 135.2) | 101.3 (94.6 to 106.2) | 126.0 (119.5 to 130.8) |
| C | 130.1 (121.8 to 135.8) | 107.0 (102.3 to 114.0) | 126.3 (120.8 to 132.0) |
| D | 141.5 (131.4 to 147.2) | 127.4 (119.7 to 137.8) | 137.7 (134.0 to 144.3) |
| A, normal physics (reference) | 63.2 | 66.0 | 63.7 |

### c. largest correction, hand displacement |C_p| (mm), and ticks at the clip

| stream | off (no correction) | nudge (8e45fac, test only) | measure (production) |
|---|---|---|---|
| A | 0.0 | 6.9 | 2.2 |
| B | 0.0 | 11.3 (10.5 to 12.4) | 5.8 (5.3 to 6.7) |
| C | 0.0 | 11.5 (10.8 to 11.9) | 5.2 (5.0 to 5.7) |
| D | 0.0 | 14.9 (13.6 to 15.8) | 5.5 (5.4 to 5.7) |
| A, normal physics (reference) | 0.0 | 6.7 | 3.2 |

Ticks at the position clip and at the rotation clip: 0 in every run and case.

(b) and (c) keep their definitions. Their values are from the 6.15 s runs: for the production rule on B the largest correction is 5.8 mm, against 5.5 mm in the 4.55 s runs, because the longer run includes a later maximum. All other (b) and (c) values match.

## Policy A: where its largest error comes from

In all three cases and in the normal-physics reference, the largest position error of policy A is at 1.81 s. That is 0.4 s after the loop starts, during the setup move that brings the arm from rest to the first target. In the hold, policy A's error is 0.0 to 0.1 mm with the nudging and production rules, and 1.5 mm with no correction. So for A the largest error comes from the initial move, not from the hold.

## Technical block

- **Logging added** (`experiments/policy_stream_test.py`, committed with this report): one joint CSV per run (`<name>_joints.csv`) with the reference joints (bridge output minus dq, the joint-level version of the stream test's reference), the measured joints and the joint velocities, every tick. The summary records `loop_t0_s` (checked: 1.4).
- **Hold timing fixed.** The earlier frame count (`seconds + hold_s` at 150 Hz) started counting at the loop's 1.4 s, so the hold ran 4.55 s. The frame count is now taken from the loop start for holds (`frames = (seconds + hold_s − 1.4) × 150`); the default (no hold) count is unchanged, and a default run reproduces `rebuild_s3/policy/B_s1_on.json` (checked in the previous round).
- **Rest definitions** (`experiments/policy_rest_table.py`): reference stop = last time after the hold start at which max over joints of |Δref/Δt| > 1e-4 rad/s; joints stop = last time at which max over joints of |qvel| > 1e-4 rad/s. The reference uses the bridge's accepted target minus dq (`layer.refs[-1][1]`), as the stream test's hand-error reference does.
- **(a) at 1 s and 2 s** is read from the per-tick CSV (`herr`, `rerr`) at the nearest row to `t_ref_stop + 1` and `+ 2`.
- **Runs.** `data/logs/policy_compare_20261125/h6/<case>/<stream>_s<seed>_on/` and `ref_<case>/A_s1_*`; queue `h6/h6_queue.sh`, progress `h6/progress.txt`; table `h6/rest_table.md`. 33 runs, all exit 0.

## Decision rows annotated (2026-11-25)

Note added to each row that quotes an affected perturbed held result: `design_plan.md` SF-124, SF-127, SF-131 and SF-138; `mpvi_decisions.md` M342. Each note says the perturbed held results cited there came from the harness without beaker compensation, and points to `docs/reports/20261125_1659_harness_fix_and_consistent_comparison.md`, which lists them.

## Next steps (not done)

- The joint rest criterion (1e-4 rad/s on every joint) is not reached in most runs. A rest test for the lab notebook would need a threshold above the creep (about 2e-4 rad/s here) or a longer hold; not changed.
