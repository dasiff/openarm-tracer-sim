# Bounce run with the held-mass ramp: result (branch held-weight-ramp-sag-off)

Written for: David (design review). Build commit 8e24282; logs data/logs/bounce_ramp_new_20261102_162317.log.

## Outcome
The new run FAILED before the place. Sag gain is 0 everywhere (M215), so the scripted pick's IN move does not reach its waypoint:
"IN: not reached after 15.1 s: 3.3 mm and 1.1 deg" (tolerance 3 mm). This happened on the first attempt (IN at 71.2 s) and on the retry (IN at 100.1 s),
then the plan stopped (retry_max 1). No handoff weighing, ramp-out or place happened, so no new-run metrics exist. VERIFIED (log lines).
Meaning: the droop correction that the pick relied on is gone; this is the limit David recorded (M223, item 2), and it blocks the test.

## Existing runs (metrics from the per-tick extracts, window T0..T0+200, T0 = 11599)
| run | baseline (mm) | rise (mm) | drop (mm) | swing (mm) | place time (s) | final beaker x, y, z (m) |
|---|---|---|---|---|---|---|
| sag on (existing) | 0 | +17.70 | -18.45 | 36.14 | 4.70 | 0.6504, 0.6601, 1.0279 |
| sag off in place only (existing) | 0 | +2.99 | -4.05 | 7.04 | 4.70 | 0.6504, 0.6601, 1.0279 |

VERIFIED: the numbers are computed by experiments/bounce_ramp_metrics.py from the extracts and episode logs.
NOT FOUND: pad height at the end of DOWN. The extracts cover ticks 11560 to 11800 only and carry phases CHECK and OVER there, so DOWN is outside them.
Needs a full-run extract (replay), which I have not run.
INFERRED: the two runs' final beaker poses are identical to 4 decimals and their phase timing is identical. Not checked further (possible copy).

## Replay (question 2): what the recording holds
VERIFIED from data/diag/logging_20261101/post_bridge/recording/chunks: heldmass has one value per tick for every tick (13,613 ticks).
It is 0 before tick 11,599, 0.205 kg at 11,599, and back to 0 at 12,308 (release). It is not a list of target changes.
Replay (src/control_loop.py, replay path, and experiments/bounce_diag_extract.py) writes each recorded value through set_held_mass each tick.
After the build, the feedforward no longer reads that store, so replay of held-mass runs will not reproduce. Not changed, per instruction.

## Recovery wait (question 1)
NOT MEASURED: the run never reached a recovery open. INFERRED from the design: the open waits up to the ramp duration (1.0 s) when a mass is set.

## Open choices for David
(a) Keep sag on during the pick (gain 0 only from the place onward) instead of everywhere. This departs from decision 1.
(b) Loosen the 3 mm pick tolerance for this test. Not a listed choice, so I have not done it.
(c) Stop the bounce test at the pick failure and record it as the result.
Also: replay (question 2) needs a decision before any replay-based check.
