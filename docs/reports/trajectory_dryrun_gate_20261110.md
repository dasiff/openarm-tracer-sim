# Heading variety, the stance generator, and the trajectory dry-run gate (2026-11-10)

Written for: David. Decisions: M301 (heading variety, generator), M302 (dry-run gate) in docs/mpvi_decisions.md. Feasibility study: docs/reports/trajectory_dryrun_feasibility_20261110.md.

(Results are added below after the two layout runs.)

## Results (two runs, one clock seed per layout; no batches)

Runs: `scratch/edge_stance/run_layout.py` (the pipeline's own selection, generator and episode). Logs: `data/logs/gen_staging_20261110_141847.log`, `data/logs/gen_moved_20261110_141847.log`.

| | staging (seed 4273723047) | moved (seed 1342282174) |
|---|---|---|
| candidates generated (passed the body-check gate) | 50 (4 gate failures) | 50 (3 gate failures) |
| candidates checked (full check) | 21 | 50 |
| passed full check | 5 (3, 13, 16, 17, 21) | 3 (4, 13, 25) |
| passed dry-run | 1 (21) | 0 |
| yielded stance | x 0.6302, y 1.0523, heading -77.6 deg, pedestal 0.1237 m, A9 score 3.000 | none: "no candidate passed both the full check and the dry-run" |
| dry-run failures | 3 place OVER (30.4 mm off reference), 1 pick IN (4.5 mm short), 1 pick OVER (17.2 mm short) | 2 place OVER (30.4 mm), 1 pick OVER (30.0 mm off reference) |
| dry-run wall time (total, each run) | 154 s (5 runs; 50.5 s for the passing one) | 70 s (3 runs) |
| full-check wall time (total) | 662 s | 1190 s |
| end-to-end | SUCCESS, 14,420 ticks, 96.1 s sim | not run (no stance) |
| peak memory | 4.1 GB | 4.1 GB |

Reading: the dry-run gate rejected 8 of 8 full-check passes that reached it. Its two commonest failures are the 30 mm reference tolerance at place OVER and pick OVER; the tolerance is a proposed value (DRYRUN_CFG ref_tol_m), not decided. Most full-check failures are the eagle image occlusion and the arm-to-hotplate-beaker clearances, not the dry-run.

Caveats: the first attempts were killed by the system's memory limit, caused by a concurrent job of another session (not by this code). The dry-run memory leak was fixed in the commit before these runs, and the check's logs are now cleared per candidate.

## Results after the criterion and sigma change (sigma 15 deg; dry-run fails only on timeout or IK divergence)

Logs: `data/logs/gen5_staging_20261110_195401.log`, `data/logs/gen5_moved_20261110_195401.log`. One clock seed per layout.

| | staging (seed 2409916947) | moved (seed 3342821395) |
|---|---|---|
| candidates generated (passed the body-check gate) | 50 (4 gate failures) | 50 (2 gate failures) |
| candidates checked (full check) | 5 | 50 |
| passed full check | 1 (candidate 5) | 9 (1, 9, 14, 16, 21, 25, 28, 38, 39) |
| passed dry-run | 1 (candidate 5) | 0 |
| yielded stance | x 0.6785, y 1.0696, heading -84.6 deg, pedestal 0.1132 m, A9 score 3.199 | none: "no candidate passed both the full check and the dry-run" |
| dry-run wall time | 19.3 s (1 run) | 175.9 s total (9 runs) |
| full-check wall time | 34.7 s of the yielded candidate's check; the five checks total about 160 s | 836.5 s total (50 checks) |
| end-to-end | SUCCESS, 14,391 ticks, 95.9 s sim | not run (no stance) |

Comparison with the previous criterion run: the yielded candidate moved from 21 to 5. Under the new criterion the staging dry-run passed on its first attempt.
Moved: dry-run failures are timeouts at place OVER (up to 678 mm from the waypoint, candidate 38) and at pick IN (3.6 to 4.9 mm short). The earlier 30 mm reference check had been catching these as divergence.
