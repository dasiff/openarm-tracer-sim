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
