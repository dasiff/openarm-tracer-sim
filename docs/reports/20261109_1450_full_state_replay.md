# Full simulation state in the stance check records, and the replay (M282), 2026-11-09

Logging only. No seeding change, no check-order change, no cache change, no result change. Not pushed.

## What was added

- Each record (checks.jsonl and failures.jsonl) now holds the whole simulation state at the start of the check: MuJoCo's full-physics state (`mj_getState`, signature `mjSTATE_FULLPHYSICS`: time, qpos, qvel, act, ctrl, the solver's warm start, mocap, user data), plus 23 derived arrays (kinematics, mass matrix, bias and passive forces). Setting: `STANCE_LOG_CFG["full_state"]` (True).
- Seconds: each computed check record holds the check's own part timings (`seconds`). Each failure record holds each group's timings from its run alone (`independent[g].seconds`).
- The replay restores the saved state with `mj_setState` plus the derived arrays (no forward step), then saves it again and compares; the round trip was exact for all replays.

## Record size

| record | before | after |
|---|---|---|
| failure record (typical) | about 12.5 KB | about 118 KB |
| of which the full state | (qpos and qvel only, 1.6 KB) | 104.7 KB (physics vector 1 KB; derived 103 KB) |

Largest derived arrays: geom_xmat 28.5 KB, geom_xpos 11.1 KB, cinert 10.3 KB, qLD 8.6 KB, M 8.3 KB, ximat 7.4 KB.

## Group timings (new records, run 255700-20261109142800, all failing checks)

| check | clearance | body | ik | place | view | occlusion |
|---|---|---|---|---|---|---|
| row 18 | 0.09 s | 0.25 s | 2.7 s | 33.4 s | 0.002 s | 0.41 s |
| row 24 | 0.15 s | 0.22 s | 1.9 s | 35.5 s | 0.001 s | 0.36 s |
| row 37 | 0.13 s | 0.29 s | 5.0 s | 25.3 s | 0.003 s | 0.39 s |
| row 45 | 0.10 s | 0.08 s | 3.3 s | 26.5 s | 0.001 s | 0.34 s |

The place group is about 90% of the cost of a failed check's group runs. Turning off the place group would remove most of the cost, but the place group is the only one that gives the place result.

## Replays

| row | new record | state restored, round trip | ok | failed | failing point | replay time | reproduced |
|---|---|---|---|---|---|---|---|
| 17 (0.6041, 1.001, -60, 0.10) | 255700-20261109142800-1 | exact | saved True, replayed False | saved none, replayed ik | saved none, replayed point 18 | 13 s | **no** |
| 24 (0.6037, 0.9805, -60, 0.15) | 255700-20261109142800-4 | exact | False / False | ik / ik | saved point 2, replayed point 10 | 63 s | **no** (ok and failed match; reason, point and ik measured differ) |

Free memory before each replay: 5530 MB (row 17), 5481 MB (row 24).

Row 17 did not reproduce after the full state was restored. Per the task, I stopped here. What was saved and compared: the full physics state, the derived arrays, the generator state, the scene's object poses and the points. What was not saved: the arm collision checker cache, the path clearance and stance checks data, and the renderers. Row 17 is the first check of its process, so the cause is one of those, or something outside all of these.

Row 24's point difference (2 saved, 10 replayed) is the same as before the state was saved, so the full state did not change it either.

## full_check_test

30/31, exit 1 (same as the previous run). The failing check is still the timing one ("fails fast", 4.40 s against 1.5 s). The check's result (home_clearance) is unchanged. Run time 164 s.

## Notes

- A first attempt failed at the first check: `qM` is not an attribute in this MuJoCo binding (the mass matrix is `M`). Fixed in 6435ae0 and rerun. The first attempt's partial lines (run header only) are in checks.jsonl and do not feed any result.
- The derived-array list is a fixed list (`src/stance_failure_log.py`, DERIVED_ARRAYS).

## Commits (local)

237ff7a (state helpers, config, M282 row), 4f77df6 (records and replay), 6435ae0 (array name fix), and this report.
