# Stance check failure logs and replay (M281), 2026-11-09

Logging only. No check decides differently, no random draw is added or moved, and no reseed was added (David's decision: the generator stays unreset). Not pushed.

## 1. How the generator is seeded and where random numbers are drawn

- Seeding: `np.random.default_rng(11)`, a fixed number, set once when a `FullCheck` is built (`src/full_check.py` `__init__`). It is not reseeded per check, so each check's random starts depend on how many draws earlier checks in the same process made. The clock is not used.
- Draws on the stance check path (`src/full_check.py`, all from `self.rng`): the random starts of `_solve_pose_point` (skill stops, phase "random", `ik_seeds` = 24 starts) and of `_solve_stance` (grasp bins, phase "random"). Everything else on the path (clearances, body distance, view, occlusion, `_place_window`) draws nothing.
- Pipeline and mock runs: each pipeline run builds its own `FullCheck` (`src/stance_select.py`), so it starts from the same seed-11 state. The mock policy (`src/mock_policy.py`) draws nothing.
- Other draws, not on the check path: `src/task_beaker_hotplate.py` home variation (own generator, `home_variation_seed` 0, zero spread by default); `src/home_clearance_map.py` (`default_rng(0)`, map build); `src/reach_map.py` (map build, config seeds); `src/llm_gemini.py` (Python `random`, unseeded, used only for the retry delay); `src/pour_scene.py`; `experiments/layout_sweep.py` (`random.Random(seed)`).

## 2. What is logged

- `data/logs/stance_failures/checks.jsonl`: every computed check (cache hits are not logged). Each line holds the full input (stance, all points, objects, approach bin, `all_angles`), the generator state before the check, the scene state (qpos, qvel, object poses), the answer, the config in force, and the commit hash and dirty flag. The first line of each process is a run header with the generator state at the start.
- `data/logs/stance_failures/failures.jsonl`: the failed checks only. Each also has every check group run alone from the same start state: clearance, body, ik, place, view, occlusion. Each group records its ok, failed, reason and measured values against their limits. For ik this includes the failing point per approach bin, the approach angles tried, and the passing bins. For place it includes the widths and the failing stop per turn.
- Pipeline and mock test logs print the generator state at the start of the run (`src/stance_failure_log.run_header`).
- Settings: `STANCE_LOG_CFG` in `src/task_config.py` (on, folder, independent_checks). Kept apart from `FULLCHECK_CFG`, which feeds the cache key.
- Mechanism: `check()` takes `only=<group>` for the diagnostic runs; the normal call still stops at its first failing item. Before a failure's group runs, the generator and the whole simulation state are restored (`_snapshot`/`_restore`), and they are restored again afterwards.
- Size: about 9 KB per failure record.

## 3. Six-candidate run (`experiments/stance_failure_six_run.py`)

The stances are the overnight given-order rows (`data/logs/overnight_20261109/B_given_noreseed.jsonl`), exact to 4 decimals. The logged run (log folder `data/logs/m281/20261109_123428/`) and a control run with logging off gave identical answers (ok, failed, reason) for all six.

| row | candidate (x, y, heading deg, pedestal) | overnight given order | six-run (logged = control) | replay (fresh process from the saved state) |
|---|---|---|---|---|
| 17 | (0.6041, 1.001, -60, 0.10) flip | PASS | PASS | NOT reproduced: replay FAIL at ik (point 18); ok, failed and reason differ; 13 s |
| 18 | (0.6655, 0.9837, -70, 0.20) flip | FAIL (ik) | FAIL (ik) | reproduced, 28 of 28 fields; 55 s |
| 21 | (0.5952, 1.0023, -60, 0.10) flip | PASS | PASS | reproduced, 4 of 4 fields; 48 s |
| 24 | (0.6037, 0.9805, -60, 0.15) other | FAIL (ik) | FAIL (ik) | 25 of 28: ok and failed match; reason and ik measured differ (failing point 2 saved, 10 replayed); 47 s |
| 37 | (1.1404, 0.9145, -130, 0.20) other | FAIL (place window) | FAIL (place window) | reproduced, 28 of 28 fields; 108 s |
| 45 | (0.9963, 1.0488, -100, 0.10) other | FAIL (place window, 5 deg) | FAIL (place window) | reproduced, 28 of 28 fields; 111 s |

Free memory before each replay was 5.1 to 5.6 GB available. Each replay ran alone (one MuJoCo process).

Open: rows 17 and 24 do not reproduce from the saved state. Checked and ruled out: the generator state (row 1's saved state equals a fresh seed-11 state), the hash seed (replay gives the same result under PYTHONHASHSEED 0 and 1), and building the points first (the same result as the logged process). So something outside the saved state matters, but its cause is not isolated. Candidates not yet checked: the sim's non-position data (warm-start, ctrl), and state inside the arm collision checkers. Row 17 is a plain pass-to-fail change, the same kind of effect as the overnight flips.

## 4. The 51-stance count

Not rerun (David's instruction). The overnight reference is 23 of 51 (given order, no reseed).

## 5. Existing stance test files (only these, each under a 5 minute timeout; logging on)

| file | now | earlier (latest recorded) | note |
|---|---|---|---|
| full_check_test | 30/31, exit 1 | 31/31 | the failing check is "fails fast with the reason": 3.86 s against a limit of 1.5 s. The answer is the same (home_clearance, same reason). The extra time is the six group runs after a failure. Not confirmed with a logging-off run. |
| transit_clearance_test | 26/27, exit 1 | 26/27 | same failing check as before |
| stance_select_test | timed out at 300 s (exit 124), not done | IndexError crash (line 117) | the run got past three checks before the timeout; the test is not finished |
| stance_select_moved_test | 5/9, exit 1 | 6/9 (11-08), 5/9 (handoff) | failure: CLOSE timeout at the pick; replay not bit-identical (known) |
| stance_select_pipeline_test | 3/7, exit 1 | 4/7 (Nov 6, two runs) | changed; not isolated. The episode fails at place recovery (HOME). Logging-off run needed to settle it. |

The bit-identical replay failure in the moved and pipeline tests is listed as pre-existing in the handoff.

## 6. Diff review (what changed in the stance check and run code)

- `src/full_check.py` `check()`: `only` parameter (default None: no change to the normal path). `points_all` keeps the caller's list for the log. `_measure` is written but not returned. The cache write moved inside `if only is None` (same behaviour). Each group keeps its order and its early returns on the normal path; the groups run unchanged in order. Purpose: diagnostic groups. No decision, draw or draw order changed.
- `src/full_check.py` `_log_computed`, `_snapshot`, `_restore`, `_config_in_force`: write files, copy and restore state. The independent runs draw random numbers, but the generator is restored afterwards, so the next check sees the same stream.
- `src/full_check.py` `__init__`: new constructor arguments and attributes; the log folder is created only when logging is on.
- `src/stance_failure_log.py` (new): JSON writing, git hash, run header. No draws.
- `src/stance_select.py`: passes `maps_dir` (logging only).
- Four pipeline and mock tests: print the run header only.
- `src/task_config.py`: `STANCE_LOG_CFG`.
- `docs/mpvi_decisions.md`: row M281.

## Commits (local, not pushed)

b6fe404 (logger, config, check groups), bac3385 (test headers, maps_dir), 7a2fb41 (replay and six-run scripts), b255b40 (progress WIP), 8732fce (points fix and script paths), dbca9e6 (progress).
