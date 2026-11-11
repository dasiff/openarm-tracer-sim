# Why row 17 does not replay (and row 24 replays differently), 2026-11-09

Read-only study. No simulator run, no code or config change, no commit, no push. Inputs: the saved files only
(data/logs/stance_failures/checks.jsonl and failures.jsonl, data/logs/m281/, data/logs/m282/, data/logs/*chain*.log, git history).
Helper: /home/vboxuser/projects/oc_scripts/row17_record_diff.py (reads the records, prints the field comparison).

Git log at the start (`git log --oneline -10`):

```
0e84ccd Report: pedestal override removal stopped at the start (per-check seed build not committed)
9ff0099 Report: full simulation state and group seconds in stance records (M282)
6435ae0 Full state: the mass matrix array is M in this MuJoCo binding (fixes the M282 save)
4f77df6 Stance check records: group seconds and the full simulation state; replay restores it (M282)
237ff7a Stance check records: full simulation state helpers and config (M282)
e4a7c07 Report: stance check failure logs and replay (M281)
dbca9e6 Progress: M281 six-candidate run started (chain m281)
8732fce Stance check log keeps every point, scripts find src (M281)
7a2fb41 Stance failure replay and the six-candidate run (M281)
bac3385 Pipeline and mock test logs: generator state at the start of the run (M281)
```

## The runs being compared

| run | record id(s) | commit | tracked dirty | started | how started |
|---|---|---|---|---|---|
| M281 logged | 250635-20261109123435-1 (row 17), -4 (row 24) | 8732fce | false | 12:34:28 (chain m281_chain_20261109_123428.log) | `experiments/stance_failure_six_run.py logged`, via `python` in scratch/chain_m281.sh, `MUJOCO_GL=egl` exported |
| M281 control | (no records, logging off) | dbca9e6 (src identical to 8732fce) | false | 12:40:15 | `stance_failure_six_run.py control`, same chain, after the logged run |
| M281 replays | replays of the M281 records | dbca9e6 (not logged by the replay) | false | 12:43:27 to 12:49:49, one at a time | `stance_failure_replay.py <file> <id>` |
| M282 logged | 255700-20261109142800-1 (row 17), -4 (row 24) | 6435ae0 | false | 14:28:27 (chain m282_chain_20261109_142754.log) | same six-run script, after the full-state change |
| M282 replays | replays of the M282 records | 6435ae0 (chain started 14:27:54, HEAD 6435ae0 at the time) | false | 14:33:04 to 14:34:20 | same replay script |

The M281 control and logged runs had identical answers on all six stances (chain log: COMPARE IDENTICAL).

## 1. Field-by-field comparison of the row 17 and row 24 inputs

The replay has no input of its own. It loads the saved record and calls `fc.check` with the saved stance, points, objects and approach
bin, after restoring the saved generator state and scene (`experiments/stance_failure_replay.py` lines 38 to 70 at 0e84ccd). So the replay
input is the saved record. The control side's live input is only available as what the control run wrote, which is the same record.
The table compares the two saved records (M281 = control run record; M282 = the record the M282 replay used). Values were compared
bit for bit (IEEE-754 bit patterns, so -0.0 and last-digit differences show).

### Row 17 (control record 250635-20261109123435-1 vs M282 record 255700-20261109142800-1)

| field | M281 control record | M282 record | result |
|---|---|---|---|
| stance (x, y) | 0.6041, 1.001 | 0.6041, 1.001 | identical |
| stance heading_deg / stance_raw[2] | -59.99999999999999 / -1.0471975511965976 | same | identical |
| pedestal (stance_raw[3]) | 0.1 | 0.1 | identical |
| arm, tasks, approach_bin, all_angles | left, [1, 3], null, false | same | identical |
| points (all 10, every field, incl. pick_path, band, beaker_xy) | as saved | identical bits | identical |
| objects | [] | [] | identical |
| object_positions (beaker_free_joint_q, hotplate_body_pos) | [1.0, 0.66, 0.962, 1,0,0,0], [0.8, 0.66, 0.96] | same | identical |
| pedestal_heights | [0.1, 0.12, 0.08] | same | identical |
| generator_state_before (PCG64) | state 142231634183583599241308901332361415022, inc 7937318808080196428804369945471644491 | same | identical; equals a fresh default_rng(11) state |
| scene_before.qpos, qvel, q_scene | as saved | identical bits | identical |
| scene_before.sim_state (full physics state) | NOT SAVED (M281 has no full state) | saved | differs in existence only |
| maps_dir | data/reach_map_before_bench | same | identical (map files unchanged since 2026-10-22) |
| cache_key | a2db6b417461 | same | identical |
| config in force (FULLCHECK_CFG, STANCE_REQUIREMENTS, PICK_PLACE_CFG, TASK_CFG_home) | as saved | identical | identical |
| config.STANCE_LOG_CFG.full_state | NOT SAVED | True | differs in existence only (logging setting) |
| commit / dirty | 8732fce / false | 6435ae0 / false | differs (commit) |
| time, id, run | 12:35:44, ...-1, 250635-... | 14:29:04, ...-1, 255700-... | differs (identity only) |
| seconds (check part timings) | NOT SAVED | saved | differs in existence only |

Only the commit, time, id/run, and the M282 additions (sim_state, full_state key, seconds) differ. None of the solve inputs differ.

### Row 24 (control record 250635-20261109123435-4 vs M282 record 255700-20261109142800-4)

Same stance, points, objects, scene, generator state and config (identical bits, as for row 17). The helper lists 13 differing leaves, all of
the kinds above plus the group `seconds` (NOT SAVED in M281). Row 24 stance: x 0.6037, y 0.9805, heading -59.99999999999999 deg
(-1.0471975511965976 rad), pedestal 0.15.

### What is not saved on either side (differences that could matter)

- The control run's in-memory points were passed through `jsonable` (`src/stance_failure_log.py` line 18) before writing: tuples become lists, numpy
  values become Python numbers (exact for float64), dict keys become strings, and any other object is written with `default=str`. The saved points
  contain only float, str, None and int values, and no non-str dict keys (helper output). So no lossy case is visible in the saved points. A
  tuple-versus-list difference cannot be seen in JSON; it would not change the solve (`np.asarray(..., float)` treats both the same).
- The replay in M281 restored only qpos and qvel and then ran `mj_forward` (old-record branch, `stance_failure_replay.py` lines 50 to 53).
  The control run did not run that step. The M282 replay restored the full physics state and checked the round trip (exact: True in both
  replay logs), and still failed in the same place. So the difference in the scene state is not the cause.
- The replay does not log its own commit, dirty flag or input, so its code version is only known from the timeline (see section 4).
- Process state that the record does not hold: the arm-checker LRU (`FullCheck._checkers`, `src/full_check.py` lines 341 to 358), the
  clearance and occlusion objects, renderers, and the effect of `build_points` on the control process. The background to this task says the
  replay also failed after `build_points` first; that run's output is not in the saved files.

## 2. Failing point 18 (row 17 replay) and the control run's values

### Limits (from config, src/task_config.py)

Point 18 is a grasp point (`pose` not "skill" or "place"; record shows "grasp"), so the grasp constraint applies (`src/grasp_rules.py` make_constraint, lines 115 to 162):

| quantity | limit | source |
|---|---|---|
| position error of the pad centre | < 0.5 mm (`ik_pos_tol` 0.0005 m) | task_config.py line 203 |
| approach axis elevation, finger axis elevation | `|z| <= sin(5 deg)` = 0.0872 (`level_deg` 5.0) | task_config.py line 192 |
| approach azimuth (bin k) | within 15 deg of the bin centre (`bin_half_deg` 15.0; 12 bins) | task_config.py lines 195, 196 |
| rule rows (push back) | start at `ik_soft` 0.8 of the tolerance | task_config.py |
| solver | `ik_max_iter` 40, `ik_damping` 0.02, `ik_max_step` 0.3 rad | task_config.py lines 204 to 206 |

Points 2 and 18 use the same grasp constraint. Skill/place points (pose constraint, 3 mm and 2 deg) do not apply to point 18.

### Measured values

| | control run, row 17 (record 250635-...-1) | replay, row 17 (M281 replay and M282 replay) |
|---|---|---|
| check result | ok True, failed [] | ok False, failed ['ik'] |
| point 18 result | solved in bin 11 at all nine error stances (approach_angles_passing task 1 = [11]) | "task 1 (pick): at the stance 'heading+1' point 18 (grasp, [1.0, 0.66, 1.005]) has no collision-free solution i..." (full reason not saved; the replay printed only 110 characters) |
| best position error at point 18 | NOT SAVED | NOT SAVED |
| best orientation error at point 18 | NOT SAVED | NOT SAVED |
| bins that pass / fail | bin 11 passes; the other bins are not recorded (passing checks save only `approach_angles_passing`) | not saved (furthest bin is in the cut-off reason) |

Answer to "is the control run's value just under the limit and the replay just over?": **not found.** No record holds the position or
orientation error of any solve. `_solve_stance` (`src/full_check.py` lines 470 to 531) returns no errors, and `_log_computed` writes only the
passing bins for a passing check. The M281 replay did not print the error either. To answer it, a run must save the best error and the
angle-bin table for each point (see section 7).

### Row 24 (control failure vs replay failure), with the saved per-bin table

Row 24 failed in the control run, so the per-bin table is saved (failures.jsonl, `independent.ik.measured`):

| bin | control: first failing stance / point | control: point name and p |
|---|---|---|
| 0 | nominal, point 1 | standoff [0.94, 0.764, 1.012] |
| 1 to 10 | nominal, point 0 | over (0.896 to 1.120, 0.54 to 0.764, 1.16) |
| 11 | heading+1, point 2 | grasp [1.0, 0.66, 1.005] |

Replay (M281 and M282, identical): `task 1 (pick): at the stance 'heading+1' point 10 (grasp, [1.0, 0.66, 1.005])`. Point 10 differs
from the control's point 2. The replay's per-bin table was printed truncated, so the other bins are not comparable. Position and
orientation errors: NOT SAVED on either side.

Both control rows 17 and 24 failed or passed at heading+1 at a pick point, which fits a near-boundary stance. But no error value is saved,
so this is a hypothesis only.

## 3. Solve code: time, threads, CPUs, per-process settings

| source | file and line | dependency on time, threads, CPUs or process settings |
|---|---|---|
| `solve_ik` | src/arm_kinematics.py lines 155 to 200 | none: fixed `max_iter`, no time budget. `np.linalg.solve` (line 195) and `@` (line 196) use numpy's BLAS (below). |
| `_solve_pose_point` | src/full_check.py lines 425 to 468 | none. `time.perf_counter()` (lines 441, 450, 455, 457) only accumulates `self.seconds`; no decision uses it. Random starts from `self.rng` (line 440). |
| `_solve_stance` | src/full_check.py lines 470 to 531 | none. Timing at lines 506, 512, 516, 518 is for `seconds` only. Random starts from `self.rng` (line 505). |
| `check` | src/full_check.py lines 648 to 720 | `time.perf_counter` for totals (lines 666, 674, 687, 696, 698) only; none decides. |
| wall-clock decision (test only) | experiments/full_check_test.py line 86 | `time.perf_counter() - t0 < 1.5` ("fails fast"). This test failed in the M282 run (4.40 s) and in the 14:34 test run. It is a test, not the solver. |
| run id and record time | src/full_check.py lines 279, 618 | `os.getpid()` and the clock in ids and records only. |
| threads | no `OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS`, `threadpoolctl`, `os.cpu_count`, `ThreadPool` or MuJoCo thread pool in src/ or experiments/ (grep) | none set by the project. |
| BLAS (library) | numpy 2.2.6 with scipy-openblas 0.3.29 (`MAX_THREADS=64`, DYNAMIC_ARCH, Haswell kernels); VM has 4 CPUs | Not set by the project. OpenBLAS may thread a call of large size; the calls here are small stacked matrices (3x7, 7x7 per start). Threading would change the last bits only if a reduction is split differently. Not verified. |
| MuJoCo | no `mj_setNumThreads`-style call; no thread pool on the check path | none found. |
| rendering (occlusion group only) | src/stance_checks.py `OcclusionMeter` (EGL renderer) | a software renderer may depend on the CPU count. It runs after the ik group, so it cannot change the ik failure in rows 17 and 24. |
| the only random draws | src/full_check.py line 276 (`default_rng(11)`), lines 440 and 505 | deterministic per process, no time or CPU dependence. The seed is set once per FullCheck and advances across checks, so a check's draws depend on the checks before it in the same process. |

Conclusion for item 3: nothing in the solve path depends on time, the thread count or the CPU count, except through BLAS, whose thread
behaviour is not verified. The only process-level state that carries from one check to the next is the shared random stream
(lines 276, 440, 505) and the checker LRU (`_checkers`, lines 341 to 358). Row 17 is the first check of its process in both runs, so
the stream and the LRU start the same way.

## 4. Other jobs on the VM and how each run was started

Timeline from chain logs (`data/logs/m281_chain_20261109_*.log`, `data/logs/m282_chain_20261109_*.log`, `data/logs/m281_tests_chain_20261109_125159.log`) and file times:

| time | event | source |
|---|---|---|
| 12:00 | overnight folder last written | data/logs/overnight_20261109 mtime |
| 12:31:36 | M281 chain attempt: logged and control both exit 1 at once (bugged start) | m281_chain_20261109_123136.log |
| 12:32:03 | M281 chain attempt: logged started, no exit line (no output kept) | m281_chain_20261109_123203.log |
| 12:34:28 | M281 chain: logged six-run start, procs 0 | m281_chain_20261109_123428.log |
| 12:40:15 | logged run exit 0; control start, free 5625 MB | same |
| 12:43:27 | control exit 0; replays start one at a time (procs 0 at each start) | same |
| 12:49:49 | last replay (row 45) exit 0; chain DONE | same |
| 12:51:59 to 13:11:00 | M281 tests chain (full_check_test, transit, stance_select, ...) | m281_tests_chain_20261109_125159.log |
| 14:26:00 to 14:27:35 | M282 chain attempt 1: six-run exit 1 (commit 4f77df6, M-matrix bug), full_check_test exit 1 | m282_chain_20261109_142600.log |
| 14:27:54 | M282 chain attempt 2 start: procs 1 (not identified; see below) | m282_chain_20261109_142754.log |
| 14:28:27 to 14:33:04 | M282 six-run logged (6435ae0) | same |
| 14:33:04 to 14:34:20 | M282 replays of rows 17 and 24, one at a time | same |
| 14:34:21 to 14:37:05 | full_check_test (one failure, 30/31 in the record) | same |

No overlap between the runs of either chain. Other sessions on the VM were not logged, so whether another process ran at the same time is
only known from these chain logs. The "procs 1" at 14:27:54 has no identified process; the chain's own `pgrep` pattern can match the shell
that runs it (the same pattern gave "procs 1" in the M283 chain before any job started), so it may be a false count.

Start methods that differ between the control and the replay:

| | M281 control (row 17 record) | M281 replay | M282 record run / replay |
|---|---|---|---|
| script | six_run.py control | replay.py | six_run.py logged / replay.py |
| FullCheck failure_log_dir | None (no logs) | None | STANCE_LOG_CFG dir / None |
| build_points before the check | yes | no | yes / no |
| scene restore | live (no restore) | qpos, qvel, mj_forward | full state, exact round trip |
| environment | `MUJOCO_GL=egl` exported in the chain (scratch/chain_m281.sh) | same chain | same (scratch/chain_m282.sh) |
| working directory | repo root | repo root | repo root |
| commit | dbca9e6 (src = 8732fce) | dbca9e6 | 6435ae0 / 6435ae0 |
| uncommitted tracked changes | none | none | none |

## 5. Row 24 with items 1 and 2

Item 1 (the field list): see the row 24 section above; the only differences are the commit, time, ids, the M282 additions and the
group seconds. Item 2: the control's saved per-bin table is in section 2; replay point 10 is not comparable to the control's point 2
because the replay's bins are not saved. Position and orientation errors: NOT SAVED.

## 6. Not found

- Position and orientation error of the best solve at any point, in any run (control or replay), for rows 17 and 24.
- The full reason string of either replay (cut to 110 characters on screen; not written to a file).
- The replay's own commit, dirty flag and input (the replay writes no log line).
- A record of the process environment (environment variables, thread counts, CPU load) for any run.
- What process "procs 1" at 14:27:54 was.
- The M281 "123203" chain attempt's output (no exit line, no log kept).
- The `build_points` replay run mentioned in the background (no file in data/logs).

## 7. What needs a run (not done here; no simulator runs in this study)

1. A run that saves, for each failing point, the best position error and orientation error and the pass/fail per bin, in both the logged and the replayed check.
2. A run that prints the full reason of each replay to a file.
3. A run that logs the thread count and the BLAS configuration (`numpy.show_config()` or `threadpoolctl.threadpool_info()`) at each process start.
4. A run that sets `OPENBLAS_NUM_THREADS=1` in both the control and the replay, to test the BLAS hypothesis (section 3).
5. A check of the arm-collision checker LRU: a replay that calls the checks of the control run's earlier stances first (the record does not hold them).

## 8. Notes on the study's own conclusions

- The inputs are the same down to the last bit (section 1); the saved records cannot show why the replay answers differently.
- The per-check seed change (M283, separate report) removes one cross-check dependence, but row 17 was already the first check of its process, so it does not by itself explain the row 17 difference.
