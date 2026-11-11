# Under-bench pick ik sweep and the moved layout (2026-11-09)

Untracked. Work in the copy `/home/vboxuser/projects/oc_elbow` (its own `src`, `.venv` linked to the real repo's venv). No commit, no push. No change to the real repo except this file.

Script: `/home/vboxuser/projects/oc_elbow/data/under_bench_ik_sweep_20261109.py` (modes `sweep` and `moved`). Data: `oc_elbow/data/under_bench_ik_sweep_sweep_20261109.json` (the doubled name is a typo in the script; the file is the sweep), `oc_elbow/data/under_bench_ik_sweep_moved_20261109.json`. Logs: `oc_elbow/data/logs/under_bench_ik_sweep_standard_20261109_185057.log`, `oc_elbow/data/logs/under_bench_ik_moved_candidates_20261109_185207.log`.

Method: pick ik at the NOMINAL stance only. Each pick task (the copy's plan has one pick task with 4 pick points), each of the 12 approach bins, `FullCheck._solve_stance` from the nominal stance, as the earlier nominal run did. Error stances not run. Body group in the moved run uses `fc.check(..., only="body")` as the earlier candidates run did.

## 1. Sweep, standard layout (x 0.775, heading -90 deg)

Edge = base y - 0.80 (the bench front edge is at y 0.800, see section 2). Torso-to-beaker = horizontal distance from the base origin (the torso origin in xy, see section 4) to the beaker centre (1.00, 0.66).

| pedestal (m) | base y | edge (m) | pick ik | angles passing (of 12) | failing point (name) | torso-to-beaker (m) |
|---|---|---|---|---|---|---|
| 0.16 | 1.105 | 0.305 | PASS | 2 | - | 0.499 |
| 0.16 | 1.080 | 0.280 | PASS | 2 | - | 0.477 |
| 0.16 | 1.055 | 0.255 | PASS | 2 | - | 0.455 |
| 0.16 | 1.030 | 0.230 | PASS | 2 | - | 0.433 |
| 0.16 | 1.005 | 0.205 | PASS | 2 | - | 0.412 |
| 0.16 | 0.980 | 0.180 | PASS | 2 | - | 0.391 |
| 0.16 | 0.955 | 0.155 | PASS | 1 | - | 0.371 |
| 0.16 | 0.930 | 0.130 | PASS | 1 | - | 0.352 |
| 0.12 | 1.105 | 0.305 | PASS | 2 | - | 0.499 |
| 0.12 | 1.080 | 0.280 | PASS | 2 | - | 0.477 |
| 0.12 | 1.055 | 0.255 | PASS | 2 | - | 0.455 |
| 0.12 | 1.030 | 0.230 | PASS | 1 | - | 0.433 |
| 0.12 | 1.005 | 0.205 | PASS | 1 | - | 0.412 |
| 0.12 | 0.980 | 0.180 | PASS | 1 | - | 0.391 |
| 0.12 | 0.955 | 0.155 | FAIL | 0 | point 0 'over' (z 1.16, bin 0 furthest; all 12 bins fail at point 0) | 0.371 |
| 0.12 | 0.930 | 0.130 | FAIL | 0 | point 0 'over' (z 1.16; all 12 bins fail at point 0) | 0.352 |

Notes:
- At pedestal 0.16 every stance down to base y 0.930 passes the nominal pick ik. The earlier failures (y 0.855 to 0.905) were not in this sweep.
- At pedestal 0.12 the stances at y 0.955 and 0.930 fail at the first point ('over', z 1.16 m) for all 12 bins.
- At y 1.105 at pedestal 0.12 the passing bins are 0 and 11; the failing points there are 'over' (0) and 'standoff' (1) in the other bins.
- Per stance the pick ik is run on the one pick task; with one task the "angles passing" is that task's count.

## 2. Object positions and distance from the bench front edge

The bench front edge is the +y face of the `bench_top` geom: y = 0.800 m. `bench_top` top at z 0.960 m. Positions are the body origins after `load_task` (no simulation step). The beaker is placed 2 mm above the bench top before settling (`beaker_settle_clearance`).

| layout | beaker centre (x, y, z) | beaker distance from front edge (centre / near face) | hotplate centre (x, y, z) | hotplate distance from front edge (centre / near face) |
|---|---|---|---|---|
| standard | (1.000, 0.660, 0.962) | 0.140 / 0.105 m | (0.800, 0.660, 0.960) | 0.140 / 0.050 m |
| moved (x shift -0.15) | (0.850, 0.660, 0.962) | 0.140 / 0.105 m | (0.650, 0.660, 0.960) | 0.140 / 0.050 m |

The near face is the object's y edge nearest the front edge (hotplate: from its geoms; beaker: its cylinder radius). The y distances do not change in the moved layout; only x changes by -0.15 m.

## 3. Code findings (no changes made)

- Object positions in the check: the check's model is built by `load_task(task_cfg)` in `FullCheck.__init__` (`src/full_check.py:250`). The beaker and hotplate are placed by `apply_task_poses` (`src/task_beaker_hotplate.py:122-139`: hotplate body_pos at line 137, beaker free-joint qpos at line 139), from `beaker_xy` and `hotplate_xy` of the `task_cfg` given. The check reads them from its own model, not from `objects`.
- The `objects` argument of `FullCheck.check` (`src/full_check.py:648`) holds unknown objects only, put in the spare slots by `set_objects` (`src/full_check.py:305`). The selection calls `full_check.check(stance, pts)` with no objects (`src/stance_select.py:585`), so it is `()`.
- The pick points come from `task_points` with the same `task_cfg` (`src/task_points.py:48`), and from the stance-time world (`src/stance_select.py:139` `step_target`, world from `StaticTaskSource(task_cfg)`, `src/world_model.py:333-340`).
- Why it always ran the standard layout: `FullCheck.__init__` defaults `task_cfg=TASK_CFG` (`src/full_check.py:238`), `make_context` (`src/stance_select.py:115`) and the `__init__` at `src/stance_select.py:66` default to `TASK_CFG`, and the under-bench scripts hard-code `TASK_CFG` (`oc_scripts/under_bench_boundary_20261109.py:10`, `oc_scripts/under_bench_ik_nominal_20261109.py:10-14`, `oc_scripts/under_bench_candidates_20261109.py:10-14`). The moved layout is a real option in the pipeline: `MOVED` is defined at `experiments/stance_select_moved_test.py:39` (same in `experiments/demo_video.py:43`), and `demo_video.py:96` passes `task_cfg=MOVED` to `make_pipeline_task`.
- Layout-independent `TASK_CFG` uses (home poses only): `src/full_check.py:98`, `:103`, `:282`, `:355`. The view points default to `TASK_CFG` in `src/stance_checks.py:169` (view group, not run here).
- To run the full check with the moved objects, the layout must be passed to all of: `FullCheck(task_cfg=MOVED)`, `make_context(..., MOVED, ...)`, `WorldModel([StaticTaskSource(MOVED)])`, so that the points match the scene. This needs no source edit: the run in section 5 does exactly that from a script.

## 4. Torso reference

`FullCheck.to_torso` (`src/full_check.py:360-365`) subtracts only the stance's x and y, so the torso origin is at the base origin in xy. The torso-to-beaker distance is therefore the xy distance from (x, y) of the stance to the beaker centre.

## 5. Moved-layout candidates (MOVED, one setup)

Layout: beaker (0.850, 0.660), hotplate (0.650, 0.660). Heading +90 deg, pedestal 0.15 m. Body group (all error stances, as the check does) and nominal pick ik.

| x | y | body group | min body distance (m) | pair at the minimum | pick ik nominal (angles passing) | failing point | torso-to-beaker (m) |
|---|---|---|---|---|---|---|---|
| 0.8769 | 0.3139 | FAIL | -0.3645 | openarm_body_link0_collision vs bench_top | FAIL (0 of 12) | 'standoff' (point 1, z 1.0125) | 0.347 |
| 0.8810 | 0.3149 | FAIL | -0.3645 | openarm_body_link0_collision vs bench_top | FAIL (0 of 12) | 'standoff' (point 1, z 1.0125) | 0.347 |
| 1.0269 | 0.3139 | FAIL | -0.3645 | openarm_body_link0_collision vs bench_top | FAIL (0 of 12) | 'standoff' (point 1, z 1.0125) | 0.389 |

The base at y 0.31 lies well inside the bench footprint (bench front edge y 0.80), so the body failure is expected from geometry alone. The pick ik was run regardless of the body result, as the script was written to do.

## 6. Wall time and free memory

| run | setup (s) | total wall (s) | free memory before | peak RSS |
|---|---|---|---|---|
| sweep, standard, 16 stances | 32.1 | 59.0 (loop 26.8) | 2.4 GB free (4.8 GB available) | 3.49 GB |
| moved candidates, 3 stances | 31.7 | 36.7 (loop 4.9) | 3.4 GB free (4.9 GB available) | 3.40 GB |

Both runs ran under `experiments/run_capped.sh` (6 GB systemd cap) with a 15-minute timeout; neither came close.

## 7. Not done and why

- Stances between y 0.930 and 0.855 at pedestal 0.12 or 0.16, and the y 0.855 to 0.905 stances from the earlier report, are outside the sweep asked for (y 1.105 to 0.930). Not run.
- No body group for the sweep stances: the request was pick ik only.
- No pick and place run, no simulation run, no fix chosen.

## 8. Not found

- No "moved layout" setting in a config file or command-line argument: the moved layout is a dict built in code (`MOVED`), so the run defines it in the script with the same formula as `experiments/stance_select_moved_test.py:39`.
- The earlier candidate runs' log files were not read (the real repo's data/logs are off limits).

## 9. Real repo

HEAD at the start of the session: 9ff0099. HEAD now: 0e84ccd, "Report: pedestal override removal stopped at the start (per-check seed build not committed)", by David Siff at 18:50:51, during this session. Not made by this session. No commits, no pushes made here. This file is the only change in the real repo (untracked). CLAUDE.md asks that reports be committed; not done, per the task's no-commit rule.
