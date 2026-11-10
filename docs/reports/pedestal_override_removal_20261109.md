# Pedestal override removal (2026-11-09)

Written for: David (decision owner), and whoever reads the run history later.

Decision (David, 2026-11-09): the forced pedestal override is removed everywhere. The staging parks the pedestal at the height the stance selection chose and the full check passed. Decision row M284 (`docs/mpvi_decisions.md`). M197 and M270 are kept as history.

## 0. Push

Pushed `a85a070..c74a205` to origin/main as a fast-forward (history kept, no force). Nothing after c74a205 was pushed.

## 1. Every place the override or a fixed 0.16 m was set, read or forced

| file | line (before) | what it did | what changed |
|---|---|---|---|
| src/task_config.py | 598 | `BENCH_STANCE_CFG["pedestal_override_m"] = 0.16` (M197, M270) | key deleted (not switched off) |
| src/stance_select.py | 622-623 | parked the staging at the override when set, else at the selected height | parks at the selected stance's own height (`st["q"]`, the height the full check passed) |
| src/mpvi_pipeline.py | 117-118 | task meta had no pedestal record | task meta `pedestal` stores `selected_m` and `checked_m` (the height the full check passed at) |
| src/episode_runner.py | 168, 208, 289 | no pedestal record in the episode summary | summary `pedestal` = selected/checked heights and the commanded and measured pedestal height at each phase's first tick |
| experiments/comp_episode.py | 7, 25-34 | `--pedestal` option set the override in-process | option and line removed (docstring updated) |
| experiments/reach_inverse_scene.py | 26, 31 | `PEDESTAL = BENCH_STANCE_CFG["pedestal_override_m"]` (0.16) | `PEDESTAL = 0.16` as a labelled constant: the height of the saved shoulder reading (`data/reach_heatmap/shoulder_q016.json`); not the run's pedestal |
| experiments/pedestal_episode.py | 1-35 | monkeypatches `apply_stance_selection` to force an arbitrary pedestal height for one run (measurement script, 2026-10-28) | **NOT changed**: still forces a height. See DECISIONS NEEDED |
| experiments/full_check_test.py | 230 | picks the 0.16 column of a test grid (`qs`) to sample | NOT changed: a grid choice in a test, not the override |
| docs/design_plan.md | 250 | no marker mentioned the override | added the M284 note to the stance item |
| docs/handoff_20261109.md | 51, 69 | describes `pedestal_override_m` and "park at 0.16" | NOT changed: a dated handoff snapshot (see NOT DONE) |
| docs/progress.md, docs/reach_map_indexing.md, docs/mpvi_decisions.md M197/M270 | history | — | not rewritten |

Unchanged on purpose: the elbow guard (OFF, M278), the sag trim, the stance selection logic, the grid values the selection produces, the full check's contents and order.

Flow after the change: selected stance (`st["q"]`) -> full check (`full_check.check(..., cand_st["q"])`) -> `task_cfg["pedestal_q"]` -> staging PEDESTAL command (`src/staging_source.py` 704, 986) -> episode summary and task meta.

Note: with navigation OFF the chosen stance is still taken from a ranked run (checked), so the parked height is always a checked height.

## 2. Commits

- `00eabac` Progress: pedestal override removal task started (2026-11-09)
- `ccea3ca` Remove the forced 0.16 m pedestal: the staging parks at the selected height (M284)
- `8166ee8` Experiments: drop the pedestal override option (M284)

## 3. Test suite

Suite: `experiments/run_tests.sh`, 40 tests. Parent = `00eabac` (before the code change), run from a worktree of that commit with `data/`, `.venv` and `deps/` linked to the main tree. Post = `8166ee8`. Summaries: `data/logs/suite_parent_20261109_summary.txt` and `data/logs/suite_post_20261109_summary.txt`.

The parent run died once at 20:49 during `stance_select_test` and was restarted with the finished tests skipped (same summary file).

Result per test (exit code and checks). Tests with the same result on both commits are not listed.

| test | parent | post | changed because |
|---|---|---|---|
| nav_pipeline_follow_test | exit 1, 2/14 | exit 1, 6/14 | expected: the parked height changed from 0.16 m (override) to the selected 0.12 m. Parent: pick failed at CLOSE (5.3 s). Post: pick passed; place failed at DOWN (3.4 mm short) and HOME (0.266 rad). Test does not reference 0.16; the result depends on the parked height. |
| face_test | exit 1, 24/27 | exit 0, 27/27 | not pedestal related (face textures and smile rendering). Not rerun yet; see NOT DONE. |

Totals: parent 36 pass, 14 fail; post 37 pass, 13 fail. The only changes are the two rows above (face_test now passes; nav_pipeline_follow_test still fails, with more checks passing). The other failing tests fail the same way on both commits: elbow_guard_test, gripper_force_test (27/29), target_mode_test (9/16), command_bridge_test (13/14), cc_mode_tests (33/39), full_check_test (30/31), stance_select_test (EGL crash), stance_select_pipeline_test (3/7), stance_select_moved_test (5/9), transit_clearance_test (26/27), mpvi_pipeline_test (6/14), mpvi_text_test (5/11). Several of these match the handoff's earlier failures. Their results did not change, so none was caused by this change.

Tests that assumed 0.16: none of the suite's tests reads `pedestal_override_m` (grep). Tests that read `TASK_CFG["pedestal_q"]` (0.155, the reference stance) are unchanged.

## 4. End-to-end run (200 g, staging layout)

Command: `python data/bounce_check_20261103/run_mass.py data/e2e_pedestal_20261110_001858 --liquid 0.100`, under `experiments/run_capped.sh` (6 GB cap). Log: `data/logs/e2e_pedestal_20261110_001858.log`.

Run: `scratch/e2e_pedestal/run_staging.py` (the `demo_video.py` pipeline with its layout name pointed at `TASK_CFG`, the staging layout; no `src/` change). Default beaker: glass 0.100 kg + liquid 0.100 kg = 200 g. Log `data/logs/e2e_staging_20261110_002222.log`; recording and episode files `data/e2e_staging_20261110_002222/episode/`. Outcome **PLAN_FAILED**, not SUCCESS.

| item | value |
|---|---|
| selected pedestal height (stance selection) | 0.12 m |
| height the full check used (passed) | 0.12 m |
| commanded pedestal height during the run | 0.1200 m (from DRIVE to the end) |
| measured settled height, start of PICK (2/4 PICK: starting) | 0.11945 m |
| measured settled height, start of PLACE (4/4 PLACE: starting) | 0.11945 m |
| lowest left elbow height during the run | not measured: the elbow-clearance monitor (`src/elbow_clearance_log.py`) did not write output for this run |
| minimum arm-to-bench distance | not measured (same reason) |
| bench contact | none recorded: the episode's contact log has no pair involving the bench in any phase (beaker walls against the finger collision geoms only) |
| phase reached | 4/4 PLACE: DOWN [failed]: "DOWN: not reached after 9.0 s: 3.4 mm and 0.3 deg from the waypoint", after one retry (RETRY 2: arm to the zombie pose), then the plan ended |
| joint 2 (left), final commanded / actual | -0.43780 / -0.43797 rad |
| joint 6 (left), final commanded / actual | -0.61087 / -0.60479 rad |

Notes: the run did not stall at HOME. It stopped at the place's DOWN move, 3.4 mm short of its waypoint, which matches the nav-pipeline result in the suite (post-change `nav_pipeline_follow_test` also fails at DOWN then HOME). The per-phase pedestal record (`episode_summary.json` key `pedestal`) shows the same 0.12 m at every phase from DRIVE on, and 0.418 m (raised) before PEDESTAL. The joint values above are the last log row (t = 114.5 s).

## 5. Held 200 g baseline comparison

Not done. The registered held 200 g baseline (`experiments/baselines/held_200g_20261104_m258/`) is reproduced by `data/bounce_check_20261103/run_mass.py`. That launcher now crashes at its first lift-off tick with `KeyError: 'C_p'` (log `data/logs/e2e_pedestal_20261110_001858.log`, line 195): it reads a `C_p` key from `gravity_ff`'s trace, which neither the current tree nor the parent commit `00eabac` produces (`src/gravity_ff.py` last changed in `6450105`). So the comparison cannot be made, and it is not caused by this change. Baseline files were not touched. The manual and teleop scenarios were not run.

The `demo_video` run above is not the held baseline (different script, staging layout, no liftoff trace), so it gives no pass or fail against the baseline.

## NOT DONE

- The `face_test` change (parent failed, post passed) has not been rerun. Not related to the pedestal code.
- `experiments/pedestal_episode.py` still forces a pedestal (see DECISIONS NEEDED).
- `docs/handoff_20261109.md` still describes the override (dated snapshot, not edited).
- Baseline files were not regenerated.
- The push of the M284 commits was not done (David's rule: push only when a feature is complete and the baselines are checked; this is held for review).
- Other sessions committed safety-filter and world-model work to main during this task (`5eb8b46`, `6f126f6`, and others). Not reviewed here.

## DECISIONS NEEDED

1. `experiments/pedestal_episode.py`: keep as a measurement tool that forces a height (outside the config, but still a forced run), or delete it?
2. Push M284 (`ccea3ca`, `8166ee8`) to origin/main, or hold until the e2e run is reviewed?
