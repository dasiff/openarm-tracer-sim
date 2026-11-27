# Merge prep: suite results, decision IDs, and readiness (2026-11-25)

Scratch branch only. Nothing was merged into main and nothing was pushed.

## Setup

- Merged branch `dryrun/safety-filter-plus-main`: safety-filter at `444d3d4` (the latest local commit; origin/safety-filter is still `be75e33`), plus `5cbace3` (M322 renumbered to M326, references updated), plus `5bd3581` (merge of main `b237951`).
- Main run: detached worktree `/home/vboxuser/projects/openarm-tracer-sim-main-suite` at `b237951`.
- Each worktree got copies of `data/` and `deps/`, and a `.venv` symlink to the main venv. `assets/` was copied into both only after the main run had finished `face_test`. Main's `face_test` therefore ran without `assets/`.
- Suite runs: `run_tests.sh main_20261125` then `run_tests.sh merged_20261125`, sequential, in one tmux session. Logs: `data/logs/suite_<tag>_*.log` and `data/logs/suite_<tag>_summary.txt` in each worktree.
- A single rerun of `face_test` on main was started and killed right away, because another session had a MuJoCo test running. It did not run.

## Results

Main: 66 tests, 50 pass, 16 fail. Merged: 71 tests, 64 pass, 7 fail (the seven fail-on-both tests below).

Fail on both (7):

| Test | Main | Merged | Earlier recorded runs on main |
|---|---|---|---|
| elbow_guard_test | FAILED, clamp at minimum | FAILED, same | Passed 2026-11-06; failing since 2026-11-09 |
| full_check_test | 30/31 | 30/31 | not checked |
| mpvi_text_test | 5/11 (LLM cache miss, expected) | 6/11 (same cause) | 5 to 7 of 11 |
| stance_select_moved_test | IndexError | IndexError | failing since 2026-11-06 |
| stance_select_test | EGL error | EGL error | EGL error since 2026-11-09 |
| target_mode_test | 9/16 | 9/16 | 9/16 in every run |
| transit_clearance_test | 26/27 | 26/27 | 26/27 in recent runs |

Fail on main, pass on merged (9). The test script is identical on both branches for the first six. The source code under test differs.

- cc_mode_tests: 33/39 on main, 39/39 on merged
- command_bridge_test: 13/14 on main, 21/21 on merged. The script differs between branches.
- face_test: 24/27 on main, 27/27 on merged. Probably the missing `assets/` on main, not confirmed by a rerun. Earlier runs on main showed 27/27.
- gripper_force_test: 27/29 on main, 34/34 on merged
- mpvi_pipeline_test: 10/14 on main, 14/14 on merged
- nav_pipeline_follow_test: 5/14 on main, 14/14 on merged. Earlier main runs ranged from 2 to 6 of 14, so this test is unstable.
- safety_filter_sequential_push_test: FileNotFoundError on main (`data/safety_filter/kong_single_pair_fixtures.json` is only on the safety-filter side). Passes on merged. The script differs.
- safety_filter_spec_test: FAIL on main ("the closing right arm moves"), passes on merged. The script differs.
- stance_select_pipeline_test: exception on main, 7/7 on merged. Earlier main runs failed at 3 to 4 of 7, and the sampler seed is random per run, so this one is flaky.

Pass on main, fail on merged: none.

Merged only (5), all pass: safety_filter_braking_test, _b_test, _c_test, _d_test, _e_test.

The nine tests in the "fail on main, pass on merged" group are not evidence that the merge is correct. They show that the merged code changes behavior, so the change needs David's review before it lands.

## Decision IDs (David's instructions 1 and 2)

1. The release-freeze row is now M326 on the scratch branch. **Main has since taken M326** (see below), so this is no longer free.
2. The only ID used on both branches with different meanings at the time of the merge was M322. Eight rows (M264, SF-31 to SF-34, SF-40, SF-41, SF-49) differ only by "SUPERSEDED" notes added on the safety-filter side. No code cites the renumbered row.
3. **New since the suite started:** main is at `5e42f93` (4 commits from today). It adds M327 to M341 (navigation and localization design, lidar WIP). Clashes with the merged branch:
   - M326: main's is "Robot localization and object finding" (navigation). The merged branch's is the release-freeze row. **Proposed fix: renumber the release-freeze row to M342**, the next free number on both sides.
   - M066, M143, M163: main has added "UPDATED by ..." notes to these rows. That is a text conflict on the same rows, not a change of meaning. It needs a manual resolution when re-merging.

## back_lift_m (SF-87, marked "confirm"). Not changed.

- What it does: after the gripper opens at the place, the pad goes straight up by 0.10 m (LIFT). Then it retreats level, along the approach direction, at that height (BACK). Only then does the arm go HOME. On main, BACK starts at the release height, and the notes say the wrist could dip into the hotplate and bench that way.
- Code path: `src/mpvi_modules.py`, `place_back_waypoints()` (line 158), called from `plan_place()` inside `make_scripted_manipulation()` (line 282). The place is used by `src/mpvi_pipeline.py` for every place. The config is `PICK_PLACE_CFG["back_lift_m"]` in `src/task_config.py` line 423.
- With the safety filter off: yes, it acts. It is planner code, not the filter. The filter's mode setting does not affect it.
- If the value is wrong: too low, and the wrist may still dip during BACK (the failure SF-87 was written to fix). Too high, and the place takes longer. LIFT runs at the slow speed (0.04 m/s), so 0.10 m adds at least about 2.5 s per place, from the speed setting, not measured. A high BACK may also fail the joint solve (not checked). `experiments/place_back_lift_test.py` checks only that LIFT equals `back_lift_m`, so it cannot catch a wrong value.

## Ready to become main?

**No.** Reasons:

1. **Main moved.** The merge and the suite were both against `b237951`. A test merge with current main (`git merge-tree`, no worktree touched) conflicts in `docs/mpvi_decisions.md` and `docs/progress.md`. Re-merge and re-run are needed.
2. **M326 clash** on current main (above). Renumber to M342 needs David's OK.
3. **Behavior changes need review.** Nine tests change from fail to pass, and the source changes behind them are in the safety-filter code. The command bridge (uniform shrink), the sag layer (integrator removed), the joint buffer (`BRIDGE_BUFFER_CFG` "range"), and the place lift (`back_lift_m`) all change output on default runs, not only with the filter on. These are decided in SF-125, SF-127, stage 3 and SF-87, but I have not verified the decisions against the code.
4. **back_lift_m** is marked "confirm" in SF-87. Still open.
5. **Flaky tests.** `stance_select_pipeline_test`, `nav_pipeline_follow_test`, `mpvi_pipeline_test` and `transit_clearance_test` have varied across runs on main. A single pass on merged is weak evidence.
6. **The safety-filter branch can still move.** The sf-d worktree has 5 uncommitted entries, and new commits may come.

## Baselines that would need regenerating after the merge

- `experiments/baselines/` is identical on both branches, so the files themselves don't change in the merge.
- The CURRENT references are `manual` (`manual_reference_20261103.csv`, commit `e127479`) and `held_pick_place_200g` (`held_200g_20261104_m258`). Both come from code that the merge changes (command bridge, sag layer, place lift), so both would stop matching. Either regenerate them after David approves the changes, or show that the output is unchanged.
- `teleop` and `policy` are NO_CURRENT, so nothing needs regenerating for them.
- `data/cc_baseline/` (untracked) was not checked. `cc_loop_tests` passed on both branches.

## Proposed next steps (not done)

1. David approves or rejects each behavior change, and confirms `back_lift_m` = 0.10 m or gives a new value.
2. David approves renumbering the release-freeze row to M342.
3. Re-merge current main into the latest safety-filter. Resolve the three M066/M143/M163 text conflicts by keeping main's updates. Resolve `progress.md` by keeping both blocks.
4. Re-run the full suite on the new merge. `run_tests.sh` on main now lists `lidar_test`, which is WIP and will fail until it is finished.
5. Regenerate the two CURRENT baselines if the changes are approved, then decide whether to merge.
