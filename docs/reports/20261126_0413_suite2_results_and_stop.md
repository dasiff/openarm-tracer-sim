# Merge run 2: suite results, and stop before main (2026-11-26)

Stopped before merging into main, as the instructions require. Main was not merged or pushed. The baselines were not regenerated.

## What was done

1. **safety-filter, pushed.** The M322 release-freeze row is now M342, with its doc references (design_plan SF-138, reports 1631, 1458 and 1724). Commit `3b345fa`, pushed to origin as a fast-forward from `be75e33`. The sf-d worktree's own uncommitted `docs/progress.md` was left alone.
2. **back_lift_m marked PROVISIONAL.** Commit `181fa74` on safety-filter: a comment in `src/task_config.py` and a note on the SF-87 row. No value change. **Not pushed yet.**
3. **Merge of main `7518703` into safety-filter**, on scratch branch `dryrun/merge-main-7518703` (commit `fbef3d8`). Resolutions:
   - `docs/progress.md`: both blocks kept.
   - `docs/mpvi_decisions.md`: main's M322 to M341 kept, then the release-freeze row M342. No duplicate IDs.
   - M066, M143, M163: main's "UPDATED by" notes kept (safety-filter did not change these rows).
   - `experiments/run_tests.sh`: main's line 18 had a missing space (`search_timeline_testsearch_display_isolation_test`). That was a bug on main, and the merge fixes it. **Main itself still has the bug.**
4. **Suites run:** main `7518703` (worktree `openarm-tracer-sim-main-suite2`, with the same test-list fix), then merged `fbef3d8`.

## Results

Main: 67 tests, 52 pass, 15 fail. Merged: 72 tests, 65 pass (60 of the 67 shared tests, plus the 5 new braking tests), 7 fail.

- **Fail on both (7):** elbow_guard_test, target_mode_test, full_check_test, stance_select_test (EGL), stance_select_moved_test (IndexError), transit_clearance_test, mpvi_text_test (cache miss, expected).
- **Pass on main, fail on merged: none.**
- **Fail on main, pass on merged (8).** The probable cause for each comes from the changed code, not from a check I ran:
  - cc_mode_tests, 33/39 to 39/39: probably the bridge shrink and sag changes.
  - command_bridge_test, 13/14 to 21/21: the script differs on the two branches, and the bridge changed.
  - gripper_force_test, 27/29 to 34/34: probably the sag layer change. The old failure was a replay mismatch.
  - nav_pipeline_follow_test, 10/14 to 14/14: flaky on main (2 to 10 across runs). Not established.
  - stance_select_pipeline_test, 3/7 to 7/7: flaky, the sampler seed is random per run. Not established.
  - mpvi_pipeline_test, 10/14 to 14/14: probably the place lift and the other changes. Not established.
  - safety_filter_spec_test: the script differs on the two branches.
  - safety_filter_sequential_push_test: its fixture file exists only on the safety-filter side. An environment difference, not a code one.
- **Pass on both:** 52, including lidar_test and face_test (with assets present).

## Why I stopped

Main moved while the suites ran. Main is now `9e005f5`, with 6 new commits by David Siff (navigation stage 2 work, home pose script deletions, run record seed). Those commits touch five files that safety-filter also changed:

- `docs/design_plan.md`, `docs/mpvi_decisions.md`, `docs/progress.md` (text)
- `src/control_loop.py` and `src/run_record.py` (code)

A dry-run merge (`git merge-tree`, no worktree touched) shows textual conflicts only in `mpvi_decisions.md` and `progress.md`. `control_loop.py` and `run_record.py` merge automatically, but main's new code in them was not run by any suite. The instruction says to stop when that happens.

## Not done

- The merge into main, and the push of main.
- Regenerating the `manual` and `held_200g` baselines. This depends on main, so it waits.
- Pushing the PROVISIONAL commit `181fa74` on safety-filter.

## Decision needed

Options:

1. Re-merge `9e005f5` into safety-filter and re-run the full suite on both (about 4 hours). This is what the instruction calls for once main moves. Pick this if the two code files need a test pass.
2. Review the 9e005f5 changes to `control_loop.py` and `run_record.py` by hand, and accept the merge without a new run.
3. Hold the merge until the navigation work is done.

Also: `origin/main` is still `b237951`. Local main has 21 commits that are not pushed, including the WIP lidar commits. A push of main would publish them.
