# Safety filter branch: merge of main (2026-11-13)

Written for: David (decision on the final fast-forward). Model: Sonnet 5.5. Nothing pushed; main not touched.

## State before the merge (step 0)
- safety-filter d794c42; main = origin/main = 92187b0; origin/safety-filter cccdd30 (2 commits behind local).
- Merge base cd1c065. main side 57 commits, branch side 54 commits.
- Local main equals origin/main (0 ahead, 0 behind): no divergence, a normal `git push origin main` will work.
- Changed on both sides: docs/design_plan.md, docs/mpvi_decisions.md, docs/progress.md, experiments/run_tests.sh.
- docs/reports/stance_search_geometry_20261111.md: untracked here, not tracked on main, not in the branch history (another session's; not touched, did not block the merge).
- Identity check: `bash experiments/run_capped.sh <log> env CC_HEADLESS=1 bash experiments/cc_bench_scenarios.sh manual <csv>`, then `python experiments/cc_bench_compare.py manual <csv>` and cmp/sha256 against experiments/baselines/manual_reference_20261103.csv (sha 3060ae41...c9b82, in INDEX.csv). Only manual has a CURRENT baseline (teleop, policy are NO_CURRENT); the held 200 g baseline is not a bench scenario and was not run.
- Known pre-existing failures (docs/progress.md, 2026-11-05): gripper_force, target_mode, command_bridge, cc_mode, nav_pipeline_follow, stance_select, stance_select_pipeline, stance_select_moved, mpvi_pipeline, mpvi_text; plus the filter's spec check "the closing right arm moves".

## Merge (23e3190)
Conflicts (2 files, none under src/safety_filter/):
- docs/mpvi_decisions.md: one hunk, both sides kept in full, main's rows first, then the branch's.
- docs/progress.md: two hunks, same rule.
- experiments/run_tests.sh and .gitignore auto-merged (union). design_plan.md auto-merged.
No hunks were kept from the branch side in place of main's lines. Rows M286, M287, M289, M297 differ from main's text because the branch edited them and main did not (main = merge base for these rows); the merged file has the branch's edit.
No conflict markers remain.

## Verification
- src/safety_filter/ identical to the branch's version before the merge (empty diff).
- Not wired: no import of safety_filter outside src/safety_filter/, experiments/, scripts/, docs/. Hits are comments or tools (models/policies/branch_jump_policy.py imports experiments.safety_filter_roundtrip, which was already on the branch; tools/*.py; src/geo_kin_self_collision_filter.py mentions it in comments only). Control loop, bridge, skills, runner: none.
- Baseline (manual scenario, merged tree): byte-identical to the stored reference (cmp exit 0, sha equal, cc_bench_compare exit 0). Log data/logs/merge_check_20261113_150930_*.
- Suite (run_tests.sh with a swap check and 45-minute cap per test; summary data/logs/suite_merged_1509_summary.txt): 64 tests run, none skipped for memory.
  - Failing and on the known list: gripper_force 27/29, target_mode 9/16, command_bridge 13/14, cc_mode 33/39, nav_pipeline_follow 10/14, stance_select (EGL error), stance_select_pipeline, stance_select_moved, mpvi_pipeline, mpvi_text 5/11, and safety_filter_spec_test (closing right arm).
  - Failing and NOT on the known list: elbow_guard, face_test 24/27, full_check 30/31, transit_clearance 26/27. All four were re-run on a detached worktree of main (92187b0) and fail identically there (logs data/logs/main_recheck_*), so none is new from the merge. They are missing from the 2026-11-05 list only.
  - All other filter tests pass, including safety_filter_structure_test (no simulator world access) and the config explanation test if part of the suite (no separate config explanation test run was found by name; not listed as run).
- Setup note: this worktree's data/ got symlinks to main's data/ entries (git-ignored) so the tests could read maps; test outputs went to main's ignored data/ folder.

## Not run
- Held 200 g baseline; teleop and policy bench (no baseline).
- A dedicated "config explanation test" was not identified by name.

## Decisions for David
- The four failing tests not on the old list (elbow_guard, face, full_check, transit_clearance) fail on main too; update the known list or look into them.
- origin/safety-filter is 2 commits behind local; pushing is yours.

## Finishing commands (do not run here)
In /home/vboxuser/projects/openarm-tracer-sim (main must be clean):
    git status --short
    git merge --ff-only safety-filter
If main moved, run `git merge main` here again and re-verify. Then, by David: `git push origin main` and `git push -u origin safety-filter`.
