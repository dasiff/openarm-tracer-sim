# Delete the nine layered-resolver diagnostic scripts

Date: 20261119 (stamp 20261119_0039). Branch safety-filter, worktree ~/projects/openarm-tracer-sim-sf-d. Nothing pushed.

Written for: David (section 5).

## 1. Summary

- Deleted the nine scripts in scripts/ that diagnose the removed layered resolver (or are shared by them): 3968 lines in total, one commit (ef68e88).
- No test file and no test runner imports or runs any of them. Checked by grep on every name, and by the import graph: nothing in experiments/ or src/ loads them.
- Nine other scripts import from the deleted modules and now fail at import. They were not in the list, so they were not deleted (section 4).
- Full test suite (experiments/run_tests.sh, 71 tests): 52 exit 0, 19 exit non-zero. None of the 19 loads a deleted file. Thirteen of them also fail in the main repo's last full run; the other six have environment causes (section 3).

## 2. Files deleted

| File | Lines |
|---|---|
| scripts/safety_filter_kong_comparison.py | 620 |
| scripts/safety_filter_diag_common.py | 114 |
| scripts/safety_filter_time_profile.py | 500 |
| scripts/safety_filter_hold_causes.py | 500 |
| scripts/safety_filter_push_size_replay.py | 621 |
| scripts/safety_filter_push_size_tables.py | 639 |
| scripts/safety_filter_solver_jump_replay.py | 359 |
| scripts/safety_filter_speed_common.py | 389 |
| scripts/safety_filter_speed_profile.py | 226 |
| **Total** | **3968** |

docs/progress.md gained the task entry and the result (5 lines).

## 3. Full test suite

Run: `bash experiments/run_tests.sh scriptsdel` in tmux full_suite_scriptsdel, 22:57 to 00:37. Summary: data/logs/suite_scriptsdel_summary.txt; per-test logs data/logs/suite_scriptsdel_<name>.log.

Result: 71 tests, 52 exit 0, 19 non-zero, 0 skipped.

The worktree's data/ is sparse: it lacks most of the data folders that the main repo has (for example data/demo_record and data/cc_baseline). Some failures come from that.

| Test | Exit | Cause (first error in its log) | Main repo, last full run (2026-11-09) |
|---|---|---|---|
| search_timeline_test | 1 | missing data/demo_record/.../candidates_list.json | not run there |
| search_display_isolation_test | 1 | same missing file | not run there |
| elbow_guard_test | 1 | assertion: clamped target at or above the minimum | exit 1 |
| face_test | 1 | pixel check; libEGL cannot open /dev/dri/renderD128 | exit 0 |
| gripper_force_test | 1 | replay not bit-identical (tick 10873) | exit 1 |
| target_mode_test | 1 | ramp assertion | exit 1 |
| command_bridge_test | 1 | applied target jumped by 0.6 rad | exit 1 |
| cc_loop_tests | 1 | EGL error (renderer) | exit 0 |
| cc_mode_tests | 1 | ARMS timed out after 30 s (STAGING_FAILED) | exit 1 |
| full_check_test | 1 | 0 stances in the windowed search | exit 1 |
| nav_pipeline_follow_test | 1 | world-model beaker entry not on the hotplate | exit 1 |
| stance_select_test | 1 | IndexError | exit 1 |
| place_theta_logic_test | 1 | place step gets no computed angle; search ran zero times | exit 0 |
| stance_select_pipeline_test | 1 | replay not bit-identical (tick 12345) | exit 1 |
| stance_select_moved_test | 1 | IndexError | exit 1 |
| transit_clearance_test | 1 | IndexError | exit 1 |
| bench_stance_test | 1 | EGL error (renderer) and zero region count | exit 0 |
| mpvi_pipeline_test | 1 | replay not bit-identical (tick 11818) | exit 1 |
| mpvi_text_test | 1 | replay not bit-identical (tick 11941) | exit 1 |

Reading the table:

- 13 of the 19 also exit 1 in the main repo's last full run. That run used different code, so this is evidence that they predate this change, not proof.
- search_timeline_test and search_display_isolation_test fail on a missing data file in this worktree. This is environmental.
- face_test, cc_loop_tests and bench_stance_test hit a renderer permission error here (libEGL cannot open /dev/dri/renderD128). They pass on the main repo. This is environmental, but I did not confirm it beyond the log.
- place_theta_logic_test passes on the main repo and fails here. I did not find its cause.
- A baseline run of the 19 at the commit before the deletion (a87f54b) was not done. The import check already shows that the deletion cannot change them.

Safety-filter tests: all 18 in run_tests.sh exit 0.

## 4. Scripts left in place that import the deleted modules

These are not tests, and they were not in the deletion list. Each now fails at import:

| Script | Imports |
|---|---|
| scripts/safety_filter_pair_counts.py | safety_filter_diag_common |
| scripts/safety_filter_reactive_stream.py | safety_filter_diag_common |
| scripts/safety_filter_mild_stream.py | safety_filter_diag_common |
| scripts/safety_filter_solver_model_check.py | safety_filter_kong_comparison |
| scripts/safety_filter_solver_roundtrip.py | safety_filter_kong_comparison |
| scripts/safety_filter_speed_gate.py | safety_filter_speed_common (the equivalence gate) |
| scripts/safety_filter_speed_golden.py | safety_filter_speed_common |
| scripts/safety_filter_dc_replay.py | safety_filter_speed_common |
| scripts/safety_filter_speed_metrics.py | safety_filter_speed_common (inside a function) |

Comment-only reference: src/safety_filter/link_rules.py line 27 names scripts/safety_filter_kong_comparison.py.

## 5. Decisions for David

1. Delete the nine scripts in section 4 too, or port them? The speed gate (safety_filter_speed_gate.py) is the equivalence gate of the speed work, so deleting speed_common removes its only way to build the golden streams.
2. The comment in src/safety_filter/link_rules.py line 27 points at a deleted script. Change the comment or leave it?
