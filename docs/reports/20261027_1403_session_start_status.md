# Session-start status, 2026-10-27 14:03

## In progress
- Full test suite, tmux `suite9`, TAG 20261027_133625 (bridge OFF, SKIP=face_test). Started 13:36:25, still running at 14:02: now on `cc_loop_tests` (last test). Summary: `data/logs/suite_20261027_133625_summary.txt`.
- Every test finished so far exited 0 (home_pose, gripper_force, grasp_lift, beaker_hollow, target_mode, sequential_policy, beaker_hotplate_correction, beaker_hotplate_success, command_bridge 14/14, pedestal_lift, rest_contacts, robot_vs_urdf, reach_map_holes, reach_map_restamp). I did not read the earlier part of the summary.
- Not yet run: `face_test` (planned to run alone after the suite finishes).

## Done earlier today (per progress.md)
- Command bridge built (M193, off by default; demo_video default ON). Bridge-ON episode: success, 92.1 s sim vs 71.5 s, 0 rejections. Measurement in `data/arm_motion/bridge_20261027_133144/compare.md`.
- Demo-review report: `docs/reports/20261027_1355_demo_review_panel_arm_measurement_bridge.md`.

## State
- Git: clean except untracked `deps/AgileX_Tracer`. Last commit cb3ab0b. Nothing pushed for this task (David said do not push).
- Rule: do not edit `src/` or `experiments/` until the suite is ALL_DONE.

## Next
1. Wait for the suite to finish, then run `face_test` alone.
2. Update the report with the suite result and commit.
3. David decides on smoothing, then one video re-render (not done).
