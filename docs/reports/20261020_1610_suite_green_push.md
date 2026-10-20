# Report 2026-10-20 16:10: full suite green, main pushed

Written for: David.

Full suite (`bash experiments/run_tests.sh 20261020_150132`, 15:01 to 16:05, summary `data/logs/suite_20261020_150132_summary.txt`): all 28 tests exit 0, including the new `mpvi_pipeline_test` (9/9: SUCCESS, the arms at least 40 mm apart with the limit restored, replay bit-identical), `cc_mode_tests` 48/48, `gripper_force_test` 34/34, `home_pose_test` 34/34, `grasp_lift_test` 4/4. No baseline was touched (nothing in the existing code paths changed).

Before the push the local WIP commits were squashed into two topic commits (code; docs) so no "WIP:" commit reaches the remote; a local backup branch `backup-before-tidy-3` keeps the old history, and the tree of the tidied branch equals the old one.

Video and stills (not in git): data/notebook/mpvi_pipeline_success/ (mpvi_pipeline_success.mp4, _still_place_down.png, _panel.png). Open items waiting for you: the post-processing module proposal (docs/reports/20261020_1447_postprocess_tags_origin.md).
