# Progress log

## Latest: design plan section 12 (real-robot replacements for sim ground truth) added; section 11 got the localization-sensor question.

## Current: nothing running. Grip force table is now 8 N, merge on hold
GRIP_FORCE_TABLE glass_beaker 5 -> 8 N (approved by David 2026-10-15; sweep docs/grip_force_sweep.md). Rerun (logs data/logs/grip8_tests_20261015_210429): grasp_lift_test 4/4 (slip 0.15 mm, no creep; the 2 N control still fails to hold), gripper_force_test 29/29, beaker_hollow_test 21/21. No baseline uses the table. Design plan 3b is DONE; open for the pour (not in the sweep) and for friction below 0.32.
Earlier (one line): battery pack DONE (BATTERY_SPECS, src/battery.py, baselines regenerated, tests pass).
Branch state (2026-10-15): on main, 27 commits ahead of origin/main (fe8c3d8): the 7 topic commits of squash/home-pose-hollow-beaker (same tree as the old 38-commit home-pose-hollow-beaker) plus 20 battery / grip force / docs commits (about 14 of them progress-log only). Proposed: rebuild main as clean topic commits (battery, baselines, grip force, final progress log), verify the tree is identical, then push main as a fast-forward. WAITING FOR DAVID'S APPROVAL of the plan; DO NOT rewrite main or push until then.

## Open
- Nothing pending. docs/design_plan.md written (blockers resolved by the user: camera rates, face marked DONE, neck added to section 1, wrist camera and camera model in section 11, camera-tuned tests in LATER). Every task's commit updates the plan's status markers.
- Camera parameters check answered in chat: all three policy cameras are in CAMERA_SPECS (+ ORBBEC_RESOLUTION / ORBBEC_FOVY_DEG) in src/robot_specs.py; the eagle camera's real pose also depends on NECK_SPECS and the vendored SO-101 XML; names are hard-coded in a few places.

## Finished (one line each)
- Recovery after the VM shutdown (2026-10-14): WIP commit inspected, baselines restored (777d1a2), face_test 23/23 and home_pose_test 28/28 passed, crash-recovery rules in CLAUDE.md.
- Teleop neck investigation: not the finger gain (kp 100 gives the same); the old solver had a redundant lift/elbow/wrist direction (1 rad spread on 0.1 degree noise).
- Neck frames once per skill, then holds (2026-10-14): solver defaults (posture 0.003 to the drive pose, move weight 0, wide simplex), staging re-sends stored targets, neck_auto once per stop, neck keys off while --record / --record-run. Tests: neck_once_test 21/21, neck_framing_test, home_pose_test 28/28, face_test 23/23. kp 100 vs 300 teleop: neck differs <= 3.5e-5 rad, same final pose.
- Baselines regenerated (temp file, checked, renamed) and committed as the new reference: ab903d0 (manual and policy equal the 12:16 files; teleop neck is new by design).
