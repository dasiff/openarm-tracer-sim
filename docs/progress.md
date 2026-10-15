# Progress log

## Latest: design plan section 12 (real-robot replacements for sim ground truth) added; section 11 got the localization-sensor question.

## Current: renders done (merge on hold, waiting for the user)
Rendered one at a time (log data/logs/render_20261014_235744.log), all in data/notebook/ (git-ignored): grasp_lift_5N_full_2026-10-14.mp4 (52 s, 37 MB, 1559 frames, 2246 s wall, verified bit-identical), control_2N_2026-10-14_clip.mp4 (4.7 s, 142 frames, 244 s wall), control_2N_2026-10-14_tilt.csv / _tilt.png (tilt logged every tick). Finding: in the 2 N control the beaker tilts 0.5 -> 48 deg and then topples to 90 deg at 3.5 s after the lift start (recorded in docs/TODO.md). Control recording: data/notebook/control_2N_2026-10-14_recording.
Next: the user decides on the merge (28+ commits, fast-forward from fe8c3d8). DO NOT merge or push until the user says so.

## Open
- Nothing pending. docs/design_plan.md written (blockers resolved by the user: camera rates, face marked DONE, neck added to section 1, wrist camera and camera model in section 11, camera-tuned tests in LATER). Every task's commit updates the plan's status markers.
- Camera parameters check answered in chat: all three policy cameras are in CAMERA_SPECS (+ ORBBEC_RESOLUTION / ORBBEC_FOVY_DEG) in src/robot_specs.py; the eagle camera's real pose also depends on NECK_SPECS and the vendored SO-101 XML; names are hard-coded in a few places.

## Finished (one line each)
- Recovery after the VM shutdown (2026-10-14): WIP commit inspected, baselines restored (777d1a2), face_test 23/23 and home_pose_test 28/28 passed, crash-recovery rules in CLAUDE.md.
- Teleop neck investigation: not the finger gain (kp 100 gives the same); the old solver had a redundant lift/elbow/wrist direction (1 rad spread on 0.1 degree noise).
- Neck frames once per skill, then holds (2026-10-14): solver defaults (posture 0.003 to the drive pose, move weight 0, wide simplex), staging re-sends stored targets, neck_auto once per stop, neck keys off while --record / --record-run. Tests: neck_once_test 21/21, neck_framing_test, home_pose_test 28/28, face_test 23/23. kp 100 vs 300 teleop: neck differs <= 3.5e-5 rad, same final pose.
- Baselines regenerated (temp file, checked, renamed) and committed as the new reference: ab903d0 (manual and policy equal the 12:16 files; teleop neck is new by design).
