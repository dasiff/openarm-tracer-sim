# Progress log

## Latest: design plan §12 (real-robot replacements for sim ground truth) added; §11 got the localization-sensor question.

## Current: ALL BUILDING PAUSED (David, 2026-10-18): no B1c import, no B5 work, no B2 map builds. Nothing running.
Done this session: history tidied into topic commits and pushed (B2.9 + B2.10, B3, docs; backup tag backup/main-before-tidy-2026-10-18 stays local); docs: §6 maximum pour tilt rate 15 deg/s (n_range stays 30 to 60), §7.1 converter (action = commanded targets, never measured follower positions; state = measured joint states; all streams aligned to the camera frames by timestamp), §11 questions for Luke (record_mcap.sh topic names vs /leader and /sim namespaces; no camera topics); report on how ball friction 0.3 beats glass 1.0 (geom priority: the higher priority geom's friction and contact softness are used instead of the maximum; confirmed on a minimal model); week summary for the lab notebook written in chat.
B1c comparison (report only, 2026-10-18): see design plan §9 B1c and §11; the clones and scripts are in the session scratchpad (mbr/). Waiting for David's go-ahead for B1c (model import), then B2 map builds, then B5.
Earlier (one line): parts 1 to 3 of 2026-10-16 (geometry proof B2 code, B2.9 + B2.10, B3) are on origin/main now.

## Done (docs only): design plan updated for Dan's direction
§8 is now MPVI as in the paper (LLM decides the order; scripted staging stands in for navigation; the A* task-level search moved to §10 LATER with its text); prompts no longer name the arm (§§3.4, 5, 6, 7, 8); new §13 (exploration and object localization; numbered 13 to avoid renumbering 9-12); VLA arm-only (7.3) with the base-adjustment option in LATER; build order (§9) revised. Follow-up answers from David applied: the demo generator picks the arm by a deterministic rule (better reach score at the stance, ties left; for the pour, each pick among the free arms) instead of a random arm (§§3.4, 5, 6, 7.2); two differences from the paper in §8 (stance, no LLM fine-tuning: the paper fine-tunes on annotated BEHAVIOR plans, Appendix C.1); §9 says staging must accept a stance input by B4; docs/TODO.md and the README planner line match.


## Open
- Nothing pending. docs/design_plan.md written (blockers resolved by the user: camera rates, face marked DONE, neck added to §1, wrist camera and camera model in §11, camera-tuned tests in LATER). Every task's commit updates the plan's status markers.
- Camera parameters check answered in chat: all three policy cameras are in CAMERA_SPECS (+ ORBBEC_RESOLUTION / ORBBEC_FOVY_DEG) in src/robot_specs.py; the eagle camera's real pose also depends on NECK_SPECS and the vendored SO-101 XML; names are hard-coded in a few places.

## Finished (one line each)
- Battery pack (Anker SOLIX C800, 11.3 kg box on the flat deck): BATTERY_SPECS, src/battery.py; arms never closer than 138 mm, not in any camera, lean unchanged; new baselines (2026-10-15).
- Grip force: the 5 N grasp was marginal; a sweep of 30 runs (docs/grip_force_sweep.md); the glass beaker is 8 N per finger (2026-10-15).
- Planner direction from Dan (2026-10-15): MPVI as in the paper first, prompts do not name the arm, deterministic arm rule (better reach score, ties left), exploration section (§13), VLA arm only, build order as B-numbers (design plan §8, §9, §13).
- B2.1 to B2.4 (2026-10-16): src/arm_kinematics.py, grasp_rules.py, reach_map.py (IK flood fill), arm_collision.py, gripper_geometry.py (pad centre and axes derived from the model); pilot maps for left and right arm bin 0 (mirror IoU 0.94); pushed.
- B2.9 furniture map (22/22) and B2.10 stance finder core (28/28), synthetic only; the room walls collide in the task scene (2026-10-18, pushed).
- B3 pouring scene with balls (21/21) and timing test: every ball flows at 15 deg/s; 30 balls 0.9x real time, 40 balls 1.5x, 60 balls 3.2x (2026-10-18, pushed).
- B1c comparison with the updated robot description (report only, 2026-10-18): design plan §9 and §11; no model change yet.
- Recovery after the VM shutdown (2026-10-14): WIP commit inspected, baselines restored (777d1a2), face_test 23/23 and home_pose_test 28/28 passed, crash-recovery rules in CLAUDE.md.
- Teleop neck investigation: not the finger gain (kp 100 gives the same); the old solver had a redundant lift/elbow/wrist direction (1 rad spread on 0.1 degree noise).
- Neck frames once per skill, then holds (2026-10-14): solver defaults (posture 0.003 to the drive pose, move weight 0, wide simplex), staging re-sends stored targets, neck_auto once per stop, neck keys off while --record / --record-run. Tests: neck_once_test 21/21, neck_framing_test, home_pose_test 28/28, face_test 23/23. kp 100 vs 300 teleop: neck differs <= 3.5e-5 rad, same final pose.
- Baselines regenerated (temp file, checked, renamed) and committed as the new reference: ab903d0 (manual and policy equal the 12:16 files; teleop neck is new by design).
