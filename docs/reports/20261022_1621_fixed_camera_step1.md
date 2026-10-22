# Fixed eagle camera, step 1: records only (no code changes)

Written for: David, to review the scope of step 2 before it starts.

## 0. Leftover processes and memory
- Before: 7,049 MB used, 138 MB free, 375 MB available. The cause was one process: `full_check_test` (4.3 GB) of the suite part 3 that I had started at 15:47 just before your message. It was the only python / renderer process left; no other leftovers from the killed runs (checked every process, the tmux server and /dev/shm).
- I ended it (tmux session `suite_p3` and its python). After: 2,369 MB used, 4,794 MB free, 5,055 MB available. Nothing of mine is running now. The suite was not rerun.
- Did leftovers explain the rise from 2.2 to 3.4 GB idle? No. At 15:30, after the OOM kill, no python of mine was alive; the extra ~1 GB was the editor and agent processes of the previous session (the VS Code server's extension hosts and a `claude` process of 300 to 600 MB each: 617, 407, 369, 301 MB at that time). They exited with that session; the same programs now use about 0.7 GB in total and idle use is back to 2.4 GB. So the full_check_test OOM at 15:29 was 4.5 GB of test plus 3.4 GB of baseline, not a leak of ours.
- The overnight summary is skipped as you said; docs/reports/20261022_1547_progress_status.md stands in for it.

## 1. New decisions-log rows (docs/mpvi_decisions.md)
| id | content |
|---|---|
| M153 | F1: the eagle camera is fixed to the torso, no neck, no camera joints; the hardware team decides position and angle, a placeholder until then (chosen in step 2). Step 2 removes the neck, all obsolete code, rewrites every camera check as "inside the fixed view" (10 % margin, occlusion), regenerates the baselines once. Stage 6 obsolete. Supersedes M049, M091, M092, M135, M151; M087's neck spread dropped (those rows are marked superseded). |
| M154 | F2: camera "Galexia Avant Pro" with depth (as you gave it); datasheet NOT FOUND, nothing recorded; depth for navigation and perception only, the VLA uses colour only; `CAMERA_SPECS` unchanged. |
| M155 | F3: "close enough" (a null navigation step) = every target of the next step within the arm's reach AND inside the fixed view (10 % margin, not occluded) from the current stance; no distance threshold; replaces the B2 criterion (M046). |
| M156 | F4: later phase: retrain the VLA end to end, navigation included (whole-body, as in the paper), on the working pipeline's runs; it learns to keep the base still; ours is arm-only now; the hard base / arm interlock stays as a LATER backstop. |
| M157 | F5: order of work: 1 record, 2 remove the neck (full suite once at its end), 3 Stage 5 redone with the fixed view, 4 the rest of the overnight report. |

Design plan updated: a banner at the top; B2 (F3), B5 obsolete, B6 reworded; two fidelity-table rows (eagle camera, VLA scope); §9 (the "before B4" item 2 replaced, B9 whole-body VLA phase, the order of work); §10 (the interlock backstop); §11 (the camera question); §14 rewritten (14.1 camera and the "inside the fixed view" rule, 14.2 must match the real robot; 14.3 the wrist camera unchanged). Config comments: the neck and framing settings are marked "[OBSOLETE, removed in step 2: M153]" (comments only; a check confirms the code part of every changed line is identical). Committed, not pushed.

## 2. The camera datasheet (F2): not found
Searched 2026-10-22 for "Galexia Avant Pro", "Galaxea Avant Pro" and "Avant Pro" with Orbbec / Galaxea / depth / RGB-D (three searches, one extended): no product of that name; the results were other cameras (Orbbec Gemini 305, 335L, 336L, 435Le, Femto Bolt; Leopard Imaging; a Samsung phone). I recorded no field of view, resolution, frame rate or depth range and did not substitute any of those. Please give the exact maker and model name, or a datasheet link or PDF.

## 3. Scope of step 2 (for review)

### Model and assets
- `models/so101/so101_new_calib_camera.xml` (the neck model): no longer attached. The scene files (`models/scenes/chemistry_lab*.xml`, 4 files) only have unrelated "ext_neck" / "flask_neck" geoms: no change.
- The eagle camera's new fixed mount on `openarm_body_link0` (placeholder), added where `CAMERA_SPECS` mounts cameras; the head / face (`src/face.py`, `FACE_CFG`) is mounted on the neck's end today: it needs a new place or goes (question 2).

### Code to delete
`src/neck.py`, `src/neck_source.py`, `src/neck_framing.py` (the framing solver, `FRAMING_CFG`, `best_margin`, `reframe_or_keep`, `margins_at`).

### Code to edit (count of neck / framing / eagle mentions)
`src/robot_specs.py` (38: `NECK_SPECS`, `NECK_AUTO_SPECS`, `FACE_CFG` mount, the eagle entry of `CAMERA_SPECS`, `ORBBEC_*`), `src/simulator.py` (14: `add_neck`, `neck=True`), `src/control_loop.py` (50: neck command mode, the drive pose, follow), `src/commands.py` (21), `src/keyboard_source.py` (47: neck keys, F7 / F8), `src/episode_runner.py` (16), `src/staging_source.py` (67: the NECK phase, `frame_points`, the neck settle and timeout), `src/task_beaker_hotplate.py` (21: `framing_points`), `src/stance_checks.py` (45: neck spread, framing slack, `OcclusionMeter` camera pose, `place`), `src/full_check.py` (16: neck draws, neck solves, framing slack), `src/stance_finder.py` (1), `src/arm_collision.py` (8: the neck as an obstacle at its drive pose, `neck_q`), `src/face.py` (12), `src/glfw_viewer.py` (3), `src/viewer_composite.py` (8), `src/viewer_layout.py` (10), `src/task_policy.py` (7), `src/run_record.py` (2), `src/replay.py` (1), `src/mpvi_vlm.py` (1: camera name stays "eagle_cam"), `src/nav_footprint.py` (2: the footprint now has no neck).

### Settings to remove or change (`src/task_config.py` unless noted)
`FRAMING_CFG` (all: rule, margin, weights, coarse steps, simplex, contact margin, `keep_margin`, `least_move_weight`; the margin 0.10 moves to the new view check), `FULLCHECK_CFG["neck_draws"]`, `["neck_draws_least_movement"]`, `STANCE_REQUIREMENTS["neck_max_spread"]` (dropped) and `["framing_min_slack"]` (becomes the view margin), `STAGING_CFG["neck_tol"]`, `["neck_settle_vel"]`, `["neck_settle_s"]`, `["neck_timeout"]`, the "neck is exempt" note of `["exclusive"]`, `TASK_CFG["framing_arms"]` (kept in meaning: the grippers that must be in view), `NECK_SPECS` / `NECK_AUTO_SPECS` / `FACE_CFG` / eagle `CAMERA_SPECS` (robot_specs), `LLM_CFG["vlm_cameras"]` unchanged. The stance comments that quote neck spread and slack numbers (`TASK_CFG` lines about 74 to 98) are rewritten.

### Tests and experiment scripts
- Delete or rewrite: `neck_once_test`, `neck_framing_test`, `neck_rule_test`, `face_test` (face on the neck).
- Rewrite their camera parts: `home_pose_test` (31), `cc_loop_tests` (21), `cc_mode_tests` (25), `vlm_checker_test` (20), `arm_collision_test` (9), `target_mode_test` (8), `full_check_test` (2), `rest_contacts_test` (2), `pedestal_lift_test` (1), `grasp_lift_test` (1), `stance_selection_timing`, `stance_two_points`, `nav_follow_sim_test` (1), `run_tests.sh` (3 entries).
- Experiment scripts that use the neck or the framing slack: `beaker_hotplate_eagle_views` (18), `beaker_hotplate_pose_check` (23), `beaker_hotplate_episode` (10), `combined_controller` (11), `home_pose_search` (23), `home_pose_candidates` (28), `home_pose_coverage` (17), `home_pose_compare` (20), `stance_from_map` (27), `stance_check_profile` (9), `full_check_profile` (5), `stance_sensitivity` (4), `enum_probe`, `enum_variance`, `grasp_ik_targets`, `reach_map_pilot`, `reach_map_build`, `replay_run`, `cc_bench_compare`, `episodes_test_sources`.

### Data and baselines
- Baselines regenerated once: `experiments/baselines/*.csv` (3 files), `data/cc_baseline/*`, the episode references of `data/beaker_hotplate_*`, and every recording used by a replay test (`data/recordings`, `data/mpvi_run`, `data/mpvi_text_run`): the model changes, so old recordings cannot be replayed.
- The VLM replies in `experiments/llm_cache` (12 files) are keyed by the image: the eagle image changes, so the VLM calls are made again (planner replies are unaffected).
- Reach maps (`data/reach_map`, 24 maps): the staleness stamp hashes the robot's bodies and geoms, which include the neck, so the stamp changes. The maps were built with the neck as an obstacle at its drive pose; without it the reachable set can only grow, so accepting them is safe but conservative; rebuilding takes hours. Decide in step 2 (question 3).
- Stance and home pose: the home pose and the TASK_CFG stance were chosen with the neck framing slack (`home_pose_*`); with a fixed camera the visibility numbers change, so the stance may need to be re-searched or re-confirmed (question 4).

### Docs
`docs/design_plan.md` (the neck text in §1 lines about 43 to 61, §2 the face, §3.4 the requirements, §7.1, §8, §12, §13 "ray from the camera"), `README.md` (the neck keys, F7, F8, the camera panel), `docs/mpvi_decisions.md` (status of rows), reports stay as history.

## 4. Questions for David (so I can start step 2 without guessing)
1. The camera's maker, model name or datasheet link.
2. The head / face (`src/face.py`): it sits on the neck's end today. Fixed on the torso next to the camera, or removed with the neck?
3. The reach maps: accept the 24 existing maps (safe, slightly conservative) or rebuild them?
4. The mount placeholder: what position and angle on `openarm_body_link0` should step 2 use until the hardware team decides? (Suggestion: the old eagle camera's pose with the neck at its drive pose is not usable, it looks 30 degrees down from a neck about 0.77 m up the torso; I can choose a mount that sees the bench top and both grippers at the TASK_CFG stance and report it.)
5. The experiment scripts for the home pose search: port them to the fixed view (a rerun of the search with the new visibility test) or leave them as history and mark them obsolete?
