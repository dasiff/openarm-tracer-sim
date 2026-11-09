# World-model audit: where object and stance positions come from (2026-11-05)

Read-only audit by Claude (Haiku 5.5). Nothing run, nothing changed, nothing committed. Static reading of src/, experiments/, models/, the stored data files and the design plan. Line numbers are of the files as they are on branch held-weight-ramp-sag-off.

Classes: (A) live world-model call, which reads MuJoCo; (B) config constant that is a position; (C) literal number in code; (D) scene-file value read directly, bypassing the world model; (E) stored result of an earlier computation; (F) something else.

## 0. Summary of the two handoff claims

1. Stance. **Partly wrong.** TASK_CFG staging_xy, staging_yaw and pedestal_q are a reference only. The pipeline recomputes the stance (x, y, yaw, pedestal) at plan time, once per plan, in `apply_stance_selection` (src/stance_select.py:562-577). It does not recompute at run time, and it reads object positions from config, not from the simulator (see section 4).
2. Hotplate. **Mostly right, but the scene is not 0.65.** The scene file says 0.80. The 0.65 is the demo's shifted value (0.80 + SHIFT_X). task_points.py:47 reads TASK_CFG (0.80 hotplate, 1.00 beaker) whatever layout is passed. It is wrong for a shifted layout, but no pipeline code calls task_points(); only experiments do (section 2).

## 1. Scene-file object poses are dead values

| File | Line | Value | Class | What happens |
|---|---|---|---|---|
| models/objects/beaker.xml | 22 | task_beaker pos 1.00 0.66 0.962 | D | Overwritten at every load: `load_task` calls `apply_task_poses` unconditionally (src/task_beaker_hotplate.py:140), which writes the beaker qpos from TASK_CFG (line 87). |
| models/objects/hotplate.xml | 13 | task_hotplate pos 0.80 0.66 0.96 | D | Overwritten: `apply_task_poses` writes model.body_pos from TASK_CFG (src/task_beaker_hotplate.py:85). |
| models/scenes/chemistry_lab_beaker_hotplate.xml | 358-359 | includes of the two object files | D | The includes only define the defaults. |

**Consequence:** a 4 cm edit to either scene file has no effect on any run. The only working path for a move is TASK_CFG (src/task_config.py:50-51).

## 2. Hotplate: every use and the value it gets

Shift in the demo: `experiments/demo_video.py:43` `SHIFT_X = -0.15`, line 44 `MOVED = {**TASK_CFG, beaker_xy: (x + SHIFT_X, y), hotplate_xy: (x + SHIFT_X, y)}`. The shift is applied to the x component of **both** the beaker and the hotplate, and to nothing else (y stays 0.66). The demo runs with MOVED: `load_task` (demo_video.py:118), `make_pipeline_task` (line 97), and the recorded facts (line 114).

| Use | File:line | Value under TASK_CFG run | Value under MOVED run | Agrees with the model? |
|---|---|---|---|---|
| Scene file default | models/objects/hotplate.xml:13 | 0.80 | (overwritten) | Not used |
| Config | src/task_config.py:51 | 0.80 | 0.80 (MOVED = 0.65) | the source |
| Model body (scene build) | src/task_beaker_hotplate.py:85 | 0.80 | 0.65 | yes (it defines the model) |
| Plan-time world (stance selection) | src/world_model.py:361, used via src/stance_select.py:574 | 0.80 | 0.65 | yes |
| Run-time world (sim) | src/world_model.py:317 (site hotplate_top) | live | live (0.65) | yes |
| Success check | src/task_beaker_hotplate.py:240-241 (live site) | live | live | yes |
| Carry monitor | src/mpvi_monitors.py:111 (plan target) | from plan | from plan | yes |
| **task_points setdown xy** | **src/task_points.py:47** | **0.80** | **0.80 (wrong)** | **NO** |
| task_points plate height | src/task_points.py:44-45 (live plate geom) | live | live (0.65) | yes, but mixed with line 47 |
| Stance home-clearance map | src/stance_select.py:75 (beaker only) | n/a | n/a | n/a |
| Demo layout record | experiments/demo_video.py:114 | 0.80 | 0.65 | record |
| Moved place accuracy | experiments/comp_measure.py:130-132 (MOVED) | n/a | 0.65 | yes |
| Moved test check | experiments/stance_select_moved_test.py:39, 54 | n/a | 0.65 | yes |
| Warning text | experiments/bounce_handoff_metrics.py:328 | prints 0.80 | prints 0.80 | print only |
| Eagle-view midpoint | experiments/beaker_hotplate_eagle_views.py:124 (TASK_CFG) | 0.80 | 0.80 | experiment |
| Pose check | experiments/beaker_hotplate_pose_check.py:74-86 (TASK_CFG) | 0.80 | 0.80 | experiment |
| Finder scene | data/home_pose_search/search_rank1_widest_range/open_b45_selected.json | 0.8 | n/a | stored, unshifted |
| Moved selection report | data/stance_select_moved_report.json (layout field) | n/a | 0.65 | stored, shifted |
| Unshifted selection report | data/stance_select_report.json (no layout field stored) | 0.80 (TASK_CFG) | n/a | stored |

**Where they disagree:** only task_points.py:47 (0.80) against the shifted model (0.65). `task_points()` takes no layout argument, so it always reads TASK_CFG. No pipeline code calls it. The callers are experiments only: f4_full_check.py:19, full_check_test.py:66, c1_validate_stance.py:40, h_validate.py:70, full_check_profile.py:87, g0_raised_check.py:60, bench_stance_test.py:92, bench_stance_diag.py:34, plus the test in src/full_check.py:44 (import only). None of these runs with MOVED. The latent risk: anyone who passes a shifted cfg to these checks gets the points for the unshifted layout without an error.

The `task_points` in beaker_hotplate_layout_search.py:263, 448, 464 is a different function that takes the layout as arguments, so it is fine.

## 3. Position sources: full table

Columns: class; users; does it follow a 4 cm move of the beaker (via TASK_CFG; a scene-file move is "no" for every row, see section 1); the same for the hotplate.

### 3a. Object positions in the pipeline

| # | File:line | Class | What it is used for | Users | Beaker 4 cm | Hotplate 4 cm |
|---|---|---|---|---|---|---|
| 1 | src/task_config.py:50 | B | beaker_xy (1.00, 0.66) | apply_task_poses, StaticTaskSource, HomeClearanceMap, task_points, experiments | yes | n/a |
| 2 | src/task_config.py:51 | B | hotplate_xy (0.80, 0.66) | same | n/a | yes |
| 3 | src/task_beaker_hotplate.py:85 | B to model | hotplate body from cfg | every load | n/a | yes |
| 4 | src/task_beaker_hotplate.py:87 | B to model | beaker qpos from cfg | every load | yes | n/a |
| 5 | src/world_model.py:317 | A | hotplate top, from the site hotplate_top | make_sim_world_model: planner, oracle, mock, monitors | n/a | yes |
| 6 | src/world_model.py:318-320 | A | beaker base, from the site beaker_base | same | yes | n/a |
| 7 | src/world_model.py:359 | B | StaticTaskSource beaker entry | apply_stance_selection (plan time only) | yes | n/a |
| 8 | src/world_model.py:361 | B | StaticTaskSource hotplate entry | same | n/a | yes |
| 9 | src/stance_select.py:574 | B | the plan-time world is StaticTaskSource(task_cfg), not the simulator | apply_stance_selection, once per plan | yes | yes |
| 10 | src/stance_select.py:133-226 (step_target) | A/B (via 9) | stop points, support, obstacles from world entries | stance selection | yes | yes |
| 11 | src/stance_select.py:75 | B | HomeClearanceMap uses beaker_xy | stance selection | yes | n/a |
| 12 | src/stance_select.py:571-572 | B | FullCheck(task_cfg), loads a model at the cfg positions | stance selection | yes | yes |
| 13 | src/mpvi_modules.py:220, 243-244 | A | object_pose(obj), object_pose(dest) | scripted pick and place (also the mock's plan) | yes | yes |
| 14 | src/mpvi_modules.py:227 | A | object_pose(carry_ref = "hotplate") | pick carry height | n/a | yes |
| 15 | src/mpvi_modules.py:420, 427, 531 | A | object_pose during close, lift, oracle | scripted pick, oracle check | yes | n/a |
| 16 | src/mpvi_modules.py:536 | C | pick check tolerance d < 0.02 m | oracle | n/a | n/a |
| 17 | src/task_beaker_hotplate.py:240-241 | D-live (grader) | check_success reads live sites; tolerances are fixed (SUCCESS_CFG, task_config.py:471-480) | oracle, tests | yes | yes |
| 18 | src/task_beaker_hotplate.py:296-298 | D-live | view_points reads live body positions | handoff camera check | yes | yes |
| 19 | src/mpvi_monitors.py:132 | D | beaker xy read from qpos directly, log only | carry slice monitor | yes | n/a |
| 20 | src/mock_policy.py:296 | D | `other = data.qpos.copy()`, the whole qpos incl. the beaker free joint | mock: passed as other_joint_qpos (see section 5) | yes (not allowed) | n/a |
| 21 | src/mock_policy.py:300-301 | A | object_poses for the current object (built, not read by infer) | mock | yes | yes |
| 22 | src/pour_scene.py:102-137 | B | pour task: beaker_xy and second_xy | pour task only, not the beaker-on-hotplate pipeline | yes (pour only) | n/a |
| 23 | src/task_config.py:162 | B | second_xy (1.20, 0.66) | pour task only | n/a | n/a |

### 3b. Stance and robot positions in the pipeline

| # | File:line | Class | What it is used for | Users | Follows beaker 4 cm | Follows hotplate 4 cm |
|---|---|---|---|---|---|---|
| 24 | src/task_config.py:78 | B (E: finder output) | staging_xy (0.7650, 1.0850): reference only; used with navigation off and by tests and map building | stance_select.py:71, 376; staging; tests | no | no |
| 25 | src/task_config.py:83 | B | staging_yaw -90 deg (bench facing) | same | no | no |
| 26 | src/task_config.py:84 | B (E) | pedestal_q 0.1550: reference from the same finder run | same | no | no |
| 27 | src/task_config.py:82 | B | staging_approach_back_m 0.2063: approach spot, "old staging spot" | staging_source.py creep-in | no | no |
| 28 | src/task_config.py:65 | B | spawn_xy (0.70, 3.40): robot start; coupled to the staging heading (comment lines 20-25) | staging_source.py:444 | no | no |
| 29 | src/task_config.py:92-93 | B (found for the unshifted scene) | home_q_left/right: the zombie pose | staging ARMS, full check, mock home | no | no |
| 30 | src/world_model.py:42 | B | _stance(cfg): the bench entry's stance | world.stance() for navigation targets | no | no |
| 31 | src/world_model.py:327, 331 | B | bench entry position = staging_xy | sim and static sources | no | no |
| 32 | src/mpvi_modules.py:108-109 | B | navigation: stance must equal task_cfg staging, else error | scripted navigation | no | no |
| 33 | src/stance_select.py:376 | B | navigation off: the stance stays at TASK_CFG's | stance selection (off by default) | no | no |
| 34 | src/stance_select.py:515, 533 | E-like (run output) | chosen stance written to task_cfg for the staging | staging_source.py:382, 436 (task_cfg updated) | yes (via the selection) | yes (via the selection) |
| 35 | src/staging_source.py:382, 436 | B (from task_cfg) | goal = the chosen stance | staging phase | yes (via 34) | yes (via 34) |
| 36 | src/bench_stance.py / src/stance_finder.py | E/F | layout-agnostic: take layout as arguments | selection, finder | yes | yes |
| 37 | data/bench_region_cache, data/reach_map | E | robot-frame regions; cache key has no object xy | selection | yes (no re-build needed) | yes (no re-build needed) |
| 38 | src/reach_map.py / reach_map_build.py:76, 92 | B (into the stamp) | bench base pose and pedestal from TASK_CFG; changing staging_xy makes the maps stale | selection | no | no |
| 39 | src/bench_stance.py:117-119 | C | search ranges (forward_range, lateral_range) in the robot frame | selection | yes (the range is relative) | yes |

### 3c. Experiments and tests (grouped)

- Read TASK_CFG beaker/hotplate (12 files): beaker_hotplate_pose_check.py:74-86, beaker_hotplate_eagle_views.py:124, h_validate.py:47, 68, beaker_hotplate_success_test.py:69, camera_mount_search.py:50, 121, nav_follow_test.py:30, reach_map_build.py:76, 92, and the others. Follow a TASK_CFG move: yes; a shifted cfg: no. Class B.
- Literal layouts (class C, fixed test scenes; follow nothing): transit_clearance_test.py:110 `hp_xy = (0.70, 0.66)`, :89, 93 `(0.85, 0.66)`, :150; stance_select_test.py:109 `(-0.9, 0.66)`, :125 `(1.0, 0.80)`, :165 `(1.45, 0.66), (1.25, 0.66)`.
- apply_task_poses with ctx-based layout (home_pose_candidates.py:110, stance_check_profile.py:142, stance_selection_timing.py:75, stance_from_map.py:177): follow the layout that is passed (yes).
- MOVED-based (class B, derived): demo_video.py:43-44; stance_select_moved_test.py:38-39 (a second copy of SHIFT_X and MOVED); comp_measure.py:130-132.

### 3d. Stored results (class E)

| File | Layout it was made for | Stored value | Follows a 4 cm move? |
|---|---|---|---|
| data/home_pose_search/search_rank1_widest_range/open_b45_selected.json | beaker (1.0, 0.66), hotplate (0.8, 0.66) | chosen (0.765, 1.085), q 0.155, heading -90 | no (the finder run would have to be repeated) |
| data/stance_select_report.json | TASK_CFG (no layout field stored) | chosen (0.755, 1.145), q 0.10 | no |
| data/stance_select_moved_report.json | beaker (0.85, 0.66), hotplate (0.65, 0.66) | chosen (0.635, 1.135), q 0.10, distance from TASK_CFG staging 0.139 m, outcome SUCCESS | no (stored) |
| data/camera_mount_search.json | TASK_CFG stance | stance and margins | no |
| data/bench_stance_last.json | last run | stance | no |

The stored selections differ from TASK_CFG's reference (0.765, 1.085, q 0.155), which shows the pipeline does recompute the stance per layout.

## 4. Stance inventory: what fixes the stance

| Item | Value | Where it came from | Depends on the object positions? | Recomputed at run time? |
|---|---|---|---|---|
| staging_xy | (0.7650, 1.0850) | Finder run 2026-10-20 (data/home_pose_search/search_rank1_widest_range/open_b45_selected.json), scene beaker 1.00 hotplate 0.80, grasp band 45-60 %, left arm, home pose below (task_config.py:31-38) | Yes (the finder scene) | No |
| staging_yaw | -90 deg | Facing the bench (decision M079) | No | No |
| pedestal_q | 0.1550 | Middle of the passing range 0.10 to 0.21 from the same finder run | Yes (same run) | No |
| staging_approach_back_m | 0.2063 | Old staging spot where the straight arm move was verified (task_config.py:82) | Indirect | No |
| home_q_left / right | 22.6, -14.7, 15.5, 126.1, -3.7, 27.3, 13.0 deg (mirror for the right) | home_pose_search rank 1 for the unshifted scene; zombie pose, placeholder until hardware gives the real home (task_config.py:92-93) | Yes (found for the unshifted layout) | No |
| spawn_xy / spawn_yaw | (0.70, 3.40), +90 deg | Chosen so the final turn is small for the staging heading (task_config.py:20-25, 65) | Indirect (the heading to staging) | No |
| Chosen stance (per plan) | x, y, yaw, pedestal from the selection | apply_stance_selection (stance_select.py:562-577) | Yes: plan time, from the config positions (StaticTaskSource) | Once per plan; never at run time |

**What depends on the beaker or hotplate when the stance is chosen:** the selection's world (StaticTaskSource, from task_cfg), the bench region (robot-frame maps, cached, layout-independent), the overlay of object regions at the object positions, HomeClearanceMap (beaker), the carry-slice check, and FullCheck (its own model at task_cfg positions, with points built from the world entries in step_target).

**Run-time check:** none. FullCheck.check is called only from stance_select.py:482 at plan time. src/full_check.py:3-5 is a docstring example. The monitors (src/mpvi_monitors.py) log only. No code checks the stance against the simulator's current object positions during a run.

## 5. Mock policy: cheat inventory

The mock's observation (src/mock_policy.py:9-15; built in make_mock_source, lines 294-304):

| Input | Where it comes from | Read by the mock? | Allowed by David? |
|---|---|---|---|
| state (16) | adapter_state (lines 50-59): robot joints and gripper, from MuJoCo data | Yes (line 138) | Not a cheat: the real robot's sensed state is the same input |
| prompt | the plan's segment description (line 96-99) | Stored at begin(); obs prompt not read | Not a cheat: the real policy gets the prompt |
| images | none | No | Not a cheat (the mock has no vision) |
| cheats.clock | obs time (line 294, 301) | Not read by infer; the timing comes from the source's sim time (advance, t) | Yes (timestamp) |
| cheats.plan_memory | the mock's own plan (self.plan, self.i) | Built (301), not read by infer; the mock's memory is internal | Yes (its own plan memory) |
| cheats.object_poses | world.object_pose(mock.obj) (line 300), sim ground truth via the world model | Built (301), not read by infer; object positions reach the mock through the plan at begin() (mpvi_modules.py:220-244) | Yes (simulator object poses, through the world model) |
| cheats.other_joint_qpos | data.qpos.copy() (line 296) | Yes: read at line 146, passed to advance and copied into a scratch MjData (lines 115-119) | **Not allowed as written.** The label says non-arm robot joints; the code passes the whole qpos, including the beaker's free joint (7 numbers: the beaker's pose), read straight from MuJoCo, bypassing the world model |
| Robot constants: home_q (TASK_CFG), finger_open (TASK_CFG) | config | Yes | Robot constants, not object poses |

**Object poses from config or the scene file:** none. The mock's object positions come from the world model (sim source) through the planner. The one exception is the beaker pose inside other_joint_qpos (line 296), which is a direct MuJoCo read. The nav stance comparison in mpvi_modules.py:108-109 uses config, but navigation is the scripted staging, not the mock.

**Does the beaker part of other_joint_qpos change the mock's actions?** Probably not: the pad pose depends on the arm joints, and the beaker is not in the arm chain. This is a rule violation, not a known effect on the output. It was not tested (no runs).

## 6. Success checks and thresholds

| Check | File:line | Compares against | Follows a move? |
|---|---|---|---|
| check_success (on plate, at rest) | src/task_beaker_hotplate.py:217-260 | Live sites (beaker_base, hotplate_top) and the live plate geom; fixed margins 0.01 m, 0.005 m, 15 deg, 0.01 m/s (SUCCESS_CFG, task_config.py:471-480) | Yes |
| Oracle pick check (held) | src/mpvi_modules.py:519-538 | world.object_pose (live) and tolerance d < 0.02 m (line 536) | Yes |
| Oracle place check | src/mpvi_modules.py:540-545 | check_success diagnostics (live) | Yes |
| FullCheck stance check | src/stance_select.py:482 | Points from step_target (world entries) | Yes (plan time) |
| Test: moved positions | experiments/stance_select_moved_test.py:54 | MOVED positions | Yes |

No success check compares against a fixed position number. The thresholds are tolerances, not positions.

## 7. Hidden coupling (assumes a layout without reading it)

1. **task_points.py:47** (above): TASK_CFG positions for any shifted or moved cfg. Latent, experiments only.
2. **Home pose** (task_config.py:31-38, 92-93): searched and verified for beaker 1.00 and hotplate 0.80. The stance selection checks home clearance for the beaker only (stance_select.py:75). No run re-checks the hotplate's home occlusion for a moved layout.
3. **Spawn and approach** (task_config.py:20-25, 65, 82): the spawn was chosen so that the turn to the staging heading is small. The comment says to re-run the layout search when the spawn changes. A moved stance (0.139 m from the reference in the stored moved run) changes that heading and is not re-checked.
4. **Staging stance in the reach-map stamp** (reach_map_build.py:76, 92): changing staging_xy or pedestal_q changes the stamp; the maps must be rebuilt.
5. **Demo shift** (demo_video.py:43-44; stance_select_moved_test.py:38-39): a second copy of SHIFT_X exists, and the shift is fixed at -0.15 m on x.
6. **Pour task** (task_config.py:162, pour_scene.py): the second beaker is fixed at (1.20, 0.66) relative to the first beaker's old position (1.00); it does not follow a beaker move. Pour only.
7. **Test layouts** (transit_clearance_test.py:110; stance_select_test.py:109, 125, 165): literal layouts.
8. **Pick check tolerance** (mpvi_modules.py:536): 0.02 m, a literal, not a position.

## 8. Legitimately fixed by design (considered; not decided)

- Robot geometry, joint limits, the generated robot file (models/robot/, from tools/import_openarm_description.py).
- Bench geometry in models/scenes/chemistry_lab_beaker_hotplate.xml (bench top at z 0.9, 2.2 x 0.8 m half extents), the bench rails removed by `bench_open` (task_beaker_hotplate.py:~95-100, task_config.py:55 `bench_open: True`), walls, lights, the hood.
- The eagle camera (fixed to the torso; no neck).
- Pedestal range and step (BENCH_STANCE_CFG pedestal_range, pedestal_step) and the search ranges in the robot frame.
- staging_yaw -90 deg (the bench's facing direction).
- The Tracer footprint (src/nav_footprint.py), the spawn and the drive limits (but see section 7, item 3).

## 9. Ranked list: positions that do NOT come from the world model

Ranked by how much breaks if the beaker or hotplate moves by 4 cm (scene file or TASK_CFG as the route):

1. **models/objects/beaker.xml:22, models/objects/hotplate.xml:13 (D).** A scene-file move does nothing; the run stays at TASK_CFG. Breaks the whole move request, silently. Overwritten by task_beaker_hotplate.py:85, 87, 140.
2. **src/task_config.py:50-51 (B), read at plan time by src/world_model.py:359-361 and src/stance_select.py:574.** The pipeline's plan-time world is built from config numbers, not from the simulator. A move that is not written into TASK_CFG (a reset, a future detector) would leave the stance selection and the plan on the old positions. Breaks the stance and the plan.
3. **src/task_points.py:47 (B).** Reads TASK_CFG beaker and hotplate whatever layout is passed. Wrong for any shifted cfg. Breaks the finder and full-check experiments (10 files); the pipeline does not use it.
4. **src/world_model.py:327, 331, 42 and src/mpvi_modules.py:108-109 (B).** The bench entry's stance and the navigation check come from config staging_xy. Stance is still chosen per plan with navigation on (stance_select.py:515), so it follows; with navigation off (stance_select.py:376) it stays fixed. Breaks navigation-off runs and tests.
5. **src/task_config.py:92-93 (home pose), 65 (spawn), 82 (approach).** Found or verified for the unshifted layout; not re-checked for a moved layout. Breaks the moved layout's clearance and drive, partly.
6. **src/mock_policy.py:296 (D).** The beaker's pose from MuJoCo passed to the mock as other_joint_qpos. Rule violation (not an allowed cheat); probably no effect on the actions.
7. **Stored results (E):** data/home_pose_search/... (unshifted), data/stance_select_report.json (unshifted), data/stance_select_moved_report.json (shifted), data/camera_mount_search.json, data/bench_stance_last.json. Stale after any change; not read by the pipeline.
8. **src/mpvi_monitors.py:132 (D, log only).** Beaker xy from qpos. Logging only.
9. **Test and experiment literals (C):** transit_clearance_test.py:110, stance_select_test.py:109, 125, 165; eagle-view and pose-check experiments (TASK_CFG, B).
10. **Demo shift copies (B):** demo_video.py:43-44 and stance_select_moved_test.py:38-39. Both fixed at -0.15 m on x.
11. **Pour (B):** task_config.py:162, pour_scene.py:102-137. Not in the beaker task.

## 10. Methods, not done

- No simulator was run; all claims are from reading the code and the stored JSON.
- Not checked: every recording meta.json under data/ (only the ones named above), and whether the mock's scratch MjData uses the beaker geometry in any contact check (only kinematics was checked).
- The progress log (docs/progress.md) was not updated: the task said to change nothing.
- This report is not committed, as requested.
