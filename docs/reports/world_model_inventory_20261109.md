# World model inventory (2026-11-09)

Read-only. No code or config changed, no simulation run. Static reading of `src/`, `experiments/` and the docs, plus the existing 2026-11-05 audit (`docs/reports/world_model_audit_20261105.md`) for the scene-file values.

## 1. World information providers

| module (class or function) | provides | data source | can add or update entries at run time | consumers |
|---|---|---|---|---|
| `src/world_model.py` `WorldModel` | knowledge base of entries (beaker, hotplate, bench): position, axis, properties, footprint, height range, found-how tag | none itself; filled by sources | yes: `upsert`, `remove`, `refresh` (each source writes on refresh) | mpvi_modules, mock_policy, grounding, support, stance_select |
| `src/world_model.py` `SimGroundTruthSource` | beaker body pose, hotplate pose, bench top (from the live model) | simulator (`data.xpos`, `data.xmat`, geoms) | yes: re-read on every `object_pose` / `refresh` | `mpvi_modules.make_sim_world_model` (used by `mpvi_pipeline.py:76`), `stance_select.py:80` |
| `src/world_model.py` `StaticTaskSource` | the same three entries from config and scene heights, no simulation | config (`TASK_CFG`, `BEAKER_CFG`) and scene heights (`task_heights`) | yes (on each call), but no sim | **no production consumer**: only experiments (`checker_start_compare.py`, `mpvi_trace.py`, `place_theta_report.py`, `stance_failure_six_run.py`, `stance_select_test.py`, `scene_heights_test.py`) |
| `src/task_beaker_hotplate.py` `task_heights` / `scene_heights` / `bench_top_z` | bench top z, plate top z, beaker z and height | config and model geoms | no (read at build) | `StaticTaskSource`, `world_model.bench_top_z`, `mpvi_modules` |
| `src/furniture_map.py` `FurnitureMap.from_model` | two-layer floor footprint of every world geom (base layer 0 to 0.43 m, arm layer 0.43 to 2.0 m) | simulator geoms (at build, from a scratch `MjData`) | no (build once) | `stance_select.py:77` (beaker excluded), `experiments/stance_from_map.py`, `bench_stance_validate.py`, `furniture_map_test.py` |
| `src/nav_footprint.py` `obstacles_from_model` | navigation obstacles from world geoms (beaker and hotplate excluded) | simulator geoms | no (called per staging call) | `staging_source.py:462-468` |
| `src/world_collision.py` | bench geoms as boxes (world obstacles for stance passes) | simulator geoms | no | `stance_passes.py` (margin `world_margin_m`), `stance_checks.py` |
| `src/support.py` | surface chain and grasp height for a manipulation step | `WorldModel` entries (no own data) | no | `grounding.py`, `mpvi_modules.py` |
| `src/grounding.py` | matches a step's object phrase to a world entry (deterministic text match) | `WorldModel` entries | no: the search module is a stub (`make_search_stub` returns `not_implemented`), operator cannot answer unattended (`make_operator(present=False)`) | mpvi_modules, orchestrator |
| `src/vla_instruction.py` | the VLA instruction text from grounded entry names | `WorldModel` (names) | no | mpvi_modules |
| `src/mock_policy.py` | object pose sent to the mock policy as a "cheat" (`object_poses`) | `WorldModel` (sim ground truth) | no | mock policy only (`mock_policy.py:302`) |
| `src/mpvi_planner.py` | the plan from the instruction text only (Gemini, structured) | text instruction; no world data | no | orchestrator |
| `src/mpvi_vlm.py` | completion verdict from the eagle image (Gemini); no positions | eagle camera image | no | orchestrator (gripper events) |
| `src/eagle_view.py` | where world points fall in the fixed eagle image (kinematic projection) | simulator (kinematics) | no | `stance_checks.py` (view margins) |
| `src/stance_checks.py` `StanceChecks` | clearance, view margin, occlusion, beaker-on-hotplate clearance | simulator (targets by body name) | no (built once per scene) | stance_select, full_check |
| `src/task_points.py` `task_points` | the 12 task points (grasp, set-down, above) for the finder and the full check | simulator (hotplate top plate geom) | no | stance_finder, full_check, mpvi_modules |
| `src/home_clearance_map.py` | clearance map of the zombie pose against world geoms | simulator geoms | no | stance_select |

Sources that can change world entries at run time: `SimGroundTruthSource` and `StaticTaskSource` (through `refresh`). The "detector" and "operator" found-tags in the `Entry` docstring are not implemented.

## 2. Ground-truth reads (non-robot bodies or geoms from the simulator or the model)

Robot bodies, joints, actuators, sensors, the gripper and the head are excluded. Two classes: reads that feed a decision the robot makes (or a stance, path or navigation choice), and evaluation or diagnostics only.

### 2a. `src/`

| file:line | what it reads | used by | evaluation/diagnostics only, or feeds a decision |
|---|---|---|---|
| `world_model.py:274-276` | beaker body id, hotplate body id, beaker geom ids | `SimGroundTruthSource` (`object_pose`) | **decision** (object poses of every pick and place) |
| `world_model.py:305-325` | beaker `xpos` and `xmat`, hotplate `xpos` and `xmat` (live) | `SimGroundTruthSource.__call__` | **decision** |
| `world_model.py:327` | `bench_top` geom | bench entry (top z, stance) | **decision** |
| `mpvi_modules.py:70-82` | builds the sim source for the pipeline | pipeline | **decision** (source of all world poses) |
| `mpvi_modules.py:234-257, 330-333, 570-571` | `object_pose` and `object_height` of the beaker and destination | pick and place waypoints, grasp height | **decision** |
| `mpvi_modules.py:459, 466` | beaker z at close and during lift (logged) | lift measurement | logged measurement (diagnostics) |
| `mpvi_modules.py` oracle checker (`make_oracle_checker`, ~558-587) | beaker and hotplate poses for each check | non-deciding events return the oracle verdict (`mpvi_vlm.py` docstring) | **decision** for events the VLM does not decide |
| `stance_select.py:77` | `FurnitureMap.from_model` on the model (beaker excluded) | stance selection | **decision** |
| `stance_select.py:87-88` | `bench_top` geom `xpos` and `size` | stance selection | **decision** |
| `furniture_map.py:42-44, 82-87, 156-172` | every world geom's pose and size (world, body frame of the robot root) | furniture map | **decision** (stance, navigation) |
| `task_points.py:45-46` | hotplate top plate `geom_xpos` and size | task points (finder, full check) | **decision** |
| `task_beaker_hotplate.py:66-67` | `bench_top` geom `geom_pos` and `geom_size` (model, static) | `bench_top_z` | **decision** |
| `task_beaker_hotplate.py:80-81` | hotplate top plate `geom_xpos` (live) | `scene_heights` | **decision** |
| `task_beaker_hotplate.py:295-299` | beaker body `xmat` (up axis) and hotplate base footprint | footprint and stance checks | **decision** |
| `task_beaker_hotplate.py:348-353` | beaker body `xpos`, hotplate body `xpos`, base geom size | scene box builder | **decision** (obstacle boxes) |
| `task_beaker_hotplate.py:269-301` | beaker free joint position and velocity | `check_success` (episode judge) | **evaluation** (ends the episode on success) |
| `stance_checks.py:82-84, 119, 197-201` | target bodies (beaker, hotplate) and world geom list | stance checks, occlusion | **decision** |
| `stance_checks.py:264-269` | beaker body and hotplate top plate; beaker free joint | beaker-on-hotplate clearance | **decision** |
| `full_check.py:90` | beaker body id | full check clearance | **decision** |
| `full_check.py:336, 616` | hotplate body id | full check clearance | **decision** |
| `stance_finder.py:123-130` | `bench_top` geom pos and size; battery body | bench slab for the finder | **decision** |
| `world_collision.py:65, 103-108, 132-141` | world geom poses and sizes (bench boxes) | stance passes | **decision** |
| `home_clearance_map.py:47-68, 135-136` | world geom poses and sizes (exclusions by body name) | home clearance map | **decision** |
| `nav_footprint.py:47-84` | world geoms (beaker and hotplate excluded) | navigation obstacles | **decision** |
| `staging_source.py:500-505` | base geoms; world geoms; bench geoms | staging clearance | **decision** |
| `mock_policy.py:302` | `world.object_pose` (sim ground truth) | mock policy cheat input | **decision** for the mock policy only (cheat by design) |
| `pour_scene.py:83, 117, 132, 138-172` | bench geom; body poses; ball positions | pour scene setup and reads | evaluation/setup (pour scene only) |
| `signal_trace.py:77-80` | beaker body id; bench geoms | signal trace log | **diagnostics** |
| `elbow_clearance_log.py:33, 41` | bench geoms and body geoms | elbow clearance log | **diagnostics** |
| `robot_helpers.py:85` | floor = body 0 (world) for contact classes | contact logs | **diagnostics** |
| `mpvi_monitors.py:95` | beaker free joint qpos | drop monitor | diagnostics (logs; the plan change it may cause was not traced) |
| `gravity_ff.py:83-84` | body masses (all bodies with mass; includes the beaker if it is in the model) | gravity feedforward | not traced further |

### 2b. `experiments/`

117 matching lines in 40 files. The per-file counts and the full list are in Appendix A. In short: the largest are `episode_test_sources.py` (13), `beaker_hotplate_layout_search.py` (10), `beaker_hotplate_staging_search.py` (8), `beaker_hollow_test.py` (8). Most are tests, searches or measurements, so they are evaluation/diagnostics. Some feed `src/` decisions only through the functions they call; those calls are the `src/` rows above.

## 3. Perception (locating objects from camera images)

| file | inputs | outputs | how the object is identified | measured accuracy (report) | speed | used today |
|---|---|---|---|---|---|---|
| `src/mpvi_vlm.py` | eagle camera image at a gripper event; the paper's completion prompt | pass/fail verdict with reason (no position, size or pose) | the VLM is asked whether the step is complete, not what an object is | live run: 11/11 agree with the oracle, and the VLM decides the completion checks (`docs/reports/20261021_1050_vlm_checker_alignment.md`, and `20261021_0335_vlm_checker_live_runs.md`) | VLM call latency not measured here | **yes** (completion checks) |
| `src/eagle_view.py` | simulator kinematics (point positions, camera pose) | margins to the image edge (fraction) | not detection: projects known points | none (kinematic) | fast (kinematic) | **yes** (stance view margins) |
| `src/grounding.py` | world entries and the step's phrases | entry matched to phrase | text matching (phrases, properties), no image | n/a | fast | **yes** |
| `src/llm_gemini.py`, `mpvi_planner.py`, `mpvi_postprocess.py` | text and images through the LLM calls | plan, labels | LLM, no localization | n/a | n/a | **yes** (plan and labels; no object localization) |
| known-surface method (ray from the eagle through a detection, intersected with the known bench height) | (would be) camera pose and detection | (would be) object position | not implemented | none | n/a | **no**: design only (`docs/design_plan.md` line 498) |
| open-vocabulary detector (GroundingDINO on the eagle camera) | (would be) image and lab vocabulary | (would be) object boxes | not implemented: `make_search_stub` returns `not_implemented` (`grounding.py:81-87`) | none | n/a | **no**: design only (`docs/design_plan.md` lines 307, 333, 491; §13) |

No code in the pipeline locates objects from camera images today. The only image consumer that decides something is the VLM completion checker, and it returns a verdict, not a position. The eagle camera has no depth in the simulator (`docs/design_plan.md` line 558).

## 4. Furniture and the lab map

**Correction (2026-11-09, see section 5 below):** the hotplate was listed in this table as furniture. It is an object, like the beaker. Only the bench and the walls are fixed furniture.

| item | where it is defined today | usable as a lab map as-is? |
|---|---|---|
| bench top | `models/scenes/chemistry_lab_beaker_hotplate.xml`: body `bench`, geom `bench_top` (box half-sizes 2.2 x 0.8 x 0.06 m, centre z 0.90 m, top 0.96 m); the top height also in `task_beaker_hotplate.py:66-67` | no: this is the simulator scene, not a surveyed map |
| bench legs | the same scene file: `bench_leg_fl`, `bench_leg_fr`, `bench_leg_bl`, `bench_leg_br` | no |
| bench position | `TASK_CFG` `staging_xy` and `staging_yaw` (`src/task_config.py`); recomputed at plan time by stance selection (`stance_select.py:562-577`, per the 2026-11-05 audit) | no |
| hotplate | `models/objects/hotplate.xml` (body `task_hotplate`, base box half 0.09 m, top plate half 0.075 m); pose `TASK_CFG` `hotplate_xy` (0.80, 0.66 in the scene file, overwritten at load by `apply_task_poses`, see the audit) | no |
| room, walls, other furniture | the same scene file (sink, fume hood, back counter, shelves, storage cabinet, etc.) and `FURNITURE_CFG` (`src/task_config.py:228`: grid bounds, bands) | no: a simulated scene with placeholders |

Design intent (`docs/design_plan.md` lines 456-462): the furniture and fixture geometry come from the scene model in simulation, and a real lab map ("a built map with labeled fixtures", SLAM, §8 or lines 452-454) is later. No existing file is a surveyed lab map; `data/` has no lab map file, and no report calls one a lab map. So the answer is: no existing file can serve as the lab map as-is.

## 5. Fixed furniture (correction and scope)

Fixed = never moves in a run: the bench top and legs, and the walls (they have collision geometry). The hotplate is an object, not fixed furniture (correction to section 4). Other static items in the scene (back counter, fume hood, storage cabinet, sink, safety station, etc.) are also fixed but outside the scope you set; they are listed in the note at the end of this section.

| item | where defined today | compiled contact flags | pose defined by | usable as a lab map as-is? |
|---|---|---|---|---|
| bench top | `models/scenes/chemistry_lab_beaker_hotplate.xml:52-53`: body `bench` (`pos` 0 0 0, line 50), geom `bench_top` box half-size 2.2 x 0.8 x 0.06 m, centre z 0.90 m (top 0.96 m) | collide yes (1, 1) | scene XML (fixed at the origin) | no |
| bench legs | same file, lines 64-70: `bench_leg_fl`, `bench_leg_fr`, `bench_leg_bl`, `bench_leg_br`, box half-size 0.06 x 0.06 x 0.45 m at x ±2.0, y ±0.7 | collide yes | scene XML | no |
| bench frame rails | same file, lines 55-61: `bench_frame_front/back/left/right` | removed at load: `remove_bench_rails` (`src/task_beaker_hotplate.py:147`) deletes them, the open-bench design (§4.1). Not in the compiled model. | scene XML (deleted) | not applicable |
| walls | `models/scenes/chemistry_lab_beaker_hotplate.xml:24-31`: `wall_n`, `wall_s`, `wall_e`, `wall_w`, box half-sizes 6.0 x 0.08 x 1.5 m (inner faces at ±5.92 m) | collide yes (1, 1); comment says they collide since 2026-10-16 | scene XML | no as a surveyed map; the room bounds are also in `FURNITURE_CFG["bounds"]` (`src/task_config.py:230`, ±6.0 m) |
| bench position (stance reference) | `src/task_config.py:80`: `staging_xy` (0.7650, 1.0850) and `staging_yaw`, a reference only; the pipeline recomputes the stance at plan time (`stance_select.py:562-577`, per the 2026-11-05 audit) | n/a | config, reference | no |

Answer: **no existing file can serve as the lab map as-is.** The bench and walls are placeholders of the simulated scene (the bench's pose is the XML origin; the walls are a 12 m box). A real lab map is a later item (design plan lines 456-462).

Note on other fixed scene items (not in your list): the back counter, fume hood, storage cabinet and sink have collision geometry and are fixed in the XML. The rolling cart, lab stool, bottles, flasks, microscope, balance, test-tube rack, waste bin and burner are movable in principle (listed as objects below). This is a question for you: whether the lab map should include the fixed items above.

## 6. Replacements (decision-feeding reads from section 2a)

Rule applied: robot state may come from the simulator; world information only from the world model, with the lab map for fixed furniture and perception for everything else. Classes: **robot state** (allowed as-is), **lab map** (fixed furniture), **perception** (objects, including the hotplate), **unclear** (needs a decision). Evaluation-only reads are not listed; they stay as they are.

| file:line | current read | should come from | note |
|---|---|---|---|
| `world_model.py:274-276` | beaker and hotplate body ids; beaker geoms | perception | object identity via the detector |
| `world_model.py:305-325` | beaker and hotplate live poses | perception | the world model's `found = "detector"` path (design plan 13) |
| `world_model.py:327` | bench top geom | lab map | fixed furniture |
| `world_model.py:285-303` `held_by` | which hand touches the beaker (sim contacts between finger geoms and beaker) | robot state | the robot's own holding verdict (`src/holding.py` HoldDetector: finger force and position); the sim contact check should not decide |
| `mpvi_modules.py:70-82` | builds the sim source for the pipeline | perception | the source is replaced |
| `mpvi_modules.py:234-257, 330-333, 570-571` | beaker and destination poses and height (pick and place waypoints, grasp height) | perception | objects |
| `mpvi_modules.py` oracle checker (~558-587) | beaker and hotplate poses at each check; decides non-deciding events | **unclear** | the sim oracle is ground truth. Options: remove it from the decision, or replace its pose inputs with perception (needs David's decision) |
| `stance_select.py:77` | furniture map from all world geoms (beaker excluded) | lab map (fixed) + perception (objects) | split by layer |
| `stance_select.py:87-88` | bench top geom pose and size | lab map | fixed furniture |
| `furniture_map.py:42-44, 82-87, 156-172` | every world geom pose and size | lab map (fixed) + perception (objects) | split by body |
| `task_points.py:45-46` | hotplate top plate (geom pose and size) | perception | the hotplate is an object |
| `task_beaker_hotplate.py:66-67` | bench top z from the model | lab map | fixed furniture (bench top height) |
| `task_beaker_hotplate.py:80-81` | hotplate top z from live geom | perception | object |
| `task_beaker_hotplate.py:295-299` | beaker axis and hotplate footprint | perception | objects |
| `task_beaker_hotplate.py:348-353` | scene boxes of beaker and hotplate | perception | objects |
| `stance_checks.py:82-84, 119, 197-201` | target bodies (beaker, hotplate) and world geoms | perception (objects); lab map (fixed furniture) | split |
| `stance_checks.py:264-269` | beaker on the hotplate; beaker free joint | perception | object |
| `full_check.py:90` | beaker body id | perception | object |
| `full_check.py:336, 616` | hotplate body id | perception | object |
| `stance_finder.py:123-130` | bench slab (top and underside) | lab map | fixed furniture |
| `world_collision.py:65, 103-141` | bench geoms as boxes (world obstacles) | lab map | fixed furniture |
| `home_clearance_map.py:47-68, 135-136` | world geoms for the home clearance | lab map (fixed) + perception (objects) | split |
| `nav_footprint.py:47-84` | world geoms as navigation obstacles (beaker and hotplate excluded) | lab map (fixed) + perception (movable objects) | split |
| `staging_source.py:500-505` | base geoms, world geoms, bench geoms | base geoms: robot state; bench: lab map; other world geoms: lab map or perception | split |
| `mock_policy.py:302` | object pose as a "cheat" to the mock policy | perception | mock policy only; the cheat has to go if the mock policy is used in a decision path |
| `gravity_ff.py:83-84` | body masses of all bodies with mass (beaker mass from the model) | **unclear** | the beaker's held mass should be the robot's own weighing estimate (`weigh_helper.py`, M258), which would be robot state. Not traced further |
| `check_success` (`task_beaker_hotplate.py:269-301`) | beaker position and velocity | evaluation only, no change | it grades the run and ends the episode; it is not a robot decision |

Conflict to note: `docs/design_plan.md` lines 456-462 say fixed fixtures such as the hotplate come from the lab map. Under your rule the hotplate is an object (perception). The design plan text should be corrected or the rule should be confirmed.

## Not done

- The lab-map question was answered from the docs and files; no hardware or external map was checked (none is in the repo).
- `gravity_ff.py:83-84` (whether the beaker is in its body list) and `mpvi_monitors.py:95` (whether the drop monitor changes the plan) were not traced further.
- VLM call latency and the number of images per check were not measured (no run).
- `experiments/` reads were listed by pattern, not checked one by one for decision use (their results go into `src/` only through the calls listed in 2a).
- Commit and progress lines: see the final message.

## Appendix A: `experiments/` ground-truth read lines (pattern match, robot names excluded)

```
experiments/grasp_ik_targets.py:58:        self.world = [g for g in range(model.ngeom) if (model.geom_contype[g] or model.geom_conaffinity[g]) and int(model.geom_type[g]) != int(mujoco.mjtGeom.mjGEOM_PLANE)
experiments/stance_sensitivity.py:78:        m.body_pos[m.body("task_hotplate").id] = [ctx.hotplate_xy[0], ctx.hotplate_xy[1], ctx.top]
experiments/bounce_handoff_metrics.py:215:    qa = lambda name: int(model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)])
experiments/bounce_handoff_metrics.py:222:    point = data.xpos[tcp] + data.xmat[tcp].reshape(3, 3) @ pad_local
experiments/bounce_handoff_metrics.py:325:    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, SUCCESS_CFG["hotplate_site"])
experiments/grip_pad_cause.py:55:            Vw = V @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
experiments/grip_pad_cause.py:96:    bb = model.body("task_beaker").id
experiments/grip_pad_cause.py:129:            bpos = data.xpos[bb].copy()
experiments/grip_pad_cause.py:130:            Rb = data.xmat[bb].reshape(3, 3)
experiments/grip_pad_cause.py:141:                r = data.geom_xpos[g] - bpos
experiments/sag_feedforward_check.py:69:        point = data.xpos[tcp] + data.xmat[tcp].reshape(3, 3) @ pad_local
experiments/stance_selection_timing.py:73:    m.body_pos[m.body("task_hotplate").id] = [ctx.hotplate_xy[0], ctx.hotplate_xy[1], ctx.top]
experiments/reach_inverse_scene.py:53:    plate = m.geom("task_hotplate_top_plate")
experiments/episode_test_sources.py:113:    k = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, WELD_NAME)
experiments/episode_test_sources.py:117:        b1, b2 = model.body("task_beaker").id, model.body("bench").id
experiments/episode_test_sources.py:119:        mujoco.mju_negQuat(q1inv, data.xquat[b1])
experiments/episode_test_sources.py:120:        mujoco.mju_mulQuat(q_rel, q1inv, data.xquat[b2])
experiments/episode_test_sources.py:122:        model.eq_data[k, 3:6] = data.xmat[b1].reshape(3, 3).T @ (data.xpos[b2] - data.xpos[b1])     # body2 (the bench) in body1's (the beaker's) frame
experiments/episode_test_sources.py:145:    R = data.xmat[hand].reshape(3, 3)
experiments/episode_test_sources.py:146:    pad = data.xpos[hand] + R @ np.asarray(C["pad_center_in_tcp"], float)
experiments/episode_test_sources.py:149:    b = model.body("task_beaker").id
experiments/episode_test_sources.py:150:    delta = float((pad[:2] - data.xpos[b][:2]) @ ah)
experiments/episode_test_sources.py:193:    beaker = model.body("task_beaker").id
experiments/episode_test_sources.py:218:        axis = data.xmat[hand].reshape(3, 3)[:, 1]
experiments/episode_test_sources.py:436:    beaker = model.body("task_beaker").id
experiments/episode_test_sources.py:510:        state["trace"].append((t, kind, float(data.xpos[beaker][2]), float(data.xpos[tcp][2]), normals()))
experiments/rest_contacts_test.py:62:    floor = m.geom("floor").id
experiments/gravity_static_check.py:68:        pad_pt = data.xpos[tcp] + data.xmat[tcp].reshape(3, 3) @ pad_c
experiments/safety_filter_roundtrip.py:365:            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jname)
experiments/beaker_hotplate_pose_check.py:5:SUPERSEDED by the reachability map (planned, docs/TODO.md) and BROKEN: it reads the old solid beaker (model.geom("task_beaker_glass")), which the hollow beaker
experiments/beaker_hotplate_pose_check.py:80:          f"{TASK_CFG['arm']} arm, neck mass {sum(model.body_mass[b] for b in range(model.nbody) if model.body(b).name.startswith('neck_')):.3f} kg")
experiments/beaker_hotplate_pose_check.py:92:            err = float(np.linalg.norm(data.body(tcp_body).xpos - pts[name])) if name in pts else float("nan")
experiments/beaker_hotplate_staging_search.py:3:SUPERSEDED by the reachability map (planned, docs/TODO.md) and BROKEN: it reads the old solid beaker (model.geom("task_beaker_glass")), which the hollow beaker
experiments/beaker_hotplate_staging_search.py:111:    W = (data.geom_xmat[g.id].reshape(3, 3) @ V.T).T + data.geom_xpos[g.id]
experiments/beaker_hotplate_staging_search.py:113:    L = (data.xmat[b].reshape(3, 3).T @ (W - data.xpos[b]).T).T
experiments/beaker_hotplate_staging_search.py:293:    bg = model.geom("bench_top")
experiments/beaker_hotplate_staging_search.py:294:    bench_hx, bench_hy = float(model.geom_size[bg.id][0]), float(model.geom_size[bg.id][1])
experiments/beaker_hotplate_staging_search.py:295:    bench_cx, bench_cy = float(model.geom_pos[bg.id][0]), float(model.geom_pos[bg.id][1])
experiments/beaker_hotplate_staging_search.py:304:    beaker_h = 2 * float(model.geom_size[model.geom("task_beaker_glass").id][1])   # cylinder size = (radius, half height)
experiments/beaker_hotplate_staging_search.py:305:    beaker_r = float(model.geom_size[model.geom("task_beaker_glass").id][0])
experiments/stance_select_moved_test.py:53:    hp = fc.model.body_pos[fc.model.body("task_hotplate").id][:2]
experiments/bounce_liftoff_metrics.py:216:    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, SUCCESS_CFG["hotplate_site"])
experiments/nav_lean_sweep.py:76:        return math.degrees(math.acos(max(-1.0, min(1.0, float(data.xmat[base_body][8])))))
experiments/pick_rot_attrib.py:48:R_pad = data.xmat[tcp].reshape(3, 3)
experiments/beaker_hollow_test.py:63:    b = m.body("task_beaker").id
experiments/beaker_hollow_test.py:72:    liquid = m.geom("task_beaker_liquid").id
experiments/beaker_hollow_test.py:74:    check("the solid fallback cylinder is gone", mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "task_beaker_glass") < 0)
experiments/beaker_hollow_test.py:79:    be = me.body("task_beaker").id
experiments/beaker_hollow_test.py:81:          mujoco.mj_name2id(me, mujoco.mjtObj.mjOBJ_GEOM, "task_beaker_liquid") < 0 and abs(float(me.body_mass[be]) - cfg["glass_mass"]) < 1e-9 and 0.040 < me.body_ipos[be][
experiments/beaker_hollow_test.py:121:    top = d.geom_xpos[m.geom("bench_top").id][2] + m.geom_size[m.geom("bench_top").id][2]
experiments/beaker_hollow_test.py:125:    hp = d.geom_xpos[m.geom("task_hotplate_top_plate").id]
experiments/beaker_hollow_test.py:126:    htop = hp[2] + m.geom_size[m.geom("task_hotplate_top_plate").id][2]
experiments/grip_failure_weight_model.py:108:    bb = model.body("task_beaker").id
experiments/grip_failure_weight_model.py:214:        Rb = data.xmat[bb].reshape(3, 3)
experiments/grip_failure_weight_model.py:216:        bpos = data.xpos[bb].copy()
experiments/grip_failure_weight_model.py:353:    true_kg = float(sim.model.body_mass[sim.model.body("task_beaker").id])          # the answer key, read only now
experiments/arm_collision_test.py:85:    probes = {"inside the torso": np.array([0.0, 0.0, 0.55]), "at the right hand": right_hand[0], "inside the camera box": ckl.data.geom_xpos[cam_geom] - torso_c}
experiments/arm_collision_test.py:98:    m2.body_pos[m.body("battery").id] = hand_world - base_pos
experiments/camera_mount_search.py:96:              and int(model.geom_type[g]) != int(mujoco.mjtGeom.mjGEOM_PLANE)]
experiments/camera_mount_search.py:164:    look = data.xpos[body].copy()
experiments/beaker_hotplate_eagle_views.py:5:SUPERSEDED by the reachability map (planned, docs/TODO.md) and BROKEN: it reads the old solid beaker (model.geom("task_beaker_glass")), which the hollow
experiments/full_check_test.py:143:    box_g = fc.model.geom("eagle_cam_body_box").id
experiments/pad_parallel_check.py:30:        Vw = V @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
experiments/pad_parallel_check.py:75:        R = data.xmat[hand].reshape(3, 3)
experiments/pad_parallel_check.py:87:            toward = data.xpos[ob] - data.xpos[b]
experiments/home_pose_test.py:70:            top = float(model.geom_pos[model.geom("bench_top").id][2] + model.geom_size[model.geom("bench_top").id][2])
experiments/stance_two_points.py:82:        plate = m.geom("task_hotplate_top_plate")
experiments/stance_two_points.py:85:        self.plate_top = float(mujoco_data.geom_xpos[plate.id][2] + m.geom_size[plate.id][2])
experiments/stance_two_points.py:86:        bench = m.geom("bench_top")
experiments/stance_two_points.py:87:        self.bench_edge_y = float(mujoco_data.geom_xpos[bench.id][1] + m.geom_size[bench.id][1])       # the front edge on the robot's side (+y)
experiments/stance_check_profile.py:140:    m.body_pos[m.body("task_hotplate").id] = [ctx.hotplate_xy[0], ctx.hotplate_xy[1], ctx.top]
experiments/carry_boundary_replay.py:38:        R = data.xmat[ctx["tcp"]].reshape(3, 3)
experiments/carry_boundary_replay.py:40:        S["bp"].append(data.xpos[ctx["beaker"]].copy())
experiments/carry_boundary_replay.py:41:        S["bR"].append(data.xmat[ctx["beaker"]].copy())
experiments/carry_boundary_replay.py:42:        S["tp"].append(data.xpos[ctx["tcp"]].copy())
experiments/carry_boundary_replay.py:44:        S["pp"].append(data.xpos[ctx["tcp"]] + R @ ctx["pad"])
experiments/pour_scene_test.py:54:    b1, b2 = d.xpos[m.body("task_beaker").id], d.xpos[m.body("task_beaker_2").id]
experiments/pour_scene_test.py:70:    P = np.array([d.xpos[b] for b in balls]) - d.xpos[m.body("task_beaker").id]
experiments/comp_measure.py:58:        S["bk"].append(data.xpos[bb].copy())
experiments/comp_measure.py:59:        S["bq"].append(data.xmat[bb].copy())
experiments/comp_measure.py:60:        S["pad"].append(data.xpos[tcp] + data.xmat[tcp].reshape(3, 3) @ pad_in)
experiments/handoff_weight_measure.py:77:    bb = model.body("task_beaker").id
experiments/handoff_weight_measure.py:122:            rel = Rm.T @ (data.xpos[bb] - p)
experiments/handoff_weight_measure.py:173:    true_kg = float(sim.model.body_mass[sim.model.body("task_beaker").id])          # the answer key, read only now
experiments/home_pose_candidates.py:109:        self.m.body_pos[self.m.body("task_hotplate").id] = [ctx.hotplate_xy[0], ctx.hotplate_xy[1], ctx.top]
experiments/beaker_hotplate_layout_search.py:3:SUPERSEDED by the reachability map (planned, docs/TODO.md) and BROKEN: it reads the old solid beaker (model.geom("task_beaker_glass")), which the hollow beaker
experiments/beaker_hotplate_layout_search.py:156:    beaker, hot = model.body("task_beaker").id, model.body("task_hotplate").id
experiments/beaker_hotplate_layout_search.py:160:               beaker_g=model.geom("task_beaker_glass").id, plate_g=model.geom("task_hotplate_top_plate").id,
experiments/beaker_hotplate_layout_search.py:161:               hotbase_g=model.geom("task_hotplate_base").id, hot_body=hot, static_other=static_other,
experiments/beaker_hotplate_layout_search.py:215:        bg = model.geom("bench_top")
experiments/beaker_hotplate_layout_search.py:216:        self.bench_hx, self.bench_hy = float(model.geom_size[bg.id][0]), float(model.geom_size[bg.id][1])
experiments/beaker_hotplate_layout_search.py:217:        self.beaker_h = 2 * float(model.geom_size[model.geom("task_beaker_glass").id][1])
experiments/beaker_hotplate_layout_search.py:218:        self.beaker_r = float(model.geom_size[model.geom("task_beaker_glass").id][0])
experiments/beaker_hotplate_layout_search.py:220:        self.hot_half_y = float(model.geom_size[model.geom("task_hotplate_base").id][1])
experiments/beaker_hotplate_layout_search.py:221:        self.hot_half_x = float(model.geom_size[model.geom("task_hotplate_base").id][0])
experiments/pick_joint_probe.py:181:    point = data.xpos[tcp] + data.xmat[tcp].reshape(3, 3) @ arm.pad_in_tcp
experiments/layout_sweep.py:48:    gid = m.geom("bench_top").id
experiments/reach_map_build.py:65:    for arr, rows in ((model.body_pos, bodies), (model.body_quat, bodies), (model.geom_pos, geoms), (model.geom_quat, geoms), (model.geom_size, geoms), (model.geom_type, geoms)
experiments/place_over_turn_measure.py:60:        R = data.xmat[ctx["tcp"]].reshape(3, 3)
experiments/place_over_turn_measure.py:61:        rows.append((float(data.time), *data.ctrl[ctx["act"]], *(data.xpos[ctx["tcp"]] + R @ ctx["pad"]), *(R @ ctx["Rp"]).flatten()))
experiments/bounce_diag_extract.py:64:    beaker = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "task_beaker")
experiments/bench_stance_validate.py:30:    edge = float(model.geom_pos[model.geom("bench_top").id][1] + model.geom_size[model.geom("bench_top").id][1])
experiments/bench_stance_validate.py:31:    plate = model.geom("task_hotplate_top_plate")
experiments/bench_stance_validate.py:32:    plate_top = float(data.geom_xpos[plate.id][2] + model.geom_size[plate.id][2])
experiments/solver_offline_check.py:9:      the same formula as MinkArm.step; compare with the forward kinematics of the tool point body (data.xpos, data.xmat).
experiments/j0_held_clearance.py:42:    bk = m.body("task_beaker").id
experiments/bounce_liftoff_run.py:70:        extra["beaker_body"] = model.body("task_beaker").id
experiments/bounce_liftoff_run.py:134:           "beaker_z_m": float(data.xpos[extra["beaker_body"]][2]), "beaker_bench_contacts": _contacts(m, data),
experiments/stance_from_map.py:175:    m.body_pos[m.body("task_hotplate").id] = [ctx.hotplate_xy[0], ctx.hotplate_xy[1], ctx.top]
experiments/stance_from_map.py:300:        m.body_pos[m.body("task_hotplate").id] = [ctx.hotplate_xy[0], ctx.hotplate_xy[1], ctx.top]
experiments/scene_heights_test.py:59:    beaker_geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] == m.body("task_beaker").id and m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH]
experiments/scene_heights_test.py:60:    beaker_top_above_origin = max(float(d.geom_xpos[g][2] + m.geom_size[g][2]) for g in beaker_geoms) - float(d.body("task_beaker").xpos[2])
experiments/grip_pad_contact.py:68:    Rt = data.xmat[tcp].reshape(3, 3)
experiments/grip_pad_contact.py:125:    bb = model.body("task_beaker").id
experiments/grip_pad_contact.py:138:        bpos = data.xpos[bb].copy()
experiments/grip_pad_contact.py:139:        Rb = data.xmat[bb].reshape(3, 3)
experiments/grip_pad_contact.py:266:    res = {"outcome": s["outcome"], "true_total_kg": float(model.body_mass[model.body("task_beaker").id]), "moments": st["moments"], "extent_over_lift_and_carry": st["extent"
```
