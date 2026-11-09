# Safety filter facts (2026-11-09)

Read-only. No MuJoCo stepping (model built and kinematics only), no change to src/ or config. Code: current main 8166ee8. Models built with `src/task_beaker_hotplate.load_task(MOVED, cameras=True)` (the simulator's own model build, with the battery, eagle camera and pedestal added at load time).

## 1. Cleanup

- `git worktree remove --force` of the scratch worktree and `rm experiments/safety_filter_clearance_study.py`: **not done**. The Bash call was denied by the permission system again (this is the third denial; the approval in this task was not enough for the tool). Both still exist. The worktree is detached at 8166ee8 and still listed by git. Run `git worktree remove --force <scratchpad>/wt_safety && git worktree prune` and `rm experiments/safety_filter_clearance_study.py` yourself, or approve a narrower permission.

## 2. Robot body collision shapes (compiled task model)

The previous report was wrong on this. It checked only the static XML (`models/robot/combined_robot.xml`). The battery and the eagle camera are added at load time by `src/simulator.py` (`add_battery`, and the eagle camera body), so they are in the compiled model but not in that XML.

| body | geom | type | size (half-sizes or radius, m) | contype | conaffinity | collides with an arm geom |
|---|---|---|---|---|---|---|
| base_link | base_link_geom | mesh | 0.0797, 0.3081, 0.3552 | 1 | 1 | yes |
| base_link | contact_wheel_right, contact_wheel_left, contact_caster_right_front, contact_caster_left_front, contact_caster_right_rear, contact_caster_left_rear | sphere | r 0.01 | 2 | 0 | no (contact-only, for the floor) |
| right_link | right_link_geom | mesh | 0.0191, 0.0625, 0.0625 | 0 | 0 | no |
| left_Link | left_Link_geom | mesh | 0.0191, 0.0625, 0.0625 | 0 | 0 | no |
| right_front_link, right_rear_link, left_front_link, left_rear_link | *_link_geom | mesh | 0.043, 0.0392, 0.0592 | 0 | 0 | no |
| pedestal_base (fixed) | pedestal_base_collision | box | 0.06, 0.06, 0.1524 | 1 | 1 | yes |
| pedestal_base (fixed) | pedestal_base_visual | box | same | 0 | 0 | no |
| pedestal_top (moving, lift) | pedestal_top_collision | box | 0.07, 0.07, 0.01 | 1 | 1 | yes |
| pedestal_top (moving, lift) | pedestal_top_visual, pedestal_column_visual | box | same | 0 | 0 | no |
| openarm_body_link0 (torso) | openarm_body_link0_collision | mesh | 0.03, 0.03, 0.375 | 1 | 1 | yes |
| openarm_body_link0 (torso) | openarm_body_link0_visual_0 to _5 | mesh | | 0 | 0 | no |
| eagle_cam_body (camera, on torso) | eagle_cam_body_box | box | 0.015, 0.045, 0.0125 | 1 | 1 | yes |
| eagle_cam_body (camera, on torso) | face_head | (visual, not collision) | 0.075, 0.09, 0.08 | 0 | 0 | no |
| battery | battery_geom | box | 0.192, 0.104, 0.122 | 1 | 1 | yes |
| neck | (no body) | | | | | no neck in the model |

"Collides with an arm geom" = MuJoCo's rule (contype of one geom AND conaffinity of the other) against every collision-capable left or right arm geom.

Document claims:
- **Wrong (previous report):** "no battery or camera collision geoms". The battery box and the eagle camera box are collision geoms in the compiled model, and both collide with the arm.
- **Correct:** `docs/design_plan.md` line 34 says the battery has collisions on. The model confirms it.
- **Wrong or superseded:** the neck. The model has no neck body or geom. The design plan marks the neck sections superseded (line 10, M153, the fixed eagle camera). I did not find a "neck framing 5 mm clear" claim in `docs/design_plan.md`; the neck text there is about 10 to 40 mm clearances and was removed. Treat that claim as coming from superseded text.
- **Pedestal:** both fixed and moving parts are collision geoms (the pedestal column itself is visual only; the collisions are the base box and the top box).
- **Cameras:** only the eagle camera has a collision box. The other cameras are rendering cameras with no geoms.

## 3. Branch jumps: 20 poses, avoidance off, current = own pose, reset=False

Poses: the same 20 left configurations as the last report (seed 20261109, reproduced by replaying the same random sequence: 30 calibration draws, then 20 round-trip draws). Right arm at `home_q_right`. Poses were run in order with reset=False, so the session carries state from one pose to the next. A second pass that resets to each pose first gave identical numbers.

"Jumped" = max joint difference > 0.05 rad. Distance to limit = to the nearer joint limit (rad).

| pose | max joint diff (rad) | jumped before (last report) | jumps now | joint | input angle (rad) | output angle (rad) | distance to limit, input / output (rad) |
|---|---|---|---|---|---|---|---|
| 1 | 0.00000 | no | no | | | | |
| 2 | 0.50709 | yes | **yes** | 3 | -1.057 | -1.565 | 0.513 / **0.006** |
| 3 | 0.00000 | no | no | | | | |
| 4 | 0.00000 | no | no | | | | |
| 5 | 0.00000 | no | no | | | | |
| 6 | 0.00003 | no | no | | | | |
| 7 | 0.00000 | no | no | | | | |
| 8 | 0.00000 | no | no | | | | |
| 9 | 0.00000 | no | no | | | | |
| 10 | 1.58823 | yes | **yes** | 5 | -0.902 | 0.686 | 0.669 / 0.885 |
| 11 | 0.00000 | no | no | | | | |
| 12 | 0.00000 | no | no | | | | |
| 13 | 0.00000 | no | no | | | | |
| 14 | 0.00000 | no | no | | | | |
| 15 | 0.00145 | no | no | 1 | 1.363 | 1.364 | 0.033 / 0.032 |
| 16 | 0.00000 | no | no | | | | |
| 17 | 0.00000 | no | no | | | | |
| 18 | 0.00000 | no | no | | | | |
| 19 | 0.00000 | no | no | | | | |
| 20 | 0.00001 | no | no | | | | |

Do the jumping configurations still jump? Yes, both. Pose 2 drives joint 3 onto its lower limit (output 0.006 rad from it, the input was 0.513 rad away). Pose 10 flips joint 5 from the lower side to the upper side (-0.902 to 0.686 rad). Pose 15 is 1.45e-3 rad, not a jump. The IK returns a different valid branch at these poses even with the filter off.

## 4. Capsule sizing around the solver's key points

### 4a. Frame fit (200 poses per arm, other arm at zombie pose)

Solver key points from `session.robot_fk_at`. MuJoCo points for the same joint angles: S = joint 1 anchor, E = joint 4 anchor minus 0.0315 m along its axis, W = joint 7 anchor, T = `openarm_<side>_hand_tcp`. One rigid transform per arm (Kabsch over all 200 poses x 4 points). The residual is measured after the fit.

| arm | key point | mean residual (m) | max residual (m) | residual std xyz (m) | share of residual explained by joint angles (R², linear) |
|---|---|---|---|---|---|
| left | shoulder S | 0.0469 | 0.0469 | 0, 0, 0 | 0.01 |
| left | elbow E | 0.0132 | 0.0172 | 0.001, 0.002, 0.001 | 0.03 |
| left | wrist W | 0.0128 | 0.0209 | 0.002, 0.003, 0.003 | 0.13 |
| left | tool T | 0.0876 | 0.1014 | 0.050, 0.048, 0.049 | 0.13 |
| right | shoulder S | 0.0463 | 0.0463 | 0, 0, 0 | (undefined: zero variance) |
| right | elbow E | 0.0141 | 0.0160 | 0.001, 0.001, 0.001 | 0.17 |
| right | wrist W | 0.0140 | 0.0178 | 0.002, 0.002, 0.002 | 0.19 |
| right | tool T | 0.0872 | 0.1008 | 0.050, 0.048, 0.050 | 0.12 |

Verdict: **mostly a constant frame offset, with the tool point pose-dependent.**
- Shoulder: constant offset (zero variance over poses). It comes from the point definition (joint 1 anchor vs the solver's joint 1-3 axis intersection), not from a different frame.
- Elbow and wrist: a constant offset of about 1.3 cm, with a small pose-dependent part (std 1 to 3 mm).
- Tool point: pose-dependent. The solver's tool point is always 0.100 m from the wrist, while the MuJoCo tool point is 0.18 to 0.20 m from the joint 7 anchor depending on pose. No single rigid transform can fit that, so the tool point of the solver is not `openarm_<side>_hand_tcp`.
- Link lengths: the solver's shoulder-to-elbow length is fixed at 0.2200 m. The MuJoCo joint-1-to-joint-4 distance is 0.258 ± 0.018 m (not rigid for this point pair, because joint 1 is not the solver's shoulder point).

### 4b. Radii (left arm; fit applied to solver key points; max over all 200 poses; mesh vertices of the collision geoms)

| part | segment (solver points) | our radius (m) | geom that sets it (pose) | Kong's radius (m) | ours / Kong |
|---|---|---|---|---|---|
| upper arm (incl. shoulder mount link0) | S to E | 0.1295 | openarm_left_link0_collision (pose 1) | 0.050 (upper) | 2.6 |
| upper arm without shoulder mount (links 1 to 3) | S to E | 0.0725 | openarm_left_link1_collision (pose 159) | 0.050 (upper) | 1.45 |
| forearm (links 4, 5) | E to W | 0.0694 | openarm_left_link4_collision (pose 61) | 0.050 (lower) | 1.39 |
| wrist and palm (links 6, 7, hand; fingers excluded) | W to T | 0.1045 | openarm_left_hand_collision (pose 7) | 0.080 (hand) | 1.31 |

The fit error (section 4a: 1.3 cm at the elbow and wrist, 9 cm at the tool point) adds to these numbers; the wrist-to-tool segment is the most affected.

### 4c. Fingers (zombie pose; both gripper extremes, joint range 0 to 0.044 m)

Capsule along the hand axis from the solver wrist toward the solver tool point, covering both fingers at both gripper positions:

| start (m from solver wrist, along hand axis) | end (m) | radius (m) | Kong hand |
|---|---|---|---|
| 0.100 | 0.195 | 0.081 | 0.080 |

Both finger extremes give the same along-axis range; the radius is the larger of the two. The finger geometry starts at the solver tool point (0.100 m from the wrist).

### 4d. Torso and pedestal column (vertical capsules, enclosing the collision geometry)

| body | pedestal position | radius (m) | z range (m, world) | geom that sets it | Kong |
|---|---|---|---|---|---|
| torso (openarm_body_link0 collision) | lowest (joint 0.000) | 0.0416 | 0.533 to 1.283 | body_link0 mesh | torso 0.060 |
| torso | highest (joint 0.418) | 0.0416 | 0.951 to 1.701 | body_link0 mesh | torso 0.060 |
| pedestal column (base + top collision) | lowest | 0.0990 | 0.220 to 0.525 | pedestal_top_collision corners | none |
| pedestal column | highest | 0.0990 | 0.220 to 0.943 | pedestal_top_collision corners | none |

The torso capsule is about 0.7 times Kong's torso radius, so it is narrower than Kong's, and the pedestal column has no Kong equivalent. The column radius is set by the corners of the 0.07 m moving top box.

Comparison with Kong: every arm radius is larger than Kong's (1.3 to 2.6 times). The fingers match (0.081 against 0.080). The torso is narrower than Kong's.

## 5. World model used by stance selection

Stance selection reads the stance-time world, `StaticTaskSource` in `src/world_model.py` (its docstring: the same entries as the simulator source, read from the configuration and the scene heights, no simulation step). I did not trace every caller of it.

| object | shape type | dimensions | pose source | height correct? |
|---|---|---|---|---|
| beaker | cylinder (footprint) | radius 0.035 m (outer diameter 0.070 m); height 0.100 m | config (`beaker_xy`, with the moved shift); z = scene beaker_z 0.962 m | yes: model beaker bottom at z 0.962, wall top at 1.062 (0.100 m above the bottom) |
| hotplate | box (footprint) | half-size 0.09 m x 0.09 m (matches the base box); height 0.068 m above the bench | config (`hotplate_xy`, moved shift); z = scene plate_top 1.028 m | yes: model plate top at z 1.028; bench top at 0.96 |
| bench | surface root (top_z only; no shape or dimensions stored) | model top: box half-sizes 2.2 x 0.8 x 0.06 m (4.4 m x 1.6 m, centre z 0.90) | config (`staging_xy`); top_z = scene bench_top 0.960 m | yes: model bench top at z 0.960 |

Shapes: the beaker is the only cylinder, the hotplate is a box footprint, and the bench has no shape stored. There are no other objects in this world model (the lab's other furniture, the robot and the pedestal are not entries). The Nov 8 fix ("static world reports the scene heights, was 0") is in place: the static heights equal the heights read from the loaded model (`task_heights` = `scene_heights`).

## Not done

- Cleanup (worktree removal, study script deletion): not done, Bash denied (section 1).
- The safety-filter study script and worktree were not removed, so nothing else in the repo changed.
- Frame fit: the shoulder point was not redefined as the joint 1-3 axis intersection; the fit uses the joint 1 anchor, as in the earlier report, so the 4.7 cm shoulder offset is a definition difference, not a measured frame error.
- Pose sample for the branch jumps: the 20 poses are the earlier ones; no new seed.
- Capsule radii were not checked against an independent MuJoCo distance computation; they use the fitted transform, so they inherit its residuals.
- The world model's bench footprint (the stance selection's use of the bench shape) and other world-model callers were not traced.
