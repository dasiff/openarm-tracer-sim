# The skill's OVER move: path planner (read-only, 2026-11-10)

Written for: David. Source: `src/mpvi_modules.py` (`plan_pick`, `move`, `step_inner`, `move_duration`), `src/mink_arm.py` (`MinkArm.step`, `joint_arc`), `src/task_points.py` (`pick_waypoints`), `src/task_config.py` (`PICK_PLACE_CFG`, `MINK_CFG`).

## What the OVER move is
- **Waypoint.** `pick_waypoints` (src/task_points.py): the OVER point is the pad centre at the standoff point (the grasp point moved `standoff_m` = 0.12 m back along the approach direction of the chosen approach bin k), at height `surface + clearance_m` = 0.20 m above the bench top. The approach bin k is `pick_bin_index()`, the bin the full check passed for this stance (M275). The hand is level along the approach.
- **Start.** The pad pose now, measured (`arm.pad_pose(data)`), because `restart_from_commanded` is False. The arm was at the zombie pose at the start of the pick (the staging's arms-at-home state).
- **Duration.** `move_duration`: the largest of the distance at `speed_m_s` (0.12 m/s), the turn at `turn_rad_s` (0.5 rad/s), `min_move_s` (1.0 s), and `min_jerk_peak_factor` (2.0) times the largest joint arc of a dry run divided by `max_joint_speed` (0.5 rad/s). The dry run is `MinkArm.joint_arc`: the IK along the straight path, from the current command, with no speed limit. It changes no state.

## How the pad path is made
1. **Reference.** `step_inner` gives the reference pad position `p_ref = p0 + (target - p0) * min_jerk(u)`, a straight line in Cartesian space with a minimum-jerk profile over the duration. The orientation is a slerp from R0 to the level pad rotation.
2. **Per-tick IK.** Each control tick calls `arm.step(data, p_ref, R_ref)`: one step of the differential IK (mink) from the **current command** `q_cmd` toward the pad reference. The solver's costs are: frame task on the pad (position 1.0, orientation 0.5), a posture task toward the home pose (0.01, which chooses the elbow), joint limits with a 10 deg margin (`joint_margin_rad`), collision avoidance against the bench and the hotplate (gain 0.85, detect 5 cm, minimum 8 mm for furniture), and a per-joint speed limit of 0.5 rad/s. The next command is `q_cmd`, which the sag trim and the controller then act on.
3. **Reach test.** The move ends when the measured pad is within `tol_m` 3 mm and `tol_rad` 2 deg of the waypoint and settled (`settle_dwell_s` 0.2 s). Timeout: `timeout_factor` (3.0) times the duration plus 6 s. The OVER move of this run was given up at 30.8 s with 51.4 mm error (the log).
4. **Blending.** A move that reaches the end within `blend_radius_m` (20 mm) of the waypoint hands over to the next move at the pad's current speed.

## What the check tests instead
The check (`FullCheck._solve_pose_point`) solves the IK for the waypoint only: from the zombie pose for the first stop of a path, with 150 iterations, and accepts the solution if it reaches the waypoint within 3 mm and 2 deg, and passes `_admissible` (no collision, hand 10 mm clear, arm 0 m clear). It does not solve the path between stops, and it has no posture task and no joint speed limit.

So the two solutions differ by construction: the executor follows a straight pad line with a per-tick IK from wherever the arm is, the check solves the endpoint once from the zombie pose.

## Key decisions that matter for the divergence
- The executor's solution branch is set by the posture task (toward home) and by the joint margins, as the arm moves; the check's branch is set by its zombie start and its own cost. They can pick different elbows for the same endpoint (see `over_path_divergence_20261110.md`).
- The dry run (`joint_arc`) predicts the joint motion; the executor's real per-tick IK can differ from it.
