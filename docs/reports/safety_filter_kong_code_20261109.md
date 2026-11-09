# Safety filter study, part A: Kong's geo_kin filter and our wrapper (2026-11-09)

Read-only. No MuJoCo stepping, no change to src/ or config. Part B (clearance measurements in the held 200 g run) is cancelled; a shadow-mode log will replace it later.

Code tree used for the checks: current main, commit 8166ee8 (the pedestal override removal, M284, landed during this session). The kinematic checks used the robot model file `models/scenes/chemistry_lab_beaker_hotplate.xml` for kinematics only.

## 1. Our wrapper: `src/safety_filter.py`

- Added in commit b6cd1c7 (2026-10-05, "Joint-position safety filter helper, round-trip test, branch jump demo, and elbow snap threshold sweep"). Last changed in 98097f4 (label removal). No decision-log row exists for it. `docs/TODO.md:117` names it as the starting point for the RAIL safety filter, which is still open.
- `SafetyFilter(collision_avoidance=True)` builds one geo_kin session through `openarm_teleop.session.make_session(collision_avoidance=..., functional_offset=False, retarget_mode="pose")`. It stores `default_q` for both arms.
- `SafetyFilter.frame(q_left, q_right)` runs the session's `robot_fk_at(q_right=, q_left=)` and builds a `RetargetFrame` from each arm's S, E, W and rotation `R_0_7 @ C_HAND`, where `C_HAND = [[0,0,1],[0,-1,0],[1,0,0]]`. It also sets the base frame from `R_base_body` and `p_base_body`. Input is 7 finite angles per arm (checked by `_arm`).
- `SafetyFilter.solve(frame, q_current_left, q_current_right, reset=False, q_reset=None)` optionally resets the session, then calls `session.solve(frame, engaged=True, q_current_right=, q_current_left=)`. It returns `(q_left, q_right)` as new arrays and raises if either goal is None.
- `SafetyFilter.filter(q_left, q_right, q_current_left=None, q_current_right=None, reset=True)` is `frame` then `solve`. The default `reset=True` makes each call independent. `reset=False` streams, with the previous output or measured joints fed back as the current joints.
- `filter_joints(q_left, q_right, **kwargs)` uses a lazily created shared `SafetyFilter(collision_avoidance=True)`.
- Caveat in the module docstring: the filter does not clip to joint limits, the IK can return a different branch near joint limits even with the filter off, and a solve takes about 0.05 ms (measured below: 0.13 to 0.18 ms median in these runs).

Callers in the repo (`grep` over `*.py`, excluding `deps/`, `legacy/`, `scratch/`):

| file | line | use |
|---|---|---|
| `experiments/safety_filter_roundtrip.py` | 81, 216, 327 | imports `C_HAND, SafetyFilter`; builds the filter on and off; the round-trip test |
| `experiments/branch_jump_demo.py` | 59, 71, 72 | builds the filter on and off |
| `experiments/elbow_snap_readable.py` | 27, 44 | builds the filter on and off |
| `experiments/elbow_snap_sweep.py` | 38, 47, 206, 207 | `LimitedFilter(SafetyFilter)` with `limited=True`; on and off |
| `models/policies/branch_jump_policy.py` | 69 | imports `branch_failures` from the round-trip test |

No caller in `src/`. The teleop path does not use the wrapper (see section 3).

## 2. geo_kin RetargetSession (geo_kin 0.1.0, compiled wheel, licensed)

- The wheel is compiled (`geo_kin.abi3.so`). The Python-visible docstrings are thin. The `__init__` text signature in the wheel is the shared G1 one (`robot="g1", hand=None, control_rate_hz=None, elbow_filter_cutoff_hz=None, collision_avoidance=True, torso=True, torso_mag=None, preprocess=None, base_alignment_mode=None, geometry_config_path=None, shoulder_width_scale=1.0, retarget_mode=None, limited=False, debug=False, robot_fk=True, torso_position_filter_cutoff_hz=None, functional_offset=True, torso_ik_limited=True, arm_ik_limited=True, offcentered_wrist_mounting=True, mobile_base=False, mobile_base_*, enable_stability_clamp=True, stability_radius=0.12, geometry_config=None, spec_dir=None, torso_upperarm_distances=None, torso_radius_scale=1.0`). The wrapper calls `make_session` (`openarm_teleop/session.py`, `resolve_session(robot="openarm", backend="licensed", **config)`).
- Compiled robots: `g1`, `openarm`. Hands: `inspire`, `psyonic`.
- Public methods (from the class):
  - `reset(q_init_right=None, q_init_left=None)`
  - `solve(frame, engaged=True, q_current_right=None, q_current_left=None, R_world_mount=None, p_world_mount=None, q_current_right_hand=None, q_current_left_hand=None)`. Returns a `RetargetOutput` with `q_goal_left` and `q_goal_right`.
  - `robot_fk_at(q_right=None, q_left=None)`: returns `{'left': {S, E, W, T, se, ew, R_0_7}, 'right': {...}, 'R_base_body', 'p_base_body'}`.
  - `joint_limits(side)`: returns `(lower, upper)`, each 7 values.
  - `sew_capsules()`: the post-XPBD filtered capsules of the last solve. Docstring: empty when the filter is off or nothing has been solved.
  - `hand_fingertips_at(side, q_hand)`, `hand_joint_names`, `hand_tip_coupling(...)`: hand-related, not used for OpenArm (no hand).
  - Attributes: `robot`, `hand`, `default_q`, `default_q_torso`, `R_mocap_world`, `p_mocap_world`. The Vega-only `R_mocap_base`, `p_mocap_base` do not exist for openarm.
- **Constructor keywords accepted for openarm** (probed with `make_session`): `collision_avoidance=True|False`, `stability_radius`, `spec_dir`, `control_rate_hz`, `limited=True`, `debug=True`, `retarget_mode='tcp'`. The wrapper's options (`functional_offset=False`, `retarget_mode='pose'`) also pass.
- **Rejected for openarm:** `torso_radius_scale` ("OpenArm supports its fixed-base reference configuration only"), `geometry_config` (same), `elbow_filter_cutoff_hz` (same), `shoulder_width_scale` (same). `torso_upperarm_distances` needs a tuple, not a list. The error text on `collision_avoidance` ("only valid for robot='g1'") is stale: the openarm session accepts it.
- **Does any argument set capsule radii, margins, iteration counts, weights, or extra obstacles?** No. The radii are fixed inside the compiled code (the 8 capsules above). No margin, iteration count, weight, or obstacle argument was found. `stability_radius` is accepted for openarm; its effect is not documented and was not measured. The `geometry_config_path` error says "its geometry configuration is compiled in".
- **Does `collision_avoidance=False` give plain SEW-Mimic with no filtering?** The docstring says `collision_avoidance=False` "exists only for tests that need a control". With it off the round trip returns the input to about 4e-6 rad (section 3). The capsule list is documented as empty when the filter is off. I did not separately dump `sew_capsules()` with the filter off.

## 3. Round trip (20 random left configurations, right arm at the zombie pose)

Setup: left joints drawn uniformly within the session's left limits (seed 20261109). Right arm at `home_q_right` from `src/task_config.py` (the zombie pose: -22.598, 14.709, -15.531, 126.122, 3.746, -27.308, -13.018 deg). The filter sees the robot's own FK, so this is the wrapper's round trip, not a comparison to MuJoCo geometry.

Earlier runs in this session used the session's default right pose, not the zombie pose; those numbers are not reported. The final numbers below are from `scratchpad/kong_final.py` (outside the repo).

Table (left arm; filter-off right-arm error was 3.5e-06 rad in every config):

| config | off, left max diff (rad) | changed? | on vs off max (rad) | on vs input max (rad) |
|---|---|---|---|---|
| 1 | 3.41e-06 | no | 0 | 0 |
| 2 | 5.07e-01 | no | 0 | 5.07e-01 |
| 3 | 3.97e-06 | no | 0 | 0 |
| 4 | 3.08e-06 | no | 0 | 0 |
| 5 | 3.65e-06 | no | 0 | 0 |
| 6 | 2.53e-05 | no | 0 | 3.0e-05 |
| 7 | 3.66e-06 | no | 0 | 0 |
| 8 | 3.04e-06 | no | 0 | 0 |
| 9 | 3.69e-06 | no | 0 | 0 |
| 10 | 1.59e+00 | no | 0 | 1.59 |
| 11 | 3.47e-06 | no | 0 | 0 |
| 12 | 3.66e-06 | **yes** | 0.2206 | 0.2206 |
| 13 | 3.64e-06 | no | 0 | 0 |
| 14 | 3.43e-06 | no | 0 | 0 |
| 15 | 1.45e-03 | no | 0 | 1.45e-03 |
| 16 | 3.93e-06 | **yes** | 0.4828 | 0.4828 |
| 17 | 3.42e-06 | no | 0 | 0 |
| 18 | 3.61e-06 | no | 0 | 0 |
| 19 | 3.96e-06 | no | 0 | 0 |
| 20 | 5.88e-06 | no | 0 | 1e-05 |

Summary: with the filter off, the round trip is exact to about 4e-6 rad in 17 of 20 configurations. Three configurations show branch jumps: configs 2 (0.51 rad) and 10 (1.59 rad) are large, config 15 (0.0015 rad) is small. The IK can return a different valid branch near joint limits even with the filter off, as the module docstring warns. With the filter on, the output differed from the filter-off output in 2 of 20 configurations (configs 12 and 16, by 0.22 and 0.48 rad). In every other configuration the filter changed nothing beyond the 1e-3 rad threshold.

Why the check could not confirm the solver's own geometry: the solver's shoulder, elbow, wrist and tool points do not match our MuJoCo joint anchors under any rigid transform tried (section 5). The round trip above does not need that match: it only checks the solver's internal consistency.

## 4. Timing (1000 `filter()` calls per setting, 1000 random left configs, zombie right)

`filter()` includes the frame build (FK) and the solve, with `reset=True` on every call, which is the wrapper's default.

| setting | median (ms) | 95th percentile (ms) |
|---|---|---|
| collision_avoidance=False | 0.167 | 0.445 |
| collision_avoidance=True | 0.128 | 0.368 |

Caveat: the two settings ran in the same process with the same random sequence, and an earlier run in this session gave different values (off 0.080 / 0.181 ms, on 0.154 / 0.403 ms). The spread between runs is about 2x, so treat these as order-of-magnitude numbers, not a ranking of the two settings.

## 5. Capsule placement at the default pose

Method. The solver's keypoints (S, E, W, T, from `robot_fk_at`) were fitted to MuJoCo points (S to the joint 1 anchor, E to the joint 4 anchor shifted 0.0315 m along its axis, W to the joint 7 anchor, T to `openarm_left_hand_tcp`) with a single rigid transform over 30 random left configurations. Fit residuals (max over configurations):

| keypoint | residual (m) |
|---|---|
| S | 0.049 (a constant offset) |
| E | 0.017 |
| W | 0.019 |
| T | 0.095 |

So the solver's geometry and our MuJoCo joint geometry agree to about 1 to 2 cm at E and W, about 5 cm at S, and about 9 cm at the tool point. The capsule placements below carry that uncertainty. The tool point is the largest discrepancy: the solver's tool point is 0.100 m from the wrist (constant), while our `openarm_left_hand_tcp` is 0.186 m from the joint 7 anchor at the default pose.

Capsule end points (solver frame, default pose; the arm base is at z = 0.698 m above the robot base origin). "Nearest" is the closest MuJoCo joint anchor or body origin after the fit, with the distance.

| capsule | end A (solver point) | end A nearest (distance) | end B (solver point) | end B nearest (distance) | radius (m) |
|---|---|---|---|---|---|
| torso | (0, 0, 0.1) | body openarm_right_link0 (0.106) | (0, 0, -0.6) | body pedestal_top (0.104) | 0.060 |
| shoulder | (0, +0.1535, 0) | joint openarm_left_joint2 (0.039) | (0, -0.1535, 0) | joint openarm_right_joint2 (0.037) | 0.075 |
| L_upper | (0, +0.1535, 0) | joint openarm_left_joint2 (0.039) | (0, +0.1535, -0.22) | joint openarm_left_joint4 (0.019) | 0.050 |
| L_lower | (0, +0.1535, -0.22) | joint openarm_left_joint4 (0.019) | (0.216, +0.1535, -0.22) | joint openarm_left_joint7 (0.017) | 0.050 |
| L_hand | (0.216, +0.1535, -0.22) | joint openarm_left_joint7 (0.017) | (0.316, +0.1535, -0.22) | body openarm_left_left_finger (0.012) | 0.080 |
| R_upper | (0, -0.1535, 0) | joint openarm_right_joint2 (0.037) | (-0.082, -0.209, -0.196) | joint openarm_right_joint4 (0.016) | 0.050 |
| R_lower | (-0.082, -0.209, -0.196) | joint openarm_right_joint4 (0.016) | (0.116, -0.132, -0.158) | joint openarm_right_joint7 (0.017) | 0.050 |
| R_hand | (0.116, -0.132, -0.158) | joint openarm_right_joint7 (0.017) | (0.216, -0.142, -0.163) | body openarm_right_hand_tcp (0.078) | 0.080 |

Pose used: left arm at the session default, right arm at the zombie pose (the run's right arm, so the right-arm rows are not the textbook default-pose geometry and are approximate; the left-arm rows are at the default pose).

Answers:
- **Does L_hand cover the fingers?** Only partly. The L_hand capsule runs from the solver wrist to a tool point 0.100 m beyond it, but our finger geometry reaches 0.186 m from the joint 7 anchor. Of the finger surface points, 90% (left, finger 2) and 88% (left, finger 1) lie within 0.080 m of the capsule's axis, with a maximum of 0.088 m. So the hand capsule is short and slightly narrow for our fingers.
- **Any capsule for the pedestal column, neck, cameras, base or battery?** No. The torso capsule (radius 0.060 m, z from 0.10 to -0.60 m in the solver base frame) is the only capsule near the base; its ends sit at `openarm_right_link0` and `pedestal_top` at about 0.10 m, so it is not a fit to the pedestal column. There is no neck, camera, base or battery capsule in the wheel. The model has no neck, camera or battery collision geoms (checked by name in `models/robot/combined_robot.xml`), and the base geoms are not represented.

## Not verified

- Decision-log row for the wrapper: none found (`docs/mpvi_decisions.md` has no row; the wrapper's only record is commit b6cd1c7 and `docs/TODO.md:117`).
- `stability_radius` and `spec_dir` are accepted for openarm; their effect was not measured.
- `sew_capsules()` with `collision_avoidance=False` was not dumped; the "empty when off" statement is from the docstring.
- Right-arm capsule placement is approximate (the right-arm solver points were not re-fitted separately).
- The collision filter's behaviour against the beaker, the bench, or any other object was not measured (Part B cancelled).

## Cleanup status

- Background monitor and background wait stopped (TaskStop on both).
- Scratch worktree (`scratchpad/wt_safety`) and `experiments/safety_filter_clearance_study.py` were NOT removed: the Bash call for the worktree removal and the script deletion was denied by the permission system. They still exist on disk. The worktree is a detached checkout of 8166ee8 (git still lists it until removed). The untested script is untracked and was not committed.
