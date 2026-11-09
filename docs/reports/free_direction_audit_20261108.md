# Audit: does everything follow the chosen approach direction? (2026-11-08)

Read-only. No code, config or data changed. Scratch scripts in `scratch/` (not in git). Not committed.

Context: David decided (M275) that the approach direction is free for picks, places and recoveries. The full check tries approach bins 0–11 and reports `pick_bin`; `plan_pick` and recovery turn the robot's forward direction by that bin.

## A. Audit: every use of forward direction, fixed heading or fixed approach axis

Columns: file and line; what it assumes; does it take the approach bin as an input today.

| # | file:line | what it does | assumes | takes bin? |
|---|---|---|---|---|
| A1 | src/mpvi_modules.py:206-209 `forward()` | robot's forward direction in the world | the robot's base x axis | — (helper) |
| A2 | src/mpvi_modules.py:211-220 `pick_approach()` | forward turned by `bin_center(pick_bin)` | the bin's centre is measured from forward, counter-clockwise (grasp_rules.bin_center) | **yes** (ctx `pick_bin`) |
| A3 | src/mpvi_modules.py:231 `plan_pick` | `a = pick_approach()` | a pick approaches along its bin | **yes** |
| A4 | src/mpvi_modules.py:233 `plan_pick` | `R = pad_rotation(a, …)` | hand level, approach `a` | **yes** (via A3) |
| A5 | src/mpvi_modules.py:240 `plan_pick` | `stand = b − a·standoff_m` | standoff along the bin's approach | **yes** |
| A6 | src/mpvi_modules.py:246 `plan_pick` | `back = … − a·retreat_m` | retreat along the bin's approach | **yes** |
| A7 | src/mpvi_modules.py:256 `plan_place` | `a = forward()` | place approaches along the robot's forward | no (θ is applied after, see A8) |
| A8 | src/mpvi_modules.py:262-266 `plan_place` | `R = Rz(θ) @ R`, `a = Rz(θ) @ a` | place approach = forward turned by θ (full-circle θ search) | takes θ, not a bin |
| A9 | src/mpvi_modules.py:271 `plan_place` | `back −= a·retreat_m` (after A8) | back-off along the θ-turned approach | takes θ |
| A10 | src/mpvi_modules.py:318-321 `recover()` | `a = pick_approach() if pick_bin is not None else forward()` | a failed pick backs out along its bin; any other step backs out along forward | pick: **yes**; place: no (should use θ) |
| A11 | src/stance_select.py:210 `step_target` | `fwd = (cos ctx.yaw, sin ctx.yaw)` | the robot's forward at the STAGING yaw (−90°), not the candidate's heading | no |
| A12 | src/stance_select.py:219-222 `step_target` | pick OVER, standoff and back points at `xy − sd·fwd` / `xy − rt·fwd` | the pick's standoff points lie on the forward direction | **no** (mismatch with A5, A6) |
| A13 | src/stance_select.py:227 `step_target` | place back point at `xy − rt·fwd` | place back-off on the forward direction (not θ) | no |
| A14 | src/stance_select.py:74, 86, 90, 184 | `self.yaw = staging_yaw`; bench frame and region use it | every candidate stance faces the staging yaw (−90°) | no |
| A15 | src/stance_select.py:464, 624 | `yaw = ctx.yaw`; the staging is parked at this yaw | the executed heading is always the staging yaw | no |
| A16 | src/full_check.py:239-246 | `R_skill` (level, straight forward, reference rotation) | fixed forward direction; now used only by place points (with θ) | no |
| A17 | src/full_check.py:367 `_pose_target` | `R = self.R_skill` for skill points | straight forward (no longer used for picks, M275) | no |
| A18 | src/full_check.py (IK path, M275) | pick points solved with `make_constraint(…, bin k)` for k in 0–11 | the check's bin is relative to the candidate heading (= staging yaw in the pipeline) | **yes** (the check's own bin loop) |
| A19 | src/task_points.py:55-77 `path_offsets(k, …)` | standoff/over/lift for bin k (`a = radians(30k)`) | bin k as 30k° from forward | **yes** (k) — but only used by the bench overlay (A20) and tests |
| A20 | src/bench_stance.py:211 `build_region` | path points for every bin k (`path_offsets(k, …)`) | the bench overlay uses bin-based pick paths | **yes** — a third convention, differs from A11–A12 |
| A21 | src/staging_source.py:383 | creep-in back along `staging_yaw` | the drive ends on the staging heading | no |
| A22 | src/staging_source.py:437 | `gyaw = staging_yaw` (drive target heading) | the robot drives to a fixed heading | no |
| A23 | src/mpvi_modules.py:110 | staging check against `staging_yaw` | staging is a fixed stance | no |
| A24 | src/stance_select.py:316-333 `carry_blocked_mask` / 490 transit | carry slice and transit use `yaw` (the staging yaw) | the carry slice is seen from the staging heading | no |
| A25 | src/face.py:152-154 (framing, neck/eagle camera) | camera frame `fwd` is the camera's own axis | the camera is fixed to the torso (no neck); its direction does not follow the hand | no (not an approach dependency) |
| A26 | src/bench_stance.py:368 `hand_standoff_m` pre-grasp points | pre-grasp points at a fixed angle `ang` | the grasp side is fixed for the bench's pre-grasp points | check the caller before relying on it |
| A27 | experiments/demo_video.py, experiments/branch_jump_demo.py | demo generation | run the pipeline, so they inherit A1–A10 (no separate approach code found by grep) | inherits |

Experiments that compute their own forward approach (measurement and probe scripts, not the executed skill): experiments/place_over_angle_measure.py, place_over_turn_measure.py, grasp_ik_targets.py, grip_pad_contact.py, carry_boundary_replay.py, gripper_geometry_test.py, pedestal_sweep_measure.py, home_pose_search.py. They do not change the executed skill.

## B. Same path? (the full check's waypoints vs the executed pick)

Computed from the code's formulas, not from a running robot. Full-check points: `stance_select.step_target` (A11–A12). Executed points: `mpvi_modules.plan_pick` (A3–A6), with the bin direction of the robot's heading. Standoff and retreat distances from PICK_PLACE_CFG (0.12 m and 0.05 m). Beaker at (1.00, 0.66) m, grasp point z 1.0125 m.

| case | waypoint | full check (x, y, z) m | executed (x, y, z) m | xy difference (mm) |
|---|---|---|---|---|
| bin 11, robot at staging heading −90° (what the pipeline does) | OVER | (1.000, 0.780, 1.160) | (1.060, 0.764, 1.160) | **62.1** |
| | standoff | (1.000, 0.780, 1.0125) | (1.060, 0.764, 1.0125) | **62.1** |
| | back | (1.000, 0.710, 1.1125) | (1.025, 0.703, 1.1125) | 25.9 |
| | hand approach direction | bin 11 at −90° | bin 11 at −90° | **0.0° (same)** |
| bin 11, candidate heading −100° (only in the analytic scene test; the pipeline never executes it) | OVER | (1.000, 0.780, 1.160) | (1.077, 0.752, 1.160) | 82.1 |
| | back | (1.000, 0.710, 1.1125) | (1.032, 0.698, 1.1125) | 34.2 |
| | hand approach direction | bin 11 at −90° (check's stance yaw) | bin 11 at −100° | 10.0° |
| bin 0, heading −90° | OVER / standoff / back | same | same | **0.0** |

Result: for the pipeline's own case (heading −90°) the approach direction matches. The waypoints do not: the OVER and standoff points the check tests are 62 mm from the points the robot moves through, and the back point 26 mm. For the bin-0 stance they coincide, because bin 0 is forward. The check tests waypoints that are not the executed ones whenever the bin is not 0.

The two data sets come from different functions: the check's points from `stance_select.step_target` (forward), the executed points from `mpvi_modules.plan_pick` (bin), and the bench overlay from `task_points.path_offsets` (bin, A20). Only `path_offsets` follows the bin.

## C. Executed pick at three stances in three bins

Not done. There is no harness that runs the pick at a chosen stance and bin: the pipeline runs the stance it selects, and `pick_error_probe.py` runs the bounce scenario at a fixed stance. Building one and running three simulations is out of the scope of this audit.

What the existing pipeline run shows (data/logs/m275_mpvi_pipeline_test_*.log): the stance (0.775, 1.105) at pedestal 0.12 m, heading −90°, passes the full check in **bin 11** (checked with the full check, `pick_bin = 11`). The IN move then fails: `IN: not reached after 15.0 s: 14.9 mm and 0.7 deg from the waypoint`. Recovery then fails: `HOME: not reached after 24.4 s: 127.6 mm and 29.2 deg`, after a tracking-limit suspension (left joint 4 at +0.255 rad, limit +0.250 rad).

Interpretation (not yet measured): the IN move starts at the executed standoff, which is 62 mm from the point the full check tested for bin 11 (table B). So the DOWN and IN moves follow a path the check did not test. Whether this is the cause of the 14.9 mm error is not established.

Not measured here: standoff point and IN direction per run, hand level error, finger-axis check, grasp success, beaker uprightness, retreat direction, contacts. These need the simulation runs.

## D. Place and recovery: where the direction comes from, and what would change

- **Place:** the direction is already free. `plan_place` (mpvi_modules.py:256-266) approaches along the robot's forward turned by θ, and the full check searches θ over the full circle (`_place_window`, full_check.py:506-512, 0–355° in `place_turn_step_deg` steps). Nothing needs to change for the place's own direction. The place back-off uses the same θ-turned approach (line 271).
- **Place recovery:** `recover()` (mpvi_modules.py:318) uses the forward direction for any step that is not a pick. After a failed place, the hand should back out along the θ-turned approach. Change needed: pass the place's θ to recovery (ctx `place_theta` is already set per segment, line 275).
- **Pick recovery:** already follows the bin (line 318, `pick_approach()`).
- **Pick points in the stance selection (A11–A12):** these are the main change needed. `step_target` should compute the pick's OVER, standoff and back points from each candidate's bin, not from the staging forward, so that the check tests the path the robot executes. The same bin-based definition already exists in `task_points.path_offsets` (A19).
- **Heading:** the pipeline always executes the staging yaw (A14–A15, A21–A24). The analytic scene test uses headings −130° to −50°, which the pipeline never executes. A free direction does not fix this: the candidate heading itself is fixed by the staging.

## Not done

- **C (executed picks in three bins):** no harness; needs a runner that forces a stance and bin, and the simulations (see C).
- **A26 (`hand_standoff_m` pre-grasp points):** the caller is not identified; not checked.
- **Measurements in C** (level error, finger axis, grasp, upright, retreat, contacts): need simulation.
- **Baseline comparison of the M275 change:** the untouched-HEAD run was denied earlier, so the IN failure is not shown to be new.

## Decisions needed

1. **Pick points in the check:** change `stance_select.step_target` so the OVER, standoff and back points follow each candidate's bin (as `task_points.path_offsets` does), so the check tests what the robot executes. This changes the 19-of-49 result.
2. **Heading:** the analytic sweep's headings (−130° to −50°) are not executed by the pipeline, which always uses the staging yaw. Decide whether the stance search should search headings at all, or only the staging yaw.
3. **Place recovery:** should a failed place back out along its θ-turned approach? (Needs the change in recover().)
4. **Simulation harness for C:** allow building the runner (forced stance and bin, three simulations), or accept the pipeline evidence only.

## Files

- Report: `docs/reports/free_direction_audit_20261108.md`
- Scratch: `scratch/audit_bins.py` (the bin of the pipeline stance and the check's pick points), `scratch/audit_B.py` (table B), `scratch/audit_B.json`, logs `data/logs/audit_bins.log`.

## Commit state

No commits made.
