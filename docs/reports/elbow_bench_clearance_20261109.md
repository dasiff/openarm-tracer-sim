# Elbow and bench clearance, home pose, and the elbow guard (2026-11-09)

Read-only investigation. No simulation was run (the model was loaded and its kinematics evaluated, no physics step). No change to src/, config, tests or docs; no commits; no push. This file is untracked. Helper scripts: `/home/vboxuser/projects/oc_scripts/elbow_heights_20261109.py` (elbow heights, output `elbow_heights_20261109.json`) and `elbow_bench_report_20261109.py` (this file).

Written for: David (the elbow guard idea and the stance rule for bases under the bench top). Nothing is decided; no fix is proposed here.

## 1. The elbow guard

- What it clamps: the 7-joint arm target of each physics step's row in the control loop. Height checked: the world z of the origin of `openarm_<side>_joint4` (the elbow joint centre, `xanchor`), computed from the measured base pose and joints 1 to 3 and the commanded pedestal of that row (`src/elbow_guard.py:48-65`, `elbow_z`).
- How it clamps: if the target puts the elbow centre below the minimum, the target is moved along the segment from the measured joints toward the target to the largest fraction that keeps the elbow at or above the minimum (bisection, `BISECT_STEPS = 30`, `src/elbow_guard.py:16`, `:67-100`, `clamp`). If the measured elbow is already below the minimum, the measured angles are held.
- When it acts: in the control loop only, on every physics step's arm row of every tick (`src/control_loop.py:458-463`, `ctrl_block`, the call to `elbow_guard.clamp`; built at `src/control_loop.py:609`). It is not used by the stance search, the full check or the stance checks (`docs/reports/guard_off_20261108.md`, section 3).
- Settings (`src/task_config.py:747-762`): `enabled` **False** (line 752). `min_joint_z_m` = 0.960 + 0.1188 + 0.012 + 0.014 = **1.1048 m** above the floor (line 755): bench top 0.960 m (`table_top_z_m`), elbow housing radius 0.1188 m (`housing_m`), allowance 0.012 m for 1 degree of tilt and 5 mm of sag, margin 0.014 m (`margin_m`, David, Nov 5). The minimum is 144.8 mm above the bench top.
- Why it was switched off: M278 (2026-11-08, David): "guard off while the approach angle work is finished; revisit". The config comment (`src/task_config.py:749-751`) says that with the guard on the pick's DOWN stalls at the grasp height, because the elbow stays above the minimum only by clamping joint 4 and the tracking suspension trips (`data/logs/in_probe_20261108.log`). The clamp prints its message once per arm (at the first clamp, `src/elbow_guard.py`, `clamp`); the in-probe log has one such message, for the left arm, after the 66.05 s line. Later clamps are counted in `hits` only, so the log does not give a count of clamped ticks.
- What was seen with it on and off (saved reports and logs):

| run | guard | HOME | replay | elbow contacts (touched) | source |
|---|---|---|---|---|---|
| staging pick, pedestal 0.12 (in_probe) | on | DOWN stalls above the grasp height (M278) | not run | not recorded | `data/logs/in_probe_20261108.log`, M278 |
| staging pick, pedestal 0.12 | off | place HOME fails: largest joint error 0.267 rad after 9 s (joint 6 at 12.0 deg, home 27.3 deg); joint limits and tracking not the cause | replay mismatch at tick 11145 (75.71 s) | none (0 N on every part) | `docs/reports/guard_off_20261108.md` section 5 |
| full episode (mpvi test, guard off) | off | place HOME fails (as above) | fails at tick 11145 | none | `docs/reports/replay_home_close_20261108.md` table |
| full episode, guard on, hooks on (rep c1, c2) | on | ok (the run reaches the place) | PASS, 14458 ticks | none | `docs/reports/replay_home_close_20261108.md` |

The guard-off replay mismatch is not run-to-run randomness (two guard-off runs give identical numbers); the guard changes the trajectory, and the replay passes with it on (`docs/reports/replay_home_close_20261108.md`, section 1). The guard does not change the stance check's home pose; it only clamps the commanded targets in the run.

## 2. Where the elbows go (home pose, kinematics only)

Stance placed by the check's own placement (`StanceChecks.place`: base at the staging stance x 0.775, y 1.105, heading -90 deg, both arms at home, pedestal set). The bench top is at z 0.960 m (`src/task_config.py` scene; the bench top geom, `models/scenes/chemistry_lab_beaker_hotplate.xml:52`).

| pedestal (m) | left elbow z (m) | left - bench top (mm) | right elbow z (m) | right - bench top (mm) | left elbow housing, lowest possible point (elbow z - 0.1188) - bench top (mm) | right, same (mm) |
|---|---|---|---|---|---|---|
| 0.05 | 1.0052 | 45.2 | 1.0129 | 52.9 | -73.6 | -65.9 |
| 0.10 | 1.0552 | 95.2 | 1.0629 | 102.9 | -23.6 | -15.9 |
| 0.15 | 1.1052 | 145.2 | 1.1129 | 152.9 | 26.4 | 34.1 |
| 0.20 | 1.1552 | 195.2 | 1.1629 | 202.9 | 76.4 | 84.1 |
| 0.25 | 1.2052 | 245.2 | 1.2129 | 252.9 | 126.4 | 134.1 |

- The housing number is a bound: the elbow collision geom lies inside a sphere of radius 0.1188 m about the joint centre (`docs/reports/20261105_1626_elbow_bench_worst_case.md`), so its lowest point is at or above joint z minus 0.1188 m. The helper's own sphere bound per geom (geom centre minus its bounding radius) gives the same lowest part, the elbow housing (`link4`), at every pedestal, but its numbers are not the same bound (for example -46.3 mm at 0.05 m) and are not used here.
- Lowest part: the elbow housing (`link4`) at every pedestal (helper bound). The forearm (`link5`) and upper arm (`link3`) are higher at home (helper bounds, not in the table).
- Closest part to the bench in saved records: see section 3 (clearance records name the geom).
- Note on the guard's own check: 1.1048 m is 144.8 mm above the bench top, reached at pedestal 0.15 m on the right arm and about 0.15 m on the left (report `docs/reports/20261105_1927_elbow_guard_implementation.md`, section 2: the minimum pedestal is 0.1496 m on the left).

## 3. Recorded contacts between arm parts and the bench or hotplate

### 3a. Physical contacts in runs (elbow monitor, `src/elbow_clearance_log.py`)

Eight `elbow_clearance.json` files; they hold two distinct runs (the guard-off staging pick and its copies; the guard-on and sag-off replay runs). Every part's `touched` flag is False and every `max_force_N` is 0 N. No contact of an arm part with the bench or the hotplate was recorded in any run. Phases use the guard-off timeline (`docs/reports/guard_off_20261108.md`, section 5 table): OVER 66.05-67.65 s, DOWN 67.65-71.37, IN 71.38-74.92, CLOSE 74.93-79.66, UP 79.67-82.50, place OVER 82.51-84.57, place DOWN 84.57-87.17, OPEN 87.17-88.57, BACK 88.57-89.78, HOME 89.79-102.66.

| run | arm | lowest elbow z (m), time | housing: min gap (mm), time | upper arm: min gap (mm), time | forearm: min gap (mm), time | touched (all parts) | phase at the times |
|---|---|---|---|---|---|---|---|
| guard off, staging pick (guardoff_probe, guardoff_mpvi_run, rep a1, a2, trace: identical numbers) | left | 1.0927 (72.7 s, IN) | 175.6 (85.6 s) | 246.7 (84.8 s) | 57.5 (74.5 s) | False | |
| guard off, staging pick (guardoff_probe, guardoff_mpvi_run, rep a1, a2, trace: identical numbers) | right | 1.1216 (49.3 s, before the pick (not tagged)) | not measured | not measured | 239.0 (65.7 s) | False | |
| guard on, replay runs (rep c1, c2; 14458 ticks) | left | 1.1041 (79.1 s, CLOSE) | not measured | not measured | 226.2 (67.3 s) | False | |
| guard on, replay runs (rep c1, c2; 14458 ticks) | right | 1.1216 (49.3 s, before the pick (not tagged)) | not measured | not measured | 239.0 (65.7 s) | False | |
| guard off, sag trim off (rep sagoff; 14762 ticks) | left | 1.0954 (73.0 s, IN) | 186.4 (85.8 s) | 254.8 (85.8 s) | 59.1 (85.8 s) | False | |
| guard off, sag trim off (rep sagoff; 14762 ticks) | right | 1.1216 (49.3 s, before the pick (not tagged)) | not measured | not measured | 239.0 (65.7 s) | False | |

### 3b. Stance-level contacts and near-contacts (static checks, not runs)

Source: `data/logs/complete_full_standard_20261108_043214.log` and `..._moved_...` (the full-check reasons, Nov 8). These are the stance checks' own measurements at the stance (no motion); they are not run contacts.

| scene | stance-level event | pedestal (m) | count | base y range (m) |
|---|---|---|---|---|
| moved | body contact: openarm_body_link0_collision vs bench_top | 0.1 | 6 | 0.305 to 0.820 |
| moved | body contact: openarm_body_link0_collision vs bench_top | 0.15 | 9 | 0.305 to 0.820 |
| moved | body contact: openarm_body_link0_collision vs bench_top | 0.2 | 9 | 0.305 to 0.820 |
| moved | home clearance: openarm_left_link4 vs bench_top (12 mm, limit 40) | 0.05 | 9 | 0.281 to 0.764 |
| standard | body contact: openarm_body_link0_collision vs bench_top | 0.1 | 7 | 0.286 to 0.820 |
| standard | body contact: openarm_body_link0_collision vs bench_top | 0.15 | 10 | 0.286 to 0.820 |
| standard | body contact: openarm_body_link0_collision vs bench_top | 0.2 | 10 | 0.286 to 0.820 |
| standard | home clearance: openarm_left_link4 vs bench_top (12 mm, limit 40) | 0.05 | 11 | 0.249 to 0.764 |

Both are at base y 0.25 to 0.82 m, the bench's footprint (the bench top runs y -0.8 to 0.8 m: `models/scenes/chemistry_lab_beaker_hotplate.xml:52`); the robot's own base stands over or at the bench edge.

### 3c. Closest pairs in the spread sheet (clearance first failures, Nov 8, `docs/reports/reach_inverse_spread_candidates.xlsx`)

| scene | closest arm part | furniture | pedestal (m) | candidates | home clearance measured (mm, min over heights) |
|---|---|---|---|---|---|
| moved | left_finger | task_hotplate_base | 0.05 | 3 | 17.8 to 33.5 |
| moved | left_finger | bench_top | 0.05 | 7 | 17.8 to 33.5 |
| moved | left_finger | task_hotplate_base | 0.1 | 13 | 26.0 to 58.0 |
| moved | left_finger | task_hotplate_top_plate | 0.1 | 1 | 26.0 to 58.0 |
| moved | left_finger | task_hotplate_top_plate | 0.15 | 2 | 85.1 to 89.0 |
| moved | left_finger | task_hotplate_base | 0.15 | 5 | 85.1 to 89.0 |
| moved | left_finger | task_hotplate_top_plate | 0.2 | 6 | 119.6 to 123.8 |
| moved | link5 | bench_top | 0.05 | 4 | 22.6 to 23.9 |
| moved | right_finger | bench_top | 0.05 | 26 | -34.7 to 35.1 |
| moved | right_finger | task_hotplate_base | 0.05 | 34 | -34.7 to 35.1 |
| moved | right_finger | task_hotplate_top_plate | 0.05 | 4 | -34.7 to 35.1 |
| moved | right_finger | task_hotplate_base | 0.1 | 14 | 15.3 to 51.5 |
| moved | right_finger | task_hotplate_top_plate | 0.1 | 24 | 15.3 to 51.5 |
| moved | right_finger | task_hotplate_top_plate | 0.15 | 22 | 65.3 to 86.0 |
| moved | right_finger | task_hotplate_base | 0.15 | 3 | 65.3 to 86.0 |
| moved | right_finger | task_hotplate_top_plate | 0.2 | 8 | 115.3 to 123.0 |
| standard | left_finger | task_hotplate_base | 0.05 | 3 | 17.8 to 33.5 |
| standard | left_finger | bench_top | 0.05 | 7 | 17.8 to 33.5 |
| standard | left_finger | task_hotplate_base | 0.1 | 13 | 26.0 to 58.0 |
| standard | left_finger | task_hotplate_top_plate | 0.1 | 1 | 26.0 to 58.0 |
| standard | left_finger | task_hotplate_top_plate | 0.15 | 2 | 85.1 to 89.0 |
| standard | left_finger | task_hotplate_base | 0.15 | 5 | 85.1 to 89.0 |
| standard | left_finger | task_hotplate_top_plate | 0.2 | 6 | 119.6 to 123.8 |
| standard | link5 | bench_top | 0.05 | 4 | 22.6 to 23.9 |
| standard | right_finger | bench_top | 0.05 | 26 | -34.7 to 35.1 |
| standard | right_finger | task_hotplate_base | 0.05 | 34 | -34.7 to 35.1 |
| standard | right_finger | task_hotplate_top_plate | 0.05 | 4 | -34.7 to 35.1 |
| standard | right_finger | task_hotplate_base | 0.1 | 14 | 15.3 to 51.5 |
| standard | right_finger | task_hotplate_top_plate | 0.1 | 24 | 15.3 to 51.5 |
| standard | right_finger | task_hotplate_top_plate | 0.15 | 22 | 65.3 to 86.0 |
| standard | right_finger | task_hotplate_base | 0.15 | 3 | 65.3 to 86.0 |
| standard | right_finger | task_hotplate_top_plate | 0.2 | 8 | 115.3 to 123.0 |

### 3d. Contact monitor per phase in pipeline and mock runs

Not found: the only per-part monitor is the elbow monitor above; its logs carry no phase tags (phases are matched to time only). No mock-run contact counts per phase were found in `data/logs` or `docs/reports`. Beaker-to-bench contacts (`contact task_beaker_bottom / bench:bench_top`) appear in the held-beaker logs; they are not arm contacts and are not counted here.

## 4. What the stance check tests along the reach path

- `_admissible` (`src/full_check.py:386-400`), applied to every IK solution of every point on every path (discrete waypoints; no swept volume between them): (a) the arm does not collide with itself, the other arm, the torso or the pedestal (`ArmCollisionChecker.check`, `src/arm_collision.py:99-118`; checker built with `bench=b`, which is None unless `bench_in_contact_check` is on: `src/full_check.py:355`, `src/task_config.py:519`, default False); (b) the elbow-flip continuity between consecutive points (`ik_flip_rad` 1.0 rad, `src/task_config.py`); (c) the hand and fingers are at least `hand_clearance` = 0.01 m from the world (`src/task_config.py:520`, check at `src/full_check.py:394-395`); (d) the other left-arm links are strictly more than `arm_clearance` = 0.0 m from the world (`src/task_config.py:521`, same check).
- Which links: `PathClearance.arm` (`src/full_check.py:92`) = every collision geom of `openarm_left_*` except link0 and the hand (so link1 to link7, the elbow housing `link4` and the forearm `link5` included). `PathClearance.hand` (`:91`) = hand and fingers. `PathClearance.world` (`:93`) = every collidable non-robot geom (bench, hotplate and its plates, objects), excluding the robot and the beaker (the beaker is checked in the placed-beaker clearance instead).
- Distances: `arm_distances` (`src/full_check.py:125-137`) reads the minimum distance of each set to the world at each joint vector (look-up radius 0.3 m).
- Whole robot, home only: `body_distance` (`src/full_check.py:138-145`) puts the left arm at home and measures the whole robot (both arms, camera box) to the world; limit `body_clearance` 0.0 m (`src/task_config.py:522`). It is not run along the path.
- Home clearance (`src/stance_checks.py:226`): arms at home to the scene, limit 40 mm (`src/task_config.py:260`); placed-beaker clearance 40 mm (`:261`).
- So along the reach path, the elbow housing and forearm are tested against the bench and hotplate only as distances at each IK solution, at a margin of 0 m (`arm_clearance`); the hand and fingers at 1 cm. The bench is not in the arm's self-collision filter (`bench=None`).

## 5. The home pose

- Definition: `src/task_config.py:94-95` (`home_q_left`, `home_q_right`, row M080, applied 2026-10-20 from the home pose search's rank 1; `docs/reports/20261020_0345_apply_rank1_suite_result.md`). It is one fixed joint vector per arm, not per stance. The per-stance inputs are the base pose and the pedestal only.
- Variation: `home_variation_rad` = 0 (`src/task_config.py:96`) and `home_variation_seed` 0 (`:98`). `home_targets` (`src/task_beaker_hotplate.py:366-384`) adds a uniform draw from a seeded generator when the spread is non-zero; the draw is per episode and identical for both the check and the staging.
| reader | file, line | what it uses |
|---|---|---|
| stance checks: home clearance, placed beaker, view, occlusion, body | `src/stance_checks.py:186` (`self.home = home_targets(...)`), `:205` (`place`) | the home joints, via `place()` every check and every view/occlusion call |
| full check: home arms in the path clearance | `src/full_check.py:98`, `:103` (`home_left` for `body_distance`) | the home joints |
| full check: zombie start of every skill path IK | `src/full_check.py:282` (`zombie_q`), `:425-440` (starts for `_solve_pose_point`) | `home_q_left` |
| full check: approach frame of the place hand | `src/full_check.py:266` | `home_q_left` (the hand's zombie rotation) |
| full check: other arm (checker) | `src/full_check.py:355` | `home_q_right` |
| staging: ARMS step (move to home) and the park | `src/staging_source.py:28-29, :434, :938` | `home_targets`, the same function |
| stance selection output | `src/stance_select.py:427` | `arm_pose_home` record |
| experiments (not the pipeline) | `experiments/home_pose_candidates.py:70, 98-99` (sets TASK_CFG home), `sag_layer_test.py`, `mock_policy_test.py`, others | the home joints |

How the parked pose and the check's home pose meet: both call `home_targets` with the same `TASK_CFG` values (`src/task_beaker_hotplate.py:366-384`): the check through `StanceChecks.__init__` (`src/stance_checks.py:186`) and `place()`, the run through `src/staging_source.py:434`. A changed home pose (the joint values in `src/task_config.py:94-95`, or `home_targets`) would reach every reader in the table above. The elbow guard does not change the home pose: it clamps commanded targets during the run only.

- Neck framing: no neck exists in the current model (M153: the eagle camera is fixed to the torso). The neck spread and framing checks existed until 2026-10-22 (`docs/reports/20261019_1820_stance_requirements_and_rank1_finder.md`, `docs/mpvi_decisions.md` row M087 superseded by M153). No current reader.

## 6. What the home pose does to view and occlusion

Source: `docs/reports/20261020_0200_arm_coverage.md` (the home pose variants A, rank 1, rank 2 at the same stances; each variant is a joint vector, so this measures the home pose itself, not the stance).

| home pose | stance | arms (% of image) | hands (%) | beaker hidden (%) | hotplate hidden (%) | framing slack (%) |
|---|---|---|---|---|---|---|
| A (before M080) | rank 2's | 11.7 | 3.6 | 23.3 | 21.8 | 12.9 |
| A (before M080) | rank 1's finder | 12.0 | 5.4 | 0.1 | 2.8 | 13.3 |
| rank 1 (now, M080) | rank 2's | 10.1 | 3.6 | 0.0 | 5.8 | 12.7 |
| rank 1 (now, M080) | rank 1's finder | 10.2 | 5.3 | 0.0 | 0.1 | 12.7 |
| rank 2 | rank 2's | 10.3 | 3.6 | 5.1 | 8.1 | 13.1 |
| rank 2 | rank 1's finder | 9.9 | 4.7 | 0.0 | 0.0 | 17.2 |

- Arms cover about 10 to 12 % of the image whatever the pose; the hands 3.6 to 6 %. The biggest effect is on the hotplate and beaker occlusion (up to 23.3 % of the beaker and 21.8 % of the hotplate hidden at rank 2's stance with variant A, 0 % with rank 1).
- The October 19 finder reports change the stance (base and pedestal) with the home pose fixed (`docs/reports/20261019_2245_finder_widest_range_rank1.md`, table: pose A at 0.655/0.995/0.11 m has hotplate hidden 26.8 % and framing 9.8 %; rank 1's stance 0.1 % and 12.7 %). They are not home-pose changes.
- Not found: no saved report measures the elbow or forearm in the eagle image separately from the arms as a whole.

## 7. Needs a run

- Elbow and forearm clearance to the bench along the path, in motion, for stances with the base over the bench footprint (the saved records are static, at pedestal 0.05 and 0.10 only).
- Contact monitor with phase tags for the ARMS step (the right elbow's minimum is at 49.3 s, before the pick, phase not tagged).
- Whether a changed home pose keeps the stance check and the run consistent (a home-pose change reaches all readers in section 5; no run has been made).
- True geometry of the lowest elbow point (the 0.1188 m sphere is a bound; the helper's per-geom bounds are rough).

## 8. Not found

- No recorded contact of any arm part with the bench or hotplate in any run (all 8 elbow monitor files: touched False, 0 N).
- No per-phase contact counts in the pipeline or mock logs.
- No record names the closest geom at the parked home pose during a run (the elbow monitor names parts, not geoms, and gives only minimum gaps).
- No neck framing reader in the current code.

## 9. Not done

- No simulation run (as asked). The elbow heights are kinematics only; the lowest-part numbers are bounds.
- The per-part min gaps are per part for the whole run, not per phase; phases were matched by time only.

