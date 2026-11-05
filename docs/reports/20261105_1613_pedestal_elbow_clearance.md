# Pedestal height for elbow clearance (geometric study)

Date: 2026-11-05, 16:13. Read-only: no code, model or config changed; nothing pushed.
Written for: David (decides the pedestal height).

## Answer

- **Elbow clearance is not the binding limit.** In the worst case (lowest grasp height 1.007 m, reach and elbow branch taken over all reachable side-grasp poses), the elbow clears the bench top (0.96 m) from a pedestal of about **0.03 m** upward. Clearance: +3 mm at 0.03 m, +23 mm at 0.05 m, +73 mm at 0.10 m.
- **Reach is the binding limit.** A hand within 5° of level (`level_deg`) reaches the beaker's near wall only up to a pedestal of about 0.16 to 0.20 m. The stance report's valid range (0.05 to 0.16 m) and David's M197 finding (0.20 m fails at the pick) both match this.
- **Clearing height (item 4): about 0.03 m.** Horizontal reach at bench-top height there is 0.58 m (±5° tilt), against 0.53 m at 0.10 m. Shoulder to beaker centre at the current stance: 0.548 m horizontal (0.601 m to the beaker centre at grasp height, pedestal 0.10 m).
- **Feasible?** Yes, for elbow clearance across the whole stance range. The pedestal is limited by reach, not by the elbow. 0.15 m (the parked pedestal, M197) is inside the valid range.

## Corrections to the brief and the repo

1. **Base height is 0.1527 m, not 0.22 m.** The XML's base_link spawn is 0.22 m, but the free base settles on its wheels. `stance_finder.robot_geometry()` gives the settled height `base_z = 0.1527 m`, which the stance finder uses. Every world height below uses 0.1527 m. Using 0.22 m would put every vertical number 0.067 m too high.
2. **The eagle camera offset is relative to the torso, not the base.** `robot_specs.EAGLE_MOUNT` has `parent = openarm_body_link0`, and that body sits on the pedestal top. So the camera's base-frame z is 0.3048 + q + 0.9055 = **1.2103 + q** (world 1.3630 + q), not 0.9055 + q.
3. **Three pedestal values are in play.** Stance selection chose 0.10 m (valid run 0.05 to 0.16 m, from `data/stance_select_moved_report.json`). `TASK_CFG["pedestal_q"]` is 0.155 m (reference stance). `BENCH_STANCE_CFG["pedestal_override_m"]` is 0.15 m, which is what the staging parks at (M197).

## 1. Links (left arm, from the scene model; right arm mirrored, not checked)

Measured by forward kinematics of `models/scenes/chemistry_lab_beaker_hotplate.xml` at zero joint angles (read-only; script in `/tmp/claude-1000/-home-vboxuser-projects-openarm-tracer-sim/137e4ff7-1e6a-485b-baff-f71dff4caeb7/scratchpad/fk_geom.py`).

| Quantity | Value (m) | Note |
|---|---|---|
| (a) Shoulder above pedestal top | 0.698 | Joint 1 and joint 2 are both at this height. Joint 2 is the pitch pivot, 0.067 m from joint 1, so the planar model uses joint 2. |
| (b) Upper arm, shoulder (J2) to elbow (J4) | 0.2243 | Varies 0.2200 to 0.2285 with joint 3 (the upper-arm roll). |
| (c) Forearm, elbow (J4) to wrist (J7) | 0.2183 | Constant across joints 5 to 7. |
| (d) Wrist (J7) to pad centre | 0.1885 | Along the hand's approach axis: hand 0.1025 + tool point 0.0835 + pad 0.0025. |

Joint limits used: joint 4 (elbow) 0 to 2.443 rad. At 0 the arm is straight (interior angle 179°), so the maximum shoulder-to-wrist distance is 0.4426 m and the elbow cannot fold past about 40° (minimum 0.1515 m). The straight-arm limit from shoulder to pad is 0.631 m. Joint 2 limits (−3.32 to 0.17 rad) are not applied.

## 2. Scene

- **(a) Bench top surface:** z = **0.960 m** (box centre 0.90, half-height 0.06).
- **(b) Legs and frame:** four legs at x ±2.0, y ±0.7, from z 0 to 0.90 (inside the top's slab). Frame panels at y ±0.78 and x ±2.18, also from z 0 to 0.90. The four frame panels are removed in the open-bench task model (`bench_open`). There is no separate crossbar or shelf below the top; the lowest bench member is the floor (z 0).
- **(c) Beaker:** bottom at z **0.962**, height 0.10 m (cylinder half-height 0.05, outer radius 0.035). Grasp band 45 %: z **1.007**; 60 %: z **1.022**.
- **(d) Hotplate top:** z **1.028** (site `hotplate_top`, 0.96 + 0.068). Hotplate centre at (0.80, 0.66).

Stance (report `chosen`): base origin (0.635, 1.135), yaw −90°. Left shoulder (J2) world xy: (0.7885, 1.1651). Beaker centre (1.00, 0.66): horizontal distance 0.548 m from the shoulder. The near wall of the beaker, where the pads grip, is 0.513 m away.

## 3. Pedestal table (left arm, world z)

Shoulder z = 1.1555 + q (= 0.1527 base + 0.3048 torso + 0.698 shoulder + q). Elbow values are the two solutions of the planar two-link IK with a level hand (tilt 0°) and the pad on the near wall (0.513 m horizontal). "Up" and "down" are the two elbow branches. "Min over reachable poses" is the lowest elbow over every reachable level side-grasp pose at that pedestal height, grasp band 1.007 to 1.022, both branches. The max-reach column is the largest horizontal reach at bench-top height (pad z 0.96) with the elbow at or above 0.96; "level / ±5°".

| q (m) | Shoulder z | Elbow, grasp 1.007 (up / down) | Elbow, grasp 1.022 (up / down) | Min elbow over reachable poses (clearance vs 0.96) | Max reach at 0.96 m, level / ±5° |
|---|---|---|---|---|---|
| 0.10 | 1.256 | 1.197 / 1.062 | 1.214 / 1.059 | 1.031 (+71 mm) | 0.518 / 0.531 m |
| 0.15 | 1.306 | 1.170 / 1.139 | 1.200 / 1.123 | 1.081 (+121 mm) | 0.465 / 0.484 m |
| 0.20 | 1.356 | unreachable | unreachable | 1.131 (+171 mm) | 0.387 / 0.416 m |
| 0.25 | 1.406 | unreachable | unreachable | 1.181 (+221 mm) | none / 0.296 m |
| 0.30 | 1.456 | unreachable | unreachable | 1.231 (+271 mm, only at about 0.16 m reach) | none / none |
| 0.35 | 1.506 | unreachable | unreachable | none reachable | none / none |
| 0.40 | 1.556 | unreachable | unreachable | none reachable | none / none |

"Unreachable" at the beaker means the wrist cannot get to the near wall with a level hand; the same is true at 0.20 m even with ±5° tilt.

**Eagle camera** (50° down tilt, 55° vertical FOV, so the view covers rays 22.5° to 77.5° below horizontal; camera at about (0.635, 1.105) world):

| q (m) | Camera z, world | Bench top visible at horizontal distance from camera |
|---|---|---|
| 0.10 | 1.463 | 0.11 to 1.21 m |
| 0.15 | 1.513 | 0.12 to 1.34 m |
| 0.40 | 1.763 | 0.18 to 1.94 m |

The bench's near edge is 0.305 m from the camera and its far edge 1.905 m. At q = 0.10 the far ~0.7 m of bench is beyond the view; raising the pedestal extends the far limit. The near edge would drop out only near q ≈ 0.97 m, far outside the range. Nothing is lost by raising the pedestal. The beaker (depth 0.445 m, lateral 0.365 m, about 39° off-axis against a horizontal half-FOV of about 43°) stays in view for all q in range.

## 4. Clearing height

Worst case = lowest grasp (1.007 m), smallest elbow over all reachable poses and both branches:

| q (m) | Min elbow z | Clearance vs 0.96 |
|---|---|---|
| 0.03 | 0.963 | +3 mm (first pedestal that clears) |
| 0.05 | 0.983 | +23 mm |
| 0.10 | 1.033 | +73 mm |
| 0.15 | 1.082 | +122 mm |

**Clearing height: about 0.03 m.** Horizontal reach at bench-top height there: 0.578 m (±5°), against 0.531 m at 0.10 m. Shoulder to beaker centre at the current stance: 0.548 m horizontal, 0.601 m to the grasp point at q = 0.10 m.

## 5. Stance range and reach maps

- **Valid range for this stance:** 0.05 to 0.16 m (`overlay_run_q`, from `data/stance_select_moved_report.json`). The clearing height 0.03 m is below the range, so elbow clearance holds across the whole valid range (≥ 23 mm at 0.05 m). 0.15 m (the override) and 0.155 m (TASK_CFG) are both inside the range.
- **Reach cross-check:** at the near wall, the level-hand model reaches up to 0.16 m (with −5° tilt) and fails at 0.20 m. This agrees with the stance report's top of range (0.16 m) and with M197's 0.20 m failure at the pick. The beaker centre fails between 0.12 and 0.15 m, so the near-wall grasp is the one that matches the stance report.
- **Reach maps:** `experiments/reach_map_build.py:76` puts `TASK_CFG["pedestal_q"]` and the staging pose into each bench-included map stamp, and lines 92 and 94 build and write the same values. Changing `TASK_CFG["pedestal_q"]` or the staging pose therefore **stales every bench-included map** (rebuild needed; `--no-bench` skips this). `pedestal_override_m` is not in the stamp: changing it does **not** stale the maps.

## Caveats and open points

- The planar model assumes the pitch plane contains the beaker direction. The upper-arm roll (joint 3) changes the shoulder-to-elbow length by −4 to +4 mm; the joint 1 to joint 2 offset is 67 mm. The reach numbers carry a few millimetres of uncertainty.
- Joint 2 limits are not applied. Joint 4 (elbow) limits are applied.
- The base is assumed level and settled at 0.1527 m. The right arm is not checked (mirrored geometry assumed).
- **Elbow sensitivity disagrees with M197.** M197 reports about 25 mm more elbow clearance from 0.10 to 0.15 m in simulation. The model gives 50 to 77 mm (about 1:1 with pedestal height, as the shoulder rises with it). The sim measured different poses, so this is not a contradiction, but the model overstates the gain.
- The legacy `TASK_CFG["grasp_q"]` is not a level grasp (approach 20° below horizontal, pad at z 1.186), so it was not used to check the reach.
- Not yet verified: a real IK solve at 0.15 m against the near-wall pose. That would confirm the reach limit independently of this planar model.

## Files

- Scratch scripts (not committed): `/tmp/claude-1000/-home-vboxuser-projects-openarm-tracer-sim/137e4ff7-1e6a-485b-baff-f71dff4caeb7/scratchpad/` (`fk_geom.py`, `elbow_plane.py`, `check_grasp_q.py`, `final_checks.py`, `near_wall.py`, `table_wall.py`, `reach_at_clear.py`).
- Sources: `models/robot/combined_robot.xml`, `models/scenes/chemistry_lab_beaker_hotplate.xml`, `models/objects/beaker.xml`, `models/objects/hotplate.xml`, `src/stance_finder.py` (`robot_geometry`), `src/robot_specs.py` (`EAGLE_MOUNT`), `src/task_config.py` (`TASK_CFG`, `BENCH_STANCE_CFG`), `data/stance_select_moved_report.json`, `experiments/reach_map_build.py`.
