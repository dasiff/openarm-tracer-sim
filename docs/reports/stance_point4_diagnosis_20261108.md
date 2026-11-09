# Why the arm solver finds no solution at point 4 (standard scene, one stance)

Read-only diagnosis. No code, config, limit, test or map changed. Scratch scripts are in `scratch/` (not in git). Not committed.

## The stance

- Source: the Gaussian inverse scene run (`experiments/reach_inverse_scene.py`, commit 7387c0d), candidate with heading -100 deg, pedestal 0.10 m, base (0.8698, 1.0722).
- Beaker grasp point (1.00, 0.66, 1.0125) m; hotplate set-down point (0.80, 0.66, 1.0825) m.
- Full check function used: `FullCheck.check` (src/full_check.py), with the pipeline's tagged step points.
- Point 4 is the pick's OVER point, at (1.00, 0.78, 1.16) m.

## Step 1: reproduce

Reproduced exactly: `['ik'] at the stance 'nominal' point 4 (over, [1.0, 0.78, 1.16]) has no collision-free solution`.

- Points in the path (non-place points, index as the solver sees them): 0 to 3 = grasp band (4 heights, pad at the beaker's grasp heights); 4 = over; 5 = standoff; 6 = lift; 7 = back. All eight are `pose: skill`, so each is solved at the skill's own pose (level hand, straight forward).
- Solved from the zombie pose (the first point's start): points 0, 1, 2, 3, 5, 6, 7 pass. **Point 4 fails.** The path stops at point 4, so points 5 to 7 are never reached.
- The arm path test uses **the left arm only** (`SIDE = "left"`). The right arm is not tried. The failing message refers to the left arm.
- Place points are tested separately (place-angle window); the failure occurs before them.

## Step 2: where point 4 is

Shoulder = left shoulder mount (torso frame). Distances are from the shoulder to the pad-centre target. Forward is the torso +x, left is +y, up is +z.

| point | straight-line | forward | left | up |
|---|---|---|---|---|
| 4 (over, fails) | 318 mm | +265 mm | +148 mm | −96 mm |
| 0 (grasp, passes) | 488 mm | +383 mm | +169 mm | −251 mm |

Maximum reach of the left arm (pad centre, from the shoulder):
- Within the solver's joint bounds (the hardware range shrunk by the 10° margin): **745 mm**.
- Within the hardware joint range: **747 mm**.
- Method: 200,000 uniform random joint samples, forward kinematics through `src/arm_kinematics.batch_fk` at the pad offset, distance from the shoulder. The maximum is a sampled estimate.

So point 4 is at about 43% of the maximum reach. Reach is not the limit.

## Step 3: the solver at point 4

Same target, same orientation requirement (the level, straight-forward hand, `FullCheck._pose_target`), same solver call as the pipeline (`solve_ik` with `make_pose_constraint`, 150 iterations, the pipeline's damping and step). Start set: the zombie pose plus 24 seeded random starts, the same count as the pipeline's random restarts.

| mode | solved within tolerance (3 mm, 2 deg) | best position error | best orientation error | notes |
|---|---|---|---|---|
| (a) collisions ON (the pipeline) | 0 of 25 | — | — | nothing converged, so the collision filter had no candidate to judge |
| (b) collisions OFF (limits and orientation ON) | 0 of 25 | 45.0 mm | 9.6 deg | zombie start alone: 144.0 mm, 20.6 deg |

Joint angles at the best (b) solution, against the hardware range and the solver's 10 deg margin:

| joint | deg | hardware range | nearest limit (deg) | flag |
|---|---|---|---|---|
| 1 | −24.1 | −200 to 80 | 104.1 | |
| 2 | −89.5 | −190 to 10 | 99.5 | |
| 3 | 76.4 | −90 to 90 | 13.6 | |
| 4 (elbow) | 130.0 | 0 to 140 | 10.0 | **at the solver's margin** |
| 5 | −80.0 | −90 to 90 | 10.0 | **at the solver's margin** |
| 6 | 35.0 | −45 to 45 | 10.0 | **at the solver's margin** |
| 7 | −47.6 | −90 to 90 | 42.4 | |

Joints 4, 5 and 6 sit exactly on the 10 deg margin that the solver applies (`GRASP_MAP_CFG["joint_margin_deg"] = 10`). The solver stops at its own bound, not at the hardware limit.

## Step 4: contacts

Not applicable: no collision-off solution converged within tolerance, so there was no solution to inspect for contacts.

## Step 5: cause (answered with one extra run, see below)

Position-only test (same solver, same bounds, orientation weight set to zero with `make_pose_constraint(..., rot_weight=0.0)`, zombie start and 24 seeded starts):
- Position error reaches **0.0 mm**.
- The orientation at that solution is **25.7 deg** off the required level, straight-forward hand.

So the cause is the **hand orientation requirement combined with the joint margin**, not reach and not collision. The position can be reached; the required orientation cannot be reached inside the solver's bounds from these starts.

Caveat: 50 random starts (step 6) also fail, so this is not a start-point problem. Whether any joint configuration inside the 10 deg margin satisfies both position and orientation at point 4 is not proved here; the search found none.

## Step 6: 50 starts

The solver call accepts a start array (`solve_ik(chain, Q0, ...)`), so 50 starts were run through the same solver call. The pipeline-level function (`_solve_pose_point`) builds its own starts and does not take one, but the underlying solver does.

| mode | converged within tolerance | succeeded (with collision filter) | failed by collision | worst colliding pair |
|---|---|---|---|---|
| (a) collisions ON | — | 0 of 50 | — | not applicable |
| (b) collisions OFF | 0 of 50 | — | — | not applicable |

Seed 2026, uniform starts within the solver's joint bounds.

## Step 7: the questions

- **Elbow guard (min elbow height from table top):** not applied in this solver path. `ElbowGuard` (src/elbow_guard.py) is used by the control loop only (src/control_loop.py), not by `FullCheck` or its solver. The 0.960 m table-top height appears only in `ELBOW_GUARD_CFG`.
- **Joint limits tighter than hardware:** yes. The solver's bounds are the hardware range shrunk by `GRASP_MAP_CFG["joint_margin_deg"] = 10` (src/grasp_rules.py, `joint_bounds`). This is the limit that stops joints 4, 5 and 6.
- **Hand orientation at point 4 vs point 0:** the same requirement. Both are `pose: skill` points, so both use `FullCheck.R_skill` (level, straight forward). Point 0 passes, point 4 does not, so the difference is the position, not the requirement.

## Not done

- **Step 3(a) and 6 collision results:** not meaningful, because no candidate converged within tolerance, so the collision filter never judged a candidate. The one "admissible" entry in the raw output is the fallback (`Qb[0]`), not a real solution.
- **Step 4 pair list:** not run, for the reason above.
- **Right arm:** not tried by the full check (left only) and not tried here.
- **"Position only" run:** first attempt in `scratch/point4_position_only.py` did not drop the orientation (it re-ran step 3). Corrected by setting `rot_weight=0.0`; the corrected numbers are the ones reported.

## Files

- Report: `docs/reports/stance_point4_diagnosis_20261108.md`
- Scratch: `scratch/point4_diag.py`, `scratch/point4_diag.json`, `scratch/point4_position_only.py`, `scratch/point4_position_only.json`

## Commit state

No commits made. `scratch/` is not added to git.

## Decisions for you

1. Whether the 10 deg joint margin should be relaxed for the solver. It is the limit that holds joints 4, 5 and 6 at point 4, but I haven't tested what a smaller margin would give.
2. Whether the level, straight-forward hand orientation is required at the over point, or whether the over point can use a tilted hand (the grasp points use the approach-bin set, which is more flexible).
