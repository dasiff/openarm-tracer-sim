# Loosening the point 4 arm test (joint margin, then approach direction)

Read-only experiment. No code in src/, no config value, no data file changed. Joint margins and approach directions were overridden in memory inside scratch scripts only (`fc.lo`/`fc.hi` replaced by `joint_bounds` with another margin; the pose target rotated by a yaw about the vertical). Scratch scripts in `scratch/` (not in git). Not committed.

Stance: standard scene, heading −100 deg, pedestal 0.10 m, base (0.8698, 1.0722). Left arm. Tolerance 3 mm and 2 deg. The same 25 starts as the earlier report (zombie pose plus 24 draws from the margin-10 bounds, seed 11).

## Step 1: joint margin at point 4 (hand level, straight forward, unchanged)

| margin (deg) | mode | converged of 25 | best position error (mm) | best orientation error (deg) | joints at margin in best solution |
|---|---|---|---|---|---|
| 10 | collisions ON | 0 | — | — | — |
| 10 | collisions OFF | 0 | 45.0 | 9.6 | joints 4, 5, 6 |
| 7 | collisions ON | 0 | — | — | — |
| 7 | collisions OFF | 0 | 37.3 | 8.4 | joints 4, 5, 6 |
| 5 | collisions ON | 0 | — | — | — |
| 5 | collisions OFF | 0 | 32.5 | 8.5 | joints 4, 5, 6 |
| 3 | collisions ON | 0 | — | — | — |
| 3 | collisions OFF | 0 | 27.1 | 8.9 | joints 4, 5, 6 |
| 0 | collisions ON | 0 | — | — | — |
| 0 | collisions OFF | 0 | 18.8 | 9.7 | joints 4, 6 |

Reading: loosening the margin reduces the position error (45 → 19 mm), but no margin gets within tolerance, even with collisions off. At margin 0 the best solution is still 9.7 deg off the required orientation. So the joint margin is not what blocks point 4.

Contacts: not produced. Step 1 asks for the contact list only when a collisions-off solution exists and none with collisions on; no collisions-off solution converged at any margin.

## Step 2: approach direction at point 4 (margin 10, collisions ON)

The hand was kept level (approach and finger-closing axes horizontal) and rotated about the vertical in 5 degree steps. Yaw is the approach direction, measured from the torso's forward axis towards the left.

Bins (`src/grasp_rules.py`, `bin_center` and `make_constraint`): 12 bins, bin k centred at 30k deg from the torso's forward axis, accepting ±15 deg (`GRASP_MAP_CFG["bin_half_deg"]`).

Converging yaws (collisions on): 290, 295, 300, 305, 310, 315, 320, 325, 330 deg. Yaws 335–355 and 0–285 do not converge. Best position errors at the non-converging yaws range from 10 mm to 227 mm (yaw 0, the pipeline's forward direction, gives 45 mm and does not converge).

| yaw (deg) | converged at point 4 | bin (centre window) | grasp point 0 | grasp point 1 | grasp point 2 | grasp point 3 |
|---|---|---|---|---|---|---|
| 290 | yes | 10 (centre 300) | no | no | no | no |
| 295 | yes | 10 | no | no | no | no |
| 300 | yes | 10 | no | no | no | no |
| 305 | yes | 10 | no | no | no | no |
| 310 | yes | 10 | no | no | no | no |
| 315 | yes | 11 (centre 330) | yes | yes | yes | yes |
| 320 | yes | 11 | yes | yes | yes | yes |
| 325 | yes | 11 | yes | yes | yes | yes |
| 330 | yes | 11 | yes | yes | yes | yes |

Only yaws 315 to 330 (bin 11) let both point 4 and the grasp points converge with one shared direction. Yaws 290 to 310 converge at point 4 but not at the grasp points.

## Step 3: the 49 geometry-clean stances, arm path test

The 49 stances: from `docs/reports/reach_inverse_spread_candidates.xlsx`, sheet "candidates standard", rows that pass all six geometry checks and fail only the arm path test (`ik`). Confirmed: 49 rows, all `ik`.

Test = the full check's arm path, run exactly as the full check runs it: the body check at each error stance, then the IK path at each error stance (nominal first, warm-started from the nominal answer), collisions on, left arm.

Variants:
- (i) as now: the pipeline's settings (margin 10, each point's own pose, the skill points at the level forward hand).
- (ii) joint margin 0 deg. Chosen because step 1 showed no margin gives a collision-free solution at point 4, so the only margin left to test is the whole hardware range. Margin 0 is not a usable setting; it is a test of whether the margin is the constraint.
- (iii) free approach direction: all points treated as grasp points (their pose tag removed), with the approach direction forced to bin k at every point, trying all 12 bins. A stance passes if any bin passes all points.

| variant | stances passing the whole path test | first failing point (stance name) for the rest |
|---|---|---|
| (i) as now (margin 10, skill poses) | 0 of 49 | point 0 (nominal) for 25; point 4 (nominal) for 20; point 4 (x+1) for 3; point 4 (pedestal−1) for 1 |
| (ii) margin 0 | 7 of 49 | point 0 (nominal) for 25; point 4 (nominal) for 14; point 4 (y−1), point 4 (pedestal−1), point 4 (x+1) for 1 each |
| (iii) free direction, bins 0–11 | 35 of 49 | 14 stances fail for every bin; the failing bins fail at point 0 (nominal) (165 bin attempts), once at point 0 (pedestal+1) and once at point 0 (heading+1) |

Passing in (ii): stances at (0.7812, 1.0510, −90°, 0.15), (0.7479, 1.0318, −80°, 0.15), (0.7731, 1.0530, −90°, 0.15), (0.7305, 1.0607, −80°, 0.10), (0.7179, 1.0629, −80°, 0.10), (0.7095, 1.0644, −80°, 0.10), (0.9963, 1.0488, −100°, 0.10).

Bins that pass in (iii): most often bins 0 and 11 together (13 stances), bins 10 and 11 (7), bin 11 only (5). 14 stances have no passing bin.

## Corrections to the earlier report

- The earlier report said point 4 fails for this stance. That is true for the one stance diagnosed. Across the 49 stances in variant (i), the first failing point is point 0 (the first grasp point) for 25 of them, and point 4 for 24.
- The earlier step 1 at point 4 found no collision-free solution even at margin 10 with collisions off. Confirmed here: no margin converges with collisions on or off.

## Not done

- **Step 1 contact list:** not produced, because no collisions-off solution converged at any margin, so there was no solution to inspect.
- **Step 3 variant (ii):** the margin was chosen as 0 (the hardware range), because no margin gave a collision-free solution at point 4. A margin between 0 and 10 that is still usable was not tested separately.
- **Step 3 variant (iii):** removing the pose tag from every point changes the path, not just the direction: the skill points are no longer solved at the skill's own pose. This is the intended test (one direction at every point), but it is not the pipeline's path.
- **Right arm:** not tested (the full check and this experiment are left-arm only).

## Commit state

No commits made.

## Files

- Report: `docs/reports/point4_loosening_20261108.md`
- Scratch: `scratch/loosen_lib.py`, `scratch/loosen_run.py` (steps 1 and 2; its step 3 crashed on a tuple-unpacking bug), `scratch/loosen_step3.py` (step 3), results in `scratch/loosen_results.json`.

## Decisions for you

1. Whether to allow a bin-based (free direction) approach for the skill points, which is what variant (iii) shows would pass 35 of the 49 stances.
2. Whether the pick's over and standoff points should use a free approach direction, while the grasp points keep the level forward hand. Variant (iii) shows the grasp points (0 to 3) are the next failing points when the direction is free.
