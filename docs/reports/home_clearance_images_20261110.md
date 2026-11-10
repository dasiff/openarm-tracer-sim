# What fails home_clearance: numbers and pictures (2026-11-10)

Written for: David. Read-only: no change to src/, experiments/ or config. Scripts: `scratch/home_clearance_images/`. Pictures and numbers: `docs/reports/home_clearance_images_20261110/` (index: `INDEX.md`).

## 1. What the check is (from the code)

- **Function**: `StanceChecks.home_clearance(stance, look=0.3)`, `src/stance_checks.py` line 226. Called by the full check at each pedestal height it tests (`src/full_check.py` lines 715-722).
- **What it measures**: the smallest distance (mujoco `mj_geomDistance`, looked up to 0.3 m) from any **moving arm part** to any **scene geom**.
  - Arm parts (`src/stance_checks.py` line 198): every geom of the two OpenArm arms (group 3, names starting `openarm_`, except `link0`). This includes the links, the hands and the fingers. The hand and finger subset (line 199) is reported separately as "hands" but the check uses "arms".
  - Scene geoms (line 200): every geom with contact on, not a plane, not part of the robot's own base (`under_root`). That is the bench top and legs, the hotplate, the beaker, and every other object on the bench (for example `micro_arm`, `micro_body`, `micro_stage`, the microscope; and the `task_hotplate_*` parts).
  - Negative values are penetration.
- **Home pose used**: `TASK_CFG["home_q_left"]` = (22.598, −14.709, 15.531, 126.122, −3.746, 27.308, 13.018) deg, and `home_q_right` mirrored (`src/task_config.py` lines 94-95, row M080). Fingers open (`finger_open`). Both arms at home.
- **Robot pose**: the base at the stance (x, y, heading), the pedestal at the stance's pedestal height. The full check tests the drawn pedestal and two others: q0, q0 + 0.02 and q0 − 0.02 (`height_offsets`, `src/task_config.py` line 513); each height must pass.
- **Limit**: 40 mm, `STANCE_REQUIREMENTS["home_min_clearance"]` = 0.040 m (`src/task_config.py` line 260, row M087). The check fails when the smallest distance is under the limit.
- **Failure reason text (now)**: `at pedestal {h:.3f} the arms at home are {mm} mm from {pair} (the limit is 40 mm)`, where `pair` is "arm geom vs scene geom" (`src/full_check.py` line 722). So the reason already names the part, the object, and the gap, at the height that failed. The sampler log prints only the failing check's name (`home_clearance`), and it keeps the reason text in memory (cut to 300 characters), so the log does not show the part or the gap. See DECISIONS.

## 2. Stances shown

Draws 3 and 9 are the two highest-scoring draws of the Revision 2 run that failed `home_clearance` (scores 3.090 and 3.071). They were rebuilt exactly from the run's seed (3623316766) with the logged check results, and the rebuilt x, y and pedestal match the log (`replay matches logged: True`). No new draw was made. The overlay stance is the one that passed at the staging layout before the switch.

Heading is −90° for all three (held, as in the run).

| stance | x | y | heading | pedestal | base distance to bench top (m; negative: over the bench) | score (A8) | closest pair (at drawn pedestal) | gap (mm) | home_clearance at drawn pedestal | min over the 3 check heights (mm) |
|---|---|---|---|---|---|---|---|---|---|---|
| draw 3 | 0.7337 | 0.2486 | −90 | 0.1315 | −0.551 | 3.090 | openarm_right_link5_collision vs micro_arm | **−47.3** | FAIL | −48.6 (at 0.1115) |
| draw 9 | 0.7049 | 0.3217 | −90 | 0.1999 | −0.478 | 3.071 | openarm_right_link7_collision vs micro_body | **−43.8** | FAIL | −63.8 (at 0.2199) |
| overlay (passed before) | 0.775 | 1.105 | −90 | 0.12 | +0.305 | not scored | openarm_left_left_finger_collision vs task_hotplate_base | +94.0 | PASS | +77.1 (at 0.10) |

Base distance: the signed distance in the floor plane from the base origin to the bench top footprint (bench top centre (0, 0, 0.9), half sizes 2.2 × 0.8 m): positive outside, negative over the top. Both failing stances put the base 0.48 to 0.55 m inside the bench top's footprint, so the robot's own base stands over the bench.

Three closest pairs at the drawn pedestal:

| stance | 1 | 2 | 3 |
|---|---|---|---|
| draw 3 | link5 vs micro_arm, −47.3 mm | link6 vs micro_arm, −20.6 mm | link6 vs micro_stage, −16.5 mm |
| draw 9 | link7 vs micro_body, −43.8 mm | link7 vs micro_arm, −39.5 mm | link5 vs micro_stage, −15.7 mm |
| overlay | left finger vs hotplate base, +94.0 mm | left finger vs hotplate top plate, +95.8 mm | right finger vs hotplate base, +101.4 mm |

All three closest pairs of draws 3 and 9 are under 40 mm (two of them are penetrations). For the overlay stance all are over 40 mm.

## 3. Pictures

Same size (640 × 480) and the same camera settings for every stance. Side and top are free cameras (side: azimuth 90°, elevation 0°; top: azimuth 90°, elevation −89°, both looking at the stance); eagle is the robot's own torso camera (`eagle_cam`, no marker). The cyan line joins the closest pair's two points (from `mj_geomDistance`), labelled with the gap in mm.

Files (in `docs/reports/home_clearance_images_20261110/`):
- draw3_side.png, draw3_top.png, draw3_eagle.png
- draw9_side.png, draw9_top.png, draw9_eagle.png
- overlay_pass_side.png, overlay_pass_top.png, overlay_pass_eagle.png

Checks on the pictures: the look-at point of each free view projects within 2 px of the image centre (the projection uses the renderer's camera pose). In draw 3's top view the robot stands over the bench with the arms at home, and the marker sits on the right arm's link next to the microscope. The overlay's top view shows the robot in front of the bench edge with the marker at the left finger and the hotplate.

Bug found and fixed during the work (not in the check): the first pictures rendered the simulation's data, while the stance was placed in the check's own copy (`StanceChecks.d`), so the robot was drawn at the origin. The renders now use the check's data. The numbers were never affected (they use the check's data).

## 4. Inner reach limit r_in (for Revision 3, measured now)

From the filtered reach maps `data/reach_map_before_bench` (left arm, all 12 approach bins), by `scratch/polar_draw/inner_limit.py`:
- r_in (smallest shoulder-to-cell-centre distance, cells with at least one reachable bin) = **0.0497 m**, one cell at shoulder-frame (0.025, −0.035, 0.025).
- Quantiles of the same distances: 0.01 % = 0.0513 m, 0.1 % = 0.0585 m, 1 % = 0.0923 m; 35 cells lie under 0.10 m.
- Histogram of reachable cells (5 cm bins, cells): 0.00-0.05: 1; 0.05-0.10: 696; 0.10-0.15: 4775; 0.15-0.20: 12790; 0.20-0.25: 24853; 0.25-0.30: 37891; 0.30-0.35: 52351; 0.35-0.40: 66785; 0.40-0.45: 79137; 0.45-0.50: 81733; 0.50-0.55: 82483; 0.55-0.60: 76765; 0.60-0.65: 62732; 0.65-0.70: 39898; 0.70-0.75: 12203 (total 635093 cells).

## NOT DONE

- The picture of draw 9 was rendered by the same code; I looked at draw 3 (side and top) and the overlay top view, not at every file.
- Memory: a check of free memory runs before each render (stop under 2 GB); it did not trigger. No value was logged per render.
- The sampler log does not print the failure reason, so the part and gap of each failed draw are not in the run's log (the numbers here come from the check's data, rebuilt).

## DECISIONS NEEDED

1. The sampler's log line could print the failure reason (the part, object and gap are already in the reason text). A small change in `src/stance_select.py` (the draw log line) and the record. Not made.
2. The sampler has no bench or obstacle term (by design: "no other filters before scoring"). Both failing draws put the base over the bench top, and the A8 score is high there, so most of the batch's high-weight stances are unusable. Should the sampler reject stances whose base is over the bench top before the full check (a filter), or keep the rule as written?
3. r_in: the literal minimum (0.0497 m) is one cell 5 cm from the shoulder, so it constrains almost nothing (35 cells under 0.10 m). Use the literal value, or a quantile (for example the 0.1 % point, 0.0585 m)?
