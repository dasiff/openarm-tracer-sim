# Reach maps: indexing and the shoulder position

Written 2026-11-08 at David's request. Source of truth: `src/reach_map.py` (`ReachMap`, `load_map`, `cubes_from_voxels`) and `src/stance_finder.py` (`_region`, `robot_geometry`, `_torso_z`), `src/stance_select.py` (`shoulder_xy`).

## Which maps

- **Filtered** (use these): `data/reach_map_before_bench/`. The bench is in the maps. Files `left_bin{k}.npz` and `right_bin{k}.npz`, k = 0..11. Cube count for left bin 0: 169,941.
- **Unfiltered**: `data/reach_map/`. No bench. Default build since Nov 5-6. Cube count for left bin 0: 177,488. Do not use for the beaker view.

## Bin index

- Bin k is the approach direction, centred k x 30 deg from the torso's +x axis, turning left (`TASK_CFG["n_bins"] = 12`).
- Bins 4 to 8 point away from the bench for a robot facing it. The pipeline's bench bins are `approach_bins = (9, 10, 11, 0, 1, 2, 3)` (in `TASK_CFG`), but the maps hold all 12.

## What a map holds

- `cubes[ix, iy, iz]`: a bool array of 1 cm cubes, `cube = 0.01` m. A cube is True when all 8 of its 5 mm voxels are reachable.
- `origin`: the position of the corner of cube (0, 0, 0), in the **shoulder frame** of that arm (metres, shoulder frame = origin at the shoulder mount, axes of the torso frame).
- The mapped point is the **pad centre** (not the tool point), so grasp points are compared with pad centres.
- The shoulder frame does not move with the pedestal (the pedestal only moves the shoulder vertically), so one map serves every pedestal height.

## Looking up a point

For a base origin at world (X, Y) with heading h (the torso's +x direction in the world, radians) and pedestal command q, and a point P (world):

1. Shoulder world xy: `shoulder_xy = (X, Y) + R(h) @ (sx, sy)`, where `(sx, sy)` is the shoulder offset in the base frame. This is `src/stance_select.py: shoulder_xy`.
2. Point relative to the shoulder, turned into the shoulder frame: `(rx, ry) = R(-h) (P_xy - shoulder_xy)`. Same rotation as `stance_finder._region`.
3. Height: `zr = P_z - shoulder_z(q)`, where `shoulder_z(q) = base_z + torso_z0 + q + sz`. This is `stance_finder._torso_z` plus the shoulder's own height.
4. Index: `ix = floor((rx - origin_x) / cube)`, likewise `iy`, `iz`. Reached from that bin if `cubes[ix, iy, iz]`.

## Shoulder position: where the numbers come from

- The shoulder is read **from the MuJoCo model** (the openarm link0 body, `openarm_{side}_link0`), as the stance selection does. Nothing here is typed in from link dimensions.
- `stance_finder.robot_geometry(model)` gives the torso-frame offset: left `(0, +0.031, 0.698)`, right `(0, -0.031, 0.698)`, `torso_z0 = 0.3048` (torso above the base origin at pedestal command 0), `base_z = 0.1527` (base origin above the floor, from `robot_specs`).
- Check at pedestal **0.16 m** (`data/reach_heatmap/shoulder_q016.json`, written by `docs/reports/reach_beaker_slider.py --geometry`): the shoulder in the base frame is `(0.0, 0.031, 1.1628)`, the torso in the base frame is `(0, 0, 0.4648)`. Both agree with `robot_geometry` to 0.0 mm.
- Shoulder height above the floor at pedestal 0.16 m: 0.1527 + 1.1628 = **1.3155 m**.

## Heading and the staging stance

- `TASK_CFG["staging_yaw"] = -90 deg` is the heading that faces the bench (`src/task_config.py`).
- The beaker view (`docs/reports/reach_beaker_slider.html`) uses **90 deg**, as David asked. At 90 deg the robot faces away from the bench. Rotate the picture for the bench-facing case (-90 deg).

## Beaker and grasp point

- The beaker is the scene body `task_beaker`, at (1.00, 0.66) m world (matches `TASK_CFG["beaker_xy"]`).
- Grasp point height: bench top 0.96 m + `PICK_PLACE_CFG["grasp_height_frac"]` (0.525) x `BEAKER_CFG["height"]` (0.10 m) = 1.0125 m.
