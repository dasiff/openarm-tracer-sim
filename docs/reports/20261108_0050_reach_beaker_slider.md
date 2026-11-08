# Beaker-centred reach slider (filtered maps, left arm, heading 90 deg)

Written for: David.

## What it shows

`docs/reports/reach_beaker_slider.html`: one page, a slider over the pedestal command x. For the scene's beaker at (1.00, 0.66) m, each floor spot (base origin, 5 cm grid) is coloured by how many of the 12 approach bins reach the beaker's grasp point from that spot. Heading 90 deg, left arm only. White = over the bench top. Grey = no bin reaches.

Counts from the filtered maps (`data/reach_map_before_bench`):

| x (pedestal, m) | reachable spots | max bins |
|---|---|---|
| 0.00 | 140 | 5 |
| 0.08 | 115 | 5 |
| 0.16 | 92 | 5 |
| 0.20 | 73 | 4 |
| 0.24 | 50 | 4 |
| 0.28 | 27 | 3 |
| 0.32 and up | 0 | 0 |

The reachable spots at 90 deg are a small half-disc just beyond the bench's +y edge, centred near x = 1.2 m, and they shrink as the pedestal rises. At 90 deg the robot faces +y, away from the bench (see the flag below).

## How it is computed

- Maps: filtered only. Left arm only. Bins 0 to 11.
- Shoulder: read from the MuJoCo model at pedestal 0.16 m (`openarm_left_link0`), as the stance selection does. Base frame: (0.0, 0.031, 1.1628) m. Checked against `robot_geometry`: 0.0 mm difference in x, y and z.
- Transform: the same as `stance_select.shoulder_xy` and `stance_finder._region`. The full formula is in `docs/reach_map_indexing.md`.
- Grasp point: (1.00, 0.66, 1.0125) m. The 1.0125 m is 0.96 m (bench top) + 0.525 x 0.10 m (beaker height), from the pick config.
- x runs over the stance search's grid, 0.00 to 0.40 m in 0.04 m steps.

## Flags for you

1. **Heading 90 deg faces away from the bench.** The task config's staging yaw is -90 deg, which faces the bench (`TASK_CFG["staging_yaw"]`). I used 90 deg as you asked. Tell me if you want -90 (the bench-facing case), or any other angle.
2. **"x minus bench height".** I read x as the pedestal command. The slider also shows x minus the bench top (0.96 m) as a label. The maps are not shifted by it. Tell me if you meant something else.
3. **The page is not run in a browser.** Node is not installed on this machine, so the page's JavaScript was not executed. I checked the same counts with a matplotlib redraw of one frame (pedestal 0.16 m; the beaker cross and the bench outline sit where they should). Please open the page and say if anything is off.
4. **Per-bin count.** Each bin is one approach direction relative to the torso. At one heading (90 deg) the count is a count of directions, from 0 to 12.

## Files

- `docs/reports/reach_beaker_slider.html` (the page; data embedded)
- `docs/reports/reach_beaker_slider.py` (`--geometry` one MuJoCo read; `--html` builds the page without MuJoCo)
- `data/reach_heatmap/shoulder_q016.json` (the shoulder read; in `data/`, git-ignored)
- `docs/reach_map_indexing.md` (how the maps are indexed; where the shoulder comes from)

Nothing under `src/` or the simulation changed. Nothing was pushed. The held-200 g baseline was not regenerated.
