# Gaussian reach model: Part A (maps) and Part B (fit) — targets missed, stopped

Written for: David (approved task: fit a Gaussian model of the unfiltered reach maps, left arm).

**Outcome: the specified model and the one alternative both miss two of the three targets. Per the task, I stopped. Part C (analytic inverse, `src/reach_model.py` stance finder, scene tests) was not started.**

## Part A: the maps (read directly from the .npz files, no stance-code lookup)

- Source: `data/reach_map/left_bin0..11.npz` (the unfiltered maps, no bench).
- Each file stores 5 mm voxels (packed bits). A 1 cm cube is reachable when all 8 of its voxels are (the maps' cube rule).
- Each bin has its own bounding box (voxel shapes from 150 x 148 x 204 up to 188 x 108 x 200). Every origin is a multiple of 1 cm, so the 12 maps sit on one common 1 cm grid.
- Common grid: **137 x 114 x 102 cubes, 1 cm**, origin (-0.69, -0.33, -0.51) m in the shoulder frame. Extent x -0.69 to 0.68, y -0.33 to 0.81, **z -0.51 to +0.51 m** (the height range). 1,593,036 cells; 646,056 reachable by at least one bin.
- Count per cell = number of bins (0 to 12) whose map holds the cube. **The maximum is 7**, not 12. Histogram of counts 0..7: 946,980 / 48,993 / 148,808 / 248,340 / 160,987 / 32,956 / 5,802 / 170.
- Indexing: shoulder frame, x forward along the heading, y left, z up. Offset of a grasp point: horizontal distance d = hypot(x, y), bearing theta = atan2(y, x) relative to the heading, vertical offset z (negative = below the shoulder). The reachable cells span d = 0.007 to 0.74 m and bearings over the full circle (-180 to 180 deg; mean +45 deg).
- A heading change turns the world offset by -heading before the lookup (as `stance_finder._region`). That shifts theta and leaves d and z unchanged, so a model of (d, theta, z) is heading-independent once the caller converts world positions to the shoulder-frame offset. Confirmed by the lookup code; no heading-dependent term appears in the maps.
- Every cell lies within 0.75 m horizontally, so the "within 1.5 m" evaluation region is the whole grid.

## Part B: fit

- Fit on the even z-slices (51), test on the odd slices (51). Least squares on the count over all cells (zeros included), `scipy.optimize.least_squares`. Two starting points each.
- Definitions: reachable = count >= 1. Precision = share of cells the model calls reachable (predicted count >= 1) that truly are. Recall = share of truly reachable cells the model catches. Within-1-bin = share of truly reachable cells where |predicted - true| <= 1 (the prediction is not rounded).

### Fitted parameters

| parameter | unit | specified form | alternative (constant angular centre) |
|---|---|---|---|
| a0 (peak count at z = 0) | bins | 2.478 | 3.429 |
| a1 | bins/m | 4.499 | 6.402 |
| b0 (radial centre at z = 0) | m | 0.2714 | 0.3728 |
| b1 | m/m | -0.6520 | -0.1448 |
| c0 (radial width at z = 0) | m | 0.6514 | 0.3079 |
| c1 | m/m | -0.9296 | -0.1905 |
| e0 (angular width at z = 0) | rad | 71.04 | 4.393 |
| e1 | rad/m | 163.37 | -9.246 |
| th0 (angular centre) | rad | 0 (fixed) | 1.5776 (90.4 deg) |
| RMSE on fit slices | bins | 1.299 | 1.180 |

In the specified form the angular width came out at 71 rad, so the fit switches the angular term off (the reachable cells are not concentrated around the heading).

### Results

| | precision | recall | within 1 bin |
|---|---|---|---|
| target | >= 80% | >= 80% | >= 80% |
| specified, fitted slices | 62.3% | 88.3% | 39.3% |
| specified, held-out slices | 62.1% | 88.2% | 39.4% |
| alternative, fitted slices | 73.3% | 88.0% | 44.9% |
| alternative, held-out slices | 73.3% | 88.0% | 44.9% |

Missed targets, plainly:
- **Precision**: specified short by 17.7 points (fitted) and 17.9 (held-out). Alternative short by 6.7 points on both.
- **Within 1 bin**: specified short by 40.7 points (fitted) and 40.6 (held-out). Alternative short by 35.1 points (fitted) and 35.1 (held-out).
- **Recall** met in both (88%).
- Fitted and held-out agree to under 0.2 points, so the miss is the model's form, not overfitting.

### The one alternative

I added a constant angular centre th0 (9 parameters). Why: the reachable cells have a mean bearing of +45 deg (the left arm reaches around its own shoulder), while the specified form centres the angular term on 0. The fit moved th0 to about 90 deg, widened the angular term, and gained 11 points of precision and 5.6 points of within-1-bin. It still misses the same two targets. No further tuning was done.

### What the picture shows

`reach_model_slices.png`: for the standard stance (staging position and yaw, pedestal 0.155 m) the beaker's grasp point is at shoulder-frame offset (0.425 forward, 0.204 left, 0.298 below) and the hotplate's place point at (0.425, 0.004, -0.230). The maps are a ring around the shoulder with a hole in the middle, a roughly flat count of 2 to 4 across the ring, sharp outer and inner edges, and sector-shaped patches (each bin covers a different part of the ring). The model gives a smooth bump; its count = 1 edge lies outside the maps' outer edge and it also fills the ring's inner hole region differently. Both offsets sit inside the maps' ring.

## Decisions needed

1. Accept a looser set of targets, or change the model form (the ring with a flat top and a hole is not a single Gaussian in d). I did not try anything else, as instructed.
2. Whether to go on to Part C with this model anyway. Not started.

## Not done

- Part C: no analytic inverse, no stance finder, no scene tests, no full-check runs (no MuJoCo process this task).
- The right arm.
- Nothing under the pipeline, stance selection, tests, baselines or maps changed. Nothing pushed; main untouched.

## Files

- `src/reach_model.py` (the forward model `predict_count`; standalone, not wired in)
- `experiments/reach_model_fit.py` (Part A and B; numpy and scipy only)
- `experiments/reach_model_plot.py` (the PNG)
- `docs/reports/reach_model_fit.json` (parameters, facts, scores)
- `docs/reports/reach_model_slices.png`
- Fit log: `data/logs/reach_model_fit_20261108_022856.log` (git-ignored)
