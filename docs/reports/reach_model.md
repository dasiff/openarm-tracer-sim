# Gaussian reach model: Part A (maps) and Part B (fit) — targets missed, stopped

Written for: David (task: fit a Gaussian model of the reach maps, left arm). **Filtered maps only** (`data/reach_map_before_bench`, the bench is in the maps), as David instructed; the task text said "unfiltered", the instruction since then overrides it. An earlier run on the unfiltered maps was replaced by this one.

**Outcome: the specified model and the one alternative both miss two of the three targets. Per the task, I stopped. Part C (analytic inverse, stance finder in `src/reach_model.py`, scene tests) was not started.**

## Part A: the maps (read directly from the .npz files, no stance-code lookup)

- Source: `data/reach_map_before_bench/left_bin0..11.npz`.
- Each file stores 5 mm voxels (packed bits). A 1 cm cube is reachable when all 8 of its voxels are (the maps' cube rule).
- Each bin has its own bounding box. Every origin is a multiple of 1 cm, so the 12 maps sit on one common 1 cm grid.
- Common grid: **137 x 114 x 102 cubes, 1 cm**, origin (-0.69, -0.33, -0.51) m in the shoulder frame. Extent x -0.69 to 0.68, y -0.33 to 0.81, **z -0.51 to +0.51 m** (the height range). 1,593,036 cells; 635,093 reachable by at least one bin.
- Count per cell = number of bins (0 to 12) whose map holds the cube. **The maximum is 7**, not 12. Histogram of counts 0..7: 957,943 / 52,739 / 152,743 / 240,981 / 153,204 / 30,014 / 5,300 / 112.
- Indexing: shoulder frame, x forward along the heading, y left, z up. Offset of a grasp point: horizontal distance d = hypot(x, y), bearing theta = atan2(y, x) relative to the heading, vertical offset z (negative = below the shoulder). The reachable cells span d = 0.007 to 0.74 m and bearings over the full circle (mean +46 deg).
- A heading change turns the world offset by -heading before the lookup (as `stance_finder._region`). That shifts theta and leaves d and z unchanged, so a model of (d, theta, z) is heading-independent once the caller converts world positions to the shoulder-frame offset.
- Every cell lies within 0.75 m horizontally, so the "within 1.5 m" evaluation region is the whole grid.

## Part B: fit

- Fit on the even z-slices (51), test on the odd slices (51). Least squares on the count over all cells (zeros included), `scipy.optimize.least_squares`. Two starting points each.
- Definitions: reachable = count >= 1. Precision = share of cells the model calls reachable (predicted count >= 1) that truly are. Recall = share of truly reachable cells the model catches. Within-1-bin = share of truly reachable cells where |predicted - true| <= 1 (the prediction is not rounded).

### Fitted parameters

| parameter | unit | specified form | alternative (constant angular centre) |
|---|---|---|---|
| a0 (peak count at z = 0) | bins | 2.5502 | 3.7235 |
| a1 | bins/m | -1.9868 | 6.9847 |
| b0 (radial centre at z = 0) | m | 0.3609 | 0.3869 |
| b1 | m/m | -0.1335 | -0.0781 |
| c0 (radial width at z = 0) | m | 0.3028 | 0.2907 |
| c1 | m/m | 0.2092 | -0.2097 |
| e0 (angular width at z = 0) | rad | 22.18 | 3.291 |
| e1 | rad/m | 54.75 | -6.926 |
| th0 (angular centre) | rad | 0 (fixed) | 1.6705 (95.7 deg) |
| RMSE on fit slices | bins | 1.323 | 1.145 |

In the specified form the angular width came out at 22 rad, so the fit switches the angular term off (the reachable cells are not concentrated around the heading).

### Results

| | precision | recall | within 1 bin |
|---|---|---|---|
| target | >= 80% | >= 80% | >= 80% |
| specified, fitted slices | 64.3% | 89.6% | 41.3% |
| specified, held-out slices | 63.6% | 89.5% | 41.3% |
| alternative, fitted slices | 73.7% | 85.3% | 46.6% |
| alternative, held-out slices | 73.7% | 85.2% | 46.6% |

Missed targets, plainly:
- **Precision**: specified short by 15.7 points (fitted) and 16.4 (held-out). Alternative short by 6.3 points on both.
- **Within 1 bin**: specified short by 38.7 points on both. Alternative short by 33.5 (fitted) and 33.4 (held-out).
- **Recall** met in both (89.6% and 85.3% fitted).
- Fitted and held-out agree to under 1 point, so the miss is the model's form, not overfitting.

### The one alternative

I added a constant angular centre th0 (9 parameters). Why: the reachable cells have a mean bearing of +46 deg (the left arm reaches around its own shoulder), while the specified form centres the angular term on 0. The fit moved th0 to about 96 deg and widened the angular term. Precision rose 9.4 points and within-1-bin 5.2 points, but recall fell 4.3 points. It still misses the same two targets. No further tuning was done.

### What the picture shows

`reach_model_slices.png`: for the standard stance (staging position and yaw, pedestal 0.155 m) the beaker's grasp point is at shoulder-frame offset (0.425 forward, 0.204 left, 0.298 below) and the hotplate's place point at (0.425, 0.004, -0.230); the hotplate height is the bench top + 0.068 m + the beaker's grasp fraction. The maps are a ring around the shoulder with a hole in the middle, a roughly flat count of 2 to 4 across the ring, sharp outer and inner edges, and sector-shaped patches (each bin covers a different part of the ring). The model gives a smooth bump; its count >= 1 edge lies outside the maps' outer edge, and the inner hole is a small circle instead of the maps' off-centre shape. Both offsets sit inside the maps' ring.

## Decisions needed

1. Accept looser targets, or change the model form (a ring with a flat top and a hole is not a single Gaussian in d). I did not try anything else, as instructed.
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
- Fit log: `data/logs/reach_model_fit_filtered_20261108_023447.log` (git-ignored)

## Follow-up (David's suggestion): Gaussian in the 3D distance r = sqrt(d^2 + z^2)

Same filtered maps, same even/odd slice split, no bearing term (`experiments/reach_model_fit_r.py`, results `docs/reports/reach_model_fit_r.json`). Part C is still not started.

| model | parameters | precision (fit / held-out) | recall | within 1 bin |
|---|---|---|---|---|
| R1: C exp(-((r - r0)/sr)^2) | C = 2.522 bins, r0 = 0.3994 m, sr = 0.2494 m | 66.0% / 66.1% | 88.8% / 88.8% | 45.0% / 44.9% |
| R2: R1 and count = 0 below z_cut | C = 2.634 bins, r0 = 0.4148 m, sr = 0.2473 m, z_cut = -0.395 m | 69.0% / 68.3% | 90.0% / 90.4% | 46.8% / 47.0% |
| specified form (above, for comparison) | 8 parameters | 64.3% / 63.6% | 89.6% / 89.5% | 41.3% / 41.3% |

- Targets (all >= 80%): **still missed**, precision by 10 to 14 points and within-1-bin by 33 to 35 points. Recall is met.
- The table cutoff is not a sharp step in these maps: the lowest reachable cell is at z = -0.445 m and the reachable cells per slice grow gradually (50, 1,193, 1,919, 2,518, 3,049 for the lowest five slices). The best cut found was -0.395 m.
- Why an r-only model cannot reach the targets: the best possible prediction from r alone (the mean count in each 1 cm band of r, learned on the fit slices) scores 64.6% precision, 91.8% recall and 44.4% within-1-bin on the held-out slices. The Gaussian in r (66%, 89%, 45%) is already as good as any function of r alone. The remaining error is the variation of the count with bearing and with the bin sectors, which an r-only model does not carry.
- I did not obtain a valid ceiling for r with z, or with bearing: the even and odd slices have different z values, so a lookup learned on one set has no entries on the other. Not reported.
