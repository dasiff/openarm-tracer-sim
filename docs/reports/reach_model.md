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

## Follow-up 2 (David): asymmetric bearing term

Formula (filtered maps, same even/odd split, `experiments/reach_model_fit_asym.py`, results `docs/reports/reach_model_fit_asym.json`, PNG `docs/reports/reach_model_asym_slices.png`):

count = C exp(-((r - r0)/sr)^2) exp(-(theta / sigma(theta))^2), 0 below z_cut. r = 3D distance, theta = atan2(y, x) in the shoulder frame, so **theta > 0 is to the robot's left**. The formula's "sigma_left (theta < 0)" is therefore the width on the robot's right-hand side; the table names the widths by the sign of theta.

| parameter | unit | A6 (6 parameters) | A8 (widths linear in z, 8 parameters) |
|---|---|---|---|
| C | bins | 3.5286 | 3.7363 |
| r0 | m | 0.4384 | 0.4527 |
| sr | m | 0.2238 | 0.2186 |
| sigma, theta < 0 (David's sigma_left): at z = 0 | rad | 0.3203 | 0.2716 |
| sigma, theta < 0: change per metre of z | rad/m | 0 (fixed) | -0.5479 |
| sigma, theta >= 0 (David's sigma_right): at z = 0 | rad | 15.96 | 50.68 |
| sigma, theta >= 0: change per metre of z | rad/m | 0 (fixed) | -117.57 |
| z_cut | m | -0.395 | -0.375 |

One-tailed behaviour: neither width is near zero at z = 0, but the fit is very lopsided. The theta >= 0 width is far above pi (16 rad in A6), so the model does not decay with bearing on the left side at all. The theta < 0 width is narrow (0.32 rad = 18 deg in A6), so on the right-hand side it falls off within about 20 to 30 degrees of the heading. In A8 the theta < 0 width shrinks with height (0.44 rad at z = -0.3 m, 0.11 rad at z = +0.3 m).

All models tried so far (held-out slices; fitted-slice values are within 1 point except A6 and A8, where they are 79.0 / 75.4 / 48.4 and 82.6 / 74.2 / 50.9):

| model | parameters | precision | recall | within 1 bin |
|---|---|---|---|---|
| target | | >= 80% | >= 80% | >= 80% |
| specified (d, z, angular centre 0) | 8 | 63.6% | 89.5% | 41.3% |
| alternative (constant angular centre) | 9 | 73.7% | 85.2% | 46.6% |
| R1 Gaussian in r | 3 | 66.1% | 88.8% | 44.9% |
| R2 Gaussian in r and z cutoff | 4 | 68.3% | 90.4% | 47.0% |
| A6 asymmetric bearing | 6 | 78.2% | 75.8% | 48.8% |
| A8 asymmetric bearing, widths linear in z | 8 | 82.3% | 74.8% | 51.3% |

Missed targets, plainly (held-out):
- A6: precision short by 1.8 points, recall short by 4.2, within-1-bin short by 31.2.
- A8: precision met (+2.3), recall short by 5.2, within-1-bin short by 28.7.
- Per the task, A8 was the last attempt, so I stopped: **Part C (analytic inverse, stance finder, scene tests) was not started.**

What the picture shows: the maps are a nearly full ring around the shoulder (both sides of the heading). The asymmetric model covers the left half well but is cut to a thin wedge on the right, where the maps still have reachable cells. That costs recall (about 75%) and the count error stays high (within-1-bin about 50%) because the maps' count varies by sector with the bins, which no smooth bearing term follows.

## Analytic inverse (A8, left and right arms, heading and pedestal swept): scene test

Code: `src/reach_model.py` (`arm_params`, `ideal_spot`, `counts_at`, `inverse_search`, `single_object_stance`), scene runner `experiments/reach_inverse_scene.py`. Outputs: `docs/reports/reach_inverse_{standard,moved}.json` (sweep), `..._fixed_heading.json` (the first pass, heading -90 deg, pedestal 0.16 m only).

**Right arm.** The A8 model mirrored: `theta` flips sign, so the wide side (sigma0 = 50.68, sigma1 = -117.57) is theta < 0 and the narrow side (sigma0 = 0.2716, sigma1 = -0.5479) is theta >= 0. Right-arm shoulder: torso-frame (0, -0.031, 0.698), from the same `robot_geometry` read as the left.

**Ideal spot (single object).** The peak is at the 3D distance r0 from the shoulder, so the horizontal distance is d = sqrt(r0^2 - z^2), with z the object's height relative to the shoulder (not d = r0; the spec's formula ignores z). Then:

    base_xy = object_xy - d * (cos h, sin h) - R(h) (sx, sy)

with (sx, sy) the arm's shoulder offset in the base frame (left +0.031, right -0.031) and z = object_z - (base_z + torso_z0 + q + sz), sz = 0.698. Exists only if z >= -0.375 and |z| < r0.

Numbers (left arm, heading -90 deg, pedestal 0.16 m): beaker z_rel = -0.303 m, d = 0.336 m; hotplate z_rel = -0.233 m, d = 0.388 m. Spots: beaker (0.969, 0.996), hotplate (0.769, 1.048) (standard layout). The count at the peak equals C = 3.736 bins.

**Sweep.** 36 headings (0 to 350 deg, 10 deg) x 5 pedestals (0.05 to 0.25 m, 5 steps) x 50 segment points. Pedestal 0.25 m gives no ideal spot for the beaker (z_rel = -0.393 < -0.375), so 36 of 180 combinations are skipped: 7,200 segment evaluations, not 9,000. 6,784 segment points are feasible (every object's count >= 1). Ranking: worst object's count, then total count.

**Degeneracy, flagged.** The peak count is C at every (heading, pedestal), so the top 8 all sit at 3.73 to 3.74 bins and differ only in the total. The analytic model has no preference between headings, so the best heading is set by the tie-break, not by reach.

**Single-object function.** Optimizes over heading and pedestal, scored by margin = min(r0 - |z|, z - z_cut). This is also degenerate in heading: every heading gives a spot, and many of them fall on the bench footprint. For the beaker alone the tie-break picks heading 0 deg, whose spot (0.59, 0.63) lies on the bench. The bench is only the full check's job, so no bench rule was added.

**Scene results** (candidate format: `(x, y, heading, pedestal)`, `FullCheck.check(stance, points)`, `src/full_check.py`; points from `src/task_points.py`):

| scene | label | best heading | best pedestal | feasible segment points | passing full check | rejections (top 8) |
|---|---|---|---|---|---|---|
| standard | shared | 250 deg | 0.05 m | 6,784 | none | 4 `home_clearance`, 4 `placed_beaker` |
| moved | shared | 250 deg | 0.05 m | 6,784 | none | 8 `home_clearance` (the 90 deg candidates reach the arm-to-scene margin with links 1 to 4 at -34 to 3 mm, i.e. penetration at the 90 deg bases on the bench footprint) |

The fixed-heading first pass (heading -90 deg, pedestal 0.16 m; 8 candidates per scene): every candidate failed `occlusion` (the robot hides 0.8 to 1.8 % of the beaker, which allows 0 %; the hotplate allows 20 %).

**Full-check rejection names, and what they mean:**
- `home_clearance`: the arms at their home pose come within a few centimetres of the bench, the hotplate or a beaker there (examples: 25 mm from `openarm_left_left_finger`, 34 mm from `openarm_left_right_f...`; the limit is 40 mm).
- `placed_beaker`: the arms come too close to a beaker standing on the hotplate.
- `occlusion`: the robot hides part of the beaker or hotplate from the eagle camera.

Note: the 90 deg candidates lie on the bench footprint (y about 0.32 to 0.37 m). The spec leaves the bench to the full check, so these are only rejected by the full check.

## Not done
- No candidate passed the full check in either scene, so the analytic route did not yet produce a usable stance.
- The split-stance fallback ("repositioning needed") was not triggered: a shared point exists at some (heading, pedestal) in both scenes, so the per-object fallback code path was only exercised through the code, not the scenes.
- The 8 candidates are adjacent points of one segment (all within a few centimetres of each other); the spec's "top 8" does not spread them out.
- Heading-facing-the-bench, and whether the shoulder is read from MuJoCo at each pedestal (it is read once at 0.16 m and shifted vertically).

## Decisions needed
1. Ideal spot: keep the 3D-consistent d = sqrt(r0^2 - z^2) (what the peak implies) or follow the spec's d = r0 literally?
2. Heading prior: the model has no heading preference. Should the sweep prefer headings that face the bench from outside its footprint, with a rule for it, or leave it to the full check?
3. Candidate spread: return the best candidate per (heading, pedestal) instead of 8 adjacent points?
4. The `home_clearance` and `placed_beaker` failures: is the analytic model's reach enough, or should the arm-clearance margins enter the ranking?

## Rerun with the beaker occlusion limit at 5% (2026-11-08)

Config change (the only one): `STANCE_REQUIREMENTS["occlusion_max_hidden"]["beaker"]` 0.0 -> 0.05. Hotplate 20%, home_clearance 40 mm, placed_beaker unchanged. Same code as 7387c0d. Outputs: `docs/reports/reach_inverse_{standard,moved}.json` (5% run); the 0% run is kept as `..._occlusion0pct.json`.

| scene | best heading | best pedestal | feasible segment points | passed full check | rejections (top 8, 5% run) |
|---|---|---|---|---|---|
| standard | 250 deg | 0.05 m | 6,784 | none | 4 `home_clearance`, 4 `placed_beaker` |
| moved | 250 deg | 0.05 m | 6,784 | none | 8 `home_clearance` |

The 5% run gives the same top 8 candidates, the same predicted counts and the same rejections as the 0% run. The beaker occlusion limit is not what blocks these candidates: the full check runs the arm clearance checks before the occlusion check, so the candidates are rejected at `home_clearance` or `placed_beaker` before occlusion is ever evaluated.

Blocking checks, with counts: `home_clearance` 4 of 8 (standard) and 8 of 8 (moved); `placed_beaker` 4 of 8 (standard) and 0 of 8 (moved). Nothing passes in either scene.

The test suite was not run for this config change; tests that pin the occlusion limit may need their expectations reviewed.
