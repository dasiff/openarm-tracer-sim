# Layout sweep, current mode (2026-11-06)

Script: `experiments/layout_sweep.py` (run as `--mode current --n 100`). Raw results: `docs/reports/layout_sweep_current_20261106.json`.

## Setup

- Usable surface: the bench top's extent in the scene, shrunk by `LAYOUT_CFG["bench_edge_margin"]` = 0.05 m on every edge: x -2.150 to 2.150 m, y -0.750 to 0.750 m.
- Object centres uniform in that rectangle; a layout is redrawn when beaker and hotplate centres are closer than the sum of their radii (beaker 0.035 m, hotplate 0.09 m, read from the world model's shapes).
- Selection: `select_bench_stance` with `stance_check_mode` = current, the full check the pipeline builds, unfiltered maps `data/reach_map` (option A). Seed 20261106.

## Result

- **Pass rate: 0 of 100 = 0.0 %.** Step 4 (two_pass on the same layouts) was therefore skipped, as specified (threshold 85 %).
- Failure reasons (the selection's stop message):
  - 69: no stance covers any of the objects
  - 31: no stance passed the full check

## Positions of the failing layouts

| # | beaker (x, y) m | hotplate (x, y) m | stop |
|---|---|---|---|
| 0 | (0.639, -0.618) | (-1.139, 0.176) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 1 | (-0.752, 0.747) | (1.641, -0.733) | no stance passed the full check: (-0.787, 1.082) pedestal 0.15: ['ik'] at the stance 'nominal' point 4 (over,  |
| 2 | (-2.043, -0.580) | (-0.978, -0.544) | no stance passed the full check: (-2.268, -0.165) pedestal 0.19: ['view'] at pedestal 0.190 beaker_corner0 is  |
| 3 | (-0.821, -0.550) | (1.678, -0.746) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 4 | (0.920, 0.291) | (-2.127, 0.111) | no stance passed the full check: (0.805, 0.856) pedestal 0.05: ['home_clearance'] at pedestal 0.050 the arms a |
| 5 | (2.005, 0.501) | (-0.082, 0.024) | no stance passed the full check: (1.630, 0.956) pedestal 0.12: ['ik'] at the stance 'nominal' point 0 (grasp,  |
| 6 | (0.830, 0.665) | (1.629, 0.327) | no stance passed the full check: (0.805, 1.000) pedestal 0.15: ['ik'] at the stance 'nominal' point 0 (grasp,  |
| 7 | (0.195, 0.347) | (-1.468, 0.419) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 8 | (-1.765, -0.327) | (-1.397, -0.162) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 9 | (-0.278, -0.072) | (-0.787, -0.663) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 10 | (-2.013, 0.310) | (0.522, 0.244) | no stance passed the full check: (-2.378, 0.745) pedestal 0.14: ['ik'] at the stance 'nominal' point 0 (grasp, |
| 11 | (-1.316, 0.191) | (-1.681, -0.317) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 12 | (0.148, -0.149) | (-1.795, -0.067) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 13 | (-2.046, -0.652) | (-0.111, -0.283) | no stance passed the full check: (-2.261, -0.247) pedestal 0.20: ['view'] at pedestal 0.200 beaker_corner0 is  |
| 14 | (-0.095, 0.477) | (-0.946, -0.233) | no stance passed the full check: (-0.330, 0.882) pedestal 0.13: ['ik'] at the stance 'nominal' point 4 (over,  |
| 15 | (0.322, -0.233) | (-0.909, -0.713) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 16 | (1.831, -0.083) | (2.000, 0.004) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 17 | (0.475, 0.379) | (-0.910, 0.329) | no stance passed the full check: (0.330, 0.864) pedestal 0.11: ['view'] at pedestal 0.110 beaker_corner7 is -4 |
| 18 | (-0.104, 0.264) | (1.349, 0.683) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 19 | (-2.028, 0.041) | (-0.484, -0.192) | no stance passed the full check: (-2.383, 0.476) pedestal 0.13: ['ik'] at the stance 'pedestal+1' point 0 (gra |
| 20 | (0.890, -0.197) | (-0.604, -0.701) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 21 | (-0.586, -0.066) | (2.149, 0.295) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 22 | (-0.090, -0.088) | (-0.410, 0.336) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 23 | (-1.134, -0.445) | (0.280, 0.356) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 24 | (0.221, -0.121) | (-1.440, -0.280) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 25 | (0.666, 0.410) | (1.151, -0.615) | no stance passed the full check: (0.441, 0.855) pedestal 0.12: ['ik'] at the stance 'nominal' point 5 (standof |
| 26 | (1.333, -0.415) | (0.709, -0.408) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 27 | (0.707, 0.351) | (0.969, 0.709) | no stance passed the full check: (0.532, 0.856) pedestal 0.10: ['home_clearance'] at pedestal 0.100 the arms a |
| 28 | (1.018, 0.011) | (1.169, -0.563) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 29 | (1.371, -0.725) | (-0.561, 0.628) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 30 | (1.824, 0.697) | (0.757, -0.301) | no stance passed the full check: (1.589, 1.102) pedestal 0.13: ['ik'] at the stance 'nominal' point 4 (over, [ |
| 31 | (-0.887, 0.273) | (1.195, -0.031) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 32 | (1.780, -0.357) | (-0.363, 0.089) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 33 | (-1.558, -0.329) | (1.279, -0.270) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 34 | (-1.235, -0.130) | (-1.478, 0.456) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 35 | (0.535, 0.606) | (0.141, 0.074) | no stance passed the full check: (0.500, 0.941) pedestal 0.15: ['ik'] at the stance 'nominal' point 4 (over, [ |
| 36 | (-0.950, -0.044) | (-1.693, -0.246) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 37 | (-0.567, 0.226) | (-0.804, 0.578) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 38 | (1.451, 0.458) | (-0.933, -0.577) | no stance passed the full check: (1.236, 0.873) pedestal 0.13: ['home_clearance'] at pedestal 0.130 the arms a |
| 39 | (1.448, 0.458) | (0.095, -0.526) | no stance passed the full check: (1.233, 0.873) pedestal 0.13: ['home_clearance'] at pedestal 0.130 the arms a |
| 40 | (-1.797, -0.199) | (1.049, -0.520) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 41 | (0.639, -0.599) | (1.485, -0.646) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 42 | (-0.916, 0.650) | (2.069, -0.204) | no stance passed the full check: (-0.951, 0.985) pedestal 0.15: ['ik'] at the stance 'nominal' point 4 (over,  |
| 43 | (-0.708, 0.169) | (-1.924, -0.370) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 44 | (0.848, -0.586) | (-1.644, -0.043) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 45 | (0.885, 0.413) | (-1.082, 0.653) | no stance passed the full check: (0.670, 0.858) pedestal 0.12: ['placed_beaker'] at pedestal 0.120 the arms at |
| 46 | (0.433, -0.218) | (-0.186, -0.079) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 47 | (1.803, 0.283) | (1.830, -0.293) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 48 | (-0.476, -0.157) | (1.237, -0.145) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 49 | (1.891, 0.500) | (-1.481, 0.199) | no stance passed the full check: (1.636, 0.935) pedestal 0.12: ['view'] at pedestal 0.120 hotplate_corner3 is  |
| 50 | (-0.306, 0.403) | (0.986, -0.200) | no stance passed the full check: (-0.551, 0.858) pedestal 0.12: ['view'] at pedestal 0.120 beaker_corner7 is - |
| 51 | (1.065, 0.125) | (-0.973, -0.054) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 52 | (0.719, 0.351) | (-0.546, -0.530) | no stance passed the full check: (0.544, 0.856) pedestal 0.10: ['home_clearance'] at pedestal 0.100 the arms a |
| 53 | (0.817, -0.157) | (1.167, 0.435) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 54 | (-2.123, 0.290) | (-1.340, 0.435) | no stance passed the full check: (-2.378, 0.735) pedestal 0.13: ['view'] at pedestal 0.130 beaker_corner7 is - |
| 55 | (-0.361, 0.265) | (0.519, -0.600) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 56 | (-0.990, -0.702) | (1.171, 0.293) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 57 | (-1.464, -0.599) | (0.030, -0.255) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 58 | (1.527, -0.479) | (-0.588, -0.237) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 59 | (1.989, -0.058) | (-1.527, -0.022) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 60 | (1.770, -0.700) | (2.145, -0.132) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 61 | (1.129, -0.350) | (-0.371, 0.146) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 62 | (0.524, -0.116) | (1.704, -0.379) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 63 | (1.521, -0.488) | (-1.775, -0.724) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 64 | (-1.673, -0.213) | (-0.575, -0.315) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 65 | (-0.164, -0.135) | (0.192, 0.291) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 66 | (-0.386, 0.357) | (0.699, 0.558) | no stance passed the full check: (-0.571, 0.853) pedestal 0.10: ['ik'] at the stance 'nominal' point 0 (grasp, |
| 67 | (-0.029, -0.431) | (1.880, -0.352) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 68 | (0.740, -0.313) | (-1.190, 0.594) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 69 | (0.982, 0.668) | (1.708, -0.239) | no stance passed the full check: (0.957, 1.003) pedestal 0.15: ['placed_beaker'] at pedestal 0.150 the arms at |
| 70 | (1.227, -0.407) | (1.965, -0.310) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 71 | (-2.021, 0.405) | (-0.247, -0.693) | no stance passed the full check: (-2.376, 0.840) pedestal 0.13: ['ik'] at the stance 'pedestal+1' point 0 (gra |
| 72 | (-0.952, 0.013) | (1.545, 0.053) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 73 | (-0.573, 0.560) | (-1.631, -0.034) | no stance passed the full check: (-0.607, 0.895) pedestal 0.15: ['ik'] at the stance 'nominal' point 4 (over,  |
| 74 | (-0.906, -0.566) | (-0.134, -0.196) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 75 | (0.898, 0.026) | (-1.866, -0.580) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 76 | (-0.821, 0.004) | (-1.353, 0.562) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 77 | (-0.027, -0.513) | (0.787, 0.365) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 78 | (-0.568, 0.390) | (0.214, 0.629) | no stance passed the full check: (-0.823, 0.855) pedestal 0.11: ['view'] at pedestal 0.110 beaker_corner7 is - |
| 79 | (-1.526, -0.394) | (-0.719, -0.125) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 80 | (0.836, 0.663) | (-1.637, 0.687) | no stance passed the full check: (0.811, 0.998) pedestal 0.15: ['ik'] at the stance 'nominal' point 0 (grasp,  |
| 81 | (0.675, -0.623) | (-0.702, -0.453) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 82 | (-2.140, 0.054) | (0.076, -0.744) | no stance passed the full check: (-2.385, 0.489) pedestal 0.14: ['view'] at pedestal 0.140 beaker_corner7 is - |
| 83 | (0.045, 0.239) | (0.366, 0.543) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 84 | (-1.700, 0.580) | (-0.253, -0.283) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 85 | (-1.639, -0.420) | (1.859, -0.086) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 86 | (0.581, -0.521) | (1.743, -0.021) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 87 | (-1.139, 0.345) | (0.856, 0.294) | no stance passed the full check: (-1.194, 0.850) pedestal 0.09: ['ik'] at the stance 'x+1' point 0 (grasp, [-1 |
| 88 | (1.780, -0.611) | (-0.864, 0.284) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 89 | (1.345, 0.163) | (1.659, -0.569) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 90 | (0.544, -0.419) | (0.775, -0.716) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 91 | (-0.714, -0.648) | (-1.752, 0.286) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 92 | (1.243, -0.344) | (0.517, 0.719) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 93 | (-1.741, -0.363) | (-1.418, 0.638) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 94 | (0.254, 0.219) | (-0.229, -0.255) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 95 | (-1.006, 0.575) | (1.051, -0.261) | no stance passed the full check: (-1.040, 0.910) pedestal 0.15: ['ik'] at the stance 'nominal' point 4 (over,  |
| 96 | (-0.492, -0.017) | (1.360, -0.632) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 97 | (-0.515, -0.396) | (1.351, 0.603) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 98 | (-1.486, 0.663) | (-0.505, -0.179) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |
| 99 | (-1.537, 0.718) | (0.101, 0.503) | no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them |

## Observations (not decisions)

- The sample covers the whole bench top, most of which is far from the robot. The robot stands near (0.77, 1.09) facing the bench at about y 0.8 m, so layouts with x far from 0.77 are out of reach, and that is most of the sample. This is a property of the rectangle the brief specifies, not of the selection; a rectangle around the robot's reach would give a more useful rate. That is a decision for you.
- The reach maps are the unfiltered option A maps; the cheap pass is not in this run (current mode).
