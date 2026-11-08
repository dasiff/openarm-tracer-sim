# Nudge ladder, current mode, unfiltered maps (2026-11-07)

Script: `experiments/nudge_ladder.py` (`--per-size 20`, seed 20261107). Raw per-layout data: `docs/reports/nudge_ladder_20261107.json`.

Standard layout (TASK_CFG, the one the demo and tests use): beaker (1.000, 0.660), hotplate (0.800, 0.660). Each object moves independently, dx and dy uniform in [-size, +size].

## Pass rate per nudge size

| group | pass | 'no stance covers any of the objects' | 'no stance passed the full check' | most common rejecting checks (failed layouts) |
|---|---|---|---|---|
| control | 1/1 | 0 | 0 | - |
| nudge_2cm | 15/20 | 4 | 1 | occlusion 1 |
| nudge_5cm | 10/20 | 5 | 5 | ik 4, occlusion 1 |
| nudge_10cm | 7/20 | 7 | 6 | ik 5, home_clearance 1 |
| nudge_20cm | 1/20 | 4 | 15 | home_clearance 8, placed_beaker 6, ik 1 |
| nudge_40cm | 1/20 | 2 | 17 | ik 11, home_clearance 5, placed_beaker 1 |
| all nudge layouts | 34/100 | | | |

Marks used below: C = 'no stance covers any of the objects', F = 'no stance passed the full check' (the most common rejecting check in brackets).

## Every failed nudge layout (offsets from the standard layout, metres)

| group | # | beaker (x, y) | beaker offset (dx, dy) | hotplate (x, y) | hotplate offset (dx, dy) | mark |
|---|---|---|---|---|---|---|
| nudge_2cm | 8 | (0.983, 0.657) | (-0.018, -0.003) | (0.801, 0.658) | (+0.001, -0.002) | C |
| nudge_2cm | 12 | (0.988, 0.643) | (-0.012, -0.017) | (0.799, 0.644) | (-0.001, -0.016) | F [occlusion] |
| nudge_2cm | 15 | (0.980, 0.648) | (-0.020, -0.012) | (0.812, 0.666) | (+0.012, +0.006) | C |
| nudge_2cm | 16 | (0.986, 0.672) | (-0.014, +0.012) | (0.808, 0.645) | (+0.008, -0.015) | C |
| nudge_2cm | 20 | (0.998, 0.648) | (-0.002, -0.012) | (0.818, 0.670) | (+0.018, +0.010) | C |
| nudge_5cm | 22 | (0.982, 0.677) | (-0.018, +0.017) | (0.817, 0.686) | (+0.017, +0.026) | C |
| nudge_5cm | 24 | (1.007, 0.641) | (+0.007, -0.019) | (0.834, 0.678) | (+0.034, +0.018) | C |
| nudge_5cm | 27 | (0.952, 0.662) | (-0.048, +0.002) | (0.811, 0.688) | (+0.011, +0.028) | C |
| nudge_5cm | 30 | (0.958, 0.675) | (-0.042, +0.015) | (0.827, 0.650) | (+0.027, -0.009) | C |
| nudge_5cm | 31 | (1.044, 0.630) | (+0.044, -0.030) | (0.785, 0.644) | (-0.015, -0.016) | F [occlusion] |
| nudge_5cm | 33 | (1.007, 0.612) | (+0.007, -0.048) | (0.840, 0.648) | (+0.040, -0.011) | C |
| nudge_5cm | 34 | (0.980, 0.634) | (-0.020, -0.026) | (0.793, 0.621) | (-0.007, -0.039) | F [ik] |
| nudge_5cm | 37 | (0.984, 0.645) | (-0.016, -0.015) | (0.784, 0.705) | (-0.016, +0.045) | F [ik] |
| nudge_5cm | 39 | (0.953, 0.673) | (-0.047, +0.013) | (0.754, 0.641) | (-0.046, -0.018) | F [ik] |
| nudge_5cm | 40 | (0.978, 0.628) | (-0.022, -0.032) | (0.771, 0.657) | (-0.029, -0.003) | F [ik] |
| nudge_10cm | 41 | (0.951, 0.705) | (-0.050, +0.045) | (0.781, 0.626) | (-0.018, -0.034) | F [ik] |
| nudge_10cm | 45 | (1.063, 0.582) | (+0.063, -0.078) | (0.720, 0.693) | (-0.081, +0.033) | F [ik] |
| nudge_10cm | 47 | (0.933, 0.725) | (-0.067, +0.065) | (0.739, 0.734) | (-0.061, +0.074) | F [ik] |
| nudge_10cm | 48 | (0.906, 0.633) | (-0.093, -0.027) | (0.719, 0.670) | (-0.082, +0.010) | F [ik] |
| nudge_10cm | 50 | (0.991, 0.700) | (-0.009, +0.040) | (0.849, 0.657) | (+0.049, -0.003) | C |
| nudge_10cm | 52 | (1.002, 0.567) | (+0.002, -0.093) | (0.721, 0.604) | (-0.079, -0.056) | F [home_clearance] |
| nudge_10cm | 53 | (0.994, 0.666) | (-0.006, +0.006) | (0.837, 0.707) | (+0.037, +0.047) | C |
| nudge_10cm | 54 | (0.941, 0.713) | (-0.059, +0.052) | (0.717, 0.570) | (-0.083, -0.090) | F [ik] |
| nudge_10cm | 56 | (0.969, 0.642) | (-0.031, -0.018) | (0.860, 0.573) | (+0.060, -0.087) | C |
| nudge_10cm | 57 | (0.920, 0.666) | (-0.080, +0.006) | (0.881, 0.595) | (+0.081, -0.065) | C |
| nudge_10cm | 58 | (0.952, 0.754) | (-0.048, +0.094) | (0.828, 0.732) | (+0.028, +0.072) | C |
| nudge_10cm | 59 | (1.025, 0.574) | (+0.025, -0.086) | (0.888, 0.612) | (+0.088, -0.048) | C |
| nudge_10cm | 60 | (0.956, 0.592) | (-0.044, -0.068) | (0.880, 0.707) | (+0.080, +0.047) | C |
| nudge_20cm | 61 | (1.027, 0.478) | (+0.027, -0.182) | (0.885, 0.644) | (+0.085, -0.016) | F [placed_beaker] |
| nudge_20cm | 62 | (0.894, 0.533) | (-0.106, -0.127) | (0.679, 0.541) | (-0.121, -0.118) | F [home_clearance] |
| nudge_20cm | 63 | (1.087, 0.524) | (+0.087, -0.136) | (0.847, 0.559) | (+0.047, -0.101) | F [home_clearance] |
| nudge_20cm | 64 | (0.894, 0.553) | (-0.106, -0.107) | (0.805, 0.812) | (+0.005, +0.152) | F [home_clearance] |
| nudge_20cm | 65 | (1.174, 0.471) | (+0.174, -0.189) | (0.929, 0.646) | (+0.129, -0.014) | F [home_clearance] |
| nudge_20cm | 66 | (0.984, 0.621) | (-0.016, -0.039) | (0.824, 0.634) | (+0.024, -0.026) | C |
| nudge_20cm | 67 | (1.198, 0.558) | (+0.199, -0.102) | (0.857, 0.704) | (+0.057, +0.044) | F [placed_beaker] |
| nudge_20cm | 68 | (1.121, 0.588) | (+0.121, -0.072) | (0.921, 0.513) | (+0.121, -0.147) | F [home_clearance] |
| nudge_20cm | 70 | (0.969, 0.477) | (-0.031, -0.183) | (0.929, 0.751) | (+0.129, +0.091) | F [placed_beaker] |
| nudge_20cm | 71 | (0.951, 0.537) | (-0.049, -0.123) | (0.823, 0.653) | (+0.023, -0.007) | C |
| nudge_20cm | 72 | (1.056, 0.575) | (+0.056, -0.085) | (0.608, 0.703) | (-0.192, +0.043) | F [home_clearance] |
| nudge_20cm | 73 | (1.086, 0.524) | (+0.086, -0.136) | (0.610, 0.682) | (-0.190, +0.022) | F [placed_beaker] |
| nudge_20cm | 74 | (0.914, 0.543) | (-0.086, -0.117) | (0.825, 0.820) | (+0.025, +0.160) | F [home_clearance] |
| nudge_20cm | 75 | (0.968, 0.792) | (-0.032, +0.132) | (0.985, 0.569) | (+0.185, -0.091) | F [ik] |
| nudge_20cm | 76 | (0.957, 0.485) | (-0.043, -0.175) | (0.794, 0.504) | (-0.006, -0.155) | C |
| nudge_20cm | 77 | (0.955, 0.638) | (-0.045, -0.022) | (0.826, 0.487) | (+0.026, -0.173) | F [placed_beaker] |
| nudge_20cm | 78 | (0.901, 0.484) | (-0.099, -0.176) | (0.617, 0.681) | (-0.183, +0.021) | F [placed_beaker] |
| nudge_20cm | 79 | (1.131, 0.629) | (+0.131, -0.030) | (0.986, 0.496) | (+0.185, -0.164) | F [home_clearance] |
| nudge_20cm | 80 | (0.970, 0.812) | (-0.030, +0.152) | (0.837, 0.701) | (+0.037, +0.041) | C |
| nudge_40cm | 81 | (0.761, 0.375) | (-0.239, -0.285) | (0.502, 0.973) | (-0.298, +0.313) | F [home_clearance] |
| nudge_40cm | 82 | (0.946, 0.697) | (-0.054, +0.037) | (0.529, 0.271) | (-0.271, -0.389) | F [ik] |
| nudge_40cm | 83 | (1.314, 0.904) | (+0.314, +0.244) | (1.096, 0.479) | (+0.296, -0.181) | F [ik] |
| nudge_40cm | 84 | (1.357, 0.839) | (+0.357, +0.179) | (0.966, 0.377) | (+0.166, -0.283) | F [ik] |
| nudge_40cm | 85 | (0.942, 0.271) | (-0.059, -0.389) | (0.869, 0.917) | (+0.069, +0.257) | C |
| nudge_40cm | 86 | (0.824, 0.515) | (-0.176, -0.145) | (1.014, 0.943) | (+0.214, +0.283) | F [placed_beaker] |
| nudge_40cm | 87 | (1.173, 0.622) | (+0.173, -0.038) | (0.706, 0.271) | (-0.095, -0.389) | F [ik] |
| nudge_40cm | 88 | (1.135, 0.603) | (+0.135, -0.057) | (0.789, 0.478) | (-0.011, -0.182) | F [home_clearance] |
| nudge_40cm | 89 | (1.067, 0.978) | (+0.067, +0.318) | (1.171, 0.956) | (+0.371, +0.295) | C |
| nudge_40cm | 90 | (1.394, 0.701) | (+0.394, +0.041) | (1.083, 0.520) | (+0.283, -0.140) | F [home_clearance] |
| nudge_40cm | 91 | (0.753, 0.331) | (-0.247, -0.329) | (0.850, 0.721) | (+0.050, +0.061) | F [home_clearance] |
| nudge_40cm | 92 | (0.868, 0.794) | (-0.133, +0.134) | (0.434, 0.502) | (-0.366, -0.158) | F [ik] |
| nudge_40cm | 93 | (0.731, 0.502) | (-0.269, -0.158) | (0.485, 0.610) | (-0.315, -0.050) | F [ik] |
| nudge_40cm | 95 | (0.796, 0.506) | (-0.204, -0.154) | (0.554, 0.503) | (-0.246, -0.157) | F [home_clearance] |
| nudge_40cm | 96 | (1.246, 0.812) | (+0.246, +0.152) | (1.113, 0.452) | (+0.313, -0.208) | F [ik] |
| nudge_40cm | 97 | (0.747, 0.901) | (-0.254, +0.241) | (0.831, 0.700) | (+0.031, +0.040) | F [ik] |
| nudge_40cm | 98 | (1.272, 1.017) | (+0.272, +0.357) | (0.406, 1.041) | (-0.394, +0.381) | F [ik] |
| nudge_40cm | 99 | (0.702, 1.020) | (-0.298, +0.360) | (0.660, 0.819) | (-0.140, +0.159) | F [ik] |
| nudge_40cm | 100 | (0.801, 0.995) | (-0.199, +0.335) | (0.829, 0.654) | (+0.029, -0.006) | F [ik] |

## Passing nudge layouts

- nudge_2cm #1: beaker (1.003, 0.655), hotplate (0.812, 0.646), stance (0.697, 1.121) pedestal 0.10
- nudge_2cm #2: beaker (1.004, 0.650), hotplate (0.789, 0.643), stance (0.684, 1.118) pedestal 0.10
- nudge_2cm #3: beaker (1.002, 0.643), hotplate (0.788, 0.643), stance (0.683, 1.108) pedestal 0.10
- nudge_2cm #4: beaker (1.019, 0.647), hotplate (0.787, 0.649), stance (0.782, 1.103) pedestal 0.12
- nudge_2cm #5: beaker (0.992, 0.671), hotplate (0.797, 0.665), stance (0.682, 1.140) pedestal 0.10
- nudge_2cm #6: beaker (1.018, 0.672), hotplate (0.811, 0.643), stance (0.706, 1.138) pedestal 0.10
- nudge_2cm #7: beaker (1.006, 0.665), hotplate (0.789, 0.673), stance (0.684, 1.130) pedestal 0.10
- nudge_2cm #9: beaker (0.986, 0.680), hotplate (0.785, 0.661), stance (0.680, 1.146) pedestal 0.10
- nudge_2cm #10: beaker (1.019, 0.666), hotplate (0.809, 0.678), stance (0.804, 1.121) pedestal 0.12
- nudge_2cm #11: beaker (1.001, 0.680), hotplate (0.813, 0.664), stance (0.678, 1.149) pedestal 0.10
- nudge_2cm #13: beaker (0.990, 0.653), hotplate (0.794, 0.663), stance (0.739, 1.118) pedestal 0.11
- nudge_2cm #14: beaker (1.018, 0.641), hotplate (0.795, 0.640), stance (0.690, 1.115) pedestal 0.10
- nudge_2cm #17: beaker (1.019, 0.649), hotplate (0.810, 0.640), stance (0.705, 1.115) pedestal 0.10
- nudge_2cm #18: beaker (1.006, 0.664), hotplate (0.805, 0.653), stance (0.700, 1.138) pedestal 0.10
- nudge_2cm #19: beaker (1.013, 0.672), hotplate (0.817, 0.642), stance (0.702, 1.137) pedestal 0.10
- nudge_5cm #21: beaker (1.022, 0.686), hotplate (0.838, 0.667), stance (0.703, 1.152) pedestal 0.10
- nudge_5cm #23: beaker (1.020, 0.681), hotplate (0.781, 0.703), stance (0.776, 1.146) pedestal 0.11
- nudge_5cm #25: beaker (1.011, 0.614), hotplate (0.818, 0.636), stance (0.763, 1.089) pedestal 0.11
- nudge_5cm #26: beaker (1.019, 0.695), hotplate (0.828, 0.611), stance (0.693, 1.156) pedestal 0.10
- nudge_5cm #28: beaker (1.044, 0.648), hotplate (0.839, 0.655), stance (0.903, 1.123) pedestal 0.10
- nudge_5cm #29: beaker (1.015, 0.702), hotplate (0.756, 0.652), stance (0.691, 1.167) pedestal 0.10
- nudge_5cm #32: beaker (1.049, 0.636), hotplate (0.771, 0.688), stance (0.766, 1.131) pedestal 0.09
- nudge_5cm #35: beaker (0.989, 0.661), hotplate (0.772, 0.639), stance (0.677, 1.134) pedestal 0.10
- nudge_5cm #36: beaker (1.019, 0.627), hotplate (0.836, 0.617), stance (0.791, 1.162) pedestal 0.08
- nudge_5cm #38: beaker (0.998, 0.634), hotplate (0.777, 0.650), stance (0.772, 1.089) pedestal 0.12
- nudge_10cm #42: beaker (1.030, 0.693), hotplate (0.715, 0.653), stance (0.720, 1.158) pedestal 0.10
- nudge_10cm #43: beaker (1.014, 0.699), hotplate (0.730, 0.608), stance (0.685, 1.143) pedestal 0.09
- nudge_10cm #44: beaker (0.987, 0.570), hotplate (0.793, 0.565), stance (0.758, 1.100) pedestal 0.08
- nudge_10cm #46: beaker (1.009, 0.618), hotplate (0.717, 0.694), stance (0.792, 1.063) pedestal 0.15
- nudge_10cm #49: beaker (1.053, 0.671), hotplate (0.712, 0.741), stance (0.777, 1.136) pedestal 0.13
- nudge_10cm #51: beaker (1.028, 0.732), hotplate (0.829, 0.678), stance (0.714, 1.203) pedestal 0.10
- nudge_10cm #55: beaker (1.039, 0.659), hotplate (0.820, 0.691), stance (0.855, 1.145) pedestal 0.11
- nudge_20cm #69: beaker (1.198, 0.837), hotplate (0.756, 0.808), stance (0.851, 1.253) pedestal 0.12
- nudge_40cm #94: beaker (1.288, 0.743), hotplate (1.119, 0.891), stance (1.154, 1.218) pedestal 0.14

## Notes

- The 'view' check (beaker/hotplate margin in the eagle image) and the 'occlusion' check are separate full-check tests; both appear as named.
- Failing layouts at 20 and 40 cm are almost all full-check failures: the objects are still covered by the reach maps, but the stance fails home clearance, IK, or the placed-beaker check.
