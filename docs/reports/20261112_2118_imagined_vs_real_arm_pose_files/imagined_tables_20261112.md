# Imagined-pose comparison tables, 2026-11-12

Sources: `data/logs/imagined_compare_staging_20261112_205653.json` and `data/logs/imagined_compare_moved_20261112_210418.json`.

Joint differences are real minus imagined, in rad. Hand and elbow differences are distances in mm. `real_next_row_max_joint_change_rad` is in rad (4 decimals). `-` means missing. The `check-vs-dry hand` column is `check_vs_dry_hand_world_mm`.

## Staging run

Run directory: `data/demo_record/imagined_staging_20261112_205653`. Accepted candidate: 32.

### 1. Check pose vs real arm, joint differences in rad (real minus check)

One row per point that has the value.

| point | J1 | J2 | J3 | J4 | J5 | J6 | J7 | max abs |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| pick approach (over the beaker) | 0.037 | -0.338 | -0.010 | 0.003 | -0.271 | -0.044 | 0.031 | 0.338 |
| pick standoff (down above the beaker) | 0.276 | 0.499 | -0.841 | -0.095 | 0.724 | -0.491 | 0.242 | 0.841 |
| grasp (in around the beaker) | -0.039 | -0.247 | 0.354 | -0.127 | -0.397 | -0.101 | -0.146 | 0.397 |
| lift | 0.465 | 0.540 | -1.056 | -0.071 | 0.850 | -0.565 | 0.366 | 1.056 |
| place approach (carry over the hotplate) | -0.002 | 0.061 | -0.060 | -0.033 | 0.069 | -0.070 | 0.016 | 0.070 |
| place down (set down) | 0.011 | 0.057 | -0.084 | -0.019 | 0.079 | -0.059 | -0.002 | 0.084 |
| retreat (back after release) | 0.022 | 0.054 | -0.084 | 0.021 | 0.056 | -0.061 | 0.048 | 0.084 |

### 2. Dry-run pose vs real arm, joint differences in rad (real minus dry)

One row per point that has the value.

| point | J1 | J2 | J3 | J4 | J5 | J6 | J7 | max abs |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| pick approach (over the beaker) | -0.024 | -0.005 | -0.001 | -0.017 | -0.024 | -0.024 | 0.005 | 0.024 |
| pick standoff (down above the beaker) | -0.019 | -0.001 | 0.010 | -0.014 | -0.003 | 0.004 | -0.002 | 0.019 |
| grasp (in around the beaker) | -0.027 | 0.003 | 0.009 | -0.030 | 0.000 | -0.001 | -0.005 | 0.030 |
| lift | -0.018 | 0.008 | -0.001 | -0.013 | 0.006 | -0.002 | 0.014 | 0.018 |
| place approach (carry over the hotplate) | -0.033 | 0.010 | 0.013 | -0.038 | 0.002 | -0.017 | 0.001 | 0.038 |
| place down (set down) | 0.059 | 0.056 | 0.034 | -0.416 | 0.141 | -0.085 | -0.465 | 0.465 |
| retreat (back after release) | 0.082 | 0.066 | 0.043 | -0.397 | 0.160 | -0.104 | -0.441 | 0.441 |
| home (arms at home pose) | 0.000 | -0.001 | 0.000 | 0.000 | 0.000 | 0.001 | 0.000 | 0.001 |

### 3. Hand (pad) and elbow differences per point (mm; last column in rad)

| point | check-vs-real hand (world) | check-vs-real hand (body) | check-vs-real elbow (world) | dry-vs-real hand (world) | dry-vs-real hand (body) | dry-vs-real elbow (world) | check-vs-dry hand | real_next_row_max_joint_change_rad |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| pick approach (over the beaker) | 1.5 | 5.0 | 75.8 | 1.0 | 4.7 | 2.9 | 0.5 | 0.0056 |
| pick standoff (down above the beaker) | 0.4 | 4.0 | 124.3 | 0.5 | 4.0 | 1.8 | 0.6 | 0.0009 |
| grasp (in around the beaker) | 1.0 | 4.0 | 55.3 | 1.3 | 5.2 | 1.8 | 1.7 | 0.0028 |
| lift | 0.8 | 3.2 | 144.3 | 0.8 | 3.8 | 1.1 | 0.7 | 0.0022 |
| pick pull-back | - | - | - | - | - | - | - | - |
| place approach (carry over the hotplate) | 7.0 | 8.1 | 13.8 | 6.6 | 8.5 | 3.5 | 2.0 | 0.0028 |
| place down (set down) | 2.5 | 3.5 | 13.9 | 99.0 | 99.1 | 21.3 | 101.0 | 0.0010 |
| retreat (back after release) | 5.2 | 4.6 | 13.9 | 101.9 | 101.9 | 26.8 | 106.3 | 0.0074 |
| home (arms at home pose) | - | - | - | 4.0 | 0.5 | 4.0 | - | 0.0000 |

### 4. Largest elbow difference

Largest elbow difference (world, all points): check-vs-real 144.3 mm at lift; dry-vs-real 26.8 mm at retreat (back after release).

## Moved run

Run directory: `data/demo_record/imagined_moved_20261112_210418`. Accepted candidate: 15.

### 1. Check pose vs real arm, joint differences in rad (real minus check)

One row per point that has the value.

| point | J1 | J2 | J3 | J4 | J5 | J6 | J7 | max abs |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| pick approach (over the beaker) | 0.013 | -0.151 | -0.144 | -0.032 | -0.392 | -0.226 | 0.000 | 0.392 |
| pick standoff (down above the beaker) | 0.124 | 0.317 | -0.482 | -0.122 | 0.340 | -0.582 | 0.023 | 0.582 |
| grasp (in around the beaker) | -0.010 | -0.239 | 0.283 | -0.012 | -0.306 | -0.142 | 0.024 | 0.306 |
| lift | 0.398 | 0.489 | -0.976 | -0.061 | 0.782 | -0.477 | 0.298 | 0.976 |
| place approach (carry over the hotplate) | -0.004 | 0.042 | -0.050 | -0.030 | 0.062 | -0.043 | -0.003 | 0.062 |
| place down (set down) | 0.011 | 0.037 | -0.070 | -0.028 | 0.073 | -0.044 | -0.026 | 0.073 |
| retreat (back after release) | 0.016 | 0.039 | -0.061 | 0.001 | 0.051 | -0.045 | 0.018 | 0.061 |

### 2. Dry-run pose vs real arm, joint differences in rad (real minus dry)

One row per point that has the value.

| point | J1 | J2 | J3 | J4 | J5 | J6 | J7 | max abs |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| pick approach (over the beaker) | -0.035 | -0.007 | 0.009 | -0.027 | -0.026 | -0.014 | 0.009 | 0.035 |
| pick standoff (down above the beaker) | -0.022 | -0.002 | 0.010 | -0.021 | -0.002 | 0.001 | -0.004 | 0.022 |
| grasp (in around the beaker) | -0.027 | 0.004 | 0.009 | -0.035 | 0.002 | -0.005 | -0.012 | 0.035 |
| lift | -0.023 | 0.008 | -0.001 | -0.025 | 0.006 | -0.007 | 0.007 | 0.025 |
| place approach (carry over the hotplate) | -0.030 | 0.010 | 0.008 | -0.044 | 0.002 | -0.009 | -0.014 | 0.044 |
| place down (set down) | 0.127 | 0.024 | 0.015 | -0.308 | 0.073 | -0.067 | -0.431 | 0.431 |
| retreat (back after release) | 0.181 | 0.025 | 0.011 | -0.271 | 0.081 | -0.085 | -0.422 | 0.422 |
| home (arms at home pose) | 0.000 | -0.001 | 0.001 | 0.000 | -0.001 | 0.001 | -0.003 | 0.003 |

### 3. Hand (pad) and elbow differences per point (mm; last column in rad)

| point | check-vs-real hand (world) | check-vs-real hand (body) | check-vs-real elbow (world) | dry-vs-real hand (world) | dry-vs-real hand (body) | dry-vs-real elbow (world) | check-vs-dry hand | real_next_row_max_joint_change_rad |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| pick approach (over the beaker) | 2.5 | 6.1 | 32.1 | 2.2 | 6.1 | 2.3 | 0.5 | 0.0017 |
| pick standoff (down above the beaker) | 0.8 | 4.2 | 77.0 | 0.7 | 4.4 | 1.6 | 0.6 | 0.0019 |
| grasp (in around the beaker) | 2.3 | 3.7 | 53.0 | 1.3 | 4.9 | 2.8 | 2.2 | 0.0008 |
| lift | 0.9 | 3.6 | 130.5 | 0.4 | 4.5 | 3.3 | 1.0 | 0.0000 |
| pick pull-back | - | - | - | - | - | - | - | - |
| place approach (carry over the hotplate) | 5.1 | 5.4 | 11.5 | 5.0 | 6.9 | 4.7 | 2.9 | 0.0011 |
| place down (set down) | 4.3 | 3.3 | 11.4 | 103.5 | 103.6 | 32.9 | 100.5 | 0.0014 |
| retreat (back after release) | 4.0 | 1.9 | 12.2 | 106.8 | 106.8 | 45.2 | 105.8 | 0.0010 |
| home (arms at home pose) | - | - | - | 4.3 | 0.1 | 4.6 | - | 0.0003 |

### 4. Largest elbow difference

Largest elbow difference (world, all points): check-vs-real 130.5 mm at lift; dry-vs-real 45.2 mm at retreat (back after release).
