# Body-check boundary under the bench, and the deepest passing stance (2026-11-09)

Untracked. Work in the copy `/home/vboxuser/projects/oc_elbow` (HEAD 9ff0099 content, the copy's own `src`). No change to the real repo, no commits, no push. Guard disabled throughout. Scripts: `/home/vboxuser/projects/oc_scripts/under_bench_boundary_20261109.py` and `under_bench_ik_nominal_20261109.py`. Data: `oc_elbow/data/under_bench_boundary_body_20261109.json`, `under_bench_ik_nominal_20261109.json`.

## 1. Result

- Body boundary: at x 0.775 (heading -90 deg) the body group passes down to edge 0.055 m (base y 0.855) at pedestals 0.10, 0.16 and 0.20, and fails at 0.030 m (y 0.830). The boundary lies between 0.030 and 0.055 m on the 0.025 m grid. Changing x to 0.70 does not move it at pedestal 0.16.
- Pick ik at nominal: the three deepest body-passing stances (pedestal 0.16, edges 0.055, 0.080, 0.105 m) all FAIL. Every approach bin (12 of 12) fails at the first pick point, the 'over' point at z 1.16 m.
- So no stance with edge distance below 0.305 m passes both checks at pedestal 0.16 m. Per the task, no pick and place was run, and no elbow monitor, phase, guard-count or contact result exists.

## 2. Body table (x 0.775, heading -90 deg; min over the 9 error stances; limit > 0)

Min distance 0.100 m means nothing within the 0.1 m look distance (not a measured gap). Pair = the arm part and the world geom that set the minimum.

| x | base y | edge (y - 0.80) | pedestal | min distance (m) | body | stance | pair |
|---|---|---|---|---|---|---|---|
| 0.775 | 1.105 | 0.305 | 0.10 | 0.0619 | PASS | pedestal-1 | openarm_left_left_finger_collision vs task_hotplate_base |
| 0.775 | 1.080 | 0.280 | 0.10 | 0.0470 | PASS | pedestal-1 | openarm_left_left_finger_collision vs task_hotplate_base |
| 0.775 | 1.055 | 0.255 | 0.10 | 0.0353 | PASS | pedestal-1 | openarm_left_left_finger_collision vs task_hotplate_top_plate |
| 0.775 | 1.030 | 0.230 | 0.10 | 0.0279 | PASS | pedestal-1 | openarm_left_left_finger_collision vs task_hotplate_top_plate |
| 0.775 | 1.005 | 0.205 | 0.10 | 0.0214 | PASS | pedestal-1 | openarm_left_left_finger_collision vs task_hotplate_top_plate |
| 0.775 | 0.980 | 0.180 | 0.10 | 0.0153 | PASS | pedestal-1 | openarm_left_left_finger_collision vs task_hotplate_top_plate |
| 0.775 | 0.955 | 0.155 | 0.10 | 0.0153 | PASS | pedestal-1 | openarm_left_left_finger_collision vs task_hotplate_top_plate |
| 0.775 | 0.930 | 0.130 | 0.10 | 0.0153 | PASS | pedestal-1 | openarm_left_left_finger_collision vs task_hotplate_top_plate |
| 0.775 | 0.905 | 0.105 | 0.10 | 0.0153 | PASS | pedestal-1 | openarm_left_left_finger_collision vs task_hotplate_top_plate |
| 0.775 | 0.880 | 0.080 | 0.10 | 0.0153 | PASS | pedestal-1 | openarm_left_left_finger_collision vs task_hotplate_top_plate |
| 0.775 | 0.855 | 0.055 | 0.10 | 0.0150 | PASS | y-1 | openarm_body_link0_collision vs bench_top |
| 0.775 | 0.830 | 0.030 | 0.10 | -0.0100 | FAIL | y-1 | openarm_body_link0_collision vs bench_top |
| 0.775 | 0.805 | 0.005 | 0.10 | -0.0350 | FAIL | y-1 | openarm_body_link0_collision vs bench_top |
| 0.775 | 0.800 | 0.000 | 0.10 | -0.0400 | FAIL | y-1 | openarm_body_link0_collision vs bench_top |
| 0.775 | 1.105 | 0.305 | 0.16 | 0.1000 | PASS | nominal |  |
| 0.775 | 1.080 | 0.280 | 0.16 | 0.1000 | PASS | nominal |  |
| 0.775 | 1.055 | 0.255 | 0.16 | 0.0924 | PASS | pedestal-1 | openarm_left_left_finger_collision vs task_hotplate_top_plate |
| 0.775 | 1.030 | 0.230 | 0.16 | 0.0858 | PASS | pedestal-1 | openarm_left_left_finger_collision vs task_hotplate_top_plate |
| 0.775 | 1.005 | 0.205 | 0.16 | 0.0793 | PASS | pedestal-1 | openarm_left_left_finger_collision vs task_hotplate_top_plate |
| 0.775 | 0.980 | 0.180 | 0.16 | 0.0753 | PASS | pedestal-1 | openarm_left_left_finger_collision vs task_hotplate_top_plate |
| 0.775 | 0.955 | 0.155 | 0.16 | 0.0753 | PASS | pedestal-1 | openarm_left_left_finger_collision vs task_hotplate_top_plate |
| 0.775 | 0.930 | 0.130 | 0.16 | 0.0753 | PASS | pedestal-1 | openarm_left_left_finger_collision vs task_hotplate_top_plate |
| 0.775 | 0.905 | 0.105 | 0.16 | 0.0650 | PASS | y-1 | openarm_body_link0_collision vs bench_top |
| 0.775 | 0.880 | 0.080 | 0.16 | 0.0400 | PASS | y-1 | openarm_body_link0_collision vs bench_top |
| 0.775 | 0.855 | 0.055 | 0.16 | 0.0150 | PASS | y-1 | openarm_body_link0_collision vs bench_top |
| 0.775 | 0.830 | 0.030 | 0.16 | -0.0100 | FAIL | y-1 | openarm_body_link0_collision vs bench_top |
| 0.775 | 0.805 | 0.005 | 0.16 | -0.0350 | FAIL | y-1 | openarm_body_link0_collision vs bench_top |
| 0.775 | 0.800 | 0.000 | 0.16 | -0.0400 | FAIL | y-1 | openarm_body_link0_collision vs bench_top |
| 0.775 | 1.105 | 0.305 | 0.20 | 0.1000 | PASS | nominal |  |
| 0.775 | 1.080 | 0.280 | 0.20 | 0.1000 | PASS | nominal |  |
| 0.775 | 1.055 | 0.255 | 0.20 | 0.1000 | PASS | nominal |  |
| 0.775 | 1.030 | 0.230 | 0.20 | 0.1000 | PASS | nominal |  |
| 0.775 | 1.005 | 0.205 | 0.20 | 0.1000 | PASS | nominal |  |
| 0.775 | 0.980 | 0.180 | 0.20 | 0.1000 | PASS | nominal |  |
| 0.775 | 0.955 | 0.155 | 0.20 | 0.1000 | PASS | nominal |  |
| 0.775 | 0.930 | 0.130 | 0.20 | 0.0900 | PASS | y-1 | openarm_body_link0_collision vs bench_top |
| 0.775 | 0.905 | 0.105 | 0.20 | 0.0650 | PASS | y-1 | openarm_body_link0_collision vs bench_top |
| 0.775 | 0.880 | 0.080 | 0.20 | 0.0400 | PASS | y-1 | openarm_body_link0_collision vs bench_top |
| 0.775 | 0.855 | 0.055 | 0.20 | 0.0150 | PASS | y-1 | openarm_body_link0_collision vs bench_top |
| 0.775 | 0.830 | 0.030 | 0.20 | -0.0100 | FAIL | y-1 | openarm_body_link0_collision vs bench_top |
| 0.775 | 0.805 | 0.005 | 0.20 | -0.0350 | FAIL | y-1 | openarm_body_link0_collision vs bench_top |
| 0.775 | 0.800 | 0.000 | 0.20 | -0.0400 | FAIL | y-1 | openarm_body_link0_collision vs bench_top |

## 3. Boundary

| pedestal | last passing edge (m) | first failing edge (m) | min distance at last passing | part and geom at the minimum |
|---|---|---|---|---|
| 0.10 | 0.055 | 0.030 | 0.015 | openarm_body_link0_collision vs bench_top |
| 0.16 | 0.055 | 0.030 | 0.015 | openarm_body_link0_collision vs bench_top |
| 0.20 | 0.055 | 0.030 | 0.015 | openarm_body_link0_collision vs bench_top |

At the passing edges 0.080 and 0.105 m the closer minimum is the hotplate: openarm_left_left_finger_collision vs task_hotplate_top_plate at pedestal 0.16 (0.0753 and 0.065 m), i.e. the body is not the limit there.

Second x (0.70, pedestal 0.16): same boundary, last passing 0.055 m, first failing 0.030 m; link0 vs bench_top sets it.

## 4. Pick ik at nominal, the three deepest body-passing stances (pedestal 0.16, x 0.775)

| base y | edge (m) | pick task 1: passing bins | failing bins | failing point (all 12 bins) | point z (m) | result |
|---|---|---|---|---|---|---|
| 0.855 | 0.055 | 0 of 12 | 12 | point 0 'over' | 1.16 | FAIL |
| 0.880 | 0.080 | 0 of 12 | 12 | point 0 'over' | 1.16 | FAIL |
| 0.905 | 0.105 | 0 of 12 | 12 | point 0 'over' | 1.16 | FAIL |

The pick has 4 pick points in this task list; the failure is at the first one in every bin, so the later points were not reached.

## 5. Stance chosen for the run

None qualified: no stance with edge < 0.305 m passes body and pick ik at pedestal 0.16 m. The run was not made.

## 6. Not done, and why

- Step 3 (pick and place), step 4 (monitor, every 10th tick), step 5 (guard would-clamp count) and the phase table: not run, because no stance qualified.
- The guard stays disabled (not changed).
- The Nov 8 probe (data/logs/guardoff_probe*, scratch/in_move_probe.py) was not read: the rules forbid reading the real repo's data/logs and untracked scratch.

## 7. Wall time and free memory

| job | wall time | free memory before |
|---|---|---|
| body boundary sweep (39 + 13 checks, one setup) | 19.8 s (setup 4.8 s) | 2.6 GB |
| nominal pick ik for three stances (one setup) | 33.0 s (setup 31.0 s) | 2.6 GB |

## 8. Not found

- No body-passing stance with edge < 0.055 m at pedestal 0.16 (the sweep stops at 0.80 m; it fails at 0.030 m and below).
- No pick-ik-passing stance within the body-passing range.

## 9. Real repo

HEAD 9ff0099. No commit, no push. This report is the only file written in the real repo (untracked).
