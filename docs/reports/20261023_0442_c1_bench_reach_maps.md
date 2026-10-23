# C1: reach maps rebuilt with the bench in the collision filter

Build: 24 maps, 92.1 min, log `docs/reports/logs/reach_map_bench_build_20261023_025339.log`; old maps kept in `data/reach_map_before_bench`, new ones are `data/reach_map`. Rows M170, M171.

Voxels (5 mm) reached per map, old vs new. The change mixes three things: neck removal (step 2), the camera box and the bench. The no-bench pilot (left bins 0 and 9) separates them:

| bin | old | no bench (pilot) | new (bench) | bench effect | neck+camera effect |
|---|---|---|---|---|---|
| left 0 | 196547 | 203853 | 200774 | -1.51 % | +3.72 % |
| left 9 | 172152 | 182723 | 182243 | -0.26 % | +6.14 % |

All 24, old to new (a gain is mostly the neck removal; no arm lost voxels overall):

| arm | bin (centre deg) | old | new | change |
|---|---|---|---|---|
| left | 0 (0) | 196547 | 200774 | +4227 (+2.2 %) |
| left | 1 (30) | 202613 | 201187 | -1426 (-0.7 %) |
| left | 2 (60) | 188184 | 187137 | -1047 (-0.6 %) |
| left | 3 (90) | 174322 | 174020 | -302 (-0.2 %) |
| left | 4 (120) | 170969 | 170969 | +0 (+0.0 %) |
| left | 5 (150) | 180845 | 180830 | -15 (-0.0 %) |
| left | 6 (180) | 188508 | 190584 | +2076 (+1.1 %) |
| left | 7 (210) | 185764 | 192972 | +7208 (+3.9 %) |
| left | 8 (240) | 173229 | 182501 | +9272 (+5.4 %) |
| left | 9 (270) | 172152 | 182243 | +10091 (+5.9 %) |
| left | 10 (300) | 173544 | 181683 | +8139 (+4.7 %) |
| left | 11 (330) | 183941 | 192172 | +8231 (+4.5 %) |
| **left** | **total** | **2190618** | **2237072** | **+46454 (+2.1 %)** |
| right | 0 (0) | 197568 | 201658 | +4090 (+2.1 %) |
| right | 1 (30) | 183568 | 192052 | +8484 (+4.6 %) |
| right | 2 (60) | 177587 | 185958 | +8371 (+4.7 %) |
| right | 3 (90) | 171853 | 181443 | +9590 (+5.6 %) |
| right | 4 (120) | 170531 | 179151 | +8620 (+5.1 %) |
| right | 5 (150) | 183873 | 191380 | +7507 (+4.1 %) |
| right | 6 (180) | 190782 | 192927 | +2145 (+1.1 %) |
| right | 7 (210) | 182159 | 182159 | +0 (+0.0 %) |
| right | 8 (240) | 177247 | 177247 | +0 (+0.0 %) |
| right | 9 (270) | 176763 | 176106 | -657 (-0.4 %) |
| right | 10 (300) | 187508 | 185793 | -1715 (-0.9 %) |
| right | 11 (330) | 198607 | 196531 | -2076 (-1.0 %) |
| **right** | **total** | **2198046** | **2242405** | **+44359 (+2.0 %)** |

Largest losses (new < old): left 1 (-0.7 %), left 2 (-0.6 %), right 9, 10, 11 (-0.4 to -1.0 %); every other map gained or is unchanged (left 4, 5 and right 7, 8 equal to the voxel, so the bench never touched them).

## Validation

TASK_CFG stance (0.765, 1.085, yaw -90 deg) at pedestal 0.155, left arm, the 12 task points (`experiments/c1_validate_stance.py`): **12 of 12 reachable** on the new maps and on the old (grasp points bins 0, 1, 11; set-down and above bins 0, 10, 11). No stop needed.

## To flag

- The bench is placed for the TASK_CFG stance; maps are used at other stances too (C2), where the bench sits elsewhere in the shoulder frame. The pilot shows the bench removes at most 1.5 % of a map, so the error from this is small, but it is an approximation.
- Pilot output `data/reach_map_nobench_pilot` (2 maps) can be deleted.
