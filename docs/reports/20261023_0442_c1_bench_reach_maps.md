# C1: reach maps rebuilt with the bench in the collision filter

Build: 24 maps, 92.1 min, log `docs/reports/logs/reach_map_bench_build_20261023_025339.log`; old maps kept in `data/reach_map_before_bench`, new ones are `data/reach_map`. Rows M170, M171.

Voxels (5 mm) reached per map, old vs new. The change mixes three things: the neck removal (step 2), the camera box and the bench. The no-bench pilot (left bins 0 and 9, built with the same code and `--no-bench`) separates them:

| map | old | no bench (pilot) | new (bench) | bench effect | neck + camera effect |
|---|---|---|---|---|---|
| left 0 | 1,428,135 | 1,487,976 | 1,456,610 | -2.11 % | +4.19 % |
| left 9 | 1,221,013 | 1,308,399 | 1,304,846 | -0.27 % | +7.16 % |

All 24, old to new (the gains are mostly the neck removal):

| arm | bin (centre deg) | old | new | change |
|---|---|---|---|---|
| left | 0 (0) | 1,428,135 | 1,456,610 | +28,475 (+2.0 %) |
| left | 1 (30) | 1,486,424 | 1,469,919 | -16,505 (-1.1 %) |
| left | 2 (60) | 1,381,366 | 1,371,152 | -10,214 (-0.7 %) |
| left | 3 (90) | 1,282,706 | 1,280,313 | -2,393 (-0.2 %) |
| left | 4 (120) | 1,244,863 | 1,244,863 | +0 (+0.0 %) |
| left | 5 (150) | 1,319,926 | 1,319,766 | -160 (-0.0 %) |
| left | 6 (180) | 1,380,362 | 1,390,155 | +9,793 (+0.7 %) |
| left | 7 (210) | 1,350,445 | 1,397,320 | +46,875 (+3.5 %) |
| left | 8 (240) | 1,242,255 | 1,317,331 | +75,076 (+6.0 %) |
| left | 9 (270) | 1,221,013 | 1,304,846 | +83,833 (+6.9 %) |
| left | 10 (300) | 1,229,589 | 1,300,031 | +70,442 (+5.7 %) |
| left | 11 (330) | 1,312,981 | 1,375,437 | +62,456 (+4.8 %) |
| **left** | **total** | **15,880,065** | **16,227,743** | **+347,678 (+2.2 %)** |
| right | 0 (0) | 1,438,372 | 1,462,330 | +23,958 (+1.7 %) |
| right | 1 (30) | 1,314,844 | 1,375,592 | +60,748 (+4.6 %) |
| right | 2 (60) | 1,257,527 | 1,326,209 | +68,682 (+5.5 %) |
| right | 3 (90) | 1,220,672 | 1,303,713 | +83,041 (+6.8 %) |
| right | 4 (120) | 1,224,504 | 1,291,919 | +67,415 (+5.5 %) |
| right | 5 (150) | 1,335,688 | 1,384,634 | +48,946 (+3.7 %) |
| right | 6 (180) | 1,394,996 | 1,405,712 | +10,716 (+0.8 %) |
| right | 7 (210) | 1,327,838 | 1,327,838 | +0 (+0.0 %) |
| right | 8 (240) | 1,287,659 | 1,287,659 | +0 (+0.0 %) |
| right | 9 (270) | 1,296,273 | 1,292,553 | -3,720 (-0.3 %) |
| right | 10 (300) | 1,376,649 | 1,362,715 | -13,934 (-1.0 %) |
| right | 11 (330) | 1,460,724 | 1,438,024 | -22,700 (-1.6 %) |
| **right** | **total** | **15,935,746** | **16,258,898** | **+323,152 (+2.0 %)** |

No arm lost voxels in total (left +2.2 %, right +2.0 %). The maps that lost voxels: left 1, 2, 3 (-1.1, -0.7, -0.2 %) and right 9, 10, 11 (-0.3, -1.0, -1.6 %): the directions that point at the bench from the arm's far side.

## Validation

TASK_CFG stance (0.765, 1.085, yaw -90 deg) at pedestal 0.155, left arm, the 12 task points (`experiments/c1_validate_stance.py`): **12 of 12 reachable** on the new maps and on the old (grasp points bins 0, 1, 11; set-down and above bins 0, 10, 11). No stop needed.

## To flag

- The bench is placed for the TASK_CFG stance; the maps are used at other stances too (C2), where the bench sits elsewhere in the shoulder frame. The pilot shows the bench removes at most about 2 % of a map, so the error from this is small, but it is an approximation.
- Pilot output `data/reach_map_nobench_pilot` (2 maps) can be deleted.
