# Grip force sweep on the grasp-and-lift (2026-10-15)

Question: which per-finger force holds the glass beaker (0.2 kg, weight 1.96 N) on the lift without slipping, across small perturbations? The grasp-and-lift is `experiments/grasp_lift_test.py` (close on the beaker, lift the hand about 9 cm, hold 4 s). Each run is a fresh simulator; every configuration is deterministic (repeats are bit-identical). Raw results: one JSON line per run in `data/logs/grip_sweep_20261015_184444.jsonl` (git-ignored); the harness was a scratch script.

Perturbations: battery mass 11.0, 11.3 and 11.6 kg (the stand-in for any small change of the setup) and the gripper-on-glass friction 0.4 (the configured value) and 0.32 (20 % lower). Slip = how far the beaker slid down in the hand (mm, at the end of the 4 s hold); creep = its sliding speed in the last second (mm/s). Held = slip under 3 mm and the beaker follows the hand. Static safety factor SF = 2 x force x friction / weight (both fingers' friction against the weight).

| friction | force N | SF | slip mm (11.0 / 11.3 / 11.6 kg) | creep mm/s (11.0 / 11.3 / 11.6 kg) | held |
|---|---|---|---|---|---|
| 0.4 | 5 | 2.04 | 2.51 / 4.68 / 4.26 | -0.002 / +0.063 / +0.032 | 1 of 3 |
| 0.4 | 6 | 2.45 | 1.66 / 1.62 / 1.49 | +0.017 / +0.020 / +0.024 | 3 of 3 |
| 0.4 | 7 | 2.85 | 0.83 / 0.85 / 0.85 | -0.007 / -0.008 / -0.005 | 3 of 3 |
| 0.4 | 8 | 3.26 | 0.12 / 0.14 / 0.25 | -0.003 / -0.002 / -0.003 | 3 of 3 |
| 0.4 | 9 | 3.67 | 0.01 / 0.01 / 0.12 | +0.007 / +0.009 / -0.022 | 3 of 3 |
| 0.32 | 5 | 1.63 | 28.12 / 30.94 / 24.24 | +5.391 / +6.041 / +4.061 | 0 of 3 |
| 0.32 | 6 | 1.96 | 16.24 / 4.29 / 4.41 | +4.394 / +0.458 / +0.531 | 0 of 3 |
| 0.32 | 7 | 2.28 | 7.86 / 8.10 / 8.03 | +1.607 / +1.599 / +1.608 | 0 of 3 |
| 0.32 | 8 | 2.61 | 0.47 / 0.62 / 0.53 | +0.002 / +0.001 / +0.003 | 3 of 3 |
| 0.32 | 9 | 2.94 | 0.24 / 0.15 / 0.25 | +0.013 / +0.039 / -0.001 | 3 of 3 |

Findings:

- At friction 0.4: 5 N is marginal (slips 2.5 mm without creep at 11.0 kg but 4.3 to 4.7 mm and creeping at 11.3 and 11.6 kg); 6 N holds (1.5 to 1.7 mm, creep about 0.02 mm/s); 7 N and up hold with little or no slip (0.8 mm at 7 N, 0.1 to 0.25 mm at 8 N).
- At friction 0.32 (a 20 % lower friction): 5 N loses the beaker (24 to 31 mm of slip, creeping 4 to 6 mm/s); 6 N and 7 N still slip (4 to 16 mm, creeping 0.5 to 4 mm/s; 7 N slips 8 mm and creeps 1.6 mm/s at every mass); 8 N holds in all three (0.5 to 0.6 mm, no creep) and 9 N more (0.15 to 0.25 mm).
- The failure is not predicted by the static SF alone: 7 N at friction 0.32 has SF 2.28 and fails, 8 N has 2.61 and holds; at friction 0.4, 5 N (SF 2.04) is marginal and 6 N (2.45) holds. The edge is about SF 2.3 to 2.5, so 8 N at friction 0.32 sits just above it.
- The finger normal forces sit within 0.15 N of the commanded force in every run (the no-slip solver splits them a little unevenly), so the cap binds at all forces (the finger ceiling is 10 N).
- Not tested: the pour (it tilts the beaker and adds load changes), faster lifts, friction below 0.32, other beakers or the real gripper.
