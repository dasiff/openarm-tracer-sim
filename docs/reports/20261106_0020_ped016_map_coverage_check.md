# Pedestal 0.16 m: do the existing reach maps cover it?

Question (urgent check while reach_map_ped016 runs): is 0.16 m already in the pedestal set the reach maps were built for?

## Findings

1. `TASK_CFG["pedestal_q"]` (src/task_config.py:85) is a single value, 0.1550 m (lift command above the collapsed pedestal). It is not a list or range. It is a reference stance for tests and map building, not used by the pipeline (row M079 area).
2. 0.16 m is not in it. The nearby pedestal lists do not contain 0.16 either:
   - `pedestal_qs` at src/task_config.py:281: 0.0, 0.0452, 0.1252, 0.2052, 0.2852, 0.3652, 0.418
   - `pedestal_qs` at src/task_config.py:315: 0.0452, 0.1252, 0.2052, 0.2852, 0.3652, 0.418
3. Result: 0.16 m is not covered by the config, so the existing maps do not cover it. The build is NOT cancelled.

## Action

- The tmux session `reach_map_ped016` is left running (log: data/logs/reach_map_build_ped016_20261105_235715.log, at wave 21 when checked, about 2.5 GB RSS).
- Output goes to data/reach_map_ped016.

## Open question for David

- The 0.16 m value is not recorded in the config. Is 0.16 meant as a new reference height (it sits between 0.1252 and 0.2052 in pedestal_qs), or as a rounding of the 0.1550 reference? This is a design-level choice, so it is not decided here.
