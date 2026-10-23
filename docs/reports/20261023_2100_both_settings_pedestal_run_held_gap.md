# Both stance settings in the suite; pedestal run, held-beaker gap, moved-objects episode (read only)

Written for: David. 2026-10-23 21:00. Stance default stays `task_cfg`. Nothing pushed.

## Both settings (code: commit 2df1d90)
`mpvi_pipeline_test` and `stance_select_pipeline_test` take `--stance-mode {task_cfg,bench_region}`; `experiments/run_tests.sh` runs each under both. Under `task_cfg` the selection checks of the stance-select test are left out (the episode, arrival, replay checks stay). Summary `data/logs/suite_i2_20261023_201334_summary.txt` (the first run, i_20261023_153934, was lost to a VM restart), all exit 0:
- stance_select_pipeline_test, bench_region: 8/8 (replay bit-identical)
- stance_select_pipeline_test, task_cfg: 5/5
- mpvi_pipeline_test, task_cfg: 14/14
- mpvi_pipeline_test, bench_region: 14/14

## (1) Pedestal run at the overlay's pick (0.755, 1.145) against TASK_CFG's stance (from the F report and data/stance_select_report.json)
| | passing heights | width | pedestal used |
|---|---|---|---|
| overlay's pick | 0.05 to 0.16 | 12 cm | 0.10 |
| TASK_CFG (0.765, 1.085) | 0.07 to 0.16 | 10 cm | 0.155 (new rule's middle 0.11) |
0.10 is the middle of its run, not the edge. The run's lower end, 0.05, is the lowest height searched (`BENCH_STANCE_CFG["pedestal_range"]` = (0.05, 0.25)), so the true run may be wider below; the 0.16 end is a real limit. TASK_CFG's 0.155 is at the top edge of its run.

## (2) Held-beaker gap (C0 a)
Not closed anywhere: `src/full_check.py` has no held or attached object; the IK path check solves the task points only and leaves the beaker out of the world geometry. At the overlay's pick the full check passes (home clearance 94 to 118 mm, 9 IK stances solved) but says nothing about the held beaker. The only episode evidence is the pipeline test: SUCCESS, arrival 3.1 mm / 0.19 deg, beaker on the hotplate, replay identical. There is no contact log: the test deletes its episode directory. A kept run (one sim job) would give the place's contacts.

## (3) A moved-objects layout through a full episode under bench_region
- Copy TASK_CFG with new `beaker_xy` and `hotplate_xy`; pass it to `make_pipeline_task(task_cfg=...)` and `load_task(cfg)`. The bench top is 4.4 m long, so shifting along it fits.
- Needed code change (one line): `apply_stance_selection` (src/stance_select.py:373) builds `FullCheck(maps=...)` without `task_cfg`, so the full check's scene and the context's model keep the default layout. `FullCheck` already accepts `task_cfg`.
- The selection's task points come from the world model (`step_target`), so they follow the moved objects; `src/task_points.task_points()` reads TASK_CFG directly, which only matters to the tests and scripts that call it.
- A new test with the checks of stance_select_pipeline_test, one episode (about 4 min). The reach maps are object independent; the region is cached per object depth.
Not run.
