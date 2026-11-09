# Pedestal override removal: stopped at the start (per-check seed build not committed)

Task: remove BENCH_STANCE_CFG["pedestal_override_m"] (M197, M270) so the pedestal stance selection chooses is the one the full check and the robot use.

## Why it stopped

The task says: check git log first; if the per-check seed build is not committed, stop and say so, because both builds edit the stance code (src/stance_select.py, src/full_check.py).

- git log: the newest stance commits are the M281 and M282 logging commits (b6fe404, bac3385, 237ff7a..9ff0099). M281 (docs/mpvi_decisions.md row M281) states the generator is NOT reseeded per check.
- docs/reports/overnight_measurements_20261109.md and docs/progress.md (2026-11-09 12:04) list "reseed per check" as a decision for David. It is not built.
- git status: no uncommitted src/ or experiments/ changes.

So the per-check seed build is not committed, and no code was changed.

## Read-only findings (no edits)

Why the override was added (rows M197, M270; reports 20261028_1300 and 20261028_1600):
- Stance selection chose a pedestal of 0.10 m in the moved layout. M197 set 0.15 m so the staging parks at a higher pedestal.
- At 0.15 m the moved-layout episode succeeded with ramp 0.5625 s and compensation on (elbow clearance 180/176 mm at the place, against 155/153 mm at 0.10 m).
- At ramp 1.0 s with compensation on, both 0.10 m and 0.15 m fail the place DOWN step (3.9 mm and 3.0 mm from the waypoint, tolerance 3 mm).
- 0.20 m fails at PICK IN: the left elbow (joint 4) sits at its 10 deg margin and the pad stops 18.9 mm short.
- M270 (0.16 m): with the elbow guard, the zombie elbow clearance at 0.15 m is 0.4 mm above the minimum, too thin; 0.16 m gives 10.4 mm.
- The 2026-11-05 geometric study (20261105_1613) says elbow clearance is not the binding limit; reach is, and the valid stance range is 0.05 to 0.16 m.

## Where the override is read (src/stance_select.py:622, task_config.py:597)

- src/stance_select.py:622 reads cfg["pedestal_override_m"] (replaces st["q"] when set).
- src/task_config.py:597 defines it (0.16).

## What is needed before this task can run

David (or the seed build) must land first. Once the per-check reseed is committed, the override removal can proceed on top of it. Nothing was pushed.
