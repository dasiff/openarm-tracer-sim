# Held-beaker contact log at two stances; moved-objects episode

Written for: David. 2026-10-25 05:00. No algorithm change; default stays `task_cfg`; no live Gemini calls. One heavy job at a time, tmux, `systemd-run --user --scope -p MemoryMax=6G`, `/usr/bin/time -v` for the peak (script `experiments/run_capped.sh`). Nothing pushed.

## Memory per job (free -h before each: 7.3 Gi total, 5.4 Gi available, swap 4 Gi unused)
| job | log | wall | peak RSS |
|---|---|---|---|
| held beaker, TASK_CFG's stance | data/logs/held_beaker_task_cfg_20261025_043502.log | 2:18 | 2.15 GB |
| held beaker, overlay's pick | data/logs/held_beaker_bench_region_20261025_043739.log | 3:08 | 3.67 GB |
| moved layout test | data/logs/stance_select_moved_20261025_044142.log | 4:13 | 4.08 GB |
| moved layout diagnosis | data/logs/moved_place_diag_20261025_044640.log | 2:47 | 3.65 GB |
No job passed 5 GB.

## 1. Held-beaker contacts, place phase (segment 4/4), both stances side by side
Script `experiments/held_beaker_contacts.py` (a test source on the pipeline episode: every contact of the beaker's geoms with pair, first and last time, largest normal force; the smallest distance of the beaker's geoms to the hotplate's and the fingers' geoms). Both episodes: SUCCESS, beaker on the hotplate. The full logs: docs/reports/logs/held_beaker_contacts_{task_cfg,bench_region}.json; the recordings are kept in data/held_beaker/{task_cfg,bench_region}/ (git ignored).
| | TASK_CFG (0.765, 1.085, pedestal 0.155) | overlay's pick (0.755, 1.145, pedestal 0.10) |
|---|---|---|
| Beaker held by the fingers (wall_0 against the right finger, wall_12 against the left finger; wall_1, 13, 23 briefly) | 60.15 to 64.99 s, largest force 8.3 N (right) and 7.9 N (left) | 60.45 to 65.89 s, 8.2 N (right) and 7.7 N (left) |
| Beaker bottom against the hotplate top plate (the intended set-down) | first contact 63.41 s, while still gripped (DOWN), largest 2.8 N; stays to the end | first contact 64.23 s, while still gripped, largest 2.9 N; stays to the end |
| Beaker against anything else in the place phase (bench, hotplate base, other geoms) | none | none |
| Closest to the hotplate in OVER (the transit, before the set-down) | 27.4 mm (beaker bottom over the plate; the plan's carry gap is 40 mm) | 26.3 mm |
| Closest to the hotplate in DOWN / OPEN | -0.8 / -0.9 mm (plate contact) | -0.7 / -1.2 mm |
| Closest to the fingers: OVER, DOWN | 0 (held) | 0 (held) |
| Fingers after the release: OPEN, BACK, HOME | 0.8, 4.5, 18.7 mm | 0, 3.9, 25.5 mm |
The place is the same at both stances to within 1 to 2 mm and 0.1 N; the beaker touches only the fingers and the plate. The beaker's bottom is pressed on the plate for about 1.6 s before the fingers open (2.8 / 2.9 N). The pick phase (not asked) had the beaker's bottom on the bench until the lift; the beaker rose 107 mm. The full check was not changed; it still does not model the held beaker (C0 a), so these two episodes are the only evidence for the place.

## 2. Moved layout (beaker and hotplate 15 cm toward smaller x)
Direction: the bench legs stand at x = +-2.0 and the objects at x = 0.8 (hotplate) and 1.0 (beaker), so the nearest legs are the ones at +2.0, and "farther from the legs" is toward smaller x: beaker (0.85, 0.66), hotplate (0.65, 0.66), spacing 20 cm as before.
Change: `apply_stance_selection` now passes `task_cfg` to `FullCheck` (src/stance_select.py:373; before, the full check's scene and the context's model kept the default layout). One diagnostic addition to the run report: `overlay_run_q` (the overlay's best spot's pedestal run). New test `experiments/stance_select_moved_test.py` (in the suite list).
Result (docs/reports/logs/stance_select_moved_report.json, log data/logs/stance_select_moved_20261025_044142.log), 6 of 9 checks:
- Full check's scene at the moved positions: PASS.
- Stance chosen: (0.635, 1.135), pedestal 0.10, which is 139 mm from TASK_CFG's stance (130 mm in x, 50 mm in y); the objects moved 150 mm in x. Region counts beaker 2 / hotplate 2; the pedestal run of the pick 0.05 to 0.16 (12 cm, the lower end is the bottom of the searched range, as before); score (2, 0.12, 2, 0.06, 0.385).
- Full check on the chosen stance: PASS (first ranked stance, one full check). Region and full check agree on this stance (counts above zero for both, first ranked passed).
- Navigation: arrival 3.1 mm / 0.19 deg, handoff 3.9 mm; the second navigation step null. Selection took 69.7 s.
- Pick: SUCCESS (the beaker rose 107 mm).
- **Episode: PLAN_FAILED at segment 4/4, the place, first waypoint OVER:** "OVER: not reached after 11.0 s: 12.2 mm and 0.8 deg from the waypoint"; the retry's recovery then failed at HOME ("the arm did not settle at home after 9.0 s", 0.030 rad), no retry for a recovery failure. The beaker was left in the hand, not on the hotplate. The replay of the failed run is bit-identical.
Diagnosis (experiments/moved_place_diag.py, docs/reports/logs/moved_place_diag.json; same run, nothing changed): in the place OVER the left arm's joint margins to its limits stayed at least 0.17 rad (joint 2, q 0.006 against the upper limit 0.1745), so it is not a joint limit; the monitor's nearest distances were the right arm 66.5 mm (the avoidance limit is 40 mm), the bench 116 mm (limit 8 mm), the torso 88 mm; no near collision. The arm's commanded joints stop moving and the pad stays 12 mm short. NOT established: whether the target is at the edge of the arm's reach (the place's OVER waypoint is at the carry height, about 9 cm above the plate, with the pad rotation of the hold; the full check solves the band heights, the "above" 10 cm higher, the standoff, over and lift of the object points, but not this waypoint or the held beaker) or whether the hand's avoidance of the hotplate (8 mm) holds it (the monitor's hotplate group includes the beaker, so it cannot show it). A split would need one more run that logs the hand to hotplate distance without the beaker and the IK's own error at the waypoint.
No setting was tuned.

## Questions for David
1. Add the place's carry-height OVER waypoint (and, with C0 a, the held beaker) to the full check's path points? That is the missing coverage that this failure points at, but not proven.
2. Run the split diagnosis (hand to hotplate distance and IK error at the waypoint), one job of about 3 minutes?
