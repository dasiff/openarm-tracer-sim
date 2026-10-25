# Parts 1 and 2 of the three-part task: Part 1 done, Part 2 stopped at the shape check (2026-10-25)

Written for David. Part 3 is not started: it depends on Part 2's carry check (the moved-layout episode picks a run of two steps, so the carry check runs in it).

## Summary
- **Part 1 done.** Tag `before-task-cfg-removal` and the branch are pushed (commit `5b5dd82948eda0e1f7d3ee4de99a2f61ed3720f7`, confirmed on origin). The old "task_cfg" stance mode is deleted. `nav_pipeline_follow_test` runs again: 14/14. M175's H line is reworded; M178 has a pointer; the deletion is row M179.
- **Part 2 stopped, as you instructed.** I measured the carry slice against the two recorded place episodes: **parts of the arm go outside the slice** (table below). I did not adjust the shape. The slice code is written and committed as work in progress; `transit_clearance_test` is not updated, no M180 row, no M176 note.
- **Not started:** Part 3 (all six steps).

## Part 1
### Audit of every use of the task_cfg mode and of TASK_CFG's stance (step 2a)
| Group | Where | What I did |
|---|---|---|
| Pipeline mode: delete | `BENCH_STANCE_CFG["mode"]` and its comment (src/task_config.py); the `stance_mode` argument and the "task_cfg" branch of `make_pipeline_task` (src/mpvi_pipeline.py); the `--stance-mode` option of `mpvi_pipeline_test`, `stance_select_pipeline_test` and `held_beaker_contacts`; the "default mode is task_cfg" checks in `stance_select_test` and `stance_select_pipeline_test`; the report field `mode`; suite runs `stance_select_pipeline_test_task_cfg` and `mpvi_pipeline_test_bench_region` | deleted |
| Test reference: keep | TASK_CFG's `staging_xy`, `staging_yaw`, `pedestal_q` in full_check_test, transit_clearance_test, nav_follow_test, furniture_map_test, arm_sag_test, the held-beaker script, f4/h/g0/bench_stance diagnostics, stance_select_test and stance_select_pipeline_test (distance from TASK_CFG's) | kept; the three values carry the comment "reference stance for tests and map building, not used by the pipeline" |
| Reach map building: keep | `reach_map_build.py` (the bench position the maps were built with), the camera reference stance in `StanceContext` | kept |

### Navigation off (step 2b)
Navigation off is not the deleted mode. It is part of the region selection (`select_bench_stance`): the robot stays at TASK_CFG's stance; a run that stance serves is kept; anything else stops with "needs navigation on". No pipeline run uses it today (only `stance_select_test` section 6 and, now, the held-beaker script's `--at-reference-stance` run). Every pipeline run therefore still has a way to choose a stance, so I went ahead. One consequence you may want to know: with navigation off the pipeline still parks at TASK_CFG's stance, so the comment on those three values says "not used by the pipeline" while that one path still reads them (the comment on `BENCH_STANCE_CFG` and row M179 say so).

### Results
- `nav_pipeline_follow_test`: 14/14, SUCCESS (log data/logs/nav_pipeline_follow_test_20261025_160142.log). Its startup crash is gone because the option it lacked no longer exists.
- Not rerun after the deletion: the other tests. They run in the full suite in Part 3, step 4.

### Decisions log rows (word for word)
| M175 | 2026-10-23 | C3, H, H2 (David's addenda; row written 2026-10-25, it was missing). `BENCH_STANCE_CFG["mode"]` "task_cfg" (default: today's stance, nothing changes) or "bench_region" (`src/stance_select.py`: the longest run k = min(4, ..) .. 1 of consecutive manipulation steps on one surface gets the overlay's stance; the second navigation step of a run is null; a run the current stance serves needs no new stance; "needs navigation on" stop when navigation is off; the full check runs on the chosen stance and the next 3 ranked, a stop names the criterion). H: support objects under the target (the hotplate for a place) ARE checked against the zombie pose clearance (40 mm, the arms at home), at plan time, by `support_ok` (`src/bench_stance.py`), which `overlay` runs on every call for the entries under the target: for each pedestal height and floor spot that already has an approach count, it rejects the spot when the zombie pose's arms come within the clearance of a support object. The support objects are NOT in the precomputed, cached region (its home-clearance map holds the furniture only, so the cache does not depend on where the hotplate stands); they are checked by `support_ok` instead, after the region is read. H2: the creep-in path (straight, `staging_approach_back_m`, +-1 cm / 1 deg) is swept in the cached region. The measured-rectangle code and `BENCH_RECT_CFG` are deleted (design plan keeps the record). Both settings pass `mpvi_pipeline_test` and `stance_select_pipeline_test` (suite runs both). `rank_keep` 4 and `rank_min_distance_m` 0.10 are the ranked-stance settings. Default superseded by M178; task_cfg mode deleted by M179. | David 2026-10-23 |
| M178 | 2026-10-25 | Default stance method flipped to "bench_region" (David's decision, Oct 25, after the held-beaker contact results and the moved-layout results). `BENCH_STANCE_CFG["mode"]` is now "bench_region"; "task_cfg" stays selectable (valid only for the test scene's layout) and the suite keeps running both settings (stance_select_pipeline_test_task_cfg, mpvi_pipeline_test_bench_region and the task_cfg-explicit runs). Only the default value and its plain-word comment changed. Known open issue: the moved-layout place fails at OVER (12.2 mm short), cause under diagnosis; the overnight measurements (docs/reports/*_overnight.md, step 2) show the arm's commands stopping with joint 2 and joint 6 on the solver's 10 degree joint margin and the hotplate avoidance inactive (hand 57 mm away), which is a reading, not a decision. Note: the task_cfg mode this row kept selectable was deleted the same day, see M179. | David 2026-10-25 |
| M179 | 2026-10-25 | The old stance method ("task_cfg" mode: park at TASK_CFG's stance) is deleted (David's decision, Oct 25: it is no longer tested or used). Tag `before-task-cfg-removal` (commit 5b5dd82) marks the code before the deletion. Deleted: `BENCH_STANCE_CFG["mode"]` (no mode setting is left, only one method remains), the `stance_mode` argument of `make_pipeline_task` (it now always selects by the bench region), the `--stance-mode` option of `mpvi_pipeline_test`, `stance_select_pipeline_test` and `held_beaker_contacts`, the two old-method suite runs (`stance_select_pipeline_test_task_cfg`, `mpvi_pipeline_test_bench_region`), the "default mode is task_cfg" checks and the `rep["mode"]` report field. Kept, with the comment "reference stance for tests and map building, not used by the pipeline": TASK_CFG's `staging_xy`, `staging_yaw` and `pedestal_q`, used by the reference tests (full_check_test, transit_clearance_test, nav_follow_test, the held-beaker script now with `--at-reference-stance`, etc.) and by the reach map build (`reach_map_build.py`, the bench position the maps were built with). Navigation off (`BENCH_STANCE_CFG["navigation"]` False) is not the deleted mode: it is part of the region selection (`select_bench_stance`), the robot stays at TASK_CFG's stance, a run that stance serves is kept and anything else stops with "needs navigation on"; unchanged, and no pipeline run uses it today (only `stance_select_test` section 6 and the held-beaker script's reference run). `nav_pipeline_follow_test`'s startup crash (missing `stance_mode`) is gone with the option. | David 2026-10-25 |

## Part 2: measurement of the carry slice (stopped)
Script `experiments/carry_slice_check.py` (measurement only), log data/logs/carry_slice_check_20261025_161037.log, numbers docs/reports/logs/carry_slice_check.json. It replays the two recorded place episodes of data/held_beaker/ (no new episode; both replays stay bit-identical up to the end of the window). Window: from the pick's lift (UP) to the end of the place's first waypoint (OVER), the beaker in the hand. The slice: apex at the left arm's shoulder (the shoulder moves under 0.01 mm during the carry), edges to the beaker's position at the pick and at the set-down, arc of the larger shoulder-to-hand distance. Distances below are the largest distance outside the slice itself, without the margin (the selection adds 90 mm: `obstacle_margin_m` 30 mm + `arm_radius_m` 60 mm).

| Largest distance outside the slice, mm | TASK_CFG stance | Overlay's pick |
|---|---|---|
| Hand (pad centre) | 1.2 | 4.5 |
| Beaker bottom, centre | 10.3 | 9.9 |
| Beaker bottom, outer rim (35 mm from its centre) | 45.2 | 44.9 |
| Forearm (the elbow to wrist line) | 105.4 | 86.6 |

Hand path:
| | TASK_CFG stance | Overlay's pick |
|---|---|---|
| Distance from the shoulder at the grasp / at the set-down, mm | 452 / 433 | 512 / 492 |
| Smallest / largest distance from the shoulder during the carry, mm | 419 / 453 | 480 / 513 |
| Largest distance from the straight line between the two points, mm | 13.6 | 10.9 |

Reading (an interpretation): the hand path is **a straight line, not an arc around the shoulder**. The distance from the shoulder dips by 13 to 14 mm in the middle (419 against 433 to 452 at the ends), which is what a straight chord does, and the hand stays within 14 mm of the chord. The slice's arc (radius equal to the larger distance) therefore lies outside the hand path and does not bulge to the hand. What goes outside the slice is small for the hand and the beaker's centre (1 to 10 mm), and larger for the forearm (87 to 105 mm), because the elbow swings out to the side of the shoulder-to-hand wedge. With the 90 mm margin everything is inside the widened slice except the forearm's centre line at the TASK_CFG stance (105 mm, 15 mm beyond the margin); that is not a claim about the arm's body, which is thicker than its centre line.

Per your instruction I stopped here and did not adjust the shape.

## State of the code (committed as work in progress, not pushed)
- `src/stance_select.py` now has `slice_distance`, `shoulder_xy`, `carry_bottom_z`, `carry_blocked` and the per-ranked-stance carry check in the run search (a stance with an obstacle in its slice is dropped; if none of the ranked stances is left, k is rejected and the search drops to k-1; the same check at the current stance when no navigation is needed). The old line check `transit_blocked` is gone, so `experiments/transit_clearance_test.py` (which imports it) will not run until it is updated. It is not updated.
- One design choice of mine, to confirm: because the slice depends on the stance, "blocked: reject k" is applied as "every ranked stance (the best and the next three) is blocked". The time the check adds is in the selection report (`carry_check_seconds`), not yet measured.

## Questions for David
1. The slice as you specified leaves the forearm outside it by up to 105 mm (the hand and the beaker by under 11 mm). Options: leave it, since the 90 mm margin covers most of it; widen the margin for the forearm; or define the slice's apex or edges differently. I have not chosen.
2. Do you want me to carry on with the transit_clearance_test update and Part 3 with the slice as it is, or wait for your decision on question 1?
