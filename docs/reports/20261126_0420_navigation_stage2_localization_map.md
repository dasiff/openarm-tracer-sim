# Navigation stage 2 of 8: the localization map, its distance grid, and housekeeping A1 to A4

Date: 2026-11-26, about 04:20. Written for David (decider) and for whoever builds the particle filter (stage 4).
Design: docs/design_plan.md section 13.1 (the two-maps rule and the stage order). Decisions: M342 to M345 (new), M343 (seed), M344 (scripts), M345 (map build choices).

## Result in one paragraph

The localization map is built and cached: a 628 x 628 grid of 2 cm cells (12.56 m square) holding the lidar-height slice of the scene (107 fixed geoms), its exact distance-lookup grid (uncapped), and a free-floor mask reused from the planner grid. Build takes 0.28 s; a cached load takes 0.12 s; the cache file is 0.20 MB. Four of six checks pass. Two do not, and both need a decision from David: (3) predicted lidar ranges agree with the real scan on 97.76 % of usable beams, not on all of them; (4) the planner grid says the staging stance and the approach spot are not floor, because the bench top's outline blocks the chassis, which runs under the bench. Housekeeping A1 to A4 is done (commits 113a8e2, f958ee9, d37c6bb; A4 changes nothing).

## Part A, housekeeping

- **A1, build order** (commit 113a8e2; M342; M339 marked updated by M340 and M342). Order recorded in design plan 13.1: 1 lidar (built), 2 localization map, 3 odometry, 4 particle filter, 5 self-orientation, 6 driving on the estimated pose, 7 hotplate tracker, 8 unknown-object search (GroundingDINO detection plus frontier exploration), before VLA training. The PENDING place of M339 is marked resolved by M340 (navigation built first) and M342. Design plan section 9 carries the same note.
- **A2, run-record seed** (commit f958ee9; M343). RunRecorder writes the run's seed to meta.json as "seed" (it was always null) and "seed_given". The control loop passes cfg["run_seed"] when set; otherwise a seed is drawn at the start of the run. The physics draws no random numbers, so nothing in the physics or the recorded hashes changes today.
  - What reads meta.json: src/replay.py (named keys: model, robot_specs, source_hz, model_sha256, mujoco, machine, steps_per_tick, loop_cfg, labels, task, mode, handoff_label), src/run_record.py load_recording (complete, n_ticks, labels), experiments/demo_video.py:59 (named keys), src/task_beaker_hotplate.py:450 (robot_specs). None compares the seed or the whole dict, so nothing needed adjusting.
  - Manual baseline after the change: byte-identical (SHA-256 3060ae41cc8518e65cad011c6f48e25c273d8a105609c63e5c41a42f6c4c9b82 = experiments/baselines/manual_reference_20261103.csv; compare max difference 0.000000 on every column). Log: data/logs/manual_baseline_a2_20261126_041449.log.
- **A3, obsolete scripts** (commit d37c6bb; M344). Deleted: experiments/beaker_hotplate_pose_check.py, home_pose_candidates.py, home_pose_compare.py, home_pose_search.py, home_pose_coverage.py.
  - What referenced them: only the five scripts themselves (home_pose_coverage imports home_pose_candidates and home_pose_compare; home_pose_search imports home_pose_candidates; home_pose_compare imports home_pose_search and home_pose_candidates). No source file, no run_tests.sh entry, no other experiment imported or ran them.
  - Kept as history: documents (design plan 3.5 and 3.4, decisions M162, several reports) and four comments in src/task_config.py (lines 17, 72, 73, 91) that name home_pose_search.py as where the staging stance came from. Config values unchanged. Baseline data under experiments/baselines/held_200g_20261104_m258 untouched.
- **A4, observation slots** (no change; nothing built). The older decision whose condition was "lidar confirmed" is Stage 4 (observation slots, M152), deferred on 2026-10-22; the condition was added by M163. Text, word for word:

```
| M152 | 2026-10-22 | Stage 4 of the overnight queue (observation slots: N10, Q7, R6 merge rules, stop-replan-verify-go on a new obstacle, the fake-obstacle test, the test scene variant with a fixture obstacle) is DEFERRED by David: there is no sensor yet and the eagle camera's depth is unknown (design plan 11). The navigation core and the follower are built so that a changed obstacle list raises `WorldModel.obstacle_version` and a replan can use it; the world model's `observe_obstacle` / `observe_free` are not built | addition (deferred) | not in the paper | n/a | not built | David 2026-10-22 | design plan 8 (navigation), 11 |
```
(docs/mpvi_decisions.md line 164, M152)

```
| M163 | 2026-10-22 | [UPDATED by M336 and M337, 2026-11-25: the lidar is confirmed by David (M336; model and specs still to be confirmed with the hardware team); the map is made once beforehand, from one slam_toolbox mapping drive on the real robot and not during tasks (M337); the text below is history] A6. A lidar is likely on the real robot (David). Plan: lidar for floor obstacles and localization (SLAM once with the lidar, for example slam_toolbox, then localization against that map); the camera's depth for objects on work surfaces. Stage 4 (observation slots, M152) stays deferred until the lidar is confirmed. Updates R5 (M143: its camera corrections become lidar localization), design plan 12 and 13 | addition (plan) | not in the paper | n/a | lidar + camera depth | David 2026-10-22 | design plan 11, 12, 13; M143, M152 |
```
(docs/mpvi_decisions.md line 175, M163; this row is marked updated by M336 and M337, old text kept)

```
- **[DEFERRED 2026-10-22, David, M152]** Observation slots (N10, Q7, R6 merge rules; replan on a new obstacle; the fixture-obstacle test scene): not built until a sensor exists and the eagle camera's depth is known. Built so far for it: `WorldModel.obstacles()` and `obstacle_version`, the grid's version (no replan when nothing changed), the follower's stop-replan-go on a deviation.
```
(docs/design_plan.md line 286, §8 Navigation)

```
  - **[UPDATED 2026-10-22, David, M163; LIDAR CONFIRMED 2026-11-25, David, M336: the robot has a lidar; model and specs to be confirmed with the hardware team; the map is made once beforehand, M337]** (history) A lidar is likely on the real robot: lidar for floor obstacles and localization (SLAM once with the lidar, for example slam_toolbox, then localization against that map); the camera's depth for objects on work surfaces. Stage 4 (observation slots, M152) stays deferred until the lidar is confirmed. The camera corrections named below become lidar localization.
```
(docs/design_plan.md line 455, §12 Robot position)

```
- **[UPDATED 2026-10-22, M163]** A lidar is likely: lidar for floor obstacles and localization (SLAM once, e.g. slam_toolbox, then localization), camera depth for objects on work surfaces; Stage 4 stays deferred until the lidar is confirmed (the lidar is now confirmed, M336; whether to build Stage 4 is not decided: needs David).
```
(docs/design_plan.md line 497, §13)

The condition "lidar confirmed" is met since 2026-11-25 (M336). Whether to build Stage 4 is not decided: it needs David. Nothing here was changed.

## Part B, the localization map

### The two maps (design plan 13.1)

- The planner grid (src/nav_grid.py NavGrid, 5 cm cells, A* in src/nav_planner.py) stays exactly as it is. It is built from the world model's obstacles and the robot footprint, not from a height band of 0 to 0.43 m. The furniture map's base layer (0 to 0.43 m, src/furniture_map.py, FURNITURE_CFG) is a third grid, for the stance finder; it is not changed either. The task text put these together; I used NavGrid for the floor mask.
- The localization map (src/localization_map.py) is one horizontal slice at the lidar's scan plane, 0.2217 m above the floor, read from LIDAR_SPECS and the model. It holds only what the lidar sees: the bench appears as its legs only.

### How it is built

- Fixed geoms: all non-robot geoms except the task objects (bodies named task_* and pour_ball_*) and the object slots (fc_slot). That gives 107 geoms. The scene's furniture (lab stool, waste bin, rolling cart, fire extinguisher, storage cabinet with its doors) is kept: it has free or hinge joints in the model, but no task moves it, and the lidar sees it.
- Cells: 2 cm (PROVISIONAL, LOCALIZATION_MAP_SPECS in src/robot_specs.py). A cell is occupied when its centre is inside a primitive geom, by the exact test of stage 1, now shared (src/localization_map.inside_primitive; the stage 1 picture imports it).
- Grid: from the walls' outside faces (-6.08 to 6.08 m) plus a 0.2 m margin: x and y from -6.28 to 6.28 m, 628 x 628 cells. Cell states: outside the room (not occupied, beyond the walls' inside faces), free, occupied.
- Distance-lookup grid: exact Euclidean distance transform (scipy) from each cell centre to the nearest occupied cell centre, stored in metres, uncapped. Zero on occupied cells.
- Floor mask: the planner's NavGrid (footprint at the drive pedestal height, the planner's obstacles) gives blocked_for(k) for k = 0 to 7; a planner cell is floor when it is free for every heading; each localization cell takes the planner cell that holds its centre; the cell must also be FREE. No new logic.
- Cache: data/localization_map/localization_map.npz (git-ignored). Its staleness key is a SHA-256 over the version, the cell size, the margin, the slice height, the room faces, the pedestal command, the planner's cell size and minimum block height, the footprint, and the name, type, size, position and orientation of each fixed geom. A cache with another key is rebuilt.

### Map size, time, cache

- Map: 628 x 628 cells of 0.02 m = 394,384 cells; x -6.280 to 6.280 m, y -6.280 to 6.280 m; slice 0.2217 m above the floor; 107 fixed geoms.
- Build time: 0.28 s (final run; 0.34 s earlier run).
- Cached load: 0.116 s (returns identical grids).
- Cache file (2 cm): 0.20 MB, compressed.

### Tests (experiments/localization_map_test.py; log data/logs/localization_map_test_final_20261126_041611.log)

| Test | Expected | Measured | Result |
|---|---|---|---|
| 1 known geometry | leg cells occupied; cells 4 cm outside each leg free; wall lines occupied; bench top footprint free | all as expected | PASS |
| 2 distance grid, five points | within one cell (0.02 m) of the geometry | worst difference 0.0200 m (the 0.3 m south-wall point: stored 0.32, geometry 0.30); others 0.0001 to 0.0100 m | PASS |
| 3 lidar agreement | every usable beam within 0.02 m | 915 of 936 usable beams agree (97.76 %): staging 308 of 312 (98.7 %), approach 308 of 312 (98.7 %), middle of the room 299 of 312 (95.8 %) | FAIL |
| 4 floor mask | the three stage 1 poses are floor; a bench leg and a point outside the room are not | the middle of the room is floor; the bench leg and the outside point are not; the staging stance and the approach spot are NOT floor | FAIL |
| 5 cache, second load | from the cache, identical grids | source cache, 0.116 s | PASS |
| 5 cache, changed cell size (0.03 m copy) | rebuilds | source built, 0.26 s | PASS |
| 6 manual baseline | byte-identical | SHA-256 3060ae41... equal; max difference 0.000000 | PASS |
| lidar regression (stage 1 test) | 16 of 16 | 16 of 16 | PASS |

### Check 3 in detail: the lidar agreement

- All 21 disagreeing beams have the map predicting a hit earlier than the real beam (none the other way: the map never misses an obstacle). The real hit geoms of the 21: wall_e 10, back_counter_base 7, wall_w 3, wall_n 1.
- The worst three beams (middle of the room, lidar at (0.347, -2.2) m):
  - beam 55 deg: predicted 4.696 m, real 9.716 m, the map hits cart_leg_2 (a 4 cm leg) which the real ray passes beside; real hit wall_e.
  - beam 67 deg: predicted 4.170 m, real 8.819 m, the map's cell next to cart_leg_1 (passes 6 mm outside the leg's face); real hit wall_n.
  - beam 335 deg: predicted 2.466 m, real 6.149 m, the map's corner cell of the back counter at (2.6, -3.25) m; real ray passes the corner; real hit wall_e.
- The rest are of two kinds: grazing corners (the fume hood's top corner at (2.8, -1.1) for beams 22 to 24: predicted 2.65 to 2.69 m, real about 6.0 m) and back-counter face beams (about 2.1 cm off: predicted 2.22 to 2.47 m, real 2.24 to 2.49 m).
- Cause: occupancy is by cell centre, so a cell whose centre is inside an object counts as wholly occupied. At 2 cm that adds up to a cell of error at each face, and a whole cell of width to thin objects (a 4 cm leg counts as up to 3 cells). Beams that graze a corner or a thin leg pick the error up. The stage 1 lidar has no such error (it is exact geometry).

### Check 4 in detail: floor mask

- The staging stance (0.765, 1.085) and the approach spot (0.765, 1.291) are not floor in the planner's grid. The planner's obstacles include the bench top's outline (height 0.84 to 0.96 m, below h_top 1.54 m), and the robot's footprint (x -0.36 to 0.34, y -0.31 to 0.31 m at the drive pedestal) overlaps it: the nearest obstacle cell is 0.26 m from the staging centre and the inscribed radius is 0.30 m.
- Geometry: the chassis runs under the open bench, as design plan 3.4 says (5.6 cm under the bench top). The planner rule "A* never drives under an overhang" makes these poses non-floor. The stance finder and the staging do use them.
- I kept the planner's rule (reuse, no new logic), so the mask excludes the staging poses. The middle of the room is floor. See "Not done".

### Picture

docs/reports/20261126_localization_map.png: three panels on the same metre axes. Left: the map (outside, free, occupied; dashed room interior; the three stage 1 poses marked). Middle: the distance-lookup grid (colour, shown up to 3 m; the stored grid is uncapped). Right: the free-floor mask.

## Decisions

- M342: the navigation build order, stages 1 to 8 (A1).
- M343: the run record's seed (A2).
- M344: the five obsolete scripts deleted (A3).
- M345: the localization map's build choices (slice at the lidar height from exact geometry, 2 cm cells, uncapped distance grid, planner-derived floor mask, cache with a staleness key; furniture kept, task objects excluded). Updated rows: M339 (by M340 and M342), M066 and M143 and M163 (earlier), M152 unchanged.

## Not done / needs David

1. Check 3 fails: 97.76 % of usable beams agree within 0.02 m, not all of them. The failures are cell-quantization: grazing corners and thin objects. Options: (a) 1 cm cells (4 times the grid, about 1.6 MB cache, still fast); (b) keep 2 cm and accept the test at 0.03 m or at a stated share; (c) a sub-cell rule (for example occupancy by the cell's own box rather than its centre). None is decided; the decision row M345 says 2 cm.
2. Check 4 fails: the planner's free space excludes the staging stance and the approach spot (the chassis runs under the bench top, and the planner treats the bench outline as an obstacle). Stage 5 would then never scatter a first guess there. Decide: (a) accept (the robot does not start under the bench) or (b) a new rule for poses under the open bench (new logic, which the stage forbids without a decision).
3. The task text called the planner grid "A* on 5 cm cells, the furniture map's base layer covering a 0 to 0.43 m height band". The A* planner's grid is NavGrid (5 cm, footprint-based; its obstacles are those whose height range overlaps the robot's band (0, h_top)). The 0 to 0.43 m band is the furniture map (FURNITURE_CFG base_band), a separate grid for the stance finder. I used NavGrid for the floor mask. Please confirm.
4. Furniture is in the map at its scene position at build time. The lab stool, waste bin, rolling cart and extinguisher carry free joints and could be moved; the map does not follow them, and the cache key does not change when they move in a run. Decide whether these are fixed for localization.
5. Stage numbering: the stage list in design plan 13.1 now has 8 stages (it said 7 in stage 1).
6. The design plan's line for stage 2 is marked BUILT with the commits; checks 3 and 4 are open (see above).
7. The "observation slots" decision needs no change (A4). Stage 4 is still undecided: its condition (lidar confirmed) is met.
8. The dryrun_suite and suite2 jobs were not read; I waited for suite2 to end before any MuJoCo run.
