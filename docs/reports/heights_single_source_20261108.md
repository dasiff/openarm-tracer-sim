# Heights from one source; carry from the plate top (2026-11-08)

Not committed. Scratch in scratch/ (not in git). Logs in data/logs/heights_*_20261108.log.

## 1. Corrections to the earlier note
- The old executed carry did NOT use the hotplate body origin. plan_pick read `world.object_pose("hotplate")`, which in the simulator world is the top site `hotplate_top` = 1.028 m (measured, scratch/heights_probe.py). The "0.960 m" and "68 mm" of the earlier note were a body-origin reading. Carry difference: 0 mm.
- The stance-time world's zeros (bench, hotplate, beaker) were real, in StaticTaskSource only. The stance check itself reads the simulator world, so it was not affected by the zeros.

## 2. Every place that uses a hotplate or bench height (before -> after)
Values in metres, world z.

| file | line (approx) | used for | before | after |
|---|---|---|---|---|
| src/mpvi_modules.py plan_pick | 243 (was 227-229 at HEAD) | pick OVER, lift (carry) | carry ref = world object_pose("hotplate") = 1.028 (sim site); z_clear = world bench_top_z 0.960 | carry ref = plate_top_z(model) = 1.028; z_clear = bench_top_z(model) 0.960. Same values |
| src/mpvi_modules.py recover (pick branch) | 327 | pick BACK | plate_top_z(model) 1.028 | unchanged |
| src/task_points.py pick_waypoints | 94-118 | OVER, standoff, lift, back | bench/plate from callers (model) | unchanged (values 0.960 / 1.028) |
| src/task_points.py plate_top_z | moved | plate top | defined here, model geom | now defined in task_beaker_hotplate.py; imported here; same value 1.028 |
| src/stance_select.py step_target (pick) | 220-222 | pick spec bench_top, carry_z_ref | bench_top_z(model), plate_top_z(model) | unchanged |
| src/stance_select.py step_target (place) | 166-180 | target_h, surface_z | world target height + shift; shift = ctx.bench_top - root top_z | world target height; the shift is gone. Sim world: shift was 0, same value. Static world: shift was 0.960, same value |
| src/stance_select.py _body and the extras | 132-208 | footprint z0/z1 of world entries | base + shift | base (no shift); same values |
| src/stance_select.py StanceContext | 72, 89 | self.bench_top = bench_top_z(model) | 0.960 | removed (only the shift used it) |
| src/stance_select.py CARRY_LIFT_M | 236 (old copy) | carry lift 0.05 | own copy | imported from task_points (same value 0.05) |
| src/world_model.py StaticTaskSource: bench entry | 362 | top_z and z | 0.0 | 0.960 (task_heights) |
| src/world_model.py StaticTaskSource: hotplate entry | 361 | z and height | z 0.0; height HOTPLATE_HEIGHT 0.068 | z 1.028; height plate - bench = 0.068 |
| src/world_model.py StaticTaskSource: beaker entry | 359 | z | 0.0 | 0.962 (beaker base site at settle) |
| src/world_model.py HOTPLATE_HEIGHT | 36 | constant copy of 0.068 | 0.068 | removed |
| experiments/reach_model_plot.py | 26, 40 | hotplate place height | HOTPLATE_HEIGHT 0.068 | task_heights plate - bench = 0.068 |
| src/world_model.py SimGroundTruthSource | 316-331 | bench, hotplate, beaker | sim values (unchanged) | unchanged; now the same as scene_heights |
| src/mpvi_modules.py plan_place | 258-272 | place OVER, DOWN, BACK | hp = world hotplate position = 1.028 | unchanged |
| src/task_config.py ELBOW_GUARD_CFG table_top_z_m | 745 | elbow guard table height | literal 0.960 | unchanged (see Decisions) |
| src/pour_scene.py apply_pour_poses | 132 | second beaker z (pour mode only) | data.xpos of body task_hotplate = 0.960 (labelled "bench top") | unchanged (value is the bench top; see Not done) |

## 3. Cause of the zeros, and the fix
- Cause: StaticTaskSource had no model, so it wrote 0.0 for the bench top, the hotplate's z and the beaker's z (a comment said so). stance_select corrected this with a shift.
- Fix: task_beaker_hotplate.scene_heights(model, data) is the single source (bench_top, plate_top, beaker_z, beaker_height). task_heights(task_cfg) reads the task model once (no simulation step, no cameras) and caches the result. StaticTaskSource uses task_heights. The shift in stance_select is removed. The HOTPLATE_HEIGHT copy is removed.

## 4. Readers of heights after the fix
- bench_top_z(model): scene_heights, apply_task_poses (body z), task_points.task_points, pick path callers (mpvi_modules, stance_select spec), full_check, experiments.
- plate_top_z(model, data) (= scene_heights plate_top, 1.028): mpvi_modules plan_pick, recover, pick path; stance_select pick spec carry_z_ref.
- task_heights(): StaticTaskSource, reach_model_plot.
- SimGroundTruthSource (sim world): bench top_z 0.960; hotplate position = hotplate_top site 1.028; hotplate height = site - bench 0.068; beaker position = beaker_base site 0.962; beaker height = BEAKER_CFG 0.100.
- WorldModel.bench_top_z() (the bench entry's top_z): from whichever source wrote it.
- support.py top_of / support_chain, stance_select target height and carry_bottom_z: read the world entries.

## 5. Tests
Earlier results (from the previous summary): full_check_test exit 0; transit_clearance 26/27; mpvi_pipeline 4/14; stance_select and stance_select_moved failing (counts not recorded).

| test | now | note |
|---|---|---|
| scene_heights_test (new) | 12/12, exit 0 | no zero; stance-time world = sim within 0.1 mm |
| full_check_test | 31/31, exit 0 | |
| transit_clearance_test | 26/27 | same count as before; FAIL "the chosen spot's own carry is clear and it is one of the ranked spots of the baseline" |
| stance_select_test | crashed (IndexError, line 117) | check "k=2 fails ..." and "two navigation steps, both move" failed; then rep["runs"][1] missing -> IndexError. Earlier: failing (count not recorded). |
| stance_select_moved_test | 6/9 | FAIL: episode outcome (PLAN_FAILED: recovery at BACK, 82 mm), plan finished, beaker on hotplate |
| mpvi_pipeline_test | 4/14 | same count as before |

## 6. Carry height and the IN move
- Carry: old executed carry 1.028 (object_pose("hotplate")), new 1.028 (plate_top_z). Difference 0 mm. The 68 mm difference in the earlier note is withdrawn.
- IN move at the staging stance: the first failure is the same as before: "IN: not reached after 15.0 s: 14.9 mm and 0.7 deg". The retry then fails at DOWN (146.3 mm, 2.3 deg), and the recovery fails at BACK (82.0 mm, 2.3 deg), not at HOME (127.6 mm, 29.2 deg) as in the earlier run. The recovery failure point changed; the IN error did not.

## Not done
- Stance_select_test: crashes at line 117 (IndexError) before its last checks; cause not investigated (its two-run stances find no second stance). Earlier counts not recorded, so no before/after count.
- Experiments with their own height literals (not changed; they are studies, not the pipeline): experiments/home_pose_candidates.py (0.068 body offsets), experiments/transit_clearance_test.py (0.96, 0.068 fixture), experiments/bench_stance_test.py (path_offsets 0.96), experiments/full_check_test.py and furniture_map_test.py (0.96 ranges), nav_core_test / nav_follow_test (bench height range 0.96). Changing these would change their expectations; the rules say not to.
- Pour mode (src/pour_scene.py line 132) uses the hotplate body origin, which is the bench top there. Correct value for that use; not changed.
- Beaker height: the geometry tops out 0.100 m above the origin, the bottom is at +0.001 m. Matches config.
- Negative check (that the test fails on a zero) was not run separately; the zero check is explicit in the code.
- Baseline comparison to untouched HEAD was not run (earlier approval was denied).

## Decisions for David
1. Elbow guard table top (task_config ELBOW_GUARD_CFG table_top_z_m = 0.960, a literal copy of the bench top). Options: keep the config value (current); or read it from scene_heights (it then depends on the model).
2. Pour mode: keep the body-origin read (correct value) or move it to bench_top_z for clarity.
3. Test literals (transit, bench_stance, full_check, nav): keep the expectations as they are (current), or let them read scene_heights.

## Files
- Report: docs/reports/heights_single_source_20261108.md
- Code: src/task_beaker_hotplate.py (plate_top_z, scene_heights, task_heights), src/task_points.py, src/world_model.py, src/stance_select.py, experiments/reach_model_plot.py
- Test: experiments/scene_heights_test.py (added to experiments/run_tests.sh)
- Decisions: docs/mpvi_decisions.md (M276 text, M277 row); docs/design_plan.md §3.3 bullet
- Logs: data/logs/heights_*_20261108.log, data/logs/heights_run_20261108.status
- Scratch: scratch/heights_probe.py, scratch/run_five_20261108.sh

## Commit state
No commits made.

## Correction (added 2026-11-08, after baseline_and_in_move_20261108.md)
The statement in section 6 that the IN move at the staging stance is "the same as before" is wrong. The 14.9 mm IN miss is in the older M275 run (data/logs/m275_mpvi_pipeline_test_20261108_143741.log). In the heights run the first failure is DOWN at 146.3 mm; there is no IN failure. The cause is the elbow guard holding joint 4 and the tracking suspension; see docs/reports/baseline_and_in_move_20261108.md.
