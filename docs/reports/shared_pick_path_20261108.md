# One shared pick path (2026-11-08): partial report

Not committed. Working copy includes the earlier M275 change. Scratch files in `scratch/` (not in git).

## Done

1. **Shared function.** `src/task_points.py`:
   - `approach_unit(heading, k)`: heading + bin_center(k).
   - `pick_waypoints(beaker_xy, grasp_z, height, bench_top, carry_z_ref, heading, k)`: OVER, standoff, lift and back points and the approach. Used by the stance check (through `expand_pick_points`), by `plan_pick` and by pick recovery.
   - `expand_pick_points(points, heading, k)`: the full check writes the pick path for each approach angle from the spec that `stance_select.step_target` attaches to the grasp band.
   - `plate_top_z(model, data)`: hotplate top from the scene model.
   - Heights come from the scene model (`bench_top_z(model)`, `plate_top_z`), not the world model: the static world used at stance time reports every height as 0.
2. **Stance check** (`src/full_check.py`): pick path per approach angle, from the stance's own heading. `check(..., all_angles=True)` returns every passing angle in `pick_bins_passing`. Default behaviour returns the first (`pick_bin`).
3. **Stance selection** (`src/stance_select.py`): `step_target` no longer builds OVER/standoff/back along the staging forward.
4. **Executed pick** (`src/mpvi_modules.py`): `plan_pick` uses `pick_waypoints`; `pick_approach` uses `approach_unit`; pick recovery backs out to the pick's BACK point.
5. **Analytic test** (`scratch/shared_path_test.py`): check points equal executed points for 12 angles at headings -90 and -100 (worst 0.0000 mm). The audit's old differences were 62.1 mm (angle 11, heading -90) and 82.1 mm (heading -100). This equality holds by construction (both sides call the same function); it is not a simulator result.
6. **Decisions log:** M276 added; M275 and design plan §3.3 aligned with the code.

## 49-stance rerun (corrected check, `scratch/shared49.json`)

51 stances: the 49 geometry-clean stances plus the staging stance and the point-4 stance.

| | count |
|---|---|
| passes the whole check | 23 of 51 (21 of the 49, plus both named stances) |
| fails at the arm path (ik), first failing point index 2 (first grasp-band point) | 16 |
| passes the pick path, fails place_angle_window | 12 |
| stances with >= 1 passing pick angle | 35 |
| passing-angle count per stance | 0: 16, 1: 21, 2: 13, 3: 1 |
| angles that pass (occurrences) | 11: 29, 10: 12, 0: 9 |

Previous run (before this change): 19 pass, 18 ik failures, 12 place failures; 35 of 49 passed the pick path in the variant-iii test. The pass count rose by 2 and the 12 place failures are unchanged.

## Not done

- **Part B, per-task angle choice at execution (step 5):** the executed pick still takes the angle recorded for the stance (`pick_bin`); the choosing function `choose_angle` and the execution-time check were not written. Needs the full check available to the skill (decision below).
- **Stance record (step 4):** `pick_approach_bin` is removed from the segment, but `pick_bin` still carries the stance's first passing angle into the plan.
- **Heading search (step 7):** not done. Stance candidates still face the staging yaw. The Gaussian sweep's range (-130 to -50 deg, 10 deg steps) is noted but not used.
- **Staging to the stance heading (step 8):** not done (staging_source.py:383, 437; stance_select.py:316-333, 490).
- **Place recovery (step 6):** not changed; still backs out along forward (mpvi_modules.py recover, non-pick branch).
- **Part C (harness and sim runs, steps 10-11):** not written or run.
- **Tests (step 13):** not run in this pass; earlier results stand (full_check_test 0; transit 26/27; mpvi_pipeline 4/14; stance_select tests failing).
- **IN move at the staging stance (step 14):** not measured.
- **Baseline worktree:** denied earlier; not retried.

## Decisions for David

1. **Angle choice at execution.** Options: (a) run the full check inside the skill at execution (needs the check and its maps in the pipeline, several seconds per selection); (b) the stance selection records the passing angles for each task and execution uses the first passing one, with a failure report if it cannot be executed; (c) both, with (a) only when the object is off the recorded position.
2. **Heading and staging.** A stance with a different heading needs the drive to end on that heading, and an approach spot behind the beaker for that heading. Options: drive to the stance heading directly; or keep staging at the staging yaw and drive the last metres.
3. **Carry reference.** The carry height uses the hotplate's top from the scene model; the executed world's hotplate position may differ. Confirm which height the executed plan should use.
4. **Place recovery.** Use the place's own angle (theta) for the back-off, or keep forward.
5. **Rename.** Identifiers still named "bin": `pick_bin` (full_check, stance_select, mpvi_modules), `pick_bin_index`, `pick_approach_bin` (removed from segments), `bin_center` (grasp_rules), `make_constraint(k)`, `approach_bins`, `pick_bins_passing`. Rename to approach angle?
