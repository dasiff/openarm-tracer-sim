# Finish the free approach angle work (2026-11-08): status report (partial)

Not committed. Uncommitted working copy includes M275 and the shared pick path (M276). Scratch files in `scratch/`.

## Stopped here

The task covers about 15 parts; this turn finished part 4 and stopped at part 1, because part 1 (per-task lists) changes how the stance check is called, and the remaining parts depend on it.

## Part 4: hotplate and beaker heights (done)

From `scratch/height_compare.py` (`data/logs/height_compare.log`):

| source | bench top (m) | hotplate (m) | plate top (m) | beaker body z (m) | beaker height (m) |
|---|---|---|---|---|---|
| compiled simulation model (scene model, used by `bench_top_z(model)` and `plate_top_z`) | 0.960 | 0.960 (body origin) | 1.028 | 0.962 | 0.100 (config) |
| stance-time world (`StaticTaskSource`, what the stance selection's world holds) | 0.000 | 0.000 | not present | 0.000 | 0.100 |

Result:
- The compiled model and the scene model agree (they are the same model).
- The stance-time world model reports every height as 0. That differs from the model by 960 mm (bench), 960 mm (hotplate), 962 mm (beaker). The earlier 0-height check failure came from this.
- **Carry reference:** the executed pick's carry height previously read the hotplate's world position (body origin, 0.960 m). The shared pick path now uses the plate top (1.028 m). That raises the carry height by 68 mm compared with the executed plan before M276. This is recorded as a decision below.

## Not done

- **Part 1 (per-task lists, choosing function, failure reason):** not started. The stance check runs all pick steps of a run as one path with one angle, so a per-task list needs the check run per task. Changes needed: `stance_select.py` run loop and two_pass, `full_check.check` per-task mode.
- **Part 2 (place recovery):** not started (`mpvi_modules.py` recover, non-pick branch still uses forward).
- **Part 3 (heading in the stance, staging along it, route clearance):** not started. Candidates still face the staging yaw (`stance_select.py` 74, 86, 90, 184, 464, 624); `staging_source.py` 383 and 437 still use the staging yaw.
- **Part 5 (harness, runs at three stances):** not written, not run.
- **Part 12 (51-stance rerun with heading search):** not run (depends on part 3).
- **Part 13 (tests, baseline worktree):** not run in this turn. Earlier results stand.
- **Part 14 (IN move at staging; non -90 pipeline run):** not run.
- **Part 15 (decisions row, design plan, M275/M276 text):** M276 and M275 exist; they still describe the shared path as the carry height "not the former 10 cm" — needs the carry decision below before the text is final.

## Decisions for David

1. **Carry reference height.** Options: (a) plate top, 1.028 m (what the code now uses; 68 mm higher than the old executed plan); (b) hotplate body origin, 0.960 m (the old executed value). Both are from the same compiled model.
2. **Stance-time world heights.** The stance selection's world holds no heights (all 0). Options: (a) read heights from the scene model at stance time (what the new check does); (b) fix the static world so it reports the scene heights.
3. **Per-task angle lists.** A run with two pick steps needs one angle list per task. Options: (a) the check runs each task separately and the run passes only if each task has a passing angle; (b) keep one angle for all picks in a run (current behaviour).
4. **Heading in the stance search.** Options: (a) search headings -130 to -50 deg in 10 deg steps (the Gaussian sweep's range and step); (b) keep the staging yaw.
5. **Staging to the stance heading.** Options: (a) drive to a spot behind the stance along its heading, then creep in (as the task says); (b) keep staging at the staging yaw and creep along the last metres only.
6. **Place recovery.** Options: (a) back out along the place's own angle (theta); (b) keep the forward direction.

## Files changed so far (uncommitted)

- `src/task_points.py`: `pick_waypoints`, `approach_unit`, `expand_pick_points`, `plate_top_z`, `CARRY_LIFT_M`.
- `src/full_check.py`: the check's pick path per approach angle; `all_angles` option; `pick_bins_passing`.
- `src/stance_select.py`: the pick points come from the spec (`step_target`); imports `bench_top_z`, `plate_top_z`.
- `src/mpvi_modules.py`: `plan_pick`, `pick_approach`, `pick_bin_index`, recovery for picks.
- `docs/mpvi_decisions.md` (M275 text, M276 row), `docs/design_plan.md` (§3.3), `docs/progress.md`.
- `docs/reports/shared_pick_path_20261108.md`, `docs/reports/free_angle_finish_20261108.md`.

## Commit state

No commits made.
