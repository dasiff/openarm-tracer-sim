# Baseline on HEAD, the stance_select_test crash, and the IN move (2026-11-08)

Not committed. Read-only with respect to the working copy: no code, config or data changed in it. Experiments ran in copies under the scratchpad (noguard, zerotest) and in a HEAD worktree (head_wt, removed afterwards; its logs are kept in data/logs/head_*_20261108.log). Scratch scripts in scratch/ (not in git).

## Correction to docs/reports/heights_single_source_20261108.md
The report said the IN move at the staging stance was "the same as before" (14.9 mm, 0.7 deg). That was wrong. The 14.9 mm IN miss is in the older M275 run (data/logs/m275_mpvi_pipeline_test_20261108_143741.log, 3 IN failures). The heights run (data/logs/heights_mpvi_pipeline_test_20261108.log) has no IN failure: its first failure is DOWN at 146.3 mm. The probe run (data/logs/in_probe_20261108/) reproduces that DOWN failure exactly.

## 1. Baseline: HEAD (51e99a7) against the working copy

| test | HEAD | working copy (M275-M277) |
|---|---|---|
| full_check_test | 31/31, exit 0 | 31/31, exit 0 |
| transit_clearance_test | 22/27, exit 1. Fails: "k=1 passes: two runs, one step each"; "each step gets its own navigation (none is null)"; "the pick returns to the zombie pose before the second navigation (end_at_zombie)"; "the second run is reached holding the beaker and says so"; "the chosen spot's own carry is clear and it is one of the ranked spots of the baseline" | 26/27, exit 1. Fails only: "the chosen spot's own carry is clear and it is one of the ranked spots of the baseline" |
| stance_select_test | crashes: IndexError at line 117 (rep["runs"][1]). Fails before the crash: "k=2 fails (the best stance covers one object), k=1 passes for each: two runs, each with its own stance"; "two navigation steps, both move (none is null)" | the same crash at line 117 and the same two failing checks |
| stance_select_moved_test | no checks reached. Exit 1 after 46 s: StanceSelectionError "no stance passed the full check: (0.625, 1.105) pedestal 0.12: ['ik'] at the stance 'pedestal-1' point 4 ... has no collision-free solution" | 6/9, exit 1. Plan starts, then PLAN_FAILED at BACK (82.0 mm, 2.2 deg). Fails: "the episode outcome is SUCCESS"; "the plan finished, every segment done"; "the beaker stands on the hotplate" |
| mpvi_pipeline_test | 4/14, exit 1. Fails: outcome PLAN_FAILED; plan not finished; pick close event not logged; left arm not home (0.6349 rad); beaker not on hotplate (200 mm); status text (7 phases); grounding at dispatch; VLA instruction; grounding log resolved entry; world model beaker entry | 4/14, exit 1. Same names; left arm 0.5083 rad; status text 8 phases; the first failure is DOWN at 146.3 mm (not OVER); the recovery fails at BACK 82.0 mm |

Notes:
- HEAD's pipeline is a different pick: its first failure is OVER (81.4 mm, 5.5 deg) at t = 80.6 s, and the recovery fails at BACK (50.0 mm, 5.5 deg). So the HEAD baseline does not show the IN miss, because HEAD does not reach that stage in the same way.
- The stance_select_test crash exists on HEAD at the same line. Per the instruction, step 2 stops for this test there.

## 2. The crash (stance_select_test line 117) and the change group
- Line 117 is the check "the two stances are far apart", which indexes rep["runs"][1].
- What is empty: the second run. In the second scene (hotplate moved to x = -0.9), select_bench_stance returns one run, [1, 3] covering the hotplate, and tried [3] alone with no stance (covered None). So rep["runs"] has length 1.
- The crash is on HEAD too (same line). Per the instruction, the group bisection is not done for this test. Not done: the cause is the stance search in the moved scene for the second object alone, which the check was written for. Whether M275/M276/M277 changes that search was not measured.
- stance_select_moved_test: HEAD and the working copy behave differently (HEAD raises before any plan; the working copy plans and fails at BACK). The group bisection for this test was not run: the M275, M276 and M277 changes share stance_select.py, task_points.py and mpvi_modules.py, so a clean revert of one group needs a split patch, which was not built.

## 3. The IN move: what the probe shows (working copy, staging stance (0.775, 1.105), heading -90, pedestal 0.12)
Pick bin used by the pipeline: 0 (pick_approach_bin in the episode summary; not 11 as the earlier audit said).

Timeline (probe run, data/logs/in_probe_20261108/):
- OVER: t 66.05 to 67.65 s. The pad reaches its target: (1.0012, 0.7827, 1.1585) against (1.00, 0.78, 1.16). OK.
- DOWN: t 67.65 s, planned 3.7 s. Target: standoff (1.00, 0.78, 1.0123). The pad does not move down: z stays at 1.1585 for the whole step (17 s). It times out at 146.3 mm (2.3 deg).
- Joint 4 (elbow): actual 120.1 deg, frozen from DOWN start until t = 74.9 s. Target 117.4 deg at 67.65 s, then 105.6 deg from 68.65 s on, frozen there. Error reaches -0.253 rad at 68.65 s.
- Tracking suspension: "[bridge] t=68.66 s: suspended: tracking error: left joint 4, value -0.253, limit +0.250".
- Joints 6 and 7 lag too (12.2 and 13.2 deg at the end), with the actual joints frozen (no motion until 74.9 s).
- Hand and finger contact force with the beaker: 0 N throughout DOWN. Hotplate force 0 N. Bench force on the beaker 1.96 N (the beaker resting on the bench). Beaker tilt during DOWN 0.04 deg.
- Left-arm collisions at the stall pose (static check): only the two left fingers touching each other (penetration 5.5 mm). No contact with the beaker, the bench or the hotplate.
- Joint torques at rest (gravity bias, static): joint 4 needs +3.94 N m against an actuator limit of 27 N m. Joint 1 1.78 N m (limit 40), joint 2 1.74 (40), joint 3 1.00 (27). So no torque saturation.
- Joint limits: joint 4 actual 120 deg within 0-140 deg; joint 6 actual -1.8 deg within -45 to 45. No joint limit is reached.

Cause (the numbers that decide it):
- ELBOW GUARD. The guard (M269, ELBOW_GUARD_CFG, enabled, min elbow joint centre 1.1048 m) printed "[elbow guard] left arm: a target would put the elbow joint centre below 1.1048 m; the arm is clamped" at t = 67.39 s, just before DOWN started. The clamp holds joint 4 back, and the tracking error grows to the 0.250 rad limit, which trips the suspension.
- Test with the guard off (copy of the working tree, only ELBOW_GUARD_CFG enabled = False; same probe): no clamp message, no suspension. DOWN ends with the pad at z 1.0128 m against the 1.0123 m target (0.5 mm). IN runs from 71.37 s and ends 2.9 mm from the grasp point (1.00, 0.66, 1.0123). CLOSE closes with 32 N peak hand force; UP lifts the beaker from 0.960 to 1.062 m. The run then fails later, at the place's HOME ("the arm did not settle at home after 9.0 s", largest joint error 0.267 rad), which is a different step.
- So the IN move is not the failure in the working copy; the DOWN stall (the guard and joint 4's tracking suspension) comes first, and the IN miss (14.9 mm) was in the older M275 run.
- Not explained: my reconstruction of the elbow height from the logged pose gives 1.17 m at the logged targets, above the guard's 1.1048 m. The guard works on the targets before it logs them, row by row; the logged targets (105.6 deg) are not the clamped ones. The exact trigger was not reproduced.

Classification: a joint or tracking limit (the elbow guard holds joint 4, then the tracking-error suspension stops the arm). Not contact, not torque, not controller gain.

Forced approach angle: no forced angle is available in the pipeline (the pick bin comes from the stance selection; the check's own approach_bin argument does not reach the pipeline). Running IN at a second passing angle: "no forced angle available, not run". No harness was built.

## 4. The follow-on failures (retry DOWN, recovery BACK)
- The retry (RETRY 2, t = 84.6 to 97.8 s): joint 4 actual 120.1 deg, target frozen at 105.6 deg (error 14.5 deg). Joints do not move until 93.4 s. The pad stays at (1.0011, 0.7827, 1.1585).
- The BACK target is (1.00, 0.71, 1.1205) (bin 0, carry height). Pad distance from it: 82 mm, the same number as the log.
- So the BACK failure has the same cause as DOWN: joint 4 held by the guard and the tracking suspension. Not confirmed with the guard off (the copy's recovery was not run separately).

## 5. The new test (step 5)
Copy with the working tree (guard restored to enabled), one change: StaticTaskSource's bench top_z set to 0.0 (src/world_model.py line 366). Result of experiments/scene_heights_test.py: exit 1, 9 of 12 checks.
- Fails: "no height reads 0 (stance-time world or running sim)" (zero: ['bench top'])
- Fails: "bench top: stance-time world and running sim agree within 0.1 mm" (stance 0.0000, sim 0.9600, diff 960 mm)
- Fails: "the bench top reads the value the scene file gives (0.960 m)" (0.0000)

## Not done
- Group bisection for stance_select_test: stopped (crashes on HEAD too, per the instruction).
- Group bisection for stance_select_moved_test: not run; the groups share three files, and a split patch was not built. HEAD differs in kind (StanceSelectionError before the plan), so the comparison is not a simple before/after.
- Forced approach angle and IN at a second angle: no forced angle available; not run.
- Exact elbow-guard trigger: not reproduced from the logged pose (see section 3).
- Guard-off recovery at BACK: not run.
- The guard-off run's place failure at HOME (after a successful pick and lift): not investigated; it is outside this task.
- Baseline comparison for the other test counts and the M277 height change beyond these tests: not run.

## Decisions for David
1. The elbow guard (min 1.1048 m, M269) blocks the pick's DOWN at this stance and pedestal: the standoff at z 1.0123 m needs the elbow below the minimum. Options: keep the guard and choose another stance or pedestal (the staging stance (0.775, 1.105, -90, 0.12) cannot pick this way); lower the guard minimum (a safety setting, your call); or change the pick path so the elbow stays above 1.1048 m (a different approach or standoff height).
2. Whether the BACK retreat at carry height should pass the guard too, given that the recovery stalls on it.

## Files
- This report: docs/reports/baseline_and_in_move_20261108.md
- Probe (working copy): data/logs/in_probe_20261108/ (in_probe.csv, episode/, recording/), data/logs/in_probe_20261108.log
- Guard-off probe (copy): scratchpad/noguard/out/run/in_probe.csv, scratchpad/noguard/out/probe.log
- Zero-height test (copy): scratchpad/zerotest/zero_test.log
- HEAD baseline logs: data/logs/head_*_20261108.log (moved from the worktree)
- Scratch: scratch/in_move_probe.py, scratch/in_move_analyze.py, scratch/in_move_joints.py, scratch/in_move_static.py, scratch/run_in_probe_20261108.sh, scratch/run_head_baseline_20261108.sh

## Commit state
No commits made.


## Correction (added after guard_off_20261108.md)
The 'not explained' elbow height above (1.17 m at the logged targets, versus the guard's 1.1048 m) was my error: the reconstruction used the XML start height of the base (0.22 m) instead of the settled 0.1527 m. With the settled height, the guard's own elbow height and the model's forward kinematics agree (0.000 mm) and give 1.1046 m at the clamp (67.40 s). See docs/reports/guard_off_20261108.md, step 7.
