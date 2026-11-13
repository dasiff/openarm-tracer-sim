# Is the imagined arm pose a good picture of what the arm really does?

Date 2026-11-12. Two complete recorded runs with the stance-search logging on, then a comparison of three poses per task point. No algorithm, setting, threshold or result was changed, and no code in `src/` was touched. Nothing was pushed.

## What was run

| | Staging layout | Moved layout |
|---|---|---|
| Run folder | `data/demo_record/imagined_staging_20261112_205653` | `data/demo_record/imagined_moved_20261112_210418` |
| Log | `data/logs/imagined_staging_20261112_205653.log` | `data/logs/imagined_moved_20261112_210418.log` |
| Seeds | stance sampler 3397871272, full check 4242 | stance sampler 1944726090 (the Nov 10 fix2_moved sampler seed), full check 4242 |
| Outcome | SUCCESS (all four plan segments done, 93.3 s of simulated time) | SUCCESS (96.8 s) |
| Accepted stance | draw 32: base (0.7639, 1.0908) m, heading -88.45 deg, pedestal 0.1644 m (the same stance as the Nov 12 proof run) | draw 15: base (0.6851, 1.1758) m, heading -97.96 deg, pedestal 0.0986 m |
| Approach angle of the pick / hand turn of the place | 0 deg / -30 deg | 0 deg / -20 deg |
| Search files (kept in the run folder) | `candidates_list.json`, `full_check_solutions.json`, `dryrun_trajectories.json`, `run_info.json` in the run folder | same |
| Peak memory | 4.3 GB | 4.3 GB |

The Nov 10 runs used clock seeds, and only the sampler seed was recorded in their logs. The full-check seed was not recorded, so 4242 was used for both runs (the task's fallback). Everything else is the Nov 10 launcher (`run_layout`, bridge on, scripted manipulation, oracle checker, plan from the cache); the only addition is the seed pinning in `pinned_run_layout.py`.

No code change was needed. The run launcher in record-only mode already hands the run folder to the search (decision M312), so the search folder is kept with the run, and nothing deletes it (the only folder removed by the video path is the video work folder).

## The three poses

* (i) check pose: the arm joints the full check solved for the accepted candidate, at the nominal stance.
* (ii) dry-run pose: the joints at the last tick of the subtask in the dry run, at the nominal stance.
* (iii) real pose: the measured joints of the real simulated arm in the last 0.1 s row of the same subtask in the episode log. The log has one row per 0.1 s, so "end of subtask" can be up to 0.1 s early; the arm moved less than 0.008 rad in the following row everywhere, so this is not what drives the differences.

Hand = the left pad centre, elbow = the origin of the fourth arm link, both from forward kinematics on the robot model. The checks of the method: (i) reproduces its own target hand position to 0.0 to 0.4 mm (except the two "retreat" points, 26 mm and 17 mm: the check's own solution there is not on its target), and (ii) reproduces the pad position logged in the dry run to 0.1 mm. "World" differences compare the ghost robot at the nominal stance with the real robot at its real stance; "body" differences remove the base position and heading, so they show the arm only.

The task's "pick approach" is the point over the beaker, "grasp" the end of the move in, "lift" the end of the pick lift. The pick has no separate carry pose in the dry run (the dry run goes straight on to the place), so the place approach over the hotplate is the carry pose. The check's extra pull-back point after the lift has no dry-run or real counterpart and is not compared. "Pick standoff" (down above the beaker) and "home" (arms back at the zombie pose, no check pose) are added for completeness.

## Arrival at the stance (nominal against actual)

| | Staging | Moved |
|---|---|---|
| Base position error | 0.4 mm in x, 3.9 mm in y: 3.9 mm | -1.0 mm in x, 4.0 mm in y: 4.1 mm |
| Heading error | 0.015 deg | 0.17 deg |
| Pedestal error | -0.51 mm | -0.58 mm |
| Time of arrival | 65.6 s | 67.5 s |

The base error is small (about 4 mm, under 0.2 deg), so it explains only a few millimetres of the hand differences below.

## Results

Full tables of every joint (real minus imagined, radians) are in `20261112_2118_imagined_vs_real_arm_pose_files/imagined_tables_20261112.md`; raw numbers in the two `imagined_compare_*.json` files there. Largest joint difference per point (rad) and hand and elbow differences (mm), staging / moved:

| Task point | Check vs real: largest joint | hand (world) | hand (body) | elbow | Dry run vs real: largest joint | hand (world) | elbow |
|---|---|---|---|---|---|---|---|
| Pick approach (over) | 0.34 / 0.39 | 1.5 / 2.5 | 5.0 / 6.1 | 76 / 32 | 0.024 / 0.035 | 1.0 / 2.2 | 2.9 / 2.3 |
| Pick standoff | 0.84 / 0.58 | 0.4 / 0.8 | 4.0 / 4.2 | 124 / 77 | 0.019 / 0.022 | 0.5 / 0.7 | 1.8 / 1.6 |
| Grasp | 0.40 / 0.31 | 1.0 / 2.3 | 4.0 / 3.7 | 55 / 53 | 0.030 / 0.035 | 1.3 / 1.3 | 1.8 / 2.8 |
| Lift | 1.06 / 0.98 | 0.8 / 0.9 | 3.2 / 3.6 | 144 / 131 | 0.018 / 0.025 | 0.8 / 0.4 | 1.1 / 3.3 |
| Place approach (carry) | 0.070 / 0.062 | 7.0 / 5.1 | 8.1 / 5.4 | 14 / 12 | 0.038 / 0.044 | 6.6 / 5.0 | 3.5 / 4.7 |
| Place down (set down) | 0.084 / 0.073 | 2.5 / 4.3 | 3.5 / 3.3 | 14 / 11 | **0.465 / 0.431** | **99 / 104** | 21 / 33 |
| Retreat (back after release) | 0.084 / 0.061 | 5.2 / 4.0 | 4.6 / 1.9 | 14 / 12 | **0.441 / 0.422** | **102 / 107** | 27 / 45 |
| Home | (no check pose) | | | | 0.001 / 0.003 | 4.0 / 4.3 | 4.0 / 4.6 |

Largest elbow difference over all points: check pose 144 mm (staging, lift) and 131 mm (moved, lift); dry-run pose 27 mm (staging) and 45 mm (moved), both at the retreat.

## What the numbers say

1. **The hand is right everywhere in the check pose, but the arm is not.** In the pick, the check pose puts the pad within 0.4 to 2.5 mm of where the real pad is, yet the joints differ by up to 1.06 rad and the elbow by 55 to 144 mm. The arm has seven joints and the hand needs six, so the check found one of many arm shapes that reach the same hand pose, and the real arm, which moves by small steps from the zombie pose, ends in another. The ghost robot's elbow and forearm sit in the wrong place in the pick: the lift is the worst, with the real elbow 13 to 14 cm away from the ghost's. If the ghost is used to judge where the arm goes (collision, camera view, clearance), the pick-side picture is not reliable.
2. **In the place, the check pose is a good picture.** Joint differences are at most 0.085 rad (about 5 degrees), the elbow within 14 mm, and the hand within 2.5 to 7 mm in the world (the 5 to 8 mm at the place approach includes the real arm's own tracking error: the real pad is 5 to 7 mm from the target).
3. **The dry-run pose is the better picture of the pick**: the joints agree to within 0.035 rad (2 degrees) and the elbow to within 3.3 mm, at every pick point and the place approach. This is expected: the dry run is the same planner stepping from the same start.
4. **The dry-run pose is wrong from the place down onward, by about 10 cm.** At place down and retreat the dry-run pad is 99 to 107 mm away from the real pad, with wrist joints 0.3 to 0.47 rad off, while the check pose and the real arm agree. The dry-run pad does not go down: in the staging run its height rises from 1.121 m at the carry to 1.191 m at the set down, where the check's set-down target is 1.075 to 1.09 m and the real pad got to within 2.5 mm of that. Reading `src/trajectory_dryrun.py`: the set-down target is the hotplate top plus the pad's offset to the beaker bottom, and the offset is taken after the pick lift (`off = pad_now - ...` after "pick UP"), so it contains the roughly 0.1 m lift, and the target is about one lift height too high. The real skill takes the offset at the grasp. This is my reading of the code, not something I tested; I did not change it. So the dry run's place down, open and back are mistimed pictures and its place is accepted on a wrong target. The dry run still passed the real place for both stances, so the place is not endangered by this in these two runs, but the dry run is not checking the place's set-down at the right height.
5. **The base error is not the reason.** About 4 mm and 0.2 deg at arrival changes the world-frame hand difference by only 0 to 3 mm against the body-frame one.

## Verdict

* The imagined arm placement, taken from the check pose, is a good picture of where the **hand** goes (within about 1 to 8 mm in both runs) and a good picture of the **whole arm in the place** (within 5 degrees per joint, elbow within 14 mm). It is a poor picture of the **arm in the pick**: right hand, but the elbow and forearm are 5 to 14 cm away from where the real arm puts them (joint differences up to 1 rad).
* The dry-run pose is a much better picture of the pick (2 degrees, elbow within about 3 mm) and of the place approach, but it is wrong from the place down onward (hand about 10 cm too high, wrist joints up to 0.47 rad off). So neither pose is a good picture of everything: use the dry-run pose for the pick and the carry, the check pose for the set-down and retreat.
* Both runs are a single example each (one stance each), so the sizes are indicative, not statistics.

## Not done / for David

* Not done: no change to the dry run's set-down target (an algorithm change, not allowed here), no third run, no rendering of the ghost robot, no replay-level timing (the log has 0.1 s rows). No push.
* David to decide: whether to fix the set-down height in the dry run; and whether the ghost robot for the pick should show the dry-run pose instead of the check pose (or the check should be made to prefer the arm shape the planner reaches).
* Not recorded: the full-check seed of the Nov 10 runs (4242 used, the task's fallback). The check's own "retreat" solutions are 17 to 26 mm off their own targets; not investigated.

## Files

Scripts and data of this report are in `docs/reports/20261112_2118_imagined_vs_real_arm_pose_files/`: `pinned_run_layout.py` (the launcher with the seeds), `imagined_vs_real_compare.py` (the comparison, forward kinematics only), the two compare JSON files and the tables. Decisions-log row M317.
