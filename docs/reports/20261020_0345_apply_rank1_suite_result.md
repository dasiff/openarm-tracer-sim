# Report 2026-10-20 03:45: rank 1 applied, full test suite: two tests need your decision (stopped before baselines, tidy, push)

Written for: David. Local main has the pose and stance applied (committed, not pushed). Baselines are NOT regenerated, the commits are NOT tidied, nothing is pushed, wip/new-stance is NOT deleted and D2 is NOT marked done, because the suite is not green.

## Applied (done, committed locally)
- `TASK_CFG`: `home_q_left` (22.598, -14.709, 15.531, 126.122, -3.746, 27.308, 13.018) deg, `home_q_right` mirrored; `staging_xy` (0.765, 1.085); `pedestal_q` 0.155 (range 0.10 to 0.21); comments say finder output with the margins. The approach spot stays 0.2063 m behind the stance (`staging_approach_back_m`).
- Design plan §1 (home pose, with the method for future scenes: the requirement set and the finder rule) and §3.4 (the stance and its margins); README and TODO wording.
- `home_pose_test` has a new check that the configured stance and pose meet every `STANCE_REQUIREMENTS` item (clearance, placed beaker, neck spread, occlusion, framing slack). The grasp tests need no edit: their arm targets are solved by IK at the stance the robot is at (`grasp_ik_targets.py`); a probe at the new stance solves for the left arm (clearance 10.5 mm, joint margin 20.4 deg).
- New `experiments/run_tests.sh` (the whole suite, one test after the other, a log per test and a summary).

## Suite run (tmux, `data/logs/suite_20261020_022806_summary.txt`, per-test logs `suite_20261020_022806_<name>.log`; 51 min)
Passed: arm_kinematics 18/18, grasp_rules 33/33, arm_collision 17/17, gripper_geometry 23/23, furniture_map 31/31, stance_finder 28/28, pour_scene 21/21, **home_pose_test 34/34** (including the new requirements check), face 23/23, neck_once 21/21, neck_framing, grasp_lift 4/4, beaker_hollow 21/21, target_mode 17/17, sequential_policy 15/15, correction 9/9, success 5/5, pedestal_lift 19/19, rest_contacts 22/22, robot_vs_urdf 25/25, reach_map_holes, cc_loop, safety_filter_model_check.
Failed: gripper_force_test 28/29 and cc_mode_tests (scenarios B and R1; R2 and R3 did not run after R1 crashed).

### 1. gripper_force_test 28/29 (not changed, not loosened)
The failing check: at the 1 N closing "each finger's contact force is the limit within the beaker's weight along the closing axis (+-0.10 N)": measured [0.883, 1.155] N. The mean is right (1.019 N, binds at the cap) and the 2 N and ceiling closings pass. A 1 N grip (pad friction 0.4) cannot hold the 0.2 kg beaker, so the beaker slips and the split between the fingers depends on the pose. At the old stance it was [1.016, 1.023].
Diagnostic (nothing changed in the repo; log `data/logs/force_frac_diag_20261020_033120.log`): the test grasps at 0.5 of the beaker's height. With the grasp at other heights inside the band (same IK, same bin 0) all 29 checks pass:

| grasp height (fraction of the beaker) | 1 N: finger forces | tolerance | result |
|---|---|---|---|
| 0.45 | 0.941, 1.097 | +-0.12 | pass 29/29 |
| **0.50 (the test's)** | **0.883, 1.155** | +-0.10 | **fail** |
| 0.55 | 0.946, 1.093 | +-0.11 | pass 29/29 |
| 0.60 | 0.968, 1.071 | +-0.12 | pass 29/29 |

So the check is marginal at this pose family (imbalance 0.07 to 0.14 N against a tolerance of 0.11 to 0.14), and the default height falls on the worst case. Options: (a) run the test at the middle of the grasp band (0.525 = the band's centre, `BEAKER_CFG["grasp_band"]`) instead of 0.5; it is a principled choice but I have not run it, and it does change what is tested; (b) widen the tolerance (a loosening; not recommended); (c) leave it failing. I recommend (a).

### 2. cc_mode_tests: B fixed, R1 has one failing check
Both failures date from the 10-19 staging reorder (ARMS first, then PEDESTAL, DRIVE, CREEP_IN), not from this pose (the handoff was already at 48.4 s at the old stance; the test was last run on 10-18). This run was the first since.
- Scenario B teleported the beaker at a fixed 46 s, but the handoff now comes at 47 to 48.5 s, so the "operator assisted" success never happened under teleop. I moved the teleport to 52 s (`delay_s` 44.6 to 50.6). No assertion changed. B now passes (3/3).
- Scenario R1's time budget (`end=4000` old ticks) was shorter than the staging, which crashed the test. I raised it to 6000. No assertion changed. R1 now runs to the end: 7 of 8 checks pass, and R2 (5/5) and R3 (4/4) pass.
- **R1 still fails one check**: "control: without restoring the pedestal stays where the operator left it" (+0 mm). The operator moves the pedestal during ARMS; ARMS now comes before PEDESTAL, and the PEDESTAL phase always ramps the pedestal to its height afterwards, so with restoring off the pedestal is put right anyway. The test's premise (a pedestal move that nothing else undoes) no longer holds for ARMS. To test the restore logic the operator has to move the pedestal in a phase after PEDESTAL (DRIVE, with the pedestal already at its height); the checks stay the same. I did not do this: it changes the scenario, and a control run with a raised pedestal while driving may hit the clearance monitor. Run: `data/logs/cc_mode_BR_20261020_032147.log` (B R1 R2 R3: 18/19).

## What I need from you
1. gripper_force_test: option (a) grasp at the band's centre (recommended), (b) or (c)?
2. cc_mode_tests R1: re-aim the scenario at DRIVE (same assertions)? 
After both pass I will rerun those two tests (about 25 minutes), then regenerate the baselines (temp file, check, rename), tidy the 77 local commits into topic commits (a backup branch first, and the final tree must equal the current one), push main and delete wip/new-stance.
