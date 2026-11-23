# Why the four ARMS tests stall and the held run reaches home (trace, read-only, 2026-11-23)

Branch `safety-filter`, HEAD `74217c2` before this commit. No production code changed. Scratch runs only, one MuJoCo process at a time. Not pushed.

## Summary

- **The commanded home pose is the same in every run, and the commanded path reaches it at 11.03 s in all four runs.** The advance check in `staging_source.py` (lines 820–821) passes in every run, so ARMS is not waiting on it. ARMS waits on the measured joints: all joints of both arms within `arms_tol` (0.03 rad) of home, then settled (`arms_settle_s` 0.3 s).
- **The stall is joint 2 sitting on its solver margin bound.** At rest joint 2 is at 0.000 rad, which is exactly the upper margin bound (10° inside its 10° hard limit of 0.1745 rad). The home command is −0.257 rad, so the arm has 0.257 rad to travel. In the stalled runs the solver output re-pins joint 2 at the bound between 6.9 and 7.4 s and keeps it there to the end (31.39 s), while the command runs on to −0.257. The measured joint 2 stays at −0.001 rad, so the measured-joint test never passes.
- **The held run does not re-pin joint 2.** It lifts joint 2 off the bound at about 2.2 s, follows the command, and its measured joints reach home at 10.19 s (trim on) or 10.29 s (trim zero).
- **The trim is not the cause.** Forcing it to zero gives the same result in both scenes (2×2 table below). It is also not the cause of the manual-baseline differences: trim-zero reproduces the normal HEAD result exactly (25 of 84 rows, max 15.024°).
- **The two scenes diverge at 3.70 s with identical inputs.** Their commanded joints are identical to 0.0 rad for the whole window, and their solver outputs are identical until 3.70 s, whether the trim is on or off. So the difference enters the solver through something the logs do not record: the scene geometry the solver sees, or the pedestal height during ARMS. The logs do not identify which. The distance check I ran uses the default scene, not the scene the grasp test builds, so it cannot separate them (see "What is not settled").
- **Collision avoidance is not what holds joint 2 at the re-pin.** In the default scene the nearest left-arm geom to the bench, hotplate or right arm is more than 0.10 m away at the re-pin (7.4 s). Around 3.4 to 3.9 s the fingers are 0.052 to 0.056 m from the bench, just outside the solver's 0.05 m detection distance. That is close to the moment the two runs separate, so it is the lead to check next, in the task scene.

## Scenarios (joint 2 and the 2×2)

All four runs use the same spawn (base at [0.7, 3.4, 0.153] m, yaw 90°) and the same commanded trajectory. Recordings: `data/logs/gap_part4_20261123_0708/` (grasp trim zero, held trim on) and `data/logs/arms_trace_20261123_1931/` (grasp trim on, held trim zero).

| Scene | Trim | Measured joints within 0.03 rad of home | Joint 2 on the margin bound | Max deviation from home at end |
|---|---|---|---|---|
| grasp_lift_test | on | never (ARMS times out at 31.39 s) | from 6.9–7.4 s to the end | 0.266 rad |
| grasp_lift_test | zero | never (ARMS times out at 31.39 s) | from 6.9–7.4 s to the end | 0.277 rad |
| held run (stance candidate 3) | on | 10.19 s; handover at 11.70 s | only at the start, 1.4 to 2.2 s | 0.002 rad at 11.7 s |
| held run (stance candidate 3) | zero | 10.29 s; handover at 11.70 s | only at the start, 1.4 to 2.2 s | 0.010 rad at 11.7 s |

## Start pose and commanded home (left arm; the right arm is the mirror image)

Start (all runs, t = 1.40 s): within 0.005 rad of zero on every joint (joint 6 is −0.005). Joint 2 is at 0.000, exactly on its margin bound.

Commanded home, `TASK_CFG home_q_left`, and the distance from it to each joint's margin bound (the solver's band, 10° inside the hard limit) and to the hard limit (rad):

| Joint | Home (rad) | Distance to margin bound, lower / upper | Distance to hard limit, lower / upper |
|---|---|---|---|
| 1 | 0.394 | 3.711 / 0.827 | 3.885 / 1.002 |
| 2 | −0.257 | 2.885 / 0.257 | 3.059 / 0.431 |
| 3 | 0.271 | 1.667 / 1.125 | 1.842 / 1.300 |
| 4 | 2.201 | 2.027 / 0.068 | 2.201 / 0.242 |
| 5 | −0.065 | 1.331 / 1.462 | 1.505 / 1.636 |
| 6 | 0.477 | 1.087 / 0.134 | 1.262 / 0.309 |
| 7 | 0.227 | 1.623 / 1.169 | 1.798 / 1.344 |

Joint 4 is 0.068 rad from its margin bound (2.269 rad) and 0.242 rad from its hard limit, and it is at the margin only for 0.6 percent of ticks in the stalled runs, so it is not the blocker. Joint 2 is the blocker: its home value is 0.257 rad from the bound, the stalled solver output sits on the bound, and the measured joint follows the solver output.

## Joint 2 timeline (grasp trim zero; the held run is the same up to 3.70 s)

- 1.40 to 2.2 s: joint 2 on the bound (`M`), both runs.
- 2.5 s on: the command moves joint 2 negative; the solver output follows it (identical in both runs).
- 3.70 s: the first tick at which the two runs' solver outputs differ (by more than 1e-6 rad). The differences grow after that: 0.03 to 0.06 rad between trim on and trim zero in the same scene, and up to 0.28 rad between the two scenes.
- 3.9 to 5.9 s: in the held run the solver output runs ahead of the command on joints 2 to 7 (joint 2 by up to 0.045 rad at 4.4 s, marked `X`), with some joints at the speed cap (`C`); it then closes on the command by 9.9 s.
- 6.9 to 7.4 s: the grasp run's joint 2 returns to the bound (`M`) while the command is at −0.15 to −0.18 rad. From then on, joints 1, 3, 5 and 6 are `X` (the solver output differs from the command by more than 0.02 rad) and joint 2 is `M`, through the end at 31.39 s.
- Held run: joint 2 reaches the command by 9.9 s and stays within 0.002 rad of it.

Cap-binding ticks (any joint at the per-tick speed cap): grasp runs 4 (all before 3.7 s); held runs 66 ticks (0.44 s), starting at 3.69 s.

## What the advance check reads (`staging_source.py` lines 820–821)

`applied_all` is `obs["applied"]["arm_targets"]`. After SF-119, `control_loop.py` `applied()` returns `src_arm_targets`, the arm targets the source commanded, not the solver output or the measured joints (`control_loop.py` lines 561–580). The check compares those commanded targets with `P["last_arm"]`, the targets ARMS returned on the previous tick. These are the same values, so the check passes on every tick in every run. `P["u"]` advances at dt/10 s per tick and reaches 1 at about 11.4 s in all four runs, which matches the commanded joints reaching home at 11.03 s. ARMS then ends only when the measured joints are within `arms_tol` of home and settled. So the stall is on the measured-joint test (joint 2 at 0.000 against −0.257), not on the advance check.

## Candidates (what the logs rule out)

| Candidate | Result |
|---|---|
| Home command | Ruled out: identical (`TASK_CFG home_q_left`) and the commanded path reaches it in both scenes. |
| Start pose | Ruled out: identical rest pose at 1.40 s. |
| Commanded trajectory and order of commands | Ruled out: identical to 0.0 rad over the shared window. |
| Solver starting state | Ruled out: identical solver outputs until 3.70 s. |
| Trim | Ruled out: all four combinations of scene and trim have the same pattern (2×2 table). The trim does change the solver output after 3.70 s (about 0.03 to 0.06 rad between trim on and trim zero in the same scene), but it does not decide whether joint 2 re-pins. |
| Base pose during ARMS | Ruled out: spawn in all runs. |
| Advance check | Ruled out: passes in every run (see above). |
| Collision avoidance with the bench, hotplate or right arm at the re-pin | Ruled out for the default scene: more than 0.10 m away at 7.4 s. Not yet checked in the task scene. |
| Scene geometry or pedestal height during ARMS | **Not ruled out.** This is the only input that can differ before 3.70 s; the two runs' setup code is different (the grasp test uses `make_grasp_lift_stage` on the task loader; the held run uses the sampler stance and the shim). |

## Manual baseline with the trim forced to zero (step 5)

Command: `experiments/trim_zero_run.py combined_controller` with the environment of `cc_bench_scenarios.sh manual` (CC_HEADLESS=1, 1248 frames). Output: `data/logs/arms_trace_20261123_1931/manual_trimzero/manual_trimzero.csv` (84 rows). Compared with `experiments/baselines/manual_reference_20261103.csv` (CURRENT) by the pre-push rule (rows with any change above 1e-9):

| Run | Rows changed | Largest change |
|---|---|---|
| Normal HEAD (`pp_20261123_0708/manual_head_b.csv`) | 25 of 84 | left_joint3 15.024° (row 67, 8.20 s) |
| Trim forced to zero (solver running) | 25 of 84 | left_joint3 15.024° (row 67, 8.20 s) |

Trim-zero adds no change beyond the normal HEAD result. The other changed joints match too: right_joint1 4.948° (9.20 s), right_joint4 0.633° (9.20 s), right_joint7 0.033° (9.60 s).

## Plain-language answers

1. **What is each scenario's home command and start?** Both the same: home is the same commanded pose; both start at rest with joint 2 exactly on its margin bound.
2. **Is joint 2 pinned, and for how long?** In the stalled runs, yes: from 6.9–7.4 s to the end (24 s of the 30 s window, 83 percent of all ticks). In the held run, only at the start (1.4 to 2.2 s).
3. **When do the arms reach home?** Held: measured joints within tolerance at 10.19 s (trim on) or 10.29 s (trim zero), handover at 11.70 s. Tests: never.
4. **What differs?** Not the command, the start pose, the trim or the advance check. The two runs' solver outputs separate at 3.70 s with identical inputs, so the difference is in what the solver sees about the scene (geometry or pedestal height during ARMS). I have not identified the element.
5. **What does the advance check read?** The commanded targets, which always match, so the check passes. ARMS waits on the measured joints.

## What is not settled

- **The scene difference is the open question.** The two setups differ in ways the logs do not record. The check I ran uses `experiments_loader.load()` (the default scene) for both runs, not the scene each test builds. A check in the task scene would settle whether the fingers come within the 0.05 m detection distance of the bench at 3.7 s in one setup and not the other.
- **Pedestal height during ARMS** was not logged in these recordings, so I cannot say whether it differs between the setups.
- **The re-pin is a solver branch, not a collision.** At the re-pin the constraint active on joint 2 is its margin; the other joints are `X` (redundancy compensation). I did not isolate why the solver picks the branch with joint 2 on the bound; the posture cost (0.01) is weak, and the stalled configuration has a higher posture cost than home.

## Recommended next checks (not run)

1. Build each run's own scene (the grasp test's task loader; the held run's sampler stance) and compare bench, hotplate, beaker and pedestal poses at t = 0.
2. In each scene, compute the finger-to-bench distance at 3.4 to 4.0 s on the common solver output, and check whether the 0.05 m detection distance is crossed in one setup only.
3. If the scene is the same, log the pedestal joint at t = 1.4 s in both runs.

## Files

- `experiments/arms_stall_trace.py` (analysis: per-run tables, constraint letters, divergence, optional scene distance check with MuJoCo; committed with this report)
- `experiments/trim_zero_run.py` (runs an experiment script with the trim forced to zero, for step 5; committed with this report)
- Outputs (git-ignored): `data/logs/arms_trace_20261123_1931/` (grasp trim on, held trim zero, manual trim zero, and `analysis/` with `arms_stall_trace.json`, the 0.5 s CSVs and `collision_check.json`); `data/logs/gap_part4_20261123_0708/` (grasp trim zero, held trim on)
