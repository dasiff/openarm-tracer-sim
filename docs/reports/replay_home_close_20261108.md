# Merge risk, replay mismatch, HOME, and CLOSE (2026-11-08)

Not committed. Read only with respect to the sag trim: no file that implements or tunes the sag trim was edited, in the real tree or in any copy. Scratch and copies are under scratch/ and the scratchpad (not in git). Outputs under data/logs/rep/.

## 0. Merge risk
Method: git branch --no-merged, git merge-base, git diff --name-only against the merge base (read only). Test merges were run in a temporary clone (scratchpad/merge_test), into which our uncommitted diff was applied as a test commit; nothing was merged in the real tree.

Unmerged branches (ahead of HEAD = 51e99a7):

| branch | commits ahead of HEAD | files changed vs merge base | sag trim touched | conflicts with our uncommitted work (test merge) |
|---|---|---|---|---|
| origin/held-weight-ramp-sag-off (remote copy of the current branch, 79 behind / 11 ahead) | 11 | docs/mpvi_decisions.md, docs/progress.md, experiments/held_mass_ramp_test.py, experiments/place_over_turn_measure.py, experiments/sag_layer_test.py, src/control_loop.py, src/mock_policy.py, src/mpvi_modules.py, src/mpvi_pipeline.py, src/orchestrator.py, src/sag_layer.py, src/signal_trace.py, src/task_config.py | **yes**: src/sag_layer.py, experiments/sag_layer_test.py (the branch moves the trim's integration out of mpvi_modules into sag_layer.py and adds SAG_LAYER_CFG) | docs/mpvi_decisions.md, docs/progress.md, src/task_config.py (text merge conflicts) |
| backup/held-before-rebase-20261105 (local) | 19 | docs/mpvi_decisions.md, docs/progress.md, docs/reports/*elbow*, experiments/elbow_guard_test.py, experiments/held_mass_ramp_test.py, experiments/place_over_turn_measure.py, experiments/run_tests.sh, experiments/sag_layer_test.py, src/control_loop.py, src/elbow_guard.py, src/mock_policy.py, src/mpvi_modules.py, src/mpvi_pipeline.py, src/orchestrator.py, src/sag_layer.py, src/signal_trace.py, src/task_config.py | **yes**: src/sag_layer.py, experiments/sag_layer_test.py | docs/mpvi_decisions.md, docs/progress.md, experiments/run_tests.sh, src/mpvi_modules.py (auto-merged), src/task_config.py |
| world-model-build (local, checked out in a worktree) | 4 | docs/mpvi_decisions.md, docs/progress.md, docs/reports/world_model_build_stage*.md, src/control_loop.py | no | docs/mpvi_decisions.md, docs/progress.md, src/stance_select.py, src/task_config.py |
| backup-before-tidy (local) | 18 | many incl. experiments/arm_sag_test.py, src/arm_actuators.py, src/control_loop.py, src/robot_specs.py | experiments/arm_sag_test.py (a sag test file; the branch's own content not inspected) | docs/design_plan.md, docs/progress.md, src/task_config.py |
| backup-before-tidy-2 (local) | 84 | many | no | docs/design_plan.md, docs/progress.md, experiments/run_tests.sh, src/task_beaker_hotplate.py, src/task_config.py |
| backup-before-tidy-3 (local) | 18 | docs, experiments/mpvi_pipeline_test.py, run_tests.sh, src/mink_arm.py, src/mpvi_modules.py, src/mpvi_pipeline.py, src/orchestrator.py, src/task_config.py | no | docs/design_plan.md, docs/progress.md, experiments/mpvi_pipeline_test.py, experiments/run_tests.sh, src/mpvi_modules.py, src/task_config.py |
| backup-before-tidy-4 (local) | 6 | docs, experiments, src/grounding.py, src/mpvi_*, src/world_model.py | no | docs/design_plan.md, docs/progress.md, experiments/mpvi_pipeline_test.py, experiments/run_tests.sh, src/mpvi_modules.py, src/world_model.py |
| backup-before-tidy-5 (local) | 10 | docs, experiments, src/llm_gemini.py, src/mpvi_*, src/task_config.py, src/world_model.py | no | docs/design_plan.md, docs/progress.md, experiments/run_tests.sh, src/mpvi_modules.py, src/task_config.py, src/world_model.py |
| origin/Joel_Work_Branch (remote, unrelated) | 1 | Joel_Work/* | no | none |

Our uncommitted files (git status): docs/design_plan.md, docs/mpvi_decisions.md, docs/progress.md, experiments/mpvi_pipeline_test.py, experiments/reach_model_plot.py, experiments/run_tests.sh, experiments/stance_select_moved_test.py, src/full_check.py, src/mpvi_modules.py, src/stance_select.py, src/task_beaker_hotplate.py, src/task_config.py, src/task_points.py, src/world_model.py (plus untracked: src/elbow_clearance_log.py, experiments/scene_heights_test.py, scratch/).

Confirmation: our uncommitted diff touches no file that implements or tunes the sag trim (src/sag_layer.py, experiments/sag_layer_test.py, and the trim's settings in PICK_PLACE_CFG are not in our diff). The word "sag" appears in our diff only in a documentation row (M183 text).

Line-level detail for the two sag branches (hunk headers, cheap read):
- src/mpvi_modules.py: the branch removes _clip_norm and _sag_trim_step and the trim's integration from make_scripted_manipulation and adds a SAG_LAYER import. Ours changes pick_approach, plan_pick and recover, all inside make_scripted_manipulation. Same function, different parts. The test merge auto-merged it with no conflict, but the function is shared, so a later merge should be read by hand.
- src/task_config.py: the branch changes PICK_PLACE_CFG (sag_reset_at_skill_start) and adds SAG_LAYER_CFG. Ours adds the M278 switch and comment inside ELBOW_GUARD_CFG. The branch's base predates the elbow guard (M269), so its side of that block is empty: a merge would **delete ELBOW_GUARD_CFG, including our M278 setting**, unless resolved by hand.
- docs/mpvi_decisions.md and docs/progress.md: text conflicts (both sides add rows and entries at the end).

Not decided here: which branch David will merge. Both sag branches must be checked against the M278 change before a merge.

## 1. Replay: where the difference comes from
Scenario: experiments/mpvi_pipeline_test.py (the whole pipeline, staging stance, guard setting as stated, same seed and code).

Runs (each twice where stated), the replay check's own output:

| config | run | replay check | first mismatch tick (t) | difference at the checkpoint (tick 11250 or 12450) |
|---|---|---|---|---|
| (a) guard OFF, elbow hooks ON (the working tree) | a1, a2 | FAIL | 11145 (75.71 s) | qpos 1.385e-4 (index 13 = openarm_left_joint4); qvel 4.06e-4 (index 12 = left_joint4); qacc_warmstart 5.1e-2; ctrl 6.66e-3 (index 8 = left_joint7 actuator) |
| (b) guard OFF, elbow hooks OFF (copy cpHoff) | b1, b2 | FAIL | 11145 (75.71 s) | identical to (a) |
| (c) guard ON, elbow hooks ON (copy cpGon) | c1, c2 | PASS | none (14458 ticks) | none |
| (d) guard ON, elbow hooks OFF (copy cpGonHoff) | d1, d2 | PASS | none (14458 ticks) | none |
| guard OFF, signal trace ON (OPENARM_SIGNAL_TRACE=1, real tree) | trace | FAIL | 11145 (75.71 s) | identical to (a) |
| guard OFF, trim OFF (SAG_LAYER_CFG enabled False, copy cpSagOff, trace ON) | sagoff | FAIL | 12306 (83.45 s) | qpos 1.28e-4 (index 15) |

The 'hooks' here: the elbow-clearance monitor (src/elbow_clearance_log.py) and the existing clearance monitor (unchanged) in both (a)/(b)/(c)/(d); the signal trace only in trace/sagoff.

Which it is:
- **Not run-to-run randomness.** The two guard-off runs with hooks on (a1, a2) give identical numbers, and so do the hooks-off runs (b1, b2). The results are deterministic in every configuration.
- **Not the hooks.** (a) and (b) are identical to the last digit, and the trace run matches (a). The hooks read the simulation (data reads; mj_geomDistance and mj_contactForce return values and write only into the monitor's own arrays); they use no random numbers; the sim clock is tick-based, so the extra work does not change simulated time. Nothing in the hooks or the run writes to the physics state or the RNG.
- **Caused by the guard being off, in the sense that this trajectory is the trigger.** With the guard on the trajectory is different (the elbow clamp changes the arm's path), and the replay passes in both (c) and (d). The guard-off trajectory has the left arm at its lowest elbow height (1.09 m, below the guard's minimum) around 72–75 s, and the mismatch begins in the weigh pause, while the source is held (see section 3). I did not identify the exact mechanism. The replay compares the recorded control inputs (ctrl) with the state the live run produced: at the checkpoint, the replayed ctrl of left_joint7 differs from the live ctrl by 6.7e-3. That points to a control value that reached the physics without being in the recorded row (a recording gap), but this is inferred, not localized: the direct ctrl writes in control_loop.py I found are startup-only, before recording.

What this means: the replay check is a real-trajectory sensitivity, not noise. It is not a change in the physics. Whether the recorder misses some control path is open (Not done).

## 2. HOME (guard off, trim on; trace run, observation)
Source: data/logs/rep/trace (episode_log.csv and signal_trace.csv). HOME window 89.8 to 102.6 s. home_q_left (deg): [22.6, -14.7, 15.5, 126.1, -3.7, 27.3, 13.0].

Table: applied target minus home, actual minus home, and target minus actual (deg), joint 1..7:

| t (s) | target − home | actual − home | target − actual |
|---|---|---|---|
| 89.8 | [-29.2, 9.7, -7.1, -44.2, -2.2, -50.2, -15.7] | [-29.4, 9.8, -7.0, -44.2, -1.4, -49.5, -15.4] | [0.2, -0.1, 0.0, 0.0, -0.8, -0.7, -0.3] |
| 91.3 | [-10.6, 13.3, -10.5, -18.1, 2.5, -32.3, -4.9] | [-11.4, 12.9, -10.4, -19.0, 0.7, -34.9, -6.1] | [0.8, 0.4, -0.1, 0.9, 1.8, 2.5, 1.2] |
| 92.8 | [3.2, 14.7, -8.4, 0.3, 8.1, -14.3, 0.8] | [3.1, 14.7, -8.3, 0.2, 8.1, -14.0, 0.2] | [0.2, 0.0, -0.1, 0.1, 0.1, -0.3, 0.6] |
| 93.7 | [2.9, 14.7, -8.3, -0.1, 6.9, -15.8, 0.3] | [3.0, 14.7, -8.4, 0.0, 7.5, -15.2, 0.2] | [-0.1, 0.0, 0.1, -0.1, -0.6, -0.6, 0.1] |
| 98.0 | [2.9, 14.7, -8.4, -0.1, 7.1, -15.9, -0.6] | [3.0, 14.7, -8.4, 0.0, 7.5, -15.3, 0.0] | [-0.0, -0.0, -0.1, -0.1, -0.4, -0.6, -0.6] |
| 102.6 | [3.0, 14.7, -8.5, -0.1, 7.4, -15.9, -0.6] | [3.0, 14.7, -8.4, 0.0, 7.5, -15.3, -0.0] | [0.1, -0.0, -0.1, -0.1, -0.1, -0.6, -0.6] |

Reading:
- The joints track their applied targets (target − actual ≤ 0.6° from 93.7 s on). Joint 6 is frozen at 11.4° (home − 15.9°) from about 93.8 s.
- The arm holds the home hand pose, not the home joints: the pad at the end of HOME is the home pad (forward kinematics, robot frame, 0.1 mm), while joints 1, 2, 3, 5 and 6 are off home by +3.0, +14.7, −8.4, +7.5 and −15.9°. That is a null-space difference of a 7-joint arm.
- The sag trim's own values (signal_trace.csv, C_p and C_r of the left arm, HOME window): |C_p| at most 10.2 mm; |C_r| at most 0.061 rad (3.5°). The limits are sag_max_m 0.04 m and sag_max_rad 0.1 rad (PICK_PLACE_CFG). The trim's values are logged; they are small.
- So the trim's size does not explain the 15.9° of joint 6. The mechanism is in the layer's code: sag_layer.py correct() does not add the trim to the source's joint targets. It takes the pad pose the source's targets imply (p_ref = FK of the targets), adds the trim (C_p, C_r), and re-solves the arm's IK (MinkArm.step) for that pad pose; the IK's output is the joint target. For a joint-only HOME move, p_ref is the home pad, so the IK keeps the pad there and stops at its own joint solution (a null-space point). This is the layer's design, and the code's docstring names the case ("the trim also runs on joint-only moves (recovery's HOME)"). I did not edit or test that code.
- Setting-off run (trim off for the whole run, sagoff): the pipeline fails earlier, at the pick's UP ("UP: not reached after 14.0 s: 7.4 mm and 1.4 deg"), so HOME is not reached in that run. The trim matters for the carry (the beaker's weight sags the arm without it). Other changes in that run: the grasp still closes (gripper_closed event logged), the place does not happen. So the setting-off run does not answer whether HOME completes without the trim, and it is not a clean HOME test.

## 3. CLOSE (moved layout, guard off)
Source: scratch/moved_probe.py on the moved layout (stance_select_moved_test's own task and stance selection). Two runs: data/logs/rep/moved_off (fingers' velocity not logged), data/logs/rep/moved_off2 (with finger velocity; the same run, same failure). The guard-on moved run: data/logs/rep/moved_on (no CLOSE: it fails at DOWN).

CLOSE window: 74.99 s (start) to 80.34 s (fail). Data (moved_off2):
- Pad relative to the beaker's centre at CLOSE start: dx −0.9 mm, dy +1.4 mm (1.7 mm horizontal); dz +54.9 mm. The grasp target (IN, pad on the beaker axis at 0.525 H) is dz +52.5 mm, so the pad is 2.4 mm above the target and 1.7 mm off in plan.
- Beaker: start (0.8500, 0.6600, 0.9598) m, tilt 0.03°; end (0.8501, 0.6601, 0.9602) m, tilt 0.26°. It moved 0.4 mm.
- Fingers (finger joint 1, mm): 40.9 at 74.99 s → 37.5 at 76.06 s, then fixed at 37.5 mm (finger 2: 40.9 → 38.0). Travel stays in the held range (3–40 mm).
- Gripper command (ctrl): 0.0 throughout (the close command). Gripper actuator force: −16.0 N throughout (the cap: 8 N per finger, two fingers). Applied gripper travel 0.0 mm, applied grip force cap 8.0 N.
- Contact forces, finger to beaker: 0 N at 74.99 s; 16.0 N from 76.06 s to the end (max 31.6 N in the closing transient). Finger to hotplate, bench, or any other body: 0 N throughout.
- Finger speed (instantaneous qvel, joint 1): 0.013 mm/s mean and 0.13 mm/s max from 76.0 to 78.0 s; 0.03 mm/s mean and 0.21 mm/s max from 78.0 to 80.4 s. The stall threshold is 2 mm/s (stall_speed_m_s 0.002) for 0.3 s (stall_dwell_s).

Which it is: the fingers **closed on the beaker and were blocked by it** (travel 37.5 mm, the beaker's diameter; 16 N of contact per the probe; the fingers are at rest). They did not miss, and the hold was steady.

Why the module said "did not stop within 5.4 s": the module did not look at the fingers in that window. A debug copy (scratchpad/rep/cpStall, a print added to the copy only) shows the module's CLOSE check ran at 74.99–75.08 s (its first 0.09 s, speed falling from 0.9 to 0.0 mm/s, still set at 75.04 s), then not again until 80.34 s, the timeout. The step function's own calls stop at 75.08 s and resume at 80.34 s (STEPENTRY debug print). In between the control loop holds the source for the weigh (src/control_loop.py, the weigh pause, src/weigh_helper.py): the grasp is weighed (STILL → SETTLE → LIFT at the set rate) and the source is not asked for commands until the lift-off (the log line "[weigh] left: lift-off at tick 11841" = 80.33 s in this run). The weigh pause ran 75.08 to 80.33 s (5.25 s). The close timeout is 5.0 s from 74.99 s (close_timeout_s), so it expired during the pause: the first check after the pause was at el = 5.35 s and failed. In the unmoved pick the same weigh ended at tick 11738 (about 78.3 s), inside the timeout, and CLOSE passed.

Guard on vs guard off at the start of CLOSE: the guard-on moved run never reaches CLOSE. It fails earlier, at the pick's DOWN ("DOWN: not reached after 17.0 s: 146.7 mm"), then at the recovery's BACK (82 mm), as in the earlier working-copy result. So there is no CLOSE for the guard-on run to compare. The difference is that the guard-on pick cannot reach DOWN, and the guard-off pick reaches CLOSE and is then stopped by the close timeout during the weigh pause.

## Not done
- Replay mechanism: the difference is deterministic and follows the guard-off trajectory, but I did not localize the recording gap (the ctrl mismatch on left_joint7 at the checkpoint). Reason: needs a per-tick comparison of the recorded and live ctrl rows, which is a code change to the recorder or a new instrumented run beyond this task.
- Setting-off HOME test: the run fails at UP first, so HOME was not reached; a clean test would need the trim off only for HOME, which needs a code change (not done).
- The CLOSE stall check's exact tick rule in the weigh pause: described from the call pattern (STEPENTRY and STALLDBG debug copies), not from a code walk-through of every branch.
- Hand orientation during CLOSE: not logged; the CLOSE data above are fingers, forces and positions only.
- Merge: no real merge was made; the test merges are in a temporary clone. A real merge will still need a hand resolution of ELBOW_GUARD_CFG and the shared function in mpvi_modules.py.
- Branch content inspection: the sag branches' content was read through hunk headers and grep only.

## Decisions for David
1. The sag branches predate the elbow guard: merging either one deletes ELBOW_GUARD_CFG, including M278, unless the conflict is resolved by hand. Decide which branch to merge and how to keep M278.
2. The close timeout (5.0 s, PICK_PLACE_CFG close_timeout_s) is shorter than the weigh pause in the moved scene (5.25 s). Decide whether the close timeout should start after the weigh (a change of design, not made here).
3. The sag trim's joint-only HOME behaviour (the layer's IK re-solve): decide whether the trim should apply to joint-only moves. Not changed.
4. Whether the replay gap (ctrl recorded vs live) needs fixing before the replay check is trusted for guard-off runs.

## Files
- This report: docs/reports/replay_home_close_20261108.md
- Replay runs: data/logs/rep/{a1,a2,b1,b2,c1,c2,d1,d2,trace,sagoff}.log and the kept run dirs; status data/logs/rep/status.txt
- Copies: scratchpad/rep/cpHoff (hooks off, guard off), cpGon (guard on, hooks on), cpGonHoff (guard on, hooks off), cpSagOff (trim off), cpStall (debug print in mpvi_modules.py, CLOSE stall inputs and step entries)
- HOME data: data/logs/rep/trace/episode/episode_log.csv, signal_trace.csv; setting-off: data/logs/rep/sagoff/
- CLOSE data: data/logs/rep/moved_off2/moved_probe.csv (guard off, with finger velocity), data/logs/rep/moved_off/, data/logs/rep/moved_on/ (guard on)
- Scratch: scratch/moved_probe.py (added finger velocity column), scratch/close_analysis.py, scratch/run_rep_20261108.sh, scratch/run_moved_off2.sh
- Merge test clone: scratchpad/merge_test (branch ours-test with our diff as a test commit; no effect on the real repo)

## Commit state
No commits made.
