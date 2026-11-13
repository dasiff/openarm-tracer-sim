# Pedestal height and the HOME stall: read-only trace (2026-11-09)

Read-only. No simulator run, no code, config, test or doc change, no commit, no push. This report is untracked.
Helper scripts (read-only, outside the repo): /home/vboxuser/projects/oc_scripts/trace_runs.py, home_shape.py, home_signals.py, home_final.py, ped_check.py; outputs trace_runs_out.md and overnight_out.md in the same folder.
HEAD at the time of the trace: 9ff0099d515c66e9507b4b4cb2de89f45944c02e. Working tree: unchanged (the untracked files listed in git status were already there).

Time and tick convention (checked against the recording labels): tick ≈ 150 × (t − 1.4 s). Examples: HOME starts at tick 13258 (label) = 89.8 s; the replay mismatch at tick 11145 = 75.71 s.

---

## Part 1: pedestal height

### 1.1 Every place a pedestal height is set, read or overridden

| # | file:line | value | role |
|---|---|---|---|
| 1 | src/task_config.py:86 (TASK_CFG["pedestal_q"]) | 0.1550 | reference stance. Used by the pipeline only with navigation off (stance_select.py:465) and by tests and map tools |
| 2 | src/task_config.py:80 (TASK_CFG["staging_xy"]) | (0.7650, 1.0850) | reference stance, same role as row 1 |
| 3 | src/task_config.py:590 (BENCH_STANCE_CFG) | block start | the stance-selection config |
| 4 | src/task_config.py:596 (BENCH_STANCE_CFG["navigation"]) | True | navigation on: the selection's stance is used, not TASK_CFG's (row 1 is then not used) |
| 5 | src/task_config.py:597 (BENCH_STANCE_CFG["pedestal_override_m"]) | 0.16 | **the parked staging pedestal** (rows M197, M270). Read at stance_select.py:622 |
| 6 | src/stance_select.py:377 (quick search, `st`) | cand["q"] | the candidate's own height from the finder's range (not the override) |
| 7 | src/stance_select.py:418 (candidate list) | t["q"] | the candidate's own height, written to the report |
| 8 | src/stance_select.py:465 (navigation off) | task_cfg["pedestal_q"] = 0.1550 | the robot stays at TASK_CFG's stance (not the default path) |
| 9 | src/stance_select.py:579, 585 (full check) | cand_st["q"] | **the full check runs at the selection's own height**, not at 0.16 |
| 10 | src/full_check.py:354-355 (checker per pedestal) | stance[3] | the arm collision checker takes the stance's pedestal |
| 11 | src/stance_select.py:622 | ped = override if set, else st["q"] | **the override decision** (0.16 with the config as committed) |
| 12 | src/stance_select.py:623 (out["task_cfg"]) | pedestal_q = ped (0.16) | the value the pipeline runs with |
| 13 | src/stance_select.py:642 (rep["chosen"]) | st["q"] (0.12 in the logs) and task_cfg_q | report only |
| 14 | src/mpvi_pipeline.py:71 | task_cfg = sel["task_cfg"] | the pipeline takes the parked task_cfg (0.16) |
| 15 | src/mpvi_pipeline.py:116 (status panel "pedestal_q") | from stance_report "chosen" (the selection's 0.12) | **display only, and it shows the selection's 0.12, not the 0.16 that is run** |
| 16 | src/mpvi_modules.py:70, 90 (world model, navigation) | task_cfg (0.16) | the scripted staging gets the parked task_cfg |
| 17 | src/mpvi_modules.py:111 (staging start check) | compares world.stance() with task_cfg (0.16) | the staging accepts only the task's own stance and height |
| 18 | src/world_model.py:41 (stance) | task_cfg["pedestal_q"] (0.16) | the staging target |
| 19 | src/staging_source.py:467 | task_cfg["pedestal_q"] | footprint for the drive (0.16) |
| 20 | src/staging_source.py:704, 713 (PEDESTAL phase) | target = task_cfg["pedestal_q"] (0.16) | the ramp from the applied height (0.418) to the target |
| 21 | src/staging_source.py:986-990 (restore after a hand-back) | task_cfg["pedestal_q"] (0.16) | re-ramp after an operator hand-back (none in these runs) |
| 22 | src/robot_specs.py:62 (PEDESTAL_SPECS["target_height"]) | 0.418 | the start command (fully raised) |
| 23 | src/control_loop.py:407, 429 | hold target = 0.418 | the held start value during warm-up |
| 24 | src/control_loop.py:561, 578-579 | hold_qpos_targets[ped] = cmd["height"] | the only place the pedestal hold target is written (pedestal mode) |
| 25 | src/policy_source.py:74-75, 112-113 | pedestal from a policy's actuator | only if a policy owns the pedestal route (not in the scripted runs) |
| 26 | src/task_policy.py:10, 18-20 | "the base and the pedestal hold" after staging | design statement: no pedestal writes after the arm stage starts |

**Answer to "does the pipeline use the selected pedestal or force 0.16?"**: the pipeline forces the override. The search's selected height (0.12 for the chosen stance in every log) is replaced by `pedestal_override_m` = 0.16 at stance_select.py:622-623. The selection's own height is used only for the quick search and the full check (rows 6, 9). The status panel still shows 0.12 (row 15).

Also note: the comment at src/task_config.py:460 says the command bridge "default is OFF", but src/task_config.py:462-463 sets `"enabled": True`, and control_loop.py:594 merges that default. The bridge is on. The comment is stale (see section 5).

### 1.2 Which pedestal each check assumed, and which the robot used

| check | pedestal assumed | where |
|---|---|---|
| quick search (cheap check) | each candidate's own height (0.12 for the chosen stance) | stance_select.py:377, 382 |
| full check of the chosen stance | 0.12 (the candidate's height) | stance_select.py:585; logs: "[stance] run of steps [1, 2]: stance [0.775, 1.105] pedestal 0.12: full check PASS" in data/logs/rep/a1.log, c1.log, guardoff_mpvi_pipeline_test_20261108.log, overnight_20261109/A_pipeline_oc_a2.log, D1_pipeline_park012.log, D2_pipeline_both016.log |
| pipeline test's check (mpvi_pipeline_test) | the same code path (apply_stance_selection → select_bench_stance), so 0.12 | src/mpvi_pipeline.py:69-71 |
| robot actually used | **0.16** command (cmd_pedestal = 0.16) | every episode log (section 1.3) |

The staging stance **(0.775, 1.105, -90, 0.12) is not in TASK_CFG**. The committed TASK_CFG is (0.7650, 1.0850), heading -90°, pedestal 0.1550 (src/task_config.py:80, 85, 86). The (0.775, 1.105, 0.12) values are the selection's result in the logs (also in scratch/audit_B.py and scratch/in_move_probe.py). Which value wins where:

| quantity | value used at run time | winner |
|---|---|---|
| staging xy | (0.775, 1.105) | the selection (stance_select.py:623), because navigation is on (task_config.py:596). TASK_CFG's (0.7650, 1.0850) is not used |
| heading | -90° | both agree |
| pedestal | 0.16 | the override (task_config.py:597) beats the selection's 0.12 (stance_select.py:622) and TASK_CFG's 0.1550 (used only with navigation off) |
| pedestal shown in the status panel | 0.12 | the selection (mpvi_pipeline.py:116): a display error, not the run |

Overnight Part D (data/logs/overnight_20261109/, copies oc_d1 and oc_d2): oc_d1 parks at 0.12 (`pedestal_override_m` 0.12, copy line 588). oc_d2 runs each candidate's full check at 0.16 (copy stance_select.py:585). The D2 log still prints "pedestal 0.12" on that line, because the print uses cand_st["q"] and not the value checked. The D1/D2 logs have no episode data (section 6).

### 1.3 Actual pedestal through the run (saved episode logs)

Representative table: data/logs/rep/trace/episode/episode_log.csv (pedestal_m is the measured height; cmd_pedestal is the loop's applied command, both in metres). The same numbers, within ±0.00004 m, are in all 15 episode logs (table in section 1.4).

| step | t start (s) | t end (s) | pedestal start (measured) | pedestal end (measured) | cmd_pedestal start | cmd_pedestal end |
|---|---|---|---|---|---|---|
| 1/3 NAV: starting | 1.40 | 1.40 | 0.41745 | 0.41745 | 0.41800 | 0.41800 |
| 1/3 NAV: ARMS | 1.50 | 31.20 | 0.41745 | 0.41745 | 0.41800 | 0.41800 |
| 1/3 NAV: PEDESTAL | 31.30 | 34.30 | 0.41675 | 0.15945 | 0.41533 | 0.16000 |
| 1/3 NAV: DRIVE | 34.40 | 54.50 | 0.15945 | 0.15945 | 0.16000 | 0.16000 |
| 1/3 NAV: CREEP_IN | 54.60 | 66.00 | 0.15945 | 0.15945 | 0.16000 | 0.16000 |
| 2/3 PICK: OVER | 66.10 | 67.60 | 0.15945 | 0.15945 | 0.16000 | 0.16000 |
| 2/3 PICK: DOWN | 67.70 | 71.30 | 0.15945 | 0.15945 | 0.16000 | 0.16000 |
| 2/3 PICK: IN | 71.40 | 74.90 | 0.15945 | 0.15945 | 0.16000 | 0.16000 |
| 2/3 PICK: CLOSE | 75.00 | 79.60 | 0.15945 | 0.15945 | 0.16000 | 0.16000 |
| 2/3 PICK: UP | 79.70 | 82.50 | 0.15945 | 0.15945 | 0.16000 | 0.16000 |
| 3/3 PLACE: OVER | 82.60 | 84.50 | 0.15945 | 0.15945 | 0.16000 | 0.16000 |
| 3/3 PLACE: DOWN | 84.60 | 87.10 | 0.15945 | 0.15945 | 0.16000 | 0.16000 |
| 3/3 PLACE: OPEN | 87.20 | 88.50 | 0.15945 | 0.15945 | 0.16000 | 0.16000 |
| 3/3 PLACE: BACK | 88.60 | 89.70 | 0.15945 | 0.15945 | 0.16000 | 0.16000 |
| 3/3 PLACE: HOME | 89.80 | 102.60 | 0.15945 | 0.15945 | 0.16000 | 0.16000 |

Every pedestal command after staging: the only change is the PEDESTAL ramp (31.3–34.3 s), from 0.418 to 0.160. No command changes after 34.4 s in any run.

### 1.4 Command changes after staging (all episode logs)

| run | pedestal command after DRIVE starts | command changes | measured min / max after DRIVE |
|---|---|---|---|
| guardoff_probe, guardoff_mpvi_run, rep a1, a2, b1, b2, trace, sagoff | 0.16 | none | 0.15942 / 0.15949 |
| rep c1, c2, d1, d2 | 0.16 | none | 0.15942 / 0.15949 |
| overnight A base, a2, a3 | 0.16 | none | 0.15942 / 0.15949 |

Mock runs: data/logs/overnight_20261109/A_mock_oc_*_probe.csv have no pedestal column (not found, section 10).

Does anything in pick, carry or place change it? **No.** The scripted manipulation (src/mpvi_modules.py) and the mock policy (src/mock_policy.py) write no pedestal command (grep: none). The only post-staging pedestal writes are the staging restore after an operator hand-back (src/staging_source.py:972-993) and the policy, keyboard and teleop routes (src/policy_source.py:112-113, src/keyboard_source.py:241). None of these ran in these logs (no operator).

---

## Part 2: HOME and the elbow guard

### 2.1 HOME target and start, and the per-joint errors (guard-off runs)

HOME target = home_q_left (src/task_config.py:94): 22.598, -14.709, 15.531, 126.122, -3.746, 27.308, 13.018 deg.
HOME window: "PLACE: HOME" phase 89.80–102.60 s (129 rows, 0.1 s), move from 89.78 s, joint step from 93.66 s (1.0 s planned), failure at 102.66 s.

Nine guard-off runs have identical HOME numbers (same table, checked by hash): guardoff_probe, guardoff_mpvi_run, rep a1, a2, b1, b2, trace, overnight A a2, overnight A a3.

| joint | limit (deg) | HOME target | position at HOME start | largest error in HOME (deg, at time) | largest error after 94 s | final position | final error | margin to nearer limit at the end |
|---|---|---|---|---|---|---|---|---|
| 1 | [-200, 80] | 22.60 | -6.83 | 29.42 (89.80 s) | 2.97 | 25.56 | +2.97 | 54.44 |
| 2 | [-190, 10] | -14.71 | -4.95 | 14.74 (102.60 s) | 14.74 | 0.03 | +14.74 | **9.97** (inside the 10° margin) |
| 3 | [-90, 90] | 15.53 | 8.50 | 10.43 (91.40 s) | 8.42 | 7.11 | -8.42 | 82.89 |
| 4 | [0, 140] | 126.12 | 81.93 | 44.19 (89.80 s) | 0.01 | 126.12 | +0.00 | 13.88 |
| 5 | [-90, 90] | -3.75 | -5.15 | 8.09 (92.80 s) | 7.46 | 3.71 | +7.46 | 86.29 |
| 6 | [-45, 45] | 27.31 | -22.24 | 49.56 (89.90 s) | 15.31 | 12.00 | **-15.31** | 33.00 |
| 7 | [-90, 90] | 13.02 | -2.41 | 15.42 (90.00 s) | 0.19 | 13.02 | -0.00 | 76.98 |

Furthest off at the end: **joint 6** (12.00° against 27.31°, 15.31° short; 33° from its +45° limit). Joint 2 ends 14.74° over its target and sits 9.97° from its +10° limit, i.e. just inside the 10° margin. Joint 6 is the stuck joint: its measured position stops changing at about 93.5 s and its command at 93.6 s.

**Overnight A base** (the place's recovery HOME, window 96.80–110.70 s, 140 rows; the HOME move is inside "RETRY 2: arm to the zombie pose"):

| joint | HOME target | position at HOME start | largest error in HOME (deg, at time) | final position | final error | margin at the end |
|---|---|---|---|---|---|---|
| 1 | 22.60 | -37.05 | 59.65 (96.80 s) | 25.55 | +2.95 | 54.45 |
| 2 | -14.71 | -33.09 | 18.38 (96.80 s) | -0.02 | +14.69 | 10.02 |
| 3 | 15.53 | 53.96 | 38.43 (96.80 s) | 7.14 | -8.39 | 82.86 |
| 4 | 126.12 | 56.38 | 69.74 (96.80 s) | 126.12 | -0.00 | 13.88 |
| 5 | -3.75 | -37.27 | 33.53 (96.80 s) | 3.70 | +7.44 | 86.30 |
| 6 | 27.31 | -34.56 | 61.87 (96.80 s) | 12.04 | -15.27 | 32.96 |
| 7 | 13.02 | -29.45 | 42.97 (97.10 s) | 13.02 | +0.00 | 76.98 |

Joint 6 stalls at 12.04° here too. Its command stops at 102.4 s, the end of the joint step (101.71 s + 1 s).

**Runs with no HOME**: rep c1, c2, d1, d2 (guard on) and rep sagoff (trim off). The guard-on runs fail at PICK: DOWN or CLOSE, then the recovery's BACK, before any HOME ("recover (zombie pose): OPEN -> BACK -> HOME", failing at BACK with 82.0 mm). The sagoff run fails at PICK: UP. So **the claim "joint 6 is fine in every guard-on run" cannot be checked: no guard-on run reached HOME.**

### 2.2 Bridge: commands, rejections, suspensions

The bridge is **on** in every run: COMMAND_BRIDGE_CFG enabled True (src/task_config.py:463), and the loop's `command_bridge` is None (default), merged at src/control_loop.py:594. Limits: max_joint_velocity 0.5 rad/s (task_config.py:465), max_target_step 0.25 rad (task_config.py:466), max_tracking_error 0.25 rad (task_config.py:467).

| run | events | suspensions | resumes | rejections (0.25 rad step or joint limit) | tick range |
|---|---|---|---|---|---|
| guardoff_probe, guardoff_mpvi_run, rep a1, a2, b1, b2, trace, sagoff, overnight A base, a2, a3 | 0 | 0 | 0 | 0 | none |
| rep c1, c2, d1, d2 | 1 | 1 | 0 logged | 0 | suspended at t = 68.66 s (tick ≈ 10,089), left joint 4, command error -0.253 rad vs limit 0.250; no "resumed" line in the log, so it stayed suspended to the run's end (tick 14458, about 97.8 s) |

- **Commands sent and accepted: not counted in any log** (only events are logged; code: src/control_loop.py:595, 623-705). Not found.
- The HOME move was a **ramp, not a jump**. For all 7 left joints, the largest command step per 0.1 s row is 1.72, 0.90, 0.67, 2.32, 1.27, 2.69 and 0.89 deg (joint 6: 0.47 rad/s, under the 0.5 rad/s bridge limit). The move ran from 89.78 s to about 93.6 s (joint 6 command from -22.9° to 11.47°). The 1 s joint step at 93.66 s did **not** change joint 6's command (11.47° → 11.43°), and it stopped the other joints short too (command final: 25.65, -0.00, 7.03, 126.03, 3.65, 11.43, 12.46 deg).

### 2.3 What else was active during HOME (guard-off runs; the guard-on runs have no HOME)

Source: data/logs/rep/trace/episode/signal_trace.csv (every tick, 89.7–102.7 s, trace run = the same numbers as the other guard-off runs), the recording labels, and src/task_config.py.

| item | state during HOME | evidence / config |
|---|---|---|
| sag trim | **on**. Label "place: HOME" from 89.8 s to the end (1933 of 1946 rows; 13 rows "place: BACK" before) | SAG_LAYER_CFG enabled (task_config.py:739); sag layer correct(): src/sag_layer.py:96-117 (the corrected pose is re-solved by the arm's IK, which the docstring at sag_layer.py:23 says also happens on joint-only moves) |
| trim size | max \|C_p\| x 9.6 mm, y 3.8 mm, z 2.4 mm; max \|C_r\| x 19 mrad, y 19 mrad, z 57 mrad (3.3°) | the report guard_off/replay_home_close used 89.8–102.6 s and gave 10.2 mm and 61 mrad; my window is 89.7–102.7 s, so the small difference is the window |
| held-weight ramp | **not active**: held mass 0.000 kg throughout HOME | held_mass_ramp_s 1.0 s (task_config.py:430); ramp-in at the weigh (tick 11144) and out at the place's DOWN |
| speed ramp | phase-start ramp 0.5625 s (task_config.py:429; mpvi_modules.py:489-513), active from the start of the HOME move (89.78–90.34 s). Whether the 93.66 s joint step counts as a phase start: not checked | |
| bridge | on, no events in guard-off runs | section 2.2 |
| elbow guard | off (task_config.py:752); the clamp returns the targets unchanged (src/elbow_guard.py:81) | |

Guard-on versus guard-off in the HOME window: the trim, the ramps and the bridge settings are the same in both groups, because only the guard flag differs (section 9). The guard-on runs were in the pick's recovery during this window, so there is no HOME comparison.

### 2.4 Replay mismatch at tick 11145 (75.71 s)

- **Phase**: "2/3 PICK: CLOSE" (75.00–79.60 s), at the start of the weigh: the held-mass rate ramp started at tick 11144 (data/logs/rep/trace.log, "[held mass ramp] rate ramp started at tick 11144: 0.0510 kg/s"). **Before** the HOME stall: HOME starts at tick 13258 (89.8 s), about 2100 ticks (14 s) later.
- **Differences** at the checkpoint, tick 11250 (data/logs/rep/a1.log line 367): qpos 1.385e-4 (index 13); qvel 4.057e-4 (index 12); qacc_warmstart 5.093e-2 (index 91); **ctrl 6.660e-3 (index 8)**. The check reports the first mismatch at tick 11145 and compares states at checkpoints.
- **Actuator index 8 = left_joint7**: src/arm_actuators.py:26 says the actuators are the 2 wheel motors first, then the 7 joints of each arm; index 8 is the left arm's 7th joint. The qpos/qvel index names (13, 12, 91) are from the earlier report and were not checked against the model.
- Same mismatch in the trim-off run: tick 12306 (83.45 s), also in PICK: CLOSE (82.70–85.70 s) in data/logs/rep/sagoff/episode/episode_log.csv. Guard-on runs (c, d): no mismatch.
- Hooks are not the cause: a1 (hooks on) and b1 (hooks off) are identical.
- **Recorder and control loop, guard on versus guard off**: the code is the same in both. The recorder stores `block`, the ctrl from ctrl_block() (src/control_loop.py:1148-1154), which includes the guard clamp (control_loop.py:460-461). With the guard off the clamp returns the targets unchanged (src/elbow_guard.py:81), so the recorded rows are the unclamped targets, as in the guard-on runs with no clamp.
- **Gravity feedforward is added after recording**: ff_off = gff.torques(data)/kp is computed from the measured pose (src/control_loop.py:973) and added to each physics step (control_loop.py:978). The replay feeds the recorded block (control_loop.py:1017) and then runs the same `advance` path, so the feedforward is recomputed from the replay's own state. **So a ctrl difference on left_joint7 follows from a small state difference (index 13, 1.4e-4) and does not by itself show a gap in the recording.** The earlier report read it as a recording gap (replay_home_close_20261108.md, section 1). The code does not support that reading. Magnitude not computed: needs a run (section 9).

### 2.5 Joint 6 range and margin

- Joint 6 range: **±0.785398 rad = ±45.0°** (models/robot/combined_robot.xml:209, `openarm_left_joint6` range="-0.785398 0.785398"; the scene includes this file at models/scenes/chemistry_lab_beaker_hotplate.xml:9; models/robot/openarm_v1_reference.json agrees).
- HOME target 27.308°: 17.69° below the upper limit and 72.31° above the lower limit.
- Margin: joint_margin_deg = 10.0 (src/task_config.py:197). **The target passes the 10° margin (17.7° ≥ 10°) and is inside the range.** The stall at 12.0° is 33° from the upper limit, so the limit is not what stops joint 6.

---

## 3. Items the brief asked for that the logs cannot settle

- Why joint 6 stalls: the logs show the command stops changing at 93.6 s and the sag trim is on. The cause (the trim re-solving the pose on a joint-only move, as the earlier report claims) is consistent with the code and the trim values but not proven: the trim-off HOME test needs a run.
- The guard-on joint 6: no guard-on HOME exists (section 2.1).

---

## 4. Summary of each run's status (from the saved logs)

| run | guard | sag trim | hooks | code | outcome / furthest phase |
|---|---|---|---|---|---|
| guardoff_probe (20:14–20:16, 11-08) | off | on | n/a (probe) | real tree, commit 51e99a7 + uncommitted (meta: dirty) | place HOME failed (0.267 rad) |
| guardoff_mpvi_run (11-08) | off | on | elbow hooks on | same | place HOME failed |
| rep a1, a2 | off | on | elbow hooks on | real tree, 51e99a7 + uncommitted (meta: dirty) | place HOME failed |
| rep b1, b2 | off | on | elbow hooks off | copy scratchpad/rep/cpHoff (no git info) | place HOME failed |
| rep trace | off | on | signal trace on | real tree, 51e99a7 + uncommitted | place HOME failed |
| rep sagoff | off | **off** | signal trace on | copy cpSagOff | failed at PICK: UP |
| rep c1, c2 | **on** | on | elbow hooks on | copy cpGon (no git info) | failed at PICK: DOWN / CLOSE, then recovery BACK |
| rep d1, d2 | **on** | on | elbow hooks off | copy cpGonHoff (no git info) | failed at PICK: DOWN, then recovery BACK |
| overnight A base | off (config line 743/747 of the copy: enabled False) | on | n/a | copy oc_base (has approach_angle, M279) | place DOWN failed, then recovery HOME failed (0.267 rad) |
| overnight A a2, a3 | off (copy config enabled False) | on | n/a | copies oc_a2, oc_a3 (have approach_angle) | place HOME failed (identical numbers to the rep runs) |

---

## 5. Differences between the guard-on and guard-off runs, other than the guard (section 9 of the brief)

| item | guard off (a, b, trace, probe, mpvi_run, overnight A) | guard on (c, d) | source |
|---|---|---|---|
| ELBOW_GUARD_CFG enabled | False | True | cpGon/cpGonHoff copies: src/task_config.py line 743 vs 752 of HEAD |
| code | real tree (51e99a7 + uncommitted; approach_angle.py not present; meta says dirty) for a and trace; copies (no git) for b, c, d | copies without git info (cpGon, cpGonHoff) | recording meta.json "git"; copy trees |
| elbow hooks (elbow_clearance_log monitor) | a1, a2 on; b1, b2 off | c1, c2 on; d1, d2 off | cpGon vs cpGonHoff differ only in experiments/mpvi_pipeline_test.py (the monitor line) |
| sag trim | on (sagoff: off) | on | SAG_LAYER_CFG; cpSagOff differs from cpHoff only in that flag |
| pedestal override | 0.16 | 0.16 | same config in all copies |
| run length | 15190 ticks | 14458 ticks | recording meta n_ticks |
| first failure | place HOME (guard-off, 102.66 s) | pick DOWN / CLOSE (guard-on) | pipeline logs |
| code version of the copies | older than HEAD: no approach_angle.py (M279), no stance_failure_log.py (M281), no STANCE_LOG_CFG | same | diff of copies vs HEAD |

- Copy-to-copy diffs (excluding bytecode): cpHoff vs cpGon: only ELBOW_GUARD_CFG enabled (line 743). cpHoff vs cpSagOff: only SAG_LAYER_CFG enabled (line 730 in the copy). cpGon vs cpGonHoff: only experiments/mpvi_pipeline_test.py (the monitor).
- The guard-off a-runs' exact tree is not stored (only "dirty" in the meta), so whether cpHoff equals the tree used for a1 at 20:45 cannot be verified.
- The overnight copies (11-09) have approach_angle and the guard off. Their HOME matches the 11-08 guard-off runs, although the place angle differs (overnight report, Part A).
- The HEAD config comment at src/task_config.py:460 says the bridge default is OFF; the code at 462-463 says ON. The bridge was on in all runs.

---

## 6. Overnight and the pipeline logs

- Overnight A (data/logs/overnight_20261109/A_pipe_oc_*_episode/episode_log.csv): all three runs keep the pedestal at 0.16 (section 1.4). Base fails at PLACE: DOWN, then its recovery HOME fails (section 2.1). a2 and a3 fail at HOME with the same numbers as the 11-08 runs.
- Overnight D1 and D2: the pipeline logs only (no episode_log.csv saved). D1 has `pedestal_override_m` 0.12 in its copy, D2 runs its full check at 0.16 in its copy. The printed "pedestal 0.12" in both logs is the selection's value (section 1.2).

---

## 7. Needs a run

1. **Bridge counts of commands sent and accepted**: a run with a counter, or a log line (no run exists with these counts).
2. **Guard-on joint 6 at HOME**: needs a guard-on run that gets through the pick and reaches HOME. The saved guard-on runs fail earlier.
3. **Sag trim off only during HOME** (the brief's item): needs a run with the trim off from the place's DOWN onward (the trim-off run fails at PICK: UP). This needs a code change to test, which this task did not make.
4. **Replay ctrl mismatch**: a run that logs the live ctrl including the feedforward term (tau_ff/kp) at each physics step, to confirm the state-driven explanation in section 2.4.
5. **Pedestal in the mock runs**: the mock probe CSVs have no pedestal column, so a run with logging is needed.
6. **D1 and D2 parked heights in the episodes**: a run with an episode log, to confirm the parked 0.12 (D1) and 0.16 (D2) heights.
7. **The trim's FK column blank after 93.6 s** in data/logs/rep/trace/episode/signal_trace.csv (left_sag_fk_minus_plan_z_m is empty from 93.67 s): the cause is not known; a run or a code read is needed.

## 8. Not found

- The TASK_CFG staging stance (0.775, 1.105, -90, 0.12): not in TASK_CFG (committed TASK_CFG is (0.7650, 1.0850, -90°, 0.1550)). The (0.775, 1.105, 0.12) values are the selection's result in the logs.
- Commands sent and accepted counts of the bridge: not in any log.
- Pedestal values for the mock runs: not in the logs (probe CSV has no pedestal column).
- Episode logs for the overnight D1 and D2 runs: not saved.
- Trim values during the guard-on HOME: none (no guard-on run reached HOME).
- Whether the 93.66 s HOME joint step counts as a speed-ramp phase start: not checked in code.
- Model names for the qpos/qvel/qacc indexes (13, 12, 91): not checked against the model (only the ctrl index 8 was checked against src/arm_actuators.py:26).

## 9. Differences between the runs: see section 5.

## 10. Not done, and why

- No simulator runs (the brief forbids runs; section 9 lists what needs one).
- No code, config, test or doc changes; no commits; no push; docs/progress.md was not updated, because the brief allows only this report file. The project rules in CLAUDE.md ask for progress.md and a committed report; the brief overrides both for this task.
- The HOME mechanism (trim re-solving the pose on a joint-only move) is taken from the code and the trim values; it is not confirmed by a run.

---

## Commit state

No commits, no pushes. HEAD is 9ff0099d515c66e9507b4b4cb2de89f45944c02e (unchanged).
