# Mock policy: the stand-in for the openpi VLA in the control loop (M260), Nov 5, 2026

Commits: c3e5719 (the mock, tests, decision row M260). Run progress: docs/progress.md.

## Part 1: findings (read before building)

### 1a. The real openpi call shape
- openpi is NOT installed on this machine (`import openpi` fails; no copy under the repository, deps/ or site-packages). So the call shape is taken from the design plan (section 7.3) and from my memory of openpi's public examples, which is NOT verified against the package.
- Design plan 7.3 (DECIDED): inputs and outputs "modeled on openpi's ALOHA and UR5 examples"; the state is built with the controller's actuator lookup; each hand's fingers collapsed to one 0 to 1 gripper value (direction "following openpi's ALOHA convention"); absolute joint position actions; every gripper command carries the max force from the grip force table; the policy server runs on PACE and the openpi client is a command source in the control loop; the harness plays the first N of 50 future commands at 30 per second (default 10, PROVISIONAL), then asks again; physics pauses while the policy thinks (default on).
- From openpi's examples (memory, unverified): the client calls `policy.infer(obs)` with a dict of state, images and prompt, and gets back a dict whose "actions" is an array of shape (action_horizon, action_dim). I found no extra fields in the design plan; the mock returns only the action array.
- The mock's call shape: `infer({"state", "prompt", "cheats"}) -> {"actions": (50, 16)}`.
- Where the real client plugs in: a source dict in the sources of src/control_loop.py with a `step(obs)` that returns a chunk dict (`{"mode": "chunk", "commands": [...]}`). The gate (src/open_catch.py SourceGate.output) asks the source only when its chunk has played out; the chunk player (src/chunk_player.py) plays play_count commands at the policy rate. Routing: the source owns the arm and gripper fields through the router (the loop's routes; the auto-mode route override of M260).

### 1b. How the scripted step machine turns into 50-step chunks
Position targets: each waypoint is a pad-centre position and an orientation (the move dictionary: target, R, speed). The scripted path is a min-jerk blend from the pad pose at the start to the target over T (the longest of distance over speed, turn over turn speed, and min_move_s), with the orientation slerped; the arm's joint command comes from MinkArm.step, one solver step per tick at 150 Hz. A chunk of absolute joint positions can be produced by running the same solver along the same path and keeping every fifth row (30 Hz). That is what the mock does.
- Command bridge (COMMAND_BRIDGE_CFG, 50 Hz, keeps the latest target per arm, 0.5 rad/s joint speed limit): takes the latest arms command, so a 30 Hz chunk passes through it unchanged. It does not depend on the script being called each tick.
- Sag trim (`_sag_trim_step`): an integrator on the MEASURED pad error, every tick, frozen during held-mass ramps. It depends on being called every tick and on the measured pose each tick. It does NOT fit a chunk of absolute targets without a design choice (NEEDS DECISION).
- Held-mass ramp (src/held_mass_ramp.py): time based (min-jerk on the simulation time), read every tick by the feedforward. Its start and target come from events (pick start, open, weighing). It survives the conversion, because the record carries the time.
- Weighing and the open catch: the helpers run in the loop each tick, not in the script; they survive unchanged (M253 to M258).
- Phase-start speed ramp (M195/M198, per tick joint speed envelope) and the blend handover (M200, decides at the end of a move): both are per-tick script logic. NOT reproduced by the mock (NEEDS DECISION whether a VLA chunk should carry them).
- Settle tests: the scripted module uses joint velocities each tick (settle_speed_rad_s for settle_dwell_s). The 16-value state has NO velocities; the mock uses differences between successive sensed states (at each query).

### 1c. Gripper values, direction, grip force
- The gripper is finger 1's travel divided by the open travel (0.044 m), clipped to 0 to 1: 1 = open. The direction convention is NOT defined in the repository (the design plan says only "following openpi's ALOHA convention"): NEEDS DECISION. Built as 1 = open, PROVISIONAL (MOCK_POLICY_CFG gripper_open_value).
- Joint directions: the arm joint positions are used in the robot's own sign convention (the same radians the actuators take); no mirror is applied to the right arm (the mock commands the left arm only). Whether openpi's ALOHA convention needs a sign or mirror change is NEEDS DECISION.
- Grip force: GRIP_FORCE_TABLE["glass_beaker"] (the force per finger the close command uses) is attached to every gripper command of the mock (design plan 7.3). The scripted OPEN carries NO force (the ceiling applies); so a mock open differs from the scripted open in the force cap (a difference reported in the run).

### 1d. Where a scripted behaviour does NOT survive the conversion unchanged
1. The sag trim (per tick integrator; not in the mock). NEEDS DECISION.
2. The phase-start speed ramp (M195/M198). Not in the mock. NEEDS DECISION.
3. The blend handover (M200): the scripted move hands over to the next move inside the blend radius; the mock waits for each waypoint's tolerance. NEEDS DECISION.
4. Settle test: joint velocities are not in the state; the mock uses differences between sensed states at query time. NEEDS DECISION (add velocities to the state, or accept the difference).
5. Move timing: the scripted move's T is fixed at the start of the step, from the commanded pose; the mock re-times the path at each query from the sensed pose (decision 4: "starting from the CURRENT sensed state on each query"), with the minimum move time applied again each time. The paths differ. NEEDS DECISION (keep the re-timing, or fix T per waypoint and carry it across queries).
6. Completion latency: the scripted module checks every tick; the mock checks each time the source is asked (every play_count commands, 0.33 s). NEEDS DECISION (ask more often, or accept).
7. Timeouts: the scripted timeouts are in simulation time since the step started. A helper pause does not stop the simulation, so a timeout keeps running during a pause. The mock keeps the same values; the run reports how close they came (NEEDS DECISION: whether timeouts should count pause time).
8. Gripper hold: while a gripper step is not active, the mock holds its GRIPPER TARGET (not the sensed value). This is needed so that a holding grasp (a finger travel of about 37 mm) is not read as an open command by the open catch (open means at least 20 mm). Built.
9. Open force: the mock's open carries the grip force; the scripted open carries none. Difference (see 1c).
10. Recovery (the orchestrator's retry through `recover`): not offered by the mock stage, so a failed mock step is not retried. NOT BUILT.
11. Right arm: the mock does not command it (the right values in the output rows are the sensed ones; the router sends the right arm to the policy).
12. Pause while thinking: the setting exists (MOCK_POLICY_CFG), but a query takes no simulated time, so it has no effect here. NOT BUILT as a wall-clock pause.

## Part 2: build (commit c3e5719)
- src/mock_policy.py (new): adapter_state (the 16-value state from the actuator lookup), chunk_commands (the chunk rows as commands, with the grip force table's max force on every gripper command), MockPolicy (planner wrapper: begin, infer, advance, memory), make_mock_manipulation (the orchestrator's stage; start() calls the scripted module's start() for the plan), make_mock_source (the chunk source "vla", with the labelled cheats).
- Wiring: make_pipeline_task(manipulation="mock") (src/mpvi_pipeline.py); module["plan"] (src/mpvi_modules.py); auto-mode route override cfg["auto_routes"] (src/control_loop.py); experiments/demo_video.py --manipulation mock (record-only). MOCK_POLICY_CFG in src/task_config.py (chunk_len 50; gripper_open_value 1.0 PROVISIONAL; pause_physics_while_thinking True, no effect in this synchronous sim).
- Launcher used for the held scenario: data/bounce_check_20261103/run_mass.py (the scratch copy) with manipulation="mock" in its Namespace, --liquid 0.100 (200 g); OPENARM_SIGNAL_TRACE=1.

## Part 3A: synthetic tests (experiments/mock_policy_test.py: 5 of 5; the earlier suites still pass)

| case | check | result |
|---|---|---|
| (a) | output exactly 50 by 16; absolute joints within 0.144 rad of the sensed state; gripper 0 to 1 | PASS |
| (b) | same sensed state and cheats give the same chunk | PASS |
| (c) | input is only state, prompt and cheats; cheats are clock, plan_memory, object_poses, other_joint_qpos | PASS |
| (d) | discarded chunk, re-query from a state short of the waypoint: same waypoint, plan index unchanged (end error 9.1 mm both times, the solver's lag) | PASS |
| (e) | chunk player plays 10, asks again at frame 46 (mock as source); every gripper command has a max force | PASS |
| earlier | weigh_helper_test 8 of 8; open_catch_test 12 of 12 | PASS |

Case (d)'s end-of-chunk error is the solver's tracking lag, not convergence; the check only bounds it at 20 mm.

## Part 3B: 200 g held run with the mock as the source (commit c3e5719; scratch, not a baseline; nothing registered)
Log: data/logs/held200_mock_20261105_024640.log. Exit 1 at 02:49:40. Outcome PLAN_FAILED: "OVER: not reached after 9.2 s: 7.8 mm and 4.9 deg from the waypoint [no retry: the manipulation module has no recovery]".

| quantity (left hand) | mock run | scripted run (M258 scratch, same settings) |
|---|---|---|
| OVER waypoint duration | 1379 ticks (9.2 s) until the timeout | 228 ticks (1.5 s), reached |
| holding ON | not reached (0 ticks) | 11014 |
| still ON | not reached (681 ticks of still, none with holding) | 11085 |
| weighing end, held mass set | not reached | 11788 |
| release start, held mass zero, gripper open, BACK | not reached | 12800, 12950 (13013 baseline), 13009, 13010 |
| pauses | none (no holding) | 774 ticks at 200 g |
| estimated mass / lift / drop / place swing | not available (the run stopped before the weighing; the M244 method needs the reference pad height, which the mock run does not log) | 0.2248 kg; M244 method: rise 4.059, drop -2.114, place swing 9.385 mm |
| queries made to the mock | not recorded (the mock counts them in its report, which the run did not write) | n/a |
| timeouts within 1 s during a helper pause | none (no pause happened); the OVER timeout expired at the failure | n/a |

Where and why: the first pick waypoint (OVER) never completed. The pad moved 66 mm in x, 47 mm in y and 22 mm in z before the timeout, but at 5.7 mm/s at the sample 150 ticks in, against 100 mm/s for the scripted move. Cause (diagnosed from the traces, not fixed): each query re-times the move from the sensed pose with the minimum move time of 1.0 s, and each chunk starts from zero velocity (min-jerk). The arm therefore crawls and the time limit (3 x T + 6 s = 9.2 s) expires 7.8 mm short. The scripted move reaches the same waypoint in 1.5 s because its duration is fixed once at the start of the step.

First difference from the scripted run: tick 9688 (the OVER start), in the ninth decimal place of the pad position; the gripper target first differs at 11000.

## Regression: the scripted path from c3e5719
Run at c3e5719 with the unmodified launcher (manipulation default "scripted"), log data/logs/held200_scripted_c3_*.log, exit 0, outcome SUCCESS. liftoff_ticks.csv, episode/episode_log.csv and episode/signal_trace.csv are byte-identical to the M258 scratch run (0 differing rows). So the mock commit leaves the scripted reference unchanged.

## Option A (M261): timing and progress, and the second mock run (commit 684a607)

### Change
- The waypoint duration is computed once, when the waypoint starts, by `move_duration` (the scripted rule, shared with the scripted module; the joint-limited dry run included) and `joints_duration`. Nothing retuned.
- Each query finds the arm's sensed progress along the waypoint's own path: the path time at which the path is nearest the sensed pad centre (401 grid points; HOME joints: nearest point in joint space). The next 50 steps continue from that progress at 30 per second, ending at the waypoint and holding it.
- Statistics: `mock_queries.csv` beside the run's files (query, tick, sim time, step, progress, path time, wall time, rows).
- NEEDS DECISION (M261): the progress measure is position only (orientation is not part of it).

### Synthetic tests (experiments/mock_policy_test.py, 9 of 9)
| case | result |
|---|---|
| (a) 50 by 16, absolute joints, gripper 0 to 1 | PASS |
| (b) same input, same chunk | PASS |
| (c) input = state, prompt, labelled cheats | PASS |
| (d) discarded chunk, re-query short of the waypoint: same waypoint | PASS |
| (e) chunk player plays 10, asks again at frame 46 | PASS |
| (f) 40 queries along an 8 cm forward move: joins against the largest in-slice step, joint space 0.90x (0.01079 rad against 0.01205), pad space 0.90x (2.24 mm against 2.49 mm); pad error after 40 queries 0.32 mm | PASS |
| (g) discarded chunk, re-query from a state behind the plan: progress 0.0000 s both times; re-query equals the first chunk (0 rad); the discarded chunk's end was 0.373 rad away | PASS |
| (h) re-query after a simulated 5 s pause, arm not moving: progress 0.6600 s before and after; first row 0.0092 rad from the sensed joints; plan index unchanged | PASS |
| (i) statistics file: one row per query, all columns | PASS |
Earlier suites: weigh_helper_test 8 of 8; open_catch_test 12 of 12.

Test-setup finding: from home the solver reaches about 3 cm up (a 10 cm lift stalls at 50.6 mm error, joints stop at 0.52 rad), and 8 cm forward or down converges to under 0.5 mm. The tests therefore move 8 cm forward.

### Run 2: 200 g held scenario with the mock as the source (commit 684a607; scratch; exit 1)
Log data/logs/held200_mock2_20261105_*.log; files in scratchpad run_200g_mock2/ (liftoff_ticks.csv, episode/, mock_queries.csv, recording/). Outcome PLAN_FAILED at segment 2/4: "IN: not reached after 15.3 s: 3.4 mm and 1.0 deg from the waypoint".

Pick steps (left, signal-trace label ticks), mock against the scripted run from the same commit:

| step | mock | scripted |
|---|---|---|
| OVER | 9688 to 10009 (322 ticks, 2.15 s) | 9688 to 9916 (229, 1.53 s) |
| DOWN | 10010 to 10561 (552, 3.68 s) | 9917 to 10468 (552, 3.68 s) |
| IN | 10562 to 12861 (2300, 15.3 s): FAILED | 10469 to 10999 (531, 3.54 s) |
| CLOSE, holding, UP, release, BACK | not reached | 11000 to 11789 (CLOSE), UP 11790 to 12237 |

Timeouts (limits from the scripted rule, timeout_factor 3 x T + 6 s):
- OVER: T 1.517 s, limit 10.55 s: reached at 2.15 s (no expiry, not within 1 s).
- DOWN: T 3.651 s, limit 16.95 s: reached at 3.68 s (not within 1 s).
- IN: T 3.019 s, limit 15.06 s: expired at 15.3 s (failure).
- CLOSE (5 s), gripper-open wait (4 s): not reached. No helper pause ran (no holding), so none expired during a pause.

IN stall: the path progress advanced to 2.589 s of 3.019 s and then stayed there (query 67 onward, 69 queries in all). The pad stopped 3.4 mm from the target (tolerance 3 mm). The final pad position is 2.95 mm (x) and 3.07 mm (y) from the scripted run's IN end, and 0.07 mm in z. The offset is horizontal, not gravity sag in z. Likely source: the scripted module's sag trim, which integrates the pad error on all axes and is absent in the mock. Not verified by a run.

Query statistics (69 queries, one every 46 ticks, the chunk length of 10 commands):
- wall time per query: mean 0.408 s, min 0.266 s, max 2.461 s (the first query, which includes the dry run), total 28.2 s.

Estimated mass, over-read, pauses, and the M244 bounce numbers: not available. The run never lifted the beaker (no holding, so no lift-off, no held mass, no release). The M244 numbers need the lift-off window and the held period, which do not exist in this run.

### Scripted reference from the same commit (684a607, unmodified launcher)
Exit 0, outcome SUCCESS. liftoff_ticks.csv and episode/episode_log.csv are byte-identical to the registered baseline held_200g_20261104_m258; the signal trace is byte-identical to the M258 scratch run (the baseline folder has no signal trace).
