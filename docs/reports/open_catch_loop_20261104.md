# Catch-the-open helper moved into the control loop, chunk player, beaker gravity setting (Nov 4, 2026)

Labels: **shown by the code/data** (file and line), **calculated**, **guess**, **NOT BUILT, needs decision**.

## Part 1 (read-only, written before any change)

### 1.1 The script's OPEN step, its gripper-open wait and its timeout

Source: `src/mpvi_modules.py` lines 411 to 442 (kind "open"), `src/task_config.py` lines 444 to 447 (timeouts and stall settings).

- **Today:** at each tick of the open step the script sets the held-mass target to 0 (`hmr.set_target`, line 414) and returns the closed gripper while the ramp runs (lines 415 to 416). When the ramp is finished it sets the open command (line 417, `open_g` = 0.044 m) and waits for the fingers: stopped (speed below 0.002 m/s for 0.3 s, lines 419 to 422) and open (within 3 mm of 0.044 m, line 423). Then it fires `gripper_opened` (line 435), resets the sag correction (line 437), and advances.
- **Timeout:** `open_timeout_s` = 4.0 s, counted from the step's own start (`el = t - t0`, line 440).
- **Blocked open for about 1 s (the helper case):** the step is not stepped while the loop holds the stream. Its clock is time-based (`t - t0`), so it keeps running. Calculated: ramp 1.0 s, plus the open travel (about 0.1 s), plus the 0.3 s stall dwell, is about 1.4 s, well under 4.0 s. The timeout is **not** hit.
- **Misreading:** the stall timer (`S["still"]`) is set only inside a step. It is reset when a step advances (line 490), so the first step after a pause starts a new 0.3 s dwell. Result: the script's wait ends about 0.3 s after the fingers stop, even if the loop already waited for them. This is a double dwell, not a wrong answer. Reported in Part 2 with numbers.

### 1.2 The drop monitor during the hold

Source: `src/mpvi_monitors.py` lines 29 to 70; called from `src/orchestrator.py` lines 238 to 247.

- The monitors run **inside the orchestrator's per-tick step**, after the segment's step. A source that is not stepped therefore does not tick the monitors.
- The drop monitor is armed by the `gripper_closed` event and disarmed by `gripper_opened`. It fires when finger travel stays below the held width minus a margin for a dwell (lines 46 to 66). During the ramp the fingers are still on the beaker, so it should not fire, unless the grasp fails (which the holding check handles).

### 1.3 Where the helper should sit in the loop

Source: `src/control_loop.py` lines 1028 to 1076.

- Each tick the loop calls each source (`out = src["step"](obs)`, line 1031), validates each command in its output (a dict or a list), and collects them into `cmds` (lines 1036 to 1059). Then `to_apply` is chosen (line 1069) and each command is applied (`bridge_take`, then `apply_cmd`, lines 1072 to 1075).
- **The helper belongs between the source and `to_apply`:** it needs the source's commands (to see an open) and it must stop the source being asked. The best point is the source loop: before `src["step"](obs)` (decides whether to ask) and after the commands are collected (decides whether an open is caught).
- The loop already accepts a list from a source and applies all of it in one tick, last command wins (earlier finding, `apply_cmd` lines 554 to 575). So a chunk must be played by a player, not applied as one list. A dict with `"mode": "chunk"` is a new shape; a list keeps its old meaning, so existing sources are unchanged.

### 1.4 Every place the held-mass compensation is computed or applied

| Place | What it does | Note |
|---|---|---|
| `src/gravity_ff.py` lines 118 to 124 (`torques`) | `m_held = held_mass_ramp.mass_at(...)`; adds `m_held * G * J_z` to the torque | **the one point where the beaker's term enters the torque** |
| `src/gravity_ff.py` lines 48 to 49 (`held_mass`) | reads the record's `now` | read only |
| `src/gravity_ff.py` line 151 (`held_pad_z_per_kg`) | pad height per kg, from the model | used by lift-off weighing |
| `src/control_loop.py` lines 925 and 930 | `ff_off = gff.torques(data) / gff.kp`, then `gff.apply_offsets(data.ctrl, ff_off)` | every tick |
| `src/control_loop.py` lines 405, 430 | `gff.apply(data.ctrl, data)` during settling | at start only |
| `src/held_mass_ramp.py` (`set_target`, `start_rate`, `stop_rate`, `mass_at`) | the mass record: rate and target ramps | state, not torque |
| `src/mpvi_modules.py` lines 450 to 470 (weighing) and 484 (handoff) | start the rate ramp, set the estimate | state, not torque |

**MuJoCo gravity compensation (gravcomp):** not in use. `robot_specs.py` line 164 records that the switch back to MuJoCo's gravcomp was deleted on 2026-10-26, and the XML contains no `gravcomp` attribute (search of `models/robot/combined_robot.xml` and `src/`).

**The setting goes at one point:** `gravity_ff.py` line 118 to 124, inside `torques`, where the held mass is multiplied into the torque. The arm's own gravity (lines 107 to 116) is above it and is not touched. `mass_at` is still called there, so the record's state and its logging keep running.

### 1.5 Chunks today

- The scripted stand-in emits one command per tick. No source returns a chunk. A chunk would enter the loop as a new dict shape (see 1.3).
- Overlapping commands today: the newest arm target replaces the stored one; the bridge keeps only the latest target per arm (`bridge_take`, lines 623 to 630).

## Part 2: what was built (commits A and B)

Commit A, `a852fbc`: the helper moved into the control loop (M253), the chunk player, the dead M252 structure removed, the holding rule shared, the decisions log, the design plan marker and the progress entry.
Commit B, `6450105`: the beaker gravity setting (M254) and its synthetic case.

Built as decided (decisions 1 to 8 of the brief):
- The helper sits in `src/control_loop.py` between each command source and the arm and gripper. Always on; no switch.
- The OPEN step of the script is a plain open command (`src/mpvi_modules.py`, OPEN branch). Its close and open stop test is the shared `stall_done` in `src/open_catch.py`.
- The holding verdict is `HoldDetector` (`src/holding.py`), updated once per tick per hand before the sources step. The signal trace uses the same class.
- Catch: `OpenCatch.on_open` (only while `holding`); `SourceGate.filter` discards the rest of that source's output and its chunk, and the source is paused (not asked) until the hold ends.
- Ramp-out: `held_mass_ramp.set_target(0)`, the existing 1.0 s ramp and the existing sag-trim freeze (the freeze reads the ramp, unchanged).
- Release: `apply_cmd` with the gripper target and the hardware ceiling force cap (an omitted `max_force` gets the ceiling in `src/commands.py`, so the release matches a plain open).
- Stop test after the release: `OpenCatch.step`, the same `stall_done` (speed under 0.002 m/s for 0.3 s, and finger at or above 0.041 m).
- Source resumed the tick after the stop test is done (sources step before the hold steps in the tick).
- Repeat open: ignored, logged `repeat_ignored`. Different open: kept the first, logged `different_ignored`. Grasp failed in the ramp: `failed_hold`, the caught open dropped, the source resumes at once.
- Chunk player: `src/chunk_player.py`, the first `CHUNK_CFG["play_count"]` (10) commands at 30 per second (one every 5 ticks at 150 Hz), then the source is asked again. A chunk is a dict with `mode: "chunk"`; a command, a list or None is handled as before.
- Gravity setting: `BEAKER_GRAVITY_CFG["enabled"]` (default True) in `src/gravity_ff.py` torques: the held term only.

Deletions (the M252 structure and the dead code), all in commit A unless noted:
- `src/open_catch.py`: the M252 `OpenCatch` class (its `__init__(start_ramp, ramp_done)`, `request`, `ramp_finished`) and the M252 `ChunkPlayer` class (`__init__(items, gripper_opened, start_ramp, ramp_done)`, `send`, `tick`).
- `src/task_config.py`: `OPEN_CATCH_CFG["enabled"]` (the switch, default False). The block is now `OPEN_CATCH_CFG = {"open_command_min_m": 0.02}`.
- `src/mpvi_modules.py`: the import of `OPEN_CATCH_CFG`, the import of the M252 `OpenCatch`, the `catch_i` resets in both skill resets, the `if OPEN_CATCH_CFG["enabled"]` branch with its `catch` request and its `ramp_finished` wait, AND the else branch that set the held mass to 0 and waited for the ramp (the pre-M252 M221 wait). The OPEN branch's own stop-test lines are replaced by the shared `stall_done`.
- `src/signal_trace.py`: `hold_count` and `holding` dicts and the inline squeeze, closure and hold lines (now `HoldDetector`).
- `src/control_loop.py`: the one-line `src["step"](obs)` loop is replaced by the gate calls (the loop body keeps its request_teleop, validate and apply path). The run's return value gains `open_catch_log`.
- `experiments/open_catch_test.py`: the M252 test functions (`make_player`, `run`, `case_a` to `case_e`) are replaced by cases a to h and the config check.

Open question found while building (NOT decided; for David):
- The pre-M252 OPEN step always set the held mass to 0 and waited for the ramp before the open (M221), whether or not the hand was holding. Decision 3 says that when NOT holding the open passes through with no delay, so the new OPEN step does no ramp at all, and the catch only runs while holding. If the holding verdict is off at an open while a held mass is still on (for example, a grasp that slipped out of the closure range), the open now passes at once with the feedforward still carrying that mass. The 200 g scenario never takes this path (the hand holds at the release), so the scenario check does not test it. Options for David: (a) keep as decided; (b) ramp to 0 before the open even when not holding (this delays a non-holding open, against decision 3); (c) ramp only when a held mass is above zero and not holding.

Choices made by me (implementation details, listed for redirection):
- The catch returns the open's gripper position only. The source's own `max_force` on the open is not kept; the release uses the ceiling.
- A caught open does not affect the source's commands after the hold (its chunk is dropped; the next ask gets a fresh chunk).
- A chunk played from a new ask starts on the ask's frame, so its first command plays at once and the next ones 5 ticks later (frame 46 in case f, not 50).

## Part 3: checks

### 3A Synthetic checks (`experiments/open_catch_test.py`, 9 of 9 pass at commit 6450105)

- (a) open mid-chunk: the earlier command plays (tick 0); the open at tick 5 is caught; nothing is sent during the hold; one release at tick 155 (the ramp is 1.0 s); the source is asked at tick 0 and at tick 202 (the tick after the resume at 201); pass.
- (b) open while NOT holding: passes at tick 5, no hold, no log entry; pass.
- (c) repeat open during the hold: logged `repeat_ignored`; ramp start and target unchanged; pass.
- (d) holding verdict off during the ramp: `failed_hold` logged; no release; commands after the failure pass through; pass.
- (e) different second open: first kept (0.044), difference logged `different_ignored`; pass.
- (f) chunk player: 10 commands at ticks 0, 5, ..., 45 (30 per second at 150 Hz), then the source is asked again and its next command plays at tick 46; pass.
- (g) single command or list: same objects returned, nothing paused, no chunk; pass.
- (h) beaker setting, real robot model at its home pose, 0.2 kg held: OFF torques equal the arm-only torques exactly (atol 1e-12); ON minus OFF has a largest entry of 0.000329 N m (the held term); pass.
- config: open threshold 0.02 m, play count 10, squeeze 2 N; pass.

### 3B Scratch 200 g run (new code at 6450105; not a baseline; baselines untouched)

Run: scratch worktree `/tmp/open_catch_wt2` at 6450105, `OPENARM_SIGNAL_TRACE=1`, inside `run_capped.sh` and `flock` on the MuJoCo lock, tmux `oc200`. Exit 0 at 18:25:41. Log `data/logs/open_catch_200g_20261104_182135.log`. Output `/tmp/claude-1000/-home-vboxuser-projects-openarm-tracer-sim/2d5c8cd9-deb3-42cb-a41b-078f148b4229/scratchpad/run_200g_new/` (outputs: liftoff_ticks.csv, episode/signal_trace.csv, episode/episode_log.csv). Full per-column comparison: `/tmp/claude-1000/-home-vboxuser-projects-openarm-tracer-sim/2d5c8cd9-deb3-42cb-a41b-078f148b4229/scratchpad/compare_200g_out.md`.

Comparison references: the registered baseline `experiments/baselines/held_200g_20261103/` (liftoff_ticks.csv and episode_log.csv; liftoff_ticks has the LEFT hand only) and the stage 1b trace `data/signal_trace_1b_20261103/r2_on/run/episode/signal_trace.csv` (both hands, pre-M249 to M251 verdict code).

Event ticks (tick numbers in the signal trace; "new" = this run, "1b" = stage 1b):

| event | left new | left 1b | right new | right 1b |
|---|---|---|---|---|
| release decision (OPEN start) | 12864 | 12864 | 12864 | 12864 |
| held mass reaches zero | 13013 | 13013 | 12864 | 12864 |
| gripper open command | 13014 | 13014 | 12864 | 12864 |
| holding turns off | 13014 | 13015 | 12864 | 12864 |
| finger reaches open (travel at least 0.041 m) | 13017 | 13017 | 12864 | 12864 |
| BACK start | 13074 | 13073 | 13074 | 13073 |

The right hand is not holding in this scenario (its held mass is 0 at the release), so its events fall on the release tick.

Hand motion, from release (12864) to BACK start (sag = pad z at release minus lowest pad z in the window; sag at BACK = pad z at release minus pad z at BACK start):

| quantity | left new | left 1b | right new | right 1b |
|---|---|---|---|---|
| sag, lowest point | 1.664 mm (tick 13074) | 1.663 mm (13073) | 0.0143 mm | 0.0141 mm |
| sag at BACK start | 1.664 mm | 1.663 mm | 0.0143 mm | 0.0141 mm |
| peak hand speed | 12.42 mm/s at 12909 | 12.42 mm/s at 12909 | 0.050 mm/s at 13016 | same |
| peak hand acceleration (absolute) | 0.755 m/s^2 at 12911 | same | 0.00337 m/s^2 at 13015 | same |

End state: the new run has one more tick (signal trace 14383 data rows against 14382 in stage 1b; liftoff_ticks 14382 against 14381 in the baseline; episode_log 959 rows in both). The episode log differs from the baseline in many columns from row 872 (87.2 s) to the last row (958); the largest difference is 0.0023 rad (left joint 1). The end state does NOT match byte for byte.

Every differing tick (new against stage 1b, signal trace, counts of ticks that differ):
- Holding and squeeze (M251, a change in the code, expected): left squeeze_ok and holding_verdict differ at tick 13014 only (1 tick); the left holding turn-off moves from 13015 to 13014.
- Closure flag (M249, expected): closure_in_range differs on ticks 0 to 1167, both hands.
- Still verdict (left): differs on 6 ticks (13099 to 13883) because the motion diverges (below).
- Step label: 3 ticks, first 13073 and last 14081 (the OPEN to BACK change is one tick later in the new run).
- Motion (both hands, pad, joint and speed columns): first differs at tick 13075 (the first tick after BACK starts, one tick later), last at 14381. Largest left differences: joint 4 0.00346 rad, pad y 0.496 mm, hand acceleration 0.0763 m/s^2. Largest right differences: joint 1 speed 0.0000103 rad/s; gripper actuator force 0.0000246 N.
- Force columns: left gripper actuator force differs on 1246 ticks (largest 0.000289 N), first at 13075.

Liftoff ticks (new against the registered baseline, left hand): the same tick range as above (pad position first differs at 13075, largest pad y 0.496 mm; the 1b trace already differs from the baseline by about 1e-8 m at 13074). Label and ramp columns differ only on the ticks listed in the file.

The BACK start is one tick later in the new run than in stage 1b (13074 against 13073). The fingers and the speed on those ticks are the same in both (finger 0.04393 m, speed 0.00015 m/s). The cause was not traced in this session.

### 3C Gravity setting OFF

No heavy run was made (per the brief). Check (h) above covers OFF.

### Not done in this report
- The M221 question (above) is not tested by this run: the hand holds at the release.
- Whether the 0.3 s stop test runs twice (once in the catch and once in the script's OPEN step after the resume) was not measured.

