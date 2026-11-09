# Catch the open command: Part 1 (read-only findings), then the build and checks (Nov 4, 2026)

Part 1 was written before any code was changed. Parts 2 and 3 follow it in this file. Labels: **shown by the data/code** (file and line), **calculated** (computed here, method stated), **guess**, **NOT BUILT, needs decision**.

Terms: a **command** is one dictionary sent to the loop for one tick (for example `{"mode": "arms", "arm_targets": ..., "gripper": ...}`). A **chunk** (in VLA usage) is a batch of future commands sent together. The **command bridge** is the limiter between the sources and the arm: it caps how fast the arm target can move (`src/control_loop.py`, bridge block at lines 582 to 586 and the functions at lines 554 to 575, 623 to 630 and 639 onward). **Feedforward** is the gravity torque added to each arm command (`src/gravity_ff.py`).

## Part 1: how commands reach the command bridge today

**1. Does the scripted stand-in produce chunks or single steps?** Single steps. The manipulation module returns one command dictionary per tick: `step(obs)` returns `out`, which is `command()` (`src/mpvi_modules.py` lines 306 to 309 and the `return out` at line 527). No module builds a list of future commands. The word "chunk" does not appear in `src/` in any control sense (search of `src/`: only unrelated chunked loops such as `src/reach_map.py`).

**2. Where would a chunk enter the code?** The loop (`src/control_loop.py`, lines 1028 to 1061) calls each source once per tick (`out = src["step"](obs)`, line 1031). A source may return a dictionary **or a list of dictionaries** (line 1036: `for cmd in ([out] if isinstance(out, dict) else out)`). The sequential policy's stage interface documents this: "The commands for this tick (a command dictionary, a list of them, or None)" (`src/sequential_policy.py` docstring line 11). So a list is the shape a chunk would use. Today nothing returns a list with future commands in it.

**3. What happens to a list that arrives in one tick?** Every command in the list is checked (bridge pre-check line 1041, validation line 1047) and appended to this tick's commands (line 1059). All of them are then applied in order in the same tick (lines 1069 to 1076: `for cmd in to_apply: ... apply_cmd(cmd)`). **The list is not queued and not played over time.** When two commands in one tick set the same arm target, the later one overwrites the earlier (`apply_cmd` lines 554 to 575 writes `hold_qpos_targets[arm_act[s]] = q` for each command in turn).

**4. What happens when a new chunk arrives while an old one is still playing?** There is no chunk, so there is no queue and no blending. Two cases exist today:
- **New arm target replaces the old one.** `apply_cmd` stores the latest target. The "interpolate" target mode restarts the ramp from the current control value when a new command arrives (`src/control_loop.py`, Target modes paragraph at lines 26 to 33 of the module docstring, and `apply_cmd` docstring line 559).
- **The bridge keeps only the latest target per arm.** `bridge_take` (lines 623 to 630) overwrites `bridge["target"][s]`. `bridge_update` (line 639 onward) then moves the applied command toward that latest target at most `max_joint_velocity_rad_s / rate_hz` per update (50 Hz, every third tick). So an old target that was not yet reached is dropped; the arm heads for the newest target at the bridge's speed limit.
- **Gripper commands** are stored directly (`apply_cmd`, `cmd.get("gripper")` loop): the latest value wins.
- **Exclusive mode** (`cfg["exclusive"]`, `exclusive_select` lines 699 onward) allows one mode per tick and keeps a pending command. This is a separate path; it is not a queue of chunk items.

**5. What "hold the hand at its current pose" corresponds to in today's OPEN step.** The open branch is `src/mpvi_modules.py` lines 411 to 418 (`kind` "open"; `closing` is false):
- **Held: the arm targets.** `command()` (lines 306 to 309) sends `arm.q_cmd`, the commanded joint targets. The open branch never calls `arm.step`, so `arm.q_cmd` does not change during the ramp (**shown by the code**: `arm.step` is called only in the move branch, line 380). The same targets are re-sent every tick.
- **Held: the gripper.** `S["grip"]` is not changed until the ramp finishes; the gripper stays at its closing command with the 8 N force cap (`command()` line 308 uses `S["grip"]`; the cap is set in the close branch line 417 and kept by `S["grip"]`). The open command is written at line 417, after `hmr.finished`.
- **Sag trim: frozen, by not running.** `_sag_trim_step` is called only in the move branch (line 376). The open branch never calls it, so the sag correction `C` does not change during the open step (**shown by the code**). The separate freeze rule also stops integration while a held-mass ramp runs (`_sag_trim_step` line 74, `hmr.ramp_running`), and the open branch's `hmr.set_target(0)` starts that ramp (line 414).
- **Arm feedforward: NOT held.** Each tick the loop recomputes the gravity torque from the measured pose and the held-mass term (`src/control_loop.py` lines 925 to 930: `ff_off = gff.torques(data) / gff.kp`, then `gff.apply_offsets(data.ctrl, ff_off)`). The held-mass part of the torque comes from the ramp, which is falling during the open step (`src/gravity_ff.py` lines 118 to 124). So the signal sent to the joints changes while the arm **target** is held. This is the sag seen in the logged run (the hand drops about 1.2 mm during the ramp-out).

**Summary of Part 1.** The scripted stand-in is a step machine that emits one command per tick. A VLA-style chunk would enter as a list (the loop already accepts that shape) but would be applied at once, with the last command winning, and an unfinished older target is replaced by the newest. Today's OPEN step already holds the arm targets and the gripper, runs the held-mass ramp, and leaves the sag correction alone. It does **not** hold the feedforward, so the hand still moves during the ramp.

## Part 2 plan (what will be built, and where)

Decision (David, Nov 4): the helper holds the stream, runs the same 1.0 s ramp-out with the same sag freeze, releases the open after the ramp, and keeps the rest of the chunk. Those behaviours already exist in the OPEN step (Part 1, point 5). The helper therefore:
- makes the open an explicit **request** (decision 1 and 6: a second request during the hold is ignored, so it does not restart the ramp);
- runs the existing ramp (`hmr.set_target` and `hmr.finished`, reused, not re-implemented);
- keeps the held arm and gripper targets (existing `command()`);
- releases the open once the ramp finishes, then uses the existing gripper-open wait (existing code, unchanged);
- keeps the remaining steps of the stand-in in their original order (the step list, unchanged).

Where it lives: `src/open_catch.py` (the helper: `OpenCatch` for the request, hold and release, and `ChunkPlayer` for the stream tests), wired into the OPEN branch of `src/mpvi_modules.py` behind the switch `OPEN_CATCH_CFG["enabled"]` (default False).

Stand-in change: none beyond the switch. The place's step list already has the open in the middle (`plan_place` line 247: OVER, DOWN, **OPEN**, then BACK and HOME at line 248), which is the shape a chunk would have at the level of steps. A stand-in that pre-generates future commands is **NOT BUILT** (see the report's NOT BUILT list).

Result expected from the design: with the switch ON the place should behave as it does today, because the request arrives at the same tick the step starts. The check (Part 3) measures this rather than assuming it.

## Parts 2 and 3: results

### Build (commits `343cb1c` and `3ae91ba`)

- **`src/open_catch.py` (new).** `OpenCatch`: one open request and its hold. `request(t, tick)` starts the hold (calls the existing ramp start, `held_mass_ramp.set_target` to 0) and returns False for a repeat during the hold (decision 6: no restart). `ramp_finished(t)` releases the hold when the existing `held_mass_ramp.finished` says the ramp is done. `ChunkPlayer`: plays a stream of commands in order; an open holds the stream until the ramp is done and the gripper-open wait has finished, then the rest of the chunk plays in order (decision 5). `ChunkPlayer` is used only by the synthetic tests (see NOT BUILT).
- **`src/mpvi_modules.py` (the OPEN step, lines 415 to 425).** With `OPEN_CATCH_CFG["enabled"]` ON: the first tick of an open step makes one `OpenCatch` request (keyed by the step index, `catch_i`, reset at lines 254 and 290); while the hold runs the step returns the same command as today (arm targets held, gripper closed); after the ramp, the existing gripper-open wait runs unchanged. With the switch OFF the `else` branch is the original two lines (`set_target`, then wait for the ramp). The import is at line 55; `catch_i` was added to both resets (lines 254 and 290).
- **`src/task_config.py` (`OPEN_CATCH_CFG`, lines 676 to 680).** One switch, `"enabled": False`, with a plain-language note. No new numbers: the 1.0 s ramp is `PICK_PLACE_CFG["held_mass_ramp_s"]` (reused). No value is PROVISIONAL because none was chosen here.
- **Stand-in change: none.** The place's step list already has the open in the middle (OVER, DOWN, OPEN, BACK, HOME, `plan_place` line 247 and 248). The stand-in still produces one command per tick, not a pre-generated chunk.
- **Decision row M252** (`docs/mpvi_decisions.md`, after M251) and a **correction note to M249** (after the M252 row; the M249 row is untouched; the note says the lower edge is on line 656, not 653).

### Part 1 answers (summary; full text above)

- The stand-in produces **single steps**: one command per tick (`mpvi_modules.py` `command()`, `step()` returns it).
- A chunk would enter as a **list** of command dictionaries from a source's `step()`. The loop accepts that shape (`control_loop.py` line 1036).
- A list is **applied in one tick**, in order; the **last arm target and gripper value win**. Nothing is queued or played over time (`control_loop.py` lines 1069 to 1076; `apply_cmd` lines 554 to 575).
- A **new target replaces the old** one: `apply_cmd` stores the latest target; the interpolate mode restarts its ramp from the current value; the **command bridge keeps only the latest target per arm** (`bridge_take` lines 623 to 630) and moves toward it at its speed limit. Nothing is blended and nothing is queued.
- "**Hold the hand at its current pose**" in today's OPEN step: the **arm targets** are held (`arm.q_cmd` is not updated because `arm.step` is not called in the open branch); the **gripper** stays closed; the **sag trim is not called** in the open branch (only in the move branch, line 376) and its ramp freeze stays on; the **arm feedforward is NOT held**: the held-mass term is recomputed every tick (`control_loop.py` lines 925 and 930) and falls with the ramp, so the joint signals change while the target is held. That is the sag.

### Synthetic tests (`experiments/open_catch_test.py`, no simulation; run at HEAD `343cb1c`)

| Case | What it checks | Result |
|---|---|---|
| (a) open in the middle | c1, c2 play; the open is held (5 hold ticks, 2 opening ticks); c3 and c4 play only after the gripper opened, in order | PASS |
| (b) open is the last command | nothing is sent after the open; the stream ends after the gripper opens | PASS |
| (c) no open | the chunk passes through unchanged; no ramp starts | PASS |
| (d) same open during the hold | the ramp starts once; the repeat is counted as ignored; the remaining command plays once | PASS |
| (e) switch OFF | default is False; the else branch is the original code (checked by parsing `mpvi_modules.py`) | PASS |

Total: 5 of 5 passed (exit code 0).

### Held run, switch ON, switch OFF and the existing run (200 g, scratch runs, not registered)

Scratch worktree at `343cb1c` (venv and mesh folder linked, six stance-finder data folders copied). Launchers: `run_off.py` is the baseline launcher with only its root path changed; `run_on.py` also sets `OPEN_CATCH_CFG["enabled"] = True`. Both runs with `OPENARM_SIGNAL_TRACE=1` (logging only). One at a time, under `run_capped.sh` (6 GB) and the project lock.

| Run | Exit | Wall clock | Peak memory |
|---|---|---|---|
| Switch ON | 0 | 3 min 32.55 s | 4,099,220 kB (4.10 GB) |
| Switch OFF | 0 | 3 min 37.52 s | 4,256,556 kB (4.26 GB) |

**Byte-identical:** `liftoff_ticks.csv` and `episode_log.csv` of the ON run and of the OFF run are identical to each other and to `experiments/baselines/held_200g_20261103/` (the existing 200 g run).

**Per-tick signal comparison:** every column of the per-tick signal trace (14,381 common ticks, left and right hands) is identical between ON and OFF, and between each of them and the existing run (stage 1b R2 trace; holding and squeeze from the closing-only replay). **No tick differs in any column.**

**Event ticks** (identical in all three; ON, OFF and existing):

| Event | Left hand | Right hand |
|---|---|---|
| Release decision (OPEN starts) | 12863 | 12863 (no right-hand motion) |
| Held mass reaches 0 | 13012 | 13012 |
| Gripper open command | 13014 | 13014 |
| Holding turns OFF (closing-only rule) | 13014 | never ON |
| Finger reaches 43.8 mm | 13023 | open throughout |
| BACK starts | 13072 | — |

**Ramp-out and opening, left hand** (identical in all three):

| Quantity | Value | Tick |
|---|---|---|
| Pad z at release (start of ramp-out) | 1083.887 mm | 12863 |
| Lowest pad z in the ramp-out window | 1082.464 mm | 13013 |
| Sag across the ramp-out (start minus lowest) | 1.423 mm | 12863 to 13013 |
| Peak hand speed during the ramp-out | 12.42 mm/s | 12909 |
| Peak hand acceleration during the ramp-out | 0.755 m/s² | 12911 |
| Peak hand speed during the opening | 5.12 mm/s | 13015 |
| Peak hand acceleration during the opening | 0.388 m/s² | 13014 |
| Pad z change during the opening | −0.093 mm | 13014 to 13023 |

Right hand, place window: pad z from 1139.351 to 1139.388 mm (0.037 mm range), peak speed 0.050 mm/s, peak acceleration 0.0034 m/s² in all three runs.

The earlier report quoted a 1.2 mm sag at tick 12912; the figure above is the full ramp-out window (to 13013), so it is larger.

**Whether the switch-ON path ran.** The log prints `OPEN_CATCH_CFG enabled = True`, and the OPEN step with the switch ON is the only code path that uses `OpenCatch`. The output matched the switch-OFF run to the bit, so the helper's path gave exactly the same result as the existing step in this run. No log line counts the requests; that is noted under NOT BUILT.

### Result in plain words

With the switch ON the scripted place produced the same numbers as with it OFF, and the same numbers as the existing run. That is what the design predicted: in the scripted place the open request arrives at the same tick the OPEN step starts, so the held-mass ramp and the sag freeze already ran exactly as David described. The check does not show a difference because none is expected here; it does not test a VLA's chunk, which arrives earlier and out of step (see NOT BUILT).

## Files

- Created and committed: `src/open_catch.py`, `experiments/open_catch_test.py` (commit `343cb1c`).
- Changed and committed: `src/mpvi_modules.py`, `src/task_config.py`, `docs/mpvi_decisions.md` (`343cb1c`; the M249 note in `3ae91ba`).
- Created, uncommitted: `docs/reports/open_catch_20261104.md` (this report).
- Scratch, outside the repo: `/tmp/open_catch_wt` (removed), `/tmp/open_catch_runs/` (`on/`, `off/`, `logs/`, `compare.py`, `compare.md`).
- Not touched: `experiments/baselines/`, `deps/`, the baseline index.

## Commits

| Commit | What it changed |
|---|---|
| `343cb1c` | Catch the open command (M252): new `src/open_catch.py`; switch `OPEN_CATCH_CFG` (default OFF) in `src/task_config.py`; OPEN step wiring in `src/mpvi_modules.py`; new `experiments/open_catch_test.py`; decision row M252 |
| `3ae91ba` | Correction note to M249: its line citation (653) is line 656 in its commit |

Not pushed. `origin/main` is still `58e756f`.

## NOT BUILT, needs decision

1. **A pre-generated chunk.** The stand-in still emits one command per tick. A VLA-style chunk (a batch of future commands produced at once) would need a change to how the script works; not made.
2. **The chunk player is not wired.** `ChunkPlayer` is used only by the synthetic tests. The real OPEN step uses `OpenCatch` directly.
3. **A new chunk arriving while the helper holds.** Not covered by the decisions. The code does not queue it; the helper ignores everything except repeats of the open. Default behaviour: the arm and gripper stay held until the ramp finishes; a new arm command during the hold is not applied.
4. **A grasp failing during the hold.** Not covered. Default behaviour: the drop monitor (`src/mpvi_monitors.py`) is armed from the gripper-closed event to the gripper-opened event, so it stays armed during the hold. It decides on finger travel. Whether a "dropped" result should stop or change the hold is not decided, and the hold code does not read the monitor.
5. **An open command that differs from the first.** Not covered. Default behaviour: a second open request during the hold is ignored, whatever its gripper target.
6. **The switch-ON path is not counted in the log.** The identical outputs show the path gave the same result, but no log line counts requests. A counter would be a logging change.
7. **Progress entry not written.** `CLAUDE.md` asks for one per task; this brief's commit list did not include it.
8. **The sag during the ramp-out is unchanged** (the feedforward still falls with the ramp, as in Part 1). David's decision 2 says the hand stays at its current pose; the code holds the arm targets, not the joint signals. Whether the hand should be held more firmly is not decided.

## Decisions David has to make

1. Whether a pre-generated chunk should be built (a larger change to the stand-in), or whether the step list is the chunk.
2. What a new chunk arriving during the hold should do (queue, replace, or be refused).
3. What a failed grasp during the hold should do (stop, retry, or wait).
4. What a second, different open during the hold should do (ignore it, or use its target).
5. Whether the feedforward should also be held during the ramp-out, so that the hand is held as well as the targets (today it sags 1.42 mm).
6. Whether to turn the switch ON for the stand-in by default, once the chunk question is settled.
