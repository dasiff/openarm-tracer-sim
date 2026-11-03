# Lift-off run 1: remaining analysis, breakaway curve, preload check, and memory note

Branch: held-weight-ramp-sag-off. Date: 2026-11-03.
Plain-language key: the "pad" is the fingertip contact point. The "trim" is the sag correction (a small offset the arm adds to its target). "Ramp" is the held-mass ramp (the weight the controller compensates, raised or lowered in steps). "Table" is the bench under the beaker. Height is z in millimetres (mm) unless stated. Force is in newtons (N); 1 gram of mass is 9.81 mN.

Claim labels: VERIFIED = read directly from a saved file or the code, with the source named. INFERRED = my reasoning from verified facts. NOT FOUND = the data or code needed is not in the saved files.

## STEP 0: state at the start

- Process check: no MuJoCo or Python simulation process was running, and no tmux session was running (VERIFIED, pgrep at 02:33).
- git status: the lift-off logging in src/mpvi_modules.py and src/gravity_ff.py was committed in 19ce0cc. The lift-off report is in dcf87d0 and the sweep progress entry in 1511bad (VERIFIED, git log).
- The flock lock: the repository has no existing flock lock file (grep for flock in scripts, the docs and CLAUDE.md found nothing). For Part C I used the flock command on data/locks/mujoco.lock. This is a choice I made; see "Needs decision".

## Part A

### A1 and A2: steps, time, place, beaker final position, trim

Already in the committed report docs/reports/20261103_0125_liftoff_run1_analysis.md, section 1 (steps, pad error, time taken, place time, beaker final position) and section 3 (trim at ramp start, at detection, after UP, and the change). Nothing new was found, so those numbers are not repeated here. VERIFIED from that report.

### A3: metric 1 and metric 2

Metric 1 (baseline tick 11180, pad height 1012.910 mm; window the 200 ticks after the held mass first becomes nonzero, ticks 11181 to 11381):
- Rise +0.077 mm, drop 0.000 mm, swing 0.077 mm. VERIFIED.
- Plain words: in this window the pad hardly moves. The window ends about 470 ticks before lift-off (tick 11851), so this measure does not see the lift at all.
- Comparison with the saved runs: the sag-on run gives rise +17.70, drop -18.45, swing 36.14 mm, and the z-only run gives +2.98, -4.44, 7.42 mm (VERIFIED: docs/reports/20261102_1951_trim_zonly_runs.md, lines 46 and 49). These are not like-for-like with run 1. In those runs the held mass starts at the handoff, so the 200-tick window covers the carry (INFERRED). The run-1 window covers only the pre-lift stage. The comparison needs a decision (see Needs decision, item 1).

Metric 2 (e = pad z minus reference pad z; the "ramp" is the ramp to the weighed estimate, which starts at the detection tick 11852 and ends at 12002 (detection plus 1.0 s), the ramp-end convention of bounce_handoff_metrics.py):
- Value before the ramp: +0.522 mm at tick 10998, the last tick with a reference. The reference is blank while no move runs, so there is no value at the ramp start itself (VERIFIED).
- Maximum after the ramp starts: +4.990 mm at tick 12602 (VERIFIED).
- Minimum after the ramp ends: -4.399 mm at tick 12659 (VERIFIED).
- Value at the end of DOWN (tick 12862, 87.147 s): +0.583 mm (VERIFIED).
- Zero crossings after the ramp end, up to 3 s after DOWN: 228. The time between the first two: 0.333 s (ticks 12144 and 12194) (VERIFIED).
- Plain words: after the ramp, the pad height crosses the reference 228 times. That is a signal chattering around zero, not one clean bounce. Its size is about ±5 mm. Whether this is the arm's own oscillation or measurement noise is NOT FOUND in these files.

### A4: gripper gate at the place's open

- Open command: tick 12863, 87.153 s (VERIFIED, OPEN step start in the trace).
- Gate start: the code (src/mpvi_modules.py, the gripper step) only lets the open event pass when three things hold together: at least close_min_s (1.0 s) after the command; the finger within open_reached_margin_m (3 mm) of its open travel; and the finger stalled (speed below 0.002 m/s) for stall_dwell_s (0.3 s). So the earliest the check can pass is 1.0 s after the command, at 88.153 s (VERIFIED from the code; the code has no separate named "gate").
- Physical open (finger 1 within 3 mm of its final open travel of 43.93 mm): 88.200 s, a delay of 1.047 s from the command (VERIFIED, episode log; the log is about 10 rows per second, so this is good to about 0.1 s).
- Gripper-open event in the run log: 88.54 s, a delay of 1.39 s from the command (VERIFIED, run log line "gripper_opened"). It comes 0.3 s after the physical open, which is the stall dwell (INFERRED from the code).
- Finger 1 at the command: 37.52 mm (VERIFIED). The travel is per finger.
- Hand (pad) z highest point from the command to the start of BACK: 1083.886 mm at the command tick itself. So the hand does not rise while the gripper opens (VERIFIED).
- Maximum beaker movement from its position at the command, up to the open event plus 0.5 s: 1.512 mm (VERIFIED, episode log beaker columns).
- Trim at the open command: x -0.941 mm, y -0.573 mm, z -4.757 mm (VERIFIED, C_p columns).

### A5: excursion from the planned path (ramp-in start to 3 s after the ramp end)

- Run 1 (this trace has only the z reference, so x, y and rotation are NOT FOUND for run 1): largest absolute pad z minus reference z = +4.026 mm at tick 11947 (VERIFIED).
- The other saved runs have all four components (VERIFIED, from their metrics files, same window):
  - z-only run 1 (data/bounce_zonly_20261102/metrics_run1.md, line 48): x +10.948 mm, y -1.167 mm, z -4.438 mm, rotation +2.120 deg.
  - handoff part 2 (data/bounce_handoff_20261102/metrics_part2.md, line 44): x +16.511 mm, y -2.044 mm, z +3.984 mm, rotation +2.522 deg.

### A6: every use of wall-clock time in src/ and the launcher scripts

The full list (104 lines: file, line and the matching text) is in the appendix at the end of this report. What each use does, and whether it can change simulated results if the VM freezes:

| file | what it does | can it change simulated results if the VM freezes? |
|---|---|---|
| src/teleop_source.py lines 75, 121, 251 | the operator clock is time.monotonic (wall clock), or the sim clock when CC_FAKE_TELEOP_SIMCLOCK=1 | YES, in a run that has a live operator or a teleop session active. The scripted pick and place switch to auto at 1.4 s of sim time (VERIFIED, run1 episode log), so these runs are not affected after that. |
| src/llm_gemini.py lines 205, 250, 259, 275 | API timeout of 60 s, retry waits with time.sleep, and timing of each call | YES, in live mode only. The default mode is cache_only (VERIFIED, src/task_config.py line 492), which never calls the API, so it cannot change a result. A timeout in live mode can change the plan. |
| src/episode_runner.py (20 hits) | timeouts and the auto and operator time counters use sim time (ticks times the tick period); time.perf_counter only sets wall_seconds in the summary (line 278) | NO for the timeouts (they count ticks, VERIFIED lines 216 and 235). NO for wall_seconds (logging only). |
| src/mpvi_modules.py lines 362, 397, 407, 440 | the step timers use el = t minus the phase start, where t is simulation time (VERIFIED, line 324 and 362) | NO. |
| src/staging_source.py lines 466, 481 (build time, logging only) and 566, 879 (phase timeouts through elapsed(t)) | elapsed(t) takes the sim-time argument t | NO (INFERRED: the caller passes the sim time; the caller was not read in full). |
| src/glfw_viewer.py (8 hits) | frame pacing for the on-screen viewer: sleep until the next tick's schedule, and HUD and snapshot timers | NO for the simulated numbers. Pacing changes how fast the run goes on the wall clock, not what each tick computes (INFERRED from the code at lines 374 to 390). Not used by the record-only launcher. |
| src/full_check.py (15), src/reach_map.py (7), src/bench_stance.py (8), src/stance_select.py (2), src/nav_follower.py (2), src/replay.py (2), experiments/demo_video.py (2) | timing statistics ("seconds" fields in reports and logs) | NO. They record durations; I did not find any branch that changes a decision on them (NOT FOUND for a full check of every line). |
| src/run_record.py line 220 | runs git with a 10 second timeout to record the code version in the run's metadata | NO for the sim, but a freeze during that call could raise a timeout error at start-up (NOT FOUND whether this is caught). |
| src/task_config.py (10 hits) and the docstrings | the timeout values, in seconds of sim time (for example arms_timeout, timeout_after_handoff) | NO. These are sim-time limits. |

## Part B: breakaway curve (0.5 N/s, run 1; the per-tick data are present, so nothing is missing)

Run 1 is the 0.5 N/s, 0.3 mm run. The sweep run r0.5_t0.3 has the same lift-off tick (11851) and is the same run. The other 0.5 N/s runs (threshold 0.1 and 1.0 mm) have their own per-tick files and could be tabled the same way; I did not table them.

### B1: table (every 5 ticks, ramp start 11181 to 40 ticks after detection 11851, i.e. tick 11891)

The full table is in the appendix (Part B table). Columns: tick, time, fictitious mass (the mass the feedforward carries, kg), pad z minus baseline (mm), beaker z minus its start (mm), table contacts (VERIFIED from the trace).

Plain words: the beaker's height changes by about 0.05 mm while the mass rises to 0.05 kg. The pad rises with the mass by about 1 mm per kg. The table is still in contact with the beaker (contact count 4 at the start, 1 to 3 from tick 11666, 0 from tick 11783 to 11891 apart from flicker).

### B2: slopes and the crossing

- The beaker first rises more than 0.05 mm above its ramp-start height at tick 11324 (76.893 s), at a fictitious mass of 0.0489 kg (VERIFIED).
- Pad z against mass before that tick (ticks 11181 to 11323): slope 1.004 mm per kg (VERIFIED).
- After it (ticks 11324 to 11892): slope 1.905 mm per kg (VERIFIED).
- The two lines meet at a fictitious mass of 0.0573 kg (VERIFIED arithmetic on the two fits).
- Important: the 0.05 mm rise is compliance, not lift-off. The table still carries the beaker until a fictitious mass of 0.2049 kg (tick 11783, where the table normal force first falls below 0.01 N, VERIFIED from the Part C run below). So "the beaker leaves" by the 0.05 mm rule is not the breakaway. The meeting point of the two lines (0.0573 kg) is also not a breakaway point (INFERRED).

## Part C: preload check (one simulation, pick only, 0.5 N/s, 0.3 mm, defaults)

### Run and logging

- Command: experiments/bounce_liftoff_run.py data/preload_20261103/run --pick-only, with the default rate (0.5 N/s) and threshold (0.3 mm) from src/task_config.py (VERIFIED from the run log header). Run under flock on data/locks/mujoco.lock and run_capped (6 GB cap). Log: data/logs/preload_20261103_0245.log. Peak memory 4.26 GB. The run stopped at tick 12303, the start of the place. Outcome: PICK ONLY, UP FINISHED (VERIFIED).
- Logging added (uncommitted, as the brief asks): the table normal force on the beaker (from the contacts with the bench geoms), the vertical force of the two left fingers on the beaker, the beaker weight from the model mass (0.200 kg times g), the actuator force and the qfrc_bias of the seven left arm joints, and the net vertical force the arm applies at the pad (least squares of J transpose F = actuator force minus bias). The diff of the final version is saved at data/preload_20261103/logging_preload.diff. The logging only reads the simulation state.
- A first attempt (data/logs/preload_20261103_0300_aborted_logging_bug.log, data/preload_20261103/run_aborted_logging_bug) stopped at tick 0 because of a shape error in my new code (mj_jac needs a 3 by nv array). No physics data came from it. The fix is in the saved diff and the run above is the second attempt.
- Sign check (VERIFIED): before the pick, with the beaker resting on the table and no finger contact, the table normal force averages 1.9620 N over 10468 ticks, and the beaker weight is 1.9620 N. So the sign convention is right.

### The forces at the key ticks (VERIFIED)

| moment | tick | table normal (N) | table minus weight (N) | pad vertical on beaker (N) | pad contacts | net vertical at the pad, arm (N) | preload (g) |
|---|---|---|---|---|---|---|---|
| ramp start | 11181 | 1.9802 | +0.0182 | -0.0180 | 8 | +0.3627 | +1.86 |
| detection | 11851 | 0.0000 | -1.9620 | +1.9639 | 7 | +2.3740 | not a preload (table unloaded) |
| end of IN (empty hand) | 10998 | 1.9628 | +0.0008 | 0.0000 | 0 | +0.5032 | +0.08 |

- Table preload at the ramp start: +0.0182 N, which is +1.86 g. It presses down (positive).
- At detection the table carries nothing: its normal force is 0. The pads carry the whole beaker (1.96 N). So "table minus weight" at detection (-200 g) is not a preload value.
- The table unloads at a fictitious mass of 0.2049 kg (tick 11783, 79.953 s), which matches the contact count dropping to 0 (VERIFIED).

### The brief's question

Is the extra mass needed at breakaway (24.5 g) equal to the preload divided by 9.81, within 5 g?
- No. The preload at the ramp start is 1.86 g. The difference from 24.5 g is 22.6 g. The table preload is not the 24.5 g.
- The arm's own vertical residual is of the same order as 24.5 g. At the ramp start, with the mass still at zero, the arm's net vertical force at the pad is +0.363 N, which is 37 g of equivalent weight. At the end of IN with an empty hand it is +0.503 N, which is 51 g. The pads and the table are in balance at the ramp start, so this residual is not transmitted to the beaker (VERIFIED for the numbers; INFERRED for the cause).
- The empty-hand value is not steady. Over the last 100 ticks before the pick's CLOSE it ranges from +0.072 N to +0.673 N, so the arm is still moving at the end of IN (VERIFIED range; INFERRED that the motion causes it).
- At detection, the arm's net vertical force (+2.374 N) exceeds the feedforward (0.2280 kg times g = 2.237 N) by 0.137 N (VERIFIED arithmetic).
- Plain words: the 24.5 g is not the table's preload. The arm's own vertical imbalance, of 0.1 to 0.5 N, is the same size as 24.5 g (about 1.5 to 5 times the 0.14 N imbalance at detection, depending on which tick you take). That imbalance is a likely candidate for the extra compensation, but this run does not test it (INFERRED). The next test would be an empty-hand, settled pose.

The window table (every 10 ticks from the end of CLOSE to 40 ticks after detection) is in the appendix (Part C table).

## Part D: memory note

### The killed run (rate 0.25 N/s, threshold 0.3 mm; data/logs/liftoff_sweep_r0.25_t0.3_20261103_011900.log)

- Killed by the 6 GB memory cap at 01:21:18 (exit 137, VERIFIED).
- Last tick reached: NOT FOUND from the log. The recording folder never got its chunks subfolder, which the recorder creates only once the settle has finished (src/run_record.py line 275, created right after the settle at line 259). So the run died before the recorder started, during the scene load or settle (INFERRED, VERIFIED for the folder state).
- Last log line: "libEGL warning: failed to open /dev/dri/renderD128: Permission denied" (VERIFIED). The run printed no progress lines, so there is no tick counter in the log either (INFERRED: the progress lines are buffered and were lost when the process was killed).
- The same combination also failed earlier, at 21:21 on 2026-11-02, in the host freeze. Both times it died before the recorder started, and the other five combinations, with the same code, all completed. The rate only changes the held mass from the weighing onward, so it cannot cause a failure during setup (INFERRED). The cause is NOT FOUND.

### The completed runs (peak memory is from the time -v line of each log; ticks are the rows logged)

| run | peak memory (kB) | ticks logged |
|---|---|---|
| run 1 (full plan, 0.5 N/s, 0.3 mm) | 4,239,100 | 14,381 |
| sweep 0.5 N/s, 0.1 mm | 4,235,776 | 11,879 |
| sweep 0.5 N/s, 0.3 mm | 4,321,280 | 12,303 |
| sweep 0.5 N/s, 1.0 mm | 4,217,196 | 12,346 |
| sweep 1.0 N/s, 0.3 mm | 4,292,288 | 11,967 |
| sweep 2.0 N/s, 0.3 mm | 4,328,936 | 11,805 |
| preload run (pick only, 0.5 N/s, 0.3 mm) | 4,259,972 | 12,303 |

VERIFIED from the logs. Each completed run peaked at 4.2 to 4.3 GB, against the 6 GB cap.

### Does any array in the logger grow with run length?

- The recorder (src/run_record.py, RunRecorder): its per-tick lists (ctrl, basevel, hashes, gripforce, heldmass, checkpoint ticks and states) are cleared at every chunk flush, which happens every 2400 ticks (16 s) (VERIFIED, _flush at lines 346 to 360). Its memory is bounded by one chunk. On disk it grows by about 150 KB per simulated second (VERIFIED, run_record.py line 23).
- The recorder's label list (self.labels) grows by one entry per label change (small; VERIFIED, line 325).
- The lift-off launcher (experiments/bounce_liftoff_run.py): its rows list keeps one dictionary per tick until the end of the run, and it is written to CSV only at the end (VERIFIED). This grows with run length (about 12,000 to 14,400 rows here). With the preload columns added, each row is wider. How much of the 4.2 GB peak this is was NOT measured.
- The episode log is written to disk line by line (VERIFIED, src/episode_runner.py line 169), so it does not grow in memory.

## Needs decision (not built)

1. **Metric 1 comparison.** Run 1's 200-tick window ends before lift-off, so its numbers cannot be compared with the sag-on and z-only figures, which cover the carry. Options: (a) compare run 1 over a window from the lift-off to the end of UP; (b) keep the window and add a window that covers the lift. Which?
2. **Metric 2 ramp end for run 1.** Run 1's ramp is the rate ramp, which has no end; the ramp to the estimate ends 1.0 s after detection. I used that. Confirm.
3. **What counts as breakaway.** The 0.05 mm rise (0.0489 kg) and the table unloading (0.2049 kg) give different answers. Which one should Part B and the preload question use?
4. **flock.** No existing flock lock was found. I used flock(1) on data/locks/mujoco.lock. Is that the lock you meant, or is there another?
5. **Commit rule versus the brief.** The brief asks for the logging to stay uncommitted, but CLAUDE.md's crash rule says to commit before any run longer than a couple of minutes. I followed the brief. The saved diff is data/preload_20261103/logging_preload.diff. Commit it, or keep it uncommitted?
6. **Empty-hand measurement.** The arm is still moving at the end of IN, so the +0.503 N value is not a settled reading. Option: add a settled, empty-hand pose (for example, after the arm has been still for 0.5 s) and measure there.
7. **The 24.5 g cause.** The arm's vertical imbalance (0.1 to 0.5 N) may be the true source of the extra compensation. Testing that needs a feedforward change, which is a design decision (options: a constant vertical offset in the feedforward; or a measured offset per pose). Which, if any?
8. **Killed-run cause.** The combination 0.25 N/s, 0.3 mm failed twice before the recorder started. Options: (a) rerun it with the same settings to see if it is repeatable (I have not); (b) leave it, since the other five completed; (c) add memory logging to the setup stage. Which?
9. **Wall-clock uses.** Live LLM timeouts (60 s) and the teleop monotonic clock can change results in live runs. Options: (a) leave as is (the default LLM mode never calls the API); (b) use the sim clock for the teleop in every run. Which?

## Appendix A: wall-clock use list (src/ and the launcher scripts; from grep, 104 lines)

The list is copied from the grep output in the section below.

```
src/llm_gemini.py:205:            self._client = genai.Client(api_key=self._key, http_options=types.HttpOptions(timeout=int(self.cfg["timeout_s"] * 1000)))
src/llm_gemini.py:250:                self.sleep(wait)
src/llm_gemini.py:259:        t0 = time.time()
src/llm_gemini.py:275:                 "seconds": round(time.time() - t0, 3), "prompt": prompt, "reply_text": rec["reply_text"]}
src/episode_runner.py:3:It does what a policy must not do itself: reset the scene, check success, enforce the timeouts, save the log and the summary. Everything else
src/episode_runner.py:15:    "limits" (dict)                     "timeout_after_handoff" and "operator_timeout" (seconds), "log_hz" (hertz), "exclusive" (bool).
src/episode_runner.py:32:            true if the operator had control at any time). TIMEOUT: limits["timeout_after_handoff"] of sim time under AUTO control (paused
src/episode_runner.py:33:            while the operator has control), or limits["operator_timeout"] under OPERATOR control (OPERATOR_NO_RESPONSE instead of TIMEOUT if
src/episode_runner.py:41:The runner drives src/control_loop.run_loop through its cfg["on_tick"] hook (success, timeouts, contacts, the log). The summary: outcome,
src/episode_runner.py:70:    (src/control_loop.run_loop) with an on-tick hook that judges success, enforces the timeouts, counts contacts and writes the log, then builds
src/episode_runner.py:104:            point of the run), unless it has ended already. Defaults to None (the episode runs until success, a timeout or a failure).
src/episode_runner.py:111:            "timeout_kind" (str or None): "auto" or "operator" for the timeouts, else None.
src/episode_runner.py:135:    wall0 = time.perf_counter()
src/episode_runner.py:164:    R = {"outcome": None, "handoff_time": None, "success_time": None, "contacts": {}, "diag": None, "t_end": None, "timeout_kind": None,
src/episode_runner.py:179:        """Judges one tick: handoff, contacts, the log row, success, timeouts and failure; returns the status text for the viewer.
src/episode_runner.py:235:            elif cur == "auto" and auto_t >= lim["timeout_after_handoff"] - 1e-6:
src/episode_runner.py:236:                R["outcome"], R["t_end"], R["timeout_kind"] = "TIMEOUT", t, "auto"
src/episode_runner.py:237:                loop["stop"]("timeout")
src/episode_runner.py:238:            elif cur == "teleop" and op_t >= lim["operator_timeout"] - 1e-6:
src/episode_runner.py:240:                    R["outcome"], R["t_end"], R["timeout_kind"] = "OPERATOR_NO_RESPONSE", t, "operator"
src/episode_runner.py:243:                    R["outcome"], R["t_end"], R["timeout_kind"] = "TIMEOUT", t, "operator"
src/episode_runner.py:244:                    loop["stop"]("timeout")
src/episode_runner.py:270:        "outcome": R["outcome"], "timeout_kind": R["timeout_kind"], "mode_at_start": mode, "sim_time_end": R["t_end"], "handoff_time": R["handoff_time"],
src/episode_runner.py:278:        "wall_seconds": time.perf_counter() - wall0,
src/glfw_viewer.py:250:    snap_state = {"last": _time.time(), "p_down": False}
src/glfw_viewer.py:251:    hud_state = {"last": _time.time(), "ms": 0.0}
src/glfw_viewer.py:386:                _time.sleep(state["deadline"] - now)
src/glfw_viewer.py:476:                rec_t0[0] = _time.time()
src/glfw_viewer.py:521:            now = _time.time()
src/glfw_viewer.py:568:            _time.time() - snap_state["last"] >= snapshot_every
src/glfw_viewer.py:570:            snap_state["last"] = _time.time()
src/glfw_viewer.py:589:                data.time, _time.time() - rec_t0[0],
src/nav_follower.py:57:        t0 = time.time()
src/nav_follower.py:59:        S["plan_seconds"] += time.time() - t0
src/mpvi_modules.py:34:correction integrates the measured pad pose error (the forward kinematics of the measured joints) into the waypoint path (PICK_PLACE_CFG sag_*). A waypoint ends when the measured pad centre is within tolerance and the arm is settled; a timeout or a mink
src/mpvi_modules.py:397:            elif el > cfg["timeout_factor"] * st["T"] + cfg["timeout_extra_s"]:
src/mpvi_modules.py:407:            elif el > cfg["timeout_factor"] * st["T"] + cfg["timeout_extra_s"]:
src/mpvi_modules.py:440:            elif el > (cfg["close_timeout_s"] if closing else cfg["open_timeout_s"]):
src/sequential_policy.py:34:timeout clock of the episode starts when stage `handoff_after` is done (None: from the start); the status reports the stage that is, or is
src/sequential_policy.py:81:        handoff_after (int | None): Index of the stage whose completion is the "handoff" (the moment the episode's timeout clock starts).
src/replay.py:253:    wall0 = time.perf_counter()
src/replay.py:504:    report["wall_seconds"] = time.perf_counter() - wall0
src/bench_stance.py:267:    t0 = time.time()
src/bench_stance.py:269:    reg["seconds"], reg["cached"] = time.time() - t0, False
src/bench_stance.py:705:    t0 = time.time()
src/bench_stance.py:708:    sec["region"] = time.time() - t0
src/bench_stance.py:709:    t0 = time.time()
src/bench_stance.py:721:    sec["filter"] = time.time() - t0
src/bench_stance.py:722:    t0 = time.time()
src/bench_stance.py:768:    sec["overlay"] = time.time() - t0
src/task_beaker_hotplate.py:348:        episode_cfg (dict): Time limits and run settings: "timeout_after_handoff" and "operator_timeout" (seconds), "log_hz", "exclusive"
src/task_config.py:355:    "drive_timeout": 120.0,                # s [row M109 PENDING David: no decision recorded]
src/task_config.py:367:    "creep_in_timeout": 90.0,              # s (0.21 m at the creep speed of 0.02 m/s takes about 11 s plus the ramps) [row M109 PENDING David: no decision recorded]
src/task_config.py:374:    "pedestal_timeout": 30.0,  # s: the PEDESTAL phase fails after this long (operator time not counted) [row M109 PENDING David: no decision recorded]
src/task_config.py:382:    "arms_timeout": 30.0,  # s: the ARMS phase fails after this long (operator time not counted) [row M109 PENDING David: no decision recorded]
src/task_config.py:390:    "timeout_after_handoff": 60.0,         # s of sim time under AUTO control after HANDOFF with no success -> outcome TIMEOUT (paused while the operator has control) [row M110 PENDING David: no decision recorded]
src/task_config.py:391:    "operator_timeout": 300.0,             # s of sim time under operator (teleop) control with no success -> outcome TIMEOUT (also the budget of a teleop-start episode) [row M110 PENDING David: no decision recorded]
src/task_config.py:443:    "timeout_factor": 3.0, "timeout_extra_s": 6.0,    # a waypoint move times out after factor * its planned time + extra [row M112 PENDING David: no decision recorded]
src/task_config.py:444:    "close_min_s": 1.0, "close_timeout_s": 5.0, "open_timeout_s": 4.0,  # s: the gripper's closing is not judged before close_min_s; it fails after close_timeout_s (closing) or open_timeout_s (opening) without stopping [row M020] [row M112 PENDING David: no decision recorded]
src/task_config.py:451:    "episode_timeout_s": 180.0,  # s: the sim-time budget of a pipeline episode after the handoff (replaces the task's own timeout) [row M112 PENDING David: no decision recorded]
src/task_config.py:498:    "timeout_s": 60.0,                     # per request [row M114 PENDING David: no decision recorded]
experiments/demo_video.py:117:    t0 = time.perf_counter()
experiments/demo_video.py:121:    facts.update(outcome=s["outcome"], plan_failure=state["failure"], status=state["status"], beaker_on_hotplate=bool(ok_success), episode_wall_s=time.perf_counter() - t0)
src/reach_map.py:196:        t = time.time()
src/reach_map.py:198:        stats["check_seconds"] += time.time() - t
src/reach_map.py:212:            t = time.time()
src/reach_map.py:214:            stats["check_seconds"] += time.time() - t
src/reach_map.py:260:    wave, t0 = 0, time.time()
src/reach_map.py:289:            log(f"  wave {wave}: frontier {len(f_idx)}, reached so far {store.n}, ik calls {stats['ik_calls']}, collision checks {stats['checks']}, {time.time() - t0:.0f} s")
src/reach_map.py:292:    return {"state": state, "store": store, "stats": stats, "waves": wave, "seconds": time.time() - t0}
src/run_record.py:220:        run = lambda *a: subprocess.run(["git", *a], cwd=PROJECT_ROOT, capture_output=True, text=True, timeout=10).stdout.strip()
src/staging_source.py:38:                     and drive_yaw_tol with the base at rest for drive_settle_s; the tolerances are never loosened. After drive_max_reapproach corrective re-approaches (or on the timeout)
src/staging_source.py:46:Phase end = the target reached, with the moving joints below the settle velocity for the settle time. Every phase has a timeout in
src/staging_source.py:47:STAGING_CFG (arms_timeout, pedestal_timeout, drive_timeout, creep_in_timeout) that does not count the time an operator had
src/staging_source.py:85:COMPENSATE_PAUSES = True  # True: the staging's timeouts do not count the time an operator had control [row M127 PENDING David: no decision recorded]
src/staging_source.py:403:        cfg (dict): The staging configuration (STAGING_CFG): the speeds, ramps, tolerances, settle values and timeouts of the phases.
src/staging_source.py:466:        t0 = time.time()
src/staging_source.py:481:        nav["build_s"] = time.time() - t0
src/staging_source.py:843:    phases = {"ARMS": (arms, "arms_timeout"), "PEDESTAL": (pedestal, "pedestal_timeout"), "DRIVE": (drive, "drive_timeout"),
src/staging_source.py:844:              "CREEP_IN": (creep_in, "creep_in_timeout")}
src/staging_source.py:851:        step first if a restore after a hand-back is pending; on the first call opens the ARMS record; fails the stage if the phase timeout (cfg key per phase, operator time
src/staging_source.py:878:        fn, timeout_key = phases[phase]
src/staging_source.py:879:        if elapsed(t) > cfg[timeout_key]:
src/staging_source.py:880:            fail(t, f"{phase} timed out after {cfg[timeout_key]:.0f} s")
src/full_check.py:225:        t0 = time.perf_counter()
src/full_check.py:256:        self.load_seconds = time.perf_counter() - t0
src/full_check.py:395:            t0 = time.perf_counter()
src/full_check.py:404:            self.seconds["ik_solve"] = self.seconds.get("ik_solve", 0.0) + time.perf_counter() - t0
src/full_check.py:409:            t0 = time.perf_counter()
src/full_check.py:411:            self.seconds["ik_admissible"] = self.seconds.get("ik_admissible", 0.0) + time.perf_counter() - t0
src/full_check.py:458:                    t0 = time.perf_counter()
src/full_check.py:464:                    self.seconds["ik_solve"] = self.seconds.get("ik_solve", 0.0) + time.perf_counter() - t0
src/full_check.py:468:                    t0 = time.perf_counter()
src/full_check.py:470:                    self.seconds["ik_admissible"] = self.seconds.get("ik_admissible", 0.0) + time.perf_counter() - t0
src/full_check.py:556:        t_all = time.perf_counter()
src/full_check.py:563:            r.update(cached=True, seconds={"total": round(time.perf_counter() - t_all, 4)})
src/full_check.py:573:            self.seconds["total"] = time.perf_counter() - t_all
src/full_check.py:579:            t0 = time.perf_counter()
src/full_check.py:581:            self.seconds[name] = self.seconds.get(name, 0.0) + time.perf_counter() - t0
src/task_policy.py:21:a time before the next stage goes on, and the time an operator had control does not count toward a phase's timeout.
src/mpvi_pipeline.py:37:        episode_cfg (dict): Time limits; the auto timeout is raised to PICK_PLACE_CFG["episode_timeout_s"]. Defaults to EPISODE_CFG.
src/mpvi_pipeline.py:65:    task = make_task(None, task_cfg, staging_cfg, {**episode_cfg, "timeout_after_handoff": PICK_PLACE_CFG["episode_timeout_s"]}, success_cfg)
src/stance_select.py:450:                    t_c = time.perf_counter()
src/stance_select.py:452:                    rep["carry_check_seconds"] = rep.get("carry_check_seconds", 0.0) + time.perf_counter() - t_c
```

## Appendix B: breakaway table (copied from the analysis output of experiments/liftoff_breakaway_gripper.py)

### B1 Breakaway table (every 5 ticks from ramp start to 40 ticks after detection)
| tick | time (s) | fictitious mass (kg) | pad z minus baseline (mm) | beaker z minus its start (mm) | table contacts |
|---|---|---|---|---|---|
| 11181 | 75.940 | 0.0003 | +0.000 | +0.000 | 4 |
| 11186 | 75.973 | 0.0020 | +0.000 | +0.000 | 4 |
| 11191 | 76.007 | 0.0037 | +0.001 | +0.001 | 4 |
| 11196 | 76.040 | 0.0054 | +0.002 | +0.002 | 4 |
| 11201 | 76.073 | 0.0071 | +0.003 | +0.003 | 4 |
| 11206 | 76.107 | 0.0088 | +0.005 | +0.005 | 4 |
| 11211 | 76.140 | 0.0105 | +0.006 | +0.006 | 4 |
| 11216 | 76.173 | 0.0122 | +0.008 | +0.008 | 4 |
| 11221 | 76.207 | 0.0139 | +0.010 | +0.009 | 4 |
| 11226 | 76.240 | 0.0156 | +0.011 | +0.011 | 4 |
| 11231 | 76.273 | 0.0173 | +0.013 | +0.013 | 4 |
| 11236 | 76.307 | 0.0190 | +0.014 | +0.014 | 4 |
| 11241 | 76.340 | 0.0207 | +0.016 | +0.016 | 4 |
| 11246 | 76.373 | 0.0224 | +0.017 | +0.017 | 4 |
| 11251 | 76.407 | 0.0241 | +0.019 | +0.019 | 4 |
| 11256 | 76.440 | 0.0258 | +0.021 | +0.021 | 4 |
| 11261 | 76.473 | 0.0275 | +0.022 | +0.022 | 4 |
| 11266 | 76.507 | 0.0292 | +0.024 | +0.024 | 4 |
| 11271 | 76.540 | 0.0309 | +0.025 | +0.025 | 4 |
| 11276 | 76.573 | 0.0326 | +0.027 | +0.027 | 4 |
| 11281 | 76.607 | 0.0343 | +0.029 | +0.029 | 4 |
| 11286 | 76.640 | 0.0360 | +0.030 | +0.030 | 4 |
| 11291 | 76.673 | 0.0377 | +0.032 | +0.033 | 4 |
| 11296 | 76.707 | 0.0394 | +0.035 | +0.035 | 4 |
| 11301 | 76.740 | 0.0411 | +0.037 | +0.037 | 4 |
| 11306 | 76.773 | 0.0428 | +0.040 | +0.040 | 4 |
| 11311 | 76.807 | 0.0445 | +0.043 | +0.043 | 4 |
| 11316 | 76.840 | 0.0462 | +0.046 | +0.046 | 4 |
| 11321 | 76.873 | 0.0479 | +0.048 | +0.049 | 4 |
| 11326 | 76.907 | 0.0496 | +0.051 | +0.051 | 4 |
| 11331 | 76.940 | 0.0513 | +0.054 | +0.054 | 4 |
| 11336 | 76.973 | 0.0530 | +0.056 | +0.056 | 4 |
| 11341 | 77.007 | 0.0547 | +0.058 | +0.059 | 4 |
| 11346 | 77.040 | 0.0564 | +0.060 | +0.061 | 4 |
| 11351 | 77.073 | 0.0581 | +0.063 | +0.064 | 4 |
| 11356 | 77.107 | 0.0598 | +0.065 | +0.066 | 4 |
| 11361 | 77.140 | 0.0615 | +0.067 | +0.068 | 4 |
| 11366 | 77.173 | 0.0632 | +0.070 | +0.071 | 4 |
| 11371 | 77.207 | 0.0649 | +0.072 | +0.073 | 4 |
| 11376 | 77.240 | 0.0666 | +0.074 | +0.076 | 4 |
| 11381 | 77.273 | 0.0683 | +0.077 | +0.078 | 4 |
| 11386 | 77.307 | 0.0700 | +0.080 | +0.081 | 4 |
| 11391 | 77.340 | 0.0717 | +0.082 | +0.084 | 4 |
| 11396 | 77.373 | 0.0734 | +0.085 | +0.087 | 4 |
| 11401 | 77.407 | 0.0751 | +0.087 | +0.089 | 4 |
| 11406 | 77.440 | 0.0768 | +0.089 | +0.091 | 4 |
| 11411 | 77.473 | 0.0785 | +0.092 | +0.093 | 4 |
| 11416 | 77.507 | 0.0802 | +0.095 | +0.097 | 4 |
| 11421 | 77.540 | 0.0819 | +0.099 | +0.101 | 4 |
| 11426 | 77.573 | 0.0836 | +0.103 | +0.104 | 4 |
| 11431 | 77.607 | 0.0853 | +0.105 | +0.107 | 4 |
| 11436 | 77.640 | 0.0870 | +0.107 | +0.110 | 4 |
| 11441 | 77.673 | 0.0887 | +0.110 | +0.112 | 4 |
| 11446 | 77.707 | 0.0904 | +0.112 | +0.114 | 4 |
| 11451 | 77.740 | 0.0921 | +0.114 | +0.117 | 4 |
| 11456 | 77.773 | 0.0938 | +0.117 | +0.119 | 4 |
| 11461 | 77.807 | 0.0955 | +0.120 | +0.123 | 4 |
| 11466 | 77.840 | 0.0972 | +0.127 | +0.129 | 4 |
| 11471 | 77.873 | 0.0989 | +0.133 | +0.135 | 4 |
| 11476 | 77.907 | 0.1006 | +0.138 | +0.141 | 4 |
| 11481 | 77.940 | 0.1023 | +0.142 | +0.145 | 4 |
| 11486 | 77.973 | 0.1040 | +0.146 | +0.149 | 4 |
| 11491 | 78.007 | 0.1057 | +0.149 | +0.152 | 4 |
| 11496 | 78.040 | 0.1074 | +0.152 | +0.155 | 4 |
| 11501 | 78.073 | 0.1091 | +0.155 | +0.158 | 4 |
| 11506 | 78.107 | 0.1108 | +0.156 | +0.160 | 4 |
| 11511 | 78.140 | 0.1125 | +0.158 | +0.161 | 4 |
| 11516 | 78.173 | 0.1142 | +0.160 | +0.164 | 4 |
| 11521 | 78.207 | 0.1159 | +0.163 | +0.167 | 4 |
| 11526 | 78.240 | 0.1176 | +0.167 | +0.170 | 4 |
| 11531 | 78.273 | 0.1193 | +0.171 | +0.175 | 4 |
| 11536 | 78.307 | 0.1210 | +0.175 | +0.179 | 4 |
| 11541 | 78.340 | 0.1227 | +0.180 | +0.183 | 4 |
| 11546 | 78.373 | 0.1244 | +0.183 | +0.187 | 4 |
| 11551 | 78.407 | 0.1261 | +0.187 | +0.190 | 4 |
| 11556 | 78.440 | 0.1278 | +0.190 | +0.194 | 4 |
| 11561 | 78.473 | 0.1295 | +0.193 | +0.197 | 4 |
| 11566 | 78.507 | 0.1312 | +0.197 | +0.200 | 4 |
| 11571 | 78.540 | 0.1329 | +0.199 | +0.203 | 4 |
| 11576 | 78.573 | 0.1346 | +0.200 | +0.205 | 4 |
| 11581 | 78.607 | 0.1363 | +0.204 | +0.208 | 4 |
| 11586 | 78.640 | 0.1380 | +0.209 | +0.213 | 4 |
| 11591 | 78.673 | 0.1397 | +0.213 | +0.217 | 4 |
| 11596 | 78.707 | 0.1414 | +0.217 | +0.222 | 4 |
| 11601 | 78.740 | 0.1431 | +0.221 | +0.226 | 4 |
| 11606 | 78.773 | 0.1448 | +0.225 | +0.230 | 4 |
| 11611 | 78.807 | 0.1464 | +0.229 | +0.234 | 4 |
| 11616 | 78.840 | 0.1481 | +0.233 | +0.238 | 4 |
| 11621 | 78.873 | 0.1498 | +0.237 | +0.241 | 4 |
| 11626 | 78.907 | 0.1515 | +0.240 | +0.245 | 4 |
| 11631 | 78.940 | 0.1532 | +0.240 | +0.245 | 4 |
| 11636 | 78.973 | 0.1549 | +0.242 | +0.247 | 4 |
| 11641 | 79.007 | 0.1566 | +0.245 | +0.250 | 4 |
| 11646 | 79.040 | 0.1583 | +0.252 | +0.257 | 4 |
| 11651 | 79.073 | 0.1600 | +0.258 | +0.263 | 4 |
| 11656 | 79.107 | 0.1617 | +0.263 | +0.268 | 4 |
| 11661 | 79.140 | 0.1634 | +0.268 | +0.273 | 4 |
| 11666 | 79.173 | 0.1651 | +0.273 | +0.278 | 3 |
| 11671 | 79.207 | 0.1668 | +0.272 | +0.278 | 4 |
| 11676 | 79.240 | 0.1685 | +0.272 | +0.277 | 3 |
| 11681 | 79.273 | 0.1702 | +0.272 | +0.277 | 3 |
| 11686 | 79.307 | 0.1719 | +0.272 | +0.277 | 3 |
| 11691 | 79.340 | 0.1736 | +0.272 | +0.278 | 3 |
| 11696 | 79.373 | 0.1753 | +0.274 | +0.279 | 3 |
| 11701 | 79.407 | 0.1770 | +0.275 | +0.280 | 3 |
| 11706 | 79.440 | 0.1787 | +0.275 | +0.280 | 3 |
| 11711 | 79.473 | 0.1804 | +0.274 | +0.280 | 2 |
| 11716 | 79.507 | 0.1821 | +0.274 | +0.279 | 2 |
| 11721 | 79.540 | 0.1838 | +0.274 | +0.279 | 2 |
| 11726 | 79.573 | 0.1855 | +0.275 | +0.280 | 2 |
| 11731 | 79.607 | 0.1872 | +0.279 | +0.284 | 2 |
| 11736 | 79.640 | 0.1889 | +0.283 | +0.288 | 2 |
| 11741 | 79.673 | 0.1906 | +0.287 | +0.293 | 1 |
| 11746 | 79.707 | 0.1923 | +0.286 | +0.291 | 2 |
| 11751 | 79.740 | 0.1940 | +0.286 | +0.291 | 2 |
| 11756 | 79.773 | 0.1957 | +0.286 | +0.291 | 2 |
| 11761 | 79.807 | 0.1974 | +0.286 | +0.290 | 2 |
| 11766 | 79.840 | 0.1991 | +0.286 | +0.290 | 1 |
| 11771 | 79.873 | 0.2008 | +0.286 | +0.289 | 1 |
| 11776 | 79.907 | 0.2025 | +0.286 | +0.289 | 1 |
| 11781 | 79.940 | 0.2042 | +0.287 | +0.290 | 1 |
| 11786 | 79.973 | 0.2059 | +0.289 | +0.291 | 1 |
| 11791 | 80.007 | 0.2076 | +0.290 | +0.290 | 1 |
| 11796 | 80.040 | 0.2093 | +0.290 | +0.290 | 0 |
| 11801 | 80.073 | 0.2110 | +0.292 | +0.290 | 0 |
| 11806 | 80.107 | 0.2127 | +0.291 | +0.290 | 0 |
| 11811 | 80.140 | 0.2144 | +0.292 | +0.290 | 0 |
| 11816 | 80.173 | 0.2161 | +0.292 | +0.290 | 0 |
| 11821 | 80.207 | 0.2178 | +0.291 | +0.290 | 1 |
| 11826 | 80.240 | 0.2195 | +0.291 | +0.290 | 0 |
| 11831 | 80.273 | 0.2212 | +0.291 | +0.290 | 0 |
| 11836 | 80.307 | 0.2229 | +0.291 | +0.290 | 0 |
| 11841 | 80.340 | 0.2246 | +0.293 | +0.291 | 0 |
| 11846 | 80.373 | 0.2263 | +0.300 | +0.296 | 0 |
| 11851 | 80.407 | 0.2280 | +0.316 | +0.311 | 0 |
| 11856 | 80.440 | 0.2280 | +0.342 | +0.337 | 0 |
| 11861 | 80.473 | 0.2280 | +0.372 | +0.367 | 0 |
| 11866 | 80.507 | 0.2280 | +0.399 | +0.394 | 0 |
| 11871 | 80.540 | 0.2279 | +0.426 | +0.421 | 0 |
| 11876 | 80.573 | 0.2279 | +0.475 | +0.469 | 0 |
| 11881 | 80.607 | 0.2278 | +0.574 | +0.565 | 0 |
| 11886 | 80.640 | 0.2277 | +0.729 | +0.718 | 0 |
| 11891 | 80.673 | 0.2276 | +0.957 | +0.942 | 0 |

## Appendix C: preload window (copied from experiments/liftoff_preload_analysis.py)

### C2 Window: every 10 ticks from the end of CLOSE to 40 ticks after detection
| tick | time (s) | table minus weight (N) | pad vertical (N) | pad contacts | net vertical at pad (N) | rate ramp mass (kg) |
|---|---|---|---|---|---|---|
| 11149 | 75.727 | +0.0434 | -0.0417 | 8 | +0.3627 | 0.0000 |
| 11159 | 75.793 | -0.0502 | +0.0487 | 8 | +0.3630 | 0.0000 |
| 11169 | 75.860 | -0.0490 | +0.0476 | 8 | +0.3629 | 0.0000 |
| 11179 | 75.927 | +0.0186 | -0.0179 | 8 | +0.3627 | 0.0000 |
| 11189 | 75.993 | +0.0047 | -0.0045 | 8 | +0.3894 | 0.0031 |
| 11199 | 76.060 | -0.0154 | +0.0150 | 8 | +0.4230 | 0.0065 |
| 11209 | 76.127 | -0.1082 | +0.1087 | 8 | +0.4566 | 0.0099 |
| 11219 | 76.193 | -0.1304 | +0.1310 | 8 | +0.4902 | 0.0133 |
| 11229 | 76.260 | -0.1619 | +0.1625 | 7 | +0.5239 | 0.0166 |
| 11239 | 76.327 | -0.1865 | +0.1864 | 7 | +0.5576 | 0.0200 |
| 11249 | 76.393 | -0.1072 | +0.1075 | 7 | +0.5913 | 0.0234 |
| 11259 | 76.460 | -0.2749 | +0.2744 | 8 | +0.6249 | 0.0268 |
| 11269 | 76.527 | -0.1802 | +0.1800 | 7 | +0.6588 | 0.0302 |
| 11279 | 76.593 | -0.2685 | +0.2688 | 7 | +0.6924 | 0.0336 |
| 11289 | 76.660 | -0.2920 | +0.2923 | 7 | +0.7251 | 0.0370 |
| 11299 | 76.727 | -0.3522 | +0.3517 | 7 | +0.7561 | 0.0404 |
| 11309 | 76.793 | -0.2857 | +0.2857 | 7 | +0.7856 | 0.0438 |
| 11319 | 76.860 | -0.4142 | +0.4137 | 8 | +0.8153 | 0.0472 |
| 11329 | 76.927 | -0.4098 | +0.4097 | 8 | +0.8453 | 0.0506 |
| 11339 | 76.993 | -0.4693 | +0.4684 | 8 | +0.8753 | 0.0540 |
| 11349 | 77.060 | -0.4663 | +0.4664 | 8 | +0.9047 | 0.0574 |
| 11359 | 77.127 | -0.5250 | +0.5245 | 8 | +0.9346 | 0.0608 |
| 11369 | 77.193 | -0.5191 | +0.5195 | 7 | +0.9642 | 0.0642 |
| 11379 | 77.260 | -0.5492 | +0.5494 | 8 | +0.9930 | 0.0676 |
| 11389 | 77.327 | -0.5636 | +0.5653 | 8 | +1.0219 | 0.0710 |
| 11399 | 77.393 | -0.5468 | +0.5453 | 8 | +1.0514 | 0.0744 |
| 11409 | 77.460 | -0.6403 | +0.6405 | 8 | +1.0810 | 0.0778 |
| 11419 | 77.527 | -0.6770 | +0.6783 | 8 | +1.1085 | 0.0812 |
| 11429 | 77.593 | -0.7090 | +0.7099 | 8 | +1.1380 | 0.0846 |
| 11439 | 77.660 | -0.7159 | +0.7168 | 8 | +1.1680 | 0.0880 |
| 11449 | 77.727 | -0.7449 | +0.7455 | 8 | +1.1972 | 0.0914 |
| 11459 | 77.793 | -0.8236 | +0.8182 | 8 | +1.2256 | 0.0948 |
| 11469 | 77.860 | -0.7888 | +0.7901 | 8 | +1.2513 | 0.0982 |
| 11479 | 77.927 | -0.8145 | +0.8102 | 8 | +1.2793 | 0.1016 |
| 11489 | 77.993 | -0.8559 | +0.8567 | 8 | +1.3083 | 0.1050 |
| 11499 | 78.060 | -0.8840 | +0.8849 | 8 | +1.3375 | 0.1084 |
| 11509 | 78.127 | -0.8606 | +0.8610 | 8 | +1.3680 | 0.1118 |
| 11519 | 78.193 | -0.8894 | +0.8900 | 8 | +1.3950 | 0.1152 |
| 11529 | 78.260 | -0.9195 | +0.9216 | 8 | +1.4226 | 0.1186 |
| 11539 | 78.327 | -0.9550 | +0.9557 | 8 | +1.4505 | 0.1220 |
| 11549 | 78.393 | -0.9869 | +0.9872 | 8 | +1.4791 | 0.1254 |
| 11559 | 78.460 | -1.0138 | +1.0152 | 8 | +1.5077 | 0.1288 |
| 11569 | 78.527 | -1.0274 | +1.0239 | 7 | +1.5365 | 0.1322 |
| 11579 | 78.593 | -1.0331 | +1.0349 | 7 | +1.5661 | 0.1356 |
| 11589 | 78.660 | -1.0745 | +1.0757 | 8 | +1.5936 | 0.1390 |
| 11599 | 78.727 | -1.1094 | +1.1105 | 7 | +1.6213 | 0.1424 |
| 11609 | 78.793 | -1.1397 | +1.1404 | 8 | +1.6493 | 0.1458 |
| 11619 | 78.860 | -1.1618 | +1.1625 | 7 | +1.6774 | 0.1492 |
| 11629 | 78.927 | -1.2180 | +1.2173 | 8 | +1.7084 | 0.1526 |
| 11639 | 78.993 | -1.2288 | +1.2283 | 7 | +1.7383 | 0.1560 |
| 11649 | 79.060 | -1.2450 | +1.2466 | 8 | +1.7664 | 0.1594 |
| 11659 | 79.127 | -1.3020 | +1.2976 | 7 | +1.7948 | 0.1628 |
| 11669 | 79.193 | -1.3886 | +1.3859 | 8 | +1.8263 | 0.1662 |
| 11679 | 79.260 | -1.3875 | +1.3853 | 7 | +1.8587 | 0.1696 |
| 11689 | 79.327 | -1.3995 | +1.3995 | 7 | +1.8911 | 0.1730 |
| 11699 | 79.393 | -1.4897 | +1.4849 | 8 | +1.9223 | 0.1764 |
| 11709 | 79.460 | -1.4378 | +1.4420 | 8 | +1.9541 | 0.1797 |
| 11719 | 79.527 | -1.5158 | +1.5167 | 8 | +1.9863 | 0.1831 |
| 11729 | 79.593 | -1.5066 | +1.5053 | 7 | +2.0152 | 0.1865 |
| 11739 | 79.660 | -1.5648 | +1.5656 | 8 | +2.0448 | 0.1899 |
| 11749 | 79.727 | -1.5675 | +1.5724 | 7 | +2.0766 | 0.1933 |
| 11759 | 79.793 | -1.7147 | +1.7119 | 7 | +2.1084 | 0.1967 |
| 11769 | 79.860 | -1.6028 | +1.6164 | 7 | +2.1404 | 0.2001 |
| 11779 | 79.927 | -1.7266 | +1.7264 | 7 | +2.1723 | 0.2035 |
| 11789 | 79.993 | -1.7254 | +1.7296 | 7 | +2.2020 | 0.2069 |
| 11799 | 80.060 | -1.9620 | +1.9589 | 7 | +2.2320 | 0.2103 |
| 11809 | 80.127 | -1.9620 | +1.9539 | 7 | +2.2613 | 0.2137 |
| 11819 | 80.193 | -1.9620 | +1.9541 | 7 | +2.2919 | 0.2171 |
| 11829 | 80.260 | -1.7764 | +1.7993 | 7 | +2.3258 | 0.2205 |
| 11839 | 80.327 | -1.9620 | +1.9620 | 7 | +2.3546 | 0.2239 |
| 11849 | 80.393 | -1.9620 | +1.9638 | 7 | +2.3725 | 0.2273 |
| 11859 | 80.460 | -1.9620 | +1.9618 | 7 | +2.3565 | 0.2280 |
| 11869 | 80.527 | -1.9620 | +1.9624 | 7 | +2.3783 | 0.2279 |
| 11879 | 80.593 | -1.9620 | +1.9721 | 7 | +2.4597 | 0.2278 |
| 11889 | 80.660 | -1.9620 | +1.9839 | 7 | +2.7215 | 0.2276 |
| 11891 | 80.673 | -1.9620 | +1.9941 | 7 | +2.7941 | 0.2276 |
