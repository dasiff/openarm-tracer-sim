# Lift-off weighing in the control loop (M258, version A), Nov 4, 2026

Commit: e50815d (the helper, tests, decision row M258). Progress notes: see docs/progress.md.

## Part 1: read-only findings (before any change)

### 1a. Timing in the 200 g run (replay of the logged signal trace, scratchpad run_200g_c1, left hand; no new simulation)

| event | tick | notes |
|---|---|---|
| CLOSE step starts (label "PICK: CLOSE") | 11000 | |
| holding verdict turns ON | 11014 | 14 ticks after CLOSE starts |
| still verdict first ON after holding | 11085 | hand speed below 0.005 m/s for 75 ticks |
| CLOSE step ends = WEIGH step starts | 11151 | the gripper's stall (0.3 s) and its end |
| WEIGH "still" state: joints settled (0.2 s) | 11151 | the WEIGH step's own settle |
| WEIGH "lift" state (rate ramp starts) | 11181 (file row) | 30 ticks after the settle |
| lift-off detected (held mass set) | 11851 (log) / 11852 (file row) | 671 ticks after the lift starts |
| UP step starts (label "PICK: UP") | 11853 | the script moves UP only after WEIGH ends |

Hand motion between the end of CLOSE (11151) and the start of the weighing: pad z changes by 0.0 mm (1.012909503 m at both ticks); the peak hand speed in that window is 0.0123 mm/s (tick 11151). The hand is essentially still.

Answer to the question: with the WEIGH step removed, the script's UP would start when CLOSE ends (tick 11151 in this run). The pause starts at the holding verdict (tick 11014), 137 ticks (0.91 s) earlier. So the pause begins before UP would start, and the source is not asked while paused, so UP cannot start before the pause. No NEEDS DECISION arises from the order, but see the note under 1c about the CLOSE step being paused while it is still running.

### 1b. Inputs of the weighing and what the control loop can sense

The weighing (src/mpvi_modules.py, WEIGH branch, before this change) takes these inputs:

| input | source | can the loop sense it today? |
|---|---|---|
| pad height per tick, for the baseline (mean of the 10 ticks before the rate ramp) | the script's own history list ZH, one value per step tick | yes: the pad centre from the joints (the same robot kinematics as the gravity feedforward) |
| joint speeds, for the settle (0.02 rad/s for 0.2 s) | data.qvel | yes |
| pad height now and the rise above the baseline | the pad centre | yes |
| held mass now (the ramp's own state) | src/held_mass_ramp | yes (the controller's own record) |
| hand height per kilogram (for the estimate) | src/gravity_ff.held_pad_z_per_kg: the robot model's geometry at the current pose | yes (the model; not a measurement of the beaker) |
| side, model, time, tick | script closure | yes |
| script step index, step history, step start time (t0), the weigh state dict | the script's step machinery | no longer needed: the pause starts from the holding verdict, its own state |
| the still verdict (hand speed < 0.005 m/s for 0.5 s) | new in the helper: StillDetector (shared with the signal trace) | yes |

Nothing needed by the weighing is unsensable. No NEEDS DECISION arises here.

### 1c. Timeout and failure today, and the rule the helper follows

- The WEIGH branch has NO timeout. The "still" and "settle" states wait as long as the joints are not still; the "lift" state waits until lift-off or the cap.
- Cap: when the mass reaches liftoff_max_mass_kg (1.0 kg) without lift-off, the held mass is set to 0 and the step advances (the branch's own rule, M237). This is not called a failure.
- The branch never calls fail(), so no failure event exists for WEIGH today.
- The helper follows the same rule: no timeout, the cap sets the held mass to 0 and resumes (logged "cap"). Holding OFF during the pause or the weighing is the new failed-weigh rule (David's decision 6).
- Note (not a decision, reported): CLOSE's timeout (close_timeout_s = 5.0 s) runs in the script's own time. The pause starts during CLOSE (1a), so CLOSE is frozen while paused. On resume the CLOSE stall test is checked before its timeout (step_inner checks done first), so a gripper that stopped during the pause completes without a timeout. The pause in this run is 5.6 s long, which is longer than close_timeout_s (5.0 s); if the gripper had not stopped before the pause, CLOSE would fail on resume. This is recorded, not changed.

### 1d. Every other reader of the WEIGH step or the weigh result

| reader | what it reads | effect of the removal |
|---|---|---|
| src/mpvi_modules.py: the plan (plan_pick) | the step list | the WEIGH entry removed |
| src/mpvi_modules.py: step start (TRACE and S["weigh"]) | the weigh state at each tick, for logging | removed (S["weigh"], TRACE key "weigh") |
| src/mpvi_modules.py: ZH buffer | pad heights for the baseline | removed (the helper keeps its own) |
| src/mpvi_modules.py: ctx["held_mass_kg"] | written by WEIGH (estimate) and by the handoff branch | the WEIGH write removed; the handoff write stays (see below) |
| experiments/bounce_liftoff_run.py | the TRACE "weigh" dict, writes weigh_state, weigh_base_m, weigh_rise_m, weigh_hold columns | now writes blank columns (the key is gone); the experiment is a one-off study and does not work for the new code |
| experiments/bounce_liftoff_metrics.py | the label "WEIGH" and the weigh columns | finds no WEIGH label; does not work for the new code |
| experiments/liftoff_breakaway_gripper.py | the weigh_state column | finds no "detected" state; does not work for the new code |
| experiments/held_mass_ramp_test.py | liftoff_check and liftoff_estimate | unchanged (the functions remain) |
| the handoff re-weigh (the 'held' step: reading_status, hand_load) | the pick's end (kind "held") | NOT part of the WEIGH step. No plan builds a 'held' step (grep), so this branch is not running today. Left as it was. |
| the signal trace | no weigh column | unchanged |
| the 200 g baseline files | liftoff_ticks.csv (weigh columns), episode_log.csv | unchanged (the baseline is not regenerated) |

## Part 2: build (commit e50815d)

Built as decided: src/weigh_helper.py (WeighHelper: the trigger on the holding ON edge for a grasp not yet weighed, the pause through the gate with reason "weigh", the still wait, the same weighing, the estimate set as the held mass, the resume, the failed-weigh cancel, the repeat rule, once per grasp); src/control_loop.py (one helper per hand, run every tick after the holding verdict and before the sources); src/open_catch.py (SourceGate: hold_source / release_source by reason, the source that last commanded a hand is the owner); src/holding.py (StillDetector, the hand-speed still verdict, used by the helper and by the signal trace); src/weigh_helper.py settle_check (the joint settle, used by the script's waypoints and by the helper).

Deletions (all in e50815d):
- src/mpvi_modules.py: the WEIGH entry of the pick plan; the whole `elif kind == "weigh"` branch of step_inner; the S["weigh"] initialisation at the step start; the TRACE key "weigh"; the ZH pad-height list (its initialisation and its two per-tick lines); the imports held_pad_z_per_kg, liftoff_check, liftoff_estimate and G (now unused); the body of settled() replaced by settle_check.
- src/signal_trace.py: the still count and previous-pad state and the inline speed and still lines (moved to StillDetector); the unused HOLD_STILL_CFG import.
- src/open_catch.py: the caught_by dictionary (replaced by held_src); paused_by values are now reason sets.

Config: no new numbers. The comment of liftoff_rate_n_per_s now names the helper (src/weigh_helper.py). Design plan row ll and decision row M258 updated.

## Part 3

### 3A Synthetic tests (experiments/weigh_helper_test.py: 8 of 8 pass; experiments/open_catch_test.py: 12 of 12 pass)

| case | check | result |
|---|---|---|
| a | holding ON, not weighed: source paused, its chunk discarded, not asked, no commands pass | PASS |
| b | the pause waits for the still verdict, the weighing runs, the held mass is set, the source is asked again (on the resume tick) | PASS |
| c | the same trigger during the pause is ignored and logged; the pause is not restarted | PASS |
| d | holding OFF during the pause: cancelled, commands pass at once, held mass zero, failed weigh logged | PASS |
| e | holding OFF during the weighing: cancelled, the rate ramp is stopped and ramped to zero, commands pass, failed weigh logged | PASS |
| f | a weighed grasp does not trigger again; a new grasp (off then on) is weighed once more | PASS |
| g | the lift phase reproduces the old code's result on the recorded lift-off of the registered run: detection tick 11851, lift start 11180, same as the old log; baseline, rise, mass and estimate within the file's rounding | PASS |
| h | all earlier open-catch tests (cases a to g, h, i, ii, iii, config) | PASS (12 of 12) |
| config | weighing values unchanged | PASS |

Case g: the old run's own log is one tick ahead of its file rows (the file's detected row is 11852; the log says 11851). Measured differences: baseline 2.4e-8 m, rise 2.4e-8 m, mass at detection 1.0e-8 kg, estimate 3.2e-7 kg (the log prints hand height per kg to 5 decimals).

### 3B Scratch run from e50815d (one run; log data/logs/held200_m258_20261104_215942.log, exit 0 at 22:03:13; output outside the repo, not registered)

Estimated mass and true mass (0.200 kg):

| run | raw mass at detection | estimate | over-read |
|---|---|---|---|
| registered baseline held_200g_20261104 | 0.227999 kg | 0.224489 kg | +24.5 g |
| M258 run | 0.2283 kg (log) | 0.2248 kg | +24.8 g |

Left-hand ticks (file rows for the baseline and the M258 run, as in the earlier reports; detection and resume from the helper's log):

| event | M258 run | baseline |
|---|---|---|
| CLOSE step starts | 11000 | 11000 |
| holding verdict ON | 11014 | 11014 |
| still verdict ON | 11085 | 11085 |
| weighing starts (M258: still verdict and settle begin; baseline: WEIGH label) | 11085 | 11150 |
| rate ramp starts (held mass first above zero) | 11117 | 11181 |
| weighing ends, held mass set | 11788 | 11852 (file row; log 11851) |
| pause ends (source asked again) | 11788 | (no pause) |
| CLOSE end / UP start | 11789 (UP label) | 11852 (UP label) |
| release start (place OPEN) | 12800 | 12863 |
| held mass reaches zero | 12950 | 13013 |
| gripper open command | 13009 | 13072 |
| BACK start | 13010 | 13073 |

Pause: from 11014 to 11788, 774 ticks = 5.16 s (still wait 71 ticks, settle 31, ramp 671).

Motion (beaker height, left hand):

| quantity | M258 run | baseline | difference |
|---|---|---|---|
| lift rise during UP (mm) | 106.530 | 106.560 | -0.030 |
| drop, release to BACK (mm) | 1.493 | 1.480 | +0.013 |
| place DOWN swing, pad z range (mm) | 38.664 | 38.696 | -0.032 |

Every differing tick, liftoff_ticks (M258 run against the baseline, left hand; first and last tick, count, largest difference):

| column | ticks | first | last | largest |
|---|---|---|---|---|
| label | 1219 | 11014 | 14081 | (text) |
| grip_cmd_m | 965 | 11014 | 14081 | (text) |
| ramp_start_kg | 2923 | 11014 | 14081 | 0.2283 kg |
| ramp_target_kg | 1976 | 11014 | 14081 | 0.2248 kg |
| target_kg | 1075 | 11788 | 12862 | 0.2248 kg |
| mass_kg | 1896 | 11117 | 13012 | 0.1570 kg |
| held_tau_L1 | 1896 | 11117 | 13012 | 0.739 N m |
| held_tau_L4 | 1896 | 11117 | 13012 | 0.606 N m |
| pad_x_m | 3201 | 11118 | 14318 | 0.0990 m |
| pad_y_m | 3201 | 11118 | 14318 | 0.0294 m |
| pad_z_m | 3201 | 11118 | 14318 | 0.0317 m |
| beaker_z_m | 3201 | 11118 | 14318 | 0.0318 m |
| arm_pad_Fz_N | 3201 | 11118 | 14318 | 1.630 N |
| act_L2 | 3201 | 11118 | 14318 | 1.398 |
| bias_L2 | 3201 | 11118 | 14318 | 1.397 |
| beaker_bench_contacts | 187 | 11602 | 11833 | 2.0 |
| weigh_state, weigh_base_m, weigh_rise_m, weigh_hold | 2692 to 2722 | 11151 | 14081 | (text; the M258 run has no WEIGH state) |

Every differing column against the earlier C1 run (signal trace, same scenario, script WEIGH): 55 columns differ; the first difference of the pad and joint columns is tick 11118 (the mass ramp, 1 tick after the ramp's first increment), and of the gripper force columns 12951 (the release). The step label differs from 11151 (the M258 run has no WEIGH label).

Note on the first difference (11118): the M258 run starts the held-mass ramp at tick 11116, while the gripper is still closing; the earlier run starts it at 11181, after CLOSE. The small mass (0.0003 kg at 11117) changes the arm's feedforward from then on, so the arm trajectories differ from 11118. Nothing in the run failed.

### 3C Not run: no other baseline registered or regenerated; experiments/baselines/ untouched.

## Not built, needs decision

- The source to pause when no command has yet come for the hand: default, no pause; the weighing still runs (log "no_source").
- CLOSE's 5 s timeout while paused: default, the pause does not stop the CLOSE timeout clock; if the gripper had not stopped by resume and the timeout passes, CLOSE fails. This run's pause (5.16 s) was longer than the timeout, and CLOSE completed because its stall test was already met.
- The weighing starts during CLOSE (in this run before the gripper's stall), because the trigger is the holding verdict (decision 2); default, built as decided.
- A failed weigh re-arms on the next holding ON (a new grasp); default, built.
- The repeat rule (decision 7) is only reachable through on_trigger; in the loop a repeat can only follow an OFF, which cancels the pause first. Default, built.
- The handoff re-weigh ('held' step) is dead code (no plan builds it); left as it was.
- Experiment tools that read the removed WEIGH step (experiments/bounce_liftoff_run.py, bounce_liftoff_metrics.py, liftoff_breakaway_gripper.py) no longer find it; default, left unchanged.
- A passive (no pause) variant is not built (not part of this task).
