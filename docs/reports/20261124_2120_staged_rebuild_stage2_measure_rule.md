# Staged rebuild, stage 2: the measure rule (T = 2 s) as the production sag layer (2026-11-24, David d0bc)

Branch `safety-filter`, worktree `openarm-tracer-sim-sf-d`. HEAD before this stage: 5e6d7be. **Stage 2 STOPPED: criterion D failed** (policy D reaches the offset clip; policy A's hand error with correction on is higher than off). Per the brief, stages 3 and 4 were not started. Nothing was tuned. Nothing pushed. `docs/progress.md` left alone.

Commits: `6de6b4c` (WIP checkpoint: the rule switched, tests not yet updated), and the stage commit that carries this report (marked WIP: the stage did not pass). Data: `data/logs/rebuild_s2/` (git-ignored): queues `queue_a.sh`, `queue_b.sh`, `queue_c.sh`, `baseline/queue_base.sh`; regression logs in `regress/`, `regress2/`, `final/regress/`; policy in `policy/` (bridge off, first pass) and `final/policy/`; teleop per tick in `teleop/on|off/run.npz`; five poses in `static/perturbed.md`; held runs in `held/` and `final/held_*`; baselines in `baseline/`; tables `analysis/stage2_tables.md`, `baseline/compare.md`.

## Plain-language answer

1. **Built.** The measure rule (the test copy of stage 2) is now the production sag layer with T = 2 s (David's choice). The integrator and its 5 s fade are gone from production; the test copy's module `src/sag_measure.py` and the switch `SAG_RULE_CFG` are removed. The held-mass freeze, the skip rules and the reset on an open-catch release are kept.
2. **Passed.** The four staging tests reach home. The held run (seed 4294874296, `--liquid 0.100`) finishes SUCCESS on nominal physics and with the deliberate model error. In teleop, with correction on and off, the offset never reaches the clip and the teleop is logged on every tick through its tracking loss. The five static holds with the model error stay inside the clip (peak 6.6 mm). The bridge and source-view tests pass, and so do the sag feedback test and the command-bridge test with the sag off. The 18 safety-filter tests exit 0.
3. **Failed: criterion D (policy A–D, bridge off, correction on and off).** (a) The offset reaches the clip in policy D: the 40 mm position clip in seed 1 (76 ticks) and the 100 mrad rotation clip in all three seeds (256, 365 and 71 ticks). Criterion D says it must never reach the clip. (b) In policy A (the static one) the hand error with correction on is higher than with it off: 1.96 mm against 1.07 mm over 30 s, and 1.29 mm against 1.00 mm over the last 2 s. Criterion D says it must be lower. Policy B and C stay inside the clip, and their hand error is about the same with correction on and off. Policy D's hand error is about 230 mm with or without correction.
4. **Surprising.** The new layer is slower than the solver-free layer it replaces: the held run's sag call averages 1.47 ms (p99 3.65 ms), against 0.22 ms (p99 0.66 ms) in stage 2c. The rule computes the gravity torques on every tick. Also, the normal-run output did not change when the non-finite guard was added: the policy A run before and after the change is identical to the last digit.

## Pass criteria

| criterion | result |
|---|---|
| A. Regression: sag tests, sag feedback, command bridge (sag off), bridge clamp, source view, full safety-filter regression | **Explained failures only.** `sag_layer_test` 13 of 13 PASS (final). `sag_feedback_test` 8 of 8. `command_bridge_test` 19 of 19 (sag off). `bridge_clamp_test` 8 of 8. `source_view_test` 3 of 3. `sag_solver_free_test` 10 of 12: two explained failures (below). Safety filter: 18 of 18 exit 0 (run before the last one-line guard change; see the note on the final code). |
| B. The four staging tests reach home | **PASS.** `home_pose_test`, `grasp_lift_test`, `beaker_hotplate_correction_test`, `nav_follow_sim_test --only on`: all exit 0. (Run before the last one-line guard change.) |
| C. Held run SUCCESS, nominal and with the model error | **PASS.** Nominal `SUCCESS; plan failure None; status ['done'×4]`. Perturbed `SUCCESS; plan failure None; status ['done'×4]`. Both on the final code. |
| D. Policy A–D, 3 seeds, bridge off, correction on and off: offset never reaches the clip; in A, hand error on lower than off | **FAIL.** See the policy table: D reaches the clip; A's hand error is higher with correction on. |
| E. Teleop, correction on and off, logged through the tracking loss: offset never reaches the clip | **PASS** (offset peak 29.3 mm on, 0 clip ticks; 0 off). Hand error after pauses reported below. |
| F. Model error, five poses: hand error 1 s after each stop, offset against droop at the stop | **Reported** (clip 0; below). Home hold reported separately. |
| Timing (sag and bridge) | Reported below (mean and 99th percentile). |
| Baselines against 1f29da7 and 5e6d7be | Reported below. Report only. |

Stage 3 and stage 4 were not started (the brief: stop when a stage fails a pass criterion).

## Technical block

### 1. Build

- `src/sag_layer.py`: `SagLayer` is the measure rule. Per arm, per tick: predicted joints `pred` (a first-order motor filter, tau_j = (kv_j + d_j)/kp_j from the model, driven by the received control `ctrl − tau_ff/kp`); measured sag `sag_m = pred − q_meas`; estimate `est += (dt/T)(sag_m − est)` with T = `SAG_LAYER_CFG["measure_T_s"]` = 2.0 (SF-127); `dq = est` scaled uniformly to the 40 mm / 100 mrad hand clip (`PICK_PLACE_CFG` sag_max_m, sag_max_rad). Skips (first tick, bridge suspension, held-mass ramp (M238, `held_ramp`), non-finite). A non-finite command, or a non-finite hand move (added after the queue B runs; see §7), sends the source's targets unchanged.
- Removed from production: the integrator, `_sag_leak` and `sag_leak_tau_s` (5 s), `_sag_trim_step`, `src/sag_measure.py` (deleted), `SAG_RULE_CFG` and the factory. `PICK_PLACE_CFG sag_gain_per_s` is no longer read by the production layer; analysis scripts still read it.
- Reset (M263): kept. The release sets the learned correction to zero. **Mapping choice, to confirm:** the reset zeroes `est` (the learned state of this rule), where the integrator zeroed its offset.
- `src/control_loop.py`: `SagLayer(...)` for each arm (the factory line removed).
- `src/task_config.py`: `SAG_LAYER_CFG` gets `measure_T_s: 2.0`; `sag_leak_tau_s` and `SAG_RULE_CFG` removed.
- Analysis scripts updated to the production class and their switch flags removed: `experiments/climb_perturb_diag.py`, `experiments/climb_static.py`.

### 2. Tests: which changed, and why

| test | change | reason |
|---|---|---|
| `sag_layer_test.py` | S1 (equals the old in-script trim) removed; leak cases L1, L2, L3 removed; SF-118 (a) rewritten as "standing commands update the estimate"; the others read `layer.est` instead of the integrator's `C`; S5(d) now gives the second stream its own simulator state | S1 and L1–L3 tested the integrator and its leak; the paired layers shared one `MjData` (a harness error). 13 of 13 PASS. |
| `sag_solver_free_test.py` | the 5 s fade check removed; the `sag_leak_tau_s = 1e9` switches removed; the harness writes the actuator control as the loop does (bridge output plus gravity offset); the run records the source-vs-measured pad error (`perr`), the layer's own pad error (`lerr`) and the estimate's pad size (`estpad`); skip checks rewritten for the estimate (the two-reference requirement is gone with the integrator) | the harness wrote the layer's command where the loop writes the bridge output; the rule needs the actuator's control. 10 of 12 PASS, two explained failures (§3). |
| `command_bridge_test.py` | buffer edges for joints 3 and 4 derived from the configured buffer rule (SF-125) instead of the old fixed 10 degrees | the range buffer changed those edges; the old hard-coded values made 3 checks fail (19 of 19 PASS now). |
| `climb_perturb_diag.py`, `climb_static.py` | switch flags removed; the measure class used directly; the per-tick error recorded with 7 joints | the layer's error is the 7-joint measured sag. |
| new: `teleop_tick_log.py` | a per-tick log of the teleop (wraps `run_loop`, chains the hook) | the old recorder writes only when an arm command is sent, so the tracking loss left a 2.6 s gap in the log (criterion E). |
| new: `stage2_tables.py`, `stage2_baseline_compare.py` | analysis only | tables and baseline differences. |

### 3. The two explained failures in `sag_solver_free_test` (not tuned)

- **Step of 0.1 rad on joint 1 (7.2 mm against 0.5 mm).** The check reads the layer's own instantaneous error. A diagnostic on the same rig: the estimate's pad size during the step is at most **0.11 mm**, the value the layer applies; the instantaneous error reaches 5.5 mm in the diagnostic and 7.2 mm in the test run (two runs of the same rig, not reconciled). The rig's measured joints lag the bridge output by a velocity estimate over three ticks, and the bridge output changes every third tick, so that estimate is quantised. The rule's estimate averages over T = 2 s and does not follow it. The bound was not changed.
- **Held on the buffer clamp (0.104 mm against 0.1 mm).** Measured in a longer run: |C_p| 0.011 mm at 1 s, 0.110 mm at 3 s, 0.104 mm at 6 s, 0.023 mm at 9 s, 0.005 mm at 12 s, 0.001 mm at 15 s. It rises while the bridge ramps to the buffer edge, then falls with a time constant of about 2 s (= T). That is a bounded transient, not a windup. The check's 6 s window catches the plateau; the bound was not changed.

### 4. Criterion D, policy runs (bridge off; `data/logs/rebuild_s2/final/policy/` for A s1 on, the first pass in `policy/`)

Offset in mm, rotation in mrad, hand error (mean over 30 s, and over the last 2 s) in mm. Correction on, then off.

| variant | seed | offset peak (mm) | rot peak (mrad) | clip ticks (pos / rot) | hand error mean on / off | last 2 s on / off |
|---|---|---|---|---|---|---|
| A | 1–3 | 2.48 | 15.24 | 0 / 0 | 1.96 / 1.07 | 1.29 / 1.00 |
| B | 1 | 4.71 | 16.41 | 0 / 0 | 39.85 / 39.74 | 48.38 / 48.24 |
| B | 2 | 5.19 | 15.61 | 0 / 0 | 44.21 / 44.01 | 42.95 / 43.06 |
| B | 3 | 7.01 | 23.32 | 0 / 0 | 39.20 / 39.06 | 46.67 / 46.38 |
| C | 1 | 3.71 | 14.62 | 0 / 0 | 31.89 / 31.79 | 31.29 / 31.24 |
| C | 2 | 3.73 | 13.18 | 0 / 0 | 35.40 / 35.20 | 37.74 / 37.51 |
| C | 3 | 7.82 | 26.78 | 0 / 0 | 31.58 / 31.42 | 30.80 / 30.47 |
| D | 1 | **40.00** | **100.00** | **76** / 256 | 241.21 / 239.69 | 227.04 / 225.39 |
| D | 2 | 38.81 | 100.00 | 0 / **365** | 234.67 / 233.85 | 249.27 / 249.08 |
| D | 3 | 39.79 | 100.00 | 0 / **71** | 230.11 / 228.91 | 184.40 / 191.21 |

A's offset is 2.5 mm, against a hand error that is higher with correction on: the offset moves the hand away from where the source puts it in this static case. I have not traced why (the rule does not use the source; a candidate is the learned offset following the stream's own sag, see question 2). The hand error during motion (B, C, D) is mostly lag, as the brief says, and is not judged.

### 5. Teleop (criterion E, bridge on, every tick logged; `teleop/on|off/run.npz`)

- Correction on: 1176 ticks, 1.40 s to 9.23 s. Offset peak |C_p| 29.3 mm, peak |C_r| 65.4 mrad, **0 ticks at either clip**. Correction off: offset 0.
- Pauses (REF unchanged for at least 1 s): two per run. The 1.41 s pause has no hand error: the logged error at 2.41 s is NaN (cause not traced). The 5.41 s pause (the fake operator loses tracking): |C_p| at the stop 24.4 mm; hand error 1 s after the stop **2.79 mm** with correction on, **2.66 mm** off.
- The per-command recorder in `climb_perturb_diag.py` wrote nothing on the tracking-loss ticks; `teleop_tick_log.py` logs every tick.

### 6. Five static holds, deliberate model error, correction on (`static/perturbed.md`; home separately)

| pose | offset vs droop at the stop (mm) | hand error 1 s after the stop (mm) | \|C_p\| at the end of the hold (mm) |
|---|---|---|---|
| home (start transient, separate) | 0.33 | 2.42 | 1.32 |
| pick | 0.11 | 1.99 | 5.40 |
| policy B, 7.4 s | 0.19 | 3.45 | 3.72 |
| policy B, 13.4 s | 0.18 | 2.55 | 4.21 |
| policy B, 21.4 s | 0.21 | 1.92 | 0.73 |

Peak |C_p| over the five holds 6.60 mm; ticks at the 40 mm clip 0. The brief's rule to exclude the home hold from the at-rest scores is applied: the mean over the four moving poses is 2.5 mm; home is reported separately.

### 7. Timing (`*_timing.json` from `sag_timing_probe.py`; per call, ms)

| run | sag layer mean | sag layer p99 | bridge mean | bridge p99 |
|---|---|---|---|---|
| held, nominal (final code) | 1.469 | 3.649 | — | 0.420 |
| held, nominal (queue A, pre-guard) | 1.197 | 2.830 | 0.045 | 0.344 |
| policy A–D, correction on (bridge off) | 0.37–0.42 | 1.02–1.22 | 0.001 | 0.003–0.004 |
| policy A–D, correction off | 0.002–0.003 | 0.007–0.010 | 0.001 | 0.003–0.004 |

The layer's cost is dominated by the gravity torques it evaluates on every tick (`gravity_ff.torques`). Report only.

### 8. Baselines (`baseline/compare.md`; report only)

Largest difference per joint column (degrees), largest hand-position difference (mm, pad forward kinematics in the robot base frame, from the joint columns).

| scenario | stage 2 vs 5e6d7be (joint / hand) | stage 2 vs 1f29da7 (joint / hand) | 5e6d7be vs 1f29da7 (joint / hand) |
|---|---|---|---|
| manual | 0.003° (right joint 1, 9.10 s) / 0.041 mm | 0.140° (right joint 4, 8.70 s) / 1.157 mm | 0.140° / 1.159 mm |
| teleop | 0.139° (left joint 4, 5.50 s) / 0.750 mm | 0.267° (left joint 5, 9.20 s) / 5.370 mm | 0.319° / 5.352 mm |
| policy | 0.001° (left joint 3, 6.90 s) / 0.015 mm | 0.138° (left joint 4, 6.60 s) / 1.121 mm | 0.138° / 1.136 mm |

### 9. Also run / not run

- Run: the sag tests and bridge tests (final code: `final/regress/`); the 18 safety-filter tests (queue A, before the one-line guard); the four staging tests (queue A, before the guard); held nominal and perturbed (final code); policy A–D × 3 seeds × correction on/off (bridge off); teleop correction on and off (per tick); five static holds (perturbed); baselines for manual, teleop and policy at 1f29da7, 5e6d7be and stage 2.
- Not run: the full hour-long `run_tests.sh` (only the sag, bridge and safety-filter groups); the safety-filter and staging groups were not re-run on the final code after the one-line guard change (the policy A run before and after the change is identical, so the guard does not affect normal output; the change only matters for a non-finite hand move).
- Worktrees `openarm-tracer-sim-bl-1f29da7` and `openarm-tracer-sim-bl-5e6d7be` were created for the baselines (their `deps` and `models/robot/assets` link to this worktree). They are still attached to the repo; removing them needs a go-ahead.

### 10. Decisions and known limits

- SF-127 (T = 2 s, the measure rule as production; the integrator and its fade removed; reset maps to the estimate): the rule is built; stage 2 did not pass criterion D, so the decision stands but its acceptance is open. See the questions.
- SF-128 (the wrist friction band as a known limit of the sag correction on joints 5–7: frictionloss/kp = 0.1 Nm / 10 Nm/rad = 10 mrad): recorded; no separate test.

## Questions for David (not decided; nothing changed for these)

1. **Criterion D.** Policy D reaches the offset clip in every seed (position in seed 1, rotation in all three), and policy A's hand error is higher with correction on. Do you want the rule changed, the criterion changed, or the stage accepted as it stands? I have not tuned anything.
2. **Policy A.** Why does a static stream end with a higher hand error when the layer corrects? I have not traced it. It may be the learned offset following the stream's own sag (the test copy's known behaviour), but that is not checked.
3. **Reset mapping (M263).** Is zeroing the estimate the right reset for this rule? I built it that way and flagged it here.
4. **The two explained `sag_solver_free_test` failures.** Accept the explanations (the instantaneous-error metric and the T = 2 s start-up transient), or change the checks? I have not changed the bounds.
5. **The layer's cost (1.47 ms mean on the held run).** Is that acceptable, or should the gravity torques be cached per tick?
6. **Teleop pause at 1.41 s.** It has no hand error because there is no bridge output yet at that time. Accept, or log from the start of the bridge?
