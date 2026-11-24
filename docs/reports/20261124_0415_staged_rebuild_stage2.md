# Staged rebuild, stage 2: solver-free sag layer (2026-11-24, final for this stage)

Branch `safety-filter`. **Stage 2 STOPPED: it does not pass its regression criterion as built.** Stages 3 and 4 were not started. Nothing was tuned. Not pushed.

Commits: `1eb7dfc` (first stage 2 WIP), `8ada562` (this round: speed term, test harness, logging). Held-run data: `data/logs/stage2c_20261124/held/`. Policy and regression logs: `data/logs/stage2c_20261124/` (`regress/`, `staging/`, `policy/`). Baselines: `data/logs/stage2c_20261124/base/`.

## Verdict in one line

The solver-free layer runs and the held run and the four staging tests pass, but the learned offset becomes unstable when the arm does not follow its command (the speed term reads the bridge output, which contains the layer's own correction), and four `sag_layer_test` checks fail. Those four are explained below; three need a harness change David has not approved and one needs his decision on the speed term.

## David's six decisions: how each was applied

1. **Bridge test with sag off.** Applied. `command_bridge_test` runs with `SAG_LAYER_CFG["enabled"] = False`: 19 of 19 PASS, including cases 2, 4 and 5 at the original tolerance.
2. **Fade and damping bias accepted, no change.** Applied (no change made).
3. **Jacobian at the measured joints.** Applied (no change made).
4. **Speed term = change between successive bridge outputs / time between.** The code now does exactly this (`SagLayer.observe` keeps the last two bridge outputs; `_ref_speed` = (out1 − out2) / (t1 − t2)). The position reference stays REF(t−1). **Consequence found in this round: see "Finding 1".**
5. **Retirements and harness changes.** Done: `sag_solver_failure_test.py` and `sag_warm_start_reset_test.py` removed (`git rm`); S1 case of `sag_layer_test` retired (renamed `case_a_retired`, removed from results); the S2 case and the pause case now feed the bridge output to the layer (helper `_cc`). Not done: the same feed for `case_c_periodic`, `case_s5_a`, `case_s5_d` (not approved; see questions).
6. **Bridge off: reference = source target.** Applied (the command is the output; REF = command − dq = source).

**`sag_feedforward_check`** (no PASS/FAIL lines): it is a read-only analysis script that prints items 1–5 from `data/diag` and `data/closed_loop`; it does not import `sag_layer`, so the solver removal does not affect it. Exit 0. Not broken; no PASS/FAIL by design.

## Changes in this round (listed)

| File | Change | Why |
|---|---|---|
| `src/sag_layer.py` | speed term from raw bridge outputs (`outs`, `_ref_speed`) | decision 4 |
| `src/control_loop.py` | `BRIDGE_CLAMP["updates"]` (per joint, per bridge update); `pending_clamp` | clamp counts as updates and seconds |
| `experiments/command_bridge_test.py` | `main()` sets the sag layer off | decision 1 |
| `experiments/sag_layer_test.py` | `case_a` → `case_a_retired`; helper `_cc` (observe fed) on the S2 and pause cases | decision 5 |
| `experiments/sag_solver_failure_test.py`, `sag_warm_start_reset_test.py` | removed | decision 5 |
| `experiments/sag_timing_probe.py` | adds update counts, seconds clamped per joint, offset peak and clip ticks per side | logging |
| `experiments/count_run.py` | adds update counts and seconds clamped per joint | logging |

## Pass criteria

| Criterion | Result |
|---|---|
| `command_bridge_test` cases 2, 4, 5 (sag layer off, decision 1) | PASS, 19 of 19 |
| `bridge_clamp_test` | PASS, 7 of 7 |
| `sag_feedback_test` | PASS, 8 of 8 |
| `sag_solver_free_test` | PASS, 12 of 12 |
| `sag_layer_test` | **FAIL, 12 PASS / 4 FAIL** (see "Failing sag checks") |
| `sag_feedforward_check` | exit 0 (prints no PASS lines by design) |
| 18 safety-filter entries from `run_tests.sh` | all exit 0 |
| Regression: no unexplained failures | **Not met.** The four failures are explained, but three need David's approval for a harness change, and the fourth points at the speed-term design (Finding 1). Scope: the sag, bridge and safety-filter tests above, not the full hour-long `run_tests.sh` suite. |
| Four staging tests reach home | **PASS.** `home_pose_test`, `grasp_lift_test`, `beaker_hotplate_correction_test`, `nav_follow_sim_test --only on`: all exit 0. |
| Held run (full, seed 4294874296, `--liquid 0.100`) SUCCESS | **PASS.** `outcome SUCCESS; plan failure None; status ['done'×4]`. |
| Policy variants A–D, 3 seeds, correction on and off (24 runs, bridge on) | All 24 exit 0. Numbers below. The offset does not respond to the stream: see Finding 2. |
| Timing: sag + bridge p99 sum ≤ 6.7 ms | **PASS.** Held run: sag 0.219 mean / 0.664 p99; bridge 0.041 / 0.323 (p99 sum 0.99 ms). Policy B seed 1 (bridge on): sag 0.203 / 0.643; bridge 0.034 / 0.223 (sum 0.87 ms). |

## Failing sag checks (`sag_layer_test`, 4 FAIL)

| Check | Failure | Cause |
|---|---|---|
| (S3) pause: first command after a 5 s pause integrates one nominal period, offset held during the pause | ΔC 47.9 mm in one tick (bound 10 mm) | The offset had already diverged over the 30 ticks before the pause (Finding 1). The step itself is one nominal period, as checked by the spy. |
| (c) no periodic reset, 10 s with gripper open | 0 trim steps of 1500 | Harness: the bridge output is not fed (`case_c_periodic` calls `correct` only); every tick is skipped as `no_reference`. Not approved. |
| SF-118 (a) standing command does not freeze the trim | 0 trim steps of 60 | Same harness gap (`case_s5_a` uses `correct_command` with no `observe`). Not approved. |
| S5 (d) no source gate: tag does not change the correction | corrected False, |C| 0 | Same harness gap (`case_s5_d`). Not approved. |

The other 12 checks pass, including the leak checks L1–L3 (time constant 5.000 s fitted) and the held-mass freeze (S2) with the approved feed.

## Finding 1 (new): the speed term feeds the layer's own correction back

The speed term uses successive bridge outputs. Those outputs equal the source targets plus `dq`, the layer's own correction. So the correction feeds back: `dq` → output → speed → predicted lag → error → offset → `dq`.

Diagnostic (scratch scripts, not in the repo; the arm is held still, as in the test harness):

| Variant | max |error| at tick 40 | at tick 150 | end |
|---|---|---|---|
| Reference and speed from command, observed every tick (bridge-off style) | 4.58 m | 4.67 m | 4.70 m |
| Reference and speed from command, observed every 3rd tick (bridge rate) | 0.16 m | 1.54 m | 1.53 m |
| Reference from command, **speed from source targets** (diagnostic only) | 0.020 m | 0.001 m | 0.023 m |

So the speed term is the path that diverges. In the real loop (arm follows its command) the held run stays small (peak offset 5.8 mm, no clip tick), and the policy runs show no sign of divergence in the offset. The divergence needs an arm that does not follow.

**This is a consequence of decision 4 as written.** It is not changed in this round; it needs David's decision (questions 1 and 2 below).

## Finding 2 (new, unexplained): the offset is the same in every policy run

Across variants A–D and seeds 1–3, the correction-on runs end with the identical offset (left: p mm [0.198, 0.878, 0.207], r mrad [3.045, 1.706, 12.887]) and the identical peak (6.03 mm). The offset does change over time (it leaves zero after the setup move and decays), but the trajectories are identical for A and B, even though the hand error differs (B moves, A is static). A correction that tracks the arm should respond to the stream. The cause is not found; candidate causes to check: the layer's error does not use the motion in the policy path, or the policy path's command does not reach the layer's reference. This must be explained before the policy numbers count as evidence for the layer. (Not investigated further in this round.)

## Policy runs: 24 runs, bridge on

Offset peak = left arm, from the probe. Clip ticks = ticks at `sag_max_m` or `sag_max_rad` (none). Hand error = `hand_error_m.mean` over the 30 s window (see Question 3). Rejections = "target step too large" events.

| Run | Offset peak (mm) | Clip ticks | Hand error on / off (m) | Rejections on / off |
|---|---|---|---|---|
| A s1 | 6.03 | 0 | 0.4147 / 0.4207 | 1 / 1 |
| A s2 | 6.03 | 0 | 0.4147 / 0.4207 | 1 / 1 |
| A s3 | 6.03 | 0 | 0.4147 / 0.4207 | 1 / 1 |
| B s1 | 6.03 | 0 | 0.7485 / 0.7536 | 18 / 18 |
| B s2 | 6.03 | 0 | 0.7293 / 0.7348 | 17 / 16 |
| B s3 | 6.03 | 0 | 0.7626 / 0.7681 | 23 / 23 |
| C s1 | 6.03 | 0 | 0.6457 / 0.6512 | 15 / 15 |
| C s2 | 6.03 | 0 | 0.6978 / 0.7044 | 12 / 11 |
| C s3 | 6.03 | 0 | 0.7279 / 0.7335 | 20 / 20 |
| D s1 | 6.03 | 0 | 0.7400 / 0.7462 | 136 / 129 |
| D s2 | 6.03 | 0 | 0.7137 / 0.7201 | 133 / 132 |
| D s3 | 6.03 | 0 | 0.6855 / 0.6921 | 125 / 125 |

Hand error with correction on is lower than off in all 12 pairs, by 4–7 mm. The offset is a few mm and never at the clip. The "A offset matches rest droop" check is not evaluated: the test prints no droop value to compare with.

**Hand error is large in every run (0.4 to 0.8 m, on and off).** A few-mm correction cannot explain that size. The metric or the reference it uses needs checking before the hand-error comparison means much (Question 3).

## Baselines (current code vs Stage 1 fix `5af2083` and vs HEAD before Stage 1 `1f29da7`)

Rule as before: rows with any change above 1e-9 rad/deg. Largest joint difference per scenario (degrees, joint, time):

| Scenario | Rows changed vs Stage 1 fix | Largest vs Stage 1 fix | Rows changed vs HEAD before Stage 1 | Largest vs HEAD before Stage 1 |
|---|---|---|---|---|
| Manual (84 rows) | 30 | left joint 3, 17.43° at 8.80 s | 30 | right joint 4, 11.06° at 8.80 s |
| Teleop (78 rows) | 58 | right joint 4, 26.09° at 5.70 s (right joint 5, 26.06° at 9.20 s) | 59 | left joint 1, 17.27° at 8.60 s |
| Policy (84 rows) | 36 | left joint 2, 4.01° at 6.90 s | 36 | left joint 4, 10.95° at 6.70 s |

The speed-term change moved these numbers little: against the interim stage 2 run (before the change) the rows changed were manual 29, teleop 58, policy 36, with largest changes of a few hundredths of a degree (not tabulated per joint).

### Clamp counts per joint (joints 1–7; left, then right)

Target clamps (per source command, `BRIDGE_CLAMP["target"]`), then updates clamped and seconds clamped (updates × 0.02 s):

| Run | Left targets | Left updates | Left seconds | Right targets | Right updates | Right seconds |
|---|---|---|---|---|---|---|
| Manual | 0, 146, 13, 146, 0, 0, 0 | 0, 50, 6, 50, 0, 0, 0 | 0, 1.00, 0.12, 1.00, 0, 0, 0 | 0, 97, 0, 97, 0, 0, 0 | 0, 33, 0, 33, 0, 0, 0 | 0, 0.66, 0, 0.66, 0, 0, 0 |
| Teleop | 0, 34, 2, 386, 0, 0, 0 | 0, 12, 1, 130, 0, 0, 0 | 0, 0.24, 0.02, 2.60, 0, 0, 0 | 0, 277, 0, 301, 0, 0, 0 | 0, 93, 0, 101, 0, 0, 0 | 0, 1.86, 0, 2.02, 0, 0, 0 |
| Policy | 0, 53, 7, 73, 0, 0, 0 | 0, 19, 3, 25, 0, 0, 0 | 0, 0.38, 0.06, 0.50, 0, 0, 0 | 0, 0, 0, 1272, 0, 0, 0 | 0, 0, 0, 408, 0, 0, 0 | 0, 0, 0, 8.16, 0, 0, 0 |
| Held run (full) | 0, 51, 0, 342, 0, 1129, 0 | 0, 18, 0, 115, 0, 380, 0 | 0, 0.36, 0, 2.30, 0, 7.60, 0 | 0, 59, 0, 342, 0, 0, 0 | 0, 21, 0, 115, 0, 0, 0 | 0, 0.42, 0, 2.30, 0, 0, 0 |

The interim run's count files (`s2job`) are not used: their manual and teleop counts are identical to its policy counts, so they cannot be right.

## Timing (per call, ms)

| Run | Sag layer mean / p99 | Bridge mean / p99 | p99 sum | Limit |
|---|---|---|---|---|
| Held run (full) | 0.219 / 0.664 | 0.041 / 0.323 | 0.99 | 6.7 |
| Policy B seed 1, bridge on | 0.203 / 0.643 | 0.034 / 0.223 | 0.87 | 6.7 |

The solver-free layer is about four times faster than the stage 1 layer (3.2 ms p99).

## Held run

`outcome SUCCESS; plan failure None; status ['done', 'done', 'done', 'done']`. Learned offset peak: left 5.82 mm and 25.6 mrad, right 5.80 mm and 18.2 mrad; no clip tick. Skips: 4 no-reference ticks per arm at the start; none suspended, none non-finite.

## Questions for David (not decided, not built)

1. **Speed term (Finding 1).** Decision 4 as written feeds the layer's own correction back through the speed term. Options: (a) keep decision 4 and add a rule that removes the layer's `dq` from the speed term (the earlier design used output minus `dq`; the third diagnostic row, speed from the source targets, is the same signal and stayed small; not run as a built variant); (b) keep it as is, since the real loop has not diverged; (c) something else. Stage 2 cannot pass until this is settled.
2. **Harness for the three unapproved checks.** `case_c_periodic`, `case_s5_a` and `case_s5_d` never feed the bridge output, so the layer skips every tick. Approve the same feed as for S2 and the pause case, or retire these checks?
3. **Policy results.** (a) The offset is identical across variants and seeds (Finding 2): should I investigate before anything else? (b) Hand error is 0.4–0.8 m in every run, on and off: is the hand-error reference right? The on-vs-off gap is 4–7 mm.
4. **Stop or continue.** Stage 2 is stopped on the regression criterion. Stages 3 and 4 wait for the answers above.

## Not run

- Stages 3 and 4.
- The full `run_tests.sh` suite (only the sag, bridge and safety-filter entries were run).
- Policy runs with bridge off (not in the stage 2 criteria).
