# Staged rebuild, stage 2: solver-free sag layer (2026-11-24)

Branch `safety-filter`, continuing from `5af2083` (stage 1 fix). **Stage 2 STOPPED: a pass criterion failed.** Command-bridge cases 2 and 5 still fail with the sag layer on (by about 1 mrad, the learned offset at work). Stage 3 and 4 were not started. The policy variants A to D and the four staging tests were not run (stopped).

## What was built

- `src/sag_layer.py`: the full solver is gone from the sag layer (no `MinkArm`). Per arm and tick:
  1. Error: expected joints = REF(t-1) - tau_j * refspeed_j, with tau_j = (kv_j + d_j) / kp_j read from the model at startup (`motor_lag_times`); pad error = Jacobian at the measured joints times (expected - measured), position and rotation.
  2. Learned offset: the existing integration (`_sag_trim_step`, gain, clip) and the 5 s fade. No update when there is no REF(t-1) and REF(t-2), while the bridge is suspended, or after a non-finite step. A non-finite error is never integrated (found and fixed by the new test).
  3. Command: the updated offset as one damped linear step, dq = J^T (J J^T + 1e-3 I)^-1 [C_p; C_r] (MINK_CFG damping); command = source targets + dq; a non-finite result sends the source targets unchanged.
  4. `SagLayer.observe(q_out, t, suspended)`: REF(t) = bridge output - dq. The control loop calls it after every bridge update (also on suspended updates), and, with the bridge off, with the command itself (REF = source target).
- `src/control_loop.py`: the observe calls (bridge on: after each update and on suspended updates; bridge off: after the sag layer).
- Removed: the warm-start reset (SF-117) and the pass-through on solver failure (SF-116/118 solver rules). The leak (SF-118) and the held-mass and pause rules are kept.
- Interpretations chosen (not in the brief, stated here): (a) the Jacobian is taken at the measured joints (from the simulator's own kinematics, no scratch copy), because the brief asks for the Jacobian at the source targets but forbids recomputing kinematics; (b) refspeed uses the time between the two references (bridge updates, every third tick), not the tick; (c) with the bridge off, the command is the output.
- Kept for experiments: `SagLayer._pose` (a measurement helper on a scratch copy, used by the policy-stream test).

## Pass criteria

| Criterion | Result |
|---|---|
| `command_bridge_test` cases 2, 4, 5 pass with the sag layer ON | **FAILED.** Case 4 passes (the clamp reaches the edge and the clamp is counted). Cases 2 and 5 reach 0.201 and 0.199 rad instead of 0.200 (the test's check is 1e-6). The learned offset of the test's arm motion is 1.1 mm (x), 0.7 mm (z) and 4.7 mrad (rotation) at the end of case 2; with the sag layer off, case 2 reaches 0.200. The sag layer learns real lag and gravity offsets in that motion, which is its job; the test's exact check does not allow for it. |
| Regression: no new failures | **Not met** (see below). |
| Four staging tests reach home | Not run (stopped). |
| Policy variants A to D, 3 seeds, correction on and off, bridge off | Not run (stopped). |
| Held run finishes SUCCESS | **PASSED** (full run, `outcome SUCCESS`, stage-2 shim adapted, see below). |
| Residual lag per joint on the ramps | Measured in the test ramp (joint 1 at 0.1 rad/s): mean residual (measured minus expected) 0.665 mrad on joint 1 and 0 on the others. Held run: not computed (stopped). |

## New tests: `experiments/sag_solver_free_test.py` (12 checks, all PASS)

- Converges to the droop (a droop fixed in the hand, 2 mm), not a multiple: C / droop = 1.055 (within 10 %); the error goes to zero (fade off).
- With the 5 s fade: the offset settles at 15/16 of the integrator's own fixed point (0.986 measured, 0.989 predicted).
- A step larger than the bridge's speed limit (0.1 rad on joint 1): the error stays within 0.095 mm; the offset changes 0.038 mm relative to its value at rest just before the step.
- A constant-speed ramp (0.1 rad/s, joint 1): the offset stays unchanged (0.0 mm).
- A joint held on its buffer clamp (joint 4 sent at 0): no windup (|C_p| 0.0000 mm).
- Skip rules: no reference (first tick), one reference only, bridge suspended, non-finite step 3, and the tick after a non-finite step: no update.

### Changes to the new test while writing it (listed, with the diagnostics behind them)

1. The first version used a joint-space droop. Its pad-space image changes with the pose (0.15 mm over the step), so "the offset does not change" was not a valid criterion. Changed to a droop fixed in the hand (pad space).
2. The convergence check ran with the 5 s fade on, whose fixed point is 15/16 of the droop. Changed: the integrator is tested with the fade off, and the fade's fixed point is a separate check.
3. The step check compared the offset with the droop, which the damped law does not reproduce exactly (the damping under-delivers in weak directions, so the offset settles about 5 % high). Changed: the offset change is measured against its value at rest just before the step.
4. The skip-rule checks compare with a 1e-9 tolerance (the fade is 1e-11 per tick even when switched off).

## Existing sag tests (not updated: the stage stopped)

- `sag_feedback_test`: 8 of 8 PASS.
- `sag_layer_test`: 1 PASS, 2 FAIL, and one crash (case c, IndexError: the pause case counts trim steps, and the harness feeds no bridge output). FAIL S1 compares the layer with the old in-script trim (the error definition changed: a retire candidate). FAIL S2 (held-mass freeze) needs the bridge output fed to the layer (`observe`).
- `sag_solver_failure_test`: solver-only (SF-116/118 failure rules), crashes on the removed `_fail_streak`: retire candidate.
- `sag_warm_start_reset_test`: solver-only (SF-117), crashes on the removed `_pose`/`_fail_streak`: retire candidate.
- `sag_feedforward_check`: exit 0, prints no PASS/FAIL lines (status unclear).
- The 18 safety-filter entries from `run_tests.sh`: all exit 0.

## Held run (full, seed 4294874296, `--liquid 0.100`)

`outcome SUCCESS; plan failure None; status ['done', 'done', 'done', 'done']`. The shim (`data/logs/gap_held_shim.py`, git-ignored scratch) was adapted: its solver-success hooks are no-ops when the layer has no solver (the same counts, now zero).

Learned offset at the end of the held run (both arms, pad position mm, rotation mrad):

| Arm | Position (x, y, z) mm | Rotation (x, y, z) mrad |
|---|---|---|
| Left | -0.725, 0.207, 0.758 | 0.463, 8.387, -3.497 |
| Right | 0.837, -0.571, -2.492 | -12.985, 11.154, -5.395 |

Skip counts: no reference at the start (4 ticks per arm); none suspended, none non-finite.

## Clamp counts per joint (target clamps; joints 1 to 7; left, then right)

| Run | Left | Right |
|---|---|---|
| Held run (full, SUCCESS) | 0, 49, 0, 342, 0, 1085, 0 | 0, 57, 0, 342, 0, 0, 0 |
| Manual baseline | 0, 133, 13, 146, 0, 0, 0 | 0, 97, 0, 97, 0, 0, 0 |
| Teleop baseline | 0, 34, 2, 386, 0, 0, 0 | 0, 277, 0, 301, 0, 0, 0 |
| Policy baseline | 0, 48, 7, 73, 0, 0, 0 | 0, 0, 0, 1272, 0, 0, 0 |
| Policy variant B (30 s, bridge on) | 0, 13, 0, 70, 0, 0, 0 | 0, 0, 0, 0, 0, 0, 0 |

## Baseline differences (`pp_compare` rule: rows with any change above 1e-9)

| Scenario | Against HEAD before stage 1 (`1f29da7`) | Against stage 1 fix (`5af2083`) |
|---|---|---|
| Manual (84 rows) | 30 rows; largest right joint 4, 11.1° (8.80 s) | 30 rows; largest left joint 3, 17.4° (8.80 s) |
| Teleop (78 rows) | 59 rows; largest left joint 1, 17.2° (8.60 s) | 58 rows; largest right joint 5, 26.1° (9.20 s) |
| Policy (84 rows) | 36 rows; largest left joint 4, 11.0° (6.70 s) | 36 rows; largest left joint 2, 4.0° (6.90 s) |

## Timing (per call, milliseconds)

| Run | Sag layer: mean / p99 | Bridge: mean / p99 | Sum of p99s | Limit |
|---|---|---|---|---|
| Held run (full) | 0.214 / 0.616 | 0.036 / 0.280 | 0.90 | 6.7 |
| Policy variant B (30 s, bridge on) | 0.285 / 0.784 | 0.043 / 0.289 | 1.07 | 6.7 |

The stage-1 numbers (layer 3.2 to 3.6 ms, p99) came from the solver-based layer. The solver-free layer is about four times faster.

## Questions for David (not decided, not built)

1. **Cases 2 and 5 of `command_bridge_test`.** They fail by 1 mrad with the sag layer on, because the sag layer's learned offset moves the command. Options: (a) accept a tolerance for the sag offset in those two checks (a test change; needs approval); (b) run the bridge test with the sag layer off (a harness change); (c) keep Stage 2 stopped.
2. **Fade and damping bias.** The 5 s fade makes the learned offset settle at 15/16 of the integrator's own fixed point (about 6 % low). The damping of 1e-3 makes the same offset about 5 % high in weak Jacobian directions. For the droop in the test these roughly cancel; in general they do not. Accept both, or change either?
3. **Jacobian location.** The brief asks for the Jacobian at the source targets and forbids recomputing kinematics. The build uses the measured joints (the simulator's own kinematics). Confirm.
4. **Reference timing.** REF is updated when the bridge updates (every third tick), and the refspeed uses the time between references. Confirm, or specify per-tick references.
5. **Retire the solver-only tests.** `sag_solver_failure_test` and `sag_warm_start_reset_test` (solver rules only), and the S1 case of `sag_layer_test` (the old in-script trim comparison). May these be retired? S2 and case c need the bridge output fed to the layer (test harness change).
6. **Bridge off.** The layer's reference is the source target (the command is the output). Confirm.

## Not run

- Stages 3 and 4.
- Policy variants A to D (24 runs), the four staging tests, and the stage-2 effect checks.
- The regression updates listed above (retirements and harness changes), pending the questions.

## Commit

Code, the new test, the design-plan row (SF-123), and this report are committed as a WIP commit (stage not passed). Not pushed.
