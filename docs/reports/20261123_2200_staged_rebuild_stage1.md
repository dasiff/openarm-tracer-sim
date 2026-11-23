# Staged rebuild, stage 1: joint-buffer clamp in the bridge (2026-11-23)

Branch `safety-filter`, HEAD `1f29da7` before this stage. **Stage 1 STOPPED: a pass criterion failed.** Per the brief, stages 2 to 4 were not started. Nothing was tuned.

## What changed

- `src/bridge_clamp.py` (new): `buffer_limits` (hard limit moved in by MINK_CFG `joint_margin_rad`, 0.1745 rad) and `clamp_to_buffer` (per joint; a value inside its buffer passes unchanged; reports which joints changed).
- `src/control_loop.py`:
  - every joint target is clamped into its buffer when the bridge takes it (`bridge_take`), for every source; the final command is clamped into the buffer after the bridge's speed cap (`bridge_update`);
  - out-of-range arm targets are no longer rejected by `bridge_precheck` (validate_command still clips to the hard limits first, as before);
  - per-joint clamp counts (`BRIDGE_CLAMP`, logging) and per-call bridge timing (`BRIDGE_TIME`, logging). The sag layer is unchanged in this stage (its solver still runs).
- `experiments/bridge_clamp_test.py` (new, 7 checks).
- `experiments/command_bridge_test.py`: case 4 changed from "an out-of-range target is rejected" to "clamped to the buffer edge". Its test target (10 rad) is also a 10 rad step, so the step rule rejects it first (see the failures below): the case is confounded and needs a redesign.
- `experiments/sag_timing_probe.py`: rewritten to time the sag layer's call and the bridge's call (the earlier version timed the two-pass prototype, which is gone).
- Decision entry SF-122 in `docs/design_plan.md`.

The rules the bridge applies (for the brief's first step): (1) suspension when the command is more than 0.25 rad from the measured joints (hold); (2) rejection of a target more than 0.25 rad from the current command on any joint (`max_target_step_rad`, `bridge_update`); (3) a per-joint clip of each step to 0.5 rad/s / 50 Hz = 0.01 rad per update (`bridge_update`); (4) rejection of arm targets outside the hard joint limits (`bridge_precheck`, now replaced by the clamp). Rules 1 and 2 are unchanged in this stage; rule 3 is unchanged; rule 4 is now the clamp.

## Pass criteria and results

| Criterion | Result |
|---|---|
| Regression has no new failures | **FAILED.** `command_bridge_test`: 14 of 14 at HEAD, 7 of 15 with stage 1. The five sag tests, `sag_feedback_test` and the safety-filter regression were not run (stopped at the first failure). |
| Held run (seed 4294874296, `--liquid 0.100`) finishes SUCCESS | **Not run** (stopped). A 12 s timing run reached the handover without errors. |
| Four staging tests reach home | Not run (stopped). |

### Why `command_bridge_test` fails

- Case 2 (a 0.2 rad step is limited to 0.01 rad per update, and the command reaches the target): the command reaches only 0.103 rad in the test's time.
- Case 3 (a 0.6 rad step is rejected as a whole): the command moves to 0.335 rad, so the rejection does not hold the command.
- Case 5 (the tracking-error suspension still resumes and the target is reached): the target is not reached in the test's time.

All three start from the same place. The test's rest pose is the bridge's initial command (the hold), and at rest **joint 4 is 0 rad, below its buffer edge of 0.1745 rad** (joint 2 is at 0, its buffer edge exactly). The clamp therefore moves the command by about 0.17 rad on the first bridge update (`bridge_update`, the final-command clamp, logged as "clamped: command into the joint buffer: left joint 4, value +0.010, limit +0.175" at t = 1.40 s). That is a jump larger than the 0.01 rad per update the speed cap allows, and it changes every later check in the test. The 0.17 rad jump is also what the held-run and baseline logs show at 1.40 s, on both arms.

Case 4 (out-of-range target, new checks): the target 10 rad is also a step far above 0.25 rad, so the step rule rejects it ("rejected: target step too large"), and the clamp is not what the test measures. The three new case-4 checks fail for that reason; the case needs a target inside the step limit but outside the buffer.

## Clamp counts per joint

Not fully recorded. `BRIDGE_CLAMP` counts every clamp per joint, but the probe did not save it; the run logs print only the first event for each distinct joint. From the logs (first events per run):

| Run | Clamped at first update |
|---|---|
| Held run to 12 s | left joint 4, right joint 4 |
| Policy B (30 s, bridge on) | left joint 4 |
| Manual baseline | left joint 4, right joint 4 |
| Teleop baseline | left joint 4, right joint 4 |
| Policy baseline | left joint 4, right joint 2 |

Rejections on the step rule (unchanged in this stage) in the baselines: manual 3, teleop 6, policy 9 distinct events (the step rule's first events).

## Baseline differences against HEAD (`pp_compare` rule: rows with any change above 1e-9)

| Scenario | Rows changed | Largest joint change |
|---|---|---|
| Manual (84 rows) | 30 | left joint 4, 24.5° (8.00 s); right joint 4, 25.8° (9.10 s) |
| Teleop (78 rows) | 59 | left joint 5, 30.6° (9.20 s); right joint 4, 21.4° (4.50 s) |
| Policy (84 rows) | 81 | left joint 4, 16.6° (7.10 s); right joint 4, 11.4° (2.00 s) |

The differences start at the first arms command (policy from 1.8 s, teleop from 3.7 s) and grow with the arms' path. The base and pedestal columns change by under 0.1 mm and 0.1° in all three scenarios (not analysed further).

## Timing (per call, milliseconds, Stage 1 with the clamp)

| Run | Sag layer call: mean / p99 | Bridge call: mean / p99 | Sum of the two p99 | Limit |
|---|---|---|---|---|
| Held run, first 12 s (`n` 3092 sag calls, 3181 bridge calls) | 1.60 / 3.95 | 0.055 / 0.27 | 4.22 | 6.7 |
| Policy-stream variant B, 30 s, bridge on (`n` 4524, 9048) | 1.50 / 3.25 | 0.038 / 0.18 | 3.43 | 6.7 |

Both are within the 6.7 ms limit, so the timing stop does not apply.

## Stop

Stage 1's regression criterion failed, so stage 2 was not started. Open question for David (not decided, not built):

1. **The rest pose is outside the buffer.** Joint 4 starts at 0 rad, 0.17 rad below its buffer edge. Options: (a) clamp the command gradually at the speed cap (the spec as written clamps at once), (b) start the arms inside the buffer (a change of rest pose or of the start), (c) keep the one-time jump as it is and change the tests' rest pose.
2. **Case 4 needs a target inside the step limit and outside the buffer** (for example joint 2 at 0.1 rad from its rest). The test design to use is to be agreed.

## Commit

Code, tests and the design-plan entry are committed as a WIP commit (stage not passed), so the state is recoverable. Not pushed.

## Not run

- Stage 1 regression (the five sag tests, `sag_feedback_test`, the safety-filter regression), the held run to SUCCESS, the four staging tests, and the policy variants of stage 1.
- Stages 2, 3 and 4 (not started, per the brief).
