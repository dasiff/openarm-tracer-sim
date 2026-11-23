# Staged rebuild, stage 1 (fixed): joint-buffer clamp on the target (2026-11-23)

Branch `safety-filter`, continuing from WIP commit `bfaf692`. **Stage 1 STOPPED AGAIN: a regression pass criterion failed.** Stages 2 to 4 were not started. Nothing was tuned.

## David's decision implemented

- The buffer clamp applies to the **target** (`bridge_take`, `src/control_loop.py`): every source's joint target is clamped into [hard lower + 0.1745 rad, hard upper − 0.1745 rad].
- The bridge's existing speed rule (0.01 rad per update at 50 Hz) and step rule (rejection above 0.25 rad) then apply to the move toward the clamped target. The final-command clamp of the first version is **removed**.
- Result: a joint outside its buffer moves inward at the speed cap; it cannot move further out; it stays inside the hard limits.

## Test changes (listed)

1. `experiments/command_bridge_test.py`, case 4 (the case the brief named): the target is now past the buffer edge but within the step limit. A closed-loop source steps its target at most 0.2 rad from the bridge's current command, first to 0.01 rad inside the edge, then to 0.05 rad past it. Checks: the command reaches the edge and never passes it; the clamped target is counted; no step rejection. (The first version of the case, a 10 rad target, was rejected by the step rule and did not exercise the clamp.)
2. `experiments/command_bridge_test.py`, new case 4b: a joint starting 0.17 rad outside its buffer (joint 4 at rest, 0 rad). Checks: it reaches the edge; it reaches the edge in about 0.35 s after the first update; no step larger than 0.01 rad per update; it never moves further out.
3. `experiments/command_bridge_test.py`, helper `run(...)`: a `joint` argument (default 2 = joint 3, as before), so the cases can read other joints. No existing check changed.
4. Not changed: any test that expects a joint at rest to be commanded outside its buffer (the only such case was the stage-1 joint 4 jump, now replaced by case 4b). No test was retired.

## Pass criteria

| Criterion | Result |
|---|---|
| Regression: no new failures | **FAILED.** Four checks fail in `command_bridge_test` (cases 2, 4 and 5). At HEAD, cases 2 and 5 passed. |
| Held run (seed 4294874296, `--liquid 0.100`) finishes SUCCESS | **PASSED** (`outcome SUCCESS`, full run). |
| Four staging tests reach home | Not run (stopped). |
| Clamp counts per joint, saved | **Saved** (below). |
| Baselines against HEAD | Done (below). |

### The four failing checks, and their cause

- **Case 2** (a 0.2 rad step on joint 3 reaches its target): the command reaches 0.108 rad.
- **Case 4** (the closed-loop approach): the command goes to 0.19 rad, and the clamp count for joint 3 is 0. The command never reaches the edge, so the clamp never happens.
- **Case 5** (the target reached after a suspension): 0.19 rad against 0.2.

The cause is the **sag layer's solver**, not the bridge. The layer runs before the bridge, and its solver, which HEAD could not use at rest, now finds a feasible solution once joint 4's target is inside its buffer. The solver then moves joint 3 through its null space (joint 3 rises to 0.27 rad and falls back to 0.11 rad, with nothing logged). With the sag layer switched off (`SAG_LAYER_CFG["enabled"] = False`), case 2 reaches 0.200 rad. Stage 2 removes the solver from the sag layer, so this interaction is expected to disappear there. Because the brief allows test updates only for a joint commanded outside its buffer at rest, I did not change these three cases; they stay as failures.

Other regression results (current code):
- `sag_layer_test` 17 PASS, `sag_solver_failure_test` 10 PASS, `sag_warm_start_reset_test` 7 PASS, `sag_feedback_test` 8 PASS, `sag_feedforward_check` exit 0 (it prints no PASS lines);
- `bridge_clamp_test` 7 of 7 PASS;
- the 18 safety-filter regression entries (from `run_tests.sh`) all exit 0 (`safety_filter_model_check` and some entries print no PASS lines).

## Held run

Command: `python experiments/sag_timing_probe.py --name held -- data/logs/gap_held_shim.py <out> --liquid 0.100` (full run, same shim, seed pinned in the shim). Result: `outcome SUCCESS; plan failure None; status ['done', 'done', 'done', 'done']`. Bridge notices at the start: the joint-4 clamp of both arms at 1.40 s, then the usual navigation events.

## Clamp counts per joint (saved this time; target clamps, `BRIDGE_CLAMP`)

| Run | Left (joints 1 to 7) | Right (joints 1 to 7) |
|---|---|---|
| Held run (full) | 0, 58, 0, 83, 0, 0, 0 | 0, 58, 0, 75, 0, 0, 0 |
| Manual baseline | 0, 57, 0, 72, 0, 0, 0 | 0, 58, 0, 57, 0, 0, 0 |
| Teleop baseline | 0, 34, 0, 81, 0, 0, 0 | 0, 123, 0, 57, 0, 0, 0 |
| Policy baseline | 0, 57, 0, 69, 0, 0, 0 | 166, 0, 0, 4, 66, 0, 29 |
| Policy variant B (30 s, bridge on) | 0, 69, 0, 67, 0, 288, 0 | 0, 0, 0, 0, 0, 0, 0 |

Joints 2 and 4 are at their buffer edges at rest (joint 2 at 0, joint 4 at 0 rad); the clamp moves them inward once per run. The policy's right-arm counts (joints 1, 5, 7) and the policy B left joint 6 are targets outside the buffers, which the source sends; they are not analysed further here.

## Baselines against HEAD (`pp_compare` rule: rows with any change above 1e-9)

| Scenario | Rows changed | Largest joint change |
|---|---|---|
| Manual (84 rows) | 30 | right joint 4, 18.8° (9.20 s) |
| Teleop (78 rows) | 59 | left joint 5, 30.3° (9.20 s) |
| Policy (84 rows) | 36 | left joint 4, 11.4° (6.70 s) |

Earlier (first version of the clamp, before the fix): manual 30, teleop 59, policy 81 rows changed. The differences start at the first arms command.

## Timing (per call, milliseconds, current code)

| Run | Sag layer call: mean / p99 | Bridge call: mean / p99 | Sum of p99s | Limit |
|---|---|---|---|---|
| Held run, full (7337 sag calls, 24118 bridge calls) | 1.53 / 3.20 | 0.035 / 0.24 | 3.44 | 6.7 |
| Policy variant B, 30 s, bridge on (4524, 9048) | 1.56 / 3.62 | 0.031 / 0.15 | 3.77 | 6.7 |

Both are within the 6.7 ms limit.

## Stop

Stage 1 fails its regression criterion (cases 2, 4 and 5 of `command_bridge_test`). Stages 2 to 4 were not started.

Question for David (not decided, not built): the bridge test's failures come from the sag layer's solver interacting with the bridge. Options:
1. Accept these three failures as the expected result of stage 1, since stage 2 removes the solver from the sag layer, and proceed to stage 2 with a stage-1 exception.
2. Change the three cases to switch the sag layer off for the bridge test (a test-harness change beyond the brief's allowed updates).
3. Keep stage 1 stopped and decide the order of the stages.

## Commit

Code, tests, the design-plan row and this report are committed as a WIP commit (stage not passed). Not pushed.

## Not run

- Stages 2 to 4.
- The four staging tests.
