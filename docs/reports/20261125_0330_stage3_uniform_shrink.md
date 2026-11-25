# Stage 3: one uniform shrink for every speed and step limit in the command bridge (2026-11-25, David d0bc)

Branch `safety-filter`, HEAD fcdcbd2 (Stage 2 finished) before this stage. One MuJoCo process at a time. Runs in `data/logs/rebuild_s3/` (queue `queue_s3.sh`, log `progress.txt`). Nothing pushed. Baseline worktrees kept.

**Stage 3 PASSES** on every criterion below, including the one David added (policy D, three seeds, bridge on, correction on: no offset at the clip).

## Plain-language answer

1. **The bridge's move rules, before this stage** (all in `src/control_loop.py` `bridge_update`):
   - Step-size rule: a target more than 0.25 rad from the command, on any joint, was rejected as a whole, and the previous accepted target stayed.
   - Speed cap: the command moved at most 0.01 rad per update (0.5 rad/s at 50 Hz), joint by joint (a per-joint clip).
   - Kept: the tracking-error suspension (0.25 rad), the buffer clamp on each target (before the shrink), and the braking override (`src/bridge_override.py`, the safety filter's jump, which is not a limit on a move, and which is listed as open below).
2. **Now: one shrink per update for both arms.** For each rule the factor is its limit over the largest per-joint move of either arm; the smallest factor applies to every joint's move on both arms. Inside the limits the factor is 1 and the target is reached exactly; a zero move stays zero; the direction of each move is kept. The speed rule binds in practice (0.25 rad is never the smaller limit at a 0.01 rad speed cap).
3. **No target is rejected any more.** The teleop run has zero rejections, and so does every policy run; in the teleop 137 updates (correction on) and 128 (off) were shrunk, the smallest factor 0.0078.
4. **Teleop after the pause (report only; David's request).** With correction on, the hand error 0.5, 1 and 2 s after the pause is 16.3, 15.2 and 10.0 mm, and the estimate falls from 36 mrad at the pause to 11 mrad at its end. With correction off it is 3.1, 2.7 and 2.6 mm. These are close to the Stage 2 numbers (17.1, 15.7, 10.3 mm): the shrink did not change the teleop build-up, and the relaxation is the every-tick update's (Stage 2).
5. **Policies with the bridge on** (B, C and D, correction on): no offset at the clip in any run (D: three seeds), no rejections or suspensions, and the arm keeps moving toward the target (pad path 6 to 8 m in 30 s; 0.3 to 0.7 m in the last 2 s).
6. **Surprising:** the policy's own speed is far above the bridge's cap (the source commands up to about 5 rad/s); the bridge limits the command to 0.5 rad/s, so the hand error stays at 15 to 21 mm (the reference is well ahead of the arm). It is bounded, not growing.

## The rules, after this stage

| rule | where | what it does now |
|---|---|---|
| tracking-error suspension | `bridge_update` | holds the command while the arm lags by more than 0.25 rad (unchanged) |
| buffer clamp | `bridge_take` | clamps each target into its joint buffer (SF-125; unchanged) |
| uniform shrink (new) | `src/bridge_shrink.py`, `bridge_update` | factor = min(1, 0.25 / peak, 0.01 / peak), peak = the largest per-joint move of either arm; each arm's move scaled by the factor |
| braking override | `src/bridge_override.py` | the safety filter's jump when braking exceeds the limits; logged as an error (unchanged; see open question) |

## Pass criteria

| criterion | result |
|---|---|
| Regression: sag tests, sag_feedback, sag_solver_free, sag_relax, bridge_clamp, command_bridge (changed case 3), bridge_shrink (new), source_view, sag_feedforward, safety-filter group | **PASS**: all 40 logs exit 0; command_bridge 21/21; bridge_shrink 13/13 |
| Staging tests (four) | **PASS**: exit 0 each |
| Policy B, C and one seed of D, bridge on, correction on: no rejections, the arm keeps moving, offset never at the clip | **PASS** (below) |
| Policy D, 3 seeds, bridge on, correction on: offset never at the clip (decision 1, added) | **PASS**: 0 clip ticks in all three seeds |
| Held run SUCCESS | **PASS**: nominal and perturbed (both `SUCCESS; plan failure None`) |

## Policy runs, bridge on, correction on (30 s, seed 1 unless stated)

| run | rejections | suspensions | offset peak (mm) | clip ticks (pos / rot) | hand error mean (mm) | hand error last 10 s (mm) | pad path, last 2 s (mm) |
|---|---|---|---|---|---|---|---|
| B | 0 | 0 | 2.65 | 0 / 0 | 17.8 | 21.2 | 650 |
| C | 0 | 0 | 2.68 | 0 / 0 | 15.3 | 15.8 | 747 |
| D, seed 1 | 0 | 0 | 1.70 | 0 / 0 | 16.4 | 16.0 | 524 |
| D, seed 2 | 0 | 0 | 2.24 | 0 / 0 | 15.1 | 14.8 | 285 |
| D, seed 3 | 0 | 0 | 1.81 | 0 / 0 | 15.5 | 15.8 | 326 |

Hand error is the pad of the reference (bridge accepted target minus dq) minus the pad of the measured joints.

## Held run

- Nominal: SUCCESS (`held_nominal/timing.log`); sag call mean 0.214 ms, p99 0.680 ms; bridge mean 0.044 ms, p99 0.337 ms.
- Perturbed: SUCCESS (`held_perturbed/trace.log`).

## Teleop after the pause (correction on and off; report only)

| correction | rejections | shrunk updates (smallest factor) | hand error 0.5 / 1 / 2 s after the pause (mm) | estimate: at pause / mean during / at +3 s (mrad) |
|---|---|---|---|---|
| on | 0 | 137 (0.0078, all speed rule) | 16.33 / 15.21 / 9.95 | 36.2 / 20.1 / 11.0 |
| off | 0 | 128 (0.0078) | 3.13 / 2.67 / 2.61 | 0 / 0 / 0 |

## Timing

Bridge, Stage 3 runs (policy B, D seed 1, held nominal): mean 0.044 to 0.053 ms, p99 0.30 to 0.35 ms, against 0.04 ms and 0.3 ms before the stage. The shrink is a few vector operations per update, so no measurable change.

## Tests changed or added

- `experiments/command_bridge_test.py`, case 3: was "a 0.6 rad step is rejected as a whole and logged"; now "a 0.6 rad step is not rejected: it moves at 0.01 rad per update, reaches the target, no rejection is logged, and 59 shrunk updates are logged (factor 0.01 / move, speed rule, left arm, joint 3)". The other cases are unchanged and pass (21/21).
- New: `experiments/bridge_shrink_test.py` (13 checks): within the limits the move is exact; over the speed limit the largest move is 0.01 rad and the direction is kept (cosine 1 within 1e-9); the step rule binds when its limit is smaller; a large left move shrinks the right arm's move by the same factor and vice versa; one arm's zero move stays zero; a repeated over-limit target reaches its target in 60 updates; the tracking-error suspension still fires.
- Code: `src/bridge_shrink.py` (new, pure functions); `src/control_loop.py` (`bridge_update`, `BRIDGE_SHRINK`, the `bridge_shrink` entry of the run result, the module docstring); `src/task_config.py` (comments of the two limits; no value changed); `experiments/teleop_tick_log.py` (saves the shrink events).

## Open questions

1. **The braking override** (`src/bridge_override.py`) still jumps the command to the braking target when the look-ahead fails. It bypasses the shrink, as it bypassed the old limits. Keep, or route through the shrink? Not changed here.
2. **Teleop after the pause** is essentially unchanged by the shrink (see the Stage 2 report and the David answer on silence). David decides on any weighting after this stage (answer 2).

## Not run yet

- The baseline differences against Stage 2 (report only) and the teleop check of rejections and shrink events beyond the single run above.
