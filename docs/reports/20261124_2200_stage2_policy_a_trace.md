# Stage 2: policy A regression traced, sag_solver_free_test failures, NaN at the pause (2026-11-24, David d0bc, follow-up)

Branch `safety-filter`, HEAD 608be13 before this round. **Stopped as the brief requires:** the policy A regression's cause is not the gravity offset (decision 3's branch), so nothing was rerun and nothing was built. Decisions 1 to 3 are recorded below and not applied. Nothing pushed. `docs/progress.md` left alone. Baseline worktrees kept (`openarm-tracer-sim-bl-1f29da7`, `openarm-tracer-sim-bl-5e6d7be`), plus a third at 7d355b8 (`openarm-tracer-sim-bl-7d355b8`, the test copy) for this trace.

## Plain-language answer

**A. The policy A regression is a bug in the logged hand error, not in the arm or the layer.** The production layer and the 7d355b8 test copy compute the same thing on policy A seed 1, tick by tick: the same predicted joints, measured joints, control, estimate, sag and command, to zero difference over all 4524 calls. The logged hand error differs (1.96 mm against 0.19 mm) because of one change I made in stage 2. `SagLayer.observe` now stores the bridge output as the reference, and the test copy stored the output minus the layer's correction dq. The hand-error metric reads that stored reference, so it counts the correction's own hand movement as error. Corrected, production's policy A value is the test copy's 0.19 mm (derived from the identical traces, not rerun). That is below the 1.07 mm with correction off, so criterion A passes once the metric is restored. The brief says to stop when the cause is not the gravity offset, so I stopped.

**Decision 3 (use the loop's applied gravity offset) is real but small.** The offset the loop applies differs from the one the layer recomputes by at most 0.58 mrad per joint (mean 0.009 mrad). That is at most about 0.3 mm of pad motion, not the 1.8 mm regression. I did not apply it.

**B. The two `sag_solver_free_test` failures are explained, and the checks, not the code, are the questionable part.** Plain-language detail below.

**C. The NaN at 1.41 s is not a pause.** The teleop only engages at about 3.3 s. Before that, no arm command had reached the bridge, so the layer had no reference and logged NaN. The pause detector read the NaN as "unchanged". Fixed in the analysis; one real pause remains (5.41 s).

## A. Policy A trace (production against the test copy, policy A seed 1, correction on, bridge off)

Runs: `data/logs/rebuild_s2/trace/production_A1/` and `test_copy_A1/` (`trace.npz` per call; runner `experiments/sag_trace_run.py`, which wraps the layer's `correct` and the gravity call and runs the policy script unchanged).

| quantity (per call, 4524 calls) | max difference, production against test copy |
|---|---|
| call time, source targets, measured joints | 0 |
| control the actuators received (`ctrl`) | 0 |
| gravity offset the layer used (recomputed, tau/kp) | 0 |
| predicted joints (after the call) | 0 |
| estimate | 0 |
| command sent | 0 |
| sag (`last_sag`) | identical (NaN on the first tick in both) |

Hand error (`policy_stream_test`, mean over 30 s; last 2 s):

| tree | hand error mean (mm) | last 2 s (mm) |
|---|---|---|
| production (HEAD 608be13) | 1.963 | 1.286 |
| test copy (7d355b8) | 0.190 | 0.258 |

The hook measures the pad against `refs[-1]`, documented as "the bridge-accepted target: bridge output minus dq". In the test copy `observe` stores `output − dq`. In production (stage 2) I changed `observe` to store the output alone, and the metric became output − 0 against the pad, so the correction's own movement counts as error. The difference is about the size of dq's hand move (1.77 mm). Correction off is unaffected (dq = 0), which is why the off value (1.07 mm) matches in both trees.

The same stored reference feeds every hand error that uses the bridge reference: policy B, C and D (bridge off) and the teleop pause error. Those values with correction on are biased by the same amount. Not affected: the offset clip counts (from the layer's own `C`, the hand-space of dq), the five static holds (which measure against the target), and the arm's motion.

**Decision 3, quantified.** The loop computes `tau/kp` once per tick at the start of its physics advance and applies it to the actuators; the layer recomputes the same expression at its own call, from the same measured pose, one tick later:

| joint | max \|loop offset − recomputed\| (mrad) |
|---|---|
| 1 | 0.32 |
| 2 | 0.54 |
| 3 | 0.38 |
| 4 | 0.58 |
| 5 | 0.24 |
| 6 | 0.54 |
| 7 | 0.41 |

Mean over all joints and calls 0.009 mrad. Largest at 2.26 s. This is real, and decision 3 would remove it. It cannot explain the regression: the trace shows the arm, the control and the layer identical in both trees, so the same offset mismatch is present in both and does not separate them. The brief's branch for decision 3 applies only when the cause is the recomputed offset, and it is not.

**What to do.** Restore the reference definition in `SagLayer.observe` (store `q_out − dq`, as the test copy and the stage 2 metric assume) and rerun policy A seed 1 to confirm 0.19 mm. That is a one-line logging change, but it is outside the branch the brief allows, so I have not made it. The teleop and policy B/C/D hand errors should be rerun after it.

## B. The two `sag_solver_free_test` failures

Both are in `experiments/sag_solver_free_test.py` on the final code (`data/logs/rebuild_s2/final/regress/sag_solver_free_test_r2.log`: 10 PASS, 2 FAIL). The test rig is a single joint-1 or joint-4 move through the bridge emulator, with the arm lagging its output by the motor's own time constant. I did not change either check or the code.

**1. "Step larger than the bridge speed limit: the error stays about zero (within 0.5 mm)."**
- *What it asserts:* when the source steps joint 1 by 0.1 rad (faster than the bridge can follow), the layer's own pad error, predicted minus measured joints taken into pad space, stays under 0.5 mm.
- *Why it fails:* 7.2 mm peak in the test run, 5.5 mm in a diagnostic run of the same rig. The rig gets the "measured" joints from the bridge output minus a velocity estimated over three ticks. The bridge output changes only every third tick, so that velocity estimate jumps, and the instantaneous error carries the jump. The estimate the layer actually applies, which is the 2 s average of that error, stays at most 0.11 mm in pad space during the step.
- *Check or code?* The check is the part that is wrong for this rule. It asserts on the instantaneous error, which the measure rule does not apply; the rule applies the average. The rig's quantised velocity is an approximation I have not checked against the full simulator. The integrator's check passed with the same rig because the integrator summed the error rather than reading it instantaneously.

**2. "Held on its buffer clamp: the offset does not wind up (position within 0.1 mm, rotation within 1e-4 rad)."**
- *What it asserts:* a joint held at its buffer edge (joint 4 commanded to 0, output at 0.1745 rad) does not build up a correction; after 900 ticks (6 s) the offset is within 0.1 mm and 0.1 mrad.
- *Why it fails:* 0.104 mm and 0.26 mrad after 6 s. A longer run of the same rig (`/tmp` diagnostic, not kept): 0.011 mm at 1 s, 0.110 mm at 3 s, 0.104 mm at 6 s, 0.023 mm at 9 s, 0.005 mm at 12 s, 0.001 mm at 15 s. The offset rises while the bridge ramps the joint to the edge, then falls with a time constant of about 2 s, the rule's own T. That is a bounded transient, not a windup. The check's 6 s window lands on its plateau, and its 0.1 mm bound is close to that plateau.
- *Check or code?* Both fit the old design's numbers more than the T = 2 s rule. The code does what the rule says (bounded, then decays). The check's window and bound are the parts I would revisit. I have not changed either.

## C. The NaN at the 1.41 s pause

- The teleop run's arms start in MANUAL. The operator engages the teleop at about 3.3 s (`ENGAGED` in `data/logs/rebuild_s2/teleop/on.log`). The bridge's first arm target arrives then; the first bridge rejection is logged at 3.42 s.
- The layer's reference (`refs`) is only set by the bridge's output. Before 3.33 s there is none, so the logged `ref` and `err_m` are NaN on all 289 ticks from 1.40 s to 3.32 s (both runs).
- The pause detector (`stage2_tables.py`) treated a NaN reference as "unchanged", so it reported a pause from 1.41 s. Fixed: a tick with no finite reference is not a pause. The regenerated tables show one pause per run, from 5.41 s (the scripted tracking loss), with hand error 2.79 mm on and 2.66 mm off. The on value carries the bias described in A.

## Decisions 1 to 3 (recorded; status)

1. **Criterion D, bridge off, report-only; Stage 3 criterion added (policy D, 3 seeds, bridge on, correction on: offset never reaches the clip).** Recorded here. Not built, since Stage 3 has not started.
2. **Release reset: do not zero the estimate; the estimate re-measures with its 2 s smoothing.** Not applied. The brief's stop rule for A applies first. When applied, it also needs the held-run hand movement in the 5 s after release, nominal and perturbed.
3. **Prediction uses the gravity offset the loop applied.** Not applied (see A). The mismatch is quantified above.

## Status of the stage

- Criterion D as revised (bridge off, policy A hand error on lower than off): **the A part passes on the corrected metric** (0.19 mm against 1.07 mm, derived from the identical traces; to be confirmed by a rerun after the one-line restore). The D clip part is report-only from now on, per decision 1.
- Not done, pending your go-ahead: the one-line restore and its rerun; decision 2 and its held-run report; decision 3; the rest of the staging and safety-filter reruns on the final code; Stages 3 and 4; the full `run_tests.sh`.

## Questions for David

1. Approve the one-line restore of the reference definition in `SagLayer.observe` (store the bridge output minus dq), with a rerun of policy A seed 1, and of teleop and policy B/C/D hand errors? I have not made the change.
2. Should decision 3 be applied now, given that its mismatch is 0.58 mrad at most and the trace shows it is not the regression?
3. For the two `sag_solver_free_test` checks: change the step check to the layer's applied estimate (as I would suggest), and the held-clamp check to a longer window, or leave them as they are and report?
4. Shall I continue with decision 2 (the release reset) and its held-run report now, since it does not depend on A?
