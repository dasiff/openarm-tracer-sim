# Stage 2, final rerun after David's follow-up answers: STOPPED on criterion C (perturbed held run) (2026-11-24, David d0bc)

Branch `safety-filter`, worktree `openarm-tracer-sim-sf-d`. HEAD before this round: baf0d52. Code commit: `5dda1ad` (WIP: decisions 2 and 3, the restored reference, the test changes; rerun pending at that commit). Runs: `data/logs/rebuild_s2c/` (queue `queue_s2c.sh`, log `progress.txt`); ablation `data/logs/rebuild_s2c/ablation_offset/`. Nothing pushed. `docs/progress.md` left alone. Baseline worktrees kept (`openarm-tracer-sim-bl-1f29da7`, `openarm-tracer-sim-bl-5e6d7be`, `openarm-tracer-sim-bl-7d355b8`).

**Stage 2 did not pass.** Criterion C (held run SUCCESS on nominal physics and with the deliberate model error) fails: the perturbed held run ends `PLAN_FAILED` at segment 2/4 (pick up the beaker): "CLOSE: the gripper did not stop within 20.5 s". A one-variable ablation shows that decision 3 (the prediction uses the offset the loop applied) is the cause: with the layer recomputing its offset as before, the same run is SUCCESS. Per the brief, stages 3 and 4 and the full `run_tests.sh` were not started.

## Plain-language answer

1. **Decision 1 (criterion D, bridge off, report-only): applied.** Policy D's clip counts are reported, not judged. Stage 3's added criterion (policy D, 3 seeds, bridge on, correction on, offset never at the clip) is recorded for Stage 3 and not built.
2. **Decision 2 (no zeroing of the estimate on release): applied.** The release keeps the estimate, which decays with its 2 s smoothing. Held run, 5 s after release: the offset falls from 5.5 mm to 1.7 mm (nominal) and from 18.6 mm to 4.6 mm (perturbed; from the ablation run, see below). The pad moves about 120 to 130 mm in those 5 s: that is the held run's next planned motion, not a release effect.
3. **Decision 3 (predict with the offset the loop applied): applied, and it breaks criterion C.** The perturbed held run fails with it and succeeds without it (ablation). The arm trajectories differ by at most 0.15 mrad, and both runs close the fingers to the same 37.5 mm; the gripper's stop detection then fails in the new run. I have not found the mechanism within that 0.15 mrad difference.
4. **Reference restored (answer 1):** `SagLayer.observe` stores the output minus dq. Every reader of the reference expects that (listed in §1). Policy A's hand error is now computed correctly.
5. **sag_solver_free_test (answer 3): both checks changed, as asked.** All 13 checks now pass.
6. **Policy A (criterion D revised): passes.** Hand error with correction on 0.18 mm, against 1.07 mm off (last 2 s: 0.25 against 1.00).
7. **Teleop (criterion E): the offset never reaches the clip (peak 29.6 mm).** But the hand error one second after the pause is 22.7 mm with correction on, against 2.7 mm off. The earlier 2.8 mm figure was measured against the output (which includes dq), so it hid this. The correction displaces the arm about 22 mm from the source at rest. This needs a decision (§5).
8. **Timing:** the held run's sag layer call falls from 1.47 ms mean (p99 3.65 ms) to 0.19 ms (p99 0.55 ms). Policy A (correction on): from 0.41 ms (p99 1.14 ms) to 0.19 ms (p99 0.61 ms). The bridge is unchanged (p99 0.42 ms, held run).

## Criteria, on the final code

| criterion | result |
|---|---|
| A. Regression: sag tests, sag feedback, command bridge (sag off), bridge clamp, source view, sag feedforward, full safety-filter group | **PASS.** sag_layer 13/13; sag_solver_free 13/13; sag_feedback 8/8; command_bridge 19/19 (sag off); bridge_clamp 8/8; source_view 3/3; sag_feedforward exit 0; the 18 safety-filter tests exit 0. |
| B. The four staging tests reach home | **PASS.** `home_pose_test`, `grasp_lift_test`, `beaker_hotplate_correction_test`, `nav_follow_sim_test --only on`: exit 0 each. |
| C. Held run SUCCESS, nominal and with the model error | **FAIL.** Nominal: SUCCESS (timing run and trace run). Perturbed (decisions 2 and 3): `PLAN_FAILED`, segment 2/4, CLOSE did not stop in 20.5 s. Perturbed ablation (decision 3 reverted): SUCCESS. |
| D (revised). Policy A (bridge off, 3 seeds): correction on lower hand error than off | **PASS.** 0.18 mm against 1.07 mm (last 2 s: 0.25 against 1.00). Policies B, C, D reported (table below). Policy D's clip counts are report-only (decision 1). |
| E. Teleop, correction on and off, logged through tracking loss: offset never at the clip | **PASS** on the clip (29.6 mm peak; 0 ticks at either clip). Hand error after the pause is reported (§5), not judged. |
| F. Five static holds, model error: reported | **Reported.** Clip 0 (peak 6.60 mm). |
| Timing | Reported (§8). |
| Baselines (report only) | Reported (§9). |

## Policy runs (bridge off, 30 s, 3 seeds; `data/logs/rebuild_s2c/analysis/stage2_tables.md`)

Hand error is the mean over 30 s (correction on / off) and over the last 2 s, in mm. Offset and rotation peaks in mm and mrad.

| variant | seed | offset peak (mm) | clip ticks (pos / rot) | hand error on / off | last 2 s on / off |
|---|---|---|---|---|---|
| A | 1–3 | 2.46 | 0 / 0 | 0.18 / 1.07 | 0.25 / 1.00 |
| B | 1 | 4.76 | 0 / 0 | 38.8 / 39.7 | 46.4 / 48.2 |
| B | 2 | 5.12 | 0 / 0 | 43.3 / 44.0 | 41.9 / 43.1 |
| B | 3 | 6.99 | 0 / 0 | 38.5 / 39.1 | 46.3 / 46.4 |
| C | 1 | 3.74 | 0 / 0 | 31.4 / 31.8 | 30.7 / 31.2 |
| C | 2 | 3.70 | 0 / 0 | 34.7 / 35.2 | 37.3 / 37.5 |
| C | 3 | 7.74 | 0 / 0 | 31.2 / 31.4 | 31.5 / 30.5 |
| D | 1 | 40.00 | 76 / 256 | 240.7 / 239.7 | 227.1 / 225.4 |
| D | 2 | 38.81 | 0 / 370 | 235.6 / 233.9 | 249.5 / 249.1 |
| D | 3 | 39.80 | 0 / 70 | 231.1 / 228.9 | 187.4 / 191.2 |

Policy D (bridge off) reaches the clip; that is report-only from now on. Policies B and C: correction on and off give the same hand error to within a few percent.

## Teleop (bridge on, every tick logged, `data/logs/rebuild_s2c/teleop/`)

- Correction on: 1176 ticks, 1.40 to 9.23 s. Offset peak |C_p| 29.6 mm, peak |C_r| 65.9 mrad; 0 ticks at either clip.
- Correction off: offset 0.
- One pause (REF unchanged at least 1 s), from 5.41 s (the scripted tracking loss; the first 3.3 s have no bridge reference and are not a pause): |C_p| at the stop 24.6 mm (on); hand error 1 s after the stop **22.7 mm** (on), **2.7 mm** (off).

## Five static holds, deliberate model error, correction on (`static/perturbed.md`)

| pose | offset against droop at the stop (mm) | hand error 1 s after the stop (mm) | \|C_p\| at the end (mm) |
|---|---|---|---|
| home (start transient, reported separately) | 0.33 | 2.42 | 1.32 |
| pick | 0.20 | 1.91 | 5.39 |
| policy B, 7.4 s | 0.21 | 3.36 | 3.73 |
| policy B, 13.4 s | 0.22 | 2.44 | 4.20 |
| policy B, 21.4 s | 0.20 | 1.91 | 0.74 |

Mean over the four moving poses of the hand error 1 s after the stop: 2.4 mm. Peak |C_p| 6.6 mm; ticks at the 40 mm clip 0.

## Held run

- Nominal (decisions 2 and 3): SUCCESS (`held_nominal/timing.log`, `held_nominal/trace.log`).
- Perturbed (decisions 2 and 3): `PLAN_FAILED`, segment 2/4 (pick up the beaker), "CLOSE: the gripper did not stop within 20.5 s" (CLOSE began at 61.3 s, failed at 81.8 s). The trace ends at 61.4 s, the last arm command (the grasp closes after that).
- Ablation (decision 3 reverted; decision 2 and the reference restore kept): SUCCESS, release at 93.69 s. This is the only control change between the failing and passing runs.
- What I compared (both runs, same timeline): the arm joints differ by at most 0.15 mrad over 1.4 to 61 s (about 0.01 mrad from 10 s on, up to 0.15 at 55 s); the finger closes to 43.9 mm and then to 37.5 mm in both. I did not find the mechanism within that difference: the gripper's stop test (the finger's speed and travel over its dwell) is the place to look, and it is sensitive at this size of difference.

### Hand movement in the 5 s after release (answer 4)

From `data/logs/rebuild_s2c/analysis/release_report.md` (the pad from the joints, base frame):

| run | release at (s) | \|C_p\| at release (mm) | \|C_p\| at +5 s (mm) | largest pad movement in 5 s (mm) | path length (mm) |
|---|---|---|---|---|---|
| nominal (decisions 2 and 3) | 75.24 | 5.5 | 1.7 | 130.4 | 231.2 |
| perturbed (ablation, decision 3 reverted) | 93.69 | 18.6 | 4.6 | 121.1 | 220.6 |

The pad's movement is the held run's next planned motion (the beaker is carried after the release); it is not a release effect. The release's own effect is the offset, which decays with the 2 s smoothing rather than dropping to zero. The perturbed row is from the ablation: the decision 3 build never reached its release.

## Timing (`sag_timing_probe.py`, per call, ms)

| run | before (recomputed offset, `rebuild_s2/final`) | after (applied offset, `rebuild_s2c`) |
|---|---|---|
| held, nominal: sag layer mean / p99 | 1.469 / 3.649 | 0.187 / 0.547 |
| held, nominal: bridge p99 | 0.420 | 0.419 |
| policy A seed 1, correction on: sag layer mean / p99 | 0.411 / 1.137 | 0.187 / 0.613 |

The bridge code is unchanged; its timing is unchanged within noise.

## Tests changed in this round

- `experiments/sag_solver_free_test.py`:
  - Step check ("step larger than the bridge speed limit"): now tests the applied estimate, not the instantaneous error. It reads the estimate's pad-space movement from its value at rest, at each tick after the step, against the 0.5 mm bound (unchanged). The estimate is preset to the 2 mm droop, so its absolute value is 2 mm by construction; the check is how far it moves.
  - Held-clamp check ("held on its buffer clamp"): now tests for windup, not size. The transient ends when the joint reaches its lower buffer edge (4.04 s here). From 3T (6 s) after that: (a) the offset does not grow (within 1 µm); (b) it is below the existing bound (0.1 mm, 1e-4 rad). Both pass (after-window maximum 0.005 mm).
  - Harness: the rig gives the layer the offset the loop applies (decision 3); the skip checks follow the loop's order.
- `experiments/sag_layer_test.py`: harness gives the layer the loop's offset; the two release cases now expect the estimate kept (decision 2).
- New analysis scripts (no simulator, except the FK step): `experiments/stage2_release_report.py`, `experiments/held_offset_ablation.py` (diagnostic; patches only in its own process).
- `experiments/stage2_tables.py`: the held table reads the new folder layout.

## Code changes in this round

- `src/sag_layer.py`:
  - `observe` stores REF = output − dq (readers: `policy_stream_test.py` hand error, `teleop_tick_log.py`, `climb_perturb_diag.py` ref record, `predictor_compare.py` ref record; all expect output − dq).
  - The gravity recomputation is removed (`self.gff`, `kp_arm`, `gravity_model`). The layer predicts with the offset set by `set_applied_offset`; with none set yet the tick is skipped (`no_offset`).
  - The open-catch completion no longer zeroes the estimate or the correction (`reset_at` kept as an event marker).
  - Module docstring updated.
- `src/control_loop.py`: `advance()` passes each arm's part of the loop's offset to its layer every tick.
- `experiments/climb_static.py`: the harness passes the offset the loop would apply (its nominal gravity copy) to the layer; `gravity_model` argument removed.

## Questions for David

1. **Criterion C.** Decision 3 makes the perturbed held run fail (CLOSE, 81.8 s), and reverting it restores SUCCESS. Do you want decision 3 reverted (and the criterion kept), the gripper's stop detection looked at first, or Stage 2 accepted with C failed?
2. **Teleop with correction on.** The arm sits about 22 mm from the source one second after a pause, and the offset goes to 25 mm at the stop. The old 2.8 mm figure was measured against the output and hid this. Should the teleop correction be looked at before Stage 3?
3. **Release hand movement.** The 120 to 130 mm pad movement is planned motion. Do you want the release reported against the plan's own waypoints instead?
4. **Policy D with the bridge off.** Report-only, as decided. Its clip counts are in the table above.

## Not done

- Stages 3 and 4 and the full `run_tests.sh`: not started (Stage 2 failed criterion C).
- The staging and safety-filter groups were run on this final code (they pass); no separate run of them after the stop.
