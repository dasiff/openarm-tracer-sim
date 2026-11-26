# Stage 2: motor-filter model, tests, bridge-off policy, manual against the 2026-11-03 reference (2026-11-24, David's d0bc decisions)

Branch `safety-filter`. **Stage 2 is not passed: criteria 1 to 3 of David's decisions do not all pass, so Stages 3 and 4 were not started.** The layer was changed as decided (motor filter, reference-speed term removed). Not pushed.

Commits: `4b304ef` (layer and tests). Data (git-ignored): `data/logs/stage2f/`.

## 1. What changed (listed)

| File | Change | Decision |
|---|---|---|
| `src/sag_layer.py` | expected joints = first-order motor filter of the reference: `expected(t) = expected(t-1) + (dt/tau_j) (REF(t-1) - expected(t-1))`; set to the measured joints on the first tick and after a suspended update; reference-speed term and `_ref_speed` removed; `last_refspeed` removed | 2 |
| `src/sag_layer.py` | the two-reference rule (`no_reference`) kept as it was (a choice: the filter needs only one reference; the skip rule was not changed, to avoid an unstated change) | — |
| `experiments/sag_solver_free_test.py` | held-still bridge-on check: the zero-mean control is the pass criterion; the constant-offset run is informational; **bridge-off refspeed check retired** (the term no longer exists); new staircase test (2b), single-step report (2c) | 1, 2 |
| `experiments/sag_solver_free_test.py` | `refspeed` removed from the recorded data | 2 |
| `data/logs/stage2e/heldstill_log.py`, `data/logs/stage2d_diag/diag_policy.py` | refspeed columns removed (scratch scripts) | 2 |

Tests not changed: the convergence, fade, step, ramp (2a), buffer-clamp and skip-rule checks keep their criteria.

## 2. Test results (`sag_solver_free_test`, the test's own emulator)

The test's emulator moves the held or following arm with an **algebraic lag** (measured = output − tau·speed). The filter is a **dynamic first-order** model. Both are shown; the criteria are not changed.

| Check | Criterion | Result (test emulator) | Result (arm emulated as the same first-order filter, diagnostic only) |
|---|---|---|---|
| Converges to the droop | ratio 0.9–1.1 | PASS, 1.055 | — |
| Error to zero at rest | < 0.05 mm | PASS | — |
| 5 s fade fixed point | 15/16 ± 0.01 | PASS, 0.9858 | — |
| Step 0.1 rad: error after the step | ≤ 0.5 mm (old check, kept) | **FAIL, 5.59 mm** | 0.559 mm (marginal fail) |
| Step 0.1 rad: offset change | ≤ 0.1 mm | PASS, 0.045 mm | — |
| (2a) constant-speed ramp: offset change | ≤ 0.5 mm | PASS, 0.23 mm | — |
| (2b) staircase, 30 Hz, 5 ticks, fixed droop | ≤ 0.5 mm | **FAIL, 0.775 mm** | **0.0023 mm, PASS** |
| (2c) single 0.02 rad step, all joints: max \|expected − measured\| per joint (report) | report only | 15.2, 13.7, 10.7, 13.1, 46.2, 41.3, 36.4 mrad (joints 1–7) | 2.4, 2.6, 3.0, 2.6, 1.1, 1.2, 1.3 mrad |
| Buffer clamp (joint 4 held at 0): \|C_p\| | ≤ 0.1 mm | **FAIL, 0.169 mm** | 0.0000 mm, PASS |
| Held still, bridge on, zero-mean sweep: bounded | \|C_p\| ≤ 10 mm, \|C_r\| ≤ 10 mrad, no growth late | **FAIL on the rotation bound**: 7.8 mm, 22.5 mrad; **no growth** (False) | 8.1 mm, 23.4 mrad; no growth |
| Held still, constant +0.01 rad on joint 3 (informational) | report | clip reached: 29.5 mm, 85.9 mrad | — |
| Skip rules (5 checks) | — | PASS | — |

Other sag tests: `sag_layer_test` **16 of 16 PASS**; `sag_feedback_test` **8 of 8 PASS**.

Reading: the filter reproduces a filter-modelled arm closely (staircase 0.002 mm, clamp 0.0000 mm). The test's algebraic emulator is a different arm model; with it, the staircase and clamp checks fail. Whether the synthetic tests should use the same motor model as the layer is David's decision (Question 2). The zero-mean held control is bounded (no growth), but its rotation amplitude (23 mrad) is above the 10 mrad bound I wrote earlier; the bound is David's decision (Question 1). The held-still growth in the earlier report is the constant unreachable offset (unchanged: it reaches the clip, as the informational run shows).

## 3. Policy variants A–D, 3 seeds, correction on and off, bridge OFF (the real simulation)

No bridge event in any run (the bridge is off). All 24 exit 0. Hand error is against the accepted reference, which with the bridge off is the source target. "Rest droop" = the hand error with correction off in A (1.07 mm, the steady droop the correction should remove).

| Run | Offset peak left (mm) | Final C_p (mm) | Final \|C_p\| (mm) | Clip ticks | Hand err on / off (mm) |
|---|---|---|---|---|---|
| A s1 / s2 / s3 | 2.18 | [−0.28, 0.92, 0.69] | 1.18 | 0 | 0.10 / 1.07 |
| B s1 | 27.5 | [−6.2, −5.4, 11.0] | — | 18 | 30.2 / 39.7 |
| B s2 | 28.3 | [7.3, 1.0, 7.0] | — | 129 | 38.9 / 44.0 |
| B s3 | 40.0 | [0.4, 6.8, 4.9] | — | 161 | 35.3 / 39.1 |
| C s1 | 22.3 | [9.1, −2.2, 6.6] | — | 9 | 29.0 / 31.8 |
| C s2 | 28.6 | [5.1, 7.2, −3.8] | — | 10 | 32.9 / 35.2 |
| C s3 | 40.0 | [−10.4, −8.1, −1.1] | — | 99 | 30.8 / 31.4 |
| D s1 | 40.0 | [9.1, 3.4, 16.2] | — | 4266 | 265.0 / 239.7 |
| D s2 | 40.0 | [19.1, 6.7, 8.7] | — | 4485 | 255.0 / 233.9 |
| D s3 | 40.0 | [19.1, 6.3, −0.5] | — | 3891 | 252.9 / 228.9 |

Criteria:
- **A (offset matches the rest droop): PASS.** Final |C_p| 1.18 mm against the rest droop 1.07 mm (10 % high); the hand error drops from 1.07 to 0.10 mm.
- **B and C (offset a few mm, never at the clip; hand error on no worse than off): FAIL on the offset.** The offset reaches 22–40 mm and clips on 9–161 ticks. The hand error with correction on is lower than off in all six runs (on is no worse), which passes that half.
- **D (stress: pass if the offset never reaches the clip): FAIL.** The offset clips on every seed (3900–4500 ticks). Hand error with correction on is higher than off in all three seeds (253–265 mm against 229–240 mm).

**Why B, C and D wind up.** Offline, the filter's expected joints were rebuilt from the saved reference (the same tau from the model: 0.045, 0.041, 0.034, 0.040, 0.11, 0.10, 0.09 s). During the stream the measured joints lag the filter's expected joints by a large, one-signed amount when the arm moves fast:

| Run | Mean residual, stream (mrad), joints 1–7 | RMS (mrad) | Max source speed from the reference (rad/s) |
|---|---|---|---|
| A s1 | 0.6, 2.2, −1.7, 0.1, 1.1, −2.9, 0.8 | 0.1–2.9 | 0 (static) |
| B s1 | −21.8, 3.6, 13.1, −36.4, 3.7, 4.8, −6.0 | 19–48 | 8.5 |
| C s1 | −23.4, 5.3, 11.1, −28.0, 6.6, −0.2, −5.9 | 19–42 | 4.0 |
| D s1 | −64.0, 59.3, 38.4, −129.6, 33.5, −29.7, 5.3 | 106–410 | 19.9 |

The first-order filter underestimates the arm's lag at speed (the arm's inertia and friction are not in the model). The integrator accumulates that residual, so the offset grows. This is a limit of the model at high speed, not a bug in the integrator; the stress run D exceeds what the model describes. It is not verified beyond these residuals.

## 4. Manual with the sag layer OFF, against the 2026-11-03 reference

Reference: `experiments/baselines/manual_reference_20261103.csv` (commit `e127479`, bridge on, gravity feedforward on, made before the sag layer existed). Current code, sag layer off (`run_sag_off.py` sets `SAG_LAYER_CFG["enabled"] = False`, the same way `command_bridge_test` does).

- 30 of 84 rows differ. **No difference before row 54 (6.9 s)**; from 6.9 s every joint differs at once.
- 6.9 s is the first bench key of step "3" (frame 816 = 6.94 s): the first time the source asks for targets beyond the joint buffer. That is the **buffer clamp (Stage 1)**, which the reference did not have.
- Largest differences: left joint 4 11.0° at 7.30 s (median 10.0°, the buffer margin 0.1745 rad); right joint 4 11.1° at 8.80 s; left joint 3 15.0° at 8.20 s (the earlier current-code manual run clamps joint 3 13 times, and joint 4 146 times). These are the clamp's effect (about the 10° buffer margin).
- Not the clamp: right joint 1 5.1° at 9.20 s (median 0.11°; the earlier current-code runs record no clamp on joint 1); left joint 1 0.78° at 7.40 s; left joint 7 0.10°. These begin at the same time as the clamp and are downstream of it (the later steps start from a different pose); the mechanism is not isolated here.

## 5. Manual and teleop with the sag layer ON (current code): offset peak per arm

| Scenario | Left peak (mm / mrad) | Left clip ticks (p / rot) | Right peak (mm / mrad) | Right clip ticks (p / rot) |
|---|---|---|---|---|
| Manual | 7.0 / 17.6 | 0 / 0 | 18.2 / 36.9 | 0 / 0 |
| Teleop | 40.0 (clip) / 100 (clip) | 153 / 144 | 40.0 (clip) / 100 (clip) | 85 / 48 |

Teleop saturates the offset on both arms, so the layer is a likely contributor to the teleop baseline differences reported earlier (a sag-off teleop run is not done here).

## 6. Verdict and questions for David (not decided, not built)

Stage 2 criteria (David's decisions 1–3):
1. Held-still bounded control: no growth, but the rotation amplitude (23 mrad) exceeds the 10 mrad bound written earlier. **Not passed as written.**
2. Motor-filter tests: (a) ramp PASS; (b) staircase FAIL with the test's emulator, PASS with the filter emulator; (c) reported. **Not passed as written.**
3. Policy bridge off: A PASS; B and C FAIL (offset clips); D FAIL (stress). **Not passed.**

So Stage 2 is not finished, and Stages 3–4 were not started. The held run, the staging tests and the wider regression were not rerun on the filter model, because criteria 1–3 do not pass.

Questions:
1. **Held-still bound.** The zero-mean control is bounded (no growth) at about 8 mm and 23 mrad. Is 10 mrad the right rotation bound, or should the check use growth only?
2. **Arm model in the synthetic tests.** Should `sag_solver_free_test` move the emulated arm with the same first-order motor filter the layer assumes? With it, the staircase, clamp and step checks pass; with the algebraic lag they fail (0.78 mm, 0.17 mm, 5.6 mm). Changing the emulator would change the test, so I have not done it.
3. **Windup at speed (policy B, C, D).** The filter underestimates the lag during fast motion (residual 20–130 mrad mean), and the offset winds up. Options: (a) accept that the layer does not correct at these speeds and set the criteria accordingly; (b) add a second motor term (not decided; would be a new model); (c) limit the integrator while the residual is large (not decided). Which, if any?
4. **Step check (0.5 mm).** This check was not in your list. It fails with the algebraic emulator (5.6 mm) and narrowly with the filter emulator (0.56 mm). Keep it, drop it, or change its criterion?

## Not run

- Stage 3 and 4 (Stage 2 not passed).
- Held run, staging tests and the full regression on the filter model (not needed while 1–3 fail; the sag-layer and feedback tests were run).
- Teleop with the sag layer off (to separate the layer from the sequence).
