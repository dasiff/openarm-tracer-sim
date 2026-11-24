# Staged rebuild, stage 2: re-run after the speed-term fix (2026-11-24, David's d0bc brief)

Branch `safety-filter`. **Stage 2 STOPPED: its bridge-on speed-term check fails** (the offset diverges through the speed term when the arm is held still and the bridge is on). Stages 3 and 4 were not started. Nothing was tuned. Not pushed.

Commits this round: `f23a9f5` (speed term from references), `6a79289` (harness, speed-term test, hand-error reference). Data: `data/logs/stage2d/` (regression `regress/`, held run `held/`, staging `staging/`, policy `policy/`, baselines `base/`, policy B timing `policyB_timing/`); diagnostics in `data/logs/stage2d_diag/`.

## 1. Speed term (David's decision 1)

Implemented as specified: `refspeed = (REF(t-1) - REF(t-2)) / (t1 - t2)`, REF = bridge output minus the layer's own dq. The raw-output history was removed.

| Check (`sag_solver_free_test`, speed term block) | Result |
|---|---|
| Bridge off, arm held still, command changing: refspeed equals the source's own speed (no dq in it), dq not zero | **PASS**: max error 1.8e-14 rad/s over 598 updates; max |dq| 0.069 rad |
| Bridge emulated, arm held still: offset stays within 10 mm and 10 mrad, no growth late in the run | **FAIL**: |C_p| reaches 29.7 mm, |C_r| 86.6 mrad, still growing |

**Why the bridge-on check fails.** With the bridge on, the bridge rate-limits the command, and the command includes `dq`. So the bridge output lags `dq`, and `REF = output − dq` keeps the difference between the current `dq` and its lagged version. The speed term therefore still contains changes in `dq` (only in the bridge's lag, not in the source). With the arm held still, that feeds the integrator and it diverges. The bridge-off path has no such lag, so it passes. This is a design question for David (Question 1); the bound was not changed.

Test note: an earlier version of the bridge-off check compared against the source speed one tick late (an error in my test, 1.1e-2 rad/s). Fixed in the test, not in the layer.

## 2. Harness (David's decision 2): changes listed

| File | Change |
|---|---|
| `experiments/sag_layer_test.py` | new helper `_cc_cmd` (correct, then observe each arm the command carries; the bridge output as the loop feeds it) |
| | `case_c_periodic` now calls `_cc` (the bridge output fed) |
| | `_moving`, `_standing` and the standing loop of `case_s5_a` now call `_cc_cmd` |
| | `case_s5_d` now calls `_cc_cmd` for both tags |
| | case (c) expects `n_ticks - 2` integrated ticks: the first two ticks have no reference (the designed `no_reference` skip) |
| | Helper bug fixed in this round: `_cc_cmd` looked up `right` when the command carries only `left` (the first re-run of the test crashed at `case_s5_a`; the earlier "EXIT 0" was the wrapper's exit code, not Python's) |
| `experiments/sag_solver_free_test.py` | new speed-term block (2 checks, described above); constant `LEAK_DEFAULT` |
| Retirements | none: the checks test behaviour that still exists (no periodic reset, standing command leaks, source gate absent) |

`sag_layer_test` **16 of 16 PASS** (rerun, `regress/sag_layer_test_rerun.log`).

## 3. Policy investigation (David's decision 3), before trusting the variants

### 3a. Why the offset is identical in every policy run

Per phase, from the per-tick skip counters (`*_diag.json`, every run):

| Phase | Applied | Skipped (reason) |
|---|---|---|
| Setup move (1551 ticks) | 1547 | 4, `no_reference` (the first two ticks, both arms) |
| Stream (2972 ticks) | 2972 | 0 |

No skip rule fires in the stream. The offset is updated on every stream tick in every run. The reason the offset is identical is that **the arm does not move in the stream in any variant**: the bridge's step rule rejects each stream target (`rejected: target step too large`, more than 0.25 rad from the command: 1 time in A, 12–23 in B and C, 123–136 in D, with the event count reaching ~1470 for one joint in A). The bridge then holds its last accepted target, the arm stays at the same pose, and the layer integrates the same error in every run. Check: the accepted reference (REF) moves less than 0.1° over the stream for A and B; the source's own target in B does move (range up to 171°), but the bridge does not accept it.

So the policy variants, as the bridge runs them now, do not move the arm after setup, and cannot test the layer under motion. That is the step-rule rejection, which Stage 3's shrink rule replaces.

### 3b. What the hand error is measured against

The hand error was measured against the **source's raw target**. That target is the one the bridge rejects, so the arm never reaches it: the raw target stays 0.4–0.9 m from the accepted pose (the "raw-target gap" column). Fixed: the hand error is now measured against the **bridge-accepted reference** (REF = bridge output minus dq, i.e. the source target after the bridge's step and clamp rules). The raw-target gap is kept in the summary as its own number. The layer was not changed.

After the fix (24 runs, correction on / off):
- hand error on: 0.1 mm; off: 1.2 mm (mean over the 30 s window; the gap between the two is the offset's effect).

## 4. Policy variants A–D, 3 seeds, correction on and off (24 runs, bridge on)

All 24 exit 0. Full table: `data/logs/stage2d/policy_table.md`. Summary (correction on; the offset is identical in every run):

| Variant | Offset peak (left, mm) | Final C_p (mm) | Clip ticks | Stream skips | Rejections (seeds 1/2/3) | Hand err on / off (mm) |
|---|---|---|---|---|---|---|
| A | 1.60 | [0.22, 0.89, 0.22] | 0 | 0 | 1 / 1 / 1 | 0.1 / 1.2 |
| B | 1.60 | same | 0 | 0 | 18 / 17 / 23 | 0.1 / 1.2 |
| C | 1.60 | same | 0 | 0 | 15 / 12 / 20 | 0.1 / 1.2 |
| D | 1.60 | same | 0 | 0 | 136 / 133 / 123 | 0.1 / 1.2 |

The "A offset matches rest droop" check is not evaluated: the policy test prints no droop value to compare with.

## 5. Pass criteria

| Criterion | Result |
|---|---|
| Speed-term check, bridge off (new) | PASS |
| Speed-term check, bridge emulated, arm held still (new) | **FAIL** (30 mm, 87 mrad, growing) |
| `sag_layer_test` | PASS 16 of 16 (rerun) |
| `sag_feedback_test`, `bridge_clamp_test`, `command_bridge_test` (sag off), `sag_feedforward_check` | exit 0 |
| `sag_solver_free_test` | 12 of 13 PASS (the bridge-on speed check fails) |
| 18 safety-filter entries | all exit 0 |
| Held run (full, seed 4294874296, `--liquid 0.100`) SUCCESS | **PASS**: `outcome SUCCESS; plan failure None; status ['done'×4]` |
| Staging: `home_pose_test`, `grasp_lift_test`, `beaker_hotplate_correction_test`, `nav_follow_sim_test --only on` | all exit 0 (reach home) |
| Policy A–D, 3 seeds, correction on and off (24 runs) | all exit 0; numbers above; the stream does not move the arm (3a) |
| Timing: sag + bridge p99 sum ≤ 6.7 ms | PASS (below) |

Held run learned offset: left peak 6.53 mm and 27.5 mrad, right 6.53 mm and 18.2 mrad; no clip tick; skips 4 no-reference ticks per arm at the start.

Timing (per call):

| Run | Sag layer mean / p99 (ms) | Bridge mean / p99 (ms) | p99 sum (ms) |
|---|---|---|---|
| Held run (full) | 0.290 / 1.034 | 0.051 / 0.429 | 1.46 |
| Policy B seed 1, bridge on | 0.245 / 0.755 | 0.041 / 0.283 | 1.04 |

## 6. Baselines (current code), against Stage 1 fix `5af2083` and HEAD before Stage 1 `1f29da7`

Largest joint difference (degrees, joint, time) and largest hand (pad) position difference (mm, arm, time; from the joints in the task scene, base movement below a micrometre):

| Scenario | Rows changed (vs stage 1) | Largest joint vs stage 1 | Largest pad vs stage 1 | Rows changed (vs HEAD before stage 1) | Largest joint vs HEAD before stage 1 | Largest pad vs HEAD before stage 1 |
|---|---|---|---|---|---|---|
| Manual (84) | 30 | left joint 3, 17.41° at 8.80 s | left 98.8 mm (8.30 s); right 158.2 mm (9.10 s) | 30 | right joint 4, 11.06° at 8.80 s | left 85.3 mm (7.30 s); right 90.6 mm (8.80 s) |
| Teleop (78) | 58 | right joint 5, 26.08° at 9.20 s | left 141.0 mm (4.30 s); right 254.2 mm (5.70 s) | 59 | left joint 1, 17.21° at 8.60 s | left 107.4 mm (4.50 s); right 301.0 mm (5.70 s) |
| Policy (84) | 36 | left joint 2, 4.01° at 6.90 s | left 69.4 mm (7.00 s); right 0.008 mm | 36 | left joint 4, 10.95° at 6.70 s | left 85.6 mm (6.70 s); right 0.002 mm |

Clamp counts per joint (joints 1–7; target clamps, and updates clamped with seconds = updates × 0.02 s):

| Run | Left targets | Left updates | Left seconds | Right targets | Right updates | Right seconds |
|---|---|---|---|---|---|---|
| Manual | 0, 133, 13, 146, 0, 0, 0 | 0, 46, 6, 50, 0, 0, 0 | 0, 0.92, 0.12, 1.00, 0, 0, 0 | 0, 97, 0, 97, 0, 0, 0 | 0, 33, 0, 33, 0, 0, 0 | 0, 0.66, 0, 0.66, 0, 0, 0 |
| Teleop | 0, 34, 2, 386, 0, 0, 0 | 0, 12, 1, 130, 0, 0, 0 | 0, 0.24, 0.02, 2.60, 0, 0, 0 | 0, 277, 0, 301, 0, 0, 0 | 0, 93, 0, 101, 0, 0, 0 | 0, 1.86, 0, 2.02, 0, 0, 0 |
| Policy | 0, 48, 7, 73, 0, 0, 0 | 0, 17, 3, 25, 0, 0, 0 | 0, 0.34, 0.06, 0.50, 0, 0, 0 | 0, 0, 0, 1272, 0, 0, 0 | 0, 0, 0, 408, 0, 0, 0 | 0, 0, 0, 8.16, 0, 0, 0 |
| Held run | 0, 49, 0, 342, 0, 1085, 0 | 0, 17, 0, 115, 0, 364, 0 | 0, 0.34, 0, 2.30, 0, 7.28, 0 | 0, 57, 0, 342, 0, 0, 0 | 0, 20, 0, 115, 0, 0, 0 | 0, 0.40, 0, 2.30, 0, 0, 0 |

Note: the baseline numbers changed from the previous round because the speed term changed (the layer is in the loop in these scenarios).

## 7. Questions for David (not decided, not built)

1. **Speed term with the bridge on (the failing check).** Decision 1 as written leaves `dq` in REF through the bridge's lag, so the offset diverges with the arm held still. Options: (a) define the reference from the source side: run the bridge's acceptance rules on the source targets separately, so REF contains no `dq` at all (more state in the bridge); (b) keep REF = output minus dq and accept that the offset is not bounded when the arm does not follow (the real loop has not shown divergence: the held run peak is 6.5 mm); (c) something else. Stage 2 cannot pass until this is decided.
2. **Policy stream.** Every stream target is rejected by the step rule, so the policy variants do not move the arm. Stage 3 (shrink instead of reject) changes this. Confirm that Stage 3 is the next step, after Question 1.
3. **Hand error reference.** Confirm the accepted-target reference (REF) is the intended reference for the hand error (the measurement is now against the bridge-accepted target).

## Not run

- Stages 3 and 4 (Stage 2 stopped).
- The full `run_tests.sh` suite (the sag, bridge and safety-filter entries only).
- Policy runs with bridge off.
