# Uniform speed scaling vs per-joint speed limit during PLACE OVER (scratch experiment, no production change)

Written for: David (controller review). Scratch experiment only. No tracked source or configuration changed. Nothing pushed.

## Summary

- **Result: criterion 2.** Uniform scaling changes the within-OVER velocity event: the jump in target velocity drops from 0.26 to 0.11 rad/s per tick. But it does **not** reduce the bounce. Peak tilt is slightly higher (3.41° → 3.55°), the DOWN bump is larger (0.80° → 1.15°), settling is later (+312 → +328 ticks), and the sag integrator peaks higher (15.9 → 18.0 mm).
- **Mechanism (new):** the event is not the solver's velocity reversing. The solver's own unconstrained velocity is smooth through the window. The jump is the per-joint limit holding L3 at +0.5 rad/s for about 16 ticks while the unconstrained solution wants L3 near zero or negative, and then releasing it.
- **What uniform scaling does instead:** it removes the clamp, but the arm's unconstrained demand is about 2.5 rad/s on L3 at the start of OVER. The single factor min(1, 0.5 / max|v|) is therefore about 0.09 at the start, so the whole arm runs at roughly a tenth of its demanded speed for about 20 ticks, then recovers to 1.0. The arm lags the reference more during that time (mean pad-to-reference error in OVER 13.1 → 15.0 mm).
- Held-object compensation was kept on. The flag is global, so it also affects the pick, but the pick ends at nearly the same tick (CLOSE 10,999 vs 11,000) and the beaker tilt at the place start is nearly the same (0.222° vs 0.228°), so the place comparison is not materially confounded.

## 1. Phase 1: trace of the implementation

- **Setting:** `src/task_config.py:409` (`MINK_CFG["uniform_speed_scaling"]`, default False). The docs string says it was added on 2026-10-28 as an off-by-default option.
- **Applied in:** `src/mink_arm.py`.
  - Line 96: when the flag is False, `MinkArm.__init__` gives mink a `mink.VelocityLimit` over the 7 arm joints at `max_joint_speed` (0.5 rad/s). When True, the constraint is not created.
  - Lines 200–201, in `step()` after `mink.solve_ik`: when True, `va = va * min(1, max_joint_speed / max|va|)`, a single factor on all 7 joints. Direction is kept.
- **Execution path:** `experiments/demo_video.py` → the scripted episode → `make_scripted_manipulation(mink_cfg=MINK_CFG)` (`src/mpvi_modules.py:122`) → `MinkArm(…, mink_cfg)`. The flag is read from the shared dict at construction and at each step, so flipping it in-process reaches the solver. (`experiments/uniform_scaling_episode.py` already does this on 2026-10-28.)
- **Scope:** global. It affects every arm move (pick and place), not only place OVER.
- **Other per-joint clip:** `src/mpvi_modules.py:464` clips the phase-start ramp per joint at 0.5 rad/s. With speeds already at most 0.5 this clip does not bind, in either mode. It is not the source of the event.
- **Earlier measurement:** `docs/reports/20261028_1300_place_over_turn_panel_labels_pedestal_sweep.md` (item 1f) said the scaling "hardly acts" in PLACE OVER. That was before the 2026-10-29 changes (held-object compensation, M196; ramp M199). In the current code it acts strongly in the first 20 ticks of OVER, so the old statement does not apply to this code.

**The event (ticks 11,640–11,665, baseline).** Per-joint limit binding: 4 joints from 11,640; 3 from 11,654; 2 from 11,658; 0 from 11,660. Logged per tick:

| tick | L3 applied (limited) | L3 solver, no limit | L5 applied | L5 solver, no limit |
|---|---:|---:|---:|---:|
| 11,645 | 0.50 (clamped) | 0.15 | −0.08 | −0.05 |
| 11,650 | 0.50 (clamped) | −0.01 | −0.22 | 0.06 |
| 11,657 | 0.50 (clamped) | −0.13 | −0.23 | 0.30 |
| 11,658 | 0.31 | −0.15 | −0.07 | 0.34 |
| 11,659 | −0.09 | −0.16 | 0.30 | 0.36 |
| 11,661 | −0.07 | −0.07 | 0.32 | 0.32 |

The unconstrained solution (`vr_L*` columns, read-only second solve with the velocity limit left out) changes smoothly, by about 0.02 rad/s per tick. The discontinuity is where the clamped output stops being clamped, and the clamped and unconstrained solutions are far apart before that.

**Where it originates:** the constrained QP's active set. Mink's per-joint `VelocityLimit` holds L3 and L5 at the bound while the other joints absorb the Cartesian error. When the bound releases, the output snaps to the unconstrained value. The IK targets are unchanged, and the reference (`ref_*`) is smooth through the window.

## 2. Phase 2: the experiment

Launcher: `data/diag/uniform_speed_scaling_20261029/run_uniform.py <out> base|uniform`. The launcher runs `experiments/demo_video.py` `record_only` (the same path as the closed-loop recordings), with held-object compensation on.

- `base`: production code, unchanged (flag False).
- `uniform`: the same code with `MINK_CFG["uniform_speed_scaling"] = True` in this process only.
- Nothing else changed: gains, IK targets, ramp, velocity and acceleration limits, sag integrator, compensation, grip, and the place handover (no quintic change).
- Read-only logging per tick: the solver's returned joint velocity (`va`), the unconstrained second solve (`vr`, base only; uniform already has none), the applied scale factor (`scale`), the binding count and mask, the Cartesian reference, the pad pose, the sag integrator, and the phase labels.

## 3. Verification

- Baseline `data/diag/uniform_speed_scaling_20261029/base_20261029_161000` (13,614 ticks, rc 0): joint command `qcmd` **identical on all 13,614 ticks** to the recorded `data/closed_loop/on_20261029_1453`, despite the extra read-only solve. The extract replay is verified (13,613 ticks, no mismatch).
- Uniform run `uniform_20261029_161000` (13,559 ticks, rc 0). Extract replay verified (13,558 ticks).

## 4. Results

Metrics: `data/logs/uniform_compare_20261029_161000.log`, computed with `data/diag/quintic_20261029/compare_quintic.py` (the same definitions as the closed-loop report, validated on the old run).

| metric | base (per-joint limit) | uniform scaling |
|---|---:|---:|
| OVER / DOWN / OPEN tick | 11,602 / 11,853 / 12,157 | 11,603 / 11,854 / 12,160 |
| peak beaker tilt (OVER..OPEN) | 3.409° at +34 | 3.548° at +37 |
| oscillation onset (+0.1° over base) | +1 | +1 |
| DOWN bump | 0.801° | 1.148° |
| settling (last tick above 0.5°) | +312 | +328 |
| max target velocity change, 11,600–11,700 | 0.260 rad/s per tick (L3, 11,661) | 0.107 rad/s per tick (L3, 11,667) |
| max target velocity change, whole run | 0.411 rad/s (L4, 11,856, the handover) | 0.369 rad/s (L5, 9,918, in the pick) |
| max target jerk, whole run | 9,241 rad/s³ (L4, 11,855) | 8,302 rad/s³ (L5, 9,917, in the pick) |
| max measured jerk, OVER..OPEN | 407 rad/s³ (L7) | 565 rad/s³ (L7) |
| max measured joint speed L3/L7, OVER..OPEN | 0.40 rad/s | 0.36 rad/s |
| pad-to-reference error, OVER | max 20.1 mm (11,679), mean 13.1 mm | max 20.1 mm (11,645), mean 15.0 mm |
| tracking error at DOWN start | 4.9 mm | 4.7 mm |
| handover gap (pad to outgoing target) | 6.9 mm | 7.4 mm |
| sag integrator peak | 15.9 mm (at +147) | 18.0 mm (at +73) |
| uniform scale factor, OVER | not applicable | min 0.084; 59 ticks below 0.5 |
| ticks with a velocity limit binding, OVER..OPEN | 64 | 64 (but the uniform run binds no constraint) |
| minimum joint-limit slack | 0.0335 rad | 0.0343 rad |
| pick CLOSE tick / tilt at 11,598 | 10,999 / 0.222° | 11,000 / 0.228° |

**Reading the table:**

- The within-OVER jump is reduced by more than half (0.26 → 0.11 rad/s per tick), and the largest per-tick velocity change in the whole run is now in the pick, not in OVER. The jump is therefore changed, not gone: the applied velocity still changes by 0.11 rad/s per tick at tick 11,667 in the uniform run (the factor is still moving through 0.3 → 1.0 there).
- The whole-run target jerk at the place handover drops from 9,241 to 8,302, but that peak is in the pick in the uniform run, and the handover itself is not improved.
- Bounce: the peak tilt, DOWN bump, settling, sag peak and measured jerk all get worse.

## 5. Interpretation

- Uniform scaling works as designed: it keeps the direction of the joint velocity and removes the per-joint clamp. The clamp was holding L3 and L5 out of step with the solver's own demand, and the release was a discontinuity.
- The cost is that the arm is slowed by a factor of about 10 in the first 20 ticks of OVER, because one joint (L3) demands about 2.5 rad/s. The reference is not slowed, so the arm lags more and recovers later. That lag is what the tilt sees.
- So the discontinuity was not the cause of the bounce. Removing it (by scaling everything) makes the bounce slightly larger.

## 6. Limitations

- One run each. The baseline reproduces the recording exactly, so the comparison is deterministic for each code path, but there is no seed sweep.
- The flag is global, so the uniform run also changes the pick's solves. The pick ends one tick apart (10,999 vs 11,000) and the place-start tilt is nearly the same (0.222° vs 0.228°), so the confound is small, but it is not zero.
- The uniform factor is applied after the solve, so the solver itself is still computing a demand it cannot follow. A joint-space trajectory that keeps the demanded speed under 0.5 rad/s would address the cause; that is a reference or IK-target change, not this flag.
- The unconstrained second solve (base only) is an extra call to the solver with the same inputs. It does not change the simulation (verified: qcmd identical), but it is a diagnostic addition, not a model of the production path.

## 7. Recommended next experiment (not implemented)

1. **Find why L3 demands ~2.5 rad/s at the start of OVER.** That is 5× the speed limit at the moment the place starts. The candidates: the place OVER's Cartesian reference starts from a pose where the arm is not at the reference (the reference is in the place module, not in the solver), or the posture task. A read-only check of `ref` vs `pad` at tick 11,602 and the posture cost would show which.
2. Only after (1): a test of a joint-space or reference-side change that keeps the demanded speed under the limit. That is a trajectory decision for David, so it needs approval first.
3. Do **not** adopt uniform scaling on this evidence: it changes the event but makes the bounce slightly worse.

## 8. Files

Created (scratch, git-ignored under `data/`):

- `data/diag/uniform_speed_scaling_20261029/run_uniform.py`: launcher (`base` or `uniform`).
- `data/diag/uniform_speed_scaling_20261029/base_20261029_161000/`: baseline recording and `instr_ticks.csv`.
- `data/diag/uniform_speed_scaling_20261029/uniform_20261029_161000/`: uniform run.
- `data/diag/uniform_speed_scaling_20261029/base_ticks.csv`, `uniform_ticks.csv`: per-tick extracts.
- Logs: `data/logs/uniform_base_20261029_161000.log`, `data/logs/uniform_basex_20261029_161000.log`, `data/logs/uniform_uni_20261029_161000.log`, `data/logs/uniform_unix_20261029_161000.log`, `data/logs/uniform_compare_20261029_161000.log`.
- This report, and `docs/progress.md` (committed locally, not pushed).

Tracked files: `git status` shows only the pre-existing untracked `deps/AgileX_Tracer`. No change to `src/`, `experiments/`, `models/` or `src/task_config.py`.

Commands run (venv, `MUJOCO_GL=egl`, through `experiments/run_capped.sh`, one at a time in tmux): `run_uniform.py … base`; `experiments/bounce_diag_extract.py` on the base recording; `run_uniform.py … uniform`; `experiments/bounce_diag_extract.py` on the uniform recording; `compare_quintic.py` on both pairs; plus read-only pandas checks of the event window.
