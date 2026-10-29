# Start-mismatch fix at PLACE OVER: the solver's start residual goes to zero, but the bounce gets slightly worse (scratch experiment, no production change)

Written for: David (controller review). Scratch variant only. No tracked source or configuration changed. Nothing pushed.

## Summary

- **Cause (Task 1).** The 18.5 mm, 3.6° offset at OVER is not created at the place start. It is a static servo error that exists during the pick's carry (command minus measured on L1 is −0.0137 rad at tick 11,590, before the held step). The place skill's `arm.reset` snaps the solver's configuration to the measured joints, but the phase-start ramp (M195) continues from the old command. The command therefore stays where it was (L1 at −0.37636 rad from 11,600 to 11,604) while the measured arm closes the gap slowly. The reference starts at the measured pad, so the solver sees the offset as a residual and demands about 1.6 rad/s on L3, which the ramp caps.
- **Proposed fix (Task 2).** Keep the reference at the measured pad. At the first move of a place skill, start the sag correction at the offset of the command the ramp continues from: C = FK(q_prev) − FK(q_meas). The first target then equals the command already being followed.
- **Result.** The mismatch is removed as the solver sees it: the residual against its own target is 0.00 mm at 11,602 (base 18.45 mm). The L3 unconstrained demand drops from 1.64 rad/s to −0.07 rad/s. The commanded joints do not jump.
- **But the bounce does not improve, and it gets slightly worse:** peak tilt 3.41° → 3.63°, DOWN bump 0.80° → 0.96°, settling +312 → +322 ticks, measured jerk (L7) 407 → 804 rad/s³. The early tilt (+0.09° at 11,602, +0.62° at 11,608, +1.56° at 11,616) is identical in both runs. The difference appears later.
- **Recommendation:** do not adopt this fix. Retain the result and investigate the remaining cause separately (section 6).

## 1. Trace of the offset (Task 1)

Replay of the production recording (`data/diag/l3_demand_20261029/dump_20261029_170000/recording`) with `experiments/bounce_diag_extract.py`. The extract matches the earlier production extract on all 13,612 ticks (q, command, servo force identical).

| tick | L1 command − measured (rad) | L1 command (rad) | L1 measured (rad) | L1 servo force − feedforward (N m) |
|---|---:|---:|---:|---:|
| 11,590 | −0.0138 | −0.37638 | −0.36264 | −0.97 |
| 11,599 | −0.0137 | −0.37638 | −0.36264 | −0.95 |
| 11,600 (NAV) | −0.0137 | −0.37636 | −0.36268 | −0.92 |
| 11,601 (PLACE starts, snap) | −0.0136 | −0.37636 | −0.36278 | −0.88 |
| 11,602 (OVER) | −0.0134 | −0.37636 | −0.36294 | −0.85 |
| 11,616 | −0.0058 | … | … | −0.11 |

Reading:
- **The offset predates the place.** It is already −0.0137 rad at 11,590, during the pick's carry, and it does not grow at the held step.
- **The snap is undone on the next tick.** `arm.reset` (the place skill's start) sets the solver's configuration to the measured joints, but `step()` then sets the command to `q_prev + ramp-scaled step`, and the ramp scale is about 1e-5 on that tick. From 11,601 the solver starts from a command that is still the old one.
- **The measured arm closes the gap, not the command.** The command is held at the old value while the measured arm moves toward it slowly (about 0.02 rad/s). The solver's demand for the 1.8 cm correction cannot move the command, because the ramp limits it.
- **Held-object compensation did not change the balance.** The servo's residual torque on L1 is −0.97 N m before the compensation and −0.92 N m after it. The offset comes from a steady servo error under the beaker (feedforward model error or joint friction). It is not the place's trajectory.

## 2. Proposed fix and why it should avoid the restart failure

Change (scratch only, place skills only, first move only; `data/diag/offset_20261029/mpvi_modules_reconcile.py`, three edits):
- `start()` keeps the command the ramp continues from (`q_start_cmd`) for a place skill, before the snap.
- At the first tick of the first move, the sag correction starts at `C_p = FK(q_start_cmd) − FK(q_meas)` and `C_r = rotvec(R_cmd R_meas^T)`, clipped to the existing limits (18 mm and 0.063 rad are within 40 mm and 0.1 rad).
- The reference is still anchored at the measured pad. The target at tick 0 is then the commanded pad.

What is unchanged: skill-start snap, ramp, minimum-jerk timing, gains, IK costs, velocity limits, gravity compensation, the sag law itself, the pick (the first changed tick is 11,602).

Why it should avoid the restart failure (hypothesis, tested here only in part). The restart test (`docs/reports/20261028_0237_restart_from_commanded.md`) anchored the reference at the commanded pad and started the sag integrator at zero, so `p_ref − p_now` began at about 1 cm and the integrator wound up. This fix starts the integrator at its true offset, so its error is zero at tick 0. The test shows the integrator's start was not the cause of a bounce increase here: the bounce got worse anyway. The restart test also changed the skill start, so it cannot be attributed to one part.

## 3. Trade-offs

| start | reference | command at tick 0 | solver residual at start | what the sag integrator sees | this test |
|---|---|---|---|---|---|
| measured (baseline) | measured pad | moves 1.8 cm during the ramp | 18.5 mm (L3 demand 1.6 rad/s, ramp-capped) | zero error | bounce 3.41°, settling +312 |
| commanded (restart, 2026-10-28) | commanded pad | unchanged | zero | 1 cm error from tick 1 | measured worse (14 cm swing, 21°) |
| reconciled (this test) | measured pad | unchanged | zero | zero error, starts at true offset | residual zero; bounce 3.63°, settling +322 |

The reconciled start removes the solver's demand but does not remove the offset itself. The offset is carried in the sag correction, and it is still there until the servo closes it.

## 4. Results

Launchers: `data/diag/offset_20261029/run_reconcile.py` (base and reconcile modes, read-only logging). Base run reproduces the production command on all 13,614 ticks (0 differences, verified against `data/diag/quintic_20261029/base_20261029_152524`). Metrics: `data/diag/offset_20261029/compare_recon.py` and `data/diag/quintic_20261029/compare_quintic.py` (validated on the old run). Logs: `data/logs/recon_20261029_173343_*.log`, `data/logs/recon_20261029_173343_compare.log`.

Start mismatch (solver residual against its own target, `cmd` = reference + sag):

| tick | base | reconcile |
|---|---:|---:|
| 11,602 | 18.45 mm | 0.00 mm |
| 11,608 | 18.55 mm | 0.15 mm |
| 11,616 | 18.64 mm | 0.92 mm |

Note: the logged "commanded pad vs reference" column (`cmis_mm`) does not include the sag term, so it reads 18.5 mm in both runs. Use the residual above.

Orientation residual vs target was not computed in this run (only the commanded-pad orientation against the reference, which also ignores the sag term). This is a gap in the report.

L3 velocity demand (rad/s): unconstrained (`vr`) at 11,602: base 1.64, reconcile −0.07. At 11,616: base 1.65, reconcile 0.06. Limited (`va`) at 11,602: base 0.50 (clamped), reconcile −0.005.

Joint tracking and torque around 11,599–11,616 (L1 command − measured, rad; L1 servo force − feedforward, N m): at 11,599, −0.0137 / −0.95 in both runs. At 11,616: base −0.0058 / −0.11, reconcile −0.0059 / −0.12. The offset decays at the same rate in both runs, because the measured arm closes it and the command does not move.

Command continuity: the largest one-tick commanded joint step between 11,600 and 11,700 is 0.00268 rad (base, at 11,660) and 0.00208 rad (reconcile, at 11,700). The largest commanded speed between 11,601 and 11,616 is 0.017 rad/s in both. No jump is introduced.

Tilt and bounce:

| metric | base | reconcile |
|---|---:|---:|
| tilt rise at 11,602 / 11,608 / 11,616 (deg) | +0.09 / +0.62 / +1.56 | +0.09 / +0.62 / +1.56 |
| peak tilt (OVER..OPEN) | 3.409° at +34 | 3.625° |
| oscillation onset | +1 | +1 |
| DOWN bump | 0.801° | 0.964° |
| settling (last tick above 0.5°) | +312 | +322 |
| max target velocity change, whole run | 0.411 rad/s per tick | 0.399 |
| max target jerk, whole run | 9,241 rad/s³ | 8,967 |
| max measured jerk, OVER..OPEN (L7) | 407 rad/s³ | 804 |
| tracking error at DOWN start | 4.87 mm | 4.77 mm |
| max tracking error, OVER..OPEN | 20.1 mm | 20.3 mm |
| handover gap | 6.87 mm | 6.91 mm |

The sag-peak metric is not comparable: in the reconcile run it picks up the initial offset at tick 0 (18.5 mm), so it is not a peak of the integrator.

Path deviation: the maximum tracking error (pad to reference) is 20.1 mm vs 20.3 mm, so the path is essentially the same.

## 5. Interpretation

- The residual that the solver saw is gone, and the commanded joints stay continuous. The fix does what it is meant to do at the level of the solver.
- The bounce does not follow. The baseline's solver-driven move of the command by about 1.8 cm over the ramp appears to help the tilt settle sooner. Holding the command still (reconcile) leaves the measured arm to close its 1.8 cm gap at its own slow rate. Settling and the DOWN bump both get longer.
- So the solver's demand was not the cause of the bounce, and removing the demand does not reduce it. The bounce depends on how the 1.8 cm servo offset closes, which the trajectory does not control in this variant.

## 6. Recommendation (not implemented; needs David's decision)

1. **Do not adopt the reconciled start.** It removes the solver's demand but makes the bounce slightly worse (peak 3.41° → 3.63°, settling +312 → +322).
2. **The next question is the servo's steady error under the beaker.** It is 1.4 cm on the pad, present before the place, and the held-object compensation did not change its L1 balance. Measure the steady error with and without the compensation, and the joint friction (stiction) at rest. The candidate fixes are the feedforward model or the held term, which are outside the trajectory and change controller behaviour, so they are David's decision.
3. **Restart_from_commanded:** the failure in 2026-10-28 is not separated from its skill-start change. A clean test would be the move-start part alone with the sag at its true offset, but this variant already shows that a zero-residual start does not reduce the bounce.

## 7. Limitations

- One run per variant. The base run reproduces production exactly, so the comparison is deterministic per code path.
- Place skills only. The pick's first move has the same snap-and-ramp structure and was not changed or tested.
- The orientation residual was not computed against the target (section 4).
- The first chain attempt was killed by the 6 GB memory cap (an extra per-tick `MjData` in the launcher). Those partial outputs (`data/diag/offset_20261029/base_20261029_172732`) are not used. The rerun with one reused `MjData` (`*_173343`) is the one reported here.
- The replay (`bounce_diag_extract`) of the reconcile run has 13,608 ticks, the base 13,613; the reconcile episode is 5 ticks shorter. The comparison uses the ticks that exist in both.

## 8. Files

Scratch (git-ignored, `data/diag/offset_20261029/`):
- `mpvi_modules_reconcile.py`: the variant (three edits, place skills only).
- `run_reconcile.py`: launcher (base and reconcile).
- `chain.sh`: the run chain.
- `compare_recon.py`: the start-mismatch and continuity comparison.
- `base_20261029_173343/`, `reconcile_20261029_173343/`: recordings and `instr_ticks.csv`.
- `base_ticks_20261029_173343.csv`, `reconcile_ticks_20261029_173343.csv`: per-tick extracts.
- `base_ticks.csv`: extract of the L3 dump recording (trace, section 1).

Logs: `data/logs/offset_extract_20261029_172356.log`, `data/logs/recon_20261029_173343_*.log`, `data/logs/recon_20261029_173343_compare.log`.

Tracked files: no change to `src/`, `experiments/`, `models/` or `src/task_config.py`.
