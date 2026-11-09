# Sag layer (M262): why it does not reproduce the old trim, and why IN stalls

Read-only diagnosis from the logs and the code. No code changed, no run made, no baseline touched. Written for David.

Runs compared: the registered baseline `experiments/baselines/held_200g_20261104_m258` (old in-script trim, commit e50815d), and the scripted run with the layer at 34b69be (`scratchpad/run_200g_scripted_f/`, log `data/logs/held200_scripted_f_20261105_042504.log`). The mock run at 34b69be (`scratchpad/run_200g_mock_f/`) has no labels and no C values in its tick log, so the mock questions are only partly answered (see Q6).

Correction to the earlier report: the baseline does carry the C columns (non-zero from tick 9689). The earlier comparison dropped them by mistake. The first difference is in the C columns at tick 9687 (the layer starts, the baseline is 0.0 there), and in the measured columns at tick 9691.

## Summary of the finding

The layer's correction is not the old trim with a different reference. Two things differ in how it runs, and they are the main causes:

1. **The correction is reset every 1.0 s.** The layer's open watch starts on every gripper-open command (control_loop.py:661-662). The gripper is commanded open for the whole pick, so the watch starts, the fingers are at rest, and `track_open` resets C and the rotation correction after `close_min_s` = 1.0 s (sag_layer.py:124-125). The logged resets are at ticks 9837, 9987, 10137 ... 17037: 49 resets, every 150 ticks, exactly 1.0 s. The old code reset only at the script's own OPEN step (place release) and at recovery start. The baseline has one reset in the whole run (tick 13010, place BACK).
2. **The reference is the script's solver output, not the planned point.** The layer takes `p_ref` as the forward kinematics of the script's joint command (sag_layer.py:92). That command is one solver step from the measured joints, so it is off the planned point: by about −1.0 mm in z at the start of OVER, and by about +2.1 mm in z on average during IN. The old code integrated the planned point directly, so its reference error was zero by construction.

The layer also trims during the recovery's joint motion (recover BACK and HOME), where the old code never trimmed (the old trim was inside the `move` branch only, mpvi_modules line 368). That is a third behaviour difference.

## Q1. The reference, and when it is taken

Quoted lines (src/sag_layer.py, `SagLayer.correct`):

```
92  p_ref, R_ref = self._pose(data.qpos, q_src)                                          # the reference: the hand pose the targets imply
93  p_now, R_now = self.arm.pad_pose(data)                                               # the measured hand pose
94  _sag_trim_step(self.C, PICK_PLACE_CFG, self.model, self.side, t, p_ref - p_now,
95                 Rotation.from_matrix(R_ref @ R_now.T).as_rotvec(), dt)
96  R_t = (Rotation.from_rotvec(self.C["r"]) * Rotation.from_matrix(R_ref)).as_matrix()
...
102     self.arm.step(data, p_ref + self.C["p"], R_t)                                    # the existing solver, one step toward the corrected pose
```

- `_pose` (lines 76-81) is forward kinematics on a scratch copy of `data.qpos` with the arm's joints replaced by `q_src`. `q_src` is the script's `arm.q_cmd` (mpvi_modules.py:319, `arm_targets`), not the planned point.
- The reference is taken **before** the layer adds its correction. C is added only at line 102, to build the target. The layer never compares the measured pad with a reference that already contains C **within a tick**.
- Indirect contamination: the script's own solver warm-starts from `arm.reset(data)` (measured joints) at each skill start. It reads `data.ctrl` only when `restart_from_commanded` is True (mpvi_modules.py:275-276). That config is **False** (task_config.py:426), so the script's solver never sees C. **Latent (NOT BUILT, needs decision):** if that flag is set to True, the script's reference would start from the ctrl that already contains C, and the removed subtraction (old mpvi_modules.py:330, `p0 = p0 - C["p"]`) would be needed to avoid double counting. The subtraction is gone and nothing puts it back.
- Timing: sag_correct runs at control_loop.py:1130, before `bridge_take` (1131), once per arm command. Both runs integrate one nominal step per tick (dt = 0.006667 s, from the t_sim column). Timing is not the cause.

## Q2. First divergence in pick OVER: values, references, and the cause

Method: each tick the layer adds `gain * dt * (p_ref - p_now)` to C (gain 3.0, dt 0.006667 s). So the error the integrator saw is `(C[k+1] - C[k]) / (3.0 * dt)`. Checked on the old run: this matches the logged `ref_z_m - pad_z_m` to within 0.04 mm at every tested tick (9688, 9700, 9750, 9800, 9850, 9900, 9914), so the method holds.

Table, ticks 9687 to 9697 (mm; C_p is x, y, z):

| tick | old C_p (x, y, z) | new C_p (x, y, z) | old ref_z - pad_z | new implied error z | new ref_z - pad_z |
|---|---|---|---|---|---|
| 9687 | 0.00, 0.00, 0.00 | −0.01, 0.01, −0.02 | 0.00 | −1.00 | 0.00 |
| 9690 | 0.00, −0.00, 0.00 | −0.03, 0.02, −0.06 | 0.00 | −1.00 | 0.00 |
| 9695 | 0.00, −0.00, 0.00 | −0.10, 0.05, −0.18 | 0.00 | −0.97 | 0.00 |
| 9697 | 0.00, −0.00, 0.00 | −0.12, 0.06, −0.22 | 0.00 | — | 0.00 |

The new reference offset (implied minus planned, z) over OVER:

| tick | 9687 | 9690 | 9695 | 9700 | 9710 | 9720 | 9740 |
|---|---|---|---|---|---|---|---|
| offset z (mm) | −1.00 | −1.00 | −0.97 | −0.85 | −0.36 | −0.07 | −0.07 |

It is back under 0.1 mm from tick 9718 on. The x and y reference offsets cannot be read back: `ref_x` and `ref_y` are not logged. From the first tick's C alone, if the first step was the only one, the offset is about (x −0.55, y +0.28, z −1.00) mm. That is inferred, not measured.

Old and new at the start of the OVER step (tick 9687): the planned point equals the measured pad (1139.889 mm in both), so the old reference error is zero and old C stays 0. The new reference is about 1 mm below the planned point, so C starts integrating. That is the first divergence, and it is the **reference choice**.

Timing and solver are not the cause at 9687: both runs step once per tick, and the first solver step from measured joints gives the −1 mm offset (the solver residual, see Q3). The reset is not involved at 9687; the first reset is at 9837.

C at the DOWN start (the next label change) shows the cumulative effect:

| step start | old C_p (x, y, z) mm | new C_p (x, y, z) mm |
|---|---|---|
| pick DOWN (tick 9916) | −2.96, −1.66, +1.31 | −6.47, −3.12, −0.70 |

Between 9687 and 9836 the new C grows to (−2.18, −9.45, −4.49) mm, with C_r (0.028, 0.045, −0.077) rad. Then the reset at 9837 drops it to (−0.08, −0.08, −0.04). At the same tick the old run has C_p (−2.62, −4.30, −2.72) mm, and it never resets, so its y correction keeps building to −1.70 mm at the end of OVER (tick 9915). The new run's y correction is about twice the old one at 9836 and is then cut by the reset. The y and rotation parts cannot be split further because the reference's y is not logged.

Orientation: the logged `rot_err_deg` is the planned orientation against the measured one, not the solver's. It is 3.3° in the new run at tick 9836 and 0.7° in the old run; the new value is high because the rotation correction is reset at 9837 and the orientation is left at 3.3°.

## Q3. The solver residual (FK of the solver output minus the planned point)

Logged data give this only for z (`ref_z_m` is the planned z; `ref_x_m` and `ref_y_m` are not logged), and only for the new run. The old run's residual is zero by construction. Values exclude the reset ticks.

| step | ticks | mean (FK of solver output − planned), z, mm | max \|offset\|, mm |
|---|---|---|---|
| pick OVER | 449 | −0.07 | 1.21 |
| pick DOWN | 1104 | +0.34 | 1.94 |
| pick IN | 2246 | **+2.15** | 2.34 |
| recover BACK | 224 | +1.00 | 3.39 |
| recover HOME | 733 | +0.49 | 1.27 |

Interpretation: during IN the solver's output sits about 2 mm above the planned z, and the layer integrates toward that point, not toward the planned path. The end-of-step miss rises for the same reason: measured-to-target error at the end of each step (err_m, the planned target, which is measured against the pad):

| step end | old err (mm) | new err (mm) |
|---|---|---|
| OVER | 2.41 | 2.59 |
| DOWN | 0.40 | 3.46 |
| IN | 1.29 | 3.31 |
| BACK | 3.11 | 4.78 |

The divergence in end-of-step miss appears in DOWN, consistent with the reference offset building during OVER and carrying into DOWN.

The x and y residuals and the orientation residual of the solver cannot be read from the logs. A small run that logs `q_src` (the script's arm.q_cmd, 7 values) and its FK per tick would give them.

## Q4. The removed per-skill reset, and the carry into the second OVER

What the old code did: the recover start zeroed C (old mpvi_modules.py:292). The per-skill-start reset was a switch (`sag_reset_at_skill_start`) that was off. The new layer has neither. In practice the periodic reset (Q7) now zeroes C every second, so C can carry only from the last reset.

Logged values at the second attempt:

| tick | label | C_p (x, y, z) mm |
|---|---|---|
| 13107 | recover HOME | (9.36, 6.48, −6.21) |
| 14026 | pick OVER (start of attempt 2) | (4.69, 1.58, −3.42) |

Calculation (labelled as a calculation, not a measurement): the carried C at 14026 is |C| = √(4.69² + 1.58² + 3.42²) ≈ 6.0 mm. It was built since the last periodic reset at tick 13887, a window of 139 ticks (0.93 s). At the layer's gain (3.0 per s) the carried correction decays only as the integrator drives it, so the start offset of attempt 2 is at most about 6 mm for at most about 1 s.

Does it account for the IN failure? No. Attempt 1 starts with C ≈ 0 at OVER and fails IN at 3.3 mm. Attempt 2 starts with a 6 mm carry and fails at 3.3 mm. The carry therefore accounts for about 0.0 mm of the IN miss difference between attempts. The IN failure is present without the carry.

The larger new behaviour is the trim during the recovery's joint motion: C grows to 9.3, 6.5, −6.2 mm during recover BACK and HOME, which the old code never trimmed. Recover HOME is a `joints` step (mpvi_modules.py:312), and the layer integrates the lag of the joint motion as if it were sag.

## Q5. The zero-time-step fix (34b69be)

Before: a second arm command in the same tick gave `dt = 0`, the solver divided by zero, the NaN raised, and the layer returned the source's targets (inert). That was the d1 defect; the d1 runs had no effect of the layer.

After (sag_layer.py:89-90): a second command in the same tick returns the layer's last output without integrating and without calling the solver.

Logged evidence: in the f run the C changes by exactly one nominal step per tick through the move steps. Over 5,000 move ticks only one tick had C unchanged from the tick before (checked by exact equality on C_p x and z). So the guard can have fired on at most a few ticks, and the logs cannot count it. The guard is not a plausible cause of the IN failure. A counter would answer exactly how often it fires.

## Q6. Which questions account for the scripted IN failure, and the mock stall

**Scripted IN failure (3.3 mm, two attempts).** Accounted for by:
- Q2 and Q3, the reference: the layer drives toward FK of the solver output, which sits about 2.15 mm above the planned z during IN (mean), so the arm is held about 2 mm short of the planned point.
- Q7, the periodic reset: at each reset during IN the correction is dropped (IN resets, C norm before reset: 7.3, 16.9, 17.9, 12.2, 11.4, 11.2, 11.0 mm). The integrator winds up to 11–18 mm within a second and loses it at the next reset, so the trim never settles. (The pad's z response to a reset was not visible within 6 ticks, 10583 to 10593, so the effect on the pad is not measured here.)
- Not accounted for: the carry (Q4, 0 mm of the IN difference between attempts), the zero-step guard (Q5).

**Mock stall (3.4 mm, 1.0°).** The logs cannot answer this from the mock's tick log (labels and C are empty for the mock run). What the mock's own log does show (mock_queries.csv): the option-A progress stops at 2.588871 s of a 3.019 s path from query 35 (t = 76.41 s) to query 69 (t = 86.83 s), so the mock commands the same path point for about 10 s. That is the mock's progress rule (nearest path point to the sensed pad), not the layer. The mock also stalled at 3.4 mm before the layer existed (run_200g_mock2 at 684a607, no trim at all). So the layer does not cause the mock stall, and the periodic reset may still hit the mock's IN if its gripper is commanded open then (unknown from the logs).

What would settle the mock: a small run that logs the layer's C, its resets (`sag_log`), and FK of the mock's chunk targets per tick, with labels. That is a run, so not done here.

## Q7. Options for a reference that works with absolute joint targets

Not chosen; listed for David.

| option | what the reference is | what it changes |
|---|---|---|
| A. Joint-space trim | integrate `q_src − q_measured` directly, no FK | no solver residual in the reference; works for any absolute-joint source; the trim is in joint space, so gains and limits must be retuned (not the M262 values) |
| B. Keep FK of the targets, make the solver converge | FK of the targets, but the source solves to convergence each tick | removes the planned-vs-solver offset while keeping the task-space trim; needs more solver work per tick |
| C. Source reports its pose target | the source sends the planned pose with its joints | the reference is the planned point again (old behaviour); the layer's contract changes from absolute joints to joints plus pose, so every source must report it (mock and openpi too) |
| D. Pose target from the source's FK at the last commanded point | FK of the previous command, held per step | a middle ground; keeps the absolute-joint contract but the reference lags by one step |

Option A is the only one that needs no pose from the source. Options B and C keep the task-space trim. Each option is a change of interface or of the trim's frame, so each needs David's decision (M262 decision 1 already says the reference is FK of the targets; that is a decision David made, not me).

Separately from the reference, the reset is a different decision (below).

## Q8. Not built, needs decision

1. **Periodic reset.** Default in the code: the open watch restarts on every open command and resets C every `close_min_s` (1.0 s) while the fingers are at rest. Old behaviour: reset only at the script's OPEN step and at recovery start. Decision: restrict the open watch to the script's release and recovery opens, or keep the rule.
2. **Trim during joint and recovery steps.** Default: the layer trims every arm command, including `joints` and recovery steps. Old behaviour: move steps only. Decision: keep for all sources, or limit to move steps (the source would need to say what it is).
3. **restart_from_commanded latent double count.** Default: False, so inactive. If set True, the script's reference would include C unless the subtraction is restored.
4. **Solver residual.** Default: reference is FK of the script's solver output (no correction for the residual). Decision: which option in Q7.
5. **Zero-step guard counter.** Default: no count. A counter in the guard would answer how often it fires.

## Files

- Created: `docs/reports/20261105_0500_sag_layer_diagnosis.md` (this file, uncommitted).
- Nothing else changed. Scratch files were only read.
