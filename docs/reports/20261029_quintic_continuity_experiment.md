# Quintic continuity experiment: the place handover (scratch, no production change)

Written for: David (controller review). Scratch experiment only. No tracked source, configuration or model file changed. Nothing pushed.

## Summary

- **Result: criterion 2 of the prompt.** Making the place handover continuous removes the reference jump and the L4 target-velocity jump at the OVER→DOWN handover, but it **does not reduce the bounce**. The peak tilt (3.41° at +34) is unchanged, because it happens in OVER, before the handover. The DOWN bump gets larger (0.80° → 0.94°), settling gets later (+312 → +365 ticks), and the measured jerk is about 3.6× higher (407 → 1453 rad/s³ on L7).
- The handover discontinuity is real and has a clear cause (section 2). Fixing it alone is not enough to reduce the bounce.
- The within-OVER event at tick 11,661 (L3 and L5) is **not** affected by the handover change. It is mink's per-joint velocity limit (0.5 rad/s) releasing inside the solver, downstream of the interpolator. It was logged and not masked.
- An earlier variant that also changed the pick's handovers was confounded (the pick finished differently and the beaker was already 2.8° tilted when the place started). It is kept as evidence, but the comparison that matters is the place-only run.

## 1. Code path investigated

All trajectory logic for the scripted pick and place is in `src/mpvi_modules.py` (`make_scripted_manipulation`):

- Segment definitions: `plan_pick` (OVER, DOWN, IN, CLOSE, UP, CHECK) and `plan_place` (OVER, DOWN, OPEN, BACK, HOME, HOME joints), around lines 195–240. Every move is `move(name, target, speed, R)`.
- Time law per move, `step_inner` (about line 320): `u = el / T`, `s = min_jerk(u)` (`src/rate_limit.py:min_jerk`, the quintic 10u³ − 15u⁴ + 6u⁵). The Cartesian reference is `p_ref = p0 + (target − p0)·s`, with slerp for the hand angle. So each move on its own is a quintic with zero speed and acceleration at both ends.
- Duration `T` (about line 335): the larger of `min_move_s`, distance/speed, angle/turn rate, and the joint-arc term (M194).
- Blended handover (M200, `blend_radius_m` = 0.020 m): when the reference reaches `u ≥ 1` and the measured pad is within 20 mm of the waypoint, the next move starts at once. Its reference is a cubic `p0 + v0·T·(u − 2u² + u³) + (target − p0)·(3u² − 2u³)`, with `p0` = the **measured** pad pose and `v0` = the **measured** pad speed.
- Phase-start ramp (M198/M195, `phase_start_ramp_s` = 0.5625 s): the solver's joint speed is multiplied by `min_jerk(t / 0.5625)`. A blended handover skips the ramp.
- Sag integrator (`C`): driven by `p_ref − p_now`, added to every reference.
- IK: `arm.step(data, p_ref + C, R)` calls mink (`src/mink_arm.py`) with a per-joint `VelocityLimit` at `MINK_CFG["max_joint_speed"]` = 0.5 rad/s (`uniform_speed_scaling` False). The solver returns joint velocities, and the commanded `q_cmd` follows them.

The discontinuities at the handover come from the M200 blend, not from the cubic alone. Per tick in `data/closed_loop/on_20261029_1453/instr_ticks.csv`:

- Tick 11,852 (last OVER tick): pad speed about 0, outgoing reference at rest at the OVER target.
- Tick 11,853 (DOWN starts): blended, `blend_v` = (0.0126, 0.0007, −0.0100) m/s. The new reference starts at the measured pad, which is 6.9 mm from the OVER target on this run.
- Ticks 11,853–11,856: the solver velocity on L3, L4 and L5 saturates at the 0.5 rad/s limit (`vel_bind` 0111111 → 0110110), then releases. L4 goes from −0.01 to −0.47 rad/s in one tick, and reaches −0.50 at 11,855.

So the reference position steps by the gap between the measured pad and the outgoing target (up to 20 mm, 6.9 mm here), and its speed steps from zero to the measured speed. The cubic's acceleration at the start is also nonzero. Blended ref acceleration over OVER..OPEN peaks at 154.5 m/s², against 0.42 m/s² after the change.

**Within-OVER event (tick 11,658–11,661, L3 and L5).** During the phase-start ramp, about 4–6 joints sit at the solver's 0.5 rad/s velocity limit (`vel_bind` 1011111, 6 joints, ticks 11,602–11,605). Around tick 11,657–11,659 the binding releases in sequence. L3 goes from +0.50 to −0.10 rad/s and L5 from −0.24 to +0.31 rad/s in about two ticks. This is a direction change in the solver's joint velocity caused by the per-joint limit (`uniform_speed_scaling` = False: a joint at the limit is held there while the others continue). It is downstream of the interpolator. The Cartesian reference through this window was not checked separately. The fix would be in the velocity-limit design (for example `uniform_speed_scaling` = True), which is a decision for David and was not tested.

## 2. Experimental change

File: `data/diag/quintic_20261029/mpvi_modules_quintic.py` (scratch, git-ignored `data/`). Copy of `src/mpvi_modules.py` with one block changed (around line 331), for place skills only:

- The new move's reference starts at the outgoing move's target (`prev["target"]`) and hand angle (`prev["R"]`), at rest. It does not use the measured pad state or the measured speed (`v0` is not set).
- The time law is the same min-jerk quintic (zero speed and acceleration at both ends), with the same `T`. `T` is computed exactly as in production (from the measured pad distance), so the duration is unchanged.
- Trigger, endpoints, path, ramp handling (no ramp at a blended handover), gains, IK, limits, sag integrator, compensation and grip logic: all unchanged.
- Pick handovers: unchanged M200 blend (the `elif` branch).

Assumption: the outgoing reference is at rest at the waypoint (min-jerk ends at zero speed), so a reference that starts from its end point with zero speed and acceleration is C² with it. Matching the measured speed (the M200 intent) cannot be done without changing the outgoing reference (for example, starting the blend before `u = 1`), which would change timing. That was not tested.

Launcher: `data/diag/quintic_20261029/run_variant.py <out> base|quintic`. It runs `experiments/demo_video.py` `record_only` (the same path as the closed-loop recordings), with held-object compensation ON. The variant file is loaded in place of `src.mpvi_modules` for `quintic` mode. The logging is read-only (solver velocity-limit binding, joint-limit slack, collision pairs, the Cartesian reference recomputed from the move's closure values, the measured pad, the handover gap).

Copy kept for the record: `data/diag/quintic_20261029/mpvi_modules_quintic_all.py` (the all-handover variant, see section 5).

## 3. Baseline verification

- `data/diag/quintic_20261029/base_20261029_152524` (production `src/mpvi_modules.py`, through the launcher): 13,614 ticks, rc 0. The joint command `qcmd` is **identical on all 13,614 ticks** to the recorded `data/closed_loop/on_20261029_1453` run.
- Its per-tick extract matches the existing closed-loop numbers: peak 3.409° at +34, onset +1, DOWN bump 0.801°, settling +312, max target dv 0.411 rad/s (L4, 11,856), max target jerk 9,241 rad/s³ (L4, 11,855). The metric script (`data/diag/quintic_20261029/compare_quintic.py`) reproduces the old report's numbers on that run before it is used on the new ones.

## 4. Results (closed-loop, same seed and settings)

Place-only variant: `data/diag/quintic_20261029/place_20261029_154500` (13,539 ticks, rc 0). Extract replay ok (13,538 ticks). Metrics: `data/logs/quintic_compare_place_20261029_154500.log`.

| metric | base (production) | quintic, place handovers only |
|---|---:|---:|
| OVER / DOWN tick (handover) | 11,602 / 11,853 | 11,602 / 11,853 (same) |
| OPEN / BACK / HOME tick | 12,157 / 12,308 / 12,517 | 12,142 / 12,293 / 12,502 |
| peak beaker tilt (OVER..OPEN) | 3.409° at +34 | 3.409° at +34 (same, before the handover) |
| first oscillation onset (+0.1° over base) | +1 | +1 |
| DOWN bump (max tilt in DOWN) | 0.801° (at +279) | 0.944° (at +303) |
| settling (last tick above 0.5°, after peak) | +312 | +365 |
| max per-tick target velocity change | 0.411 rad/s (L4, 11,856) | 0.352 rad/s (L5, 9,918, in the pick; not at the place handover) |
| max target jerk | 9,241 rad/s³ (L4, 11,855) | 7,928 rad/s³ (L5, 9,917, in the pick) |
| reference max acceleration (OVER..OPEN) | 154.5 m/s² (handover) | 0.42 m/s² |
| reference max speed (OVER..OPEN) | 1.02 m/s | 0.225 m/s |
| max measured jerk, OVER..OPEN | 407 rad/s³ (L7) | 1,453 rad/s³ (L7) |
| max measured joint speed L3/L7, OVER..OPEN | 0.40 rad/s | 0.55 rad/s |
| tracking error at DOWN start (pad vs ref) | 4.9 mm | 6.8 mm |
| tracking error, max over OVER..OPEN | 20.1 mm | 20.1 mm |
| sag integrator peak | 15.87 mm (at +147) | 15.87 mm (at +147) |
| sag clip ticks | 0 | 0 |
| ticks with a velocity limit binding, OVER..OPEN | 64 | 58 |
| minimum joint-limit slack | 0.0335 rad | 0.0340 rad |
| within-OVER event (11,661, L3) | 0.26 rad/s per tick | 0.26 rad/s per tick (unchanged) |

The "max target velocity change" in the variant is in the pick (tick 9,918), which is outside the place. The place-handover jump (L4 0.41 rad/s) is gone; the within-OVER event remains.

Reading the table against the prompt's decision criteria:

- The two identified discontinuities: the handover one is removed in the reference (ref acceleration 154 → 0.4 m/s²) and in the L4 target (the 0.41 rad/s jump is gone). The within-OVER one remains (section 1).
- Tilt: the peak and onset are unchanged, and the DOWN bump and settling are worse. The bounce is not reduced.
- Measured jerk and tracking: worse. With the continuous reference, the arm lags the 20 mm gap more and catches up later, with higher measured jerk and speed.
- Duration and endpoints: unchanged handover tick and move durations. OPEN, BACK and HOME are 15 ticks earlier (the DOWN move ends a little earlier), so the later segments shift.

Conclusion: **the quintic continuity experiment supports criterion 2.** It improves target smoothness at the place handover but does not materially reduce bouncing, and it increases measured jerk and the DOWN bump. The bounce is driven by the OVER move itself (the 3.41° peak at +34, before any handover), not by the handover.

## 5. Deviations and the confounded run

- **All-handover variant (`quintic_20261029`, `mpvi_modules_quintic_all.py`).** The first version applied the change to every move-to-move blend, including the pick's OVER→DOWN and DOWN→IN. The pick then finished differently: CLOSE at 74.39 s instead of 74.73 s, and the beaker was already at 2.8° tilt at tick 11,598 (base: 0.22°). Its metrics (peak 2.72°, settling and onset) are not comparable with the base, so they are not used as a result. The variant was restricted to place handovers and rerun; the place-only run is the one reported.
- **Measured-jerk figure.** The earlier closed-loop report gave 604 rad/s³ (L7) for the same window. With this definition over OVER..OPEN, the base run gives 407 (L7); I could not reproduce 604 with any window tried. The whole-run maximum is a spike at tick 11,002 (during the pick, 11,718), outside the window. Both runs use the same definition, so the comparison is consistent, but the absolute value differs from the old report.
- **Onset in the all-handover run.** It was reported as "None" because the pre-OVER tilt differed between runs. This is not a result.
- **Single run each.** The base reproduces the recording exactly, so the comparison is deterministic for a given code path. There is no seed sweep, so the variant's effect on tilt is one sample.
- **Timing.** The OVER move for the place is the same length (the handover at 11,853 in both). The later segments shift by 15 ticks, which the prompt allows as a consequence of the DOWN move ending earlier. No duration was lengthened or shortened on purpose.

## 6. Limitations

- The variant is a reference-only change. The measured arm still lags the continuous reference by up to 20 mm at the start of DOWN (tracking error 6.8 mm at DOWN start, 4.9 mm in the base). A different design that keeps the measured speed (matching the M200 intent) would need the outgoing reference to be changed, which affects timing. Not tested.
- The within-OVER event is not addressed. Its cause (the per-joint velocity limit releasing at 0.5 rad/s, with `uniform_speed_scaling` False) is identified from the logs but not tested. A test would need a decision on the velocity-limit design.
- The arm's tracking loop (PD gains, feedforward, sag integrator) was not changed and was not examined. A measured-jerk increase suggests the tracking loop, not the reference, is the limiting factor for the bounce.

## 7. Files

Created (all git-ignored under `data/`, nothing committed except this report and the progress log):

- `data/diag/quintic_20261029/mpvi_modules_quintic.py`: place-only variant (the one used).
- `data/diag/quintic_20261029/mpvi_modules_quintic_all.py`: all-handover variant (confounded, kept as evidence).
- `data/diag/quintic_20261029/run_variant.py`: launcher (`base` or `quintic`).
- `data/diag/quintic_20261029/compare_quintic.py`: metrics (validated against the old run).
- `data/diag/quintic_20261029/base_20261029_152524/`: baseline recording and `instr_ticks.csv`.
- `data/diag/quintic_20261029/quintic_20261029_153200/`: all-handover run (confounded).
- `data/diag/quintic_20261029/place_20261029_154500/`: place-only run (reported).
- `data/diag/quintic_20261029/base_ticks.csv`, `quintic_ticks.csv` (all-handover extract), `place_ticks.csv`: per-tick extracts from `experiments/bounce_diag_extract.py`.
- Logs: `data/logs/quintic_base_20261029_152524.log`, `data/logs/quintic_run_20261029_153200.log`, `data/logs/quintic_xbase_20261029_153200.log`, `data/logs/quintic_xq_20261029_153200.log`, `data/logs/quintic_place_run_20261029_154500.log`, `data/logs/quintic_place_x_20261029_154500.log`, `data/logs/quintic_compare_20261029_153200.log` (all-handover), `data/logs/quintic_compare_place_20261029_154500.log` (place-only).
- This report and `docs/progress.md` (committed locally, not pushed).

Tracked files: `git status` shows no change to `src/`, `experiments/`, `models/` or `src/task_config.py` from this work. Verified before the final commit.

Commands run (all in the venv, `MUJOCO_GL=egl`, through `experiments/run_capped.sh`, one at a time, in tmux): `run_variant.py … base`; `run_variant.py … quintic` (all-handover, then place-only); `experiments/bounce_diag_extract.py` on each recording; `compare_quintic.py` on the pairs above. Plus `git commit` of the progress log (local).

## 8. Recommendation (not implemented)

1. **Do not adopt the reference-only quintic handover.** It is smoother at the target but makes the bounce slightly worse (DOWN bump +0.14°, settling +53 ticks) and measured jerk 3.6× higher.
2. **Look at the tracking loop next, and at the velocity limit.** The reference is now smooth but the arm still bounces. The next question is why the measured arm responds with 1,450 rad/s³ jerk to a smooth reference (the PD gain, the feedforward, and the sag integrator are the candidates). This is a controller design question for David.
3. **The within-OVER event (11,661) is a velocity-limit design question.** `uniform_speed_scaling` = True would scale the whole joint velocity instead of holding one joint at the limit. Changing it is a design decision, so it needs David's approval before any test.
4. The M200 blend's intent (keep the measured speed across a handover) is still useful. A continuous version needs the outgoing move to carry its speed past `u = 1`, which changes timing. Ask David before testing.
