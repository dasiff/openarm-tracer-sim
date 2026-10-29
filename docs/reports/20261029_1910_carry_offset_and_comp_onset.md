# Carry offset (A) and the early bounce after held-object compensation (B)

Written for: David (controller review). Read-only analysis of existing logs, plus one scratch run (held term ramped over 0.5 s, one change). No production code, configuration or gain changed. Nothing pushed.

## Summary

- **A (steady carry offset) is gravity plus joint friction at the servo, not a missing controller term.** The arm's own gravity model is exact (residual 0.02 N·m alone). Before compensation, the beaker's weight at the pad centre explains the residual on L1 and L7 to 0.01 N·m. Each joint's offset equals that residual divided by its servo gain (kp = 70, 70, 70, 60, 10, 10, 10). The wrist joints L5–L7 have kp = 10, so the same torque gives a 7× larger offset. Their residuals are inside the 0.1 N·m joint friction band, so they are friction-held, and a better gravity model cannot remove them.
- **B (the early bounce) is caused by the compensation step.** The held term steps the servo targets by 13.7 mrad on L1, 38 mrad on L7, and so on, in one tick. The arm relaxes into its new equilibrium over about 25 ticks while the trajectory command is still static. The hand and the beaker rotate together (the beaker is rigid in the grip), so the beaker tilts 1.5° before the place's own motion starts. This is a monotonic relaxation, not an oscillation.
- **Controlled test:** ramping the held term over 0.5 s (one change) cuts the first tilt peak from 3.41° to 1.53°, moves the onset from +1 to +28 ticks, and reduces the peak joint speed from 0.40 to 0.30 rad/s. Settling (+312 → +315) and the DOWN bump (0.80° → 0.78°) are essentially unchanged.
- **Recommendation:** address the activation step first (it is the largest single contributor to the early peak, and a ramp is cheap). Do not treat the steady carry offset as the bounce's cause. Confirm with the delay experiment in section 7 before deciding how far to go.

## 1. Control law (verified, direct measurement)

- Arm joints are MuJoCo position actuators. Per joint (`src/robot_specs.py`, the loaded model): kp = 70, 70, 70, 60, 10, 10, 10 N·m/rad; kv = 2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5 N·m·s/rad; torque limits 40, 40, 27, 27, 7, 7, 7 N·m.
- Actuator force is `kp(act − q) − kv·qd`. Fitting the production run's `force` gives residuals of 0.002–0.023 N·m RMS on every joint (the fitted damping equals the spec kv).
- The feedforward (gravity, plus the held term from M196) is added to the target: `act = tgt + ff/kp`. Checked: the maximum difference is 5e-6 rad, so the convention is as coded.
- Joint friction in the model: `frictionloss = 0.1` N·m and `damping = 0.4` N·m·s/rad on each arm joint (`models/robot/combined_robot.xml`).
- Saturation: no. Maximum |force| is 8.5 N·m on L1 (limit 40), and 1.3 N·m on L7 (limit 7).
- Integrators: none in joint space. The only integrator is the Cartesian sag correction (`src/mpvi_modules.py` step_inner, gain 3 /s, clip 40 mm and 0.1 rad). It is reset to zero at every skill start and whenever the gripper opens.

## 2. Static balance during carry (direct measurements, `data/logs/carry_analysis_20261029.log`)

Let g = force − ff (the servo's torque beyond the feedforward). In static equilibrium g = kp·o, where o = tgt − q is the command minus the measured joint.

| window (production run, no held term before 11,599) | g L1 | g L2 | g L3 | g L4 | g L5 | g L6 | g L7 | o L1 (mrad) | o L7 (mrad) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| arm alone, ticks 50–300 | +0.02 | −0.02 | +0.02 | +0.06 | 0.00 | 0.00 | 0.00 | +0.3 | +0.2 |
| carry before comp, 11,500–11,598 | −0.99 | −0.21 | −0.33 | +0.77 | −0.09 | +0.10 | −0.28 | −14.2 | −27.1 |

Notes:
- The arm alone residual (≤ 0.06 N·m) is the gravity-model error. It is small compared with the beaker's torque.
- The offset equals the residual over kp: L1 −0.99/70 = −14.1 mrad (measured −14.2); L7 −0.28/10 = −28 mrad (measured −27); L4 +0.77/60 = +12.8 mrad (measured +13.6).

## 3. Beaker weight, pad centre vs centre of mass (calculation, `data/diag/carry_20261029/lever_check.py`)

At tick 11,598 the beaker was placed in the model at its recorded offset from the pad (upright). Torque from a straight-down weight at the pad centre (the held term's point) and at the beaker's centre of mass:

| joint | residual before comp (measured) | torque at pad centre | torque at COM (0.205 kg) |
|---|---:|---:|---:|
| L1 | −0.963 | −0.962 | −0.962 |
| L2 | −0.180 | −0.121 | −0.120 |
| L3 | −0.300 | −0.261 | −0.261 |
| L4 | +0.667 | +0.765 | +0.765 |
| L5 | −0.076 | +0.011 | +0.010 |
| L6 | +0.078 | −0.001 | −0.000 |
| L7 | −0.366 | −0.379 | −0.379 |

- **The pad centre and the COM give the same torque** (the COM is 0.1 mm horizontally from the pad). So the held term's geometry is right, and the lever-arm hypothesis is rejected.
- **L1 and L7 agree to 0.01 N·m.** The mismatch on L2 to L6 is 0.04–0.10 N·m, which is inside the 0.1 N·m joint friction band. The servo can rest anywhere in that band, so the wrist offsets are friction-held (measured, the inference is calculation).
- After compensation the gravity residual is ~0 on every joint (the pad-centre torque minus the held term is within 0.02 N·m), so the held term is a correct gravity model. Wrist offsets that remain after compensation are the stiction band (±0.1/kp: ±10 mrad on L5–L7).

## 4. Integrator state (direct measurement, `data/closed_loop/on_20261029_1453/instr_ticks.csv`)

- The sag correction is 17.5 mm in z at tick 11,596 (UP/CHECK). That is the integrator's steady correction for the carry. It is not saturated (clip 40 mm).
- At the place start (OVER, 11,601) the skill's `start()` sets it to zero. The carry correction is dropped there. This is the same reset identified in the start-mismatch report.
- The integrator does not contribute to the early tilt: it is zero from OVER onward until the reference moves away from the measured pad (the sag column is 0.000 until 11,610).

## 5. Tilt vs joint error (direct measurement)

- `tilt` (beaker axis from vertical) and `htilt` (hand tool axis) move in exact opposition: at 11,624, Δtilt = +2.481°, Δhtilt = −2.492°. `rtilt` (beaker vs hand axis) stays at 90.60° ±0.001°. The beaker is rigid in the grip and its tilt is the hand's orientation, a function of the measured joints.
- So the command-minus-measured offset does not tilt the beaker directly. It affects tilt only through the joint motion the servo produces.
- Largest error is not largest tilt: L7 has the largest offset (36 mrad at 11,598) but its residual is gravity-held. The joints that move the hand in the early window are L1 (−12 mrad by 11,624), L4 (+9 mrad) and L7 (−22 mrad). A per-joint tilt sensitivity by least squares was tried and rejected (rank-deficient with 7 collinear joints over 15 samples; the values are not physical).

## 6. Causal timeline around activation (direct measurement; production run)

| tick | event | evidence |
|---|---|---|
| 11,590–11,598 | static carry; command static (|Δtgt| < 2e-4 rad per tick on every joint) | extract |
| 11,599 | held term applied (the module's weight reading, 0.205 kg). Servo target steps by ff/kp: L1 −13.7 mrad, L7 −38 mrad | `ff_held` step, `act` |
| 11,600 | max joint speed 0.053 rad/s; hand tilt +0.009° | extract |
| 11,602 | tilt +0.09°; command still static (PLACE OVER starts, reference from measured pad) | extract |
| 11,604–11,616 | relaxation: pad rises 1.07 mm → 8.4 mm; tilt +0.23° → +1.56°; commanded speed < 0.02 rad/s until 11,616 | extract, instr |
| 11,612 | command first moves more than 2e-4 rad/tick; joint speed peaks ~0.16 rad/s at 11,606 then decays | extract |
| 11,626–11,636 | OVER command and relaxation superpose in opposite directions; joint speed reaches its minimum (0.027 rad/s) at 11,636 | extract |
| 11,636 | tilt peak 3.41°, then falls to 0.5° by 11,644, then rises to 2.7° at 11,683 | extract |

The first 16 ticks show no command motion and no oscillation in L1: its velocity rises monotonically to its peak and then decays (plot `data/diag/carry_20261029/out/timeline_on_off.png`).

## 7. Controlled experiment: held term ramped over 0.5 s (one change)

Launcher `data/diag/carry_20261029/run_ramp.py` (scratch): the module's held-mass set is applied as a linear ramp from 0 to 0.205 kg over 75 ticks, starting one tick after the set. Nothing else differs. Baseline: production-identical (the pre-11,599 targets and joints match the production run exactly, 0.0 difference over 1,599 ticks). Replay verified (exit 0). Logs: `data/logs/ramp_20261029_185915_*.log`.

| metric | production (step) | ramp 0.5 s | compensation off (run 1453) |
|---|---:|---:|---:|
| tilt at +2 / +8 / +16 / +24 / +36 (deg) | 0.31 / 0.84 / 1.78 / 2.70 / 3.41 | 0.22 / 0.22 / 0.24 / 0.28 / 0.35 | 0.22 / 0.22 / 0.22 / 0.21 / 0.07 |
| first peak tilt | 3.41° at +34 | 1.53° at +104 | 3.12° at +68 (run 1505 report) |
| oscillation onset (+0.1°) | +1 | +28 | +41 (run 1505 report) |
| min of max joint speed, 11,626–11,645 | 0.027 rad/s | 0.011 rad/s | 0.013 rad/s |
| max joint speed, 11,600–11,700 | 0.40 rad/s | 0.30 rad/s | 0.28 rad/s |
| DOWN max tilt | 0.80° | 0.78° | 1.27° |
| settling (last tick above 0.5°) | +312 | +315 | +329 |
| OPEN tick | 12,158 | 12,153 | — |

Plot: `data/diag/carry_20261029/out/tilt_step_vs_ramp.png`.

Reading:
- The early peak is mostly the step's relaxation. With the ramp, the same relaxation is spread over ~0.5 s, the joint speed drops, and the tilt peak halves.
- The ramp does not remove the bounce. The later peak (1.53° at +104) is the OVER motion acting on the relaxed arm. Settling does not improve.
- Compensation off (run 1453) also has a bounce (3.12°) with onset at +41: the compensation is not the source of the bounce, only of its early timing and size. This matches the earlier report.

## 8. What the evidence establishes, and what it does not

Established (direct measurement or calculation):
- The servo law, the feedforward convention, and the absence of saturation (section 1).
- The offset equals the untracked torque over kp, and the untracked torque is the beaker's weight (sections 2–3). The gravity model is exact (arm residual 0.02 N·m).
- The held term is correct in gravity (post-compensation residual within 0.02 N·m), and the pad-centre point is the right one (COM under the pad).
- The beaker's tilt is the hand's orientation, rigid in the grip (section 5).
- The first tilt rise is the arm relaxing after the step, while the command is static (section 6). The ramp halves the early peak (section 7).

Not established (hypotheses, or needs more signal):
- Whether the wrist offsets (L5–L7) are stiction. The 0.1 N·m band explains their size, but no run shows a stick-slip breakaway. A test would log the friction (residual minus gravity) through a slow command ramp.
- Whether the superposition at 11,636 is the cause of the second peak (section 6). It is timing evidence only.
- Whether a ramp of another length helps (0.25 s, 1 s). One run only (75 ticks).
- The sag-integrator's 17.5 mm during the carry: why the carry needs it at all (the gravity model is exact after compensation, so it is the pre-compensation mismatch). The integrator's role before compensation is not separated here.

## 9. Smallest next experiment that would distinguish the leading explanations

Delay the start of PLACE OVER by about 0.6 s after the held step (hold the arm at CHECK until the relaxation has finished), with compensation on and the step (not ramp) activation unchanged. One change only (a wait before OVER).
- If the second peak and the 11,636 reversal disappear and the first peak falls toward the no-compensation level, the bounce is the superposition of relaxation and OVER motion (interaction hypothesis).
- If the peak stays near 3.4°, the relaxation alone (dynamic) is the cause and the trajectory timing is not.

A baseline is valid (production-identical run; qcmd verified in earlier runs). The wait is a trajectory timing decision (a change of the place sequence), so it needs David's approval before it becomes production.

## 10. Recommendation

1. **Address the activation step first.** It is the largest single contributor to the early bounce, the ramp is a small change, and the ramp run shows it halves the first peak. Before adopting it, decide the ramp length (the 0.5 s tested is one value) and record it as a decision. This is David's call.
2. **Then test the superposition** (section 9). If the bounce still follows the OVER motion, the next lever is the place's start timing, not the controller.
3. **Leave the wrist's steady offset alone for now.** It is a friction-limited steady error on the low-gain joints (L5–L7). It moves the hand by only a few millimetres, and the bounce does not depend on it in this analysis.

## 11. Files

Scratch (git-ignored, `data/diag/carry_20261029/`):
- `analyze_carry.py`: the windowed tables and timeline plot (`out/window_table.csv`, `out/timeline_on_off.csv`, `out/timeline_on_off.png`).
- `lever_check.py`: pad centre vs COM torque at a tick (the table in section 3).
- `run_ramp.py`: the ramp launcher; `ramp_20261029_185915/` (recording, episode); `ramp_ticks_20261029_185915.csv` (extract).
- `out/tilt_step_vs_ramp.png`: the step vs ramp vs off comparison.

Inputs (existing): `data/diag/closed_loop_20261029/on_ticks.csv`, `off_ticks.csv`, `data/closed_loop/on_20261029_1453/`, `data/closed_loop/off_20261029_1453/`.

Logs: `data/logs/carry_analysis_20261029.log`, `data/logs/ramp_20261029_185915_run.log`, `data/logs/ramp_20261029_185915_xr.log`.

Tracked files: no change to `src/`, `experiments/`, `models/` or `src/task_config.py`.
