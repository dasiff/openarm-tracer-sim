# Stage 2: offline comparison of arm predictors (2026-11-24, David's analysis brief; no layer change)

Branch `safety-filter` (HEAD 8e45fac). Analysis only: the layer is unchanged. Commit: the analysis script `experiments/predictor_compare.py` and this report. Data (git-ignored): `data/logs/stage2g/` (logged runs `*.npz`, full table `predictor_left.md`).

## Plain-language answer

1. **Which predictor removes the one-signed drift?** Only the full inverse-dynamics predictor with the weight removed (P3c) removes it, and only where the arm moves fast. In policy D (speeds up to about 20 rad/s) the motion-minus-rest gap is 13 mrad on average for the current filter (P1) and 13 mrad for the second-order model (P2), against 0.8 mrad for P3c. In policies B and C the three predictors are all within about 2 mrad, so there is little to remove. In teleop no predictor removes the drift (15 to 18 mrad, worst on joint 1).
2. **Does the second-order model (P2) help?** Hardly. Adding the joint's inertia changes the gap by less than 1 mrad at the policy speeds, and by 0.6 mrad in D. The residual in D is not inertia alone; it is the Coriolis and centrifugal terms (and damping), which P2 leaves out. P3c is the full dynamics, so it has them.
3. **What does each cost per tick?** P1: about 20 µs (Python, 7 joints). P2: about 85 to 115 µs (Python), plus the mass-matrix diagonal, which the simulator already computes every physics step, so reading it is nearly free in the loop. P3c (inverse dynamics, two MuJoCo calls): about 1.2 ms per arm per tick, which is too costly to run every tick on both arms against the 6.7 ms budget; it would fit at the bridge's 50 Hz rate, or in a reduced form.
4. **What would P3 change about what the correction learns?** Its raw offset contains the weight: at rest it is −39 mrad on joint 1 in B and −14 mrad in C (the weight over kp, the rest samples differ). The loop already adds the weight through the gravity feedforward (see below), so the correction must not learn it again. With the weight removed (P3c), the correction would learn only the motion-dependent part, which is the drift it currently accumulates in fast motion.

## Is the arm's own weight compensated in the actuator path?

Yes, in the control loop, not in the physics:
- The XML has no `gravcomp` (`models/robot`, `models/scenes`), and `src/arm_actuators.py` states that gravity compensation is not in the physics.
- `src/control_loop.py` line 1019: `ff_off = gff.torques(data) / gff.kp` (gravity torque of the arm's own bodies at the measured pose, divided by kp, per tick), then `gff.apply_offsets(data.ctrl, ff_off)` adds it to the servo's control: `ctrl = target + tau_ff / kp` (`src/gravity_ff.py`).
- So the servo's applied force already includes the weight. At rest the measured joints sit on the target (P1 at rest: +0.14 mrad averaged over joints).
- The layer's reference (REF = the source target) does not include the feedforward, so any predictor built on REF alone sees the weight as a steady offset at rest only if it models gravity. P3b (command = REF, no feedforward) shows it: −39 mrad on joint 1 at rest in B and −14 mrad in C.

## Method (short)

- Runs: policy B, C and D, seed 1, correction OFF, bridge OFF (logged again with per-tick velocities, applied controls and forces, since the saved runs lacked them). Teleop: the baseline with the sag layer OFF, bridge ON (the default; the bridge output is the reference there).
- Motion mask: |dREF/dt| > 0.05 rad/s for the joint; rest: |dREF/dt| < 0.005 rad/s and |velocity| < 0.01 rad/s. Left arm only (the right arm was logged but not analysed).
- P1: the layer's filter, recursive from the first tick.
- P2: I_j a_j = clip(kp_j (REF_j − x_j) − (kv_j + d_j) v_j, F_j), I_j = the diagonal of the mass matrix at the measured pose (armature included), F_j the actuator's force range, gravity and Coriolis not included, integrated from the measured state.
- P3: inverse dynamics of the measured motion (`mj_inverse`, accelerations from a central difference of the logged velocities). P3a: needed minus the logged actuator force (a consistency check; it is near zero at rest, which confirms the identity). P3b: needed minus the servo law applied to REF, divided by kp. P3c: P3b minus the weight (the same inverse dynamics at zero speed), divided by kp.
- Hand equivalent: the pad displacement caused by one joint's gap, |J_pos,j| × |gap_j| (mm).
- Check run first: `mj_inverse` returns M·a + bias − passive; it does not subtract the actuator force. The first version added the force and double-counted it; the corrected P3a is near zero at rest.

## Results (correction OFF; mrad; motion − rest is the drift to remove)

| Run | Predictor | Mean abs(motion − rest), joints | Worst joint | Joints one-signed (agreement ≥ 0.7) | Rest mean over joints |
|---|---|---|---|---|---|
| B | P1 (current filter) | 1.16 | 3.8 (joint 4) | 0 of 7 | +0.14 |
| B | P2 (second order) | 1.03 | 3.2 (joint 1) | 0 of 7 | −0.22 |
| B | P3c (inverse dynamics − weight) | 1.36 | 2.8 (joint 6) | 0 of 7 | −0.03 |
| C | P1 | 2.02 | 5.8 (joint 7) | 0 of 7 | +0.15 |
| C | P2 | 2.30 | 6.1 (joint 7) | 0 of 7 | +0.33 |
| C | P3c | 1.60 | 4.2 (joint 7) | 0 of 7 | +0.59 |
| D | P1 | 13.34 | 28.8 (joint 4) | 0 of 7 | +0.20 |
| D | P2 | 12.68 | 25.2 (joint 4) | 0 of 7 | −0.04 |
| D | P3c | **0.83** | 1.3 (joint 5) | 0 of 7 | −0.04 |
| Teleop (bridge on) | P1 | 17.48 | 44.6 (joint 1) | 3 of 7 | −2.64 |
| Teleop | P2 | 18.21 | 41.2 (joint 1) | 3 of 7 | −2.35 |
| Teleop | P3c | 15.31 | 42.9 (joint 1) | 3 of 7 | −1.15 |

Per joint, policy D (motion mean / rest mean, mrad):

| Joint | P1 | P2 | P3c |
|---|---|---|---|
| 1 | +0.08 / −0.01 | +8.86 / −0.00 | +0.33 / −0.04 |
| 2 | +18.68 / +0.55 | +20.97 / +0.59 | +1.27 / +0.04 |
| 3 | +13.61 / −0.56 | −3.18 / −0.76 | +0.47 / −0.06 |
| 4 | −27.85 / +0.90 | −25.85 / −0.62 | −1.32 / −0.21 |
| 5 | −0.73 / +0.50 | −0.63 / +0.50 | +1.44 / +0.09 |
| 6 | −6.53 / −0.00 | −6.30 / −0.00 | +0.74 / −0.03 |
| 7 | +24.49 / −0.01 | +24.46 / −0.01 | −0.50 / −0.04 |

Teleop per joint (motion / rest, mrad), P1 and P3c: joint 1 +44.5 / −0.2 and +43.4 / +0.5; joint 5 −19.5 / −5.8 and −12.0 / −5.2; joint 7 +23.0 / +2.1 and +20.0 / +7.1. The teleop drift on joints 1 and 7 is not removed by inertia or by the full dynamics, which points at the bridge's reference (a staircase of 50 Hz updates with a 0.01 rad step cap, not a smooth target). This is a hypothesis; not tested here.

Full per-joint tables for all four runs, with P3a and P3b and the hand-equivalent values, are in `data/logs/stage2g/predictor_left.md`.

## Earlier figure corrected

The −36 mrad on joint 4 in the earlier report came from the **correction-on** run (`B_s1_on_nobridge`), where the offset feeds back into the motion. With the correction off, the same joint shows −2.9 mrad (P1, motion mean). The drift in the correction-on run is partly the correction's own effect.

## Caveats

- Accelerations are central differences of logged velocities (noisy: P3 spreads of 50–400 mrad are mostly this).
- The teleop run has the bridge on, so its reference is the bridge output, not the source; the policy runs have the bridge off.
- The right arm was logged but not analysed.
- P2 and P3 use the model's inertia and damping; friction (frictionloss 0.1 N m per joint) is in P3's passive term and not in P2; the friction lag on joints 5 to 7 (kp 10) is about 10 mrad, which can matter for those joints.
- No predictor here is adopted. This report does not change the layer.

## Decisions for David (not made here)

1. Which predictor the layer should use: P1 (current, cheap) is adequate at policy B and C speeds; P3c is needed only for the fast motion of D, and it costs about 1.2 ms per arm per tick (or it could run at the bridge's 50 Hz rate).
2. If P3c is used, the gravity term must come out of it (the feedforward already supplies the weight), or the correction learns the weight twice.
3. Whether the teleop drift on joints 1 and 7 should be attacked (its likely cause is the bridge reference, not the predictor).
