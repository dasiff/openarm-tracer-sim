# Stage 2: the correction-on climb, the teleop gap, and a deliberate model error (2026-11-24, David's d0bc brief; analysis and test only)

Branch `safety-filter` (HEAD e62ab94 before this round). The layer and the loop are unchanged. Not pushed.
Scripts (committed with this report): `experiments/climb_perturb_diag.py` (logging wrapper and mode dispatch), `experiments/climb_analysis.py` (climb and teleop analysis), `experiments/climb_static.py` (static holds). Data (git-ignored): `data/logs/stage2h/` (logged runs, plots, tables), `data/logs/stage2g/` (teleop log).

## Plain-language answer

1. **Climb with the correction on.** The offset's own motion is in the loop. The reference the layer compares against is the source target, and it does not include the correction's own joint change `dq`. So when `dq` moves the arm, the arm lags the command by the motor's lag, and the layer reads that lag as error. Over the window where the offset climbs (the fast phase of policy B, around 3 to 6 s), 60 to 90 % of the joint-space error is the lag of `dq`: the regression of the error on the filtered `dq` has slopes of −0.6 to −0.9 on joints 1 to 6 (joint 7: −0.1), and the reproduction of the layer's expected joints matches the logged values to 0.06 mrad. This is candidate "the offset moving the arm in a way the comparison does not account for". The other candidates are not the cause: the straight-line step is accurate even at awkward poses (achieved versus intended hand move: median 0.45 mm, max 2.7 mm, at singular values down to 0.05); the offset does not track the singular value (correlation +0.02); the fade is far too slow (5 s) to make the climb, which happens in under 1 s. I did not close the pad-space sign argument for why the loop keeps climbing rather than settling; that is the decisive test, not run here (see question 1).
2. **Teleop gap on joints 1 and 7.** No. The reference (bridge output) and the joint target the servo actually receives (control minus the gravity offset) differ by 0.3 to 3 mrad on average in motion, and by at most about 7 mrad between bridge updates, where the actuators interpolate the target. That explains a small part of the 15 to 18 mrad gap. The rest is the first-order filter against the arm's own response to a 50 Hz staircase reference; that is a predictor question, not a reference error. The right arm shows the same pattern.
3. **Deliberate model error** (forearm +15 % mass, upper arm +10 %, forearm centre of mass 10 mm along the link). The gravity term does use the unperturbed model (logged: forearm mass 0.616 kg in the gravity term, 0.708 kg in the physics). The static droop runs from 0.3 to 3.6 mm (perturbed) against 1 to 2 mm (nominal), and the learned offset sits 0.1 to 1.9 mm above the droop at each pose (largest at the policy poses), and settles in about 2.7 to 3.5 s after a move. On policy A the hand error with correction off rises from 1.07 to 1.82 mm; with correction on it is 0.17 mm. **The held run with the perturbation did not reach its pick:** it failed in its first phase (arms to home, 30 s timeout) because the arms stayed at their warm-up pose. The nominal held run passes the same phase, so the perturbation is the difference. I did not find out why the source target stayed at zero in that phase, so the held-run item is not answered.

## Technical block

### 1. Correction-on climb (policy B, seed 1, bridge off, correction on; `B_s1_on_climb.npz`)

Per-tick log of both arms: source target, full joint positions and speeds, control and applied force, the reference REF(t−1), dq, the offset before and after, the expected joints, the pad error and the skip rule (`climb_perturb_diag.py run`).

| time (s) | \|C_p\| (mm) | \|C_r\| (mrad) | error (mm) | intended hand move J·dq (mm) | achieved pad(q) − pad(q−dq) (mm) | achieved − intended (mm) | smallest singular value | joint 4 to buffer edge (deg) |
|---|---|---|---|---|---|---|---|---|
| 1.0 | 15.5 | 40.6 | 7.9 | 15.5 | 15.5 | 0.24 | 0.076 | 41.0 |
| 3.0 | 12.4 | 24.6 | 6.3 | 12.1 | 12.0 | 0.34 | 0.134 | 10.2 |
| 6.0 | 32.5 | 56.5 | 23.3 | 31.7 | 31.6 | 1.72 | 0.086 | 51.3 |
| 10.0 | 16.9 | 57.5 | 9.7 | 16.9 | 16.9 | 0.41 | 0.104 | 54.0 |
| 15.0 | 17.5 | 61.9 | 6.5 | 16.4 | 16.5 | 0.49 | 0.051 | 42.3 |
| 20.0 | 13.4 | 47.5 | 12.7 | 12.5 | 13.1 | 1.29 | 0.057 | 24.6 |
| 30.2 | 17.3 | 74.0 | 8.9 | 17.4 | 17.3 | 0.50 | 0.122 | 37.7 |

- Peak |C_p| 34.5 mm at 5.95 s; final 17.3 mm; peak |C_r| 100 mrad (the clip).
- Smallest singular value: minimum 0.000 at the first tick (before the arm moves), median 0.079; the offset does not correlate with it (+0.02 over the second half).
- Max |dq| per joint (mrad): 128, 77, 117, 140, 117, 80, 127. Skips: 4522 applied, 2 `no_reference`.
- Correlation with the offset (second half): singular value +0.02; joint 4 margin +0.08; achieved minus intended +0.33.
- Joint 4 starts 10° below its buffer edge (the zero pose; the buffer clamp holds it at the edge).
- The regression of joint-space error on the filtered dq (correction on): slopes −0.78, −0.91, −0.69, −0.84, −0.58, −0.87, −0.09 for joints 1 to 7. Mean dq while |C_p| > 10 mm: −25, +8, +8, −39, +8, −1, −9 mrad; mean error: +21, −7, −9, +41, −7, +0.3, +9 mrad (the error is about −slope × the filtered dq).
- Plot: `data/logs/stage2h/climb_B_left.png` (offset, error, intended and achieved hand move, smallest singular value, joint 4 margin against time).

### 2. Teleop gap, joints 1 and 7 (bridge on, correction off; `teleop_sagoff.npz`, both arms)

Received target = control − tau_ff/kp (the gravity term from the same class on the nominal model, evaluated at each logged pose). REF = bridge output (dq = 0). Table: `data/logs/stage2h/teleop_gap.md`.

| Joint | Left: motion mean (mrad) | Left: motion std | Left: max between updates | Right: motion mean | Right: motion std | Right: max between updates |
|---|---|---|---|---|---|---|
| 1 | +1.18 | 3.86 | 6.66 | +1.03 | 3.94 | 6.67 |
| 2 | +1.82 | 2.22 | 3.48 | +0.32 | 4.57 | 6.67 |
| 3 | −0.34 | 3.77 | 6.67 | +0.38 | 3.50 | 6.67 |
| 4 | −2.88 | 2.89 | 6.67 | −2.96 | 2.97 | 6.67 |
| 5 | −1.46 | 3.14 | 6.67 | −1.45 | 3.14 | 6.67 |
| 6 | −0.69 | 3.48 | 6.67 | −0.61 | 3.43 | 6.67 |
| 7 | +0.80 | 3.89 | 6.66 | +0.71 | 3.97 | 6.67 |

- Bridge updates (REF changes) and source commands arrive at about 50 Hz (386 control calls in 1152 bench frames); the actuator control changes between them on every tick (it is interpolated over the source period). That interpolation is the 6.7 mrad "between updates" value.
- The reference and the received target differ by 0.3 to 3 mrad on average in motion (the largest joints are 2 to 4). The teleop gap in the predictor (P1 against the measured joints) is 15 to 18 mrad on joints 1, 5 and 7. So the reference signal explains a small part; the rest is the predictor against the arm's response to a staircase-and-ramp input (a predictor question).

### 3a and 3b. Static holds (`climb_static.py`; five poses: home, the held run's pick pose (tick 9300), three poses on policy B's path)

Both arms are simulated with the perturbed physics (or the nominal, as the control); the gravity term uses the unperturbed model; the bridge is not in the loop. Droop = target minus settled hand pose (pad), position norm and rotation.

| Pose | Nominal physics: droop (mm) / rot (mrad) | Perturbed: droop (mm) / rot (mrad) | Perturbed, correction on: \|C_p\| at the end (mm) | Settling after the move (s) |
|---|---|---|---|---|
| home | 1.0 / 4.4 | 0.3 / 1.1 | 1.4 | not within the hold (no move) |
| pick | 1.9 / 10.7 | 2.6 / 12.2 | 3.4 | 3.5 |
| B path, 7.4 s | 1.2 / 4.3 | 3.3 / 7.2 | 4.4 | 2.7 |
| B path, 13.4 s | 1.4 / 6.1 | 3.6 / 7.3 | 5.5 | 2.8 |
| B path, 21.4 s | 1.1 / 4.9 | 3.3 / 12.0 | 3.5 | not within the hold |

The learned offset sits 0.1 to 1.9 mm above the droop at each pose, the largest gaps at the policy poses. The droop changes with pose by about 3 mm (perturbed) against about 1 mm (nominal). The perturbation adds about 2 mm of droop at the policy poses and about 0.7 mm at the pick pose, and reduces it at home (0.3 against 1.0 mm). Settling after a move is 2.7 to 3.5 s.

### 3c. Policy A and B (seed 1, bridge off), correction on and off, perturbed physics (`data/logs/stage2h/policy/`)

| Variant | Correction | Offset peak left (mm) | Final \|C\| (mm) | Clip ticks (pos / rot) | Hand error mean (mm) | Nominal, same run (`stage2f`): hand error (mm) |
|---|---|---|---|---|---|---|
| A | on | 3.39 | 2.18 | 0 / 0 | 0.17 | 0.10 |
| A | off | 0 | 0 | 0 / 0 | 1.82 | 1.07 |
| B | on | 30.16 | 16.07 | 0 / 21 | 30.25 | 30.16 (peak 27.5, clip 18) |
| B | off | 0 | 0 | 0 / 0 | 40.21 | 39.74 |

Note: the perturbed policy runs wrote their summary JSON into the same output folder as the unperturbed runs, and the policy test names files by variant, seed and correction, not by perturbation, so the perturbed JSON overwrote the earlier unperturbed JSON in `data/logs/stage2h/policy/`. The unperturbed numbers above are from `data/logs/stage2f/policy_nobridge_filter/` (earlier round). The perturbed per-tick logs are the `*_pert.npz` files.

B's result is mixed with the climb of item 1, as the brief notes: the offset peak (30 mm) is the climb, not the model error; the model error adds about 0.7 mm to A's hand error with correction off and does not change B's hand error.

### 3d. Held run, perturbed, correction on

Outcome: **PLAN_FAILED**: segment 1 (navigate to the beaker), "ARMS timed out after 30 s". The logged arms stayed at the warm-up pose (left joint 4 at 0.17 rad, its buffer edge); the source target stayed at zero for the whole run; the arms never reached home (joint 4 home 2.2 rad). The nominal held run (same shim, same seed) passes the same phase. I did not trace why the source target stayed at zero under the perturbation. So the held run's offset at the pick pose is not available; the static pick pose (3a/3b) is the only pick-pose number.

### Gravity model (item 3 check)

Logged in the run info of every perturbed run: `gravity_model_is_separate_copy: true`; the gravity term's forearm mass 0.6157 kg (nominal), the physics forearm mass 0.7080 kg. Bodies: forearm = `openarm_{side}_link5` (the body carried by joint 5, the link after joint 4), upper arm = `openarm_{side}_link3` (the body carried by joint 3, the link after joint 2). Inertia tensors are not changed by the perturbation.

## Caveats

- **Process rule.** The climb analysis (`climb_perturb_diag.py climb`) loaded MuJoCo while one queued policy run was still running (about a minute, no crash). Everything after that ran with no other simulation.
- **Overwritten policy JSON** (see 3c).
- Teleop: the bridge is on in the teleop baseline (its default), so the reference is the bridge output; the right arm is included.
- The static holds are a reduced loop (no bridge, no sequencing of the policy); the held run is the full loop.
- The climb mechanism is from a regression and a reproduction of the layer's expected joints, not from a closed-loop test with the lag term removed.

## Questions for David (not decided)

1. **Test that would settle the climb.** Make the expected joints follow the command the arm actually receives (REF plus dq, i.e. the filter of the command, not the source) in a test copy, and rerun policy B with the correction on. If the climb stops, the offset's own lag is the cause. This is a layer change, so I have not made it.
2. **Held-run failure under the perturbation.** Should I trace why the source target stays at zero in the first phase (the warm-up and capture in `src/staging_source.py`)? It is the only held-run item I could not answer.
3. **Model error size.** The deliberate error gives a 1 to 2 mm extra droop, and it is enough to break the held run's first phase. Is that the intended test (a stress on the arms-ready check), or should the perturbation be smaller?
4. **Teleop drift.** The reference explains only 1 to 3 mrad of the 15 to 18 mrad gap; the remaining part is the predictor against the arm's staircase-and-ramp response. Is that the predictor question you want to take forward?
