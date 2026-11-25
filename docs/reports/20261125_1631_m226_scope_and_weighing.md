# Release freeze scope (M322) and the lift-off weighing under the arm mass error (2026-11-25)

Branch `safety-filter`. One MuJoCo process at a time. Runs in `data/logs/replay_check_20261125/` (`m226s_*`, git-ignored). Nothing pushed. `docs/progress.md` not touched.

Commit: `e92c3f2` (`src/sag_layer.py`, `src/control_loop.py`, decision row M322 in `docs/mpvi_decisions.md`, SF-138 in `docs/design_plan.md`). Analysis only for part 2: no source change.

Written for: David (decision 1 check and decision 2 weighing analysis).

## Plain-language answer

**1. The freeze follows the open catch now, and the checks pass.**

- The sag estimate keeps its value from the moment the open-catch helper catches an open and starts the ramp-out, until the gripper opens or the open is cancelled. A ramp to zero from the weigh helper's cancel or cap branch no longer freezes it.
- Held nominal: SUCCESS. The estimate is frozen from 75.09 s to 75.47 s, which is the open-catch window (the ramp-out itself is the existing ramp rule, 74.09 to 75.09 s).
- Held perturbed: SUCCESS. The earlier failure (PLAN_FAILED at the pick's UP) is gone.
- `held_mass_ramp_test`: 41 of 41 pass, including 12b.
- `mpvi_pipeline`: 14 of 14 pass, exit 0, replay bit-identical over 12,567 ticks.
- So all four checks pass, and I committed as you asked (`e92c3f2`).

**2. Which weigh branch fired, and why.**

- **Nominal:** lift-off. The pad rose 0.3 mm over 5 ticks in a row at 0.1250 kg of raw held mass (tick 9490, t = 64.66 s). The estimate was 0.1218 kg, after subtracting the rise divided by the hand height per kg (0.09535 m/kg).
- **Perturbed (as the harness ran it):** the cap branch, at 1.00 kg (t = 81.79 s). The mass ramp went to 0.9997 kg, but the pad never rose: the largest rise was −0.006 mm. So the held mass was set to zero.
- **Correction to my earlier note:** the perturbed run took the cap branch, not the cancel branch. The holding verdict stayed on throughout the lift.
- **The cause of the perturbed failure is the harness, not the arm.** In perturbed mode the gravity term is computed with an unperturbed copy of the model. The held-mass ramp is keyed to the physics model, so the gravity term never saw the held mass: its held torque was zero on every joint for the whole run. With the held term restored, the perturbed lift-off works (see below).

**3. With the held term restored, a 10 to 15% arm mass error does not make lift-off fail.** Lift-off fires on the same tick in both runs (tick 9490). The raw mass at lift-off is 0.1274 kg perturbed against 0.1250 kg nominal, and the estimate is 0.1242 kg against 0.1218 kg. That is a 2.4 g, or about 2%, shift in the estimate. The lift-off test needs the pad to rise 0.3 mm, and in both runs the pad rises about 2.5 mm per kg, so the mass at which it fires is set by that response.

**4. Would it fail on a real arm with a 10 to 15% mass error?** Not from the arm's mass error alone, in this model. The test fails only if the pad rises less than 0.3 mm per kilogram of held compensation, because then the ramp reaches the 1.0 kg cap first. Here the rise is about 2.5 mm/kg, so the margin is about eight times. A real arm with friction or a much softer response could fail that way. I have not tested that, and the sim does not model it.

## Four points, new scope (left arm, est in mrad J1..J7)

| point | nominal (SUCCESS) | perturbed (SUCCESS, cap branch, held term not reaching the gravity term) |
|---|---|---|
| before weighing (62.20 s) | −0.70, 1.59, 1.67, −0.65, 9.52, −11.00, 7.66 | −4.03, −2.04, −0.13, 1.54, 10.45, −10.83, 7.60 |
| end of weighing (64.67 s nominal; 81.78 s perturbed cap) | −0.70, 1.60, 1.68, −0.64, 9.52, −11.01, 7.67 | −4.03, −2.04, −0.13, 1.54, 10.45, −10.83, 7.60 (frozen by the existing ramp rule) |
| before ramp-out (74.07 s) | −4.15, 0.91, −0.73, 2.84, 3.91, 2.09, −14.85 | not applicable: no held mass ramp-out in this run |
| at release (`place: LIFT`, 75.49 s nominal; 93.70 s perturbed) | −4.10, 0.90, −0.72, 2.81, 3.94, 1.97, −14.77 | −15.18, 1.17, −3.38, 10.57, 0.31, −0.88, −32.75 |

- Hand offset |C_p| nominal: 6.13 mm before ramp-out, 6.08 mm at release. The old scope gave 6.12 mm at release; the new scope holds the estimate through the open, so the release value is almost unchanged.
- Freeze windows, nominal: `held_ramp` 62.21 to 75.08 s (668 calls), `open_hold` 75.09 to 75.47 s (58 calls).
- Freeze windows, perturbed: `held_ramp` 62.17 to 82.78 s (3,093 calls), `open_hold` 92.69 to 93.07 s (58 calls). The cap's ramp-to-zero at 81.79 s does not freeze, which is the point of the new scope.

## Weighing log (the lift-off phase)

| quantity | nominal | perturbed (harness, held term absent) | perturbed (held term restored, analysis only) |
|---|---|---|---|
| lift starts (settle done) | 62.21 s | 62.17 s | 62.17 s |
| held-mass ramp | 0 to 0.1250 kg | 0 to 0.9997 kg | 0 to 0.1274 kg at detection |
| max pad rise | 0.304 mm | −0.006 mm | 0.304 mm |
| lift-off counter (≥ 0.3 mm, 5 ticks) | fires at tick 9490 | never above 0 | fires at tick 9490 |
| cap (1.00 kg) | not reached | reached at 81.79 s: held set to zero | not reached |
| cancel (holding verdict off during the weigh) | no | no | no |
| estimate | 0.1218 kg (hand height per kg 0.09535 m) | none | 0.1242 kg |
| outcome | SUCCESS | SUCCESS (cap) | SUCCESS (lift-off) |

Pad rise rises roughly linearly with the held-mass ramp in both successful runs: 0.3 mm at about 0.125 kg, so about 2.5 mm per kg of held compensation.

## Technical block

- **Freeze change.** `SagLayer.open_hold` (default False) is set in `sag_correct` before every correction: `catch[s].state in ("ramp", "opening")` (`src/open_catch.py` `OpenCatch.step`). The skip rule is `elif self.open_hold: skip = "open_hold"`, after `ramp_running` (`held_ramp`). `held_mass_ramp.changing` (M226's flag) is still set by `set_target` to zero and read by the trace and `held_mass_ramp_test`; the sag no longer reads it.
- **Checks.** `held_mass_ramp_test`: exit 0, 41 PASS, 0 FAIL (12a, 12b, 12c PASS). `m226s_nominal`: exit 0, SUCCESS. `m226s_perturbed`: exit 0, SUCCESS. `m226s_mpvi_pipe`: exit 0, 14/14 checks, replay bit-identical 12,567 ticks.
- **The harness defect.** `climb_perturb_diag.py` (`--perturb`) computes the gravity term with `NOMINAL_MODEL` (an unperturbed copy). `held_mass_ramp` records are keyed by `id(model)`, so `mass_at(nominal copy, side)` finds no record and returns 0. Evidence: `held_tau_L2` in the perturbed `liftoff_ticks.csv` is exactly 0 for the whole lift, while the nominal run's rises to −0.051 N m at 0.1165 kg. The rebuild_s3 perturbed run (SUCCESS, Stage 3) had the same defect.
- **Analysis-only fix.** `weigh_wrapper.py` with `ALIAS_RAMP=1` makes `held_mass_ramp._record` fall back to the physics model's record for that hand. Output `m226s_perturbed_alias` (exit 0, SUCCESS): lift-off at tick 9490; `held_tau_L2` −0.0476 N m at 70 s, 0 at 80 s (after the cap).
- **Weigh logic** (`src/weigh_helper.py`): IDLE → STILL (holding and still verdict) → SETTLE (joints slow for `settle_dwell_s`) → LIFT. In LIFT: `start_rate` at `liftoff_rate_n_per_s / G` (0.5 N/s, about 0.051 kg/s); each tick `liftoff_check` counts consecutive ticks with rise ≥ `liftoff_rise_mm` (0.3 mm), firing at `liftoff_hold_ticks` (5); then `stop_rate`, `set_target(estimate)`. Cap: `m_now ≥ liftoff_max_mass_kg` (1.0 kg): `stop_rate`, `set_target(0)`. Cancel: holding goes false during LIFT: `_cancel` (`stop_rate`, `set_target(0)`).
- **Estimate** (`src/weight_estimate.py` `liftoff_estimate`): mass at detection minus rise / hand height per kg. Nominal: 0.1250 − 0.00031 / 0.09535 = 0.1218 kg. Perturbed (restored): 0.1274 − 0.00031 / 0.09535 ≈ 0.1242 kg.
- **Limits of this analysis.** One seed; the perturbation is a link-mass error only (forearm +15% and upper arm +10%, from `climb_perturb_diag.py`), not a beaker or gripper error and not joint friction. The helper's own log (`weigh_events.json`) shows the cap line; the per-tick logs are from `weigh_ticks.npz`.

## Next steps (not done)

1. Decide whether the perturbed diagnostic should key the held-mass record to the physics model (a harness fix in `experiments/climb_perturb_diag.py`), so that future perturbed runs need no wrapper. Not changed here.
2. Lift-off margin on a real arm: a joint-friction or softer-response model, to find the rise per kilogram at which the test starts to hit the 1.0 kg cap.
