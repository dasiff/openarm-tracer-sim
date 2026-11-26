# Stage 2: "measure the sag directly" test copy at four time constants, and the perturbed held run (2026-11-24, David's d0bc brief)

Branch `safety-filter` (HEAD ad5d0c8 before this round). The production layer is unchanged. The test copy is a separate module (`src/sag_measure.py`), chosen by config (`SAG_RULE_CFG`, default `integrator` = production). Nothing pushed. No T was chosen; David will choose.

## Plain-language answer

1. **The test copy removes the climb, but it corrects less.** With the measure-the-sag rule nothing reaches the 40 mm / 100 mrad clip, and the joint offsets stay small (dq peak 10–33 mrad against 25–140 mrad for production). The held-still zero-mean control is bounded for every T (no growth). The price: the hand error comes down less. Policy B's hand error is 37.6–38.8 mm for T = 0.25–2 s, against 30.2 mm for production (40 mm uncorrected). Policy A is the best case (0.03–0.19 mm against 1.07 mm uncorrected, and production's 0.10 mm). Teleop's hand error is 36–49 mm on the left arm against 35 mm for production (52 mm uncorrected). Shorter T corrects more and climbs more; longer T corrects less. At the five poses, the estimate misses the droop by 1.5–2.6 mm on average, against 1.0 mm for production. So no T is clearly best; the choice of T trades correction against error.
2. **Why the perturbed held run never moved.** In the perturbed run, the first arms command for the left arm's joint 4 lies just below its lower joint limit (0 rad). The loop clips it to 0, and from then on the source's target is pinned at exactly zero for the whole run. The staging source moves its arms only while the target the loop reports back equals the target it sent, within a tolerance. The clipped and unclipped values never match, so its move never advances and the arms wait at the warm-up pose until the 30 s timeout. The nominal run never clips (its joint 4 starts rising at 1.41 s). The separate gravity-model copy is not the cause: it changes only the feedforward; the warm-up and the pose capture are the same in both runs (the first 2 s match within 0.003 rad), and no errors are logged. The pre-clip value is not logged, so the sign of the first command is inferred, not read.

## Technical block

### 1. The test copy (`src/sag_measure.py`, class `MeasureSagLayer`)

Per arm, per joint, every tick, as specified:
- a. `predicted_j`: the same first-order motor filter (same tau_j), driven by the command the actuator receives (control minus the gravity offset tau_ff/kp), dq included.
- b. `sag_measured_j(t) = predicted_j(t−1) − measured_j(t)`.
- c. `sag_est_j`: first-order low-pass of `sag_measured_j`, time constant T, unity gain.
- d. `dq_j = sag_est_j`, added to that joint's command; if the hand move exceeds 40 mm or 100 mrad, dq is scaled down uniformly.
- e. Skips as production: no update on the first tick, while the bridge has the arm suspended, on a non-finite value; the estimate holds during a skip.

Why the rule does not feed back on dq: the predicted joints follow the full command, so predicted minus measured is the arm's shortfall relative to the command it was given; the dq the layer added cancels out of that difference.

Selection: `SAG_RULE_CFG = {"rule": "integrator", "measure_T_s": 0.5}` in `src/task_config.py`. `src/control_loop.py` builds its layers through `make_sag_layer` (one line changed; the default returns the production `SagLayer`, so the default path is unchanged).

### 2. One row per T (production row first)

Policy B, A and C: seed 1, bridge off, nominal physics; the hand error is measured against the accepted target; "ticks at clip" for production is from the production clip; for the measure rule the hand-space clip never bound (dq stayed small). Held-still: zero-mean sweep, bridge emulated. Five poses: perturbed physics, correction on (estimate against droop, mean absolute error over the five poses). Teleop: bridge on, nominal physics, hand error against the accepted reference, mean over the run. Per-T detail is in `data/logs/stage2k/tables.md` (policy and held-still) and `data/logs/stage2k/teleop_hand.md`.

| Rule | Policy A hand error on (mm) | Policy B hand error on (mm) | Policy C hand error on (mm) | Max dq peak, policy (mrad) | Clip ticks, policy | Held-still zero-mean max \|C_p\| / \|C_r\| (mm / mrad), bounded? | Five poses: mean abs. error vs droop (mm) | Teleop hand error left / right (mm) | Teleop dq peak left (mrad, max joint) |
|---|---|---|---|---|---|---|---|---|---|
| Off (no correction) | 1.07 | 39.74 | 31.79 | 0 | 0 | — | — | 52.2 / 67.8 | 0 |
| Production (integrator) | 0.10 | 30.16 | 28.97 | 140 (B) | 18 (B), 9 (C) | 7.8 / 22.5, bounded (no growth); rotation above the 10 mrad bound | 1.0 | 35.0 / 41.0 | 136 |
| Measure, T = 0.25 s | 0.03 | 37.62 | 31.46 | 33 (B) | 0 | 9.0 / 26.1, bounded; above 10 mrad | 2.4 | 36.1 / 52.7 | 49 |
| Measure, T = 0.5 s | 0.10 | 37.63 | 30.94 | 18 (B) | 0 | 4.9 / 14.2, bounded; above 10 mrad | 2.6 | 40.5 / 55.9 | 48 |
| Measure, T = 1 s | 0.17 | 38.18 | 31.09 | 12 (B) | 0 | 2.6 / 7.4, bounded; within 10 mrad | 1.8 | 45.7 / 59.9 | 48 |
| Measure, T = 2 s | 0.19 | 38.83 | 31.40 | 10 (B) | 0 | 1.3 / 3.8, bounded; within 10 mrad | 1.5 | 48.7 / 63.2 | 34 |

Notes on the table:
- Unreachable variant (constant +0.01 rad on joint 3; informational only): production reaches 29.5 mm / 86 mrad; the measure rule reaches 34.6, 26.1, 13.5 and 6.9 mm for T = 0.25, 0.5, 1 and 2 s. Its growth flag is set for T = 0.5, 1 and 2 s and for production; the measure rule at T = 0.25 s has a large steady value without the growth flag.
- "Bounded" is the no-growth check. The rotation bound of 10 mrad (set earlier, unchanged here) is passed only at T ≥ 1 s.
- Policy A's dq peak (10 mrad on the late joints at T ≥ 1 s) sits at a value that may be a buffer edge; not checked here.

### 3. Five poses, the estimate and the droop, and the settle times (correction on, perturbed physics)

Per pose, the estimate at the end of the hold (|C_p| in mm, the hand equivalent of the joint dq) against the droop of the correction-off hold (`data/logs/stage2k/static_*.md`):

| Pose (droop, mm) | Production | T = 0.25 s | T = 0.5 s | T = 1 s | T = 2 s |
|---|---|---|---|---|---|
| home (0.33) | 1.41 | 1.03 | 0.87 | 1.70 | 1.32 |
| pick (2.60) | 3.44 | 7.65 | 5.64 | 5.62 | 5.40 |
| policy B, 7.4 s (3.29) | 4.35 | 6.58 | 8.14 | 5.38 | 3.72 |
| policy B, 13.4 s (3.56) | 5.49 | 5.97 | 5.68 | 3.67 | 4.21 |
| policy B, 21.4 s (3.34) | 3.46 | 3.80 | 1.17 | 0.81 | 0.73 |

Settling (time from the end of the move until |C_p| is within 0.1 mm of its end value): production 2.7–3.5 s at the moving poses; the measure rule 1.7–3.5 s (the measure rule settles faster at some poses, but its end value is further from the droop). Production's estimate is the closer of the five on average.

### 4. The perturbed held run: trace of the zero source target

Compared: the nominal held run (logged this round, `data/logs/stage2i/held_nominal/`: outcome SUCCESS, exit 0) and the perturbed run (`data/logs/stage2h/held_pert.npz`, PLAN_FAILED).

| Quantity (left arm) | Nominal | Perturbed |
|---|---|---|
| Source target change over the first 3 s | 450 changes, first at 1.41 s | none; constant |
| Joint 4 source target | rises: 0.005 (2.0 s), 0.025 (2.5 s), 0.071 (3.0 s) | pinned at 0.000 for the whole run |
| Joint 4 measured | 0.185 (2.0 s), 0.177 (2.5 s) | 0.184 (2.0 s), 0.176 (2.5 s) |
| Loop message "arm_targets['left'] clipped to the joint limits" | none | printed twice, at the first arms command |
| Outcome | SUCCESS | PLAN_FAILED: "ARMS timed out after 30 s" |

Mechanism (from the code, `src/commands.py` clip; `src/staging_source.py` ARMS phase):
1. The loop checks every arms command against the joint limits (`validate_command`) and clips to them. Joint 4's range is [0, 2.443] rad (`models/robot/combined_robot.xml`).
2. The staging source advances its ARMS move only while `np.allclose(applied targets, the targets it last returned)`.
3. In the perturbed run its first command for joint 4 is just below 0 rad, so the clip sets it to 0; what the loop reports back (0) and what the source sent (slightly negative) never match; the move parameter stays at zero; the command stays at its start; ARMS times out at 30 s.
4. In the nominal run the first joint-4 command is inside the range and rises from the first tick.

Not verified: the raw pre-clip value (not logged), so the sign is inferred from the clip and from the constant zero target. The root is the perturbed rest pose at joint 4, a hair below its lower limit at the first command; this is a setup interaction, not the gravity copy. I did not fix it.

Gravity copy check: the feedforward uses the nominal model (logged forearm mass 0.616 kg in the feedforward, 0.708 kg in the physics); the warm-up, the rest-pose capture and the first 2 s of the joints match the nominal run; no errors. The copy changes the feedforward only.

## Code and runs (for the record)

- New: `src/sag_measure.py` (the test copy and the factory), `SAG_RULE_CFG` in `src/task_config.py`, the factory call in `src/control_loop.py` (one line; default unchanged).
- Scripts: `experiments/climb_perturb_diag.py` (run mode takes `--rule` and `--T`; logs the clip scale; `heldstill` and `static` modes), `experiments/climb_static.py` (static holds with the rule; held-still control), `experiments/measure_tables.py` (tables), `experiments/measure_teleop_hand.py` (teleop hand error and dq peaks, MuJoCo forward kinematics).
- Runs (each its own folder; nothing overwritten): `data/logs/stage2j/` (held-still, static, first policy attempt; the measure-rule policy runs logged no ticks because the logging wrapper missed the test copy's `correct` and was then fixed: their summary JSON is valid, their logs are not), `data/logs/stage2k/` (the corrected policy and teleop runs; the production and off rows copied from stage2j), `data/logs/stage2i/held_nominal/` (the nominal held run, logged).
- Process: all simulator runs ran one at a time; the table and held-run trace analyses read files (no simulator) except the teleop hand error, which ran after the queues finished.

## Questions for David (not decided)

1. **Choose T.** The table trades correction against error: T = 0.25 s corrects most in A and C but climbs in B; T = 2 s corrects least and keeps every joint small. Which trade-off do you want?
2. **The measure rule at the pick pose.** The estimate is 5.4–7.7 mm against a 2.6 mm droop at every T. Not explained here (it may be the hold's approach direction, or friction in the joints). Do you want that traced before a T is chosen?
3. **The joint-4 start (held run).** The first arms command for joint 4 sits just below its lower limit in the perturbed case. Options: keep the perturbation and fix the start in the staging source, or accept that the held run needs the nominal physics. I have not changed anything.
