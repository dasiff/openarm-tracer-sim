# Braking policy, Phase C: fixes from the Phase B review (build report)

Date: 2026-11-16. Branch safety-filter, not pushed. Written for David. Phase B is in [braking_policy_part2b_20261116.md](braking_policy_part2b_20261116.md).

## 1. Summary

- Fixes 1 to 8 are built, tested and committed (13 commits, section 2). Several needed a second and third commit once the integration runs showed what the unit tests could not.
- **Fix 9 (integration) did not pass.** With the decided configuration the pick-and-place does not complete. Two causes are now known: the stop target at the bench blocks the palm from reaching the grasp pose (decision 1), and during a violent HOLD the two arms overlapped (decision 2, the most serious).
- Two deviations from the brief, both in section 6: fix 1 is an inscribed capsule, not a flat-ended cylinder kind; fix 3 needed a change to `applied()` that the brief did not describe.

## 2. Commits (branch safety-filter, oldest first; none pushed)

| Hash | Fix | One line |
|---|---|---|
| f0372f9 | 1 | Beaker collision shape spans the cylinder's extent (segment half length 0.015 m; total 0.10 m) |
| 8ad86c0 | 2 | Palm against an allowed-contact surface: surface pair with held_world_margin_m and the surface's approach ceiling |
| acc12bf | 3 | Free motion shaped by default; `applied()` reports the sources' arm targets while the layer is on (staging stall fix) |
| beb01bc, 29da412 | 4 | Stop target margin + PUSH_STOP_BEYOND_M; midpoint removed; no-creep test |
| 91c455a | 5 | Jerk source defaults to "state"; "sent" kept behind the flag |
| 28f25b6 | 5 (fix) | "state" ramp starts from the measured speed only when it is faster than the profile (a stationary arm stalled) |
| fede7f6 | 6 | Look-ahead uses a closing component (dot product, tool point) |
| d7df3c2 | 6 (fix) | Closing component uses the velocity of the hand point nearest the obstacle |
| 58aeafe | 9 prep | Measured acceleration = least-squares slope over 5 ticks |
| c0a531c | 9 prep | Look-ahead emergency compares the stop with the margin, not the stop target |
| 5e95b5e, 095904c | 7 | Scripted grasp names the object (held_object); the route filter kept it only after the second commit |
| f169cd4 | 8 | Negative-margin experiment switch removed from the driver |

Phase C branch head: d7df3c2 (plus this report and the progress entry).

## 3. Files changed (since db6f4f0)

13 files, 510 insertions, 58 deletions. Source: `src/safety_filter/braking.py`, `src/safety_filter/touch.py`, `src/safety_filter/config.py`, `src/safety_filter/world.py`, `src/safety_filter_plugin.py`, `src/control_loop.py` (applied() gate; route filter), `src/commands.py` (held_object validation), `src/mpvi_modules.py` (grasp names the object). Experiments: `experiments/safety_filter_braking_c_test.py` (new), `experiments/safety_filter_braking_test.py`, `experiments/safety_filter_braking_b_test.py`, `experiments/safety_filter_braking_analyze.py` (Phase C checklist), `experiments/safety_filter_braking_episode.py` (experiment switch removed, speed default state).

## 4. Test results (final code)

| Suite | Result |
|---|---|
| braking_test | 0 FAIL |
| braking_b_test | 0 FAIL |
| braking_c_test (new, Phase C) | 0 FAIL |
| plugin | 0 FAIL |
| touch | 0 FAIL |
| never_touch | 0 FAIL |
| kong_style | 0 FAIL |
| resolver | 0 FAIL |
| shapes, gap_rule, structure, distance, link_rules | 0 FAIL |
| spec | 2 FAIL ("the closing right arm moves"): same as the Phase A and B baselines |
| gripper_force_test (run once, the closest check to the control loop) | 27/29: the same two failures as the Phase B commit (replay bit-identity, "three closings measured"), so pre-existing |
| sequential_push | cannot run: needs data/safety_filter/kong_single_pair_fixtures.json (missing, pre-existing) |

Phase C test coverage: beaker extent and flat-bottom contact (step 1); palm-surface pair and feed ceiling (2); free-motion shaping under a pulsed source, held target reached exactly, small target reached (3); stop target and no creep, stop inside the target does not restart (4); state source default, sent behind the flag (5); state ramp not stalled by a stationary arm, and the regression reproduced against the old formula (5 fix); closing component parallel at 2 cm no braking, diagonal uses the normal component, closest point velocity (6); held_object validation, route filter, verdict as trigger, identity from command (7); acceleration window (fix); emergency against the margin (fix).

Full suite (about an hour): not run.

## 5. Integration run (fix 9)

### 5.1 Runs

| Run | Code | Outcome | Where it stopped |
|---|---|---|---|
| brkP_plain | acc12bf-era (before the 5-fix) | PLAN_FAILED | Navigate: ARMS timed out (state ramp stalled) |
| brkQ_plain | 28f25b6 | PLAN_FAILED | Pick: CLOSE "empty grasp" (palm stopped above the grasp pose by the stop target) |
| brkR_plain | 095904c | PLAN_FAILED | Pick: CLOSE "empty grasp"; the name "beaker" now reaches the filter (150 ticks) |
| brkS_plain | 58aeafe | PLAN_FAILED | Pick: DOWN not reached (269 mm); three false emergencies (stop target used as the room) |
| brkT_plain | c0a531c | PLAN_FAILED | Pick: DOWN not reached (269 mm); four emergencies at 2.3 to 4.0 cm from the bench, speed 0.034 m/s |
| brkU_plain (final decided code, d7df3c2) | d7df3c2 | PLAN_FAILED | Pick: IN not reached (834 mm); recovery BACK "mink found no solution"; two emergencies at measured speeds 0.45 and 0.67 m/s |
| brkX (experiment: stop target = margin for all pairs) | e3998c1 + one line | PLAN_FAILED | Grasp reached: beaker held with its name (166 ticks); then "gripper did not stop within 5 s" (not in the layer's scope; not resolved) |

### 5.2 Checklist (final decided code, brkU_plain_20261116_230541)

| Item | Result | Numbers |
|---|---|---|
| Hand speed <= cap (0.20 m/s) | **No** | Measured peak 0.570 m/s; commanded (sent) peak 0.688 m/s by the frozen-state FK (1573 ticks above the cap) |
| Accel and jerk within limits, free motion | **No** | Measured peak 4.48 m/s² and 499 m/s³ (p99 jerk 50); commanded peak 49.1 m/s² (the 0.4 to 0.7 m/s excursion) |
| Accel and jerk within limits, braking | **No** | Braking-tick commanded peaks 48.4 m/s² and 14 594 m/s³ (the same excursion) |
| Hand stops at the new target and stays | **Not tested** | No stop at the target was reached in an approach to a surface before the failure; the unit test (step 4) passes |
| Beaker reaches the hotplate at ceiling speed | **Not reached** | The beaker never reached the plate (hotplate surface gap min 0.145 m, i.e. not approached) |
| No bridge override errors | **Not met** | Two overrides, at t = 60.3 s and 65.3 s, from the two look-ahead emergencies |
| No false look-ahead failures | **Not met** | Two emergencies, both at measured speeds above the cap (0.45 m/s and 0.67 m/s) after a HOLD |

Closest distances (brkU): the closest pair recorded for the left arm was **−73.6 mm (an overlap)**, left finger against the right shoulder mount, at t = 75.6 s. The arms physically intersected during the HOLD that followed the failed OVER and DOWN moves. The filter did not prevent it.

### 5.3 Phase B against Phase C

| Quantity | Phase B (brkJ_plain_sent, success) | Phase C (brkU, decided code, failed) |
|---|---|---|
| Outcome | SUCCESS (8280 calls) | PLAN_FAILED at the pick (5897 calls) |
| Measured peak speed | 0.160 m/s | 0.570 m/s |
| Measured peak accel / jerk | 1.33 m/s² / 242 m/s³ | 4.48 m/s² / 499 m/s³ |
| Commanded peak accel / jerk | 26.8 m/s² / 5421 m/s³ (pulsed, unshaped) | 49.1 m/s² / 14 594 m/s³ (the excursion) |
| Look-ahead errors | 3 (false, at the bench) | 2 (real overspeed, after a HOLD) |
| Bridge overrides | 1 (from the false emergency) | 2 |
| Closest gap of the look-ahead's emergencies | 22.6 mm (palm vs bench) | −73.6 mm at the deepest point (left finger into the other arm's shoulder mount, a separate record) |
| Held identity | pad match | command name ("beaker", 150 ticks) |

Phase B also had the 29 mm short place (brkG, decided margin). Phase C has not yet reached the place.

## 6. Deviations from the brief, and why

1. **Fix 1 is not a flat-ended cylinder.** The beaker's collision shape is the capsule of Phase B with its segment half length corrected to half height minus radius (0.015 m). Its extent equals the cylinder's (0.10 m), and its lowest point is the flat bottom, so contact with a horizontal surface is exact (test: gap equals height above the plate, 0 to 35 mm). The capsule lies inside the cylinder, so lateral contact at the bottom rim is under-covered by up to 3.5 cm. A flat-ended cylinder kind needs new distance functions for the geometry and the batched pair table. That is a larger change than the fix, so I did not build it (decision 3).
2. **Fix 3 changed `applied()`, which the brief did not describe.** The staging source advances its clock only when the loop's applied arm targets equal its own last command. A step-limited output lags the target, so with shaping on, staging never advanced (probe: raw travel 0.003 to 0.028 rad in 12 s). The fix: while the braking layer is on in mode "on", `applied()` reports the sources' arm targets (the layer shapes the way to them, it does not change them). Off and shadow are unchanged, so existing pipeline output stays byte-identical. After the fix, raw travel was 0.25, 1.46 and 0.48 rad in the same windows.
3. **Fix 5 needed a repair.** Switching the default to "state" stalled the ramp: the measured speed of a stationary arm is zero, so the ramp restarted from zero every tick and the output stayed still (replay: the output stayed 2 rad behind the source). The ramp now starts from the profile, and from the measured speed only when that is faster. A regression test reproduces the stall with the old formula (old: 0.0002 rad in 300 ticks; new: passes).
4. **Route filter (fix 7).** `_filter_by_route` rebuilt arms commands from arm targets and gripper only, silently dropping `held_object`. The name now goes with the gripper the source owns. Found only by the live run.

## 7. Open findings (not fixed)

1. **The decided stop target blocks the grasp (decision 1).** The palm's closest approach to the bench during the pick is about 2.3 cm. The stop target for a plain obstacle is margin + PUSH_STOP_BEYOND_M = 3 cm, so the look-ahead stops the palm above the grasp pose. The resulting "empty grasp" is reproduced in brkQ, brkR, brkS and brkT. The experiment with the stop target at the margin for every pair (brkX) reached the grasp, with the name attached, so the stop target is the cause of this failure. The experiment is not committed; it is a one-line change in `stop_gap` in a scratch worktree.
2. **Violent HOLD and self-collision (decision 2, most serious).** In brkU, after the OVER and DOWN moves failed, the source target moved about 4 rad away from the sent command, the resolver held (BRANCH_GUARD), and the layer held too. During the hold the sent command still moved at about 3 rad/s (one 0.022 rad step per tick) toward the far target, and the measured arms reached 0.45 to 0.67 m/s. At the deepest point the left finger was 7.4 cm inside the right shoulder mount. The look-ahead did not stop this, because a HOLD stops the layer only on its own ramp, from the measured speed (above the cap).
3. **The cap is not enforced while decelerating an overspeed arm.** When the measured speed is above the cap, the ramp starts from it, and the ramp's deceleration cannot bring the speed down to the cap in one tick. Either the cap is clamped instantly (breaking the acceleration limit) or the cap is broken for a few ticks. The layer does the second. Decision 4.
4. **"Gripper did not stop within 5 s"** after a successful grasp in the experiment (brkX). The beaker attached with its name, the left fingers stalled on it at 37.5 mm, and the stall criterion of the manipulation source did not see a stop. Not diagnosed; the layer does not command the fingers, so this is the manipulation's stall check, or the new beaker shape, to look at.
5. **The stopping formula is cautious.** With the state source, a stationary arm reads small accelerations (about 0.3 m/s² under servo-level noise, from the slope test). The formula's acceleration term then adds about 1 cm of stopping distance at 3 cm/s. The emergency is now measured against the margin, which removes the false emergencies seen at the stop target, but the formula still reads a moving palm as needing more room than the bench gap allows.

## 8. Decisions for David

| # | Decision | Why |
|---|---|---|
| 1 | Stop target for a plain obstacle (the palm on the bench): keep margin + PUSH_STOP_BEYOND_M (decided, blocks the grasp), or stop at the margin boundary like the surfaces (the experiment reached the grasp), or exempt palm-bench pairs from the look-ahead stop. | Finding 1. The pick cannot pass with the decided target. |
| 2 | How a HOLD must act on an arm that is still moving fast: a hard stop of the command at the measured position, or the S-curve from the measured speed (current). | Finding 2: the current HOLD let two arms overlap. |
| 3 | A flat-ended cylinder kind, or the inscribed capsule (current, with 3.5 cm under-cover of the bottom rim). | Fix 1 deviation. |
| 4 | Cap during deceleration of an overspeed arm: instant clamp (violates the acceleration limit) or a few ticks above the cap (current). | Finding 3. |
| 5 | Who owns the "gripper did not stop" check: the manipulation source (its stall detection after a grasp), or the layer. | Finding 4. |
| 6 | Emit the held name for a grasp only once the fingers are around the object, or as now (at the start of the close). | The name is used with the verdict; the verdict decides when the attach happens. |

## 9. Not done

- Fix 9 is not met: no run completed the pick-and-place with the decided configuration. The checklist items "hand stops at the target", "beaker reaches the hotplate at ceiling speed" were not reached in a live run. Their unit tests pass.
- The full suite (about an hour) was not run.
- The scripted manipulation's "gripper did not stop" failure (finding 4) was not diagnosed.
- The overlap (finding 2) was not investigated further.
- Phase C report commit: see the progress entry. Nothing pushed.
