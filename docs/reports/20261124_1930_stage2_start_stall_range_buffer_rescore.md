# Stage 2: start-pose stall, range-based buffer, at-rest misses, smoothing-time rescore (2026-11-24, David's d0bc brief)

Branch `safety-filter` (worktree `openarm-tracer-sim-sf-d`), HEAD 7d355b8 before this round. Nothing pushed. No smoothing time T chosen. One MuJoCo process at a time; each run in its own folder under `data/logs/stage2m/` (this round's runs) and `data/logs/stage2n/` (analysis of them).

## Plain-language answer

1. **The start-pose stall is fixed.** The sources now see exactly the arm targets they sent. The loop no longer clips arm targets to the joint limits before the sources see them. The bridge's buffer clamp is now the only joint-range limit. The perturbed held run (forearm mass +15 %, upper arm +10 %) completes with SUCCESS; before the fix it failed with "ARMS timed out after 30 s". The first left joint-4 command before any clip was −2.17e-5 rad, just under its lower limit of 0 rad. That tiny negative value is what made the old clip hold the arm at the warm-up pose.

2. **The range-based buffer is built.** Each joint keeps 5 % of its own range on each side, capped at 10°. The margins are 10, 10, 9, 7, 9, 4.5 and 9° for joints 1–7, the same on both arms. Against the old fixed 10°, joint 6 (90° range, 4.5° margin) stops clamping in the held run: 7.56 s down to zero. Joint 4 clamps a little less (2.30 s down to 2.02 s). Joint 2 is unchanged. In teleop joint 4 clamps 1.70 s instead of 1.84 s. The joint-4 clamp in the held runs (about 2 s) is at the start pose: the source's target sits below the 7° lower buffer edge while the arm rises from rest. That is the job decision 1 moved into the bridge.

3. **The at-rest misses of the measure rule sit in the wrist (joints 5–7).** The wrist's friction band is 10 mrad (0.1 Nm of friction over a gain of 10 Nm/rad), so a wrist can rest anywhere within 10 mrad of its command. In 6 to 8 of the 15 wrist cells per measure rule, the measured sag sits at ±10 mrad, the band edge (the estimate does so in 0 to 8 cells). The wrist command offset differs from the at-rest droop by 3 mrad or more in 8 to 14 of those 15 cells. The wrist is held near its target by its band, so the miss hardly shows in where the arm settles: for the measure rules, the residual miss at the end of each hold is 1.1 mm or less for every joint at every pose, and most are under 0.5 mm (production reaches 1.38 mm at the pick pose). Joints 1–4 have a band of 1.4–1.7 mrad, and their estimates at the pick pose are within 0.3–1.0 mrad of the droop. No joint target sat on either buffer at any of the five poses, so the buffer is not a cause.

4. **Smoothing-time rescore: one table, no T chosen (below).** Only the five static holds give valid (a) and (b) numbers. Policy B has no stops: its source is a continuous path and never holds for 0.5 s. The teleop run's one pause is a tracking loss in which nothing was logged for 2.6 s, so no logged tick falls 1 s after the stop. Its (a) and (b) cells are therefore "no 1 s sample", not numbers.

## Technical block

### 1. Start-pose stall (decision 1)

- **Mechanism (confirmed).** `validate_command` (`src/commands.py`, arm clip) clipped the first left-arm command for joint 4 from −2.173e-5 rad to 0. The staging source's ARMS phase advances only when the applied targets equal the targets it last returned; they never matched, so the ARMS phase timed out at 30 s. The first five logged joint-4 commands are all negative (−2.173e-5 to −2.093e-5 rad) and the joint-4 lower limit is 0 rad.
- **Fix.** `src/control_loop.py`: the source-side validation uses `limits_src`, which omits the arm limits (`limits_src = {k: v for k, v in limits.items() if k != "arm"}`; the arms validation calls `validate_command(cmd, limits_src)`). The buffer clamp in `src/bridge_clamp.py` is the only joint-range limit. Test `experiments/source_view_test.py`: 3/3 PASS (the sources see exact targets; the old validation would have clipped the same command; the bridge clamp limits joints 3, 5 and 6 into their buffers; the test's clamped indices are [2, 4, 5], 0-based).
- **Pre-clip value.** Logged in `info["arms_commands_raw_first60"]` by `experiments/climb_perturb_diag.py` (`validate_logged` wrapper, first 60 commands). Left joint 4, first command: −2.1731288577569947e-05 rad.
- **Held runs** (production layer, correction on, seed as in the brief):

| run | perturbation | buffer | outcome |
|---|---|---|---|
| `data/logs/stage2m/held_pert_prod` | forearm +15 %, upper arm +10 % | range | SUCCESS, no plan failure (was PLAN_FAILED before the fix) |
| `data/logs/stage2m/held_nom_range` | none | range | SUCCESS |
| `data/logs/stage2m/held_nom_fixed` | none | fixed 10° | SUCCESS |

### 2. Range-based buffer (decision 2)

Per-joint table, both arms (the two arms have the same joint ranges, `models/robot/combined_robot.xml`). Margin = min(5 % of range, 10°) for the range rule; 10° for the fixed rule.

| joint | range (°) | range-rule margin (°) | fixed margin (°) |
|---|---|---|---|
| 1 | 280 | 10.0 | 10 |
| 2 | 200 | 10.0 | 10 |
| 3 | 180 | 9.0 | 10 |
| 4 | 140 | 7.0 | 10 |
| 5 | 180 | 9.0 | 10 |
| 6 | 90 | 4.5 | 10 |
| 7 | 180 | 9.0 | 10 |

Clamp counts: updates (50 Hz bridge updates) and seconds, from `bridge_clamp_update_counts` and `bridge_clamp_seconds` in each run's info. Target ticks are in brackets for the held runs.

| run (buffer) | left j2 | left j4 | left j6 | right j2 | right j4 |
|---|---|---|---|---|---|
| held nominal, fixed 10° | 18 / 0.36 s [51] | 115 / 2.30 s [343] | 378 / 7.56 s [1128] | 21 / 0.42 s | 115 / 2.30 s [343] |
| held nominal, range | 18 / 0.36 s [51] | 101 / 2.02 s [301] | 0 / 0 s | 21 / 0.42 s | 101 / 2.02 s [301] |
| held perturbed, range (production) | 20 / 0.40 s [56] | 101 / 2.02 s [301] | 0 / 0 s | 19 / 0.38 s [55] | 101 / 2.02 s [301] |
| teleop, correction off, fixed 10° | 12 / 0.24 s | 92 / 1.84 s | 0 | 93 / 1.86 s | 92 / 1.84 s |
| teleop, correction off, range | 12 / 0.24 s | 85 / 1.70 s | 0 | 93 / 1.86 s | 85 / 1.70 s |
| teleop, correction on, range (production layer) | 12 / 0.24 s (+ j3: 1 / 0.02 s) | 88 / 1.76 s | 0 | 93 / 1.86 s | 94 / 1.88 s |

Teleop fixed-buffer left joint 3 has one update (0.02 s) that the range rule does not clamp (its margin is 9°). Test `experiments/bridge_clamp_test.py`: 8/8 PASS (margins, edges, exact pass-through, one clamped joint does not change the others, a target on the edge is not counted).

### 3. At-rest misses of the measure rule (decision 3; rule not changed)

Full per-joint tables for every pose and rule: `data/logs/stage2n/static_breakdown.md` (generated by `experiments/static_breakdown_table.py` from `data/logs/stage2m/static_*.md`; analysis only).

**A labelling correction.** The md column "hand contribution: droop (mm)" comes from `climb_static.py`'s correction-on end state (target minus settled position with correction on). It is the residual miss, not the at-rest droop of part (a). The script's header now says so (the existing md files still carry the old header). The `sag` and `dq` hand columns are as labelled.

**Friction band** (frictionloss 0.1 Nm on every joint, divided by kp: 70, 70, 70, 60, 10, 10, 10 Nm/rad): joints 1–3 1.43 mrad, joint 4 1.67 mrad, joints 5–7 10.00 mrad.

**Pick pose (held-run tick 9300), measure T = 0.25 s, and production, correction on at the end of the hold.** Values in mrad; the miss is the residual (target minus settled) in mm of hand.

| joint | droop (a) | measured sag (T=0.25) | estimate (T=0.25) | dq (T=0.25) | residual miss (T=0.25, mm) | residual miss (production, mm) |
|---|---|---|---|---|---|---|
| 1 | −4.42 | −5.01 | −5.15 | −5.15 | 0.08 | 0.28 |
| 2 | −3.39 | −3.91 | −4.05 | −4.05 | 0.05 | 0.70 |
| 3 | −1.75 | −2.75 | −2.76 | −2.76 | 0.00 | 1.38 |
| 4 | +1.28 | +0.96 | +0.96 | +0.96 | 0.00 | 0.12 |
| 5 | +4.19 | −8.86 | −8.92 | −8.92 | 0.01 | 0.36 |
| 6 | +4.31 | +10.00 | +10.00 | +10.00 | 0.00 | 0.46 |
| 7 | +6.86 | −10.00 | −10.00 | −10.00 | 0.00 | 0.10 |

Residual miss, correction on, end of each hold, mm per joint (from the generated file; "largest" is the biggest single joint):

| pose | rule | largest residual (mm) | joint |
|---|---|---|---|
| home | measure T=0.25 / 0.5 / 1 / 2 | 0.59 / 1.10 / 0.57 / 0.43 | 7 / 7 / 4 / 3 |
| pick | measure T=0.25 / 0.5 / 1 / 2 | 0.08 / 0.04 / 0.13 / 0.53 | 1 / 2 / 2 / 1 |
| policy B 7.4 s | measure T=0.25 / 0.5 / 1 / 2 | 0.05 / 0.08 / 0.39 / 0.74 | 3 / 1 / 6 / 4 |
| policy B 13.4 s | measure T=0.25 / 0.5 / 1 / 2 | 0.00 / 0.04 / 0.69 / 0.62 | 1 / 2 / 7 / 7 |
| policy B 21.4 s | measure T=0.25 / 0.5 / 1 / 2 | 0.01 / 0.12 / 0.07 / 0.45 | 1 / 1 / 1 / 1 |
| (all poses) | production | 1.38 (pick) | 3 |

**Wrist cells (joints 5–7, 15 per rule, five poses):**

| rule | \|measured sag\| at the band edge (≥ 9.98 mrad) | \|estimate\| at the band edge | estimate opposite in sign to droop (\|droop\| ≥ 1 mrad) | \|dq − droop\| ≥ 3 mrad |
|---|---|---|---|---|
| production | 0 | 2 | 6 | 10 |
| measure T=0.25 | 8 | 8 | 5 | 12 |
| measure T=0.5 | 8 | 4 | 6 | 14 |
| measure T=1 | 6 | 1 | 6 | 13 |
| measure T=2 | 6 | 0 | 5 | 8 |

**What causes the misses (from the numbers above; consistent with the data, not fully proven):**

- **Stiction on the wrist.** A wrist joint can stop anywhere within its 10 mrad band of the command. The command offset on joints 5–7 is therefore wrong by 3 mrad or more in 8 to 14 of 15 cells per rule, while the residual at rest stays at 1.1 mm or less for the measure rules. At rest the rule's miss is invisible in the settled position. It appears when the wrist moves (the offset's hand move and the hand error after a stop, parts (a) and (b) of the rescore).
- **The measured sag reads the band edge.** On the wrist, the measured sag (predicted minus measured) sits at ±10 mrad in 6 to 8 of the 15 cells for the measure rules and never for production. The reading is saturated, so it does not carry the droop value, and the estimate follows it. The estimate's sign is wrong in 5 to 6 cells per rule. I have not traced what sets the exact ±10 mrad value. The measure copy has no clip on the sag itself (`src/sag_measure.py`, checked); the wrist actuators have a ±7 Nm force range (not the limit); the bridge is not in the static holds.
- **Joints 1–4 are measured well by the measure rule.** Their bands are 1.4–1.7 mrad, and at the pick pose the estimate is within 0.3–1.0 mrad of the droop; the residual there is ≤ 0.13 mm.
- **The buffer is not a cause.** No joint target at any of the five poses was on the fixed or the range buffer (every row "no / no").
- **Hand contributions of the offset** (the dq column, T = 0.25, pick): joint 1 2.47 mm, joint 2 1.35, joint 3 1.04, joint 4 0.36, joint 5 0.99, joint 6 1.85, joint 7 1.89. The wrist accounts for about 4.7 mm of the offset's hand move at the pick pose, most of it not reflected in the residual (stiction).

### 4. Smoothing-time rescore (decision 4; `experiments/rule_rescore.py`)

One table. Production and each T. Run: `data/logs/stage2n/rescore_gapcheck.md` (log `rescore_gapcheck.log`; the earlier `rescore.md` in the same folder is superseded: it counted the teleop gap as a pause).

| rule | a. hand error 1 s after the stop, five poses (mm) | a. policy B stops (mm) | a. teleop pauses (mm) | b. offset vs at-rest droop at the stop, five poses (mm) | b. policy B stops (mm) | b. teleop pauses (mm) | c. peak offset, policy B (mm) | c. ticks at clip, policy B | c. peak offset, teleop (mm) | c. ticks at clip, teleop | c. peak offset, five poses (mm) | c. ticks at clip, five poses |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| production | 4.83 | none (0) | no 1 s sample (1 pause) | 2.43 | none (0) | no 1 s sample (1 pause) | 34.5 | 0 | 40.0 | 137 | 40.0 | 9 |
| measure T=0.25 s | 4.06 | none (0) | no 1 s sample (1 pause) | 2.97 | none (0) | no 1 s sample (1 pause) | 24.3 | 0 | 40.0 | 140 | 35.6 | 0 |
| measure T=0.5 s | 2.92 | none (0) | no 1 s sample (1 pause) | 1.35 | none (0) | no 1 s sample (1 pause) | 14.6 | 0 | 40.0 | 108 | 20.3 | 0 |
| measure T=1 s | 2.19 | none (0) | no 1 s sample (1 pause) | 0.63 | none (0) | no 1 s sample (1 pause) | 8.5 | 0 | 40.0 | 48 | 10.6 | 0 |
| measure T=2 s | 2.47 | none (0) | no 1 s sample (1 pause) | 0.20 | none (0) | no 1 s sample (1 pause) | 4.7 | 0 | 29.3 | 0 | 6.6 | 0 |

Notes on the table:
- **(a) and (b) five poses** are the mean of the five static holds. The home hold (first, from a set pose, no ramp) gives 11.15 mm (production), 6.41, 3.40, 2.18 and 2.42 mm for the rules in order. It looks like a start transient, not a settled stop. Without it the (a) means are 3.25 (production), 3.47 (T=0.25), 2.80 (T=0.5), 2.19 (T=1) and 2.48 (T=2) mm. The table keeps all five, as defined.
- **Policy B: none.** The policy B source is a continuous path (`experiments/policy_stream_source.py`, variant B). The longest hold of its source is 0.07 s; the stop rule needs 0.5 s. Its B cells have no events by construction. The c. columns for B are peak and clip over the run and do not need a stop.
- **Teleop: no 1 s sample.** The run is 5 s long. At about 5.4 s the scripted operator loses tracking ("Teleop: tracking lost — holding last pose", `data/logs/stage2m/teleop_on_integ.log`). The controller then goes to manual, and nothing is logged from about 5.4 s to 7.96 s (a 2.56 s gap in `left_t`). The pause detector counts that gap as one pause, but no logged tick is within 50 ms of 1 s after the stop, so the (a) and (b) cells are not measured. The c. teleop columns use the logged ticks only.
- **c. teleop** clip counts (137, 140, 108, 48) are from the teleop runs with correction on and the range buffer. Not compared with the earlier stage-2k numbers (different buffer and run).

### 5. Tests and checks run this round

- `experiments/bridge_clamp_test.py`: 8/8 PASS (exit 0).
- `experiments/source_view_test.py`: 3/3 PASS (exit 0).
- `experiments/rule_rescore.py` (MuJoCo, pad forward kinematics): exit 0; output `data/logs/stage2n/rescore_gapcheck.md`.
- `experiments/static_breakdown_table.py` (no simulator): output `data/logs/stage2n/static_breakdown.md`.
- Pipeline baselines (`experiments/baselines/`, `data/cc_baseline/`) were **not** re-compared this round. The range buffer and the source-side validation are decided changes (decisions 1 and 2), so output changes are expected; the byte-level comparison is still open.

### 6. Decision log and code

- `docs/design_plan.md`: SF-124 (source view), SF-125 (range buffer), SF-126 (test copy; T not chosen). The rows refer to this report.
- Code and scripts: `src/control_loop.py` (limits_src), `src/bridge_clamp.py` (buffer_margins, buffer_limits), `src/task_config.py` (BRIDGE_BUFFER_CFG, SAG_RULE_CFG), `src/sag_measure.py` (last_sag logging), `experiments/bridge_clamp_test.py`, `experiments/source_view_test.py`, `experiments/climb_perturb_diag.py`, `experiments/climb_static.py` (header label), `experiments/rule_rescore.py`, `experiments/static_breakdown_table.py`.
- `docs/progress.md` not touched (per the brief). Not pushed.

## Questions for David (not decided; nothing changed for these)

1. **Choose T.** The rescore table gives no T. The trade-off is in the table: shorter T corrects more and moves more of the offset into the hand; T = 2 s has the smallest offsets and the smallest five-pose distance (0.20 mm), but its hand error at rest is no better than T = 1 s (2.47 vs 2.19 mm).
2. **The home hold (11.15 mm production, 1-s value).** Count it as a settled stop, or as a start transient? It changes the five-pose (a) mean by about 1.6 mm for production.
3. **Teleop 1-s samples.** The teleop run needs to keep logging through the tracking loss, or the pause needs a longer run, before (a) and (b) can be scored for teleop.
4. **Policy B stops.** Policy B has no stops. Either define stops for the path (for example, at fixed segment ends) or drop its stop columns.
5. **The ±10 mrad wrist saturation.** Trace it (a separate test) or accept it as a known limit of the measure rule's reading on the wrist?
