# Perturbed-harness fix (held-object link), earlier results from the defective runs, and the consistent comparison (2026-11-25)

Branch `safety-filter`. One MuJoCo process at a time; every run in its own folder under `data/logs/` (git-ignored). Nothing pushed. No production code changed (`src/` untouched in this round). `docs/progress.md` not touched.

Written for: David (lab notebook: the consistent comparison of the three sag cases, and the corrected perturbed held run).

## Plain-language answer

**1. The harness fix works.** The perturbed held run now weighs the beaker the same way a live run does. The deliberate model error is unchanged: the gravity term still uses the unperturbed model, and the held-object weight now comes from the same record a live run reads.

- Weighed mass: lift-off at tick 9490, at 0.1274 kg raw, giving an estimate of 0.1242 kg (nominal: 0.1250 kg raw, 0.1218 kg estimate).
- Outcome: SUCCESS.
- Estimate at the four points (left arm, mrad, joints 1 to 7): see the table in the technical block. The estimate is frozen from the ramp-out through the release (74.74 to 75.12 s).

**2. Earlier results that came from defective runs.** Every perturbed held run before this fix had zero held-object compensation. Those results are listed in the technical block. Their outcomes stand as records, but they are not evidence about weighing under the model error. The perturbed teleop, move-and-stop, static and policy runs had no held object, so the defect could not reach them.

**3. The consistent comparison (measurement only, bridge on, safety filter off, model error on, 30 s stream plus 3 s hold).** The table in the technical block gives every measurement, with the mean and the range over seeds for B, C and D.

Main reading:

- **Hold error (a):** with no correction, the hand is 12 mm (B), 8 mm (C) and 11 mm (D) off 1 s into the hold. The production rule brings that to 10, 7 and 10 mm. The nudging rule brings it to 7, 5 and 9 mm. Policy A is close to zero for both rules.
- **Corrections (c):** the nudging rule makes corrections of 11 to 15 mm, against about 5 mm for the production rule. Neither rule reaches the clip.
- **Whole-run maxima (b):** the largest position error is in motion, not in the hold, in every run. In 1 of 10 perturbed runs (off), 5 of 10 (nudge) and 2 of 10 (measure) it is at about 1.8 s, the start of the stream; in the rest it comes later in the motion. The maxima are 26 to 53 mm and do not separate the cases well.
- **Reference:** policy A on normal physics gives the same hold error as A with the model error (1.9 against 1.5 mm off, 0.0 against 0.0 mm measure). The deliberate error did not raise the steady droop of A at the hold.

## The four points, fixed perturbed held run (`data/logs/harness_fix_20261125/perturbed_held`)

| point | t (s) | est J1..J7 (mrad) |
|---|---|---|
| before weighing | 62.15 | −4.03, −2.05, −0.14, 1.53, 10.44, −10.81, 7.58 |
| end of weighing (lift-off, weighed) | 64.66 | −4.03, −2.04, −0.13, 1.54, 10.45, −10.83, 7.60 |
| before ramp-out | 73.74 | −9.68, 0.34, −1.57, 5.68, 4.48, 3.32, −14.76 |
| at release (`place: LIFT`) | 75.14 | −9.65, 0.35, −1.56, 5.67, 4.50, 3.24, −14.71 |

Freeze windows: ramp-out `held_ramp` 73.74 to 74.74 s; open `open_hold` 74.74 to 75.12 s. Between the weigh and the ramp-out, the estimate moved by the hold's own learning (the beaker is held, no ramp runs); that is the existing rule, unchanged.

## Earlier reported results from runs with the defect

The defect: in perturbed mode the gravity term reads the held-mass record with the unperturbed copy (no record found), so the held-object torque was zero. It affects only runs that have a held object (weighing or catch ramps). Results from those runs:

| report | result as reported | status |
|---|---|---|
| `20261124_1700_stage2_climb_teleop_perturbation.md` §3d | perturbed held run PLAN_FAILED, segment 1 (ARMS timed out) | affected; not re-run |
| `20261124_1830_stage2_measure_sag_test_copy.md` §4 | perturbed held run PLAN_FAILED, zero source target (`stage2h/held_pert.npz`) | affected; not re-run |
| `20261124_1930_stage2_start_stall_range_buffer_rescore.md` (table row "held perturbed") | held perturbed range numbers | affected |
| `20261124_2120_staged_rebuild_stage2_measure_rule.md` criterion C | perturbed held SUCCESS | affected |
| `20261124_2300_stage2_final_rerun_stopped.md` | perturbed held PLAN_FAILED at CLOSE; decision-3 ablation (held_offset_ablation.py, same climb harness) SUCCESS, the basis of the decision-3 conclusion | affected: the decision-3 conclusion rests on these runs |
| `20261125_0100_stage2_teleop_offset_trace.md` | criterion C failed for the gripper stop check (perturbed) | affected |
| `20261125_0300_stage2_every_tick_recheck.md` | criterion C PASS (perturbed SUCCESS) | affected |
| `20261125_0330_stage3_uniform_shrink.md` | held perturbed SUCCESS (`rebuild_s3/held_perturbed`) | affected |
| `20261125_0415_stage4_filter_on_stopped.md` | perturbed held PLAN_FAILED with the filter on; filter-event table | affected |
| `20261125_1458_replay_fix_and_m226.md` | perturbed held PLAN_FAILED under M226 (pick UP) | affected; its cause (cap ramp-out) is real under the rule, but the scenario had no held compensation |
| `20261125_1631_m226_scope_and_weighing.md` | perturbed weighing: cap branch, SUCCESS; the "harness, not the arm" conclusion; the 10 to 15% answer (from the analysis run with the record aliased) | affected in its perturbed table; the analysis-run answer stands and matches the fixed run |
| `docs/design_plan.md` SF-127 (criterion C) and SF-131 (filter on, nominal and perturbed) | decision rows quoting those held results | affected |

Not affected: the perturbed teleop runs (`20261125_0100`, the move-and-stop and teleop tables), the static five-pose runs (`climb_static.py`), the policy streams (no held object), and the Stage 2 relaxation and bridge tests.

## Technical block

### Harness fix (commit with this report)

- `experiments/perturb_error.py` (new, shared): `perturb_model` (moved unchanged from `climb_perturb_diag.py`: forearm mass +15 %, centre 10 mm, upper arm +10 %) and `link_held_mass(copy, physics)`. The link patches `held_mass_ramp.mass_at` once so that a lookup made with the unperturbed copy reads the physics model's record. Lookups with any other model object are unchanged.
- `experiments/climb_perturb_diag.py`: imports the shared function; after the copy is created for the gravity term it calls `link_held_mass(nom, model)`.
- Verification run: `harness_fix_20261125/perturbed_held` (exit 0, SUCCESS). `held_tau_L2` at 70 s is −0.0476 N·m (zero in the defective run). Lift-off at tick 9490 (0.1274 kg raw, estimate 0.1242 kg), identical to the analysis-only run with the record aliased (`m226s_perturbed_alias`).

### Test-only nudging rule

- `experiments/sag_nudge_rule.py` (new): the rule of commit 8e45fac, as a function that replaces `SagLayer.correct` in the test process only: motor-filter expected joints, Jacobian error, leak exp(−dt/5 s) (the 8e45fac "5 s fade"), integrator with `PICK_PLACE_CFG` gain and clips (3.0 per s, 0.04 m, 0.1 rad, unchanged), held-ramp hold, and the MINK damping (`MINK_CFG`, unchanged). Not the default; selected only with `policy_stream_test.py --rule nudge`.
- `experiments/policy_stream_test.py`: new options, defaults unchanged: `--perturb on|off` (default off), `--hold-s` (default 0: the final target is held after `--seconds`), `--rule measure|nudge` (default measure). Default reproduces the Stage 3 production summary exactly (checked: policy B seed 1, bridge on, 30 s: ticks, position, rotation, hand error and steady droop identical to `rebuild_s3/policy/B_s1_on.json`).

### The comparison

- Runs: `data/logs/policy_compare_20261125/<case>/<stream>_s<seed>_on/` (perturb on, bridge on, safety filter off, 30 s stream, 3 s hold from t = 30.0 s). Cases: `off` (`--correction off`), `nudge` (`--rule nudge`), `measure` (default). Streams: A (seed 1), B, C, D (seeds 1 to 3). Reference rows: `ref_*/A_s1_*` (normal physics, same cases). 34 runs, all exit 0; queue `compare_queue.sh`, progress `progress.txt`.
- Measurements from each run's per-tick CSV (`experiments/policy_compare_table.py`): reference = bridge-accepted target minus the correction (the stream test's definition). (a) error 1 s after the hold starts (t = 31.0 s); (b) largest error over the run and when; (c) largest |C_p| (mm), and clip ticks from the summary.

Table (position mm; rotation mrad in the second row of each block). B, C, D: mean (range over three seeds).

| measurement | stream | off (no correction) | nudge (8e45fac, test only) | measure (production) |
|---|---|---|---|---|
| a. position error at 1 s into the hold (mm) | A | 1.5 | 0.1 | 0.0 |
| | B | 12.2 (10.6 to 14.5) | 7.1 (6.0 to 9.0) | 10.1 (9.3 to 11.4) |
| | C | 7.6 (4.0 to 10.1) | 5.3 (1.5 to 8.1) | 7.4 (1.8 to 11.1) |
| | D | 10.6 (7.5 to 15.6) | 9.4 (6.2 to 12.0) | 10.4 (5.6 to 15.7) |
| | A, normal physics (ref) | 1.9 | 0.0 | 0.0 |
| a. rotation error at 1 s (mrad) | A | 14.8 | 0.7 | 0.2 |
| | B | 74.6 (62.4 to 86.2) | 50.3 (37.7 to 63.9) | 66.3 (53.3 to 81.5) |
| | C | 37.2 (16.2 to 59.2) | 25.5 (5.5 to 45.5) | 35.4 (9.8 to 62.7) |
| | D | 52.8 (29.4 to 95.7) | 38.2 (16.3 to 73.2) | 48.8 (23.0 to 92.6) |
| | A, normal physics (ref) | 16.9 | 0.6 | 0.2 |
| b. largest position error over the run (mm) | A | 26.3 | 29.5 | 26.8 |
| | B | 35.0 (29.2 to 38.1) | 29.5 (29.5 to 29.5) | 32.2 (26.8 to 36.3) |
| | C | 37.3 (36.2 to 39.3) | 32.7 (29.5 to 36.3) | 35.9 (33.1 to 37.5) |
| | D | 50.1 (45.6 to 52.8) | 44.0 (39.6 to 48.6) | 46.8 (41.4 to 50.6) |
| | A, normal physics (ref) | 26.6 | 29.7 | 27.0 |
| b. largest rotation error over the run (mrad) | A | 62.6 | 65.7 | 63.3 |
| | B | 132.0 (128.2 to 135.2) | 101.3 (94.6 to 106.2) | 126.0 (119.5 to 130.8) |
| | C | 130.1 (121.8 to 135.8) | 107.0 (102.3 to 114.0) | 126.3 (120.8 to 132.0) |
| | D | 141.5 (131.4 to 147.2) | 127.4 (119.7 to 137.8) | 137.7 (134.0 to 144.3) |
| | A, normal physics (ref) | 63.2 | 66.0 | 63.7 |
| c. largest correction, hand displacement (mm) | A | 0.0 | 6.9 | 2.2 |
| | B | 0.0 | 11.3 (10.5 to 12.4) | 5.5 (5.0 to 6.2) |
| | C | 0.0 | 11.5 (10.8 to 11.9) | 5.2 (5.0 to 5.7) |
| | D | 0.0 | 14.9 (13.6 to 15.8) | 5.5 (5.4 to 5.7) |
| | A, normal physics (ref) | 0.0 | 6.7 | 3.2 |
| c. ticks at the position clip (and rotation clip) | all streams and cases | 0 | 0 | 0 |

- **Where the whole-run maximum (b) happens.** Always in motion (the hold error is smaller). For 1, 5 and 2 of the 10 perturbed runs (off, nudge, measure) it is at about 1.8 s, the start of the stream; the rest are later in the motion. The per-run "when" column is in `data/logs/policy_compare_20261125/table.md`.
- **Caveats.** One deliberate error (link masses, the stated sizes). Three seeds for B, C, D; one for A. The hold is 3 s after 30 s of stream, with the start-up motion in the stream as in the Stage 3 runs.

### Earlier results from defective runs: data

Affected reports are listed in the plain-language section above, with their own run folders.

## Commits and next steps

- Committed with this report: `experiments/perturb_error.py`, `experiments/climb_perturb_diag.py` (harness fix), `experiments/policy_stream_test.py` (options; default unchanged), `experiments/sag_nudge_rule.py` (test-only rule), `experiments/policy_compare_table.py` (table), `experiments/weigh_log_wrapper.py` (read-only logging wrapper used for the weighing logs), and this report.
- Not done: re-running the affected earlier held runs (their outcomes are listed, not re-run); a decision on whether the 8e45fac rule should be kept as a test reference.
