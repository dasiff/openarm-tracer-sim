# Phase S: held-object identity, sag trim freeze, reversal release, layered removal check

Date: 2026-11-19. Branch `safety-filter` (worktree `~/projects/openarm-tracer-sim-sf-d`). Not pushed (per the brief).
Written for: David (decisions needed are in section 6).

## 1. Summary

| Item | What the brief asked | Status | Commit |
|---|---|---|---|
| S1 | Unidentified held object and generic box; identity chain; detector swap; remove the finger-gap capsule | Built, tested | 677dbb0 |
| S2 | Pad match rule: overlap, nearest to palm forward, behind-hand rejection, width warning | Built, tested | 677dbb0 |
| S3 | Stop release on source reversal, against the last different command; no overshoot | Built, tested | daf1f88 |
| S4 | ObjectDetector interface, SimObjectDetector stand-in, config switch | Built, tested | b37506a |
| S5 | Sag trim per-arm freeze replaces the "vla" source gate | Built, tested | c2fec3d |
| S6 | Remove layered resolver; revert 5438aea and 93477e6 | Already done in history (SF-108 and two reverts); verified, no new code removal needed | 677dbb0 (docstrings, test) |

Full regression on the final state: **33 of 33 exit 0**, summary `data/logs/phS_reg_20261119_124334.summary`.

## 2. Things the brief got wrong or that were already done

- **Worktree.** The brief names `~/projects/openarm-tracer-sim-safety-filter`, but that worktree has `safety-filter-speed` checked out (9e6718e). Branch `safety-filter` is checked out at `~/projects/openarm-tracer-sim-sf-d`, which is where all commits went.
- **S6 was already done.** Commits 3fd6925 ("Revert 93477e6") and 437c951 ("Revert 5438aea code") exist, and the layered resolver was removed in ba435a2..e5ddcda (SF-108). I did not revert again. Verified by `experiments/safety_filter_no_layered_test.py` (6 of 6).
- **"Phase P audit"** is not in the repo (no document names it). Not used.

## 3. Per-item notes

### S5: sag trim per-arm freeze (SF-109, commit c2fec3d)
- Each arm's trim holds (no integration; C kept, not zeroed; still applied to the command) after **3 commands in a row** that differ from the previous command by at most **1e-9 rad** on every joint.
- **Tick count chosen: 3** (about 20 ms at 150 Hz). One or two repeated targets between chunk updates do not freeze it.
- Removed: the `"vla"` source gate, `MANIPULATION_SOURCES` in the sag layer, the M264 takeover zeroing, `manipulation()`.
- Kept: `cmd["manip"]` in the control loop, still used by the safety filter's `process()`. Only the sag layer stopped reading it.
- Tests: `experiments/sag_layer_test.py` 17 of 17 (S5 a to f, plus the old cases re-pointed to a changing sweep, since constant targets now freeze). `held_mass_ramp_test` exit 0.

### S4: ObjectDetector (SF-110, commit b37506a)
- `src/safety_filter/object_detector.py`: `FingerState`, `ObjectDetector` (abstract, `identify(arm, finger_state, world)`), `SimObjectDetector` (nearest non-fixture object to the pads, named only within `sim_radius_m` = 2 cm, PROVISIONAL), `DETECTORS` registry, `make_object_detector()`.
- Config: `OBJECT_DETECTOR = {"implementation": "sim", "sim_radius_m": 0.02}`.
- Test: `experiments/safety_filter_object_detector_test.py` 9 of 9.

### S3: reversal release (SF-111, commit daf1f88)
- `BrakingLayer._track_source()`: per arm, compares each source command with the last **different** command (any joint more than `src_diff_tol_rad` = 1e-9 rad away). A reversal is a new different command whose step opposes the step that brought the arm to the last different command.
- A reversal seen while braking is held as pending. The stop releases on the first tick where the latched pair no longer closes on the new command (so the arm never moves back toward the obstacle).
- Measurements (test scenario, 1 mrad per tick source, stop latched inside the stop zone):
  - stop held through 40 repeats: yes;
  - reversal starts at tick 586; release at tick 595 (9 ticks later, the time the reversed command needed to clear the stop point);
  - pre-S3 code gives the same release tick (595), so this scenario does not distinguish old from new; the tracker tests (1a to 1c) do;
  - lowest sent joint minus the final target: +1.81e-3 rad (no overshoot);
  - largest sent step 0.000619 rad vs largest commanded step 0.001 rad;
  - peak tool speed 0.0464 m/s (cap 0.2 m/s).
- Test: `experiments/safety_filter_release_reversal_test.py` 9 of 9.

### S2: pad match (SF-112, commit 677dbb0)
- Candidates: graspable, non-fixture objects whose shape is within `HELD_MATCH_EXTRA_M` (1 cm; the existing name, now an overlap tolerance) of the segment between the finger pads (radius = finger radius).
- Forward of the palm only: the object's centre must lie ahead of the palm along the palm-to-pad direction. Behind the hand never counts.
- Among candidates the nearest to the palm (forward distance of the centre) wins. The M289 rule took the nearest to the pad midpoint by centre distance; the test shows the two differ (near beaker wins here, far beaker is nearer to the midpoint).
- Width check: width across the pads of the matched object vs finger gap; warning if off by more than `HELD_WIDTH_TOL_M` = 2 cm. Never rejects.

### S1: unidentified object (SF-113, commit 677dbb0)
- Identity chain: (1) task-plan name from the policy command, (2) pad match, (3) unidentified.
- Generic box: square section of the finger gap; bottom at the work surface under the pads (the highest work-surface top below the pads whose footprint holds the pad midpoint); top at grip height plus half the table-to-gripper distance. Centred on the pads in x and y, axis-aligned.
- Exempt from its support surface (`PlacedShape.support`, set by the held tracker). Checked against everything else (tested: other fixtures, hotplate as held-surface, another object, the other hand, the torso).
- Detector: asked once with the finger gap. A name swaps the box for the object's real shape: the world object's shape if the filter has it, else the catalog's, centred at the pads. Logged as a `swap` event.
- Display label "identifying object": set by the tracker while the detector runs (`display_label(side)`). **Not drawn on the viewer.** The sim stand-in answers in the same tick, so the label would be visible only with an asynchronous detector.
- The finger-gap capsule fallback is removed: a held box replaces it.
- Released unidentified box becomes a graspable world object (`unidentified_<side>_<n>`).
- Episode annotation: `HeldTracker.events` (attach with identity, unidentified, swap, release). The plugin writes them into the run's `meta` (`held_events`) and records a per-tick `held_identity` column. **Not exercised by a full MuJoCo episode in this phase**: the tests call the tracker directly.
- Test: `experiments/safety_filter_held_identity_test.py` 24 of 24.

## 4. Existing tests updated (decided behavior changes)
- `experiments/safety_filter_touch_test.py`: "no match, no name" now expects the unidentified box, not an empty gripper.
- `experiments/safety_filter_braking_b_test.py`: three checks: the name-agrees check now allows the width warning (the fixture's 7.3 cm beaker is wider than its 3.4 cm finger gap); the two "empty gripper" checks now expect the unidentified box.
- `experiments/sag_layer_test.py`: M264 cases replaced by S5 cases; integration cases send changing targets.
- `experiments/safety_filter_resolver_test.py`: removed the stale "Needs Kong's licensed wheel ... for the elbow swing" note.
- `src/safety_filter/link_rules.py`: docstring no longer describes the removed elbow swing and hand slide. The categories stay (kong_style uses them for its shares, SF-43 to SF-45).

## 5. Not verified / not done
- **MuJoCo pipeline baselines were not re-run.** S5 changes the sag trim for every arm command of every source (the gate is gone). Pipeline output is expected to change; `experiments/baselines/` and `data/cc_baseline/` need a run before any push.
- No full MuJoCo episode was run for the held-object identity or the episode annotation (section 3, S1).
- The "identifying object" label is not drawn on the viewer.
- Nothing pushed. No `Co-Authored-By` lines added.
- Untracked files left alone: `.venv` (symlink to the main repo's venv) and `docs/reports/20261118_1655_braking_phaseL_vs_M_fresh_vm.md` (not from this phase).

## 6. Decisions needed from David
1. **S5 freeze count (3 commands, 1e-9 rad).** Chosen by me; a count of 2 would react faster, 4 would filter more noise.
2. **S5 pipeline effect.** Every source's arm commands are now corrected; the old "vla only" design was there to stop windup during navigation. Confirm the freeze is enough, and run the baselines.
3. **S3 release timing.** A reversal releases only once the reversed command clears the stop point (9 ticks in the test). Confirm, or say the stop should release on the reversal tick even inside the stop zone (a safety question).
4. **S3 scope.** The Phase I commanded-motion rules still compare raw with the last sent command. Should they use the last different command too?
5. **Pad match "nearest to palm".** Measured as the forward distance of the object's centre; an object whose centre is behind the palm but whose body reaches the pads is rejected.
6. **Provisional values** (PROVISIONAL in config): overlap tolerance 1 cm, width warning 2 cm, sim detector radius 2 cm.
7. **Unidentified box on the floor.** When no work surface lies under the pads, the box's bottom goes to z = 0 with a warning.
8. **Safety-filter `manip` flag.** `MANIPULATION_SOURCES = ("vla",)` still decides the filter's manipulation flag. Only the sag gate was asked to go; say if the filter's source gate should go too.
9. **Display.** Should the viewer draw "identifying object"? It needs an asynchronous detector to ever show.

## 7. Commits (branch safety-filter, local, not pushed)
- 7290d3c Phase S: progress entry (plan, findings)
- c2fec3d Phase S5: sag trim per-arm freeze (SF-109)
- b37506a Phase S4: ObjectDetector (SF-110)
- daf1f88 Phase S3: reversal release (SF-111)
- 677dbb0 Phase S1/S2/S6: held-object identity, layered verification (SF-112, SF-113, SF-114)

Diff against 0c96d55 (the start): 21 files, 1056 insertions, 147 deletions.

## 8. Test results
- Full safety-filter regression on the final state: 33 of 33 exit 0 (`data/logs/phS_reg_20261119_124334.summary`; logs `data/logs/phS_reg_20261119_124334.<test>.log`).
- Earlier run on the S5/S4/S3 state: 31 of 31 exit 0 (`data/logs/phS_reg_20261119_123444.summary`).
- New tests: `safety_filter_object_detector_test` 9/9, `safety_filter_release_reversal_test` 9/9, `safety_filter_held_identity_test` 24/24, `safety_filter_no_layered_test` 6/6, `sag_layer_test` 17/17.

## 9. David's answers (2026-11-19)

Recorded as SF-115 (`docs/mpvi_decisions.md`, `docs/design_plan.md`).

- D1 keep 3 commands and 1e-9 rad.
- D2 confirm the S5 change; run the baselines before any push. Run: manual scenario, pre-S5 commit 7290d3c and HEAD, compared with the CURRENT manual reference (section 10).
- D3 keep the 9-tick ramp-down (the braking profile does its job).
- D4 yes, unify the R2/R3 commanded-motion rules to the last-different-command comparison, as a separate follow-up task (not done here).
- D5 SF-112 matches the build.
- D6 accept all provisional values.
- D7 accept the z = 0 floor fallback with a warning as a placeholder.
- D8 what the manip gate still does: the flag only sets the plug-in's `manip_started`, which `src/cat_demo.py` reads to decide when the made-up objects join the world. No filter check depends on it. No change made.
- D9 defer the viewer label.

## 10. Baseline check before push (D2, 2026-11-19)

Logs: `data/logs/phS_base_*`, `data/logs/phS_tp_*`, `data/logs/phS_held_*` (summaries `phS_base_20261119_144254.summary`, `phS_tp_20261119_145106.summary`, `phS_held_20261119_144446.summary`).

| Scenario | Reference | Pre-S5 (7290d3c) vs reference | HEAD (eee4005) vs reference | Pre-S5 vs HEAD | HEAD repeat |
|---|---|---|---|---|---|
| manual (84 rows, 23 columns) | CURRENT `manual_reference_20261103.csv` (e127479) | exact, max diff 0 | exact, max diff 0 | identical | byte-identical |
| teleop (78 rows) | none (NO_CURRENT) | n/a | n/a | identical to 9 decimals | byte-identical |
| policy (84 rows) | none (NO_CURRENT) | n/a | n/a | identical to 9 decimals | byte-identical |
| held 200 g pick-and-place | CURRENT `held_200g_20261104_m258` (e50815d) | NOT RUN: aborts at t = 57 s, `KeyError: 'C_p'` in the launcher | not finished: no output for about 26 min, CPU-bound, stopped | n/a | n/a |

What this shows:
- The arms move in the teleop and policy runs (joint ranges 0.03 to 3.4 rad), and the CSV prints nine decimals, so the trim changes these runs by less than 1e-9 degrees. Most likely the gravity feedforward removes the sag error; not traced.
- The held launcher reads `mpm.TRACE["left"]["C_p"]`, which the scripted trim wrote before M262 (Nov 5). Since M262 moved the trim into the control loop, this launcher cannot run on any later code. The held baseline was therefore stale before Phase S.
- **Open, unexplained:** the HEAD held run printed no status line in about 26 minutes (CPU-bound at 100%, 4.1 GB), while the pre-S5 run printed 176 lines in 5.5 minutes and aborted at t = 67 s. The stall appeared only in the held launcher, not in manual, teleop or policy (12 to 60 s each). Not diagnosed. A faulthandler stack dump of HEAD's held run would show where it stalls (the attempt was denied; not retried).

Recommendation: do not push until the held-run stall is diagnosed and the held baseline is either re-registered from current code or explicitly set aside by David.

## 11. Diagnosis of the held-run "stall" (2026-11-19; no fix made)

Logs: `data/logs/diag_held_head_20261119_161755.log` (no flushes, killed), `data/logs/diag2_held_head_*` (flushed print), `data/logs/diag4_pre_*` and `data/logs/diag4_head_*` (same seed).

Launch command (held 200 g, scratch worktree of HEAD 3d72ffe; the launcher copy is `data/bounce_check_20261103/run_mass.py` with its ROOT line set to the worktree and one `faulthandler.dump_traceback_later(45, repeat=True)` line added at the top):

    cd <scratch worktree of 3d72ffe> && OPENARM_SIGNAL_TRACE=1 timeout 115 bash experiments/run_capped.sh <log> env MUJOCO_GL=egl timeout 90 .venv/bin/python <launcher copy> <out_dir> --liquid 0.100

Findings:
- **Where it is:** the control loop never starts within 90 s (control ticks reached: 0). The run is in the stance search: `apply_stance_selection` (stance_select.py) -> `iter_verified_stances` (edge_sampling.py) -> `check` -> `full_check.check` -> `_place_window` -> `solve_logged` -> `_solve_stance` -> `_solve_pose_point` -> `_admissible` -> `arm_distances` -> `_min`, which calls `mujoco.mj_geomDistance` for each geom pair.
- **Last function entered (dump):** `full_check._min`, inside `arm_distances`, inside `_admissible`. Two dumps (45 s and 90 s) show the same depth.
- **Memory:** flat, not growing. `ps` samples during the 26-minute run: 4,138,908 kB at 20:08 and again at 20:28. The 90-second runs peaked at 4.10 to 4.16 GB.
- **Errors:** none. No Python traceback. The 90-second runs end by `timeout` (exit 124).
- **Why there was no output:** stdout is block-buffered, and the stance search prints its `[sampler]` lines with plain `print`. The killed run (no flushes) had 0 `[sampler]` lines while its dump showed 8 stance-search frames. The run with a flushed print showed 2. The pre-S5 run had output because it ended in an exception, which flushes stdout.
- **Why HEAD looked slower:** the sampler seed is `time.time_ns() % 2**32` when unset (edge_sampling.py, iter_verified_stances), so each run tries a different candidate order. Pre-S5 seed 4294874296 gave candidate 1 at (0.8954, 1.0863, pedestal 0.1606): FAIL in 30.15 s. HEAD with the same pinned seed gave the same candidate, the same FAIL reason, in 35.26 s. Per-candidate behaviour matches; the runs differ by seed.
- **Interpretation (not confirmed):** the 26-minute HEAD run was most likely still searching candidates, about 30 to 40 s each, when it was killed. The pre-S5 run with the same seed reached the loop after about 5 minutes of search. Confirming this needs HEAD run to the loop with the pinned seed, unbuffered (`python -u`), for about 6 minutes; the 90-second limit did not allow it.

Not done: no change to the branch. Scratch edits (a flushed print in `stance_select.check`, the seed pin at `edge_sampling.py` line 410) were made only in scratch worktrees, which have been removed. Steps 2 and 4 of the brief (tick print, prints inside the loop) were not applicable, since the loop never starts in 90 s; the stance-search print and the stack dumps were used instead. The stale launcher (`KeyError 'C_p'`) is unchanged.

Recommendation: before any push, run the held HEAD case with seed 4294874296 unbuffered for about 6 minutes and confirm it reaches the loop like pre-S5 does. Then decide the held baseline: the launcher is stale since M262, and its seed should be pinned if it is to be regenerated.

## 12. Held 200 g run on HEAD with the pinned seed (2026-11-19, diagnostic; no fix made)

Setup: commit a14b423 (code identical to 3d72ffe). Scratch edits only, removed afterwards: the sampler seed pinned to 4294874296 in `iter_verified_stances`, and the launcher's trim read changed to `SAG_TRACE`/`SAG_LAYERS` (the stale `mpm.TRACE["C_p"]` read is what crashed the pre-S5 run). Command: `OPENARM_SIGNAL_TRACE=1 timeout 660 bash experiments/run_capped.sh <log> env MUJOCO_GL=egl timeout 600 .venv/bin/python -u <launcher copy> <out_dir> --liquid 0.100`. Log: `data/logs/held10_head_20261119_163146.log`; outputs `data/logs/held10_head_out_20261119_163146/`. Launch 16:32:01, exit 1 at 16:38:53 (6 min 52 s wall clock).

| Question | Answer |
|---|---|
| Reached the control loop | Yes. Recording started 16:35:13 (about 3 min 12 s after launch). |
| Stance search | Candidate 1 FAIL in 44.5 s (0.8954, 1.0863, pedestal 0.161); candidate 2 FAIL in 0.04 s; candidate 3 PASS (38.5 s) and dry run PASS (19.8 s), pedestal 0.0704. |
| Pick | Attempt 1 (t = 69.5 s): OVER 4.5 s, DOWN 3.7 s, IN 3.0 s. IN not reached in 15 s (3.3 mm and 1.1 deg from the waypoint). Recovery OPEN, BACK, HOME. |
| Pick attempt 2 (t = 102.1 s) | Same sequence; IN failed again with the same numbers. Retries used up (retry_max 1); outcome PLAN_FAILED. |
| Carry | Not reached. |
| Place | Not reached. |
| Stops, braking, held-object identity | None in the log. The safety filter is off by default (SAFETY_FILTER_PLUGIN_CFG mode "off"), so no braking and no held tracking ran. |
| Sag freeze events, left | Freeze 30.98 s, resume 69.47 s (pick start); freeze and resume pairs at 91.52, 92.28/92.71, 92.73/93.72, 100.89/100.97, 101.86/102.08, 124.11/124.17; freeze at 124.81 s, never resumed. |
| Sag freeze events, right | Freeze 30.98 s, never resumed. |
| Final trim, left | p = (-13.4, 16.9, -33.7) mm, |p| = 40.0 mm (the limit, sag_max_m 0.04); r = (0.015, -0.083, -0.053) rad, |r| = 0.100 rad (the limit, sag_max_rad 0.1). |
| Final trim, right | p = (24.8, -17.3, -26.2) mm, |p| = 40.0 mm; r = (-0.070, 0.064, -0.033) rad, |r| = 0.100 rad. |
| Frozen at exit | Both arms. |
| Frozen during carry | Not applicable (no carry). The left trim was integrating during pick attempt 1 (69.5 to 91.5 s). |
| Errors | No traceback. The sag layer logged `solver_no_solution`: left 12,850 times (first at t = 1.400 s, last at 125.267 s), right 4,480 times (first at 1.400 s, last at 31.260 s). |

Solver failures by window: left before 30.98 s 4,437; during the frozen window 44; during pick attempt 1 (69.5 to 91.5 s) 3,307; after 91.5 s 5,062. Right before 30.98 s 4,437; after 43. The exception text is not recorded (the shim logged only the kind).

Interpretation (not verified):
- Before S5 the sag layer ran the solver only for manipulation commands. S5 runs it for every arm command. From the first control tick (t = 1.400 s, with the trim still about zero) the solver fails on the staging commands. When it fails, the layer returns the source targets, so no correction is applied.
- The trim is integrated before the solver call, so it keeps integrating on the failing ticks and reaches its limits on both arms.
- The solver keeps failing during the pick (3,307 failures in attempt 1), so the pick runs uncorrected. The 3.3 mm miss at IN fits that, but this link is untested.
- Not measured: whether the pick succeeds before S5 with the same shim and seed. The pre-S5 run never reached the pick (its launcher crashed at t = 57 s).

Recommended next steps (none done): run pre-S5 (7290d3c) with the same shim and seed to compare the pick; add the exception text to the shim's log. Do not push until the pick succeeds on HEAD or the S5 gate is changed.
