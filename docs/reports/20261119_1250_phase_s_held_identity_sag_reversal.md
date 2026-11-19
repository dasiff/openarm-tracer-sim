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
