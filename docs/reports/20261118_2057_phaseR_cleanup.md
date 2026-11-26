# Phase R cleanup: slide-aware elbow swing, and the sequential_push re-freeze (blocked)

Written for: David (the Phase R brief). Branch safety-filter, not pushed.

## Summary

1. **Elbow swing (done, committed 93477e6).** The layered resolver's elbow swing now aims at the arm's current hand pose after any hand slide in the same tick, not the raw command. The raw option (the SF-49 setting) is removed. It did not make the closing right arm check pass: the swing no longer undoes the slide, but the forearm contact still does not clear, and the case still holds. The left closing arm still passes.
2. **sequential_push re-freeze (not done, blocked on a decision).** The test is at 41 of 43 before and after step 1, as expected. The two failing checks are real consequences of the wider pair distance, and the one joint-level change is in the expected direction. But the fixture generator refuses to write fixtures for cases that now have more than one pair to push, so re-freezing needs a decision about that rule. Nothing was re-frozen.
3. **Full safety-filter suite (run).** 25 of 27 files exit 0. The two that exit 1 are the same two known failures: spec_test (38 of 39) and sequential_push (41 of 43).

## Worktree note

The task named `~/projects/openarm-tracer-sim-safety-filter` on branch `safety-filter`. That worktree is on `safety-filter-speed` (HEAD 9e6718e). Branch `safety-filter` is checked out in `~/projects/openarm-tracer-sim-sf-d` (HEAD 9266014), so all work was done there. A `.venv` symlink to the main repo's venv was added there (untracked, not staged). The sequential fixture `data/safety_filter/kong_single_pair_fixtures.json` (git-ignored) was copied from the safety-filter-speed worktree; its sha is a062edf0c3910ce9f1d521381372a0d1db4edd5c (written 2026-11-12 19:20). A pristine copy is in the session scratchpad.

## Part 1: slide-aware elbow swing

### Change

`src/safety_filter/resolver.py`, `_elbow_swing`: the hand target is `self.kin.hand_pose(self._state(t, q), side)`, the arm's current joints. Before, it was `_swing_hand_target`, which read the raw pose when the setting was "raw" (the spec tests set it so) and the slid target when "slid". The helper and the config key `elbow_swing_hand_target` (SF-49) are removed. The callers that set or read the key are updated: `experiments/safety_filter_spec_test.py`, `experiments/safety_filter_resolver_test.py`, `experiments/safety_filter_kong_style_test.py` (section 12), `scripts/safety_filter_kong_comparison.py`, `scripts/safety_filter_closing_arm_fix_check.py`. Decision row SF-106 in `docs/mpvi_decisions.md`; design_plan notes SF-49 as removed and lists SF-106.

For the no-slide case the new target equals the old raw target (the joints still equal the raw command for that arm), so the change only matters once a slide or an earlier swing has moved the arm in the tick.

### spec_test results

| Check | Before (9266014) | After (93477e6) |
|---|---|---|
| closing left arm moves | pass | pass |
| closing right arm moves | **FAIL** | **FAIL** |
| parked arm untouched (right, while left closes) | pass | pass |
| both closing (mirrored) | pass | pass |
| all other checks (38 of 39 in total) | pass | pass |

Totals: 38 of 39 before, 38 of 39 after; exit 1 both times. Logs: `data/logs/phaseR2/spec_before_20261118_203649.log`, `data/logs/phaseR2/spec_after_fix_20261118_203904.log`.

The right closing case after the fix, from `scripts/safety_filter_closing_arm_fix_check.py`: HOLD, reason ACCURACY, 10 rounds, methods HAND_SLIDE, 9 × ELBOW_SWING, STEP_SHORTENED. Left closing: HAND_SLIDE, 1 round.

### Why the check still fails (read-only trace, scratchpad script)

- After the slide: the hand has moved 0.410 mm from last and 7.910 mm from raw. The worst pair becomes right_link5_2 vs torso, shortfall 0.700 mm (stop line 0.5 mm).
- Each swing: the hand moves 0.000 mm, so the swing no longer pushes the hand back into the palm. The palm pair is not the worst pair in any round.
- But the forearm-to-torso shortfall only falls from 0.700 to 0.696 mm over the 9 swings. (The progress log and the SF-106 row say 0.697; the last trace value is 0.696.)
- The contact sits 14.5 to 14.7 mm from the shoulder-wrist line (elbow radius 185 mm). The elbow displacement rule is `elbow_displacement_mode: "plain"`, which config marks as a PLACEHOLDER. My reading, not proven: with that rule a 0.7 mm push moves this contact only a small fraction of a millimetre per swing.
- After 10 rounds, step shortening runs; its accuracy check compares to the raw pose (SF-36, `shortening_accuracy_target: "raw"`), and the slide has already moved the hand 7.9 mm off raw. That check fails, giving ACCURACY.

So the post-slide target is the right target, but it is not sufficient: the forearm swing has too little effect, and the shortening accuracy target then fails after a slide. Both are design questions (see decisions).

## Part 2: sequential_push

### Step a: as-is

41 of 43 (exit 1). Log `data/logs/phaseR2/seq_asis_20261118_204323.log`. Failing checks:

| Check | Result |
|---|---|
| hand_link7_vs_box_converges: output within 1e-9 rad of the saved one, same status | FAIL: max joint difference 5.2e-2 rad, status KONG_PUSH both |
| forced SHAPE_RECHECK (stop_m = -1): HOLD with that reason | FAIL: HOLD with SHORTEN_ZERO |

### Step b: what changed

Done with `touch.wake_dist` patched to 0 (the narrow pair set from the Phase R bisect) and the wide default, on the saved fixture inputs.

**hand_link7_vs_box_converges**

- Narrow set reproduces the saved output exactly (max difference 0.000e+00): 1 sweep, 1 pair pushed.
- Wide set: 103 candidate pairs (narrow: 6). Newly active pairs: 97.
- The one pair that matters, `bx | left_palm_2` (the palm against the box), was not in the narrow set. At the raw command it is 56.8 mm apart (gap rule shortfall 7.52 mm). At the saved output it is 5.21 mm short, against a stop line of 0.5 mm, so the saved output leaves a gap-rule violation. At the new output it is 0.445 mm short.
- Joint change: left arm only, largest 2.96° at joint 3 (total about 5.2e-2 rad).
- Sweeps: 10 (the round limit); 11 pushes, 10 of them on `bx | left_palm_2`; final status KONG_PUSH (all checks pass).

Reading: the direction is the one David's decision predicts (the resolver reacts to a farther pair that the narrow cutoff ignored, and the saved output was short on it). The size is not small: 3 degrees on one joint, and 10 sweeps where the case says "converges in one sweep". The case text is now wrong.

**forced SHAPE_RECHECK (stop_m = -1)**

- Narrow: HOLD SHAPE_RECHECK after 10 rounds.
- Wide: HOLD SHORTEN_ZERO at round 0. The pair log lists about 100 pairs pushed or shortened. With stop_m = -1 every active pair counts as short, so the wider set reaches the shortening path before the shape re-check.
- Still a hold with the last command returned; only the reason label differs. The test is forcing a degenerate scene, not a realistic one.

### Step c: re-freeze (blocked)

`scripts/safety_filter_kong_single_pair_fixtures.py` checks that exactly one pair needs a push in every case; if not, it prints "NOT single-pair cases" and exits 2 without writing. Under the wide distance four of the eight cases fail this check: upper_arm_vs_box (2 pairs), hand_vs_box_link7 (2), hand_link7_vs_box_converges (2), arm_vs_arm_both_closing (3). The test's fixture check still passes for three of them (their outputs are within tolerance); only hand_link7 changes.

Changing the guard, or redefining the four scenes, changes what the frozen set means. That is David's call, so step c was not done and step d (re-run) is not applicable. The fixture file is git-ignored and was never committed in this worktree, so the test cannot run from a clean checkout either.

## Full safety-filter suite (step e)

Run sequentially, one MuJoCo process at a time, after step 1 (HEAD 93477e6). Logs: `data/logs/phaseR2/suite_20261118_204723/` (one `.log` per file, `ALL_DONE` marker).

- 25 files exit 0, and no FAIL or Traceback lines in their logs.
- 2 files exit 1: `safety_filter_spec_test` (38 of 39; the known closing right arm check) and `safety_filter_sequential_push_test` (41 of 43; blocked, above).
- The 27 files: braking_b, c, d, e, f, g, h, i, j, k, l, m, braking, distance, fk, gap_rule, jit, kong_style, link_rules, never_touch, plugin, resolver, sequential_push, shapes, spec, structure, touch.

The `resolver` and `kong_style` tests, which step 1 edited, both exit 0.

## Commits

| Hash | Subject | Notes |
|---|---|---|
| 93477e6 | Phase R step 1: slide-aware elbow swing (SF-106); raw setting removed | Code, config, callers, SF-106, design_plan. Its message gives 0.697 mm; the last trace value is 0.696 mm. |
| (this commit) | Phase R step 2 (partial): sequential_push diagnosis; re-freeze blocked; report | progress.md and this report only. |

## Files changed

- Step 1 (93477e6): `src/safety_filter/resolver.py`, `src/safety_filter/config.py`, `experiments/safety_filter_spec_test.py`, `experiments/safety_filter_resolver_test.py`, `experiments/safety_filter_kong_style_test.py`, `scripts/safety_filter_kong_comparison.py`, `scripts/safety_filter_closing_arm_fix_check.py`, `docs/mpvi_decisions.md`, `docs/design_plan.md`, `docs/progress.md`.
- This commit: `docs/progress.md`, `docs/reports/20261118_2057_phaseR_cleanup.md`, and the SF-106 correction (0.697 to 0.696) in `docs/mpvi_decisions.md`.
- Not committed (git-ignored or untracked): `data/safety_filter/kong_single_pair_fixtures.json`, `data/logs/phaseR2/`, `.venv` (symlink).

## Not done

- Step 2c: the re-freeze of the sequential fixtures (blocked on the guard decision below).
- Step 2d: re-running sequential_push after a re-freeze.
- The closing right arm check: the fix is in, but the check still fails. The swing lever rule and the shortening accuracy target are not changed (both are design choices).
- The fixture file is not committed (git-ignored).
- Nothing pushed.

## Decisions for David

1. **Frozen sequential fixtures under the wide distance.** Options: (a) relax the single-pair guard, record each case's pairs, re-freeze all cases, and mark hand_link7 as a documented exception (its new output is the one that corrects the palm-box violation); (b) redefine the four scenes so only one pair is short. My recommendation is (a); the other three cases produce the same outputs, only hand_link7 changes.
2. **Forced SHAPE_RECHECK check.** Accept SHORTEN_ZERO for this scene, or force the recheck another way. stop_m = -1 makes every active pair count as short, so the wide set now always reaches shortening first.
3. **Closing right arm.** Choose among: (a) the swing's elbow displacement rule (`elbow_displacement_mode`, now a PLACEHOLDER); (b) the shortening accuracy target after a slide (SF-36: "raw" fails whenever a slide moved the hand); (c) hand slide also removing the tangential drift (candidate (b) from the 2026-11-12 report). The report's candidate (c), mirrored capsules, is a physical-layout question and stays with David.
4. **Fixture commit.** Should `data/safety_filter/kong_single_pair_fixtures.json` be force-added to git (as the other tracked data files are), so the sequential test runs from a clean checkout?
