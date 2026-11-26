# Phase R cleanup 3: the layered resolver is removed; kong_style is the only resolver

Date: 2026-11-18. Branch safety-filter, worktree ~/projects/openarm-tracer-sim-sf-d. Nothing pushed.
Decision row: SF-108 (docs/mpvi_decisions.md). Earlier rows SF-31 to SF-34, SF-40, SF-41, SF-49, SF-106, SF-107 are marked superseded; their text is kept.

Written for: David (decisions in section 9).

## 1. Summary

- **Removed.** The layered resolver: its hand slide, elbow swing, the fallback step shortening after a failed shape re-check, the layered lever rule and elbow placeholder, and the config switch `RESOLVER["mode"]` with its `layered` and `cbf` options. Six config keys are gone. Two diagnostic scripts that only studied the layered path are deleted (in git history).
- **Reverted first.** The code of 5438aea (tangential drift, SF-107) and 93477e6 (slide-aware elbow swing, SF-106). Their docs stay.
- **Still works.** kong_style is the only resolver, and `Resolver.step` is still the interface a future resolver would replace. kong_style's code is unchanged: an AST comparison with docstrings ignored says identical to the pre-removal file. The shared pieces it uses (step shortening, the four checks, the release band, the converter) stay.
- **Tests.** The checks that hold for the resolver as a whole run against kong_style and pass. The hand-slide and elbow-swing checks are removed, along with the one check that failed (the 10 mm surface rule). Nothing that passed before fails now.
- **Full suite.** 27 of 27 files exit 0, zero failing checks, zero tracebacks. At HEAD the suite had one failing check and one non-zero exit (spec_test). sequential_push_test is 43 of 43 with its existing fixtures; no re-freeze was needed.

## 2. Commits

| Hash | Subject |
|---|---|
| 437c951 | Revert 5438aea code (tangential drift, SF-107): layered hand slide back to the into-only removal (SF-33) |
| 3fd6925 | Revert 93477e6 (SF-106, slide-aware elbow swing; raw setting removed) |
| ba435a2 | WIP: Phase R cleanup 3 (1/4): layered resolver code removed (SF-108) |
| 9309ffe | Phase R cleanup 3 (2/4): safety-filter tests run against kong_style only (SF-108) |
| 207852d | Phase R cleanup 3 (4/4): decision rows and design plan for the layered removal (SF-108) |
| 6e187b3 | Phase R cleanup 3: restore the resolver's world-data sentence in its module docstring |
| 29b38ae | WIP: Phase R cleanup 3: progress entry (plan, steps 1 to 3 done, suite run started) |

No Co-Authored-By trailers. Author David Siff.

Reverts 1 and 2 had conflicts with later docs commits (design plan, decision log, progress log). Resolution: the docs were kept at HEAD, so no row or marker was deleted. The kong_style_test conflict (93477e6 revert) was resolved to the pre-93477e6 expectation; that test then changed again in commit 2.

## 3. Files changed (task commits, 277dd24..HEAD)

| File | Change |
|---|---|
| src/safety_filter/resolver.py | layered methods (_step_layered, _fix, _hand_slide, _elbow_swing, _swing_hand_target) removed; step() calls kong_style.step; MODES and layered-only imports removed; module docstring rewritten |
| src/safety_filter/push_direction.py | elbow_displacement and lever_weights removed (layered only); pair_push and contact_fraction unchanged |
| src/safety_filter/config.py | RESOLVER keys mode, elbow_swing_hand_target, shorten_after_failed_push, swing_unmovable_m, swing_straight_m, elbow_displacement_mode removed |
| src/safety_filter/kong_style.py | docstrings and comments only (code identical, AST check) |
| experiments/safety_filter_spec_test.py | layered switches, hand-slide section, elbow section, head-on shortening section removed; see section 5 |
| experiments/safety_filter_resolver_test.py | hand-slide and elbow-swing sections removed; kong_style checks added (section 5) |
| experiments/safety_filter_kong_style_test.py | in_mode decorators and section 12 (layered old results, cbf check) removed |
| experiments/safety_filter_sequential_push_test.py | the mode patch in main() removed |
| experiments/safety_filter_gap_rule_test.py | lever-weight and elbow-placeholder checks removed |
| scripts/safety_filter_closing_arm_fix_check.py | deleted (layered diagnostic only) |
| scripts/safety_filter_closing_arm_asymmetry.py | deleted (patched the removed _hand_slide) |
| docs/mpvi_decisions.md | SUPERSEDED notes on SF-31, 32, 33, 34 (partly), 40, 41, 49, 106, 107; row SF-108 added; no row deleted |
| docs/design_plan.md | markers on the layered bullets; SF-108 bullet; scope and status lines updated |
| docs/progress.md | task entry, plan, steps, suite run record |

## 4. Lines removed (approximate)

| Area | Removed | Added | Note |
|---|---|---|---|
| src/safety_filter (resolver, push_direction, config) | about 190 | about 15 | resolver about 150, push_direction 32, config 6 |
| scripts (2 files deleted) | 103 | 0 | |
| experiments (5 test files) | 214 | 71 | net about 143 |
| **Total, task commits vs 277dd24** | **506** | **96** | net about 410 (the two reverts add and later remove code; net counts are what matter) |

## 5. spec_test

Before (HEAD): 38 of 39 (1 FAIL: 'motion along the surface is kept within 2 mm (command with 10 mm along the surface)', 9.690 mm). After: **27 of 27, exit 0**.

| Check group | Before | After | Note |
|---|---|---|---|
| 1 pass-through | 1 | 1 | kept |
| 2 gap rule numbers | 4 | 4 | kept |
| 3 hand against hand | 8 (hand slide) | 4 | removed: HAND_SLIDE status, exact target orientation (1e-6), into-obstacle <= 1 mm, surface spec <= 2 mm, surface 10 mm (the failing one). Kept: scenario, realized orientation within 0.5 deg, other arm untouched. Added: push is not held (KONG_PUSH) |
| 4 elbow swing and step shortening (head-on stream) | 8 | 0 | removed: ELBOW_SWING tick and its checks; STEP_SHORTENED tick and its checks. kong_style produces no shortened step in this stream. The shortening checks (on the raw line, grid) moved to resolver_test (section 6) |
| 6 forced holds | 4 | 4 | kept; SHAPE_RECHECK forced with the braking shrink used by safety_filter_kong_style_test.py |
| 7 arm against arm | 5 | 5 | kept, all pass under kong_style |
| 8 rounds | 2 | 2 | kept |
| 9 determinism | 3 | 3 | kept; one label renamed from "hand slide scenario" to "hand-hand scenario" |
| 10 Stage 1 subprocesses | 4 | 4 | kept |
| **Total** | **39** | **27** | 13 removed (one of them the failing check), 1 added |

The module-level `mode = "layered"` and `elbow_swing_hand_target = "raw"` lines are removed.

## 6. resolver_test

Before: 37 ok. After: **28 ok, exit 0**. Removed 12 (hand slide, elbow swing, the literal accuracy target check, and the old NO_SOLVER check). Added 3 (on the forced shoulder case: the shortened step lies on the raw step; its fraction is on the 1/4096 grid; the NO_SOLVER check now uses a push that needs the converter).

Changes to keep kong_style results meaningful:
- SHAPE_RECHECK is forced with `stop_m = -1` plus the braking shrink used by safety_filter_kong_style_test.py. With `stop_m = -1` alone, kong_style gives SHORTEN_ZERO (probed).
- Both-arms-closing uses one sweep with the default stop_m. The old `stop_m = 0.1` was a layered round-selection trick; under kong it pushes nothing and only shortens.

## 7. Other tests and the full suite

Full suite: `data/logs/phaseR3/run_safety_suite.sh phaseR4`, one MuJoCo process at a time, 22:29 to 22:35. Logs: `data/logs/phaseR3/suite_phaseR4/` (summary.txt, one log per test). Baseline: `data/logs/phaseR3/suite_20261118_214117/` (code of 277dd24).

| Test | Baseline exit / checks | After exit / checks | Note |
|---|---|---|---|
| sequential_push | 0 / 43 ok | 0 / 43 ok | no re-freeze needed |
| spec | 1 / 38 ok, 1 FAIL | 0 / 27 ok | section 5 |
| resolver | 0 / 37 ok | 0 / 28 ok | section 6 |
| kong_style | 0 / 38 ok | 0 / 35 ok | 3 layered checks removed |
| gap_rule | 0 / 17 ok | 0 / 15 ok | 2 layered checks removed |
| link_rules | 0 / 63 ok | 0 / 53 ok | 5 removed config keys (2 checks each) |
| structure, touch | 0 | 0 | unchanged |
| the other 19 files | 0 | 0 | unchanged (braking_*, distance, fk, jit, never_touch, plugin, shapes print no per-check ok lines) |
| **Total** | **1 non-zero exit, 1 FAIL check, 0 tracebacks** | **0 non-zero exit, 0 FAIL, 0 tracebacks** | ok lines fell from 298 to 263 (removed checks; no new failures) |

## 8. Not done

1. **Nine Stage 2 measurement scripts in scripts/ still name the layered resolver.** Not edited. Their layered rows or patches no longer work as written, and some would silently run kong_style under a "layered" label:
   - safety_filter_kong_comparison.py (rows 3 and 4; the 93477e6 revert brought back `elbow_swing_hand_target` in it)
   - safety_filter_diag_common.py (row 4)
   - safety_filter_time_profile.py (row 4; patches the removed _hand_slide and _elbow_swing)
   - safety_filter_hold_causes.py (row 4; patches the removed _fix)
   - safety_filter_push_size_replay.py (row 4 branches)
   - safety_filter_push_size_tables.py (row 4)
   - safety_filter_solver_jump_replay.py (row 4)
   - safety_filter_speed_common.py (layered golden streams and mode switch; the speed gate uses these)
   - safety_filter_speed_profile.py (patches the removed _hand_slide and _elbow_swing)
2. **Coverage gap: SF-36 literal accuracy target.** The check that a shortened step is refused when it slows the hand by more than 2 mm was removed, because kong_style's sweep gives a shortened arm its own pose (SF-47). The literal target still applies to jump-through-guard shortening, but no test covers it now.
3. **Head-on stream has no shortened step under kong_style.** The spec's shortening section is gone; the shortening checks run only on the forced shoulder case in resolver_test.
4. Nothing pushed (as instructed).

## 9. Decisions for David

1. **Confirm the removal and SF-108.** The brief asked for it; SF-108 is recorded as "David to confirm". The spec items for the hand slide (SF-33, the SF-107 surface rule) and the elbow swing (SF-32) are no longer implemented or tested. Their rows are marked superseded, not deleted.
2. **The nine scripts in section 8.1:** delete, archive to legacy/, or port (rows 3/4 and layered golden streams)?
3. **Orientation target.** kong_style's target is the orientation after the jump-through guard (SF-46). In the hand-hand case it is 0.011 deg off the raw command. The exact 1e-6 check was dropped; the 0.5 deg accuracy check is kept. Confirm that the target is the post-guard pose.
4. **SF-36 coverage.** Add a kong_style test for the literal raw-pose target on guard-shortened steps, or accept the gap?
5. **Status names.** ELBOW_SWING and HAND_SLIDE stay in STATUSES so older recordings decode. Keep them, or renumber after checking the recordings?
6. **Spec shortening scenario.** Should spec_test get a kong_style scenario that produces a shortened step, instead of relying on resolver_test?

## 10. Notes

- Suite log directory and baseline are in section 7. The progress entry is in docs/progress.md.
- Untracked files left alone: `.venv` (symlink), `docs/reports/20261118_1655_braking_phaseL_vs_M_fresh_vm.md` (another session).
