# Phase R: where sequential_push changed (bisect), and the requirements pin

Written for: David (project lead), to decide what to do about sequential_push and spec_test.
Branch safety-filter. Nothing pushed. No code, fixture or config changed by this task.

## Summary

- **Yes, the braking work changed the resolver's behaviour, and one commit did it: 51e6b42 "Braking step 8".** Both failing checks change at that commit: `hand_link7_vs_box_converges` (joint difference 5.2e-2 rad) and `SHAPE_RECHECK` (HOLD SHORTEN_ZERO instead of its forced reason). Its parent passes both checks and it fails both; the later commits tested also fail.
- It was not a stray edit. The commit widens the activation and release distance of every pair rule in `src/safety_filter/touch.py`, a rule the resolver also uses. The commit message describes it. The change is intended for the braking layer, but it also changes which pairs the resolver's sweep sees, and the frozen single-pair fixtures and the sequential_push checks were made before it.
- A diagnostic confirms the cause: at 51e6b42 with that widening switched off (and nothing else), the test gives 43 of 43 and exit 0.
- Requirements: `numpy==2.2.6` committed (018cb79). `numba==0.68.1` was already pinned on this branch (since e544a8d, 2026-11-17), so no numba change was needed.
- spec_test closing-arm failure added to the progress log as a known failure, pointing to `docs/reports/20261112_0830_safety_filter_closing_arm_failure.md`.

## Setup

- Good: 9e6718e (safety-filter-speed tip). Bad: 5b413d8 (safety-filter tip, the "current HEAD").
- Scratch worktree `~/projects/openarm-tracer-sim-bisect` (now removed), detached at each commit.
- The fixture `data/safety_filter/kong_single_pair_fixtures.json` (sha256 68dbb6cf...) was copied in from `~/projects/openarm-tracer-sim-safety-filter`. Its other git-ignored `data/safety_filter` files (715 files, including subfolders) were copied too, so the data matches the good worktree. Tracked files come from checkout.
- `deps/` was not initialised in the scratch worktree (the robot model needs `deps/OpenArm-Combined` meshes), so it was symlinked to the good worktree's `deps/`. No submodule pointer changes in 9e6718e..5b413d8 (checked), so the link is valid at every commit.
- Test: `experiments/safety_filter_sequential_push_test.py`, run with the main repo's `.venv`, `MUJOCO_GL=egl`, timeout 3600 s. About 80 s per run.
- Runner: `data/logs/phaseR/run_step.sh` (ignored by git). One log per commit: `data/logs/phaseR/seq_<sha>.log`. A run is bad if any FAIL line for the chosen check, or a crash (no summary line → skipped). No commit was skipped.

## Sanity

| Commit | Subject | Result | Log |
|---|---|---|---|
| 9e6718e | Part B steps 3 to 5 complete (good) | 43 of 43, exit 0 | seq_9e6718e.log |
| 5b413d8 | Braking Phase Q report (bad) | 41 of 43; hand_link7 FAIL, SHAPE_RECHECK FAIL | seq_5b413d8.log |

The first attempt failed at startup (no meshes). Its logs were deleted; it is not a result.

## Bisect

Two bisects, one per check, bad 5b413d8, good 9e6718e. Git reported 57 revisions left, about 6 steps. The second bisect reused the cached logs; it ran no new tests.

| Commit | Subject (short) | hand_link7 | SHAPE_RECHECK | Log |
|---|---|---|---|---|
| 0da5ba3 | Braking Phase D fix 4 (cylinder distances) | bad | bad | seq_0da5ba3.log |
| 7e57688 | Braking: S-curve shapes only what the layer modulates | bad | bad | seq_7e57688.log |
| 0f10ade | Braking step 3: hand speed cap | good | good | seq_0f10ade.log |
| 38b8c7b | Braking step 9: bridge override with error log | bad | bad | seq_38b8c7b.log |
| 0a9f554 | Braking step 6: HOLD as a smooth stop | good | good | seq_0a9f554.log |
| 3d285ea | Braking step 7: allowed_contact flag | good | good | seq_3d285ea.log |
| **51e6b42** | **Braking step 8: bridge_reach_m derived, cutoffs widened** | **bad** | **bad** | seq_51e6b42.log |

Result: `51e6b42 is the first bad commit` in both bisects (bisect logs: `data/logs/phaseR/bisect_log_hand.txt`, `bisect_log_shape.txt`). Its parent 3d285ea was tested directly and passes both checks, and 51e6b42 fails both. Only one commit is responsible.

Commits visited in total: 9 (2 sanity, 7 bisect steps). Each run took about 80 s; the second bisect took about 1 s.

## The first bad commit: 51e6b42, "Braking step 8"

Author David Siff, 2026-11-16. 11 files, 381 insertions, 41 deletions.

What each file does:

| File | Change | On the sequential_push path? |
|---|---|---|
| `src/safety_filter/touch.py` | New `wake_dist()` = `worst_stop_dist_m` (the stopping distance from the hand speed cap). `_rule` sets each pair's activation distance to `max(margin + PUSH_START_BEYOND_M, wake_dist)` and its release distance to `max(margin + PUSH_STOP_BEYOND_M, wake_dist)`. `evaluate_pairs` keeps pairs within `max(old max_act, wake_dist)`. | **Yes. This is the cause.** The resolver's candidate pairs come from these rules. |
| `src/safety_filter/tick.py` | Pair-table cache key includes `worst_stop_dist_m`. Sleep timers add a drift total and an arm-lag term to the pair wake bound. | Yes, but not the cause (see the diagnostic). |
| `src/safety_filter/config.py` | `bridge_reach_m` derived from the arm model (was the placeholder 0.56 m). New `derived()`, `arm_lag_rad`, `drift_budget_m`. | Not on this path (the bridge and the derived stop distance do not change the resolver). |
| `src/safety_filter/braking.py`, `braking_math.py`, `src/safety_filter_plugin.py` | Braking layer: bridge, approach ceiling slow-down. | No. |
| `experiments/safety_filter_braking_b_test.py` (new), `braking_test`, `kong_style_test`, `never_touch_test`, `touch_test` | Tests. | No. |

In plain language: before this commit, a pair of shapes was only tracked once it came within its activation distance (its margin plus a fixed start distance). After it, any pair within the distance the hand needs to stop at the speed cap is tracked too, even when that is further out than the old cutoff. The resolver's sweep therefore sees pairs it did not see before, and it pushes differently. The hand-link-7 case's output then differs from the saved one by 5.2e-2 rad, and the forced SHAPE_RECHECK case is shortened to zero (HOLD SHORTEN_ZERO) before its forced check can give its own reason.

## Diagnostic (not committed)

At 51e6b42 a wrapper (`data/logs/phaseR/diag_wake0.py`) set `touch.wake_dist` to return 0 and ran the test unchanged:

- Result: **43 of 43, exit 0** (`data/logs/phaseR/diag_wake0_51e6b42.log`). Unpatched 51e6b42: 41 of 43.
- So the widening in `touch.py` is sufficient to explain both failing checks.
- Limit: the patch covers only `touch.wake_dist`. `tick.py` still reads `worst_stop_dist_m` directly (cache key, sleep timers), and that path was not isolated. With the `touch.py` widening removed, both checks pass, so `touch.py` explains both failures.

## Requirements

- Commit 018cb79: `requirements.txt` line `numpy>=1.24.0` → `numpy==2.2.6` (the installed version, pinned as mujoco is).
- numba: `numba==0.68.1` is already in this branch's `requirements.txt` (added in e544a8d, 2026-11-17). The progress-log note from Phase Q that numba is "NOT in requirements.txt" was out of date; it is corrected in the progress log.
- Check (inline, not committed): `requirements.txt` parses (13 lines); all 8 pinned packages match the installed versions in the main repo's `.venv` (mujoco 3.13.0, numpy 2.2.6, mink 1.3.0, qpsolvers 4.13.0, daqp 0.10.3, google-genai 2.31.0, imageio-ffmpeg 0.6.0, numba 0.68.1). The other 5 lines are ranges and were not changed.
- Not changed: main's `requirements.txt` has no numba line (main does not contain the safety-filter code). Out of scope.

## Deferred list

`docs/progress.md`: known failure entry for `experiments/safety_filter_spec_test.py` (closing-arm case, failing since 2026-11-12, spec_test never passed since its creation at 9089dfe). Three candidate resolver changes in `docs/reports/20261112_0830_safety_filter_closing_arm_failure.md` (line 47), none built.

## Not done

- Nothing fixed. The fixture was not regenerated. The test was not edited.
- No push.
- The Monitor and bisect runs left `data/logs/phaseR/` (git-ignored) in place.
- Other sequential_push checks (the other 41 checks, and the 8 subprocess safety-filter tests) passed at every commit visited.

## Decisions for David

1. The widened pair cutoff (51e6b42) is shared by the resolver and the braking layer. Choose one:
   - (a) Keep it for both. Then the single-pair fixtures and the sequential_push checks need to be re-frozen and re-examined. (Not done here.)
   - (b) Apply the widening to the braking layer's pair lists only and keep the resolver's old cutoffs, so the resolver's frozen behaviour returns. Needs a design decision on which pairs the braking layer owns.
   - Lean: (b), because the sequential_push fixtures were frozen for the resolver's old pair set, and changing those fixtures should be a separate decision.
2. spec_test closing-arm failure: pick one of the three candidate resolver changes, or leave it deferred.

## Commits

- 018cb79 Requirements: numpy pinned to 2.2.6 (step 2).
- Commit for this report (step 1).
- Commit for `docs/progress.md` (step 3 and the bisect result).
