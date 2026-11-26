# Braking Phase Q: wiring reverted, test results and environment check

Written for: David (project lead). Branch safety-filter, worktree ~/projects/openarm-tracer-sim-sf-d. NOT pushed.

## 1. Summary

- The Phase O wiring (b8c1efd, SF-104) is reverted in 719e44c. The filter's shapes and resolver read MuJoCo again, exactly as in Phase M (6a5110c). arm_fk.py (SF-103) stays, unused.
- After the revert, the safety-filter tests match the Phase M commit file by file. braking_j passes again. touch passes again.
- The touch failure in the Phase O report was caused by the wiring, not by the environment or by the test. I corrected the Phase O report.
- The environment is unchanged since 2026-09-23 for MuJoCo and numpy. The VM restart did not change any package. numba was installed on 2026-11-17.
- Two tests fail at both the reverted HEAD and Phase M: spec (one closing-arm check, failing since it was written on 2026-11-12) and sequential_push (missing fixture file here; with the fixture copied, two checks still fail).

## 2. Commits

| Commit | Content |
|---|---|
| 719e44c | Revert of b8c1efd: SF-104 kept and marked reverted; SF-105 added; wiring test removed; shapes.py and arm_conversion.py back to 6a5110c; arm_fk.py back to its step-2 state |
| (this report) | Phase Q report; Phase O report corrected on touch; progress log |

Files changed by the revert: `src/safety_filter/shapes.py`, `src/safety_filter/arm_conversion.py` (both identical to 6a5110c), `src/safety_filter/arm_fk.py` (identical to f6d5859), `experiments/safety_filter_fk_wiring_test.py` (deleted), `docs/mpvi_decisions.md` (SF-104 marked reverted, SF-105 added), `docs/progress.md` (kept as history).

## 3. Safety-filter tests: after the revert, against Phase M

Both sets were run one after the other in the same window: the reverted HEAD (719e44c) and the Phase M commit (6a5110c, scratch worktree, same data and submodule copies). 27 files at HEAD, 26 at Phase M (the FK test is new).

| Test | Reverted HEAD | Phase M (6a5110c) | Note |
|---|---|---|---|
| braking_b, c, d, e, f, g, h, i, k, l, m; braking_test | exit 0 | exit 0 | unchanged |
| **braking_j** | **exit 0** | exit 0 | **passes again** (failed with the wiring) |
| distance, gap_rule, jit, kong_style, link_rules, never_touch, plugin, resolver, shapes, structure | exit 0 | exit 0 | unchanged |
| **touch** | **exit 0** | exit 0 | **passes again** (failed with the wiring) |
| fk_test (new in SF-103) | exit 0 | not present | new file |
| spec | exit 1 | exit 1 | same one failing check (closing right arm) |
| sequential_push | exit 1 | exit 1 | missing fixture file in this worktree |

Result: 25 of 27 pass at HEAD, 24 of 26 at Phase M. The FAIL lines of every common test are identical between the two runs. No other file changed result compared with Phase M.

## 4. Environment (read-only; nothing changed)

| Package | Installed now | Before | Evidence |
|---|---|---|---|
| MuJoCo | **3.13.0** | 3.3.4 until 2026-09-23 | `requirements.txt` pins `mujoco==3.13.0`; the dist-info dates from 2026-09-23; the uv cache holds both 3.3.4 and 3.13.0 (both 2026-09-23); the version-comparison memory says the project moved to 3.13.0 on that date |
| numba | **0.68.1** (llvmlite 0.50.0) | not installed before 2026-11-17 | dist-info dated 2026-11-17; no earlier numba in the uv cache. **Not listed in `requirements.txt`**, and no commit has ever added it there, although the Phase F report says it was added. |
| numpy | **2.2.6** | not recorded | dist-info dated 2026-09-23; `requirements.txt` says `numpy>=1.24.0` (not pinned). The uv cache also holds numpy 2.5.3 (2026-09-23); the order of the two is not recorded. |

VM restart (boot 2026-11-18 16:33): no package file changed. All dist-info directories predate the boot. The restart did not change the environment.

touch test history: it passed in the regression runs before the wiring (j1reg 2026-11-17, k1reg and l1reg 2026-11-18 02:xx, m2reg 15:22 today, all with MuJoCo 3.13.0). It failed only in the wiring run (o_reg, 17:15 today). The cause is the wiring: the test passes `RobotShapes.data` to `mj_geomDistance` as its MjData argument, and after the wiring that object was a RobotKinematics (the traceback shows `Invoked with: ... <src.safety_filter.arm_fk.RobotKinematics object>`). It passes again after the revert.

## 5. spec test (the closing right arm must move)

**What it checks.** The resolver against the written spec: gap-rule numbers, pass-through, hand slide, elbow swing, step shortening, forced holds, arm against arm, round limit and determinism; it also runs the Stage 1 tests as subprocesses.

**The failing check.** A head-on hand-hand approach: the left hand closes 8 mm in one tick with the right arm parked. The left case passes (hand slide). The right case ends in HOLD (ACCURACY) after 10 rounds, so the right arm stays at its last command. Analysis in `docs/reports/20261112_0830_safety_filter_closing_arm_failure.md`: the elbow swing and the hand slide undo each other in one tick (the elbow swing aims at the raw hand pose; the hand slide moves it).

**When it last passed.** Never, as far as the history shows. The check was added in commit 9089dfe (2026-11-12, "2 checks fail"); the other of those two checks (hand orientation) now passes. The Phase D report lists the spec test as failing before Phase D, unchanged.

**What fixing it involves.** This is a design decision, not a test fix. The closing-arm report asks David five questions about how the Kong algorithm combines the elbow swing and the hand slide in one tick (hand target, loop and limit, shorten or hold, tangential component, contact normal). Three candidate fixes are listed there (a: swing aims at the slid hand target; b: slide also removes tangential drift; c: mirrored capsules); none was built. Once decided, it is a resolver change plus a check that the Stage 1 and kong tests still pass.

## 6. sequential_push test (missing fixture)

**What it checks.** The one-pair-at-a-time sweep of the kong_style resolver (SF-52 to SF-58) on fixed poses: frozen single-pair cases compared with the saved fixture (which must match the old summing sweep; two cases are documented exceptions), processing order, skip rule, re-measure rule, one solver call per pushed pair, no summed push, rounds and determinism, pass-through, closing-arm decision, forced failures, status record, no summing left in src, and the other safety-filter tests as subprocesses.

**Why it fails here.** It reads `data/safety_filter/kong_single_pair_fixtures.json`. That file is written by `scripts/safety_filter_kong_single_pair_fixtures.py` (commit 3be78ca, 2026-11-12) and is git-ignored. It exists only in the other worktree (`openarm-tracer-sim-safety-filter`, 15216 bytes, written 2026-11-12 19:20, sha256 68dbb6cf...). This worktree never had it.

**When it last passed.** Cat demo report (2026-11-13): sequential_push exit 0 in a worktree with the fixture. Diagnostic run today, at the other worktree's commit 9e6718e with its fixture: **43 of 43 checks pass**.

**Diagnostic (scratch, nothing in the repo changed).** With the fixture copied into a scratch copy of the Phase M commit, the test gives **41 of 43**. Two checks fail:
- `hand_link7_vs_box_converges`: the frozen output differs by 5.2e-2 rad (tolerance 1e-9). This case is not one of the two documented exceptions.
- `SHAPE_RECHECK`: the forced case returns HOLD with SHORTEN_ZERO, not the expected HOLD reason.

So the cause lies in code changes on the safety-filter line between 9e6718e and 6a5110c (110 commits). I did not bisect that range.

**What fixing it involves.**
1. Copying the fixture file into this worktree is not enough. Two checks would still fail.
2. For `hand_link7_vs_box_converges`, David must decide whether the frozen expectation is stale (the behaviour changed on purpose) or the code regressed. Then either regenerate the fixture (with a note in the decision log) or fix the code. Regenerating it from today's code would freeze the new behaviour and defeat the regression check, so that needs David's call.
3. `SHAPE_RECHECK` needs a look at the forced-failure setup in the test.
4. A bisect of the 110-commit range would show which change moved `hand_link7_vs_box_converges`.

## 7. Not done

- No test was fixed (spec, sequential_push, touch-related: none needed after the revert).
- No environment change (MuJoCo, numba, numpy untouched).
- No bisect of the sequential_push regression.
- The fresh-VM benchmark report (`docs/reports/20261118_1655_braking_phaseL_vs_M_fresh_vm.md`) is still uncommitted from an earlier task.

## 8. Decisions for David

1. Whether the spec closing-arm check should drive a resolver change (a, b or c in the closing-arm report), or be accepted as a known failure until then.
2. Whether `hand_link7_vs_box_converges` is stale (regenerate the fixture) or a regression (fix the code). Then say whether to bisect the 110 commits to find the cause.
3. numba: add it to `requirements.txt` (the Phase F report said it was added; it was not), with a pin like mujoco's.
4. numpy: decide whether to pin it (`requirements.txt` says `>=1.24.0`; installed 2.2.6).
5. touch test: no change needed now. Keep the test's use of `RobotShapes.data` as MjData in mind for any future interface change.
