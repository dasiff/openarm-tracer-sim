# Stance search logging: fixes, proof runs and tests

Date: 2026-11-12, 03:50 UTC. Branch main, local commits only (nothing pushed).
Supersedes the claims of `docs/reports/20261111_stance_search_logging_validation.md`, which marked
items "met" that were not working. That report is left as it was (dated); this one replaces it.

## Summary

The three JSON files were written, but most of their content was wrong or empty. The full-check
solutions were never written, the dry-run file had no per-tick data, every candidate was stamped with
the end-of-search time, and the accepted candidate was always -1. All of this is fixed. Each spec item
is now shown by a real stance search on the staging layout (no stub), and the search decisions are
unchanged: a before/after comparison with fixed seeds gives identical results.

## What was wrong before (found by reading the code and the first smoke search)

1. Timings: the logger clock was started after the full-check build, and every candidate was stamped
   after the search had finished, so each showed the whole search time (73.62 s).
2. Full-check solutions: the recording condition was never true (a candidate was added to the list after
   its check ran), and the keys it read do not exist. `full_check_solutions.json` was `{}` in every run.
3. Accepted candidate: read a key that does not exist, so it was always -1.
4. Heading: `heading_deg` held radians.
5. Dry run: per-tick data was never kept; the error field read a key that does not exist (always null).
6. Body-check rejects were not in the candidate list.
7. The three files were written only on the success path.
8. Commit 7c32ac8 removed a pass-only condition, but a second condition in the same line still made the
   recording never run, so that commit changed nothing that is visible.

## Commits (local, in order)

| Commit | Change | Decisions row |
|---|---|---|
| 1f5126a | Per-candidate timings, numbering, accepted candidate, files on every exit | M313 |
| 0ad9dda | Search writes into its run's folder; run record names the folder | M312 |
| 42288f5 | Full-check arm solutions per candidate (nominal stance) | M314 |
| f8634c6 | Per-tick dry-run data, rounded to 4 decimals, place angle field | M315 |
| c2c350e | Logging test rewritten; proof script | - |
| d8394e0 | Place turn in the planner's convention (-180 to 180) | M314 updated |
| b859aab | Test: clock started before the first candidate | - |
| c661cca | A check that raises is recorded as "interrupted" (status rule was crashing the save) | M313 updated |
| b1307ef | Test: expected value of 4-decimal rounding | - |
| (this commit) | This report, M316 (corrections to M309 to M311), progress log, two checks moved into experiments/ | M316 |

## Spec items 1 to 5

| # | Item | Status | Evidence (real search, final code) |
|---|---|---|---|
| 1 | Location: files in the run's folder, run record names it, no overwrite | **Met** | Final proof run folder `data/demo_record/stance_search_proof_20261112_025903/` holds `candidates_list.json`, `full_check_solutions.json`, `dryrun_trajectories.json`, and `run_info.json` (names the folder, the sampler seed 3397871272 and the full-check seed 4242). The sampler record also names it. Test part A: a second search in the same run folder gets its own subfolder. |
| 2 | candidates_list.json: all candidates incl. body-check rejects, number, base x/y, heading, pedestal, check results and reasons, wall-clock and simulator times start and finish per candidate, accepted candidate, written on failure and exception | **Met** | Final proof: 51 drawn (1 body-check reject, 4 full-check rejects, 1 accepted = number 32, 45 not checked). Exception path (below): candidate 12 recorded as "interrupted", accepted -1, all three files written, original error reaches the caller. |
| 3 | full_check_solutions.json: every candidate reaching the full check, arm joints at each task point it solves, labelled by candidate, arm, task point, approach angle | **Met** (nominal stance only; see "not done") | Final proof: 5 candidates reached the full check (10, 12, 23, 32, 46); each has solves. Candidate 32: 12 pick solves (one per approach bin) and 72 place solves (one per 5 degrees). Labels: arm, kind, approach angle, point index, path, pose, target, 7 joint angles. No-accept run: all 50 candidates that reached the full check have entries (6192 solves). |
| 4 | dryrun_trajectories.json: every candidate reaching the dry run; per tick commanded joints and pad position for each subtask; place angle in degrees as its own field; 4 decimals | **Met** | Final proof, candidate 32 (passed): 10 subtasks, rows equal ticks, 3,070 ticks in total, place turn -30.0 deg and pick bin 0 (0.0 deg) as fields. No-accept run, candidate 22 (failed): pick IN not reached (3.9 mm from its waypoint, tolerance 3 mm) after 2,312 ticks, with the failure text. Test checks rounding and row counts on every candidate. |
| 5 | Behaviour: 7c32ac8 changes only what is recorded; diff stat of each logging commit | **Met** for behaviour; **commit history not clean** (see below) | Before/after comparison with both seeds pinned (`experiments/stance_search_regression_check.py`): identical draws, full-check results, dry-run results (subtask ticks and times), gate rows, chosen candidate and task config. Lines of 7c32ac8 in `src/stance_select.py`: the old line was `if r.get("ok") and len(results) <= len(rec.get("draws", [])):`; it became `if len(results) <= len(rec.get("draws", [])):`. That changes only what is recorded, not the check or any return value. |

### Diff stats of the logging commits (one sentence each)

- 501d97c: 1259 files. In `src/` and `experiments/` only the two logging files changed, but the commit
  also contains `scratch/` files, reports, `compare_nosag.py`, `extract_height_data.py`, a 13,604-line
  file `measstart`, `.claude/settings.local.json` and `docs/progress.md`. **It does not touch only logging.**
  Not rewritten (would change history); David to decide.
- 2f92ad9: `src/stance_select.py` +32; logging only.
- 9005df1: `docs/reports/20261111_stance_search_logging.md` +223; documentation only.
- 7c32ac8: `src/stance_search_logging.py` (21 lines) and `src/stance_select.py` (7 lines); logging only (the condition above).
- c5757a6: `experiments/stance_search_logging_test.py` +125 and `src/stance_search_logging.py` -4; logging and file writing only.
- 9bcc379: `docs/mpvi_decisions.md` +3; documentation only.
- a232909: `docs/reports/20261111_stance_search_logging_validation.md` +284; documentation only.

## Timings and simulator clock (final proof run, selection 256.2 s)

- First candidate (number 0): wall 0.078 s to 0.102 s after the search start (sampler start).
- Last candidate (number 50): wall 1.355 s to 1.384 s.
- Accepted candidate 32: body check 0.876 s to 0.912 s; full check 201.566 s to 233.067 s (31.5 s);
  dry run 233.067 s to 250.548 s (17.5 s).
- Simulator clock (scene data time) and check clock: 0.0 s at every candidate's start and end, in every run
  (smoke run, both pinned and unpinned proof runs, final proof run, no-accept test run).
- **The simulator sits paused during the stance search.** No physics step runs in that code path (the
  dry run and the full check use kinematics only). Caveat: 0.0 s at every event shows the clock did not move;
  it cannot show the clock was never reset.

## Proof search: files and sizes (final run, 025903)

| File | Size |
|---|---|
| candidates_list.json | 32,859 bytes |
| full_check_solutions.json | 908,897 bytes (5 candidates) |
| dryrun_trajectories.json | 249,502 bytes (1 candidate) |
| run_info.json | sampler seed 3397871272, full-check seed 4242, ok true |

One candidate record (accepted, number 32): base (0.7639, 1.0908), heading -88.4489 deg, pedestal 0.1644 m,
status accepted, with the three checks' start and end times as above.

### Labelled arm solutions, accepted candidate 32 (left arm; q in rad)

Pick, approach bin 0 (0.0 deg from the stance heading), ok:

| Point | Path | Target (m) | q1..q7 |
|---|---|---|---|
| 0 | over | 0.9968, 0.7800, 1.1600 | 0.3842, -0.1309, -0.1146, 2.0791, -0.2152, 0.1511, 0.0559 |
| 1 | standoff | 0.9968, 0.7800, 1.0125 | -0.0062, -0.8343, 0.9356, 1.4363, -1.0705, 0.5025, -0.7629 |
| 2 | grasp | 1.0000, 0.6600, 1.0050 | -0.2698, -0.0117, -0.1967, 0.8742, 0.1071, 0.0330, -0.3581 |
| 3 | grasp | 1.0000, 0.6600, 1.0100 | -0.2596, -0.0081, -0.1966, 0.9122, 0.1079, 0.0329, -0.3308 |
| 4 | grasp | 1.0000, 0.6600, 1.0150 | -0.2548, -0.0070, -0.1963, 0.9260, 0.1079, 0.0339, -0.3410 |
| 5 | grasp | 1.0000, 0.6600, 1.0200 | -0.2509, -0.0058, -0.1963, 0.9389, 0.1076, 0.0347, -0.3505 |
| 6 | lift | 1.0000, 0.6600, 1.1205 | -0.6409, -0.9055, 1.1493, 1.5126, -1.2261, 0.6091, -0.3366 |
| 7 | back | 0.9986, 0.7100, 1.1205 | 0.0230, -0.4203, 0.2156, 1.7607, -0.3085, 0.4725, 0.1694 |

Place, hand turn -30.0 deg, ok (6 points):

| Point | Path | Target (m) | q1..q7 |
|---|---|---|---|
| 0 | carry | 0.8000, 0.6600, 1.1225 | -0.3296, -0.2250, 0.4640, 1.3887, -0.3433, -0.1243, 0.0249 |
| 1 | setdown | 0.8000, 0.6600, 1.0750 | -0.3319, -0.1898, 0.4977, 1.1577, -0.2942, -0.1523, -0.1915 |
| 2 | setdown | 0.8000, 0.6600, 1.0800 | -0.3301, -0.1942, 0.4942, 1.1828, -0.2989, -0.1502, -0.1706 |
| 3 | setdown | 0.8000, 0.6600, 1.0850 | -0.3279, -0.1988, 0.4904, 1.2110, -0.3042, -0.1470, -0.1454 |
| 4 | setdown | 0.8000, 0.6600, 1.0900 | -0.3267, -0.2029, 0.4868, 1.2358, -0.3090, -0.1445, -0.1237 |
| 5 | back | 0.7986, 0.7100, 1.0845 | -0.1285, -0.3272, 0.4520, 1.4535, -0.3347, -0.1363, -0.1350 |

Pick points carry no pose tag in the check's own data (`pose` is null for them).

### Dry run of candidate 32 (passed; pick bin 0, place turn -30.0 deg)

| Subtask | Ticks (= rows) | Subtask | Ticks (= rows) |
|---|---|---|---|
| pick OVER | 269 | place OVER | 281 |
| pick DOWN | 584 | place DOWN | 292 |
| pick IN | 481 | place OPEN | 75 |
| pick CLOSE | 150 | place BACK | 217 |
| pick UP | 429 | place HOME | 292 |

Comparison at one point: the dry run's pick OVER ends at pad (0.9963, 0.7798, 1.1600), 0.5 mm from the
check's target, with joints (0.4455, -0.4636, -0.1237, 2.0996, -0.4616, 0.1312, 0.0818). The check's
solution for the same point is (0.3842, -0.1309, -0.1146, 2.0791, -0.2152, 0.1511, 0.0559). Same pad
point, different arm configuration (joint 2 differs by 0.33 rad, joint 5 by 0.25 rad). Observed here, not analysed.

## Your question: is the beaker too far from the burner for any position to work?

I did not choose the beaker or hotplate positions. Both are fixed for the staging layout in `TASK_CFG`:
beaker (1.00, 0.66) and hotplate (0.80, 0.66). The search chooses only the base stance.

What the runs show (distance from the base to the hotplate centre):

- Final proof run: accepted base at 0.43 m. The four full-check failures were at 0.48 to 0.52 m.
- No-accept test run (50 candidates reached the full check; none accepted):

| Full-check failure | Count | Base distance to hotplate |
|---|---|---|
| Place-angle window too narrow | 14 | 0.42 to 0.60 m |
| Pick reach (no collision-free solution) | 12 | 0.37 to 0.54 m |
| Eagle camera occlusion | 7 | 0.36 to 0.56 m |
| Arms at home too close to something | 15 | 0.27 to 0.64 m |
| Beaker corner too close to a body | 1 | 0.41 m |
| Passed full check, failed dry run (pick IN, 3.9 mm vs 3 mm) | 1 | 0.51 m |

So distance matters for the place stops (all place-window failures are at 0.42 m or more), but it is not the
only limit, and several failures happen close to the hotplate for other reasons. No evidence says the layout
cannot work: one run accepted a stance and passed the dry run. The reach filter in the sampler (reach limit
0.6654 m from the shoulder to each task point, decisions row M300) already keeps out-of-reach draws, so
hotplate distance is not the same as reach. Five and fifty candidates are not enough to settle it. A sweep of
base positions at fixed headings would answer it; not done (needs David's go, since it is new runs).

## Tests (exit codes read from the logs; see the note on run_capped.sh)

| Test | Result | Notes |
|---|---|---|
| experiments/stance_search_logging_test.py | exit 0, 23 of 23 | Part B: real search, 50 drawn, 50 full checks, none accepted, 1 dry run (failed); the no-accept case is covered. |
| experiments/trajectory_dryrun_test.py | exit 0 | Ran after the per-tick commit; the dry run's results are unchanged. |
| experiments/full_check_test.py | exit 1, 30 of 31 | Failing check: "a candidate that fails the cheap checks fails fast": 3.12 s against a 1.5 s limit (first call includes building the checker). **Already fails at 8194664** (3.06 s, exit 1). Not fixed. |
| experiments/stance_select_test.py | exit 1 | Failing: "two runs, each with its own stance" and "two navigation steps both move", then IndexError: the test calls the retired overlay selection (`select_bench_stance`), which M299 and M300 replaced. **Identical failure at 8194664.** Not fixed. |
| experiments/stance_search_exception_check.py | exit 0 | Injected error in the second full check: error reaches the caller unchanged; the three files are written; candidate 12 is "interrupted"; accepted -1. |
| experiments/stance_search_regression_check.py (old a232909 vs new) | identical | See item 5. |

Note: `experiments/run_capped.sh` always exits 0 (its last line reports the real exit code only inside the
log). My test driver therefore recorded "exit 0" for failing tests; the codes above are from the logs.
Not changed (project script).

## Files produced by the runs

| Run | Folder (under data/demo_record/) | Note |
|---|---|---|
| Smoke, before the per-tick and solution fixes | stance_search/20261112_020337_e5beb2f8 | Solutions and dry-run files empty (old code); accepted 28 (draw order) |
| Proof 1, unpinned | stance_search_proof_20261112_021935 | Accepted at its first candidate: not "several candidates"; before the place-angle fix |
| Proof 2, pinned | stance_search_proof_20261112_022145 | Before the place-angle fix (0 to 355 degrees) |
| **Final proof, pinned** | **stance_search_proof_20261112_025903** | **Final code; the proof in this report** |
| No-accept test search | stance_search/20261112_030407_29b3b091 | Final code; 50 full checks, none accepted |
| Exception check | stance_exception_check_20261112_030321 | Final code; injected error |

Sizes in the no-accept run: candidates_list.json 51,304 bytes; dryrun_trajectories.json 278,083 bytes;
full_check_solutions.json **8,086,190 bytes** (50 candidates, indented JSON). The solutions file is the
largest; a compact format would cut it, not done.

## Decisions for David

1. Numbering and timing origin (decisions row M313, marked PROPOSED): numbers follow the sampler's draw order
   (body-check rejects included); timings start at the sampler's start. Confirm or change.
2. Commit 501d97c (1259 files, including scratch files, `measstart`, `.claude/settings.local.json`): leave
   as it is and note it, or rewrite the local history (needs your approval; nothing is pushed).
3. Solutions file size (8 MB for 50 candidates): compact format, or keep readable.
4. Video runs (`demo_video.py` without `--record-only`) write their search into the default folder, because
   their work folder is deleted after a successful video. Keep it?
5. Whether the solutions should also cover the error stances (only the nominal stance is recorded).
6. The two failing tests above (already failing at 8194664): fix the timing limit and retire the overlay
   test, or leave them.
7. The distance question: approve a sweep of base positions at fixed headings (new runs)?
8. The earlier validation report (20261111) is now wrong; keep it as history or mark it superseded?

## Not done

- Nothing pushed.
- No video was rendered and no stance was re-run for it.
- Error-stance solves are not recorded (nominal stance only).
- Only one failing dry run (candidate 22) was checked with the final code.
- The distance sweep is not run.
- The solutions file is not compacted.
- The commit history of 501d97c is not changed.
- `run_capped.sh` exit code not changed.
- Bash permission: the user's instruction "allow all bash commands without asking" was applied by adding `Bash`
  to the allow list in `~/.claude/settings.json`; the deny rules for force push, reset --hard, clean, rm -rf and
  sudo are kept (deny rules take precedence, so those commands are still refused).
