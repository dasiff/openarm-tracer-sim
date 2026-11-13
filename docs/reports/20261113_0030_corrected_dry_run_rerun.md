# Corrected dry run: set-down fix applied, seeds recorded, both layouts rerun

Date 2026-11-13. Written for David. Nothing pushed.

## What was done

1. **The approved fix** (decisions row M318, commit 52b0eb3): in `src/trajectory_dryrun.py` the dry run now takes the pad at the grasp (after the move in, before the close) for the pad-to-beaker offset, instead of the pad after the lift. Two lines, nothing else in the function changed. Tests that were run: `trajectory_dryrun_test` 4 of 4 pass (exit 0), `stance_search_logging_test` 23 of 23 pass (exit 0). No test or baseline holds a place-down, place-open, retreat or home expected value (searched `experiments/` and `experiments/baselines/`), so no expected value was changed.
2. **Seeds recorded** (row M319, commit 386cef0): `candidates_list.json` of the search folder now holds the sampler seed and the full-check pinned seed (null when not pinned); each candidate's full-check record holds that check's own seed; the run's `run_info.json` holds `sampler_seed`, `full_check_seed_pin` and `full_check_seed_of_accepted` (the video's facts file too). Logging only. The logging test has five new checks and passes 28 of 28 (exit 0).
3. **Rerun of both layouts**, same seeds and settings as the last two runs:

| | Staging | Moved |
|---|---|---|
| Run folder | `data/demo_record/rerun_staging_20261113_000814` | `data/demo_record/rerun_moved_20261113_001429` |
| Seeds (read back from `run_info.json`) | sampler 3397871272, full check 4242 | sampler 1944726090, full check 4242 |
| Outcome | SUCCESS | SUCCESS |
| Accepted candidate | draw 32, base (0.7639, 1.0908), heading -88.45 deg, pedestal 0.1644 m: the same as before | draw 15, base (0.6851, 1.1758), heading -97.96 deg, pedestal 0.0986 m: the same as before |
| Drawn / rejected by body check / rejected by full check / not checked | 51 / 1 / 4 / 45 (same as before) | 51 / 1 / 11 / 38 (same as before) |
| Candidates that reached the dry run | only the accepted one (as before) | only the accepted one (as before) |
| Search time to the accepted candidate | 235.5 s (before 275.6 s) | 627.7 s (before 631.1 s) |
| Whole run | 6 min 03 s (before 7 min 13 s) | 12 min 45 s (before about 12 min 51 s) |
| Dry run of the accepted candidate | 17.0 s (before 17.9 s) | 18.5 s (before 18.8 s) |
| Peak memory | 4.3 GB | 4.3 GB |

The accepted candidate did not change, and the corrected dry run rejected nothing that the old one passed: in both runs the dry run only ever saw the accepted stance, and it passed. The search time differences (staging 40 s faster) are in the full-check part and the machine's load, not the dry run (about 1 s). The place-down, open, retreat and home traces changed as expected: place down now ends at pad height 1.0850 m (staging) and 1.0842 m (moved), against 1.1909 m and 1.1904 m before; the real arm's pad reached 1.0918 m and 1.0869 m. Place down takes 181 ticks instead of 292, place home 368 (staging) and 414 (moved) ticks instead of 292 and 494.

## Arrival at the stance

Identical to the previous runs, so the run is repeatable: staging 3.9 mm (0.4 in x, 3.9 in y), heading 0.015 deg, pedestal -0.51 mm, at 65.6 s; moved 4.1 mm (-1.0 in x, 4.0 in y), heading 0.17 deg, pedestal -0.58 mm, at 67.5 s.

## Dry-run pose against the real arm, new runs (staging / moved)

Largest joint difference in radians (real minus dry run), hand and elbow in mm.

| Task point | Largest joint (rad) | Hand, world (mm) | Hand, body (mm) | Elbow (mm) |
|---|---|---|---|---|
| Pick approach (over the beaker) | 0.024 / 0.035 | 1.0 / 2.2 | 4.7 / 6.1 | 2.9 / 2.3 |
| Pick standoff | 0.019 / 0.022 | 0.5 / 0.7 | 4.0 / 4.4 | 1.8 / 1.6 |
| Grasp | 0.030 / 0.035 | 1.3 / 1.3 | 5.2 / 4.9 | 1.8 / 2.8 |
| Lift | 0.018 / 0.025 | 0.8 / 0.4 | 3.8 / 4.5 | 1.1 / 3.3 |
| Place approach (carry) | 0.042 / 0.049 | 7.0 / 5.2 | 9.1 / 7.4 | 4.4 / 5.3 |
| Place down | 0.024 / 0.033 | 7.0 / 2.8 | 8.7 / 5.1 | 1.7 / 2.7 |
| Retreat | 0.028 / 0.014 | 4.2 / 1.7 | 5.4 / 3.4 | 1.1 / 2.9 |
| Home | 0.001 / 0.003 | 4.0 / 4.3 | 0.5 / 0.1 | 4.0 / 4.6 |

For comparison, the check pose still differs from the real arm by up to 1.06 rad at the lift (elbow up to 144 mm) in the pick, and is within 0.087 rad in the place. Per-point detail is in `20261113_0030_corrected_dry_run_rerun_files/` (`rerun_compare_*.json`).

## Verdict

Yes: the dry-run pose is now a good picture of the real arm at every task point of both runs. The largest joint difference anywhere is 0.049 rad (2.8 degrees), the hand is within 7 mm in the world frame (within 9.1 mm in the body frame, which includes the real arm's own tracking error and the base offset of about 4 mm) and the elbow within 5.3 mm. The place down and retreat, which were 0.42 to 0.47 rad and about 10 cm off before the fix, are now within 0.033 rad and 7 mm. The largest remaining differences are at the place approach (up to 7 mm hand, 0.049 rad), where the real arm is still settling when the log row is taken. Two runs, one stance each: indicative, not statistics. The arm-pose picture from the stance check is still the poorer one for the pick (unchanged by this task).

## Not done / for David

* Not done: no full test suite, only the tests that hold dry-run output and the logging test (the other tests do not use this function); no push; the video runs were not made; the older accepted candidates in the earlier logs were not rechecked at the corrected height (their stance data is not in those logs).
* Seeds: the Nov 10 runs still have no recorded full-check seed. New runs record it.
* The check pose's pick-side arm shape (up to 1 rad off the real arm) is untouched and is still a decision for David (the ghost robot for the pick).

## Commits and rows

52b0eb3 (fix, M318 approved), 386cef0 (seeds, M319), this report with row M320.
