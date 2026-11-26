# Stage 2 finished: the sag layer runs every control tick, relaxation test, recheck (2026-11-25, David's answers 1 to 3)

Branch `safety-filter`, HEAD 9cfa15d before this round. One MuJoCo process at a time. Runs in `data/logs/rebuild_s2e/` (queue `queue_s2e.sh`, log `progress.txt`). Nothing pushed. Baseline worktrees kept.

**Stage 2 PASSES** on the criteria below. Criterion C now passes (both held runs SUCCESS). The earlier perturbed failure (CLOSE, gripper stop-check) did not recur in this run; David's decision that it is a gripper bug stands, and it does not block.

## Plain-language answer

1. **The sag layer now runs every control tick.** Before, it ran only when a source sent a new arm command. Now, on a tick with no new arm command, the layer runs on the arm's last source target plus its correction, and the bridge takes that. The skip rules (first tick, bridge suspension, non-finite) are unchanged, and there is no special rule for stillness. The change is in `src/control_loop.py` (`sag_held`, and a block after the source commands), not in the layer.
2. **The relaxation test passes.** With the arm holding still and a wrong estimate of twice the droop, the estimate relaxes toward the droop with the 2 s smoothing: 13.1% of the initial error left at 4 s (model 13.5%) and 4.8% at 6 s (model 5.0%).
3. **Teleop, correction on, with the every-tick update.** During the out-of-view pause the estimate now keeps moving, and it relaxes from 37.9 mrad at the pause start to 11.1 mrad at its end. Hand error 0.5, 1 and 2 s after the pause: **17.1, 15.7 and 10.3 mm** (correction on), against **3.1, 2.7 and 2.6 mm** (off). The after-pause numbers are report-only.
4. **Teleop is not yet fixed.** The estimate relaxes toward the droop, but slowly: the hand error is still 10 mm two seconds after the pause, against 2.6 mm off. The estimate's build-up during the engagement motion (report 20261125_0100) is what the relaxation has to undo, and it does so with the 2 s smoothing. David decides on any weighting after Stage 3 (answer 2).
5. **Move-and-stop at 0.5 rad/s is unchanged**: 0.0233 mm nominal and 0.0291 mm perturbed one second after each stop, identical to 9cfa15d. The source in that test sends a command every tick, so the new every-tick branch does not run there.
6. **Timing:** the held run's sag call averages 0.225 ms (p99 0.72 ms), up from 0.187 ms (p99 0.55 ms), because the layer now runs on 21,052 calls instead of 7,569. Policy A's calls are unchanged (its source already sends every tick).

## Criteria, on the final code

| criterion | result |
|---|---|
| A. Regression: sag_layer 13/13, sag_solver_free 13/13, sag_feedback 8/8 (B2 changed, see below), sag_relax 3/3, bridge_clamp 8/8, command_bridge 19/19 (sag off), source_view 3/3, sag_feedforward exit 0, safety-filter group 18/18 exit 0 | **PASS** |
| B. The four staging tests reach home | **PASS** (exit 0 each) |
| C. Held run nominal and perturbed SUCCESS | **PASS** (both `SUCCESS; plan failure None; status ['done'×4]`) |
| D. Policy A, correction on lower hand error than off (3 seeds) | **PASS**: 0.18 mm on, 1.07 mm off (last 2 s: 0.25 against 1.00) |
| E. Teleop offset never at the clip | **PASS**: |C_p| peak 29.6 mm, 0 clip ticks (correction on) |
| F. Five holds, model error | Reported: clip 0, peak 6.60 mm, hand error 1 s after each stop 1.9 to 3.4 mm (unchanged) |
| Move-and-stop 0.5 rad/s, not worse than 9cfa15d (0.024 nominal, 0.029 perturbed, 1 s) | **PASS**: 0.0233 and 0.0291 mm, identical to 9cfa15d |
| Teleop after pause (report only) | Reported above |

## Changes in this round

- `src/control_loop.py`: `sag_held` (the last source arm target per arm); an arm command sets `arms_now`; on a tick with no arm command the layer runs on the held target and the bridge takes the correction (bridge on, and bridge off as in the normal path).
- `experiments/sag_relax_test.py` (new): the relaxation test above.
- `experiments/sag_feedback_test.py`: B2 changed from "one correction per arm command" to "every arm command is corrected, and the correction also runs on the ticks in between (answer 1)". The old check failed for the intended reason (294 corrections for 15 commands).
- Decision rows: the every-tick update is answer 1 (this report); the stillness rule is none (answer 1).

## Timing (`sag_timing_probe.py`, per call, ms)

| run | before (9cfa15d) | now |
|---|---|---|
| held nominal: sag calls / mean / p99 | 7,569 / 0.187 / 0.547 | 21,052 / 0.225 / 0.723 |
| held nominal: bridge p99 | 0.419 | 0.355 |
| policy A s1 on: sag mean / p99 | 0.187 / 0.613 | 0.222 / 0.643 |

## Teleop, after the pause (correction on and off; report only)

Pause: the out-of-view window starts at 5.41 s (the same in both runs; the fake operator's cycle). The reference-based pause detector finds no pause in the correction-on run now, because the correction moves the reference every tick, so the window is taken from the schedule.

| correction | hand error 0.5 s after (mm) | 1 s after (mm) | 2 s after (mm) | estimate at the pause start (mrad) | mean during the pause (mrad) | at the end (mrad) |
|---|---|---|---|---|---|---|
| on | 17.14 | 15.74 | 10.34 | 37.89 | 20.88 | 11.06 |
| off | 3.07 | 2.66 | 2.60 | 0 | 0 | 0 |

Bridge rejections in the teleop runs: 19 (correction on) and 17 (off); one of them falls inside the pause in each run. **Answer 1's and Stage 3's expectation was zero rejections.** Stage 2 still has the rejection rule; Stage 3 removes it, and its teleop rerun is the check.

## Tests changed in this round

- `experiments/sag_feedback_test.py`, B2: see above (reason: the correction now runs every tick).
- New: `experiments/sag_relax_test.py`.

## Questions for David

None blocking. Answer 2 (s0) waits for the teleop rerun after Stage 3.
