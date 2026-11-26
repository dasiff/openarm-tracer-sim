# ARMS stall: start-pose and hotplate tests (Part 1), and the Step 2 timing gate (Part 2, step 0) (2026-11-23)

Branch `safety-filter`, HEAD `11f46bf` before this commit. Measurement only: no production code changed (the scratch start-pose and hotplate changes are in the probe script's flags and were not kept in `src/`). One MuJoCo process at a time. Not pushed. **Part 2 stopped at its timing gate and was not built.** Steps 1 to 3 of Part 2 were not run.

## Part 1: start-pose and hotplate tests

### Method

`experiments/scene_stall_probe.py` (two new scratch flags, `--start-margin-offset` and `--hotplate-x`) on the grasp test, with the trim on, all runs to 31.5 s:
- **ref**: unmodified grasp test (the reference for the other grasp runs);
- **a. start pose**: left joints 2 and 4 set to 0.03 rad inside their margin bounds (joint 2 at −0.030 rad, joint 4 at 0.2045 rad) at the first tick, with their actuator commands set to the same values. Everything else unchanged.
- **b. hotplate 0.65 m**: the grasp test with the hotplate at x = 0.65 m (the held run's position), original start.
- **c. held run with the hotplate at 0.80 m**: **not run.** Moving the hotplate changes the held run's stance search (`demo_video`'s MOVED layout); with the hotplate at 0.80 m the search had reached candidate 26 without a pass after 20 minutes, so it is not the same stance. The run was stopped. Injecting the previous stance would need a change to the sampler, which is beyond a scratch edit.

Analysis: `experiments/start_hotplate_analyse.py`. Data: `data/logs/sf121_part1_20261123_2028/` (rerun outputs in the same folder; the first grasp attempt wrote no outputs and was redone).

### Results (left arm; the right arm behaves the same)

| Run | Arms reach home (within 0.03 rad) | Joint 2 on margin | Joint 4 on margin | Max deviation from home at 31.4 s | First tick off the reference |
|---|---|---|---|---|---|
| ref (grasp, unmodified) | never | 7.3 s to 31.4 s (24.15 s) | 3.65 s to 3.80 s (0.15 s) | 0.266 rad | — |
| a. start pose 0.03 rad inside the margins | never | 7.3 s to 31.4 s (24.15 s) | 3.00 s to 4.40 s (1.45 s) | 0.266 rad | 1.40 s (left), 3.70 s (right) |
| b. hotplate at 0.65 m | never | 7.3 s to 31.4 s (24.15 s) | 3.65 s to 3.80 s (0.15 s) | 0.266 rad | none: identical to ref in every solver output |
| c. held run, hotplate at 0.80 m | not run | | | | |

Notes:
- In run a the solver pulls joints 2 and 4 onto their margin bounds within about 0.1 s of the start (the commanded targets at rest are zero, below joint 4's bound), so the start offset is not held. The early trajectory changes (divergence at 1.40 s) but the end state is the same as the reference: joint 2 pinned from 7.3 s and no return home.
- Run b gives bit-identical solver outputs to the reference in the whole 31.5 s window. So the hotplate position has no effect on the grasp test's arm trajectory, which agrees with the earlier rebuild check (`experiments/arms_hotplate_check.py`, difference 0.0).

### Plain-language answer

- **Is the start outside the margin band the cause?** No, not alone. Moving the start 0.03 rad inside both margin bounds changes the early trajectory but not the outcome: joint 2 still goes onto its margin at 7.3 s and the arms never reach home.
- **Is the hotplate position the cause?** No, not alone. With the hotplate at 0.65 m, the grasp test is identical to the reference.
- **Both together?** Not tested (run c was not run).
- **So what separates the held run?** Neither single change. The held run's other settings are candidates and have not been tested: its beaker liquid mass (0.100 kg), the liftoff settings in the shim, and the orchestrator's wrapper around the staging. Its commanded trajectory and its solver inputs match the grasp test up to 3.69 s, so the difference enters later.

## Part 2: step 0, timing gate (stopped; nothing built)

### Method

`experiments/sag_timing_probe.py` (now times a scratch prototype of Step 2 on each call's Pass 1 output and the current learned offset; the layer's state is not changed). Runs: held run to 12 s (trim on) and policy-stream variant B (seed 1, correction on, bridge off, 30 s). Data: `data/logs/sf121_timing_20261123_2056/`.

Step 2 prototype: the learned offset as one linear joint change, dq = J^T (J J^T + damping·I)^-1 e, with the arm Jacobian at the pad centre computed on a scratch copy at Pass 1's joints (kinematics, centre of mass and the Jacobian), damping = MINK_CFG damping. Command = Pass 1 + dq, clamped to hard limits. No iteration.

| Run | Today's layer `correct()` mean / p99 (ms) | Solver step alone p99 (ms) | Step 2 mean / p99 (ms) | Layer + Step 2 p99 (ms) |
|---|---|---|---|---|
| Held run, first 12 s (trim on) | 1.60 / **4.82** | 2.33 | 0.26 / **0.71** | 5.72 |
| Policy variant B, 30 s | 1.77 / **5.74** | 4.20 | 0.31 / **1.13** | 6.51 |

Gate (David's spec): stop if Step 2 adds more than 0.5 ms at the 99th percentile. **Failed in both runs** (0.71 ms and 1.13 ms). Stopped; nothing built, no decision row written.

Notes on the numbers: today's layer p99 is not stable between runs (the held run gave 3.37 ms in the earlier timing and 4.82 ms here, on the same code path). The VM's load varies. The gate uses the 99th percentile of the same kind of run, so the failure is not sensitive to that: Step 2's mean is 0.26 to 0.31 ms, and the 99th percentile is 2 to 4 times its mean.

## Options for David (not decided)

1. **The budget.** Today's layer is already above 3.3 ms at the 99th percentile (3.4 to 5.7 ms in these runs). Is the per-tick budget for the whole layer 6.7 ms (a full tick), or is the 0.5 ms Step 2 figure a hard limit?
2. **Cutting Step 2's cost.** It recomputes forward kinematics and the Jacobian on a scratch copy. The Pass 1 pad pose is already known, so a cheaper Jacobian (reusing the kinematics of the solver's own configuration) may bring it under 0.5 ms. Not prototyped here, because the gate says stop.
3. **Which offset Step 2 uses.** The updated learned offset of this tick, or the previous tick's? This was not stated, so it is a question.
4. **The leak on a Pass 1 failure.** The spec says "do not update the learned offset"; the existing SF-118 leak still runs on that tick. Keep the leak, or freeze it?
5. **The error still contains servo lag.** The error is the previous tick's reference (Pass 1's output) minus the measured pad. Pass 1 removes the solver's lag, but the servo still lags its command by about 40 to 45 ms in joints 1 to 4 and about 90 to 110 ms in joints 5 to 7 (Part 3 of the gap split). So during motion the learned offset will still pick up the servo lag. Should the correction also remove the servo lag, or is that intended to be handled elsewhere?
6. **The start pose.** Joint 4 starts below its margin band, and the solver pulls joints onto their bounds in the first 0.1 s. Should the rest pose move inside the band, or should the margin start rule change?

## Not run

- Part 2 steps 1 to 3 (build, tests, the policy-stream variant matrix with clamp counts, the held run to SUCCESS): not run, because the timing gate failed.
- Part 1 run c (held run with the hotplate at 0.80 m): not run (the stance search changes; see above).
- The both-changes run (start pose and hotplate together): not run.

## Files

- `experiments/scene_stall_probe.py` (scene probe with the two scratch flags; a normal end of the script also writes its outputs; committed with this report)
- `experiments/start_hotplate_analyse.py` (analysis of the Part 1 runs; committed with this report)
- `experiments/sag_timing_probe.py` (timing probe; now times the Step 2 prototype; committed with this report)
- Outputs (git-ignored): `data/logs/sf121_part1_20261123_2028/` (ref_grasp, start_offset, hotplate065_grasp, `part1_analysis.json`), `data/logs/sf121_timing_20261123_2056/` (held, policyB)
