# Held-object ramp and PLACE OVER delay: validation runs (2026-11-01)

Written for: David (project lead), to decide whether the ramp or the delay is worth carrying forward.

## Question

The 2026-10-29 diagnosis found that the held-object compensation step starts a 3.4 deg tilt peak of the beaker about 37 ticks after the held step (tick 11636). Two candidate fixes were tested in scratch runs, with no change to `src/` or `src/task_config.py`:

1. **Ramp:** blend the held term in over 75 ticks (0.5 s at 150 Hz) instead of stepping it.
2. **Delay:** hold the PLACE OVER command for 0.6 s after the held step before the arm moves on to DOWN.

## Runs

Three episodes on the moved-layout run (same recording setup as the 10-29 blend20 episode), each one sample:

| name  | variant                        | folder                                        |
|-------|--------------------------------|-----------------------------------------------|
| base  | current held term, no ramp, no delay | `data/diag/delay_20261029/base_20261101_1207/`  |
| ramp  | held term ramped over 75 ticks | `data/diag/delay_20261029/ramp_20261101_1207/`  |
| delay | PLACE OVER held 0.6 s          | `data/diag/delay_20261029/delay_20261101_1207/` |

Runs were done in the `var2` tmux chain (base, xbase, ramp, xramp, delay, xdelay), all exit code 0. Per-tick extracts are `data/diag/delay_20261029/{base,ramp,delay}_ticks_20261101_1207.csv` (git-ignored). Analysis: `data/diag/delay_20261029/analyze_variants.py`, output `data/diag/delay_20261029/out_1207/` (`metrics.txt` and three plots).

## Results

Ticks are relative to the held step at tick 11599. Tilt is in degrees.

| metric                         | base  | ramp  | delay |
|--------------------------------|-------|-------|-------|
| first tilt peak after held (tick, deg) | 37, 3.41 | 107, 1.53 (after release) | 47, 3.93 |
| max tilt before release (deg)  | 3.41  | 1.15  | 3.93  |
| onset (ticks after held)       | 4     | 31    | 4     |
| second peak (tick, deg)        | 83, 2.76 | 294, 0.78 | 156, 3.87 |
| DOWN bump (deg)                | 0.80  | 0.78  | 0.64  |
| settle (ticks after held)      | 315   | 318   | 454   |
| max track error (rad)          | 0.040 | 0.037 | 0.040 |
| max command step (rad/tick)    | 0.00326 | 0.00314 | 0.00333 |

Other checks:
- **Ramp:** the measured held-term fraction matches the intended ramp to within 0.0013 at every sampled tick (tick 11609: 0.147 vs 0.147; tick 11636: 0.507 vs 0.507). Targets and joint positions match the base run exactly before the held step.
- **Delay:** the DOWN phase starts 90 ticks later (tick 11944 vs 11854, i.e. 0.6 s). The delayed arm motion matches the base motion shifted by 96 ticks: the largest joint displacement difference is 0.045 rad, against 0.26 rad of total displacement over the 200 ticks. The DOWN endpoint differs by 0.007 rad at most.

## Reading

- **The ramp halves the early peak.** Tilt before release drops from 3.41 to 1.15 deg, and the onset is pushed from tick 4 to tick 31. This agrees with the 10-29 finding that the early bounce is the arm relaxing after the held step.
- **The ramp moves the peak, it does not remove it.** The ramp run's largest post-held peak is 1.53 deg at tick 107, just after release. It is smaller than the base run's first peak (3.41 deg) and its second peak (2.76 deg at tick 83), so the ramp lowers the worst peak from 3.41 to 1.53 deg, but a peak remains.
- **The delay does not reduce the peak.** The delay run's first peak is 3.93 deg, slightly higher than base, and its second peak is 3.87 deg at tick 156. Holding the place command does not stop the early wrist motion. It only postpones DOWN, and settling takes about 140 ticks longer (454 vs 315).
- **Single samples.** Each variant is one episode. The 1.53 vs 3.41 difference is large compared with the tick-to-tick changes seen in the base run, but the repeat-run spread was not measured here.

## Caveats

- Phase labels differ from the base run from tick 12153 (end of PLACE OVER, 5 ticks earlier in the ramp run and about 90 ticks later in the delay run). The phase labels are not used for any metric before tick 12153.
- The delay run recorded 13694 ticks, against 13612 for base; metrics use the common ticks (1 to 13612).
- The ramp here is 0.5 s. The 2026-10-28 log records that a 1.0 s ramp (M195 in the working plan) failed the PLACE DOWN step at a 3 mm waypoint tolerance. This run does not test 1.0 s, so that failure is not explained here.

## Recommendation (not decided)

- The ramp is the only one of the two that reduces the first peak, so it is the candidate to carry forward. The ramp length (0.5 s here, 1.0 s earlier) is a design value for David.
- Do not carry the delay forward as a fix for this bounce. It adds 0.6 s per placement and does not reduce the peak.
- The 1.53 deg post-release peak in the ramp run is not analysed here. Check whether it is the same wrist overshoot before counting the ramp as a full fix.

No code, config, or baseline changed. Outputs are in `data/diag/` (git-ignored).
