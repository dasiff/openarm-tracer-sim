# Lift-off weighing, run 1: analysis from the saved files

Branch: held-weight-ramp-sag-off. Read only: no simulation was run for this analysis. Files: data/bounce_liftoff_20261102/run1/ (trace, episode log, recording), data/logs/liftoff_run1_20261102_211656.log (run log). Script: experiments/bounce_liftoff_metrics.py (reruns every number below).

Plain-language key: the "pad" is the fingertip contact point of the left gripper. "Trim" is the sag correction (the small offset the arm adds to its target so it does not sag under load). Steps: IN (move the pad into the grip position), CLOSE (close the gripper), WEIGH (raise the held mass slowly until the beaker leaves the table), UP (lift), OVER (move over the hotplate), DOWN (lower), OPEN (open the gripper), BACK (move away), HOME (return the arms). Height is z, in millimetres (mm) unless stated.

## Step 0: state before the analysis

- No MuJoCo or Python simulation process was running. No tmux session was running.
- git status: the lift-off logging changes in src/mpvi_modules.py and src/gravity_ff.py were uncommitted; the last commit was 90b8ce6 (VM stall report).

## Part A

### 1. Steps (the run log has no FAILED line, and each step appears once in the trace, so every step passed on its first attempt)

| step | start (s) | time to next step (s) | pad error to waypoint at last tick (mm) | pad z minus reference z at last tick (mm) |
|---|---|---|---|---|
| pick OVER | 65.98 | 1.53 | 2.41 | -1.02 |
| pick DOWN | 67.51 | 3.68 | 0.40 | -0.37 |
| pick IN | 71.19 | 3.54 | 1.29 | 0.52 |
| pick CLOSE | 74.73 | 1.01 | not available (no waypoint) | not available |
| pick UP | 80.41 | 2.99 | 0.79 | -0.50 |
| place OVER | 83.41 | 1.68 | **6.54** | 1.06 |
| place DOWN | 85.09 | 2.06 | 0.63 | 0.58 |
| place OPEN | 87.15 | 1.39 | not available | not available |
| place BACK | 88.55 | 1.34 | **3.11** | -1.48 |
| place HOME | 89.89 | 5.39 | not available | not available |

- Place: from OVER start (83.41 s) to the end of HOME (95.27 s) is 11.85 s. The release path, OVER to BACK, is 5.13 s.
- Final beaker position on the hotplate: x 0.6500 m, y 0.6595 m, z 1.0279 m. The hotplate top site in the scene is at x 0.6500 m, y 0.6600 m, z 1.0280 m. The beaker sits 0.5 mm from the site in the horizontal plane and 0.1 mm below its height. The run's outcome line is SUCCESS. The beaker's upright pose was not checked here; the SUCCESS line is the plan's own check.

Open question: the pad error at the last tick of place OVER is 6.54 mm, and BACK is 3.11 mm. Both are above the 3 mm waypoint tolerance (tol_m in src/task_config.py), yet both steps passed. The last tick may fall inside the blend into the next move (blend_radius_m is 20 mm). I did not check this further (see "Needs decision" below).

### 2. Weighing

- Rate ramp: 0.0510 kg/s (0.500 N/s), starting at 0 kg. It started at tick 11181 (75.940 s).
- Lift-off detected at tick 11851 (80.407 s): 4.47 s after the ramp started.
- Force at detection: 2.2367 N. Raw held mass at detection: 0.2280 kg.
- Pad rise at detection: 0.311 mm above the baseline (mean pad height over the 10 ticks before the ramp).
- Hand height per kilogram (held_pad_z_per_kg): 0.08984 m/kg.
- Corrected mass = raw minus rise divided by hand height per kg = 0.2280 - 0.00031/0.08984 = 0.2245 kg.

| value | kg | error vs true 0.200 kg | error |
|---|---|---|---|
| raw (at detection) | 0.2280 | +28.0 g | +14.0 % |
| corrected (the estimate the run used) | 0.2245 | +24.5 g | +12.3 % |

The 0.2245 kg in the run log is the corrected value. The log prints the raw 0.2280 kg in the same line. The true mass is 0.200 kg (glass 0.100 kg plus liquid 0.100 kg).

Beaker and table, from the trace:
- The beaker had 4 table contacts from the grip until tick 11665 (79.167 s).
- A 4-tick glitch during the grip, ticks 11002 to 11005 (74.747 to 74.767 s): the beaker jumped about 0.5 mm and the table-contact count dropped to 0. It settled back to its original height by tick 11010, and the count returned to 4. No lasting effect.
- From tick 11666 (79.173 s) the count falls to 1 to 3 with brief returns to 4. It reaches 0 for the first time at tick 11783 (the hand has not yet reached 0.3 mm), and is 0 for good from tick 11834 (80.293 s). At that tick the pad is 0.291 mm above its baseline and the beaker is 0.290 mm above its height at the ramp start.
- The hand reaches 0.3 mm at tick 11848 (80.387 s), and detection is at tick 11851.
- Answer to "before or after the hand rose": the beaker leaves the table with the hand, not before it. Its rise matches the pad's rise to 0.001 mm, and it has lost table contact for good 14 ticks (about 0.09 s) before the hand reaches the 0.3 mm threshold.
- Pad slip: the difference between beaker height and pad height is -53.091 mm at the ramp start and -53.306 mm after UP. The change of 0.2 mm is the most any slip could account for. No slip is visible at the resolution of the trace.
- Grip command and applied finger travel are both 0 m during the weighing (the fingers are already closed on the beaker).

### 3. Trim over the lift (sag correction, in mm and degrees)

The trim is frozen while the weighing runs (the freeze flag is 1 from the ramp start to detection) and is released before UP ends (the flag is 0 at UP end).

| when | tick | x (mm) | y (mm) | z (mm) | rot x (deg) | rot y (deg) | rot z (deg) |
|---|---|---|---|---|---|---|---|
| at ramp start | 11181 | -2.081 | -2.660 | -1.308 | -0.569 | -0.157 | -0.786 |
| at detection | 11851 | -2.081 | -2.660 | -1.308 | -0.569 | -0.157 | -0.786 |
| after UP finishes | 12300 | +2.463 | -0.668 | -3.562 | +0.876 | +0.570 | +0.519 |
| change over the lift | | +4.543 | +1.992 | **-2.254** | +1.446 | +0.727 | +1.305 |

The z change over the lift is -2.254 mm in this run. The earlier build gave +18.85 mm (from the brief). The freeze holds the trim still during the weighing, so the weighing does not ride on a moving trim.

### 4. Metric 1 (pad height; baseline = the tick before the held mass first becomes nonzero)

Definitions as in experiments/bounce_handoff_metrics.py (section 6). Baseline tick 11180, pad height 1012.910 mm. Window: 200 ticks after the baseline (ticks 11181 to 11381).

- Rise +0.077 mm; drop 0.000 mm; swing 0.077 mm.

This window ends about 470 ticks before lift-off, so it measures none of the lift. For information only, the same measure over the whole held period (tick 11181 to the end of UP) gives a rise of 107.3 mm. That is the lift itself, not a bounce, so it is not a bounce measure.

### 5. Metric 2 (pad height minus reference pad height; window from the CLOSE start to 3 s after the end of DOWN)

- Window: tick 10999 (74.73 s) to tick 13312 (90.15 s).
- Maximum +4.990 mm at tick 12602. Minimum -4.399 mm at tick 12659. Value at the end of the window -0.761 mm.

The brief's text for metric 2 is cut off after the window, so only these values are given.

## Needs decision (not built)

1. **Metric 1 window.** The 200-tick window after the held mass starts ends before lift-off, so it cannot show a lift effect. Options: (a) keep it as defined and add a separate window from lift-off to the end of UP; (b) redefine the window from lift-off on. Which one?
2. **Metric 2 definition.** The brief is cut off after the window. I need the rest of the definition to report it as intended.
3. **Place OVER tolerance.** The pad error at the last tick of place OVER is 6.54 mm (BACK 3.11 mm), above the 3 mm tolerance, yet the steps passed. Options: (a) check whether the last tick is inside the blend (20 mm), and if not, find out why the step passed; (b) accept the blend's last tick as the measure and change how the error is recorded. Which?
4. **Sweep specification.** The brief's text stops before any sweep section. I ran the six combinations saved in docs/progress.md (rate, threshold): (0.25, 0.3), (0.5, 0.3), (1.0, 0.3), (2.0, 0.3), (0.5, 0.1), (0.5, 1.0), all with --pick-only. Confirm these, or send the intended list.
5. **Hotplate position (open from earlier):** the configured hotplate_xy is 0.80 m and the scene uses 0.65 m (the moved layout). The beaker landed on the scene's site, so the run is consistent with the scene. Whether the config value should change is still open.

## Status

- Commit 19ce0cc (WIP): the lift-off logging, the removals of the "Code generated by Claude" marker lines (David's decision), and experiments/bounce_liftoff_metrics.py. Not pushed.
- Sweep: started at 01:19 in tmux session liftoff_sweep (six combinations, one after another). Progress is in data/logs/liftoff_sweep_chain_20261103_011900.log.
