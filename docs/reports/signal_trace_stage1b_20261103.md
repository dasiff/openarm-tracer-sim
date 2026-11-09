# Stage 1b: provisional holding and stillness values, log only (Nov 4, 2026; decisions dated Nov 3)

## Summary in plain words

- **All three proof runs match.** With the switch off (R1), the two result files are byte-identical to the held baseline in `experiments/baselines/held_200g_20261103/`, and no `signal_trace.csv` appears. With the switch on (R2), the same two files are still identical, and the trace has the new columns with no empty ones. The manual bench (R3) matches the CURRENT baseline with zero difference in all 22 compared columns. So the new code changes no output.
- **What was built.** Six provisional values now live in one dictionary, `HOLD_STILL_CFG`, in `src/task_config.py`. The signal trace computes, for each hand, two raw condition flags, a holding verdict, hand speed, hand acceleration, a speed-below flag and a still verdict. Nothing reads these verdicts. Every value is marked PROVISIONAL and SIM-ONLY, and they must be recalibrated on real hardware.
- **How well the holding verdict agrees with the scripted skills (left hand, which holds the beaker).** The verdict turns on at tick 11014 and off at tick 13015, with no flicker in between. It is on for 2,001 ticks (13.3 s). Measured against the scripted "holding" window (from the end of CLOSE to the start of OPEN), it agrees on 14,095 of 14,382 ticks (98.0 %). The two disagreements:
  - It turns on 136 ticks **before** the CLOSE step's last tick. The fingers are inside the closure range while they are still closing, and the squeeze is already above 2 N.
  - It turns off 151 ticks **after** the OPEN step starts. The squeeze only drops once the release begins, so the verdict follows the squeeze.
- **The right hand** never holds in this run, and its verdict stays off, which is correct. Its squeeze never reaches 2 N.
- **Still verdict.** The left hand's still verdict is on for the whole scripted weigh step (702 of 702 ticks), and it is **off for all 150 ticks in the second just before place DOWN**. So, in this run, the arm is not still at that moment. There are also several short "on" stretches elsewhere (the shortest is 5 ticks, 0.033 s). Their cause is not checked.
- **Acceleration ranges** (signed, left hand; calculated from the speed differences): the approach has a median of about −0.008 m/s² and a p95 of about 0.21 m/s². Weigh is the calmest phase, with its range between −0.016 and 0.029 m/s². The closing step has the widest extremes, −5.2 to +5.5 m/s², from the start of the step. Most phases' ranges overlap with several others (the full table is below). No acceleration limit was chosen, as asked.
- **Closing on nothing (check, from the model, no simulation run).** With nothing between the pads, the two finger pads touch at a finger travel of about **0.00275 m** (2.75 mm), not at 0 m. Scanned with 0.05 mm steps on the robot-only model, which has no beaker in it. That position is **inside** the configured closure range (0.002 to 0.042 m). So the closure test alone cannot tell "closing on nothing" from "holding a thin object". Whether the squeeze test does is a calculated expectation, not a measured result: with the servo's stiffness of 600 N/m at 0.00275 m, the squeeze would be about 1.65 N, below the 2 N threshold. That has not been tested in a run.

Terms: a **verdict** is an on/off value computed from the raw signals. A **confirming time** is how long a condition must hold before the verdict changes. **Causal** means the value at a tick uses only that tick and earlier ones, so it could run live. **p95** is the value 95 % of samples fall at or below. **Stiffness** (kp) is the servo's spring constant. **SIM-ONLY PLACEHOLDER** means the number is for simulation and must be replaced with a measured value.

Labels: **shown by the data/code** (read from a file or a saved value, with file and line); **calculated from the data** (computed from saved values, method stated); **guess** (a judgement, not checked); **unknown** (the material does not say).

## Step 1: state

- Branch `held-weight-ramp-sag-off`. At the start: no tracked changes. Untracked: `deps/AgileX_Tracer`, `.claude/`, `compare_nosag.py`, `extract_height_data.py`, `measstart`.
- `uptime` 12:01: load average 6.20 on 4 CPUs. The load came from a system `unattended-upgrade` (a package update run by the operating system, not this project), not from a simulation. It was still running at the start and may have slowed the runs.
- `free -m`: 7424 MB total, 2175 MB used, 3319 MB free. `systemd-oomd` active. No tmux session or simulation process running.
- `origin/main` was `58e756f` (checked with `git ls-remote`).

## Step 2: config and code (commit 70b1cb7)

**Config.** `src/task_config.py`, a new dictionary `HOLD_STILL_CFG` at the end of the file (lines 645 to 664), the same flat style as the other dictionaries. A dictionary of plain values was chosen so it matches the file's existing pattern. Each value has a comment with: PROVISIONAL, SIM-ONLY PLACEHOLDER, the date it was chosen, that it is not from data, that it must be recalibrated on hardware, and the decision row M248. The squeeze comment states that 2 N is the TOTAL force (1 N per finger) and gives the sign convention (negative when squeezing):

- `squeeze_threshold_N` = 2.0 (line 649): newtons, total gripper force, absolute value of the signed actuator force.
- `grasp_closure_min_m` = 0.002 (line 653) and `grasp_closure_max_m` = 0.042 (line 654): metres of finger travel. The 2 mm margin at the closed end is a guess.
- `holding_confirm_ticks` = 15 (line 657): 0.1 s at 150 Hz.
- `still_speed_limit_m_s` = 0.005 (line 660): metres per second of hand (pad centre) speed.
- `still_confirm_ticks` = 75 (line 663): 0.5 s at 150 Hz.

**Logging code.** `src/signal_trace.py`:
- line 28: `from src.task_config import HOLD_STILL_CFG`.
- lines 113 to 147, `_verdicts()`: the verdict code, one hand at a time. It uses only the current tick and the stored values from earlier ticks (causal). The docstring gives the formula for speed and acceleration.
  - The squeeze test (line 126): `abs(force) >= squeeze_threshold_N`.
  - The closure test (line 127): `grasp_closure_min_m <= finger1 <= grasp_closure_max_m`.
  - Holding (lines 129 to 133): a counter of consecutive ticks in which both conditions hold. The verdict turns on when the counter reaches `holding_confirm_ticks`, and turns off when the squeeze fails. If the closure leaves the range while the squeeze stays high, the verdict is left unchanged. That case is not decided (see NOT BUILT).
  - Hand speed (line 137): `norm(pad_now - pad_previous) / dt`, metres per second.
  - Hand acceleration (line 141): `(speed_now - speed_previous) / dt`, signed, metres per second squared. "nan" on the first tick (and on the second for acceleration).
  - Still (lines 143 onward): a counter of consecutive ticks with speed below the limit. Still turns on at `still_confirm_ticks` and turns off as soon as the speed goes above the limit.
- line 150, `write()`: `dt` from consecutive times (line 155), and the verdicts appended for each hand (line 171).
- The module docstring (lines 1 to 10) now says the module computes log-only verdicts, and that nothing reads them.

**Switch.** The verdict code runs only inside `SignalTrace`, which exists only when `OPENARM_SIGNAL_TRACE=1` (see `src/episode_runner.py`, unchanged in this stage). With the switch off, none of this code runs.

**Compile and diff.** `python -m py_compile src/signal_trace.py src/task_config.py`: OK. The diff is saved at `data/signal_trace_1b_20261103/stage1b.diff` (147 lines).

**Synthetic check (no simulation).** A scratch script in `/tmp` fed the verdict code made-up sequences: holding turned on on the 15th tick (index 14), turned off at once when the squeeze dropped, and held its state when the closure left the range with the squeeze high. Speed was 0.150 m/s for 1 mm per tick, acceleration 0 for constant speed, and still turned on at index 75. Result: all as expected.

## Step 3: proof runs (one at a time, tmux, 6 GB cap)

The baseline README gives the launcher as `data/held_run/run_mass_wt.py`, which lived only in the scratch worktree deleted on Nov 3. The original `data/bounce_check_20261103/run_mass.py` is the same script with `ROOT` set to this repo, which is what these runs need. **Command used:** `python data/bounce_check_20261103/run_mass.py <out> --liquid 0.100`, inside `experiments/run_capped.sh`, under `flock data/locks/mujoco.lock`.

| Run | Switch | Outcome | Wall clock | Peak memory | Exit |
|---|---|---|---|---|---|
| R1 | off | liftoff_ticks.csv and episode_log.csv **byte-identical** to the baseline; no `signal_trace.csv` | 4 min 09.06 s | 3,825,320 kB (3.83 GB) | 0 |
| R2 | on | both files **byte-identical** to the baseline; `signal_trace.csv` 66 columns, 14,382 rows, **no empty column** | 3 min 06.77 s | 4,331,052 kB (4.33 GB) | 0 |
| R3 | on (bench) | 84 rows, **zero difference** on all 22 compared columns; sim-time difference 0 | 0 min 09.42 s | 2,215,528 kB (2.22 GB) | 0 |

Logs: `data/logs/s1b_R1_off_20261104_120301.log`, `s1b_R2_on_20261104_120722.log`, `s1b_R3_bench_on_20261104_121042.log`. Outputs: `data/signal_trace_1b_20261103/`. No run was killed or retried. Memory pressure: none was reported, and the peak memory stayed under the 6 GB cap; whether the system's load (see step 1) slowed the runs was not measured.

R2 has 14,382 data rows: the same extra final tick (14381) seen in stage 1.

## Step 4: commit

- **Commit `70b1cb7`**: "Stage 1b: provisional holding and stillness values in HOLD_STILL_CFG; log-only verdicts and hand acceleration in the signal trace (no behavior change)". Two files, explicit paths: `src/task_config.py` and `src/signal_trace.py`.

## Step 5: decision log and progress (commit 85b77ce)

- Decision **M248** added: "Provisional holding and stillness values (David, 2026-11-03)". The row lists all six values with units and what they depend on, the closure rule being general, the log-only status, and the NOT decided list. A detail section with a table of the six values is above the OPEN section.
- Notes added beside M246 items 1, 2 and 3 (not rewritten), pointing to M248.
- A progress entry added.
- Commit **`85b77ce`**: "Decision M248: provisional holding and stillness values (log only); notes under M246; progress entry". Two files, explicit paths.

## Step 6: every PROVISIONAL in the repo

Searched with `git grep` on HEAD (case-sensitive word `PROVISIONAL`), and the lowercase word "provisional" for older uses.

**Added today (uppercase PROVISIONAL, all 10 hits, all new in this stage):**

| File | Line | Note |
|---|---|---|
| `src/task_config.py` | 642 | Header comment of `HOLD_STILL_CFG` |
| `src/task_config.py` | 646 | Squeeze threshold (`squeeze_threshold_N`) |
| `src/task_config.py` | 650 | Grasp range, low end (`grasp_closure_min_m`) |
| `src/task_config.py` | 655 | Holding confirming time (`holding_confirm_ticks`) |
| `src/task_config.py` | 658 | Still speed limit (`still_speed_limit_m_s`) |
| `src/task_config.py` | 661 | Still confirming time (`still_confirm_ticks`) |
| `src/signal_trace.py` | 7 | Docstring: the thresholds are the PROVISIONAL values |
| `docs/mpvi_decisions.md` | 261 | M248 row |
| `docs/mpvi_decisions.md` | 409 | M248 detail section |
| `docs/progress.md` | 539 | Progress entry |

The 2 mm guess is also flagged in its comment (`src/task_config.py` line 652: "is a GUESS").

**Older (lowercase "provisional", placeholders or settings that were never decided; not uppercase):**
- `src/control_loop.py` line 67: "`cfg["settle_*"]` (provisional)" (settle thresholds for the base, pedestal and arms).
- `src/control_loop.py` line 133: "`# "settled" thresholds ... provisional`".
- `src/task_config.py` line 491: "says the checker gets the head camera; provisional (row M016)" (the VLM checker's camera setting).
- `docs/design_plan.md` lines 242 and 300, and `docs/mpvi_decisions.md` line 28, and `docs/reports/20261021_1050_vlm_checker_alignment.md` line 86: the VLM checker's camera and model choices, marked provisional in those files.
- `docs/reports/signal_survey_20261103.md` line 62: refers to the settle thresholds as "provisional".
- Additional lowercase lines in the 2026-11-03 notes (`docs/mpvi_decisions.md` 380, 385, 407, 429) are this stage's own notes.

## Step 7: verdicts against the scripted steps (R2 trace)

The full script output (the file `/tmp/signal_trace_1b_20261103/verdicts.md`) is below. It is read-only analysis.

```
## Scripted step tick ranges (first, last tick with that label)
- PICK: OVER: 9688 to 9916
- PICK: DOWN: 9917 to 10468
- PICK: IN: 10469 to 10999
- PICK: CLOSE: 11000 to 11150
- PICK: WEIGH: 11151 to 11852
- PICK: UP: 11853 to 12300
- PLACE: OVER: 12303 to 12554
- PLACE: DOWN: 12555 to 12863
- PLACE: OPEN: 12864 to 13072
- PLACE: BACK: 13073 to 13273
- PLACE: HOME: 13274 to 14080

## left hand: holding verdict
- on/off changes: 2 (ticks: [11014, 13015])
  - tick 11014 -> ON (step: 2/4 PICK: CLOSE [running]; squeeze_ok 1, closure_in_range 1)
  - tick 13015 -> OFF (step: 4/4 PLACE: OPEN [running]; squeeze_ok 0, closure_in_range 1)
- ON stretch: ticks 11014 to 13014 (2001 ticks); steps at start/end: 2/4 PICK: CLOSE [running] / 4/4 PLACE: OPEN [running]
- agreement with 'scripted holding' (CLOSE end to OPEN start, ticks 11150 to 12863): 14095 of 14382 ticks agree
  - disagree ticks 11014 to 11149 (136 ticks): verdict ON, step 2/4 PICK: CLOSE [running]
  - disagree ticks 12864 to 13014 (151 ticks): verdict ON, step 4/4 PLACE: OPEN [running]
- squeeze_ok ON ticks: 2015; closure_in_range ON ticks: 5708

## right hand: holding verdict
- on/off changes: 0 (ticks: [])
- agreement with 'scripted holding' (CLOSE end to OPEN start, ticks 11150 to 12863): 12668 of 14382 ticks agree
  - disagree ticks 11150 to 12863 (1714 ticks): verdict OFF, step 2/4 PICK: CLOSE [running]
- squeeze_ok ON ticks: 0; closure_in_range ON ticks: 3690

## still verdict (both hands): every stretch ON

### left hand
- ticks 75 to 423 (349 ticks = 2.327 s); steps at start/end: 1/4 NAV: ARMS [running] / 1/4 NAV: ARMS [running]
- ticks 4150 to 4481 (332 ticks = 2.213 s); steps at start/end: 1/4 NAV: ARMS [running] / 1/4 NAV: PEDESTAL [running]
- ticks 10987 to 11000 (14 ticks = 0.093 s); steps at start/end: 2/4 PICK: IN [running] / 2/4 PICK: CLOSE [running]
- ticks 11085 to 11884 (800 ticks = 5.333 s); steps at start/end: 2/4 PICK: CLOSE [running] / 2/4 PICK: UP [running]
- ticks 12299 to 12332 (34 ticks = 0.227 s); steps at start/end: 2/4 PICK: UP [running] / 4/4 PLACE: OVER [running]
- ticks 12893 to 12897 (5 ticks = 0.033 s); steps at start/end: 4/4 PLACE: OPEN [running] / 4/4 PLACE: OPEN [running]
- ticks 12986 to 13014 (29 ticks = 0.193 s); steps at start/end: 4/4 PLACE: OPEN [running] / 4/4 PLACE: OPEN [running]
- ticks 13090 to 13098 (9 ticks = 0.060 s); steps at start/end: 4/4 PLACE: BACK [running] / 4/4 PLACE: BACK [running]
- ticks 13957 to 14381 (425 ticks = 2.833 s); steps at start/end: 4/4 PLACE: HOME [running] / PLAN DONE
- ON during scripted WEIGH (ticks 11151 to 11852): 702 of 702 ticks
- ON in the 1 s just before place DOWN (ticks 12405 to 12554): 0 of 150 ticks

### right hand
- ticks 75 to 423 (349 ticks = 2.327 s); steps at start/end: 1/4 NAV: ARMS [running] / 1/4 NAV: ARMS [running]
- ticks 4150 to 4481 (332 ticks = 2.213 s); steps at start/end: 1/4 NAV: ARMS [running] / 1/4 NAV: PEDESTAL [running]
- ticks 9733 to 14381 (4649 ticks = 30.993 s); steps at start/end: 2/4 PICK: OVER [running] / PLAN DONE
- ON during scripted WEIGH (ticks 11151 to 11852): 702 of 702 ticks
- ON in the 1 s just before place DOWN (ticks 12405 to 12554): 150 of 150 ticks

## hand acceleration (signed, m/s^2), per phase, left hand: min / median / p95 / max
- open, approach (PICK OVER, DOWN): n 781; ticks 9688 to 10468; min -0.51715, median -0.0080312, p95 0.20659, max 0.79462
- open, beaker between pads (PICK IN): n 531; ticks 10469 to 10999; min -0.14445, median -0.0063818, p95 0.090576, max 0.40383
- closing (PICK CLOSE): n 151; ticks 11000 to 11150; min -5.2005, median -0.00011271, p95 0.0027765, max 5.4673
- closed, WEIGH (at rest until lift-off): n 702; ticks 11151 to 11852; min -0.016384, median -6.0691e-05, p95 0.0049833, max 0.02895
- lift (PICK UP): n 448; ticks 11853 to 12300; min -0.37362, median 0.0034552, p95 0.15045, max 0.20958
- carry / over (PLACE OVER): n 252; ticks 12303 to 12554; min -0.49846, median -7.058e-06, p95 0.66645, max 0.7092
- place DOWN: n 309; ticks 12555 to 12863; min -1.288, median -0.010502, p95 0.37297, max 0.50668
- opening (PLACE OPEN): n 209; ticks 12864 to 13072; min -0.75503, median -0.00074382, p95 0.095418, max 0.38831
- empty again (BACK, HOME): n 1008; ticks 13073 to 14080; min -0.2998, median 6.1605e-06, p95 0.15799, max 0.45465

## overlap of acceleration ranges between phases (min..max)
- overlap: 'open, approach (PICK OVER, DOWN)' and 'open, beaker between pads (PICK IN)' share [-0.14445, 0.40383] m/s^2
- overlap: 'open, approach (PICK OVER, DOWN)' and 'closing (PICK CLOSE)' share [-0.51715, 0.79462] m/s^2
- overlap: 'open, approach (PICK OVER, DOWN)' and 'closed, WEIGH (at rest until lift-off)' share [-0.016384, 0.02895] m/s^2
- overlap: 'open, approach (PICK OVER, DOWN)' and 'lift (PICK UP)' share [-0.37362, 0.20958] m/s^2
- overlap: 'open, approach (PICK OVER, DOWN)' and 'carry / over (PLACE OVER)' share [-0.49846, 0.7092] m/s^2
- overlap: 'open, approach (PICK OVER, DOWN)' and 'place DOWN' share [-0.51715, 0.50668] m/s^2
- overlap: 'open, approach (PICK OVER, DOWN)' and 'opening (PLACE OPEN)' share [-0.51715, 0.38831] m/s^2
- overlap: 'open, approach (PICK OVER, DOWN)' and 'empty again (BACK, HOME)' share [-0.2998, 0.45465] m/s^2
- overlap: 'open, beaker between pads (PICK IN)' and 'closing (PICK CLOSE)' share [-0.14445, 0.40383] m/s^2
- overlap: 'open, beaker between pads (PICK IN)' and 'closed, WEIGH (at rest until lift-off)' share [-0.016384, 0.02895] m/s^2
- overlap: 'open, beaker between pads (PICK IN)' and 'lift (PICK UP)' share [-0.14445, 0.20958] m/s^2
- overlap: 'open, beaker between pads (PICK IN)' and 'carry / over (PLACE OVER)' share [-0.14445, 0.40383] m/s^2
- overlap: 'open, beaker between pads (PICK IN)' and 'place DOWN' share [-0.14445, 0.40383] m/s^2
- overlap: 'open, beaker between pads (PICK IN)' and 'opening (PLACE OPEN)' share [-0.14445, 0.38831] m/s^2
- overlap: 'open, beaker between pads (PICK IN)' and 'empty again (BACK, HOME)' share [-0.14445, 0.40383] m/s^2
- overlap: 'closing (PICK CLOSE)' and 'closed, WEIGH (at rest until lift-off)' share [-0.016384, 0.02895] m/s^2
- overlap: 'closing (PICK CLOSE)' and 'lift (PICK UP)' share [-0.37362, 0.20958] m/s^2
- overlap: 'closing (PICK CLOSE)' and 'carry / over (PLACE OVER)' share [-0.49846, 0.7092] m/s^2
- overlap: 'closing (PICK CLOSE)' and 'place DOWN' share [-1.288, 0.50668] m/s^2
- overlap: 'closing (PICK CLOSE)' and 'opening (PLACE OPEN)' share [-0.75503, 0.38831] m/s^2
- overlap: 'closing (PICK CLOSE)' and 'empty again (BACK, HOME)' share [-0.2998, 0.45465] m/s^2
- overlap: 'closed, WEIGH (at rest until lift-off)' and 'lift (PICK UP)' share [-0.016384, 0.02895] m/s^2
- overlap: 'closed, WEIGH (at rest until lift-off)' and 'carry / over (PLACE OVER)' share [-0.016384, 0.02895] m/s^2
- overlap: 'closed, WEIGH (at rest until lift-off)' and 'place DOWN' share [-0.016384, 0.02895] m/s^2
- overlap: 'closed, WEIGH (at rest until lift-off)' and 'opening (PLACE OPEN)' share [-0.016384, 0.02895] m/s^2
- overlap: 'closed, WEIGH (at rest until lift-off)' and 'empty again (BACK, HOME)' share [-0.016384, 0.02895] m/s^2
- overlap: 'lift (PICK UP)' and 'carry / over (PLACE OVER)' share [-0.37362, 0.20958] m/s^2
- overlap: 'lift (PICK UP)' and 'place DOWN' share [-0.37362, 0.20958] m/s^2
- overlap: 'lift (PICK UP)' and 'opening (PLACE OPEN)' share [-0.37362, 0.20958] m/s^2
- overlap: 'lift (PICK UP)' and 'empty again (BACK, HOME)' share [-0.2998, 0.20958] m/s^2
- overlap: 'carry / over (PLACE OVER)' and 'place DOWN' share [-0.49846, 0.50668] m/s^2
- overlap: 'carry / over (PLACE OVER)' and 'opening (PLACE OPEN)' share [-0.49846, 0.38831] m/s^2
- overlap: 'carry / over (PLACE OVER)' and 'empty again (BACK, HOME)' share [-0.2998, 0.45465] m/s^2
- overlap: 'place DOWN' and 'opening (PLACE OPEN)' share [-0.75503, 0.38831] m/s^2
- overlap: 'place DOWN' and 'empty again (BACK, HOME)' share [-0.2998, 0.45465] m/s^2
- overlap: 'opening (PLACE OPEN)' and 'empty again (BACK, HOME)' share [-0.2998, 0.38831] m/s^2
```

Interpretation (plain): the holding verdict follows the squeeze, not the scripted step. It turns on about 136 ticks before the scripted CLOSE step ends, because the closure test is already met while the fingers are still moving. It turns off 151 ticks after the scripted OPEN step starts, because the squeeze only drops then. Between those edges the two agree. Right hand: never holds in this run, so its verdict is correctly off.

The still verdict is off for the whole second before place DOWN (ticks 12405 to 12554). In this run the hand is still moving at that point, so a re-weigh there would not start (this is an observation, not a rule).

## Step 8: closing on nothing, where would the fingers stop? (read-only, no simulation steps)

Method: the robot-only model `models/robot/combined_robot.xml` was loaded in a scratch script in `/tmp` (`pad_contact_scan.py`, then a refinement). The left finger joints were set to the same travel, `mujoco.mj_forward` was called once per position (this computes contacts without stepping the simulation), and the contacts between the two finger bodies were recorded. There is no beaker in this model.

- **Finger joint range (shown by the model, `combined_robot.xml` lines 227 and 234):** 0 m to 0.044 m for both finger joints.
- **Finger collision shapes (shown by the model):** each finger body has one mesh collision shape (`openarm_left_left_finger_collision` and `openarm_left_right_finger_collision`), half-size about 0.0157 × 0.0364 × 0.0619 m, offset about ±0.0115 m in y from the centre line.
- **Pad centre (shown by `src/gripper_geometry.py`, via `gripper_frame`):** 0.00017, 0.0, 0.0025 m in the tool frame; closing axis (0, 1, 0).
- **Contact result (shown by the scan):** no finger-to-finger contact from 0.044 m down to 0.0050 m. At 0.0025 m (0.5 mm grid) the two meshes overlap. Refined at 0.05 mm steps, the first contact is at **0.00275 m** (2.75 mm), with a penetration of 0.02 mm (a touch).
- **So closing on nothing would stop at about 2.75 mm of finger travel, not at 0 m.** The joint limit at 0 m is not reached.

Comparison (facts only):
- The closure range in the new config runs from 0.002 m to 0.042 m. The stop point (about 0.00275 m) is **inside** that range, so the closure test alone is met by closing on nothing.
- Calculated, not measured: the servo's stiffness at the finger is kp = 2 × 300 = 600 N/m (`src/robot_specs.py` line 183 gives 300 per finger; `src/arm_actuators.py` line 63 doubles it). At 0.00275 m with the command at 0, the spring force is about 600 × 0.00275 ≈ 1.65 N. That is below the 2 N threshold, so the squeeze test would be expected to fail. **This has not been tested in a run.** A closing-on-nothing run would be needed to confirm it (see NOT BUILT).

## Step 9: secret search (masked values only)

Run on the untracked reports under `docs/reports/` before this report was written. The only one was `held_baseline_20261103.md` (20,269 bytes, under 1 MB; committed as `43a931c`). Results: key shapes (`AIza`, `sk-`, `ghp_`, `xox`, `AKIA`, `Bearer `) 0 each; e-mail-shaped text 0; `-----BEGIN` 1 match, a false positive (the report's own description of the shape); keyword-with-value matches are descriptions only; Gemini key 0 matches (counted with `grep -c -F`, value never printed). Nothing was left out. This report (`signal_trace_stage1b_20261103.md`) is written after that search, and is not committed.

## Step 10: verification

- `git status --short` at the end: this report (uncommitted) and the untracked files that were already there (`deps/AgileX_Tracer`, `.claude/`, `compare_nosag.py`, `extract_height_data.py`, `measstart`). No tracked file has changed.
- `git diff --stat`: empty.
- Files changed by this stage's commits (`70b1cb7`, `85b77ce`, `43a931c`): `src/task_config.py`, `src/signal_trace.py`, `docs/mpvi_decisions.md`, `docs/progress.md`, `docs/reports/held_baseline_20261103.md`. The baseline folder, the index and the existing baseline files were not touched in this stage.
- Nothing pushed: `git ls-remote --heads origin main` = `58e756f`.

## Run lengths and memory (summary)

R1: 4 min 09.06 s, peak 3.83 GB. R2: 3 min 06.77 s, peak 4.33 GB. R3: 9.42 s, peak 2.22 GB. None was killed. Memory pressure: no run was reported as slowed by memory; the machine's load (6.20 at the start) came from a system update, and its effect on run time was not measured.

## Commits

| Commit | Content |
|---|---|
| `70b1cb7` | Stage 1b code and config: `HOLD_STILL_CFG` (`src/task_config.py`) and log-only verdicts and acceleration (`src/signal_trace.py`) |
| `85b77ce` | Decision M248, notes under M246 items 1 to 3, progress entry (`docs/mpvi_decisions.md`, `docs/progress.md`) |
| `43a931c` | Held baseline report (`docs/reports/held_baseline_20261103.md`), committed in step 9 |

Not pushed.

## Exact commands

    git status --short ; git log --oneline -3 ; git branch --show-current ; uptime ; free -m ; systemctl is-active systemd-oomd
    python -m py_compile src/signal_trace.py src/task_config.py
    git diff -- src/signal_trace.py src/task_config.py > data/signal_trace_1b_20261103/stage1b.diff
    python -I /tmp/signal_trace_1b_20261103_unit.py                       # synthetic check of the verdict code; no simulation
    tmux new-session -d -s s1b_r1 "... unset OPENARM_SIGNAL_TRACE && flock data/locks/mujoco.lock bash experiments/run_capped.sh data/logs/s1b_R1_off_<stamp>.log python data/bounce_check_20261103/run_mass.py data/signal_trace_1b_20261103/r1_off/run --liquid 0.100"
    cmp data/signal_trace_1b_20261103/r1_off/run/liftoff_ticks.csv experiments/baselines/held_200g_20261103/liftoff_ticks.csv   (and episode_log.csv)
    tmux new-session -d -s s1b_r2 "... OPENARM_SIGNAL_TRACE=1 flock ... run_mass.py data/signal_trace_1b_20261103/r2_on/run --liquid 0.100"
    tmux new-session -d -s s1b_r3 "... OPENARM_SIGNAL_TRACE=1 flock ... bash experiments/run_capped.sh <log> env CC_HEADLESS=1 bash experiments/cc_bench_scenarios.sh manual data/signal_trace_1b_20261103/r3_on/manual.csv"
    python experiments/cc_bench_compare.py manual data/signal_trace_1b_20261103/r3_on/manual.csv
    git add src/task_config.py src/signal_trace.py && git commit ...           # 70b1cb7
    git add docs/mpvi_decisions.md docs/progress.md && git commit ...          # 85b77ce
    git add docs/reports/held_baseline_20261103.md && git commit ...           # 43a931c
    python -I /tmp/signal_trace_1b_20261103/verdicts.py > /tmp/signal_trace_1b_20261103/verdicts.md   # read-only comparison
    python -I /tmp/signal_trace_1b_20261103/pad_contact_scan.py               # read-only; mj_forward only, no steps
    git grep -n "PROVISIONAL" HEAD ; git grep -n -i -o -E ".{0,40}provisional.{0,30}" HEAD -- src experiments docs models

## NOT BUILT, needs decision

1. **Acceleration limit for stillness.** Not decided, and no acceleration limit was chosen. The acceleration ranges (above) exist now; David decides.
2. **Closure with the squeeze still high.** When the closure leaves the range while the squeeze stays high, the code leaves the holding verdict unchanged. The decision does not say what should happen. David decides; the code behaviour is a placeholder choice.
3. **Every final threshold.** All six values are PROVISIONAL and SIM-ONLY. They must be recalibrated on hardware, and David decides the final numbers.
4. **The 2 mm margin at the closed end** of the grasp range (0.002 m) is a guess. The pads touch at about 0.00275 m in the robot-only model, which is inside the range, so a closing-on-nothing run would show whether the squeeze test separates the cases.
5. **Closing-on-nothing run.** Not made. The check above is from the model, and the expected squeeze (about 1.65 N, calculated) is untested. A run that closes on nothing needs a launcher option or layout change that does not exist yet, as stage 1 found.
6. **Short "still" stretches** (5, 9 and 14 ticks in the left hand) are not explained. Their cause is not checked.
7. **Whether the still verdict should drive a re-weigh before place DOWN.** In this run it is off for the whole second before place DOWN, so the rule would not fire there. That is for David to decide, not built.
8. **The baseline README's launcher path** (`data/held_run/run_mass_wt.py`) no longer exists, since the scratch worktree was removed. The same script with the repo's root is used here. Whether to fix the README is David's call.
9. **Older placeholders** (the settle thresholds, the VLM checker's camera and model choices) are listed in step 6 but are not changed here.
10. **Untracked files** from earlier tasks and the `deps/AgileX_Tracer` display question are unchanged.
