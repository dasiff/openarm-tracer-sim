# Elbow height: actual vs commanded, from the logged runs (read-only)

Date: 2026-11-05, 17:00. Read-only: no code, model, map or src/stance_passes.py change; existing logs read, analysis scripts in scratch only. Not pushed.
Written for: David (sets the margin in the elbow-height rule).

## Answer

- **Largest downward difference (actual elbow lower than commanded on a level base): −6.8 mm, left arm**, measured at 10 Hz during PLACE OVER with the beaker held (joint tracking only). The right arm's worst is **−2.8 mm** (base drive). The 1 Hz checkpoint breakdown gives a smaller worst (−2.8 mm left, −2.9 mm right), so the checkpoints undersample.
- **Base tilt in the logs: essentially zero.** The largest tilt is 0.006° pitch (ped020, drive). The 1° (7 mm) assumption is **not tested** by these logs: the simulated base does not tilt at any measurable level, so the logs cannot say whether 1° is too small, about right, or too large.
- **Base sag in the logs: essentially zero.** Largest −0.28 mm (base lower than the 0.1527 m settled height). The 5 mm assumption is again **not tested** for the same reason.
- **Margin implied by the measured lag (the one number the logs can speak to):** 1× = 6.8 mm, 2× = 13.7 mm, 3× = 20.5 mm (above 1.0908 m: 1.0976, 1.1045, 1.1113 m). Options only; David decides (see below).
- **Zombie pose at pedestal 0.16 m:** left elbow 1.1152 m, **+24.4 mm** above 1.0908 m; right 1.1229 m, **+32.1 mm**. A 20 mm margin leaves 4.4 mm (left) and 12.1 mm (right).

## Logs used

Current configuration only (post-neck, eagle camera on the torso, arm model unchanged): 94 runs with both an episode log (10 Hz sensed and commanded joints, pedestal, base x/y/yaw) and a recording (physics checkpoints every 150 ticks, i.e. about 1 Hz, with the full base pose and arm joints). Runs span 2026-10-22 to 2026-11-05. The list is in `/tmp/claude-1000/-home-vboxuser-projects-openarm-tracer-sim/137e4ff7-1e6a-485b-baff-f71dff4caeb7/scratchpad/runs.txt`. Main ones: `data/diag/nosag_20261101/run1`, `data/preload_20261103/run`, `data/held_beaker/bench_region`, `data/demo_record/*`, `data/bounce_*`, `data/closed_loop/*`.

Logs that do not record what is needed: the per-tick diagnostic CSVs (`data/diag/*/instr_ticks.csv`, `logging_ticks.csv`) have sensed joints and some targets but no base pose; `data/diag/height_20261101/extract_on.csv` has tilt and base z for a single 240-tick window only. The recordings' `ctrl` array holds torques, not joint targets, so commanded joints come from the episode logs' `cmd_*` columns.

## Method

- **Tick to time:** sim_time = tick/150 + 1.4 s (checked against phase-label ticks).
- **Elbow height:** FK of the left/right elbow joint (joint 4) in the scene model, with the pedestal joint set as in each source.
- **Deviation** = FK(actual base, actual joints, actual pedestal) − FK(level base at 0.1527 m, commanded joints, commanded pedestal). The telescoping breakdown is exact:
  - joint tracking = FK(actual joints) − FK(commanded joints), on the actual base;
  - pedestal tracking = pedestal actual − commanded;
  - tilt = FK(actual tilt) − FK(level), at the actual base z;
  - base height (sag) = FK(actual base z) − FK(0.1527 m).
- **1 Hz (checkpoints):** all four terms, from the physics state in the recording (base quaternion and z, arm qpos).
- **10 Hz (episode rows):** only the joint-tracking and pedestal terms, because the base pose is not logged at 10 Hz. Base tilt and sag are taken as zero here, which the 1 Hz data supports (|tilt| ≤ 0.006°, sag ≤ 0.3 mm).
- **Commanded values** at checkpoints are linearly interpolated between the 10 Hz rows.

## Table 1: largest downward difference per arm, with breakdown (mm)

| Arm | Source | Total | Joint tracking | Pedestal tracking | Base tilt | Base sag | Where |
|---|---|---|---|---|---|---|---|
| Left | 10 Hz rows (joint + pedestal) | **−6.83** | −6.83 (pedestal incl. in joint-only term) | (incl.) | not measured (≈0 in 1 Hz data) | (≈0) | PLACE OVER, beaker held; `demo_record/restart_20261028_022906`, t = 80.7 s |
| Left | 1 Hz checkpoints (full breakdown) | −2.78 | −2.22 | −0.55 | −0.00 | −0.00 | PLACE OVER; `skillramp_20261028_031343`, tick 12000 |
| Right | 10 Hz rows | **−2.82** | −2.82 (incl. pedestal) | (incl.) | not measured | (≈0) | base drive; `demo_record/20261027_132008`, t = 19.3 s |
| Right | 1 Hz checkpoints (full breakdown) | −2.90 | −2.18 | −0.54 | +0.003 | −0.18 | base drive; `demo_record/ped020_20261028_125039`, tick 6150 |

The 10 Hz row figures are joint-plus-pedestal combined (the 10 Hz pass did not split them); the pedestal lag on its own is at most 0.58 mm in any row.

**Lag vs droop:** the joint term is one number. Tracking lag (velocity-driven) and static droop (gravity or held weight) cannot be separated from these logs. The worst left case is in the beaker-held phase, so droop from the 0.2 kg beaker may contribute; the right arm's worst is during a base drive with no beaker.

## Table 2: largest tilt and sag against the assumptions

| Quantity | Largest seen (logs) | Assumption | Verdict |
|---|---|---|---|
| Base tilt | 0.0058° pitch (ped020, drive, tick 7350); 0.0017° roll | 1° ≈ 7.0 mm at the shoulder lever (0.40 m) | Not tested. The simulated base does not tilt above 0.006°. |
| Base sag (base z below 0.1527 m) | −0.28 mm (min over 8577 checkpoints) | 5 mm | Not tested. The sim base is effectively rigid vertically. |

The sim's base shows effectively no tilt or sag, so "too small", "about right" and "too large" cannot be concluded from these logs. A real base must be measured to set the tilt and sag allowances.

## Worst moments (all logged runs)

| Moment | Left, 10 Hz (mm) | Right, 10 Hz (mm) | Note |
|---|---|---|---|
| Zombie rise (ARMS / PEDESTAL) | −0.93 | −0.97 | Slow move; joint term only |
| Fast moves (PLACE OVER/DOWN, beaker held) | **−6.83** | −0.53 | Left worst; the right arm is small here |
| Base acceleration (DRIVE/CREEP) | −2.59 | **−2.82** | Max base accel 0.15 m/s², 0.2 rad/s² (basevel units assumed SI); right arm worst |
| Carrying held beaker (PICK UP/WEIGH) | −2.41 | −0.54 | Held weight |
| Place (open, back, home) | −3.39 | −0.52 | No beaker weight after release |
| Sag trim off (`nosag` runs) vs others | −2.16 (n = 2714 rows) vs −6.83 (n = 82694 rows) | −2.78 vs −2.82 | Not a clean comparison: "others" mixes older runs; the trim status of those is not recorded |

The worst moments are short transients. At 1 Hz the largest left value was −2.8 mm; the 10 Hz pass found −6.8 mm, so the 10 Hz figure is the one to use for a margin.

## Margin options (for David to decide; not chosen here)

The rule: minimum elbow z = 1.0908 m + margin. The 1.0908 m already includes the 12 mm tilt-and-sag allowance, which the logs do not test. The measured joint lag is not in that sum, so the margin has to cover it.

Largest downward difference: **6.8 mm** (10 Hz, left). Using the 1 Hz figure instead gives 2.8 mm.

| Multiple | Margin, from 6.8 mm | Minimum elbow z | Margin, from 2.8 mm | Minimum elbow z |
|---|---|---|---|---|
| 1× | 6.8 mm | 1.0976 m | 2.8 mm | 1.0936 m |
| 2× | 13.7 mm | 1.1045 m | 5.7 mm | 1.0965 m |
| 3× | 20.5 mm | 1.1113 m | 8.5 mm | 1.0992 m |

The 3× value from the 10 Hz figure (1.1113 m) is close to the 1.111 m (20 mm margin) option already discussed.

## Zombie pose at pedestal 0.16 m

| Arm | Elbow z at zombie (m) | Margin above 1.0908 m (mm) | Margin above 1.111 m (mm) |
|---|---|---|---|
| Left | 1.1152 | +24.4 | +4.4 |
| Right | 1.1229 | +32.1 | +12.1 |

These are the static zombie values from the previous report, before any lag. A 6.8 mm worst lag would take the left arm to 1.1084 m (17.6 mm above 1.0908 m).

## Caveats

- **Simulated base:** the sim base does not tilt or sag in these logs. Real tilt and sag may be larger, and the allowances cannot be checked against sim data.
- **Placeholder gains:** `src/robot_specs.py` marks the arm actuator gains as PLACEHOLDERS, so the sim's joint lag may not match the real arm.
- **Sampling:** 1 Hz checkpoints undersample transients (the 10 Hz figure is 2.4 times the 1 Hz figure). The 10 Hz rows are logged state, not a continuous physics signal.
- **Pedestal heights:** the logged runs use several pedestal heights (for example 0.15 m in `ped015`, 0.20 m in `ped020`, and the spawn height in others), not only 0.16 m. The lag is measured across them.
- **Interpolated commands:** commanded joints are interpolated between 0.1 s rows; this smooths fast moves slightly.
- **Base velocity units:** `basevel` is assumed to be SI (m/s, rad/s).

## What was not checked

- Real-robot tilt and sag (no real-robot logs in the repo).
- Separation of tracking lag from gravity droop (needs a load-off run for the same motion).
- Pedestal exactly at 0.16 m (no logged run at that height).
- Continuous (per tick) joint states: the recording stores full state only every 150 ticks; the 10 Hz episode rows are the finest joint-level source.
- Sag-trim on/off as a controlled comparison: the only off runs are the `nosag` family (2026-11-01), and the on-runs' trim status is not recorded in the logs.

## Decisions needed from David

1. **Margin:** choose 1×, 2×, or 3× of the measured worst lag (6.8 mm → margins 6.8, 13.7, or 20.5 mm), or set the margin by another rule. Also say which lag figure to use: the 10 Hz worst (6.8 mm) or the 1 Hz checkpoint worst (2.8 mm).
2. **Tilt and sag allowances:** the sim cannot test the 1° and 5 mm values. Measure the real base (tilt under arm load and during moves), or keep the 12 mm allowance as an assumption.
3. **Gains:** the lag depends on placeholder gains. Accept the sim lag as a proxy, or re-measure on hardware.
4. **Sampling:** accept the 10 Hz episode rows as the lag measurement, or request a per-tick joint log for the worst windows (this needs new instrumentation, so I have not done it).
