# Freeze experiment, part 2: runs A and B. Both stop at the pick's UP step, before the place

Written for: David (design review). Branch held-weight-ramp-sag-off.
Build: 13cebeb (sag gain 3.0, freeze, skill-start key off). Launcher 91aaa16, metrics script 5ed1b28. Log files: data/logs/freeze_runA_20261102_182353.log and data/logs/freeze_runB_20261102_182353.log. Traces: data/bounce_freeze_20261102/run_A/ and run_B/ (bounce_ticks.csv). Metric output: data/bounce_freeze_20261102/metrics_part2.md.

**Plain summary.** Neither run reached the place. In both, the beaker lift (UP) stalled 17 to 20 mm below its target, the correction was frozen the whole time, and the plan stopped. Run A and Run B are byte-identical, so Run B did not test the skill-start key. The freeze, as built, blocks the correction from learning the beaker's weight during the lift, and the weighing that would end the freeze only happens after the lift. This is a design gap for David (M229, PENDING). Nothing was retried.

Acronyms: IN, CLOSE, UP, CHECK = the pick's steps (move to beaker, close gripper, lift, weigh); OVER, DOWN = the place's steps; C = the sag correction (the integral trim); e = measured pad z minus reference pad z; T0 = first tick with a held mass.

## Step 0 (state at the start)
- Processes: no MuJoCo or teleop process (VERIFIED, pgrep). No tmux server (VERIFIED).
- Other writers: two Claude processes were running (PIDs 253425, running 2 h 14 min, and 278373, running 24 min). I cannot tell from the process list whether either is this session. No repo file had changed in the 10 minutes before the start (VERIFIED, find). During the runs only the one run process was active (VERIFIED, pgrep).
- git log -5 at the start: e52ca5b (Part 1 report), 13cebeb (build), bb559a0 (docs), d2e0ad7 (WIP, another session; not touched), 84550e6 (WIP).
- git status: about 70 source files modified, all logging or marker removals, as in Part 1. This task added one uncommitted logging hunk set in src/mpvi_modules.py (the TRACE dict and four trace lines). It is saved as data/bounce_freeze_20261102/logging_part2_uncommitted_mpvi.diff (51 lines). The trace only reads state: it does not change what the simulation does (VERIFIED from the code; no separate baseline check was run).

## 1. Pick and place steps, first attempt (run A; run B is identical)
| step | attempts | passed first attempt | pad error at end | time taken | correction frozen at end |
|---|---|---|---|---|---|
| PICK: IN | 1 | yes | 1.29 mm | 3.54 s | no |
| PICK: CLOSE | 1 | yes (stall) | not a move | 1.00 s | yes |
| PICK: UP | 1 | **NO: timeout** | **20.18 mm** (last traced tick, log says 20.2 mm and 3.2 deg) | 14.35 s to the timeout | **yes** |
| PICK: CHECK | never started | - | - | - | - |
| PLACE: OVER, DOWN | never started | - | - | - | - |

VERIFIED (log lines and trace). The UP timeout is the "not reached after 14.3 s" message of the plan's own check. The plan then stopped without retry: the log says "no retry: the object (beaker) is already in the left hand". What this means: the lift never reached its waypoint, so the weighing (CHECK) never ran. Why it matters: the place cannot be tested until the lift is fixed.

IN now ends at 1.29 mm, not 3.3 mm as in the sag-off runs, because the gain is back at 3.0 (M225). This agrees with the decision, but it is one run.

## 2. What happens during UP (VERIFIED from the trace, run A)
- The lift starts at tick 11150 (75.73 s). The pad starts 107.6 mm from the waypoint (the waypoint is straight up, 0.107 m).
- Pad height follows the reference until about tick 11510 (78.13 s), where e is -17.6 mm.
- From about 78.3 s the pad settles at about 1.101 m, against a reference of 1.1205 m. e stays between -19.0 and -20.2 mm until the timeout at tick 13302 (90.08 s).
- The held mass is 0 on every tick (VERIFIED: mass_kg maximum 0.0). The correction is frozen from tick 11000 (CLOSE) and is constant on every tick after it (C_p z = -0.001308 m throughout UP).

INFERRED (not tested by a counter run): the beaker's weight (about 0.2 kg) is not compensated during the lift, the frozen correction cannot learn it, so the arm droops about 19 mm and the lift times out. The old sag-on run lifted without this problem because its correction kept learning during the lift. What it means in plain words: the freeze turns off the one mechanism that would have carried the lift.

## 3. Metric 1 (baseline = tick before the held mass first becomes nonzero)
| run | T0 | baseline | rise | drop | swing | place time | final beaker x, y, z (m) |
|---|---|---|---|---|---|---|---|
| sag on (existing) | 11599 | 0.00 mm | +17.70 mm | -18.45 mm | 36.14 mm | 4.70 s | 0.6504, 0.6601, 1.0279 |
| sag off in place only (existing) | 11599 | 0.00 mm | +2.99 mm | -4.05 mm | 7.04 mm | 4.70 s | 0.6504, 0.6601, 1.0279 |
| Run A | NOT AVAILABLE (no held mass) | - | - | - | - | NOT AVAILABLE | 0.8458, 0.6626, 1.0379 (beaker in the hand at the timeout, not on the hotplate) |
| Run B | NOT AVAILABLE (no held mass) | - | - | - | - | NOT AVAILABLE | same as Run A |

VERIFIED (the two existing rows reproduce the earlier table). Run A and B have no T0: the weighing never ran. The final beaker row is where it was when the plan stopped.

## 4. Metric 2 (error e against the planned path)
NOT AVAILABLE for all four runs. The two existing runs have no reference column (VERIFIED from the extract headers: only padz, phase and the held-mass columns). Runs A and B have no T0 and no end of DOWN, so the window is undefined.

## 5. Freeze windows and correction values (VERIFIED, run A and run B alike)
| window (ticks) | time (s) | C_p start and end (m) | C_r start and end (rad) | largest change inside |
|---|---|---|---|---|
| 11000 to 13302 (CLOSE to the timeout; never ends) | 74.73 to 90.08 | (-0.002081, -0.002660, -0.001308) both | (-0.009936, -0.002742, -0.013724) both | 0.00 m and 0.00 rad |

The largest change is exactly 0 because the freeze held the whole time, as designed (the frozen value is still added to the command). The correction at the end of the pick is the same value. The correction at the start of the place is not available (no place).

The freeze ends only when a ramp-in to a weighed mass reaches its target (M226). The weighing is at CHECK, which never runs, so the freeze stays on until the run stops.

## 6. Resets of the correction (read only)
| reset | tick | time (s) | inside a freeze window? |
|---|---|---|---|
| run start (first traced tick, pick start) | 9687 | 65.98 | no |
| pick skill start (key on in run B; the zeroing had no visible effect) | 9687 | 65.98 | no |
| place skill start | never reached | - | - |
| gripper-open event | never reached | - | - |
| recovery | none | - | - |

VERIFIED. Run A and run B differ in no tick: the CSVs are byte-identical (cmp). So the key's zeroing at 9687 changed nothing (the correction was zero there, INFERRED from identical output). Run B did not test the key, because the only place the key acts, a place skill start, was never reached. This is the main gap of part 2.

## Limits and NOT BUILT, needs decision
1. **Lift gap (M229, PENDING).** The freeze starts at CLOSE and ends only after the weighing at CHECK, which comes after the lift. Options seen (not decided): (i) freeze from the start of CHECK instead, so the correction learns the beaker's weight during the lift (this goes against M226's purpose); (ii) weigh the beaker before the lift; (iii) weigh from the lift's own torque before UP. The choice belongs to David (it changes the freeze rule).
2. **Run B cannot test the skill-start key** until a run reaches a place. This needs item 1 decided first.
3. **Metric 1 and 2 for runs A and B, place duration, pad height at the end of DOWN:** NOT AVAILABLE, because the place never ran.
4. **No retry** with different settings, as instructed.
5. **Baselines** (data/cc_baseline, experiments/baselines) are not checked. The gain change alters existing output on purpose; a check belongs with the next build that runs the place.
6. **Push:** none. The feature is not complete.

Acronyms are spelled out where first used; claims are marked VERIFIED, INFERRED or NOT FOUND in the text.
