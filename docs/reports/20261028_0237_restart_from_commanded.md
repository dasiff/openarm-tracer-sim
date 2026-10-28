# Restart each skill and move from the commanded pose: measured, and it does not fix the bump (it makes it worse)

Written 2026-10-28. Not pushed, video not re-rendered. Written for: David. Setting `PICK_PLACE_CFG["restart_from_commanded"]` (code in `src/mpvi_modules.py`); the episodes below ran with it True, the committed default is now False (see the end).

## What I changed
The "start from the measured pose" happens in two places, and I changed both under one setting:
1. **Skill start** (`start()` of the manipulation module): it called `arm.reset(data)`, which sets the command to the MEASURED joints. With the setting on it takes the ctrl that was applied on the previous tick (`data.ctrl` of the arm's 7 actuators). This is the one that matters at PICK UP to PLACE OVER: that boundary is a skill boundary (pick ends, place starts), so `arm.reset` ran there. Without it, changing only the move start would not have touched that boundary.
2. **Move start** (first tick of each waypoint move): the reference start pose was the measured pad pose; it is now the pad pose at the last commanded joints (`arm.q_cmd`) minus the sag correction C (the correction is added to every target, so the first target equals the commanded pose, not the commanded pose plus C twice). The duration dry run (`joint_arc`) already started from `q_cmd`, so its inputs shift a little: planned times changed (below).
Not changed: sag correction law, settling check, minimum-jerk law, waypoints, bridge, duration formula, phase structure, `recover()` (still resets to the measured joints). My own bug on the first try (a shadowed variable, KeyError at the pick start, run aborted before any motion) was fixed and the episode rerun; the numbers below are from the rerun.
Runs: episode `data/demo_record/restart_20261028_022906` (log `data/logs/demo_record_restart_20261028_022906.log`; bridge ON, duration fix ON, restart ON); measurement `data/arm_motion/restart_20261028_022906/` (replay bit-identical); beaker and hand series `data/arm_motion/carry_boundary/carry_restart.npz`; baseline = `smooth2_20261027_171036` (the episode of the video; baseline carry series `carry_base_full.npz`). New script `experiments/boundary_cmd_steps.py`.

## (a) Does it still succeed
Yes: SUCCESS, 4 of 4 steps, beaker on the hotplate, 13944 ticks, 93.0 s sim (baseline 13869, 92.5 s).

## (b) Commanded velocity step at every boundary (largest one-tick change within +-0.1 s, rad/s; joint)
| into | baseline | restart |
|---|---|---|
| PICK start / OVER | 0.103 (j7) | **0.500 (j4)** |
| PICK DOWN | 0.161 (j7) | 0.128 (j5) |
| PICK IN | 0.336 (j7) | 0.234 (j4) |
| PICK CLOSE | 0.008 | 0.008 |
| PICK UP | 0.167 (j7) | **0.333 (j4)** (0.500 within 0.3 s) |
| **PLACE start / OVER (the carry boundary)** | **0.553 (j4)** | **0.507 (j1)** |
| PLACE DOWN | 0.278 (j6) | 0.330 (j4) |
| PLACE OPEN | 0.010 | 0.008 |
| PLACE BACK | 0.500 (j7) | 0.321 (j4) |
| PLACE HOME | 0.243 (j7) | 0.259 (j4) |
At the carry boundary the commanded velocity still jumps, by the same size (0.51 against 0.55 rad/s), and the pattern is worse: the first command is at the 0.5 rad/s limit on SIX joints at once (j1, 2, 3, 4, 5, 7 at -0.5, -0.5, -0.5, +0.5, +0.5, -0.5) where the baseline had two (j1 and j7); the commanded speed is at the limit for 44 of 392 ticks of PLACE OVER (11 %, joint 1 for all 44) against 8 of 323 (2.5 %). It improved at PLACE BACK, PICK IN and PICK DOWN and got worse at PICK start, PICK UP and PLACE DOWN.

## (c) Peak actual jerk, 150 Hz, rad/s^3 (restart / baseline)
Mid-phase: PICK OVER 71 / 72; DOWN 8 / 8; IN 39 / 40; CLOSE 6.9 / 3.1; UP 99 / 85; PLACE OVER 120 / 87; DOWN 1795 / 3845; OPEN 83 / 73; BACK 41 / 34; HOME 38 / 40.
Boundary window (+-0.5 s), into: PICK DOWN 57 / 61; IN 56 / 76; CLOSE 12104 / 12242 (the gripper); **UP 1160 / 141**; **PLACE OVER (the carry boundary) 261 / 228**; PLACE DOWN 103 / 64; OPEN 458 / 493; BACK 89 / 100; HOME 46 / 35. PLACE OVER's first 1.5 s: 261 / 228 (all in its first 0.5 s); commanded jerk 11411 / 12430.

## (d) Peak actual joint speed per phase, rad/s (restart / baseline)
PICK OVER 0.571 / 0.520; DOWN 0.344 / 0.347; IN 0.403 / 0.398; CLOSE 0.316 / 0.321; UP 0.350 / 0.354; **PLACE OVER 0.814 / 0.348**; DOWN 0.487 / 0.550; OPEN 0.103 / 0.089; BACK 0.422 / 0.432; HOME 0.485 / 0.487.
Phase durations: PICK OVER 2.85 / 2.75 s, **PLACE OVER 2.61 / 2.15 s**. Planned times moved because the dry run now starts from the commanded pose: PICK OVER 1.7 -> 1.8 s (arc 0.43 -> 0.46 rad), PLACE OVER 1.7 -> 1.9 s (arc 0.29 -> 0.32), the rest unchanged.

## (e) Beaker tilt in the world during PLACE OVER
**Peak 21.5 degrees** at t 81.03 (baseline 2.8 degrees at 80.77). Pad centre height: it RISES 140 mm (1.1203 to 1.2572 m at 81.05) and comes back (the baseline dipped 17 mm). The pad's peak speed is 0.56 m/s (baseline 0.24) and its peak turn rate 1.44 rad/s (baseline 0.55). Tilt in the hand rises from 0.02 to 0.62 degrees (the grip gives a little); slide in the grip is the same, 2 mm; the beaker stays held.

## (f) Place accuracy
Final beaker base position (end of the run, at rest) against the hotplate centre (0.65, 0.66): baseline (0.6499, 0.6606): offset -0.1 mm, +0.6 mm (0.6 mm); restart (0.6496, 0.6608): -0.4 mm, +0.8 mm (0.9 mm). Height 1.0279 m and tilt 0.02 degrees in both; both pass the success check. So the place is as accurate as before (0.3 mm difference, not meaningful) despite the swing. Measured at the last replay sample, not at the release.

## Reading
The change did not remove the step at the carry boundary and made the first 0.5 s of PLACE OVER much worse (a 14 cm upward swing of the hand). My explanation, NOT tested: with the reference now starting at the commanded pose, the pad pose of the command is a few centimetres away from where the hand really is (the servos hold the arm 0.07 to 0.11 rad below the command on joints 1, 4 and 7 under the beaker's weight). The sag correction integrates (reference minus measured pad pose); it used to start at zero because the reference started at the measured pose; now it starts with that few-centimetre error and pushes the command hard (0.5 rad/s on six joints) until it winds up. In the baseline, the reset to the measured joints removed the servo's holding offset instead (the 17 mm dip). Neither start is consistent with a sag correction that measures against the measured pose. Not tried, because you asked for no tuning: seeding the correction with the offset, starting the reference at the measured pose but keeping the command, or compensating gravity in the servo.

## Not done, and why
Step 4 (decisions row M195, design plan marker, full suite with the setting on) is NOT done. Your conditions (success and place within tolerance) are met, but the result contradicts the purpose of the change (the step stays, the PLACE OVER swing is 14 cm and 21 degrees) and the suite takes hours; I did not want to log it as a decision or run it before you see this. I set the committed default of `restart_from_commanded` to False so the pipeline output stays as it was; one word from you and I run the suite with it ON, or revert the code. Experiments use `PICK_PLACE_CFG` overrides to turn it on.

## Questions
1. Revert, keep the setting off, or run step 4 anyway?
2. Try one of the untested variants above (a different design for the sag correction's start)?
