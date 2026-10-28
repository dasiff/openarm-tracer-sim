# Minimum-jerk phase-start ramp and skill-start ramp (M195 update)

Written 2026-10-28. Written for: David. Not pushed, video not re-rendered. The full suite is started after this report (results to be added at its end).

## What was built
1. Skill starts: `start()` keeps the last command before `arm.reset`; the first tick's velocity is measured from it, so the snap to the measured joints goes through the ramp (`PICK_PLACE_CFG["skill_start_ramp"]`, True).
2. Shape (your corrected message): the speed of each arm joint is capped by 0.5 rad/s * (10u^3 - 15u^4 + 6u^5), u = t / 0.5625 s (85 ticks), at in-skill phase starts and skill starts. 0.5625 s = 1.875 * 0.5 / 1.667, so the peak acceleration is the old 1.667 rad/s^2. `phase_start_max_accel_rad_s2` is only a comment now.
Choices of mine: t is taken at the end of each tick ((k+1) dt), so the first tick already has a tiny non-zero cap; the envelope caps speed only (it does not limit velocity change like the old ramp); at a skill start the snap is not removed, the solver's own motion from the last command is capped.

## Episodes (moved layout, bridge ON, duration fix ON)
- ramp-only baseline (linear 0.3 s, in-skill only): `ramp_20261028_025303`.
- plain: linear ramp + skill-start ramp: `skillramp_20261028_031343` (log `data/logs/demo_record_skillramp_20261028_031343.log`).
- smooth: minimum-jerk 0.5625 s + skill-start ramp: `smoothmj_20261028_031803`. Measurements in `data/arm_motion/<name>/` (compare.md vs the baseline), carry data in `data/arm_motion/carry_boundary/carry_{ramp,skillramp,smoothmj}.npz`.

## (a) Success
All three SUCCESS, 4 of 4 steps. Smooth: 13884 ticks, 92.6 s.

## (b) Commanded velocity at the carry boundary (largest one-tick change within +-0.1 s, any joint)
baseline (ramp-only) 0.532 rad/s; plain 0.034; smooth 0.032 (joint 7 only, the end of UP's slow creep of -0.032 rad/s going to 0; the other joints 0.007 or less). The 0.53 rad/s snap is gone in both.

## (c) PLACE OVER beaker tilt (world, peak) and hand dip
| | tilt | dip |
|---|---|---|
| baseline | 2.82 deg | 17.6 mm |
| plain | **5.55 deg** | **40.1 mm** |
| smooth | 2.92 deg | 18.8 mm |
The plain linear version made the carry worse (the 0.1 rad snap becomes a fast move at 0.5 rad/s then, and PLACE OVER peaked at 0.572 rad/s). The smooth version is within 0.1 deg and 1.2 mm of the baseline; it does not improve tilt or dip, it removes the one-tick step.

## (d) Peak jerk, 150 Hz actual, rad/s^3 (smooth / plain / baseline)
Boundary window into: PICK DOWN 17.0 / 25.5 / 25.5; IN 35.3 / 99.2 / 99.2; CLOSE 12068 / 12068 / 12241 (the gripper); UP 310 / 195 / 248; PLACE OVER (carry) 178 / 288 / 202; DOWN 75 / 91 / 71; OPEN 528 / (n.r.) / 483; BACK 54 / (n.r.) / 75; HOME 20 / (n.r.) / 27 (n.r.: the plain run's remaining rows not tabulated here, see its compare.md).
Mid-phase: PICK OVER 71.7 / 71.8 / 71.8; DOWN 8.2; IN 38.3 / 42.2 / 42.2; CLOSE 5.6 / 3.9 / 4.0; UP 99 / 103 / 92; PLACE OVER 96.9 / 156 / 70.0; **DOWN 2776 / 2178 / 1777**; OPEN 73.8 / 92 / 72; BACK 42 / 38.5 / 35.5; HOME 38 / 37.5 / 38.7.
Notes: PLACE DOWN mid-phase jerk is the contact (the beaker meeting the plate) and is higher than the baseline (2776 vs 1777; the video's episode had 3845), it varies with where the arm is when it arrives, so I do not read it as an effect of the ramp. PICK UP boundary is a bit higher (310 vs 248). The carry boundary is lower than the baseline (178 vs 202); the 750 Hz jerk there is 1181 vs 1283.

## (e) Place accuracy
Beaker base vs the hotplate centre (0.65, 0.66): smooth 0.65 mm (offset -0.23, +0.61); plain 0.74 mm; baseline 0.89 mm. Height 1.0279 m, tilt 0.02 deg in all.

## (f) Duration
Whole episode 92.6 s (baseline 92.5, plain 92.0). PLACE OVER 2.30 s vs 2.16 s (+0.14 s); the other phases within 0.06 s. No phase longer by more than 0.14 s.

## (g) Full test suite (bridge OFF, both ramps ON)
TAG 20261028_034644, run 03:46 to 05:47 (summary data/logs/suite_20261028_034644_summary.txt, ALL_DONE). 46 tests ran with face_test skipped: 45 exit 0, 1 fail. face_test was run alone afterwards (log data/logs/face_test_20261028_034644.log): 27/27 checks, exit 0. Total 46 of 47 tests pass. The one failure is mpvi_text_test, 6/11 checks, the expected failure carried over from earlier suites. safety_filter_model_check passes (max FK difference 4.3e-11).

## Open points for David
- The smooth ramp fixes the one-tick snap but does not improve tilt and dip in PLACE OVER; if the visible bump is the tilt itself (2.8 deg), it comes from the reference move (the hand turn), not the snap.
- Linear plain skill-start ramp is not recommended (tilt doubled). The flag `skill_start_ramp` can turn the skill-start part off separately.
