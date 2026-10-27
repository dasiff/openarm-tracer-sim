# Part B: phase durations from the IK arc length (built, measured; stopped at step 5)

Written 2026-10-27 17:00. Not pushed. Written for: David, who decides on an acceleration limit.

## What was built (row M194)
`MinkArm.joint_arc` (`src/mink_arm.py`): IK on a scratch configuration at 20 equal steps along the straight-line reference (plus the start), same tasks and limits as the real step but without the speed limit (it only slows the dry run); per joint the sum of absolute changes between consecutive solutions; largest over the 7 joints = arc. `src/mpvi_modules.py`: T = max(1.875 * arc / 0.5, old distance/speed/turn time, min_move_s). Settings `joint_limited_duration`, `arc_points`, `min_jerk_peak_factor` in PICK_PLACE_CFG. Nothing else changed (min-jerk, mink, sag, settling, paths, waypoints, phases; no filter, no acceleration limit).
Run: moved layout, bridge ON, fix ON. Log `data/logs/demo_record_smooth_20261027_165912.log`, measurement `data/arm_motion/smooth_20261027_165912/` (replay bit-identical, 13833 ticks), compare against the original baseline (no bridge, no fix) `baseline_20261027_132008`.

## (a) Succeeds
Yes: SUCCESS, 4 of 4 steps. Episode 92.2 s sim (13833 ticks) against 92.1 s with the bridge only.

## (b) and (f) Planned vs actual time, and arc vs endpoint
| move | arc rad | endpoint rad | planned before s | planned after s | actual before s | actual after s |
|---|---|---|---|---|---|---|
| PICK OVER | 0.43 | 0.43 | 1.0 | **1.6** | 2.49 | 2.58 |
| PICK DOWN | 0.66 | 0.64 | 3.7 | 3.7 | 3.91 | 3.91 |
| PICK IN | 0.60 | 0.59 | 3.0 | 3.0 | 3.21 | 3.46 (bridge) |
| PICK UP | 0.48 | 0.48 | 2.7 | 2.7 | 2.90 | 2.91 |
| PLACE OVER | 0.29 | 0.29 | 1.7 | 1.7 | 2.14 | 2.16 |
| PLACE DOWN | 0.18 | 0.12 | 1.0 | 1.0 | 1.21 | 1.89 |
| PLACE BACK | 0.29 | 0.29 | 1.4 | 1.4 | 1.83 | 1.84 |
| PLACE HOME (move) | 0.69 | 0.69 | 1.4 | **2.6** | 4.28 | 4.37 |

Arc and endpoint travel are almost the same in every phase (largest difference PLACE DOWN, 0.18 against 0.12). So the straight-line path does NOT make a joint travel much farther than its endpoint distance; the 1.875 factor is what made the change. (My earlier guess that the path arc was the main reason was not right; the report of 16:20 named both.) "Actual before" is the original no-bridge run; the bridge-only run has the same values except ARMS and PLACE DOWN.
Actual time is still longer than planned for the two moves that changed: PICK OVER 2.58 s against 1.6 s planned, PLACE HOME 4.37 s against 2.6 s planned (the actual includes the settle wait; 2.5 s is also what OVER needed before).

## (c) Peak jerk, 150 Hz, rad/s^3 (this run / original baseline; the baseline's boundary values are from compare.md)
Mid-phase: PICK OVER 126 / 484; DOWN 8 / 8; IN 37 / 39; UP 102 / 49; PLACE OVER 79 / 70; DOWN 1485 / 1628 (the bridge-only run had 6755); OPEN 74 / 59; BACK 38 / 39; HOME 91 / 333.
Boundaries (window +-0.5 s): OVER to DOWN 8.8 / 37; DOWN to IN 71 / 94; IN to CLOSE 11954 / 9241 (the gripper closing); CLOSE to UP 296 / 162; PLACE OVER to DOWN 84 / 162; DOWN to OPEN 483 / 729; OPEN to BACK 100 / 141; BACK to HOME 34 / 88. The windows at the start of PICK OVER (1125) and of PLACE OVER (224) are mixed with the base's creep-in and the previous phase.
The pulses: PICK OVER now has ONE actual speed peak (was 3), PLACE HOME one (was 3).

## (d) Peak actual joint speed per phase (rad/s, this / original)
PICK OVER 0.532 / 0.665; DOWN 0.344 / 0.341; IN 0.399 / 0.396; CLOSE 0.308 / 0.222; UP 0.354 / 0.351; PLACE OVER 0.347 / 0.345; DOWN 0.473 / 0.419; OPEN 0.090 / 0.138; BACK 0.431 / 0.423; HOME 0.509 / 0.585. Still above 0.5 in PICK OVER (0.532) and PLACE HOME (0.509), by 6 % and 2 %.

## (e) Saturation of the 0.5 rad/s command limit, share of the phase's ticks (this / original)
PICK OVER 27 % (joints 2, 3, 5, 6) / 52 %; PLACE HOME 17 % (joints 1, 4) / 39 %; all other phases 0 to 3 % (PLACE OVER 3 %, BACK 1 %). So two phases are still above 10 %: PICK OVER and PLACE HOME.

## Step 5: boundary spikes
Above 100 rad/s^3 peak jerk (actual, 150 Hz): IN to CLOSE 11954 (fingers hit the beaker, not an arm move), CLOSE to UP 296, DOWN to OPEN 483, OPEN to BACK 100. The COMMANDED velocity still steps by 1.3 to 3.3 mrad in the first tick at CLOSE to UP, PLACE OVER to DOWN, and the commanded jerk at the boundaries is 4000 to 11 000. Per your step 5 I stop here and add no acceleration limit. Not done: step 7 (full suite, bridge ON + fix) and Part C (video re-render); both wait for your decision, because an acceleration limit would change the motion again.

## Questions
1. Add the joint-space acceleration limit, or accept the remaining boundary values (most are small in the actual motion, 9 to 100, except the gripper closing)?
2. Why PICK OVER still reaches 0.53 rad/s and 27 % saturation at 1.6 s planned is not understood (the sag correction and the reference restarting from the measured pose are the candidates); a longer duration (a factor above 1.875) is a possible lever, not tried.
