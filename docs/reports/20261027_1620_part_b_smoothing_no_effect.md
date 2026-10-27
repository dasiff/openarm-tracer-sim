# Part B (joint-limited phase duration): built as specified, it changes nothing in this episode

Written 2026-10-27 16:20. Not pushed. Written for: David, who decides the next step.

## Result
- Built exactly as specified: `MinkArm.joint_travel` (`src/mink_arm.py`) solves the IK from the command now to the move's end pose, with the same tasks and limits as the real step but without the speed limit, and returns the farthest joint's travel. `src/mpvi_modules.py` sets T = max(travel / 0.5 rad/s, old distance/speed time, min_move_s). Config flag `PICK_PLACE_CFG["joint_limited_duration"]` (True). Min-jerk, mink, sag, settling, paths, waypoints unchanged; no filter, no acceleration limit.
- Episode (moved layout, bridge ON, fix ON), log `data/logs/demo_record_smooth_20261027_154742.log`, measurement `data/arm_motion/smooth_20261027_154742/`: (a) SUCCESS, 4 of 4 steps, replay bit-identical, 13818 ticks.
- **(b) Planned times are exactly the old ones in every phase**, because travel / 0.5 is always below the old time:

| move | farthest joint (rad) | travel / 0.5 (s) | planned before = after (s) |
|---|---|---|---|
| PICK OVER | 0.43 | 0.86 | 1.0 |
| PICK DOWN | 0.64 | 1.28 | 3.7 |
| PICK IN | 0.59 | 1.18 | 3.0 |
| PICK UP | 0.48 | 0.96 | 2.7 |
| PLACE OVER | 0.29 | 0.58 | 1.7 |
| PLACE DOWN | 0.12 | 0.24 | 1.0 |
| PLACE BACK | 0.29 | 0.58 | 1.4 |
| PLACE HOME (move) | 0.69 | 1.38 | 1.4 |

  So the motion is identical to the bridge-only run of this morning (same jerk, same speeds, same pulses: OVER 2.53 s actual, three pulses, 0.686 rad/s peak, 52 % of ticks at the limit). (c), (d), (e) therefore equal the Part A table: peak jerk mid-phase OVER 478, PICK UP 88, PLACE DOWN 6755, PLACE HOME 420; peak joint speed 0.686 (OVER), 0.587 (HOME); saturation above 10 % of ticks in PICK OVER (52 %, joints 1, 3, 5, 6, 7) and PLACE HOME (38 %, joints 1, 2, 4, 5, 6, 7).
- Boundary spikes in the commanded velocity are still far above 100 rad/s^3 (commanded peak jerk 2000 to 12 000 at the PICK and PLACE boundaries; actual peak jerk 16.8 to 259 at the boundaries except IN to CLOSE 11 958, the gripper closing). Per your step 4 I stop here.

## Why it does nothing
The travel is the endpoint-to-endpoint joint change, divided by the average speed the limit allows. A minimum-jerk profile peaks at 1.875 times its average speed, so a joint that needs 0.43 rad in 1.0 s peaks at about 0.8 rad/s, above the 0.5 rad/s limit. The check should compare travel * 1.875 / 0.5. (Straight Cartesian lines may also make a joint travel more in the middle than at the end, which a start/end dry run cannot see.) I have not tried either.

## For you
1. Allow T = max(1.875 * travel / 0.5, ...) (the min-jerk peak factor)? With it OVER would be about 1.6 s planned against 2.5 s needed now; whether it removes the pulses is untested. I would test it with one episode run.
2. Or sample the dry run along the path (e.g. 10 intermediate poses) to get the real joint path length, with or without the factor.
3. Or the joint-space acceleration limit (your step 4 decision).

## Not done
M194 row (held until you choose: the row would record a decision that does nothing as it stands); the full suite with bridge ON (nothing changed in the motion, so it would show the Part A result; not run). The flag is committed True, with the added "farthest joint" text in the planned-time log line.
