# Measurement: the PICK: UP to PLACE: OVER boundary in demo_moved_smooth2

Written 2026-10-28. Measurement only: no motion, setting or source file under src/ changed. Not pushed. Written for: David.

## Data
Episode: `data/demo_record/smooth2_20261027_171036` (factor 2.0, bridge ON, 13869 ticks). The final video's run has the same settings and the same tick count (13869, `replay_ticks` in its json); I did not compare the two recordings' states, so "same episode" means same settings and tick count. Video time = sim time + 4.07 s (read off the panel at 8, 66 and 95 s), so your 84.6 to 85.9 s is sim 80.5 to 81.8 s.
Stored series: `data/arm_motion/smooth2_20261027_171036/` (left arm, 750 Hz; jerk definitions as in `experiments/left_arm_motion_report.py`). New: `data/arm_motion/carry_boundary/carry.npz` (beaker, hand tool point and pad pose, 750 Hz, 76.5 to 84.0 s; replay bit-identical, log `data/logs/carry_boundary_20261028_021020.log`). Scripts: `experiments/carry_boundary_analysis.py`, `carry_boundary_replay.py`, `carry_boundary_report.py` (measurement only).
Boundary times: UP ends at tick 11864 (t 80.495); one tick of "3/4 NAV: starting", one tick of "4/4 PLACE: starting"; PLACE: OVER runs ticks 11866 to 12189 (t 80.508 to 82.661, 2.15 s, planned 1.7 s).

## (a) Peak jerk, rad/s^3 (actual joint, 150 Hz; joint with the peak; commanded, 150 Hz)
| window | actual 150 Hz | commanded 150 Hz |
|---|---|---|
| UP, middle half | 85 (joint 7) | 202 |
| last 0.5 s of UP | 69 (7) | 126 |
| boundary +-0.5 s | **228 (7)** | **12430** |
| PLACE OVER first 0.5 s | **228 (7)** | 12430 |
| PLACE OVER 0.5 to 1.0 s | 69 (7) | 334 |
| PLACE OVER 1.0 to 1.5 s | 87 (7) | 367 |
| PLACE OVER middle half | 87 (7) | 367 |
So the actual jerk is 2.6 times the mid-phase value, all in the first 0.5 s of PLACE OVER; the first 1.5 s beyond that is like mid-phase. The commanded jerk at the start is 12430, 35 to 60 times its mid-phase value. At 750 Hz the actual jerk (1138 in the first 0.5 s) is the same size as mid-phase (1174 and 1239): that series is dominated by sampling noise and shows nothing.

## (b) Commanded against actual joint velocity
The commanded velocity is the finite difference of the commanded joint position at 150 Hz. With the bridge it moves only every 3rd tick, so it is constant over 3 ticks.
- Before the boundary (6 ticks): commanded velocity is constant and tiny: [+0.007 +0.002 +0.001 -0.005 +0.005 -0.006 -0.032] rad/s (joint 7 creeping at -0.032); actual speed about 0.001 rad/s.
- **First command of PLACE OVER (t 80.507): it steps to [+0.500 +0.118 +0.268 -0.431 +0.298 -0.298 +0.500] rad/s on all seven joints.** Largest one-tick changes: joint 1 0.49, 2 0.18, 3 0.27, 4 0.55, 5 0.36, 6 0.38, 7 0.53 rad/s. It holds for 3 ticks, then at the next update joint 1 drops to 0.26 and the others to 0.05 to 0.12; joint 7 stays at 0.50 for 8 ticks in all. By 0.1 s later the command is smooth (norm 0.11, rising with the minimum-jerk profile).
- For comparison, the largest one-tick change of the commanded velocity elsewhere is 0.008 rad/s in UP and 0.016 in PLACE OVER (0.3 s in from the ends).
- Actual velocity follows smoothly and small: joint 1 0.005 at the first command, 0.081 six ticks later; joint 7 reaches 0.19 to 0.20 rad/s in the first 0.2 s. The actual speed norm is 0.23 at 0.0 to 0.2 s, 0.13 at 0.2 to 0.3 s, then rises to 0.60 at 0.9 to 1.0 s: the dip at 0.2 to 0.3 s is the "uneven" part (a first small burst, then the main move).
The commanded step is the cause of the first burst; whether it comes from the solver or the bridge's rate limit is not separated (both limit at 0.5 rad/s).

## (c) The 0.5 rad/s limit in PLACE OVER
Commanded speed of at least 0.49 rad/s: **8 of 323 ticks (2.5 %)**, all at the start (k = +2 to +9): joint 7 for 8 ticks, joint 1 for 2 ticks. After 0.1 s no joint's commanded speed passes 0.40 (joint 2 peaks at 0.40 to 0.35). Actual joint speed in PLACE OVER never passes 0.348 rad/s.

## (d) What runs between UP and PLACE OVER
- Nothing waits. The orchestrator logs segment 2/4 done, 3/4 started and done (navigation to the hotplate: already at the stance, zero length) and 4/4 started, all at t 80.49. That is 1 tick of "NAV: starting" and 1 of "PLACE: starting"; PLACE OVER starts 13 ms after UP's last tick.
- The settle wait belongs to UP: UP was planned 2.7 s and took 2.91 s; the extra 0.2 s is the settle dwell (joints slower than 0.02 rad/s for 0.2 s) inside the UP phase.
- The arm is at zero speed when PLACE OVER starts: actual speed norm 0.0010 to 0.0018 rad/s over the 6 ticks around the boundary (settle limit 0.02). It is not at zero command error: the command is 0.09 to 0.11 rad away from the measured position on joints 1, 4 and 7 (the arm sags under the beaker; gravity compensation does not carry its weight), and that offset stays through the boundary.

## (e) The hand turn
PLACE OVER moves the pad centre 202 mm (0.850, 0.660, 1.120 to 0.648, 0.660, 1.121; level, as planned) and turns the hand **25.5 degrees (0.445 rad) about the vertical** (pad x axis yaw 179.9 to 154.4 degrees).
Planned time: 1.7 s. Its terms: distance/speed = 0.202/0.12 = 1.68 s (this sets it); turn = 0.445/0.5 = 0.89 s; min_move_s 1.0 s; joint arc 2.0 * 0.29/0.5 = 1.16 s. So the turn does not set the time; it takes the same 1.7 s as the translation. Mean turn rate over the actual 2.15 s: 0.21 rad/s; peak (0.1 s differences) 0.55 rad/s at t 81.21; peak pad speed 0.24 m/s against the 0.12 m/s nominal (the minimum-jerk peak is about 1.9 times the mean).

## (f) The beaker in the grip
- In the hand: it does not slide or tilt: slide at most 0.4 mm in the last 0.5 s of UP, 2.5 mm over PLACE OVER and the first 0.5 s of DOWN (a slow drift along the pad's y axis, peak 2.2 mm/s; 1.8 mm/s in UP); tilt in the hand at most 0.02 degrees.
- **In the world it swings:** the hand holding it dips and tilts. At 80.50 the beaker axis is 0.35 degrees off vertical; it goes to 2.72 degrees at 80.75 to 80.80 (0.25 s later), then back to 0.8 degrees by 81.15, tilting about 4.7 cm per metre toward -y. At the same time the pad centre's height drops **17 mm** (1.1203 to 1.1033 m at 80.80) and returns (1.1186 at 81.15), although OVER is planned level. So what the video shows is the whole hand sagging and tilting for about 0.6 s as the arm starts to move, with the beaker rigid in it, not a slipping beaker.

## Why the earlier boundary table left it out
It was not missing from the measurement. `compare.md` (and `metrics.md`) list it, under the labels of the one-tick steps in between: "-> NAV: starting", "-> PLACE: starting" and "-> PLACE: OVER", all three with the identical window value 227.5 (actual jerk) and a commanded jump of 3.3 mrad per tick (the largest in the arm boundaries). My report tables (docs/reports/20261027_1700_part_b_smoothing.md and 20261027_2310_part_a_b_report.md) listed the boundaries by the phase names I recognised as arm moves and skipped those three rows. The 1700 report even said the start of PLACE OVER was "mixed with the base's creep-in and the previous phase" (224 at the time); that was wrong for PLACE OVER (the base is stopped there; the window is only the end of UP and the start of PLACE OVER). The value (228) is higher than the arm boundaries I did discuss (CLOSE to UP 141, OPEN to BACK 100), and lower than DOWN to OPEN (493) and the gripper closing. The 2310 report dropped the boundary rows without saying so. Two further reasons it was easy to miss: the window is +-0.5 s, so it only covers the first 0.5 s of OVER; and the actual-jerk number (228) is modest, while the step is in the commanded velocity (0.5 rad/s, jerk 12430), which is where I did not look at this boundary.

## Not done
No cause was tested for the commanded velocity step (candidates: the solver's first step after the reference restarts from the measured pose with the sag offset; the bridge's 0.01 rad per update limit). No motion change, as asked.
