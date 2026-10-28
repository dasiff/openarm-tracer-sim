# Phase-start velocity ramp (M195): measured; the carry boundary is not fixed

Written 2026-10-28. Interim: the full suite is still running (see the end). Not pushed, video not re-rendered. Written for: David.

## What was built
`step()` of the manipulation module (`src/mpvi_modules.py`) is now a wrapper: for the first N = ceil(`phase_start_ramp_s` / dt) = 45 ticks of every phase, each arm joint's commanded velocity may change by at most `phase_start_max_accel_rad_s2` * dt = 1.667 * (1/150) = 0.0111 rad/s per tick; afterwards, and mid-phase, nothing changes. Settings in `PICK_PLACE_CFG` with comments (0.3 s, 0.5 / 0.3 rad/s^2; 0 s turns it off). Row M195 and the design plan marker are written. Untouched: sag correction, settling check, minimum-jerk law, waypoints, bridge, duration fix, phase structure; `restart_from_commanded` stays OFF. The default 1.667 needed no tuning.
Episode: `data/demo_record/ramp_20261028_025303` (bridge ON, duration fix ON, restart OFF, ramp ON; log `data/logs/demo_record_ramp_20261028_025303.log`); measurement `data/arm_motion/ramp_20261028_025303/`; beaker and hand `data/arm_motion/carry_boundary/carry_ramp.npz`. Baseline: `smooth2_20261027_171036` (the video's episode).

## (a) Success
Yes: SUCCESS, 4 of 4 steps, 13868 ticks, 92.5 s (baseline 13869, 92.5 s). Replay bit-identical; 0 bridge events.

## (b) Commanded velocity: peak one-tick step at each boundary, rad/s (baseline / ramp)
| into | baseline | ramp |
|---|---|---|
| PICK start / OVER (skill start) | 0.103 | 0.103 |
| PICK DOWN | 0.161 | 0.033 |
| PICK IN | 0.336 | 0.034 |
| PICK CLOSE | 0.008 | 0.008 |
| PICK UP | 0.167 | 0.033 |
| **PLACE start / OVER (skill start, the carry boundary)** | **0.553** | **0.532** |
| PLACE DOWN | 0.278 | 0.033 |
| PLACE OPEN | 0.010 | 0.009 |
| PLACE BACK | 0.500 | 0.033 |
| PLACE HOME | 0.243 | 0.033 |
The ramp limit is 0.0111 rad/s per tick; the bridge changes the command only every 3rd tick, so one step of the applied command is 3 * 0.0111 = 0.033 rad/s, which is what every boundary inside a skill now shows (the limit holds). The two skill starts are unchanged.

## Why the carry boundary is not fixed
At a skill start the first command is not a velocity of the solver: `start()` calls `arm.reset(data)`, which sets the command to the MEASURED joints, 0.07 to 0.11 rad away from the ctrl applied a tick before (the arm hangs below its command under the beaker's weight). The bridge then moves the applied command toward it at its own limit, 0.5 rad/s, so the applied command goes from 0.03 to 0.5 rad/s in one update on several joints (k = +2: +0.500 +0.125 +0.248 -0.500 +0.297 -0.391 +0.500 rad/s). My ramp limits the change of the solver's velocity from the command it starts from, and that command is already the reset value, so it has nothing to cap. This agrees with the numbers (both skill starts unchanged to the third decimal, every in-skill boundary cut to the limit) but I did not test it separately. The earlier `restart_from_commanded` attempt tried to remove this jump and failed for another reason (report 20261028_0237). A ramp on the applied command itself (after `arm.reset`, or in the bridge) would cover the skill starts; I did not build it: it is a different place and a design choice.

## (c) Peak actual jerk, 150 Hz, rad/s^3 (ramp / baseline)
Mid-phase: PICK OVER 72 / 72; DOWN 8 / 8; IN 42 / 40; CLOSE 4.0 / 3.1; UP 92 / 85; PLACE OVER 70 / 87; DOWN 1777 / 3845; OPEN 72 / 73; BACK 36 / 34; HOME 39 / 40.
Boundary window (+-0.5 s), into: PICK DOWN 26 / 61; IN 99 / 76; CLOSE 12241 / 12242 (the gripper); UP 248 / 141; PLACE OVER (carry) 202 / 228; PLACE DOWN 71 / 64; OPEN 483 / 493; BACK 75 / 100; HOME 27 / 35. So: lower at PICK DOWN, PLACE BACK and HOME, PLACE DOWN mid-phase is half; slightly higher at PICK IN (99 vs 76) and CLOSE to UP (248 vs 141); the rest about equal. PLACE OVER's first 1.5 s: 202 / 228, commanded jerk 11971 / 12430 (the jump above).

## (d) Peak actual joint speed per phase, rad/s (ramp / baseline)
PICK OVER 0.520 / 0.520; DOWN 0.347 / 0.347; IN 0.398 / 0.398; CLOSE 0.321 / 0.321; UP 0.354 / 0.354; PLACE OVER 0.350 / 0.348; DOWN 0.455 / 0.550; OPEN 0.090 / 0.089; BACK 0.439 / 0.432; HOME 0.487 / 0.487.

## (e) PLACE OVER: beaker tilt and hand dip
Peak beaker tilt in the world 2.82 degrees at t 80.77 (baseline 2.76); the pad centre dips 17.6 mm (baseline 17.3 mm). Unchanged, as expected.

## (f) Place accuracy
Final beaker base against the hotplate centre (0.65, 0.66): 0.89 mm (offset -0.55, +0.70 mm); baseline 0.57 mm (the 0.6 mm you quote). Height 1.0279 m and tilt 0.02 degrees in both. 0.3 mm difference, not meaningful. Measured at the last replay sample.

## (g) Durations
No phase is noticeably longer: all phase durations within 0.02 s of the baseline (PLACE OVER 2.16 / 2.15 s, PLACE DOWN 1.93 / 1.95 s); the whole episode 92.5 s in both. The planned times are unchanged (the ramp does not enter the duration formula).

## Suite
Full suite with the ramp ON, bridge OFF (default), fix ON: tmux `suite11`, TAG 20261028_030910, summary `data/logs/suite_20261028_030910_summary.txt`, face_test skipped in the run and to be run alone after it (as before). Expected about 4.5 hours; `mpvi_text_test` is expected to fail (VLM cache). Results will be added to this file when it ends. Until ALL_DONE, I do not edit `src/` or `experiments/`.

## Questions
1. The carry boundary bump comes from the reset at a skill start. Should I limit the applied command's velocity there too (after `arm.reset`)? Or restart the command from the applied ctrl but seed the sag correction (the variant that failed before, with its cause now clearer)?
2. A ramp on the gripper command at CLOSE and OPEN (your item 1 says CLOSE and OPEN too): the arm is held during both, so the ramp does nothing there; the gripper target is a force-limited position in metres and the grip force tests depend on it, so I did not build it. Do you want it?
