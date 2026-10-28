# PLACE OVER turn rate (item 1), panel phase labels (item 2), pedestal height sweep (item 3)

Written 2026-10-28 for David. Measurements only except item 2 (panel text) and the off-by-default option `MINK_CFG["uniform_speed_scaling"]` (item 1f). Not pushed, no video re-rendered.
Data: `docs/reports/logs/place_over_turn_smoothmj.json`, `docs/reports/logs/place_over_turn_ticks_smoothmj.csv` (the per-tick table), `docs/reports/logs/pedestal_sweep_*.json`.
Episode measured in item 1: `data/demo_record/smoothmj_20261028_031803` (the one in the 11:19 video; bridge ON, min-jerk and skill-start ramps), replayed bit-identically.

## Item 1: PLACE OVER turn rate (the carry)

Premise to correct first: **no joint reaches the 0.5 rad/s limit during PLACE OVER** (recorded commanded speed, 150 Hz tick level): peaks per joint in rad/s: J1 0.22, J2 0.41 (at 0.89 s), J3 0.35, J4 0.32, J5 0.19, J6 0.21, J7 0.31. The only speed clipping in the phase is the phase-start (skill-start) ramp cap, in the first 0.34 s (below). And the turn is not done in 0.89 s: the move uses one minimum-jerk profile s(u) for the translation and the turn, so both run over the whole planned time.

### (a) Turn rate against translation speed
- Planned time 1.68 s (logged "1.7 s planned") = max of: minimum move 1.0 s; distance/speed 0.2014 m / 0.12 m/s = **1.68 s (binds)**; turn/turn limit 25.48 deg / (0.5 rad/s = 28.6 deg/s) = 0.89 s; 2.0 x largest joint arc 0.287 rad / 0.5 = 1.15 s. So 0.89 s is only the turn limit's lower bound, it does not bind.
- Mean turn rate 15.2 deg/s (0.265 rad/s) over the 1.68 s against a mean translation speed of 120 mm/s. Per the min-jerk profile both peak at 1.875 x the mean at the middle of the move (t = 0.84 s): turn 28.3 deg/s, translation 224 mm/s (reference). Measured (pad pose of the recording): turn peak 34 deg/s, translation peak 265 mm/s (the sag correction adds to the reference).
- Ratio: 25.5 deg of turn over 201 mm of travel: 0.127 deg per mm, constant over the move (same profile).

### (b) Joints at the limit
None in the mid-phase (above). In the first 0.34 s (ticks 3 to 51 of the phase) 7, then 5, 4, 2, 3 joints are held at the ramp's cap (the per-joint clip of the min-jerk envelope 0.5 x min_jerk(t/0.5625 s), 3 ticks later in the actuator command because of the command bridge): the skill-start snap to the measured joints is spread over the ramp. At tick 51 the cap lets go and the joint velocities change at once (J3 0.31 to 0.12 rad/s, J2 0.02 to 0.24 within 10 ticks): this is the visible "entering and leaving": it is the end of the snap-removal, not the 0.5 rad/s solver limit. See (e).

### (c) Turn rate halved (turn_rad_s 0.25), from the IK dry run
Turn bound becomes 1.78 s, so the planned time is 1.78 s (+0.10 s) instead of 1.68 s; the largest joint speed in the dry run falls from 0.331 to 0.312 rad/s; no joint saturates. (The turn was already spread over the whole move, so halving the limit changes little.)
Dry run = the skill's solver on a scratch arm from the joint state at the phase start, the same timing formula and ramp, no sag correction, no bridge, no skill-start snap. It does not reproduce the recorded run exactly (largest difference 0.14 rad in the commanded joints, recorded peak 0.405 against the dry run's 0.331 rad/s for J2): the recorded run has the sag correction (up to 26 mm) and the snap.

### (d) Turn first, then translate, from the IK dry run
| leg | planned time | binding term | largest joint speed | any joint at the limit |
|---|---|---|---|---|
| turn in place (25.5 deg) | 2.53 s | joint arc (2.0 x arc / 0.5; the turn alone needs a long joint arc: the same pad turn with no translation) | 0.473 rad/s (J6) | no |
| straight move (201 mm) | 1.94 s | joint arc | 0.481 rad/s (J2) | no |
| total | **4.47 s** (+2.8 s over 1.68 s; the settle dwell of each leg not counted) | | | |
Neither leg reaches 0.495 rad/s, both come close (J6 0.473, J2 0.481). Doing the turn first is slower by 2.8 s and the joints are nearer the limit than in the combined move.

### (e) Per-tick table, first 1.5 s
`docs/reports/logs/place_over_turn_ticks_smoothmj.csv`, 225 rows (150 Hz ticks, tick 0 = the first tick of the phase). Columns: tick, time, profile fraction u, joints at the 0.5 rad/s limit, joints at the ramp cap, the commanded velocity of the 7 joints (rad/s, the actuator command; it changes every 3rd tick because the command bridge takes a command at 50 Hz, so the values come in groups of three), the ramp cap, the reference translation speed (mm/s) and turn rate (deg/s) of the minimum-jerk profile, the sag correction (position mm, rotation deg), the largest tick-to-tick change of a joint's velocity and which joint, and the difference between my rebuilt command and the recorded one.
Reading it:
- ticks 0 to 2: nothing (bridge delay, 3 ticks); ticks 3 to 21: all 7 joints at the cap (about +-0.0116 to 0.07 rad/s, all joints the same speed = the cap, the direction is the snap's, not the reference's); 21 to 51: the set falls to J1/J3/J4/J7, then J3/J7/J4; tick 51 (0.34 s): the cap lets go.
- from tick 51 the velocities follow the reference (reference speeds grow from 0 to 224 mm/s at 0.84 s, back to 0 at 1.68 s). No joint at 0.5 rad/s after that.
- Whether the solver switches solutions: the velocity changes sharply three times, all at the end of the cap period: tick 39 (J1, 0.21 rad/s in one update), tick 42 (J4, 0.23), tick 51 (J3, **0.32**, the largest) and tick 54 (J7, 0.11); from tick 55 on the largest change in one update is 0.04 rad/s (at tick 57 and 60). No switching later in the 1.5 s.
- The sag column is rebuilt (it is not recorded): it grows 0 to 26 mm during the first 1.3 s and is the integral of the measured pad pose error, so it is the same as the real run's up to the target tolerance. The rebuilt commands differ from the recorded ones by up to 0.105 rad (mostly an offset set in the first 0.3 s), so use the joint velocity columns (recorded), and the sag and reference columns as approximate.

### (f) Speed limit as one factor on the whole velocity (uniform scaling)
Option added, default off: `MINK_CFG["uniform_speed_scaling"]` (src/task_config.py, src/mink_arm.py step(), the phase-start cap in src/mpvi_modules.py). On: the solver has no per-joint velocity constraint; after the solve the whole 7-joint velocity is multiplied by min(1, 0.5 / its largest joint speed) before it is integrated into q_cmd; the phase-start cap is scaled the same way. mink's solver is unchanged. Episode: `experiments/uniform_scaling_episode.py` (moved layout, bridge ON, both ramps), `data/demo_record/uniform_20261028_123126`, measured the same way as the smooth episode (compare: `data/arm_motion/uniform_20261028_123126/compare.md`).
- **Success:** yes, 4 of 4 steps, 13860 ticks (92.4 s; the smooth episode 13884 ticks, 92.6 s).
- **Smoothness of PLACE OVER:** not changed. The largest commanded joint speed (0.1 s average) has two peaks, as before: uniform 0.28 rad/s at 0.31 s and 0.41 at 0.90 s; per-joint 0.27 at 0.35 s and 0.40 at 0.91 s. The first peak is the snap spread by the ramp, the second is the move itself. Because no joint reaches the limit in this phase the scaling hardly acts here; it acts in PICK OVER (peak joint speed 0.500 uniform, 0.520 per-joint, actual).
- **Beaker tilt and hand dip in PLACE OVER:** tilt 2.73 deg (per-joint 2.92); dip 17.0 mm (per-joint 18.8); slide 2.1 mm (2.2); tilt in the hand 0.01 deg (0.03).
- **Duration:** phases within 0.12 s of the per-joint run (PICK OVER 2.70 s against 2.75 s, PLACE OVER 2.18 against 2.30); nothing is longer.
- **Peak jerk, 150 Hz actual, rad/s^3, uniform / per-joint:**

| | boundary into the phase | mid-phase |
|---|---|---|
| PICK OVER | 1125 / 1125 (the staging end) | **121 / 72** |
| PICK DOWN | 8.3 / 17.0 | 8.1 / 8.2 |
| PICK IN | 22.9 / 35.3 | 38.7 / 38.3 |
| PICK CLOSE (the gripper) | 11834 / 12068 | 4.4 / 5.6 |
| PICK UP | 279 / 310 | 87 / 99 |
| PLACE OVER (the carry) | **79 / 178** | 114 / 97 |
| PLACE DOWN | 114 / 75 | 3591 / 2776 (the contact) |
| PLACE OPEN | 486 / 528 | 74 / 74 |
| PLACE BACK | 60 / 54 | 43 / 42 |
| PLACE HOME | 32 / 20 | 36 / 38 |
Summary: the carry boundary jerk halves (79 against 178), PICK OVER mid-phase jerk rises (121 against 72, where the limit binds), PLACE DOWN contact jerk is higher (it varies from run to run). No phase is slower.

## Item 2: panel phase labels (done)
`src/viewer_layout.py`: `phase_action()` and the table `PHASE_ACTIONS`; `panel_from_status()` gives the plan block an "action" and, only while the phase is NAV: ARMS, a "now" text; `panel_block()` shows the action as a "doing" row (normal text) above the internal code (small, as before) and the NOW row shows "Stowing arms" during ARMS, then the step text again.
| code | shown |
|---|---|
| NAV: ARMS | Stowing arms (also the NOW row) |
| NAV: PEDESTAL (not in your list, added) | Setting torso height |
| NAV: DRIVE | Driving to the beaker / Driving to the hotplate (from the step's text "Navigate to X"; "Driving to the next position" if the step text has another form) |
| NAV: CREEP_IN | Creeping into position |
| NAV: DONE (added) | Stopped at the bench |
| PICK: OVER / DOWN / IN / CLOSE / UP | Move over beaker / Lower to beaker / Close on beaker / Grip beaker / Lift beaker |
| PICK: CHECK (added) | Check the beaker is held |
| PLACE: OVER / DOWN / OPEN / BACK / HOME | Move over hotplate / Lower to hotplate / Release beaker / Back away / Return to home pose |
| PICK: BACK, HOME, OPEN (the end-at-home and recovery moves, added) | Back away / Return to home pose / Release beaker |
| any phase with status [checking] (added) | Checking the result |
| GROUNDING / ASK OPERATOR / RETRY n (added) | Finding the object / Waiting for the operator to choose / Retrying: arm to the home pose |
| "starting" (between steps, added) | Holding still, next step starting |
| PLAN DONE | All steps done |
Checked on the status texts of the episode and on a rendered panel (ARMS, DRIVE, PLACE OVER). Not changed: the NOW row during NAV: PEDESTAL still shows "Navigate to the beaker" (you asked for ARMS only; the same fix would be one more entry).


## Item 3: pedestal height sweep (0.10, 0.15, 0.20 m), moved layout, measurement only

Method. Same stance in all three (the selection's base position, heading and place hand turn; it chose pedestal 0.10); only `task_cfg["pedestal_q"]` of its result is replaced (`experiments/pedestal_episode.py`). 0.10 is the existing episode `smoothmj_20261028_031803` (same code, bridge ON); 0.15 and 0.20 are `data/demo_record/ped015_20261028_125039` and `ped020_...`. Nothing was tuned. Each recording is replayed bit-identically (`experiments/pedestal_sweep_measure.py`); the phase windows come from the episode log (10 Hz). The default pedestal height is unchanged (0.10).
"Elbow" = the collision geoms of `openarm_<side>_link4` (the body of joint 4), distance by `mj_geomDistance` to every non-robot collision geom (bench, hotplate, beaker), at every control tick of the window, both arms. Raw numbers: `docs/reports/logs/pedestal_sweep_{010,015,020}.json`.

### (a) Does the episode succeed?
| pedestal | result |
|---|---|
| 0.10 m | SUCCESS (92.6 s) |
| 0.15 m | SUCCESS, 4 of 4 steps |
| 0.20 m | **PLAN_FAILED** at step 2 (pick), phase IN: "not reached after 15.7 s: 18.9 mm and 0.0 deg from the waypoint"; the retry (arm home, step again) failed the same way, 18.9 mm. The place was never reached. |
Why 0.20 fails: at the end of IN the left arm's joint 4 (the elbow joint) is at **10.0 deg (0.174 rad) from its limit, exactly the solver's joint margin (MINK_CFG joint_margin_rad = 10 deg)**; the solver holds the joint there, so the pad stops 18.9 mm short of the beaker centre with the orientation right. At 0.10 and 0.15 the IN waypoint keeps joint 4 at 56.9 and 33.9 deg from its limit (joint 2 is the closest, 25.6 and 22.1 deg).

### (b) Closest elbow to an obstacle (mm), in the extended phases
| phase | 0.10 m | 0.15 m | 0.20 m |
|---|---|---|---|
| PICK OVER | 276 | 304 | 326 |
| PICK DOWN | 278 | 293 | 286 |
| PLACE OVER | 155 | 180 | not reached |
| PLACE DOWN | 153 | 176 | not reached |
(PICK IN, for reference: 177, 184, 204.) The nearest obstacle is **always the bench top (bench/bench_top, its surface under the elbow), always the left elbow** (the right arm stays at home, its elbow is never nearer); never an edge, the hotplate or the beaker. The place is the tightest (the arm reaches farthest out over the bench there): 153 mm at 0.10 m, 176 mm at 0.15 m, so +5 cm of pedestal gives +23 to +28 mm of elbow clearance (about half the lift). For the hand: the fingers are 34 mm above the bench top at PICK DOWN and 20 to 22 mm at IN (the solver keeps 8 mm) at every height, and 26 to 27 mm above the hotplate at PLACE DOWN (0.10 and 0.15).

### (c) Joints closest to their limits (left arm; the solver's own margin is 10 deg = 0.175 rad)
Closest during the phase (minimum over the window) and at the waypoint at the end of the phase:
| phase | 0.10 m during | 0.10 m at waypoint | 0.15 m during | 0.15 m at waypoint | 0.20 m during | 0.20 m at waypoint |
|---|---|---|---|---|---|---|
| PICK OVER | J4 0.213 rad, 12.2 deg | J4 0.279, 16.0 | J4 0.242, 13.9 | J4 0.425, 24.4 | J4 0.243, 13.9 | J2 0.596, 34.1 |
| PICK DOWN | J4 0.279, 16.0 | J2 0.550, 31.5 | J4 0.425, 24.3 | J2 0.496, 28.4 | J2 0.434, 24.9 | J2 0.434, 24.9 |
| PLACE OVER | J2 0.244, 14.0 | J2 0.248, 14.2 | J2 0.224, 12.9 | J2 0.229, 13.1 | not reached | |
| PLACE DOWN | J2 0.237, 13.6 | J2 0.237, 13.6 | J2 0.209, 12.0 | J2 0.209, 12.0 | not reached | |
(PICK IN: J2 0.446 rad 25.5 deg; J2 0.386 rad 22.1 deg; J4 0.166 rad 9.5 deg at 0.20 m.) The "during PICK OVER" values are mostly the home pose (J4 13.9 deg from its limit at home) and the first part of the move. The place is nearest to the joint-2 limit (shoulder), and a higher pedestal brings it nearer (14.2 deg at 0.10 m to 13.1 deg at 0.15 m at the place OVER; 13.6 to 12.0 deg at DOWN), the pick is nearest to the joint-4 limit (elbow) and a higher pedestal first helps (J4 at the OVER waypoint 16.0 to 24.4 deg) and then, at IN and 0.20 m, hits the margin. The right arm stays 13.9 deg (J4, home pose) from its limit throughout.

### (d) Can the hand still reach the beaker and the hotplate surface?
- 0.15 m: yes, in the episode (success), and in the IK dry run (below).
- 0.20 m: the pick fails at IN, 18.9 mm short (above). IK dry run (the left arm from home to each recorded waypoint of the 0.10 episode along a straight pad path, the skill's solver with its limits, from the joint state at the start of PICK OVER of each episode): pose error at the end, mm: PICK OVER 0.5 / 0.4 / 0.5, PICK DOWN 0.6 / 1.0 / 1.9, PLACE OVER 1.0 / 1.3 / **2.0**, PLACE DOWN 1.2 / 1.7 / **3.1** for 0.10 / 0.15 / 0.20 m. The skill accepts a waypoint within 3 mm, so at 0.20 m the place set-down is at the edge (3.1 mm short) and the OVER stop is 2.0 mm short; these are the dry run's numbers, the episode never got to the place. The IN waypoint is not in the dry run: the episode itself shows the shortfall (18.9 mm). The joint-range-only dry run (no collision avoidance) gives the same errors, so the shortfall is the joint limits (with their 10 deg margin), not the bench.

### (e) Maximum downward reach
Lowest pad-centre height reached (m above the floor, from the OVER pose lowered in 5 mm steps at the pick's standoff position and at the place position, each step solved to convergence, stop when the error exceeds 2 mm), against the height the skill needs (pick DOWN 1.013 m, place DOWN 1.090 m; bench top 0.96 m):
| | 0.10 m | 0.15 m | 0.20 m |
|---|---|---|---|
| pick, solver limits (joints with margin, bench and hand collision avoidance) | 0.984 (28 mm below needed) | 0.979 (33 mm) | 1.009 (**3 mm** below needed) |
| pick, joint range only (no collision avoidance) | 0.909 (search stopped at 100 mm below) | 0.959 (53 mm) | 1.009 (3 mm) |
| place, solver limits | 1.066 (24 mm below needed) | 1.076 (14 mm) | the OVER pose is already 2.0 mm short |
| place, joint range only | 1.026 | 1.076 | the OVER pose is 2.0 mm short |
So at 0.10 m the arm could reach well below the grasp height (the limit there is the bench top: the fingers would touch it, 0.984 m is 24 mm above the top); at 0.15 m the margin below the needed pick height is still 33 mm (joint range only 53 mm) and 14 mm at the place; at 0.20 m the margin at the pick is 3 mm: the reach limit is the arm's, and the real IN move, which goes farther out, fails.

### Reading
Raising the pedestal from 0.10 to 0.15 m gives about 25 mm more elbow clearance over the bench at every extended phase and keeps the episode successful with the same stance; the shoulder (joint 2) margin at the place shrinks from 13.6 to 12.0 deg and the margin below the place height from 24 to 14 mm. At 0.20 m the pick fails at IN because the elbow joint reaches its margin. Caveats: one episode per height (a single stance chosen for 0.10 m; the stance finder was not rerun for the other heights, so a stance better suited to them might do better, this was not tried); the 0.20 m place phases were not measured in an episode; the dry-run reach uses straight pad paths from home and no beaker.
