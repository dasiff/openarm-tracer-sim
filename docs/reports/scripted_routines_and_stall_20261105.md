# Scripted pick and place: how the move works, the three corrections as loop layers, and the IN stall (Nov 5, 2026)

Read-only study. No code changed, no simulation run, no baseline registered or regenerated. Every number below is from the existing logs.

## Part 1: how a scripted move works, tick by tick

### The steps
Pick (src/mpvi_modules.py plan_pick, lines 236 to 258):
1. OVER: move the hand above the beaker's stand-off point at the clearance height (fast).
2. DOWN: lower the hand to the grasp height in front of the beaker (slow).
3. IN: move the hand forward onto the beaker, between the fingers (slow).
4. CLOSE: close the gripper with the grip force (a gripper step; ends when the fingers stop).
5. UP: lift the beaker to the carry height (slow).

Place (plan_place, lines 259 to 283):
1. OVER: move the held beaker above the hotplate spot, keeping the hand angle chosen by the stance search (fast).
2. DOWN: lower the beaker onto the hotplate (slow).
3. OPEN: open the gripper (a gripper step).
4. BACK: pull the hand back along its approach (slow).
5. HOME: move the arm to the zombie pose (fast move).
6. HOME (joints): settle the joints at the home pose (joint step).

### One tick of a move step (the scripted order)
Each physics tick the orchestrator calls the manipulation module's step(obs) (line 475), which calls step_inner (line 350).

| # | stage | file and line | input | output | state carried | reset by | in advance or live |
|---|---|---|---|---|---|---|---|
| 1 | start of the step (first tick) | 364 to 379 | the measured pad pose (or the commanded pose if restart_from_commanded) | p0, R0, T, slerp; the blend's start speed v0 if handed over | st (the step dict) | a new step | in advance (fixed at the start), except v0 which is live |
| 2 | time law | 382 to 388 | elapsed time el, T | u = el/T, s = min_jerk(u); p_ref = p0 + (target - p0) s; R_ref = slerp(s) | none | — | in advance (a function of time) |
| 3 | measured pad pose | 390 | joint positions (data) through the kinematics | p_now, R_now | none | — | live |
| 4 | sag trim | 392 (function at line 73) | p_ref - p_now and the rotation error | C["p"], C["r"] (the integrator) | C, carried for the whole run, per arm | never reset for a skill (sag_reset_at_skill_start False, line 437); not integrated while a held-mass ramp runs | live (depends on the measured pad) |
| 5 | inverse kinematics | 396 (MinkArm.step, src/mink_arm.py 177 to 202) | p_ref + C["p"], R_ref with the rotation trim | arm.q_cmd: the 7 absolute joint targets, after one velocity step of dt | q_cmd (the solver's own joint state), carried | arm.reset at a skill start (lines 298, 325), i.e. q_cmd snaps to the measured joints | live (the solver starts from the previous q_cmd) |
| 6 | completion and blend tests | 404 to 416 | measured pad, target, joint speed | advance flag | pad_prev, still timer | a new step | live |
| 7 | command | 340 to 344 (command), 416 | arm.q_cmd, the gripper value | {"arms": 7 targets, "gripper"} | none | — | live |
| 8 | phase-start speed ramp | step() 475 to 509 (lines 492, 497 to 506) | the commanded change v = (q_cmd - q_before)/dt | the joint speed capped by min_jerk over 0.5625 s from the phase start | ramp_k (counts ticks since the start) | a new step sets ramp_k = 0; a blended handover sets it to no ramp | the trigger is in advance (the step start); the speed is live |
| 9 | out of the module | orchestrator (step each tick); sequential policy | the command | the source's output | — | — | — |
| 10 | source to loop | src/control_loop.py 1072 to 1093 (validate), 1110 (bridge_take) | the command | the clean command | — | — | — |
| 11 | command bridge | control_loop.py 647 (bridge_take: keeps the latest target), 656 (bridge_update, every 3rd tick at 50 Hz) | the latest target per arm, the measured joints | the command moved at most 0.5 rad/s (0.01 rad per update), rejected if a step is over 0.25 rad, suspended if the tracking error is over 0.25 rad; applied at line 690 | the command cmd_q (carried) | — | live (depends on the measured joints) |
| 12 | apply to the controller | apply_cmd (559), ramp_from / hold_qpos_targets | the bridge's command | hold_qpos_targets; a linear ramp from the last ctrl over the command period | — | — | — |
| 13 | per physics step | ctrl_block (446 to 457) | the ramp | the ctrl of each actuator (linear interpolation) | ramp_k, ramp_from | — | — |
| 14 | gravity feedforward | gff.apply (control_loop.py 410, 435; src/gravity_ff.py) | the measured pose and the held mass | ctrl += tau_ff / kp | the held-mass record | the held-mass ramp | live |

### What a source that sends only absolute joint targets can supply in advance
- In advance (a chunk can carry these): the time law (stage 2), the path from p0 to the target, and the joint targets that the solver would produce along that path, as long as the solver is started from the same joints. The mock (M260/M261) does exactly this and reproduces the scripted timing.
- Cannot be supplied in advance: the sag trim (stage 4 integrates the measured error), the blend handover (stage 6 uses the live pad speed at the handover), the phase-start speed ramp's start (stage 8 needs the moment the phase starts, which is a chunk boundary for a source), the completion test (stage 6, live), and the bridge's suspension (stage 11, live).

## Part 2: the three corrections as layers in the control loop

### Sag trim
- What it is for: the arm sags under its own weight and under the held beaker, so a command that is exactly on the path leaves the hand a little low or to one side. The trim is a slow integral of the measured error that pushes the command the other way, until the measured hand reaches the path.
- Reference path today: p_ref(s), the scripted min-jerk path from the planner (stage 2). A loop layer would need the path the source intends. A source that sends only joint targets does not provide a pad path, so the loop would have to compute the pad pose of each target by kinematics (forward kinematics of the chunk's joints) and use that as the reference. **NEEDS DECISION:** reference = forward kinematics of the source's own target (options: the chunk's joints per tick; or the path of the step if the loop knows it). Option 1 is the simplest and needs nothing from the planner.
- Per-tick dependence: the integral is taken every tick with dt (line 392). A loop layer runs every tick, so it does not depend on the script; the reference must be available each tick, which a 30 Hz chunk gives by interpolation.
- Held-mass ramp: the trim is frozen while a held-mass ramp runs (M238, hmr.ramp_running, line 83). The loop layer can read the same record.
- Helpers: while the gate pauses a source (open catch, weighing), no reference is produced. **NEEDS DECISION:** during a pause, hold the trim value (as it is now) or keep integrating against the held target. Holding is the default.

### Phase-start speed ramp
- What it is for: at the start of a phase the solver's first command can jump, so the ramp eases the joint speed in from zero over 0.5625 s (M195, M198). It is a speed envelope, not a position change.
- What would trigger it without step boundaries: the scripted trigger is the start of a step (S["t0"] is None, line 491). A source has no step boundaries, only chunk boundaries. **NEEDS DECISION:** trigger at each chunk's first command; or at a large jump in the source's target (for example over 0.05 rad in one tick); or never (rely on the source's own smoothness). Each changes when the ramp acts.
- Per-tick dependence: the ramp is applied on each tick of its window (step() line 497). In a loop layer it needs the commanded velocity each tick, which the loop has.
- Interaction with the bridge: the bridge already caps joint speed at 0.5 rad/s (COMMAND_BRIDGE_CFG) and rate-limits each update (0.01 rad). It does not ease a start from rest, so it is not a substitute for the ramp.
- Held-mass ramp and helpers: none directly. A pause stops the source; the ramp's window would be counted in ticks, so after a pause the ramp would restart only if its trigger fires again. **NEEDS DECISION:** whether a resumed source gets a fresh ramp.

### Blend handover
- What it is for: between two moves the scripted hand does not stop; if it is within 20 mm of the waypoint at the end of its time law and the next step is a move, the next move starts at once with the hand's current speed (M200, lines 407 to 410). The next move's position is a cubic that starts from that speed (line 385).
- Is it needed when consecutive chunks join smoothly? The mock's joins are 0.90 times the largest in-slice step in joint and pad space (mock test f), so the source's own chunks already join without a stop. The blend is needed only when a source stops at each waypoint. **NEEDS DECISION:** keep a blend layer (options: a loop layer that blends two source moves by the pad speed at the join; or none, relying on the source's chunks). Option "none" is what the mock does today.
- Per-tick dependence: the decision is made on each tick (line 408) from the live pad. A loop layer needs the next target to decide; a chunk gives it.
- Interaction: none with the held-mass ramp. With the gate: a pause between two moves would end the blend, since the handover speed is the live speed before the pause.

### Summary of NEEDS DECISION items in Part 2
1. The sag trim's reference for a source that sends only joints (options: forward kinematics of each target; the planned path if the loop has it).
2. The trim during a helper pause (hold, or keep integrating against the held target; default hold).
3. The ramp's trigger without step boundaries (chunk start, target jump, or none).
4. Whether a resumed source gets a fresh ramp.
5. Whether the blend is kept as a loop layer or dropped (default dropped, as the mock does).

## Part 3: the IN stall, replayed from the logs

### (a) How completion of a move is judged (scripted; lines 404 to 416)
- Position: the measured pad centre within tol_m = 0.003 m (3.0 mm) of the target.
- Orientation: the angle between the measured pad frame and the target frame within tol_rad = 2.0 degrees.
- Joint speed: every arm joint slower than settle_speed_rad_s = 0.02 rad/s for settle_dwell_s = 0.2 s.
- Time law: u = 1 (the planned time T has elapsed).
- Blend alternative: within blend_radius_m = 0.020 m at u = 1 with a next move, advance at once (line 408).
- Timeout: 3 x T + 6.0 s (timeout_factor 3.0, timeout_extra_s 6.0), line 413.
- Mock (M260/M261): the same position and angle tolerances, the same time law, the same timeout; the joint speed is the difference between successive sensed states; no blend.
- IN values: T = 3.019 s (both runs), timeout 15.06 s.

### (b) Scripted run, IN (ticks 10468 to 10998, 3.53 s): the sag trim per axis (mm)
Target = (0.849998, 0.660000, 1.012314) m: the beaker's position at the pick's start and the reference height at IN's end.

| moment | tick | x | y | z | |C| |
|---|---|---|---|---|---|
| start | 10468 | -3.07 | -0.72 | +1.02 | 3.31 |
| +1 s | 10618 | -3.03 | -3.65 | -2.58 | 5.40 |
| +2 s | 10768 | -3.57 | -5.32 | -0.11 | 6.41 |
| end | 10998 | -2.10 | -2.67 | -1.30 | 3.64 |

The trim grows for about 2 s (the integral of the lag) to a maximum of 6.53 mm at tick 10714, then falls as the pad closes in. The pad error to the target: 120.0 mm at the start, 0.80 mm minimum at tick 10895, 1.29 mm at the end (the logged err_m, 0.28 degrees orientation).

### (c) Mock run, IN (signal-trace ticks 10562 to 12861): pad error to the same target
| query (tick) | progress (s of 3.019) | pad error (mm) |
|---|---|---|
| 20 (10561) | 0.000 | 120.76 |
| 26 (10837) | 1.200 | 83.06 |
| 32 (11113) | 2.476 | 5.90 |
| 38 (11389) | 2.589 | 3.37 |
| 50 (11941) | 2.589 | 3.36 |
| 68 (12769) | 2.589 | 3.36 |
| 69 (12815) | 2.589 | 3.36 |

The error stops improving at about tick 11300 (query 16 of IN: 3.37 to 3.39 mm from then to the end). The progress stops at 2.589 s of 3.019 s at about tick 11240 and stays there: the last 0.43 s of the path lies within about a millimetre of the target, so the progress search has nothing further to find. The final offset from the target: x +1.81, y +2.80, z +0.58 mm (3.38 mm). The orientation at the failure: 1.0 degree (from the failure message).

### (d) Calculation (labelled as such; not a simulation)
Assumption: the arm's pad offset moves one-for-one with the command's offset, and the scripted trim at IN's end is the correction the arm would need.
- Mock offset from the target: (+1.81, +2.80, +0.58) mm.
- Scripted trim at IN's end: (-2.10, -2.67, -1.30) mm.
- Sum: (-0.29, +0.13, -0.72) mm, length 0.80 mm: inside the 3.0 mm tolerance.
- The brief's shortfall relative to the scripted pad (2.95, 3.07, 0.07) plus the same trim: (+0.85, +0.40, -1.23) mm, length 1.5 mm: also inside.
This is a calculation, not a result. The trim's value is history-dependent, so the real correction would differ.

### (e) Does this confirm the missing trim?
Partly. The offset is steady (flat for about 1,600 ticks), horizontal (x and y), and its sign is opposite to the scripted trim at IN's end, which is the direction a correcting integral would push. That fits a steady offset that a missing integral correction would leave in place. It does not prove it: no run has added the trim.
Other possible contributors:
- Orientation: the mock was 1.0 degree off at the failure; the scripted rotation trim is also absent in the mock (it corrects both position and rotation), and a 1 degree rotation moves the pad centre by the lever arm times 0.017 rad (the lever arm was not computed here).
- The mock's kinematics-only target: the servo's gravity sag is not in the mock's targets at all, so the offset is consistent with it.
- Timing: the mock's IN starts from the sensed pose at its first query, not the commanded pose; the speed ramp is also absent. These change the path's speed, not the steady offset, so they are unlikely to explain a flat 3.37 mm plateau.

### (f) Which other waypoints could stall the same way
Scripted trim at the end of each move, and the logged error at the end (scripted run):

| waypoint | end tick | |C| at end (mm) | x, y, z (mm) | logged error at end (mm) | rot (deg) |
|---|---|---|---|---|---|
| pick OVER | 9915 | 3.66 | -2.97, -1.70, +1.29 | 2.41 | 1.47 |
| pick DOWN | 10467 | 3.31 | -3.07, -0.72, +1.01 | 0.40 | 0.24 |
| pick IN | 10998 | 3.64 | -2.10, -2.67, -1.30 | 1.29 | 0.28 |
| pick UP | 12237 | 4.41 | +2.46, -0.66, -3.60 | 0.78 | 0.44 |
| place OVER | 12490 | 5.73 | -5.60, -0.66, -1.00 | 6.54 | 1.03 |
| place DOWN | 12799 | 4.92 | -0.94, -0.57, -4.79 | 0.64 | 0.16 |
| place BACK | 13210 | 4.25 | +3.22, +0.01, -2.78 | 3.11 | 0.36 |

- Pick OVER and DOWN had trims above 3 mm and the mock completed both. So a large end trim alone does not stall a move; the offset must be steady and in the direction the arm cannot close.
- The candidates, by size of the trim that a mock would have to do without: place OVER (5.7 mm; its logged error of 6.5 mm shows the scripted step ended by the blend, not by the tolerance), place DOWN (4.9 mm), place BACK (4.3 mm, logged error 3.11 mm, just over tolerance), pick UP (4.4 mm).
- HOME (joints) has no trim and no pad tolerance.
- This is a judgement from the scripted trim only; it is not tested. Placing the mock on any of these would decide it.

## Not confirmed / not done
- The blend handover: the step boundaries fall exactly at the end of each planned time (OVER 229 ticks, DOWN 552 ticks), so the advance was immediate; the log does not say whether the blend or the tolerance triggered it. The pad speed at OVER to DOWN is continuous (0.0060 to 0.0063 m/s), which is consistent with a blend; at DOWN to IN it rises from near rest (0.0008 to 0.0063 m/s in 5 ticks), which is consistent with a ramp. The speeds are small, so this is only suggestive.
- The lever arm between the pad centre and the wrist (for the orientation contribution) was not computed.
