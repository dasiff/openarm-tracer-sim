# Helper inventory and the lowering-to-release seam (Nov 4, 2026)

Read-only. No code changed, no simulation run, no baseline registered or regenerated. Nothing committed.

## Summary in plain words

- **What the scripted pipeline does.** A plan is a list of primitives (pick, place). Each primitive is a list of scripted steps: pick = OVER, DOWN, IN, CLOSE, WEIGH, UP; place = OVER, DOWN, OPEN, BACK, HOME. The step changes are driven by the step's own finish test (a time profile plus a check on the measured hand position and joint stillness, or a gripper stall test), never by an outside event.
- **The seam (place DOWN → OPEN) today.** When DOWN finishes (hand within 3 mm and 2° of its target, and the arm's joints slower than 0.02 rad/s for 0.2 s), the script starts OPEN at **tick 12863**. That is the **release decision**. At that same tick the held-mass ramp to zero starts (1.0 s, 150 ticks), while the gripper is still closed. The gripper command opens only when the ramp is finished, at **tick 13014**. Nothing senses the beaker touching the plate.
- **What the data show across the seam (200 g run, replayed, no new run).**
  - The beaker's bottom comes within 0.05 mm of the plate top at **tick 12698**, 165 ticks before the release decision. The hand then overshoots its target by **+3.0 mm** (tick 12774) and rises with the beaker, and only settles around tick 12840. That is the largest hand excursion in the seam.
  - During the held-mass ramp-out (12863 to 12912) the hand sags about 1.2 mm and reaches **11.7 mm/s** (tick 12908). It then stops abruptly at tick 12911, with a deceleration of **−0.76 m/s²** (the largest acceleration in the seam). The beaker bottom is then 0.2 mm above the plate top (calculated from geometry, not from a contact log).
  - At the gripper-open command (tick 13014) the hand sees accelerations of **+0.39 and −0.33 m/s²**, about half the size of the ramp-out stop.
- **Bounce risk.** The largest hand motions are at the DOWN overshoot (12698 to 12840) and at the ramp-out stop (12908 to 12912). Both happen **before** the gripper opens. The release itself adds a smaller shock.
- **Helpers easiest and hardest to move off the script** (observations, not a recommendation): see the end.

Terms: a **primitive** is one of the few high-level actions a VLA would be given (pick, place). A **step** is a scripted phase inside a primitive. A **lead** means a signal changes before the script's release decision; a **lag** means after it. **Feedforward** is a torque the controller adds to cancel gravity; the **held-mass term** is its part for the beaker's weight. **Sag trim** is the integral correction that cancels the arm's droop. **Ramp** is a smooth change of the held mass over time (minimum-jerk curve). **Verdict** means an on or off value computed from raw signals. **Reference** means the height the planned path asks for.

Labels: **shown by the data/code** (read from a file or a saved value, with file and line); **calculated** (computed here, method stated); **guess**; **NOT DETERMINED** (needs a run); **NEEDS DECISION** (for David; not decided here).

## Part 1: scripted phases, grouped by primitive (best guess, for David to correct)

Sources: step lists in `src/mpvi_modules.py` (`plan_pick` lines 201 to 224, steps at 215 to 217 and the optional end-at-zombie steps at 222; `plan_place` lines 225 to 249, steps at 247 to 248), navigation names in `src/viewer_layout.py` line 185 and the episode log.

| Primitive (guess) | Scripted phase | Boundary type (guess) |
|---|---|---|
| Navigation (outside the grasp) | NAV: ARMS, NAV: DRIVE, NAV: CREEP_IN, NAV: PEDESTAL, then PLAN DONE | primitive boundaries |
| Pick, approach | pick: OVER and DOWN (`src/mpvi_modules.py` line 215), then IN (216) | OVER→DOWN and DOWN→IN: **inside a primitive** |
| Pick, grasp | pick: CLOSE (216, kind "close") → WEIGH (217, kind "weigh") | IN→CLOSE and CLOSE→WEIGH: **inside**. WEIGH is the lift-off weighing |
| Pick, lift | pick: UP (217, kind "move", to carry height) | WEIGH→UP: **inside**. End of UP = **primitive boundary** (the beaker is held at carry height; the next primitive starts from outside) |
| Place, carry and over | place: OVER (247, kind "move") | pick→place: **primitive boundary** (guess) |
| Put on hotplate, set down and release | place: DOWN (247) → OPEN (248, kind "open") | OVER→DOWN: **inside**. **DOWN→OPEN is inside** (this is the seam David asked about, and the script's release decision) |
| Put on hotplate, retreat | place: BACK (248) → HOME (248, kind "move") → HOME joints (kind "joints") | OPEN→BACK: **inside**. End of HOME joints = **primitive boundary** |
| Recovery (failure only) | OPEN (if the hand holds nothing, `src/mpvi_modules.py` 301) → BACK → HOME → HOME joints | triggered by a failure, not a phase |
| Primitive start (both) | start of the segment (`start()` line 250): the pick sets the held mass to 0 (line 270) | **primitive boundary** |

Optional end-at-zombie steps (BACK, HOME, joints in `plan_pick` lines 218 to 222) are used only when `end_at_zombie` is set.

## Part 2: helpers

Each helper: where it lives, what turns it on and off, whether that trigger is a primitive boundary or inside a primitive, which sensed signals could replace an inside trigger (already in the code, or would have to be built), what else it needs from the script, how tangled it is, and what would stop it under a bare policy.

**2.1 Arm gravity feedforward (arm part).** `src/gravity_ff.py` `ArmGravityFeedforward.torques` line 107; applied at line 136; `held_pad_z_per_kg` line 151. Trigger: **always on**, every tick, from the measured pose (no phase). Needs from the script: nothing for the arm links. Bare policy: works.

**2.2 Feedforward held-mass term.** Same file, lines 118 to 124: `held_mass_ramp.mass_at` per tick. Trigger: **the held-mass value**, which is set only by scripted steps (pick start, lift-off estimate, ramp-out at OPEN; see 2.4 to 2.7). With no ramp set, the mass is 0 and the term adds nothing. Trigger is **inside a primitive** (the mass is set by steps). Sensed replacement: the weight reading (2.6) or the torque-based estimate (`hand_load` in `src/weight_estimate.py` line 48). Needs: the ramp state. Bare policy: **the held term is always 0**: the beaker's weight is never compensated.

**2.3 Command bridge.** `src/control_loop.py` lines 582 to 586 (rate-limit block; `bridge_every` line 585), setting `COMMAND_BRIDGE_CFG` in `src/task_config.py` line 461 (on by default since 6d9b542). Trigger: **always on**, at its own rate (50 Hz, every third tick). No phase input. Needs: nothing from the script. Tangled: it limits whatever the skills command; no shared state with the scripted steps. Bare policy: works.

**2.4 Sag trim (integral pad correction).** `src/mpvi_modules.py` `_sag_trim_step` lines 71 to 81; called in move steps at line 376; reset at skill start (lines 265 to 267, gated by `sag_reset_at_skill_start`, which is False in `src/task_config.py` line 437); reset at the gripper-open event (line 437); reset in recovery (line 292). Trigger: **inside a primitive** (runs only in scripted **move** steps, and resets at scripted events). It does not integrate while a held-mass ramp runs (`hmr.ramp_running`, held_mass_ramp.py line 58). Sensed replacement: already has its input (measured pad pose). Needs from the script: the reference path (`p_ref`, from the time profile), the step kind, and the ramp state. Tangled: **strongly**. Its freeze depends on the held-mass ramp (2.5, 2.6), and its reset depends on the gripper-open event (2.8). Bare policy: without a reference path, nothing integrates it.

**2.5 Ramp in (pick start).** `src/mpvi_modules.py` line 269 to 270 sets the held mass to 0 at pick start (kind of primitive boundary). Trigger: **primitive boundary**.

**2.6 Ramp in (lift-off estimate, WEIGH step).** Starts at line 450 (`start_rate`, rate 0.5 N/s, when the arm is settled), stops and sets the estimate at lines 457 to 459 (`liftoff_check` in `src/weight_estimate.py` lines 117 to 124; `liftoff_estimate` line 126). Trigger: **inside a primitive**, but **sensed**: the arm is still (joint speed below 0.02 rad/s for 0.2 s, `settled` lines 311 to 318) and the pad has risen 0.3 mm above its baseline for 5 ticks in a row. Needs from the script: the baseline (mean of the last 10 pad heights, from the script's own history `ZH`), the rate, the 0.3 mm threshold, the 1 kg cap (lines 465 to 470), and `held_pad_z_per_kg` (model-based, from the model's gains and Jacobian). Tangled: it is the only real weight estimate, and it feeds the feedforward (2.2) and the sag freeze (2.4). Bare policy: works only if the weigh step runs; the baseline and the hand-height constant come from the model.

**2.7 Ramp out (release).** `src/mpvi_modules.py` lines 413 to 416 (kind "open": `hmr.set_target(0)` every tick of the open step, 1.0 s ramp). Trigger: **the OPEN step start, which is the script's release decision (tick 12863)**. That step starts when the DOWN waypoint finishes (2.8). **Inside a primitive** (the step boundary is scripted). Sensed replacement: none in the code today. Needs from the script: the step boundary and the 1.0 s duration (`held_mass_ramp_s`, `PICK_PLACE_CFG` line 428). Tangled: it starts the sag freeze (M226 c) and it gates the gripper opening (2.8).

**2.8 Gripper-open wait (kind "open").** `src/mpvi_modules.py` lines 411 to 442. Steps: wait until the ramp is finished (line 415, time-based), set the open command (line 417, `open_g` = 0.044 m, `src/task_config.py` line 294), wait for the finger to stop (speed below 0.002 m/s for 0.3 s, lines 419 to 422) and reach open (within 3 mm, line 423), then fire the `gripper_opened` event (line 435), reset the sag correction (line 437), advance (line 439); timeout 4 s (line 440). Trigger: **inside a primitive** (the OPEN step). Finish is **sensed** (finger travel and finger speed). Needs from the script: the ramp end (time) and the open position. Tangled: the gripper opens only after the ramp (2.7). Bare policy: the finish test works; the open command itself is scripted.

**2.9 Gripper close step (kind "close").** Lines 411 to 442 with `closing` true: command 0 m with force cap `GRIP_FORCE_TABLE["glass_beaker"]` = 8 N per finger (`src/task_config.py` line 152); finish on sensed stall; **holding check (travel)** lines 426 to 433: travel between 3 and 40 mm (`held_travel_range_m`, `src/task_config.py` line 446). Trigger: **inside** (pick CLOSE step). Sensed: travel and finger speed. Needs: the force cap for the object (a scripted table lookup) and the close timing (`close_min_s` = 1.0 s). Bare policy: works, except the force cap is looked up per object.

**2.10 Holding checks.** (a) the CLOSE travel check (above, sensed). (b) **Drop monitor**, `src/mpvi_monitors.py` `make_drop_monitor` lines 29 to 70: armed on the `gripper_closed` event, decides from finger travel alone (fires when travel is below the held width minus `drop_margin_m` for `drop_dwell_s`, lines 46 to 66); its reason string comes from the **oracle** (simulator truth, line 60). (c) **Oracle checker**, `src/mpvi_modules.py` `make_oracle_checker` lines 550 to 591: uses the simulator's object pose (`world.object_pose`) and the pad pose to say whether the beaker is held (lines 572 to 591). Trigger: **inside a primitive** (pick close to open). Sensed: (a) and (b)'s decision. Simulator-only: (b)'s reason and (c). The holding verdict (`src/signal_trace.py`, stage 1b) is **log-only** and is not used by any helper. Bare policy: the decision works (travel); the reason and the default checker (`src/mpvi_pipeline.py` line 30, `checker="oracle"`) need the simulator.

**2.11 Waypoint completion and move timing (kind "move").** `src/mpvi_modules.py` lines 365 to 400. Time law: planned time T from distance and speed (lines 342 to 355; speeds `speed_m_s` 0.12, `slow_speed_m_s` 0.04, lines 421 in `src/task_config.py`). Completion: time has run out and the measured pad is within `tol_m` 3 mm and `tol_rad` 2°, and settled (lines 395 to 396); blend handover to the next move within 20 mm (lines 392 to 394, M200); timeout at 3 T + 6 s (lines 397 to 399). Trigger: **step boundaries are scripted; the completion test is sensed** (measured pad pose, measured joint speed). Needs from the script: the target poses, the step order, the speeds, the next step's kind (blend). Tangled: the sag trim (2.4) integrates in exactly these steps. Bare policy: the targets come from the simulator's world (`plan_place` lines 226 to 227 read `world.object_pose`, which is ground truth; see the M245 known cases in `docs/mpvi_decisions.md`). That is the main thing that stops it under a bare policy.

**2.12 Phase-start velocity ramp (M195, M198).** `src/mpvi_modules.py` lines 509 to 525. Trigger: **every step start** (the step counter changes), ramp 0.5625 s. Sensed: none (the solver's own speed, faded in). Needs from the script: the step boundaries. Tangled: a new step resets it; it does not change the held mass. Bare policy: a policy would need its own start ramp.

**2.13 Skill-start resets.** `src/mpvi_modules.py` `start()` lines 250 to 268 (restart from commanded, the arm reset, the optional sag reset, the pick's mass set to 0). Trigger: **primitive boundary**.

**2.14 Recovery.** `src/mpvi_modules.py` `recover()` lines 283 to 304: triggered by a failed step. It opens the gripper only if `world.held_by(side)` is false (line 301). `world.held_by` is in `src/world_model.py` line 183 and reads the simulator (**simulator-only**). Bare policy: needs a sensed "holding" answer (the holding verdict, or a contact or current signal).

**2.15 Re-weighing when the arm is still (the handoff reading, kind "held").** `src/mpvi_modules.py` lines 471 to 488 (uses `reading_status` in `src/weight_estimate.py` lines 21 to 46, call at line 478; `hand_load` line 48; ramp-in to the reading at line 484). **NOT ACTIVE**: no skill plans a step of kind "held" (search of `src/` and `experiments/` shows only the handler, no planner). So today the only weighing is the lift-off weigh (2.6). Trigger if activated: **inside a primitive**, and **sensed** (joint speed below 0.005 rad/s for 0.3 s, at most 1 s wait). Needs: the joint velocities and torques (sensed history). Dead code, so it is not in any current path.

**2.16 Navigation steps.** `src/mpvi_modules.py` `make_scripted_navigation` lines 103 to 135; names NAV: ARMS, DRIVE, CREEP_IN, PEDESTAL. Trigger: scripted phase boundaries (primitive-level for navigation). Simulator-only elements: the base pose and the world model. Not part of the grasp seam.

**2.17 Simulator world model (not a helper, but used by the helpers).** `make_sim_world_model` `src/mpvi_modules.py` line 83, backed by `SimGroundTruthSource` in `src/world_model.py` line 259. The primitives' targets use its object poses (2.11). A visual detector would replace it (decision M245).

## Part 3: the seam from lowering onto the hotplate to opening the gripper

### 3.1 What the code does, tick by tick (today)

Event order, with the file and line that decide each step.

1. **DOWN is running** (the pad moves from the carry height toward the hotplate position plus the pad-to-beaker offset plus `setdown_gap_m` = 2 mm, `plan_place` line 243 and `src/task_config.py` line 420). The move is on a time law (min-jerk over the planned time T, `src/mpvi_modules.py` lines 342 to 355). The **sag trim integrates** (line 376). The held mass is constant (no ramp running).
2. **DOWN finishes** when the time law has run out (u ≥ 1), the measured pad is within 3 mm and 2°, and the joints are below 0.02 rad/s for 0.2 s (lines 395 to 396). In the 200 g run this is at **tick 12862**. Nothing checks whether the beaker has touched the plate.
3. **Release decision, tick 12863.** The OPEN step starts (`advance` at line 439 of the DOWN move, then the next step on the next tick). At its first tick, `hmr.set_target(0)` starts the **held-mass ramp to zero** (line 414, 1.0 s). The **sag freeze starts** (M226 c, through `is_load_changing` in `held_mass_ramp.py` line 102). The gripper command stays closed (line 416 returns while the ramp runs).
4. **Ramp runs, 12863 to 13013.** The held term in the feedforward falls (`gravity_ff.py` lines 118 to 124). The sag correction holds its value (frozen).
5. **Ramp finishes, tick 13013** (`finished`, `held_mass_ramp.py` line 45: time since start ≥ 1.0 s). **Gripper command opens, tick 13014** (line 417: `S["grip"] = open_g`).
6. **Gripper opens.** Finger travel passes 42 mm at 13018 and reaches the open position at 13023. The stall test (below 0.002 m/s for 0.3 s, lines 419 to 422) and the position test (3 mm of open, line 423) then fire the `gripper_opened` event (line 435). The sag correction is reset (line 437), and the drop monitor is disarmed (`src/mpvi_monitors.py`, `gripper_opened` branch).
7. **BACK starts** at tick 13073 (the next step).

**Which helpers change state in between:** the held-mass ramp (2.7), the sag freeze and reset (2.4), the feedforward held term (2.2), the gripper command (2.8), the drop monitor (2.10). The holding verdict (log only) and the still verdict (log only) also change; neither feeds back.

### 3.2 What each signal does through the seam (200 g held run, replayed, no new run)

Source: the stage 1b run `data/signal_trace_1b_20261103/r2_on/` (hand, finger, force, verdicts, held mass, arm force) and its `liftoff_ticks.csv` (beaker bottom, reference). Holding verdict and squeeze are from the closing-only rule (M251) replay. Plate top is 1.028 m (`models/objects/hotplate.xml` line 13 body z 0.96, line 16 top plate centre z 0.064 with half-thickness 0.004).

| Tick | Script step | Pad z (mm) | Beaker bottom (mm) | Pad minus reference (mm) | Hand speed (mm/s) | Hand accel (m/s²) | Held mass (kg) | Arm pad vertical force (N) | Gripper force (N) | Finger1 (mm) | Holding (M251 rule) | Still |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 12690 | DOWN | 1082.637 | 1028.206 | −1.61 | 5.41 | +0.158 | 0.2245 | 1.964 | −16.000 | 37.517 | ON | off |
| 12698 | DOWN | 1082.503 | 1028.043 | −0.98 | 2.10 | −0.009 | 0.2245 | 2.020 | −16.000 | 37.516 | ON | off |
| 12704 | DOWN | 1082.499 | 1028.031 | −0.81 | 2.01 | −0.019 | 0.2245 | 2.110 | −16.000 | 37.517 | ON | off |
| 12727 | DOWN | 1082.790 | 1028.307 | −0.52 | 6.20 | +0.110 | 0.2245 | 2.444 | −16.000 | 37.516 | ON | off |
| 12774 | DOWN | 1086.328 | 1031.816 | **+3.02** | 1.31 | −0.107 | 0.2245 | 1.972 | −16.000 | 37.516 | ON | off |
| 12840 | DOWN | 1083.895 | 1029.379 | +0.59 | 1.46 | −0.008 | 0.2245 | 2.144 | −16.000 | 37.516 | ON | off |
| 12862 | DOWN (last) | 1083.889 | 1029.374 | +0.58 | 0.43 | +0.006 | 0.2245 | 2.129 | −16.000 | 37.516 | ON | off |
| **12863** | **OPEN starts: release decision** | 1083.886 | 1029.370 | n/a | 0.48 | +0.007 | 0.2245 | 2.130 | −16.000 | 37.516 | ON | off |
| 12880 | OPEN (ramp) | 1083.796 | 1029.281 | n/a | 1.06 | +0.008 | 0.2218 | 2.116 | −16.000 | 37.516 | ON | off |
| 12898 | OPEN (ramp) | 1083.488 | 1028.979 | n/a | 5.23 | +0.074 | 0.2050 | 1.981 | −16.000 | 37.516 | ON | off |
| 12908 | OPEN (ramp) | 1082.921 | 1028.420 | n/a | **11.67** | +0.111 | 0.1879 | 1.836 | −16.000 | 37.516 | ON | off |
| 12911 | OPEN (ramp) | — | — | — | — | **−0.755** (largest in seam) | — | — | — | — | ON | off |
| 12912 | OPEN (ramp) | 1082.723 | 1028.207 | n/a | 2.67 | −0.587 | 0.1796 | 1.713 | −16.000 | 37.517 | ON | off |
| 12920 | OPEN (ramp) | 1082.827 | 1028.312 | n/a | 0.37 | −0.222 | 0.1608 | 1.499 | −16.000 | 37.516 | ON | off |
| 12950 | OPEN (ramp) | 1082.707 | 1028.188 | n/a | 1.21 | +0.018 | 0.0791 | 0.694 | −16.000 | 37.516 | ON | off |
| 12986 | OPEN (ramp) | 1082.503 | 1027.976 | n/a | 0.46 | −0.009 | 0.0098 | −0.009 | −16.000 | 37.516 | ON | **ON** |
| 13010 | OPEN (ramp done) | 1082.465 | 1027.938 | n/a | 0.10 | −0.001 | 0.0000 | −0.116 | −16.000 | 37.516 | ON | ON |
| **13014** | **gripper command opens** | 1082.463 | 1027.936 | n/a | 2.65 | **+0.388** | 0.0000 | −0.125 | **+3.785** | 37.623 | **OFF** | ON |
| 13016 | opening | 1082.450 | 1027.922 | n/a | 2.91 | −0.331 | 0.0000 | −0.116 | +0.556 | 40.110 | OFF | off |
| 13018 | opening | 1082.428 | 1027.908 | n/a | 2.35 | +0.008 | 0.0000 | −0.112 | −0.024 | **42.244** (leaves range) | OFF | off |
| 13023 | open | 1082.368 | 1027.895 | n/a | 1.79 | −0.024 | 0.0000 | −0.107 | +0.007 | 43.807 | OFF | off |
| 13073 | BACK starts | 1082.220 | 1027.891 | n/a | 0.15 | −0.000 | 0.0000 | −0.104 | +0.044 | 43.927 | OFF | off |

"Pad minus reference" is not logged after DOWN (the reference is blank), so it is n/a from 12863 on.

**Lead and lag against the two script decisions** (release decision = tick 12863, OPEN starts; gripper-open decision = tick 13014). Negative = before the decision; positive = after.

| Event (signal) | Tick | Lead (−) / lag (+) vs release decision 12863 | vs gripper open 13014 | Basis |
|---|---|---|---|---|
| Beaker bottom within 0.05 mm of plate top (geometry) | 12698 | **−165** | −316 | calculated from beaker_z and plate top 1.028 m; no contact log |
| Arm pad vertical force peak (estimate, rises ~0.4 N) | 12727 | −136 | −287 | calculated (`arm_pad_Fz_N`; depends on the feedforward, see caveat) |
| Hand peak above reference (+3.02 mm) | 12774 | **−89** | −240 | shown by logged pad and reference |
| Still verdict: a 5-tick ON stretch (12893 to 12897), then OFF until | 12986 | +123 | −28 | still verdict (speed based) |
| Largest hand acceleration −0.755 m/s² | 12911 | **+48** | −103 | calculated from pad speed |
| Accel stop −0.587 m/s² (beaker 0.2 mm above plate) | 12912 | +49 | −102 | calculated; contact not logged |
| Held mass first below 0.2 kg (ramp, scripted) | 12902 | +39 | −112 | shown (scripted ramp) |
| Still verdict ON (joints and pad still) | 12986 | +123 | −28 | still verdict (speed based) |
| Held mass reaches 0 (ramp finished) | 13012 | +149 | −2 | shown (scripted ramp) |
| **Gripper command opens** | 13014 | **+151** | 0 | shown (`grip_applied_m`, scripted) |
| Squeeze fails (closing only, M251) | 13014 | +151 | 0 | shown (force, M251 rule) |
| Holding turns OFF (M251 rule) | 13014 | +151 | 0 | replay of the verdict code |
| Gripper accel spike +0.388 m/s² | 13014 | +151 | 0 | calculated |
| Finger leaves the position range (above 42 mm) | 13018 | +155 | +4 | shown (finger, grip range 0.005 to 0.042 m) |
| Finger reaches open (43.8 mm) | 13023 | +160 | +9 | shown |

**What changes visibly when the beaker touches the plate.** The beaker's own height is the clearest signal, but only in the geometry (the plate top is from the scene file). The arm's vertical force estimate rises by about 0.4 N (1.96 → 2.44 N) in the 40 ticks around 12690 to 12727, then drops as the held mass is ramped down. **Caveat (shown by the code):** `arm_pad_Fz_N` is computed from the servo forces minus the model's bias, and the servo forces include the scripted feedforward (`src/gravity_ff.py`). So it is **not an independent weight sensor**: it falls with the scripted ramp (2.13 N at 12863 to −0.1 N at 13010). Hand acceleration shows the touchdown-like events (12911 and 12912), but those happen during the ramp-out, when the hand is also being moved by the ramp. The gripper force changes only at the open command (13014). The holding and squeeze flags change at the same tick as the gripper command, not at touchdown.

### 3.3 What the hand does across the release (where the bounce risk sits)

Three excursions, all before the gripper opens:

1. **DOWN overshoot (12690 to 12840).** The pad goes 1.6 mm below its reference (12690), then rises to **+3.0 mm above it** (12774). The beaker's bottom reaches the plate at 12698 (calculated) and the beaker rises 3.8 mm with the hand (1028.0 → 1031.8 mm). The hand speed peaks at 6.2 mm/s (12727). **Cause: NOT DETERMINED** (the sag trim `C_z` moves from −1.4 to −2.8 mm across the same window, `liftoff_ticks.csv` column `C_p_z`, so the integral correction is involved, but the solver's and the servo's part is not separated).
2. **Ramp-out sag and stop (12863 to 12912).** The hand sags 1.2 mm (1083.9 → 1082.7 mm) and reaches 11.7 mm/s at 12908. It then stops abruptly: −0.59 m/s² at 12912 and −0.76 m/s² at 12911, the largest in the seam. The beaker is 0.2 mm above the plate at that tick. **Cause: NOT DETERMINED.** The sag freeze is on during the ramp (`frozen` flag = 1), so the correction does not follow it. The likely link is the feedforward held term falling as the mass ramps (`gravity_ff.py` lines 118 to 124), but that is a guess until a run that removes the ramp or logs the held torque confirms it.
3. **Gripper opening (13014 to 13018).** Accelerations of +0.39 and −0.33 m/s² (calculated), with the hand nearly still (pad z changes 0.02 mm). Smaller than (2).

**Bounce risk** sits in (1) and (2), which come **before** the release decision (1) and during the ramp it starts (2). A set-down detector that only looked at the gripper opening would be too late for both.

### 3.4 Candidate set-down signals (not chosen, not built)

| Candidate | What it shows in this log | What it would need |
|---|---|---|
| Beaker height (from the simulator) | Stops falling near 1028 mm at 12700; touches the plate level at 1028.0 mm; the hand then rises | Simulator truth only today (`beaker_z_m`, validation). A real robot would need a vision or height estimate |
| Hand acceleration (from positions) | Largest stop at 12911 (−0.76) and 12912 (−0.59), during the ramp-out; spike +0.39 at 13014 | Already computable from the logged positions (stage 1b). Would need its own threshold (not decided) |
| Arm pad vertical force (torque minus model bias) | Rises 0.4 N near 12690 to 12727, then falls with the scripted ramp | Already logged; not independent of the feedforward (see caveat). Would need the feedforward separated out |
| Hand speed (from positions) | Peaks 11.7 mm/s at 12908; 6.2 mm/s at 12727 | Already logged; needs a threshold (not decided) |
| Gripper command force (actuator) | −16 N while the hand holds; flips to +3.8 N only at the open command; no change at touchdown | Already logged (stage 1b). Says nothing about the beaker touching the plate |
| Finger position | Constant (37.5 mm) until the open command | Already logged; flat through the seam, so no set-down signal |
| Holding verdict (M251 rule) | Turns OFF at the open command (13014), not at touchdown | Already computed (log only). Cannot be used for set-down: it stays on through the touchdown |
| Still verdict | Off through the seam until 12986 | Already computed (log only) |
| Contact with the plate (simulator contacts) | Not logged. The bench-contact column covers only the bench, not the hotplate | Would need a contact log for the hotplate geoms (a logging change and a new run) |
| Weight reading (lift-off estimate) | Fixed at 0.2245 kg through DOWN; ramps to 0 from the scripted release | Only the scripted estimate exists. A sensed weight would need the handoff reading (2.15), which is not active |

## Part 4: observations, NEEDS DECISION, NOT DETERMINED

### NEEDS DECISION (for David; not decided here)

1. **Primitive boundaries.** Which phases belong to which primitive. This report's grouping is a guess (Part 1). In particular: is the DOWN→OPEN seam inside "put on hotplate" (assumed here), and is the lift (WEIGH→UP) inside "pick" or a separate "lift" primitive?
2. **What starts the release.** Today the ramp-out starts at the script's step boundary (tick 12863), 165 ticks after the calculated touchdown. Should the release start from a sensed set-down instead, and what sensed signal should decide it?
3. **Whether the sag freeze stays during the ramp-out**, and whether the held-mass ramp should be time-based (1.0 s) or follow the sensed load.
4. **Whether the lift-off weighing stays inside the pick**, and whether the handoff re-weigh (2.15, dead code today) should be switched on.
5. **The oracle (simulator truth) checks.** The default checker is the oracle, and the drop monitor's reason uses it. Which checks are allowed to use simulator truth in a VLA run?
6. **The thresholds that govern these helpers** are still marked PENDING in `src/task_config.py`: the 2 mm set-down gap (line 420), the 3 mm and 2° waypoint tolerances (line 434), the 0.02 rad/s settle speed (line 435), the 0.3 mm lift-off rise (line 441), the 0.002 m/s stall speed (line 445) and the 3 mm open margin (line 447). David decides these.
7. **The bounce at DOWN (12698 to 12840).** Whether to slow the approach, change the sag gain, or accept it. The choice is David's.

### NOT DETERMINED (what would answer it)

1. **Whether the beaker actually touches the plate** at 12698 and at 12912. The beaker bottom is 0.03 mm and 0.2 mm above the plate in the logged data, which is inside the geometry tolerance. **Run:** a 200 g run with the hotplate's contacts logged (a logging addition; no behaviour change), compared with the existing trace.
2. **The cause of the DOWN overshoot (+3.0 mm at 12774).** **Run:** a 200 g run that logs the solver's reference and the servo's command per tick, and the sag correction separately, over ticks 12680 to 12840.
3. **The cause of the 1.2 mm sag during the ramp-out.** **Run:** a 200 g run with the held-mass feedforward term held at its start value during the ramp (a test switch), compared with the baseline.
4. **What the real OpenArm gripper reports** (motor current, effort, or position) and whether it would show the touchdown. Unknown from the repo; needs hardware.
5. **Whether a bare policy (no scripted steps) works** through this seam. **Run:** a policy-driven run with the scripted steps removed; not done.
6. **Whether the handoff re-weigh would change anything** if the step were planned. **Run:** a 200 g run with the "held" step added (a planning change; not built here).
7. **Whether the closing-on-nothing case stops at 2.75 mm** in the full scene (the 2.75 mm is from the robot-only model; see the stage 1b report). **Run:** the closing-on-nothing run (not possible without a layout change; see the stage 1b report).

### Observations (labelled as observations, not recommendations)

- **Easiest to move to sensed signals (observation):** the gripper-open wait (2.8). Its finish test already uses sensed finger travel and speed; only the ramp wait and the open command are scripted. The holding and drop checks (2.10) also already decide on sensed travel; only their reasons use simulator truth. The waypoint completion test (2.11) is already sensed (measured pad pose and joint speed); only its timing and targets are scripted.
- **Hardest to move (observation):** the held-mass chain (2.2, 2.4 to 2.7). It is set only by scripted steps, its freeze and reset are tied to two other scripted events (the ramp and the gripper-open event), and the weight estimate depends on the model (`held_pad_z_per_kg`). The sag trim (2.4) cannot be moved on its own, because its freeze follows the held-mass ramp and its reset follows the gripper event. Under a bare policy the held term is zero, so the beaker's weight is not compensated at all.
- **Other bare-policy blockers (observation):** the primitive targets come from the simulator's world model (2.11, `plan_place` lines 226 to 227); `world.held_by` in recovery is simulator truth (2.14); the phase-start velocity ramp (2.12) and the skill-start resets (2.13) assume the script's step boundaries.

## Files

- Created: `docs/reports/helper_inventory_seam_20261104.md` (this report, uncommitted).
- Scratch, outside the repo: `/tmp/holdchange_20261104/seam.py` (the alignment and seam script), `/tmp/holdchange_20261104/seam.npz` (per-tick seam arrays), `/tmp/holdchange_20261104/seam_tables.md` (the by-tick table generator output), `/tmp/holdchange_20261104/m251_left.npz` (verdicts under the M251 rule, from the earlier replay).
- No code, config, baseline or decision-log file was changed.
