# Held-object gravity compensation (M196), ramp 1.0 s (M195), pedestal 0.15 m (M197), NAV: PEDESTAL label

Written 2026-10-28 for David. Not pushed, no video re-rendered. The full suite result is added at the end when it finishes (section "Full suite").
Code: commits since c67b2e4 (WIP commits). Episodes (moved layout, bridge ON, duration fix ON, ramps ON): `data/demo_record/comp{1,2,3,3b,2off}_20261028_135510`; measurements `data/arm_motion/comp/{base,item1,item2,item3,item3b}` (`comp_metrics.json`, `metrics.md`), scripts `experiments/comp_episode.py`, `comp_measure.py`, `comp_gap_report.py`, `comp_jerk_table.py`. "base" = the earlier smooth episode `smoothmj_20261028_031803` (compensation OFF, ramp 0.5625 s, pedestal 0.10), re-measured with the same scripts (it reproduces the earlier 2.92 deg, 18.8 mm, 0.34 s).

## Short version
1. Compensation works as specified and the episode **succeeds with ramp 0.5625 s** (item 1) at 0.10 m and at 0.15 m. The reading is **200 g** (205 g at 0.15 m), not 158 g: the task beaker is 100 g glass + 100 g liquid, 150 g was the test variant.
2. **With ramp 1.0 s and compensation ON the episode FAILS**, at 0.10 m and at 0.15 m, in the place's DOWN step (not reached, 3.9 mm / 3.0 mm from the waypoint, tolerance 3 mm; the beaker stands on the hotplate). Ramp 1.0 s with compensation OFF succeeds (`comp2off`); compensation ON with 0.5625 s succeeds (`comp1`, `comp3b`). So the failure comes from the two together. The config default is 1.0 s as you decided; I did not tune anything. **Your call** (options at the end).
3. The "0.09 to 0.11 rad gap" does not go away with compensation, and cannot: it is (ctrl - measured), which at rest is always (torque the joint must give) / kp. What compensation changes is the skill's own command minus measured (the sag). Both are in (b).
4. Compensation is a torque STEP at the start (and at the end, when the gripper opens): carry-boundary jerk 178 -> 1186 rad/s^3, PLACE BACK boundary 54 -> 982. Ramping the compensation in and out would remove it (not built; your decision).
5. Pedestal 0.15 m: elbow clearance 180 / 176 mm at the place, as in the sweep; episode succeeds at ramp 0.5625 s.
6. Panel label done and checked on a rendered frame.

## Item 1: compensation ON, ramp 0.5625 s, pedestal 0.10 m (episode `comp1`)
Build (M196). `src/gravity_ff.py`: `HELD_MASS` registry, `set_held_mass`; `torques()` adds `m g J_z` of the pad centre's translational Jacobian (the sign of your `J^T [0,0,-m g]` is the generalized gravity force; the motor torque that holds it is its negative, as for the arm's own links; the measured weight uses the same convention). `src/mpvi_modules.py`: the pick's "held" step (CHECK) waits for a still arm (0.005 rad/s over 0.30 s, at most 1.0 s) then runs `weight_estimate.hand_load`; the mass is set on that tick and cleared on the tick after the gripper has opened and at every pick start. `src/control_loop.py` / `run_record.py` / `replay.py`: the compensated mass is recorded per tick (`heldmass`) so replays are bit-identical (verified: all replays ok). Setting `PICK_PLACE_CFG["held_gravity_comp"]` (True). Deviation: it starts on the reading's own tick (the last tick of the pick), not the tick after, because the module is not stepped between the pick and the next skill.

(a) **Succeeds**, 4 of 4 steps. Reading 200.4 g (unexplained torque share 0.14).

(b) Gap at the PICK UP to PLACE OVER boundary, left arm, rad (joints 1, 4, 7):
| | baseline (no comp) | comp ON |
|---|---|---|
| ctrl - measured (what earlier reports called the 0.09 to 0.11 gap), boundary | -0.105, +0.075, -0.091 (max 0.105) | -0.119, +0.087, -0.126 (max 0.126) |
| skill command (before the feedforward) - measured, boundary | -0.012, +0.012, -0.038 (max 0.038) | -0.012, +0.012, -0.036 (max 0.036) |
| same, largest in the first 1.0 s of PLACE OVER | max 0.054 (J7) | max 0.037 |
| same, 1.0 to 2.0 s of PLACE OVER | max 0.057 | max 0.030 |
Reading: ctrl - measured is (torque needed) / kp whatever the feedforward, so it stays 0.09 to 0.13 (it even grows by the held torque / kp at the boundary tick, where the compensation starts). The sag, the skill command minus measured, is already small at the boundary (the sag correction carries it) and is smaller afterwards (0.037 against 0.054 in the first second). The boundary value is unchanged because the compensation starts on that very tick. Per-joint values: `python experiments/comp_gap_report.py data/arm_motion/comp/base data/arm_motion/comp/item1`.

(c) PLACE OVER: peak beaker tilt in the world **3.40 deg** (baseline 2.92), hand dip **17.5 mm** (18.8), beaker slide in the hand 2.8 mm (3.1).

(d) First 0.5625 s of PLACE OVER (85 ticks): ticks with any joint at the ramp cap **56** (0.39 s to the last one; baseline 47, 0.33 s). Not fewer: the skill-start snap is the same size (the sag at the boundary is unchanged) and the compensation torque step adds a little motion.

(e) Peak actual joint jerk, 150 Hz, rad/s^3, boundary into the phase / mid-phase (baseline | comp ON):
| phase | baseline | comp ON |
|---|---|---|
| PICK OVER | 1125 / 71.7 | 1125 / 71.7 |
| PICK DOWN | 17.0 / 8.2 | 17.0 / 8.2 |
| PICK IN | 35.3 / 38.3 | 35.3 / 38.3 |
| PICK CLOSE | 12068 / 5.6 | 12068 / 5.6 |
| PICK UP | 310 / 99.1 | 310 / 99.1 |
| PLACE OVER (carry) | 178 / 96.9 | **1186** / 109 |
| PLACE DOWN | 74.9 / 2776 | 121 / 1922 (the contact) |
| PLACE OPEN | 528 / 73.8 | 461 / 80.4 |
| PLACE BACK | 54.3 / 42.2 | **982** / 127 |
| PLACE HOME | 20.4 / 38.0 | 21.0 / 35.9 |
The two large entries are the compensation switching on (the last tick of the pick; the arm moves 0.16 rad/s in joint 7 within 0.05 s) and off (one tick after OPEN). Everything before the pick's end is identical (same inputs).

(f) Place accuracy: beaker base 0.52 mm from the hotplate centre (offset -0.03, +0.52 mm; baseline 0.65 mm), height 1.0279 m, tilt 0.015 deg.

## Item 2: ramp 1.0 s (M195 updated), compensation ON, pedestal 0.10 m (episode `comp2`)
`phase_start_ramp_s` 1.0 (peak acceleration 1.875 * 0.5 / 1.0 = 0.9375 rad/s^2), comment and M195 updated.

(a) **PLAN_FAILED.** Place DOWN: "not reached after 9.0 s: 3.9 mm and 0.1 deg from the waypoint"; the retry (arm home) failed too (BACK, 11.2 mm). Pick, carry and the first two steps are fine.
What the recording shows (replay `data/arm_motion/comp/item2`): during PLACE OVER the beaker slides 2.2 mm further down in the grip than in the 0.5625 s run (pad-to-beaker height 59.25 mm at the end of OVER, 57.0 mm in the baseline and in `comp1`; the slide in the hand over the phase 4.7 mm against 2.8); the dip is 26.9 mm (17.5); in DOWN the beaker reaches the plate (z 1.0282 m) while the pad is still 3.9 mm from its target, and stays there with the arm at rest (all joint speeds below 0.002 rad/s for the last 8 s). So the planned set-down height, computed from the pad-to-beaker offset at the start of the place, no longer matches. I have not found why the slide is larger with the longer ramp and compensation together.
Controls: ramp 1.0 s with compensation OFF (`comp2off`): SUCCESS; compensation ON with 0.5625 s (`comp1`): SUCCESS.

(b) Gap (as item 1): boundary ctrl - measured -0.119 / +0.087 / -0.126 (max 0.126); skill command - measured: boundary max 0.036, first 1.0 s max 0.047, 1 to 2 s max 0.029.
(c) tilt 3.71 deg, dip 26.9 mm, slide 4.7 mm (baseline 2.92, 18.8, 3.1).
(d) ticks with any joint at the cap in the 150-tick ramp: 121 (0.83 s to the last one).
(e) jerk, boundary into / mid (rad/s^3): PICK OVER 1125 / 453, DOWN 19.0 / 8.3, IN 28.1 / 37.2, CLOSE 12055 / 4.2, UP 403 / 91.8, PLACE OVER 1190 / 320, PLACE DOWN 90.5 / 1939; the rest of the place did not run. The longer ramp does not remove the boundary jerk (it is the compensation step, 1190) and the mid-phase jerk rises (PLACE OVER 320 against 109, PICK OVER 453 against 71.7: the cap now acts for 1.0 s inside moves of 1.7 to 2.7 s).
(f) Place accuracy: not applicable, the place failed (the beaker ended 51.7 mm from the centre after the failed set-down and recovery).

## Item 3: pedestal 0.15 m (M197)
`BENCH_STANCE_CFG["pedestal_override_m"] = 0.15` (None gives the selection's own height; the selection and its full check still ran at their own height, M197).
(a) With the new defaults (ramp 1.0 s, compensation ON, 0.15 m, episode `comp3`): **PLAN_FAILED** at the same place DOWN (3.0 mm from the waypoint); reading 206 g. With ramp 0.5625 s and compensation ON (`comp3b`): **SUCCESS**. All numbers below are from `comp3b`.
(b) Elbow clearance from the bench (left elbow, link 4 collision geoms to the bench top, minimum over the phase, mm): PICK OVER **304**, PICK DOWN **293**, PLACE OVER **180**, PLACE DOWN **176** (sweep, 0.15 m: 180 / 176 at the place; 0.10 m: 155 / 153).
(c) PLACE OVER: peak tilt **3.42 deg**, hand dip **18.6 mm**, slide 3.0 mm (0.10 m with compensation: 3.40, 17.5, 2.8).
(d) Jerk, boundary into / mid (rad/s^3): PICK OVER 2117 / 112, PICK DOWN 6.9 / 9.8, PICK IN 24.8 / 52.8, PICK CLOSE 10313 / 38.1, PICK UP 197 / 73.9, PLACE OVER 1160 / 130, PLACE DOWN 107 / 1977 (contact), PLACE OPEN 564 / 55.4, PLACE BACK 1075 / 143, PLACE HOME 31.7 / 45.6. PICK OVER (the staging end) is higher than at 0.10 m (2117 against 1125) and its mid-phase is higher (112 against 72); everything else is as at 0.10 m, plus the two compensation steps.
(e) Place accuracy: 0.49 mm from the hotplate centre (offset -0.23, +0.44 mm), height 1.0279 m, tilt 0.019 deg.
(f) Joint margins at the place (left arm, minimum over the phase; the solver's margin is 10 deg): PLACE OVER joint 2 (shoulder) 12.8 deg, PLACE DOWN joint 2 **12.0 deg** (the sweep's 12.0); the nearest other joint at the place is joint 4 on the right arm at home, 13.9 deg. Reach: unchanged from the sweep (14 mm below the place height).

## Item 4: panel label for NAV: PEDESTAL (done)
`src/viewer_layout.py`: the "doing" row and the NOW row say "Adjusting pedestal height" during NAV: PEDESTAL (the table entry was "Setting torso height"); when the phase is DRIVE the NOW row is the step text again. Checked on stills of the rendered `comp3b` recording (`data/arm_motion/comp/panel_item3b/stills_panel_1_4_NAV_PEDESTAL.png`: NOW "Adjusting pedestal height", doing "Adjusting pedestal height", code "NAV: PEDESTAL [running]", nothing clipped; `..._NAV_DRIVE.png`: NOW "Navigate to the beaker", doing "Driving to the beaker").

## Open points for David
- **Ramp 1.0 s with compensation fails the moved-layout episode** (items 2 and 3 as defaulted). Options: (1) keep 0.5625 s (everything succeeds); (2) keep 1.0 s and ramp the compensation in and out (also removes the 1186 and 982 jerk spikes); (3) I find out why the beaker slides 2 mm more with 1.0 s (the set-down height is planned from the offset at the place's start). I did not change anything to make it pass.
- The compensation step (jerk 1186 at the carry boundary, 982 at the release) is a consequence of "starts at the reading, ends after OPEN" as specified; a smooth in/out ramp of the compensation is a design choice for you.
- The earlier "gap 0.09 to 0.11 rad" is not a measure of the sag (it stays 0.09 to 0.13 by construction); the sag is skill command minus measured (0.037 against 0.054 in the first second of PLACE OVER).
- Tilt in PLACE OVER is higher with compensation (3.4 against 2.9 deg), the dip lower (17.5 against 18.8 mm).
- The pedestal override is applied after the stance selection and its full check; the plan is not re-checked at 0.15 m.

## Full suite
(running; added at its end)
