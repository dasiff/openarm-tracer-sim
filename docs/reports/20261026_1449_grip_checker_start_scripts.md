# Grip, the 300 g failure, the weight model; the checker's starting guess; old scripts (2026-10-26)

Written for: David, deciding about the grip, the place at heavy loads, the weight model, and the checker's start. Not pushed. Everything in steps 1 and 2 is measurement only: no change to the robot's behavior and no default changed.

Note: two messages came; the first asked for step 1 only (grip, the 300 g failure, the weight model), the second for the checker start (its "step 2"), the design plan item, the old scripts and this report. I did them in that order.

## Summary

**Passed / done**
- Step 1 runs (normal layout, 150 / 200 / 300 g, one episode each) all ran and logged everything for 1a to 1c. 150 and 200 g succeed; 300 g fails (as before).
- Checker start: in both layouts the neighbourhood start gives the SAME answer as today's at every one of the 727 and 693 stops, the same chosen spot and the same angle; today's checker used no random rescue at all. Default unchanged.
- Design plan: the 300 g drop is added to the safe-return item. Old scripts: patched and checked (details below).

**What the measurements say (my interpretations are marked)**
- **The beaker does not slide in the grip, it TILTS.** Along the beaker's axis it moves at most 0.8 mm (1.6 mm at 300 g) and sideways at most 1.0 mm (2.1 mm). Its axis tilts 12 to 13 degrees at 150 and 200 g and up to 26 degrees at 300 g during the lift. The grip force is as commanded (8 N per finger, 7.8 to 8.3 N measured). The two finger contacts face each other only to within 15 degrees (165 degrees round the axis; their line misses the axis by 4.5 mm), and sit about 50 mm up the beaker, about 24 mm above its centre of mass. Interpretation: not too little force; the pinch is slightly off-centre, so the beaker swings like a pendulum about the pinch line (4.5 mm off-axis over 24 mm below gives about 11 degrees).
- **The 300 g failure is that tilt plus the sag correction's cap.** The tilted beaker's rim touches the plate before the hand reaches DOWN (first contact 8.6 mm above DOWN at 300 g, tilt 19 degrees); the sag correction then keeps pushing and winds up to both its caps (4 cm and 0.1 rad) while the plate holds the beaker, leaving 4.4 mm. At 150 and 200 g the same contact happens (5.5 and 4.8 mm early) but the hand ends 2.9 and 2.7 mm from DOWN, just inside the 3 mm tolerance. Joint limits and motor torque are not involved. The recovery then drops the beaker (details in 1b).
- **The four-number weight model does not work at this signal level.** Fitting the weight and the balance point together makes the weight worse (195.7 / 233.7 g for 150 / 200 g, +30 % and +17 %, against +6 % and +2 % with the one-number fit) and puts the balance point 50 mm from the pad centre horizontally when the true one is 4 mm; the unexplained torque barely changes (14 % against 15 %). The vertical part of the balance point cannot be found at all from one pose (explained in 1c). So the one-number fit stays the better estimate.

**Failed / not good**
- `arm_hold_diagnosis.py` fails (KeyError 'MiddleBottom': it looks for a pedestal joint that was renamed long ago, so it was already broken before my patch).
- The 300 g strict-still handoff reading is again impossible (the arm creeps); only a loose reading exists.

**New**
- `experiments/grip_failure_weight_model.py`, `src/weight_estimate.hand_load_with_balance_point`, `FULLCHECK_CFG["start_mode"]` (temporary; default "today"), `FullCheck.start_log`, `experiments/checker_start_compare.py` and `checker_start_report.py`, row M187.

## Step 1: grip, the 300 g failure, the weight model

Runs: `experiments/grip_failure_weight_model.py --liquid 0.05 | 0.10 | 0.20` (glass 0.10 kg, masses set in the harness through `load_task(beaker_cfg=...)`), logs `data/logs/gf_20261026_140242_*`, raw results `data/grip_failure/20261026_140242/liquid_*/result.json` (10 Hz series included). The beaker's own axis is its body z axis; the pad centre's place on the beaker is tracked in the beaker's frame from the start of CLOSE.

### 1a. The grip
Per phase (closing = the CLOSE step; lift = UP and the check; carry = the place's OVER and DOWN):

| total | closing: tilt / axial / sideways | lift: tilt max (end) / axial / sideways | carry: tilt max (end) / axial / sideways |
|---|---|---|---|
| 150 g | 0.8 deg / 0.1 mm / 0.8 mm | 12.9 (12.8) deg / 0.9 mm / 1.0 mm | 13.9 (9.3) deg / 1.0 mm / 1.0 mm |
| 200 g | 0.7 deg / 0.0 mm / 0.8 mm | 11.9 (11.7) deg / 0.8 mm / 1.0 mm | 12.9 (7.6) deg / 0.9 mm / 1.0 mm |
| 300 g | 0.4 deg / 0.0 mm / 0.8 mm | **26.3 (16.9)** deg / 1.6 mm / 2.1 mm | 19.2 (9.5) deg / 2.5 mm / 1.8 mm |

- **When:** closing: nothing (under 1 degree, 0.8 mm sideways); the lift: the tilt appears (the beaker swings as it leaves the table and the arm accelerates); the carry: it stays 8 to 19 degrees and has not come back to upright when the beaker is lowered (7.6 to 9.5 degrees at the end of DOWN). The slide is at most 2.5 mm in any phase and mass.
- **Grip force used:** cap 8 N per finger (GRIP_FORCE_TABLE glass_beaker); the gripper actuator applies 16 N (two fingers). Measured contact normal force per finger at the end of closing 7.8 / 8.1 to 8.2 N, at the end of the lift 8.0 / 8.0 N, at the end of DOWN 8.0 / 8.0 N (7.6 / 8.4 N at 300 g). During closing there is a transient impact of up to 18.4 N (left finger) and 13.6 N (right).
- **Where each finger touches:** at 53 mm (left) and 50 mm (right) above the beaker's bottom (the beaker is about 100 mm tall; its centre of mass is 19 to 24 mm below the pad centre, i.e. about 25 to 30 mm above the bottom), both 35.3 mm from the axis (the outside wall). **Facing:** the two contacts are 165.4 degrees apart seen from the axis (180 would be opposite), the chord between them misses the axis by 4.5 mm, and their normals are 179.6 degrees apart (they push in opposite directions). At 300 g at the end of DOWN 169 degrees and 3.3 mm.
- **Interpretation (mine, not tested):** not a force too low (8 N each, steady, nothing slides). The cause is the pinch line being 4.5 mm off the beaker's axis and about 20 to 25 mm above its centre of mass: the beaker then rests tilted like a pendulum, tan(angle) about 4.5 / 24, which gives about 11 degrees (measured 12 to 13 at 150 and 200 g); at 300 g it swings up to 26 degrees on the lift and settles at 17. The off-centre pinch is consistent with the "4.7 mm off-axis" stop the documents already mention for the hand (grip_force_sweep / gripper_force_test: the hand stops a few millimetres off-axis).

### 1b. The 300 g failure
- **Did the beaker touch the hotplate before the hand got to DOWN? Yes.** First contact of the beaker's bottom with the plate at t = 63.46 s, while the hand was **8.6 mm (norm 8.8 mm) from DOWN** (vertical 8.6 mm above it); the beaker's origin was 9.4 mm above the plate then, because the beaker is tilted about 19 degrees (its lower rim reaches down 35 mm x sin 19 degrees = 11 mm). The same happens at the lighter masses: 5.5 mm (150 g) and 4.7 mm (200 g) before DOWN.
- **Sag and the sag correction's cap: yes.** At 300 g the correction has wound up to both caps (4.0 cm and 0.100 rad) from t = 67.3 s on and stays there; the hand stays 4.4 to 4.6 mm from DOWN until the 9 s timeout (4.4 mm, 0.2 degrees; tolerance 3 mm). At 200 g the correction is only 4.3 mm / 0.016 rad and the final error 2.7 mm; at 150 g 18.8 mm / 0.061 rad and 2.9 mm. The correction integrates the error the contact causes, so it is a symptom, not the cause.
- **Joint limits: no.** Smallest margin to a joint limit 0.31 to 0.33 rad (joint 2), to the solver's own bound (limit minus 10 degrees) 0.14 to 0.15 rad. **Motor torque: no**, at most 18 % of a cap (joint 1).
- **The recovery's move home dropped the beaker.** The recovery's first step (BACK, meant for an empty hand: back off 5 cm) starts while the beaker is still pressed on the plate: the beaker tilts up to 53 degrees in the hand (t = 73 s) and is dragged; it settles at about 45 degrees for 6 s, then the arm is commanded home. The beaker left the fingers at **t = 81.65 s**, in the HOME step, with the **hand moving at 0.25 m/s** (peak in the recovery 0.26 m/s; arm joints at 0.65 rad/s, the 0.5 rad/s limit of the plan's speed plus the joint-space move) and the finger forces going from 8.7 / 7.3 N to 0 within one sample; the fingers then closed on nothing (opening 37 mm to 2.7 mm). Interpretation: a beaker already tilted 8 to 45 degrees in the grip is thrown out by the acceleration of the return home; the design plan's safe-return item has this noted.

### 1c. The weight model at the pick handoff
Method: `hand_load_with_balance_point`: extra torque = applied motor torque minus the arm's own gravity torque; fit a straight-down weight W at a point displaced from the pad centre by a. The torque this gives is `W Jp_z - m^T Jr` with `m = W (z x a)`: it depends on W and the two HORIZONTAL components of a only. **The vertical part of the balance point cannot be determined from the torques at this pose (a force acts along its own line of action wherever it sits on that line), so I fit three numbers, not four**, and report the vertical offset as unobservable (setting it to 0 to express the horizontal offset in the hand's frame). The estimator reads only angles, the applied torque and the arm's own gravity torque; the true centre of mass (`xipos` of the beaker relative to the pad centre) and the mass are read from the sim only for this table.

| total (true) | one-number fit (weight) | three-number fit (weight) | estimated horizontal offset of the balance point (world x, y; mm) | true horizontal offset (mm) | unexplained torque: one-number / three-number |
|---|---|---|---|---|---|
| 150 g, strict still | 159.3 g (+6 %) | 195.7 g (+30 %) | (-22.9, +75.3) | (-0.2, -3.6) | 17 % / 15 % |
| 150 g, loose still | 157.1 g (+5 %) | 185.9 g (+24 %) | (-21.9, +62.6) | (-0.2, -3.6) | 17 % / 15 % |
| 200 g, strict still | 204.1 g (+2 %) | 233.7 g (+17 %) | (-16.9, +51.2) | (-0.2, -3.6) | 15 % / 14 % |
| 200 g, loose still | 201.5 g (+1 %) | 222.8 g (+11 %) | (-15.5, +38.5) | (-0.2, -3.6) | 15 % / 14 % |
| 300 g, loose still (no strict reading: arm never still) | 294.8 g (-2 %) | 312.0 g (+4 %) | (-8.0, +22.2) | (-0.2, -3.7) | 10 % / 10 % |

The true centre of mass is 19 to 24 mm BELOW the pad centre (vertical, unobservable here) and 0.2 mm / 3.6 mm off it horizontally. In the hand's frame the estimate is (pad x, y, z) = (16.7, 0.3, -51.3) mm for 200 g strict against a true (0.2, 23.6 vertical, 3.8) mm: the vertical axis of the hand frame is its y axis, and its estimated value 0.3 mm is just the unobservable part set to 0; the horizontal part is wrong by 16.7 and 51 mm.
- **Reading:** the extra freedom does not reduce the unexplained torque (15 to 14 % at 200 g), so the offsets and the extra weight are fitting something the model cannot describe (the position servos' steady error, the small mismatch of the gravity model, joint-1 and wrist coupling), not the beaker's centre of mass. The fit leaves the same 10 to 17 % unexplained either way. The one-number fit is within 2 to 6 % at 150 and 200 g and stays the better estimate; the balance point needs either a second pose (hand tilted) or a better model of the residual torque before it can be found. Interpretation, not tested.
- **Empty-hand reading after the place** (the arm still at the zombie pose): one-number fit +2.3 g (200 g run) and +2.5 g (150 g run); three-number fit -12.5 and -11.8 g (with a meaningless offset, since there is no load); the unexplained share is 98 to 100 % in both (the extra torque is a few tenths of a newton metre, a horizontal 0.6 N force-equivalent noise floor).
- Timing of the reading: strict still at the end of the pick as before (the arm was still when the module declared the pick done); at 300 g the arm creeps at 0.01 to 0.04 rad/s when the place starts, so there is no strict reading.

## Step 2 (second message): the checker's starting guess
Temporary setting `FULLCHECK_CFG["start_mode"]`: "today" (default, unchanged) or "neighbourhood" (first stop of each path from the zombie pose, later stops from the previous stop's answer, no other start, no random rescue in the pass/fail decision; random restarts run on the side and are only logged as "rescuable"). The error stances and the other hand turns each solve their own path from the zombie in the neighbourhood mode (today they start from the nominal stance's answer). The legacy grasp-bin path (no "pose" key) is untouched. Run: the full stance selection both ways, normal and moved layout (`experiments/checker_start_compare.py`, `checker_start_report.py`; logs `data/logs/cs_20261026_142253_*`, results `data/checker_start/20261026_142253/*/result.json`).

| | normal layout | moved layout |
|---|---|---|
| chosen spot, today / neighbourhood | (0.755, 1.145), pedestal 0.10 / same | (0.635, 1.135), pedestal 0.10 / same |
| place angle, today / neighbourhood | -25 / -25 degrees (window -50 to 0, 11 turns, both) | -25 / -25 degrees (window -50 to -5, 10 turns, both) |
| full-check calls | 1 (the first-ranked spot passes), both | 1, both |
| stops solved | 727 | 693 |
| today: passed from a warm start / forced random / **random rescue** / failed | 658 / 8 / **0** / 61 | 620 / 11 / **0** / 62 |
| stops whose answer differs between the two ways | **0** | **0** |
| neighbourhood: failed stops a random start could reach | 0 | 0 |
| selection time, today / neighbourhood | 49.8 / 57.4 s | 45.4 / 47.0 s |

- **How often today's checker used a random rescue:** never, in either layout (a rescue = the starts tried first all failed and a random start passed). It used a random start only where none other exists: 8 (normal) and 11 (moved) first stops of a path (grasp, over, standoff, lift, back, carry, setdown paths; at the chosen spot's nominal stance and angle 5 of them). None of those matters for the answer: in the neighbourhood mode the same stops start from the zombie pose and pass identically. So no rescued stop is on the chosen spot's path.
- **Every stop whose answer differs:** none. The 61 and 62 failed stops are the hand turns outside the window (the same stops fail in both ways, and a random start cannot reach them either).
- **Does the chosen spot or angle change:** no.
- **Caveat:** only the first-ranked spot is checked in these scenes (it passes), so the comparison covers the passing path and the full 72-turn window search at that spot, not a scene where several spots fail. On these scenes the change would cost nothing; the loophole (a random rescue deciding a pass) did not occur. Row M187 and the design plan item hold this, NOT DECIDED. The default is unchanged.

## Step 3 (second message): the design plan
The safe-return item now says (David, NOT DECIDED): a recovery move home at 300 g dropped the beaker, so when holding something the move home must be slow, and setting the object down first is an option to consider.

## Step 4 (second message): old scripts that write `data.ctrl` themselves
Added the gravity helper (`gravity_ff.for_tests(model).apply(ctrl, data)`; for the pour scripts a small `hold_ctrl`):
- `experiments/arm_hold_drive_test.py` (the NEW model only; ran: both runs complete; NEW: worst joint deviation 0.56 rad in the turn, 0.054 rad from target at the end), `experiments/branch_jump_demo.py` (both its sim loops; ran: completes, wrist gap 52 mm at the jump and 0.2 mm at the end), `experiments/arm_hold_diagnosis.py` (HoldSim and `static_checks`; **fails with KeyError 'MiddleBottom' before and after, a stale pedestal joint name**), `experiments/pour_helpers.py` and `experiments/pour_timing_test.py` (ran; `pour_scene_test` that uses the helpers 21/21), `experiments/workspace_sweep.py` and `experiments/run_with_dummy_policy.py` (syntax-checked only: the first writes a large sweep, the second opens a viewer).
Not changed, and why: `combined_controller.py` (runs through `run_loop`, which applies the feedforward itself), `arm_hold_fixes.py` (uses HoldSim from `arm_hold_diagnosis.py`; not run), `realistic_snap.py` (no ctrl writes; uses the other scripts' classes; not run), `chemistry_lab_tracer_teleop.py` and `empty_room_joint_test.py` (load their own XML without the arm actuators), the experiments I had changed earlier (`arm_sag_test`, `beaker_hotplate_success_test`, `pedestal_lift_test`, `rest_contacts_test`, `gravity_ff_test`).
**Look obsolete (my reading; not deleted, for you to decide):** `arm_hold_diagnosis.py` (broken, and the question it was written for is answered), `arm_hold_fixes.py` (what-if tests for the same issue, depends on it), `realistic_snap.py` (depends on both), `workspace_sweep.py` and `run_with_dummy_policy.py` (early scripts using the legacy controller and the single_block scene), `chemistry_lab_tracer_teleop.py` and `empty_room_joint_test.py` (early scripts on their own models).
Also re-run after the src changes of this task: `full_check_test` 31/31, `place_theta_logic_test` 14/14, `stance_select_pipeline_test` 7/7 (with its replay), `pour_scene_test` 21/21.

## New decisions rows (word for word)

| M187 | 2026-10-26 | The full check's starting guess, MEASUREMENT ONLY (David, Oct 26; the default does not change, David decides after the report; NOT DECIDED). A TEMPORARY setting `FULLCHECK_CFG["start_mode"]` (src/task_config.py, read in `FullCheck._solve_pose_point`): "today" (default, today's behaviour: the first stop of a path starts from random arm shapes, later stops from the previous stop's answer and, for an error stance or the next hand turn, from the nominal stance's answer; random restarts rescue a stop the first starts failed) or "neighbourhood" (as the robot's own solver: the first stop of each path starts from the zombie pose, later stops from the previous stop's answer, no other start; a stop that fails from there FAILS in the pass/fail decision; random restarts still run on the side and are only logged as "rescuable": failed, but another arm shape could reach it). `FullCheck.start_log` records, for every solved stop, how it was started, whether it passed, and whether a random rescue was used ("rescue": the starts tried first failed and a random start passed) or was forced ("forced_random": no start was available). The legacy grasp-bin path (points with no "pose" key, only used by full_check_test's old points) is unchanged. Measured (`experiments/checker_start_compare.py`, the full stance selection both ways, normal and moved layout, data/checker_start/20261026_142253): in both layouts and both ways the first-ranked spot passes the full check and is chosen, with the same place angle (normal (0.755, 1.145) pedestal 0.10, theta -25 degrees; moved (0.635, 1.135) pedestal 0.10, theta -25 degrees); today's checker used NO random rescue at any stop (0 of 727 and 0 of 693 stops) and a random start only where none other existed (8 and 11 first stops of a path); no stop in either layout differs between the two ways (727 and 693 stops, the same pass and fail at every one); no failed stop could have been rescued by a random start; selection time 49.8 against 57.4 s (normal) and 45.4 against 47.0 s (moved). | David 2026-10-26 (measurement requested; the setting is NOT DECIDED) |
