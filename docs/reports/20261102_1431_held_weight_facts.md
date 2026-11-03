# Held-weight compensation: facts gathered (read-only)

Status: no simulation running at the start (pgrep showed only system daemons). Nothing under src/ or data/ was changed. Not committed. Scratch script (outside the repo): the session scratchpad, task45.py. Labels: VERIFIED = read in code or data; INFERRED = reasoned from code or data; NOT FOUND = not read or not present.

## Task 0: M201 default column (VERIFIED)
- docs/mpvi_decisions.md, row M201: default column now reads "held-weight term always on in the shared bottom layer, mass 0 when empty; mass slewed in and out; slew time pending".
- docs/design_plan.md section 15 has no default column (its tables are Item, Decisions-log row, Description), so nothing changed there.

## Task 1: rows M212, M213, M214 (VERIFIED, text in the decisions log and in section 15)
Shown in full in the chat reply and in git diff docs/mpvi_decisions.md (added lines). Section 15 rows l, m, n are the same text.

## Task 2: weight offset per joint (VERIFIED)
Source: data/diag/bounce_20261029/window_11400_11900.csv. Note: the user named extract_on.csv; it has no "PLACE: DOWN" label (it has CHECK, OVER, UP and blank), so this file was used. Its columns L{i}_ff_held and L{i}_ff_grav are the same quantities as in extract_on.csv (the two files agree at tick 11598).
- Held part is logged separately: experiments/bounce_diag_extract.py lines 94-110 compute the total feedforward with the held mass set, and the gravity-only feedforward with it cleared; L{i}_ff_held = total minus gravity-only. VERIFIED.
- kp (servo stiffness, N m per rad): 70, 70, 70, 60, 10, 10, 10 for joints 1 to 7. Joint 4 is 60, not 70. Checked against the built model's actuator gains (VERIFIED) and src/robot_specs.py lines 155-161 (VERIFIED).
- Offset = held torque / kp (rad), shown in mrad.

Held mass first nonzero in the file: tick 11599 (hm_l = 0.205 kg). The file's ff_held is already nonzero at 11599.

Tick 11600 (first tick after switch-on):
| joint | held torque N m | kp | held offset mrad | gravity torque N m | gravity offset mrad |
|---|---|---|---|---|---|
| L1 | -0.9614 | 70 | -13.73 | -6.4334 | -91.9 |
| L2 | -0.1207 | 70 | -1.72 | -2.0088 | -28.7 |
| L3 | -0.2609 | 70 | -3.73 | -1.3274 | -19.0 |
| L4 | +0.7646 | 60 | +12.74 | +3.9292 | +65.5 |
| L5 | +0.0108 | 10 | +1.08 | +0.0797 | +8.0 |
| L6 | -0.0005 | 10 | -0.05 | -0.0038 | -0.4 |
| L7 | -0.3790 | 10 | -37.90 | -0.5328 | -53.3 |

Last tick in the file for PLACE: DOWN: tick 11900 (the file ends there; PLACE: DOWN runs from 11854 to 11900 in the file; the step 6 log labels DOWN to tick 12156, but it has no torque columns). Held offsets at 11900: L1 -13.75, L2 +3.61, L3 -2.63, L4 +13.16, L5 -4.95, L6 +0.30, L7 -37.90 mrad.
- Largest held offset: joint 7 (-37.9 mrad), because its kp is 10. Largest gravity-only offset: joint 1 (-91.9 mrad at 11600, -102.9 at 11900).
- Check on M201: its figure of -13.72 mrad for L1 and -38.10 mrad for L7 are one-tick changes in the servo target (ctrl target "act") from tick 11598 to 11599 (-0.013718 and -0.038103 rad), not the held offset. VERIFIED from the file. Joint 1's target change equals its held offset to rounding; joint 7's does not (the target also moves by the tracking part).

## Task 3: the hold gates (VERIFIED from code unless marked)
There is no function called "hold gates". The checks on a weight reading are:
1. Still gate (src/weight_estimate.py line 42): every joint of the arm must be slower than the threshold for the whole window. Threshold WEIGHT_READING_CFG["still_speed_rad_s"] = 0.005 rad/s (src/task_config.py line 146). Speed is the largest absolute joint speed of the arm, taken each tick (src/mpvi_modules.py line 442). Window WEIGHT_READING_CFG["window_s"] = 0.30 s (task_config.py line 147), which is 45 ticks at 150 Hz (weight_estimate.py line 40, and the buffer length in mpvi_modules.py lines 439-441).
2. Timing gate (weight_estimate.py lines 41 and 45): the arm must be still within WEIGHT_READING_CFG["max_wait_s"] = 1.0 s after the handoff trigger (task_config.py line 148). Later than that gives "no reading".
3. Positive mass only (src/mpvi_modules.py line 423): the mass is set only if it is greater than zero.
4. Not a gate: the fit-quality numbers from hand_load (unexplained share, angle from straight down) are only printed (mpvi_modules.py line 425). A grep found no threshold on them. NOT FOUND in the code: any numeric limit on fit quality.
5. The reading exists only in the "held" step of the scripted pick (mpvi_modules.py lines 410-428), which runs after the grasp.

What the code does on failure (VERIFIED, mpvi_modules.py lines 415-428):
- "wait" (not still yet, still within time): the step does not advance; it checks again next tick.
- "no reading" (out of time): no mass is set; the held mass stays at its previous value (normally zero: it is cleared at run start, control_loop.py line 376, and after the gripper opens, mpvi_modules.py line 446); a message is printed; the step advances.
- Mass not greater than zero: nothing is set; the step advances.
- It never keeps an old non-zero mass as a fallback except by leaving it untouched, as above.

## Task 4: what the estimate reads, and whether it needs the skill
(a) Signals read each tick (VERIFIED, mpvi_modules.py lines 442-444 and weight_estimate.py lines 61-78):
- Largest absolute arm joint speed (data.qvel of the arm's joints): collected by the skill step.
- Full joint positions (data.qpos): collected by the skill step, averaged over the window at the reading.
- Applied actuator force of the 7 arm actuators (data.actuator_force): collected by the skill step.
- The arm's own gravity torque (gravity_ff.torques): bottom layer (src/gravity_ff.py), with the model's own body masses.
- Pad pose and the Jacobian (arm.pad_pose, mujoco.mj_jac): arm geometry, bottom layer.
- The handoff time (S["t0"]) and the decision to read: skill layer (scripted pick code).
- The world model is NOT read by the estimate. The lift (world.object_pose) is logged only (mpvi_modules.py line 411).
So the sensing is bottom layer; the collection window, the trigger time and the decision to read are the scripted skill's. The estimate runs only inside the scripted pick step, so with a vision language action model (VLA) driving, nothing collects the window and no reading would be taken. INFERRED from the code path.

(b) Sag correction (VERIFIED, mpvi_modules.py lines 336-352): it needs the reference pad pose p_ref and reference rotation R_ref, which the skill builds from its own move plan (start pose st["p0"], target st["target"], and the slerp). It also needs the measured pad pose. Answer: NO, it could not run unmodified under a VLA, because the reference comes from the skill's move plan. INFERRED that a VLA supplies no such reference.

(c) Timing (VERIFIED by a timed script, scratchpad task45.py, single process, this VM): one call of hand_load averages 2.70 ms (300 calls). Of that, creating the MuJoCo data object inside hand_load (weight_estimate.py line 61) takes 2.15 ms (80 %); this is the main cost, which a reused buffer would avoid (design choice, not made here).
- One tick at the loop's 150 Hz rate is 6.67 ms: the estimate costs 40 % of a tick if run every tick.
- One tick at 50 Hz (the bridge's rate, not the loop's) is 20 ms: 13.5 %.
- Discrepancy: the request said "50 Hz control tick"; the loop's control tick is 150 Hz (control_loop.py docstring and LOOP_CFG source_hz = 150). Both are given above.
- The timing used the same estimator function with the arm geometry rebuilt from the model (same formulas as src/mink_arm.py pad_pose). Its cost was measured, not the full controller.

## Task 5: is the instantaneous estimate usable while moving? (VERIFIED for the window, with stated limits)
Data: window_11400_11900.csv, 501 ticks (11400 to 11900). The estimate was run on every tick with the estimator's own function (weight_estimate.hand_load), using logged joint angles and applied forces. The base pose is NOT logged in this file; the base was taken at its initial recording pose (INFERRED approximation; the base pose changes the vertical Jacobian only slightly, not measured). The true mass is the beaker_mass column of the step 6 log: 0.200 kg (VERIFIED, constant).
- Still ticks (largest arm joint speed under 0.005 rad/s): 51 ticks, median 0.206 kg, minimum 0.198, maximum 0.400, standard deviation 0.027 kg.
- Moving ticks: 450 ticks, median 0.203 kg, minimum -0.071 kg, maximum 0.405 kg, standard deviation 0.080 kg.
- By phase (VERIFIED): PICK UP still median 0.206 kg; PICK UP moving median 0.205 kg (range 0.167 to 0.211); PLACE: DOWN moving median 0.158 kg (range 0.142 to 0.226); PLACE: OVER moving median 0.197 kg (range -0.071 to 0.405).
- Coverage gap: the window has only one carry tick ("NAV: starting") and no full carry. The pick grasp before tick 11400 is not in this file. So the carry part of the question cannot be answered from these logs.
- Missing from the logs for a full answer: base pose per tick (needed for the exact Jacobian), the carry phase, and the pick from grasp onward. Nothing was invented.

## Open points for David (not decided here)
- Option for Task 4(c): reuse one data object inside hand_load (removes about 80 % of the cost) versus keep the per-call allocation. Choice needed.
- The "50 Hz control tick" in the request is the bridge rate; the loop's tick is 150 Hz. Which rate the per-tick estimate should be budgeted against needs a decision.
- The PLACE: DOWN phase in the window ends at tick 11900; the step 6 log has DOWN until tick 12156. Whether a longer window is wanted for the place offset is a choice.
