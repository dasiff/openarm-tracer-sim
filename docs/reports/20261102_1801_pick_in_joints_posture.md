# Pick IN move, sag gain 0: per-joint attribution, friction settings, posture target, posture-weight sweep

Written for: David (design review). Branch held-weight-ramp-sag-off. Data: data/bounce_ramp_20261102/joint_probe (read-only run,
experiments/pick_joint_probe.py); attribution: experiments/pick_rot_attrib.py; sweep: experiments/solver_posture_sweep.py
(output data/bounce_ramp_20261102/posture_sweep.md). Row used: t = 86.19 s, the last IN tick before the timeout's recovery reset
(the reset sets the commanded joints to the measured ones, so rows after it show zero offsets). Acronyms: IN = the pick's move to the beaker;
L1 to L7 = arm joints 1 to 7 (the static probe's order, verified); kp = the servo's position gain (N m per rad); TCP = tool point.

## 1. Per-joint attribution (VERIFIED; 1-s averages in brackets where they differ)
Commanded minus measured angle (cmd-meas); pad contribution = -J_k (cmd-meas) with J_k the pad's translational Jacobian (contribution to
c minus b, the servo's miss); servo = MuJoCo actuator force; model = gravity model torque (src/gravity_ff.py); diff = servo minus model.
| joint | cmd-meas (mrad) | pad x (mm) | pad y (mm) | pad z (mm) | rotation (deg, x / y / z) | kp | servo (N m) | model (N m) | diff (N m) |
|---|---|---|---|---|---|---|---|---|---|
| L1 | +0.06 | 0.00 | +0.02 | -0.03 | +0.004 / 0 / 0 | 70 | -6.682 | -6.678 | -0.004 |
| L2 | -0.16 | +0.07 | 0.00 | +0.01 | 0 / -0.008 / +0.004 | 70 | -1.621 | -1.631 | +0.011 |
| L3 | +1.06 | +0.31 | -0.01 | +0.08 | -0.013 / +0.026 / +0.053 | 70 | -0.583 | -0.657 | +0.074 |
| L4 | -0.61 | -0.06 | -0.06 | +0.22 | -0.034 / +0.002 / -0.009 | 60 | +3.571 | +3.607 | -0.037 |
| L5 | -9.65 | +0.80 | +0.01 | +0.11 | -0.039 / +0.492 / +0.249 | 10 | +0.122 | +0.026 | +0.096 |
| L6 | +9.91 | +1.68 | +0.02 | -0.01 | +0.005 / -0.256 / +0.507 | 10 | -0.074 | +0.025 | -0.099 |
| L7 | +10.00 | -0.01 | +0.02 | -1.89 | +0.573 / +0.006 / -0.002 | 10 | -0.633 | -0.533 | -0.100 |
| sum | | +2.79 | 0.00 | -1.50 | +0.50 / +0.26 / +0.80 | | | | |
| measured c minus b | | +2.80 | -0.03 | -1.49 | norm 0.977 deg | | | | |
Sum check (VERIFIED): the seven contributions sum to (+2.79, 0.00, -1.50) mm against a measured (+2.80, -0.03, -1.49) mm; the rotation
contributions sum to norm 0.978 deg against a measured 0.977 deg. The 1-s averages give the same picture (sum +2.82, +0.01, -1.51 mm).

Which joints make the x miss (+2.8 mm): L6 (+1.68 mm) and L5 (+0.80 mm) carry about 89 % of it (2.48 of 2.79 mm), with L3 (+0.31 mm) the rest. Which make the
z miss (-1.5 mm): L7 (-1.89 mm), partly offset by L4 (+0.22 mm). Which make the 0.98 deg rotation: L5 (0.55 deg), L6 (0.57 deg) and L7 (0.57 deg),
with their axis components partly cancelling. VERIFIED for the numbers.

Mechanism (INFERRED): L5, L6 and L7 sit about 10 mrad from their commands, and the servo's torque minus the model's is -0.10 to +0.10 N m on
exactly those three joints. The model file gives those joints a friction of 0.1 N m (section 2). A servo gain of 10 N m per rad times 0.01 rad is 0.1 N m,
so the wrist joints sit at the edge of their friction band: the servo's torque cannot overcome the friction, and the joint stops about 10 mrad short.
The gravity model has no friction term, so the difference is not in the model. The elbow and the first three joints match the model to 0.07 N m or less.

## 2. Friction, damping, armature in the model file (VERIFIED from the compiled model, MjModel)
All seven joints of each arm (both arms): frictionloss 0.1 N m, damping 0.4 N m s per rad, armature 0. Source: models/robot/combined_robot.xml,
default classes motor_DM8009, motor_DM4340 and motor_DM4310 (joint lines 13, 17, 21; all three set frictionloss 0.1 and damping 0.4). The finger
joints (motor_finger) have damping 2 and no friction. Per joint: identical for L1 to L7.

## 3. Posture target (VERIFIED)
- Defined in src/mink_arm.py (MinkArm.__init__): the posture target is the arm's home pose on the arm's joints, q_post[qadr] = home_q, set by
  posture.set_target(q_post). Its weight is cfg["posture_cost"] on the arm's velocity dofs only (cost[vadr] = posture_cost, lines 82-84).
- home_q comes from task_cfg["home_q_left"] (src/task_config.py:92; radians from 22.598, -14.709, 15.531, 126.122, -3.746, 27.308, 13.018 degrees),
  passed by src/mpvi_pipeline.py:71 and src/mpvi_modules.py:5. Randomisation is off: home_variation_rad is all zeros (src/task_config.py:94).
- Posture target in radians: L1 0.3944, L2 -0.2567, L3 0.2710, L4 2.2011, L5 -0.0654, L6 0.4766, L7 0.2272.
- Commanded joints at the end of IN: L1 -0.4579, L2 -0.2163, L3 0.1631, L4 0.6528, L5 -0.2704, L6 -0.0631, L7 -0.4568.
- Difference (commanded minus target, the posture error): -0.852, +0.041, -0.107, -1.549, -0.195, -0.550, -0.694 rad. The elbow (L4) is
  1.55 rad (89 degrees) from its home value at the end of IN.

## 4. Posture-weight sweep, offline (scratch configs; MINK_CFG not changed)
Start: the commanded joints at the end of IN, the waypoint held. Posture target: the commanded joints at the START of the IN move
(L1 0.1444, L2 -0.3159, L3 0.1056, L4 1.3264, L5 -0.3384, L6 0.0293, L7 -0.3998 rad), not the home pose. Run until the joints stop
moving (under 1e-9 rad per step) or 5000 steps. VERIFIED (computed).
| posture weight | steps | miss (mm) | rot miss (deg) | L4 (elbow) change (rad) | all joints change (rad, norm) |
|---|---|---|---|---|---|
| 0.01 | 137 (stopped) | 1.625 | 0.048 | -0.029 | 0.049 |
| 0.003 | 1203 (stopped) | 0.157 | 0.005 | -0.053 | 0.076 |
| 0.001 | 5000 (NOT stopped) | 0.018 | 0.0005 | -0.055 | 0.078 |
| 0.0001 | 5000 (NOT stopped) | 0.0002 | 0.0000 | -0.056 | 0.077 |
Reading: the miss goes to a fixed point that shrinks as the weight shrinks; at 0.001 and 0.0001 the run is still creeping after 5000 steps, so
those two values are the miss at step 5000, not a settled value. The elbow moves 1.7 to 3.2 degrees in the negative direction. It moves AWAY from
the posture target (1.326 rad at the IN start): its distance from that target grows from 0.674 rad at the start to 0.703 rad at weight 0.01 and to
0.729 rad at 0.001. So the frame task, not the posture task, is what moves the elbow here. INFERRED from the sign and size.
Note: with the home pose as the target (earlier offline run, weight 0.01) the fixed point was 3.39 mm; with the IN-start pose it is 1.63 mm.
The posture target matters as much as the weight.

## 5. Everything that sets or uses the posture weight (VERIFIED by search)
- Set: src/task_config.py:405 MINK_CFG["posture_cost"] = 0.01 [row M088].
- Read by: src/mink_arm.py:83 (the posture task's cost); the solve at mink_arm.py:154 (joint_arc, the dry run for move timing) and :197 (step, the
  per-tick solve). Every MinkArm uses MINK_CFG or a copy of it (cfg argument).
- MinkArm built by: src/mpvi_modules.py:143 (the pick and place skills and their recovery, through make_scripted_manipulation);
  experiments: moved_place_split_diag.py:48, pedestal_sweep_measure.py:159 (which also calls arm.posture at :172), place_over_turn_measure.py:199,219,
  place_over_angle_measure.py:128, and this branch's solver_offline_check.py and solver_posture_sweep.py.
- Skills that depend on it: the pick and place skills and the recovery, through MinkArm (the arm's home pose sets the target).
- Reach map (src/reach_map.py): NOT FOUND. Its IK is src/arm_kinematics.solve_ik (damped least squares with no posture term).
- Stance finder (src/stance_finder.py, src/stance_checks.py, src/stance_select.py, src/bench_stance.py): NOT FOUND. No MinkArm and no posture term;
  it uses batch FK and the reach map.
- Home clearance map (src/home_clearance_map.py): NOT FOUND (no solver).
- src/full_check.py and src/weight_estimate.py: no posture term. weight_estimate uses MinkArm only for its pad and joint indices.

## Open
- The wrist friction band (section 1) is the main contributor to the x and rotation miss. Whether to model it, or change the wrist gains, is David's call.
- The posture target and weight together set the solver's fixed point (section 4). Changing either is a design choice; none was made here.
