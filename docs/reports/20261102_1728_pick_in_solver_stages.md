# Pick IN move, sag gain 0: where the 3.3 mm comes from (solver vs servo), solver settings, offline solver test

Written for: David (design review). Branch held-weight-ramp-sag-off. Run: data/bounce_ramp_20261102/stage_probe (read-only probe,
experiments/pick_stage_probe.py); offline checks: data/bounce_ramp_20261102/solver_offline.md (experiments/solver_offline_check.py).
Definitions: a = waypoint target pad pose (the move's target); b = pad pose from the commanded joints (forward kinematics, the same
MinkArm.pad_pose function as the measurement); c = measured pad pose. Errors are differences of these, in mm and degrees.
Acronyms: IN = the pick's move to the beaker; TCP = tool point (the hand's body the solver drives); FK = forward kinematics; QP = quadratic program.

## 1. Stage split, last 1 s of IN (attempt 1, t = 85.27 to 86.27 s; the move times out at 86.27 s)
| t (s) | stage | x (mm) | y (mm) | z (mm) | rotation (deg) |
|---|---|---|---|---|---|
| 85.27 | b minus a (solver's miss) | -0.63 | +2.38 | +2.34 | 0.122 |
| 85.27 | c minus b (servo's miss) | +2.85 | -0.01 | -1.50 | 0.980 |
| 85.27 | c minus a (total) | +2.22 | +2.37 | +0.83 | 1.070 |
VERIFIED (from the probe rows; 0.1 s samples in the probe CSV). Rows at t = 86.27 s and later are not used: the recovery that follows the timeout resets the commanded joints to the measured ones, so b equals c there. The commanded pose b is 3.4 mm from the waypoint a at all times in the last
second, and it does not move (the joint command is still). The total c minus a holds at 3.3 mm and drifts about 0.01 mm per second.
Plain meaning: the solver's commanded pose already misses the waypoint by 3.4 mm; the servo does not reach that command either (it lags by 3.2 mm,
mostly in x). The two stages partly cancel in z and add in x.
Which stage holds most: the solver. Its miss alone (3.4 mm, mostly y and z: 2.4 mm each) is about the size of the total, and the servo's
x lag (2.85 mm) is mostly offset by its z sign. VERIFIED for the numbers; the "most" judgement is INFERRED from them.

## 2. Solver settings and state at the end of IN (VERIFIED from src/task_config.py MINK_CFG and the probe)
- Frame task (pad centre, through the tool point): position weight 1.0, orientation weight 0.5.
- Posture task (toward the home pose, the arm's 7 joints): weight 0.01.
- Damping (Levenberg-Marquardt term in mink.solve_ik): 1e-3.
- Solver: daqp (a QP backend); tolerance: NOT FOUND in our code. We pass no tolerance; the daqp default applies (value not checked).
- Velocity limit: 0.5 rad/s per joint. At the end of IN no joint is at it (velocities 0; vel_bind 0). Not active.
- Joint limits with a 10 degree margin: smallest slack 0.216 rad. Not active.
- Collision avoidance (furniture, minimum distance 8 mm): closest pair openarm_left_hand / bench, about 23 mm; 9 of 112 pairs inside the 50 mm
  detect distance. The closest distance is well above the 8 mm minimum, so it is not stopping the move. INFERRED from mink's rule.
- Collision avoidance (right arm, minimum distance 40 mm): closest pair openarm_left_link5 / openarm_right_link5 at 200 mm; none inside the detect distance. Not active.
- Solver residuals at the last IN tick (the solve's own error, mink's compute_error): frame task position (-2.34, -0.64, +2.38) mm, orientation
  (-0.00064, 0.00202, -0.00015) rad (norm 0.12 deg). Posture task: (-0.85, +0.04, -0.11, -1.55, -0.20, -0.55, -0.69) rad for joints 1 to 7.
  VERIFIED. The frame residual's 3.3 mm norm is the same miss as stage b minus a.

## 3. Offline solver test (no dynamics): hold the waypoint, 200 iterations from the commanded joints at the end of IN
Start state: the full state from the recording checkpoint at tick 12,900 (t = 86.00 s), with the arm at the last commanded joints.
Start miss 3.307 mm, 1.066 deg (matches the probe). VERIFIED.
| iteration | miss, posture weight 0.01 (as in MINK_CFG) | miss, posture weight 0 |
|---|---|---|
| 1 | 3.38 mm, 0.43 deg | 3.42 mm, 0.43 deg |
| 10 | 3.39 mm, 0.12 deg | 1.76 mm, 0.05 deg |
| 20 | 3.39 mm, 0.12 deg | 0.011 mm, 0.0004 deg |
| 50 | 3.39 mm, 0.12 deg | 0.000 mm, 0.000 deg |
| 200 | 3.39 mm, 0.12 deg | 0.000 mm, 0.000 deg |
VERIFIED (computed). Meaning: with the posture task on, the solver goes to a NONZERO FIXED POINT: it stops at 3.39 mm and does not move further
(joints move about 0.01 rad in 200 iterations). With the posture task off, it reaches zero within about 50 iterations. The posture task is what keeps the
miss, not joint limits, velocity limits or collisions (none is active). INFERRED that the posture task's pull toward the home pose balances the
frame task at this pose; not separately proven.

## 4. Pad versus tool point (VERIFIED, 20 random joint sets within the limits)
- Measured pad pose (src/mink_arm.py, MinkArm.pad_pose): R = R_tcp @ R_pad_in_tcp; p = p_tcp + R_tcp @ pad_in_tcp (R_tcp and p_tcp: the tool point body's xmat and xpos).
- Desired pad to tool point (MinkArm.step): R_tcp = R_des @ R_pad_in_tcp.T; p_tcp = p_des - R_tcp @ pad_in_tcp.
- Both use the same vectors: pad_in_tcp (the pad centre in the tool point frame) and R_pad_in_tcp (the pad frame in the tool point frame).
- Numerical test: pad from the measurement, converted to the tool point pose, compared with the tool point's forward kinematics: max position difference
  0.000000 mm, max rotation difference 0.00000000 deg. VERIFIED. (The test checks the algebra; both sides come from the same two vectors, so it cannot catch a wrong vector.)

## 5. Joint order (VERIFIED)
Static probe (actuators, in its order): left_joint1_ctrl ... left_joint7_ctrl, which are L1 to L7. Solver order (arm_ids): the same actuators;
joint names openarm_left_joint1 to openarm_left_joint7; qpos addresses 10 to 16. The static check's joint list is L1 to L7 in that order.

## Open
- The solver's fixed point (3.4 mm with posture weight 0.01) is the main lever for the miss. A change of posture weight or of the frame-task weight is a design
  choice (David); not made here.
- The daqp tolerance value is not recorded in our code (NOT FOUND).
