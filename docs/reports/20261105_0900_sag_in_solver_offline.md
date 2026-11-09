# Why IN still fails: the arm solver offline (M264 follow-up)

Written for David. Read-only and offline: the task model was loaded and solver calls made on it; no physics was stepped; no code changed; no baseline touched; nothing committed. Scripts are in the session scratchpad (listed in the message).

## Set-up checks

- The task model (load_task, no step) reproduces the logged pad x and y to 0.003 mm from the logged joints and base pose. The logged pad z is 67.30 mm lower than the model at the default base height (0.22 m), constant at ticks 9688, 9915, 10469 and 12724. The run's base sat at 0.1527 m; this offset is used for all states here.
- The arm solver is src/mink_arm.py MinkArm with MINK_CFG (position cost 1.0, orientation cost 0.5, posture cost 0.01, joint margin 10 deg, velocity limit 0.5 rad/s, furniture collision minimum 8 mm, right-arm minimum 40 mm), dt 1/150, one call per tick, as the script calls it.
- Check on the solver: from the scripted OVER and DOWN starts, the offline steady state matches the live steady state to within 0.02 mm (below).

## Q1. Planned point minus FK of the solver's targets (mm x,y,z; deg rx,ry,rz)

Scripted run (signal trace, 229, 552 and 2256 rows):

| step | start | middle | end | mean |
|---|---|---|---|---|
| pick OVER | −0.56, +0.28, −1.00 \| 0.25, 0.13, −0.07 | −0.25, +0.08, +0.01 \| 0.00, −0.01, +0.02 | −0.38, +0.11, −0.02 \| 0.00, −0.01, +0.03 | −0.32, +0.13, −0.11 \| 0.02, 0.01, 0.01 |
| pick DOWN | −2.02, +2.83, −0.48 \| −0.13, −0.45, +0.68 | −0.36, +0.21, +0.34 \| 0.02, −0.01, +0.02 | −0.37, +0.38, +0.88 \| 0.05, −0.01, +0.02 | −0.40, +0.24, +0.38 \| 0.02, −0.01, +0.03 |
| pick IN | −0.36, +0.37, +0.89 \| 0.05, −0.01, +0.02 | −0.63, +2.38, +2.34 \| 0.12, −0.01, +0.04 | −0.63, +2.38, +2.34 \| 0.12, −0.01, +0.04 | −0.59, +2.13, **+2.15** \| 0.11, −0.01, +0.04 |

Mock run (the mock logs the planned point and FK only on the ticks its layer runs, so OVER has 80 of 368 rows and DOWN 170 of 782; one DOWN middle value, z −13.8 mm, is an unexplained outlier):

| step | start | middle | end | mean |
|---|---|---|---|---|
| OVER | +0.01, −0.01, +0.00 | −0.35, +0.11, −0.01 | +0.19, −0.36, +0.16 | (outlier-affected) |
| DOWN | −0.38, +0.11, −0.02 | −0.35, +0.24, −13.76 (outlier) | −0.37, +0.38, +0.58 | (outlier-affected) |
| IN | −0.36, +0.38, +0.87 | −0.61, +1.82, +2.23 | −0.63, −0.13, +2.37 | −0.59, −0.67, **+2.16** |

The +2.15 mm in z during IN is present in both runs (scripted +2.15, mock +2.16). The mock's IN end has y at −0.13 mm rather than +2.38, so the mock's y offset is smaller. At OVER and DOWN the offsets are below one millimetre in both.

## Q2. Offline solver on the waypoints

Targets are the planned endpoints from the scripted log; orientation targets are the measured pad orientation at the step end (the planned orientation is constant through the pick, and the measured error at each end is 0.13 deg or less).

**(a) Normal per-tick settings, min-jerk path, the step's own duration, then a 150-tick hold.** Residual = FK minus target, mm, after the final tick:

| waypoint | final residual x,y,z | orientation | joint margin | furniture check |
|---|---|---|---|---|
| OVER | −0.41, +0.10, −0.03 | 0.03 deg | closest joint 4 at 14.3 deg | not near |
| DOWN | −0.37, +0.39, +0.89 | 0.06 deg | closest joint 2 at 18.2 deg | 24 mm |
| IN (with all constraints) | +3.45, +103.28, +0.95 | 0.96 deg | closest joint 2 at 18.7 deg | 7.7 mm (pair finger / hotplate); the solver's minimum is 8 mm |

The live steady state at OVER and DOWN is (−0.38, +0.11, −0.02) and (−0.37, +0.38, +0.88) mm. The offline finals match these to within 0.03 mm, so the offline solver reproduces the live run.

**(b) Target held, more solver calls.** Residual against calls (mm x,y,z; deg):

| calls | OVER | DOWN | IN (all constraints) |
|---|---|---|---|
| 50 | −40.5, +16.9, +0.6; 3.55 | −8.4, +6.8, +75.0; 5.70 | +0.2, +75.0, +33.7; 0.83 |
| 100 | −10.0, −0.1, −0.2; 0.88 | −7.4, +7.0, +46.6; 3.38 | −1.9, +36.3, +45.1; 0.76 |
| 200 | −0.41, +0.10, −0.03; 0.03 | −0.64, +1.36, +2.56; 0.13 | −0.39, +15.7, +50.8; 1.03 |
| 450 to 6000 | unchanged | −0.37, +0.39, +0.89; 0.06 | unchanged at −0.39, +15.7, +50.8; 1.03 |

Convergence is reached by about 200 calls for OVER and DOWN and stays fixed to 6000 calls. So none of the offsets is an iteration count effect.

**Ablation of the IN waypoint (1500 calls each; residual FK minus target, mm; orientation):**

| constraints | residual x, y, z | orientation |
|---|---|---|
| all constraints | −0.39, +15.71, +50.78 | 1.03 deg |
| no collision limit | −0.62, +2.36, +2.32 | 0.12 deg |
| no joint limits | −0.39, +15.71, +50.78 | 1.03 deg (joint limits are not active) |
| no posture term | −0.01, +14.71, +50.17 | 1.02 deg |
| no collision, no posture | 0.00, 0.00, 0.00 | 0.00 deg |
| no collision, no velocity limit | −0.62, +2.36, +2.32 | 0.12 deg |
| no collision, no posture, no velocity limit | 0.00, 0.00, 0.00 | 0.00 deg |
| no collision, frame position weight x10 | −0.01, +0.03, +0.02 | 0.13 deg |
| no collision, no posture, position weight x10 | 0.00, 0.00, 0.00 | 0.00 deg |

The no-collision case gives (−0.62, +2.36, +2.32) mm, the live IN-end value (−0.63, +2.38, +2.34) to within 0.02 mm. Collision avoidance is therefore not what holds the live IN end, and the live IN end matches the posture-balanced solution.

**Joint margins (IN, all constraints, the stuck solution):** the closest joint is joint 2 at 16.6 deg from its 10 deg margin, so no joint limit binds there. The no-collision solution was not checked for joint margins; the no-joint-limit ablation gives the same residual as the full run, so no joint limit is active in either.

## Q3. Does the solver's own shortfall explain the +2.15 mm?

Yes, for the live steady state. The residual that remains with the collision limit off is +2.36 (y) and +2.32 (z) mm, and the live run ends at +2.38 and +2.34 mm. The offset disappears completely when the posture term is removed (0.00 mm), and it falls to 0.03 mm when the frame position weight is raised tenfold. Its size does not change between 200 and 6000 calls. So the +2.15 mm is a weight effect: the posture term (cost 0.01, which chooses the elbow) balances the frame position term (cost 1.0) at a steady offset. It is neither an iteration effect nor a reach limit: the same target is reached exactly without the posture term.

The target is reachable. What the solver cannot do is reach it while keeping the posture term's pull. The old trim compensated for that pull because it integrated the planned point, and the layer integrates the solver's own output, which already contains the pull.

One caveat is unresolved: with the furniture collision limit on, the offline IN approach stops about 100 mm short of the target (residual +15.7, +50.8 mm) at a finger distance of 7.7 mm from the hotplate. The live run did not stop there. The collision-free solution at the target lies 36 mm inside the hotplate by the same distance helper, so the helper and the live contact geometry disagree and I do not trust its absolute distances. This is recorded as NOT BUILT, needs decision (see below).

## Q4. Calculation: would a planned-point reference give the old trim at IN's end?

Labelled as a calculation, not a simulation. The layer's correction at IN's end is (+0.33, −2.59, −3.79) mm; the old trim is (−2.10, −2.67, −1.30) mm; the difference is (+2.43, +0.08, −2.49) mm. The trim integrates gain times error, and the error is a feedback quantity, so integrating the logged offset over IN (about 96 mm·tick-sum) is not a valid estimate.

Calculation: the layer with the planned point is the old in-script trim with the same inputs. The two runs' trims agreed to 0.05 mm at the ends of OVER and DOWN, when the inputs were the same. A planned-point layer would therefore be expected to end IN within about 0.1 mm of the old values (−2.10, −2.67, −1.30) mm, as long as the arm followed the old trajectory. That expectation is inferred from the identity with the old algorithm and is not measured.

## Q5. Options for the reference (none chosen)

| option | what it changes | script | mock | real VLA | evidence |
|---|---|---|---|---|---|
| (A) joint-space trim | integrates q_source − q_measured; no FK | the joint targets carry the posture pull, so the trim would track the same shortfall | same | the VLA's joints carry the same bias | the shortfall is in the joint targets; A alone does not remove it (rules out A as the fix) |
| (B) converged solve inside the source | the source solves to the planned pose without the posture pull (or with a raised frame weight) | removes the 2.2 mm at IN (the frame task reaches the target at 0.00 mm without posture) | the mock's own solver would need the same change | the VLA would not solve in-loop; it would need a planned-pose-consistent solve | supported: the offset disappears with the posture term removed |
| (C) source reports its planned pose | the layer's reference is the planned point (the old behaviour), so the trim integrates planned minus measured | works | works (the mock has the planned path) | a VLA outputs joints, so it would need to report poses too (breaks the absolute-joint interface) | supported: matches the old trim per Q4 |
| (D) FK of the last command held per step | the reference is fixed at the step's first FK | still integrates the posture offset | same | same | ruled out for IN: the offset is the solver's steady state, so holding the reference does not remove it |
| extra: change the posture weight or the frame weight in MINK_CFG | a solver setting, not a reference | the offset falls to 0.03 mm with frame weight x10; changes the elbow choice | affects the mock's solver too | affects the VLA only if it uses the same solver | supported by the ablation; a David decision because it changes the elbow |

The evidence rules out (A) as a fix and (D) for IN. (B), (C) and the solver-weight change are supported; (B) and the weight change address the cause, (C) addresses the reference.

**If the target were unreachable**, the stance or the waypoint would have to change. The evidence does not show that: the target is reachable without the posture term, and the live run reached it to within 2.4 mm. The collision question at IN (the finger near the hotplate) is the one thing that could make the endpoint unreachable, and it needs the geometry checked in the sim before choosing.

## Q6. Other waypoints (baseline run's achieved poses as targets)

The baseline run reached these steps, so their end poses are reachable by construction. The start states come from the baseline's episode log at the step start, and the FK matches the logged pad to 0.01 mm there.

| waypoint | ticks (N) | offline end residual x, y, z (mm) | orientation | notes |
|---|---|---|---|---|
| pick UP | 11789–12237 (449) | −0.50, +3.96, −104.98 (no collision: −0.34, +0.97, +0.41) | 1.7 deg | blocked by the furniture collision limit; removing the furniture limit reaches the target (0.41 mm z); removing the right-arm limit changes nothing |
| place OVER | 12239–12490 (252) | +0.14, +1.28, +0.51 | 0.04 deg | converges by 200 calls |
| place DOWN | 12491–12799 (309) | +0.17, +1.54, +0.90 | 0.06 deg | converges by 200 calls |
| place BACK | 13010–13210 (201) | −0.01, +0.89, +0.57 | 0.04 deg | converges by 200 calls |

At the place waypoints the same sign pattern appears: the residual is about +1 mm in y and +0.5 to +0.9 mm in z (the posture offset). It is smaller than at IN, because the posture pull is weaker there (joint margins 1.9 to 2.7 deg, so joint 2 is nearer its upper margin). So the miss is present at the other waypoints at the 1 to 2 mm level, and at IN it is the largest. UP is blocked by the furniture collision limit in this offline test, which the live run did not show, so UP is also not explained.

## NOT BUILT, needs decision

1. **Collision at the IN endpoint.** The offline IN approach is stopped by the furniture collision limit (8 mm), and the furniture distance helper reports the target 36 mm inside the hotplate. The live run did not stop there. Default: no change; the live behaviour is taken as correct, and the helper's absolute distances are not trusted.
2. **UP blocked offline.** The offline UP stops 105 mm short under the furniture limit. The live run lifted. Default: no change; this is unexplained.
3. **Posture weight.** Default: MINK_CFG posture cost 0.01 unchanged; the weight decision is David's.
4. **Reference.** Default: forward kinematics of the source's targets, as decided.
