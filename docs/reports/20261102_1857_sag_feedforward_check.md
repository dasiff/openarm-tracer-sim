# Existing sag-on run around the first held tick: trim, feedforward equivalent, settling, pad movement (read only)

Written for: David (design review). Branch held-weight-ramp-sag-off. No simulation was stepped; the script only reads logs and computes kinematics (experiments/sag_feedforward_check.py, not in src/).
Inputs: data/diag/height_20261101/extract_on.csv (the existing sag-on run's per-tick extract, ticks 11560 to 11800), and the recording of the same run (data/closed_loop/on_20261029_1453/recording) for the model's geometry and joint gains (kp).

Acronyms: C = the sag correction (the integral trim on the pad pose error, decisions-log M225); kp = the joint servo's position gain (N m/rad); Jz = vertical pad motion per radian of one joint (pad Jacobian); m g = held mass times gravity.

**Plain summary.** By the first held tick (11599, when the mass first reads 0.205 kg), the trim had already reached about 17.5 mm up in z. Its value at the earliest tick in the files (11560) was already 17.2 mm, so the trim had learned the beaker's sag during the lift, before the weighing. The feedforward's own estimate of the sag is 19.2 mm up in z at the estimated mass, so the trim is about 1.6 mm (9%) short in z. In x and y the trim disagrees with the feedforward estimate by 0.7 mm and 3.3 mm. The trim was still rising at about 1.2 mm per second at the held tick, so it had not settled. The pad itself did not move between 11560 and 11599 (0.04 mm).

## Setup checks (VERIFIED)
- The feedforward's held torque, recomputed as m g Jz at 0.205 kg from the joint angles, matches the logged L1 to L7 held torques at tick 11599 to within 0.0002 N m (script output). So the Jacobian and the mass are consistent with the logged feedforward.
- kp per left joint from the model's actuators: 70, 70, 70, 60, 10, 10, 10 N m/rad (VERIFIED from the model; joints 1 and 2 match the 70 in src/robot_specs.py, the rest were not compared).
- The gripper close tick is NOT in the files: the extract starts at tick 11560 (phase UP). The brief's fallback applies: the earliest tick in the files (11560) is used wherever a close tick is asked for.
- The sag columns in the extract (sag_x_mm, sag_y_mm, sag_z_mm) are taken as the trim value in mm. The script that wrote this extract is not in the repo (NOT FOUND); the meaning is INFERRED from data/diag/height_20261101/SUMMARY.md ("target_z = reference_z plus sag correction").
- The extract has no rotation column for the trim (NOT FOUND in the files), so the rotation part of item 1 is NOT AVAILABLE.

## Item 1: trim extra (tick 11598 minus tick 11560, the earliest tick in the files)
| component | tick 11598 (mm) | tick 11560 (mm) | difference (mm) |
|---|---|---|---|
| x | 3.056 | 3.334 | -0.278 |
| y | -3.061 | -3.151 | +0.090 |
| z | 17.529 | 17.206 | +0.323 |
| rotation | NOT AVAILABLE | NOT AVAILABLE | NOT AVAILABLE |

VERIFIED (extract values). The gripper close tick (about tick 11000, from the run log) is not in the files, so this is not the change since close, which is what the brief asks for. It is the change over the last 39 ticks before the held mass. INFERRED: the trim had already learned most of the sag before tick 11560. What it means: the trim was not starting from zero at the held tick. Why it matters: the change since close, which would show how much the trim learned while the beaker was being lifted, needs a close tick that this file does not contain.

## Item 2: feedforward equivalent at tick 11598 (pose of the tick before the held mass)
Method: each left joint's held offset = m g Jz / kp; the pad motion is the sum over the seven joints of (pad Jacobian column times offset); the rotation is the same sum with the rotation Jacobian.

| mass | x (mm) | y (mm) | z (mm) | rotation (deg, about x, y, z) |
|---|---|---|---|---|
| estimated 0.205 kg | +2.402 | +0.208 | +19.154 | +0.081, -3.716, -0.020 |
| true 0.200 kg | +2.344 | +0.203 | +18.687 | +0.079, -3.625, -0.019 |

VERIFIED (computed from the logged joint angles and the model). The sign is the feedforward's: the controller adds tau/kp to the joint target (src/gravity_ff.py, ctrl = q_target + tau_ff/kp), so these values are the target shift the feedforward applies. Gravity's own sag is the opposite direction, so this is the amount by which the target is raised to cancel the sag, in the same direction as the trim. What it means: the feedforward's own estimate of the sag is 19.2 mm up in z for the estimated mass. The trim value is 17.5 mm up in z at the held tick.

## Item 3: mismatch (item 2 minus item 1)
Literal definition (estimated mass):
| component | mismatch (mm) | as % of item 1 |
|---|---|---|
| x | +2.68 | -964% |
| y | +0.12 | +131% |
| z | +18.83 | +5830% |

VERIFIED arithmetic, but the percentages are not meaningful: item 1 is a small change over 39 ticks, so dividing by it inflates the figure. Use the level comparison below instead. The true-mass values differ from the estimated-mass values by 0.06 mm in x and 0.47 mm in z (VERIFIED arithmetic): the 2.4% mass difference matters little.

Level comparison (the trim's value at the held tick against the feedforward estimate, same direction and units; this is my addition, not in the brief):
| component | trim at 11598 (mm) | feedforward, 0.205 kg (mm) | trim minus feedforward (mm) |
|---|---|---|---|
| x | +3.056 | +2.402 | +0.65 (27% of the feedforward value) |
| y | -3.061 | +0.208 | -3.27 (large: the signs differ) |
| z | +17.529 | +19.154 | -1.63 (-9%) |

INFERRED: the z values agree in sign and roughly in size, so the trim is close to the feedforward's estimate in z. The x and y values do not agree; the trim has a y offset of -3.06 mm that the feedforward does not produce. The likely source is the arm's posture and the solver's own offset (not in the gravity model), not the beaker's weight. This is the reason the trim in x and y is not the mass alone.

## Item 4: settling of the trim z
- The brief asks for every tick from 0.6 s before the held mass (tick 11509) to tick 11598. The extract covers ticks 11560 to 11598 only (39 of 90 ticks). Ticks 11509 to 11559 are NOT AVAILABLE in the files.
- Values, ticks 11560 to 11598 (mm): 17.206 at 11560, rising by about 0.008 to 0.009 per tick, to 17.529 at 11598 (full list in the script output).
- Largest change over the last 0.3 s (the last 45 ticks; the extract covers 39 of them, ticks 11560 to 11598): range 0.323 mm; largest single-tick change 0.009 mm.

VERIFIED (values). INFERRED: the trim is still rising in a straight line at about 0.0085 mm per tick, which is about 1.3 mm per second at 150 ticks per second (the tick rate). So the trim has not settled at the held tick; it was still learning the sag. What it means: when the held mass starts, the trim is still moving. Why it matters: if the mass is later compensated by the feedforward, the trim and the feedforward would both be correcting the same sag during the ramp, which is one reason for the freeze design (M226).

## Item 5: pad movement from the earliest tick (11560) to the first held tick (11599)
- From the joint angles (forward kinematics): x -0.009 mm, y +0.030 mm, z -0.031 mm; norm 0.044 mm. VERIFIED.
- The logged pad height (padz_mm_rel) changed from +0.031 mm to -0.001 mm, which agrees (VERIFIED).
- The gripper close tick is not in the files, so the movement since close is NOT AVAILABLE. Over the 39 ticks available, the pad was nearly still.

## Limits
- Only the existing sag-on run's extract is used. Its exact recording (ticks 11560 to 11800) is the one in data/diag/height_20261101; its geometry is taken from data/closed_loop/on_20261029_1453, which the metrics script already uses as the same scenario (INFERRED that they are the same scene: they share the held tick 11599).
- Rotation of the trim is not in the file (NOT AVAILABLE).
- Item 1 is the change over the last 39 ticks, not the change since close (NOT AVAILABLE for the close tick).

## Correction (added after the item R check of the handoff build)
The close tick IS in the files: data/closed_loop/on_20261029_1453/instr_ticks.csv has the trim (sag_p, sag_r) on every tick. The first CLOSE tick is 10999 (VERIFIED). This file's first held-mass tick is 11600 (held_left), so the tick before is 11599 (VERIFIED).
- Trim at the close tick (10999): x -2.081, y -2.660, z -1.308 mm; rotation -0.5693, -0.1571, -0.7863 degrees (VERIFIED).
- Trim at tick 11599: x +3.049, y -3.058, z +17.538 mm; rotation -3.2010, +0.4973, +0.5485 degrees (VERIFIED).
- Change between them: x +5.130, y -0.398, z +18.846 mm (VERIFIED arithmetic).
- What this changes: the 0.32 mm z change above was the change over the last 39 ticks in the extract, not the change since the close. The change since the close is 18.85 mm in z, which is within 0.3 mm of the feedforward estimate of 19.15 mm (0.205 kg) and 18.69 mm (0.200 kg). INFERRED: the trim learned almost exactly the mass share during the lift. In x the change is 5.13 mm against the feedforward's 2.40 mm, and in y it is -0.40 mm against 0.21 mm (INFERRED: the x and y parts come from posture and solver offsets, not from the mass).

## Second correction (added after the handoff run, which checked the pad position)
The feedforward values in items 2 and 3 and in the first correction note above used joint angles with the base at its default yaw: the script set only the arm joints. The base is turned about -90 degrees in this scene, so the x and y components (and the rotation) were in the wrong frame. The z component is unaffected by the base yaw, so the z results stand.
- Corrected feedforward at 0.205 kg (full state, base included; INFERRED pose from checkpoint 11550, method unchanged): x +0.199 mm, y -2.401 mm, z +19.150 mm; rotation -3.715, -0.067, -0.020 degrees. At 0.200 kg: x +0.195, y -2.343, z +18.687 mm.
- The full state reproduces the logged pad position at tick 11550 to 0.1 mm (VERIFIED), which the earlier script did not check.
- Trim change since the close (first correction note): x +5.130, y -0.398, z +18.846 mm. Corrected difference (trim change minus feedforward): x +4.93 mm, y +2.00 mm, z -0.30 mm. The z agreement stands; the x and y mismatch is larger than the first note said in y and differs in sign there.
