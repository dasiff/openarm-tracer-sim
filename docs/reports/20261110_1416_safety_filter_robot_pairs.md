# Safety filter study: robot-vs-robot pairs at the poses we use (2026-11-10)

Read-only: robot_shapes.json, config and margins unchanged. Kinematics only (no stepping). Script (deleted 2026-11-10, superseded by the per-link and rule-check scripts; results are in this report): safety_filter_robot_pairs_check.py (log data/logs/robot_pairs3_141501.log). Scene setup shared with the pick-and-place check (experiments/safety_filter_pick_scene.py, now takes a pedestal height).

## 0. Location check
- /home/vboxuser/projects/openarm-tracer-sim-safety-filter, branch safety-filter.

## 1. Poses

- **P1, zombie pose, both arms**: the zombie pose is the home pose in this code (TASK_CFG home_q_left / home_q_right; full_check and staging start from it). So P2 (home pose) is identical to P1; the script recomputes it and confirms "identical = True".
- **P3, left arm at M1 to M9, right arm at zombie**: the IK of the pick-and-place check. Left fingers: open (0.044 m) at M1 to M3, M8, M9; closed on the beaker (0.037 m) at M4 to M7. Right fingers open.
- **P4, close-in carry with the held beaker**: this differs from the zombie pose only in the left arm (M5 carry and M6 over the plate, held). The right arm is at zombie. So P4 is the M5 and M6 rows of P3.
- **Pedestal**: 0.155 m (staging height) and 0.120 m.
- Solved status: at 0.155 all nine moments converged. At 0.120, M1 converged to 4.0 mm and M9 to 1.3 mm (kept, not within 1 mm); M2 to M8 converged.

## 2. Pedestal height does not change robot-vs-robot gaps

All arm parts and the torso ride the pedestal, so with the same joints the robot-vs-robot gaps are identical at 0.155 and 0.120 m (P1 and the M2 to M6 rows are identical). Only the moments whose IK solution changed (M1, M7, M8, M9) differ, and only by their joint solution. So the table below is given once and the 0.120 differences are listed after it.

## 3. Checked pairs within margin + 2 cm (per pose, pedestal 0.155 m)

Margins: robot vs robot 30 mm (no hand-hand pairs fell inside 20 mm at these poses). "shape" = filter shape gap, "real" = MuJoCo gap on the real geoms (scoring only).

| pose | pair | shape gap (mm) | real gap (mm) | flags |
|---|---|---|---|---|
| P1 = P2 | left_forearm_1 vs torso | +36.9 | +92.2 | BAND |
| P1 = P2 | right_upper_arm vs torso | +17.7 | +64.0 | **MARGIN** |
| P1 = P2 | right_forearm_1 vs torso | +55.9 | +92.2 | (outside band) |
| P1 = P2 | right_palm_1 vs torso | +189.7 | +199.7 | (outside band) |
| P3 M1 | right_upper_arm vs torso | +17.7 | +64.0 | **MARGIN** |
| P3 M1 | right_forearm_1 vs torso; right_palm_1 vs torso | +55.9; +189.7 | +92.2; +199.7 | outside band |
| P3 M2 | left_upper_arm vs right_shoulder_mount | +59.7 | +125.0 | outside band |
| P3 M2 | right_upper_arm vs torso | +17.7 | +64.0 | **MARGIN** |
| P3 M3 | left_upper_arm vs right_shoulder_mount | +59.6 | +125.0 | outside band |
| P3 M3 | right_upper_arm vs torso | +17.7 | +64.0 | **MARGIN** |
| P3 M4 | left_upper_arm vs right_shoulder_mount | +59.6 | +125.0 | outside band |
| P3 M4 | right_upper_arm vs torso | +17.7 | +64.0 | **MARGIN** |
| P4 M5 | left_upper_arm vs right_shoulder_mount | +56.8 | +125.0 | outside band |
| P4 M5 | right_upper_arm vs torso | +17.7 | +64.0 | **MARGIN** |
| P4 M6 | left_upper_arm vs right_shoulder_mount | +44.1 | +125.0 | outside band |
| P4 M6 | left_forearm_1 vs right_palm_1 | +43.3 | +112.8 | outside band |
| P4 M6 | left_forearm_1 vs right_finger_right | +40.5 | +103.0 | outside band |
| P4 M6 | left_forearm_1 vs torso | +56.7 | +114.0 | outside band |
| P4 M6 | right_upper_arm vs torso | +17.7 | +64.0 | **MARGIN** |
| P3 M7 | left_forearm_1 vs right_palm_1 | +30.7 | +86.8 | BAND |
| P3 M7 | left_forearm_1 vs right_finger_right | +28.5 | +83.6 | **MARGIN** |
| P3 M7 | left_forearm_1 vs torso | +39.2 | +99.1 | BAND |
| P3 M7 | left_forearm_2 vs right_palm_1 / right_finger_right | +49.6 / +50.9 | +86.8 / +83.6 | outside band |
| P3 M7 | right_upper_arm vs torso | +17.7 | +64.0 | **MARGIN** |
| P3 M8 | left_forearm_1 vs right_palm_1 | +30.5 | +86.6 | BAND |
| P3 M8 | left_forearm_1 vs right_finger_right | +28.3 | +83.5 | **MARGIN** |
| P3 M8 | left_forearm_1 vs torso | +38.4 | +98.5 | BAND |
| P3 M8 | right_upper_arm vs torso | +17.7 | +64.0 | **MARGIN** |
| P3 M9 | left_forearm_1 vs torso | +15.1 | +72.4 | **MARGIN** |
| P3 M9 | left_link7_1 vs right_palm_1 | +57.2 | +95.3 | outside band |
| P3 M9 | right_upper_arm vs torso | +17.7 | +64.0 | **MARGIN** |

Held object (M4 to M7): no held-object pair within 50 mm of anything in the other arm or the body, at any pose (the script's held count is 0 at every pose). Fingers: the right finger pairs above are the only finger pairs within 50 mm.

**Exempt pairs** (listed separately, touch rules): the same-arm pairs (about 50 to 60 per pose; all within one arm, SF-4/SF-10 family; the script's reason string mislabels them "held object vs its own hand", a label in src/safety_filter/touch.py to fix); shoulder mounts vs the torso (touch by construction, SF-8, shape −48.7 mm, real +1 mm); left upper arm vs torso (SF-8, shape +18 mm, real +64 mm); left shoulder mount vs right shoulder mount (fixed to the torso, shape −14 mm, real +62 mm); pedestal top vs torso (robot body against itself, shape −34 mm, real +8 mm).

**Pedestal 0.120 m differences** (all other rows identical):
- P1, P2, M2 to M6: identical to 0.155.
- P3 M1: right_upper_arm vs torso MARGIN (identical).
- P3 M7: left_forearm_1 vs right_finger_right BAND (+39.3 mm), and no MARGIN (0.155 had MARGIN at +28.5 mm): the M7 solution differs.
- P3 M9: left_forearm_1 vs torso BAND (+30.8 mm); no MARGIN (0.155 had MARGIN at +15.1 mm). M9 converged only to 1.3 mm.
- P3 M1 at 0.120 converged to 4.0 mm (left arm), so its row is less exact.

## 4. Summary of every MARGIN flag

No REAL-INSIDE flag at any pose or pedestal height: every MARGIN pair has real gap above its margin. So no honest shape could pass a margin-30 test on these poses; every MARGIN is a loose shape.

| # | pair (poses) | shape / real (mm) | over-coverage (mm) | what sets the over-coverage |
|---|---|---|---|---|
| 1 | right_upper_arm vs torso (P1, P2, P3 M1 to M9, P4 M5, M6; both pedestals) | +17.7 / +64.0 | 46.3 | The upper arm capsule (radius 96.7 mm, the largest over the links 1 to 3 poses; the joint 2 to joint 4 segment). The real closest pair is right_link1 mesh vs the torso mesh. The torso capsule (radius 41.6 mm, the corner of the square torso tube) also reaches out, but the upper arm capsule is the larger share (not decomposed further). |
| 2 | left_forearm_1 vs right_finger_right (P3 M7, M8, 0.155) | +28.5 / +83.6, +28.3 / +83.5 | 55 | The elbow piece of the forearm (radius 76.1 mm, 0 to 0.73 of the line) against link 6 of the left arm, which sits over the right finger at the plate. The real closest pair is link 6 mesh vs the right finger mesh. |
| 3 | left_forearm_1 vs torso (P3 M9, 0.155) | +15.1 / +72.4 | 57 | The elbow piece (radius 76.1 mm) against link 4 of the left arm; the real closest pair is link 4 mesh vs the torso mesh. At 0.120 the same pair is BAND (+30.8 mm, real +86.3). |

All three are loose shapes. The forearm elbow piece is the fattest shape near the elbow; its radius is set by the link 4 to 6 hull vertices at the elbow region (SF-14). The upper arm radius is set by links 1 to 3 (96.7 mm, unchanged since the build).

## 5. Right palm 1 vs torso (SF-17 finding corrected)

- At P1 to P4 and both pedestal heights: shape +189.7 mm, real +199.7 mm. Never within the margin or the band.
- The earlier finding in the shape refinement report (right palm 1 overlapping the torso by 20.7 mm) was measured at the all-joints-zero configuration (joint zero is not a pose we use). At the zombie/home pose the right palm is 190 mm from the torso.
- **Verdict: no problem at the poses we use.** SF-17's open item is withdrawn (the palm capsule's over-coverage matters only at joint zero, which the filter never sees). The SF-9 exemption list stays the three shapes.

## 6. Parked right arm: pushed or not

- **Yes, the filter would push the parked right arm** at P1 (zombie) and at every P3/P4 pose, both pedestal heights, via one MARGIN pair: right_upper_arm vs torso (shape +17.7 mm, margin 30 mm). The pair is active (inside margin + 1 cm), so the resolver would move the right arm to open the shape gap to margin + 2 cm (50 mm), although the real clearance is 64 mm. This is a false push caused by the upper arm capsule's over-coverage (row 1 above).
- The right arm also has MARGIN on the right fingers at P3 M7 and M8, via left_forearm_1 vs right_finger_right (row 2). This pair would push the right finger (and the left forearm) at the plate.
- Right-arm MARGIN flags at P1 and P3 are therefore: right_upper_arm vs torso (every pose); right fingers at M7 and M8 (through the left forearm).

## 7. Suggested decisions (not changed here)
1. Upper arm vs torso: the shape over-covers the torso side by 46 mm at the zombie pose. Either shrink the upper arm capsule's torso-side extent (a per-link capsule set, as for the forearm and link 7), or exempt the right upper arm from the torso like the left one, with a reason. The left upper arm is exempt by SF-8 because it overlaps by construction; the right one does not (real 64 mm), so the exemption cannot be extended without a reason.
2. Forearm elbow piece vs torso and vs the other arm's fingers at the plate: a 3-piece forearm, or a smaller elbow piece, would clear rows 2 and 3 (the rule of SF-16 applies to the pick poses only, so this is a choice for David).
3. Fix the same-arm exemption label in touch.py ("held object vs its own hand: SF-4" is printed for every same-arm pair).

## 8. Not done
- The 0.120 m IK for M1 (4.0 mm) and M9 (1.3 mm) was kept but not converged to 1 mm; the rows above are as solved.
- The upper arm's over-coverage was not split between the arm capsule and the torso capsule (estimated only).
- No change to robot_shapes.json, config or margins (read-only, as asked).
- No hand-hand pairs fell within 50 mm at these poses (the list would have shown them).
