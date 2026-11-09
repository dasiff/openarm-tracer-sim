# Safety filter: the solver on our joint layout, capsules on our own model, grasp clearance in the hand frame (2026-11-09)

Read-only, kinematics only (no MuJoCo stepping), no change to src/ or config. Model: `load_task(MOVED, cameras=True)`, code at main. Scratch script (not committed): `scratchpad/own_model.py`, results `scratchpad/own_model.json`.

## 1. Our-directions round trip

### What was passed to the solver

- Solver frame: the wrapper's own frame (`src/safety_filter.py`, `SafetyFilter.frame`): base pose from `robot_fk_at` (`R_base_body` = identity, `p_base_body` = (0, 0, 0.698)), `engaged=True`, `reset=False`, collision avoidance OFF (`SafetyFilter(collision_avoidance=False)`).
- Our model's world points and directions are mapped into the solver frame with the rigid rotation R and translation t of the 4-point fit (same fit as the facts report, seeds 20261109+1 left, +2 right). Directions use the rotation only: d_solver = Rᵀ d_world. Points use x_solver = Rᵀ(x_world − t).
- Hand: `R_world_wrist = Rᵀ · R7_world · C_HAND`, where R7_world is our link 7 rotation and C_HAND is the wrapper's constant (`src/safety_filter.py:57-60`).
- Variant "axes" (as the task asked): S = joint 1 anchor (converted); u = world direction of joint 3 axis; l = world direction of joint 5 axis; E = S + 0.3 u; W = E + 0.3 l (in the solver frame).
- Variant "anchors": S = joint 1 anchor, E = joint 4 anchor, W = joint 7 anchor (all converted).
- The other arm is at its zombie pose (its solver frame is the wrapper's own FK at zombie).
- Current pose = q for the measured arm, zombie for the other arm.

### Sign convention check

The solver's own FK (`robot_fk_at`) gives E − S and W − E at the same poses. Our joint 3 axis agrees with the solver's E − S, and our joint 5 axis agrees with the solver's W − E. Mean |dot| = 1.000 on both arms, no flips. Neither axis was flipped.

### Results (220 poses per arm: 200 random within limits, plus 20 earlier poses for the left arm and 20 new random poses for the right)

| arm | variant | poses within 1e-3 rad | branch jumps (> 0.1 rad) | others (1e-3 to 0.1 rad) | median max-diff (rad) | max (rad) | joints in the other poses (count) | joints in the jumps (count) |
|---|---|---|---|---|---|---|---|---|
| left | axes | 0 / 220 | 34 | 186 | 0.0169 | 2.485 | j3 66, j2 80, j1 39, j5 1 | j3 21, j5 8, j1 5 |
| left | anchors | 0 / 220 | 220 | 0 | 0.364 | 2.028 | (all jumps) | j3 74, j5 70, j4 20, j2 21, j1 21, j7 10, j6 4 |
| right | axes | 0 / 220 | 25 | 195 | 0.0091 | 1.586 | j1 104, j2 39, j3 52 | j3 16, j5 7, j1 2 |
| right | anchors | 0 / 220 | 220 | 0 | 0.330 | 2.636 | (all jumps) | j3 79, j5 56, j2 30, j1 28, j4 13, j7 9, j6 5 |

### Verdict

- The solver's joint layout matches ours in direction: the axis signs agree, and the axes-built inputs bring the solver's answer to within about 1–2 × 10⁻² rad of our joints for most poses (median 0.017 left, 0.009 right).
- It does not match exactly: no pose is within 1e-3 rad. About 15% of poses (34 left, 25 right) land on another branch (> 0.1 rad).
- Positional offsets matter a great deal: with the anchors (true joint positions, not the solver's own keypoint definitions), every pose jumps on both arms.
- Not isolated: the 1e-2 rad mismatch could come from (i) the rotation of the 4-point fit used to map directions into the solver frame (its residuals are 1–2 cm, so the rotation carries error of similar order), (ii) the solver's IK tolerance, or (iii) `reset=False`, which carries the previous solution into each pose. I did not test (iii) with `reset=True` per pose, or (i) with an exact frame. See Not done.

## 2. Capsules on our own model

### 2a. Hand, in the link 7 frame (fingers excluded)

- Spread over 50 poses: left 7.5 × 10⁻¹⁶ m, right 5.9 × 10⁻¹⁶ m (zero, as expected: the hand is rigid to link 7).
- Palm capsules (minimum total volume, enclosing the hull vertices of both arms' hand collision geoms, which are the same set for every pose), in the link 7 frame (m):

| arm | k | capsule | end A | end B | radius | total volume (m³) |
|---|---|---|---|---|---|---|
| left | 1 | 1 | (0.0000, −0.0590, 0.0975) | (0.0000, 0.0590, 0.0975) | 0.0287 | 0.000403 |
| left | 2 | 1 | (−0.0193, 0.0161, 0.0918) | (0.0073, −0.0780, 0.1057) | 0.0146 | 0.000159 |
| left | 2 | 2 | (−0.0044, 0.0741, 0.1044) | (0.0184, −0.0159, 0.0906) | 0.0150 | |
| left | 3 | 1 | (−0.0168, 0.0280, 0.0958) | (−0.0218, −0.0273, 0.0947) | 0.0095 | 0.000104 |
| left | 3 | 2 | (−0.0005, −0.0244, 0.0901) | (0.0050, 0.0860, 0.1071) | 0.0106 | |
| left | 3 | 3 | (0.0240, 0.0224, 0.0939) | (−0.0005, −0.0868, 0.1019) | 0.0100 | |
| right | 1 | 1 | (0.0000, −0.0590, 0.0975) | (0.0000, 0.0590, 0.0975) | 0.0287 | 0.000403 |
| right | 2 | 1 | (−0.0193, 0.0161, 0.0918) | (0.0073, −0.0780, 0.1057) | 0.0146 | 0.000159 |
| right | 2 | 2 | (−0.0044, 0.0741, 0.1044) | (0.0184, −0.0159, 0.0906) | 0.0150 | |
| right | 3 | 1 | (−0.0005, 0.0868, 0.1019) | (0.0240, −0.0224, 0.0939) | 0.0100 | 0.000103 |
| right | 3 | 2 | (0.0055, −0.0820, 0.1082) | (−0.0023, 0.0259, 0.0866) | 0.0109 | |
| right | 3 | 3 | (−0.0213, 0.0295, 0.0951) | (−0.0182, −0.0244, 0.0966) | 0.0090 | |

The fits are the minimum-volume heuristic from the palm study (alternating assignment and Nelder-Mead on end points, restarted), not a proven global optimum. The palm capsules are all small (radius ≤ 0.029 m) because the hand's collision mesh is small relative to link 7.

### 2b. Fingers, one capsule in the link 7 frame (both fingers, fully open and fully closed)

| arm | end A | end B | radius | volume (m³) |
|---|---|---|---|---|
| left | (0.0023, 0.0513, 0.1412) | (0.0015, −0.0520, 0.1412) | 0.0538 | 0.001595 |
| right | (0.0026, 0.0461, 0.1398) | (0.0031, −0.0458, 0.1399) | 0.0553 | 0.001590 |

The capsule is oriented along the finger-separation (closing) direction, not along the pointing axis. It is wide: the two fingers span about ±5 cm in y, so one capsule covering both must be about 5.5 cm in radius.

### 2c. Arm capsules, on our joint anchors, over 200 poses per arm

| arm | part | segment (our model) | radius (m) | geom that sets it | Kong's radius (m) |
|---|---|---|---|---|---|
| left | upper arm (links 1–3) | joint 2 anchor → joint 4 anchor | 0.0967 | openarm_left_link1_collision | upper 0.050 |
| right | upper arm (links 1–3) | joint 2 anchor → joint 4 anchor | 0.0967 | openarm_right_link1_collision | upper 0.050 |
| left | forearm (links 4–6) | joint 4 anchor → joint 7 anchor | 0.0761 | openarm_left_link4_collision | lower 0.050 |
| right | forearm (links 4–6) | joint 4 anchor → joint 7 anchor | 0.0761 | openarm_right_link4_collision | lower 0.050 |
| left | shoulder mount (link 0, torso frame, k=1) | A (0.0013, 0.0634, 0.7172) → B (0.0013, 0.0635, 0.6788) | 0.0598 | openarm_left_link0_collision | shoulder 0.075 |
| right | shoulder mount (link 0, torso frame, k=1) | A (0.0013, −0.0598, 0.7141) → B (0.0013, −0.0614, 0.6808) | 0.0601 | openarm_right_link0_collision | shoulder 0.075 |

Forearm-to-hand coverage: the link 7 collision geoms are NOT fully inside the forearm capsule on either arm. The largest stick-out is 0.0226 m, on `openarm_left_link7_collision` (right arm: `openarm_right_link7_collision`, same value).

### Comparison with Kong's radii

Every arm radius is larger than Kong's: upper 1.9× (0.0967 vs 0.050), forearm 1.5× (0.0761 vs 0.050), shoulder mount 0.8× (0.0598 vs 0.075; Kong's shoulder is the same-named capsule between the two shoulders, which is not our link 0 alone). Our segments are on our own joint anchors, which is the point of the study; the numbers reflect the geometry, not the solver's points.

## 3. Grasp clearance in the hand frame (no arm IK)

Cylinder: radius 0.035 m, height 0.100 m, axis along the hand's up direction (cross product of the approach axis and the closing axis, from `src/gripper_geometry.py` `gripper_frame`), through the finger pad centre, with the pad centre at 52% of the height. The hand is not level at the zombie pose (approach axis z = −0.042), so the cylinder is tilted with the hand; the task asked for the hand frame, not a level hand.

Fingers: open = upper joint limit (q = 0.0440 m); closed to the cylinder = the joint value at which the inner faces are 0.070 m apart (q = 0.0376 m, found by bisection, `palm_redo.faces_sep`). The palm and hand gaps do not depend on the finger state; the finger mesh gap does.

| arm | shape | gap open (m) | gap closed (m) |
|---|---|---|---|
| left | palm k = 1 | 0.0274 | 0.0274 |
| left | palm k = 2 | 0.0438 | 0.0438 |
| left | palm k = 3 | 0.0477 | 0.0477 |
| left | real hand mesh | 0.0554 | 0.0554 |
| left | finger capsule (2b) | −0.0415 | −0.0415 |
| left | real finger mesh (extra) | 0.0063 | −0.0000 (touching) |
| right | palm k = 1 | 0.0274 | 0.0274 |
| right | palm k = 2 | 0.0438 | 0.0438 |
| right | palm k = 3 | 0.0477 | 0.0477 |
| right | real hand mesh | 0.0554 | 0.0554 |
| right | finger capsule (2b) | −0.0416 | −0.0416 |
| right | real finger mesh (extra) | 0.0063 | −0.0000 (touching) |

Reading the table: the finger capsule overlaps the cylinder by about 4 cm in both states. Its radius (0.054 m) and its position across the closing axis put its axis through the gap between the fingers, where the cylinder sits, so the capsule is not usable as a finger model at the grasp. The real finger meshes come within 6 mm when open and touch when closed.

## Left vs right (differences over 5 mm)

Differences over 5 mm, listed explicitly:
- Finger capsule (2b), end A y: left 0.0513 m, right 0.0461 m (difference 5.2 mm).
- Finger capsule (2b), end B y: left −0.0520 m, right −0.0458 m (difference 6.2 mm).

Other differences are below 5 mm: the finger capsule radius (0.0538 vs 0.0553, 1.5 mm), the finger capsule end A x (0.3 mm), the shoulder mount radius (0.3 mm), and the palm k = 3 radii (within 0.6 mm; the capsules are listed in a different order on the two arms). The upper arm, forearm, palm k = 1 and 2, hand mesh and grasp gaps agree to the printed precision. The arms are not exactly mirror images in the finger capsule, so each arm's own value should be used.

## Not done

- Test of the two possible causes of the round-trip mismatch: `reset=True` per pose (to remove the carry-over from `reset=False`), and an exact solver-to-MuJoCo frame (the rotation used here carries the 4-point fit's error). Not run; the mismatch is not isolated.
- The 20 right-arm poses are new draws; the 20 left-arm poses are the earlier draws. The task asked for the earlier poses on both arms, but no earlier right-arm poses exist.
- The "anchors" variant is reported as all-jump. I did not test whether a different branch or a tolerance change would make it match; that is outside the question asked.
- The palm capsule fits are a heuristic minimum (restarts plus Nelder-Mead), not a global optimum.
- No commit of the scratch script (it is in the scratchpad, not the repo).
