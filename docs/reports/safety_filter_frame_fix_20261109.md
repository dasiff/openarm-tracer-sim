# Safety filter: rotation from directions, round trip, branch jump origin, hand with link 7, per-finger capsules (2026-11-09)

Read-only, kinematics only (no MuJoCo stepping), no change to src/ or config. Scratch script (not committed): `scratchpad/frame_fix.py`; results `scratchpad/frame_fix.json`. The 220 poses per arm are the same as the own-model study: 200 random within limits, plus the 20 earlier left-arm round-trip poses (left) or 20 new random poses (right). The other arm is at its zombie pose.

## 1. Rotation from directions only

Pairs (solver → our world): solver unit(E − S) with our joint 3 axis; solver unit(W − E) with our joint 5 axis; the three columns of the solver's `R_0_7 @ C_HAND` with the three columns of our `R7 @ C_HAND`. Best-fit rotation R (Kabsch, over all pairs, no points used) with R·p ≈ q.

| arm | direction type | mean residual (deg) | max residual (deg) |
|---|---|---|---|
| left | E − S vs joint 3 axis | 0.0000 | 0.0000 |
| left | W − E vs joint 5 axis | 0.0000 | 0.0000 |
| left | hand column x | 0.0000 | 0.0000 |
| left | hand column y | 0.0000 | 0.0000 |
| left | hand column z | 0.0000 | 0.0000 |
| right | E − S vs joint 3 axis | 0.0000 | 0.0000 |
| right | W − E vs joint 5 axis | 0.0000 | 0.0000 |
| right | hand column x | 0.0000 | 0.0000 |
| right | hand column y | 0.0000 | 0.0000 |
| right | hand column z | 0.0000 | 0.0000 |

The fitted R is the identity on both arms (diagonal 1 to 1e-6, off-diagonal below 1e-6). So the solver's base frame has the same orientation as our world frame, and its directions agree with our axes exactly. (The 4-point, position-based fit used earlier gave a non-identity rotation; its error is what caused the 0.01–0.02 rad mismatch in the own-model report. I did not compute that rotation's angle here.)

## 2. Round trip with that rotation (axes variant)

Inputs: S = the solver's own S at q (arbitrary, as the task allowed), E = S + 0.3u, W = E + 0.3l, u = Rᵀ(joint 3 axis), l = Rᵀ(joint 5 axis), hand = Rᵀ·R7·C_HAND. Avoidance off, current pose = q. Error = max over the measured arm's 7 joints.

| arm | reset | within 1e-3 rad | within 1e-4 rad | branch jumps (> 0.1 rad) | non-jump median (rad) | non-jump max (rad) | joints with error ≥ 1e-4 (count) |
|---|---|---|---|---|---|---|---|
| left | False | 198 / 220 | 197 / 220 | 19 | 3.7 × 10⁻⁶ | 0.0750 | j1 2, j3 2 |
| left | True (q_reset = q) | 198 / 220 | 197 / 220 | 19 | 3.7 × 10⁻⁶ | 0.0750 | j1 2, j3 2 |
| right | False | 198 / 220 | 197 / 220 | 22 | 3.7 × 10⁻⁶ | 1.29 × 10⁻⁴ | j1 1 |
| right | True (q_reset = q) | 198 / 220 | 197 / 220 | 22 | 3.7 × 10⁻⁶ | 1.29 × 10⁻⁴ | j1 1 |

The non-jump poses with an error above 1e-4 (left): pose 2 (0.0750 rad, joint 3), pose 118 (0.0091 rad, joint 3), pose 215 (0.0015 rad, joint 1); right: pose 61 (1.3 × 10⁻⁴ rad, joint 1). Each one matches the solver's own round trip to within 1e-5 rad (table in section 3), so none is caused by our input.

Reset setting made no difference: the solver gives the same answers with and without the reset.

Compared with the previous study (rotation from points, 34 and 25 jumps, no pose within 1e-3): the direction-based rotation fixes the mismatch on the non-jump poses. Within 1e-3 rad rises from 0 to 198 / 220 on both arms.

## 3. Branch jump origin

For each pose, the solver's own round trip: its own FK of q fed back in (`SafetyFilter.frame(q)`), current = q, reset = True, as in the Kong round-trip report.

| arm | jumps with our input (reset False, same for True) | jumps with its own input | both | only with our input | only with its own input |
|---|---|---|---|---|---|
| left | 19 | 19 | 19 | 0 | 0 |
| right | 22 | 22 | 22 | 0 | 0 |

Every jump we see is a jump the solver makes on its own round trip, on the same pose. Our input adds no branch jumps. The jumps are a property of the solver's IK at those poses (the module docstring's caveat about joint limits), not of the directions we pass in.

## 4. Hand group with link 7: palm fit (k = 1, 2, 3), link 7 frame

Vertices: the hand body and link 7 collision geoms together, in the link 7 frame (rigid, so one pose). Minimum-volume fit as in the palm study (heuristic, restarts plus Nelder–Mead on the end points).

| arm | k | capsule | end A (link 7 frame, m) | end B (link 7 frame, m) | radius (m) | total volume (m³) |
|---|---|---|---|---|---|---|
| left | 1 | 1 | (−0.0000, 0.0233, 0.1305) | (−0.0000, −0.0594, 0.0405) | 0.0657 | 0.002847 |
| left | 2 | 1 | (0.0002, −0.0589, 0.0979) | (−0.0002, 0.0591, 0.0973) | 0.0287 | 0.000596 |
| left | 2 | 2 | (0.0000, 0.0001, 0.0357) | (0.0000, −0.0168, 0.0054) | 0.0289 | |
| left | 3 | 1 | (0.0000, 0.0001, 0.0357) | (0.0000, −0.0168, 0.0054) | 0.0289 | 0.000211 |
| left | 3 | 2 | (−0.0310, −0.0063, 0.0983) | (0.0003, 0.0850, 0.1083) | 0.0053 | |
| left | 3 | 3 | (0.0311, 0.0061, 0.0982) | (−0.0003, −0.0850, 0.1084) | 0.0053 | |
| right | 1 | 1 | (−0.0000, −0.0233, 0.1305) | (−0.0000, 0.0594, 0.0405) | 0.0657 | 0.002847 |
| right | 2 | 1 | (−0.0002, −0.0591, 0.0973) | (0.0002, 0.0589, 0.0979) | 0.0287 | 0.000596 |
| right | 2 | 2 | (0.0000, −0.0001, 0.0357) | (0.0000, 0.0168, 0.0054) | 0.0289 | |
| right | 3 | 1 | (0.0000, 0.0312, 0.0109) | (0.0000, −0.0660, 0.0936) | 0.0286 | 0.000436 |
| right | 3 | 2 | (−0.0292, −0.0037, 0.0979) | (0.0003, 0.0851, 0.1084) | 0.0053 | |
| right | 3 | 3 | (0.0279, 0.0081, 0.0960) | (0.0280, −0.0078, 0.0960) | 0.0016 | |

Caveat on k = 3: the left and right fits found different decompositions with different volumes (0.000211 against 0.000436 m³). The heuristic is not reliable at k = 3; the left and right k = 3 results should not be compared directly.

### Grasp clearance, hand + link 7 (cylinder r 0.035, h 0.100, axis along the hand's up direction, pad centre at 52% height; zombie pose; fingers do not change these gaps)

| arm | shape | gap open (m) | gap closed (m) |
|---|---|---|---|
| left | palm + link 7, k = 1 | −0.0382 | −0.0382 |
| left | palm + link 7, k = 2 | 0.0272 | 0.0272 |
| left | palm + link 7, k = 3 | 0.0487 | 0.0487 |
| left | real hand + link 7 mesh | 0.0554 | 0.0554 |
| right | palm + link 7, k = 1 | −0.0382 | −0.0382 |
| right | palm + link 7, k = 2 | 0.0272 | 0.0272 |
| right | palm + link 7, k = 3 | 0.0492 | 0.0492 |
| right | real hand + link 7 mesh | 0.0554 | 0.0554 |

The k = 1 capsule penetrates the cylinder by 3.8 cm; k = 2 and k = 3 stay clear. The real hand + link 7 mesh is 5.5 cm from the cylinder surface.

## 5. One capsule per finger body (own finger frame)

Each finger body's collision geoms, in that finger's own body frame (rigid, so the same for every state), minimum-volume single capsule.

| arm | finger body | end A (finger frame, m) | end B (finger frame, m) | radius (m) | volume (m³) |
|---|---|---|---|---|---|
| left | openarm_left_left_finger | (−0.0100, −0.0049, 0.0628) | (0.0049, 0.0258, −0.0033) | 0.0318 | 0.000372 |
| left | openarm_left_right_finger | (−0.0100, 0.0049, 0.0628) | (0.0049, −0.0258, −0.0033) | 0.0318 | 0.000372 |
| right | openarm_right_left_finger | (−0.0100, −0.0049, 0.0628) | (0.0049, 0.0258, −0.0033) | 0.0318 | 0.000372 |
| right | openarm_right_right_finger | (−0.0100, 0.0049, 0.0628) | (0.0049, −0.0258, −0.0033) | 0.0318 | 0.000372 |

### Clearance of each finger capsule to the cylinder

| arm | finger | gap open, q = 0.0440 m (m) | gap closed, q = 0.0376 m (m) |
|---|---|---|---|
| left | left finger | −0.0170 | −0.0226 |
| left | right finger | −0.0170 | −0.0226 |
| right | left finger | −0.0170 | −0.0226 |
| right | right finger | −0.0170 | −0.0226 |

Both finger capsules penetrate the cylinder in both states (1.7 cm open, 2.3 cm closed). The capsule radius (0.032 m) is about the finger's full width, so the capsules overlap the cylinder where the pads are. Closing the fingers deepens the overlap, as expected. The per-finger capsule is too large to use for clearance at the grasp.

## Left vs right

- Round trip, branch jumps and the rotation: identical in the reported figures (the rotation is the identity on both arms).
- Hand + link 7 palm fit, k = 1 and 2: identical to the reported precision. k = 3: different decompositions and volumes (see caveat above).
- Grasp clearance with the hand + link 7 mesh: k = 3 differs by 0.5 mm (0.0487 vs 0.0492). All other differences are below 0.5 mm, so none exceeds 5 mm.
- Finger capsules: identical on both arms (within the printed precision).

## Not done

- A test of the round trip with an exact solver frame beyond the identity result: not needed, since the rotation is the identity and the solver's own round trip agrees with ours on every pose.
- k = 3 palm fits: the heuristic gave different decompositions on the two arms; a better k = 3 optimiser was not written.
- Per-finger clearance uses each finger body's own capsule. Whether a single capsule per finger is worth keeping, given it penetrates the cylinder, is a design question not settled here.
- Commit of the scratch script: not committed (it is in the scratchpad).
