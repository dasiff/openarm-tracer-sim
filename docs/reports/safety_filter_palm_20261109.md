# Safety filter palm shape and arm capsules, both arms (2026-11-09)

Read-only. No simulation stepping (kinematics only), no change to src/ or config. Model: `load_task(MOVED, cameras=True)`, code at current main. Each arm measured with its own geoms and its own solver key points; the other arm at its zombie pose. Nothing mirrored.

Solver-to-MuJoCo map: the same rigid fit per arm as the facts report (200 random poses of that arm, seed 20261109+1 left, +2 right). Residuals (mean / max, m): left S 0.047 / 0.047, E 0.013 / 0.017, W 0.013 / 0.021, T 0.088 / 0.101; right S 0.046 / 0.046, E 0.014 / 0.016, W 0.014 / 0.018, T 0.087 / 0.101.

## 1. Hand frame check

Hand frame: origin at the arm's solver wrist point W (mapped by the fit), axes R_fit @ R_0_7 @ C_HAND. Hand body collision geoms (fingers excluded), 50 random poses per arm.

| arm | max vertex displacement over poses (m) | mean | verdict |
|---|---|---|---|
| left | 0.0327 | (per vertex mean not reported) | **not near zero: steps 2 and 4 skipped** |
| right | 0.0278 | | **not near zero: steps 2 and 4 skipped** |

Diagnosis (checked, not part of the rule): the same hand vertices in MuJoCo's own link-7 frame have zero spread (0.000000 m, left). The solver's R_0_7 equals MuJoCo's link-7 rotation up to the identity (residual 0.0000), so the orientation is right. The spread comes from the origin: W is mapped through the rigid fit, and the fit leaves a pose-dependent residual of 1–2 cm at W. The hand frame built on that origin is not rigid to within the required tolerance. The fingers and palm check in step 3d use the same frame, so their numbers carry this error and are flagged below.

## 2. Palm capsule fits: skipped

Skipped under the rule for steps 2 and 4 (the hand frame is not rigid). No palm-fit numbers are reported. The fits were computed in the scratch run but are not used here, because they are expressed in the frame that failed the check.

## 3. Arm capsules (200 random poses per arm, rigid fit method)

Radii are maxima over the 200 poses, from mesh vertices of the named geoms.

| arm | part | segment or end points | radius (m) | geom that sets it | Kong's radius (m) | ours / Kong |
|---|---|---|---|---|---|---|
| left | upper arm (links 1–3) | shoulder S to elbow E (solver, mapped) | 0.0728 | openarm_left_link1_collision | upper 0.050 | 1.46 |
| right | upper arm (links 1–3) | S to E | 0.0729 | openarm_right_link1_collision | upper 0.050 | 1.46 |
| left | shoulder mount (link 0) | fixed to torso, torso-frame capsule A (0.0013, 0.0634, 0.7172), B (0.0013, 0.0635, 0.6788) | 0.0598 | openarm_left_link0_collision | shoulder 0.075 | 0.80 |
| right | shoulder mount (link 0) | fixed to torso, A (0.0013, −0.0598, 0.7141), B (0.0013, −0.0614, 0.6808) | 0.0601 | openarm_right_link0_collision | shoulder 0.075 | 0.80 |
| left | forearm (links 4–5) | elbow E to wrist W | 0.0687 | openarm_left_link4_collision | lower 0.050 | 1.37 |
| right | forearm (links 4–5) | E to W | 0.0674 | openarm_right_link4_collision | lower 0.050 | 1.35 |
| left | fingers (both, open and closed) | hand axis from W (hand frame, along +x): start 0.0823 m, end 0.2130 m | 0.1019 | fingers, max over poses | hand 0.080 | 1.27 |
| right | fingers (both, open and closed) | start 0.0839 m, end 0.2111 m | 0.0986 | fingers, max over poses | hand 0.080 | 1.23 |

Caveat on the finger row: it is computed in the hand frame that failed the spread check, so the start, end and radius carry its origin drift (about 3 cm over poses). Read it as indicative only.

The torso (Kong 0.060) is not an arm part here; the mount row above is the shoulder-mount capsule of step 3b. Our mount capsule (0.060) is narrower than Kong's shoulder capsule (0.075).

## 3e. Forearm looseness and the 2-capsule forearm (follow-up)

Method: for each of the 200 poses, the forearm (links 4–5) vertices are expressed in that pose's own elbow-to-wrist frame (origin E, axis along E→W, which moves with the arm). Ten evenly spaced points from the elbow (t = 0 to 1, along the mean E→W length 0.2160 m). At each point, the real radial extent is the largest distance from the axis of the forearm vertices in a slab of width (length/9), taken as the maximum over the 200 poses. Gap = the 3c radius minus that extent. Positive = the capsule surface is outside the geometry.

| point | from elbow (m) | left extent (m) | left gap (m) | right extent (m) | right gap (m) |
|---|---|---|---|---|---|
| 1 | 0.000 | 0.0650 | 0.0036 | 0.0638 | 0.0036 |
| 2 | 0.024 | 0.0686 | 0.0001 | 0.0674 | −0.0000 |
| 3 | 0.048 | 0.0687 | −0.0000 | 0.0673 | 0.0001 |
| 4 | 0.072 | 0.0655 | 0.0031 | 0.0647 | 0.0027 |
| 5 | 0.096 | 0.0605 | 0.0081 | 0.0587 | 0.0088 |
| 6 | 0.120 | 0.0622 | 0.0064 | 0.0607 | 0.0067 |
| 7 | 0.144 | 0.0629 | 0.0058 | 0.0613 | 0.0061 |
| 8 | 0.168 | 0.0604 | 0.0083 | 0.0596 | 0.0078 |
| 9 | 0.192 | 0.0591 | 0.0095 | 0.0565 | 0.0109 |
| 10 | 0.216 | 0.0584 | 0.0103 | 0.0555 | 0.0119 |

Minimum gap: left −0.0000 at point 3 (elbow side, 0.048 m); right −0.0000 at point 2. The forearm is tight to its 3c capsule near the elbow and loose by 0.8–1.2 cm toward the wrist on both arms.

Two-capsule forearm fit (heuristic, canonical per-pose E→W frame, union of the 200 poses; end points in that frame, not world):

| arm | capsule | A (m) | B (m) | radius (m) | total volume (m³) |
|---|---|---|---|---|---|
| left | 1 | (−0.0201, 0.0006, 0.0163) | (0.0137, −0.0305, −0.0044) | 0.0650 | |
| left | 2 | (−0.0031, −0.0061, 0.2040) | (0.0090, 0.0106, −0.0006) | 0.0635 | 0.005496 |
| right | 1 | (0.0225, −0.0181, −0.0385) | (−0.0086, 0.0085, 0.2226) | 0.0616 | |
| right | 2 | (−0.0486, −0.0053, −0.0011) | (0.0178, 0.0376, 0.0154) | 0.0598 | 0.005930 |

Comparison: the single 3c capsule (radius 0.0687 left, 0.0674 right, length 0.2160 m) has volume 0.004558 m³ (left) and 0.004369 m³ (right). The 2-capsule heuristic is larger than the single capsule on both arms, so it does not improve on one capsule. The heuristic is not a global optimum, and a second capsule can always be made degenerate (zero volume, inside the first), so the single capsule is an upper bound on the true 2-capsule minimum. Conclusion: the forearm is well described by one capsule of its 3c radius; the 2-capsule fit gives no gain with this method.

## 4. Wrist coverage: skipped

Skipped under the rule (hand frame not rigid, step 4). Not reported.

## 5. Grasp clearance: not done

The palm-fit gaps (k = 1, 2, 3) depend on step 2, which was skipped. The real-mesh and finger-capsule gaps were not computed in this run (the level-hand pose check and the finger-closing solve were not run). See Not done.

## Left vs right differences over 5 mm

| quantity | left | right | difference (mm) |
|---|---|---|---|
| upper arm radius | 0.0728 | 0.0729 | 0.1 |
| shoulder mount radius | 0.0598 | 0.0601 | 0.3 |
| forearm radius | 0.0687 | 0.0674 | 1.3 |
| finger capsule radius (flagged, see caveat) | 0.1019 | 0.0986 | 3.3 |
| finger capsule start | 0.0823 | 0.0839 | 1.6 |
| finger capsule end | 0.2130 | 0.2111 | 1.9 |
| forearm 2-capsule volume (m³) | 0.005496 | 0.005930 | (not a radius or gap; differs 0.4e-3) |

No radius or gap differs by more than 5 mm between the arms. The finger capsule is the closest to the threshold but is flagged as unreliable.

## Not done

- Step 2 (palm capsule fits k = 1, 2, 3) and step 4 (wrist coverage): skipped because the hand-frame spread is 3.3 cm (left) and 2.8 cm (right), not near zero (your rule). The fix needed is in the solver-to-MuJoCo origin mapping at W (a rigid fit does not hold to the required tolerance); MuJoCo's own link-7 frame is rigid (zero spread).
- Step 5 (grasp clearance, real mesh, k fits, finger capsule, open and closed): not run. It needs the pad centre per arm and the level-hand pose check (`src/gripper_geometry.py`), and the finger-closing solve to the cylinder. The k-fit gaps also need step 2.
- The finger capsule (3d) is reported but computed in the failed frame; it is indicative only.
- Commit of this report: see the final message.
