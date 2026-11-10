# SEW-Mimic paper notes (arXiv 2602.01632)

Reference note for the safety filter (design plan section 16, rows SF-1 to SF-7 and M285 to M298 in [mpvi_decisions.md](mpvi_decisions.md)). Written in our own words; no passages are copied.

**How this was read (caveat):** the text was obtained from the arXiv HTML page through an automated summariser, in two passes. Section and algorithm numbers below come from that summary and have not been checked against the PDF. The Section VI-C numbers appeared in the first pass only; the second pass reported that section as absent. Treat every number here as unverified until David checks it against the paper. The note is not an excerpt: paper facts that are not in `mpvi_paper_excerpts.md` are marked as such (design plan section 0).

## 1. Filter loop (Sec. V, Algorithm 4)

- The paper's safety filter sits after the retargeting step. For each desired arm pose it builds capsules from the keypoints of the desired pose, nudges them apart with XPBD-style iterations (Algorithm 10), checks for collisions, and then recovers the keypoints and tool orientation for SEW-Mimic on each arm.
- If no collision-free pose is found, the paper returns the original pose.
- **We follow:** the idea of a filter after the command and before the joints go out; a per-arm loop; returning the last good pose when the push fails (M295: hold still).
- **We changed:** we do not use the paper's failure fallback to the original pose; we hold the last filtered command (M295). Our placement is every control tick, after the helpers and before the sag trim (M292). Our keypoints come from our own MuJoCo model, not the solver's key points (SF-1, which supersedes M286). Our shapes enclose the real geometry, and the filter gets its margins from config (M288).

## 2. Continuous-time check (Appendix E.2, Algorithm 9)

- The paper interpolates linearly from the current keypoints to the desired ones. The number of interpolation steps depends on the largest keypoint move relative to the capsule radius (rounded up). The first colliding interpolated pose is returned, so a limb cannot jump through another in one step. If none collides, the final pose is used.
- **We follow:** the goal (no jumping through an obstacle in one step).
- **We changed:** our shortening rule is M293 (step back toward the last filtered pose) and our approach-speed rule is the gap rule M294 (30 per second times the gap above the margin, provisional). We have not built the interpolation check in Algorithm 9 as written; whether it is needed is open until the Stage 2 comparison.

## 3. Push step and length restore (Appendix E.3, Algorithms 10 and 11)

- Algorithm 10 is the push: for each capsule pair it checks distance and hysteresis thresholds; for penetrating pairs it computes a violation and accumulates push directions on the links. The elbow is moved for the upper arm; both ends are moved for the lower arm and hand, with the wrist weighted differently. A multiplier with a compliance term is clamped at zero, and the capsules are rebuilt.
- Algorithm 11 restores each link to its original length after the push (XPBD-style, endpoint weights, near-degenerate links skipped).
- **We follow:** the push as a per-pair correction, and restoring the upper arm and forearm lengths each round (M293).
- **We changed:** no compliance term (M293 says "no compliance"); the lever rule for pushing along a link is ours, not the paper's (M293, 30% from end A gives end A 70%); push directions come from the closest points on the two shapes (M293). Our round limit is 10 with a 0.5 mm stop (M293). The rule "elbow only for the upper arm, hand contacts never tilt the hand" (M293) is ours.
- Open point for David: the paper's weight on the wrist term looks, in the summary, as if it is near zero for unit normals. We do not use that weight.

## 4. Tool re-aim (Algorithm 12, RecoverTool): replaced

- The paper rotates the tool x-axis toward a desired direction (axis from the cross product, angle from the dot product). That is undefined when the two vectors are parallel or opposite.
- **We replace it.** The solver receives the commanded hand orientation, which keeps the hand level (M295), and the hand orientation is our link 7 orientation times C_HAND (SF-1). We do not re-aim the tool inside the filter.

## 5. Contact normal formula (Appendix E.1): not used

- E.1 gives the signed capsule distance (negative when the capsules intersect) and approximates each capsule's contact normal from the closest points on the two segments, scaled by the closest-point parameter.
- The notation looks inconsistent in the summary: the scaling by the parameter is not explained, the normal for one capsule uses a difference of the two closest points with mixed index labels, and Algorithm 10 uses one index where E.1 uses two.
- **We do not use it.** Our distance code returns the gap, the closest points and the push direction directly (task brief, distance functions), and the push direction is the line between the closest points (M293). The formula is to be checked against the PDF only if we later want to compare.

## 6. Capsule radii (Appendix I): ours differ

- The paper gives RB-Y1 radii: torso 0.15 m, shoulder 0.12 m, upper arm 0.06 m, lower arm 0.065 m, wrist 0.07 m. Capsule lengths come from the keypoints of each link.
- **Ours:** sizes come from our robot model by a build tool (SF-5), and the shapes must enclose the real geometry (M288: "never smaller"). The upper arm capsule is fat (about 9.7 cm, accepted, SF-3). The hand uses link 7 plus two palm capsules and one capsule per finger (SF-3). The paper's radii are for a different robot and are not used.

## 7. Reported results (Sec. VI-C): unconfirmed

- The first pass reported a rolling punch on the RB-Y1 (both arms, 10 rotations in 10 seconds). Without the filter: alignment error 0, about 0.8 ms per solve, self-collision in nearly half the retargeted poses. With the filter: alignment error 0.019, about 4.0 ms per solve, self-collision in about 1.3% of poses.
- The second pass did not find Section VI-C. The numbers are not confirmed. Not used for any decision.
- **Our test plan** (M297 test list stands; stages per SF-7): the rolling punch is one of our Stage 2 and Stage 3 motions, scored against MuJoCo ground truth, not against the paper's numbers. Pass lines stay provisional (M297).

## Summary of what we follow and what we changed

| part | follow | change | rows |
|---|---|---|---|
| filter loop (Sec. V, Alg. 4) | filter after the command, per arm | hold last filtered command on failure; our own model for keypoints | M292, M295, SF-1 |
| continuous check (App. E.2, Alg. 9) | goal: no jump through an obstacle | shortening and gap rules are ours; interpolation not yet built | M293, M294 |
| push and length restore (App. E.3, Alg. 10 and 11) | per-pair push; length restore | no compliance; lever rule and round limit are ours | M293 |
| tool re-aim (Alg. 12) | none | replaced by the commanded level hand | M295, SF-1 |
| contact normal (App. E.1) | none | not used; direction from closest points | M293 |
| capsule radii (App. I) | none | ours from our model, fat upper arm | M288, SF-3, SF-5 |
| results (Sec. VI-C) | none | unconfirmed; not used | M297 |

Stage order and the rename: SF-7 and SF-6.
