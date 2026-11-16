# Never-close pairs by divide and conquer: soundness argument

Standing reference (not a dated report). It says why a pair declared never-close by `tools/never_close_dc.py` can never come within
margin + 1 cm inside the joint limits. If the method or any assumption below changes, this file must be updated in the same commit as the code.
First written 2026-11-15 (Part B of the safety filter speed task), before the tool. Decision rows: SF-27 (design), SF-75 onward.

## (1) The lower bound
Pair of filter shapes S1, S2. A capsule is all points within r of a segment [A, B]; a box is {C + R u : |u_i| <= h_i}. "Core" = the segment, or the solid box (r = 0). Each shape is fixed in ONE body frame. Handle points of a shape: the 2 segment ends, or the 8 box corners.
- gap(q) = dist(core1(q), core2(q)) - r1 - r2 (the filter's gap, for shapes that do not overlap).
- Variables V of the pair: the filter's state variables (arm hinges, finger slide, pedestal slide) that are on one body's path to the root but not on the other's. V1 = those on S1's side, V2 = on S2's side.
- Joint box Q = product of [lo_j, hi_j] over j in V; centre c; half widths w_j = (hi_j - lo_j) / 2.
- Lever arm lam_j(p) of variable j on a handle point p: for a slide, 1 (m per m); for a hinge, a number that is at least the distance of p from the hinge axis in EVERY pose (computed below).
- Travel bound of shape i over the box: T_i(Q) = max over its handle points p of sum over j in V_i of w_j x lam_j(p).
- Centre value: G(c) = a number that is at most gap(c) (capsule vs capsule: the closed-form segment distance minus eps_num; capsule vs box: the smallest of K+1 equally spaced samples of the segment's distance to the box, minus len/(2K), minus eps_num; len = segment length).
- LOWER BOUND: LB(Q) = G(c) - T_1(Q) - T_2(Q).
- Rule: Q is settled when LB(Q) >= thr = margin + 1 cm (config PUSH_START_BEYOND_M). Otherwise Q is cut in two and both halves are tested; a pair is declared never-close only when every piece of the full joint limits is settled.

## (2) Why LB(Q) <= gap(q) for every q in Q
Step 1, common joints do not matter. The two bodies' paths to the root share a trunk. A joint on the trunk moves both shapes by the same rigid motion, which changes no distance. So positions are taken in the frame of the last shared body, where S_i depends only on V_i.
Step 2, one hinge. If only hinge j turns, by an angle d, a point p attached further down the chain moves on a circle around the axis. Its distance from the axis, rho, does not change during this motion. It travels an arc of length rho x |d|, so it ends at most rho x |d| <= lam_j(p) x |d| from where it started. A slide of d moves p by exactly |d|.
Step 3, the lever arm is valid in every pose. Work from p up the chain, keeping a ball (centre fixed in the current body frame, radius) that contains p for every value of the joints passed so far. Start: centre p, radius 0. Passing a hinge with anchor a, any angle: the ball (centre x, radius s) becomes the ball (centre a, radius |x - a| + s), because a rotation about an axis through a keeps every distance from a. Passing a slide with range [lo, hi] along unit axis e: centre x + e (lo + hi) / 2, radius s + (hi - lo) / 2. Passing a fixed body offset: the centre is moved rigidly, the radius is unchanged. When hinge j is reached with ball (x, s) in the frame where its axis is fixed: lam_j(p) = (distance of x from the axis) + s. Every possible position of p is inside the ball, so its distance from the axis is at most lam_j(p). No assumption on the other joints' values is needed (they may even be outside their limits).
Step 4, many joints. Go from c to q by changing one variable at a time. Each change moves p by at most |q_j - c_j| x lam_j(p) (steps 2 and 3, which hold in every pose, so also at each intermediate pose). By the triangle inequality |p(q) - p(c)| <= sum_j |q_j - c_j| lam_j(p) <= sum_j w_j lam_j(p) <= T_i(Q).
Step 5, every point of the core. A body moves rigidly, so a point x of the core that is a fixed weighted average of the handle points (weights >= 0, sum 1: a point of a segment is (1-s)A + sB; a point of a box is a weighted average of its 8 corners) stays that weighted average. Its movement is the same weighted average of the handle movements, so |x(q) - x(c)| <= T_i(Q).
Step 6, the distance. Take q in Q and the closest points x1 on core 1 and x2 on core 2 at q (same material points followed back to c). dist(q) = |x1(q) - x2(q)| >= |x1(c) - x2(c)| - T_1 - T_2 >= dist(c) - T_1 - T_2. (If the cores overlap at q, dist(q) = 0 and the same line gives 0 >= dist(c) - T_1 - T_2.) Radii and box sizes do not change, so gap(q) >= gap(c) - T_1 - T_2 >= G(c) - T_1 - T_2 = LB(Q).
Step 7, the centre value. Capsule vs capsule: the closest distance of two segments has a closed form (the filter's own function, geometry.capsule_capsule_batch); eps_num covers rounding. Capsule vs box: f(s) = distance of the point A + s(B - A) from the box changes by at most len x |ds| (a distance to a set changes no faster than the point moves). Every s in [0, 1] is within 1/(2K) of a sample, so min f >= (smallest sample) - len/(2K).
Step 8, the pieces. Halving a box along one variable gives two boxes whose union is the box. If every piece has LB >= thr, gap(q) >= thr for every q in the joint limits.
Step 9, the filter's own numbers. The filter's gap functions return the distance between two actual points of the cores (capsule-capsule), or the smallest of sampled true distances (capsule-box), minus the radii. Both are at least the true gap. So "true gap >= thr everywhere" means the filter can never compute a gap below thr for the pair.

## (3) Assumptions
A1. Between each shape's body and the last shared body there are only rigid bodies joined by hinges and slides, at most one joint per body.
A2. Each shape is rigid and fixed in one body frame; radii and box half sizes are constant.
A3. The variables stay inside the limits used for the proof: the model's joint ranges (jnt_range) for the 14 arm joints, 0 to 0.418 m pedestal, 0 to 0.044 m finger travel.
A4. Joints that are not filter variables (wheels) keep the value the filter gives them (0); the base joint is on the shared trunk.
A5. The tool's forward kinematics is the filter's (same compiled model, same shapes file, fingerprint checked).
A6. The lever arm is computed for every handle point of the shape (both ends, all 8 corners), not for a typical point.
A7. Only robot shapes of robot_shapes.json are covered. Held objects and world objects are never declared.
A8. The threshold is on the gap (margin + 1 cm), the same quantity the filter measures.

## (4) What breaks if an assumption fails, and the check on our robot
A1. A ball joint, a closed loop or two joints on one body would make steps 2 and 3 wrong (a point could move further than the bound). CHECK: model has 1 free joint (base, on the shared trunk), 16 hinges, 5 slides; max 1 joint per body; the two equality constraints (finger mimic) are not used by the filter's kinematics (it writes qpos directly). The tool asserts this at start and refuses otherwise.
A2. A capsule strung between two bodies would not move rigidly, so step 5 (weighted average with fixed weights of ONE body's points) would not hold as written. CHECK: all 45 shapes have both ends on one body and no t_range. The tool refuses a shapes file where this is not so.
A3. If a command leaves the joint limits the proof says nothing about that pose, and a declared pair could come closer than thr unseen. The filter does not clip commands (it records the overshoot and holds only after a push). CHECK: in the six golden recordings no raw, last or filtered command is outside the limits (largest overshoot +0.0000 rad on the punch raw commands, which reach the limit itself; pedestal 0 to 0.417 m; fingers 0 to 0.0439 m). This is the SAME assumption the Stage 1 list already makes. It is the weakest assumption: listed under Decisions for David (option: prove over limits widened by a small pad, config value, default 0 = as Stage 1).
A4. A wheel shape placed at another wheel angle would be elsewhere. The filter never turns the wheels in its own model, so the shapes it checks are the ones proven. Unchanged from Stage 1.
A5. A different model or shapes file would make the proof about another robot. CHECK: the tool stops if the shapes file's fingerprint differs from the compiled model's, compares its own batched kinematics with RobotShapes.place (MuJoCo) on random poses at start (stops above 1e-9 m), and the output carries the fingerprint, which the loader must check.
A6. Using only one end would understate the travel. CHECK: by construction (max over all handle points).
A7. Nothing to check: such pairs are not candidates.
A8. See (5): the threshold is the activation distance, not a larger one.

## (5) Where the bound is loose, and one consequence to know
- Loosest point: the lever arm. It is pose-independent: the chain is taken as fully stretched and perpendicular to every axis at once (step 3 adds up link offsets). The true lever is often half of it or less. Second: travel is added up as if every joint moved the point in the same direction and both shapes moved straight toward each other.
- Size of the loss: on one box, gap(q) can exceed LB by up to 2 x (T_1 + T_2) (when the centre is the closest pose, the bound still subtracts the full travel, and the far side of the box gains as much). At the smallest box (0.05 rad per joint, w = 0.025 rad) T = 0.025 x sum of lever arms: with 7 joints and lever arms of 0.1 to 0.8 m this is 2 to 8 cm per arm. So a pair whose true closest approach is less than thr plus a few cm can NOT be declared, however long the tool runs. Those pairs stay checked (safe direction). Capsule vs box adds len/(2K) (1.2 mm for a 0.3 m capsule at K = 128).
- Consequence of the threshold (not a soundness gap, but David should know): thr = margin + 1 cm is where the filter starts to act on a pair ONLY IF the pair's boxes are that close; the gap rule, however, also limits the closing speed of a candidate pair whose gap is above margin + 1 cm when a single tick would close more than 20 percent of the gap above the margin. A declared pair is no longer speed-limited in that way. It still can never come within margin + 1 cm, so nothing unsafe follows, but a decision can differ from the unextended filter on a fast approach. The Stage 1 list has the same property. The gate (check iv) measures whether it happens on the recorded streams. Listed under Decisions for David.
- No gap was found in the reasoning that blocks the tool. A3 (commands inside the joint limits) is inherited from Stage 1 and is raised as a decision, with the conservative default kept.

