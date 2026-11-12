# Safety filter Stage 2: the right-arm-closing failure, and why left and right differ

Written for: David, to put the case to Kong. Numbers come from `scripts/safety_filter_closing_arm_asymmetry.py` (prints everything below; code at commit 0c54640). All in the robot base frame, metres or mm.

## The case

Hand-hand approach (the spec's scenario), one hand closing 8 mm in one tick, the other arm parked at the zombie pose:
- Left closing: left hand moves y 0.034 → 0.026, right arm at home.
- Right closing (mirror): right hand moves y −0.034 → −0.026, left arm at home. The commanded joint angles are exact mirrors (`q_right = MIRROR × q_left`, checked).
- Pair under pressure at the start in both: left_palm_2 / right_palm_2, gap 15.14 mm at the last command and 7.14 mm at the raw command. Only the moving arm is "closing".

Result: left closing → HAND_SLIDE in 1 round, passes. Right closing → HOLD, reason ACCURACY, after 10 rounds (the arm stays at its last command).

## What happens in the right case, round by round

1. Worst pair (left_palm_2, right_palm_2), shortfall 6.97 mm. HAND_SLIDE: the hand's into-obstacle motion is removed. After it the palm pair is fine, but the worst pair is now (right_link5_2, torso), shortfall 0.70 mm (stop line 0.5 mm). The right forearm capsule is 12.10 mm from the torso at the last command and 11.73 mm after the slide.
2. Worst pair (right_link5_2, torso): ELBOW_SWING. The swing converter (Kong's, directions only) is told to reach the **raw** hand pose, which is the hand pushed into the other palm. After the swing the palm pair's shortfall is back to 7.02 mm.
3. Rounds 3 to 10 repeat 1 and 2 exactly (shortfalls 7.03 / 0.81 mm). The loop never settles. After 10 rounds step shortening is tried; its accuracy check compares to the raw hand pose and fails (ACCURACY), so the tick holds.

## Why left and right differ

The robot is nearly but not exactly mirror-symmetric, and the hand slide is sensitive to a small difference.

- The fitted palm capsule left_palm_2 and right_palm_2 are not mirror images: endpoints differ by up to 2.4 mm in x and 2 mm in y at HOME (left a (0.2242, 0.0855), b (0.2066, 0.1639); right a (0.2221, −0.0846), b (0.2071, −0.1661)).
- More generally the two arms' shape sets are not mirrors: the left upper arm has two link-3 capsules (left_link3_1, left_link3_2), the right has one (right_link3), and the link-1, link-5 and palm_1 capsules differ in endpoints and radius (palm_1 radius 0.0124 left, 0.0094 right).
- So the contact normal is tilted differently. Push direction out of the obstacle: left (+0.0337, +0.9994, −0.007); right (−0.0337, −0.9994, +0.007). A true mirror of the left one would be (+0.0337, −0.9994, −0.007). The x tilt has the wrong sign for the right arm.
- The hand slide keeps the part of the hand motion along the surface. With the tilted normal the 8 mm y motion leaves a 0.27 mm sliding component in x: **+x (away from the torso) for the left hand, −x (toward the torso) for the right hand.**
- The right forearm is 12.10 mm from the torso at the last command, inside the torso's slowing band, so a 0.37 mm drift toward it creates a 0.70 mm shortfall (above the 0.5 mm stop). On the left the same drift moves away (left link7_2 / torso 28.09 → 28.24 mm), the shortfall is 0.19 mm, and the tick passes. (The left forearm is even closer to the torso, 8.22 mm, but it moves away.)

So it is a knife-edge: the outcome depends on the sign of a 0.3 mm sliding drift, which comes from a 2 mm capsule-fit asymmetry. The same would happen to the left arm in another pose. It is a property of the algorithm as I built it, not a left/right bug.

## The question for Kong

Two contacts on one arm need different corrections:
- palm against the other palm: the hand must not go into the obstacle (hand slide: hand position moves, orientation stays as commanded);
- forearm against the torso: the elbow must swing, with the hand pose kept at the commanded one (swing about the shoulder-wrist line, hand target = raw).

The second target undoes the first. How does his algorithm combine them in one tick? Specific questions:
1. In the elbow swing, is the hand target the raw command, or the hand pose after any earlier correction in the same tick?
2. Does he correct one pair at a time (worst first) and loop, or solve all contacts of an arm together? Is there a loop limit, and what happens when it does not converge?
3. What does his filter do when the elbow swing and the hand slide conflict like this: shorten the step, hold, or accept a small violation?
4. Is the hand slide meant to keep the tangential component (my reading), or to hold the hand still against the contact?
5. How is the contact normal defined for capsule ends (closest-point direction)? This is where a 2 mm capsule difference becomes a 2° normal tilt.

## What I did not change (and what could be tried)

Not fixed: each option is a design choice. Candidates, each small: (a) elbow swing aims at the hand target already set by the slide this tick; (b) hand slide removes the tangential drift too, or limits it to the torso side; (c) use mirrored capsules for the two arms. None was built.

The same failure shows in the spec test (`experiments/safety_filter_spec_test.py`, check "the closing right arm moves"). The other failing check there (hand orientation 6.4e-4 rad from one linear Jacobian step, spec wanted 1e-6) is a separate matter.
