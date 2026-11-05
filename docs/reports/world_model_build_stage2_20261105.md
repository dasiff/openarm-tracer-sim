# World-model build, stage 2: the world-model collision checker (2026-11-05)

Branch world-model-build. This stage adds the checker (src/world_collision.py), a provisional margin in the config (world_margin_m = 0.03 m, PROVISIONAL, row M271), and this report. It gates nothing, changes no ranking, and does not touch the verification code, the sag layer or the design plan. The comparisons and robustness runs are scratch scripts outside the repo.

## 1. The checker

`src/world_collision.py`:
- `boxes_from_world(world)`: one oriented box per object or fixture of the world model (position, shape, height; a cylinder becomes a square box of its radius; the bench surface root has no box).
- `robot_group_points(model, data, side)`: the robot's own collision geoms for the hand, the fingers and the forearm (links 4 to 7), as convex hulls of their vertices, sampled on the hull surface.
- `box_signed_distance` and `check_boxes(groups, boxes, margin, target_ids)`: the signed distance from each point to each oriented box (negative = overlap, measured as the depth to the nearest box face), the smallest per group, and the clearance (that minimum capped at the margin). The target beaker is exempt from the fingers.

**Decision I made under item 1 (for your check):** the robot's geometry comes from its own model as convex hulls of the mesh vertices, sampled densely on the hull surface (vertices plus three subdivisions per hull edge), rather than MuJoCo's own geom-to-geom distance. Reason: the same code then serves any box source (a detector later), and it reads only robot geometry. Limits: a box entirely inside a robot hull is not detected (no hull surface point lies inside a small box); the depth convention is the depth to the nearest box face, while MuJoCo reports penetration, so depths differ (see §2).

Exemptions applied: the target beaker against the fingers; the forearm group includes links 4 to 7 (the shoulder chain, link 1 to 3, is not checked). The support pair (beaker on hotplate) is an object pair and is not computed here.

Boxes from the world model (moved layout shown, hotplate at x 0.65): hotplate centre (0.65, 0.66, 0.994), half sizes (0.09, 0.09, 0.034), z 0.960 to 1.028; beaker centre (0.85, 0.66, 1.012), half sizes (0.035, 0.035, 0.05), z 0.962 to 1.062.

## 2. Checker against the simulator's true geoms (answer key)

Poses: both stored stances (unshifted layout for (0.755, 1.145, q 0.10); moved layout for (0.635, 1.135, q 0.10)), both arms, bins 0, 1, 11, 2, 3, 9, 10, and the grasp, pre-grasp and over-the-top poses: 84 poses. The pose is the zombie-start IK of the verification rule (24 of 84 accepted); the other 60 are evaluated at their best solution and labelled. The sim answer key uses MuJoCo's geom-to-geom distance (search cap 0.5 m).

- Compared pairs (uncapped sim values, both objects, all groups): 418. The sim cap (0.5 m) is excluded.
- **Sign:** no disagreement. The checker and the sim agree on overlap versus clear for every compared pair.
- **Magnitude:** median absolute difference 3.0 mm.
- **Largest hotplate difference:** 26.6 mm, right arm, bin 2, grasp, fingers, moved stance: checker −33.9 mm, sim −60.5 mm. It is a depth convention difference (nearest face of the box versus MuJoCo penetration), not a sign difference.
- **Largest beaker difference:** 14.5 mm, right arm, bin 10, over-the-top, forearm, stored chosen stance: checker 262 mm, sim 277 mm. The beaker box is the square bounding box of its cylinder, so its corners lie 14.5 mm further out (0.035 × (√2 − 1) ≈ 14.5 mm). This is the box being conservative, not an error.

## 3. Stage 1's hotplate overlap listing through the checker

Accepted poses (zombie-start rule) overlapping the hotplate box, any group: exactly two, both right arm, bin 2, pre-grasp, at the stored stances.

| Stance | Stage 1 (true geoms, depth of the worst group) | Checker (box), hand / fingers / forearm | Difference |
|---|---|---|---|
| Unshifted, (0.755, 1.145, q 0.10) | 31.7 mm (fingers) | −7.5 / −28.9 / +26.5 mm | fingers 2.8 mm shallower in the box model |
| Moved, (0.635, 1.135, q 0.10) | 31.0 mm (fingers) | −9.8 / −30.7 / +26.5 mm | fingers 0.3 mm shallower |

The result reproduces stage 1: the same two poses are the only accepted overlaps, and the depths agree to within 3 mm. Note that the over-the-top right bin 2 pose is clear: its smallest group clearance is 28.5 mm (unshifted) and 28.1 mm (moved).

## 4. Robustness (boxes shifted and resized by up to the margin)

Variants: centre ±0.03 m on each axis (x, y, z separately); yaw ±0.333 rad (the angle at which a 0.09 m box edge moves 0.03 m; my choice); half sizes ±0.03 m on each axis (x, y, z separately); two combined cases (sizes +0.03 with centre −0.03 on all axes; sizes −0.03 with centre +0.03 on all axes); and yaw plus a +0.03 shift in x and y. Each variant changes both boxes. Only the 24 accepted poses are used for the flip and ordering counts.

Flips (clear ↔ overlap, accepted poses), nominal in brackets:

| Variant | Flips | Poses that flip (nominal → variant, mm) |
|---|---|---|
| x centre +0.03 | 4 | left bin 0 grasp (10.6 → −17.7); left bin 1 grasp (4.2 → −19.0); the same for the moved stance |
| x centre −0.03 | 0 | — |
| y centre ±0.03 | 3 / 0 | +: left bin 1 pre-grasp, right bin 1 and 2 (the bin 2 over-the-top); −: none |
| z centre +0.03 (z-shift, box top rises) | 2 | right bin 2 over-the-top (28.5 → −1.5; 28.1 → −1.9) |
| z centre −0.03 (z-shift, box top falls) | 0 | — |
| z size +0.03 (z-size, top rises) | 2 | the same two right bin 2 over-the-top poses |
| z size −0.03 (z-size) | 0 | — |
| yaw −0.333 rad | 4 | left bin 0 grasp (10.6 → −2.1); left bin 1 grasp (4.2 → −13.2) |
| yaw +0.333 rad | 0 | — |
| x size +0.03 | 4 | the same grasp poses as the x centre +0.03 |
| combined: sizes +0.03, centre −0.03 | 0 | — |
| combined: sizes −0.03, centre +0.03 | 0 | — |
| combined: yaw +0.333 rad and centre +0.03 in x and y | 7 | left bins 0 and 1 grasp; left bin 1 pre-grasp (19.5 → −17.9); right bin 1 pre-grasp (6.9 → −21.1) |

Findings:
- **The grasp z extent does not matter at this stance.** The z-shift and z-size flips come only from the right arm's over-the-top pose at bin 2. The left grasp poses flip with x and yaw, because their fingers are only 4 to 11 mm from the box side.
- **The margin does not cover the left grasps.** A 3 cm change in x, or a yaw change of about 19°, flips the left bin 0 and bin 1 grasps to overlap.
- **Ordering is not robust.** With the clearance capped at 30 mm, most poses tie at the cap, so the bin and stance orders are set by a few poses. The bin order changes in 4 of the 17 variants (x centre −0.03, y centre +0.03, x size −0.03, y size +0.03). The stance order changes in 7 of the 17 (y centre +0.03, z centre +0.03, yaw −0.333 rad, y size +0.03, z size −0.03, and both combined cases with the centre shifted). The stance order is the more fragile of the two.

## 5. Timing and memory

- Checker per pose (robot hull points plus the box test): mean 45 ms, maximum 66 ms, over 84 poses.
- Checker per box variant (robot points reused): about 17 ms per call; 1,512 calls in 26 s.
- The whole answer-key run (84 poses, load included), under run_capped: 20 s wall, peak RSS 2.64 GB. Within the 5 GB budget.

## 6. What I did not do

- No ranking key, no gating, no cheap or slow pass, no change to pedestal_override_m, the sag layer or the design plan.
- The margin is a PROVISIONAL config value (world_margin_m = 0.03 m, row M271). It is used only to cap the clearance. A value change is a decision for you.
- The bench is not a box in the checker (the bench has no box in the world model). The hand-to-bench clearance is not part of this stage.

## 7. Items for David

1. Confirm the hull-surface geometry choice (§1), or ask for a geom-distance version.
2. The grasp clearance of left bins 0 and 1 is 4 to 11 mm. A margin of 3 cm or a 19° yaw error flips them. Decide whether those grasps are acceptable with the VLA and contact sensing, before the clearance key is ranked.
3. The commit label: the brief asks for none on this stage. Nothing was added.
