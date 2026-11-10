# Safety filter: shape refinement (forearm, link 7) under the shape-count rule (2026-11-10)

Written for: David. Kinematics only; no simulation stepping; no push. Pipeline and src/control_loop.py untouched.

## 0. Location check

- /home/vboxuser/projects/openarm-tracer-sim-safety-filter, branch safety-filter.
- Python: /home/vboxuser/projects/openarm-tracer-sim/.venv/bin/python (the worktree has a `.venv` symlink to it for experiments/run_capped.sh; not committed).

## 1. Rule (SF-16)

At every pose of the pick-and-place check (M1 to M9), no non-exempt filter shape may be inside its 1 cm arm-vs-world margin. The 2 cm push-start band is allowed. Each part gets the fewest capsules that meet this, and its capsules fully enclose the real geometry (stick-out <= 0) over the sampled poses.

## 2. Link 7 vs beaker breakdown (step 1, measured before any change)

Script: experiments/safety_filter_link7_beaker_check.py. Gaps in mm (negative = overlap). Real = MuJoCo mj_geomDistance on the real link 7 collision mesh (scoring). Exact cylinder = solid cylinder, radius 35 mm, height 100 mm, analytic distance on 4000 segment samples.

| moment | capsule to exact cylinder | real link 7 to beaker capsule | link 7 capsule to beaker capsule (as now) | real link 7 to beaker real geoms (as now) |
|---|---|---|---|---|
| M2 grasp 45 % | -10.0 | +59.5 | -10.0 | +59.5 |
| M3 grasp 60 % | -10.2 | +59.3 | -10.2 | +59.3 |
| M4 lift start | -9.9 | +59.5 | -9.9 | +59.5 |

Over-coverage = real (+59.5) minus filter (-10.0) = 69.5 mm.

Decomposition:
- Capsule to exact cylinder equals capsule to beaker capsule (difference 0.0 mm): the beaker's catalog capsule caps cause none of the over-coverage at these moments.
- Real link 7 to beaker capsule equals real to real (difference 0.0 mm): the beaker shape causes none of it on the real side either.
- So the whole 69.5 mm comes from the LINK 7 CAPSULE (its wall lies 10 mm inside the beaker while the real mesh is 59.5 mm away).

Correction of the earlier report (docs/reports/20261110_1329_safety_filter_fixup.md, section 4 and section 6 items 2): I had attributed the link 7 over-coverage to the beaker capsule's end caps. The breakdown above shows the cap is not the cause.

What sets link 7's capsule (link 7 frame, mm):
- Radius 32.93 mm: set by 4 real vertices at (1.5, -28.5, 94.5), the upper end of the link 7 mesh (the wrist end, next to the hand).
- Ends: lower end set by vertex (0, -34.5, -17.5); upper end by vertex (0.2, 28.5, 94.5). The capsule axis runs from the lower to the upper extreme of the mesh, so the length is set by the link 7 mesh's extent along that axis.
- Where the beaker's nearest point is along link 7's z (hand) axis, from the link 7 origin: +132 mm on the capsule side, +94.5 mm on the real side.

Verdict: the link 7 capsule causes all of the over-coverage. Per the brief, the beaker capsule is not changed; link 7 is changed (step 3).

## 3. Forearm: 2 and 3 capsules (step 2)

Script: experiments/safety_filter_forearm_fit.py (fit set: 2000 poses, 8 limit corners plus random, joints 4 to 6; the line is the joint 4 to joint 7 anchor line; convex-hull vertices of links 4 to 6). Independent check: 500 new random poses (seed 99); max stick-out at each capsule: +0.000 mm (within rounding).

| count | split points (fraction of line) | radii (cm) | volume (cm^3) |
|---|---|---|---|
| 1 | none | 7.614 | 5823.7 |
| 2 | 0.73 | 7.614 (elbow, 0 to 0.73), 4.524 (wrist, 0.73 to 1) | 5517.3 |
| 3 | 0.74, 0.78 | 7.614, 0.0 (middle piece covers no vertex), 4.524 | 5486.9 |

Line length: 21.83 cm.

Choice: 1 capsule fails the rule (forearm capsule inside the 1 cm bench margin at M2 at -9.6 mm, M3 at +0.3 mm, M4 at -3.9 mm). 2 capsules pass (section 5). 3 capsules give no additional radius reduction (the middle piece is empty; -0.6 % volume over 2). So the fewest that satisfy the rule is 2.

## 4. Link 7 (step 3)

Link 7 with 2 capsules in the link 7 frame (rigid to the link, so the enclosure is exact for every pose; fit by the tool's two-capsule routine, volume-minimising split; checked by the shapes test on the hand/link 7 geoms).

| piece | radius (mm) |
|---|---|
| left_link7_1 | 32.93 |
| left_link7_2 | 17.08 |

One capsule (current) fails the rule (-10 mm at M2 to M4). Two capsules pass (section 5). Count chosen: 2.

Beaker unchanged. Report of what the rule would need from the beaker: nothing for non-exempt shapes. The beaker's capsule is used as now; with the two link 7 pieces, the link 7 gaps to the beaker are +11 to +12 mm (BAND, not MARGIN) at M2 to M7. Fingers and held objects are exempt from the world (M289), so their beaker gaps do not count for the rule.

## 5. Pick-and-place check after the change (step 4)

Script: experiments/safety_filter_pick_place_check.py (kinematics; same IK and moments as before; staging stance, pick bin 0, place bin 0). Log: data/logs/pick_check_135629.log. Gaps in mm: shape gap (filter) and real gap (MuJoCo, scoring) to bench top / hotplate / beaker. Flags on the shape gaps: MARGIN < 10 mm, BAND < 20 mm.

| moment | shape | shape bench / plate / beaker | real bench / plate / beaker | flags (shape) |
|---|---|---|---|---|
| M1 | upper arm | +262.7 / +295.0 / +340.8 | +294.9 / +312.3 / +353.2 | |
| M1 | forearm (2 pieces) | +130.6 / +198.5 / +227.6 | +147.3 / +207.0 / +244.3 | |
| M1 | link 7 (2 pieces) | +45.2 / +115.3 / +130.3 | +77.1 / +147.5 / +178.3 | |
| M1 | palm | +56.1 / +99.1 / +147.7 | +73.1 / +110.2 / +162.3 | |
| M1 | fingers | +7.7 / +50.1 / +81.1 | +34.0 / +66.5 / +85.2 | MARGIN(bench) exempt |
| M2 | upper arm | +162.3 / +226.7 / +267.8 | +230.4 / +295.5 / +335.9 | |
| M2 | forearm | +21.3 / +102.4 / +108.8 | +33.9 / +109.7 / +125.5 | |
| M2 | link 7 | +12.0 / +68.8 / +11.7 | +16.4 / +81.2 / +59.5 | BAND(bench), BAND(beaker) |
| M2 | palm | +15.2 / +19.2 / +29.2 | +16.4 / +25.5 / +43.5 | BAND(bench), BAND(plate) |
| M2 | fingers | +0.3 / +0.8 / -16.8 | +14.5 / +33.5 / +5.9 | MARGIN(bench, plate, beaker) exempt |
| M3 | upper arm | +159.5 / +226.8 / +264.6 | +227.6 / +295.5 / +332.8 | |
| M3 | forearm | +31.2 / +102.7 / +108.6 | +40.2 / +110.0 / +125.4 | |
| M3 | link 7 | +26.0 / +69.3 / +11.5 | +30.4 / +81.6 / +59.3 | BAND(beaker) |
| M3 | palm | +29.2 / +19.6 / +29.1 | +30.5 / +26.0 / +43.3 | BAND(plate) |
| M3 | fingers | +14.4 / +1.8 / -16.5 | +28.5 / +34.0 / +6.2 | BAND(bench), MARGIN(plate, beaker) exempt |
| M4 | upper arm | +161.0 / +227.1 / +266.2 | +229.1 / +295.8 / +334.3 | |
| M4 | forearm | +27.0 / +102.3 / +108.8 | +37.1 / +109.6 / +125.5 | |
| M4 | link 7 | +20.1 / +68.7 / +11.7 | +24.5 / +81.1 / +59.5 | BAND(beaker) |
| M4 | palm | +23.3 / +19.1 / +29.2 | +24.5 / +25.4 / +43.5 | BAND(plate) |
| M4 | fingers | +8.3 / +7.7 / -23.0 | +22.5 / +40.4 / -1.2 | MARGIN(bench, plate, beaker) exempt |
| M4 | held beaker | -35.0 / +75.0 / - | +0.0 / +75.0 / - | MARGIN(bench) exempt (M289); real contact at 0.0 (the beaker stands on the bench) |
| M5 | upper arm | +87.0 / +159.3 / +217.5 | +152.6 / +227.9 / +285.3 | |
| M5 | forearm | +68.4 / +76.3 / +108.3 | +109.1 / +102.7 / +120.9 | |
| M5 | link 7 | +126.6 / +60.4 / +11.2 | +131.0 / +69.3 / +59.1 | BAND(beaker) |
| M5 | palm | +129.8 / +65.6 / +28.8 | +131.0 / +68.5 / +43.1 | |
| M5 | fingers | +115.0 / +47.0 / -23.4 | +129.1 / +61.1 / -1.4 | MARGIN(beaker) exempt |
| M5 | held beaker | +73.0 / +12.2 / - | +108.0 / +40.0 / - | BAND(plate) exempt |
| M6 | upper arm | +75.3 / +125.2 / +210.1 | +119.2 / +173.4 / +266.7 | |
| M6 | forearm | +62.0 / +68.0 / +108.2 | +93.5 / +102.1 / +118.4 | |
| M6 | link 7 | +126.6 / +58.7 / +11.0 | +131.0 / +65.8 / +58.9 | BAND(beaker) |
| M6 | palm | +129.8 / +62.7 / +28.7 | +131.0 / +65.0 / +42.9 | |
| M6 | fingers | +115.0 / +47.0 / -23.5 | +129.1 / +61.1 / -1.4 | MARGIN(beaker) exempt |
| M6 | held beaker | +73.0 / +5.0 / - | +108.0 / +40.0 / - | MARGIN(plate) exempt |
| M7 | upper arm | +101.3 / +151.4 / +227.5 | +141.5 / +196.5 / +282.3 | |
| M7 | forearm | +49.5 / +66.1 / +109.1 | +88.6 / +80.2 / +122.2 | |
| M7 | link 7 | +89.9 / +22.1 / +11.9 | +94.4 / +33.0 / +59.8 | BAND(beaker) |
| M7 | palm | +93.1 / +26.9 / +29.5 | +94.4 / +31.3 / +43.8 | |
| M7 | fingers | +78.2 / +10.2 / -22.7 | +92.4 / +24.4 / -1.1 | BAND(plate), MARGIN(beaker) exempt |
| M7 | held beaker | +35.0 / -33.0 / - | +70.0 / +2.0 / - | MARGIN(plate) exempt (M289); real 2 mm above the plate, as set |
| M8 | upper arm | +101.7 / +151.8 / +227.8 | +142.0 / +196.8 / +282.6 | |
| M8 | forearm | +49.3 / +66.1 / +108.9 | +88.3 / +79.8 / +122.1 | |
| M8 | link 7 | +89.6 / +21.8 / +11.8 | +94.1 / +32.6 / +59.6 | BAND(beaker) |
| M8 | palm | +92.8 / +26.6 / +29.4 | +94.1 / +30.9 / +43.6 | |
| M8 | fingers | +77.9 / +9.9 / -16.7 | +92.1 / +24.1 / +5.9 | MARGIN(plate, beaker) exempt |
| M9 | upper arm | +141.7 / +192.3 / +267.8 | +178.1 / +231.5 / +317.0 | |
| M9 | forearm | +83.7 / +111.5 / +158.0 | +118.4 / +123.1 / +169.5 | |
| M9 | link 7 | +88.6 / +39.6 / +60.8 | +93.1 / +63.1 / +108.8 | |
| M9 | palm | +91.7 / +48.4 / +78.4 | +93.0 / +60.5 / +92.8 | |
| M9 | fingers | +76.9 / +8.9 / +17.4 | +91.0 / +33.8 / +24.6 | MARGIN(plate) exempt, BAND(beaker) |

Remaining MARGIN flags on NON-EXEMPT shapes: NONE. Every MARGIN flag left is on a finger (exempt from the world, M289) or on the held beaker (exempt from the world, M289). Non-exempt shapes are at BAND or better (no MARGIN) at every moment.

The rule is met at every pose of M1 to M9.

## 6. Shapes test, touch test, timing

- Shapes test (experiments/safety_filter_shapes_test.py): PASS. 200 poses, both arms, each arm drawn on its own. Largest stick-out 2.9e-15 m (floating point; limit 1e-12). Multi-capsule parts (forearm, link 7, palm) checked as unions: each vertex lies in at least one capsule (largest 3e-16 m). Anchor check: the forearm pieces meet at the line's ends (error 3e-17 m). Fingerprint refusal: PASS.
- Touch test (experiments/safety_filter_touch_test.py): PASS (torso list check added; see section 7).
- Distance test: PASS (unchanged: capsule-capsule 1.2e-16 m vs MuJoCo; capsule-box 3.5e-5 m vs brute force, 4.99e-6 m for the batch version).
- Structure test: PASS.
- Timing, same 200 random states, A/B in one run (run_capped in tmux, same load):

| stage | previous commit (median ms) | this change (median ms) |
|---|---|---|
| place | 0.31 to 0.35 | 0.32 to 0.35 |
| broad phase | 0.59 to 0.64 | 0.59 to 0.64 |
| pair checks | 2.0 to 2.2 | 2.0 to 2.1 |
| whole tick | 3.06 to 3.31 | 3.06 to 3.07 |

  Note: the machine was loaded (another session's MuJoCo job ran at the same time). In a quiet window, earlier today, the current code measured place 0.10, broad 0.17, pairs 0.58, whole tick 0.87 ms. The quiet-window numbers meet the targets (pairs under 1 ms, tick under 2 ms); the loaded numbers do not. The change adds 2 forearm and 2 link 7 capsules; the candidate count (median 2) and checked pairs (median 14 to 15) are the same before and after.

## 7. Torso exemption re-evaluated (SF-9 method)

Script: experiments/safety_filter_torso_evidence.py (zero pose: pedestal 0.15 m, fingers 0.02 m, as in SF-9; 200 random poses, both arms random in each pose). Gap in mm to the torso capsule.

| shape | zero pose | min over 200 poses | poses overlapping |
|---|---|---|---|
| left shoulder mount | -48.7 | -48.7 | 200 |
| right shoulder mount | -48.7 | -48.7 | 200 |
| left upper arm | -16.3 | -14.0 | 1 |
| right upper arm | +18.1 | -40.0 | 5 |
| left forearm 1 (elbow piece) | +4.3 | -54.6 | 2 |
| left forearm 2 (wrist piece) | +58.2 | -55.4 | 1 |
| right forearm 1 | +44.3 | -81.2 | 3 |
| right forearm 2 | +66.7 | -66.5 | 3 |
| left link 7 piece 1 / 2 | +71.2 / +126.3 | -46.6 / -49.9 | 1 / 1 |
| right link 7 piece 1 / 2 | +71.2 / +126.3 | -39.8 / -49.8 | 3 / 1 |
| right palm 1 | -20.7 | -8.6 | 1 |
| right palm 2 | +60.9 | -3.9 | 1 |
| fingers | +27 to +102 | -10.7 to +13.7 | 0 to 2 |

Decision: NO CHANGE to TORSO_TOUCHING_IDS (both shoulder mounts and the left upper arm). No forearm piece and no link 7 piece overlaps the torso at the zero pose, so both stay checked. The touch test now asserts the list and that no forearm or link 7 piece is exempt.

Two findings for David (not changed):
1. RIGHT PALM 1 overlaps the torso at the zero pose by 20.7 mm in the shape. The real right hand is 39.4 mm from the real torso mesh (MuJoCo, same pose). So the palm capsule over-covers the real hand toward the torso. By the SF-9 wording ("overlap at the zero pose") it would qualify for exemption, but it does not touch by construction, and SF-9's own list (three shapes) leaves it out. I kept it checked. Consequence: the filter's torso check is active at the zero pose for right palm 1. Decide: shrink the palm capsule, or add it to the exempt list with a reason.
2. The zero-pose gap of left forearm 1 (+4.3 mm) and of right upper arm (+18.1 mm) is inside the 3 cm robot-robot margin: the torso check is active at the home pose for these shapes. This is unchanged from before (the elbow piece has the same radius as before); it is the same margin question.

## 8. What changed in the code

- tools/build_filter_shapes.py: forearm = N capsules on the joint 4 to 7 line (SF-14), split by minimum volume (convex-hull vertices; 2000 poses); link 7 = N capsules in the link 7 frame (SF-15); the report prints per-part dictionaries.
- src/safety_filter/shapes.py: a capsule may cover a fraction range [t0, t1] of the segment between its two body-local ends (vectorised placement). Forearm pieces use it; all other capsules use [0, 1].
- src/safety_filter/config.py: SHAPE_BUILD gains forearm_capsules = 2, forearm_fit_poses = 2000, link7_capsules = 2.
- data/safety_filter/robot_shapes.json regenerated (32 shapes; fingerprint unchanged, 781198f99c7e46e8...).
- experiments: shapes test (multi-capsule union, forearm anchors), touch test (torso list check), pick-and-place check (uses the shipped pieces; the old looseness and two-capsule fit moved to safety_filter_forearm_fit.py), new scripts: safety_filter_pick_scene.py (shared setup), safety_filter_link7_beaker_check.py, safety_filter_forearm_fit.py, safety_filter_torso_evidence.py, safety_filter_tick_timing.py.

## 9. Not done

- The beaker catalog shape is unchanged (as asked). The beaker's capsule caps are not the cause of the link 7 over-coverage (section 2), so the rule needs nothing from the beaker shape for non-exempt shapes.
- The timing target holds only in a quiet window (section 6). Not re-measured on a quiet machine after the change.
- Right palm 1 vs torso (finding 1) was not decided; not changed.
- The forearm fit used the 2000 poses of the build (joints 4 to 6); the pick-and-place poses are the nine moments. Enclosure is checked over the 2000 + 500 poses; the moments are a subset in practice.
- Pipeline and src/control_loop.py were not touched; nothing was pushed.
