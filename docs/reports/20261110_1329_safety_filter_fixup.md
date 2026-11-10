# Safety filter Stage 1 fix-up, and can the shapes pick and place? (2026-11-10)

Written for: David (Stage 1 review). Kinematics only; no simulation stepping; no shape or config change for the measurement; no push.

## 0. Location check

- Folder: /home/vboxuser/projects/openarm-tracer-sim-safety-filter (worktree), branch safety-filter.
- Python: /home/vboxuser/projects/openarm-tracer-sim/.venv/bin/python (MuJoCo 3.13.0).

## 1. Commits (on safety-filter, not pushed)

| hash | message | files |
|---|---|---|
| 322215f | safety filter: fix-up: lab map follows bench_open (SF-8), gas fixtures; beaker is a capsule; torso exemption narrowed; batched pair checks | tools/survey_lab_map.py, data/safety_filter/lab_map.json (regenerated), src/safety_filter/{config,world,touch,geometry,shapes}.py, experiments/safety_filter_distance_test.py, experiments/safety_filter_touch_test.py |
| 734734d | safety filter: SF-8..SF-13 decision rows (lab map and bench_open, torso exemption, held objects, base and pedestal, fallback height, beaker capsule); paper note Sec. VI-C from the PDF | docs/mpvi_decisions.md, docs/sew_mimic_paper_notes.md |
| (this commit) | Report: safety filter fix-up and the pick-and-place check | experiments/safety_filter_pick_place_check.py, docs/reports/20261110_1329_safety_filter_fixup.md, docs/progress.md |

Fixes 1 to 4 and 6 were grouped in one code commit (they touch the same modules); the decision rows and the paper note are a second commit.

## 2. Lab map (fix 1)

- The map now follows TASK_CFG["bench_open"] (True, M078). With the open bench the four frame rails (bench_frame_front, _back, _left, _right) are not written, as the simulator deletes them (src/task_beaker_hotplate.remove_bench_rails). They are listed under "excluded" with the reason.
- Added as fixed fixtures: gas_manifold (box half 0.25, 0.07, 0.03) and gas_knob_1, _2, _3 (cylinders, box half 0.03, 0.03, 0.02).
- The map records `"bench_open": true` and `LabMapSource` raises `LabMapMismatch` for a filter expecting the other setting (tested: a filter expecting False is refused).
- Scene fingerprint unchanged (ada206db631ff2e3...). Map is now 23 boxes: bench (top + 4 legs), bench_gas (4), sink (3), walls (4), back_counter (2), fume_hood (2), storage_cabinet (1), safety_station (2).

## 3. Torso exemption (fix 3): evidence and decision

Measured with the filter's own shapes (no stepping): gap of each left-arm shape to the torso capsule at the zero pose, and over 200 random poses (joints within limits; both arms random in each pose). Source: throwaway script, results below (also in SF-9).

| shape | gap at zero pose (mm) | min over 200 random poses (mm) | poses overlapping |
|---|---|---|---|
| left shoulder mount | -48.7 | -48.7 | 200 / 200 |
| left upper arm | -16.3 | -26.8 | 4 / 200 |
| left forearm | +4.3 | -100.8 | 5 / 200 |
| left link 7 | +67.0 | -71.1 | 3 / 200 |
| left palm 1 | +21.5 | +12.4 | 0 / 200 |
| left palm 2 | +58.8 | -8.4 | 1 / 200 |
| left fingers (both) | +27.4 / +101.8 | -4.4 / -12.2 | 1 / 200 (left) |
| right upper arm | +18.1 | +3.8 | 0 / 200 |
| right shoulder mount | -48.7 | -48.7 | 200 / 200 |
| right forearm | +35.8 | -113.9 | 4 / 200 |
| right link 7 | +67.0 | -56.6 | 6 / 200 |
| right palm 1 | -3.4 | -27.7 | 4 / 200 |
| right palm 2 | +60.9 | -43.7 | 5 / 200 |

Decision (SF-9): exempt only the pairs that touch by construction, i.e. overlap at the zero pose: both shoulder mounts (fixed to the torso; overlap in every pose) and the LEFT upper arm (-16 mm at zero). The right upper arm is +18 mm at zero and is checked. Everything else is checked: forearm, link 7, palm, fingers, held objects. The left upper arm's overlaps at 4 random poses are a consequence of the exemption and are not separately checked (flag for David).

Note: the asymmetry left/right in the upper arm comes from the model: the left arm's collision mesh is the "flip" version (link1_collision_flip etc.), so the two fitted capsules are not mirror images.

Real-geometry torso gaps were not computed in this step (shape gaps only).

## 4. Beaker catalog (fix 4)

The catalog shape is now a CAPSULE (M287, SF-13): radius 0.035 m, segment half length 0.05 m along the beaker axis, centre 0.05 m above the bottom. The capsule contains the cylinder; its end caps reach 3.5 cm past the rim and base (conservative). Cylinders in the lab map (faucet post, safety post, eyewash bowl, gas knobs) stay enclosing boxes. Held objects are capsules too.

Consequence, seen in section 6: the capsule's caps make the beaker's shape extend 35 mm below its base and 35 mm above its rim. Options for David: (a) keep the capsule and accept the over-reach; (b) make the capsule's half length h - r (ends inside the cylinder), which does not contain the rim corners (they lie 1.41 r from the end points), so it would need a larger radius; (c) keep a capsule for the held object only. Not changed here (report only).

## 5. Speed (fix 2): before and after

Same 200 random states (seed 0, both arms random, pedestal 0.15 m) as the Stage 1 timing; median per stage, ms.

| stage | before (first Stage 1 run earlier today) | after (this fix-up, 2 runs) |
|---|---|---|
| place (own FK, shapes) | 0.90 | 0.48 and 0.32 to 0.39 |
| broad phase | 1.23 | 0.87 and 0.61 to 0.73 |
| pair checks | 13.4 | 2.96 and 2.1 to 2.9 |
| whole tick | 15.7 | 4.37 and 3.1 to 4.0 |

Target was: pair checks under 1 ms and the whole tick under 2 ms (median). **Not met.** Remaining cost: numpy call overhead in the batched capsule-box routine (about 1 ms even for one pair, because the golden-section refinement is about 14 vectorised steps), plus the per-candidate loop. What was done: vectorised placement; AABB boxes and the broad phase in arrays; batched capsule-capsule and capsule-box gaps (gap-only path, no closest points); pairs cut by box separation before rule lookup. Next steps if needed: a compiled kernel (numba or C), or fewer candidate pairs (per-link broad phase).

Accuracy after the change (experiments/safety_filter_distance_test.py, PASS):
- capsule-capsule, 2000 random pairs (about half deep): max error vs mj_geomDistance 1.2e-16 m; batch vs scalar 5.6e-17 m. Unchanged.
- capsule-box, 2000 random pairs: max error vs a 40001-sample brute force 3.5e-5 m (the scalar routine); batch vs brute 5.0e-6 m; on outside pairs vs mj_geomDistance 3.9e-8 m. Unchanged to the same order as before (3.5e-5 m).
- MuJoCo wrong on the sign for one capsule-box pair (a segment through the box; known).

## 6. Pick-and-place check (measurement, report only)

Method (experiments/safety_filter_pick_place_check.py): task scene (TASK_CFG, default layout, open bench). Base at the staging stance (0.765, 1.085), heading -90 deg, pedestal 0.155 m. **Stance selection was NOT run**; the staging stance is the finder's reference stance from TASK_CFG (it is the finder output for this scene per the config comment, but the pipeline's stance selection was not re-run here). Left arm solved with the scripted skill's IK (MinkArm.step, iterated on a fixed state; no stepping). Converged to 1 mm and 0.05 rotation (Frobenius) at all nine moments; residuals 0.6 to 1.0 mm. The approach bin is chosen by convergence: pick bin 0 and place bin 0 (the first bin, nearest to the target direction, at which the grasp and the set-down both converge).

Moments: M1 approach (standoff point), M2 grasp at 45 %, M3 grasp at 60 %, M4 lift start (52.5 %, held), M5 carry (over the midpoint, carry height, held), M6 over plate (held), M7 place down (beaker resting: bottom 2 mm above the plate, held), M8 release (same pose, fingers open), M9 recovery (retreat 5 cm along the approach at set-down height).

Gap = signed distance, mm, negative = overlap. Shape gap uses the filter's shape (capsule/box) to: bench top (lab map box), the hotplate (base and plate boxes), the beaker (catalog capsule at its current pose). Real gap = MuJoCo mj_geomDistance between the part's real collision geoms and the real geoms of the bench top, the hotplate, and the beaker's collision geoms (scoring only; mj_geomDistance is not reliable for penetrating pairs, which do not occur in these real gaps except the held beaker at the bench in M4, which touches at 0.0). Flags use the filter's own margins: MARGIN = shape gap < 1 cm (arm-vs-world margin), BAND = < 2 cm (push-start band). Fingers are exempt from the world (M289) and held objects are exempt from the world; their flags are informational.

| moment | shape | shape gap bench / hotplate / beaker (mm) | real gap bench / hotplate / beaker (mm) | flags (shape) |
|---|---|---|---|---|
| M1 approach | upper arm | +262.7 / +295.0 / +340.8 | +294.9 / +312.3 / +353.2 | |
| M1 approach | forearm | +99.7 / +167.6 / +196.7 | +147.3 / +207.0 / +244.3 | |
| M1 approach | link 7 | +30.2 / +96.6 / +107.9 | +77.1 / +147.5 / +178.3 | |
| M1 approach | palm | +56.1 / +99.1 / +147.7 | +73.1 / +110.2 / +162.3 | |
| M1 approach | fingers | +7.7 / +50.1 / +81.1 | +34.0 / +66.5 / +85.2 | MARGIN(bench) (exempt) |
| M2 grasp 45% | upper arm | +162.3 / +226.7 / +267.8 | +230.4 / +295.5 / +335.9 | |
| M2 grasp 45% | forearm | **-9.6** / +71.5 / +77.9 | +33.9 / +109.7 / +125.5 | **MARGIN(bench)** |
| M2 grasp 45% | link 7 | +12.0 / +64.7 / **-10.0** | +16.4 / +81.2 / +59.5 | BAND(bench), **MARGIN(beaker)** |
| M2 grasp 45% | palm | +15.2 / +19.2 / +29.2 | +16.4 / +25.5 / +43.5 | BAND(bench), BAND(plate) |
| M2 grasp 45% | fingers | +0.3 / +0.8 / **-16.8** | +14.5 / +33.5 / +5.9 | MARGIN (exempt) |
| M3 grasp 60% | upper arm | +159.5 / +226.8 / +264.6 | +227.6 / +295.5 / +332.8 | |
| M3 grasp 60% | forearm | +0.3 / +71.8 / +77.7 | +40.2 / +110.0 / +125.4 | MARGIN(bench) |
| M3 grasp 60% | link 7 | +26.0 / +65.1 / **-10.2** | +30.4 / +81.6 / +59.3 | MARGIN(beaker) |
| M3 grasp 60% | palm | +29.2 / +19.6 / +29.1 | +30.5 / +26.0 / +43.3 | BAND(plate) |
| M3 grasp 60% | fingers | +14.4 / +1.8 / **-16.5** | +28.5 / +34.0 / +6.2 | exempt |
| M4 lift start | upper arm | +161.0 / +227.1 / +266.2 | +229.1 / +295.8 / +334.3 | |
| M4 lift start | forearm | **-3.9** / +71.4 / +77.9 | +37.1 / +109.6 / +125.5 | **MARGIN(bench)** |
| M4 lift start | link 7 | +20.0 / +64.6 / **-9.9** | +24.5 / +81.1 / +59.5 | MARGIN(beaker) |
| M4 lift start | palm | +23.3 / +19.1 / +29.2 | +24.5 / +25.4 / +43.5 | BAND(plate) |
| M4 lift start | fingers | +8.3 / +7.7 / -23.0 | +22.5 / +40.4 / -1.2 | exempt |
| M4 lift start | held beaker | **-35.0** / +75.0 / - | +0.0 / +75.0 / - | exempt from world (M289); cap artifact |
| M5 carry | upper arm | +87.0 / +159.3 / +217.5 | +152.6 / +227.9 / +285.3 | |
| M5 carry | forearm | +68.4 / +64.3 / +77.4 | +109.1 / +102.7 / +120.9 | |
| M5 carry | link 7 | +126.6 / +59.7 / **-10.6** | +131.0 / +69.3 / +59.1 | MARGIN(beaker) |
| M5 carry | palm | +129.8 / +65.6 / +28.8 | +131.0 / +68.5 / +43.1 | |
| M5 carry | fingers | +115.0 / +47.0 / -23.4 | +129.1 / +61.1 / -1.4 | exempt |
| M5 carry | held beaker | +73.0 / +12.2 / - | +108.0 / +40.0 / - | BAND(plate) (exempt) |
| M6 over plate | upper arm | +75.3 / +125.2 / +210.1 | +119.2 / +173.4 / +266.7 | |
| M6 over plate | forearm | +62.0 / +62.9 / +77.3 | +93.5 / +102.1 / +118.4 | |
| M6 over plate | link 7 | +126.6 / +58.6 / **-10.8** | +131.0 / +65.8 / +58.9 | MARGIN(beaker) |
| M6 over plate | palm | +129.8 / +62.7 / +28.7 | +131.0 / +65.0 / +42.9 | |
| M6 over plate | fingers | +115.0 / +47.0 / -23.5 | +129.1 / +61.1 / -1.4 | exempt |
| M6 over plate | held beaker | +73.0 / +5.0 / - | +108.0 / +40.0 / - | exempt (M289) |
| M7 place down | upper arm | +101.3 / +151.4 / +227.5 | +141.5 / +196.5 / +282.3 | |
| M7 place down | forearm | +49.2 / +41.4 / +78.2 | +88.6 / +80.2 / +122.2 | |
| M7 place down | link 7 | +89.9 / +21.9 / **-9.9** | +94.4 / +33.0 / +59.8 | MARGIN(beaker) |
| M7 place down | palm | +93.1 / +26.9 / +29.5 | +94.4 / +31.3 / +43.8 | |
| M7 place down | fingers | +78.2 / +10.2 / -22.7 | +92.4 / +24.4 / -1.1 | BAND(plate), MARGIN(beaker) (exempt) |
| M7 place down | held beaker | +35.0 / -33.0 / - | +70.0 / +2.0 / - | exempt (M289); real beaker 2 mm above the plate, as set |
| M8 release | upper arm | +101.7 / +151.8 / +227.8 | +142.0 / +196.8 / +282.6 | |
| M8 release | forearm | +48.9 / +41.1 / +78.0 | +88.3 / +79.8 / +122.1 | |
| M8 release | link 7 | +89.6 / +21.6 / **-10.0** | +94.1 / +32.6 / +59.6 | MARGIN(beaker) |
| M8 release | palm | +92.8 / +26.6 / +29.4 | +94.1 / +30.9 / +43.6 | |
| M8 release | fingers | +77.9 / +9.9 / -16.7 | +92.1 / +24.1 / +5.9 | MARGIN(plate), MARGIN(beaker) (exempt) |
| M9 recovery | upper arm | +141.7 / +192.3 / +267.8 | +178.1 / +231.5 / +317.0 | |
| M9 recovery | forearm | +79.2 / +84.4 / +127.2 | +118.4 / +123.1 / +169.5 | |
| M9 recovery | link 7 | +88.6 / +28.9 / +38.6 | +93.1 / +63.1 / +108.8 | |
| M9 recovery | palm | +91.7 / +48.4 / +78.4 | +93.0 / +60.5 / +92.8 | |
| M9 recovery | fingers | +76.9 / +8.9 / +17.4 | +91.0 / +33.8 / +24.6 | MARGIN(plate), BAND(beaker) (exempt) |

Summary of what would slow or push (shape rows, non-exempt):
1. **Forearm vs the bench at the grasp (M2, M3, M4): flagged, real clearance 34 to 40 mm.** The forearm capsule (radius 7.6 cm) overhangs the bench edge by up to 10 mm in the shape (-9.6 mm at M2). False alarm: it would slow or push the elbow during the grasp. Looseness below.
2. **Link 7 vs the beaker at the grasp, lift, carry, place (M2 to M8): flagged; real clearance 59 to 70 mm.** The beaker capsule's end caps (3.5 cm past the rim) reach into the wrist's space. False alarm for the same reason as section 4.
3. **Palm vs the hotplate and the bench (BAND at M2, M3, M4): real clearance 25 to 43 mm; the shape is close (19 to 29 mm).** Palm capsules are within 2 cm of the plate at the grasp (shape +19 to +29 mm), so the push-start band would be active there; the real clearance is 25 to 43 mm.
4. Fingers and the held object: flagged but exempt (fingers from the world, held from the world, M289). The held capsule's -35 mm at the bench is the same cap artifact.

What does NOT flag: upper arm (all moments, more than 1 m from the bench), palm at the carry and place.

Conclusion for the question "can the shapes pick and place?": the filter's shapes would not block the pick-and-place (no real collision in any moment; real clearances 14 mm or more to the bench and the plate, except the held beaker touching the bench at M4 and resting on the plate at M7, both intended). But the forearm and link 7 shapes would flag and slow the filter at the grasp for false reasons. The fix candidates (for David to decide): tighter forearm capsules (section 7), and a non-cap beaker shape (section 4).

## 7. Forearm looseness and the two-capsule forearm

Looseness: gap between the forearm capsule surface and the real forearm geometry (mm), at 10 points from the elbow (joint 4 anchor, t = 0) to the wrist (joint 7 anchor, t = 1). Points use the real forearm vertices within a slice of ±1/18 of the length, at the current capsule radius 7.61 cm. Over the nine moment poses.

| t | 0.00 | 0.11 | 0.22 | 0.33 | 0.44 | 0.56 | 0.67 | 0.78 | 0.89 | 1.00 |
|---|---|---|---|---|---|---|---|---|---|---|
| loosest (max over poses), mm | 0.0 | 1.0 | 2.2 | 6.3 | 15.3 | 22.5 | 21.8 | 24.6 | 34.1 | 34.1 |
| tightest (min over poses), mm | 0.0 | 1.0 | 2.2 | 6.3 | 15.3 | 21.8 | 20.4 | 22.8 | 33.2 | 30.9 |

The current capsule is 15 to 34 mm looser than the real forearm from the middle to the wrist, and it always encloses (no negative value).

Two-capsule forearm (fit on the same nine moment poses; both capsules on the elbow-wrist line; split chosen to minimise the total volume, searched in steps of 0.01 along the line; each capsule's radius is the largest distance of the vertices it covers, assigned by their position along the line):

- Split point: 0.49 of the elbow-wrist line (length 21.8 cm); elbow capsule from t 0 to 0.49, wrist capsule from 0.49 to 1.
- Radii: elbow 7.61 cm, wrist 5.57 cm (the single capsule was 7.61 cm).
- Volume: 5608 cm^3 for the two, against 5823 cm^3 for one (-4 %). The elbow piece keeps the fat radius because the forearm's biggest cross-section is near the elbow.

Gaps of the two-capsule forearm (mm; elbow / wrist capsule; shape to bench / hotplate / beaker, min of the two):

| moment | elbow / wrist capsule to bench | min to bench / plate / beaker | flags |
|---|---|---|---|
| M1 approach | +198.8 / +120.1 | +120.1 / +188.0 / +217.1 | |
| M2 grasp 45% | +80.7 / +10.8 | +10.8 / +91.9 / +98.3 | BAND(bench) |
| M3 grasp 60% | +80.3 / +20.7 | +20.7 / +92.2 / +98.1 | |
| M4 lift start | +80.7 / +16.5 | +16.5 / +91.8 / +98.3 | BAND(bench) |
| M5 carry | +69.4 / +88.8 | +69.4 / +84.7 / +97.8 | |
| M6 over plate | +62.2 / +82.4 | +62.2 / +83.3 / +97.7 | |
| M7 place down | +61.6 / +69.5 | +61.6 / +61.8 / +98.6 | |
| M8 release | +61.6 / +69.3 | +61.6 / +61.5 / +98.4 | |
| M9 recovery | +100.9 / +99.6 | +99.6 / +104.8 / +147.6 | |

Two capsules remove the grasp-time flags on the elbow piece (+80 mm to the bench) and leave a BAND flag only on the wrist piece at M2 and M4 (10 to 17 mm to the bench, real clearance 34 to 40 mm). The volume gain is small; the wrist radius is the one that matters.

## 8. Decision rows added (SF-8 to SF-13, commit 734734d)

- SF-8: lab map follows bench_open; gas manifold and knobs fixed; the filter refuses a mismatched map.
- SF-9: torso exemption limited to the pairs that touch by construction (both shoulder mounts; the left upper arm).
- SF-10: held objects exempt from their whole own arm (extends SF-4).
- SF-11: base and pedestal not checked against the world.
- SF-12: fallback held capsule height 8 cm (PROVISIONAL).
- SF-13: round objects are capsules; cylinders in the lab map may stay boxes.

## 9. Not done

- Stance selection was not run; the pick-and-place poses use the finder's reference staging stance (0.765, 1.085; pedestal 0.155 m) and the layout of TASK_CFG. The table is for that stance only.
- Real-geometry gaps for the torso were not computed (shape gaps only).
- The two-capsule forearm was fitted on the nine moment poses only, not on the 2000-pose build set; a fit on the wider set would be larger.
- Time per tick: target (pair checks under 1 ms, whole tick under 2 ms) not met (section 5).
- The beaker capsule question (section 4) is for David; not changed.
- robot_shapes.json was not changed (report only, as asked).
