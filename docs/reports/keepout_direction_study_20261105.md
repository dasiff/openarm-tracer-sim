# Obstacle keep-out: direction and arm study (2026-11-05)

Read-only. Scene and robot loaded with load_task (no physics steps). The keep-out was run through the code's own filter_obstacles (src/bench_stance.py:337-392) on a grasp grid, and a per-bin closest-approach check was made for representative grasps. Scratch scripts are outside the repo. No repo file changed.

## 1. Bins

- Approach bin k: the hand moves along direction k × 30° in the robot's base frame (k = 0 forward, positive k toward the robot's left). 12 bins exist (k = 0 to 11, src/bench_stance.py region builder, docs/design_plan.md:81).
- Used at the bench: approach_bins = (9, 10, 11, 0, 1, 2, 3), 7 bins (src/task_config.py:588-595). Bins 4 to 8 point back at the robot and are not used.
- Arms: each arm has its own reach maps (data/reach_map: 12 left + 12 right files). Stance selection uses only arm = "left" (src/task_config.py:595; apply_stance_selection loads sides=(cfg["arm"],), src/stance_select.py:571).
- The reach-map region is built per object depth, bench, and per pedestal; the obstacle filter runs after the cache, per run (src/bench_stance.py get_region docstring), so changing the keep-out needs no map or cache rebuild.
- "Best count 3 without obstacles": the stance cell where the beaker is reachable by 3 of the 7 used bins with no obstacle filter. "0 with them": the same cell with the obstacle filter, which removes all 3. Each count is the number of bins reachable at that cell (bench_stance.py counts, lines 278-281; the message at lines 728-734).
- Both arms exist in the maps; the selection ignores the right arm.

## 2. Keep-out per bin, arm and layout

Geometry used by the test (base frame; the hotplate is the only obstacle):
- Circle radius 0.09 m (the hotplate's larger half-size, src/stance_select.py:123-124), axis at the beaker grasp point + the hotplate offset.
- Threshold for a sample: radius + obstacle_margin_m (0.03) + arm_radius_m (0.06) + half the sample step. Blocked when the sample is closer than that to the axis (distances are to the circle's edge) within the obstacle's height range.
- Each bin's arm chain: shoulder → pre-grasp point (0.15 m back along the bin direction) → grasp point, 24 samples per segment.

Layouts: control gap 0.20 (beaker 0.85, hotplate 0.65 in the demo; the relative offset is what the test uses); A (beaker 0.81, hotplate 0.65, gap 0.16); B (beaker 0.85, hotplate 0.69, gap 0.16). The hotplate offset in the base frame is (0, −0.20) for the control and (0, −0.16) for A and B, because the stance is fixed at heading −90°. The test depends only on that offset, so A and B are identical by construction.

### 2a. Share of grid grasp points kept per bin (q = 0.10; grid 0.10–0.90 m forward, ±0.60 m lateral, 0.02 m step)

| Bin (deg) | Control 0.20, left | Control 0.20, right | A 0.16, left | A 0.16, right | B 0.16, left | B 0.16, right |
|---|---|---|---|---|---|---|
| 9 (−90) | 0.958 | 0.935 | 0.000 | 0.000 | 0.000 | 0.000 |
| 10 (−60) | 0.979 | 0.966 | 0.000 | 0.000 | 0.000 | 0.000 |
| 11 (−30) | 0.986 | 0.977 | 0.000 | 0.000 | 0.000 | 0.000 |
| 0 (0) | 0.979 | 0.969 | 0.000 | 0.000 | 0.000 | 0.000 |
| 1 (30) | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 |
| 2 (60) | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 |
| 3 (90) | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 |
| 4 to 8 (not used) | 0.00 to 0.90 | 0.00 to 0.87 | 0.000 | 0.000 | 0.000 | 0.000 |

The verdict is identical for every bin at 0.16 (all removed) and differs by bin at 0.20. Not identical in the control: the bins fail or pass according to the approach side, not to the stance.

### 2b. Representative grasp (base frame G = (0.475, 0.215), left arm, q = 0.10; the demo's control stance)

| Bin | Gap 0.20: grasp to surface (m) | Closest sample (segment) | Distance to surface (m) | Threshold (m) | Margin (m) | Result | Gap 0.16 (A and B): margin (m) | Result |
|---|---|---|---|---|---|---|---|---|
| 9 | 0.110 | pre-grasp → grasp | 0.110 | 0.0933 | +0.017 | pass | −0.023 | blocked |
| 10 | 0.110 | pre-grasp → grasp | 0.110 | 0.0933 | +0.017 | pass | −0.023 | blocked |
| 11 | 0.110 | pre-grasp → grasp | 0.110 | 0.0933 | +0.017 | pass | −0.023 | blocked |
| 0 | 0.110 | pre-grasp → grasp | 0.110 | 0.0933 | +0.017 | pass | −0.023 | blocked |
| 1 | 0.110 | pre-grasp → grasp | 0.083 | 0.0933 | −0.010 | blocked | −0.045 | blocked |
| 2 | 0.110 | shoulder → pre-grasp | 0.013 | 0.0988 | −0.086 | blocked | −0.099 | blocked |
| 3 | 0.110 | shoulder → pre-grasp | 0.000 (inside) | 0.1004 | −0.100 | blocked | −0.100 | blocked |
| 4 | 0.110 | shoulder → pre-grasp | 0.000 (inside) | 0.1020 | −0.102 | blocked | −0.102 | blocked |
| 5 | 0.110 | shoulder → pre-grasp | 0.034 | 0.1034 | −0.069 | blocked | −0.085 | blocked |

Why the verdict is the same for every bin at 0.16: the grasp point is always exactly the gap from the hotplate axis. At 0.16 it is 0.070 m from the surface, inside the 0.0933 m threshold, so the grasp-point test fails before the arm path is considered. That is the blocker for every bin.

Why the verdict differs at 0.20: the grasp point (0.110 m from the surface) passes, so the outcome is decided by the pre-grasp and shoulder samples. Bins 0 and 9 to 11 put the pre-grasp point on the far side (away from the hotplate), so their closest sample is the grasp point. Bins 1 to 5 put the pre-grasp point or the shoulder segment on the hotplate side, so the arm path enters the keep-out.

Right arm: the same pattern (passing bins 0, 9, 10, 11 at 0.20 with shares 0.94 to 0.97; none at 0.16). Its shoulder is at y = −0.031.

Stored-stance check (q = 0.10): the stored chosen stance (0.755, 1.145) has the beaker at G = (0.485, 0.245). At that point the test passes bins 0, 9, 10 and 11 and blocks 1, 2 and 3. The stored selection reports 2 bins for the beaker at that cell, so the reach or camera filters remove two more of the four. The stored file does not name the bins.

## 3. Vertical geometry

| Item | Value | Source |
|---|---|---|
| Hotplate footprint | rectangle, base box half-sizes 0.09 × 0.09 m, base height 0.06 m | models/objects/hotplate.xml; model geom sizes |
| Hotplate top plate | half-sizes 0.075 × 0.075 m, 8 mm thick, centre z 1.024 m | model |
| Hotplate top surface | z = 1.028 m (site hotplate_top) | model |
| Obstacle z range used by the keep-out | z0 = 0.960 (bench top), z1 = 1.028 (hotplate top) | _body (src/stance_select.py:123-132), sim world |
| Keep-out footprint used | circle radius 0.09 m (the larger half-size, not the corner distance 0.127 m) | src/stance_select.py:123-124 |
| Beaker | outer diameter 0.070 m (radius 0.035), height 0.100 m, base z 0.962 | src/task_config.py:116-117; model |
| Grasp band | 45 to 60 % of the height: pad z 1.007 to 1.022 | BEAKER_CFG grasp_band (task_config.py:136) |
| Grasp pad centre | 1.0145 m (fraction 0.525) | PICK_PLACE_CFG grasp_height_frac (task_config.py:415) |
| Fingers at grasp (vertical span, tool x ±0.0305) | z 0.984 to 1.045 | model extents + pad z (estimate) |
| Hand body at grasp (tool x ±0.0285) | z 0.986 to 1.043 | model extents + pad z (estimate) |
| Pad centre relative to hotplate top | 1.4 cm below it | computed |
| Pad centre relative to bench top | 5.45 cm above it | computed |

Vertically relevant parts of the keep-out:
- The grasp points are inside the obstacle's height range (1.0145 is below 1.028), so the grasp-point test is effectively a 2D circle at grasp height.
- The fingers and hand straddle the hotplate top: they are level with it. A chain sample above 1.028 gets the vertical distance added (dz, bench_stance.py:384-385), so the shoulder → pre-grasp segment, which starts at z 1.2555 (shoulder at q = 0.10), is partly relieved above the hotplate top.
- The test is therefore 2D at grasp height and 3D only for the upper part of the arm chain. The circle uses 0.09 m, not the box corner distance 0.127 m, so the hotplate's corners are not covered by the circle.

## 4. Hand and finger geometry (model, zombie pose; the tool frame is the hand_tcp body)

Model shapes read from the compiled model (meshes, no stepping), in the tool frame:

| Part | Approach axis (tool z) | Closing axis (tool y) | Width (tool x) |
|---|---|---|---|
| Left finger | −0.086 to +0.009 m (tip 9 mm past the tool point) | −0.003 to +0.032 m (zombie pose) | ±0.0305 m |
| Right finger | same as left | −0.032 to +0.003 m | ±0.0305 m |
| Hand body | −0.0915 to −0.0755 m (8 to 9 cm behind the tool point) | ±0.084 m | ±0.0285 m |
| Pad centre | +0.0025 m | 0 | 0 |

- Finger opening maximum: 0.044 m each (`gripper_frame`). The pads close on the beaker's 70 mm outer diameter, leaving about 7 mm a side (src/task_config.py:116 comment).
- Approach axis: tool z. Closing axis: tool y. Both are derived from the compiled model (src/gripper_geometry.py `gripper_frame`).

Clearance from the hotplate's edge (estimates from these sizes, approach aligned with the line through the hotplate and the beaker):

| Case | Nearest part | 0.16 m | 0.20 m | Basis |
|---|---|---|---|---|
| Far side (hand behind the beaker, tool toward the hotplate) | finger tip, 9 mm past the tool point | 0.061 m | 0.101 m | estimate |
| Far side | beaker wall (not part of the gripper) | 0.035 m | 0.075 m | geometry |
| Near side (hand between beaker and hotplate, tool away from the hotplate) | hand body (8 to 9 cm behind the tool point) | −0.022 m (overlaps the footprint by 2.2 cm) | +0.019 m | estimate |
| Near side | finger base (8.6 cm behind) | −0.016 m | +0.024 m | estimate |

Measured (stored data, not an estimate): the finder's IK verification at the chosen stance for the 0.20 layout (src/task_config.py:31-38 records it) gives a hand and finger clearance of 18.6 mm at grasp 50 %, 18.8 mm at 45 %, and 24 to 30 mm at higher grasps, against a required 10 mm. The stored data does not say which object is nearest. The 18 to 19 mm is consistent with the finger bottoms sitting about 17 to 19 mm above the bench top at 45 to 50 %, so the nearest object is probably the bench, not the hotplate.

## 5. Stored arm and bin findings

- Finder result (data/home_pose_search/search_rank1_widest_range/open_b45_selected.json): left arm, 12 IK points, all solved with the arm free (points_ok_arm_free 12, arm_blocked_points []), chosen stance (0.765, 1.085), heading −90°, pedestal 0.155. The stored IK points are not labelled by bin.
- Stance geometry at the finder stance: the beaker is 42.5 cm forward of the base and 23.5 cm to the robot's left, so the hotplate is at 3.5 cm left of the centre line, 20 cm to the right of the beaker. The left arm reaches the beaker from the robot's left, the far side from the hotplate.
- Stored selection (data/stance_select_report.json): chosen (0.755, 1.145), pedestal 0.10, beaker count 2 and hotplate count 2 (of 7 bins). Bins not stored.
- Does the stored data say a far-side bin passes at the standard layout? Not named. The reconstruction in 2b gives bins 0 and 9 to 11 passing the keep-out at the stored grasp point.
- Pipeline pick arm (demo layout): left. make_pipeline_task defaults to arm = "left" (src/mpvi_pipeline.py:30), and BENCH_STANCE_CFG arm = "left" (task_config.py:595).
- The design plan says only the left arm was IK-verified: "left arm" in the finder scene, the B2.3 pilot "left arm, bin 0" (docs/design_plan.md:137).

## 6. Design plan check

| Statement (line) | Quote (abridged) | Built? |
|---|---|---|
| L72 | "both arms, all 12 approach directions, collision filter on, 24 maps" | Partly. The maps exist for both arms and all 12 bins. The selection uses only the left arm, and only 7 bins. |
| L81 | "Approach directions every 30 deg (12 bins that tile the circle), each accepting +/-15 deg." | Yes for the maps. The ±15° acceptance is in the map build. |
| L110 | "each labeled left / right / either (either prefers both)" | No. The keep-out and the selection take one arm (left) and no per-point label. |
| L113 | "rank 'either' points reachable by both arms higher" | No. Only the left arm is scored in the bench selection. |
| L123 | "the obstacle and support filters at plan time" | Yes, as a circle per other object and per bin. |
| L101 | "Over-the-top approach ... then descends to grasp height, then moves in sideways." | Partly. The skill moves over and down; the keep-out does not model the over-the-top path (the arm chain is shoulder → pre-grasp → grasp). |
| L403 | "The bin list may need approaches from above or below." | Not built (LATER, design plan 10). |
| L407 | "Keep the bench-specific values (... bin list, heading rule) as settings in BENCH_STANCE_CFG" | Yes (task_config.py:588-595, surfaces block). |

Note: the design plan has no statement on the obstacle keep-out's direction. The direction-aware intent comes from David's view of L110 and L113, not from a stated rule.

## 7. Options for a direction-aware keep-out (nothing built)

| Option | What it touches | What it leaves unchanged | Risk | Notes |
|---|---|---|---|---|
| A. Test the hand and finger envelope (the measured boxes from §4) for each bin instead of the padded point | filter_obstacles (src/bench_stance.py:337-392); a new shape from gripper_frame (src/gripper_geometry.py); the config keys obstacle_margin_m and arm_radius_m (task_config.py:604-606) | The reach maps, the region cache (the filter runs after it), the chain and the shoulder | Medium: changes every number at 0.20; needs revalidation | Likely passes far-side bins at 0.16 (fingertip 6.1 cm, estimate) |
| B. Per-bin keep-out using the real approach axis for the hand envelope (boxes oriented by the bin, from −0.0915 to +0.009 m along the bin direction) | the same as A, plus the bin's orientation | the same | Medium | Same as A, with the bin's own axis |
| C. Test only the part of the path on the hotplate's side (ignore grasp samples for bins whose hand is on the far side) | filter_obstacles, with a sign test on each sample against the approach axis | the reach maps and the cache | Low to medium | Makes the far side pass at 0.16 without changing the envelope; risks hiding a real collision |
| D. Keep the test; change only the placeholders | task_config.py:604-606 | everything else | Low in code, high in result | The flip is 0.09 + margin + arm radius + half step. A margin of 0.03 and an arm radius below 0.035 would pass 0.16; that is config only, but it does not test the real shape |
| E. Circle → rectangle (the hotplate's box) in the approach frame | filter_obstacles | the rest | Low | Does not help on the axis (edge distance is the same); helps only at the corners |

Recommended default: A, implemented as B (the hand envelope per bin's approach axis). It is the only option that tests the real shape, it keeps the cache and the maps, and it removes the direction-independent grasp-point failure at 0.16. NOT BUILT, needs decision.

## 8. Not done

- No stance selection or episode run, as asked.
- The clearances in §4 are estimates from the zombie-pose tool-frame shapes, not from the grasp pose (no IK solve; NOT BUILT: the grasp-pose extents would need an IK solve, leave for the design step).
- The bin list for the stored chosen stance is a reconstruction (§2b, stored-stance check), not a stored record.
