# Keep-out follow-up: right-arm maps, ranking, real grasp-pose clearances (2026-11-05)

Read-only. Scratch worktree world-model-sources (tip c71d633) outside the repo. IK and collision checks through the verification's own pieces (src/grasp_rules.make_constraint, joint_bounds, src/arm_kinematics.solve_ik, batch_fk, src/arm_collision.ArmCollisionChecker, src/stance_checks.StanceChecks.place). No physics steps, no map builds, no cache writes, no stance search. Scratch scripts and JSON are outside the repo.

## 1. Right-arm maps and stamps

- data/reach_map: 24 grasp maps, 12 per arm (left_bin0 to left_bin11, right_bin0 to right_bin11), plus build_summary.json.
- Stamp check: the current model's stamp is computed by experiments/reach_map_build.py (stamp_config and stamp_of) for each arm. The builder is under experiments/, not src/ as the question said.

| Arm | Stamp in all 12 files | Current model stamp | Match |
|---|---|---|---|
| left | cc7abc885f48235d | cc7abc885f48235d | yes, all 12 |
| right | 1cb2f2ec3b284cba | 1cb2f2ec3b284cba | yes, all 12 |

- No left or right map is stale. The stamp hashes the bench base pose and pedestal from TASK_CFG (staging 0.765, 1.085, −90° and pedestal 0.155, reach_map_build.py:76 and :92). Changing those settings would make both arms stale.

## 2. Ranking score

**Order of the filters in the overlay (src/bench_stance.py overlay, 669 onward):**
1. Region reach: the cached per-object region (maps × spot × pedestal). Can drop bins, not cells.
2. Camera view: counts are multiplied by the view mask. Can zero a bin's count at a spot.
3. Obstacle filter (filter_obstacles, per object, 337-392): removes bins whose arm chain meets another object's cylinder. Can drop bins.
4. Support filter (support_ok, H): zeroes counts where the zombie arms come within the home clearance of a support. Can zero counts.
5. allowed mask (extra_blocked: furniture, home clearance, creep-in, carry slice): removes cells.
6. nobj per cell and spot: the number of objects with a count above zero. valid = cells where nobj equals the best nobj.

**Ranking (lexicographic, no weights).** The keys are applied in this order (bench_stance.py 758-772):
1. best_n: the number of objects covered at the cell (higher is better).
2. width: the widest run of consecutive pedestal heights valid at the cell, times pedestal_step (0.01 m; task_config.py, BENCH_STANCE_CFG "pedestal_step").
3. minc: the weakest covered object's bin count at the middle pedestal of that run.
4. edge: the distance from the cell to the nearest invalid cell, in metres, rounded to 0.01 m (distance transform).
5. dist_obs: the distance from the cell to the nearest other object's footprint, capped at 9.0 m (only when more than one object).

The first key is the most significant. The chosen cell is the one that wins all keys. The ranked list keeps stances at least rank_min_distance_m = 0.10 m apart and up to rank_keep = 4 entries (task_config.py:624-625).

**Where the score is read afterwards:**
- stance_select.select_bench_stance (stance_select.py 477 to 485): the full check runs on res["ranked"][:1 + full_check_fallbacks], with full_check_fallbacks = 3 (task_config.py:589). A stance that fails the full check falls back to the next ranked one.
- The chosen stance is written back to task_cfg (stance_select.py 515 to 519). The write-back and the report use the first ranked entry (533).

**Where a per-cell clearance term would go:** add a key after dist_obs in the keys stack and the lexsort (bench_stance.py 758-759), and in the ranked tuple (764). The score tuple (772) and the report would need the new field too. stance_select reads it only through res["ranked"] and "score".

**0.16 survivors if the obstacle filter were skipped:** NOT COMPUTED. The overlay needs the scene's camera, furniture and support context, and running it here would mean building that context (a stance-search-like run). The nearest things I could read:
- The refusal message gives the raw best count 3 for the beaker (reach × view, no obstacles), so the best cell would reach the beaker in at most 3 of the 7 bins before the other filters.
- From the grasp-pose IK (section 3), the left arm can solve the grasp for bins 11 and 0 at 0.16 m (and bin 1 only with a hotplate overlap). Bins 9, 10, 2 and 3 do not solve at this stance. So the IK-feasible survivors at 0.16 m would be bins 11 and 0 (plus bin 1 as an overlapping pose), not all seven bins.

**Stored score of the chosen stance** (data/stance_select_report.json): (best_n 2, width 0.12 m, minc 2, edge 0.06, dist_obs 0.397), counts beaker 2 and hotplate 2.

## 3. Real grasp-pose clearances (IK and MuJoCo geom distances)

Setup: base stance (0.635, 1.135, −90°), pedestal 0.15 m (pedestal_override_m, the value runs used). Beaker bottom centre at the layout's beaker xy, y 0.66. Pad centre at the pipeline's grasp height (0.962 + 0.525 × 0.10 = 1.0145 m). Pre-grasp: 0.15 m back along the bin direction at the same height. Over-the-top: 5 cm above the hotplate top (z 1.078) above the pre-grasp point. Bin direction constraint and solver are the verification's (make_constraint, solve_ik with 25 starts: zombie pose plus 24 random; 150 iterations; accept if the pad is within 3 mm and the approach is within bin_half_deg and level). The solution is the one with the largest joint margin; the table gives the range over all accepted solutions.

Clearances are MuJoCo mj_geomDistance between the arm's geoms and the hotplate's real box geoms (base box 0.18 × 0.18 × 0.06 and top plate 0.15 × 0.15 × 0.008), the bench top, and the beaker's own wall (not counted as an obstacle). Negative = overlap. Convex-hull limits apply to mesh geoms, so overlaps are upper bounds.

Joint margin: the solver's bounds keep every joint at least joint_margin_deg = 10° from its limits (task_config.py:196, GRASP_MAP_CFG). The finder's stored IK uses a 30° margin, which is a different rule (STANCE_REQUIREMENTS).

**Left arm, control gap 0.20 (beaker 0.85, hotplate 0.65)**

| Bin | Pose | IK | Hand and fingers to hotplate, mm (range over accepted solutions; chosen) | Forearm to hotplate, mm (chosen) | Hand and fingers to bench, mm | Fingers to own beaker wall, mm | Chosen joint margin, deg |
|---|---|---|---|---|---|---|---|
| 11 | grasp | ok (8) | 52.4 to 63.8 (52.4) | 109.9 | 24.6 | 5.6 | 22.5 |
| 11 | pre-grasp | ok (8) | 146.7 to 147.1 | 232.2 | 32.3 | 105.7 | 13.6 |
| 11 | over | ok (11) | 155.3 to 162.9 | 235.7 | 94.0 | 105.7 | 30.5 |
| 0 | grasp | ok (6) | 9.6 to 21.8 (10.5) | 60.4 | 16.0 | 5.5 | 25.0 |
| 0 | pre-grasp | ok (9) | 79.0 to 89.6 | 187.4 | 37.0 | 105.7 | 34.2 |
| 0 | over | ok (8) | 93.3 to 105.9 | 196.6 | 102.1 | 107.0 | 38.4 |
| 1 | grasp | ok (16) | −0.3 to 5.9 (4.4) | 48.4 | 23.0 | 5.6 | 14.2 |
| 1 | pre-grasp | ok (10) | 19.1 to 19.6 | 120.1 | 33.3 | 105.4 | 10.0 |
| 1 | over | ok (12) | 51.3 to 56.9 | 129.7 | 95.5 | 104.9 | 10.0 |
| 9 | all three | NO (best error 80, 171, 160 mm) | | | | | |
| 10 | all three | NO (best error 58, 48, 23 mm) | | | | | |
| 2 | all three | NO (best error 35, 152, 91 mm) | | | | | |
| 3 | all three | NO (best error 100, 240, 243 mm) | | | | | |

**Left arm, layout A gap 0.16 (beaker 0.81, hotplate 0.65)** (layout B, beaker 0.85 and hotplate 0.69, gives the same numbers to within 1 mm, and is in the JSON)

| Bin | Pose | IK | Hand and fingers to hotplate, mm (range; chosen) | Forearm to hotplate, mm | Hand and fingers to bench, mm | Fingers to beaker wall, mm |
|---|---|---|---|---|---|---|
| 11 | grasp | ok (10) | 13.5 to 17.3 (16.3) | 72.4 | 23.7 | 5.4 |
| 11 | pre-grasp | ok (9) | 109.6 to 110.1 | 201.4 | 35.7 | 106.0 |
| 11 | over | ok (9) | 121.3 to 131.5 | 206.2 | 84.8 | 107.2 |
| 0 | grasp | ok (8) | −27.9 to 5.5 (5.5) | 62.2 | 17.1 | 5.6 |
| 0 | pre-grasp | ok (5) | 51.1 to 67.8 | 169.2 | 45.2 | 105.9 |
| 0 | over | ok (12) | 72.0 to 89.1 | 178.4 | 101.1 | 107.0 |
| 1 | grasp | ok (9) | −27.9 to −23.8 (−27.3): overlaps the hotplate | 10.3 | 23.5 | 5.5 |
| 1 | pre-grasp | NO (best error 7.2 mm) | | | | |
| 1 | over | NO (best error 6.1 mm) | | | | |
| 9, 10, 2, 3 | all | NO | | | | |

**Right arm** (the stance was chosen for the left arm; the right arm's grasp did not solve for any bin at either layout)

| Bin | Control 0.20: grasp | Control: pre-grasp | Control: over | A 0.16: pre-grasp | A: over |
|---|---|---|---|---|---|
| 9, 10, 11, 0 | NO (best errors 205 to 441 mm) | NO | NO | NO | NO |
| 1 | NO | ok: hand to hotplate 3.9 to 5.3 mm; forearm 91 mm | ok: 44.9 to 52.0 mm | ok: 3.7 to 14.4 mm | ok: 38.8 to 55.6 mm |
| 2 | NO | ok: −43.2 to −31.6 mm (overlap); forearm 28 mm | ok: 14.5 to 27.9 mm | ok: −36.6 to −29.9 mm (overlap) | ok: 17.8 to 25.9 mm |
| 3 | NO | NO | NO | NO | NO |

- The right arm's bin 1 and bin 2 pre-grasp and over-the-top poses solve, but at the near side and with overlaps into the hotplate box. These solutions are not usable as they stand.
- The right arm's home hand: the right tool point at the zombie pose is at (0.484, 0.834, 1.139) in the demo stance. That is 0.241 m from the hotplate centre. The right hand body's nearest geom distance to the hotplate box is 192 mm (194 mm at layout B). The design plan says "the right hand's home is only 18 cm from the hotplate" (docs/design_plan.md:323). My measurement is about 19 cm to the box, and 24 cm to the centre. It is stance-specific, so it may differ at another stance.

**Comparison with the earlier estimates** (docs/reports/keepout_direction_study_20261105.md, §4):
- Far-side fingertip to hotplate edge: estimate 61 mm at 0.16 m and 101 mm at 0.20 m. Measured at the far-side grasp (bin 11, left): hand and fingers 16 mm at 0.16 m and 52 mm at 0.20 m. The grasp point is closer to the hotplate than the estimate assumed, because the real hand is oriented differently from the zombie pose.
- Hand body on the near side: estimate −22 mm at 0.16 m and +19 mm at 0.20 m. Measured near-side (bin 1, grasp): −28 mm at 0.16 m, and +4 mm at 0.20 m (bin 1 chosen 4.4 mm). The estimate is confirmed in direction, and the real shape is closer than the estimate at 0.20 m.
- Bench: estimate from the finger bottoms, 17 to 19 mm; measured 16 to 25 mm at grasp. Consistent. The finder's stored 18.6 mm therefore most likely reflects the bench.

## 4. Rejection and what could be saved

- The raise: src/bench_stance.py:733-734 (inside overlay, when best_n is 0).
- The catch: src/stance_select.py:461 (`except bs.BenchStanceError`). At k = 1 the message is stored as out["stop"] (464) and the function returns normally. There is no raise. (The other catch, 554, is in the pour helper.)
- The escape: src/mpvi_pipeline.py:69-70 (`if not sel["ok"]: raise StanceSelectionError(sel["stop"])`). This is why make_pipeline_task raises and demo_video.record_only aborts.
- Fields available at the raise point that a saved record could hold: the object ids, phrases, grasp xy, z_points, depth and footprint radius of each object (the objects list); the per-object raw best count (counts(regs)) and the blocker names (self_blockers, bench_stance.py 621-629); the maps stamp (maps_stamp, 64-66); the bench yaw and bench_open; the BENCH_STANCE_CFG values; the TASK_CFG layout (beaker and hotplate xy); the pedestal grid (qs); the arm; the code version (git commit).
- Random scene generator or seed: none exists. The only random draws are IK starts (full_check.py:250 seeded 11; home_clearance_map.py:158 seeded 0), the home variation seed (task_config, home_variation_seed 0), and the pour seeds (pour_scene.py). The layout search (experiments/beaker_hotplate_layout_search.py) is a deterministic grid. A future generator would need its seed saved with each rejected scene.

## 5. Left-arm hard-wiring and what each place needs

| Place (file:line) | Hard-wired | What it needs for the right arm or both arms |
|---|---|---|
| src/task_config.py:595 BENCH_STANCE_CFG "arm" = "left" | the arm whose maps are used | an arm setting per run, or a both-arms option |
| src/task_config.py:86 TASK_CFG "arm" = "left" (legacy) | scripted grasp poses (legacy) | none (legacy) |
| src/task_config.py:303 layout search arm = "left" | legacy layout search | none (legacy) |
| src/stance_select.py:73 ctx maps filtered by cfg["arm"] | one arm's maps | both arms' maps, combined per cell |
| src/stance_select.py:402, 455, 460, 553 (overlay and carry calls use cfg["arm"]) | one arm | per-arm overlay, then a per-cell max |
| src/stance_select.py:574 load_maps(sides=(cfg["arm"],)) | loads only the left maps | load both |
| src/mpvi_pipeline.py:30, 40 make_pipeline_task arm="left" | manipulation arm | the manipulation arm per step |
| src/full_check.py:47 SIDE = "left" | every IK check (chain, grasp cfg, joint bounds, shoulder, zombie pose, checker) | a parameter per check |
| src/full_check.py:89, 233-238, 251, 309, 331 | left chain, shoulder, zombie pose, collision checker (uses home_q_right as the other arm), maps (SIDE, k) | the same, per arm |
| src/full_check.py:240 home Rh from home_q_left | right home frame | the right home frame |
| src/home_clearance_map.py:256 left hand tcp and gripper_frame(m, "left") | the hand whose pads are checked for the held beaker | the right hand's pad frame |
| src/stance_finder.py:260, 321, 331 | arm feasibility per point (both arms already considered per point label) | none (already both-arm) |
| src/bench_stance.py 248, 257, 681, 694 (cfg arm default) | configurable, not hard-wired | none |

What the right arm already has: shoulder (rg shoulder right at (0, −0.031, 0.698)), home pose (home_q_right in TASK_CFG), joint bounds and a grasp configuration (grasp_cfg for right works), 12 maps with a current stamp, and both arms at home in stance_checks.place. What is missing: the full check's single SIDE constant, the held-beaker hand clearance for the right pad, and a combined two-arm score.

What matters for the right arm's approach: the right home hand is about 19 cm from the hotplate box (section 3), and its grasp of the beaker does not solve from this stance at all. A right-arm plan would need a stance chosen for the right arm.

## 6. Over-the-top and pre-grasp

- The pre-grasp poses (0.15 m back at grasp height) are in section 3. Left arm, control: bins 11, 0 and 1 solve, with hand to hotplate 147, 90 and 19 mm. At 0.16 m: bins 11 and 0 solve (110 and 51 to 68 mm); bin 1 does not.
- The over-the-top waypoint (hand 5 cm above the hotplate top, above the pre-grasp point) solves for bins 11, 0 and 1 with the left arm (control: 156, 93 and 51 mm to the hotplate; A: 121 mm, 72 to 89 mm, and bin 1 NO).
- The current filter's treatment of samples above z = 1.028 (bench_stance.py 383 to 385), exact:
  - dz = max(z0 − z, z − z1, 0), with z0 = 0.960 and z1 = 1.028 for the hotplate;
  - d = min over the chain samples of sqrt( max(hypot(x − cx, y − cy) − r, 0)² + dz² ), with r = 0.09;
  - a sample is blocked when d < r_arm + half the step, with r_arm = 0.09 (0.09 + 0.03 + 0.06 = 0.18 total; see §2 of the study for the exact threshold).
- Consequence for a 5 cm over-the-top path (z = 1.078, dz = 0.050): a sample is blocked if sqrt((h − 0.09)² + 0.05²) < 0.093, which means h < 0.168 m (within 16.8 cm of the hotplate axis). Above z = 1.121 (dz ≥ 0.093) no sample is ever blocked by the hotplate. The shoulder chain at z ≈ 1.2555 is never blocked by the hotplate.
- So the filter excludes a 5 cm over-the-top path in a cylinder of radius 16.8 cm around the hotplate axis, and nothing above 1.121 m.

## 7. Options (nothing built)

The design options from the keep-out study (§7) stand. Two additions from this study:
- The grid's reachability claim for bins 9 and 10 does not hold for the verification's IK at this stance (best errors 23 to 80 mm). That is a discrepancy between the map and the solver to check before any rule change.
- Several accepted grasp solutions overlap the hotplate box (A, bins 0 and 1; B, bins 0 and 1). The verification's collision check does not include the hotplate, so a solution that passes it can still overlap the hotplate. A hotplate check on the solutions would be needed for any direction-aware rule.

## 8. Not done

- The score for the 0.16 survivors was not computed (§2).
- The right-arm grasp could not be solved at this stance for any bin (§3); the right arm's pre-grasp and over solutions near the hotplate overlap it.
- Caveats: convex-hull distances for mesh geoms; one stance and pedestal (0.635, 1.135, 0.15); 25 IK starts per pose.
