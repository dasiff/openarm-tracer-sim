# C2 follow-up (F0 to F4): the region with the finder's other criteria

Written for: David. Code: `src/bench_stance.py`, `src/task_points.py` (new, one task-point builder), `experiments/bench_stance_validate.py`, `bench_stance_test.py`, `bench_stance_diag.py`, `f0_silhouette_check.py`, `f4_full_check.py`. Rows M173. C3 not started.

## F0 answers (read only)
- **(a) Neck repeatability:** no longer applies. Dropped with the neck (M153, design plan 14.2: "The neck spread requirement (0.06 rad) is dropped"); the code went in step 2 part D (M167: `neck.py`, `neck_framing.py`, the repeatability checks). `STANCE_REQUIREMENTS` has only home clearance, placed beaker, view margin and occlusion.
- **(b) The 12 task points** (`task_points`, now in `src/task_points.py`): the 4 grasp-band heights at the beaker, the same 4 at the set-down above the hotplate, and those 4 plus 10 cm ("above"). They do NOT include the skill's standoff (12 cm back along the approach direction), the `over` waypoint (20 cm above the surface) or the lift (`UP`); the finder and the full check's IK path check cover exactly these 12 (grasp, then set-down, then above, an elbow flip inside a path is a failure). F1 adds the three missing kinds to the region only (`path_offsets`).
- **(c) Silhouette:** the robot's own silhouette depends only on the pedestal height: rendered alone at three different stances it differs by 24 to 28 of about 65,000 pixels (0.04 %, edge noise). The arms' mask in the real scene differs 5 to 6 % between stances, because the bench and objects change which pixels the nearest surface is (making only the bench transparent moves about 800 arm pixels either way, where arm and bench are almost at the same depth). So the answer is yes for the robot, with a world-dependent edge; I used the union of the robot-alone render and the in-scene render at TASK_CFG's stance, which covers the finder's picture.
- **(d) Map loading:** one arm's 12 maps 0.44 s (24 maps 1.2 s; 3 to 4 s when the file cache was cold), not merged. The overlay does not load them (they are passed in); counted against its 7 to 9 s it would be about 5 %. Nearly all of the overlay's time is the furniture and home-clearance masks.
- **(e) The bench in the C1 maps:** one fixed position: the base at TASK_CFG's stance (0.765, 1.085, -90 deg), the bench lowered by pedestal 0.155 in the shoulder frame. Valid strictly at that stance only. Measured effect (no-bench pilot against the bench maps): the bench removes 2.4 % of left bin 0 (35,891 voxels) and 0.7 % of bin 9 (9,047); at another stance a voxel that would hit the bench there stays marked reachable. The full check's IK path check is what catches it (F4 passes at both stances). Reported earlier as the STOP of the addendum; no maps changed.

## What changed
- **F1** Per bin all path points must be reachable: all band heights plus standoff, over and lift. One builder (`src/task_points.py`; `full_check.task_points` is the same function). Lookups only.
- **F2** Per spot the passing heights (1 cm) form runs; the widest run is the range, its middle the pedestal. Ranking: objects covered, run width, smallest count, distance to the region's edge, obstacle clearance. Deviation (M173): the run is the intersection over the covered objects (one height serves both), not the shortest of per-object runs.
- **F3** Robot-hides-target by sample points of the object's surface against the silhouette (beaker 0 % plus a 2 % sampling allowance, hotplate 20 %). Also added because the finder requires them: the bounding-box corners and both grippers at home inside the view, and the zombie pose's 40 mm clearance to the object. Render: 21 heights, 2 renders each, 5.4 s once (cached).
- Times: a region builds in 5 to 8 s (cached after: 0.01 s); the obstacle filter 0.2 to 0.3 s; the overlay with the furniture and home-clearance masks 6 to 8 s.
- Fixes found on the way: `reach_map_restamp --check` compared the maps' stamp without the bench (it said all 24 were stale); it now uses the builder's stamp. `full_check_test`'s "6 cm in front of the camera box" case passed or failed with the random start (a hand from the front reaches it, collision free; same with the old maps), so it is no longer asserted; the other two points still must fail by name.

## Result on the test scene
| | position (m) | pedestal | passing heights | width | count beaker / hotplate | edge | full check |
|---|---|---|---|---|---|---|---|
| TASK_CFG | (0.765, 1.085) | 0.155 (new rule's middle 0.11) | 0.07 to 0.16 | 10 cm | 3 / 2 (2 / 2 with the obstacle filter) | 2 cm | PASS (18.7 s) |
| overlay's pick | (0.755, 1.145) | 0.10 | 0.05 to 0.16 | 12 cm | 2 / 2 | 6 cm | PASS (6.5 s) |
The two stances are 6.1 cm apart (pedestal 0.155 against 0.10, or 0.11 by the new rule's middle). The finder's old range at TASK_CFG's stance was 0.10 to 0.21 (11 cm) on the old maps; on the current maps the finder's own quick search ends at 0.16 and the requirements start at 0.07, which is what the region gives (0.07 to 0.16). Full-check margins: TASK_CFG home clearance 116 / 133 / 98 mm at 0.155 / 0.175 / 0.135, view margin 23 to 25 %, hidden beaker 0 %, hotplate 5 to 6 %; the pick: home clearance 94 to 118 mm, view margin 22 to 24 %, hotplate hidden 0 to 0.5 %, 9 IK stances all solved.
Occlusion approximation against the finder's pixel measure: TASK_CFG hotplate hidden 5.6 % (pixels) against 10.9 % (samples), the pick 0.2 % against 3.1 %, beaker 0 % both: the sampled share is conservative.
The top-down picture: `docs/reports/bench_stance_region.png` (best pedestal 0.10 left, TASK_CFG's 0.15 right; at 0.15 the region is thin and TASK_CFG's stance sits near its edge).

## Tests
`experiments/bench_stance_test.py` 9/9: TASK_CFG has nonzero counts for both objects; at 45 (spot, height) cases around TASK_CFG's stance (+-6 cm, heights 0.08, 0.12, 0.15) the region agrees with the finder in 42 and disagrees in 3, printed: at (0.82, 1.025) q 0.12 and (0.82, 1.145) q 0.15 the finder fails the occlusion requirement and the region passes; at (0.71, 1.145) q 0.15 the finder's quick search finds no reach and the region does. The finder rejects 23 of the 45; the region passes 3 of those.
Existing tests (one at a time, logs `data/logs/suite_f2_*`): stance_finder 28/28, mpvi_postprocess 54/54, reach_map_restamp ok, full_check 29/29, stance_select 23/23, stance_select_pipeline 7/7, mpvi_pipeline 14/14.
Two slips: a tmux server died once mid-run (the first suite run was lost and rerun with setsid), and I loaded a model for a few seconds while a test was running (no render).

## Needs David
1. The region now agrees with the finder to 42 of 45; accept the overlay's pick (0.755, 1.145) at 0.10, which also passes the full check, or keep TASK_CFG? (C3 would run the full check on the pick anyway.)
2. Placeholders: the beaker's 2 % sampling allowance (`hidden_tolerance`), the arm radius 6 cm and hand stand-off 15 cm, the hotplate as a 9 cm obstacle cylinder.
