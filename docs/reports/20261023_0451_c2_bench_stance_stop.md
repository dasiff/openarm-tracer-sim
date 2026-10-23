# C2: inverse-reach region, obstacle filter, overlay. STOP at C2e (the best stance is 6.3 cm from TASK_CFG's)

Code: `src/bench_stance.py` (new), `BENCH_STANCE_CFG` in `src/task_config.py`, `experiments/bench_stance_validate.py`. Rows M170 to M172. Picture: `docs/reports/bench_stance_region.png`. Log: `docs/reports/logs/bench_stance_validate_*.log`. C1 passed (report `*_c1_bench_reach_maps.md`). C3 and the tests are NOT started.

## C2a: bench-approach bins
All 12 bins are level side grasps already (approach and finger axes within 5 degrees of horizontal), so none is "from above or below". What the bench rules out is the hand coming from behind the object: bins 4 to 8 point back at the robot. Stored: `approach_bins = (9, 10, 11, 0, 1, 2, 3)` (bins 9 and 3 are the hand moving along the bench edge). This is geometric, not from data; at the TASK_CFG stance only bins 0, 1, 10, 11 are reachable.

## C2b to C2d: built
- Region: one object, one arm (left): array [21 pedestal commands 0.05 to 0.25 m, 7 bins, 81 forward x 121 lateral cells of 1 cm] of "this bin reaches at least one grasp-band height", times the camera filter (the lowest and highest band point both at least 10 % inside the fixed eagle view). Cached by (arm, depth, band heights, bench_open, the maps' stamps, camera and robot numbers) in `data/bench_region_cache/`. **Build time 0.07 to 0.10 s per region** (array lookups; the camera poses come from the model once).
- Obstacle filter: cylinder (other object, +3 cm) against the chain shoulder -> pre-grasp point (15 cm back along the approach direction) -> grasp point, thickened by 6 cm. Not cached. **0.6 to 0.8 s per call** (2 objects, 21 heights, 7 bins).
- Overlay: shifts the regions to the object positions on a 1 cm world grid and scores (objects reachable, smallest count, distance to the edge of the region, obstacle clearance). With the furniture map and the arms-at-home clearance mask included: 7 to 9 s per call, nearly all of it those two masks (21 heights), not the region.

## Deviation to approve (row M172)
The spec puts three geometry filters inside the cached region. Only the camera filter (c) is: it depends only on the object relative to the base. Filters (a) (base footprint against legs, walls, furniture) and (b) (zombie pose 40 mm from the bench) depend on where along the bench the spot is, which a region that slides along the bench cannot hold, so the overlay applies them with the stance finder's own `FurnitureMap.blocked` and `HomeClearanceMap.clear_mask`. Same result per cell. Say if you want them in the region (then the cache key gets the bench's position).
Also placeholders to confirm: arm radius 6 cm, hand stand-off 15 cm (conservative), hotplate as an obstacle = 9 cm radius cylinder (its geom is a box; a placeholder), and "at least one band height reachable" per bin as in the spec (the full check needs all).

## C2e: validation, test scene (beaker and hotplate 14 cm from the edge)
- The overlay contains TASK_CFG's stance (0.765, 1.085) at pedestal 0.15: **beaker 2 bins, hotplate 2 bins** (unfiltered 3 and 3; the hotplate cylinder removes one beaker bin). Allowed by the furniture and home-clearance masks.
- Score (objects, smallest count, distance to region edge, obstacle clearance): TASK_CFG **(2, 2, 9 cm, 0.336 m)**; the algorithm's best **(2, 2, 16 cm, 0.279 m)** at (0.745, 1.025), pedestal 0.14.
- **They differ by 6.3 cm in xy (and 1.5 cm in pedestal): more than the 3 cm limit, so I stop.** Both stances are inside the region with the same counts. The best is deeper inside the region (16 vs 9 cm from its edge); TASK_CFG was picked by the finder as the middle of the passing pedestal range and also had to pass the full check (IK over all 12 points, occlusion, error stances), which the region does not do. I have not run the full check on the algorithm's best.

## Questions for David
1. Accept the 6.3 cm difference (the region is looser than the finder: it ignores the IK path, occlusion and the 9 error stances), or tighten the region first? If tightening: the cheapest is C3's full check on the chosen stance (already in the spec) with the shifted-spot retry.
2. Approve the filters (a), (b) in the overlay (M172) and the placeholders above.
