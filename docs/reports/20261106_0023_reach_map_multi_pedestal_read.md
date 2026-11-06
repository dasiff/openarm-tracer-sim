# Reach maps across pedestal heights: what the code does now (read-only)

Question from David: the system should look up reach at any pedestal height and pick the best per stance, without rebuilding maps on each pedestal change. Read-only check; no builder or map was changed.

## 1. What the pedestal_qs lists are

- `SEARCH_CFG["pedestal_qs"]` (src/task_config.py:281) is for the offline staging search (experiments/beaker_hotplate_staging_search.py, SEARCH_CFG section). It is a fixed tuple of 7 commands.
- `LAYOUT_CFG["pedestal_qs"]` (src/task_config.py:315) is for the layout search (experiments/beaker_hotplate_layout_search.py, LAYOUT_CFG section). It is a fixed tuple of 6 commands.
- Neither is read by experiments/reach_map_build.py or src/reach_map.py. They are not the set the map builder iterates over.
- The pipeline's live pedestal grid is `STANCE_CFG["pedestal_step"]` = 0.04 m (src/task_config.py:248), or the `robot_specs` range when `pedestal_qs` is None (src/stance_finder.py:308-309). `stance_from_map.py` uses a 0.01 m grid (`--q-step`).

## 2. Does the builder loop over pedestal heights?

No. experiments/reach_map_build.py has no loop over pedestal heights. The only pedestal value it reads is `TASK_CFG["pedestal_q"]` (0.1550 m), in two places:
- the bench geoms are lowered by that value in the collision filter (line 92, `bench = {... "pedestal_q": TASK_CFG["pedestal_q"]}`);
- the same value is written into the staleness stamp (lines 76 and 94).

Everything else in the filter is fixed at the lowest command (`ArmCollisionChecker` with no `pedestal_q`, so the pedestal stays at its lowest command).

## 3. Does the map data structure support multiple heights?

Yes, for lookup. The map is in the arm's shoulder frame (`P = pad - shoulder`, reach_map_build.py: `P = P - sh`). The pedestal only moves the shoulder vertically. src/stance_finder.py:178-179 shifts the point's height per pedestal command (`zr = P[2] - zt`) and looks it up in the same map. So one map serves every pedestal height for lookup, and `find_stances` already searches several heights against it.

The one pedestal dependence baked into a map is the bench filter. A map built at 0.1550 m has the bench at that relative height. Using it for another height is only approximate for points near the bench. That is the real reason maps are tied to a pedestal height.

Staleness: because the stamp hashes `bench.pedestal_q`, any change of `TASK_CFG["pedestal_q"]` makes every existing map stale, and the build rebuilds all 24 maps. That is the rebuild David wants to avoid.

## What it would take (options for David to decide)

A. Build once with no pedestal-dependent bench, or with the bench at one reference height, and drop `pedestal_q` from the stamp. Maps then cover every height by lookup. Cost: the bench collision near the bench edge is approximate. This needs a decision on whether that approximation is acceptable.

B. Build one map per bench height (loop over a list of heights in the builder, with the height in the file name and stamp). The map structure needs no change. Cost: maps and build time multiply by the number of heights, and lookup must pick the nearest built height.

C. Keep one map and compute the bench check at lookup time (the stance check already does collision checks). Cost: more work per lookup; the map becomes height-independent.

Also needed in any option: the stamp hashes `pedestal_q` for the bench (line 76, 94), so the choice of option decides whether that field stays in the stamp.

No change made. Decisions are David's.
