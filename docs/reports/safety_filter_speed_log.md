# Safety filter speed work: progress log

Task: make the filter faster without changing what it decides (Part A: computation; Part B: pre-computed never-close pairs).
Branch `safety-filter-speed` from origin/main 01398b7. Local tag `safety-filter-pre-speed` = 01398b734a005404203838d1f7e72941995b19ce (not pushed).
Nothing is pushed; main and the main worktree are untouched.

## How to resume
- Read this file top to bottom; the last "NEXT" line is where to continue.
- Heavy runs go through `bash scripts/safety_filter_speed_run.sh <log name> <command...>` (waits for 1.5 GB free swap and for no other job, 6 GB cap, 45 minute limit, log in data/logs/).
- Golden recordings: data/safety_filter/speed_golden/<stream>_<mode>.npz (git-ignored; manifest docs/reports/safety_filter_speed_golden_manifest.json).
- Reference source tree for the summary metrics: data/safety_filter/speed_ref_tree (rebuild: `python scripts/safety_filter_speed_golden.py --ref-tree`).

## Step 0 (read, branch, tag, golden recordings)
- 0.1 done: fetch, no tracked changes, origin/main has src/safety_filter_plugin.py and the Stage 4 and cat demo reports; branch and tag made.
- 0.2 done: read the reports' numbers, design plan section 16 (highest row SF-68), src/safety_filter/, the plug-in, the scripts.
  What the filter remembers between ticks: only `Resolver._active` (the pairs inside their release distance, margin + 2 cm, after the last tick; they are re-added to the candidate set next tick).
  Everything else is per tick (`_Tick`: pose cache, gap cache, gaps at the last command g0, the closing-arm decision, rules). The plug-in remembers the last command sent and the hold timer. The held tracker remembers the attached object. Kong's solver is reset before every call (solver_current_reset).
- 0.3 streams: punch original and reactive (filter-only, no simulator; built by scripts/safety_filter_kong_comparison.py build_stream, kept in speed_golden/punch_stream.npz);
  shadow pick and place (demo_video --record-only --safety-filter shadow) and the cat run (experiments/safety_filter_cat_run.py) in the simulator.
  The old simulator runs took their stance-search seeds from the clock, so they cannot be repeated; the golden runs pin sampler seed 1944726090 and full-check seed 4242 (M320's moved-layout rerun).
- 0.4 scripts written: scripts/safety_filter_speed_common.py (recorder, recording file, streams), scripts/safety_filter_speed_golden.py, scripts/safety_filter_speed_metrics.py (summary measured with the TAG's code), scripts/safety_filter_speed_run.sh.
  Smoke check (260 punch ticks): recorded, replayed with a fresh resolver: 0 mismatches (bit-identical output, status, active set); 650 ms mean per call.
- RUN IN PROGRESS: tmux `golden`, script <scratchpad>/golden_chain.sh: the six golden recordings one after the other
  (punch_original, punch_reactive, cat, shadow_pp with kong_style; punch_original, cat with layered). Progress lines: data/logs/speed_golden_chain.txt; logs data/logs/speed_golden_<stream>_<mode>_<stamp>.log.
  A killed chain is restarted with the same script (finished recordings are skipped, a punch run resumes from its checkpoint).

- 0.4 DONE 06:13: six golden recordings (manifest docs/reports/safety_filter_speed_golden_manifest.json). Time per call of the tag's code (ms, mean / median / p95 / max):
  punch_original kong_style 1041 / 938 / 2240 / 4970 (1576 ticks, 1534 KONG_PUSH); punch_reactive kong_style 28.2 / 21.7 / 72.4 / 189; cat kong_style 37.0 / 6.0 / 165.5 / 3322 (6436 ticks; 944 not PASS at 211 mean; 751 HOLD SHORTEN_ZERO);
  shadow_pp kong_style 6.2 / 4.6 / 14.5 / 37.2 (7783 ticks, all PASS, outcome SUCCESS); punch_original layered 129 / 122 / 350 / 603; cat layered 34.2 / 4.2 / 218 / 878.
- RUN IN PROGRESS (started 08:24): tmux `chain2`, script <scratchpad>/chain2.sh, progress data/logs/speed_chain2.txt:
  (a) the 13 filter tests on the unchanged working tree -> data/logs/speed_tests_ref.txt (do not edit src/ before the line TESTS_DONE);
  (b) step 1 profile `--label before` on the tag's code (data/safety_filter/speed_profile/before.json);
  (c) step 2 gate proof on the tag's code (--src-root data/safety_filter/speed_ref_tree): perturbed copy (+2 mm on capsule gaps, scratch only) first 300 punch ticks, then part A `--label unchanged`, part B punch and cat.
  Results in data/safety_filter/speed_gate/<label>/gate_A.json, gate_B.json.

## Step 1: profile of the tag's code (data/safety_filter/speed_profile/before.json; script scripts/safety_filter_speed_profile.py; timers add about 15 percent, shares are what counts)
Replays of the golden inputs (400 ticks of each punch stream, all ticks of the others). Own time share of the tick, by tick class:
- Pass-through ticks (all streams, 2 to 7 ms with timers): broad phase evaluate_pairs 64 to 71 percent inclusive (38 to 40 own; of it touch rule per pair 9 to 13, shape boxes 8 to 9, batched gaps 4); shape placement 12 to 25; world object conversion (as_placed) 14 percent on the simulator streams (72 conversions per tick: the 23 fixtures of the lab map rebuilt every tick); forward kinematics 3 to 6. Pair distance checks: 1.4 to 3.5 per tick, 3 to 5 percent.
- Acting ticks, punch original: 504 ms per tick, 6680 pair distance checks per tick (5672 arm vs arm, 1008 arm vs body) = 55 percent; gap cache lookups 7 percent; broad phase 25 percent inclusive (138 calls per tick: one per sampled pose and trial); placement 7.
- Acting ticks, punch reactive: 6.2 ms, 31 checks (9 percent), broad phase 42 inclusive, FK of S/E/W points 18, placement 9.
- Acting ticks, cat: 208 ms, 124 pair checks per tick (42 percent), broad phase 45 inclusive (18 calls per tick; capsule-box batch 15 own), world conversion 4.
- Layered, punch original acting: 199 ms, 1111 checks (57 percent).
Reading: the 3 ms pass-through is almost all fixed cost (rebuilding objects and listing pairs by Python loops, two poses, two kinematics runs), not distance work; near contact the cost is the number of exact pair checks inside the guard and shortening.

## Step 2 and the array-form work (state)
- Gate scripts: scripts/safety_filter_speed_gate.py (part A replay, part B closed loop, thresholds in the script as the assistant's choice).
- src changes applied, WIP, not yet gated: new src/safety_filter/tick.py (_Tick moved out of resolver.py, array form, bounds, pair scheduler), tick_arrays.py (pair table, pose arrays, gap intervals), shapes.py (set_state skips unchanged state, place_arrays), gap_rule.py (bounds_valid, array rule), resolver.py/kong_style.py (begin(), use of the bounds), config.py (SLEEP).
- Scheduler per David's latest message: min-heap on wakeup tick, interval floor((G - release) / (v dt (1 + 0.20))), at least 1; world or hold change wakes the affected pairs; report of the interval distribution per pair type and mean pairs checked per tick via Resolver.statics.sleep.report().
  My additions, flagged: a command step faster than the speed bound (0.5 rad/s, the bridge limit) wakes that arm's pairs; a state change of the base, pedestal or fingers above 0.2 mm wakes all pairs. The punch streams move far faster than 0.5 rad/s, so there the scheduler is mostly off by design.

NEXT: read chain2 results; write the profile table and the gate proof here; commit steps 1 and 2. Then apply the drafted array-form tick (scratchpad newsrc/tick_arrays.py, tick.py) and gate it.

## Plan change (David, 2026-11-15, during step 0)
- Step 3 candidates replaced, in this order unless measurement says otherwise: 1 pair sleep timers (a pair is not checked again until the travel bound of its two parts uses up its gap minus margin minus slowing band; pass-through = every pair asleep; reset when the world list or the holding verdict changes); 2 conservative advancement in place of the halving search of the guard (APPROVED within the gate tolerance; report every differing tick; remove the halving code once the gate passes); 3 one swept broad-phase pass per step (numpy, no tree); 4 batched closed-form forward kinematics and exact capsule distances only for pairs whose box gap is below margin plus travel; 5 allocation and overhead (preallocated arrays, no per-pair dicts, buffered recording); 6 compiled kernel (numba first) with a Python fallback.
- Part B (never-close extension) moves AFTER step 5, "if time remains".
- Design prepared while the golden runs go (no src change yet; drafts in the session scratchpad): array form of a tick (pair table built once per item set, pose arrays, box separation for all pairs at once, gap intervals from batched calls, exact scalar gap only where an interval does not decide or the value is used). The sleep timers and the "exact only when near" rule of candidates 1 and 4 sit on top of it.

## Config rule (David, 2026-11-15)
Speed limits are read from the config and no number is hard coded: SLEEP["joint_speed_rad_s"] is COMMAND_BRIDGE_CFG["max_joint_velocity_rad_s"] (src/task_config.py, read in src/safety_filter/config.py, not copied); the cache sizes, bound tolerances and epsilons of the array form and the scheduler are in config.py (ARRAY_FORM, SLEEP), each with its unit and reason. Check again before every commit that touches src/safety_filter.

## Step 2: the gate is proven (commit of this entry)
- Unchanged code (the tag's src, --src-root data/safety_filter/speed_ref_tree): part A zero difference on all six golden recordings (0 failing ticks, largest command difference 0.0, exact status, active set after each tick, sweeps, pairs pushed, solver calls and step fraction all equal); part B on punch original, punch reactive and cat: summaries inside every tolerance and the commands and statuses bit-identical to golden (data/safety_filter/speed_gate/unchanged/gate_A.json, gate_B.json).
- Perturbed scratch copy (+2 mm on every capsule-capsule gap, session scratchpad, never committed): part A on the first 300 punch ticks: 280 failing ticks, largest command difference 0.056 rad. Detected.
- Part B note: the first cat run on the reference tree failed at the LLM cache (the cache folder experiments/llm_cache is looked up relative to the source tree; the reference tree now links experiments and assets). Rerun passed.
- Thresholds (config values in the script, chosen by the assistant, to be confirmed by David): 1e-3 rad per joint; status class same (PASS / HOLD / changed); same hold reason; counts within 1 percent of the golden count; smallest true gap within 1 mm; hand error mean and max within 0.5 mm.

## Step 3, change 1 (array-form tick + gap intervals + heap pair scheduler): part A PASSES (src commit before this entry; part B, tests, profile running: tmux chain4, data/logs/speed_chain4_s3_1.txt)
Part A on all six golden recordings: 0 failing ticks, largest command difference 0, exact status, active set after tick, sweeps, pairs pushed/skipped/shortened, solver calls and fraction identical on every tick. Time per call, mean / median / p95 / max ms (golden mean in brackets):
- punch original kong_style 81.9 / 85.5 / 139 / 271 (1041)
- punch reactive kong_style 10.2 / 9.4 / 17.7 / 44.4 (28.2)
- shadow pick and place 2.17 / 1.75 / 4.3 / 43.7 (6.21)
- cat kong_style 7.79 / 2.20 / 41.7 / 186 (36.96)
- punch original layered 48.5 / 46.5 / 94 / 211 (128.8); cat layered 14.5 / 2.29 / 116 / 197 (34.2)
Targets (mean 6.7 ms, p95 20 ms on punch, reactive punch, cat): not yet met (punch original 82 ms, reactive 10.2 ms mean, cat 7.8 ms mean and p95 41.7). Pass-through target 1 ms: 2.17 ms mean on shadow, not met.
NEXT: chain4 results; then step 3 change 2 (conservative advancement replacing the halving search of the guard), then 3 to 6 by measured payoff.

## Step 3, change 1 KEPT (commit "Speed: the joint speed bound is passed to the Resolver ..."; gate label s3_1b)
Gate A: 0 failing ticks on all six recordings, largest command difference 0, all counters identical. Gate B: punch original, punch reactive and cat identical to golden (commands and status bit-identical). Tests: 12 of 13 filter tests pass; only safety_filter_spec_test fails (known failure, same as the reference run). Along the way structure_test and sequential_push_test failed because config.py imported task_config; fixed by passing the bridge speed from the caller (the plug-in reads the loop's own merge of COMMAND_BRIDGE_CFG; no bridge = no speed bound = sleep off).
Pair scheduler statistics (profile s3_1b, wakeup interval in ticks, mean / median / max): punch original arm-vs-arm 9.9 / 7 / 97, arm-vs-body 90.7 / 67 / 655; cat arm-vs-arm 14.1 / 13 / 97, arm-vs-body 88.3 / 66 / 971, arm-vs-world 919 / 523 / 2000 (cap); shadow arm-vs-world 813 / 416 / 2000. Mean pairs checked per tick: punch 473 of 518 (the punch commands are faster than the 0.5 rad/s bound, so every pair wakes), shadow 60 of 1214, cat 170 of 1247.
What the profile still shows: pass-through ticks 1.7 to 2.4 ms are fixed overhead (tick start-up pose build 22 to 24 percent, active set update 20, pair listing 20, worst pair search 8 to 15); acting ticks: punch original 90 ms = pair listing 30 percent (140 poses per tick), exact gap lookups 23 (463 per tick), step shortening 20; cat acting 45 ms = shortening 28, gap lookups 24.
NEXT: change 1b (a pair whose arms do not move is never closing: no need to look at it), change 2 (conservative advancement for the guard).

## Step 3, change 2 KEPT as a verified fast path (gate label adv4; src commit "WIP: guard advancement is a verified fast path ...")
- First version, pure conservative advancement (David's approved form): punch original 49 ms mean (from 82), cat p95 19.5 ms. Part A: 0 failing ticks on 5 recordings; on cat ONE tick (5672) differs, fraction 0.742 (golden) against 0.0955, command difference 1.9 mrad. A fine scan of that tick's path (3000 poses) shows a pair below the guard gap from 10.6 percent of the step on: the golden sampled test jumped through it, the advancement is the safer decision. But part B on the cat fails: the closed loop diverges after tick 5672 (HOLD 751 -> 695, KONG_PUSH 192 -> 2101, BRANCH_GUARD holds 0 -> 672, smallest true gap -19.2 -> -28.6 mm, hand error max 82 -> 163 mm). The rule is not to loosen thresholds and a change is kept only if it passes, so pure advancement is NOT kept (it is under Options for David).
- Kept form: the advancement proves a clear path (no sampling needed) or proposes the halving's grid cell, which the old sampled test then confirms with three evaluations (full step fails, proposed fraction passes, next grid step fails); if not confirmed, the full halving runs as before. Decisions are the sampled test's, so nothing differs.
- Gate A: 0 failing ticks and every counter identical on all six recordings. Gate B: identical to golden on punch original, punch reactive, cat. Tests: 12 of 13 (spec_test known failure).
- Time per call (gate B runs, mean / median / p95 / max ms; golden mean): punch original 61.1 / 60.9 / 100 / 213 (1041); punch reactive 10.0 / 9.1 / 18.7 / 78 (28.2); cat 6.26 / 2.50 / 23.9 / 309 (37.0). Timing on this VM varies by about 20 percent between runs.
NEXT: change 1b (a pair none of whose arms moves is never closing), then batched exact gaps for capsule pairs; then wrap up (suite, report, plan rows).

## Step 3, change 1b KEPT (pairs none of whose arms moves are not looked at; gate label s1b)
Gate A: 0 failing ticks on all six recordings, command differences 0, every decision counter identical. The only difference is the between-tick memory: "active set after tick" differs on 1483 shadow ticks, 1302 cat ticks (kong and layered): the pairs that were skipped because their arms do not move are no longer kept in the release band; they come back as soon as an arm of theirs moves (pending set). The active set only decides which pairs are candidates next tick, and a pair with no moving arm is never acted on, so decisions are unchanged (part B identical). Gate B: identical to golden on punch original, punch reactive, cat. Tests 12 of 13 (spec_test known).
Time per call (gate B, mean / median / p95 / max ms): punch original 60.2 / 61.5 / 101 / 150; punch reactive 8.80 / 8.15 / 17.2 / 45.0; cat 5.33 / 1.75 / 23.1 / 254. Gate A pass-through (shadow) mean 1.90, median 1.63 ms. Mean pairs checked per tick: punch 472 of 518 (commands faster than the 0.5 rad/s bound), shadow 38 of 1214, cat 146 of 1247.

## Not done (decided to stop here, see the report)
Step 3 candidates 3 (one swept broad-phase pass), 4 (batched closed-form FK) and 5 (allocation) were not built separately: the array form of change 1 already replaced the per-pair Python broad phase; the remaining cost is spread over many small numpy calls (pass-through 1.6 to 1.9 ms median). Step 5/6 compiled kernel: not built (numba is not installed and the shared environment must not be touched; a C kernel loaded with ctypes would need a bit-parity test against the scalar capsule distance). Step 4 / Part B (never-close extension): not started ("if time remains").

## Full suite result (2026-11-15)
speedfinal: 51 pass, 15 fail. 11 known old failures; the 4 others (elbow_guard, face, full_check, transit_clearance) fail identically on the tag's code (logs data/logs/maintree_*). No new failures.

# PART B: never-close list by divide and conquer (task of 2026-11-15, evening)

## Part B step 0: read and check
- Branch safety-filter-speed, HEAD 7209b57 at the start. Nothing running, 4.0 GB swap free.
- Design: docs/design_plan.md section 16, PENDING item "Divide-and-conquer never-close test" (row SF-27, approved design, deferred). It is a sketch, not a full specification: split joint ranges, a guaranteed region per part ("each joint's range becomes an arc, stacked along the arm"), settled when the regions stay apart by margin + 1 cm, per-pair cap about 2 minutes, unproven pairs stay checked, stored with the fingerprint. The task's method (gap at the box centre minus lever arm x half range) is one way to build that guaranteed region (a ball of that radius around each part's centre-pose position), so it follows SF-27. Gaps between SF-27 and what is built here (all listed under Decisions for David):
  1. SF-27 "tiny pieces use a real pose's gap": NOT built. The task says a box at the minimum size that is not proven leaves the pair checked; a single pose's gap proves nothing about the rest of the box.
  2. SF-27 "blocked by the torso" flood: NOT built. Every pose inside the joint limits is treated as reachable (more conservative).
  3. SF-27 "absorbs the reach-limit test, cross-checks the 3-joint sweep": the tool can be run on the already listed pairs as a cross-check (reported, not used to remove anything from the list).
  4. SF-27 does not say whether the proof is on the real geometry (as the Stage 1 sweep) or on the filter's shapes. The task says "capsule distance", so the proof is on the filter's own shapes (capsules and boxes). Because the shapes enclose the real geometry (M288), this is the stricter of the two.
- No branch named never / close / divide / dc exists; no tool or report mentions divide and conquer except the SF-27 rows and the speed report's "not found" line. Nothing to reuse except tools/never_close_sweep.py (variables of a pair, lever arm idea, file layout).
- Highest SF row in use: SF-74. New rows start at SF-75.
- Existing list: data/safety_filter/never_close_sweep.json (keyed by real geoms, fingerprint 781198f9...), merged into data/safety_filter/robot_shapes.json block `never_close` by `tools/build_filter_shapes.py --merge-never-close`. The runtime (src/safety_filter/shapes.py) reads ONLY robot_shapes.json: 147 shape pairs listed, 518 of 665 still checked.
- Robot chain read from the compiled model (22 joints): free base joint, 2 wheel hinges (not filter variables, the filter leaves them at 0), pedestal slide (0 to 0.418 m), 7 hinges per arm, 2 finger slides per hand (0 to 0.044 m). At most one joint per body. Every hinge anchor is at its body origin. Every shape in robot_shapes.json is attached to ONE body (no capsule spans two bodies, none has a t_range).
- Filter tests at HEAD (reference for "tests still pass"): see the end of step 1 below.

## Part B step 1: soundness argument (written before any code)

### (1) The lower bound
Pair of filter shapes S1, S2. A capsule is all points within r of a segment [A, B]; a box is {C + R u : |u_i| <= h_i}. "Core" = the segment, or the solid box (r = 0). Each shape is fixed in ONE body frame. Handle points of a shape: the 2 segment ends, or the 8 box corners.
- gap(q) = dist(core1(q), core2(q)) - r1 - r2 (the filter's gap, for shapes that do not overlap).
- Variables V of the pair: the filter's state variables (arm hinges, finger slide, pedestal slide) that are on one body's path to the root but not on the other's. V1 = those on S1's side, V2 = on S2's side.
- Joint box Q = product of [lo_j, hi_j] over j in V; centre c; half widths w_j = (hi_j - lo_j) / 2.
- Lever arm lam_j(p) of variable j on a handle point p: for a slide, 1 (m per m); for a hinge, a number that is at least the distance of p from the hinge axis in EVERY pose (computed below).
- Travel bound of shape i over the box: T_i(Q) = max over its handle points p of sum over j in V_i of w_j x lam_j(p).
- Centre value: G(c) = a number that is at most gap(c) (capsule vs capsule: the closed-form segment distance minus eps_num; capsule vs box: the smallest of K+1 equally spaced samples of the segment's distance to the box, minus len/(2K), minus eps_num; len = segment length).
- LOWER BOUND: LB(Q) = G(c) - T_1(Q) - T_2(Q).
- Rule: Q is settled when LB(Q) >= thr = margin + 1 cm (config PUSH_START_BEYOND_M). Otherwise Q is cut in two and both halves are tested; a pair is declared never-close only when every piece of the full joint limits is settled.

### (2) Why LB(Q) <= gap(q) for every q in Q
Step 1, common joints do not matter. The two bodies' paths to the root share a trunk. A joint on the trunk moves both shapes by the same rigid motion, which changes no distance. So positions are taken in the frame of the last shared body, where S_i depends only on V_i.
Step 2, one hinge. If only hinge j turns, by an angle d, a point p attached further down the chain moves on a circle around the axis. Its distance from the axis, rho, does not change during this motion. It travels an arc of length rho x |d|, so it ends at most rho x |d| <= lam_j(p) x |d| from where it started. A slide of d moves p by exactly |d|.
Step 3, the lever arm is valid in every pose. Work from p up the chain, keeping a ball (centre fixed in the current body frame, radius) that contains p for every value of the joints passed so far. Start: centre p, radius 0. Passing a hinge with anchor a, any angle: the ball (centre x, radius s) becomes the ball (centre a, radius |x - a| + s), because a rotation about an axis through a keeps every distance from a. Passing a slide with range [lo, hi] along unit axis e: centre x + e (lo + hi) / 2, radius s + (hi - lo) / 2. Passing a fixed body offset: the centre is moved rigidly, the radius is unchanged. When hinge j is reached with ball (x, s) in the frame where its axis is fixed: lam_j(p) = (distance of x from the axis) + s. Every possible position of p is inside the ball, so its distance from the axis is at most lam_j(p). No assumption on the other joints' values is needed (they may even be outside their limits).
Step 4, many joints. Go from c to q by changing one variable at a time. Each change moves p by at most |q_j - c_j| x lam_j(p) (steps 2 and 3, which hold in every pose, so also at each intermediate pose). By the triangle inequality |p(q) - p(c)| <= sum_j |q_j - c_j| lam_j(p) <= sum_j w_j lam_j(p) <= T_i(Q).
Step 5, every point of the core. A body moves rigidly, so a point x of the core that is a fixed weighted average of the handle points (weights >= 0, sum 1: a point of a segment is (1-s)A + sB; a point of a box is a weighted average of its 8 corners) stays that weighted average. Its movement is the same weighted average of the handle movements, so |x(q) - x(c)| <= T_i(Q).
Step 6, the distance. Take q in Q and the closest points x1 on core 1 and x2 on core 2 at q (same material points followed back to c). dist(q) = |x1(q) - x2(q)| >= |x1(c) - x2(c)| - T_1 - T_2 >= dist(c) - T_1 - T_2. (If the cores overlap at q, dist(q) = 0 and the same line gives 0 >= dist(c) - T_1 - T_2.) Radii and box sizes do not change, so gap(q) >= gap(c) - T_1 - T_2 >= G(c) - T_1 - T_2 = LB(Q).
Step 7, the centre value. Capsule vs capsule: the closest distance of two segments has a closed form (the filter's own function, geometry.capsule_capsule_batch); eps_num covers rounding. Capsule vs box: f(s) = distance of the point A + s(B - A) from the box changes by at most len x |ds| (a distance to a set changes no faster than the point moves). Every s in [0, 1] is within 1/(2K) of a sample, so min f >= (smallest sample) - len/(2K).
Step 8, the pieces. Halving a box along one variable gives two boxes whose union is the box. If every piece has LB >= thr, gap(q) >= thr for every q in the joint limits.
Step 9, the filter's own numbers. The filter's gap functions return the distance between two actual points of the cores (capsule-capsule), or the smallest of sampled true distances (capsule-box), minus the radii. Both are at least the true gap. So "true gap >= thr everywhere" means the filter can never compute a gap below thr for the pair.

### (3) Assumptions
A1. Between each shape's body and the last shared body there are only rigid bodies joined by hinges and slides, at most one joint per body.
A2. Each shape is rigid and fixed in one body frame; radii and box half sizes are constant.
A3. The variables stay inside the limits used for the proof: the model's joint ranges (jnt_range) for the 14 arm joints, 0 to 0.418 m pedestal, 0 to 0.044 m finger travel.
A4. Joints that are not filter variables (wheels) keep the value the filter gives them (0); the base joint is on the shared trunk.
A5. The tool's forward kinematics is the filter's (same compiled model, same shapes file, fingerprint checked).
A6. The lever arm is computed for every handle point of the shape (both ends, all 8 corners), not for a typical point.
A7. Only robot shapes of robot_shapes.json are covered. Held objects and world objects are never declared.
A8. The threshold is on the gap (margin + 1 cm), the same quantity the filter measures.

### (4) What breaks if an assumption fails, and the check on our robot
A1. A ball joint, a closed loop or two joints on one body would make steps 2 and 3 wrong (a point could move further than the bound). CHECK: model has 1 free joint (base, on the shared trunk), 16 hinges, 5 slides; max 1 joint per body; the two equality constraints (finger mimic) are not used by the filter's kinematics (it writes qpos directly). The tool asserts this at start and refuses otherwise.
A2. A capsule strung between two bodies would not move rigidly, so step 5 (weighted average with fixed weights of ONE body's points) would not hold as written. CHECK: all 45 shapes have both ends on one body and no t_range. The tool refuses a shapes file where this is not so.
A3. If a command leaves the joint limits the proof says nothing about that pose, and a declared pair could come closer than thr unseen. The filter does not clip commands (it records the overshoot and holds only after a push). CHECK: in the six golden recordings no raw, last or filtered command is outside the limits (largest overshoot +0.0000 rad on the punch raw commands, which reach the limit itself; pedestal 0 to 0.417 m; fingers 0 to 0.0439 m). This is the SAME assumption the Stage 1 list already makes. It is the weakest assumption: listed under Decisions for David (option: prove over limits widened by a small pad, config value, default 0 = as Stage 1).
A4. A wheel shape placed at another wheel angle would be elsewhere. The filter never turns the wheels in its own model, so the shapes it checks are the ones proven. Unchanged from Stage 1.
A5. A different model or shapes file would make the proof about another robot. CHECK: the tool stops if the shapes file's fingerprint differs from the compiled model's, compares its own batched kinematics with RobotShapes.place (MuJoCo) on random poses at start (stops above 1e-9 m), and the output carries the fingerprint, which the loader must check.
A6. Using only one end would understate the travel. CHECK: by construction (max over all handle points).
A7. Nothing to check: such pairs are not candidates.
A8. See (5): the threshold is the activation distance, not a larger one.

### (5) Where the bound is loose, and one consequence to know
- Loosest point: the lever arm. It is pose-independent: the chain is taken as fully stretched and perpendicular to every axis at once (step 3 adds up link offsets). The true lever is often half of it or less. Second: travel is added up as if every joint moved the point in the same direction and both shapes moved straight toward each other.
- Size of the loss: on one box, gap(q) can exceed LB by up to 2 x (T_1 + T_2) (when the centre is the closest pose, the bound still subtracts the full travel, and the far side of the box gains as much). At the smallest box (0.05 rad per joint, w = 0.025 rad) T = 0.025 x sum of lever arms: with 7 joints and lever arms of 0.1 to 0.8 m this is 2 to 8 cm per arm. So a pair whose true closest approach is less than thr plus a few cm can NOT be declared, however long the tool runs. Those pairs stay checked (safe direction). Capsule vs box adds len/(2K) (1.2 mm for a 0.3 m capsule at K = 128).
- Consequence of the threshold (not a soundness gap, but David should know): thr = margin + 1 cm is where the filter starts to act on a pair ONLY IF the pair's boxes are that close; the gap rule, however, also limits the closing speed of a candidate pair whose gap is above margin + 1 cm when a single tick would close more than 20 percent of the gap above the margin. A declared pair is no longer speed-limited in that way. It still can never come within margin + 1 cm, so nothing unsafe follows, but a decision can differ from the unextended filter on a fast approach. The Stage 1 list has the same property. The gate (check iv) measures whether it happens on the recorded streams. Listed under Decisions for David.
- No gap was found in the reasoning that blocks the tool. A3 (commands inside the joint limits) is inherited from Stage 1 and is raised as a decision, with the conservative default kept.

### File format and loader: PLAN ONLY (structural, David decides; not implemented)
- Tool output data/safety_filter/never_close_dc.json, same layout as the sweep file: top level fingerprint, rule ("SF-27"), the parameters, `entries`; each entry: a_geoms, b_geoms, margin_m, shape_pairs (ONE shape pair per entry, because the proof is per shape pair), variables, n_vars, status ("certified", "reachable" = a pose below thr was found, "not certified" = a smallest box failed, "time cap"), listed, needed_m, plus the proof numbers (boxes, smallest lower bound, lowest centre gap and its pose, seconds).
- Proposed loader (NOT built): `tools/build_filter_shapes.py --merge-never-close` reads the sweep file, the DC file, or both, refuses a file whose fingerprint differs, and writes the union into robot_shapes.json `never_close.pairs` (each pair marked with its source). The runtime loader in src/safety_filter/shapes.py then needs no change: it already refuses a stale fingerprint. Alternative: shapes.py reads never_close_dc.json next to robot_shapes.json at run time.
- Until David decides, the checks that need the extended list in use (gate, profile) run on a scratch copy of the source tree whose config points at a trial shapes file (robot_shapes.json plus the DC pairs in the existing `never_close` block). Nothing under src/ changes.

### Filter tests at HEAD 7209b57 (reference, data/logs/partb_tests_ref.txt)
12 of 13 exit 0; safety_filter_spec_test exit 1 (the known old failure "the closing right arm moves"). Same as before the speed work.
NEXT: step 2, tools/never_close_dc.py (only after this commit and the standing soundness file).

## Part B step 2: tool built and run (tools/never_close_dc.py; settings config.NEVER_CLOSE_DC)
- Before any pair the tool checks A1 and A2 on the model and shapes file, compares its batched placement with the filter's on 300 random states (largest difference 8.9e-16 m) and its lever arms with each point's real distance from each hinge axis at those states (never smaller).
- Full run on the 518 pairs not on the Stage 1 list (logs data/logs/never_close_dc_20261115_181850.log and _185935.log; two pieces of at most 40 minutes, checkpointed): certified 219, reachable 265 (a real pose below the threshold was found), not certified 16 (a smallest box failed), time cap 18 (120 s). By type: arm vs body 200 certified of 273; arm vs arm 19 certified of 245.
- Certified pairs by number of variables: 3: 1, 4: 39, 5: 27, 6: 42, 7: 20, 8: 62, 9: 28. Smallest (searched closest gap - threshold) among certified pairs: 23.5 mm.
- Cost: 3597 s of pair time (about 60 minutes wall), 122.9 million boxes, peak memory 2.16 GB.
- Output data/safety_filter/never_close_dc.json (git-ignored). NOT yet used by anything.
NEXT: step 3 checks in tmux `dc3` (chain: certificate, falsification, golden replay, trial tree, gate A, gate B, profile); progress in data/logs/partb_chain.txt.

## Part B step 3 (in progress): soundness checks
- i. Certificate check DONE, PASS (data/safety_filter/never_close_dc_verify.json; logs data/logs/partb_verify_20261115_192014.log, _200037.log): every one of the 219 declared pairs was proven again from scratch with no time cap: settled volume exactly 1.0 for all, every box bound at or above the threshold (smallest surplus 3.7e-9 m), the same box counts and smallest bounds as the first run for all 219 (21.15 million settled boxes in all). Independent re-check with the filter's own scalar functions: 231,052 settled boxes, 8 random poses in each (1,848,416 poses): the filter's gap was never below the box's lower bound (smallest surplus 21.6 mm) and never below the threshold (lowest gap 24.4 mm above it).
- ii. Falsification search: stopped at 69 of 224 pairs (all 69 PASS) because one gap evaluation cost 2.7 ms (every shape object was built per call). The script now reads only the pair's two shapes from the placement arrays and calls the same scalar functions (difference 0 on 400 random poses; 0.65 ms capsule-capsule, 1.9 ms capsule-box, whose exact function samples 4001 points). The checkpoint keeps the 69 pairs. It runs LAST in the chain (about 2.5 hours).
- RUN: tmux `dc4`, scratchpad partb_chain2.sh: replay, trial tree, gate A, gate B, profile (trial and base), base gate A, then falsification in 40-minute pieces. Progress data/logs/partb_chain.txt (ends with CHAIN_DONE); each step's log data/logs/partb_<step>_<stamp>.log.
