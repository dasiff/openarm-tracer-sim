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
