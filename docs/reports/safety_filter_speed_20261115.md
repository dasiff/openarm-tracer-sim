# Safety filter speed work (Stage 5 speed), 2026-11-15

Written for David. Branch `safety-filter-speed` (from origin/main 01398b7), local tag `safety-filter-pre-speed` = 01398b734a005404203838d1f7e72941995b19ce. Nothing pushed, main and the main worktree untouched. Step-by-step log with every number: `docs/reports/safety_filter_speed_log.md`.

## Result in one paragraph
The filter decides exactly what it decided before: part A of the gate (every recorded input replayed in order) shows 0 failing ticks, a largest command difference of 0 and identical counters on all six golden recordings; part B (closed loop) is identical to golden, command for command, on punch original, punch reactive and the cat scene. Time per call fell on every stream. The primary target (mean 6.7 ms, 95th percentile 20 ms) is met on the cat scene mean (5.3 ms) and on the reactive punch p95 (17.2 ms); it is NOT met on the punch original (60 ms mean), the reactive punch mean (8.8 ms) or the cat p95 (23.1 ms). The secondary target (pass-through 1 ms mean) is NOT met (1.6 to 1.9 ms). Compiled code (step 5/6) and Part B (never-close extension) were not done.

## Before and after (time per filter call, ms)
Before = the tag's code on the golden inputs (`speed_golden/*_summary.json`). After = the last kept commit; closed loop (B) for the three gated streams, replay (A) for the others. This VM's timing varies about 20 percent between runs.

| stream | ticks | before mean | median | p95 | max | after mean | median | p95 | max | after, pass-through only (mean) |
|---|---|---|---|---|---|---|---|---|---|---|
| punch original, kong_style | 1576 | 1041.4 | 938.0 | 2239.6 | 4970.1 | 60.2 (B) | 61.5 | 101.1 | 150.2 | 2.71 |
| punch reactive, kong_style | 1576 | 28.2 | 21.7 | 72.4 | 189.0 | 8.80 (B) | 8.15 | 17.2 | 45.0 | 1.98 |
| cat scene, kong_style | 6436 | 36.96 | 5.97 | 165.5 | 3322.1 | 5.33 (B) | 1.75 | 23.1 | 253.5 | 1.58 |
| shadow pick and place, kong_style | 7783 | 6.21 | 4.64 | 14.5 | 37.2 | 1.90 (A) | 1.63 | 5.05 | 88.4 | 1.90 |
| punch original, layered | 1576 | 128.8 | 121.7 | 350.3 | 603.2 | 47.3 (A) | 42.9 | 98.1 | 238.0 | 4.29 |
| cat scene, layered | 6348 | 34.20 | 4.18 | 217.8 | 878.0 | 15.2 (A) | 1.54 | 126.3 | 212.8 | 1.61 |

Targets: mean at most 6.7 ms and p95 at most 20 ms on punch (original and reactive) and the cat scene; pass-through at most 1 ms. Status: cat mean met, reactive p95 met, the rest not met. How far: punch original is 9 times over the mean target (it is a stress stream with 1534 of 1576 ticks pushing, raw commands 5 times faster than the bridge allows). What it would take: compiled distance and FK kernels (the exact capsule distance is 35 percent of a punch-original tick, the guard 22 percent) and a cheaper tick start-up for pass-through (pose build, pair listing and active-set update are each 0.2 to 0.5 ms of Python and numpy calls).

## Time per stage (share of the tick; step 1 profile of the tag's code, then after)
Profile script: `scripts/safety_filter_speed_profile.py` (timers from outside; they add 15 to 40 percent, so the shares matter, not the milliseconds). Files: `data/safety_filter/speed_profile/before.json` and `s1b.json`.

Before (tag), own time share:
- pass-through ticks (2.2 to 7.1 ms with timers): broad phase `evaluate_pairs` 64 to 71 percent inclusive (pair rule per pair 9 to 13, shape boxes 8 to 9, batched gaps 4), shape placement 12 to 25, world object conversion 14 (72 object conversions per tick), forward kinematics 3 to 6, pair distance checks 3 to 5.
- acting ticks: pair distance checks 55 percent (punch original, 6680 per tick: 5672 arm-arm, 1008 arm-body), 42 percent (cat, 124 per tick: 61 capsule-capsule and 63 capsule-box against the world), broad phase 25 to 45 inclusive, placement 5 to 7.
After (s1b), own time share:
- pass-through: tick start-up (pose build) 19 to 29 percent, active-set update 18 to 26, pair listing 17 to 21, worst-pair search 6 to 8, tick body 11 to 25, forward kinematics 5 to 9. Pair distance checks: none on average.
- acting, punch original (54 ms with timers): exact gap lookups 35 percent (458 per tick, through the object-form functions), guard 22, pair listing 13, forward kinematics 12. Cat acting (31 ms): guard 29, gap lookups 20.

Pair checks per tick by type, before: punch original 5672 arm-arm + 1008 arm-body; punch reactive 5.4 + 25.4; cat 61 capsule-capsule + 63 capsule-box (world); shadow pass-through 1.7 in all. After: the batched gap intervals decide almost every pair question, so the object-form `gap_between` is called 0.9 + 1.5 (punch original), 0.05 + 1.6 (reactive), 0.4 (cat) times per tick; the exact scalar gaps that remain go through `tick_arrays.exact_gap` (458 per tick on punch original, 26 on reactive, 57 on cat acting ticks).

## What was kept, and its gate results
All gate numbers: `data/safety_filter/speed_gate/<label>/gate_A.json`, `gate_B.json` (git-ignored); thresholds in `scripts/safety_filter_speed_gate.py`, chosen by me, David to confirm: command within 1e-3 rad per joint, same status class and hold reason; closed loop: status and hold counts within 1 percent, smallest true gap within 1 mm, hand error mean and max within 0.5 mm.

| change | commit message start | gate A: failing ticks, largest per-tick command difference | gate B | tests |
|---|---|---|---|---|
| 1 array-form tick, gap intervals, heap pair scheduler (SF-70 to SF-72) | "Speed: the joint speed bound is passed to the Resolver ..." | 0, 0 rad (all counters identical) | identical on 3 streams | 12 of 13 |
| 2 jump-through guard: advancement as a verified fast path (SF-73) | "WIP: guard advancement is a verified fast path ..." | 0, 0 rad | identical on 3 streams | 12 of 13 |
| 1b pairs with no moving arm are not looked at (SF-72) | "WIP: pairs without a moving arm ..." | 0, 0 rad (only the informational active set differs on 1302 to 1483 simulator ticks) | identical on 3 streams | 12 of 13 |

Time per call after change 1 alone (A): punch original 81.9 ms, reactive 10.2, cat 7.8, shadow 2.17.

What each change does (all in `src/safety_filter/`; the old object-form functions in `touch.py`, `geometry.py` are untouched and remain the definition of every number):
1. `tick.py` (new; `_Tick` moved out of `resolver.py`), `tick_arrays.py` (new): the shapes of a pose are arrays; the pairs the touch rules check are listed once per set of world objects and held shapes; the broad phase is one array comparison; threshold questions (is the shortfall above the trigger, is a pair inside its release distance, did the path pass below the guard gap) are first put to cheap bounds (box separation, travel of the two parts, batched capsule distances with a proven error interval) and only the pairs the bounds cannot decide get the exact scalar distance; every VALUE that is pushed or recorded is the scalar function's. `shapes.py`: `set_state` skips the kinematics when the state is the one last set; `place_arrays`. `gap_rule.py`: `bounds_valid`, array form of the rule. The pair scheduler (`SleepTimers` in `tick.py`): a min-heap on wakeup tick; interval = floor((G - release) / (v dt (1 + 0.20))), at least 1; v from the command bridge's joint speed limit and the worst-case lever arms; the speed limit is READ from the config (the plug-in passes `COMMAND_BRIDGE_CFG["max_joint_velocity_rad_s"]` to `Resolver(joint_speed_rad_s=...)`; without a bridge there is no bound and the scheduler is off); every number is in `config.py` (`SLEEP`, `ARRAY_FORM`).
2. Guard: conservative advancement along the path either proves the path clear (the sampled test would pass, so it is skipped) or proposes the halving's grid cell, which the old sampled test confirms with three evaluations instead of thirteen; if not confirmed the full halving search runs.
1b. A pair with no moving arm cannot be closing (the resolver only acts on closing arms), so the scheduler does not wake it until an arm of it moves.
Existing source files changed: `src/safety_filter/{config,gap_rule,kong_style,resolver,shapes}.py` and `src/safety_filter_plugin.py` (four lines: it passes the bridge speed limit); new: `src/safety_filter/{tick,tick_arrays}.py`. Both resolvers (kong_style and layered) use the shared parts and both pass the gate (layered on punch original and cat).

## What was dropped, and why
- Pure conservative advancement (the form David approved): punch original 49 ms mean (vs 60 kept), cat p95 19.5 ms. Part A: 0 failing ticks on 5 recordings, one cat tick (5672) differs: the guard fraction is 0.0955 instead of 0.742, command difference 1.9 mrad. A fine scan of that tick's path (3000 poses) shows a pair below the guard gap from 10.6 percent of the step on, which the old sampled test jumped through: the advancement is the safer decision. But the cat closed loop then diverges (holds 751 to 695, KONG_PUSH 192 to 2101, BRANCH_GUARD holds 0 to 672, smallest true gap -19.2 to -28.6 mm, hand error max 82 to 163 mm), part B fails, and a change is kept only if the gate passes; so the kept form lets the old sampled test confirm every advancement. Left as an option below.
- Step 3 candidates 3 (one swept broad-phase pass), 4 (batched closed-form forward kinematics) and 5 (allocation) were not built as separate changes (see "Not done").
- Reading the bridge limit from `config.py` was tried first and dropped: `structure_test` forbids the filter package from importing project modules; the caller passes it instead.

## Not done
- Step 3 candidates 3, 4, 5, and step 5/6 (compiled kernel): not built. Numba is not installed and the shared environment must not be touched; a separate environment would need the licensed geo_kin wheel too. A C kernel loaded with ctypes (gcc is on the VM) would work without a new environment but needs a bit-parity test against the scalar distance; a build step on the VM and on the PACE cluster would be needed (`gcc -O2 -ffp-contract=off -shared`), with the pure-Python path as the automatic fallback.
- Step 4 / Part B (never-close extension): not started ("if time remains"). Nothing in `robot_shapes.json` or the never-close list changed.
- No existing branch or note about a divide-and-conquer never-close method was found (`git branch -a`, grep of docs and scripts); that part of step 4.1 was only searched, not built on.
- The full suite result is in the section below.

## Tests
Filter test set (13 tests): on the tag's code 12 pass, `safety_filter_spec_test` fails ("the closing right arm moves", a known old failure, docs/reports/safety_filter_merge_main_20261113.md); on every kept commit the same 12 pass and the same one fails. Full suite: see the last section of this file (written when it finished).

## Options for David (none implemented)
- Use the pure conservative advancement for the guard (about 20 percent faster on punch original, 49 vs 60 ms): it changes one cat decision (the safer one) and the cat closed loop then diverges, so it would need the gate's cat part B tolerance to be judged on a different footing (for example by comparing safety outcomes, not trajectories).
- Single-pass guard in the Kong style (one sweep instead of the sampled halving): would drop the guard's 22 to 29 percent share; changes where shortening happens.
- Fewer guard samples or halvings: faster shortening search; changes the shortened fraction by up to the coarser step.
- Convert once per arm per sweep instead of once per pair: fewer solver calls; changes the pushed poses.
- Raise the 0.5 mm listing threshold or clip joints instead of holding: fewer pairs and holds; changes decisions.
- Treat the bridge speed limit as the real bound on the filter's input (reject or clamp faster commands before the filter): the scheduler would stay on during the punch streams, which move 5 times faster than the bridge allows; changes what the filter sees on those streams.
- Compiled distance and FK kernels (ctypes C or numba): expected 2 to 3 times on acting ticks, a smaller gain on pass-through; costs a build step and a parity test.

## Decisions for David (one line each)
- Gate thresholds (1e-3 rad, 1 percent counts, 1 mm gap, 0.5 mm hand) are mine; confirm or change.
- Scheduler safety_buffer 0.20 (your value) and my guards: a command step faster than the bridge limit x dt x 1.2 wakes that arm's pairs; a base, pedestal or finger change over 0.2 mm wakes all pairs; confirm the 0.2 mm.
- A pair none of whose arms moves is not looked at (change 1b): the informational active set after a tick differs from before on 1302 to 1483 simulator ticks; confirm that the active set is not part of "what it decides".
- The speed bound is the bridge limit passed in by the caller; with no bridge (or a replay) the scheduler is off.
- Pure advancement dropped; keep it as an option or not.
- Golden simulator runs use a pinned stance-search seed (sampler 1944726090, full check 4242, the moved-layout rerun of M320) because the old runs took seeds from the clock and could not be repeated; confirm.
- Part B and compiled code not done: say whether to continue.
