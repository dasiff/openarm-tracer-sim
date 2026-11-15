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

NEXT: read chain2 results; write the profile table and the gate proof here; commit steps 1 and 2. Then apply the drafted array-form tick (scratchpad newsrc/tick_arrays.py, tick.py) and gate it.

## Plan change (David, 2026-11-15, during step 0)
- Step 3 candidates replaced, in this order unless measurement says otherwise: 1 pair sleep timers (a pair is not checked again until the travel bound of its two parts uses up its gap minus margin minus slowing band; pass-through = every pair asleep; reset when the world list or the holding verdict changes); 2 conservative advancement in place of the halving search of the guard (APPROVED within the gate tolerance; report every differing tick; remove the halving code once the gate passes); 3 one swept broad-phase pass per step (numpy, no tree); 4 batched closed-form forward kinematics and exact capsule distances only for pairs whose box gap is below margin plus travel; 5 allocation and overhead (preallocated arrays, no per-pair dicts, buffered recording); 6 compiled kernel (numba first) with a Python fallback.
- Part B (never-close extension) moves AFTER step 5, "if time remains".
- Design prepared while the golden runs go (no src change yet; drafts in the session scratchpad): array form of a tick (pair table built once per item set, pose arrays, box separation for all pairs at once, gap intervals from batched calls, exact scalar gap only where an interval does not decide or the value is used). The sleep timers and the "exact only when near" rule of candidates 1 and 4 sit on top of it.
