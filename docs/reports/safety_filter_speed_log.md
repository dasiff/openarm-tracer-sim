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

NEXT: when the chain is done: manifest, filter test set on the tag's code (reference results), commit; then step 1 (profile).
