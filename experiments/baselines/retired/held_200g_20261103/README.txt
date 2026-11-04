HELD PICK-AND-PLACE BASELINE, 200 g (registered Nov 3, 2026)
=============================================================

What this folder is
-------------------
A reference copy of two output files from ONE held pick-and-place run: the scripted pick of a
200 g glass beaker (glass 100 g plus liquid 100 g) and the place onto the hotplate. A later code
change should give the same two files, byte for byte, if it did not change the held-object
logic. If the files differ, the change is real and must be explained (or the baseline regenerated
on purpose; see the last section).

Decision: David, Nov 3, 2026. Registered as ONE baseline, 200 g only. Reason: the manual bench
scenario never reaches the holding, weighing and trim logic, so it cannot check the staged moves.
This run does reach that logic. Its weight over-read (the estimate minus the true mass) is about
25 g, so changes to the weighing logic should show in it.

Files
-----
- liftoff_ticks.csv   one row per control tick (150 per second): 14,381 data rows (ticks 0 to 14380),
                      58 columns, starting "tick, t_sim, pad_x_m, ...", ending "bias_L7".
- episode_log.csv     one row every 0.1 s of simulated time: 959 data rows, 51 columns, starting
                      "sim_time, phase, mode, ...", ending "right_finger_force_N".

Fingerprints (SHA-256 of the file contents, from `sha256sum`)
-------------------------------------------------------------
- liftoff_ticks.csv  fa001adf804b24c8b9af58bea56fd8eef50a94284d8abfd78dd4be3c234a2211
- episode_log.csv    f346f50322ec40b7cc590e4d8bf0ef919ef8ff85b54048b3e585b8a16fbac8a5

How it was made
---------------
- Commit: 2b0586c ("Save the stage 1 signal trace report (Nov 3)"), on branch held-weight-ramp-sag-off.
  The run used a clean scratch copy (git worktree) of that commit, so there were no uncommitted edits
  (clean_code = yes).
- Date: 2026-11-03.
- Command (run inside experiments/run_capped.sh, which caps memory at 6 GB; the environment variable
  OPENARM_SIGNAL_TRACE was unset):
    python data/held_run/run_mass_wt.py <output folder> --liquid 0.100
  data/held_run/run_mass_wt.py is a copy of data/bounce_check_20261103/run_mass.py with ONE line
  changed: the ROOT path points at the scratch copy. Nothing else differs.
- Settings: beaker liquid mass 0.100 kg (set by --liquid; glass mass stays 0.100 kg); the other
  settings are the committed defaults (liftoff rate 0.5 N/s, lift-off rise 0.3 mm, and so on).
- Inputs: the same data as the Nov 3 bounce run. Read-only caches copied into the scratch copy:
  data/reach_map, data/bench_region_cache, data/stance_map, data/home_pose_search,
  data/beaker_hotplate_layout, data/gravity_ff (copies, same bytes as the originals). The mesh
  folder deps/OpenArm-Combined and the Python environment .venv were linked, not copied.
- Two runs were made, one after the other. They were byte-identical to each other, and each was
  byte-identical to the Nov 3 bounce run data/bounce_check_20261103/run_200g (same two files).

What this baseline can see
--------------------------
- The scripted pick (approach, close, weighing at rest, lift-off detection, lift), the held-mass
  ramp, the sag trim, and the scripted place (over, down, open, back, home) for a 200 g beaker.
- Every number in the two files: joint and pad positions, finger positions and force, the
  feedforward terms, the held mass and the ramp state, and the step names.

What it cannot see (known limit)
--------------------------------
- A change that matters only at another mass is NOT seen. Example: at 250 g the weight over-read is
  about 0.4 g, so a weighing change that only affects that mass would not appear here.
- It cannot see 150 g or 250 g behaviour, policy-driven runs (no scripted skills), or the manual
  bench scenario (that scenario uses run_loop directly and never reaches the holding logic).
- It does not check the camera images, the vision model (VLM) or the planner (the step order comes
  from a cached plan).

When to regenerate
------------------
Regenerate deliberately, and only at an intended output change. Do not regenerate to make a test
pass. A regenerated copy gets a new folder name with its date and short commit hash, the old one
stays in place, and the decision and the index row are updated in the same commit. Nothing here is
regenerated automatically.
