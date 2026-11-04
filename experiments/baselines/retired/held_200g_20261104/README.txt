HELD PICK-AND-PLACE BASELINE, 200 g (registered Nov 4, 2026)
=============================================================

What this folder is
-------------------
A reference copy of two output files from ONE held pick-and-place run: the scripted pick of a
200 g glass beaker (glass 100 g plus liquid 100 g) and the place onto the hotplate. It replaces
held_200g_20261103 (retired, in ../retired/held_200g_20261103/). A later code change should give
the same two files, byte for byte, if it did not change the held-object logic. If the files differ,
the change is real and must be explained (or the baseline regenerated on purpose).

Decision: David, Nov 4, 2026 (decision row M257 in docs/mpvi_decisions.md). Reason: later stage checks
compare against the code as it now is, after the open-catch changes (M249 to M256).

Files
-----
- liftoff_ticks.csv   one row per control tick (150 per second): 14,382 data rows (ticks 0 to 14381),
                      58 columns, starting "tick, t_sim, pad_x_m, ...", ending "bias_L7". LEFT hand only.
- episode_log.csv     one row every 0.1 s of simulated time: 959 data rows, 51 columns, starting
                      "sim_time, phase, mode, ...", ending "right_finger_force_N".

Fingerprints (SHA-256 of the file contents, from `sha256sum`)
-------------------------------------------------------------
- liftoff_ticks.csv  d159660ac1e9da7db4e563cb5d07779434976cbed671340c69d008712d290e9d
- episode_log.csv    0096b1aec8bb0cdf00ce9cf8639c5e026c2cf0bde51b9859bc384fa9a101923e

How it was made
---------------
- Commit: a560eb6 ("Open while not holding: start the ramp if held mass is left; release carries the caught
  force cap (M255, M256)"), on branch held-weight-ramp-sag-off. The run used a clean scratch copy (git worktree,
  outside the repository) of that commit, so there were no uncommitted edits (clean_code = yes).
- Date: 2026-11-04.
- Command (inside experiments/run_capped.sh, which caps memory at 6 GB; with the environment variable
  OPENARM_SIGNAL_TRACE=1, the same switch as the scratch run from 6450105; the signal trace file is not
  part of this baseline):
    python <scratch copy of the 200 g launcher, ROOT set to the scratch copy> <output folder> --liquid 0.100
  The launcher is data/bounce_check_20261103/run_mass.py with ONE line changed: the ROOT path points at the
  scratch copy.
- Settings: beaker liquid mass 0.100 kg (--liquid); glass mass 0.100 kg; the other settings are the committed
  defaults (liftoff rate 0.5 N/s, lift-off rise 0.3 mm, and so on).
- Inputs: the same stance and reach caches as the Nov 3 run (data/reach_map, data/bench_region_cache,
  data/stance_map, data/home_pose_search, data/beaker_hotplate_layout, data/gravity_ff), copied into the
  scratch copy.

What this baseline can see
--------------------------
- The scripted pick (approach, close, weighing at rest, lift-off detection, lift), the held-mass ramp,
  the sag trim, the open catch and the scripted place (over, down, open, back, home) for a 200 g beaker.
- Every number in the two files.

What it cannot see (known limit)
--------------------------------
- A change that matters only at another mass is NOT seen (150 g, 250 g).
- Policy-driven runs (no scripted skills), or the manual and teleop scenarios.
- The bench tools do not read this baseline: experiments/cc_bench_compare.py accepts only manual, teleop
  and policy.

Regeneration
------------
Do not overwrite this folder. A new reference gets a new folder with its date, and this README is updated by
hand with the new commit, command and fingerprints. INDEX.csv says which folder is CURRENT.
