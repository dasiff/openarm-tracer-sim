HELD PICK-AND-PLACE BASELINE, 200 g (registered Nov 4, 2026, the M258 weighing in the control loop)
==========================================================================================================

What this folder is
-------------------
A reference copy of two output files from ONE held pick-and-place run with a 200 g beaker (glass 0.100 kg plus liquid 0.100 kg).
It replaces held_200g_20261104 (retired, in ../retired/held_200g_20261104/). Its weighing runs in the control loop (decision M258):
the holding verdict pauses the source, and the weighing (the same rate, rise threshold and cap as before) runs while the source is
paused. The earlier baseline had the weighing as a script step, so this run differs from it from tick 11014 on (the holding verdict).

Decision: David, Nov 4, 2026 (decision row M259 in docs/mpvi_decisions.md).

Files
-----
- liftoff_ticks.csv   one row per control tick (150 per second), 58 columns, LEFT hand only. Starts "tick, t_sim, pad_x_m, ...",
                      ends "bias_L7". 14,382 data rows (ticks 0 to 14381). Its weigh_state, weigh_base_m, weigh_rise_m and weigh_hold
                      columns are blank (the WEIGH step is gone); the lift-off tick is in the run's log (11788).
- episode_log.csv     one row every 0.1 s of simulated time: 959 data rows, 51 columns.

Fingerprints (SHA-256 of the file contents, from `sha256sum`)
-------------------------------------------------------------
- liftoff_ticks.csv  76c44f66ebee1a4be1dab13fb7a1f74684c6c49d8f53adf57e1730e9677da341
- episode_log.csv    237bff5f6ab05ea33bf84d5cf83a7eb16c23cb8dd39875be731011f6193c4171

How it was made
---------------
- Commit: e50815d ("Lift-off weighing moves into the control loop as a pausing helper (M258)"), branch held-weight-ramp-sag-off.
  The run used a clean scratch copy (a git worktree outside the repository) of that commit: clean_code = yes. The source code
  (src/) at HEAD is identical to e50815d at the time of registration.
- Date: 2026-11-04 (run started 21:59:42, exit 0 at 22:03:13).
- Command (inside experiments/run_capped.sh, 6 GB cap, under the MuJoCo lock; environment variables MUJOCO_GL=egl and
  OPENARM_SIGNAL_TRACE=1, the same switch as the earlier scratch runs; the signal trace file is not part of this baseline):
    python <scratch copy of the 200 g launcher, ROOT set to the scratch copy> <output folder> --liquid 0.100
  The launcher is data/bounce_check_20261103/run_mass.py with ONE line changed: the ROOT path points at the scratch copy.
- Settings: beaker liquid mass 0.100 kg (--liquid), glass 0.100 kg; the other settings are the committed defaults.
- Inputs: the same stance and reach caches as the earlier runs (data/reach_map, data/bench_region_cache, data/stance_map,
  data/home_pose_search, data/beaker_hotplate_layout, data/gravity_ff), copied into the scratch copy.

What this baseline can see
--------------------------
- The scripted pick (approach, close, the weighing in the control loop, the pause, lift), the held-mass ramp, the sag trim, the
  open catch and the scripted place (over, down, open, back, home) for a 200 g beaker. Every number in the two files.

What it cannot see (known limit)
--------------------------------
- A change that matters only at another mass is NOT seen (150 g, 250 g: the scratch runs of Nov 4 are not baselines).
- Policy-driven runs (no scripted skills), or the manual and teleop scenarios.
- The bench tools do not read this baseline: experiments/cc_bench_compare.py accepts only manual, teleop and policy.

Regeneration
------------
Do not overwrite this folder. A new reference gets a new folder with its date, and this README is updated by hand with the new
commit, command and fingerprints. INDEX.csv says which folder is CURRENT.
