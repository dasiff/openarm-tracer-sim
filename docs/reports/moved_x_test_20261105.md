# Sideways move test: beaker and hotplate moved 0.04 m in x (2026-11-05)

Branch world-model-sources, tip 444cc4f (scratch worktree outside the repo). No repo code, config or doc changed; nothing committed or pushed. Runs in data/moved_x_20261105/ (git-ignored) with logs in data/logs/moved_x_*_20261105.log.

## Method

- Scratch launcher outside the repo: a copy of data/bounce_check_20261103/run_mass.py with ROOT set to the worktree. Before the run it sets demo_video.MOVED from two environment variables (x deltas) and wraps a few functions to record the probe values (scene after load, stance-selection world, planned targets, oracle and success results). Probe file: probe.json in each run folder.
- Reference layout = the layout the held baseline used: demo_video MOVED (beaker x 0.85, hotplate x 0.65, y 0.66). The baseline README says it was made with that layout (it calls dv.record_only).
- Each run through experiments/run_capped.sh (6 GB cap, peak RSS in the log), with OPENARM_SIGNAL_TRACE=1 as the baseline did, and --liquid 0.100.

## Run 0 (identity)

Compared the two baseline files with the run's outputs:

| File | Baseline SHA-256 (README) | Run 0 SHA-256 | cmp |
|---|---|---|---|
| liftoff_ticks.csv | 76c44f66ebee1a4be1dab13fb7a1f74684c6c49d8f53adf57e1730e9677da341 | same | identical |
| episode_log.csv | 237bff5f6ab05ea33bf84d5cf83a7eb16c23cb8dd39875be731011f6193c4171 | same | identical |

Result: byte-identical. The baseline README's row counts (14,382 and 959) do not match the files (14,320 and 956 lines, header included). The hashes match, so the README counts are wrong, not the run.

A first attempt (data/moved_x_20261105/run0_identity) stopped at the first oracle call because of a bug in my probe wrapper (wrong argument count). It is not a result; the second attempt (run0_identity_b) is the one compared.

## Run 1 (control)

Skipped. Stance selection is always on: make_pipeline_task calls apply_stance_selection unconditionally (src/mpvi_pipeline.py:66-67), and BENCH_STANCE_CFG["navigation"] is True (src/task_config.py:584). Run 0 is the control: same layout, selection on. Chosen stance (0.635, 1.135), yaw -90 deg, pedestal 0.10 (log: "[stance] run of steps [1, 3]: stance [0.635, 1.135] pedestal 0.10: full check PASS").

## Runs

| Run | Move | Exit | Wall time | Peak RSS | Pick (oracle) | Place (check_success) | Result |
|---|---|---|---|---|---|---|---|
| 0 identity | none | 0 | 3:35.3 | 4.14 GB | PASS, beaker to pad 3.1 mm | PASS | PASS (byte-identical) |
| 1 control | none | skipped (run 0) | | | | | |
| 2 beaker -0.04 | beaker x 0.81 | 1 | 0:48.5 | 2.67 GB | not reached | not reached | FAIL |
| 3 hotplate -0.04 | hotplate x 0.61 | 0 | 3:29.6 | 4.15 GB | PASS, beaker to pad 3.9 mm | PASS | PASS |
| 4 beaker +0.04 | beaker x 0.89 | 0 | 3:34.2 | 4.20 GB | PASS, beaker to pad 3.7 mm | PASS | PASS |
| 5 hotplate +0.04 | hotplate x 0.69 | 1 | 0:45.1 | 2.73 GB | not reached | not reached | FAIL |

Runs 2 and 5 stop inside stance selection: "no stance covers any of the objects: 1:beaker: best count 3 without obstacles, 0 with them; blocked by ['hotplate']" (StanceSelectionError, raised by src/mpvi_pipeline.py:70). In both, the beaker and hotplate are 16 cm apart (nominal spacing 20 cm). In runs 3 and 4 the spacing is 24 cm and selection passes. Not fixed; no repo change was allowed for this task.

## Follow check (x values, m; tolerance 1 mm)

Expected values: beaker and hotplate at the moved value for the object moved, and the reference value for the other.

| Run | Object | Scene after load (qpos / body / site) | Stance world | Pick target IN (xy) | Carry UP (xy, z) | Place DOWN (xy) minus hotplate centre | Success: hotplate top site | Success: beaker base at end | Verdict |
|---|---|---|---|---|---|---|---|---|---|
| 0 | none (0.85 / 0.65) | 0.85 / 0.65 / 0.65 | 0.85 / 0.65 | 0.849998 | 0.849998, 1.1205 | 0.649909 (-0.09 mm x, -0.86 mm y) | 0.65 | 0.649975 | PASS |
| 3 | hotplate 0.61 | beaker 0.85, hotplate 0.61 / 0.61 | 0.85 / 0.61 | 0.849998 (beaker reference, -0.002 mm) | 0.849998, 1.1205 | 0.609829 (-0.17 mm x, -0.83 mm y) | 0.61 | 0.609960 | PASS |
| 4 | beaker 0.89 | beaker 0.89, hotplate 0.65 / 0.65 | 0.89 / 0.65 | 0.889998 (-0.002 mm) | 0.889998, 1.1205 | 0.649751 (-0.25 mm x, -1.21 mm y) | 0.65 | 0.649693 | PASS |

Carry height: UP z = 1.1205 m in all three runs (it is set by the grasp band and the hotplate's height, not by the x move). Only the xy of the carry target is checked for follow.

The place target is the pad's planned position over the hotplate: the hotplate centre plus the grasp offset measured at the pick (src/mpvi_modules.py place planning). That offset is about 1 mm in y, so the place y is 0.8 to 1.2 mm off the hotplate centre by design, and the x offset is below 0.3 mm.

## Stance and pedestal per run

| Run | Chosen x, y (m) | Yaw | Selection pedestal | Pedestal used (staging) | Sensed pedestal_m at grasp (log, episode_log) | Changed from unmoved? |
|---|---|---|---|---|---|---|
| 0 (reference) | 0.635, 1.135 | -90 deg | 0.10 | 0.15 | 0.1494 | reference |
| 3 | 0.605, 1.145 | -90 deg | 0.10 | 0.15 | 0.1494 | yes: 0.032 m (-0.030 x, +0.010 y) |
| 4 | 0.635, 1.135 | -90 deg | 0.10 | 0.15 | 0.1494 | no |

- The pedestal used comes from pedestal_override_m = 0.15 m (src/task_config.py:587; M197, David 2026-10-28), which overrides the selection's 0.10. It is not the TASK_CFG reference 0.155. Sensed reading at grasp: 0.1494 in all runs including the baseline; the earlier 0.1527 reading was not reproduced here, and the 0.22 offline value is not in these runs.

## Caches

- Region cache (data/bench_region_cache, 15 files) and reach maps (data/reach_map, 25 files): no file was written during any run (checked by modification time after the start of run 0). So every lookup was a hit, or a region was not needed.
- The region key (src/bench_stance.py _key, get_region) contains the object's distance from the bench edge (depth), the bench-open flag, the map stamps, the robot and camera numbers. It does not contain the object's x position. A sideways (x) move keeps depth, so the cached region is reused, and the object's position is applied afterwards in the overlay, which is not cached.
- Stale-answer risk: not seen. The cached part is robot-relative, and x-dependent filters are recomputed (per the cache's own docstring). This is a reading, not a test. A y move would change depth and miss the cache.
- The LLM plan came from the cache (key d4d9caea...) in every run.

## Home pose (moved beaker)

- The pipeline reports only "full check PASS" for the stance (run 0, 3, 4 logs). No numeric home clearance is printed, and none is in the recording metadata. Not measured.
- The home clearance map takes the beaker position from the scene's world (M266): in run 4 the beaker is at 0.89 in that map.
- Hotplate occlusion is not checked for the hotplate's moved position (as the audit noted).

## Bounce numbers (information only)

Metric definitions from docs/reports/20261103_1640_bounce_check.md: e = pad z minus reference pad z (mm). UP rise and drop = max and min of e over lift-off -150 to +450 ticks; place DOWN swing = max - min of e over the held period (first tick with mass > 0 to the end of place OPEN). Computed directly from liftoff_ticks.csv (columns pad_z_m, ref_z_m, mass_kg, label) with the lift-off tick from the run log.

| Run | Lift-off tick | UP rise (mm) | UP drop (mm) | UP swing (mm) | Place DOWN max / min (mm) | Place DOWN swing (mm) |
|---|---|---|---|---|---|---|
| Baseline held_200g_20261104_m258 | 11788 | 4.06 | -2.11 | 6.17 | 4.99 / -4.40 | 9.38 |
| Run 0 (identity) | 11788 | 4.06 | -2.11 | 6.17 | 4.99 / -4.40 | 9.38 |
| Run 3 (hotplate -0.04) | 11778 | 3.50 | -3.72 | 7.22 | 4.28 / -4.91 | 9.19 |
| Run 4 (beaker +0.04) | 11713 | 3.53 | -3.02 | 6.55 | 4.48 / -4.36 | 8.84 |

For reference, the Nov 3 table printed 4.03, -2.12 and 9.39 for 200 g; this method gives 4.06, -2.11 and 9.38 on the baseline, so it agrees to within 0.03 mm. These are information only.

## Odd things, passing runs included

- Runs 2 and 5 fail at stance selection with the beaker and hotplate 16 cm apart; runs 3 and 4 (24 cm) pass. The obstacle rule is the likely cause; not decided.
- The place target's y offset from the hotplate centre is 0.8 to 1.2 mm (the grasp offset), a little larger in run 4.
- Lift-off moves with the beaker: tick 11788 (baseline), 11778 (hotplate move), 11713 (beaker +4 cm). The bounce drop changes from -2.1 mm to -3.0 and -3.7 mm.
- The baseline README row counts do not match its files (see run 0).

## Not done

- Runs 2 and 5 did not reach the pick or place, so their follow checks and bounce numbers are not available.
- No numeric home-clearance value; no hotplate-occlusion check.
- No y moves and no height moves (later steps).
