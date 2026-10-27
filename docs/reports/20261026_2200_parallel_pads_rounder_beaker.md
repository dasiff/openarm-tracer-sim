# Parallel finger pads, 48-segment beaker, 300 g grip, handoff time limit (steps 1-7 of 2026-10-26) and the suite hangs

Written 2026-10-27 03:25. Not pushed.

## Erratum (pad angle)

The report `20261026_1846_pad_cause_twisting_friction.md` says the finger pads are toed in by 3.09 degrees each. That is WRONG. The 3.09 degrees came from also counting the pad's chamfer triangles (a 25 degree cone, 3 mm deep). The flat face alone (normals within 2 degrees, 0.2 mm deep) has yaw 0.14 deg and pitch 0.13 deg per pad, 0.39 deg between the two faces, gap 5.43 mm vs 5.37 mm over 12.2 mm (`data/pad_parallel/before.json`, `experiments/pad_parallel_check.py`). The pads are already parallel; no model change was made. The facets of the 24-segment beaker are within 1 degree of square (24 x 9.22 mm). What stands from that report: contacts land 2.3 mm from the pad tip with 24 segments and the half-segment rotation moves them to 6.9 mm, so the contact position depends on which facet meets the pad.

## Step 2: parallel pads
No change (see the erratum). The 24-segment run on the current pads (`data/grip_failure/20261026_213939`) gave the same numbers as the condim 4 run.

## Step 3: 48-segment beaker
`BEAKER_CFG["n_segments"]` = 48 (row M191). Physics-only timing bench (`data/logs/seg_timing.json`): 0.77 -> 0.91 CPU-s per simulated second (+17 %). `beaker_hollow_test` is parametrized (21/21); `pour_scene_test` hard-coded the 24-segment pair counts, fixed to follow `n_segments`, now 21/21.

## Step 4: 300 g grip force (48 segments, results `data/grip_failure/20261026_220606`, table `data/logs/s45_table.txt`)
All three succeed. Slide in the grip (lift / carry): 8 N 7.8 / 17.4 mm, 9 N 7.1 / 15.5 mm, 10 N 1.9 / 5.4 mm. The sag corrector hits its caps (40 mm, 0.1 rad) at 8 N. The pick handoff gives "no reading" under the strict still rule (loose 308.1 g, +2.7 %).

## Step 5: handoff weight reading time limit
`WEIGHT_READING_CFG["max_wait_s"]` = 1.0 s (row M192), `reading_status()`, test `weight_reading_timing_test` 8/8. David approved 1.0 s on 2026-10-27; the PENDING marks are removed.

## Step 6: full suite (tag 20261026_222306, SKIP=face_test)
41 modules exit 0. Four did not: `pour_scene_test` 19/21 (test fixed, rerun 21/21 on 2026-10-27 03:17), `transit_clearance_test` and `mpvi_pipeline_test` killed after 40 min and 2 h (exit 143), `mpvi_text_test` 6/11 (the Gemini VLM replies are missing; same as before, no live call possible). `face_test` is run alone.

## The two hangs: NOT reproduced
Rerun alone with a faulthandler dump every 300 s (`data/logs/fh_20261027_030523_*`): `transit_clearance_test` 27/27 in 2 m 15 s, `mpvi_pipeline_test` 14/14 in 4 m 20 s (replay bit-identical, 10769 ticks). The dumps are empty: neither test reached 300 s. So the 48-segment code did not make them slow or stuck.

What points to the VM instead:
- The kernel journal has soft-lockup and watchdog messages at 00:02-00:08 (CPU stuck for 526 s), 00:46-00:52 (systemd-oomd watchdog timeout, rcu stalls) and 02:32-02:37. `transit_clearance_test` started at 00:07, right after the first stall; it was killed at 00:46, the second.
- In the step 4 runs the identical 300 g episode took 1.79, 5.06 and 1.70 wall seconds per simulated second at 8, 9 and 10 N; the 9 N run was 3x slower for the same work.

What I cannot say: whether the suite runs were slow (starved) or stuck. Their stdout was buffered, so the logs do not show where they were. I did not kill them on evidence of a loop, only on elapsed time.

Proposal (not done): in `run_tests.sh` run each test with `python -u -X faulthandler` and a `faulthandler.dump_traceback_later(600, repeat=True)` wrapper, plus `timeout`, so a future hang shows its stack. Nothing else to fix.

## Grip force (read only, no default change)
The only real figure in the description files is `effort: 9` on the finger joint in openarm_description's `joint_limits.yaml` (the design plan, section 11, records it); by URDF convention that is per finger. The real hand's gains are kp 5, kd 0.1. The 333 N in the MuJoCo XML is a placeholder. `ARM_ACTUATOR_SPECS["fingers"]["force_limit"]` is 9 N per finger. So 10 N is NOT within the real limit: it exceeds it by 11 %, and only runs in the sim through the harness ceiling override. My proposal: keep 8 N as the glass-beaker default and 9 N as the ceiling; 10 N cuts the 300 g slide a lot (carry 17.4 -> 5.4 mm) but needs David's decision to exceed the 9 N figure, or a measured real value.

## Viewer status panel (read only)
The upper-left panel is the HUD (`src/viewer_layout.hud_rows`): sim time, base speed, base vx/vy, yaw rate, yaw, position, backend rows, then one "episode" row with the orchestrator's phase text, `<n>/<count> <NAV|PICK|PLACE>: <module phase> [<status>]` (examples: `2/4 PICK: OVER [running]`, `ASK OPERATOR, step failed: ...`, `PLAN DONE`), and the controller mode. The phase names come from the modules (staging steps, pick/place steps OVER, DOWN, CLOSE, UP, CHECK, BACK, HOME). It does NOT show the typed instruction, and the planned window (previous / current / next steps, "+k more") from the design plan is not built. Planning and choosing the parking spot happen before the episode starts and are not shown as phases (not verified for the navigation module's own phase text).

## Open for David
1. 10 N above the 9 N figure: allow, or measure on hardware?
2. Add the hang diagnostics (wrapper with timeout and stack dump) to `run_tests.sh`?
3. Show the typed instruction and the plan window in the panel (not built)?
