# Safety filter Stage 4: the plug-in (2026-11-13)

Written for: David. Model: Sonnet 5.5. Branch safety-filter-plugin (from safety-filter 4fe6272). Not pushed; main and src/safety_filter/ untouched.

## Built
- src/safety_filter_plugin.py (glue, outside the package): `SafetyFilterPlugin` (modes off/shadow/on, hold-policy slot, recording, run limits) and `make_sim_plugin` (builds resolver, world model stand-in + lab map fixtures, held-object tracker, joint-state reader).
- Control loop (src/control_loop.py): one call per arms command with arm targets, after the helpers and before the sag trim. Mode off builds nothing and imports nothing. Keys: safety_filter_mode, safety_filter_run_name, safety_filter_max_minutes, safety_filter_max_ticks. Config SAFETY_FILTER_PLUGIN_CFG (task_config.py). Flags `--safety-filter` on combined_controller.py and demo_video.py (plus run name and limits on demo_video).
- Tests: experiments/safety_filter_plugin_test.py (18 checks, stub filter, exit 0), in run_tests.sh. Structure test still passes. Smoke test experiments/safety_filter_plugin_smoke.py; summary tool experiments/safety_filter_plugin_summary.py.
- Rows SF-59 to SF-64 in docs/mpvi_decisions.md and the "Stage 4 build" subsection of design_plan.md section 16.

## Identity proof (manual bench scenario, headless, log data/logs/plugin_ident_20261113_185018_*)
- off: cmp exit 0, sha256 3060ae41cc85... = reference; cc_bench_compare 0 mm.
- shadow: cmp exit 0, same sha; 243 filter calls; 0 mm.
- on: differs (sha 68ae82e56e2c...), max diff 0.00075 mm (right finger, t=9.3 s).
- The manual scene has no task beaker, so the world model stand-in cannot be built there: the filter saw fixtures only (printed; row SF-62).
- First attempt failed on that (KeyError task_beaker); fixed with the fixtures-only fallback, then the three runs above.

## On-mode smoke (log data/logs/plugin_smoke_20261113_185502.log)
Left arm straightens its elbow over 2 s; one made-up box (8 cm cube, world list only) at the hand position at 80 % of the reach. 474 calls: PASS 71, KONG_PUSH 177, HOLD 226 (reason SHAPE_RECHECK). Smallest gap to the box with the commands sent: 1.7 mm (left palm); the raw path would reach -80.5 mm. The hand stopped short (goal 0.7 rad away). Filter time here mean 984 ms, max 2.06 s per call (a heavy contact case). Measurement only.
First attempt used a reach beyond the joint limits (the bridge rejected targets) and a box overlapping the start pose; fixed in the script.

## Shadow pick and place (demo_video.py --record-only, moved layout, scripted staging, cache_only plan)
- 600 ticks first (stopped by the tick limit as designed): 600/600 PASS, filter mean 4.0 ms, max 8.2 ms.
- Full run (log data/logs/plugin_shadowfull_20261113_190810.log, max_minutes 180, not reached): outcome SUCCESS (the earlier off run, rerun_moved_20261113_001429, also SUCCESS). 7779 filter calls (ticks that carried arm targets), covering 92.9 s of sim time, 100 % PASS, no non-PASS ticks to group.
- Filter time per call: mean 3.2 ms, median 2.9, p95 5.7, max 24.4 ms. Estimated for a full 14313-tick run: 0.8 min of filter time.
- Run results unchanged by shadow in outcome (SUCCESS both). A bit-for-bit comparison with the off run was not made (recordings do not replay bit-identically, progress note 2026-11-13).

## Options / notes
- Smooth stop is the next piece (slot ready). Under HOLD with repeat_last_safe the smoke run held 226 ticks.
- Filter time in contact cases is large (up to 2 s per call): fine for shadow, a real-time problem for on.
- Decision rows SF-59..64 were written after the code commits (same branch).
