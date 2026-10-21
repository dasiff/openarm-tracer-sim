# VLM completion checker: live runs, camera choice (2026-10-21)

Task: "put the beaker on the hotplate". Model gemini-3.8-flash, the paper's VLM Completion Checking Prompt verbatim. The VLM decides; the sim oracle is logged beside it.

## Runs

| Run | Grip check camera | Grip: VLM / oracle | Release: VLM / oracle | Outcome |
|---|---|---|---|---|
| 1 (live, 00:05) | left wrist | NO / PASS (DISAGREE) | not reached | PLAN_FAILED at segment 2/4, 5/8 test checks |
| 1 rerun (cache only, 02:36) | left wrist | NO / PASS (DISAGREE) | not reached | same; image hash 23af79d498a4 identical, replay bit-identical |
| 2 (live, 02:45) | eagle | YES / PASS (agree) | YES / PASS (agree), left wrist | SUCCESS, 8/8 |
| 2 rerun (cache only, 03:19) | eagle | YES / PASS (agree) | YES / PASS (agree) | SUCCESS, 8/8, exit 0 |

Logs: `data/logs/mpvi_text_vlm2_20261021_000536.log`, `mpvi_text_vlmcache_20261021_023648.log`, `mpvi_text_vlm_eagle_20261021_024501.log`, `mpvi_text_vlm_eagle_cache_20261021_031945.log`. Run 2 output: `data/mpvi_text_run/vlm_eagle_20261021_024501` (images in `vlm_images/`). Trace with each check: `docs/reports/20261021_0330_trace_put_the_beaker_on_the_hotplate.md`.

## Why the camera changed (David, 2026-10-21)

At `gripper_closed` the left wrist view is filled by the beaker between the two fingers: no bench, no hotplate, no sign of contact. It cannot separate "held" from "the gripper closed around a beaker that still stands on the bench". With the prompt's "be strict", NO is a fair reading of that picture. The eagle view shows the beaker in the gripper in the scene, and the VLM answered YES. The grip check now uses the eagle camera; the release check keeps the wrist camera (`LLM_CFG["vlm_cameras"]`, src/mpvi_vlm.py). Unit tests: experiments/vlm_checker_test.py 36/36.

## Caveats

- One run per configuration, temperature 0: this shows the pipeline works, not the VLM's error rate. Agreement is 2 of 2 with the eagle view, 0 of 1 with the wrist view at the grip.
- The release check was never tested against another view. Whether the wrist view can tell a beaker resting on the hotplate from one held above it is unknown (expected: no).
- Nothing checks the lift with a camera. The VLM checks fire at gripper events only (as in the paper); the lift is checked by the scripted module from the simulator's true beaker pose (src/mpvi_modules.py, `lift_check_m`). A grip that slips on the lift would pass the VLM grip check. Proposed (not built, waiting for David): a VLM check after the lift with the eagle camera.
- The live run took about 32 min of wall time against about 8 for the cache-only replays; stdout was block-buffered, so the log lagged (the run was not hung).

## Docs and code touched

src/mpvi_vlm.py, src/task_config.py (`vlm_cameras`), experiments/vlm_checker_test.py, docs/design_plan.md (§8 D4 item 1, fidelity table, real replacement), experiments/llm_cache (new entries: the eagle grip reply and the release reply, with images).
