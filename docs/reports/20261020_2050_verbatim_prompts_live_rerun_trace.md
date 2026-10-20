# Report 2026-10-20 20:50: verbatim paper prompts, live rerun from text to SUCCESS, trace tool

Written for: David.

**The trace report to read: [docs/reports/20261020_2043_trace_put_the_beaker_on_the_hotplate.md](20261020_2043_trace_put_the_beaker_on_the_hotplate.md)**

## Done
1. **Prompts verbatim** in `src/mpvi_prompts.py`: the Appendix D Task Planning Prompt (as pasted: it says "house") and the VLM Completion Checking Prompt (stored for the checker, not used yet). Both are filled by `fill_prompt()` (the JSON braces forbid `str.format`); a test pins their first and last lines, the JSON example and the fields. Versions `planner-appendix-d-v1` and `vlm-check-appendix-d-v1` (part of every cache key). The placeholder's replies were removed from the cache.
2. **Fidelity table** (and section 8): model `gemini-3.8-flash` because 2.5 Flash is no longer available to new API keys (unavoidable), the same model for the VLM checker; the planning prompt is now "faithful" (verbatim).
3. **Live rerun, stop_on_disagreement ON: SUCCESS.** The planner's reply was stored by a probe call just before, the labeling call was live (57 s, one try). Outcome SUCCESS, all four segments done, replay bit-identical (11114 ticks). New replies are in `experiments/llm_cache/` (2 files); `mpvi_text_test` from the cache alone passes 7/7 too. Run files: `data/mpvi_text_run/live2_20261020_203906/` (llm_log.json, summary.json).
4. **Trace tool** `experiments/mpvi_trace.py "<instruction>"`: no simulator, from the cache by default (`--mode cache_or_live|live` to call the API, `--out FILE`, `--side`). Sections: planning request as sent (model, settings, prompt, schema), raw reply, parsed plan, labeling request and raw reply, parsed labels, keyword cross-check per step (and whether stop_on_disagreement would stop), tags and execution units with the closing check, and a grounding dry run (the hand simulated, the world model from the configuration).
5. Docs only, as you asked: evaluation task 6 "object not where the plan assumes" (with and without a door), the three candidate improvements to compare against the faithful baseline, and the need for an "open" skill, in design plan section 10.

## What the verbatim prompt made the LLM do
Plan (4 segments, not 3):
0. navigation "Navigate to the beaker" (target `beaker`)
1. manipulation "Grasp the beaker"
2. navigation "Navigate to the hotplate" (target `hotplate`)
3. manipulation "Place the beaker onto the hotplate" (done when: "The beaker is resting upright on top of the hotplate and the gripper is released")

Labels: segment 1 pick (the beaker: manipulated), segment 3 place (the beaker: manipulated; the hotplate: destination). Keyword cross-check: "grasp" = pick and "place" = place, no disagreement, nothing unmatched or ambiguous. Tags: both manipulation steps are their own unit with a check (each is followed by a navigation step): pick checked at `gripper_closed` (the check now decides and passed), place at `gripper_opened`.

## One change you did not ask for, needed to make the verbatim plan run
The paper's prompt gives no list of locations, so the navigation targets are the LLM's words ("beaker", "hotplate"), not our "bench". I made a navigation target a phrase matched against the world model: an object or fixture with a `location` gives that location's stance, so "Navigate to the beaker" is a drive to the bench; an unknown, stanceless or ambiguous target stops the plan with its reason (5 new checks in `grounding_test`). The scripted staging still reaches only the test task's stance (another stance stops the plan); the second navigation segment (to the hotplate, same stance) finds the staging done and ends at once, so the robot does not move between the pick and the place. Evaluation task 6 is where this limit will bite.

## Tests (all green)
mpvi_postprocess_test 51/51, grounding_test 56/56, llm_calls_test 35/35 (no API), mpvi_pipeline_test 13/13, mpvi_text_test 7/7 from the cache (`data/logs/suite_20261020_204340_summary.txt`). Not pushed: six commits are local. The VLM checker is next, on your word.
