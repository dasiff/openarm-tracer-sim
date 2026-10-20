# Report 2026-10-20 20:35: Gemini planner and labeling calls built; first live test from text to SUCCESS

Written for: David.

**Outcome: SUCCESS.** Instruction "put the beaker on the hotplate" went through the Gemini planner, the Gemini labeling call (stop_on_disagreement ON), the pipeline and the simulator: outcome SUCCESS, every segment done, the beaker on the hotplate (10 mm from its centre), replay bit-identical (11112 ticks). The same run from the stored replies (no API call) gave the identical result. Live run: `data/mpvi_text_run/live_20261020_201843/` (llm_log.json, summary.json, recording; log `data/logs/mpvi_text_live_20261020_201843.log`); cache replay: `data/mpvi_text_run/replay/`.

## What the LLM produced
Plan (planning call, 29.7 s, one try):
0. [navigation] "Go to the bench." (done when: "The robot is positioned at the bench."; target bench)
1. [manipulation] "Pick up the beaker." (done when: "The beaker is held in the gripper.")
2. [manipulation] "Place the beaker on the hotplate." (done when: "The beaker is placed on the hotplate.")
Task name "put_beaker_on_hotplate". Labels (labeling call, 252 s, 4 tries: three 504s, backoff 5, 9 and 16 s, then success):
- segment 1: pick, check False, objects: "the beaker" (manipulated); keywords say pick
- segment 2: place, check True, objects: "the beaker" (manipulated), "the hotplate" (destination); keywords say place
Keyword cross-check: no disagreement, nothing unmatched, nothing ambiguous.

## Built
- `google-genai==2.31.0` installed and pinned; Pydantic as before.
- `src/llm_gemini.py`: structured output (the Pydantic JSON schema as `response_json_schema`), retry with backoff on 429/5xx/network (hint-aware, capped; a per-day quota is not retried), cache by model + prompt version + schema + prompt, modes cache_only (default, never calls the API) / cache_or_live / live (bypass), a log of every prompt and reply, the key from GEMINI_API_KEY or the secrets file and masked everywhere (checked: not in the log, the summary or the cache).
- `src/mpvi_planner.py`, `src/mpvi_prompts.py`, `LLM_CFG` in `src/task_config.py`; stored replies committed in `experiments/llm_cache/` (2 files).
- Tests: `llm_calls_test` 34/34 (fake client, no API), `mpvi_text_test` 7/7 live and 7/7 from the cache; both are in `run_tests.sh`. The earlier tests still pass (postprocess 51, grounding 52).

## Three things that differ from what you asked, and one to decide
1. **Model: gemini-3.8-flash, not 2.5 Flash.** The API answers 404 "gemini-2.5-flash is no longer available to new users" for this key and suggests 3.8; I listed the key's models and pinned `gemini-3.8-flash` in LLM_CFG. This is a difference from the paper (and the VLM checker will face the same). Tell me if you want another model (e.g. 3.5-flash).
2. **The planning prompt is a placeholder written by me, not the paper's Appendix D.** That text is not in the repository (design plan 8 had marked it "check"), so the verbatim rule is not met. Please paste Appendix D; I replace `src/mpvi_prompts.py`, bump the prompt version and rerun the live test.
3. **Schema field:** the SDK's `response_schema` refuses `additionalProperties: false` (400), so the schema goes as `response_json_schema`; same enforcement.
4. **The LLM is not deterministic even at temperature 0:** my first probe gave task_name "place beaker on hotplate", the live run "put_beaker_on_hotplate" (the segments were the same). That is what the cache is for: tests and replays always get the stored reply.

Free tier: two calls were enough, but server errors (503 once, 504 three times) cost 4 minutes on the labeling call; the backoff handled them. Not pushed (commits are local). The VLM checker is next, on your word.
