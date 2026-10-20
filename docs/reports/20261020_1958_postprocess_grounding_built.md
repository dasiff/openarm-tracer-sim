# Report 2026-10-20 19:58: post-processing call, world model, grounding and search stub built; face fix; green

Written for: David.

**Result:** built in your order (world model and grounding with fakes, then the post-processing module, then the pipeline). Touched entries, run together (`data/logs/suite_20261020_194634_summary.txt`): mpvi_postprocess_test 51/51, grounding_test 52/52 (both without the simulator), mpvi_pipeline_test 13/13 (SUCCESS, replay bit-identical, 11112 ticks, the same arm clearance as before: 41.3 mm), face_test 27/27. The rest of the suite was not rerun: the run loop, the arms and the baselines are untouched (the viewer files only wrap the scene update for the head).

**Built (design plan section 8, D4 addendum, now DECIDED):**
- `src/mpvi_schemas.py`: Pydantic models for both calls. `Plan` is exactly the paper's four segment fields (extra keys refused); `Labels` adds the primitive (pick / place / other) and object phrases with roles (manipulated / destination). Pydantic 2.14 installed in the venv and added to requirements.txt. The provider (native structured output or Instructor) is still PENDING.
- `src/mpvi_postprocess.py`: the labeling call's interface (`make_llm_labeler(structured_call)`, `make_stored_labeler` for the hand-written labels), schema validation of the reply, check and group tags as plain code, the keyword cross-check (last match wins, ambiguous flag, unmatched and disagreements logged, `stop_on_disagreement` default off). The LLM's answer is the one used.
- `src/world_model.py`: knowledge base (id, phrases, position, axis, properties, how found, last seen), the sim ground-truth source (beaker held-by from finger contacts), every module reads through it.
- `src/grounding.py`: grounding at dispatch time, deterministic matching; not found calls the search stub ("object not found, search not implemented: ..."); several matches ask the operator (console and status text; scripted operator for tests) or stop with "ambiguous reference" and the candidates when unattended; place steps prefer and require the object in the manipulating hand.
- Orchestrator takes the tagged plan, grounds each manipulation step as it starts, stops on "no primitive for this step", refuses an untagged plan. The scripted pick and place and the oracle checker use the grounded ids.
- Face: it turns toward the camera of the viewer and the video composite (render time only). The video was not re-rendered.

**Not built, on record (LATER in section 8):** the operator's answer keys 1 to 9 in the viewer (the control loop does not pass key state to policy sources yet; the interface, console listing and status text exist, so unattended is the only live mode today); the pick's carry height still reads the "hotplate" entry (`carry_ref`) instead of the plan's next place step; the labeling prompt's wording is not tuned; no caching of the labeling reply by instruction yet (no real provider).

**Replay note:** a replay replays recorded commands, not the policy's decisions, so an operator's choice is already in the recording; section 8 says so (it earlier said the choice was recorded as an input).
