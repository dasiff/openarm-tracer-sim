# Report 2026-10-20 14:47: where the MPVI post-processing tags come from, and a proposal to make it its own module

Written for: David. Report only; nothing changed.

## Where they come from today
Not written by hand in the plan, and not a separate step in the pipeline. They are computed by the function `postprocess(plan, objects)` in `src/mpvi_plan.py`, and the orchestrator calls it itself when it is built (`make_orchestrator` in `src/orchestrator.py`, one line: `steps = postprocess(plan, world_objects)`). So the planner's output (today the hand-written `BEAKER_ON_HOTPLATE_PLAN`, which holds the paper's four plain keys per segment and no tags) goes straight into the orchestrator, which tags it internally. The function is separate code, but it is not a stage between planner and orchestrator, and the orchestrator is the only caller.

What it adds to each segment (a copy; the plan is not changed): `index`, `check` (True on the last segment of each unit of consecutive manipulation segments, so also on a final manipulation segment with no navigation after it), and for manipulation segments `primitive`, `object` and `group` (the unit's number).

Two differences from what you describe:
- **Primitive matching is narrower:** the lower-cased description must START with one of pick: "pick up", "pick", "grasp", "pick-up"; place: "place", "put", "set down". "Grab" is missing, and "Carefully pick up the beaker" or "Now put the beaker down" would not match.
- **No match raises a ValueError** (the whole plan is refused when the orchestrator is built) instead of "none".

## Proposal: post-processing as its own small module between planner and orchestrator
Pipeline: planner (hand-written plan now, LLM output later) -> `postprocess(plan)` -> tagged plan -> orchestrator. The module `src/mpvi_postprocess.py`:
- **Input:** a plan in the paper's format. It validates it (the checks of `validate_plan`, which moves with it, because an LLM plan is where malformed input comes from) and returns a NEW plan of the same shape whose segments carry the tags. The plan stays plain data (JSON), so it can be logged, cached by instruction (design plan 8, D4 item 8) and replayed.
- **Primitive tag:** matched on whole words anywhere in the description, case-insensitive: pick up / pick-up / grasp / grab -> "pick"; place / put / set down -> "place"; no match -> `primitive: None` (not an error here). If a description contains both ("pick up the beaker and put it on the hotplate"), the earlier match in the text wins and the tag `ambiguous: true` is added so it is visible; I would not split the segment (the paper's segments are single steps).
- **Check tag:** as now, per the paper's rule: the last manipulation step before a navigation step, and also the last one of the plan; consecutive manipulation segments form one unit (`group`), the unit's criterion is its last step's. Also `object` (first known object name in the description) as now; the object vocabulary is an argument, later the world model's.
- **Orchestrator change:** it takes the tagged plan and no longer calls `postprocess`; on construction it checks that every manipulation segment has the tags it needs. A manipulation step whose primitive is None stops the plan when it starts, with the reason "no primitive found for this step: <description>" (the stop-and-report behaviour, so the failure log says why; the plan is not refused up front, so a plan with one unknown step still runs the steps before it).
- **Where it is called:** in `make_pipeline_task` (`src/mpvi_pipeline.py`), between the plan and `make_orchestrator`, so a swapped planner changes nothing downstream.

## Tests (no simulator, all plain Python, a few seconds)
1. Wording: each of the listed phrasings for pick and place, with capitals, punctuation, a leading adverb ("Carefully pick up ..."), "pick-up", "set down" vs "set the ... on"; a description with no match gives `None`; words inside other words do not match ("input", "replace", "pickle").
2. Both words in one description: the earlier one wins and `ambiguous` is set.
3. Check and group tags: the test plan (nav, pick, place: both manipulation steps one unit, check on the place only); a plan with navigation between units (two units, a check on each unit's last step); a plan ending in navigation; a manipulation step first.
4. Input untouched (the original plan is not modified), the output is JSON-serialisable, and post-processing an already tagged plan gives the same result (idempotent).
5. Validation errors: unknown type, a navigation segment without a target, a manipulation segment with one, missing keys, empty plan.
6. Orchestrator with fake modules: a step with `primitive None` stops the plan with the reason and the earlier steps have run; an untagged plan is refused at construction.
7. The end-to-end test stays as it is (the same plan, the same result), and it also checks that the plan reaching the orchestrator carries the tags.

Size: about 1 to 2 hours, no simulator needed, then one rerun of `mpvi_pipeline_test` (about 3 minutes).

## Decisions I need from you
1. Go ahead with the module as above?
2. "Grab" and the other words: any more verbs for pick or place (for example "lift", "carry", "move ... to")? I would leave them out until the LLM's real wordings are seen.
3. Ambiguous descriptions: first match wins and is flagged (my proposal), or treat as no primitive?
