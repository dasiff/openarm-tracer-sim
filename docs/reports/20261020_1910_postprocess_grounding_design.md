# Report 2026-10-20 19:10: design of the post-processing call, world model, grounding and search stub

Written for: David. Design only: nothing built, nothing pushed for this. The design is recorded in docs/design_plan.md section 8 ("D4 addendum", marked PROPOSED), in the fidelity table (one new row) and in B8 of section 9 (the search item, DECIDED as you said).

## Two messages that disagree, and what I did
You sent two messages while I was on the face fix. The first said: build `src/mpvi_postprocess.py` as proposed (keyword matching), run the MPVI test and the touched suite entries, push main. The second said: design only, report and wait, with post-processing as a second LLM call and keywords as a cross-check. I took the second as the later one and did NOT build the module or push. Your three answers from the first message are folded into the design as the keyword cross-check (no extra verbs, unmatched descriptions logged, LAST match wins with `ambiguous: true` and a log line). The second message also ends mid-sentence in item 4 ("Record all of this in section 8, including the"); I could not tell what followed.

## The design in short
1. **Post-processing, second LLM call.** One call per plan, cached by instruction. Per manipulation segment, in a schema the API enforces: `primitive` (pick / place / other) and `objects` (phrases). `check` and `group` stay rule-based code. A reply that fails the schema stops the plan before it starts. Keywords cross-check the primitive; disagreements are logged. Until the LLM is connected the hand-written plan supplies the two fields. "other" stops the step with a reason.
2. **World model (`src/world_model.py`).** Knowledge base entries: id, name and phrases, position and orientation, properties (full/empty, held by, hot), how found, last seen (sim time). Fixtures from the lab map. Every module reads objects only through it. A source fills it: sim ground truth now, perception later, same entries.
3. **Grounding (`src/grounding.py`), at dispatch time.** Deterministic phrase matching with property words, no LLM. One match: goes on. None: the search module is called; the stub says "search not implemented" and the plan stops. Several: the operator is asked (viewer panel and console, candidates with positions and properties, keys 1 to 9); the choice is logged and recorded as an input so a replay stays bit-identical. Unattended: stop with "ambiguous reference" and the candidates.
4. **B8:** the real search is GroundingDINO on the eagle camera, then frontier exploration; its findings enter the world model as `found = detector`.

## Decisions I need from you
1. Which answer wins when the LLM and the keywords disagree? I propose the LLM's, with the keywords only logging and a setting to stop on a disagreement.
2. Roles on the object phrases (manipulated / destination): a place step needs both "the beaker" and "the hotplate", so I added them to the schema. OK?
3. What came after "including the" in item 4 of your message?
4. Go ahead and build this design (order: world model and grounding with fakes, then the post-processing module, then the pipeline test), and then the push of main including 47f9f9b?
