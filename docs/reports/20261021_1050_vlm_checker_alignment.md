# VLM checker alignment with David's decisions of 2026-10-20 (D1 to D7)

Written for: David, who made the decisions and decides what happens next.
Commits: a4e1a14 (decisions log, D1, D2), d555989 (D3 to D7), 38f0ec6, 99edab3 (runs). Nothing pushed.

## 1. What was built, against D1 to D7

| # | Decision | Built | What I changed |
|---|---|---|---|
| D1 | VLM decides, oracle logged beside | already so; has its row | none |
| D2 | eagle camera for the checker | both gripper events use `eagle_cam` (`LLM_CFG["vlm_cameras"]`) | the release check moved from the wrist to the eagle camera; its old cached reply went stale |
| D3 | capture at the gripper event once settled | the close event already waited for settled fingers; the OPEN event did not check the open position, now it does | `open_reached_margin_m` in `PICK_PLACE_CFG`; open event needs the fingers within 3 mm of the open travel |
| D4 | oracle on the same frozen tick as the image; return to home logged separately | the image is rendered on demand at the event tick (`obs["render"]`, one added key in `src/control_loop.py`), the oracle is read in the same call; the tick is in `details["frozen"]`. The oracle already judged only the step's criterion (place: on the plate, upright, gripper open; no arm-at-home). The return to home is a separate log line after a place ends (`state["skill_end"]`) | no policy cameras needed any more (the episode runs without them: faster). Without `obs["render"]` the checker falls back to the latest policy frame |
| D5 | enum output, exact reply, unreadable or failed call = NOT COMPLETE | `text_call(choices=...)` in `src/llm_gemini.py`; `VLM_ANSWERS` in `src/mpvi_prompts.py`; `read_answer` is an exact match; reasons "checker reply unreadable" / "checker call failed" with the raw reply | replaced the old "starts with YES" reader; an empty reply (refusal) is a new `LLMEmptyReply` and counts as unreadable |
| D6 | group fill | the orchestrator puts the concatenated descriptions of the manipulation group (joined with "; ", my choice) in `segment["group_description"]`; the criterion is the final segment's; one check per group (already so: `check` is tagged on the last segment of the group) | new; before this the prompt had only the checked segment's own description |
| D7 | drop monitor, logging only | `src/mpvi_monitors.py`, called by the orchestrator every tick; exceptions inside it are caught and logged | new |

Not asked, not done: the paper's manipulation module also receives the group's concatenated descriptions (E4); our scripted skill gets the segment text unchanged. The cached free-text replies of the first wrist and the first eagle runs are still in `experiments/llm_cache` (4 of the 7 entries are now unused); I did not prune them.

## 2. The enum test

Live call, `google-genai` 2.31.0, `gemini-3.8-flash`, `response_mime_type="text/x.enum"` with a two-string `types.Schema` enum: **accepted**, the reply was exactly one of the two strings, so no JSON-wrapper deviation. Script: `experiments/enum_probe.py` (throw-away cache). **Caution:** that call, on the stored eagle image that had answered YES as free text, answered **NO**. One call, not repeated, so I cannot say whether it is run-to-run variance or an effect of the enum constraint. In the later live run both new calls (enum) answered YES. This is a point to watch for the VLM-versus-oracle measurement.

## 3. D3 thresholds (`PICK_PLACE_CFG`)

- Fingers "stopped": speed below 0.002 m/s (`stall_speed_m_s`) for 0.3 s (`stall_dwell_s`), and at least 1.0 s after the command (`close_min_s`).
- Close-and-hold: finger travel at least 3 mm (`held_travel_range_m` lower bound: stopped short of fully closed, something between them) and at most 40 mm (did not close). An empty grasp (below 3 mm) is a module error, no event.
- Open: fingers at rest AND within 3 mm of the open travel of 44 mm (`open_reached_margin_m`, the oracle's margin).
- Timeouts: close 5 s, open 4 s (the step fails).
- Drop monitor (D7): finger opening more than 3 mm below the opening measured at the grasp (`drop_margin_m`) for 0.05 s (`drop_dwell_s`).

## 4. The cache key

SHA-256 of the JSON list `[model, prompt version, schema name, the whole prompt text, image hash]`. For the VLM calls the schema name is "text", the prompt text is the filled verbatim prompt (so it contains the group description and criterion) and the image hash is the SHA-256 of the PNG bytes. **The image is in the key**, which is why the release reply went stale when the camera changed. The key is unchanged by this work: the enum option and the camera name are NOT in it. Consequence: any rendering change (camera pose, shadows, resolution, a different tick) misses the cache and needs one live call per check.

## 5. Is the final place checked?

**Yes.** The post-processing tags `check` on the last manipulation segment of a group and on the last segment of the plan, so the final place carries it; in the live runs the place check ran and decided (VLM YES, oracle PASS).

## 6. Results

| Run | Result |
|---|---|
| `vlm_checker_test` | 45/45 (fake client, no API) |
| `mpvi_monitor_test` (new) | 11/11: the drop monitor on fake observations incl. a forced slip, a blip, after the open event, a missing signal; the real oracle's place check with two conditions where only one holds (each way) |
| `llm_calls_test`, `mpvi_postprocess_test`, `grounding_test` | 35/35, 51/51, 56/56 |
| `mpvi_pipeline_test` (oracle, regression for the control-loop change) | 13/13, exit 0 |
| live run `mpvi_text_test --mode cache_or_live --checker vlm` (log `data/logs/mpvi_text_vlm3_20261021_104344.log`) | SUCCESS, 11/11, exit 0; grip: eagle, VLM YES / oracle PASS (agree); release: eagle, VLM YES / oracle PASS (agree); drop monitor: nothing logged; home line after the place: at home, largest joint error 0.004 rad; replay bit-identical. The grip call needed one retry after a 503, the release call one after a 504 (76 s) |
| cache-only rerun (log `data/logs/mpvi_text_vlm3_cache_20261021_104753.log`) | SUCCESS, 11/11, exit 0, same verdicts from the cache, replay bit-identical |

Not done: the full suite (`experiments/run_tests.sh`) was not rerun. The drop monitor has not been tried in a forced in-sim slip, only on fake observations; in the clean runs it logged nothing, so its agreement with the oracle's held verdict is not measured yet.

## 7. Findings you did not ask for

1. **The orchestrator stops the whole plan at the first failed step** (`fail()` in `src/orchestrator.py`). The decisions log's failure row says the paper's default is "mark failed and advance" (E5) and that continuing to a timeout is the VLA's behaviour (E6). The code does neither. Your D5 says the failed check "follows the normal failure path": today that is a stop. I noted this in the row and changed nothing.
2. The enum NO on a previously-YES image (section 2).
3. The release image is the eagle view of the beaker on the plate with the open gripper (a clear YES). The grip image shows the beaker in the gripper in the scene.

## 8. The decisions log, in full

# MPVI decisions log

One row per MPVI decision, newest at the bottom. Every MPVI decision gets its row in the same commit that implements it (design plan §0). The paper passages are in [mpvi_paper_excerpts.md](mpvi_paper_excerpts.md) (E1 to E6); a paper fact not in them is marked "not in excerpts, ask David". The fidelity table is in design plan §8.

Type: **faithful**, **deviation** (with the reason), **addition** (ours, not in the paper), **paper silent** (the paper does not specify, our choice), **temporary** (stand-in until the real part exists).

Rows dated before 2026-10-21 were seeded on 2026-10-21 from the design plan text only (§8, its addenda and the fidelity table); the code was not audited against them (design plan §10: "Fidelity audit of the MPVI code against the decisions log").

| date | decision | type | paper reference | decides or logs | default | decided by | code location |
|---|---|---|---|---|---|---|---|
| 2026-10-19 | The VLM decides whether a check passes; the sim oracle's verdict is computed for every check and logged beside the VLM's (agree or disagree), and never decides (restated 2026-10-20, D1) | faithful (the VLM decides) + addition (oracle logging) | E1, E6 for the VLM deciding; the oracle is not in the paper | VLM decides, oracle logs | VLM decides | David | src/mpvi_vlm.py `make_vlm_checker`; src/mpvi_modules.py `make_oracle_checker` |
| 2026-10-19 | The planner prompt, the VLM completion-checking prompt: the Appendix D texts verbatim | faithful | Appendix D prompts (src/mpvi_prompts.py) | n/a | verbatim | David | src/mpvi_prompts.py |
| 2026-10-19 | Plan format `{"task_name", "segments": [{"type", "description", "completion_criteria", "target"}]}`; each manipulation segment tagged with its primitive (pick or place), whether it is the last manipulation step before a navigation step, and the group (consecutive manipulation segments form one unit with the last segment's criterion) | faithful | E3, E4, E6 | n/a | as in the paper | David | src/mpvi_postprocess.py, src/mpvi_schemas.py, src/mpvi_plan.py |
| 2026-10-19 | Completion checks fire at gripper events: close-and-hold for a pick, open for a place | faithful | E1, E6 (proprioceptive trigger) | VLM decides | gripper events | David | src/mpvi_modules.py (events), src/orchestrator.py |
| 2026-10-19 | A step that fails or times out is marked failed and the plan advances; extra setting: stop and call the operator instead (off by default); every failure logged with its step and reason | faithful (advance) + addition (operator stop, failure log) | E5 | n/a | advance (operator stop off) | David | src/orchestrator.py `fail()`: today the plan STOPS at the first failed step (no advance, no continue-until-timeout); found while implementing D5, see the 2026-10-21 alignment report |
| 2026-10-19 | The orchestrator is synchronous in sim time: the physics waits while a module thinks | deviation (the simulation does not run in real time while the LLM, the VLM and the VLA compute, design plan §7.3) | not in excerpts, ask David | n/a | sync | David | src/orchestrator.py |
| 2026-10-19 | Navigation is the scripted staging, stopping at a stance from the reachability map instead of a handoff distance | temporary (scripted staging stands in for the module) + deviation (stance instead of handoff distance, because our VLA is arm-only) | not in excerpts, ask David | n/a | scripted staging | David | src/staging_source.py, src/mpvi_modules.py `make_scripted_navigation` |
| 2026-10-19 | Manipulation is a scripted skill driven by the primitive tags and the target object until the arm-only VLA exists | temporary + deviation (arm-only VLA, not whole-body, design plan §7.3) | not in excerpts, ask David | n/a | scripted skill | David | src/mpvi_modules.py, src/mink_arm.py |
| 2026-10-19 | The pour check uses the beaker's tilt as its trigger (no gripper event marks the end of a pour) | addition (the paper's primitives are pick and place) | E6 (primitives pick and place) | oracle (VLM pour check is LATER) | tilt | David | design plan §6 (not built in the checker) |
| 2026-10-20 | The planner and the VLM use `gemini-3.8-flash`, pinned in config | deviation (Gemini 2.5 Flash is refused for new API keys, 404 on 2026-10-20) | E1 (Gemini 2.5 Flash) | n/a | gemini-3.8-flash | David | src/task_config.py `LLM_CFG["model"]` |
| 2026-10-20 | No LLM fine-tuning | deviation (no annotated lab plans exist) | design plan §8 (the paper fine-tunes on BEHAVIOR plans; not in excerpts, ask David) | n/a | none | David | n/a |
| 2026-10-20 | The planner is called once, before the episode | temporary (replanning comes with exploration, design plan §13) | not in excerpts, ask David | n/a | once | David | src/mpvi_pipeline.py |
| 2026-10-20 | Post-processing labels the primitive and the object phrases with a second LLM call (enforced structured output); keyword matching cross-checks it, disagreements are logged and a setting (`stop_on_disagreement`) stops on them | paper silent (E3 says what the post-processing extracts, not how) + addition (keyword cross-check) | E3 | logs (stops if the setting is on) | stop_on_disagreement on in the text test | David | src/mpvi_postprocess.py, src/llm_gemini.py |
| 2026-10-20 | Navigation targets and the objects of a step are grounded against the world model (a knowledge base; sim ground truth as its source); a search call when nothing matches (a stub now); the operator is asked when several match (off in unattended runs) | faithful in kind (knowledge-base lookup and search, design plan §8) + temporary (sim source, search stub) + addition (ask the operator) | not in excerpts, ask David | n/a | lookup, stub search | David | src/world_model.py, src/grounding.py |
| 2026-10-20 | Object finding: sim ground truth in the world model now; later GroundingDINO on the eagle camera into the same knowledge base | temporary | not in excerpts, ask David | n/a | sim ground truth | David | src/world_model.py |
| 2026-10-20 | The checker gets the EAGLE camera image, for the grip check and the release check. Decided after the left wrist camera gave a false negative on a held beaker (the view is filled by the beaker, no scene). The paper is inconsistent: the Figure 3b caption (E2) says the trigger "directs the VLM to the corresponding wrist view (R WRIST)"; Appendix C.4 (E6) says the checker "receives the head-camera image". Provisional | faithful to C.4 (E6), deviation from Figure 3b (E2) | E2, E6 | VLM decides | eagle_cam | David | src/task_config.py `LLM_CFG["vlm_cameras"]`; src/mpvi_vlm.py |
| 2026-10-21 | No VLM check after the lift and none after the carry: the paper has none; checks fire only when a pick or place has just terminated, and only for the terminal state before a navigation step | faithful | E1, E6 | n/a | none | David | not built (decided against) |
| 2026-10-21 | A consecutive manipulation group is checked once, for its terminal state before navigation, with the group's concatenated descriptions and the final segment's criterion | faithful | E4, E6 | VLM decides | once per group | David | src/mpvi_postprocess.py, src/orchestrator.py (code status not audited) |
| 2026-10-21 | A VLM answer is YES if it starts with YES, NO if it starts with NO, anything else counts as NO and is flagged unparsed; a failed VLM call is a failed check with the reason, never read as YES | paper silent | Appendix D prompts (the answer format only) | VLM decides | NO on doubt | Claude (implementation detail) | src/mpvi_vlm.py `parse_verdict` |
| 2026-10-21 | D3. Capture moment: the VLM image is taken at the gripper event once it has settled, not after a fixed delay and not when the skill ends. Close-and-hold: gripper commanded closed, finger speed below 0.002 m/s for 0.3 s (at least 1.0 s after the command), finger travel at least 3 mm (stopped short of fully closed: something between them; an empty grasp is a module error, no event). Open: commanded open, fingers at rest 0.3 s AND within 3 mm of the open travel (0.044 m) | faithful | E2 ("closes and holds at a nonzero width"), E6 (proprioceptive trigger) | VLM decides | the thresholds in PICK_PLACE_CFG | David (thresholds: Claude) | src/mpvi_modules.py (close/open step), src/task_config.py `PICK_PLACE_CFG` stall_speed_m_s, stall_dwell_s, close_min_s, held_travel_range_m, open_reached_margin_m |
| 2026-10-21 | D4. The oracle judges the SAME frozen tick as the image, and only the step's completion criterion (pick: fingers stalled in the held range and the beaker's middle within 2 cm of the pad centre; place: on the hotplate and upright, and the gripper open). The image is rendered on demand at that tick (obs["render"], the control loop's hook), so no policy camera runs; the tick is in the verdict's details. The return to home is NOT in the verdict: logged as its own line after the skill ends (state["skill_end"]) | addition (oracle timing and the home line; logs only) | not in the paper (the oracle is ours); E6 for the terminal-state check | oracle logs | frozen tick | David | src/mpvi_vlm.py (frame, details["frozen"]), src/control_loop.py (obs["render"]), src/mpvi_modules.py `end_report`, src/orchestrator.py |
| 2026-10-21 | D5. The answer is forced by Gemini's enum output (response_mime_type text/x.enum) to exactly "YES, the task is complete" or "NO, the task is not complete", and our code checks the reply is exactly one of them. The enum option is not part of the cache key. google-genai 2.31.0 with gemini-3.8-flash accepts it (live probe 2026-10-21, experiments/enum_probe.py), so no JSON-wrapper deviation | paper silent (the paper only says the VLM returns a binary judgment, E6) | E6, Appendix D prompt | VLM decides | enum | David | src/llm_gemini.py `text_call(choices=)`, src/mpvi_prompts.py `VLM_ANSWERS`, src/mpvi_vlm.py `read_answer` |
| 2026-10-21 | D5. A reply that is not exactly one of the two answers, a refusal or empty reply, or a call still failing after the existing retries counts as NOT COMPLETE, with the reason "checker reply unreadable" or "checker call failed" and the raw reply logged. The step then follows the normal failure path of the orchestrator. Supersedes the row of 2026-10-21 on "YES if it starts with YES" | paper silent | not in excerpts, ask David | VLM decides | NOT COMPLETE on doubt | David | src/mpvi_vlm.py |
| 2026-10-21 | D6. The check's {description} is the concatenated descriptions of all segments of the manipulation group (joined with "; "), {completion_criteria} the final segment's criterion; a group is checked once. The separator is our choice | faithful (the concatenation and the final criterion); paper silent (the separator) | E4, E6 | VLM decides | "; " | David (separator: Claude) | src/orchestrator.py `with_group`, src/mpvi_vlm.py; the tags are src/mpvi_postprocess.py |
| 2026-10-21 | D7. Drop monitor: from the pick's close event to the open event it logs "dropped" once when the finger opening stays more than 3 mm below the opening measured at the grasp (the width held) for 0.05 s; the entry has the time, the finger opening, the width held and the oracle's held? verdict at that tick. Real-robot signals only (the measured finger opening and the commanded state); no gripper effort (the sim has no effort readback matching the real robot's). It never fails or stops a step (an exception inside it is logged as monitor_error). Whether it should act is decided later, from how often it fires and agrees with the oracle | addition | not in the paper (E6 only says proprioceptive heuristics trigger the checks) | logs only | on | David (thresholds: Claude) | src/mpvi_monitors.py, src/orchestrator.py (monitors), src/mpvi_pipeline.py, PICK_PLACE_CFG drop_margin_m, drop_dwell_s |
