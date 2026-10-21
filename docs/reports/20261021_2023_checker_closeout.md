# Checker close-out (F1 to F7), 2026-10-21

Written for: David, who made the decisions F1 to F7 and reviews the result.
Commits: 71179e4 (retry, operator, group text), 774c38f, 96784a0 (docs), 48485aa and later (runs); nothing pushed.

## 1. What changed

- **F1 failure handling** (src/orchestrator.py, src/mpvi_modules.py `recover`, src/grounding.py operator `kind`, src/task_config.py `ORCH_CFG`): a failed MANIPULATION step is retried `retry_max` times (default 1): the failing arm alone goes back to its zombie pose (fingers opened first when the hand is empty, the hand backs off, pad centre and joints to the zombie pose; the other arm is not commanded), the world model is refreshed, the objects are grounded again, the skill runs again, the check fires again. After the retries the operator is asked (console line and the viewer status text: the step, the reason, the checker's image path; choices retry again, skip, abort; logged); unattended, the plan stops with the reason. No retry, straight to the operator or the stop: a place whose object is not in the hand, an object knocked over (up axis more than 30 degrees from vertical, my setting), a clearance or contact abort reported by the module (`error_kind()`), and, on the same rule, a navigation step, a grounding failure, a pick whose object is already in the hand, a failing recovery. These read the world model and the module, never the oracle. Every attempt is in `state["attempts"]` with the checker's verdict and the oracle's beside it. Design plan 8 D4 item 2 and the fidelity table are updated.
- **F2** the final manipulation group is checked (section 2).
- **F3** enum variance test (section 3). **F4** forced slip (section 4).
- **F5** the group's concatenated description and the final segment's criterion are passed to the manipulation module's `start()` for every segment of the group (the scripted skill keeps them for its report only).
- **F6** the cache key (section 5). **F7** design plan 10 has the situation-dependent retries as LATER.
- Not asked, added: test seams in `make_pipeline_task` (`orch_cfg`, `grip_force`, `module_hook`; the grip force override never touches `GRIP_FORCE_TABLE`), a `SKIP` environment variable in `experiments/run_tests.sh`, `experiments/enum_variance.py`, `experiments/mpvi_retry_sim_test.py`, `experiments/forced_slip_test.py`, `experiments/stance_selection_timing.py` (for the navigation proposal).

Consequences of the no-retry rules you should know about:
1. A failed completion check is retried only when the hand is empty afterwards (a missed grasp). A pick check that fails with the beaker in the hand (a VLM false negative on a good grasp) and a place check that fails with the beaker out of the hand go straight to the operator or the stop.
2. No manipulation-side source of a clearance or contact abort exists yet (the staging's clearance error is a navigation failure; the episode runner's contact monitor only counts). The rule is in place and tested with a fake module.
3. Answering the operator question from the viewer (keys) is not built (design plan 10, LATER): in an interactive run a present operator who does not answer is waited for until the episode timeout.

## 2. F2: is the final manipulation group checked?

Before: yes. `postprocess` tags `check` on the last segment of every manipulation group and on the last segment of the plan (`check = i == len(segs) - 1 or segs[i + 1]["type"] != "manipulation"`), and the live runs of 2026-10-21 ran the place check at the end of the plan (VLM YES, oracle PASS). After: the same code; `experiments/orchestrator_retry_test.py` now pins it (a plan ending with a pick and a place: the pick untagged, the place tagged and its check decides; a plan navigation, pick, navigation, place: both checked). Nothing had to be made so.

## 3. F3: enum variance (live, cache bypassed, nothing changed)

The eagle image of the enum probe (the stored eagle image of the grip), the verbatim prompt, 10 calls each way, temperature 0 (log `data/logs/enum_variance_20261021_111536.log`, json `..._111815.json`):

| Arm | YES | NO | Errors |
|---|---|---|---|
| free text | 10 | 0 | 0 |
| enum | 7 | 0 | 3 |

The 3 enum errors are Gemini quota errors (429 RESOURCE_EXHAUSTED, daily quota), not answers. So only 7 valid enum answers exist, all YES; counted with the one NO of the earlier probe, enum gave 7 YES and 1 NO in 8 valid answers. I did not rerun the 3 missing calls (the quota); say if you want them after it resets. Free text gave 10 of 10 YES, so the single NO does not show a systematic difference, but it does not rule out a small one.

## 4. F4: forced slip at 2 N (a measurement; GRIP_FORCE_TABLE unchanged)

`experiments/forced_slip_test.py`, log `data/logs/closeout_forced_slip_20261021_112025.log`. One pick with the grip force overridden to 2 N, the oracle checker deciding, `retry_max` 0, the drop monitor logging.
- The pick closed (the fingers stalled at 37.7 mm travel; the oracle's held verdict at the close event: PASS) and the lift began. The beaker did **not** rise (its base stayed at 959.8 to 960.7 mm); the finger opening crept from 37.67 to 37.59 mm while the hand rose (a creeping slip with the pads still pinching), then at about t = 59.65 s the fingers closed to 2.6 mm with nothing between them.
- **The drop monitor fired** at t = 59.70 s (below the held width since 59.647 s, the 0.05 s dwell): finger opening 3.3 mm against 37.7 mm held, **oracle held? False** at that tick (the beaker 75 mm from the pad centre): they agree.
- The plan ended later, when the scripted lift check found the beaker risen 0 mm (`UP: the beaker rose 0 mm ... dropped or slipped`), outcome PLAN_FAILED.
- Reading it: the monitor was blind during the creeping part (the fingers stayed within 0.1 mm of the held width while the beaker did not rise) and fired only when the pads let go. A slip in which the beaker slides down and stays between the fingers would not fire it. Nothing was tuned.

## 5. F6: the checker's cache key

`cache_key` (src/llm_gemini.py) is the SHA-256 of the JSON list [model name, prompt version, schema name ("text" for the VLM calls), the whole filled prompt text, the SHA-256 of the image's PNG bytes]. **The image is in the key**, and so are the group description and the criterion (they are in the prompt text). The camera name, the enum option, the attempt number and the retry are not. Consequence: any change of the rendered image (camera pose, resolution, shadows, the tick of the capture) misses the cache and needs one live call per check; each retry that changes the scene produces new keys. Nothing changed; a decisions-log row says so.

## 6. Test results

| Test | Result |
|---|---|
| `orchestrator_retry_test` (fakes) | 25/25: retry then success; a failed check retried when the hand is empty; retry exhausted then the scripted operator (retry, skip, abort each once, the choice logged, the question with the image-path slot); unattended stop; retry_max 0; each no-retry case; the final group checked; the group text reaching `start()` |
| `mpvi_retry_sim_test` (sim, real module) | 7/7: SUCCESS after an injected failure in attempt 1 of the pick; the pick grounded twice; all 1011 recovery commands name the left arm only; the right arm's joints moved at most 0.0008 rad; at the end of the recovery the left arm is at its zombie pose (largest joint error 0.007 rad) with the fingers open; replay bit-identical (12552 ticks) |
| `vlm_checker_test` | 45/45 |
| `mpvi_monitor_test` | 11/11 |
| the suite (`experiments/run_tests.sh`, three runs; summaries `suite_20261021_115344_summary.txt`, `suite_20261021_124725_summary.txt`) | all 32 tests exit 0: the first eight (arm_kinematics to home_pose_test) in the partial first run, 24 in the last run, including `mpvi_pipeline_test` 13/13, `mpvi_text_test` 11/11 and `cc_mode_tests` 48/48; `face_test` alone 27/27 (log `data/logs/face_test_alone_20261021_151835.log`) |
| the full pipeline run to success, recorded and replayed | `mpvi_text_test` in the suite (cache only, the real plan: navigation, pick, navigation, place): SUCCESS, replay bit-identical; and the live run of the earlier task |

Not run: `face_test` inside the suite (it needs the whole VM: it renders in two processes of about 2 GB each and the kernel killed it twice, at 11:33 and 12:05, with the editor and the Claude sessions running; the suite runner now has a `SKIP` variable for this).

## 7. New decisions-log rows (2026-10-21, F1 to F6)

| date | decision | type | paper reference | decides or logs | default | decided by | code location |
|---|---|---|---|---|---|---|---|
| 2026-10-21 | F1a. A failed MANIPULATION step is retried up to `retry_max` times (setting, default 1) instead of marked failed and skipped. A retry: the failing arm alone returns to its zombie pose (the module's `recover()`: fingers opened if the hand is empty, the hand backs off, pad centre and joints to the zombie pose; the other arm is not commanded, an object it holds stays held), the world model is refreshed, the step's objects are grounded again, the skill runs again, and the completion check fires again as usual. Reason (David's, not in the excerpts): the paper's steps can be skipped because they are independent; ours depend on each other (no place without a pick). The VLA's skills start from the zombie pose (§6), so this is the retry the VLA can do | deviation (reason: David's, above) | E5 (the paper's mark-failed-and-advance) | n/a | retry_max 1 | David | src/orchestrator.py (`fail`, `begin_retry`, `recovery_step`), src/mpvi_modules.py `recover`, src/task_config.py `ORCH_CFG` |
| 2026-10-21 | F1b. After the retries the operator is asked through the existing operator interface (console line and the viewer's status text): the step, the reason, the checker's image path; choices retry again, skip, abort; the choice is logged. Skip marks the step failed and goes on; abort stops the plan | addition | not in the paper | operator decides | asked when an operator is present | David | src/orchestrator.py (`fail`, `operator_step`), src/grounding.py (`ask(..., kind="failure")`); answering by viewer keys is LATER |
| 2026-10-21 | F1c. An unattended run (no operator present, the existing setting) stops the plan with the reason instead of asking | addition | not in the paper | stops | stop | David | src/orchestrator.py `fail` |
| 2026-10-21 | F1d. No retry, straight to the operator or the stop: a place whose manipulated object is not in the hand (world model hand contents), the object knocked over (world model orientation: the up axis more than `knocked_over_deg` from vertical), a clearance or contact abort reported by the module (`error_kind()`). These read the world model and the module, never the oracle. Added by Claude, same reading: a navigation step, a grounding failure, a pick whose object is already in the hand, a failing recovery are not retried either | addition (David's three cases) + Claude's four, same rule | not in the paper | n/a | knocked_over_deg 30 (Claude's choice) | David (the three), Claude (the rest, the 30 degrees) | src/orchestrator.py `no_retry_reason`, ORCH_CFG. Note: no manipulation-side source of a clearance or contact abort exists yet (the staging's clearance error is a navigation failure; the runner's contact monitor only counts): the rule is in place and tested with a fake module |
| 2026-10-21 | F1e. Every attempt is logged: attempt number, outcome, reason, the checker's verdict and the oracle's verdict beside it, the checker's image path, the decision (retry, operator, stop) and the operator's choice | addition | not in the paper | logs only | on | David | src/orchestrator.py `end_attempt`, `state["attempts"]` |
| 2026-10-21 | F2. The final manipulation group of a plan (no navigation after it) is checked like the others: the post-processing tags `check` on the last segment of a group and on the last segment of the plan. The code already did this (before and after the same; the live runs had the place check); a test now pins it | paper silent (E6 verifies the terminal state preceding a navigation subtask; the end of the plan is not mentioned) | E6 | VLM decides | checked | David | src/mpvi_postprocess.py (the `check` tag), experiments/orchestrator_retry_test.py |
| 2026-10-21 | F5. The group's concatenated description (and the final segment's criterion) is passed to the manipulation module's `start()` for every segment of the group (what the VLA will read as its task text); the scripted skill ignores it and only keeps it for its report | faithful | E4 | n/a | passed | David | src/orchestrator.py `with_group`, src/mpvi_modules.py `start` |
| 2026-10-21 | F6. The checker's cache key (src/llm_gemini.py `cache_key`) is the SHA-256 of the JSON list [model, prompt version, schema name ("text" for the VLM), the whole filled prompt text, the SHA-256 of the image's PNG bytes]. THE IMAGE IS IN THE KEY (and so the group description and the criterion, which are in the prompt). The camera name, the enum option and the retry attempt are not. So any change of the rendered image misses the cache and needs a live call. Nothing changed | paper silent | not in excerpts | n/a | as described | David | src/llm_gemini.py `cache_key`, `_run` |

(The earlier row "A step that fails or times out is marked failed and the plan advances" is marked SUPERSEDED in the log. The full log is `docs/mpvi_decisions.md`.)
