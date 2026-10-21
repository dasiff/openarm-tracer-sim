# Part 1: the drop monitor decides; A1 daily-quota stop; answer to E1 (2026-10-21)

Written for: David, who decided P1, P2, A1 and asked E1.
Commits: see `git log` from "Part 1 + A1" onward; nothing pushed. The navigation proposal is its own report: `docs/reports/20261021_2121_navigation_design.md`.

## 1. What changed

- **P1, the drop monitor decides (M044).** `src/mpvi_monitors.py` gives each "dropped" entry `decides: True` (`PICK_PLACE_CFG["drop_monitor_decides"]`, default True; False restores logging only). The orchestrator fails the current step on such an entry (kind "drop") and the failure goes into the retry path of F1: a pick is retried when the object is out of the hand and upright (the world model says so), a place or a navigation step (a drop during the carry) is not and goes to the operator or the stop. The scripted lift check in `src/mpvi_modules.py` (the check that read the beaker's rise from the simulator and failed the step below 3 cm) no longer decides: it logs `UP: the beaker rose X mm [logged only]` and keeps the number in the module's report; `lift_check_m` is removed from the config. The attempt log carries the monitor's oracle "held?" verdict beside the failure.
- **P2, the LATER items** are in design plan 10: a continuous slip monitor with a failure plan; a rolling-window smoother for long trips and replanning on the move with a seam that keeps position, heading and curvature continuous. (Also there: coarse-to-fine or quadtree grids, naming of unknown obstacles, tracking moving objects, a paused reframe, the MoveIt-style arm planner, the payload-aware gravity feedforward.) The design plan section on the pick check (8, D4 addendum 6) says the monitor now decides.
- **A1, daily quota (M043).** `src/llm_gemini.py` raises `LLMDailyQuota` (a subclass of `LLMError`) for a 429 that names a per-day limit, as before not retried. The checker (`src/mpvi_vlm.py`) answers with `details["pipeline_problem"] = "checker unavailable: daily quota"`; the orchestrator stops the plan with that text, logs it in `state["pipeline_problems"]` and as an attempt with the outcome "pipeline_problem" (so it is not counted among the step failures), no retry, no operator question, and the episode outcome is `PIPELINE_PROBLEM`. Every other failed call (a timeout, a server error after the backoff, a per-minute 429) keeps the rule: not complete, retry, then the operator.
- **The standing rule** is in design plan 0. I applied it to the settings I touched: `stall_speed_m_s`, `stall_dwell_s`, `held_travel_range_m`, `open_reached_margin_m` (M020), `drop_margin_m`, `drop_dwell_s`, `drop_monitor_decides` (M025, M044), `retry_max` (M026), `knocked_over_deg` (M029), `vlm_cameras` (M016): each has a plain-word comment (what, unit, why, row id). The other settings of those dictionaries were not audited; that would be the next step of the standing rule.
- **Decisions log:** ids M001 to M073; the rows for your messages 1 to 3 and R7 are M042 to M073 (the standing rule M042, A1 M043, P1 M044, B1 to B6 M045 to M050, C1 and C2 M051 and M052, the navigation answers M053 to M059, S-Q9 to S-Q11 M060 to M063, V1 to V4 M064 to M067, L1 to L3 M068 to M070, R7 M071 to M073); the gravity compensation row (M035) and the build order row (M036) were updated in place.

## 2. The forced slip again, with the monitor deciding

`experiments/forced_slip_test.py` (2 N, `GRIP_FORCE_TABLE` untouched, `retry_max` 0, log `data/logs/part1_forced_slip_20261021_212442.log`):
- Before (the scripted lift check decided): the monitor fired at t = 59.70 s and logged; the plan ended later, when the lift check found 0 mm.
- Now: the monitor fired at t = 59.70 s (finger opening 3.3 mm against 37.7 mm held, the oracle's held verdict False at that tick, the beaker 75 mm from the pad centre) and **it failed the step at that moment**: "the object was dropped: the finger opening is 3.3 mm, it held 37.7 mm", outcome PLAN_FAILED (no retry because `retry_max` was 0 and no operator was present). The same time, no lift check involved.
- What the monitor does not see (unchanged, and the first LATER item above): in this run the beaker did not rise for about 1.3 s while the fingers stayed within 0.1 mm of the held width; the monitor fired only when the pads let go. A beaker that slides down and stays between the pads would not fire it, and nothing else would catch it before the place check of the VLM; the lift measurement is only a logged number now.

## 3. Tests

| Test | Result |
|---|---|
| `vlm_checker_test` | 48/48; new: a daily-quota error gives "checker unavailable: daily quota", one call, no waiting; the orchestrator stops with it, logs a pipeline problem (not a step failure), no `recover()`, outcome PIPELINE_PROBLEM; a per-minute 429 is still retried |
| `mpvi_monitor_test` | 15/15; new: the monitor firing fails the step and the retry runs (fake observations: a drop in the pick, the pick retried and done at attempt 2); with the setting off it only logs; a drop during the carry fails the navigation step, not retried, the operator asked |
| `orchestrator_retry_test` | 25/25 |
| the suite without `face_test` (`data/logs/suite_20261021_212621_summary.txt`) | all 31 tests exit 0 (including `mpvi_pipeline_test` 13/13, `mpvi_text_test` 11/11 cache only with the replay bit-identical, `cc_mode_tests` 48/48); `face_test` not rerun (it passed alone 27/27 earlier today and nothing face-related changed; it needs the whole VM, see the close-out report) |
| `forced_slip_test` | measurement, section 2 |

What is not tested: the daily-quota path against the real API (only a fake client raising the same error class as the real one; the real quota was seen to raise it in the enum variance run on 2026-10-21); the monitor deciding inside a full retried episode in the simulator (the retry in the simulator was tested with an injected failure, `mpvi_retry_sim_test`, and the monitor with a forced slip that has no retry; the two are connected only in the fake-observation test).

## 4. E1: why does the scripted pick end with the arm extended at the carry height, not at the zombie pose?

**It is drift, not a decision.** The sources: design plan 6 says "Pick: start at home, grasp, lift, end at home holding the beaker", and 1 says the arms stay at home except during manipulation. The D3 design proposal you approved (`docs/reports/20261020_1323_d3_design_proposal.md`) was ONE continuous pick-and-place run: phases 1 to 8 pick, phase 9 RISE_CARRY (up to the carry height over the hotplate), phases 10 to 15 place, and only phase 17 RETURN_HOME at the very end of the whole run; there was no return home between the pick and the place. When the MPVI skeleton cut that run into two skills (pick, place) for the orchestrator, the pick kept its last motion (UP to the carry height, `carry_ref`, "the surface the next place goes to") as its end, and the place was planned "from where the arm and the held object are now" (`plan_place`). Nobody wrote a rule that the pick should end extended; the skeleton report (`20261020_1356_mpvi_skeleton_runs.md`) lists the carry reference as a LATER item (take it from the plan's next place step). It worked because the plan's navigation step between the two is a no-op (the stance is the same).
Under B2 and B3 (M046, M047) the pick should end at the zombie pose except before a null step, and a place demo starts from the zombie pose or from the lifted post-pick pose (B4, M048). So the fix is a return-to-zombie-pose phase at the end of the pick (holding the beaker, with the clearance of C1) and a place that can start from either pose; I have not changed the code. This also decides what the lean measurement (navigation proposal N9) should hold: the zombie pose with the beaker.

## 5. Open for you

- Whether to build the pick's return to the zombie pose now (it changes the baselines of the pipeline tests: the pick would end somewhere else) or with stance selection and the null steps (B2, B3), as planned.
- The open questions of the navigation proposal (R1 to R6 in its section 10).
