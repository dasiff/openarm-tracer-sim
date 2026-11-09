# Per-task approach-angle lists, mock gripper fix, tidy, and the stance search (2026-11-09)

Decision rows: M279 (per-task lists and the choice), M280 (mock gripper fix). Scratch in scratch/ (not in git). Nothing pushed.
Terms: "approach angle" in all new names, comments, docs and reports. Existing "bin" identifiers are kept (listed in section 7).

## 0. Part 0: which stance search the pipeline runs (read only)
- The pipeline (src/mpvi_pipeline.py:66-67) calls apply_stance_selection (src/stance_select.py, the function near the end of the module), which calls select_bench_stance (src/stance_select.py:436).
- The stance is produced by the "current" search: the reach-map overlay (bench_stance.overlay, called through _overlay at src/stance_select.py:657-659; the calls at :477, :545, :666) ranks the stances by approach counts on the reach maps (data/reach_map, via the maps folder argument), then the full check (FullCheck.check) decides the top candidates (src/stance_select.py:577; rank_keep and full_check_fallbacks limit them).
- stance_check_mode: the default is "current" (src/task_config.py:607, from the OPENARM_STANCE_CHECK_MODE environment variable). "two_pass" is selected only by that setting (src/stance_select.py:564, :595). Neither mpvi_pipeline_test nor stance_select_moved_test sets it, so the pipeline runs "current".
- The Gaussian reach model (src/reach_model.py) is imported only by experiments/reach_model_*.py, experiments/reach_inverse_scene.py and tests of those. Nothing in src (outside src/reach_model.py) imports it. It is not in the pipeline's stance search.
- Pedestal height: the overlay ranks candidate pedestal commands (bench_stance.py:110, pedestal_commands, 0.05 to 0.25 m). The selection's own pedestal (the candidate's q) is the one the full check tests. The staging then parks at cfg["pedestal_override_m"] = 0.16 m (src/stance_select.py:622; BENCH_STANCE_CFG). So the check is at the candidate's pedestal while execution parks at 0.16 m (a known mismatch, see Decisions).
- Heading: fixed. ctx.yaw = task_cfg["staging_yaw"] (= -90 deg, src/stance_select.py:74; the selection uses ctx.yaw). The overlay, the check and the staging all use this heading; no heading is searched in the pipeline.

## 1. The per-task design (what changed)
- Points: each pick point carries its task (the plan step index) in its pick_path spec, and each place point carries it too (src/stance_select.py:221 "task": index; the three place points at lines 225-227, task=index).
- Check (src/full_check.py, check(), the block starting at line 633): each pick task is solved on its own along every approach angle (the bins, or the one given); its list is the bins that pass its whole path at every error stance (the nominal first; later stances start from the nominal solution, as before). Each place task keeps the full-circle search (_place_window) and reports its passing turns (passing_deg). A stance passes only if every task has at least one passing angle. The result carries res["task_angle_lists"] = [{"task", "kind", "angles"}]: pick angles are bin indices, place angles are degrees.
- Helpers: _task_of and _ordered_tasks (module level, src/full_check.py before class FullCheck).
- Stance selection (src/stance_select.py): the run keeps approach_angles per task step (line 600), a place served from the current stance records its list too (line 511), and each plan segment gets approach_angles (line 636). The old pick_approach_bin and place_theta_deg segment keys and place_theta_by_step are removed.
- Execution (src/mpvi_modules.py): start() reads the segment's list through task_approach_angles (line 289). A missing or empty list fails the step with its own reason before anything is planned: "approach angles missing" or "approach angles empty" (the skill does not run; no forward fallback). pick_bin_index() takes the one choice from the list (line 216); plan_place takes its turn from the list (line 262); plan_place and recovery share place_approach_xy (lines 268, 344).
- Recovery (src/mpvi_modules.py recover, lines 313-345): the failed step's primitive is captured before the recovery overwrites it (line 319). A failed pick backs out along its pick's angle (as before, through the list). A failed place backs out along the place's own angle (line 338-344) instead of the forward direction.
- New module src/approach_angle.py: ApproachAngleError (reasons "missing" and "empty"), task_approach_angles, choose_approach_angle (the first passing angle; the only choice; David's ranking replaces this function), combine_task_lists (the stance rule as a pure function, used by the analytic test; see Not done), place_approach_xy (the place direction).
- Readers updated: src/mpvi_pipeline.py:112-113 (panel: the first angle of the place list), experiments/stance_select_test.py (FakeCheck returns the per-task lists), experiments/place_theta_logic_test.py (reads approach_angles).

## 2. Step 7: does a task's starting posture depend on the end of the previous task?
Yes, in two places (not changed; see Decisions):
- The place path starts where the pick ended: plan_place takes the pad's current height as the OVER height (mpvi_modules.py, the over point uses p[2], the pad now) and the hand-to-beaker offset from the measured pad and beaker (off = p - b). The pick ends at its UP (the carry height and the grasp offset), so the place's OVER height and the held-beaker offset are set by the end of the pick. The check's place points use the stance model's offset instead (task_points / step_target setdown points), so this start is not what the check tested.
- A pick starts from the measured pad pose (arm.pad_pose) and the arm's current command (arm.reset / restart_from_commanded, default False). The first move's start is whatever the previous task left (the place's HOME, or the zombie pose). The check's OVER point is from the stance, not from that start.
Also: the M074 rule makes a pick end at the zombie pose when the next navigation step moves, so the next step decides the pick's end posture.

## 3. Mock gripper fix (M280) and the mock run
- Fix: src/mock_policy.py reset(holding) sets the gripper target to closed (0.0) when the segment starts while the hand holds the object, and to open (1.0) otherwise; begin() passes holding (world.held_by); make_mock_manipulation passes holding=world.held_by(side). The open value and the direction convention are unchanged (M260 undecided).
- Mock run (200 g, trim on, data/logs/pt/mock): the fingers stay at 37.5 mm and the beaker stays at z 1.068 m through the place's OVER move (t 87 to 95 s, x from 0.97 to 0.795, the hotplate). The beaker stays in the hand through the place's OVER. Before the fix (run of 2026-11-09 earlier) the fingers opened to 43.9 mm and the beaker fell at 88 s.
- New result downstream: the place's OVER fails at 101.0 s: "OVER: not reached after 15.0 s: 3.4 mm and 0.2 deg from the waypoint". So DOWN, OPEN and the completion check are not reached in the mock run. Not investigated (see Not done).

## 4. Tidy (Part 3)
- Removed the dead setting: experiments/bounce_freeze_run.py (Run B exits with a message; its text updated) and experiments/bounce_handoff_metrics.py (two strings). Nothing in src reads sag_reset_at_skill_start.
- src/mpvi_modules.py docstring (lines 33-34) now says the trim is a control-loop layer (src/sag_layer.py, applied at src/control_loop.py sag_correct).

## 5. The 51 stances through the per-task check
Two runs, both on the same stances and points as the earlier 49-stance runs (49 geometry-clean candidates from docs/reports/reach_inverse_spread_candidates.xlsx "candidates standard", plus the staging stance and point 4):
- (a) the per-task check, in one process (generator shared across stances, as in the pipeline): 23 pass, 16 fail the grasp IK, 12 fail the place window.
- (b) the per-task check with the random generator reset before each stance (the same as a fresh process; scratch only): 21 pass, 18 fail the grasp IK, 12 fail the place window.
- HEAD a85a070 (before this task) with the same reset: 21 pass, 18 grasp IK, 12 place window; the same status on all 51 stances as (b).
- The 23 of 51 of the earlier record is from older code (before M276/M277) run in one process. Between (a) and (b), two stances flip: [0.595, 1.002] and [0.591, 1.003] pass in the shared run and fail when run alone (fresh process: [0.591, 1.003] fails at 'heading+1' point 2 on both versions). That is an order artifact of the shared random generator, not a result of this change (see Decisions).
- Failures: the grasp IK fails at task 1 (the pick step), at band point 2 (the grasp at z 1.005 m), at an error stance ('x-1', 'heading+1', 'pedestal+1', 'nominal'), in every angle. The place window fails at task 3 (the place step) with the widest window under 10 deg (the place minimum).
- Which stances changed status: none between HEAD and the per-task check (51 of 51 the same, with the same reset). The per-task change adds the passing lists; it does not change which stances pass.

Table (per stance; status with the same reset; the pick lists and the place turns, first passing; the first failing point; HEAD with the same reset):

| # | stance (x, y, heading, ped) | status (per-task, reseeded) | pick approach-angle lists (bin indices) | place turns, first passing (deg) | first failing point | HEAD a85a070 (same reset) |
|---|---|---|---|---|---|---|
| 1 |  (0.870, 1.072, -100, 0.1) | PASS | [11] | -60..-10 (11 turns) |  | PASS |
| 2 |  (0.842, 1.077, -100, 0.1) | PASS | [11] | -60..-5 (12 turns) |  | PASS |
| 3 |  (0.806, 1.072, -90, 0.1) | PASS | [11] | -60..-10 (11 turns) |  | PASS |
| 4 |  (0.793, 1.074, -90, 0.1) | PASS | [11] | -60..-5 (12 turns) |  | PASS |
| 5 |  (0.781, 1.076, -90, 0.1) | PASS | [11] | -60..-5 (12 turns) |  | PASS |
| 6 |  (0.781, 1.051, -90, 0.15) | PASS | [0, 11] | -60..-5 (12 turns) |  | PASS |
| 7 |  (0.748, 1.032, -80, 0.15) | PASS | [11] | -65..-20 (10 turns) |  | PASS |
| 8 |  (0.773, 1.053, -90, 0.15) | PASS | [11] | -60..-5 (12 turns) |  | PASS |
| 9 |  (0.822, 1.069, -90, 0.1) | PASS | [11] | -65..-20 (10 turns) |  | PASS |
| 10 |  (0.731, 1.061, -80, 0.1) | PASS | [11] | -60..-10 (11 turns) |  | PASS |
| 11 |  (0.718, 1.063, -80, 0.1) | PASS | [11] | -60..-5 (12 turns) |  | PASS |
| 12 |  (0.710, 1.064, -80, 0.1) | PASS | [11] | -60..-5 (12 turns) |  | PASS |
| 13 |  (0.671, 1.012, -70, 0.15) | PASS | [11] | -60..-10 (11 turns) |  | PASS |
| 14 |  (0.658, 1.038, -70, 0.1) | PASS | [0, 11] | -60..-5 (12 turns) |  | PASS |
| 15 |  (0.654, 1.038, -70, 0.1) | PASS | [11] | -60..-5 (12 turns) |  | PASS |
| 16 |  (0.613, 1.000, -60, 0.1) | PASS | [11] | -60..-10 (11 turns) |  | PASS |
| 17 |  (0.658, 1.015, -70, 0.15) | PASS | [11] | -60..-5 (12 turns) |  | PASS |
| 18 |  (0.604, 1.001, -60, 0.1) | PASS | [11] | -60..-10 (11 turns) |  | PASS |
| 19 |  (0.665, 0.984, -70, 0.2) | FAIL ik | [] | - | task 1: stance 'x-1' point 2 (grasp) | FAIL ik |
| 20 |  (0.661, 0.985, -70, 0.2) | FAIL ik | [] | - | task 1: stance 'x-1' point 2 (grasp) | FAIL ik |
| 21 |  (0.645, 1.018, -70, 0.15) | PASS | [11] | -60..-5 (12 turns) |  | PASS |
| 22 |  (0.595, 1.002, -60, 0.1) | FAIL ik | [] | - | task 1: stance 'heading+1' point 2 (grasp) | FAIL ik |
| 23 |  (0.656, 0.986, -70, 0.2) | FAIL ik | [] | - | task 1: stance 'x-1' point 2 (grasp) | FAIL ik |
| 24 |  (0.591, 1.003, -60, 0.1) | FAIL ik | [] | - | task 1: stance 'heading+1' point 2 (grasp) | FAIL ik |
| 25 |  (0.604, 0.981, -60, 0.15) | FAIL ik | [] | - | task 1: stance 'pedestal+1' point 2 (grasp) | FAIL ik |
| 26 |  (0.561, 0.936, -50, 0.15) | FAIL ik | [] | - | task 1: stance 'nominal' point 2 (grasp) | FAIL ik |
| 27 |  (0.590, 0.983, -60, 0.15) | FAIL ik | [] | - | task 1: stance 'x-1' point 2 (grasp) | FAIL ik |
| 28 |  (0.539, 0.957, -50, 0.1) | FAIL ik | [] | - | task 1: stance 'nominal' point 2 (grasp) | FAIL ik |
| 29 |  (0.556, 0.936, -50, 0.15) | FAIL ik | [] | - | task 1: stance 'nominal' point 2 (grasp) | FAIL ik |
| 30 |  (0.530, 0.958, -50, 0.1) | FAIL ik | [] | - | task 1: stance 'nominal' point 2 (grasp) | FAIL ik |
| 31 |  (0.547, 0.938, -50, 0.15) | FAIL ik | [] | - | task 1: stance 'nominal' point 2 (grasp) | FAIL ik |
| 32 |  (0.526, 0.958, -50, 0.1) | FAIL ik | [] | - | task 1: stance 'nominal' point 2 (grasp) | FAIL ik |
| 33 |  (0.542, 0.939, -50, 0.15) | FAIL ik | [] | - | task 1: stance 'nominal' point 2 (grasp) | FAIL ik |
| 34 |  (0.521, 0.959, -50, 0.1) | FAIL ik | [] | - | task 1: stance 'nominal' point 2 (grasp) | FAIL ik |
| 35 |  (0.537, 0.940, -50, 0.15) | FAIL ik | [] | - | task 1: stance 'nominal' point 2 (grasp) | FAIL ik |
| 36 |  (0.516, 0.959, -50, 0.1) | FAIL ik | [] | - | task 1: stance 'nominal' point 2 (grasp) | FAIL ik |
| 37 |  (0.512, 0.960, -50, 0.1) | FAIL ik | [] | - | task 1: stance 'nominal' point 2 (grasp) | FAIL ik |
| 38 |  (1.140, 0.914, -130, 0.2) | FAIL place_angle_window | [0, 10, 11] | none | place window: no place angle window: no hand turn reaches every place stop within t | FAIL place_angle_window |
| 39 |  (1.188, 0.947, -130, 0.15) | FAIL place_angle_window | [10] | none | place window: no place angle window: no hand turn reaches every place stop within t | FAIL place_angle_window |
| 40 |  (1.163, 0.907, -130, 0.2) | FAIL place_angle_window | [10, 11] | none | place window: no place angle window: no hand turn reaches every place stop within t | FAIL place_angle_window |
| 41 |  (1.166, 0.906, -130, 0.2) | FAIL place_angle_window | [10, 11] | none | place window: no place angle window: no hand turn reaches every place stop within t | FAIL place_angle_window |
| 42 |  (1.138, 1.011, -120, 0.1) | FAIL place_angle_window | [10] | none | place window: no place angle window: no hand turn reaches every place stop within t | FAIL place_angle_window |
| 43 |  (1.124, 0.980, -120, 0.15) | FAIL place_angle_window | [10] | none | place window: no place angle window: no hand turn reaches every place stop within t | FAIL place_angle_window |
| 44 |  (1.059, 1.002, -110, 0.15) | FAIL place_angle_window | [10] | none | place window: no place angle window: no hand turn reaches every place stop within t | FAIL place_angle_window |
| 45 |  (1.063, 1.001, -110, 0.15) | FAIL place_angle_window | [10] | none | place window: no place angle window: no hand turn reaches every place stop within t | FAIL place_angle_window |
| 46 |  (0.996, 1.049, -100, 0.1) | FAIL place_angle_window | [10, 11] | -70..-65 (2 turns) | place window: no place angle window: the widest window of working hand turns is 5 d | FAIL place_angle_window |
| 47 |  (1.070, 1.000, -110, 0.15) | FAIL place_angle_window | [10, 11] | none | place window: no place angle window: no hand turn reaches every place stop within t | FAIL place_angle_window |
| 48 |  (1.082, 1.033, -110, 0.1) | FAIL place_angle_window | [10] | none | place window: no place angle window: no hand turn reaches every place stop within t | FAIL place_angle_window |
| 49 |  (1.008, 1.047, -100, 0.1) | FAIL place_angle_window | [10] | none | place window: no place angle window: no hand turn reaches every place stop within t | FAIL place_angle_window |
| 50 | staging (0.775, 1.105, -90, 0.12) | PASS | [0, 11] | -55..-5 (11 turns) |  | PASS |
| 51 | point 4 (0.870, 1.072, -100, 0.1) | PASS | [11] | -60..-10 (11 turns) |  | PASS |

Notes on the table: the place column is the first passing turn in the place list (the executed angle is the first of the list); the list is in the window order the search returns, which runs from -55 to -5 for the staging stance (its middle is -30). The HEAD column is the same reset run.

## 6. Tests and the earlier results
| test | earlier (last result) | now | note |
|---|---|---|---|
| approach_angle_list_test (new, analytic) | - | 14/14 | new |
| full_check_test | 31/31 | 31/31 | exit 0 |
| scene_heights_test | 12/12 | 12/12 | exit 0 |
| transit_clearance_test | 26/27 | 26/27 | the same failing check: "the chosen spot's own carry is clear and it is one of the ranked spots of the baseline" |
| stance_select_test | crash at line 117 (IndexError) | crash at line 123 (IndexError) | the same statement ("the two stances are far apart", rep["runs"][1]); the line moved because the test's fake grew by six lines; pre-existing (also on HEAD) |
| stance_select_moved_test | 5/9 (CLOSE timeout during the weigh pause) | 5/9 | the same failures: outcome, plan finished, beaker on hotplate; the replay check also fails |
| mpvi_pipeline_test | 10/14 (place HOME fails, 0.267 rad) | 6/14 | REGRESSION: the place now fails earlier at DOWN (7.2 mm, 2.1 deg, 9 s) after the place angle changed (see Decisions, the first-passing place turn) |
| mock 200 g pick and place (scratch/mock_probe.py) | beaker lost at the place OVER (fingers to 43.9 mm at 88 s) | beaker stays in hand through the place OVER | the place OVER then fails at 101 s (3.4 mm, 0.2 deg); not released |

## 7. Remaining "bin" identifiers (kept, not renamed)
In src: pick_bin_index (the function name; its docstring says it is kept), approach_bin (full check parameter and _solve_stance argument), approach_bins (BENCH_STANCE_CFG and the overlay), n_bins, bin_half_deg (GRASP_MAP_CFG), bin_center (src/grasp_rules.py), _bin_order (full check), tried_bins (IK rows), bins_not_hard and ok_bins (stance_passes), bin_world_angle (stance_passes), the "bin" key of IK rows and the bins loop in full_check.check.
Not renamed, as instructed.

## 8. Commits (local, nothing pushed)
See the commit list in the final message: the mock fix, the tidy, the per-task code, and the docs and this report.

## Not done (with the reason)
- The mpvi_pipeline_test regression (6/14): not fixed. The cause is the place angle now being the first passing turn (-55 for the staging stance, the edge of its window), where the earlier rule used the window's middle (-30). The instruction is "first passing angle for now", so it stays; the options are under Decisions.
- The shared random generator: the check's result depends on the order of earlier checks (two stances flip). Not changed; it is a design choice (reseed per stance or not), listed under Decisions.
- combine_task_lists is not called by full_check.check: the check's own loops apply the same rule (every task needs a passing angle). Wiring it in would be a refactor; left as the tested statement of the rule.
- The analytic test covers the stance rule as a function (combine_task_lists) and the choice, the reasons and the place direction. It does not run the full check on a two-pick stance (that needs the simulator's arm model). The check's per-task loop is exercised by the 51-stance runs and by full_check_test (31/31).
- "A failed skill does not run" with an empty list: the gate is in start() (the step fails before planning); not exercised by a simulator run here. Only the reason function is tested analytically.
- The mock's place OVER stop at 3.4 mm / 0.2 deg at 101 s: not investigated (the reason is outside this task's scope: the mock's own tracking tolerance or the carried beaker's load).
- The staging check/execution pedestal mismatch (the check is at the candidate's pedestal, execution parks at 0.16 m): not changed (it is the existing behaviour of the pipeline; recorded as Decisions).
- Execution timing: each pick task now tries all 12 angles (no early stop), so checks take longer; the 51-stance run took about 17 minutes in both runs. Not optimised.

## Decisions for David
1. The place angle choice. Options: (a) first passing turn for places too (as in this change; the pipeline goes 10/14 -> 6/14 because the staging stance's first passing turn is -55, the window's edge, while its middle is -30); (b) the middle of the widest window (the old rule) as the choice for places, with picks staying first-passing; (c) the first passing turn counted from the middle of the window outward. The one function choose_approach_angle is where this goes.
2. The random generator. Options: (a) reseed per check call (results do not depend on the order of stances; the 51-stance result is then 21, which equals HEAD); (b) keep the shared generator (the result depends on the order; 23 in the batch run, 21 when run alone).
3. The place's starting posture (step 2): the place's OVER height and the held-beaker offset come from the pick's end. Options: (a) keep (as now; the check does not test that start); (b) plan the place OVER at the carry height and the offset from the stance model (what the check tests); (c) return to the zombie pose between tasks.
4. The staging pedestal: the check runs at the candidate's pedestal, the staging parks at 0.16 m. Options: (a) keep; (b) check at 0.16 m (the pedestal the staging uses); (c) park at the candidate's pedestal.
5. Whether a stance's per-task check should keep the full 12-angle search for every task (as now, slower) or stop at the first passing angle for execution (faster, but then the list is not complete).

## Files
- Code: src/approach_angle.py (new); src/full_check.py (helpers at lines 203-220 region, the per-task block at lines 633-682); src/stance_select.py (lines 221, 503, 511, 586, 600, 630-636); src/mpvi_modules.py (imports line 50; pick_bin_index 213-216; plan_place 262-268; start 286-292; recover 313-345); src/mock_policy.py (reset, begin, make_mock_manipulation); src/mpvi_pipeline.py (112-113); experiments/stance_select_test.py, experiments/place_theta_logic_test.py, experiments/approach_angle_list_test.py (new), experiments/run_tests.sh (the new test); experiments/bounce_freeze_run.py, experiments/bounce_handoff_metrics.py (tidy).
- Data and logs: scratch/per_task_51.json (batch), scratch/per_task_51_reseed.json (reset), scratch/head85/head_51_reseed.log (HEAD, reset); data/logs/pt/ (test logs and the mock run); data/logs/per_task_51_20261109.log, data/logs/per_task_51_reseed_20261109.log.
- Docs: docs/mpvi_decisions.md (M279, M280), docs/design_plan.md section 3.3, this report.

## Commit state
Local commits only; nothing pushed.
