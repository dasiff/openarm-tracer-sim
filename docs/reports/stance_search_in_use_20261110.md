# Which stance search the pipeline runs, and where the 0.12 m pedestal comes from (2026-11-10)

Written for: David, before designing continuous stance values. Read-only: no src/ or experiments/ change. Reference commit: `b62488a` (HEAD; the code is the same as `8166ee8` for src/).

Summary: the pipeline does not use the Gaussian (analytic) inverse. It runs the reach-map overlay in `src/bench_stance.py`, ranked in `src/stance_select.py`, and checked by `src/full_check.py`. The 0.12 m height is the middle of the widest run of valid pedestal heights on a 1 cm grid, so it is on the grid. The brief's "5 steps" applies only to the analytic search, which the pipeline does not call.

## 1. Call path (pipeline, live `src/`)

Entry: `experiments/demo_video.py` `record_only` (line 96) calls `make_pipeline_task`. The staging-layout run used `scratch/e2e_pedestal/run_staging.py`, which only points `demo_video.MOVED` at `TASK_CFG`.

| # | function | file:line | what it does |
|---|---|---|---|
| 1 | `make_pipeline_task` | src/mpvi_pipeline.py:30 | packages the task; calls the selection |
| 2 | `apply_stance_selection` | src/stance_select.py:672 (called at src/mpvi_pipeline.py:67) | builds the maps (`bs.load_maps`), `FullCheck`, and `make_context` (once) |
| 3 | `make_context` / `StanceContext.__init__` | src/stance_select.py:115 / :51 | heading `yaw = task_cfg["staging_yaw"]` (line 74); `cams = bs.camera_poses(model, bs.pedestal_commands(cfg), …)` (line 75) |
| 4 | `select_bench_stance` | src/stance_select.py:436 | runs over the plan's manipulation steps; for each run calls the overlay |
| 5 | `step_target` | src/stance_select.py:139 | each step's surface and target heights (from `src/support.py`) |
| 6 | `_overlay` → `bs.overlay` | src/stance_select.py:477 (run search), :545 (run with transit filter), :655-659 (wrapper) → src/bench_stance.py:669 | ranks (x, y, pedestal) on the reach maps; returns `ranked` (all thinned stances) and `run` (the valid pedestal range) |
| 7 | `_best_stance` | src/stance_select.py:662, called at :521 | when a run of k steps is not covered, tries k-1 steps (fallback) |
| 8 | candidate list | src/stance_select.py:577 | `cands = ranked[:rank_keep][:1 + full_check_fallbacks]` |
| 9 | `full_check.check` | src/stance_select.py:585 → src/full_check.py:648 | the first candidate that passes is chosen; the next ones are tried only if it fails |
| 10 | `out["task_cfg"]` | src/stance_select.py:623 | `staging_xy`, `staging_yaw`, `pedestal_q = float(st["q"])` (M284) |

Switching settings (current values, from `src/task_config.py` BENCH_STANCE_CFG unless noted):

| setting | value | where read | effect |
|---|---|---|---|
| `stance_check_mode` | `"current"` (from env `OPENARM_STANCE_CHECK_MODE`, default "current"), task_config.py:616 | src/stance_select.py:564 and :595 | "two_pass" adds a cheap and a slow pass (`_two_pass`, :358) before the full check. Not active. |
| `navigation` | True (task_config.py:597) | src/stance_select.py:460-465 | False keeps the robot at TASK_CFG's stance instead of the selected one |
| `rank_keep` | 4 (task_config.py:640) | stance_select.py:577 | number of ranked stances the full check may see |
| `full_check_fallbacks` | 3 (task_config.py:599) | stance_select.py:577 | fallbacks after the best |
| `max_run_steps` | 4 (task_config.py:598) | stance_select.py:470-472 | longest run of steps served from one stance |
| `two_pass_full_check_cap` | 8 (task_config.py:619) | stance_select.py:571 | only in two_pass |

## 2. Where the 0.12 m comes from (this run)

Trace: `scratch/stance_search_trace/select_only.py` (run alone; no episode, no arm physics), log `data/logs/stance_trace_20261110_020246.log`, output `data/logs/stance_trace_20261110_020246.json`. Stance selection result: ok, run steps [1, 3] (pick the beaker, place on the hotplate).

Rule that produces 0.12:
1. The pedestal candidates are `bs.pedestal_commands(cfg)` = `np.arange(0.05, 0.25 + 1e-9, 0.01)`, rounded to 6 decimals (src/bench_stance.py:110-113; config `pedestal_range` (0.05, 0.25), `pedestal_step` 0.01, task_config.py:608-609). That is 21 values: 0.05, 0.06, …, 0.25.
2. In `overlay` (src/bench_stance.py:669), for each (x, y) cell the valid pedestal heights are those where the most objects are covered. `widest_run` finds the widest run of consecutive valid heights and `qmid = (q_lo + q_hi) // 2` (src/bench_stance.py ~line 712-714: "its middle is the spot's height").
3. Trace: the chosen spot's run is `overlay_run_q = [0.05, 0.20]` (indices 0 to 15, 16 values). `qmid = (0 + 15) // 2 = 7`, and the 8th value is **0.12**.
4. The overlay returns `q = 0.12`; the stance is `ranked[0]` (score [2, 0.16, 2, 0.04, 0.356]).

So 0.12 is on the 1 cm grid. It is the middle of the 0.05–0.20 run of heights that cover both objects. It is not a 5-step value.

Other values of this run's stance:
- **x, y = (0.775, 1.105)**: the 1 cm grid of the overlay, `xs = arange(lo, hi, 0.01) + 0.005`, where `lo = min(object x) − 0.95 m` (src/bench_stance.py ~line 700; `reach_m = 0.95`). The cell is chosen by the ranking (widest run, minimum count, edge distance, object distance; lines 716-735). The origin of the grid depends on the object positions.
- **heading = −90°**: fixed. `ctx.yaw = task_cfg["staging_yaw"]` (src/stance_select.py:74). Nothing searches heading in the pipeline.
- **pick approach angle** (for step 1, the beaker): the full check's list of passing pick bins is [0, 11]; `choose_approach_angle` takes the first, bin **0** (0°) (src/approach_angle.py:45-56; the list comes from src/full_check.py:754, `range(n_bins)`).
- **place approach angle** (for step 3, the hotplate): the list is −55° to −5° in 5° steps (`place_turn_step_deg` 5, task_config.py:529); the first passing is **−55°** (`place_theta_deg` = −55). The window's middle (−30°) is not used; the choice is "first passing" (M279).

## 3. Which values are continuous, and which are discrete (search in use)

| value | continuous or discrete | list / range / step | set in |
|---|---|---|---|
| x | discrete | 1 cm cells, offset by 0.5 cm; x range = object x ± 0.95 m (the overlay grid), region built over forward 0.10–0.90 m | BENCH_STANCE_CFG `grid_m` 0.01, `forward_range`, `lateral_range` (task_config.py:610-612); src/bench_stance.py:116-119, :698-700 |
| y | discrete | 1 cm cells as x; lateral −0.60 to +0.60 m in the bench frame | same |
| heading | fixed (no search) | −90° = `TASK_CFG["staging_yaw"]` | src/stance_select.py:74 |
| pedestal | discrete | 0.05 to 0.25 m, 0.01 m step (21 values) | BENCH_STANCE_CFG `pedestal_range`, `pedestal_step` (task_config.py:608-609); src/bench_stance.py:110 |
| pick approach angle | discrete | 12 bins, centre k × 30° from the torso's forward axis (k = 0..11); the full check tests all 12, the chosen is the first passing | `n_bins` 12, `bin_half_deg` 15 (task_config.py:195-196); src/full_check.py:754 |
| place approach angle | discrete | every 5° (0 to 355°, circle); chosen = first passing (of the widest window) | `place_turn_step_deg` 5, `place_turn_min_window_deg` 10 (task_config.py:529-530) |

Note: the full check also tests heights q0 + {0, +0.02, −0.02} m (`height_offsets`, task_config.py:513; src/full_check.py:615 and :690), and the error stances (x ±1 cm, y ±1 cm, heading ±1°, pedestal ±2 cm; full_check.py:9, `_error_stances` :533). So the robot parks at 0.12 m, but the check also tests 0.10 and 0.14 m.

## 4. Every stance search implementation in the repo

| file | entry function (line) | what it does | who calls it |
|---|---|---|---|
| src/stance_select.py | `apply_stance_selection` (672) → `select_bench_stance` (436) | the pipeline's stance selection: runs of steps, reach-map overlay ranking, full check of the top candidates | src/mpvi_pipeline.py:67 (the pipeline); experiments/stance_select_test.py, stance_select_pipeline_test.py, stance_select_moved_test.py, place_theta_logic_test.py, demo_video.py (through the pipeline) |
| src/bench_stance.py | `overlay` (669), `get_region` (239), `build_region` (163) | reach-map overlay on the (x, y, pedestal) grid: counts approach bins per cell and height, ranks cells | src/stance_select.py (via `_overlay` 655, lines 477, 545); experiments/bench_stance_test.py, bench_stance_validate.py |
| src/stance_select.py | `_two_pass` (358) | cheap IK pass and slow hand-path pass, a filter before the full check | only when `stance_check_mode` = "two_pass" (src/stance_select.py:569); not active |
| src/stance_select.py | `_best_stance` (662) | fallback: the best stance for k−1 steps when k steps are not covered | src/stance_select.py:521 (pipeline, when a run is shorter) |
| src/stance_finder.py | `find_stances` (286), `find_stances_windowed` (368) | older furniture-map finder with a heading search (B2.9) | experiments/full_check_test.py:237, :241; experiments/stance_selection_timing.py:89; experiments/stance_from_map.py:71, :185 (not the pipeline) |
| src/reach_model.py | `inverse_search` (246) | analytic (Gaussian A8) inverse: 36 headings × 5 pedestals (0.05, 0.10, 0.15, 0.20, 0.25) × 50 segment points; calls `single_object_stance` (line 274) | experiments/reach_inverse_scene.py:59 only (not the pipeline) |
| src/reach_model.py | `inverse_stances` (153), `_scan` (201) | older version of the same search | no callers (dead code) |
| src/reach_model.py | `single_object_stance` (219), `ideal_spot` (132) | one-object analytic spot | `inverse_search` (274); experiments/reach_inverse_scene.py:ideal_spot |
| src/support.py | `stance_method` (132) | chooses a stance method per surface for navigation phrases; not a search | src/stance_select.py:164 |
| src/world_model.py | `_stance` (39) | returns the staging stance from task_cfg; not a search | src/world_model.py (lookup) |

## 5. The Gaussian inverse and the 51 test stances

- The Gaussian (A8) analytic inverse is **not** in the pipeline path. It is called only by `experiments/reach_inverse_scene.py` (line 59). Nothing in src/ outside `src/reach_model.py` imports it (`docs/reports/per_task_angle_lists_20261109.md`, Part 0, says the same, and the check in this task confirms it).
- The 51 test stances (overnight, Nov 8-9) are built by `/home/vboxuser/projects/oc_scripts/ov_lib.py` (function near line 34): **49 rows** from `docs/reports/reach_inverse_spread_candidates.xlsx`, sheet "candidates standard", plus the staging stance and "point 4" (51 total). The reports say the 49 rows are from the **analytic inverse** ("sweep, fixed heading, 0% occlusion", `docs/reports/failed_runs_study_20261109.md:244`, Nov 8). That is the family of `src/reach_model.py`, not the family the pipeline runs.
- **So the 51 test stances and the pipeline's stance do not come from the same search.** The pipeline uses the reach-map overlay (`bench_stance.overlay`) and the test stances come from the analytic inverse spreadsheet.
- Gap: the script that wrote `reach_inverse_spread_candidates.xlsx` is not in the repo (grep for `Workbook`/`save` finds no writer for it). Its exact source is therefore not confirmed here.

## 6. Does the full check get the search's values unchanged?

Yes, for x, y and pedestal. In `select_bench_stance` the check is called with `(cand_st["xy"][0], cand_st["xy"][1], yaw, cand_st["q"])` (src/stance_select.py:585). The trace shows the check's stance as the same floats as the chosen stance: xy (0.7750000000000009, 1.1050000000000013), q 0.12 (`full_check` entry in the trace JSON). Inside the check the pedestal is used with offsets (section 3), which is the check's own design, not a substitution. The only later change is `float()` in `out["task_cfg"]` (stance_select.py:623).

## 7. Latest completed episode and latest release on the hotplate

Judged from the episode logs (`episode_summary.json` and `episode_log.csv`):
- **Completed (whole episode)**: the outcome is SUCCESS, the plan is finished, and the phase log runs `4/4 PLACE: OPEN` → `BACK` → `HOME` → `PLAN DONE`. Also `gripper_opened` verdict passed.
  - Log: `data/moved_x_20261105/run4_beaker_plus04/episode/episode_summary.json` (and `episode_log.csv`)
  - Date: 2026-11-05 13:29 (file time)
  - Commit: `444cc4fff4fb9a7f6e5d30867c9b1fd277dc5edd`, not dirty (`recording/meta.json`)
- **Released on the hotplate, then stalled (latest)**: a `gripper_opened` verdict passed in the place segment (time 88.6 s, segment index 2), and the final beaker is inside the hotplate footprint (`a_xy_in_footprint` ok), but the plan failed later at HOME.
  - Log: `data/logs/overnight_20261109/A_pipe_oc_a3_episode/episode_summary.json`
  - Date: 2026-11-09 05:01 (file time)
  - Commit: a copy of `3e93f73` with variant 3 of `choose_approach_angle` (`/home/vboxuser/projects/oc_a3`, not a git repository; `docs/reports/overnight_measurements_20261109.md` setup section). Fail: "HOME: the arm did not settle at home after 9.0 s".
- The newest run (today, `data/e2e_staging_20261110_002222`) did **not** release: it failed at DOWN before the gripper opened.

## 8. NOT DONE

- The overnight A_pipe copies (oc_a2, oc_a3) are not git commits; their code is recorded only as a description.
- The generator of `docs/reports/reach_inverse_spread_candidates.xlsx` is not in the repo (section 5).
- `docs/reports/per_task_angle_lists_20261109.md` Part 0 still says the staging parks at `cfg["pedestal_override_m"] = 0.16 m`. That is out of date after M284 (commits ccea3ca, 8166ee8). Not edited (another document; out of scope).
- The trace runs the selection, not the episode. Whether the robot stays at 0.12 m through the run was checked in the earlier end-to-end run (`data/logs/e2e_staging_20261110_002222.log`), not here.
- No commit of the scratch scripts (`scratch/stance_search_trace/`, `scratch/e2e_pedestal/`); scratch/ is untracked by design.

## 9. DECISIONS NEEDED

1. Continuous stance values: the current search is grid-based at x, y (1 cm), pedestal (1 cm), and pick and place angles (30° bins, 5° turns). Which of these should become continuous first, and should the overlay's 1 cm grid stay for the ranking while the final values are refined?
2. The 51 test stances come from the analytic-inverse family, the pipeline from the reach-map overlay. Should the 51-stance check be re-run on the pipeline's stances (or on the analytic family for both), so the two are comparable?
3. Place angle: keep "first passing" (−55° here, the window's edge) or use the window's middle (−30°)? Currently the choice is first passing (M279).
