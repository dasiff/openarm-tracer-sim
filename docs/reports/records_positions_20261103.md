# Records update: positions, scope, and the open item on moving weight and trim logic (2026-11-03)

Written for: David. Plain language. Documentation only: no code, config, setting or number was changed, and no simulation was run. Left uncommitted on purpose.

## Step 1 result in plain words

- **Repo state at the start:** no tracked changes. The latest commit was `d7e7691`, as expected. The only untracked files were the older ones from earlier tasks (`deps/AgileX_Tracer`, `.claude/`, `compare_nosag.py`, `extract_height_data.py`, `measstart`) and the three reports from the earlier close-out.
- **Line references in `docs/reports/lookups_20261103.md`:** I checked about 60 unique file-and-line references against the files as they are now (the cited text is compared with the text at that line). **All of them match.** The apparent mismatches in an automatic first pass came from my own extraction pairing a number with the wrong file (for example a line in `src/control_loop.py` next to a sentence about `src/mpvi_modules.py`). Those are not errors in the report.
- **Two wording points, not wrong references:**
  - `src/mpvi_modules.py` lines 476 to 488 are cited for the dead handoff reading. The branch actually starts at line 471 (`elif kind == "held":`); 476 is the first line inside it. Both numbers point at the same dead branch.
  - `src/mpvi_modules.py` lines 203 to 246 are cited for the plan functions. `plan_pick` starts at line 201 and `plan_place` at 225; the range is fine.
- **The earlier corrections** (the ones Claude Code flagged last session) are in the report as corrected: task-points plate and bench lines (`src/task_points.py` 44–46, not 43–45; bench top at 46); `z_clear` at `src/mpvi_modules.py` line 211 (not 212); the one-number fit's return at `src/weight_estimate.py` line 113 (not 94–100); and `src/world_model.py` line 3, which is a docstring example and not a live call.
- **Did any source file change on disk?** No, as far as the records show. `src/task_points.py` was last modified 2026-11-02 12:06:51 and `src/mpvi_modules.py` 2026-11-02 21:16:07. Both are older than this session's commits (`75cea83`, `5c42375`, `d7e7691` at 2026-11-03 18:01 to 18:03, and the later commits). `git diff HEAD -- src/task_points.py src/mpvi_modules.py` is empty, so both files are identical to HEAD. The file that did change on disk is `docs/reports/lookups_20261103.md` itself: it was last modified at 18:12:37, by my own line-number corrections made after the earlier notice. I did not revert anything.
- **Scope of the hotplate check (step 4):** I checked each statement against the files. All four are correct. Details are in the next section.

## Step 4: hotplate check, and where the correction went

Searched `docs/` for `hotplate_xy`, "0.80" with "0.65", and "stale".

- **Found in the decision log:** `docs/mpvi_decisions.md` row M078 (2026-10-22) records `hotplate_xy = (0.80, 0.66)`. It is consistent with the files. It does not call the value stale. **Correction added** as a dated note beside it (text kept).
- **Found in the design plan and progress log:** no existing note calls the config stale or says the scene differs. Nothing to correct there.
- **Found only in reports (outside the three files I may edit):**
  - `docs/reports/20261102_1926_trim_handoff_part2_run.md` line 104 says "TASK_CFG hotplate_xy is (0.80, 0.66) m … the hotplate's top site in the scene is at x 0.65 m … Either the config value is stale or the scene places the hotplate elsewhere". Line 116 lists "correct the config, or move the hotplate in the scene" as options. Line 126 says the inconsistency "is not one" because the run used the moved layout. So the report contains both the question and its answer.
  - `docs/reports/20261103_0125_liftoff_run1_analysis.md` line 93 says the configured value is 0.80 and the scene uses 0.65 (the moved layout), and that the beaker landed on the scene's site.
  - **Not edited:** no dated correction was added beside these, because the brief limits edits to the three docs files. Listed under "NOT BUILT".

**The four statements, checked against the files:**
1. `models/objects/hotplate.xml` line 13: `<body name="task_hotplate" pos="0.80 0.66 0.96">`. **Correct: 0.80.**
2. `src/task_config.py` line 51: `"hotplate_xy": (0.80, 0.66)`. **Correct: 0.80.**
3. `experiments/demo_video.py` lines 43 and 44: `SHIFT_X = -0.15`, and `MOVED` adds `SHIFT_X` to `hotplate_xy` and `beaker_xy`. **Correct.**
4. The simulator world model and the scripted place plan read the shifted value: `src/world_model.py` line 317 reads the live site `hotplate_top`, and line 325 writes the hotplate entry from it; `src/mpvi_modules.py` lines 228–229 read the place destination from the world model. **Correct.** Nuance: in a moved run the live site is the shifted value, because `apply_task_poses` writes the shifted position into the model at load (`src/task_beaker_hotplate.py` line 85).
5. `src/task_points.py` line 47 reads the unshifted config value: `np.array(TASK_CFG["hotplate_xy"], float)`. **Correct.**

## The added text (quoted from the commits)

### Commit `42530fb`, `docs/mpvi_decisions.md`

**New row M245 (bottom of the table):**

> | M245 | 2026-11-03 | No hard-coded positions. No code may hard-code object positions. The world model gets them from the built MuJoCo scene now and from a visual object detector later. How this is done is not decided. Known cases and open questions are in the M245 detail section below the table. | decides | paper silent | n/a | the scene now; a visual detector later | David (2026-11-03) | src/world_model.py (SimGroundTruthSource line 259; StaticTaskSource line 334); src/task_points.py line 47 (known case, not changed) |

**M245 detail section** (after the M244 detail). Its content:
- The decision as David worded it (quoted above).
- **Known cases, from the look-up report, task-points first:** (1) `src/task_points.py` line 47, the hotplate and beaker positions read from config and not from the scene; (2) `src/task_points.py` line 40, the grasp band fraction from config; (3) `src/task_beaker_hotplate.py` lines 85 and 87, the loader writes configured positions into the model at load; (4) `src/world_model.py` lines 359 to 362 (`StaticTaskSource`), config positions as world entries, used by `src/stance_select.py` line 574; (5) `models/objects/hotplate.xml` line 13 and `models/objects/beaker.xml` line 22, hard-coded default positions in the scene files; (6) `experiments/demo_video.py` line 43 and `experiments/stance_select_moved_test.py` line 38, a hard-coded shift for the moved layout; (7) config positions that are references or task definitions (`src/task_config.py` lines 65, 78, 84, 92–93, 105–107, 162); (8) many experiment scripts that read `TASK_CFG` positions directly.
- **Config-to-scene pairs not compared from the files:** the robot base's world position (config lines 65 and 78; the scene places the base only in its own frame, `models/robot/combined_robot.xml` line 119); the second beaker (config line 162); the joint poses (config lines 92–93 and 105–107); the hotplate's plate size (`models/objects/hotplate.xml` line 16, no config key); the staging yaw (config line 83).
- **Open (not decided):** the interface (how the world model asks the scene, and how a detector plugs in later); how a shifted layout such as the demo video's is handled; the order of work; and which positions stay as task definitions.

### Commit `42530fb`, `docs/mpvi_decisions.md`: the M244 scope note

> **Scope of the bounce results (Nov 3, 2026).** All bounce numbers in this entry (M244) are for the scripted pipeline: the pick and place skills in `src/mpvi_modules.py`. Under a policy-driven arm with no scripted skills, the pieces below would not run, per `docs/reports/lookups_20261103.md` section 3b. Only the command bridge and the arm's own gravity feedforward would run. The held part of the gravity feedforward runs every tick but adds nothing while its mass is zero, which only a scripted weighing sets. The table below is quoted from that report (files and lines as cited there):

The table is the 23-line piece table from `lookups_20261103.md` section 3b, copied as it stands in that report. It covers the sag trim and its freezes, the trim reset at gripper open and at recovery, the skill-start reset (off), the held-mass ramp, the lift-off weighing and detection and estimate, the gravity feedforward (arm part: runs; held part: runs but adds nothing), the command bridge (runs), the routing (runs), the gripper-open wait, the CLOSE and oracle holding checks, the drop monitor, the dead handoff reading, the one-number and three-number weight fits, and the world model's holding check.

### Commit `42530fb`, `docs/mpvi_decisions.md`: the open item

> ## OPEN (not decided), Nov 3, 2026: Moving weight and trim logic below the policy
>
> - David's decision of Nov 2 (M216, row above; design plan item p) put the held-weight term in the shared bottom layer: always on, zero when the hand is empty.
> - On Nov 3 David asked whether to move the held-weight and trim pieces below the policy one at a time.
> - Piece by piece: the table in the scope note of M244 (quoted above from `docs/reports/lookups_20261103.md` section 3b) lists what runs below, what is triggered by scripted skill phases, and the dead code.
> - **First open question:** what tells the lower layer that it is holding something. Today: finger travel within a range (scripted CLOSE check), finger travel with object distance (oracle check), and contact (simulator only). No finger force is used anywhere. Candidate, not decided: finger position together with finger force, with thresholds pending. Also open: when it weighs (today only at the scripted pick's WEIGH step).
> - NOT DECIDED. No code or setting was changed for this item.

### Commit `42530fb`, `docs/mpvi_decisions.md`: the M078 note

Appended to the M078 decision cell (the M078 text is unchanged):

> UPDATE 2026-11-03 (see M245): the hotplate_xy value (0.80, 0.66) matches the scene file models/objects/hotplate.xml line 13 and the config src/task_config.py line 51. The 0.65 seen in the moved demo layout is a run-time shift (experiments/demo_video.py lines 43-44, SHIFT_X = -0.15), not a stale config value. The simulator world model and the scripted place plan read the shifted value; src/task_points.py line 47 reads the unshifted config value.

I checked by script that the M078 text, apart from this note, is identical to before.

### Commit `42530fb`, `docs/progress.md`

One entry at the bottom, in the existing format, beginning "TASK 2026-11-03 (records update: positions and scope; documentation only, no code, no simulation, no push)". It records the step 1 result, M245, the M244 scope note, the open item, the M078 note, the reports' commit, and that the design plan was not edited.

### Design plan: not edited

`docs/design_plan.md` has no open-items list that fits this item. Section 11 is "Open questions for the hardware team", a different list. So the open item went into the decision log, as the brief allowed, and I did not touch the design plan.

## Commit hashes

- **Commit 1 (reports):** `382d1c3`, "Save the Nov 3 look-up and close-out reports": `docs/reports/lookups_20261103.md`, `docs/reports/bounce_closeout_20261103.md`, `docs/reports/bounce_closeout_edit_20261103.md`.
- **Commit 2 (docs):** `42530fb`, "Records: decision M245 (no hard-coded positions), M244 scope note, open item on moving weight and trim logic, M078 note": `docs/mpvi_decisions.md` and `docs/progress.md`.

**Nothing was pushed.** The branch has no upstream configured. The commits are local.

## Secret search

- **Files searched:** the three reports committed in `382d1c3`, and the lines added to `docs/mpvi_decisions.md` and `docs/progress.md` in `42530fb`.
- **Keywords** (`api key`, `secret`, `password`, `passwd`, `token`, `bearer`, `private key`, `key=`, Google `AIza`, `sk-`, AWS `AKIA`, GitHub `ghp_`, Slack `xox`): the only hits are in `docs/reports/bounce_closeout_20261103.md`, where the words describe the earlier search, and in the progress entry, where they describe this search. No secret value.
- **E-mail addresses:** none.
- **Long random strings** (24 or more characters, mixed case and digits): none.
- **Sizes:** all three reports are under 1 MB (largest `docs/reports/lookups_20261103.md`, about 43 KB).
- **Left out:** nothing.

## NOT BUILT, needs decision

1. **Dated corrections in two reports.** `docs/reports/20261102_1926_trim_handoff_part2_run.md` (lines 104, 116 and 126) and `docs/reports/20261103_0125_liftoff_run1_analysis.md` (line 93) describe the 0.80 and 0.65 difference. A dated note beside each was not added, because reports are outside the three files I may edit. Line 116 of the first lists "correct the config" as an option; the repo's own files show the config is right.
2. **The interface** for positions: how the world model asks the scene, and how a visual detector plugs in later. Open in M245.
3. **How a shifted layout** (the demo video's, `experiments/demo_video.py` line 43) is handled. Today the shift is applied to the config at run time, and `src/task_points.py` line 47 does not follow it. Open in M245. Not decided here.
4. **The order of work** for moving positions to the scene and detector. Open in M245.
5. **Which positions stay as task definitions** (joint poses, reference stance, second beaker, spawn). Open in M245.
6. **The five config-to-scene pairs** that could not be compared from the files (listed in M245). Comparing them needs the model loaded or an IK check, which was not done.
7. **The moving of the held-weight and trim pieces below the policy**, one at a time, and the holding signal. Open item in the decision log, not decided.
8. **Design plan:** no open-items list there, so nothing was added. If you want the open item in the design plan too, that is a decision for you.
