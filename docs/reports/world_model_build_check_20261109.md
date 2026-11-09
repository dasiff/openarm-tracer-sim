# world-model-build branch against HEAD: is anything not already in the code? (2026-11-09)

Read only. No change, commit, merge, rebase, checkout or delete in the working copy or on origin. Git commands used: git cherry, git log, git diff, git show, git grep, git ls-tree (read only). No temporary clone was needed.

HEAD: 51e99a7 (branch held-weight-ramp-sag-off, which holds our uncommitted work). world-model-build: local branch, merge base c71d633 (75 commits behind HEAD, 4 ahead).

## 1. Commits on world-model-build not in HEAD (git cherry -v HEAD world-model-build)

git cherry marks all four with "+" (no equivalent patch in HEAD). That is not enough on its own: the same work went into HEAD later with changes, so the table below also checks each file's content (section 3).

| commit | date | message | status vs HEAD |
|---|---|---|---|
| 4127a0d | 2026-11-05 | Stage 1 of the world-model build: investigation report, no behaviour change | **not in**: adds only docs/reports/world_model_build_stage1_20261105.md, which is absent from HEAD. Its finding (bins 9 and 10 blocked by the approach-direction rule) is in HEAD's stage 3 report, and the rule it names was replaced by M275 (free approach angle) |
| c050fea | 2026-11-05 | Stage 2 of the world-model build: world-model collision checker | **already in** (later version): src/world_collision.py is in HEAD and HEAD has 31 more lines (bench_boxes, M273); the stage 2 report is in HEAD; the decision row is in HEAD as M271 (the branch numbers it M269) |
| e7995ad | 2026-11-05 | Stage 3 of the world-model build: cheap and slow passes (not wired) | **already in** (later version): src/stance_passes.py is byte-identical to the branch; HEAD wires it behind stance_check_mode "two_pass" (stance_select.py 361–436, 567, 598; M274), the setting is in task_config.py:607, the stage 3 report is in HEAD, and the decision row is HEAD's M272 (the branch numbers it M270) |
| c7c1fab | 2026-11-05 | WIP: elbow guard and pedestal from the bench top (not identical; not pushed) | **partly in**. The guard minimum is in HEAD as M269 with the same value (1.1048 m = table 0.960 + 0.1448, which is the branch's own sum). **Not in**: pedestal_for_scene (branch src/elbow_guard.py:73, minimum pedestal 0.1496 m from the model), the branch's shoulder-pitch clamp in src/control_loop.py (+16 −29), and the pedestal hook in src/stance_select.py (+4 lines: import, ped/ped_reason call, rep["pedestal_reason"]). HEAD instead clamps every arm target (M269, src/elbow_guard.py clamp) and sets the staging pedestal to a fixed override of 0.16 m (M270; stance_select.py:623) |

## 2. For each "partly in" or "not in" commit: what it changes
- 4127a0d: one report file (71 lines). The investigation: why bins 9 and 10 fail the verification IK at the demo stance (the approach-direction rule). Files: docs/reports/world_model_build_stage1_20261105.md.
- c7c1fab (WIP, not decided): (a) src/elbow_guard.py computes the bench top, the guard (table + 0.1448 m) and a minimum pedestal (0.1496 m) from the model; (b) src/control_loop.py clamps the shoulder pitch when the elbow joint centre would be too low (HEAD clamps all arm targets instead, M269); (c) src/stance_select.py takes the staging pedestal from pedestal_for_scene, with an override (HEAD: fixed 0.16 m, M270). Files: src/elbow_guard.py, src/control_loop.py, src/stance_select.py, docs/progress.md.

## 3. Feature check in HEAD (current code)

| feature | in HEAD? | where (file:line) |
|---|---|---|
| world-model-only collision checker (boxes from the world model, the robot's hand/finger/forearm points, signed distance per box) | **yes** | src/world_collision.py: boxes_from_world:26, robot_group_points:96, bench_boxes:114, check_boxes:145 |
| stance_check_mode, value "current" | **yes**, default | src/task_config.py:607 (OPENARM_STANCE_CHECK_MODE, default "current"); src/stance_select.py:567 (the switch) |
| stance_check_mode, value "two_pass" | **yes** | src/stance_select.py:567–569 (stops if no full-check object), :598 (runs the two passes), :361 _two_pass, :406 failure log |
| cheap pass: nominal IK | **yes** | src/stance_select.py:361–402: fc._solve_stance on every run point (the full check's own solver, any bin) |
| cheap pass: reach-map score | **no** (the cheap pass uses nominal IK only; the reach-map score is not part of it) | — (no "score" or "reach" in _two_pass, stance_select.py:361–402) |
| slow pass: hand, finger and forearm clearance | **yes** | src/stance_passes.py: pose_clearance:61, evaluate_cell:69, classify:105, robot_group_points in src/world_collision.py:96; called from stance_select.py:376–400 |
| bench as a collision body, in the world-model checker | **yes** | src/world_collision.py:114 bench_boxes; src/stance_select.py:376 (boxes_from_world + bench_boxes(model, bench_geoms(model))) |
| bench as a collision body, in the full check's ArmCollisionChecker | **yes** | src/arm_collision.py:24 bench_geoms, :49 bench argument of __init__, :75–78 bench geoms lowered by the pedestal; src/full_check.py:35, :317–318 (passes the bench) |
| decision rows M271 onward | **yes**: M271 (world margin, provisional), M272 (stance_check_mode, pending), M273 (bench as collision body), M274 (two_pass wiring), then M275–M278 (ours) | docs/mpvi_decisions.md (rows M271–M278) |

Note: the branch's own decision rows are numbered M269 (world margin) and M270 (stance_check_mode, "world_box"). In HEAD those numbers are the elbow guard (M269) and the pedestal (M270). If the branch's rows were copied in, the numbers would clash.

## 4. src/stance_select.py: world-model-build against HEAD, in plain words
- HEAD is the newer side. HEAD has what the branch lacks: the two-pass selection (cheap nominal-IK pass, slow hand/finger/forearm pass, a failure log for when no candidate passes; stance_select.py 361–436), the switch in the selection (567–598), the bench in the world boxes (376), and the cap of the candidate list at rank_keep before the full check (580). The branch's file is the 2026-11-05 version: none of that is wired.
- The branch has one thing HEAD does not: the pedestal hook (+4 lines: the staging pedestal from pedestal_for_scene, with its reason written to the report). HEAD uses the fixed override instead.

## 5. Does world-model-build hold work not in HEAD, and is it still wanted?
Yes, three pieces, none of them wanted as they are:
1. The pedestal from the bench top (pedestal_for_scene, minimum 0.1496 m; the staging hook in stance_select). HEAD's pedestal is the fixed 0.16 m decided in M270, and the Gaussian stance search (7387c0d) and our approach-angle work choose the stance and pedestal another way. Not wanted now unless David wants a model-based pedestal minimum.
2. The shoulder-pitch clamp in control_loop.py. Superseded by the full clamp (M269), which is now switched off (M278). Not wanted.
3. The stage 1 investigation report. It documents a blocker that M275 removed (the approach-direction rule). Keep as history only.
Do not merge the branch's src/world_collision.py over HEAD: it lacks bench_boxes (HEAD's M273), so a file-level overwrite would remove the bench collision body.

## Not done
- No test runs of the branch: the task is a read-only comparison of code and history. Reason: a run would mean checking out or building the branch.
- Patch-id equivalence was not used as the test (it says "not in" for all four commits; the content check in section 1 is the basis).

## Files
- This report: docs/reports/world_model_build_check_20261109.md

## Commit state
No commits made.
