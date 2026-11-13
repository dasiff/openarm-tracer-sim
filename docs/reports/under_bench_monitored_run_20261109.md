# Monitored run at an under-the-bench stance (2026-11-09)

Untracked. Read-only with respect to the real repo: nothing in src/, config, tests or docs was changed, no commits, no push. Work was done in a copy built from HEAD `9ff0099`: `/home/vboxuser/projects/oc_elbow`. The copy got the reach maps (`data/reach_map`, `data/reach_map_before_bench`), the overnight candidate file (`data/logs/overnight_20261109/B_given_noreseed.jsonl`), the mesh submodule `deps/OpenArm-Combined` (not in the git archive), and a symlink to the real `.venv` (read only). Helper: `/home/vboxuser/projects/oc_scripts/under_bench_candidates_20261109.py`.

## 1. Result: no stance passed; the monitored run was not made

The task says to stop if no candidate passes both the body group at pedestal 0.16 m and the pick ik group at its nominal stance. None did.

Candidates: the three smallest edge distances (base_y - 0.80 m) in the committed analytic-inverse files `docs/reports/reach_inverse_standard.json` and `_moved.json`. The standard and moved files list the same base y values at heading 90 deg. Each candidate was checked at pedestal 0.16 m, then at its own pedestal (0.15 m), in one FullCheck setup.

| candidate (source) | x, y (m) | heading (deg) | pedestal (m) | edge (m) | saved first failure | body at 0.16: ok, min distance (m) | ik at nominal: ok, first failing point | body at own 0.15: ok, min distance (m) | ik at own 0.15 |
|---|---|---|---|---|---|---|---|---|---|
| standard | 1.0269, 0.3139 | 90 | 0.15 | -0.486 | placed_beaker | fail, -0.355 | fail, point 1 (standoff, bin 10 furthest) | fail, -0.365 | fail, same |
| moved | 0.8769, 0.3139 | 90 | 0.15 | -0.486 | home_clearance | fail, -0.355 | fail, same | fail, -0.365 | fail, same |
| moved | 0.8810, 0.3149 | 90 | 0.15 | -0.485 | home_clearance | fail, -0.355 | fail, same | fail, -0.365 | fail, same |

Body: at all three the robot's base (`openarm_body_link0_collision`) penetrates the bench top ("touches", min distance negative). Ik: the pick task fails at the nominal stance at point 1 (standoff at [0.896, 0.600, 1.012]) in every approach bin. The two "moved" rows are the same geometry: the body distance depends on base y, and the bench spans far more than 0.15 m in x. (The FullCheck runs in the standard layout for all three, so their results are identical.)

Why these candidates: they are the smallest edge distances in the committed files (the most under the bench top). Excluded by the rule and the check budget: the 0.20 m pedestal candidates at y 0.3632 and 0.3646 (not checked; the budget was three). Not used: the Nov 8 complete-check logs have base y down to 0.249 m (edge -0.551), but they are untracked data in the real repo's `data/logs`, which the rules say not to read. They would be the next candidates to check if that rule is relaxed.

Nothing was driven, placed or run. Step 2 (monitored pick and place), step 3 (phase tags), step 4 (contacts and suspensions), step 5 (guard would-clamp count) and step 6 (guard-on repeat) depend on a passing stance, so they were not done.

## 2. Wall time and memory

| job | wall time | free memory before |
|---|---|---|
| copy setup (archive, data subset, mesh submodule copy) | a few seconds plus the 67 MB submodule copy | 3.9 GB free |
| candidate checks, first run (`under_bench_candidates_20261109.log`) | 229 s total (setup 69.6 s, first body+ik check 143 s including first-use costs) | 3.3 GB free |
| candidate body rerun, with the body distance recorded (`..._b.log`) | 82 s total (setup 64.5 s) | 3.2 GB free |

The first run did not record the body distance (it read the body measurement after the ik call overwrote it). The rerun repeats the same three candidates with that fixed; the rule of three candidates is unchanged. Its logs are `data/under_bench_candidates_20261109*.log` and `.json` in the copy.

## 3. Not done, and what needs a run

- The monitored pick and place at an under-bench stance (needs a stance that passes body at 0.16 m and nominal ik). The stance rule finds none among the committed candidates; a run needs a different candidate set (the complete-check logs, or a new search) and a decision from David on whether base-over-bench stances are the target.
- The per-tick monitor (elbow centre z, x and y against the footprint, link distances to the bench top and hotplate) and the phase table: not built, since no run is made.
- Guard-on repeat: not made.
- Not found in the saved data: any stance with the base over the bench footprint that passes the body group at 0.16 m. None of the three checked does.

## 4. Real repo state

HEAD `9ff0099`. No commits, no pushes in this task. This report is the only file created in the real repo, and it is untracked.
