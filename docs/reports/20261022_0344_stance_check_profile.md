# Profile of one layer 3 stance check (2026-10-22)

Written for: David, who decides from these numbers how to make the stance check faster. Timing and counting only: no code path and no setting was changed, no optimization was tried.

Scripts (committed): `experiments/stance_check_profile.py` (the profile), `experiments/stance_checker_build_time.py` (a small follow-up). Logs and data (not in git): `data/logs/stance_check_profile_run_20261022_032708.log`, `data/logs/stance_check_profile_20261022_034130.json` (every counter), `.../stance_check_profile_20261022_034130.prof` (the cProfile data, open with `pstats`), `data/logs/stance_checker_build_time_20261022_034229.log`.

What was run, with the code of `experiments/stance_selection_timing.py` (the 235 s of the navigation proposal): the scene of the "open_b45" case; for each pedestal height of the candidate's range on a 1 cm grid the checks of `StanceChecks.requirements` (cheap ones first, the slow ones only where those pass), then, if any height passes, the IK verification `Runner.evaluate` at the middle of the passing heights. Four runs: **A cold** and **A warm** (the stance in `TASK_CFG`, 10 heights 0.105 to 0.205 m, all pass), **A under cProfile**, and **B** (a candidate that fails early: the first layer 2 candidate at heading -90 degrees, 12 heights). The wrappers that count and time are removed before the cProfile run. Machine: the 7.4 GB VM, nothing else running.

## 1. Time

| | Cold (first call) | Warm (second call) |
|---|---|---|
| **Total, stance A** | **284.7 s** | **252.4 s** |
| requirements, all 10 heights | 119.3 s | 106.1 s |
| IK verification (one call) | 165.3 s | 146.3 s |
| (under cProfile, with the profiler's overhead) | | 282.5 s |
| **Stance B (fails early), 12 heights** | | **0.54 s** |
| Setup, once per process (the case's scene, `Runner`) | 38.0 s | |

(The 235 s of the proposal was the same check on a quieter moment; the run-to-run spread on this VM is about 15 %.)

Per sub-check, stance A. "Warm %" is of the warm total of 252.4 s.

| Sub-check | Calls | Cold | Warm | Warm % |
|---|---|---|---|---|
| zombie-pose clearance (`home_clearance`) | 10 (one per height) | 0.37 s | 0.34 s | 0.1 % |
| placed-object clearance (`beaker_on_hotplate_clearance`) | 10 | 0.04 s | 0.04 s | 0.0 % |
| neck framing solve and repeatability (`neck_spread`) | 10 (90 solves) | 81.0 s | 92.8 s | 36.8 % |
| occlusion renders (`occlusion`) | 10 (20 renders) | 26.8 s | 1.25 s | 0.5 % |
| framing slack (`framing_slack`, `best_margin`) | 10 | 11.2 s | 11.6 s | 4.6 % |
| **IK path verification** (`Runner.evaluate`) | 1 | 165.3 s | 146.3 s | 58.0 % |
| anything else (the loop, `place()` x 50, bookkeeping) | | about 0 | about 0 | 0 % |

The cold-run surplus: the occlusion renderers and the transparent copy of the model are built on first use (about 25 s: `occlusion._count` takes 26.7 s of which the renders take 3.7 s); and the IK verification's nominal analysis took 17.4 s instead of 2.2 s (see 3 below for the arm collision checker). Stance B: the cheap checks fail at all 12 heights ("home_clearance" and "placed_beaker"), 0.48 s and 0.05 s in all; 10,603 geometry distance calls; the slow checks never run.

## 2. Counts

**Heights.** The requirements are evaluated at every pedestal height of the candidate's valid range on a 1 cm grid (A: 0.105 to 0.205 m, 10 heights; B: 0.07 to 0.18 m, 12 heights). The reason is the passing pedestal range (S-Q9): the longest run of consecutive heights that pass is the passing range, so the whole range is scanned. The cheap checks run at every height; the neck, occlusion and framing only at heights where they pass (for A all 10).

**Neck (per height and in total).** `neck_spread` makes 9 framing solves per height: one for the unperturbed state and 8 with 0.1 degree noise on the arm joints (the first four are the `neck_once_test`'s draws): 90 solves for A, plus one `best_margin` search per height (the framing slack): 10. Time per solve: 1.03 s (92.8 s / 90) warm, 0.90 s cold. Each solve is a global search: a coarse grid of 7 values on each of the 4 neck joints (7^4 = 2,401 cost evaluations, `FRAMING_CFG["coarse_steps"]`), then Nelder-Mead refinements of the best 6 grid points and of the start pose (7 per solve, up to 1,500 evaluations each, `n_refine`, `refine_evals`). Total 420,344 cost evaluations (`_evaluate`), 4,670 per solve on average: 216,090 (51 %) in the grids and 204,254 in the 630 refinements (324 each). One cost evaluation takes 0.175 ms (73.6 s / 420,344). `best_margin` (framing slack): a 9^4 grid (6,561) and 8 refinements of at most 400 evaluations, about 9,260 evaluations per call, 1.16 s per call.

**Renders.** 20 in all (2 per height: the scene as it is, and the same with every geom of the robot transparent), segmentation renders of the eagle camera at its (placeholder) resolution of 640 x 480; 57 ms per render warm (1.135 s / 20), 186 ms average cold (the first ones are slower); plus the pixel counting, 0.1 s per height in all. Building the two renderers and the transparent model copy on first use costs about 23 s once.

**IK verification.** The path points are the 12 task points of the scene (3 grasp heights, 3 set-down heights, 3 heights 10 cm above the set-down, as in the stance search; labeled for the left arm in this check). Each point is tried in 12 approach bins with 24 random starting joint vectors per bin: **3,456 starts per stance**. The stances evaluated, in this order (`Runner.evaluate`): the nominal stance (1, with the arm collision and furniture analysis), the joint-margin ladder (10, 14, 18, 22, 26, 30 degrees: 6 stances, one per rung), the base shift ladder (radii 1 to 8 cm, 8 directions each: 64 stances), the heading ladder (2, 4, 6, 8 degrees, both signs: 8 stances) and the pedestal ladder (half and one step, both signs: 4 stances): **83 stances, 286,848 starts in 252 batches** of the damped least squares IK (`solve_ik`, up to `ik_max_iter` = 40 iterations): **11.0 million iterations** in all, an average of 38.4 per start (a start stops when its constraint is met: only 20,989 of the 286,848, 7 %, converged, so nearly all run the full 40). Time: 146.2 s in `solve_ik` (0.58 s per batch, 0.51 ms per start, **1.77 s per stance**); 21 `Ctx.solve` calls (one per stance set), 20 `plain` calls (82 stances) plus the nominal analysis. No warm start: every start is a random joint vector (the warm-start path, `solve_warm`, exists but this case does not use it: `Runner.warm` is False); the starts of one stance do not use the solutions of its neighbours. A stance with a smaller tolerance than A would break out of a ladder at its first failing rung and run fewer stances; A's tolerances (7 cm, 8 degrees, 4 cm) are wide, so every ladder ran to its end (A is the stance the whole selection chose).

## 3. Anything recomputed that does not change

- **Scene loading, model compiling:** once per process (38 s for the case's scene and the IK chain, `Runner`; `StanceChecks` 0.0 s). Not per check. The furniture map and the reachability maps are not used by layer 3 (they belong to layer 2): not loaded here.
- **The arm collision checker** (one per pedestal height, a model copy; `Ctx.checker` keeps only the two most recent): the cold run measured 14.8 s for its single build, but standalone it takes 1.04 s the first time and 0.38 to 0.46 s each time after, also for another pedestal height (`stance_checker_build_time`: 0.16, 0.16, 0.12, 0.20, 0.10, 0.16 m); through `Ctx.checker` with its two-entry cache a new height costs 1.7 s (0.12 m) and 0.54 s (0.20 m). So the 14.8 s of the cold run is not reproduced by itself; I do not know its cause (candidates: memory pressure after the renderers were built on a VM without swap, or other load at that moment). Not part of the warm time.
- **The 8 perturbed neck solves repeat the whole global search** (the 2,401-point grid and 7 refinements) of what is nearly the same problem as the unperturbed one (0.1 degree of arm noise moves the target points by under a millimetre): 72 of the 90 solves, about 66 s of the 92.8 s. The same holds across the 10 heights (the torso moves 1 cm each time); no solve starts from the neighbour's answer (`local=False`).
- **The IK ladders** solve the same 12 points for 83 stances from random starts each time; the nominal solve is also done twice (the first ladder rung is the nominal 10 degree margin and repeats what `analyze` just solved), about 1.8 s.
- `place()` is called 5 times per height (50 calls, 0.05 s: negligible); a new `MjData` workspace is made for every framing solve (100 times, 0.44 s: negligible).

## 4. Top 15 functions by cumulative time (cProfile, stance A warm, 281.7 s with the profiler)

| # | Function (file:line) | Calls | Cumulative | What it does |
|---|---|---|---|---|
| 1 | `Ctx.solve` (stance_two_points.py:142) | 21 | 148.4 s | IK for a set of targets, looping over the 12 approach bins |
| 2 | `solve_ik` (arm_kinematics.py:156) | 252 | 148.3 s | damped least squares IK of thousands of starts at once (up to 40 iterations) |
| 3 | `<lambda>` (neck_framing.py:284) | 420,254 | 98.5 s | the cost function the neck search minimizes |
| 4 | `batch_fk_jac` (arm_kinematics.py:96) | 10,332 | 97.6 s | the arm's forward kinematics and Jacobian for many joint vectors |
| 5 | `_fk` (arm_kinematics.py:111) | 10,332 | 97.5 s (65.4 s itself) | the loop over the 7 joints that multiplies the transforms (numpy on arrays) |
| 6 | `_evaluate` (neck_framing.py:200) | 420,344 | 92.8 s | the cost of one neck pose: camera pose, projection of the points, fit, aim and posture terms |
| 7 | `minimize` (scipy _minimize.py:53) | 710 | 72.4 s | scipy's minimizer entry (the neck's refinements: 630, the slack search: 80) |
| 8 | `_minimize_neldermead` (scipy _optimize.py:682) | 710 | 72.2 s | the Nelder-Mead simplex search itself |
| 9 | `sorted` (builtin) | 93,090 | 58.2 s | sorting the scored grid points; the time is the evaluation of the grid inside it |
| 10 | `function_wrapper` (scipy _optimize.py:537) | 231,016 | 55.1 s | scipy's wrapper around the cost function in each refinement step |
| 11 | `<genexpr>` (neck_framing.py:289) | 216,180 | 50.5 s | scoring one coarse-grid point of the neck search (2,401 per solve) |
| 12 | `constraint` (grasp_rules.py:136) | 10,332 | 23.5 s | the grasp rule's residual and Jacobian inside the IK (hand level, position) |
| 13 | `_rodrigues` (arm_kinematics.py:74) | 72,324 | 21.3 s | the rotation about a joint axis, used by the forward kinematics |
| 14 | `_project` (neck_framing.py:58) | 512,816 | 19.8 s | projecting the world points through the camera model into the image |
| 15 | `pose` (neck_framing.py:160) | 512,916 | 19.7 s | the camera pose from the neck joint angles |

(Below these: `numpy.cross` 16 s, `numpy.clip` 14.7 s, `numpy.linalg.solve` 12 s inside the IK's least squares, `mj_kinematics` 8.3 s. The geometry distance calls of the clearance checks and the renders are not in the top 40.) Reading the table: lines 1, 2, 4, 5, 12, 13 are the IK (about 148 s); lines 3, 6 to 11, 14 and 15 are the neck solves and the slack search (about 100 s of the profiled run's 282 s; the warm timers say 104 s: 92.8 s + 11.6 s).

## 5. My reading

Two things dominate, and a third is small:

1. **The IK verification is 58 % of the check (146 s).** It runs a random-start IK for 83 stances (the nominal one and the tolerance ladders around it), each with 3,456 starts of about 38 iterations, and 93 % of the starts never converge so all run the full 40 iterations. The cost is Python and numpy arithmetic of the arm's forward kinematics (about two thirds of it), not MuJoCo. Nothing is shared between the stances: no warm start from the neighbouring stance's solutions, and the ladders always re-solve all 12 points.
2. **The neck framing solves are 37 % (93 s plus 12 s of slack search = 41 %).** 90 solves per check, each a full global search of about 4,700 cost evaluations (half of them in a 2,401-point grid that is the same every time), and 72 of them solve a problem that differs from the unperturbed one by 0.1 degree of arm noise; the 10 heights each start from scratch too.
3. **The rest is small once the process is warm:** the occlusion renders cost 1.25 s (57 ms each), the clearances 0.4 s; the one-time costs are the 38 s setup and about 25 s for the renderers on first use.

A stance that fails the cheap checks (B) costs half a second for all its heights; the checks are slow only for stances that pass. I did not optimize anything.
