# Manual bench drift, part 2: every commit that changes the output (2026-11-03)

Written for: David (repo owner). Read-only investigation. Nothing was fixed, regenerated or committed. This report is left uncommitted, as asked.

## Summary

- **Change points: 1.** Between `5b7349e` (the gravity compensation switch, already confirmed intended) and HEAD `1056ab6`, exactly one commit changes the output of the manual bench scenario: **`6d9b542`**, 2026-10-29 12:53, "Command bridge on by default (M193)".
- **Labels: INTENDED 1, NOT INTENDED 0, CAN'T TELL 0.** The commit flips the default of the command bridge (a rate limiter on the arm commands) from off to on for every run. The decision log still says the opposite ("OFF everywhere else"). David confirmed on Nov 3 that the change was intended. The decision log, design plan and progress log have not been updated to say so yet.
- **HEAD equals the last change point.** The output after `6d9b542` is byte-identical to HEAD's output, across the 8 later candidate commits and HEAD.
- **The two oomd commits did not change the output.** `1070817` (its parent is `e9aaa63`) and `1056ab6` (its parent is `19ce0cc`) are both byte-identical to their parents.

## Table of change points

| Change point | Commit | Date | Commit message (short) | Rows differing (of 84) | First differing cell | Largest absolute difference | Label |
|---|---|---|---|---|---|---|---|
| 1 of 1 | `6d9b542` | 2026-10-29 12:53 | Command bridge on by default (M193) | 30 | row 55 (file line 56), column `base_x`: before -0.225279838, after -0.225279841 | 1.777676 (row 77, column `right_joint1`) | INTENDED (David, Nov 3) |

Terms used in the table:
- "Row" is one line of the output file after the header. There are 84 of them, one every 75 physics steps (about 0.1 s of simulated time).
- "Column" is one measured quantity: a base position, the base's heading (yaw), a joint angle or a finger position.
- "Largest absolute difference" is the biggest change in any cell. Joint values are in radians, `base_yaw_deg` is in degrees.

## 1. The commit list (step 1)

- The range `5b7349e..1056ab6` has **222 commits** (excluding `5b7349e`, including `1056ab6`).
- **No merge commits.** The history is a single straight line, so "oldest first" is the same as the order from `git rev-list --reverse --topo-order`. The full list is in `/tmp/drift_out2/commits_all.txt`.
- **How I decided which commits could change the output.** The scenario starts `experiments/cc_bench_scenarios.sh`, which runs `experiments/combined_controller.py`. That program imports `src/` (checked: `src/` imports only other `src/` files and installed packages, not `experiments/`, `docs/` or `tools/`). The robot and scene files come from `models/`, and the robot meshes come from the `deps/` submodule. So a commit can change the output only if it changes one of: `src/`, `models/`, `deps/` (including a submodule pointer), `experiments/combined_controller.py` or `experiments/cc_bench_scenarios.sh`.
- **38 commits touch those paths ("candidates"). 184 do not ("skipped").** The skipped list is in `/tmp/drift_out2/skipped.txt`. Skipped commits are mostly docs, progress notes, reports, tests and experiment scripts. None of them is an input to the scenario.
- No commit in the range touched `models/` or `deps/`. The submodule pointers are the same as at `5b7349e`.

## 2. End points (step 2)

- `5b7349e` run once: `/tmp/drift_out2/csv/01_5b7349e.csv`. It is byte-identical to the earlier run in `/tmp/drift_out/rep_5b7349e_1.csv`.
- `1056ab6` (HEAD) run once: `/tmp/drift_out2/csv/40_1056ab6.csv`. It is byte-identical to the earlier run in `/tmp/drift_out/head1056_run1.csv`.

## 3. Walk forward (step 3)

I ran the scenario **once at each candidate, oldest first**, comparing each output with the previous run's output byte for byte. I did not split the range, because a full walk of 40 runs took about 15 minutes.

Results (`SAME` means byte-identical to the row above it in the table; `CHANGED` means it differs):

| Run | Commit | Date | Commit message (short) | Compared with previous run |
|---|---|---|---|---|
| 1 | 5b7349e | 10-26 11:22 | WIP: Part C step 1 code: src/gravity_ff.py, temporary gravity_comp switch | start |
| 2 | 5b84ab8 | 10-26 11:27 | WIP: direct-ctrl tests and spawn_and_settle apply the gravity feedforward | same |
| 3 | 83d4b61 | 10-26 13:28 | WIP: step 6 code: src/weight_estimate.py (not run) | same |
| 4 | dc396f0 | 10-26 13:33 | WIP: step 2: gravity switch deleted (code), tests adjusted | same |
| 5 | 5be9722 | 10-26 14:01 | WIP: weight + balance point estimator (measurement only) | same |
| 6 | 83c81ab | 10-26 14:22 | WIP: step 2 code: FULLCHECK_CFG start_mode | same |
| 7 | d75e569 | 10-26 15:26 | Step 1: neighbourhood start is the only behaviour, start_mode deleted, M187 decided | same |
| 8 | c5a22cb | 10-26 17:04 | Step 2: condim 4 + torsional 0.0012 for finger-pad/beaker pairs (M189) | same |
| 9 | 7422e01 | 10-26 22:04 | Step 3: beaker wall ring 48 segments (M191 pending) | same |
| 10 | f24a75f | 10-26 22:06 | Step 5: handoff reading time limit | same |
| 11 | 6a6a1e1 | 10-27 03:18 | Report: pads parallel (erratum), 48-segment beaker | same |
| 12 | 31c834a | 10-27 13:20 | WIP: shared status panel, video without legend/fps row | same |
| 13 | 748ddc3 | 10-27 13:23 | WIP: replay step hook, render_at_ticks, panel_stills | same |
| 14 | e56bbb8 | 10-27 13:30 | WIP: command bridge (COMMAND_BRIDGE_CFG, loop setting, off by default) and its test | same |
| 15 | 44961c8 | 10-27 13:31 | Command bridge: merged repeat log entries, test added to the suite | same |
| 16 | 04755f6 | 10-27 15:42 | WIP: joint-limited phase duration (dry-run IK travel / speed limit) | same |
| 17 | 53c8bf5 | 10-27 15:52 | Part B: joint-limited duration built; no effect on the moved-layout episode | same |
| 18 | 7d2308a | 10-27 16:59 | WIP: joint-limited duration from the IK arc length, 1.875 peak factor | same |
| 19 | af44f69 | 10-27 17:10 | Part B: duration factor 2.0 (David); run next | same |
| 20 | eb08606 | 10-28 02:25 | WIP: restart_from_commanded (M195 row and tests not done) | same |
| 21 | aee139e | 10-28 02:29 | WIP: restart_from_commanded: fix shadowed ids in start() | same |
| 22 | 94d1064 | 10-28 02:37 | restart_from_commanded measured: no fix; default OFF pending David | same |
| 23 | 01ca551 | 10-28 02:53 | WIP: phase-start velocity ramp (M195 row, tests, suite not done) | same |
| 24 | 00b350f | 10-28 03:13 | WIP: skill-start ramp | same |
| 25 | 598ae37 | 10-28 03:17 | WIP: minimum-jerk phase-start ramp (0.5625 s) | same |
| 26 | b247c10 | 10-28 12:31 | WIP: item 1 measure script rerun, MINK_CFG uniform_speed_scaling option (off) | same |
| 27 | 29860e6 | 10-28 13:54 | WIP: held-object gravity compensation (M196), ramp 1.0 s, pedestal override 0.15 m (M197) | same |
| 28 | 496568c | 10-29 12:04 | WIP: phase-start ramp blends the solver speed along the min-jerk curve (M198) | same |
| 29 | 6e863f4 | 10-29 12:16 | M199: phase-start ramp back to 0.5625 s | same |
| 30 | bc14a77 | 10-29 12:22 | WIP: blend between moves within 20 mm (M200) | same |
| **31** | **6d9b542** | **10-29 12:53** | **Command bridge on by default (M193)** | **CHANGED** |
| 32 | 55725db | 11-01 21:52 | Session 2: Servo control signal logging to control_loop.py | same |
| 33 | 1a1f41e | 11-01 21:53 | Session 3: Gravity feedforward logging to gravity_ff.py | same |
| 34 | 8e24282 | 11-02 16:23 | Bounce branch build: held-mass ramp owns the mass, sag gain 0 (M215-M222) | same |
| 35 | 13cebeb | 11-02 18:06 | Sag correction back on at gain 3.0, with a freeze (M225-M227) | same |
| 36 | 3d44e7d | 11-02 19:03 | Trim handoff blend: no freeze at the grasp (M230-M232) | same |
| 37 | c52d09c | 11-02 19:41 | Trim handoff z only; begin_load_change removed (M230-M234) | same |
| 38 | a9f8379 | 11-02 21:16 | Lift-off weighing (WEIGH step), trim cleanup (M235-M240) | same |
| 39 | 19ce0cc | 11-03 01:18 | WIP: lift-off trace logging, marker removals, and the run 1 analysis script | same |
| 40 | 1056ab6 | 11-03 13:40 | Exempt sim runs from systemd-oomd (run_capped.sh ManagedOOMPreference=omit) | same |

Outputs of all 40 runs: `/tmp/drift_out2/csv/`. Logs: `/tmp/drift_out2/logs/`.

Reading the table: the 29 candidate runs after `5b7349e` (runs 2 to 30) are byte-identical to it. This matches the earlier finding that the output is set by `5b7349e` and not changed by anything before `6d9b542`. After `6d9b542` every run is identical to HEAD.

### Why the skipped commits were not run

I ran 2 of the 184 skipped commits as a check (`35fdb2a`, which touches `docs/progress.md` and `experiments/bounce_diag_extract.py`, and `021a7c0`). Both are byte-identical to HEAD's output. I also ran the parent of an oomd commit (see section 5). The rest were not run. They are not in the scenario's inputs, so the path-based rule should hold. This is a reasoned step, not a per-commit run.

## 4. The change point in detail (step 4)

### Numbers

Compared with the previous output (run 30, `bc14a77`):
- Rows differing: **30 of 84**.
- First differing cell: row 55 (file line 56), column `base_x`. Before: -0.225279838. After: -0.225279841. The difference is 3 in the 9th decimal place, so the change starts very small.
- Largest absolute difference: **1.777676** (row 77, column `right_joint1`, a joint angle in radians).
- Columns that differ: 22 of 23.

The bridge change is therefore not a rounding effect. The largest changes are in the arm joints (`right_joint1` 1.78 rad, `left_joint3` 1.32 rad). These are the same columns that differed between `5b7349e` and HEAD in the first report.

### What the commit changed

Two one-line changes, in two files:

| File | Change |
|---|---|
| `src/task_config.py` | `COMMAND_BRIDGE_CFG["enabled"]` goes from `False` to `True`. The comment is changed to "Default on from David's request, 2026-10-29 [row M193]". |
| `src/control_loop.py` | The comment describing the bridge says "off by default" is changed to "on by default". No code changes. |

Plain-language summary: the command bridge is a rate limiter for the arm commands. It takes a new arm target at 50 Hz (every third tick of the 150 Hz loop), and it limits how fast a command can move, rejects a target that jumps too far, and holds the command while the arm has not caught up. With the default turned on, every run, including the manual bench scenario, now passes its arm commands through this limiter. Before this commit, the bridge was only on in the demo video script.

### What the records say

**Commit message** (`6d9b542`): "Command bridge on by default (M193)". The commit has no body text. It does not say why, or whether any baseline was checked.

**Decision log** (`docs/mpvi_decisions.md`, row M193, dated 2026-10-27). The row says the bridge is "OFF by default (`enabled` False: the keyboard, teleop and staging runs and the baselines are unchanged)". It also says "ON in `experiments/demo_video.py` by default (`--no-bridge` turns it off), OFF everywhere else." The commit `6d9b542` did not update this row. The row's source is "David 2026-10-18 / 2026-10-20 (limits and OFF default)".

**Design plan** (`docs/design_plan.md`, line 37): "[DONE 2026-10-27, row M193: ... OFF by default, ON in the demo video script]". Line 43 says "[DECIDED 2026-10-20, David] The bridge is a loop setting (`COMMAND_BRIDGE_CFG`), OFF by default". Neither line was updated by `6d9b542`.

**Progress log** (`docs/progress.md`):
- Line 26 (2026-10-28, David's instructions): "then the full suite (bridge OFF, all fixes ON)". So on Oct 28 David asked for the suite to run with the bridge off.
- Line 138 (2026-10-27): "COMMAND_BRIDGE_CFG (task_config), bridge in control_loop (cfg command_bridge, off by default) ... demo_video.py default bridge ON (--no-bridge)".
- No progress entry on 2026-10-29 mentions a default change. I searched for "bridge" with "default", "on by" and "off by" and found none for that date.

**Config comment** (`src/task_config.py`, the line changed by the commit): "Default on from David's request, 2026-10-29 [row M193]". This is the only record of the request. It is in the code, and the decision log has no row for it.

**Treating these as claims, not proof**: the commit title and the config comment say the change was David's request. The decision log says the opposite, and the progress log does not record the request. I could not verify the request from the files.

### Label: INTENDED (confirmed by David, Nov 3)

David confirmed that the change was intended. The evidence I found before his answer, for the record:

Evidence before the confirmation (the label is now INTENDED by David's word, not by a decision row):
1. For INTENDED, I would need a decision record that explains the change. The only records are a config comment ("David's request, 2026-10-29") and a commit title. Neither has a decision row number, and the decision row for the bridge (M193) says the default is OFF everywhere else.
2. For NOT INTENDED, I would need a record that David did not want the change. I found none. The config comment says he asked for it.
3. The change does change the baseline: the manual scenario's output is different from this point on. The baseline file (`experiments/baselines/manual_reference.csv`) was not regenerated, and the commit does not mention it.

So the records alone did not settle it; David's confirmation does. Still open: no decision row or design-plan line records the change yet.

## 5. The total: do the change points add up? (step 5)

- The last change point is `6d9b542`, and its output is the reference for all later runs.
- **Runs 32 to 40 are byte-identical to the output of `6d9b542`.** Run 40 (HEAD) is byte-identical to run 31 (`6d9b542`), so the output of `6d9b542` is HEAD's output. Checked with `cmp`.
- The chain is consistent: from `5b7349e` (run 1) to HEAD (run 40), the only change is at run 31. The change points add up to the full difference.

### The two oomd commits

- `1070817` ("WIP: run_capped.sh exempts sim runs from systemd-oomd"): its parent is `e9aaa63`. I ran `e9aaa63` and `1070817`. Their outputs are **byte-identical**. Output: `/tmp/drift_out2/csv/parent_e9aaa63.csv` and `/tmp/drift_out2/csv/oomd_1070817.csv`.
- `1056ab6` ("Exempt sim runs from systemd-oomd ..."): its parent is `19ce0cc`. Run 39 (`19ce0cc`) and run 40 (`1056ab6`) are **byte-identical**.

So the oomd change (the setting that keeps the systemd out-of-memory killer away from simulation runs) does not change the simulated output. Your statement holds.

## Commands run

Main repo (read only):
- `git rev-list --count 5b7349e..1056ab6` (222), `git rev-list --merges --count ...` (0), `git rev-list --reverse --topo-order ...`
- `git diff-tree --no-commit-id --name-only -r <commit>` for each commit (to decide candidates)
- `git log`, `git show`, `grep` on `docs/mpvi_decisions.md`, `docs/design_plan.md`, `docs/progress.md` and `src/`
- `.venv/bin/python -I /tmp/drift_out/tools/cmp_csv.py <ref> <out>` (my comparison script, reads two CSV files, writes no files in the repo)

Scratch copy:
- `git worktree add /tmp/drift_wt2 5b7349e`
- Symlinks inside the scratch copy: `.venv` to the main repo's `.venv`, and `deps/OpenArm-Combined` to the main repo's submodule folder (the robot's meshes are read from there; the submodule commit is the same in every commit in this range).
- `.drift/run_capped.sh` in the scratch copy is HEAD's copy of `experiments/run_capped.sh`, kept outside git tracking so that commits without the file, or with a different one, can be checked out.

Each simulation used:

```
flock /home/vboxuser/projects/openarm-tracer-sim/data/locks/mujoco.lock \
  bash /tmp/drift_wt2/.drift/run_capped.sh /tmp/drift_out2/logs/<tag>.log \
  env CC_HEADLESS=1 bash experiments/cc_bench_scenarios.sh manual /tmp/drift_out2/csv/<tag>.csv
```

No `OPENARM_GRAVITY_COMP` override, so the default settings were used. All 40 walk runs, 2 skipped-commit spot checks, the oomd parent run and the oomd run exited 0.

Driver: `/tmp/drift_out2/driver.sh`, run in a detached tmux session `drift2`, with the list in `/tmp/drift_out2/run_list.txt` (40 lines). The driver ran the runs one at a time, under the project's MuJoCo lock. Log: `/tmp/drift_out2/driver.log`.

Byte comparison: `cmp` on consecutive CSVs (see the table in section 3).

Caveats:
- The skipped commits were not all run (see section 3).
- The scratch copy has no `data/` folder (it is git-ignored). This is the same caveat as in part 1. The clean HEAD output matched the earlier main-tree output byte for byte, so `data/` does not seem to affect this scenario.
- The bisect in part 1 assumed a single change. This walk does not, because it ran every candidate. It found one change point.
- The main working tree was not changed. No commits, no pushes, and no changes to `experiments/baselines/`.

## NOT BUILT, needs decision

1. **Record the decision for `6d9b542`.** David says it was intended. The decision log (M193) still says OFF everywhere except the demo video script, the design plan (lines 37 and 43) says the same, and no progress entry records the change. Not edited. Say if you want these updated (and the request written into M193).
2. **The baseline file.** `experiments/baselines/manual_reference.csv` still matches the code before `6d9b542`. Nothing was regenerated. Decide whether to regenerate it to HEAD's output, or keep it as the pre-bridge reference.
3. **Teleop and policy references.** Not checked. This task covered the manual scenario only. They may also have changed at `6d9b542` or elsewhere.
4. **The 184 skipped commits.** Not run. They are excluded by the rule in section 1 and by two spot checks. If you want full proof, the remaining 184 runs would take about 45 minutes.
5. **The progress log entry for `6d9b542`.** There is no entry for it. The confirmation should be written down (a decision row, or a progress line).
