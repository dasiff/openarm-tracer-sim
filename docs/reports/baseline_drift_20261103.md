# Manual bench baseline drift (2026-11-03)

Written for: David (repo owner). Read-only investigation. Nothing was fixed, regenerated or committed. This report is left uncommitted, as asked.

## Summary

- **The drift is real.** The current branch (`held-weight-ramp-sag-off`, HEAD `1056ab6`) does not reproduce `experiments/baselines/manual_reference.csv`. The output is not byte-identical, and all 84 data rows differ.
- **The first commit that changed it is `5b7349e`** (2026-10-26 11:22:09, "WIP: Part C step 1 code: src/gravity_ff.py, temporary gravity_comp switch, control_loop wiring (untested); Part B: stance_select_test 15/15"). The last good commit is its parent, `5b831ee`.
- **The cause is that commit's default setting.** It makes the arms use a calculated gravity torque (`src/gravity_ff.py`) instead of MuJoCo's built-in gravity compensation. Running the same commit with `OPENARM_GRAVITY_COMP=physics` (the old way) reproduces the reference **byte for byte**.
- **It is not your uncommitted hunks.** A clean copy of HEAD gives exactly the same bytes as the latest main-tree output (`data/oomd_check_20261103/old.csv`, made today at 13:39 with your hunks in place).
- **The baseline itself is repeatable.** The commit that wrote the current reference file (`4540b51`) reproduces it exactly, twice in a row.
- **Intended or not: INTENDED.** Decision M185 (David, 2026-10-26, option A) and the design plan's step 1 ("ACCEPTED by David") describe this exact change. The gap is that the reference file was never regenerated after that decision, so it is now out of date. Your call.
- **Further drift after that commit.** HEAD also differs from `5b7349e`'s default output, starting at row 55, with a largest difference of 1.78 rad on `right_joint1`. 38 later commits touched `src/` or the scenario. I did not look into this (see "NOT BUILT, needs decision").

## 1. The manual bench scenario command (step 1)

The scenario is run by `experiments/cc_bench_scenarios.sh`. The command used for the Nov 3 byte check (in `data/oomd_check_20261103/chain.sh`) is:

```
flock data/locks/mujoco.lock bash experiments/run_capped.sh <log> env CC_HEADLESS=1 bash experiments/cc_bench_scenarios.sh manual <out.csv>
```

- `CC_HEADLESS=1` means no window and no display (offscreen rendering).
- The `manual` case sets 1248 frames and a fixed key script, then runs `.venv/bin/python experiments/combined_controller.py`.
- The output has 85 lines (a header and 84 data rows) and 23 columns.
- The same job is used by `experiments/cc_bench_regen.sh`, which runs it twice and checks the two runs match before replacing the reference.
- Each run took about 5 to 15 seconds.

## 2. History of the file (step 2)

The file was **not** created on Oct 22. It first appeared on Oct 8. The content dated Oct 22 comes from `4540b51`.

| Commit | Date | Commit title | Blob (short) | What it did to `manual_reference.csv` |
|---|---|---|---|---|
| db017b8 | Oct 8 | Scripted staging source, episode runner and test sources | 97dba81 | Created the file |
| fe8c3d8 | Oct 13 | Base lean fix, 150 Hz loop tick, interpolated arm targets | 5a065a9 | Content changed |
| 007eddd | Oct 15 | New reference baselines (manual, teleop, policy) for the current code | a98fd36 | Regenerated |
| ff42a26 | Oct 15 | New reference baselines with the battery pack | 1a51759 | Regenerated |
| 5d57b51 | Oct 18 | New reference baselines on the imported robot; CC_HEADLESS=1 | b8505e4 | Regenerated |
| 4540b51 | Oct 22 | Docs: README and design plan updated for the fixed camera (no neck) | a3fede0 | **Rewrote all 84 data rows.** The commit also touched 10 other files. Its title says docs only. |
| ac64f22 | later | Fixed camera step 2 (E3): baselines regenerated | a3fede0 (unchanged) | Did not touch this file (its message says "manual unchanged"; the blob confirms it) |

- No commit after `4540b51` changed the file up to HEAD `1056ab6`. The working-tree copy is blob `a3fede0`, the same as `4540b51`.
- The regenerated output from Oct 22 (`data/cc_baseline/regen/manual_1.csv`) is byte-identical to the reference.

## 3. Is the drift real? (step 3)

Method: a clean copy of HEAD `1056ab6` in a scratch worktree (no uncommitted hunks), run once through the 6 GB cap, then compared with the reference.

| Check | Result |
|---|---|
| Byte-identical to reference | **No** (23,930 bytes against 23,911) |
| Rows and columns | 85 rows each, 23 columns each, same header |
| First differing cell | Row 1 (file line 2), column `base_x`. Reference -0.998681215, clean HEAD -0.999527444 |
| Rows that differ | 84 of 84 |
| Largest absolute difference | 1.776354 (row 77, column `right_joint1`) |
| Columns that differ | 22 of 23 (`sim_time` is the same) |

**Main working tree.** The latest existing output is `data/oomd_check_20261103/old.csv` (made today at 13:39). It is byte-identical to the clean HEAD output. So your uncommitted hunks do not change the output of this scenario. The drift comes from commits.

## 4. Repeatability (step 4)

The commit that wrote the current reference, `4540b51`, was run twice in the scratch copy.

- Run 1 and run 2 are byte-identical to each other.
- Both are byte-identical to `experiments/baselines/manual_reference.csv`.

So the baseline file and the machine agree. The drift is caused by a commit after `4540b51`, and the hunt went ahead.

## 5. Bisect (step 5)

Range: good `4540b51`, bad `1056ab6`. That is 340 commits, which took 8 bisect steps. No commit was skipped, because all 8 runs finished.

| Step | Commit | Result |
|---|---|---|
| 1 | c5a22cb | differs |
| 2 | c72a5dc | matches |
| 3 | 26dd074 | differs |
| 4 | 9532f9e | matches |
| 5 | 3297da9 | matches |
| 6 | 45597f9 | differs |
| 7 | 5b7349e | differs |
| 8 | 5b831ee | matches |

Result: **first bad `5b7349e`, last good `5b831ee`.**

Checks on the two endpoints, each run twice:
- `5b831ee`: both runs match the reference, and they match each other.
- `5b7349e`: both runs differ from the reference, and they match each other.

Limit of the method: bisect assumes that once the output changes it stays changed. I did not test every commit in the range. The mechanism test below is the direct check.

**Mechanism test.** `5b7349e` with `OPENARM_GRAVITY_COMP=physics` (the old gravity method) is **byte-identical** to the reference. With its default setting (`feedforward`) it differs, and its first differing cell is the same as HEAD's (row 1, `base_x`, -0.999527444). So this one switch explains the drift at `5b7349e`.

## 6. The first-bad commit (step 6)

### What it changed

| File | Change, in plain words |
|---|---|
| `src/gravity_ff.py` (new, 82 lines) | Works out the gravity pull on each arm joint from the arm's own link masses at the measured arm pose. |
| `src/control_loop.py` (+11) | Adds that pull to each joint's target on every control tick (command = target + torque / gain). Also used in the settling steps. |
| `src/arm_actuators.py` (14 lines) | With the new setting, MuJoCo's built-in gravity help is turned off on the arm and finger bodies, and the actuator range is widened so the sum is not clipped. |
| `src/robot_specs.py` (+5) | Adds a switch, `ARM_ACTUATOR_SPECS["gravity_comp"]`, default `"feedforward"`. The environment variable `OPENARM_GRAVITY_COMP=physics` selects the old method. |

In short: the simulator no longer does the arms' gravity compensation. The controller computes it instead. Every tick the arm commands shift by a small amount, so every later value in the scenario moves.

### What the nearby records say

Commit messages:
- `5b831ee` (last good): "WIP: progress, stance_select_test run started".
- `26dd074` (same change series): "WIP: gripper_force_test welded-beaker check allows the finger's own weight under the feedforward (M185)".
- `dc396f0` (2026-10-26 13:33, later): "WIP: step 2: gravity switch deleted (code), tests adjusted, not yet run". This removed the switch, so the calculated torque became the only method.
- `863316c` (2026-10-26 13:50, later): "Steps 1-7: report (...switch deleted...)".
- The message of `5b7349e` itself says "(untested)". That is out of date: `docs/progress.md` records `gravity_ff_test` 12/12 passing about 3 minutes later.

Decision and plan records:
- `docs/mpvi_decisions.md`, row **M185** (dated 2026-10-26): "Arm gravity compensation, step 1 of 3: a calculated extra torque replaces MuJoCo's gravcomp (David's decision, Oct 26, option A)". Part (d) says the switch is temporary and "deleted once David decides". That matches the code.
- `docs/design_plan.md`, line 356: "Step 1: BUILT 2026-10-26, measured, ACCEPTED by David (the switch back to MuJoCo's gravcomp deleted 2026-10-26, the calculated torque is the only method)."
- `docs/progress.md`, line 386: "C1: M185, gravity_ff, switch (temporary, default feedforward), old vs new identical, zombie_held fails in both". Line 392: "STEP 2 DONE (switch deleted ...)".

What "old vs new identical" means: from the wording and the report it refers to episode results (plain episodes succeed with the same numbers) and to the suite's replay tests. It does **not** mean the bytes of `manual_reference.csv` were checked. Nothing in the records says the reference was regenerated or byte-checked after this commit. I could not confirm the progress line's intended meaning, so please check it.

### Classification: INTENDED

Evidence:
1. M185 is a decided row, David's choice (option A, Oct 26). The code in `5b7349e` matches parts (a) to (d) of that row.
2. The design plan says David accepted step 1.
3. The output change comes from one switch. Setting it to the old method gives the reference byte for byte (section 5).

Gap that does not change the label: the reference file was not regenerated after a decided change. The Nov 3 oomd report already flagged this as "NOT BUILT, needs decision".

## Commands run

Main repo (read only, no writes):
- `git log --follow`, `git log --diff-filter=A`, `git show --stat`, `git show <commit>:experiments/baselines/manual_reference.csv` (blob comparison), `git ls-tree`, `git rev-list --count`, `git hash-object`
- `cmp` and `cat` on `data/oomd_check_20261103/*`, `data/cc_baseline/regen/manual_1.csv`, `experiments/baselines/manual_reference.csv`
- `.venv/bin/python -I /tmp/drift_out/tools/cmp_csv.py <reference> <output>` (my comparison script, in `/tmp/drift_out/tools/`)

Scratch copy: `git worktree add /tmp/drift_wt 1056ab6`, then `ln -s` of the main `.venv` into it, and a symlink of `deps/OpenArm-Combined` to the main repo's submodule folder (the robot XML loads its meshes from there; the submodule commit is the same in every commit checked).

Each simulation was run as:

```
flock /home/vboxuser/projects/openarm-tracer-sim/data/locks/mujoco.lock \
  bash /tmp/drift_wt/.drift/run_capped.sh /tmp/drift_out/<name>.log \
  env CC_HEADLESS=1 [OPENARM_GRAVITY_COMP=physics] \
  bash experiments/cc_bench_scenarios.sh manual /tmp/drift_out/<name>.csv
```

- `.drift/run_capped.sh` is HEAD's copy of `experiments/run_capped.sh` (same 6 GB cap, same oomd setting). It lives in the scratch copy only, outside git tracking, because `4540b51` has no `run_capped.sh` and other commits track a different one.
- Runs (all exit 0): HEAD `1056ab6` once; `4540b51` twice; `5b7349e` once in physics mode; the bisect's 8 steps; and the endpoints `5b831ee` and `5b7349e`, twice each.
- Bisect: `git bisect start 1056ab6 4540b51`, then `git bisect run bash /tmp/drift_out/tools/bisect_step.sh` (exit 0 = matches reference, 1 = differs, 125 = run failed). Then `git bisect reset`.
- Outputs and logs are in `/tmp/drift_out/` (CSV files, `.log` files from `run_capped.sh`, `bisect/`, `bisect_run.log`, `tools/`).

Caveats of the method:
- The scratch copy has no `data/` folder (it is git-ignored). The scenario did not need it: the clean HEAD output matches the Nov 3 main-tree output byte for byte. This is one scenario only.
- Runs were short (under 30 seconds each), so no tmux session was used. Every run held the project's MuJoCo lock, one at a time.
- No commits, no pushes, no changes to `experiments/baselines/`, and no changes in the main working tree. `git bisect` left no refs behind.
- `deps/AgileX_Tracer` shows as modified in `git status`, but that comes from untracked `__pycache__` folders dated Oct 22, which predate this task. I did not touch it.
- `docs/progress.md` was not updated, because you asked for no changes in the main tree beyond this report. CLAUDE.md asks for a progress entry and a commit of reports. I followed your instructions here.

## NOT BUILT, needs decision

1. **Regenerate `experiments/baselines/manual_reference.csv`?** Not done. You need to pick which state the reference should match: the output at `5b7349e` with the calculated gravity torque, or the output at HEAD (see item 2).
2. **Drift after `5b7349e`.** HEAD differs from `5b7349e`'s default output, starting at row 55 (`base_x`), with a largest difference of 1.78 rad on `right_joint1` (22 columns). 38 commits touched `src/` or the scenario in between. A second bisect over those would take about 8 steps and about 5 minutes. I did not run it, because it was outside the question.
3. **Teleop and policy references.** Not checked; this task covered the manual scenario only.
4. **Meaning of "old vs new identical"** in `docs/progress.md` line 386. Please confirm it meant episode results, not the bytes of the reference file.
5. **Bisect assumption.** Only 8 of the 340 commits were run, and the bisect assumes the output does not change back. The physics-mode check supports the conclusion for `5b7349e`, but I did not check the other commits.
