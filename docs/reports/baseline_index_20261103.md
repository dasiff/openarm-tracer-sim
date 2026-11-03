# Baseline index, retirement of old references, and the new tools (2026-11-03)

Written for: David (repo owner). Tooling and file moves only. No simulation was run. The regeneration script was not run for real. Nothing was pushed. This report is left uncommitted.

## Summary

- **What moved.** The three old references were moved with `git mv` into `experiments/baselines/retired/`: `manual_reference.csv`, `teleop_reference.csv`, `policy_reference.csv`. Their contents are unchanged. Their SHA-256 hashes match the values taken before the move.
- **What stayed.** `experiments/baselines/manual_reference_20261103.csv` and its README note stayed where they are, with their names unchanged.
- **New index.** `experiments/baselines/INDEX.csv` lists every reference file in the folder. The only CURRENT baseline is `manual_reference_20261103.csv` (made from clean commit e127479, committed in 8f10aac). Teleop and policy are NO_CURRENT. Their rows have no file.
- **Commit:** `21721ad`, "Baselines: retire the old references, add INDEX.csv (CURRENT per scenario), regen and compare tools use it (M242)". It contains 11 files: 3 moves, 2 new files and 1 new tool in the baselines area, the two tools, the scenario script (comments only), and two docs. Nothing was pushed.
- **Decision.** M242 in `docs/mpvi_decisions.md` records David's baseline policy. Progress entry added in `docs/progress.md`.
- **Tested:** the index reader, the comparison tool (against the Nov 3 output, and its refusal paths), the regeneration script's syntax and its `--dry-run` path, and the byte-level check of the moved files.
- **Not tested:** the real path of the regeneration script (it was not run, by rule). See "NOT BUILT, needs decision".

## 1. Inventory before moving (step 1)

Recorded before any move. The hashes are in `/tmp/baseline_inventory_before.sha256`.

| File (before the move) | Scenario | Date | Creating commit | SHA-256 |
|---|---|---|---|---|
| `experiments/baselines/manual_reference.csv` | manual | 2026-10-22 | 4540b51 (the commit that wrote this exact content) | `a2323325656c3d4106de8eaed52600962dc2bbae4cc1ae1ebade1fa44f65f5f3` |
| `experiments/baselines/teleop_reference.csv` | teleop | 2026-10-22 | ac64f22 | `5e54bd3015b1afb691218091e22c94102da201aa434737c85df6622b4006610b` |
| `experiments/baselines/policy_reference.csv` | policy | 2026-10-22 | ac64f22 | `aba570c962109744f474c86b62ba020e6f3b0220be4cec152974be1432f2a6d4` |
| `experiments/baselines/manual_reference_20261103.csv` | manual | 2026-11-03 | 8f10aac (made from e127479) | `3060ae41cc8518e65cad011c6f48e25c273d8a105609c63e5c41a42f6c4c9b82` |
| `experiments/baselines/manual_reference_20261103.README.txt` | (note) | 2026-11-03 | 8f10aac | `bf3f9c58490d55a1a7365fb17e21836b943ec8e85833d08bbd88b5b17c25bba7` |

How the creating commit was found: `git log --follow` lists the commits that touched each file. For each file, I compared the file's blob hash with the blob at each commit. The commit whose blob matches the current content is the creator. For `manual_reference.csv`, that is 4540b51. For `teleop_reference.csv` and `policy_reference.csv`, it is ac64f22. (The earlier versions of these files are in git history; see "NOT BUILT".)

Other places where baselines live (not moved; git-ignored): `data/cc_baseline/` (including `regen/`), and `data/baselines_tmp*`. See "NOT BUILT".

## 2. Moves (step 2)

```
mkdir -p experiments/baselines/retired
git mv experiments/baselines/manual_reference.csv  experiments/baselines/retired/manual_reference.csv
git mv experiments/baselines/teleop_reference.csv  experiments/baselines/retired/teleop_reference.csv
git mv experiments/baselines/policy_reference.csv  experiments/baselines/retired/policy_reference.csv
```

Hash check after the move (`sha256sum experiments/baselines/retired/*.csv`): all three match step 1.

## 3. The index (step 3)

File: `experiments/baselines/INDEX.csv`. Columns: `scenario, file, date, commit, status, reason, sha256, clean_code`. The file is written by `experiments/cc_bench_index.py`.

```
scenario,file,date,commit,status,reason,sha256,clean_code
manual,retired/manual_reference.csv,2026-10-22,4540b51,RETIRED,retired Nov 3 2026 by David,a2323325656c3d4106de8eaed52600962dc2bbae4cc1ae1ebade1fa44f65f5f3,unknown
teleop,retired/teleop_reference.csv,2026-10-22,ac64f22,RETIRED,retired Nov 3 2026 by David,5e54bd3015b1afb691218091e22c94102da201aa434737c85df6622b4006610b,unknown
policy,retired/policy_reference.csv,2026-10-22,ac64f22,RETIRED,retired Nov 3 2026 by David,aba570c962109744f474c86b62ba020e6f3b0220be4cec152974be1432f2a6d4,unknown
manual,manual_reference_20261103.csv,2026-11-03,e127479,CURRENT,made from clean commit e127479 (committed in 8f10aac),3060ae41cc8518e65cad011c6f48e25c273d8a105609c63e5c41a42f6c4c9b82,yes
teleop,,,,NO_CURRENT,all old baselines retired Nov 3 2026; no regenerated baseline yet,,
policy,,,,NO_CURRENT,all old baselines retired Nov 3 2026; no regenerated baseline yet,,
```

Notes on the rows:
- `clean_code` for the old files is `unknown`. The run records from October do not say whether the runs were made from a clean tree, so I did not write `yes` or `no`.
- The manual CURRENT row's `commit` is e127479 (the commit it was made from), as the step asked. Its `reason` says it was committed in 8f10aac.
- Exactly one row per scenario is CURRENT, or the scenario has one NO_CURRENT row. Teleop and policy each have one RETIRED row (the old file) and one NO_CURRENT row. Their NO_CURRENT rows have no file, which is what was asked.

`experiments/baselines/README.md` explains the columns, the statuses, that the tools read the CURRENT row, and that every regeneration is deliberate and logged.

## 4. Regeneration script (step 4)

`experiments/cc_bench_regen.sh`, rewritten. Usage: `bash experiments/cc_bench_regen.sh [--dry-run] [--commit REV] <manual|teleop|policy>`.

What it does, in order (the header of the file says the same):
1. Reads the CURRENT row for the scenario. If the index is missing or unreadable, it stops. For NO_CURRENT, it continues, with nothing to retire.
2. Refuses if the new CSV or README already exists (never overwrites).
3. Makes a clean worktree of the commit (default HEAD) in a temporary folder outside the repo. Stops if that fails. Links in `.venv` and the robot mesh folder (the mesh folder is a submodule, which is empty in a fresh worktree).
4. Runs the scenario twice, through `run_capped.sh`, one at a time under `data/locks/mujoco.lock`. Stops if a run fails.
5. Stops, writing nothing into the repo, if the two runs are not byte-identical. The outputs are kept under `/tmp/cc_regen_out.*` for a look.
6. Writes `experiments/baselines/<scenario>_reference_<YYYYMMDD>_<shorthash>.csv` (through a temporary name, then renamed) and the README note `<same name>.README.txt`. The note contains: the commit, the date, the command, the settings copied from `cc_bench_scenarios.sh` for that scenario, the run result, the row and column counts, and the SHA-256.
7. Moves the old CURRENT file to `retired/` with `git mv` (staged, not committed).
8. Updates `INDEX.csv` through `cc_bench_index.py promote`: the old row becomes RETIRED, a NO_CURRENT row is dropped, and the new CURRENT row is added.
9. Removes the worktree (the links are unlinked first, so the removal cannot touch the main repo's venv or meshes).

It never commits. `--dry-run` prints the plan and runs nothing.

Dry-run results (run in the repo, no changes made):
- `bash experiments/cc_bench_regen.sh --dry-run manual`: plans a new file, retires `manual_reference_20261103.csv`, and updates the index. Exit 0.
- `bash experiments/cc_bench_regen.sh --dry-run teleop`: plans a new file and reports nothing to retire (NO_CURRENT). Exit 0.
- `bash experiments/cc_bench_regen.sh --dry-run --commit nosuchrev manual`: refused, "is not a commit". Exit 2.
- `bash experiments/cc_bench_regen.sh --dry-run`: refused, asks for a scenario. Exit 2.
- After these, `git worktree list` showed only the main repo, and no new files appeared in `experiments/baselines/`.

Syntax check: `bash -n experiments/cc_bench_regen.sh` passed.

The script's settings extraction was tested on its own: the awk expression that copies a scenario's case block from `cc_bench_scenarios.sh` prints the `manual` and `teleop` blocks correctly.

## 5. Comparison tool (step 5)

`experiments/cc_bench_compare.py`. Usage: `python experiments/cc_bench_compare.py <manual|teleop|policy> new.csv [--index PATH]`.
- It reads the CURRENT row from the index and compares the new file with that baseline. The comparison logic is unchanged.
- Index missing or unreadable: message, exit 2.
- No row for the scenario: message, exit 2.
- NO_CURRENT: message, exit 3.
- The CURRENT file missing: message, exit 2.
- Never falls back to a retired file. Never guesses.

A helper module, `experiments/cc_bench_index.py`, does the index reading and writing. Both the comparison tool and the regeneration script use it.

Tests (no simulation):
- `python experiments/cc_bench_compare.py manual data/oomd_check_20261103/old.csv`: the current baseline against the Nov 3 output. Result: both have 84 data rows, the maximum sim-time difference is 0, and every column's maximum difference is 0 (sum of all maximum differences: 0). Exit 0.
- `python experiments/cc_bench_compare.py teleop data/oomd_check_20261103/old.csv`: exit 3, with the message that the scenario has no CURRENT baseline.
- `python experiments/cc_bench_compare.py manual data/oomd_check_20261103/old.csv --index /nonexistent/INDEX.csv`: exit 2, with the message that the index is missing.
- `python experiments/cc_bench_compare.py nosuch ...`: the argument parser refuses the scenario name.

One problem found during the tests: the first version of the "NO_CURRENT / missing index" code exited with code 1 instead of 2 or 3, because a message passed to `sys.exit` always gives 1. It was fixed (the message is printed, then the code is exited) before the tests above were run.

Comments in `experiments/cc_bench_scenarios.sh` (lines 4 and 5) that named the old file were replaced by text that points to the index and to the regeneration script. No code in that script changed.

## 6. Decision log and progress (step 6)

- `docs/mpvi_decisions.md`: row **M242** (2026-11-03), "Baseline policy (David, 2026-11-03)". Next free number was M242: nothing above M241 is used in docs, src, experiments, README, CLAUDE.md or scripts.
- `docs/progress.md`: one progress entry, at the end of the file. Before editing, `git diff` showed no uncommitted changes of yours in either file.

## 7. Commit and verification (step 7)

Staged by explicit path, then one commit: **`21721ad`**. The staged set (checked with `git diff --cached --name-status` before committing):

```
M  docs/mpvi_decisions.md
M  docs/progress.md
A  experiments/baselines/INDEX.csv
A  experiments/baselines/README.md
R  experiments/baselines/manual_reference.csv -> experiments/baselines/retired/manual_reference.csv
R  experiments/baselines/policy_reference.csv -> experiments/baselines/retired/policy_reference.csv
R  experiments/baselines/teleop_reference.csv -> experiments/baselines/retired/teleop_reference.csv
M  experiments/cc_bench_compare.py
A  experiments/cc_bench_index.py
M  experiments/cc_bench_regen.sh
M  experiments/cc_bench_scenarios.sh
```

Verification after the commit:
- SHA-256 of each moved file equals its step 1 hash (all three match; see section 1).
- `git diff --stat`: 66 files changed, 50 insertions, 69 deletions. The same as before this task, so your uncommitted hunks are unchanged.
- `git status --porcelain`: 84 entries (17 untracked, 67 modified). These are your uncommitted hunks and untracked reports, the same set as before this task. (The number 83 that appeared earlier in this session was taken before some of the reports were written; it is not a change in your hunks.)

## Commands run

Read-only checks:
```
sha256sum experiments/baselines/*
git ls-files experiments/baselines
git check-ignore -v experiments/baselines/manual_reference.csv   # not ignored
git log --follow --format='%h %ad %s' --date=short -- <file>
git hash-object <file>; git rev-parse --short=7 <commit>:<file>  # blob match
git diff -- <each file I edit>                                    # empty before editing
```

Moves: as in section 2.

Index: written with `cc_bench_index.write(rows)` (the helper's own writer, atomic temporary file then rename); then read back with `cc_bench_index.py current manual` (prints `manual_reference_20261103.csv`, exit 0), `... current teleop` (exit 3), `... current nosuch` (exit 4), and a missing path (raises).

Tests: as in section 5 and section 4.

Syntax checks: `bash -n experiments/cc_bench_regen.sh`, `bash -n experiments/cc_bench_scenarios.sh`, `python -m py_compile` on the two Python files.

Commit: `git add -- <paths>` for the files listed above, then `git commit`.

## NOT BUILT, needs decision

1. **The regeneration script was not run for real.** Its real path (creating the worktree, linking the mesh folder, running the scenario twice, the byte check, writing the files, the `git mv`, and the index update) is untested. Only `--dry-run` and the syntax check were run, as the rule said. The first real run should be a deliberate one, with you present, because it runs the simulation and changes the repo state.
2. **The earlier versions of the old references are not in the index.** Git history has earlier versions of these files. `manual_reference.csv` was written by five earlier commits (db017b8, fe8c3d8, 007eddd, ff42a26, 5d57b51) before the current version from 4540b51. `teleop_reference.csv` and `policy_reference.csv` were written by the same five earlier commits (not checked blob by blob) before the current version from ac64f22. Their files are not on disk, so I did not add rows for them. Decide whether they should get RETIRED rows that point to git commits.
3. **Other copies of baselines in `data/` (git-ignored) were not moved.** These are `data/cc_baseline/` (including `regen/`, with `manual_1.csv`, `manual_2.csv`, `teleop_1.csv` and so on) and `data/baselines_tmp*`. They are outputs of old regeneration runs, not files in `experiments/baselines/`. Decide whether to move or delete them.
4. **The note beside the new baseline is now out of date.** `experiments/baselines/manual_reference_20261103.README.txt` says the decision of which file the tests read "has not been made yet". M242 now records it: the tools read the CURRENT row. I did not edit the note, because it describes what was true when it was written.
5. **Teleop and policy have no current baseline.** Not regenerated, as decided. The comparison tool stops for them until one is made.
6. **Old documents that name the old files.** Dated reports and progress entries (for example `docs/reports/baseline_regen_20261103.md`, which lists the old scripts' behaviour) describe the state when they were written. They were not edited.
7. **The worktree setup uses the main repo's mesh folder.** This is the same as in the earlier runs. The regeneration script links it in (it does not copy it). It was not tested in a real run (see item 1).
8. **Uncommitted reports.** `docs/reports/baseline_regen_20261103.md` and this report are both uncommitted, as asked.
