# New manual bench baseline (2026-11-03)

Written for: David (repo owner). The new baseline is in the repo. The old file is untouched. This report is left uncommitted.

## Summary

- **New file:** `experiments/baselines/manual_reference_20261103.csv` (85 rows, 23 columns, 23930 bytes), with a note beside it: `experiments/baselines/manual_reference_20261103.README.txt`.
- **Commit:** `8f10aac`, "Baseline: new manual bench reference from e127479 (manual_reference_20261103.csv)". It contains exactly the three files: the CSV, the README note and `docs/progress.md`. Not pushed.
- **Made from:** commit `e127479` (the records commit), run in a clean worktree of that commit.
- **Two runs matched:** yes. The two runs from e127479 are byte-identical (same SHA-256 `3060ae41cc8518e65cad011c6f48e25c273d8a105609c63e5c41a42f6c4c9b82`).
- **Matches HEAD's earlier output:** yes. The output is byte-identical to the earlier HEAD run (`1056ab6`, in `/tmp/drift_out2/csv/40_1056ab6.csv`) and to the Nov 3 main-tree output (`data/oomd_check_20261103/old.csv`). The records commit only changed docs, as expected.
- **Old file:** `experiments/baselines/manual_reference.csv` is unchanged. SHA-256 before and after: `a2323325656c3d4106de8eaed52600962dc2bbae4cc1ae1ebade1fa44f65f5f3`.
- **Readers:** no code reads the baseline file. Only two comments, a writer script that would overwrite it if run, and a general rule in CLAUDE.md mention it. Details in section 5. Which file the tests should read is your decision.

## 1. Commands run

Pre-checks (read only):
- `sha256sum experiments/baselines/manual_reference.csv` (saved to `/tmp/old_baseline_before.sha256`)
- `git ls-files experiments/baselines`: the three reference files are tracked by git.
- `git check-ignore -v experiments/baselines/manual_reference.csv`: not ignored.
- `git diff --stat -- docs/progress.md`: empty before the edit.

Clean worktree (outside the repo):
```
git worktree add /tmp/baseline_wt e127479
ln -s /home/vboxuser/projects/openarm-tracer-sim/.venv /tmp/baseline_wt/.venv
rmdir /tmp/baseline_wt/deps/OpenArm-Combined
ln -s /home/vboxuser/projects/openarm-tracer-sim/deps/OpenArm-Combined /tmp/baseline_wt/deps/OpenArm-Combined
mkdir -p /tmp/baseline_out
```
The venv link and the mesh link are needed because the venv and the robot meshes are git-ignored or submodule content, as in the earlier investigation. `git status` inside the worktree prints an error ("expected submodule path 'deps/OpenArm-Combined' not to be a symbolic link"). This is caused by the mesh link, and it does not affect the run. It is a side effect of the setup, not a problem with the commit.

Run 1 (in the worktree):
```
flock /home/vboxuser/projects/openarm-tracer-sim/data/locks/mujoco.lock \
  bash experiments/run_capped.sh /tmp/baseline_out/run1.log \
  env CC_HEADLESS=1 bash experiments/cc_bench_scenarios.sh manual /tmp/baseline_out/run1.csv
```
Result: exit 0 at 14:52:12. 85 lines.

Run 2 (same command, output `/tmp/baseline_out/run2.csv`, log `/tmp/baseline_out/run2.log`): exit 0 at 14:52:25.

Comparisons:
- `cmp run1.csv run2.csv`: identical.
- `cmp run1.csv /tmp/drift_out2/csv/40_1056ab6.csv`: identical.
- `cmp run1.csv data/oomd_check_20261103/old.csv`: identical.
- `cmp run1.csv experiments/baselines/manual_reference.csv`: differs (the stale Oct 22 file, expected; first difference at byte 292, line 2).
- `.venv/bin/python -I /tmp/drift_out/tools/cmp_csv.py run1.csv run2.csv`: both files 23930 bytes, 85 rows, 23 columns.

Settings, as written in `experiments/cc_bench_scenarios.sh` (manual case). No environment overrides were added beyond `CC_HEADLESS=1`, which the script itself requires for headless runs: `CC_BENCH_FRAMES=1248`, `CC_BENCH_KEYS=12-108:UP,108-228:SPACE,240-336:LEFT,336-456:SPACE,480-624:PAGE_DOWN,648-792:PAGE_UP,816-888:3,936-1008:E,1032-1128:A,1152-1188:8,1200-1248:I`, `CC_INSETS=0`, `CC_HUD=0`, and the program `experiments/combined_controller.py`. The defaults in the code (gravity feedforward on, command bridge on) apply.

Copy into place (step 3):
```
cp /tmp/baseline_out/run1.csv experiments/baselines/manual_reference_20261103.csv.tmp
mv experiments/baselines/manual_reference_20261103.csv.tmp experiments/baselines/manual_reference_20261103.csv
```
The temporary file and the rename are the same pattern the project's regen script uses, so an interrupted copy does not leave a half-written file.

Note written (step 4): `experiments/baselines/manual_reference_20261103.README.txt`. It has the commit, the date, the command and settings, the two-run result, the size, the reasons (M185, M241), the relation to the old file, and the note that further changes will need deliberate regeneration.

Progress entry (step 6): `docs/progress.md`, two lines added at the end. `git diff` before the edit showed no changes of yours in that file.

Commit (step 7):
```
git add -- experiments/baselines/manual_reference_20261103.csv \
  experiments/baselines/manual_reference_20261103.README.txt docs/progress.md
git commit -q -m "Baseline: new manual bench reference from e127479 ..."
```
Commit `8f10aac`, three files, 124 insertions. Checked before committing: none of the three is git-ignored (`git check-ignore`).

Cleanup (step 8):
```
rm /tmp/baseline_wt/.venv
rm /tmp/baseline_wt/deps/OpenArm-Combined
git worktree remove --force /tmp/baseline_wt
git worktree prune
```
The two links were removed first, so the removal cannot touch the main repo's venv or meshes. Both were still there afterwards.

## 2. Comparison with HEAD's earlier output (step 2)

They match, byte for byte. The records commit changed only docs, so the scenario's output could not change, and it did not. This is also the same output as the Nov 3 main-tree run that started the investigation.

## 3. Checks after cleanup (step 8)

- `git status --porcelain | wc -l`: 83. Before this task it was 83 (your 82 uncommitted entries plus the records report from the last task). The commit removed the three new files from the status list, and they are now tracked.
- `git diff --stat`: 66 files changed, 50 insertions, 69 deletions. Same as before. Your uncommitted hunks are unchanged.
- Old baseline SHA-256 before: `a2323325656c3d4106de8eaed52600962dc2bbae4cc1ae1ebade1fa44f65f5f3`. After: the same.
- Branch: `held-weight-ramp-sag-off`. Latest commits: `8f10aac` (this baseline), then `e127479` (records).

## 4. Tracked or ignored (step 5, first part)

The baselines folder is tracked by git. `git ls-files experiments/baselines` lists `manual_reference.csv`, `policy_reference.csv` and `teleop_reference.csv`, and none of the new files is ignored. The data folder (`data/`) is the ignored one.

## 5. Places that read or write the manual reference (step 5)

Searched the repo (tests, experiments, scripts, docs, CLAUDE.md, README), excluding `data/`, `deps/` and `.venv/`. I edited none of them.

**Code and scripts**

1. `experiments/cc_bench_scenarios.sh`, line 5: a comment, "Compare a new run with: python experiments/cc_bench_compare.py experiments/baselines/manual_reference.csv <csv-out>". It tells a reader to compare against the old file. It does not read the file.
2. `experiments/cc_bench_scenarios.sh`, line 4: a comment, "Reference results on the imported robot (B1c, 2026-10-18 ...): experiments/baselines/<scenario>_reference.csv". It names the file pattern, not a reader.
3. `experiments/cc_bench_regen.sh`, line 2: a comment, "Regenerates the reference baselines ... (experiments/baselines/<scenario>_reference.csv)".
4. `experiments/cc_bench_regen.sh`, line 16: **a writer.** `cp ... experiments/baselines/${sc}_reference.csv.tmp && mv ... experiments/baselines/${sc}_reference.csv`. For the scenario `manual` this would overwrite `manual_reference.csv`. The script also regenerates `teleop` and `policy`. Running it would change the old file. It has not been run in this task.
5. `experiments/cc_bench_compare.py`, lines 1 to 5: the usage text, "python experiments/cc_bench_compare.py before.csv after.csv". It takes the file names as arguments and does not have a default path.
6. `experiments/cc_loop_tests.py`, line 10: a comment that mentions `cc_bench_compare.py`. It does not name the baseline file.

**Rules and documents**

7. `CLAUDE.md`, line 58: "Existing pipeline output must stay byte-identical unless a change is decided: check the baselines (`experiments/baselines/`, `data/cc_baseline/`) before and after." It names the folder, not the file.
8. `docs/reports/*` (the reports from Oct 20 to Nov 3 that mention `manual_reference.csv`, including `docs/reports/20261103_1341_oomd_exempt_run_capped.md` line 17 and the records and drift reports): historical mentions, not readers.

**Not found:** no test in `experiments/run_tests.sh` and no `*_test.py` reads any `*_reference.csv` file. The automated suite does not compare the scenario output with any baseline. I did not check every test for indirect reads, only by the names and paths in the search above.

## NOT BUILT, needs decision

1. **Which baseline the tests and scripts should read.** Decide between the old Oct 22 file and the new `manual_reference_20261103.csv`. Also decide whether `cc_bench_scenarios.sh` line 5 and `cc_bench_compare.py` should point at the new file. Not changed.
2. **`experiments/cc_bench_regen.sh`.** It overwrites `manual_reference.csv` (and the teleop and policy references). Decide whether it should write dated files like the new one instead. Not changed, and it has not been run.
3. **The teleop and policy baselines.** Not regenerated. The task covered the manual scenario only, and the bridge and gravity changes may affect them too. Decide whether to make new ones.
4. **Retiring the old file.** `experiments/baselines/manual_reference.csv` is kept, as asked. Decide later whether to delete it or keep it as history.
5. **The `git status` error inside a worktree** (the mesh link is a symbolic link where git expects a submodule). It does not affect runs. Noted here only; nothing to do unless you want the worktree setup changed.
