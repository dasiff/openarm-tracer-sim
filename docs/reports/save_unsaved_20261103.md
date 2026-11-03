# Saving the unsaved work, and the label removal from the baseline scripts (2026-11-03)

Written for: David (repo owner). This report is left uncommitted, so it will show as untracked in `git status`. Nothing was run (no simulation, no script). Nothing was pushed. No stash, reset, checkout or revert was used. `src/control_loop.py` was not touched.

## Summary

Four commits, all local, none pushed:

| Commit | What it saves | Files |
|---|---|---|
| `902da63` | Ten reports under `docs/reports/`, and a progress entry | 11 (10 reports and `docs/progress.md`) |
| `173cfac` | Three diagnostic scripts under `scripts/diag/`, with the label removed | 3 |
| `884e138` | The lift-off launcher `experiments/bounce_liftoff_run.py`, with its read-only logging | 1 |
| `f4f5564` | Removes the label line from the three baseline scripts that commit `21721ad` added | 3 |

**Still unsaved:**
- `src/control_loop.py` (+7 -4). Left as it was, as asked. Its label removal and its logging code go in a later commit.
- Left out of commit 1 because of the secret-search rule (section 2): six reports, `docs/reports/baseline_drift_20261103.md`, `docs/reports/baseline_drift_part2_20261103.md`, `docs/reports/baseline_index_20261103.md`, `docs/reports/baseline_regen_20261103.md`, `docs/reports/marker_cleanup_20261103.md` and `docs/reports/uncommitted_groups_20261103.md`. Each contains a path outside the repo.
- Not saved, as decided: `measstart`, `.claude/settings.local.json`, `compare_nosag.py`, `extract_height_data.py`, and the submodule `deps/AgileX_Tracer`.
- This report.

## 1. State at the start (step 1)

Saved with `git status --porcelain` and `git diff --stat` (`/tmp/su_status_start.txt`, `/tmp/su_diffstat_start.txt`). It matched the expected state:
- `src/control_loop.py`: +7 -4.
- `experiments/bounce_liftoff_run.py`: +43 -1.
- Submodule: `M deps/AgileX_Tracer`.
- Untracked: the 16 reports under `docs/reports/`, the three diagnostic scripts (under `scripts/`), `compare_nosag.py`, `extract_height_data.py`, `measstart`, and `.claude/`.
- Nothing was staged. HEAD was `57f84bf`.

Terms: an **untracked** file is one that git does not yet track. The **submodule** is `deps/AgileX_Tracer`, a separate git repository inside this one.

## 2. Secret search before saving (step 2)

Every file to be saved was checked: the 16 reports, the three diagnostic scripts and the launcher (20 files). The sizes are in section 2.1.

### 2.1 Sizes

All 20 files are under 1 MB. The largest is `docs/reports/marker_cleanup_20261103.md` at 50,875 bytes. No file was left out for size.

### 2.2 What was searched for

1. **Words:** `api key` (also `apikey`, `api_key`), `secret`, `password`, `passwd`, `token`, `bearer`, `private key`, `BEGIN ... PRIVATE`, `key=`, `GEMINI_API`, `secrets.env`. Case-insensitive.
2. **Key formats:** text starting `AIza` (a Google key), and `sk-` (a common secret-key prefix), each followed by many characters.
3. **E-mail addresses:** any text shaped like `name@domain.tld`.
4. **Long random-looking strings:** 40 or more letters, digits or symbols, with no `/` (to skip file paths).
5. **The actual Gemini key value:** the project's secrets file (`~/.config/openarm-tracer-sim/secrets.env`, outside the repo) was read by a check script, which printed only a yes or no. The key value was not printed, and it does not appear in any candidate file.
6. **Paths outside the repo:** any text starting `/home/vboxuser` (other than the repo's own path `/home/vboxuser/projects/openarm-tracer-sim`), `/tmp/`, `/Users/` or `C:\`.

### 2.3 Results

| Check | Result |
|---|---|
| Words (item 1) | Two lines, both in `scripts/diag/check_bridge_identity.py`: `key=lambda` in ordinary Python code. **False positives, not secrets.** The file is saved. |
| Key formats (item 2) | No hits. |
| E-mail addresses (item 3) | No hits. |
| Long random strings (item 4) | No hits. The 64-character SHA-256 checksums in the reports are public file fingerprints, not secrets. |
| Gemini key value (item 5) | Not present in any candidate file. |
| Paths outside the repo (item 6) | **Six reports have hits** (table below). The launcher's only path is the repo's own location, which is inside the repo and is already in HEAD. It passes. |

Reports left out for paths outside the repo:

| Report | Hits | What the hits are |
|---|---|---|
| `baseline_drift_20261103.md` | 8 | Scratch paths under `/tmp/drift_out`, `/tmp/drift_wt` |
| `baseline_drift_part2_20261103.md` | 18 | Scratch paths under `/tmp/drift_out2`, `/tmp/drift_out` |
| `baseline_index_20261103.md` | 2 | Scratch paths under `/tmp` |
| `baseline_regen_20261103.md` | 17 | Scratch paths under `/tmp` |
| `marker_cleanup_20261103.md` | 10 | Scratch paths under `/tmp` |
| `uncommitted_groups_20261103.md` | 7 | Includes a **personal path**: `/Users/dasif/Downloads/uncommitted_inventory_20261103.md` (a Windows user folder) |

**My judgement, flagged for you:** the first five contain only scratch paths under `/tmp`. Those contain no personal information. The last one contains a personal Windows user path, which is private. Your rule says to leave out any file with a hit, so I left all six out. Section 6 asks whether the `/tmp` paths should count.

## 3. Commit 1: the reports (step 3)

**Staged (10 reports, passed the check):**
- `docs/reports/20261102_1220_logging_step5_ticks_11598_11600.md`: the logging extract for ticks 11598 to 11600.
- `docs/reports/20261102_1300_logging_step6.md`: the logging extract, steps 0 to 6.
- `docs/reports/20261102_1358_bridge_identity.md`: the bridge identity check.
- `docs/reports/20261102_1431_held_weight_facts.md`: facts about held-weight compensation.
- `docs/reports/command_chain_trace_20261101.md`: trace of the command chain during the place skill.
- `docs/reports/nan_recheck_20261101.md`: recheck of the NaN search.
- `docs/reports/nan_search_20261101.md`: search for NaN values in earlier runs.
- `docs/reports/records_update_20261103.md`: the records update for the command bridge default.
- `docs/reports/trace_verification_20261101.md`: checks of the command-chain trace.
- `docs/reports/uncommitted_inventory_20261103.md`: the list of uncommitted files from earlier today.

**Also staged:** `docs/progress.md`, with one entry that says which reports were saved and which were left out, and why.

**Left out:** the six reports in section 2.3.

**Check before commit:** `git diff --cached --name-only` listed exactly these 11 files. `docs/progress.md` had no uncommitted changes of yours before the edit, so the whole file was safe to stage.

**Commit:** `902da63`, "Save 10 reports (logging, NaN, bridge, held-weight and trace checks, Nov 1 to 3) and the uncommitted inventory".

## 4. Commit 2: the diagnostic scripts (step 4)

Each of the three files had the label as whole-line comments, or as a comment at the end of a code line. There were no bare-text labels. I removed the label text only:
- Whole-line label comments were deleted.
- End-of-line label comments lost only the comment text. The code on that line stays.

| File | Label lines removed (whole line) | End-of-line label text removed | Lines before | Lines after |
|---|---|---|---|---|
| `scripts/diag/check_bridge_identity.py` | 58 | 0 | 146 | 88 |
| `scripts/diag/compare_runs.py` | 19 | 0 | 90 | 71 |
| `scripts/diag/verify_20261101.py` | 22 | 94 | 127 | 105 |

**Checks, per file:**
- **Only label text differs.** `git diff --no-index --numstat` against the original (kept in `/tmp/su_orig/`). `check_bridge_identity.py`: 0 added, 58 removed. `compare_runs.py`: 0 added, 19 removed. `verify_20261101.py`: 94 lines changed (the same code with the trailing label removed), 22 removed.
- **Code unchanged.** The syntax tree (Python's `ast` module, comments ignored) of each new file is identical to the original.
- **Compiles.** `py_compile` into a temporary folder (`/tmp/su_compile/`). All three compile.
- **Not run.**

**Stage and commit:** `scripts/diag/check_bridge_identity.py`, `scripts/diag/compare_runs.py`, `scripts/diag/verify_20261101.py`. `git diff --cached --name-only` listed exactly these three. Commit `173cfac`.

## 5. Commit 3: the lift-off launcher (step 5)

**The complete diff** of `experiments/bounce_liftoff_run.py` (shown in full with `git diff`). It has four hunks:
1. **One line added** in `_ids()`: the two finger bodies of the left hand, looked up once.
2. **A new function `_beaker_forces()`** (27 lines added). It reads the contacts on the beaker and returns three numbers (table force, finger force, number of finger contacts). It only reads.
3. **Thirteen lines added** in the logging wrapper `torques_logged()`. They compute the table and finger forces, the beaker's weight, the seven arm actuator forces and bias forces, and the arm's vertical pad force, and write them into the row. They only read.
4. **The column list** gets the new columns added.

**What the removed line is.** The only removed line is the closing part of the column list: `+ [f"C_p_{a}" for a in "xyz"] + [f"C_r_{a}" for a in "xyz"] + ["frozen", "trace_ok"])`. It is changed to continue onto a new line that adds the new columns. It is not the label. The file has no `Code generated by Claude` text, in HEAD or in the working copy.

**Does anything else use it?** No file under `src/` imports it. It is run only as a script. The file was not changed in this task.

**Stage and commit:** `experiments/bounce_liftoff_run.py` only. Commit `884e138`.

## 6. Commit 4: the label removal from the baseline scripts (step 6)

Commit `21721ad` (mine, earlier today) added the label to, or kept it in, these files. I found them with `git show --stat 21721ad` and then checked each file for the label. `experiments/cc_bench_index.py` was already clean (its label was removed in commit `57f84bf`).

| File | Label line | Removed |
|---|---|---|
| `experiments/cc_bench_compare.py` | line 1 | yes |
| `experiments/cc_bench_regen.sh` | line 1 | yes |
| `experiments/cc_bench_scenarios.sh` | line 1 | yes |

The other files in `21721ad` do not carry the label: `experiments/baselines/INDEX.csv`, `experiments/baselines/README.md`, the three moved baseline files under `retired/`, and `experiments/cc_bench_index.py`.

**Checks:**
- Only line 1 removed in each file (`git diff --no-index --numstat`: 0 added, 1 removed).
- `experiments/cc_bench_regen.sh`: `bash -n` passes. `experiments/cc_bench_scenarios.sh`: `bash -n` passes.
- `experiments/cc_bench_compare.py`: `py_compile` passes, and its syntax tree is identical to the original.
- None of the three was run.

**Stage and commit:** the three files by explicit path. `git diff --cached --name-only` listed exactly these three. Commit `f4f5564`.

The four label lines in `docs/` (the rule text in `CLAUDE.md` and `docs/design_plan.md`, the decision M243, and the progress entries) are text about the rule, not code. They were not changed.

## 7. The loose files and measstart (step 7). Nothing changed, nothing committed.

**`compare_nosag.py`** (5,293 bytes, Nov 1, 17:27). A comparison script for the bounce task. It compares an experiment run in which the sag correction (a small height adjustment during the place) is frozen against the baseline run. It checks that the two runs match up to tick 11,599, then reports where they first differ and the height values. It imports `extract_height_data.py` (line 18), so the two files must be kept together. Nothing else imports it. It was made on Nov 1 from the same investigation as `extract_height_data.py`.

**`extract_height_data.py`** (8,795 bytes, Nov 1, 17:16). It reads the per-tick logs and extracts arm and pad height data around the place handoff (ticks 11,560 to 11,800). It checks its own column choices against raw values and stops if a check fails. Nothing imports it except `compare_nosag.py`. It was made on Nov 1 for the same investigation.

**`measstart`** (16,102,728 bytes, Nov 1, 20:32). A CSV with one header line and 13,603 data rows, 139 columns: per-tick joint positions, velocities, targets and forces from the "measstart" diagnostic run.

- **Checksum:** `c759658291069ac3ed0e990c8ee73cafda2a75cefd1827f5b4653a70f04f9fad` for both files. `cmp` reports them identical.
- **Its copy:** `data/diag/measstart_20261101/measstart_ticks.csv`. The data folder is git-ignored, so this copy is not in git.

## 8. Verification (step 8)

- **`git status --porcelain`:**
  - Tracked, uncommitted: `M src/control_loop.py` and `M deps/AgileX_Tracer` (the submodule). The only uncommitted tracked change in the main repo is `src/control_loop.py`.
  - Untracked, left: `.claude/`, `compare_nosag.py`, `extract_height_data.py`, `measstart`, the six reports from section 2.3, and this report.
- **`git diff --stat`:** 1 file changed, 7 insertions, 4 deletions. That is `src/control_loop.py`, as expected.
- **The four commit hashes:**
  - `902da63`: the reports and the progress entry.
  - `173cfac`: the diagnostic scripts.
  - `884e138`: the lift-off launcher.
  - `f4f5564`: the baseline script label removals.
- **Pushed:** nothing. Local commits only.
- **Staged:** nothing is staged.
- **`~/backups/`:** not changed.

## 9. Exact commands run

Read-only checks:
```
git log --oneline -1
git status --porcelain > /tmp/su_status_start.txt
git diff --stat > /tmp/su_diffstat_start.txt
git diff --numstat
git ls-files --others --exclude-standard docs/reports      (the candidate list)
stat -c '%s %n' <files>                                    (sizes; check for 1 MB)
grep -n -I -i -E "<secret words>" <files>                  (item 1 of section 2.2)
grep -n -I -E "[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}" <files>
grep -n -I -o -E "[A-Za-z0-9+/_=-]{40,}" <files>           (then filtered: no "/")
grep -n -I -o -E "(/home/vboxuser|/Users/|/tmp/|C:\\\\)[^ )'\"|,;]*" <files>   (then filtered: not the repo path)
.venv/bin/python -I (secret-value check: prints yes/no only)
git diff --numstat -- docs/progress.md                      (empty: clean)
git diff -- experiments/bounce_liftoff_run.py              (full diff)
git show --stat --format= 21721ad
grep -c -i "generated by claude" <files>
git show HEAD:experiments/bounce_liftoff_run.py | grep -c -i "generated by claude"
sha256sum measstart data/diag/measstart_20261101/measstart_ticks.csv
cmp measstart data/diag/measstart_20261101/measstart_ticks.csv
grep -n "^import\|^from" compare_nosag.py extract_height_data.py
```

Edits:
- Progress entry: appended to `docs/progress.md` (commit 1).
- Diagnostic scripts: a Python script removed the label text (whole-line comments deleted; end-of-line comment text stripped). Originals kept in `/tmp/su_orig/`.
- Baseline scripts: a Python script removed line 1 (the label) from each of the three files. Originals kept in `/tmp/su_orig6/`.

Checks:
```
git diff --no-index --numstat /tmp/su_orig/<file> scripts/diag/<file>
ast.dump comparison (Python, comments ignored)
.venv/bin/python -I -c "py_compile.compile(<file>, cfile=/tmp/su_compile/..., doraise=True)"
bash -n experiments/cc_bench_regen.sh ; bash -n experiments/cc_bench_scenarios.sh
```

Staging and commits, each with an explicit list of paths, and each checked with `git diff --cached --name-only` first:
```
git add -- <10 reports> docs/progress.md        -> 902da63
git add -- scripts/diag/check_bridge_identity.py scripts/diag/compare_runs.py scripts/diag/verify_20261101.py  -> 173cfac
git add -- experiments/bounce_liftoff_run.py    -> 884e138
git add -- experiments/cc_bench_compare.py experiments/cc_bench_regen.sh experiments/cc_bench_scenarios.sh   -> f4f5564
```

Not used: `git add -A`, `git add .`, `git commit -a`, stash, reset, checkout, revert, push, and any simulation or script run.

## NOT BUILT, needs decision

1. **The six reports left out** (section 2.3). Decide:
   - Whether scratch paths under `/tmp` count as private. If not, the first five can be saved as they are.
   - For `uncommitted_groups_20261103.md`, which contains a personal Windows path: remove or replace that path, then save. I did not edit it.
2. **`compare_nosag.py` and `extract_height_data.py`.** Not saved, as you asked. They are a pair (the first imports the second). Decide whether to keep them (move into `scripts/diag/`), or delete them.
3. **`measstart`** (16 MB at the repo root). A copy exists in `data/diag/measstart_20261101/`. Decide whether to delete the root copy. Not done.
4. **`.claude/settings.local.json`.** Not saved. It is not ignored by `.gitignore`. Decide whether to add it to `.gitignore`. Not done.
5. **`src/control_loop.py`** (+7 -4). Still uncommitted, as asked. It goes in the later logging commit with its label removal and the logging code.
6. **The label text that remains** in other code files (from the earlier report `docs/reports/marker_cleanup_20261103.md`, section 7): `experiments/` (about 140 places), `tools/`, `legacy/`, the XML files in `models/`, and the root. Not changed, as they are outside this task. Decide later.
7. **The launcher's commit message** says it is the logging for the Nov 3 preload run. That is taken from the report `docs/reports/20261103_0300_liftoff_run1_breakaway_preload_memory.md`. I did not check the run's numbers, because nothing was run.
8. **This report** is uncommitted, so it will show as untracked. Decide whether it should be saved.
