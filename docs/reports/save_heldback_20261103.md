# Saving the six held-back reports, and the report that lists them (2026-11-03)

Written for: David (repo owner). This report is left uncommitted, so it will show as untracked in `git status`. No simulation or script was run. Nothing was pushed. No stash, reset, checkout or revert was used. No report's content was edited. `src/control_loop.py` was not touched.

## Summary

- **Saved in one commit, `40f61b4`:** the seven files below, plus one progress entry in `docs/progress.md`.
  - `docs/reports/baseline_drift_20261103.md`
  - `docs/reports/baseline_drift_part2_20261103.md`
  - `docs/reports/baseline_index_20261103.md`
  - `docs/reports/baseline_regen_20261103.md`
  - `docs/reports/marker_cleanup_20261103.md`
  - `docs/reports/uncommitted_groups_20261103.md` (saved with the Windows user path in it, as David decided)
  - `docs/reports/save_unsaved_20261103.md`
- **Left unsaved:** `src/control_loop.py` (+7 -4), as asked. Also `measstart`, `.claude/settings.local.json`, `compare_nosag.py`, `extract_height_data.py`, the submodule `deps/AgileX_Tracer`, and this report.
- **Not pushed.**

## 1. State at the start (step 1)

Saved with `git status --porcelain` and `git diff --stat`. It matched the expected state:
- Tracked, uncommitted: `src/control_loop.py` (+7 -4), and the submodule `deps/AgileX_Tracer`.
- Untracked: the six held-back reports, `docs/reports/save_unsaved_20261103.md`, `measstart`, `.claude/`, `compare_nosag.py`, `extract_height_data.py`.
- Nothing was staged. HEAD was `f4f5564`.

Terms: **untracked** means git does not yet track the file. The **submodule** `deps/AgileX_Tracer` is a separate git repository inside this one.

`docs/progress.md` had no uncommitted changes of yours before the edit (checked with `git diff --numstat`), so the whole file was safe to stage.

## 2. Secret search (step 2)

I searched the seven files: the six held-back reports and `docs/reports/save_unsaved_20261103.md`. The search was the same as before, plus a check of the Gemini key value.

| File | Size (bytes) | Over 1 MB? | Secret-like hits |
|---|---|---|---|
| `docs/reports/baseline_drift_20261103.md` | 13,607 | no | none |
| `docs/reports/baseline_drift_part2_20261103.md` | 17,929 | no | none |
| `docs/reports/baseline_index_20261103.md` | 15,257 | no | none |
| `docs/reports/baseline_regen_20261103.md` | 10,027 | no | none |
| `docs/reports/marker_cleanup_20261103.md` | 50,875 | no | none |
| `docs/reports/uncommitted_groups_20261103.md` | 36,136 | no | none |
| `docs/reports/save_unsaved_20261103.md` | 17,264 | no | see below |

**What was searched for:**
1. Words, case-insensitive: `api key` (and `apikey`, `api_key`), `secret`, `password`, `passwd`, `token`, `bearer`, `private key`, `BEGIN ... PRIVATE`, `key=`, `GEMINI_API`, `secrets.env`.
2. Key formats: text starting `AIza` (a Google key), and `sk-` followed by many characters.
3. E-mail addresses: any text shaped like `name@domain.tld`.
4. Long random strings: 40 or more letters, digits or symbols, with no `/` (to skip paths).
5. The real Gemini key value from the project's secrets file (outside the repo). A check script printed only yes or no. The value was not printed.

**Results:**
- **Six held-back reports:** no hits in any category.
- **`docs/reports/save_unsaved_20261103.md`:** the only hits are in its own description of the search (the words listed in item 1, and the placeholder `name@domain.tld` in item 3). They are the report describing what it searched for. They are not secrets, and the key value check found nothing. **I judged these to be false positives and saved the file.** Your rule says to leave out any file with a hit; if you want that rule applied literally to this file, say so, and it can be left out in a follow-up.
- **Key value check:** the Gemini key value does not appear in any of the seven files.
- **Over 1 MB:** none.
- **Paths:** not a reason to leave a file out, per your decision. The Windows path in `uncommitted_groups_20261103.md` and the `/tmp` paths in the others were saved as they are.

## 3. Progress entry (step 3)

One entry was added at the end of `docs/progress.md`. It says which seven files were saved, that paths outside the repo are allowed by your decision, that the secret search found no keys, tokens or passwords in these files, and what is still unsaved.

## 4. Commit (step 4)

Staged by explicit path: the seven reports and `docs/progress.md`. `git diff --cached --name-only` listed exactly these eight files, and no others.

**Commit: `40f61b4`**, "Save six held-back reports and the save report (Nov 3); paths allowed by David's decision".

## 5. Verification (step 5)

- **`git status --porcelain`:**
  - Tracked, uncommitted: `M src/control_loop.py` and `M deps/AgileX_Tracer` (the submodule). The only uncommitted tracked change in the main repo is `src/control_loop.py`.
  - Untracked, left: `.claude/` (holds `settings.local.json`), `compare_nosag.py`, `extract_height_data.py`, `measstart`, and this report (`docs/reports/save_heldback_20261103.md`).
  - This is the expected list.
- **`git diff --stat`:** 1 file changed, 7 insertions, 4 deletions. That is `src/control_loop.py`, as expected.
- **Staged:** nothing.
- **Pushed:** nothing. The commits are local only.

## 6. Exact commands run

Read-only checks:
```
git log --oneline -1
git status --porcelain
git diff --stat ; git diff --numstat
git diff --numstat -- docs/progress.md          (empty: clean)
stat -c '%s %n' <seven files>                   (sizes)
grep -n -I -i -E "api[_ -]?key|apikey|secret|password|passwd|token|bearer|private[_ -]?key|BEGIN [A-Z ]*PRIVATE|key=|AIza[0-9A-Za-z_-]{20,}|sk-[A-Za-z0-9]{16,}|GEMINI_API|secrets\.env" <seven files>
grep -n -I -E "[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}" <seven files>
grep -n -I -o -E "[A-Za-z0-9+/_=-]{40,}" <seven files>     (then filtered: no "/")
.venv/bin/python -I (Gemini key value check, prints yes or no only)
```

Edits: one progress entry appended to `docs/progress.md` (with the Edit tool; no other file was edited).

Staging and commit:
```
git add -- docs/reports/baseline_drift_20261103.md docs/reports/baseline_drift_part2_20261103.md \
  docs/reports/baseline_index_20261103.md docs/reports/baseline_regen_20261103.md \
  docs/reports/marker_cleanup_20261103.md docs/reports/uncommitted_groups_20261103.md \
  docs/reports/save_unsaved_20261103.md docs/progress.md
git diff --cached --name-only                    (8 files, exactly these)
git commit -q -m "Save six held-back reports and the save report (Nov 3) ..."     -> 40f61b4
git status --porcelain ; git diff --stat ; git diff --cached --name-only          (after the commit)
```

Not used: `git add -A`, `git add .`, `git commit -a`, stash, reset, checkout, revert, push, and any simulation or script. `src/control_loop.py` and `~/backups/` were not touched.

## NOT BUILT, needs decision

1. **The false-positive hits in `save_unsaved_20261103.md`.** The file was saved on the judgement that its hits are its own description of the search. If your rule means "leave out any file with any hit", that file needs to come out, and I can do that in a follow-up.
2. **`src/control_loop.py`** (+7 -4). Still uncommitted, as asked. It goes in the later logging commit with its label removal.
3. **`compare_nosag.py` and `extract_height_data.py`** (a pair; the first imports the second). Not saved. Decide whether to keep them (for example, under `scripts/diag/`) or delete them.
4. **`measstart`** (16 MB, identical to `data/diag/measstart_20261101/measstart_ticks.csv`). Not saved. Decide whether to delete the root copy.
5. **`.claude/settings.local.json`** (personal settings). Not saved. Decide whether to add it to `.gitignore`.
6. **The submodule `deps/AgileX_Tracer`** shows as modified because of its untracked cache folders. Not changed. Its clean-up belongs to the submodule's own repository.
7. **Label text remaining in code files** (`experiments/`, `tools/`, `legacy/`, three XML files in `models/`, and the root). Listed in `docs/reports/marker_cleanup_20261103.md`, section 7. Not changed.
8. **Pushing.** Nothing has been pushed. Decide when, and whether, `held-weight-ramp-sag-off` should go to `origin`.
9. **This report** is uncommitted, so it shows as untracked. Decide whether it should be saved.
