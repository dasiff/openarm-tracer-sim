# Saving src/control_loop.py (2026-11-03)

Written for: David (repo owner). This report is left uncommitted, so it will show as untracked. No simulation or script was run. Nothing was pushed. No stash, reset, checkout or revert was used. `experiments/baselines/` was not touched.

## Summary

- **Committed:** `src/control_loop.py`, as **`a57cc4f`**. The commit removes the four label lines (the label is withdrawn, M243) and adds `BRIDGE_ACCEPTED_LOG`, a diagnostic record of the command bridge's accepted target per arm. Nothing reads it for control, so the arm commands are unchanged.
- **Progress entry:** committed separately as **`29b8583`**, in `docs/progress.md`.
- **Not pushed.**
- **Left unsaved:** `measstart`, `.claude/settings.local.json`, `compare_nosag.py`, `extract_height_data.py`, the submodule `deps/AgileX_Tracer`, and this report.
- **Step 7 of the request is blank** (it ends at "7."), so I did nothing for it.

## 1. State at the start (step 1)

- Tracked, uncommitted: `src/control_loop.py` (+7 -4) and the submodule `deps/AgileX_Tracer`.
- Untracked: `.claude/`, `compare_nosag.py`, `extract_height_data.py`, `measstart`, and `docs/reports/save_heldback_20261103.md` (the report from the last task, which was not in the expected list).
- Nothing was staged. HEAD was `40f61b4`.

This matches the expected state, apart from the one report from the last task, which is new since the expected list was written.

## 2. The full diff (step 2)

`git diff src/control_loop.py` shows exactly the +7 -4 described earlier, in five hunks:

1. **Removed (line 1):** the label line above the module docstring.
2. **Removed (old line 104) and added (lines 107 to 109):** the label line above `SERVO_LOG` is removed. Added: a comment, `BRIDGE_ACCEPTED_LOG = {"tick": -1, "accepted_L": None, "accepted_R": None}` (line 108), and a blank line.
3. **Added (lines 662 to 665):** after the bridge's update of the accepted targets, four lines copy the accepted target of each arm into `BRIDGE_ACCEPTED_LOG`. The arm command `apply_cmd(...)` on the next line is unchanged.
4. **Removed (old line 921):** a label line inside `advance()`, above a logging comment.
5. **Removed (old line 1082):** a label line above the servo logging block.

Terms: the **bridge** is the command limiter on the arm targets. A **dictionary** (`dict`) is Python's key-and-value store. `BRIDGE_ACCEPTED_LOG` is a module-level dictionary that holds the last values for the diagnostic extraction.

The four removed lines are the label; the seven added lines are the record. Nothing else differs.

## 3. Timing check (step 3)

| File | Last modified |
|---|---|
| `src/control_loop.py` | Nov 2, 12:12:13 |
| `data/oomd_check_20261103/old.csv` (the Nov 3 run output) | Nov 3, 13:40:16 |

The source file is older than the run output, so the check passes and I made the commit.

## 4. Compile check (step 4)

`py_compile` on `src/control_loop.py`, with the output written to a temporary folder (`/tmp/cl_compile.*`, removed afterwards). It compiles. The file contains no label text.

## 5. Commit (step 5)

Staged by explicit path: `src/control_loop.py` only. `git diff --cached --name-only` listed only that file, and `git diff --cached --numstat` gave 7 added and 4 removed.

**Commit `a57cc4f`:** "Control loop: remove the label line; add the diagnostic record of the command bridge's accepted targets". The message says the label is removed, the record is added, and the earlier main-tree run with this change was byte-identical to the baseline.

## 6. Progress entry (step 6)

`git diff -- docs/progress.md` was empty before the edit, so the file had no changes of yours. One entry was added at the end of the file, in the existing format. Staged by explicit path: `docs/progress.md` only. **Commit `29b8583`**, separate from the control loop commit.

## 7. Verification (step 8)

- `git status --porcelain`:
  - Tracked, uncommitted: only `M deps/AgileX_Tracer` (the submodule). `src/control_loop.py` is no longer uncommitted.
  - Untracked: `.claude/`, `compare_nosag.py`, `extract_height_data.py`, `measstart`, and this report.
- `git diff --stat`: no tracked changes in the main repo.
- Staged: nothing.
- Branch: `held-weight-ramp-sag-off`, ahead of nothing I can see from the local status (`git status -sb`). I did not push.

## NOT BUILT, needs decision

1. **Step 7 of the request is blank.** I did nothing for it. Tell me what it should be.
2. **The earlier-run evidence.** The byte-identical claim in the commit message rests on the Nov 3 main-tree run (`data/oomd_check_20261103/old.csv`) and on the earlier comparison of `manual_reference_20261103.csv`, as you said. I did not re-run anything, as asked. The timing check shows only that the source was not edited after that run; it does not prove the run used this exact text.
3. **The untracked files** that are still left: `measstart`, `.claude/settings.local.json`, `compare_nosag.py`, `extract_height_data.py` (decisions from earlier, still open), and this report (not decided whether to save it).
4. **The submodule `deps/AgileX_Tracer`** still shows as modified because of its untracked cache folders. Its clean-up belongs to the submodule's own repository.
5. **The label text** that remains in other code files (listed in `docs/reports/marker_cleanup_20261103.md`, section 7) is not changed by this task.
