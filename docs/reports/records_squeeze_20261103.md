# Records: squeeze signal decision, reports saved, deps check (Nov 3, 2026)

Documentation and read-only checks only. No code changed, no simulation run, nothing pushed, nothing under `deps/` edited or staged.

## deps/AgileX_Tracer, in a few sentences (read-only)

The submodule is at its recorded commit: `9cb44e6`, the same hash as the gitlink in the main repo index, so the commit is unchanged. There are no tracked changes and no stash. What makes it show up is untracked content: nine `.pyc` files (Python bytecode caches) in three folders, `my_cartographer_launch/test`, `my_nav2_pkg/test` and `wit_ros2_imu/test`, all dated 2026-10-22. These caches are not ignored, because `deps/AgileX_Tracer` has no `.gitignore` of its own, and the main repo's `.gitignore` (which has `__pycache__/`) does not apply inside a separate repository. Git reports a submodule with untracked content as modified, so the main repo shows it as changed, not as an untracked path. The nested repos (`tracer_ros2`, `ugv_sdk/test/googletest`) are clean for git: `tracer_ros2` tracks its own cache file, and the `googletest` cache is ignored by its `.gitignore` line 12 (`*.pyc`). I did not change anything.

## Deps details

- `git submodule status` (main repo): `9cb44e6f58ee5e846e3a6d3e79eacdef615c0205 deps/AgileX_Tracer (heads/main)`, with no `+` or `-` prefix, so the checked-out commit matches the recorded one.
- `git -C deps/AgileX_Tracer status --short`: three untracked folders (`??`). With `--untracked-files=all` they list nine files, all `.pyc`:
  - `my_cartographer_launch/test/__pycache__/` (3 files: `test_copyright`, `test_flake8`, `test_pep257`, `cpython-312-pytest-9.1.1.pyc`)
  - `my_nav2_pkg/test/__pycache__/` (same 3 names)
  - `wit_ros2_imu/test/__pycache__/` (same 3 names)
- Recorded commit: `git ls-files -s deps/AgileX_Tracer` gives `160000 9cb44e6f... 0`. The submodule's `git rev-parse HEAD` is `9cb44e6f...`. Same, so unchanged. The submodule's last commit is `9cb44e6 Revise README for repository URL and launch instructions`. Stash list is empty.
- `.gitmodules`: three submodules are listed (`OpenArm-Combined`, `openarm_mujoco`, `AgileX_Tracer`). The submodule's git data is in `.git/modules/deps/AgileX_Tracer` (its `config` names nested submodules, including `tracer_ros2` and `ugv_sdk`). The submodule's `.git` file points there.
- `__pycache__` coverage:
  - In `deps/AgileX_Tracer` itself: **not covered**. There is no `.gitignore` and no `info/exclude` rule; `git check-ignore` returns no match for the cache path.
  - Also present: `wit_ros2_imu/launch/__pycache__/` has one file that is tracked in the submodule (`rviz_and_imu.launch.cpython-38.pyc`).
  - In nested repos: `tracer_ros2/f710_teleop_config/launch/__pycache__/` is tracked there; `ugv_sdk/test/googletest/googletest/...` and `googlemock/test/...` (7 files) are ignored by `googletest/.gitignore` line 12 (`*.pyc`).
- Why the main repo shows it as changed (not untracked): the main index has a gitlink (a pointer to a commit) for `deps/AgileX_Tracer`, so git treats the path as a submodule, not as an untracked folder. The porcelain v2 line for the path is `1 .M S..U ... deps/AgileX_Tracer`: the commit is unchanged (`.`), no tracked file changed (`.`), and untracked content is present (`U`). The short listing printed `?` for this path in my runs, while the per-path query printed ` M`. I could not explain that difference, so it is listed under NOT BUILT.
- When these caches were written: the files are dated 2026-10-22 16:20 (and the one tracked file 2026-09-23). They are not from this session. The start-of-session status snapshot did not show `deps/AgileX_Tracer`; why it appears now is **unknown**.

## Added text in the decision log

Under M246 item 1, in `docs/mpvi_decisions.md`, a dated sub-bullet. Item 1's own text is unchanged.

> **ADDENDUM 2026-11-03 (squeeze signal; decided by David, Nov 3, 2026):** The finger squeeze signal for the holding rule is the gripper's own command or effort force: the actuator output force in the simulator, standing in for a real gripper's motor current or effort reading. It is NOT pad contact force, because a real robot may not have contact pads. Pad contact forces stay simulator-only diagnostics; the holding rule must not depend on them.
> - **Known** (from the three bounce runs, `left_finger_force_N` in `episode_log.csv`): it sits at the 8 N cap through the whole hold and does not change with the beaker's mass.
> - **Not yet known:** what it reads with the gripper open around the beaker, or when closing on nothing. To be logged in the first staged step. Which signal the real OpenArm gripper reports: unknown.
> - **Still NOT decided:** the closure range, the squeeze threshold, the confirming time, the stillness limits, and whether the staged approach (item 4) is confirmed.

Progress entry: one new entry at the end of `docs/progress.md`, in the same format as the others.

## Commits

- **Reports commit: `610437c`**, "Save the Nov 3 records and signal survey reports". Files: `docs/reports/records_positions_20261103.md` (was untracked, 105 lines) and `docs/reports/signal_survey_20261103.md` (was untracked, 386 lines). No other untracked file under `docs/reports/` existed.
- **Docs commit: `8b40505`**, "Decision M246: record the squeeze-signal addendum (Nov 3); progress entry". Files: `docs/mpvi_decisions.md` (+4 lines) and `docs/progress.md` (+2 lines). Each file had no uncommitted changes before the edit, so nothing needed `git add -p`.

Both commits have the Co-Authored-By line from the attribution reminder. Neither commit was pushed.

## Secret search (reports committed in 610437c)

Same search as before, applied to both reports:

- Keys and tokens (`AIza`, `sk-`, `api key`, `secret`, `token`, `password`, `passwd`, `bearer`): `records_positions_20261103.md` has hits only in its own "Secret search" section (line 87 and line 90), where the words describe the search. No real key. `signal_survey_20261103.md` has no keyword hits.
- Gemini key from `~/.config/openarm-tracer-sim/secrets.env`: matched against both files; 0 matches in each. The key was not printed.
- E-mail addresses: none in either file.
- Long random strings (40 or more characters of letters, digits and `+/_-`): the hits are file paths (for example `docs/reports/20261103_0125_liftoff_run1_analysis`), not secrets.
- Size: both files are under 1 MB (13.9 KB and 43.5 KB).
- Files left out: none.

## NOT BUILT, needs decision

1. **Still not decided** (David): the closure range, the squeeze threshold, the confirming time, the stillness limits, and whether the staged approach (M246 item 4) is confirmed.
2. **What the gripper reads with the beaker held open, or closing on nothing:** unknown. The addendum says it is to be logged in the first staged step; no logging was added here.
3. **Which signal the real OpenArm gripper reports** (motor current, effort, or other): unknown.
4. **Why `git status` shows `deps/AgileX_Tracer` as `?` in the short listing while the per-path query shows ` M`:** not explained. The index entry is a gitlink, so git should report the submodule as modified; the `?` display is unexplained.
5. **Untracked `.pyc` caches in `deps/AgileX_Tracer`:** the submodule has no `.gitignore`, so the nine files stay visible. Whether to add a `.gitignore` in the submodule (a change under `deps/`, which this task forbids) or to exclude these paths another way is a decision for David. Not changed.
6. **Why the caches appear in the status now** and were not in the start-of-session snapshot: unknown.
7. **`docs/design_plan.md`** was not edited (the task limited edits to two docs files).
