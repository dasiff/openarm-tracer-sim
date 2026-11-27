# Safety-filter: push check and dry-run merge into main (2026-11-25)

Git steps only. No MuJoCo process and no test was started. main, the sf-d worktree and the safety-filter-speed worktree were not changed.

## 1. Push safety-filter to origin

- Local `safety-filter` is at `be75e33`. `origin/safety-filter` is also at `be75e33`.
- `git push origin safety-filter` (no force) returned "Everything up-to-date". Nothing new was pushed, because the branch was already on origin.
- The sf-d worktree has 7 uncommitted entries (another session's work: `docs/progress.md`, `experiments/policy_stream_test.py`, three untracked reports, `.venv`). They are not in `be75e33` and are not part of this dry run.

## 2a. What main has that safety-filter lacks

- `git rev-list --count safety-filter..main` = 21.
  - 20 video-export commits, 2026-11-16 (`043441d` to `b237951`), all by David Siff.
  - Merge `05f2dc1` (2026-11-16, David Siff), which brought the D&C never-close work up to `427f766` into main.
- Merge base of main and safety-filter: `427f766`.
- Files main changed since the merge base (18):
  - docs: `design_plan.md`, `mpvi_decisions.md`, `progress.md`, two video-export reports and a contact-sheet PNG
  - src: `replay.py`, `task_config.py`, `viewer_composite.py`
  - experiments: `demo_video.py`, `search_video.py`, and 6 files under `experiments/video_export_timing/`
- Commits safety-filter has that main lacks: 191, all by David Siff, 2026-11-16 to 2026-11-25. They touch 193 files, 30 of them under `src/`.

## 2b. Conflicts in the dry-run merge (`git merge --no-ff --no-commit main` in the scratch copy)

Two files conflict. Everything else auto-merges, including `src/task_config.py` and `docs/design_plan.md`. No conflict markers were left in any other tracked file.

1. **`docs/progress.md`** (hunks at lines 1 to 77). Both sides add new entries at the top.
   - Resolution: keep both blocks. Mechanical, no behavior change.

2. **`docs/mpvi_decisions.md`** (line 515). Decision-number clash on **M322**.
   - safety-filter's M322 is the release-freeze scope (2026-11-25). It is cited in `docs/design_plan.md` (SF-138), in `docs/reports/20261125_1631_m226_scope_and_weighing.md`, and in the commit `e92c3f2`.
   - main's M322 is "demo video without shadows" (2026-11-16). It is cited in code: `src/task_config.py`, `src/replay.py`, `src/viewer_composite.py`, `experiments/search_video.py`, and the video reports.
   - main also has M323 to M325. safety-filter has no rows above M322.
   - Proposed resolution: keep main's M322 unchanged. Renumber safety-filter's release-freeze row to **M326** (the next free number on either branch), and update its three doc references: the row, the SF-138 line in `design_plan.md`, and the title and commit line in the 20261125_1631 report. No code on the safety-filter side cites M322, so no code changes.
   - This does not change behavior, but it renumbers a decision that is David's. **Needs your OK before I apply it.**

## 2c. Changes that auto-merge but change behavior (list, not resolved)

These came in without a textual conflict. They change what main does by default, so they need David's confirmation. This list comes from reading the config and its comments. I did not read the code paths or run anything.

- **Command bridge, uniform shrink** (`COMMAND_BRIDGE_CFG`, rows M193 and stage 3 "d0bc"). The per-joint clip and the rejection of far targets are replaced by one uniform shrink of the move. The bridge is on by default (`enabled: True`). Touches `src/bridge_shrink.py`, `src/bridge_clamp.py`, `src/commands.py`. This changes output of every run.
- **Joint buffer, new mode "range" as default** (`BRIDGE_BUFFER_CFG`, SF-125). The buffer changes from the fixed 10 degrees to min(5 % of the joint's range, 10 degrees). Changes output of every run.
- **Sag layer measure rule** (`SAG_LAYER_CFG`, SF-127, SF-118). The leaky integrator and its 5 s leak are removed from production. `measure_T_s = 2.0` is added. Touches `src/sag_layer.py`, and also `src/control_loop.py`, `src/held_mass_ramp.py`, `src/gravity_ff.py`. Changes output of every run.
- **Pick-and-place back lift** (`PICK_PLACE_CFG["back_lift_m"]` = 0.10, SF-87). The config comment itself says "0.10 m is the value used for the Phase H run; confirm." **Open question for David.**
- **Safety-filter hold policy** (`SAFETY_FILTER_PLUGIN_CFG["hold_policy"]` default changes from `repeat_last_safe` to `braking`). Only matters when the filter mode is "on". The default mode is "off", so default runs should not change.

Per CLAUDE.md, the baselines (`experiments/baselines/`, `data/cc_baseline/`) must be checked before and after. That is the full-suite step, which waits for the other session to finish.

## 2d. Fast-forward or merge commit?

- Main is not an ancestor of safety-filter, because main has 21 commits safety-filter lacks.
- safety-filter is not an ancestor of main, because it has 191 commits main lacks.
- So main cannot take safety-filter as a fast-forward. It needs a **merge commit** (`git merge --no-ff safety-filter`), and that commit must resolve the two conflicts above.
- The other route, rebasing safety-filter onto main, rewrites a branch that is already on origin. That would need a force push, so I did not consider it further without your approval.

## Scratch worktree

- Path: `/home/vboxuser/projects/openarm-tracer-sim-dryrun-merge`
- Branch: `dryrun/safety-filter-plus-main`, local only, not pushed. It starts at `be75e33`.
- The dry-run merge was aborted so this report could be committed. The conflicts are reproducible with `git merge --no-ff --no-commit main` in that worktree.
- This report is committed on the scratch branch only.

## Not done

- No full test run, no baseline comparison, and no MuJoCo process. Those are the later step.
- Re-run this dry run if the sf-d session commits more to `safety-filter` before the merge. This one is against `be75e33`.
