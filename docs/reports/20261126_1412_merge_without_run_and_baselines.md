# Merge without a new run, and baselines (2026-11-26)

## Done

1. **safety-filter pushed**: `181fa74` (back_lift_m marked PROVISIONAL), fast-forward from `3b345fa`. Earlier, `3b345fa` (release-freeze row M342) was pushed too.
2. **Main merged with safety-filter, and pushed.**
   - Main was `9b899d0` when the merge was made (main had moved from `9e005f5`, with the new navigation commits).
   - Decision IDs: main uses M342 to M345 for navigation. Safety-filter's release-freeze row became **M346** in the merge (M346 is free on both branches). The references were renamed in `design_plan.md` (SF-138), the 1458, 1631 and 1724 reports. Main's own references were left alone.
   - `experiments/run_tests.sh`: the missing space on line 18 is fixed in the merge. Main had the bug.
   - `progress.md`: both blocks kept.
   - Main's `src/control_loop.py` and `src/run_record.py` changes were reviewed: a seed passed to the run recorder, written to meta.json. No behavior change to physics.
   - The scratch merge `0b9db77` and the main merge `4d80d52` have the same tree, apart from the progress.md order.
   - No suite run after this merge (David's decision).
3. **Manual baseline regenerated** from clean commit `4d80d52`: `manual_reference_20261126_4d80d52.csv`, two byte-identical runs. The old reference is in `retired/`. Compared with the retired file, 30 of 84 data rows differ; the largest numeric difference is about 0.46. This is expected from the approved bridge and sag changes.
   - `INDEX.csv`: manual is now CURRENT on the new file. Two held rows had unquoted commas in their reason text, so the index tool could not read the file; they are quoted now, with the same text.
   - Commit `af35487`, pushed.

## Not done: held baseline (blocked, needs David)

- The 200 g held launcher `data/bounce_check_20261103/run_mass.py` reads `tr["C_p"]` (line 176). Current code no longer writes that trace (M262 moved the trim to the control loop). The run aborts at about 34 s with `KeyError: 'C_p'`. Both runs were stopped; no output was written.
- Choices for David:
  1. Leave the C_p columns blank. The held file's schema then changes from the Nov 4 reference.
  2. Fill C_p from the current sag layer's correction. This needs a decision on what the column means now.
  3. Retire the held reference and set the held scenario to NO_CURRENT until a launcher that works on the current code exists.
- Meanwhile `held_pick_place_200g` is still CURRENT on the Nov 4 reference. It was made from pre-merge code and will not match the new main.

## Other things to know

- **The safety-filter branch on origin still calls the release-freeze row M342.** The rename to M346 was made in the merge, not on safety-filter. Main is M346 for it; safety-filter is M342, which main now uses for navigation. Fix: a small commit on safety-filter renaming the row to M346, with no code change. Needs David's OK.
- Pushing main sent about 211 commits that had never been pushed, including the WIP lidar and navigation commits. Those are David's, but they had not been pushed.
- Left in place: the held-regen worktree at `/home/vboxuser/projects/openarm-tracer-sim-held-regen`, the scratch worktrees, and the branches `dryrun/safety-filter-plus-main` and `dryrun/merge-main-7518703`.
