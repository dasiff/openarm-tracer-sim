# Exempt sim runs from systemd-oomd (run_capped.sh)

Written for: David (repo owner). Brief d7b4.

## What changed
- `experiments/run_capped.sh` line 9: added `-p ManagedOOMPreference=omit` to the `systemd-run` scope. Nothing else in the file changed. Commit `1070817` (WIP, with the progress task entry), final commit for this report adds the progress line.
- Why: systemd-oomd killed the lift-off run r0.25_t0.3 at 01:21 on Nov 3 (journal: scope `run-p7127-i1685.scope` killed for memory pressure, 71.79% > 50% for > 20 s) while it used 3.6 GB, under the 6 GB cap. The older progress line blaming the 6 GB cap is wrong (see Step 4).

## Step 0 (clean)
- No MuJoCo process, no tmux server, `data/locks/mujoco.lock` free (flock test), no `run-*.scope` units. Nothing blocked.

## Step 2: byte-identical check
- Job: `experiments/cc_bench_scenarios.sh manual` with `CC_HEADLESS=1` (the job used by `cc_bench_regen.sh` for its byte-identical check). 1272 frames, 85 CSV rows. Chosen as the quickest existing deterministic job.
- Old line: HEAD copy of run_capped.sh (temp file `experiments/run_capped_oldcopy_tmp.sh`, deleted after). Output `data/oomd_check_20261103/old.csv`, log `data/logs/oomd_check_old_20261103_1339.log`, exit 0, wall 15.0 s.
- New line: output `data/oomd_check_20261103/new.csv`, log `data/logs/oomd_check_new_20261103_1339.log`, exit 0, wall 5.5 s.
- Result: **BYTE-IDENTICAL** (`cmp` silent). Both runs under `flock data/locks/mujoco.lock`, one at a time, in tmux session `oomd_check` (chain `data/oomd_check_20261103/chain.sh`).
- Unexpected, not caused by this change: the output differs from the stored reference `experiments/baselines/manual_reference.csv`, which is identical to the Oct 22 regen run (`data/cc_baseline/regen/manual_1.csv`). Both new runs (old and new line) agree with each other, so the drift came from code changes since Oct 22. **NOT BUILT, needs decision**: whether the baseline should be regenerated (not done here).

## Step 3: the setting applies
- While the new-line run was alive, the watcher recorded the first scope (`data/logs/oomd_check_scope_20261103_1339.log`):
  - `SCOPE run-p6262-i5572.scope`
  - `ManagedOOMPreference=omit`
  - `MemoryMax=6442450944` (6 GB)
- Both values as expected.

## Not in this task (NOT BUILT, needs decision)
- Retrying r0.25_t0.3.
- Exempting tmux scopes.
- Any change to oomd's own settings.
- Regenerating the manual baseline (see Step 2).

## Commit
- Code commit `1070817` (WIP label, contains run_capped.sh and the task entry in docs/progress.md). Final commit: progress line plus this report. Not pushed.
