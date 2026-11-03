# Bench scenario baselines

This folder holds the reference outputs of the control-loop bench scenarios (`manual`, `teleop`, `policy`). A reference is the CSV that a scenario run must match byte for byte when the code has not intentionally changed.

## INDEX.csv

`INDEX.csv` says which file is the reference for each scenario. One line per file, with these columns:

- `scenario`: manual, teleop or policy.
- `file`: the file name, relative to this folder. Retired files are under `retired/`.
- `date`: the date the file was made.
- `commit`: the commit the run was made from.
- `status`: see below.
- `reason`: why the status is what it is.
- `sha256`: the SHA-256 hash of the file's contents, to check it has not changed.
- `clean_code`: `yes` if the run was made from a clean copy of the commit (no uncommitted edits), `unknown` if this is not recorded.

## Statuses

- **CURRENT**: the reference the comparison tool uses. Exactly one per scenario.
- **RETIRED**: an old reference, kept for history. Nothing reads it. It is in `retired/`.
- **NO_CURRENT**: the scenario has no reference right now. The comparison tool stops with a message and an error code. It never falls back to a retired file.

## Which tools use it

- `experiments/cc_bench_compare.py <scenario> <new.csv>` reads the CURRENT row for the scenario and compares the new run with that file.
- `experiments/cc_bench_regen.sh <scenario>` makes a new reference from a clean copy of a commit (default HEAD), then updates this index. It never overwrites a file and never commits.

## Rules

- Every regeneration is deliberate, and logged in `docs/progress.md` and in the decision log (`docs/mpvi_decisions.md`).
- A regenerated file gets a new name with its date and short commit hash. Its README note is written automatically.
- Old references are moved to `retired/`. They are not deleted.
