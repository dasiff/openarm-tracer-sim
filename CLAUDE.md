# CLAUDE.md

## Crash recovery rules (the VM and sessions crash often)

1. **Progress log.** Keep `docs/progress.md`. At the start of each task, write: the task (with design plan section if any), the planned steps, and the current step.
   After each step, update it: what's done, what's next, open questions, and any run in progress (command, log file, start time). Keep it short; replace old task
   entries with a one-line summary when a task finishes.

2. **Commit often.** Commit after each completed step and always before starting any run longer than a couple of minutes. Unfinished work gets a commit message starting
   "WIP:" saying what's done and what isn't. Never push WIP without asking.

3. **Long runs survive the session.** Run anything longer than a couple of minutes in a detached tmux session, with all output written to a timestamped log file under
   `data/logs/`. Record the command and log path in `docs/progress.md` before starting. Check results by reading the log, not by waiting in the session.

4. **No half-written files.** Never overwrite a reference, baseline, or dataset file in place. Write to a temporary file, check it's complete, then rename it over the
   old one. An interrupted run must leave the old file intact.

5. **Starting a session.** Before doing anything else, read `docs/progress.md`, run `git status` and `git log --oneline -5`, check for tmux sessions still running, and
   report what was in progress and what state it's in. Then wait for instructions.

6. **Reports survive crashes.** Before sending any report, write it to
   `docs/reports/<date>_<time>_<topic>.md` and commit it. The reply points to the file.
