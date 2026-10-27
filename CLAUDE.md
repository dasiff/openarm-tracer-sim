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

## Project overview

MuJoCo simulation of an Agilex Tracer mobile base + OpenArm bimanual robot doing materials-lab tasks (first task: beaker onto a hotplate), with a planned MPVI-style
planner (Gemini LLM for the step order, VLM completion checker, scripted navigation, learned arm policy). README.md has the controls, the project tree and the episode/recording
docs; read it before changing the controller or viewer. The roadmap and the status of every planned item is `docs/design_plan.md`; open questions for the team are in `docs/TODO.md`;
reference values no code reads are in `docs/robot_reference.md`.

Where things live: `src/` (library: simulator, control loop, command sources, policies, stance finder, reach maps, navigation, MPVI modules), `experiments/` (launchers, tests and
one-off studies; every test is a script `experiments/<name>_test.py` that exits 0 on pass), `models/` (generated robot file `models/robot/` — never edit by hand, regenerate with
`tools/import_openarm_description.py`; scenes; objects), `deps/` (submodules, never edited), `legacy/` (old code kept for old experiments), `data/` (git-ignored: logs, maps,
recordings, baselines), `docs/reports/` (dated reports).

## Environment and running things

- Always `source .venv/bin/activate` first (Python 3.12, MuJoCo 3.13.0; it sets the geo_kin license variable). Never copy the license into the repo.
- Headless rendering: `MUJOCO_GL=egl`. For the viewer from Claude's shell: `DISPLAY=:0 XAUTHORITY=$(ls /run/user/1000/.mutter-Xwaylandauth.* | head -1)`. The VM has no GPU
  (8 GB RAM, 4 CPUs), so camera renders are slow and the viewer runs at about 0.9x real time.
- **One MuJoCo process at a time**, never in parallel or in background subagents (parallel sims OOM-crashed the VM). Check `pgrep -af -i 'mujoco|python.*teleop'` first.
  Heavy jobs: `bash experiments/run_capped.sh <log> <command...>` (6 GB cap, peak RSS in the log).
- Full test suite: `[SKIP="test_a ..."] bash experiments/run_tests.sh [TAG]` in a detached tmux session (about an hour); summary in `data/logs/suite_<TAG>_summary.txt`.
  A test passes if its exit code is 0. Add a new test to the list in that script.
- Never use `mjr_overlay` text per frame in the viewer (it leaks GPU ids on this VM and aborts the process); the HUD is an OpenCV bitmap.
- The Gemini API key is in `~/.config/openarm-tracer-sim/secrets.env` (outside the repo). Never print it or write it into the repo, docs, logs or memory.

## Working rules (from docs/design_plan.md section 0, standing)

- Design decisions are David's: stop and ask when a choice affects architecture, interfaces, physical layout, or robot/camera parameters. Implementation details are your call.
  Do not guess an undecided design; write the question down and skip only the blocked part.
- Build only what is asked; propose extras instead of adding them. Keep it simple first and note simplifications on the "later" list.
- Every decided value lives in config (`src/task_config.py`) with a plain-word comment: what it does, its unit, why this value, and its decisions-log row id.
  Every MPVI decision gets a row (M001, ...) in `docs/mpvi_decisions.md` in the same commit that implements it. Paper facts are cited only from `docs/mpvi_paper_excerpts.md`
  or the Appendix D prompts; anything else is "not in excerpts, ask David".
- Each task's commit updates the affected status markers in `docs/design_plan.md`. Do not leave the plan and the code disagreeing.
- All Claude-written code includes the line `Code generated by Claude` (as the first comment).
- Existing pipeline output must stay byte-identical unless a change is decided: check the baselines (`data/baselines`, `experiments/baselines`) before and after.
- Naming: the arms-at-home pose is the "zombie pose" (code term: home pose; `home_q_left` / `home_q_right`). The eagle camera is fixed to the torso; there is no neck.

## Git

- Commits use the repo author David Siff. Push only when asked, only `git push origin main` to github.com/dasiff over HTTPS (never the gatech mirror). Never push WIP without asking.
