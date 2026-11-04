# Held pick-and-place baseline, 200 g, and two read-only checks (Nov 3, 2026)

## Summary in plain words

- **The baseline is registered.** Two runs of the 200 g bounce launcher, made one after the other in a clean scratch copy of commit `2b0586c`, matched each other and the existing Nov 3 run `data/bounce_check_20261103/run_200g`, byte for byte, on both files (`liftoff_ticks.csv` and `episode/episode_log.csv`). The baseline is `experiments/baselines/held_200g_20261103/`, with an index row `held_pick_place_200g` (status CURRENT).
- **Fingerprints (SHA-256):** `liftoff_ticks.csv` = `fa001adf804b24c8b9af58bea56fd8eef50a94284d8abfd78dd4be3c234a2211`; `episode_log.csv` = `f346f50322ec40b7cc590e4d8bf0ef919ef8ff85b54048b3e585b8a16fbac8a5`. The same hashes were computed for run 1, run 2 and the existing run.
- **Check A (the −16 N):** it is the **total** force of the one gripper actuator, which drives both fingers. Its sign is negative because the actuator pulls the fingers toward closed. The units are newtons. The value equals the lower limit of the force range that the controller sets from the 8 N per-finger cap: 2 × 8 N = 16 N. So the value is the cap being reached, not a measured grip. In the 10 Hz episode log, the same force is divided by two (per finger): 8 N.
- **Check B (the closed position):** the finger joints run from 0 m (closed) to 0.044 m (open). Closing on nothing would drive the fingers to the lower joint limit, 0 m, unless the pads meet first. Whether they do is **unknown** from the files, because the geometry was not loaded. The holding value 0.0375 m and the open value 0.0439 m both lie inside that range. The beaker's outer diameter is 0.070 m (cylinder radius 0.035 m in `models/objects/beaker.xml`).
- **Kills and slowness:** no run was killed or retried; all exit codes were 0. Peak memory was 4.12 to 4.21 GB per run, under the 6 GB cap. Whether memory pressure slowed any run is **not stated** in the stage 1 report, and this check did not measure it.

Terms used: a **position actuator** is a simulated servo that pushes a joint toward a target. **kp** and **kv** are its stiffness and damping gains. A **joint range** is the lowest and highest position a joint can reach. A **force range** caps the force an actuator can apply. **SHA-256** is a fingerprint of a file's bytes: identical files give identical fingerprints. **Scratch worktree** is a second copy of the repo in another folder, used so the main tree is not changed.

Labels: **shown by the data/code** (read from a file or a saved value, with file and line); **calculated** (computed here from saved values, with the method); **guess** (a judgement, not checked); **unknown** (the material does not say).

## Step 1: state

- Branch `held-weight-ramp-sag-off`, HEAD at the start `2b0586c` (then `1992ec5`, `5ffed19` from this task). `git status`: no tracked change. Untracked: `deps/AgileX_Tracer`, `.claude/`, `compare_nosag.py`, `extract_height_data.py`, `measstart` (all older; not touched).
- `uptime` 22:20: load average 0.32. `free -m`: 7424 MB total, 1788 MB used, 4433 MB free, 5635 MB available. `systemd-oomd` active.
- `git fetch origin`: rc 0. `origin/main` is still **`58e756f`** (checked with `git rev-parse` and `git ls-remote --heads origin main`).

## Step 2: secret search and commit

Untracked report under `docs/reports/` at that moment: `signal_trace_stage1_20261103.md` (22,160 bytes, under 1 MB; nothing left out). Masked search result:

- Key shapes (`AIza`, `sk-`, `ghp_`, `xox`, `AKIA`, `Bearer `): 0 each.
- E-mail-shaped text: 0.
- Private-key block (`-----BEGIN`): 1 match, **false positive** (the report's own description of the shape).
- Keyword with a value (api_key, token, secret, password): matches are descriptions and a code expression (`api_key=self._api_key`, value masked `self****`), **false positives**. No key value.
- Gemini key (read from the project's secrets file, counted with `grep -c -F`, value never printed): 0 matches.

Committed in one commit: **`2b0586c`** "Save the stage 1 signal trace report (Nov 3)" (explicit path).

## Step 3: scratch worktree and the two runs

**Scratch worktree:** `git worktree add --detach /tmp/held_baseline_wt HEAD` (HEAD `2b0586c`).

**Linked or copied inputs (listed as asked):**

- `.venv` (Python environment): **linked** to the main repo's `.venv`.
- `deps/OpenArm-Combined` (robot mesh files, a submodule that is empty in a new worktree): the empty folder was removed and **linked** to the main repo's copy. The main repo's `deps/` was not changed.
- Read-only caches the stance finder loads by relative path, **copied** (not linked, so runs cannot write into the main tree; 15 MB in total): `data/reach_map`, `data/bench_region_cache`, `data/stance_map`, `data/home_pose_search`, `data/beaker_hotplate_layout`, `data/gravity_ff`.
- Launcher: `data/held_run/run_mass_wt.py`, a copy of `data/bounce_check_20261103/run_mass.py` in which **only line 34 changed** (the `ROOT` path points at the scratch worktree). `diff` confirmed this. Without that change the launcher would import the main repo's code.

**Settings:** `OPENARM_SIGNAL_TRACE` unset (checked: 0 matches in the environment). The command, run inside `experiments/run_capped.sh` (6 GB cap) in tmux, one at a time, under the project lock `data/locks/mujoco.lock`:

    flock <repo>/data/locks/mujoco.lock bash experiments/run_capped.sh <log> python data/held_run/run_mass_wt.py <out> --liquid 0.100

**Run 1 and run 2:**

| Run | Start (log) | Exit (log) | Wall clock (log) | Peak memory (log) | Exit code |
|---|---|---|---|---|---|
| 1 | 22:20:49 | 22:24:45 | 3 min 55.78 s | 4,122,516 kB (4.12 GB) | 0 |
| 2 | 22:24:57 | 22:28:12 | 3 min 14.69 s | 4,205,644 kB (4.21 GB) | 0 |

Logs: `data/logs/held_200g_run1_20261103_222049.log` and `data/logs/held_200g_run2_20261103_222457.log`. Outputs (main repo, git-ignored): `data/held_baseline_20261103/run1/` and `run2/`.

No run was killed, so no retry was needed.

## Step 4: byte comparison (SHA-256)

| File | run 1 | run 2 | existing `run_200g` |
|---|---|---|---|
| `liftoff_ticks.csv` | fa001adf… | fa001adf… | fa001adf… |
| `episode/episode_log.csv` | f346f503… | f346f503… | f346f503… |

Also checked with `cmp`: run 1 = run 2, run 1 = existing, run 2 = existing, for both files. **All match.** So the baseline was registered (step 5).

## Step 5: registration

- Folder `experiments/baselines/held_200g_20261103/`: `liftoff_ticks.csv` (14,381 data rows, 58 columns, 8.8 MB), `episode_log.csv` (959 data rows, 51 columns, 0.6 MB), `README.txt` (plain-language note: commit, date, command, settings, byte checks, fingerprints, what it can and cannot see, when to regenerate).
- `experiments/baselines/INDEX.csv`: one new row (`held_pick_place_200g`, file = the folder, date 2026-11-03, commit `2b0586c`, status CURRENT, sha256 field = both fingerprints with file names, clean_code yes). Checked: all rows still have 8 fields; `cc_bench_index.py current manual` still returns `manual_reference_20261103.csv`.
- `experiments/baselines/README.md`: a short section cross-referencing the new folder (the README lists the scenarios).
- **Comparison tool:** `python experiments/cc_bench_compare.py held_pick_place_200g <file>` was tried (no simulation). It stopped with `invalid choice: 'held_pick_place_200g' (choose from manual, teleop, policy)`. **Skipped** for that reason; the tool was not edited.
- Existing baseline files were not modified, renamed or deleted.

Commit **`1992ec5`**: "Register the held pick-and-place baseline, 200 g (two files, index row, README note)" (5 files, explicit paths).

## Step 6: decision log and progress

- Decision log: new row **M247** (Nov 3, 2026, "Held pick-and-place baseline, 200 g") with David's decision, the reason, the known limit (a change that matters only at another mass, such as 250 g, would not be seen), and the statement that the manual baseline cannot see the holding, weighing and trim logic. A detail section "M247 detail" was added above the OPEN section. A dated **NOTE (see M247)** was added beside M246 item 4.
- Progress: one new entry.
- Commit **`5ffed19`**: "Decision M247: held pick-and-place baseline, 200 g (Nov 3); progress entry" (2 files, explicit paths).

## Check A: the −16 N force (read-only)

**Question:** what the −16 N is, one finger or the sum, the sign, the units, and how it relates to the 8 N cap and to `left_finger_force_N`.

**What the trace writes (shown by the code):**

- `src/signal_trace.py` line 113: `float(data.actuator_force[a['gripper']])`, written unrounded to nine decimals. This is the raw MuJoCo actuator force of the left hand's gripper actuator (`a['gripper']` is the gripper actuator id from `src/robot_helpers.py` `arm_ids`).
- Column name (`src/signal_trace.py` line 40): `left_gripper_actuator_force_N`. Units: newtons (N).
- Command force column (line 114): `ap['gripper_force'][s]`, the cap per finger in N.

**One finger or the sum (shown by the code):**

- `src/arm_actuators.py` line 19: "the actuator drives both fingers, so its gain, damping and force limit are 'fingers_per_actuator' (2) times those."
- `models/robot/combined_robot.xml` line 362: one position actuator `left_gripper_ctrl` on `openarm_left_finger_joint1`. Lines 340 to 350: the two finger joints are tied together (a joint equality constraint, line 350) and split by a tendon with coefficient 0.5 each (lines 344 and 345).
- So the −16 N is the total the one actuator applies across both fingers. Per finger it is half: **8 N** (calculated: 16 / 2).

**Sign (shown by the code; the formula is MuJoCo's documented one, not checked in this repo):**

- `src/arm_actuators.py` line 37: `act.gainprm[0] = kp`. Line 39: `act.biasprm[:3] = [0.0, -kp, -kv]`. With MuJoCo's affine bias, force = kp·ctrl − kp·q − kv·qdot (q = finger travel, qdot = its speed).
- The closing target is ctrl = 0 (`src/mpvi_modules.py` line 417 sets the grip position to 0.0 when closing). While closed on the beaker, q is about 0.0375 m, so the spring term is negative and large. The force is pushed to the lower bound of the force range, −16 N.
- Negative therefore means "pushing the fingers toward travel 0 (closed)". Positive would be opening. **Calculated check:** kp for the actuator = 2 × 300 = 600 N/m (`src/robot_specs.py` line 183 gives kp 300 per finger and `fingers_per_actuator` 2; `src/arm_actuators.py` line 63 multiplies kp by 2). With q = 0.0375 m the spring term alone is −600 × 0.0375 = −22.5 N, which is beyond the 16 N bound, so the bound is what is seen (calculated, not a measurement).

**Force range (the actuator's limit; shown by the code):**

- Load time: `src/arm_actuators.py` line 63 sets the force range to ±(fingers_per_actuator × force_limit) = ±(2 × 9.0) = **±18 N** (`src/robot_specs.py` line 183: `force_limit` 9.0, `fingers_per_actuator` 2).
- XML class default: `models/robot/combined_robot.xml` line 26: `forcerange="-333 333"` for the `motor_finger` class (default block lines 24 to 27). The load-time code overrides it (line 43 of `src/arm_actuators.py` applies the range).
- Run time: `src/control_loop.py` line 531 sets `model.actuator_forcerange[...] = [-n·F, n·F]`, with F the per-finger cap from the command. For the glass beaker F = 8.0 N (`src/task_config.py` line 152, `GRIP_FORCE_TABLE["glass_beaker"]`). So the run-time range is ±2 × 8 = **±16 N**. The −16 N is the lower end of that range.

**Relation to `left_finger_force_N` (shown by the code and the data):**

- `src/episode_runner.py` line 217: the 10 Hz log writes `abs(data.actuator_force[...]) / fingers_per_actuator` (divided by 2). So the episode log's value is the per-finger magnitude, always positive.
- Calculated from the stage 1 trace (`data/signal_trace_20261103/r2_on/...`): the episode log value equals |trace force| / 2 to within 5e-10 N over 959 samples (the stage 1 report). So 16 N in the trace is 8 N in the episode log.

## Check B: the closed position (read-only; the model was not loaded)

- **Finger joint range:** `models/robot/combined_robot.xml` line 227 (`openarm_left_finger_joint1`) and line 234 (`openarm_left_finger_joint2`): `range="0 0.044"`. So the fully **open** position is 0.044 m (upper limit) and the fully **closed** position is **0 m** (lower limit). The right hand has the same ranges (lines 227 and 234 are the left joints; the right-hand joints are defined the same way in the same file, not re-checked line by line here).
- **Where closing on nothing would stop:** the control target is 0 (see check A), so with nothing between the pads the fingers would move toward the lower limit, 0 m. Whether the pads or the fingers' collision shapes touch first, and stop short of 0, is **unknown** from these files (it needs the collision geometry, not loaded here).
- **Config fact:** `src/task_config.py` line 446: `held_travel_range_m` = (0.003, 0.040) m, with the comment "under 3 mm is 'closed on nothing' (no event, a module error), over 40 mm is 'did not close'". This is a stated range, not a threshold chosen here.
- **Comparison with the observed values (shown by the data):** holding the 200 g beaker, finger1 is 0.0375 m (stage 1 trace; 0.03751 to 0.03752 in all phases while holding). Open, finger1 is 0.0439 m (about 0.0005 m below the 0.044 m upper limit). The 0.0375 value is inside the configured range 0.003 to 0.040 m.
- **Beaker width in the scene:** `models/objects/beaker.xml` line 24: `task_beaker_glass` is a cylinder with `size="0.035 0.05"` (radius 0.035 m, half-height 0.05 m), so the outer diameter is **0.070 m**. `src/task_config.py` line 115: `outer_diameter` 0.070 m, with the comment "the gripper opens to ~84 mm between the finger pads (7 mm of margin a side)". That comment is the source's own statement; not checked here.

## Step 9: the stage 1 report and run data

- `docs/reports/signal_trace_stage1_20261103.md` gives: "Each run was one at a time, inside tmux, under `experiments/run_capped.sh` (6 GB cap), and R3 under the project lock `data/locks/mujoco.lock`. No run was killed or retried." (line 50). It does **not** give run lengths or peak memory.
- From the run logs (`data/logs/sig_R1_off_20261103_215038.log`, `sig_R2_on_20261103_215434.log`, `sig_R3_bench_on_20261103_215853.log`):
  - R1 (switch off): wall clock 3 min 45.39 s; peak memory 4,158,640 kB (4.16 GB); exit 0.
  - R2 (switch on): wall clock 3 min 52.17 s; peak memory 4,194,872 kB (4.19 GB); exit 0.
  - R3 (bench, switch on): wall clock 9.93 s; peak memory 2,192,992 kB (2.19 GB); exit 0.
- Killed or slowed by memory pressure: the stage 1 report does not say. The logs record free memory before and after each run but make no statement about slowdown. **Not determined.**

## Commit hashes

| Commit | Content |
|---|---|
| `2b0586c` | Save the stage 1 signal trace report (Nov 3) |
| `1992ec5` | Register the held pick-and-place baseline, 200 g (folder, index row, README note) |
| `5ffed19` | Decision M247 and progress entry |

Not pushed. `origin/main` is still `58e756f`.

## Secret search on this report

Run on this file (19,948 bytes, under 1 MB). Key shapes (`AIza`, `sk-`, `ghp_`, `xox`, `AKIA`, `Bearer `): 0 each. E-mail-shaped text: 0. Private-key block marker: 1 match, a **false positive** (this report's own description of the shape, line 27). Keyword-with-value matches: descriptions only (line 28), no key value. Gemini key (counted with `grep -c -F`, value never printed): 0 matches. Nothing was left out.

## Exact commands

    git status --short ; git log --oneline -3 ; git branch --show-current ; uptime ; free -m ; systemctl is-active systemd-oomd
    git fetch origin ; git rev-parse --short origin/main ; git ls-remote --heads origin main
    F=$(git ls-files --others --exclude-standard docs/reports) ; grep -c -E '<pattern>' $F ; sed -n 's/^[^=]*=//p' ~/.config/openarm-tracer-sim/secrets.env > <tmp key file> ; grep -c -F -f <tmp key file> $F ; rm <tmp key file>
    git add docs/reports/signal_trace_stage1_20261103.md && git commit -m "Save the stage 1 signal trace report (Nov 3)"
    git worktree add --detach /tmp/held_baseline_wt HEAD
    ln -s <repo>/.venv /tmp/held_baseline_wt/.venv
    rmdir /tmp/held_baseline_wt/deps/OpenArm-Combined ; ln -s <repo>/deps/OpenArm-Combined /tmp/held_baseline_wt/deps/OpenArm-Combined
    mkdir -p /tmp/held_baseline_wt/data && cp -r <repo>/data/{reach_map,bench_region_cache,stance_map,home_pose_search,beaker_hotplate_layout,gravity_ff} /tmp/held_baseline_wt/data/
    sed 's|^ROOT = ".*"|ROOT = "/tmp/held_baseline_wt"|' <repo>/data/bounce_check_20261103/run_mass.py > /tmp/held_baseline_wt/data/held_run/run_mass_wt.py ; diff <repo>/data/bounce_check_20261103/run_mass.py /tmp/held_baseline_wt/data/held_run/run_mass_wt.py
    tmux new-session -d -s held_r1 "cd /tmp/held_baseline_wt && unset OPENARM_SIGNAL_TRACE && flock <repo>/data/locks/mujoco.lock bash experiments/run_capped.sh <repo>/data/logs/held_200g_run1_<stamp>.log python data/held_run/run_mass_wt.py <repo>/data/held_baseline_20261103/run1 --liquid 0.100"
    (run 2: the same with run2, held_r2)
    sha256sum <run1 and run2 and existing files> ; cmp <pairs>
    cp <run1>/liftoff_ticks.csv experiments/baselines/held_200g_20261103/ ; cp <run1>/episode/episode_log.csv experiments/baselines/held_200g_20261103/
    source .venv/bin/activate && python experiments/cc_bench_compare.py held_pick_place_200g <file>      # refused: invalid choice
    unlink /tmp/held_baseline_wt/.venv ; unlink /tmp/held_baseline_wt/deps/OpenArm-Combined ; git worktree remove --force /tmp/held_baseline_wt ; git worktree prune
    git add experiments/baselines/held_200g_20261103 experiments/baselines/INDEX.csv experiments/baselines/README.md && git commit -m "Register the held pick-and-place baseline, 200 g (two files, index row, README note)"
    git add docs/mpvi_decisions.md docs/progress.md && git commit -m "Decision M247: held pick-and-place baseline, 200 g (Nov 3); progress entry"
    git status --short ; git diff --stat ; git ls-remote --heads origin main

## Verification (step 10)

- `git status --short`: this report (uncommitted), plus the untracked files that were already there. `git diff --stat`: empty (no tracked change).
- Files changed by this task's commits: `experiments/baselines/held_200g_20261103/` (3 files), `experiments/baselines/INDEX.csv`, `experiments/baselines/README.md`, `docs/mpvi_decisions.md`, `docs/progress.md`, and the stage 1 report (`2b0586c`). Nothing under `deps/`; no existing baseline file changed.
- Scratch worktree removed; the main repo's `.venv` and `deps/OpenArm-Combined` were checked after the unlink and are intact.
- Nothing pushed: `origin/main` is `58e756f`.

## NOT BUILT, needs decision

1. **The comparison tool does not accept this scenario.** `experiments/cc_bench_compare.py` and `experiments/cc_bench_regen.sh` accept only `manual`, `teleop` and `policy`. I did not edit them (the brief forbids it). So the held baseline is not compared by the tool; checks are by SHA-256 only. To use it, the tools would need a change. Decision for David.
2. **The index assumes one file per row.** The README says "One line per file", and `cc_bench_index.py` returns one file per scenario. The held baseline is one row with a folder path and two fingerprints. The tools do not read this row, so nothing breaks, but the format is different. Decision for David whether to split it into two rows (that would make two CURRENT rows for one scenario, which the tool treats as an error).
3. **Size.** `liftoff_ticks.csv` is 8.8 MB, committed in `1992ec5`. The repo has no size rule for baselines; this is flagged, not judged.
4. **Closing on nothing.** Where the fingers stop with nothing between the pads is unknown from the files (collision geometry not loaded). A test would need a run, which is not part of this task.
5. **Memory pressure and slowdown** during the runs: not determined (see step 9).
6. **Regeneration rule.** The README says regeneration is deliberate; no tool does it for this folder. Decision for David whether one is wanted.
7. **Untracked files** from earlier tasks (`deps/AgileX_Tracer`, `.claude/`, `compare_nosag.py`, `extract_height_data.py`, `measstart`) and the `deps/` display question are unchanged.
