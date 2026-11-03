# Signal trace, stage 1: raw logging for the holding and stillness work (Nov 3, 2026)

Terms are explained the first time they appear. Labels: **shown by the data/code** (read from a file or a saved value, with file and line), **calculated from the data** (computed here from saved values; method stated), **guess** (a judgement, not checked), **unknown** (the material does not say).

## Summary in plain words

- **All three proof runs passed under the rule.**
  - **R1** (switch off, the 200 g launcher): `liftoff_ticks.csv` and `episode/episode_log.csv` are **byte-identical** to the existing `run_200g`. No `signal_trace.csv` was written.
  - **R2** (switch on, same run): the same two files are **byte-identical**. `signal_trace.csv` exists with **14,382 data rows**, against 14,381 in `liftoff_ticks.csv`. The extra row is the final tick (14,381). No field is empty.
  - **R3** (manual bench, switch on): matches the CURRENT baseline (`manual_reference_20261103.csv`) with **zero difference** on all 84 rows. **Caveat:** the bench scenario calls `run_loop` directly (`experiments/combined_controller.py` line 355) and never reaches the new code. R3 therefore shows only that the bench is unchanged. R1 and R2 are the tests that exercise the change.
- **Logged:** for both hands, the two finger positions (m), the signed gripper actuator force (N), the command force cap (N per finger), the gripper target (m), the pad-centre position (m), the seven arm joint positions and seven joint speeds (rad, rad/s), and the held-mass value (kg). For both hands, the pad-to-beaker vertical contact force and the beaker-bench contact count are **simulator-only** diagnostics (named `sim_only_`).
- **Skipped:** no signal was skipped. Acceleration was not logged, as the brief asked (it is computed later from positions). The closing-on-nothing case was **not run** (no existing option removes the beaker without changing logic; see NOT BUILT).
- **What the ranges show** (R2 trace, the table below):
  - **Holding vs closing on nothing:** not measured. There is no closing-on-nothing run, so this comparison is **unknown**.
  - **Holding vs open around the beaker:** these separate clearly. While holding, the signed actuator force is a constant −16 N (2 × the 8 N per-finger cap) and finger1 sits at 0.03752 m. With the gripper open, the force is about +0.044 N and finger1 is about 0.04393 m.
  - **Open around the beaker vs open and empty:** **not separable** by these signals. The ranges are nearly the same. Force: 0.04292 to 0.04463 N (approach) and 0.04268 to 0.04430 N (beaker between pads). Finger1: 0.04393 m in both.
  - **Where the ranges overlap:** only in the transitions. The closing phase passes through the holding values, and the opening phase passes back through the open values. Those are the places where a threshold would be hard to place.
- **10 Hz vs 150 Hz:** the old 10 Hz episode-log finger values match the new 150 Hz trace **exactly** at the 959 matching times (largest difference 0 m). The episode log's finger force equals half the magnitude of the trace's actuator force, to within 5e-10 N.

Terms: **qpos** is a joint's position in the simulator's state (here the finger slide position, metres). **Actuator force** is the force a motor (here the simulated servo) applies. **Feedforward** is a torque added to the controller's command. **Control tick** is one step of the 150 Hz loop (about 6.7 ms). **Joint speed** is the joint's velocity (`qvel` in MuJoCo, the simulator's physics library).

## The code as written

Two files changed, one new file. Committed in `621b32b` (code) and `6c722d2` (docs addendum). Nothing else in `src/` or `experiments/` changed.

**`src/signal_trace.py`** (new, 123 lines). Read-only logging. Key lines:
- line 27: `ENV = "OPENARM_SIGNAL_TRACE"`; the switch is off unless its value is exactly `"1"` (lines 32-34, `enabled()`).
- lines 37-47, `columns()`: the column names. Names starting with `sim_only_` are simulator-only.
- lines 50-58, `_held_mass()`: the held-mass formula of `src/held_mass_ramp.mass_at` (same formula), **without writing** the record's `now` field. `src/gravity_ff.py` line 49 reads that field, so a write would change the controller. This module does not write it.
- line 61, `class SignalTrace`. Its constructor (lines ~62-76) opens `signal_trace.csv` in the episode's output folder and reads the finger, wrist and beaker ids once. `gripper_frame` (from `src/gripper_geometry.py`) is called once to get the pad-centre offset. That function builds its own scratch model data and does not change the simulation.
- lines 78-98, `_pad_contacts()`: **simulator-only**. Vertical contact force on the beaker from each hand's finger bodies (`mujoco.mj_contactForce`, a read), and the count of beaker-bench contacts.
- lines 100-120, `write()`: one row per tick. Reads `obs` (tick, time, the applied command) and `data` (positions, velocities, actuator forces). Writes one CSV row.

**`src/episode_runner.py`** (7 added lines):
- line 61: `from src import signal_trace`.
- line 178-179: the switch and the trace object. `sig_trace` is `None` unless `out_dir` is set **and** `OPENARM_SIGNAL_TRACE=1`.
- lines 218-219: called once per control tick, after the existing log row and before the end-condition checks: `sig_trace.write(obs, phase, data)`.
- lines 268-269: the file is closed at the end of the run.

No command, setting, number or existing output file was changed. `python -m py_compile` passed on both files. The diff of `src/episode_runner.py` is saved at `data/signal_trace_20261103/stage1_episode_runner.diff`.

## Proof runs (step 4)

| Run | What | Switch | Result |
|---|---|---|---|
| R1 | 200 g launcher, same command and settings as `data/bounce_check_20261103/run_200g` (`run_mass.py <out> --liquid 0.100`), output `data/signal_trace_20261103/r1_off/run` | off | exit 0. `liftoff_ticks.csv` and `episode/episode_log.csv` **byte-identical** to `run_200g`. No `signal_trace.csv`. Log: `data/logs/sig_R1_off_20261103_215038.log` |
| R2 | same, output `data/signal_trace_20261103/r2_on/run` | on (`OPENARM_SIGNAL_TRACE=1`) | exit 0. Same two files **byte-identical**. `signal_trace.csv`: 14,382 data rows (ticks 0 to 14381), 52 columns, no empty field. Log: `data/logs/sig_R2_on_20261103_215434.log` |
| R3 | manual bench, `env CC_HEADLESS=1 bash experiments/cc_bench_scenarios.sh manual ...` (same as `docs/reports/baseline_drift_part2_20261103.md`), output `data/signal_trace_20261103/r3_on/manual.csv` | on | exit 0. `python experiments/cc_bench_compare.py manual ...`: 84 rows, **max difference 0.000000 on every column**, sim-time difference 0. Log: `data/logs/sig_R3_bench_on_20261103_215853.log`. The bench does not reach the new hook (see Summary). |

Each run was one at a time, inside tmux, under `experiments/run_capped.sh` (6 GB cap), and R3 under the project lock `data/locks/mujoco.lock`. No run was killed or retried.

**R2 row count, explained:** `liftoff_ticks.csv` has ticks 0 to 14380 (14,381 rows). `signal_trace.csv` has ticks 0 to 14381 (14,382 rows). The extra row is tick 14381, the last tick, which the new hook writes and the older trace does not. The reason the older trace stops one tick earlier is **unknown**. My guess is that the older trace is written from inside the gravity feedforward call, which may not run on the final tick. Not checked.

## Extra cases (step 5)

- **(a) Gripper open with the beaker between the pads (before CLOSE):** this is the `pick: IN` step, ticks 10469 to 10999 in R2. The gripper is open (finger1 about 0.04393 m, force about +0.044 N, command cap 9 N). See the table below. The pads are around the beaker by the end of this step: CLOSE then holds the beaker at travel 0.0375 m. The beaker's position is not in `signal_trace.csv`; its contact count is the simulator-only bench count, which is 4 during IN. **Whether the beaker touches the finger pads during IN is not shown by these columns** (the pad contact column is 0 in IN). Unknown from these columns alone.
- **(b) Closing on nothing: not run.** The launchers (`run_mass.py`, `bounce_liftoff_run.py`, `demo_video.py`) have options for the liquid mass, the rate, the threshold, the pick-only stop, the bridge and the record folder, but none removes or moves the beaker. The only candidate is a `beaker_xy` override in a copy of the launcher. That changes the layout the plan is built from, and I did not check that it produces a closing-on-nothing miss. The brief says not to build one. Listed under NOT BUILT.

## Survey tables (step 6)

Source: `/tmp/signal_trace_20261103/survey_stage1.py` (read-only), output `/tmp/signal_trace_20261103/survey_stage1.md`. Phases are the step labels in the trace. Each cell is min / median / p95 / max. Ticks are the first and last tick of each phase, inclusive. Hand speed and acceleration are calculated from the pad-centre positions (finite differences of the 150 Hz positions). Arm joint speed is the largest absolute speed of the left arm's seven joints, from the logged joint speeds.

## Phase table (R2 trace, 150 Hz; min / median / p95 / max)
| phase | ticks | n | closure finger1 m | closure finger2 m | actuator force N (signed) | command force cap N | hand speed m/s | hand accel m/s2 | arm joint speed max rad/s | sim-only pad contact z N | sim-only bench contacts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| open, empty: approach (OVER, DOWN) | 9688-10468 | 781 | 0.043926 / 0.043927 / 0.043927 / 0.043927 | 0.043923 / 0.043925 / 0.043927 / 0.04393 | 0.042917 / 0.043926 / 0.043992 / 0.04463 | 9 / 9 / 9 / 9 | 1.7142e-05 / 0.047677 / 0.10205 / 0.10955 | 3.1817e-05 / 0.063499 / 0.50272 / 0.88678 | 2.5539e-05 / 0.24354 / 0.49111 / 0.5015 | 0 / 0 / 0 / 0 | 4 / 4 / 4 / 4 |
| open, beaker between pads (pick IN, before CLOSE) | 10469-10999 | 531 | 0.043926 / 0.043927 / 0.043927 / 0.043928 | 0.043925 / 0.043925 / 0.043931 / 0.043932 | 0.042681 / 0.043926 / 0.043997 / 0.044303 | 9 / 9 / 9 / 9 | 0.00019481 / 0.038472 / 0.061012 / 0.061416 | 0.0072011 / 0.047616 / 0.17496 / 0.40383 | 0.00052028 / 0.24056 / 0.37663 / 0.37788 | 0 / 0 / 0 / 0 | 4 / 4 / 4 / 4 |
| closing (CLOSE) | 11000-11150 | 151 | 0.03573 / 0.037516 / 0.037518 / 0.040937 | 0.036278 / 0.037947 / 0.037948 / 0.040858 | -16 / -16 / -16 / -16 | 8 / 8 / 8 / 8 | 8.2167e-07 / 2.1136e-05 / 0.012175 / 0.074549 | 0.00012323 / 0.0020747 / 0.8781 / 6.6378 | 2.4887e-05 / 7.192e-05 / 0.018934 / 0.3095 | -1.2621 / -0.012476 / 0.094595 / 1.5951 | 0 / 4 / 4 / 4 |
| closed, holding at rest (WEIGH) | 11151-11852 | 702 | 0.037515 / 0.037516 / 0.037516 / 0.037517 | 0.037946 / 0.037948 / 0.037949 / 0.03795 | -16 / -16 / -16 / -16 | 8 / 8 / 8 / 8 | 3.6893e-06 / 8.3704e-05 / 0.00021004 / 0.00076396 | 8.3752e-05 / 0.0022938 / 0.013518 / 0.047314 | 2.6477e-05 / 0.00036369 / 0.00064785 / 0.0022426 | -0.040957 / 0.87146 / 1.9574 / 1.9658 | 0 / 4 / 4 / 4 |
| lift (UP) | 11853-12300 | 448 | 0.037515 / 0.037516 / 0.037518 / 0.03752 | 0.037946 / 0.037948 / 0.037949 / 0.037949 | -16 / -16 / -16 / -16 | 8 / 8 / 8 / 8 | 0.00013351 / 0.038949 / 0.073054 / 0.078654 | 2.9079e-05 / 0.090581 / 0.22991 / 0.39093 | 0.00020482 / 0.22463 / 0.50428 / 0.51438 | 1.8106 / 1.9657 / 1.9985 / 2.0988 | 0 / 0 / 0 / 0 |
| carry / over (place OVER) | 12303-12554 | 252 | 0.037516 / 0.037517 / 0.037518 / 0.037518 | 0.037945 / 0.037947 / 0.037949 / 0.037949 | -16 / -16 / -16 / -16 | 8 / 8 / 8 / 8 | 0.0002391 / 0.13074 / 0.24159 / 0.24233 | 4.9636e-05 / 0.31168 / 0.67345 / 0.72687 | 0.00034299 / 0.22204 / 0.32916 / 0.33173 | 1.8156 / 1.962 / 2.0429 / 2.098 | 0 / 0 / 0 / 0 |
| place DOWN | 12555-12863 | 309 | 0.037516 / 0.037516 / 0.037517 / 0.037517 | 0.037947 / 0.037949 / 0.03795 / 0.03795 | -16 / -16 / -16 / -16 | 8 / 8 / 8 / 8 | 0.00034401 / 0.0094978 / 0.096478 / 0.10142 | 0.0077092 / 0.11349 / 0.40919 / 1.2905 | 0.0064603 / 0.049446 / 0.28886 / 0.29635 | 1.2845 / 1.9618 / 2.0339 / 2.0945 | 0 / 0 / 0 / 0 |
| opening (place OPEN) | 12864-13072 | 209 | 0.037516 / 0.037516 / 0.043927 / 0.043927 | 0.037947 / 0.037949 / 0.043928 / 0.043929 | -16 / -16 / 0.043929 / 3.785 | 8 / 8 / 9 / 9 | 6.5288e-05 / 0.00071899 / 0.0064754 / 0.012422 | 0.00020373 / 0.0083874 / 0.11999 / 0.7625 | 0.00017266 / 0.0021305 / 0.020435 / 0.03531 | -0.26662 / 0.09986 / 1.9611 / 1.9616 | 0 / 0 / 0 / 0 |
| empty again (BACK, HOME) | 13073-14080 | 1008 | 0.043926 / 0.043927 / 0.043927 / 0.043928 | 0.04392 / 0.043925 / 0.043929 / 0.04393 | 0.04347 / 0.043907 / 0.043959 / 0.04481 | 9 / 9 / 9 / 9 | 1.0959e-05 / 0.035711 / 0.077911 / 0.080756 | 5.5772e-12 / 0.039665 / 0.23997 / 0.50886 | 3.6228e-05 / 0.15841 / 0.48857 / 0.51801 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 |

## Overlap of ranges between phases (signed actuator force N and finger1 closure m)

### force
- open, empty: approach (OVER, DOWN): min 0.042917, max 0.04463
- open, beaker between pads (pick IN, before CLOSE): min 0.042681, max 0.044303
- closing (CLOSE): min -16, max -16
- closed, holding at rest (WEIGH): min -16, max -16
- lift (UP): min -16, max -16
- carry / over (place OVER): min -16, max -16
- place DOWN: min -16, max -16
- opening (place OPEN): min -16, max 3.785
- empty again (BACK, HOME): min 0.04347, max 0.04481

### f1
- open, empty: approach (OVER, DOWN): min 0.043926, max 0.043927
- open, beaker between pads (pick IN, before CLOSE): min 0.043926, max 0.043928
- closing (CLOSE): min 0.03573, max 0.040937
- closed, holding at rest (WEIGH): min 0.037515, max 0.037517
- lift (UP): min 0.037515, max 0.03752
- carry / over (place OVER): min 0.037516, max 0.037518
- place DOWN: min 0.037516, max 0.037517
- opening (place OPEN): min 0.037516, max 0.043927
- empty again (BACK, HOME): min 0.043926, max 0.043928

## 10 Hz episode log vs 150 Hz trace at matching sim times: 959 matched samples
- max |finger1 difference| 0 m; max |finger2 difference| 0 m
- max |episode finger force - |trace actuator force| / 2| 5e-10 N over 959 samples

## Reading the ranges (step 6, no thresholds)

- **Holding vs open:** clearly apart in this run. Finger1 is 0.03751 to 0.03752 m while holding (the phases from WEIGH to PLACE OPEN, before the release), and 0.04392 to 0.04393 m when open. The signed force is −16 N while holding and about +0.044 N when open. Finger2 shows the same split.
- **Open around the beaker vs open and empty:** **not separable** by closure or force. The two ranges almost coincide. A threshold on these two signals could not tell them apart.
- **Holding vs closing on nothing:** **unknown**, because the closing-on-nothing run was not made. The closing phase here is a real grasp (CLOSE ends at travel 0.0375 m).
- **Overlap:** the CLOSE phase passes through the hold values (force reaches −16 N at its end; finger1 runs from 0.0439 to 0.0375 m during the step). The OPEN phase passes back (force goes from −16 N to +3.8 N at its maximum, finger1 back to 0.0439 m). A threshold placed on either signal would fall inside these transitions.
- **Command force cap** (`grip_force_cmd`): 8 N in every phase from CLOSE to PLACE OPEN, 9 N with the gripper open. It is a command, so it shows the controller's intent, not the grip. It separates holding from open exactly, but it is not a measurement.
- **Simulator-only contacts:** the pad-to-beaker vertical force is about 1.9 to 2.0 N during lift and carry, and 0 when open, which is what it should be. It is not used by any rule, and stage 1 does not use it.
- **Arm joint speed** (left arm, largest of seven joints): about 0.0004 rad/s median in WEIGH (still), 0.22 to 0.24 rad/s during lift and carry. Calculated from the logged joint speeds.

## Other figures

- R2 trace: 14,382 ticks, 52 columns, sim-time step 1/150 s. The 10 Hz episode log has 959 samples in the same run, and every one matches the trace (largest difference 0 m for finger1 and finger2). The episode log's finger force equals half the trace's actuator force magnitude (largest difference 5e-10 N).
- Signal values as logged: finger positions in m, actuator force in N (signed), command cap in N per finger, target in m, pad position in m, joint positions in rad, joint speeds in rad/s, held mass in kg.

## Commits

| Commit | What |
|---|---|
| `621b32b` | Stage 1 holding signals: diagnostic logging of raw signals (off by default, `OPENARM_SIGNAL_TRACE=1`); no behavior change. Files: `src/signal_trace.py` (new), `src/episode_runner.py` |
| `6c722d2` | Decision M246 addendum (Nov 3): staged approach confirmed; stage 1 is raw logging only. File: `docs/mpvi_decisions.md` |
| `d6435c8` | Save the merge and push report (Nov 3). File: `docs/reports/merge_push_20261103.md` (this was the only untracked report) |
| `6ced61c` | Progress: stage 1 signal logging, proofs R1 to R3 (Nov 3). File: `docs/progress.md` |

Each commit was made by explicit path. No push was made. This report is not committed (as the brief asked).

## Secret search

Done on the untracked reports under `docs/reports/` before commit `d6435c8`, which was the only untracked report: `docs/reports/merge_push_20261103.md`, 11,663 bytes (under 1 MB). This report is not included.

- Google key (`AIza`), `sk-`, `ghp_`, `xox`, `AKIA`, `Bearer `: **0 hits** each.
- Private-key block (`-----BEGIN`): 1 match, **false positive** (line 44 of that report describes the shape in its own table).
- Keyword with a value (api_key, token, secret, password): 3 matches, **false positives** (the report quotes the code expression `api_key=self._api_key`, value masked `self****`, and describes the scan). No key value.
- E-mail-shaped text: 3 matches, **false positives**. They are the Anthropic no-reply address in the `Co-Authored-By` lines of commit messages quoted in the report, and the placeholder used as an example. No personal address.
- Gemini key (read from `~/.config/openarm-tracer-sim/secrets.env`, counted with `grep -c -F`, value never printed): **0 matches**.
- Nothing was left out.

## Exact commands (summary)

```
git status --short ; git log --oneline -3 ; git branch --show-current ; uptime ; free -m ; systemctl is-active systemd-oomd
git check-ignore -v data/signal_trace_20261103/x.csv        # data/ is git-ignored (.gitignore line 20)
python -m py_compile src/signal_trace.py src/episode_runner.py
git diff -- src/episode_runner.py > data/signal_trace_20261103/stage1_episode_runner.diff
tmux new-session -d -s sig_r1 "... unset OPENARM_SIGNAL_TRACE && bash experiments/run_capped.sh data/logs/sig_R1_off_<stamp>.log python data/bounce_check_20261103/run_mass.py data/signal_trace_20261103/r1_off/run --liquid 0.100"
cmp data/bounce_check_20261103/run_200g/liftoff_ticks.csv data/signal_trace_20261103/r1_off/run/liftoff_ticks.csv
cmp data/bounce_check_20261103/run_200g/episode/episode_log.csv data/signal_trace_20261103/r1_off/run/episode/episode_log.csv
tmux new-session -d -s sig_r2 "... OPENARM_SIGNAL_TRACE=1 bash experiments/run_capped.sh data/logs/sig_R2_on_<stamp>.log python data/bounce_check_20261103/run_mass.py data/signal_trace_20261103/r2_on/run --liquid 0.100"
cmp (the same two files, run_200g vs r2_on)
tmux new-session -d -s sig_r3 "... OPENARM_SIGNAL_TRACE=1 flock data/locks/mujoco.lock bash experiments/run_capped.sh data/logs/sig_R3_bench_on_<stamp>.log env CC_HEADLESS=1 bash experiments/cc_bench_scenarios.sh manual data/signal_trace_20261103/r3_on/manual.csv"
source .venv/bin/activate && python experiments/cc_bench_compare.py manual data/signal_trace_20261103/r3_on/manual.csv
python -I /tmp/signal_trace_20261103/survey_stage1.py > /tmp/signal_trace_20261103/survey_stage1.md
git add src/signal_trace.py src/episode_runner.py && git commit ... ; git add docs/mpvi_decisions.md && git commit ...
git add docs/reports/merge_push_20261103.md && git commit ...
git add docs/progress.md && git commit ...
```

Monitors were used to wait for each run's `JOB_DONE` marker instead of polling.

## Verification (step 8)

Checked after the last commit (`6ced61c`):
- `git status --short`: the new report (this file, uncommitted), plus the untracked files that were already there (`deps/AgileX_Tracer`, `.claude/`, `compare_nosag.py`, `extract_height_data.py`, `measstart`). No tracked file has changed.
- `git diff --stat`: empty.
- Files changed by this session's commits (`git diff --name-only 58e756f..HEAD`): `docs/mpvi_decisions.md`, `docs/progress.md`, `docs/reports/merge_push_20261103.md`, `src/episode_runner.py`, `src/signal_trace.py`. `deps/` and `experiments/baselines/` were not touched.
- Nothing pushed: `git ls-remote --heads origin main` still shows `58e756f`. Local `main` is at `58e756f` (no commits ahead of origin). The working branch is 4 commits ahead of `main` and not pushed.

## NOT BUILT, needs decision

1. **Closing-on-nothing run (step 5b):** not made. No existing launcher option removes or moves the beaker without a change to the layout the plan is built from. Whether to build a test layout for this (a `beaker_xy` override or a new option) is David's call. Until it is run, the holding-vs-closing comparison is unknown.
2. **Why the older `liftoff_ticks.csv` has one tick fewer** than the new trace (14,381 vs 14,382 rows): unknown. A guess is the gravity feedforward call, not checked.
3. **R3 does not test the new hook.** The bench path calls `run_loop` directly. If the stage 1 logging is to be checked on a bench-style run, that needs a bench hook (a decision for David, not built here).
4. **Acceleration** is not logged, as the brief asked. It is computed from positions in this survey only.
5. **Beaker position is not in `signal_trace.csv`.** Whether the beaker is between the pads during the IN step is therefore not shown by the new columns (the older `liftoff_ticks.csv` has `beaker_z_m` and the trace's bench-contact column, but not pad contact). Whether to add it is open.
6. **Verdicts are not computed** (as the brief asked). The closure range, force threshold, confirming time and stillness limits are still undecided.
7. **`docs/design_plan.md`** was not edited (not in the allowed list for this task).
8. **Untracked files outside `docs/reports/`** (`.claude/`, `compare_nosag.py`, `extract_height_data.py`, `measstart`) and the `deps/AgileX_Tracer` display question are unchanged.
