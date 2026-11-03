# Bounce tests: freezing the sag trim in place DOWN, and giving the arm the true beaker mass

Date: 2026-11-03. Branch `held-weight-ramp-sag-off`, HEAD `29b8583`. Left uncommitted. The main repo received only this report; all code changes were made in a scratch copy, which has been removed.

Labels: **shown by the data** (read straight from a saved file or log line), **calculated from the data** (a formula applied to saved values, stated), **guess** (a possible explanation not tested here), **unknown** (not determined).

## Summary in plain words

- **Did the control match?** Yes. Run A (200 g, both switches off, with the bridge target logged) matches the existing `run_200g` on all 58 shared columns, in all 14,381 rows: zero differing cells. The bridge columns exist only in A and are not compared. (**shown by the data**)
- **Trim frozen during place DOWN (run B):** the swing fell from 9.39 mm (existing 200 g run) to **6.41 mm**, and the −4.4 mm undershoot disappeared. But the step **failed**: the hand stopped 3.4 to 3.5 mm from its waypoint, over the 3 mm limit, and the DOWN step timed out after 9 s. A retry failed too (5.6 mm). The reason is that the sag trim also carries a horizontal (x) correction, which in the normal run moves from −5.5 mm to −1.0 mm during DOWN and brings the hand to the waypoint. Freezing the trim holds that x correction too. (**shown by the data**)
- **True mass (runs C and D):** the lift rise fell from 4.03 to **2.81 mm** at 200 g (against 4.03 mm existing) and from 4.13 to **2.89 mm** at 150 g. Both are now under the 3 mm line. The predicted drop was 2.2 to 2.3 mm; the measured drop was about 1.2 mm, roughly half. (**shown by the data; the predicted values are from the earlier report and are calculated**)
- **Pass or fail against 3 mm:**

| run | rise | drop | swing | step completed? | result |
|---|---|---|---|---|---|
| control A, 200 g (existing 200 g) | 4.03 **fail** | 2.12 pass | 9.39 **fail** | yes | fail |
| B, 200 g, trim frozen in place DOWN | 4.03 **fail** | 2.12 pass | 6.41 **fail** | **no** (DOWN not reached, 3.45 mm error) | fail |
| C, 200 g, true mass | 2.81 pass | 2.28 pass | 9.32 **fail** | yes | fail (swing) |
| D, 150 g, true mass | 2.89 pass | 2.33 pass | 8.98 **fail** | yes | fail (swing) |
| existing 150 g (for D) | 4.13 **fail** | 2.14 pass | 9.00 **fail** | yes | fail |

No run passes all three. The swing fails in every run.

---

## 1. State at the start (step 1)

- `git status`: no tracked changes in the main tree. Untracked files only (the `docs/reports` files from earlier tasks, `deps/AgileX_Tracer`, `.claude/`, and some scratch scripts). `src/control_loop.py` is committed (`git diff --stat` empty), so nothing was copied for it.
- Branch `held-weight-ramp-sag-off`; `git log -1`: `29b8583 Progress: src/control_loop.py committed (a57cc4f)`.
- `uptime` 17:10: load average 0.44. `free -m`: 7424 MB total, 1980 MB used, 4320 MB free, 5443 MB available, swap 887 MB used. `systemd-oomd`: active.
- `data/` is git-ignored (`.gitignore:20`), so the outputs are in `data/bounce_tests_20261103/`.

## 2. The scratch copy and the switch code (step 2)

**Scratch copy:** `git worktree add --detach /tmp/bounce_test_wt HEAD`. As `experiments/cc_bench_regen.sh` does, the venv and the mesh folder were linked in (`.venv` and `deps/OpenArm-Combined`, pointing at the main repo's copies). The scratch copy also needed the read-only inputs the stance finder loads by relative path (`data/reach_map`, `data/bench_region_cache`, `data/stance_map`, `data/home_pose_search`, `data/beaker_hotplate_layout`, `data/gravity_ff`). I copied those (15 MB total) rather than linking all of `data/`, so runs could not write caches back into the main tree. The first two attempts of run A failed because those inputs were missing (`StanceSelectionError: no stance covers ... beaker`); the outputs were not used. After the copy, A ran cleanly.

**Note on the environment-variable pattern:** the brief cites `OPENARM_GRAVITY_COMP`, but that switch was deleted on 2026-10-26 (decisions-log M185 update). The new switches follow the same style: an environment variable read once when the module loads, off by default.

**The patch:** `data/bounce_tests_20261103/test_switches.patch` (`git diff -- src experiments` in the scratch copy; the launcher is a new file). `python -m py_compile` passed for `src/mpvi_modules.py`, `src/control_loop.py` and `experiments/bounce_test_run.py`. The main repo's `deps/OpenArm-Combined` is a submodule; the symlink that replaces it shows in `git diff` as a change, so it was left out of the patch.

**Switch 1: `OPENARM_TEST_FREEZE_TRIM_DOWN`** (`src/mpvi_modules.py`, the exact lines from the patch):

```
# TEST ONLY (bounce tests, 2026-11-03; not a design setting; off by default): OPENARM_TEST_FREEZE_TRIM_DOWN=1 holds the sag trim at its current value
# during the place DOWN step (it does not integrate and is not zeroed or reset), then it resumes as normal. The real robot never sets this.
TEST_FREEZE_TRIM_DOWN = os.environ.get("OPENARM_TEST_FREEZE_TRIM_DOWN", "0") == "1"
```

and, in the move branch (about line 393), the existing trim call is skipped only for the place's DOWN move:

```
            if not (TEST_FREEZE_TRIM_DOWN and S["ctx"].get("primitive") == "place" and st["name"] == "DOWN"):   # TEST ONLY (OPENARM_TEST_FREEZE_TRIM_DOWN): during place DOWN the trim keeps its value (the call is skipped)
                _sag_trim_step(C, cfg, model, side, t, p_ref - p_now, Rotation.from_matrix(R_ref @ R_now.T).as_rotvec(), dt)
```

The brief asked to reuse the existing freeze. I did not: the existing freeze lives inside `_sag_trim_step` (`hmr.ramp_running`), so reusing it would have meant changing the ramp check. Skipping the existing call at the one place where the DOWN step is known keeps the trim's value unchanged, which is what the brief asks, and no new correction algorithm was written. The freeze covers all of the trim (x, y, z and rotation), because that is what the existing call integrates.

**Switch 2: `OPENARM_TEST_TRUE_MASS`** (`src/mpvi_modules.py`):

```
# TEST ONLY (bounce tests, 2026-11-03; not a design setting; off by default): OPENARM_TEST_TRUE_MASS=1 uses SIMULATOR TRUTH. At lift-off the held-mass ramp
# is given the beaker's TRUE mass from the simulated model, in place of the weight estimate. The real robot cannot read the true mass, so this only
# shows what the controller would do with a perfect weight; it is never a real-robot setting.
TEST_TRUE_MASS = os.environ.get("OPENARM_TEST_TRUE_MASS", "0") == "1"
```

Helper (new, after `_sag_trim_step`):

```
def _true_beaker_mass(model):
    """TEST ONLY (OPENARM_TEST_TRUE_MASS): the mass (kg) of the whole beaker from the simulated model, the subtree of the beaker body (glass plus liquid).
    Simulator truth: the real robot cannot read it."""
    return float(model.body_subtreemass[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, BEAKER_CFG["body"])])
```

At lift-off (the single `set_target` call, about line 476):

```
                    est = liftoff_estimate(m_now, rise, hpz)
                    fed = _true_beaker_mass(model) if TEST_TRUE_MASS else est       # TEST ONLY (OPENARM_TEST_TRUE_MASS): the ramp takes the simulator's true mass instead of the estimate
                    hmr.stop_rate(model, side, t)
                    hmr.set_target(model, side, fed, t, obs["tick"])
                    W2.update(state="detected", m_det=m_now, rise=rise, hpz=hpz, est=est, t_det=t)
                    S["ctx"]["held_mass_kg"] = fed
                    if TEST_TRUE_MASS:
                        print(f"[TEST ONLY] true beaker mass {fed:.4f} kg written into the held-mass ramp instead of the estimate {est:.4f} kg", flush=True)
```

The start-of-run print (about line 181) is:

```
    if TEST_TRUE_MASS:                                                 # TEST ONLY: printed once at run start, to check it against the nominal mass (0.150, 0.200 or 0.250 kg)
        print(f"[TEST ONLY] true beaker mass at run start (subtree of {BEAKER_CFG['body']}): {_true_beaker_mass(model):.4f} kg", flush=True)
```

Nothing else in the ramp changed: 1.0 s min-jerk, the trim freeze during the ramp, and the detection rule are as before.

**Mass check (required before the true-mass runs):** `data/bounce_tests_20261103/check_true_mass.py` built the scene for each liquid mass (no stepping) and read the beaker's subtree mass. Results (**shown by the data**): 0.150, 0.200 and 0.250 kg, all **MATCH** the nominal values. The true-mass runs were then allowed.

**Bridge logging (logging only):** the scratch launcher `experiments/bounce_test_run.py` reads `BRIDGE_ACCEPTED_LOG` from `src/control_loop.py` (set at lines 663 to 665, every bridge update; initialised at line 108) and writes, on each tick row, `bridge_tick` and `bridge_L1` to `bridge_L7` (the left arm's accepted target, held between bridge updates). It is written in the per-tick CSV `liftoff_ticks.csv` (row built in `torques_logged`). The control loop itself was not changed. Those columns exist in all of this session's runs (A, B, C, D).

## 3. Runs (step 3 and 4)

All runs: scratch copy, `experiments/run_capped.sh` (6 GB cap), tmux session `bounce_tests` / `bounce_tests2` / `bounce_tests3`, one at a time, through `data/bounce_tests_20261103/chain1.sh`, `chain2.sh` and `chain3.sh`.

| run | settings | start | end | length | exit | peak memory (resident) |
|---|---|---|---|---|---|---|
| A try 1 (failed, inputs missing) | 200 g, both off | 17:12:30 | 17:13:33 | 1:02.6 | 1 | 2.94 GB |
| A try 2 (failed, inputs missing) | 200 g, both off | 17:13:33 | 17:14:11 | 0:38.5 | 1 | 2.67 GB |
| A2 (control, used) | 200 g, both off | 17:14:42 | 17:17:47 | 3:05.5 | 0 | 4.14 GB |
| B try 1 | 200 g, trim frozen in DOWN | 17:18:15 | 17:21:49 | 3:33.2 | 1 (plan failed) | 4.21 GB |
| B try 2 (retry) | same | 17:21:49 | 17:25:30 | 3:40.9 | 1 (plan failed) | 4.20 GB |
| C | 200 g, true mass | 17:25:30 | 17:28:08 | 2:37.7 | 0 | 4.21 GB |
| D | 150 g, true mass | 17:28:08 | 17:31:24 | 3:16.4 | 0 | 4.21 GB |

B "exit 1" is not a crash. It is the planner's own result: the DOWN step failed (see section 4). Both tries failed identically, so the rule for retrying a killed run does not apply, and no third try was made.

### Run A (control) against the existing `run_200g`

Compared on the 58 shared columns of `liftoff_ticks.csv` (`/tmp/bounce_analysis/compare.py`): **14,381 rows, 0 rows with any difference, 0 differing cells.** Columns only in A: the bridge columns. (**shown by the data**)

## 4. Measurements (step 5)

Definitions as in the earlier report (`/tmp/bounce_analysis/measure.py`, which reproduces the existing numbers exactly):
- **Rise / drop:** largest and smallest value of e (hand minus reference, mm) during the pick's UP step.
- **Place DOWN swing:** largest minus smallest e from DOWN start to 3 s later.
- **First peak / first trough:** the first local maximum above +1 mm and the first local minimum below −1 mm after DOWN start (I changed the trough rule from the first extremum to "first beyond 1 mm" to skip a noise wiggle at 12555; the existing numbers are unchanged by this).
- **Ringing period:** ticks between the first two peaks.
- **Trim z:** the z component of the sag trim over the DOWN step.

### 4a. Table of the measurements

| quantity | existing 150 g | existing 200 g = A | B (freeze, 200 g) | C (true, 200 g) | D (true, 150 g) |
|---|---|---|---|---|---|
| lift rise (mm) | +4.13 (11802) | +4.03 (11947) | +4.03 (11947) | +2.81 (11939) | +2.89 (11794) |
| lift drop (mm) | −2.14 (12020) | −2.12 (12165) | −2.12 (12165) | −2.28 (12168) | −2.33 (12023) |
| place DOWN swing (mm) | 9.00 | 9.39 | 6.41 | 9.32 | 8.98 |
| first peak (mm, tick) | +4.89 (12458) | +4.99 (12602) | +5.90 (12611) | +4.95 (12603) | +4.86 (12458) |
| first trough (mm, tick) | −4.11 (12514) | −4.40 (12659) | **none below −1 mm** (min −0.52 at 12697) | −4.38 (12659) | −4.11 (12514) |
| ringing period (ticks) | 170 | 172 | none (no second peak) | 171 | 170 |
| trim z over DOWN (mm) | −1.08 → −5.20 (range 4.78) | −0.99 → −4.75 (range 4.29) | **−0.987 flat for the whole first DOWN** | +1.35 → −2.36 (range 4.25) | +1.34 → −2.77 (range 4.78) |
| held mass at detection (kg, raw) | 0.1791 | 0.2280 | 0.2280 | 0.2280 | 0.1791 |
| held mass at ramp end (kg) | 0.1754 (estimate) | 0.2245 (estimate) | 0.2245 | **0.2000 (true)** | **0.1500 (true)** |
| held mass at detection +75 ticks (kg) | — | 0.2262 | — | 0.2140 | 0.1645 |
| true mass (kg) | 0.150 | 0.200 | 0.200 | 0.200 | 0.150 |
| held mass at the rise tick (kg) | 0.1763 | 0.2254 | 0.2254 | 0.2095 | 0.1602 |
| place DOWN start / end (ticks) | 12410 / 12722 | 12554 / 12862 | 12554 / (failed at 13904) | 12554 / 12862 | 12410 / 12722 |
| lift-off detection (tick) | 11707 | 11851 | 11851 | 11851 | 11707 |

**Timeline:** the detection tick, the UP step, the DOWN start and end, and the OPEN step all fall on the same ticks in C and D as in the existing runs. The only change at lift-off is the ramp target (the true mass instead of the estimate). Calculated from the logs. (**shown by the data**)

**Held term over the ramp-in (shown by the data):**
- 200 g, C: the ramp starts at 0.2280 kg (raw at detection), goes to 0.2140 kg at 75 ticks and to 0.2000 kg at the ramp end (1.0 s), the true mass. The existing run went to 0.2245 kg, so the held term is 24.5 g lower.
- 150 g, D: the ramp starts at 0.1791 kg, goes to 0.1645 kg at 75 ticks and to 0.1500 kg at the ramp end. The existing run went to 0.1754 kg, so the held term is 25.4 g lower.

**Did the predicted drop in the rise appear?** The predicted drop from removing the over-read was 2.2 mm (200 g) and 2.3 mm (150 g) (from the earlier report, calculated from the config stiffness). The measured drop was **1.22 mm (200 g: 4.03 → 2.81)** and **1.24 mm (150 g: 4.13 → 2.89)**. So about 55 % of the predicted drop appeared. The rise is still above zero, and the rise numbers for a 0.05 kg-scale change in held mass are only three points, so the match is partial. (**calculated from the data, against the earlier prediction**)

**Did the DOWN swing change?** Only slightly: 9.39 → 9.32 mm (200 g, −0.07 mm) and 9.00 → 8.98 mm (150 g, −0.02 mm). The DOWN swing is not driven by the held mass in these runs. (**shown by the data**)

### 4b. Run B: the trim frozen in place DOWN

- **Was the trim constant during DOWN? Yes, in the first DOWN (shown by the data).** The sag trim's z, x and y components stayed at **z −0.987 mm, x −5.476 mm, y −0.658 mm** from the start of the first place DOWN (tick 12554) through tick 13892, the end of that attempt. (The measurement's "trim z range 9.5 mm" is a mistake of the measurement window, which covered the second DOWN after the recovery; the first DOWN is flat.)
- **The step failed.** The planner's message: `DOWN: not reached after 9.0 s: 3.6 mm and 0.7 deg from the waypoint` (log line 304). The trace shows the pad error to the waypoint at 3.2 to 3.45 mm from about tick 12704 onward, above the 3 mm tolerance. The vertical error (e) settled near +0.19 mm, so the remaining error is horizontal, which the frozen x trim (−5.5 mm) does not correct.
- **Why:** in the normal run (control A) the x trim goes from −5.48 mm to −1.0 mm during DOWN (about 4.5 mm of correction), while z goes from −1.0 to −5.2 and back to −1.4 (the ringing). (**shown by the data**) Freezing the whole trim removes the x correction that brings the hand to the waypoint.
- **Retry:** after recovery, the DOWN step was tried again (planner's attempt 2). It failed again: `not reached after 9.8 s: 5.6 mm and 1.2 deg`, with a steady +5.3 mm vertical error. (**shown by the data**)
- **Swing and ringing:** the swing in the first DOWN was 6.41 mm (against 9.39 mm in A, a fall of 2.98 mm, or 32 %). The −4.4 mm undershoot disappeared and the first peak grew from +4.99 mm to +5.90 mm. No second peak appeared, so the ringing period cannot be measured. (**shown by the data**)
- **What it means:** shown by the data, the −4.4 mm undershoot is absent when the trim is frozen. That the trim's z component causes the undershoot is a **guess**, not measured. Separately, shown by the data, the same trim carries the x correction that the hand needs to reach the waypoint, so the 6.41 mm swing is not a valid pass for a run that never finished the step.

### 4c. Pass or fail (against 3 mm, as David set it)

| run | rise | drop | swing | step completed |
|---|---|---|---|---|
| A (control, 200 g) | 4.03 **FAIL** | 2.12 pass | 9.39 **FAIL** | yes |
| B (trim frozen, 200 g) | 4.03 **FAIL** | 2.12 pass | 6.41 **FAIL** | **NO** (DOWN not reached: 3.45 mm after 9 s; retry 5.6 mm) |
| C (true mass, 200 g) | 2.81 pass | 2.28 pass | 9.32 **FAIL** | yes |
| D (true mass, 150 g) | 2.89 pass | 2.33 pass | 8.98 **FAIL** | yes |

Note for B: the rise and drop are measured before the freeze starts (the freeze begins at DOWN), so they equal A. The swing is the only number that changed, and the step itself failed. A B result cannot be counted as a pass on any measure.

### 4d. Bridge accepted target around DOWN start (run A, saved)

The bridge's accepted target (left arm joint 1, radians) is updated every 3 ticks. Servo force is the actuator force on joint 1 (N·m). Hand error e is hand minus reference (mm). DOWN starts at tick 12554.

| tick | bridge target (rad) | servo force (N·m) | e (mm) |
|---|---|---|---|
| 12544 | −0.50249 | −8.117 | 0.90 |
| 12553 | −0.50067 | −8.008 | 1.06 |
| 12555 | **−0.50559** (first step, −0.0049) | −8.109 | 0.00 (trace restarts at the DOWN move) |
| 12558 | −0.51028 | −8.431 | 0.07 |
| 12561 | −0.51134 (target turns back) | −8.572 | 0.25 |
| 12564 | −0.51120 | −8.467 | 0.53 |
| 12570 | −0.51051 | −8.191 | 1.33 |
| 12580 | −0.50841 | −8.002 | 2.73 |
| 12590 | −0.50550 | −7.910 | 3.99 |
| 12601 | −0.50124 | −7.851 | **4.98 (peak)** |
| 12613 | −0.49728 | −7.878 | 4.22 |

**Lead or lag (shown by the data):** the target's first step is at tick 12555, the servo force starts to change at 12558 (3 ticks later), and the hand error starts to rise at 12562 (7 ticks after the target's first step). So the target **leads** the hand. The target reverses at 12561, and the hand error keeps rising until 12601 (40 ticks after the reversal), so the hand **lags** the target. The servo force follows the target within about 3 ticks.

Runs B, C and D also saved the bridge target; only run A's is shown here.

## 5. Kill check and memory (step 6)

- **systemd-oomd:** active throughout.
- **Journal** (`journalctl --since "2026-11-03 17:09"`, 46 lines): no OOM, kill, or memory-pressure lines. (**shown by the data**)
- **dmesg:** `Operation not permitted` for this user, so the kernel log could not be read. **Unknown** for kernel-level kills.
- **Run peaks:** 2.7 to 4.2 GB resident (table in section 3), all under the 6 GB cap. No run was killed by the cap.
- **Memory at the end** (`free -m`): 7424 MB total, 1893 MB used, 3571 MB free, 5530 MB available, swap 896 MB used (about the same as at the start: 887 MB). No pressure beyond the swap level that was already there.
- Run exit codes: A2, C, D 0; B 1 (planner failure, not a crash); A tries 1 and 2 were 1 (missing inputs in the scratch copy, fixed).

## 6. Exact commands

From the main repo `/home/vboxuser/projects/openarm-tracer-sim` unless noted:
- State: `git status --short`, `git log -1 --oneline`, `git branch --show-current`, `uptime`, `free -m`, `systemctl is-active systemd-oomd`, `git diff --stat -- src/control_loop.py`.
- Scratch: `git worktree add --detach /tmp/bounce_test_wt HEAD`; `ln -s .../.venv /tmp/bounce_test_wt/.venv`; the mesh folder link (after `rmdir`); copying `data/{reach_map,bench_region_cache,stance_map,home_pose_search,beaker_hotplate_layout,gravity_ff}` into `/tmp/bounce_test_wt/data/`.
- Edits in `/tmp/bounce_test_wt`: `src/mpvi_modules.py` (the switch lines above); `experiments/bounce_test_run.py` (copy of `data/bounce_check_20261103/run_mass.py` with ROOT set from the file's own location, the bridge import, the bridge row lines and the bridge columns).
- Compile and patch: `python -m py_compile src/mpvi_modules.py experiments/bounce_test_run.py src/control_loop.py`; `git add -N experiments/bounce_test_run.py`; `git diff -- src experiments > data/bounce_tests_20261103/test_switches.patch`.
- Mass check: `bash /tmp/bounce_test_wt/experiments/run_capped.sh <log> python data/bounce_tests_20261103/check_true_mass.py` (inside `chain1.sh`).
- Runs: `data/bounce_tests_20261103/chain1.sh` (A, failed twice), `chain2.sh` (A2, control), `chain3.sh` (B, C, D; `env OPENARM_TEST_FREEZE_TRIM_DOWN=1` for B and `env OPENARM_TEST_TRUE_MASS=1` for C and D, each through `run_capped.sh`).
- Measurement: `python /tmp/bounce_analysis/compare.py <A csv> <existing csv>`; `python /tmp/bounce_analysis/measure.py <run dir> <true kg>` for each run; inline Python for the per-tick checks (trim components and x, the bridge table, the error column `err_m`, the held-mass ramp values). The inline snippets are not saved as files; their numbers are in this report.
- Kill check: `journalctl --since "2026-11-03 17:09:00" --no-pager | grep -i -E 'oom|killed'`; `dmesg` (not permitted); `free -m`; `grep Elapsed / Maximum resident / exit` in the run logs.
- Cleanup: `unlink /tmp/bounce_test_wt/.venv`, `unlink /tmp/bounce_test_wt/deps/OpenArm-Combined` (the symlinks only, so the removal cannot reach into the main repo); `git worktree remove --force /tmp/bounce_test_wt`; `git worktree prune`.
- Main-tree check: `git status --short` (no tracked file listed; only untracked files, including this report); `git diff --stat` (empty).

## 7. NOT BUILT, needs decision

1. **A z-only freeze** (hold the z component of the trim during DOWN, but keep integrating x and y). The brief asked for the whole trim to hold, which is what was built. The result suggests a z-only freeze is the more useful test, but that is a design choice and was not built. **Decision needed from David.**
2. **Trim frozen with a different DOWN length or tolerance.** Not tried. The step failed at the 3 mm tolerance, and I did not change any setting to make it pass.
3. **Bridge accepted target for runs B, C and D.** Saved, but only run A's table is shown here.
4. **Kernel log (dmesg):** not readable by this user. Kills in the kernel are **unknown** from this check; the journal shows none.
5. **Why the first DOWN of run B settled at +0.19 mm vertically but 3.4 mm horizontally** is not measured per axis beyond the trim values and the total error. Not analysed further.
6. **Mechanism of the 55 % drop-in-rise** (the measured 1.2 mm drop against the 2.2 mm predicted): the three beaker masses are three points, and the data cannot separate the mass effect from the change in the ramp's end point. **Unknown.**
7. **Why the second DOWN attempt in run B held a steady +5.3 mm vertical error** (the trim was at −0.009 mm at its start). Not explained by these data. **Unknown.**
8. **The earlier Nov 3 analysis** (`docs/reports/bounce_analysis_20261103.md`) followed the trim's z component only. Its x component is larger (−5.5 mm at DOWN start in run A) and is what the freeze test breaks. That report's section 3 should be read with this in mind. Not edited here, because the brief allows only this new report.
