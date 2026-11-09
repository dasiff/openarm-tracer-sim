# Squeeze counts the closing force only (decision M251, Nov 4, 2026)

## Summary in plain words

- **Sign convention, confirmed from the logged data and the code.** Closing is negative. While holding the beaker the logged force is −16.0 N (the controller's 8 N per-finger cap, doubled because one actuator drives both fingers). Open is about +0.04 N. An opening push at release is positive: +3.785 N at tick 13014 and +1.603 N at tick 13015.
- **The change (commit `0431fb6`).** The squeeze test is now "force at or below −2 N" instead of "absolute force at or above 2 N". An opening push no longer counts as a squeeze.
- **Effect on the real 200 g run (replayed on the logged data, no new simulation).** Holding turn-on stays at tick 11014. Holding turn-off moves from **tick 13015 to tick 13014**, one tick earlier, which is what was expected. **Only that one tick differs in either hand.** The right hand, the still verdict and the closure flags are identical everywhere.
- **Synthetic tests all pass**: a closing force above 2 N turns holding on after the confirming time; an opening force of the same size does not; an opening push during a hold turns it off at once. Two extra cases also pass: exactly −2.0 N counts as squeeze, −1.9 N does not, and a position out of range still turns holding off.
- **Other places that read the squeeze**: checked, and only the holding test was changed (see the table below).

Terms: **absolute value** is the size of a number with its sign removed (|−16| = 16, |+3.8| = 3.8). **Replay** runs the verdict code again on recorded data; no simulation runs. **Turn-on confirming time** (0.1 s, 15 ticks) still applies only to turning holding on.

Labels: **shown by the data/code** (read from a file or saved value, with file and line); **calculated** (computed here, method stated); **guess**; **unknown**.

## Step 1: state

- Branch `held-weight-ramp-sag-off`; at the start no tracked change; HEAD `112cbc4`. The remote `origin/main` is `58e756f`. No run was active. Untracked (older, not touched): `deps/AgileX_Tracer`, `.claude/`, `compare_nosag.py`, `extract_height_data.py`, `measstart`, and the reports `holding_changes_20261104.md` and `signal_trace_stage1b_20261103.md` (both uncommitted, as requested earlier).
- Next free decision number: **M251**.

## Step 2: sign convention (confirmed)

**From the logged data** (`data/signal_trace_1b_20261103/r2_on/run/episode/signal_trace.csv`, left hand, 200 g):

| Tick | Step | Gripper target (m) | Finger1 (m) | Logged force (N) | Servo formula kp·(target − finger1) (N) |
|---|---|---|---|---|---|
| 9700 | PICK OVER (open) | 0.0440 | 0.043927 | +0.0439 | +0.044 |
| 11010 | PICK CLOSE | 0.0000 | 0.037512 | −16.0000 | −22.507 (clipped to −16 N) |
| 11200 | PICK WEIGH (holding) | 0.0000 | 0.037516 | −16.0000 | −22.510 (clipped) |
| 13013 | PLACE OPEN, last closed tick | 0.0000 | 0.037516 | −16.0000 | −22.510 (clipped) |
| 13014 | PLACE OPEN, release starts | 0.0440 | 0.037623 | **+3.7850** | +3.826 |
| 13015 | PLACE OPEN | 0.0440 | 0.038703 | **+1.6034** | +3.178 |
| 13016 | PLACE OPEN | 0.0440 | 0.040110 | +0.5555 | +2.334 |
| 13018 | PLACE OPEN | 0.0440 | 0.042244 | −0.0239 | +1.054 |
| 13030 | PLACE OPEN (open) | 0.0440 | 0.043927 | +0.0430 | +0.044 |

Facts from the same data: the logged force is never zero; 2,945 ticks are negative and 11,437 positive. Where the target is 0 and the fingers hold the beaker, the force is exactly −16.0 N. The small differences from the formula at release come from a damping term (the servo's velocity term, proportional to finger speed, which opposes motion), which I did not measure separately.

**From the code (shown by the code):**
- `src/arm_actuators.py` line 37: `act.gainprm[0] = kp`; line 39: `act.biasprm[:3] = [0.0, -kp, -kv]`. MuJoCo's position actuator force is therefore kp·ctrl − kp·q − kv·qdot, where ctrl is the commanded position, q the finger position and qdot its speed (MuJoCo's documented formula; not checked in this repo beyond the lines above).
- `src/arm_actuators.py` line 63: the gripper's kp is `fingers_per_actuator` × 300 = 600 N/m.
- `src/mpvi_modules.py` line 417: while closing, the commanded position is 0.0 m; when opening it is `open_g` (0.044 m, `src/mpvi_modules.py` line 159).
- `src/signal_trace.py` line 113 (before this change) wrote the raw `data.actuator_force` value with its sign.

**Conclusion:** closing grip = negative force; opening push = positive force; open (no grip) = small positive force. The threshold is therefore applied to the negative side.

## Step 3: the one change, and every other reader of the squeeze

**Change** (`src/signal_trace.py`, `_verdicts`, lines 127 to 129):
- old (commit `112cbc4`): `squeeze_ok = abs(force) >= cfg["squeeze_threshold_N"]`
- new (commit `0431fb6`): `squeeze_ok = force <= -cfg["squeeze_threshold_N"]`, with the decision comment M251 on lines 127 and 128.
- The docstring of `_verdicts` (lines 118 to 119) describes the new test.

**Every place that reads the gripper force, and what the change did there:**

| Place | What it reads | Absolute value? | Change made? |
|---|---|---|---|
| `src/signal_trace.py` line 129 (holding squeeze test) | the actuator force | yes (before) | **yes**: now closing force only |
| `src/episode_runner.py` line 217 (10 Hz log column `left_finger_force_N`) | `abs(actuator force) / 2` | yes | **no**. It is a log column (a magnitude, per finger), not a verdict. Changing it would go beyond the squeeze test. An opening push still appears as a positive number there (1.89 N at tick 13014). |
| `experiments/grip_failure_weight_model.py` line 258 | `abs(actuator force) / cap`, an analysis script | yes | **no**. Not part of the holding verdict. |
| Weigh trigger (`src/mpvi_modules.py` around lines 455 and 502 to 504; `src/weight_estimate.py` `liftoff_check`, line 117) | pad height rise and the arm's seven joints (`act_ids` are the arm joints) | n/a | **no** gripper force is read, so nothing to change. |
| Still verdict (`_verdicts`, speed and its counter) | hand speed only | n/a | **no**. |

## Step 4: config comment and decision row

- `src/task_config.py`, `HOLD_STILL_CFG`, lines 648 to 650: the squeeze comment now says that squeeze means the CLOSING force only, that an opening push does not count, that the test is force at or below −2 N, and the sim values seen. It still says PROVISIONAL and SIM-ONLY (line 646 header and the value line).
- `docs/mpvi_decisions.md`, row **M251** (line 264), dated 2026-11-04: David's decision, the reason (a positive opening push at release counted as squeeze, so holding stayed on one tick longer), what is unchanged, and that it **refines** the squeeze rule in M246 and M248. M246 and M248 are not rewritten.

## Step 5: replay on the logged 200 g trace

Method: the verdict code from commit `112cbc4` (absolute-value squeeze) is the "before"; the current code (closing-only, commit `0431fb6`) is the "after". Both use the current `HOLD_STILL_CFG` values, so only the squeeze rule differs. The inputs are the logged squeeze-related values from the 200 g stage 1b run. The "before" replay reproduces the logged holding verdicts exactly for both hands (0 mismatches), which validates the replay.

**Holding changes (left hand):**

| Event | Before (absolute value) | After (closing only) | Same? |
|---|---|---|---|
| Holding turns ON | tick 11014 (PICK CLOSE) | tick 11014 (PICK CLOSE) | yes |
| Holding turns OFF | tick 13015 (PLACE OPEN) | **tick 13014** (PLACE OPEN) | **no (one tick earlier)** |
| Holding ON stretch | 11014 to 13014 (2,001 ticks) | 11014 to 13013 (2,000 ticks) | differs by one tick |

**Every tick where anything differs, in either hand:**

| Hand | Tick | Squeeze before | Squeeze after | Holding before | Holding after |
|---|---|---|---|---|---|
| left | 13014 | 1 (absolute value 3.785, at least 2) | 0 (+3.785 is not at or below −2) | 1 | 0 |
| right | none | — | — | — | — |

Only that one tick differs in the whole run, in either hand. The still verdict and the closure flag are identical before and after in both hands.

**Release, with the new rule (left hand):**

| Tick | Force (N) | Finger1 (m) | Squeeze (closing) | Position in 0.005 to 0.042 m | Holding |
|---|---|---|---|---|---|
| 13013 | −16.0000 | 0.037516 | yes | yes | ON |
| 13014 | +3.7850 | 0.037623 | **no** (opening push) | yes | **OFF** |
| 13015 | +1.6034 | 0.038703 | no | yes | OFF |
| 13018 | −0.0239 | 0.042244 | no | **no** (above 0.042) | OFF |

With the new rule, the **squeeze fails first, at tick 13014**. The **position fails 4 ticks later, at tick 13018** (before: 3 ticks, because the squeeze failed at 13015 under the old rule).

## Step 6: synthetic tests (no simulation)

| Test | Input | Expected | Result |
|---|---|---|---|
| Closing force turns holding on | force −16 N, position 0.0375 m, 20 ticks | ON at the 15th tick (index 14) | ON at index 14: pass |
| Opening force of the same size does not | +16 N, 40 ticks | never ON; squeeze flag 0 | never ON; squeeze 0: pass |
| Opening push during a hold turns it off at once | −16 N for 20 ticks, then +3.785 N | OFF on the first push tick | OFF; squeeze 0: pass |
| Threshold edge, closing | −2.0 N | squeeze 1 | 1: pass |
| Threshold edge, just under | −1.9 N | squeeze 0 | 0: pass |
| Position out of range while closing | −16 N, position 0.0439 m | OFF at once | OFF: pass |

## Step 7: the confirming time and the other settings

Unchanged, as asked: the 0.1 s (15-tick) confirming time for turning on; the immediate turn-off when either test fails; the 0.005 to 0.042 m position range; the 2 N threshold; and all stillness settings (0.005 m/s, 75 ticks).

## Commits

| Commit | What it changed |
|---|---|
| `0431fb6` | Squeeze counts the closing force only (M251): `src/signal_trace.py` (line 129, and the docstring), `src/task_config.py` (squeeze comment), `docs/mpvi_decisions.md` (row M251), `docs/progress.md` (entry) |

Not pushed. `origin/main` is still `58e756f`.

## Files

- Changed and committed: `src/signal_trace.py`, `src/task_config.py`, `docs/mpvi_decisions.md`, `docs/progress.md`.
- Created, uncommitted: `docs/reports/squeeze_closing_only_20261104.md` (this report).
- Scratch files outside the repo: `/tmp/holdchange_20261104/prev_signal_trace.py` (commit `112cbc4`'s signal trace), `/tmp/holdchange_20261104/replay_m251.py` (the replay), `/tmp/holdchange_20261104/m251_left.npz` and `m251_right.npz` (per-tick results).

## Commands (summary)

    git status --short ; git log --oneline -3 ; git branch --show-current ; uptime ; free -m ; systemctl is-active systemd-oomd
    git show 112cbc4:src/signal_trace.py > /tmp/holdchange_20261104/prev_signal_trace.py
    python -m py_compile src/signal_trace.py src/task_config.py
    grep -rn "abs(.*actuator_force\|abs(force)" src experiments --include=*.py      # other readers of the squeeze
    python -I - (inline)                                                               # sign check from the logged data
    python -I /tmp/holdchange_20261104/replay_m251.py                                  # read-only replay, before vs after
    python -I - (inline)                                                               # synthetic tests
    git add src/signal_trace.py src/task_config.py docs/mpvi_decisions.md docs/progress.md && git commit ...   # 0431fb6

## NOT BUILT, needs decision

1. **The 10 Hz log column `left_finger_force_N`** still uses the absolute value (`src/episode_runner.py` line 217). It is a log column, not a verdict, so it was not changed. An opening push still shows as a positive number there. David decides whether that column should stay as a magnitude.
2. **`experiments/grip_failure_weight_model.py` line 258** uses the absolute value in an analysis script. Not changed.
3. **Holding now turns off at tick 13014,** when the force first turns positive (the opening push). This happens before the position fails (tick 13018). If a beaker slipped out, the position test would be the one to fail first. That case is still untested on real data.
4. **Stage 1b proof runs R1 to R3 were not repeated** for this change. The change is in the logging path, which runs only with `OPENARM_SIGNAL_TRACE=1`, and the bench and the switch-off run do not touch it. David decides whether a fresh proof is wanted.
5. **The M249 row cites line 653 for the lower edge.** The value is on line 656 (carried over from the earlier change). The row was not rewritten. A correction note could be added in a later docs commit.
6. **All six provisional values remain PROVISIONAL and SIM-ONLY.** Final values need hardware calibration.
7. **The still verdict** is off for the whole second before place DOWN in this run. Whether a re-weigh should be triggered by it is a separate decision.
