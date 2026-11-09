# Two changes to the holding verdict, and their effect on the 200 g run (Nov 4, 2026)

## Summary in plain words

- **Change 1 (commit `63e2d72`):** the lower edge of the finger-position range moved from 0.002 m to 0.005 m (decision M249). Reason: the pads touch each other in the sim at about 2.75 mm, so the old edge would count closing on nothing as "in range".
- **Change 2 (commit `112cbc4`):** holding is now on only while both tests pass (position in range and squeeze at or above threshold), and it turns off at once when either fails (decision M250). Before, it stayed on if the position left the range while the squeeze stayed high. The 0.1 s confirming time still applies only to turning holding on.
- **Effect on the real 200 g pick-and-place.** I did not need a new run. The verdict code only reads values that were already logged in the stage 1b run, so I replayed the real verdict code on those logged values. The replay reproduces the logged verdicts exactly for both hands (0 mismatches) before the change, so the replay is faithful.
  - **Holding turn-on (tick 11014) and turn-off (tick 13015): identical before and after.** So the turn-on, carry and weigh steps are identical, as expected.
  - The turn-off does **not** come earlier. I had expected the fingers to leave the range before the squeeze dropped, but in this run the **squeeze fails first**: at tick 13015 (3 ticks before the finger position fails, at tick 13018). So Change 2 has no effect on this run.
  - **The one difference I found:** the closure flag (position in range or not) changes for ticks 0 to 1167, the first 7.8 s of the run, when the fingers rest at about 2.7 mm before the grasp. It changes nothing in the holding verdict, because the squeeze is low then.
  - The still verdict is identical before and after.
- **Synthetic check of Change 2** (scratch script, no simulation): all four cases behave as decided. Holding turns on at the 15th tick; it turns off at once when the position leaves the range with the squeeze high; it turns off at once when the squeeze drops; and it turns off at once when the position falls below 0.005 m (closed on nothing).

Terms: a **verdict** is an on/off value computed from raw signals. A **replay** runs the same code on recorded data, not on a live simulation. **Squeeze** is the absolute value of the gripper's actuator force, in newtons (N), total for both fingers. **Position** is the finger1 joint position in metres (m).

Labels: **shown by the data/code** (read from a file or saved value, with file and line); **calculated** (computed here, method stated); **guess**; **unknown**.

## The changes as committed

**Change 1 (commit `63e2d72`).** `src/task_config.py`, `HOLD_STILL_CFG` (line 656): `grasp_closure_min_m` = 0.005 (was 0.002). Only the lower edge; the upper edge stays 0.042 m. The comment (lines 652 to 655) now says: the lower edge is decision M249; the pads touch each other at about 2.75 mm, which is a measurement from the sim (robot-only model, no beaker, contacts scanned in 0.05 mm steps), not from hardware; 2.75 mm plus the 2 mm margin is 4.75 mm, rounded to 5 mm; the upper edge is 2 mm below fully open. The row M249 was added to `docs/mpvi_decisions.md`.

**Change 2 (commit `112cbc4`).** `src/signal_trace.py`, `_verdicts`, the holding line (line 132; the decision comment is at lines 130 to 131):
- old: `if not squeeze_ok: holding = False`, and `elif hold_count >= 15: holding = True`. So a failed position test left the verdict unchanged.
- new: `self.holding[s] = both and (self.holding[s] or self.hold_count[s] >= holding_confirm_ticks)`. Holding is on only while both tests pass. Turning on needs both to hold for the confirming time; turning off is immediate when either fails.
- The docstring (lines 119 to 122) describes the new rule.
- `src/task_config.py`, the `holding_confirm_ticks` comment (lines 658 to 659): it now says the confirming time applies only to turning ON, and that turning OFF has no delay (M250).
- The row M250 was added to `docs/mpvi_decisions.md`. It notes that M250 changes the behaviour M248 left open. M248 is not rewritten.

**Config comments.** The comments on all six values still say PROVISIONAL and SIM-ONLY. The 1.65 N squeeze-on-nothing figure is an **untested calculation** (600 N/m × 0.00275 m ≈ 1.65 N), and the 2.75 mm touch point is a **sim measurement**, both as requested.

## Before and after, tick by tick (left hand, 200 g run, replayed)

| Event | Before (commit `85b77ce`, lower edge 0.002 m) | After (commit `112cbc4`, lower edge 0.005 m) | Same? |
|---|---|---|---|
| Holding turns ON | tick 11014 (in CLOSE, 136 ticks before CLOSE's last tick 11150) | tick 11014 | yes |
| Holding turns OFF | tick 13015 (in PLACE OPEN, 151 ticks after OPEN starts at 12864) | tick 13015 | yes |
| Holding ON stretch | 2,001 ticks (11014 to 13014) | 2,001 ticks | yes |
| Carry (PLACE OVER) and weigh (PICK WEIGH) holding | on, unchanged | on, unchanged | yes |
| Still verdict, whole run | identical | identical | yes |
| Right hand holding | never on | never on | yes |
| Closure flag (left and right), ticks 0 to 1167 | in range (2.7 mm is above 0.002 m) | out of range (2.7 mm is below 0.005 m) | **differs** |
| Closure flag, all other ticks | unchanged | unchanged | yes |

## Release, which test fails first (left hand)

| Tick | Step | Actuator force (N) | Finger1 (m) | Squeeze ≥ 2 N | Position in 0.005 to 0.042 m | Holding |
|---|---|---|---|---|---|---|
| 13013 | PLACE OPEN | −16.0000 | 0.037516 | yes | yes | ON |
| 13014 | PLACE OPEN | +3.7850 | 0.037623 | yes (absolute value 3.785) | yes | ON |
| 13015 | PLACE OPEN | +1.6034 | 0.038703 | **no** (absolute value 1.603) | yes | **OFF** |
| 13016 | PLACE OPEN | +0.5555 | 0.040110 | no | yes | OFF |
| 13017 | PLACE OPEN | +0.1265 | 0.041331 | no | yes | OFF |
| 13018 | PLACE OPEN | −0.0239 | 0.042244 | no | **no** (above 0.042) | OFF |

**The squeeze fails first, at tick 13015. The position fails 3 ticks later, at tick 13018.** The holding verdict turns off at 13015 in both the old and new rule, so the rule change does not move it in this run.

Two things in this table were not expected:
- At tick 13014 the actuator force is **+3.785 N** (pushing the fingers open), but the test uses the absolute value, so it counts as a squeeze and holding stays on one extra tick. The absolute-value rule was decided in M248; whether an opening push should count as a squeeze is a question for David (see below).
- The "151 ticks after release starts" figure in the old note is still correct for this run. The fingers did not leave the range before the squeeze dropped, so the turn-off did not come earlier.

## Recheck of the other numbers

- Stage 1b R1 to R3 proofs were not rerun, since neither change touches the switch-off path: the verdict code runs only inside `SignalTrace`, which exists only when `OPENARM_SIGNAL_TRACE=1`. The config change touches only the verdict thresholds, which no control code reads. The replay above is the check of the changed behaviour.
- The replay's validation: the old code, replayed on the logged inputs, reproduces the logged holding and still verdicts for both hands exactly (0 mismatches).

## Commits

| Commit | Change | Files |
|---|---|---|
| `63e2d72` | Change 1: lower edge 0.002 m to 0.005 m (M249) | `src/task_config.py`, `docs/mpvi_decisions.md` |
| `112cbc4` | Change 2: holding off when either test fails (M250); progress entry for both | `src/signal_trace.py`, `src/task_config.py`, `docs/mpvi_decisions.md`, `docs/progress.md` |

Not pushed. `origin/main` is still `58e756f`.

## Scratch files (outside the repo)

- `/tmp/holdchange_20261104/old_signal_trace.py` (the stage 1b code, from `git show 85b77ce`)
- `/tmp/holdchange_20261104/replay.py` (the replay), and `left.npz`, `right.npz` (per-tick results)
- `/tmp/signal_trace_1b_20261103_unit.py` is the earlier stage 1b check; the new synthetic check was an inline script (not kept).

## Commands (summary)

    git status --short ; git log --oneline -3 ; git branch --show-current ; uptime ; free -m ; systemctl is-active systemd-oomd
    python -m py_compile src/task_config.py                       # Change 1
    git add src/task_config.py docs/mpvi_decisions.md && git commit ...   # 63e2d72
    python -m py_compile src/signal_trace.py src/task_config.py   # Change 2
    git add src/signal_trace.py src/task_config.py docs/mpvi_decisions.md docs/progress.md && git commit ...   # 112cbc4
    git show 85b77ce:src/signal_trace.py > /tmp/holdchange_20261104/old_signal_trace.py
    python -I /tmp/holdchange_20261104/replay.py                  # read-only replay on the logged trace
    python -I - (inline)                                          # release table, closure-flag count, synthetic check

## NOT BUILT, needs decision

1. **Opening push counted as squeeze.** The squeeze test uses the absolute value. At release the actuator pushes the fingers open (+3.785 N at tick 13014), which counts as a squeeze, so holding stays on one tick longer. David decides whether the squeeze should count only the closing direction (negative force). Not changed here.
2. **Lower edge 0.005 m** is 4.75 mm rounded to 5 mm. David confirms the rounding.
3. **The 1.65 N squeeze-on-nothing figure** is a calculation, not a measurement. Whether the squeeze test separates "closed on nothing" from "holding" is untested. A closing-on-nothing run would test it. No such run exists (stage 1 found no option that removes the beaker without changing logic).
4. **Position-first slip case untested.** In this run the squeeze fails first, so Change 2's new turn-off (position failing while the squeeze is high) was never exercised on real data. Only the synthetic check covers it.
5. **Closure flag at the start of the run** (ticks 0 to 1167) changes with the new lower edge. It does not affect holding. Whether that is acceptable is a note for David.
6. **All six values remain PROVISIONAL and SIM-ONLY.** The final thresholds need hardware calibration.
7. **Still verdict** is off for the whole second before place DOWN in this run. Whether a re-weigh should be triggered by it is a separate decision.
8. **Stage 1b proof runs** (R1 to R3) were not repeated for these two changes; see Recheck above. David decides whether a fresh proof is wanted.
9. **No run was made, so no baseline was touched.** The 200 g baseline (`experiments/baselines/held_200g_20261103/`) and the index are unchanged, as asked.
10. **Wrong line number in the committed M249 row.** The row cites `src/task_config.py` line 653 for `grasp_closure_min_m`. The value is on line 656 in commit `63e2d72` and in HEAD (checked with `git show`). The row was not rewritten, as the brief asks. A dated correction note could be added in a later docs commit, if David wants one.
