# Braking policy, Part 2 Phase B: held objects, pruning, bridge override, integration run (build report)

Date: 2026-11-16. Branch safety-filter, not pushed. Written for David. Phase A is in [braking_policy_part2a_20261116.md](braking_policy_part2a_20261116.md). No decision of the brief was changed; where something was unclear the simplest reading was taken behind a named config value (section 7).

## 1. Summary

- Steps 7, 8 and 9 are built, tested and committed. Step 10 (the integration episode) was run and **found problems that need your decisions** (section 6). The most important:
  1. With the decided zero margin for a held object against an allowed-contact surface, the place cannot finish: the beaker's catalog shape is a capsule whose rounded bottom reaches 3.5 cm below the real beaker, so the filter stops the real beaker 29 mm above the hotplate.
  2. The hotplate never reached the filter before: the simulator's world model keeps it as a fixture entry and the filter's adapter only took objects. That is fixed. It also means the statement in my Part 1 report that the hotplate "enters the filter as an ordinary world object" was wrong for the live run.
  3. The jerk and acceleration limits are not met by the commanded or the measured hand motion in free movement, because the layer passes free motion under the speed cap untouched (shaping it stalls the staging source, section 5.2).
- The integration runs also forced several repairs of my Phase A layer (section 4.3).

## 2. Commits (all on safety-filter, not pushed)

| Hash | One line |
|---|---|
| 3d285ea | Step 7: allowed_contact flag, held objects against the world, held identity from the command, fallback capsule removed |
| 51e6b42 | Step 8: reach from the arm model, cutoffs widened to the stopping distance, sleep bound with arm lag and drift, obstacle feed, approach ceiling |
| 38b8c7b | Step 9: bridge override with error log |
| 8a06433, 7fd15cf, c5017c6, 7e57688 | Layer repairs found while setting up step 10 (section 4.3) |
| 726eac5, dbd005a, e7f8d16, 8561be9 | Look-ahead repairs and the hotplate adapter fix (section 4.3), experiment switch |
| (latest commits of the branch) | Latch guard, restart rule (a stricter rule deadlocked the pick and was reverted), progress notes |
| 78136c2, e512bc7, c7da54e, 611370e, 5335355, 2edbec8, e4fa71d | Integration driver, analysis tool, progress notes (WIP commits) |

## 3. Derived constants (placeholders: cap 0.20 m/s, accel 0.50 m/s^2, jerk 2.00 m/s^3)

| Constant | Value | Source |
|---|---|---|
| bridge_reach_m | **0.5611 m** | computed from the arm model at start: sum of the six link lengths between the joint anchors (0.0671, 0.0728, 0.1569, 0.1006, 0.1262, 0.0375 m) plus the distance to the tool point (0 for the link 7 origin); the larger arm. `BRAKING["bridge_reach_m"]` is `None` (derived); a number overrides it |
| bridge_tool_speed_m_s | 0.2806 m/s | 0.5 rad/s x reach |
| effective_accel_limit | 0.50 m/s^2 | the tunable binds (the bridge could deliver 42 m/s^2) |
| tick_cushion_m | 0.00133 m | cap x tick |
| arm_lag_rad | 0.25 rad | the bridge's tracking error limit |
| worst_stop_dist_m | **0.16347 m** | the fixed wake radius; every activation cutoff and release distance is `max(old value, this)` |

## 4. What was built

### 4.1 Step 7: held objects, allowed-contact surfaces

- `OBJECT_CATALOG`: `allowed_contact` (hotplate True, beaker False). `WorldObject` and `PlacedShape` carry it.
- New pair kinds (touch.py): held vs an allowed-contact surface: `held-surface`, margin `HELD_SURFACE_MARGIN_M` = 0.0, with the surface name; held vs every other world object: `held-world`, `MARGINS["held_world_margin_m"]` = 0.01 (PLACEHOLDER — tuned by test). Fingers stay exempt from the world.
- Held object identity: `HeldTracker.update(..., name=)`. The object named by the policy command is the primary source, looked up in the catalog; the pad match is a sanity check (a warning when it disagrees or fails); with no name the pad match is the source; with neither the gripper is empty. The 8 cm fallback capsule and `HELD_FALLBACK_HEIGHT_M` are removed. The plug-in takes `held_names` ({side: name}) from the command field `held_object`; **the scripted manipulation does not emit that field yet**, so every run below used the pad match.

### 4.2 Step 8: pruning, sleep, feed

- Every activation and release distance is `max(old, worst_stop_dist_m)` (touch.py `wake_dist`); table and sleep signatures include it. Existing cutoffs are kept where the stopping distance is smaller (tested).
- Sleep bound: per pair the arm lag (0.25 rad x the lever arms of both items) and a drift allowance (`SLEEP["drift_budget_m"]` 0.005, PLACEHOLDER) are subtracted from the room; slow base, pedestal and finger drift is accumulated since a pair was scheduled and wakes it when the total passes the allowance (tested: a drift of 1e-4 per tick woke the pair after 50 ticks).
- Obstacle feed `ObstacleFeed` (braking.py): all candidate pairs of the hand shapes (palm, fingers, link 7, held) per arm, with the filter's own pair rules, gap, margin and ceiling.
- Look-ahead: every closing pair is evaluated, speed = closing component (config `lookahead_speed`; "full" is kept), acceleration in the formula clamped to the effective limit, latch released when nothing closes or the hand stopped with room left; an allowed-contact pair gives a speed limit (the ceiling) through the same ramp.

### 4.3 Repairs to Phase A found by the integration setup

| Finding | Repair |
|---|---|
| Layer shaped every command; the staging source advances only when its last command was applied as given, so it stalled (ARMS timeout) | Free motion under the hand speed cap passes untouched; the S-curve starts on the cap, a stop, a ceiling, HOLD, and ends when it has converged on the commanded speed |
| After a clipped step the profile kept a stale speed | Profile follows the speed really sent |
| `"state"` jerk source was never fed | `process()` now calls `measure()` with the measured joints |
| Acceleration from per-tick differences of pulsed commands gave stopping distances of thousands of metres | Look-ahead acceleration clamped to the effective limit |
| Latch deadlocked after an early stop | Released when stopped with room left |
| Emergency fired for a hand at rest and for tangential motion | Emergency needs a closing speed above `stop_speed_m_s`; closing-speed reading |
| The hotplate never reached the filter | Adapter takes catalog fixtures; `entry_at_top` places it from its top face |

### 4.4 Step 9: bridge override

Look-ahead failure (stopping distance exceeds the room by more than `emergency_overshoot_ticks` = 2 ticks of travel at the cap, hand really moving, not already braking): the arm stops in that tick (no ramp), an error is logged (`layer.errors`, a `[braking] ERROR` line), and the plug-in asks the loop (`take_override()`) to jump the bridge command to the target (`src/bridge_override.py`, called from control_loop.py). The bridge side logs a `[bridge] ERROR` only if the jump was beyond its usual limits.

## 5. Tests

Run on the final code: braking_test 72 PASS, braking_b_test 60 PASS, plugin 18, touch 57, never_touch 14, kong_style 39, resolver 38, gap_rule 18, link_rules 66, structure 7, shapes 1, distance 1: all exit 0. Two fail for reasons not caused by this work: `safety_filter_spec_test` (the check "the closing right arm moves" fails identically on the Phase A baseline, exit 1) and `safety_filter_sequential_push_test` (needs data/safety_filter/kong_single_pair_fixtures.json, which does not exist). The full suite (about an hour) was not run.

Step 7 tests: catalog flags; `held-surface` zero margin with the surface named; `held-world` margin; fingers exempt; held 5 mm above and touching the hotplate; identity by name, by pad, name vs pad disagreement, unknown name, empty gripper, release. Step 8: reach equals an independent calculation and follows the tool offset; cutoffs widened and kept where smaller; sleep at worst_stop_dist - 1 cm awake and, for a part without joints, at +1 cm asleep; with arm lag the sleep needs the lag and the drift allowance on top; drift wake; feed candidates; ceiling slow-down (arrives at the ceiling, within the limits). Step 9: forced failure sets the override and the error log; a normal approach never does; the bridge function jumps only when needed; the plug-in passes the request on once.

### 5.1 Integration runs (scripted pick and place, moved layout, filter ON, braking layer, pinned stance seeds, hook: phantom wall)

| Run | Margin held-vs-surface | Jerk source | Wall | Outcome | Calls |
|---|---|---|---|---|---|
| brkG_plain_sent | 0.0 (decided) | sent | no | PLAN_FAILED (DOWN 29 mm short, then retry); run on the code of commit e7f8d16 | 13335 |
| brkJ_plain_sent | -0.04 (EXPERIMENT) | sent | no | **SUCCESS** (100.7 s plan; 8280 calls) | 8280 |
| brkJ_plain_state | -0.04 (EXPERIMENT) | state | no | PLAN_FAILED (BACK blocked, 45.6 mm) | 9399 |
| brkJ_wall_sent | -0.04 (EXPERIMENT) | sent | yes | PLAN_FAILED (DOWN 8.2 mm short) | 10094 |

The experiment margin only exists to let the rounded capsule bottom reach the plate (switch `--held-surface-margin`; the config default stays 0.0). Recordings: data/braking_episode/<run>/ and data/safety_filter/plugin_runs/<run>.npz.

Checks of the brief:

| Check | Result |
|---|---|
| Hand speed never above hand_speed_cap_m_s (0.20) | **Yes** in every run: commanded peak 0.1785 to 0.1974 m/s, measured peak 0.160 to 0.180 m/s (brkG reached 0.273 measured in its failing second attempt; its commanded series stayed at 0.197) |
| Acceleration and jerk within limits | **No** (section 5.2) |
| Beaker stops smoothly at the midpoint | **Not at the midpoint.** Wall run: from a peak of 0.152 m/s the beaker braked on the S-curve (1.47 s from the peak to rest, braking started at a gap of 41 mm) and first stopped at a gap of 3.7 mm (target 5 mm: 1.3 mm past it, 3.7 mm from the surface). It then restarted and crept (stop-go, up to 106 mm/s for a moment) to a minimum gap of **-1.4 mm** (the conservative beaker capsule overlapping the wall model), and was finally pushed back by the resolver's gap rule to 9.5 to 9.8 mm (the margin boundary, 10 mm). After the wall was removed it speeds up again: 0.076 m/s one second later. The bench itself was never approached closely in the scripted task (the wall is a test hook) |
| Beaker reaches the hotplate at the approach ceiling | **Partly**: with the decided margin the ceiling engaged on 652 ticks and the beaker slowed to 0.050 m/s by a gap of 12 mm and to 3.6 mm/s at 0.3 mm (then blocked, finding 1). With the experiment margin it never had to engage: the scripted DOWN already approaches at about the ceiling value (commanded 0.0525 m/s within 5 mm, measured up to 0.082 m/s) |
| No bridge override errors | **No for the "sent" source**: 3 look-ahead failures at t = 77.77 s (palm and link 7 against the bench, 22 to 27 mm gap, 0.03 m/s), all one tick; the "state" source had none. No `[bridge] ERROR` lines in any run (the jump was within the bridge's usual limits) and 0 bridge log events |
| Closest distances | held-vs-hotplate (capsule) -29 mm at contact with the experiment margin, speed 6e-6 m/s commanded at the closest tick; wall run: minimum -1.4 mm to the wall model while it was present, resting at 9.5 to 9.8 mm |
| Filter time per call | plain sent run mean 4.0 ms (p95 8.3, max 97); runs with HOLD ticks mean 11.0 to 11.9 ms (p95 52 to 96, max 255 ms); the failed default-margin run mean 27.6 ms (max 344 ms) |

### 5.2 Acceleration and jerk, and the jerk source

Tool point, brkJ_plain_sent (whole run): commanded peak acceleration 26.8 m/s^2 and jerk 5421 m/s^3 (the source's own pulsed commands: it advances its clock on some ticks only), 5-tick smoothed 5.3 m/s^2 and 830 m/s^3; **measured** (the physical arm, which the bridge smooths): peak 1.33 m/s^2, jerk 242 m/s^3, p99 jerk 5.1 m/s^3; smoothed 0.54 m/s^2. Limits are 0.5 m/s^2 and 2 m/s^3. The layer does not shape free motion under the cap, so these are the source's and the bridge's numbers. The layer's own S-curves (braking, restart, ceiling, HOLD) are within the limits in the tests.

Jerk source comparison (brkJ_plain_sent against brkJ_plain_state; first 95 s of sim time, before the state run diverged; the default stays "sent"):

| Series (peak jerk m/s^3) | sent | state |
|---|---|---|
| commanded (sent) | 5421 | 4763 |
| measured | 242.3 | 235.9 |
| measured, p99 over moving ticks | 5.06 | 8.28 |
| measured, 5-tick smoothed | 248 | 44 (whole run: 164) |
| peak measured speed (m/s) / acceleration (m/s^2) | 0.1602 / 1.327 | 0.1617 / 1.250 |

The two sources give **no visible difference in the free-motion series**. The visible differences are elsewhere: "sent" had three false look-ahead failures (the commanded series is pulsed, so its per-tick acceleration is meaningless, which is why the look-ahead clamps it); "state" had none, but its episode failed in BACK: after the set-down the palm shape overlapped the hotplate model by up to 35 mm, the resolver held (1396 HOLD ticks) and the look-ahead kept the arm stopped (1501 stop ticks), where the "sent" run happened not to hit this. One run each, so this is not a statistical statement.

## 6. Findings that need David (nothing was decided for you)

1. **Capsule against a zero-margin contact (blocks the place).** Beaker shape = capsule, radius 0.035 m, half length 0.05 m (SF-8): its end caps reach 3.5 cm below the real bottom. A held-vs-hotplate margin of 0 stops the real beaker about 29 mm above the plate. Options are yours (a flat-bottom shape, a shorter capsule, a negative margin for held-vs-surface, or leaving the beaker to the contact test of the simulator). `HELD_SURFACE_MARGIN_M` is the named value.
2. **The palm shape overlaps the hotplate model at set-down** (up to 3.5 cm) now that the hotplate is in the filter's world; arm parts against it use arm_world_m (not an allowed-contact rule). It decided the "state" run's failure. Needs a rule for arm parts against allowed-contact surfaces, or a shape change.
3. **Free motion is not shaped.** The brief says braking and acceleration both ramp at the jerk limit; the layer applies the S-curve to what it modulates and passes motion under the cap as it is, because the staging source (and probably others) stall when their command is changed. If you want every command shaped, sources that gate on "applied as given" must change.
4. **The commanded series of the scripted sources is pulsed**, so "sent" is a poor basis for acceleration; the "state" source behaves better in the look-ahead but its run failed for the reason in 2. Default left on "sent" as you asked.
5. **The scripted manipulation does not emit `held_object`**; identity came from the pad match in all runs.
6. The look-ahead brakes for the **closing component** of the speed (`lookahead_speed` "closing"); the whole hand speed ("full") stopped a hand that only moved along the bench (a false emergency in the first plain run).
7. **The stop target lies inside the resolver's push-out zone.** The look-ahead stops the hand at the midpoint of the margin (5 mm for a 10 mm margin), but the gap rule inside the resolver (unchanged, as decided) treats every gap below the margin as a violation to push out of. So the hand cannot rest at the midpoint: it settles at the margin boundary (9.5 to 9.8 mm in the wall run). Also, when the stop comes early (the S-curve stops 2 to 3 cm short of the target because the formula is cautious), the hand restarts and approaches in stop-go steps; a stricter restart rule deadlocked the pick (the hand stopped 23 mm early for the bench and never restarted: brkI run, outcome PLAN_FAILED at IN, 72.5 mm short), so the one-tick-cushion restart stayed. A speed limit that falls with the remaining room would approach smoothly; that changes the algorithm, so it is yours to decide.
8. Sleep: with the arm lag added, "an obstacle at worst_stop_dist + 1 cm can sleep" holds only for parts without joints (shoulder mounts); for arm parts the sleep threshold is worst_stop_dist + lag + drift allowance. Tested both ways.
9. The wall test hook (a time-based plan keeps running while the arm is held) failed the DOWN step in both wall runs by 8 to 13 mm; the beaker stop and restart themselves worked.
10. The Part 1 report said the hotplate enters the filter as a world object; in the live run it did not until now (finding in section 1).

## 7. Config values added (all in config.py, placeholders marked "PLACEHOLDER — tuned by test")

`MARGINS["held_world_margin_m"]` 0.01; `HELD_SURFACE_MARGIN_M` 0.0; `BRAKING["approach_ceiling_m_s"]` {"hotplate": 0.05}; `BRAKING["bridge_reach_m"]` None (derived); `BRAKING["lookahead_speed"]` "closing"; `BRAKING["emergency_overshoot_ticks"]` 2; `BRAKING["stop_speed_m_s"]` 0.002 (now used: the at-rest level); `SLEEP["drift_budget_m"]` 0.005; `OBJECT_CATALOG` `allowed_contact` and `entry_at_top`; `SAFETY_FILTER_PLUGIN_CFG["hold_policy"]` "braking" (from Phase A).

## 8. Not done

- The full test suite (about an hour) was not run; the 14 safety filter tests above were.
- The held object name is not emitted by the scripted manipulation.
- No run succeeded with all of: the decided margin, the wall, and a completed place; the cause is finding 1.
- The statistical question "does the jerk source matter" is one run per source.
- Pushing: not done, as instructed.
