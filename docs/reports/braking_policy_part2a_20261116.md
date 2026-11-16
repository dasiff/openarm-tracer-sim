# Braking policy, Part 2 Phase A: core braking layer (build report)

Date: 2026-11-16. Branch safety-filter, not pushed. Phase B not started. Decisions as in David's Part 2 brief; no decision changed.

## What was built

| File | What |
|---|---|
| src/safety_filter/braking_math.py (new) | `stopping_distance` (the two-step formula) and `JerkRamp` (discrete, exact jerk-limited speed ramp) |
| src/safety_filter/braking.py (new) | `BrakingLayer`: `measure`, `commanded_speed`, `scale_step`, `cap`, `step` (S-curve), `check` (look-ahead), `process` (the whole tick); `make_tool_fn` (tool point of the real model) |
| src/safety_filter/config.py | `BRAKING` (four tunables + named settings), `derive()`, `BRAKING_DERIVED` |
| src/safety_filter_plugin.py | optional `braking` layer; HOLD and an error inside the filter become a smooth stop; default hold policy is braking |
| src/task_config.py | `SAFETY_FILTER_PLUGIN_CFG["hold_policy"]` default "braking" (was "repeat_last_safe") |
| experiments/safety_filter_braking_test.py (new), experiments/run_tests.sh | 51 checks, added to the suite list |

The resolver and its gap rule are untouched. The braking layer runs after the resolver on its output.

## Derived constants at the placeholder tunables

hand_speed_cap 0.20 m/s, accel_limit 0.50 m/s^2, jerk_limit 2.00 m/s^3 (all PLACEHOLDER — tuned by test).

| Constant | Value |
|---|---|
| tick_dt_s | 0.006667 s (150 Hz) |
| tick_cushion_m | 0.00133 m |
| effective_accel_limit | 0.50 m/s^2 (the bridge could deliver 0.28 m/s per tick, i.e. 42 m/s^2, so the tunable binds) |
| worst_stop_dist_m | **0.1635 m** (cap speed, acceleration at its limit, jerk-aware formula, plus cushion) |

0.1635 m is far larger than today's 1 to 6 cm cutoffs; Phase B widening will wake many more pairs.

## Behaviour measured on the test arm (look-ahead test, obstacle margin 2 cm)

Speeds up to the cap in 0.647 s; braking starts at gap 0.075 m (2.947 s); stopped after 0.647 s having travelled 0.0643 m; stops at x = 0.58933 m against the target 0.59000 m (the midpoint of the margin), 0.7 mm short. Acceleration plateaus at the limit; jerk never above 2 m/s^3 (HOLD test: no single-tick speed drop above accel_limit x dt = 3.3 mm/s).

## Tests (experiments/safety_filter_braking_test.py: 51 PASS, 0 FAIL)

| Step | Commit | What was tested |
|---|---|---|
| 1 | 2158963 | tunables change the derived constants (cap, jerk, tick rate, bridge limit); formula value; module constants equal `derive()` |
| 2 | 3c75889 | measured and commanded speed = displacement / dt (linear fake arm); real model: joint 1 turn gives r x angle / dt (independent geometry); the base's motion is in the measured speed, not in the commanded speed |
| 3 | 0f10ade | fast command capped to the cap (linear and nonlinear fake arm, real model); slow command untouched; one scale factor for all joints |
| 4 | f39169a | step to full speed and to rest: S shape (ramp up, plateau at the limit, ramp down), acceleration <= limit, jerk <= limit, no overshoot, stopping distance <= formula |
| 5 | fd7df18 | braking starts at the tick where the formula's distance reaches the distance to the target (independent recomputation), not earlier; stops at the midpoint; moving away never brakes; stays stopped while the command keeps closing; released and speeds up when it moves away |
| 6 | 0a9f554 | HOLD: no speed drop above accel_limit x dt, jerk within limit, ends at rest; same through the plug-in; an error in the filter is a smooth stop; plug-in default is braking |

Regression (run on this branch, all exit 0): safety_filter_plugin_test, structure, gap_rule, touch, shapes, never_touch (14 PASS), link_rules, resolver tests. The full suite was not run.

One failure found and fixed on the way: the first ramp snapped the acceleration to 0 on arrival (jerk 19 m/s^3 at the end of the ramp, against 2). Replaced by a discrete ramp that only picks an acceleration from which the jerk limit can still bring it to 0 by the target speed. An implementation fix, not an algorithm change.

## Not done / not active yet

- **The look-ahead is built and tested but has no obstacle feed in real runs.** `obstacle_fn` (nearest obstacle gap and margin per arm, out to worst_stop_dist_m) is a parameter of the plug-in and defaults to none, so in a real run only the hand speed cap, the S-curve and HOLD braking are active. The feed needs the widened pruning (Phase B, step 8).
- Held objects, allowed-contact surfaces, approach ceilings, bridge override with error log, pruning widening, integration run: Phase B.
- `approach_ceiling_m_s` exists as an empty dict (defaults to the hand cap); nothing reads it yet.
- A smooth stop on a timeout: the timeout (`limit_reached`) ends the run through the loop's stop request; no braking is added there (a HOLD before it is smooth).
- No simulator run in Phase A (pure numbers, the real model's kinematics only).

## Decisions for David (simplest reading taken, behind a named config value)

| # | Choice made | Config value |
|---|---|---|
| 1 | The measured speed used for jerk limiting is taken from the commands the filter sent, not from the measured joints (which lag the command and would make the ramp overshoot). `"state"` is implemented, not tested. | `BRAKING["speed_source"]` = "sent" |
| 2 | "What the bridge can deliver per tick" = the bridge's joint cap turned into a tool speed with a reach of 0.56 m, divided by the tick. Binding value is the tunable. | `bridge_reach_m` = 0.56 |
| 3 | Tool point = link 7 origin (the filter's hand frame). | `tool_offset_local_m` = (0, 0, 0) |
| 4 | The look-ahead uses the hand's full speed as the closing speed (cautious), and a pair counts as closing when the wanted command makes its gap shrink. | `closing_eps_m` = 1e-6 |
| 5 | A braking arm stays stopped while the wanted command keeps closing on the obstacle, and is released when it no longer does. | in `check()` |
| 6 | The arm never moves past the commanded pose; a command that drops below the current speed is followed by the ramp only when it is a stop (HOLD or look-ahead). | in `step()` |
| 7 | A stopping acceleration already negative (slowing) counts as 0 in the stopping distance (cautious). | in `stopping_distance` |
| 8 | The plug-in default hold policy is now "braking" in task_config; "repeat_last_safe" stays available. Braking also runs in shadow mode (the record shows what it would send; the robot gets the raw command). | `SAFETY_FILTER_PLUGIN_CFG["hold_policy"]` |
| 9 | `stop_speed_m_s` 0.002 is defined for "stopped" but nothing uses it yet. | `stop_speed_m_s` |

## Housekeeping

Built in a temporary worktree of safety-filter (data/ and deps/ were linked to the main checkout there and never staged). Phase A commits: 2158963, 3c75889, 0f10ade, f39169a, fd7df18, 0a9f554.
