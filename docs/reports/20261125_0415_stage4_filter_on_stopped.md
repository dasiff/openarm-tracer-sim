# Stage 4: safety filter on by default: STOPPED, default reverted (2026-11-25, David d0bc 2026-11-24)

Branch `safety-filter`, HEAD 2a0b53f (Stage 3) before this stage; the Stage 3 commit is `2a0b53f`. One MuJoCo process at a time. Runs in `data/logs/rebuild_s4/` (queue `queue_s4.sh`, baseline `queue_base4.sh`). Nothing pushed.

**Stage 4 did not pass.** The default was changed to "on" (the loop's `safety_filter_mode`, and the launcher's `--safety-filter` default), and with the filter active the held run fails, the grasp staging test fails, and the command-bridge test is killed. Per the brief, the stage stops and the filter is not fixed. **The default has been reverted to "off" in the working tree; nothing of the change is committed.** The full `run_tests.sh` was not run.

## Plain-language answer

1. **The held run fails with the filter on.** Nominal and perturbed: `PLAN_FAILED` at segment 2/4 (pick up the beaker): "IN: not reached after 14.9 s: 3.2 mm and 0.1 deg from the waypoint". The filter recorded 1,054 (nominal) and 1,092 (perturbed) `KONG_PUSH` decisions in about 9,900 calls, and no holds or braking in these runs. With the filter off (Stage 3) the same held runs are SUCCESS.
2. **The grasp-lift staging test fails.** The timeline reaches its end with outcome TIMEOUT, the beaker is not carried up (0.0 mm), and the fingers do not press at the 8 N cap.
3. **The command-bridge test is killed.** With the filter on, the process grows past the 6 GB cap at its fifth case (exit 143), because the test runs six loops in one process and each loop with the filter on holds memory. With the filter off it passes 21/21.
4. **Policy B with the filter on:** the filter returns HOLD on 2,090 of 4,524 calls (46%), every one with reason `BRANCH_GUARD`, and the braking override fired once (`ERROR override: braking exceeded the bridge limits`). The hand error falls (mean 10.3 mm against 17.8 mm off; last 2 s 1.0 mm against 22.4 mm) because the arm moves less: the pad path is 4.3 m, against 8.2 m with the filter off.
5. **Regression otherwise:** 26 of 27 regression logs exit 0 (the sag tests, the bridge tests, the source-view test, and the safety-filter group, which passes its own tests with the filter on). Staging: `home_pose_test`, `beaker_hotplate_correction_test` and `nav_follow_sim_test` exit 0; `grasp_lift_test` exits 1.

## Pass criteria (Stage 4)

| criterion | result |
|---|---|
| Full safety-filter regression and sag tests: no new failures | **FAIL**: command_bridge_test killed (memory, filter on); the safety-filter group and the sag tests pass |
| Held run with the filter active finishes SUCCESS | **FAIL**: nominal and perturbed both PLAN_FAILED at segment 2/4 (IN not reached) |
| Staging tests (reported) | home_pose 0, beaker_hotplate_correction 0, nav_follow_sim 0, grasp_lift_test 1 (TIMEOUT, beaker not carried) |

## Filter events in the held runs (`data/safety_filter/plugin_runs/`)

| run | recording | calls | PASS | KONG_PUSH | HOLD | braking | held events |
|---|---|---|---|---|---|---|---|
| held, nominal | run_20261125_034245.npz | 9,929 | 8,875 | 1,054 | 0 | 0 | none |
| held, perturbed | run_20261125_035126.npz | 9,922 | 8,830 | 1,092 | 0 | 0 | none |
| policy B, bridge on, correction on | run_20261125_035656.npz | 4,524 | 2,434 | 0 | 2,090 (all BRANCH_GUARD) | 1 override error | — |

Stops (HOLD ticks) in the held runs: none. Releases: the held runs do not reach a release (both fail before it). Held-object identity: no events recorded.

## Staging tests

| test | exit |
|---|---|
| `home_pose_test` | 0 |
| `grasp_lift_test` | 1: TIMEOUT; "the beaker is carried up with the hand: 0.0 mm"; "each finger presses at the 8 N cap: the cap binds: []"; the control does not hold |
| `beaker_hotplate_correction_test` | 0 |
| `nav_follow_sim_test --only on` | 0 |

## Baseline differences against Stage 3 (report only; bench scenarios, filter on against off)

| scenario | largest joint difference (deg) | joint, time | largest hand difference (mm) |
|---|---|---|---|
| manual | 0.974 | left joint 3, 9.80 s | 5.57 (right hand, 9.80 s) |
| teleop | 0.485 | right joint 5, 5.30 s | 5.91 (right hand, 5.70 s) |
| policy | 0.282 | right joint 2, 2.00 s | 3.18 (right hand, 2.00 s) |

## Notes

- `experiments/demo_video.py` has its own `--safety-filter` default ("off"); it was not changed.
- The plug-in's own `SAFETY_FILTER_PLUGIN_CFG["mode"]` stays "off" (the loop reads its own `safety_filter_mode`; the plugin's test asserts the config default).
- Not run: the full `run_tests.sh` (the brief asks for it after Stage 4, which did not pass).

## Questions for David

1. **The filter's holds on policy B** (46% of calls, `BRANCH_GUARD`) and its `KONG_PUSH` decisions on the held run: are these expected with the filter at its default, or is the guard too tight for this stream?
2. **The held-run failure with the filter on** (segment 2, IN not reached, 3.2 mm short): should the filter's push be checked against the pick waypoint before Stage 4 is re-run?
3. **The command-bridge test's memory with the filter on**: the test calls the loop six times in one process. Should each case run in its own process, or should the filter's per-run memory be looked at?
4. **Default**: the default stays "off" on this branch until 1 to 3 are decided.
