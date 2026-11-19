# Final layered cleanup: golden entries, elbow snap script, safety-filter tests

Date: 20261119 (stamp 20261119_0514). Branch safety-filter, worktree ~/projects/openarm-tracer-sim-sf-d. Nothing pushed.

Written for: David (decisions in section 5).

## 1. Summary

- speed_gate's golden set keeps only the four kong_style recordings. The two layered recordings (punch_original_layered, cat_layered) are out of the gate. Their files stay in the golden folder and in git history.
- Deleted scripts/safety_filter_elbow_snap_sweep.py. Nothing imports it (grep). experiments/elbow_snap_sweep.py is a different file and stays.
- speed_gate part A on the full recordings, without --only, completes: PASS, 4 of 4 recordings, 0 failing ticks.
- The 18 safety-filter tests all exit 0.

## 2. Commits

| Hash | Subject |
|---|---|
| 7395f77 | Final layered cleanup: golden entries and elbow snap script removed |
| (this commit) | Final layered cleanup: results and report |

## 3. Files changed

| File | Change |
|---|---|
| scripts/safety_filter_speed_util.py | GOLDEN_SET = the four kong_style entries; comment (SF-108) |
| scripts/safety_filter_elbow_snap_sweep.py | deleted |
| docs/progress.md | task entry and results |

speed_gate.py itself is unchanged: GOLDEN_SET is defined in the helper module since the port (3e4fa97).

## 4. Results

speed_gate part A, full recordings, no --only (tmux partA_final, log data/logs/speed_gate_A_final.log, output data/safety_filter/speed_gate/final_layered_A/gate_A.json):

| Recording | Ticks | Failing ticks | Largest command difference |
|---|---|---|---|
| punch_original_kong_style | 1576 | 0 | 0 rad |
| punch_reactive_kong_style | 1576 | 0 | 0 rad |
| shadow_pp_kong_style | 7783 | 0 | 0 rad |
| cat_kong_style | 6436 | 0 | 0 rad |

Result: PART A PASS (4 recordings), exit 0. The golden folder was linked temporarily from the other speed worktree and removed afterwards.

Safety-filter suite, the 18 tests of experiments/run_tests.sh, one at a time (tmux sf18b, summary data/logs/safety18_summary.txt): all 18 exit 0, no tracebacks.

| Test | Exit | Checks ok |
|---|---|---|
| safety_filter_model_check | 0 | |
| safety_filter_distance_test | 0 | |
| safety_filter_shapes_test | 0 | |
| safety_filter_plugin_test | 0 | |
| safety_filter_braking_test | 0 | |
| safety_filter_braking_b_test | 0 | |
| safety_filter_braking_c_test | 0 | |
| safety_filter_braking_d_test | 0 | |
| safety_filter_braking_e_test | 0 | |
| safety_filter_never_touch_test | 0 | |
| safety_filter_touch_test | 0 | 56 |
| safety_filter_structure_test | 0 | 6 |
| safety_filter_link_rules_test | 0 | 53 |
| safety_filter_gap_rule_test | 0 | 15 |
| safety_filter_resolver_test | 0 | 28 |
| safety_filter_spec_test | 0 | 27 |
| safety_filter_kong_style_test | 0 | 35 |
| safety_filter_sequential_push_test | 0 | 43 |

Timing note: safety_filter_resolver_test started at 04:14:03 and finished at 05:11:08. It normally takes about 11 seconds. The machine was probably asleep during that window. The system journal returned no suspend entries, so this is not confirmed. The test log is complete (28 checks ok, 0 failing), so the result stands.

## 5. Not done and decisions for David

Not done:

1. The gate's golden set is now kong only, so the layered recordings are not checked by any gate run. Their golden files stay on disk.
2. The golden recordings are not in this worktree. The part A run used a temporary link to the other speed worktree.

Decisions for David:

1. Keep the layered recordings out of the gate for good, or keep them as a documented known gap?
2. speed_metrics.py stays as the subprocess for part B. Keep it, or fold it into the gate (see the port report, section 6).
3. The helper module (scripts/safety_filter_speed_util.py) is about the size of speed_common. Keep it, or move its parts into the gate?
