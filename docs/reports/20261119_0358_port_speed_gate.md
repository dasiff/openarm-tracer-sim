# Delete broken importers, port speed_gate, fix a comment

Date: 20261119 (stamp 20261119_0358). Branch safety-filter, worktree ~/projects/openarm-tracer-sim-sf-d. Nothing pushed.

Written for: David (decisions in section 6).

## 1. Summary

- Deleted seven scripts that imported the deleted modules. The eighth on the list, scripts/safety_filter_speed_metrics.py, was kept: speed_gate runs it as a subprocess in part B.
- Ported speed_gate's dependency on the deleted speed_common into scripts/safety_filter_speed_util.py. speed_gate and speed_metrics change one import line each; their code is unchanged. The port is not small: 27 names (about 320 lines) plus two reactive-stream helpers, because run_sim drives the simulator.
- Four functions in the helper module changed where the layered resolver or a deleted module was in the way. The gate's outputs are identical before and after on the checks I could run (kong recordings, 40 ticks; punch stream, 40 ticks).
- Fixed a stale comment in src/safety_filter/link_rules.py.
- The 18 safety-filter tests all exit 0.

## 2. Commits

| Hash | Subject |
|---|---|
| b5e23b1 | Delete seven broken importers of the deleted scripts |
| 3e4fa97 | Port speed_gate and speed_metrics to scripts/safety_filter_speed_util.py |
| e5ca964 | Fix a comment that names the deleted kong_comparison script |
| 3ed0573 | Progress: the port of speed_gate and the 18-test run planned |
| (this commit) | Port speed gate: results and report |

## 3. Files deleted

| File | Lines | Reason |
|---|---|---|
| scripts/safety_filter_pair_counts.py | | imports diag_common (deleted) |
| scripts/safety_filter_reactive_stream.py | | imports diag_common |
| scripts/safety_filter_mild_stream.py | | imports diag_common |
| scripts/safety_filter_solver_model_check.py | | imports kong_comparison |
| scripts/safety_filter_solver_roundtrip.py | | imports kong_comparison |
| scripts/safety_filter_speed_golden.py | | imports speed_common (the golden recordings' generator) |
| scripts/safety_filter_dc_replay.py | | imports speed_common |
| **Total** | **1284** | |

Not deleted: scripts/safety_filter_speed_metrics.py (on the list). speed_gate runs it in part B (line 207) to compute the summaries with the golden tag's source tree. Deleting it would change the gate's behaviour. It now imports the helper module.

## 4. The port

scripts/safety_filter_speed_util.py (374 lines) holds:

- Verbatim from speed_common (27 names): CAT_HOLD_STOP_S, CLOSED_LOOP_SET, GOLDEN_DIR, GOLDEN_SET, PUNCH_CHECKPOINT_TICKS, REF_TREE, ROOT, Recording, SIDES, SIM_FULLCHECK_SEED, SIM_MAX_MINUTES, SIM_SAMPLER_SEED, STREAMS, TICK_FLOAT, TICK_INT, TICK_TEXT, TickRecorder, _obj_key, bridge_speed, name_of, punch_stream, resolver_mode, run_punch, run_sim, run_stream, time_stats_ms, write_json_atomic.
- Verbatim from the deleted reactive_stream: REACTIVE_MAX_STEP_RAD, reactive_command, limits_arrays.

Changes, all where the layered resolver or a deleted module was used (SF-108):

| Function | Change |
|---|---|
| resolver_mode | accepts kong_style only; no longer reads or sets RESOLVER["mode"]; any other mode raises |
| run_sim | accepts kong_style only; no longer saves or sets RESOLVER["mode"] |
| run_punch | fixtures from experiments/safety_filter_resolver_fixtures.py (was the deleted comparison script); reactive helpers from this file |
| punch_stream | reads data/safety_filter/speed_golden/punch_stream.npz; if the file is missing it raises (it used to rebuild it from the deleted comparison script) |

The AST check confirms that the other 23 definitions are identical to speed_common at the pre-removal commit 3fd6925. The module has no unresolved global names.

speed_gate.py and speed_metrics.py: one import line each (`import safety_filter_speed_util as sc`).

## 5. Checks

Import: `python scripts/safety_filter_speed_gate.py --help` and `python scripts/safety_filter_speed_metrics.py --help` exit 0. Importing both modules works. `resolver_mode("layered")` raises; `resolver_mode("kong_style")` works; a missing punch_stream.npz raises FileNotFoundError.

Behaviour, old against new. The old tree is commit 3fd6925 (the last commit with speed_common and the layered mode; a87f54b was tried first and is after SF-108, so it failed with KeyError 'mode'). Golden recordings and speed_ref_tree were linked from the other speed worktree, temporarily:

| Check | Old | New | Result |
|---|---|---|---|
| Part A, punch_original_kong_style, punch_reactive_kong_style, shadow_pp_kong_style, cat_kong_style; 40 ticks | PASS, 0 failing ticks | PASS, 0 failing ticks | 71 leaf values compared, 0 differences (timings, paths and git head excluded) |
| Part B, punch_original_kong_style; 40 ticks (speed_metrics runs as a subprocess) | FAIL vs full golden summary (expected for 40 ticks) | FAIL vs full golden summary (expected) | 73 leaf values compared, 0 differences |

Not run: the simulator streams (cat, shadow_pp in part B), which need MuJoCo and up to 40 minutes each. run_sim's change (no mode) is not exercised by any run here.

Full safety-filter suite (the 18 tests of experiments/run_tests.sh, one at a time): all 18 exit 0.

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

Log: data/logs/safety18_summary.txt (per-test logs data/logs/safety18_<name>.log).

## 6. Not done and decisions for David

Not done:

1. scripts/safety_filter_elbow_snap_sweep.py imports solver_roundtrip, which is now deleted. It fails at import. It was not on any list, so I left it alone.
2. The gate's GOLDEN_SET still lists two layered recordings (punch_original_layered, cat_layered). The layered resolver is gone, so they cannot be replayed. Part A without --only stops with an error at the first of them, after the kong recordings. Run it with --only for the kong recordings.
3. The punch_stream fallback (rebuilding the file from the deleted comparison script) is gone. The file must exist in the golden folder, and it does in the speed worktree.
4. The golden recordings' generator, speed_golden.py, is deleted. Recordings can no longer be regenerated from this repo; only the existing files can be used.

Decisions for David:

1. Layered golden entries: remove them from GOLDEN_SET, or keep them as known failures? Keeping them stops part A at the first layered recording.
2. speed_metrics.py: keep it as it is (current), or fold its summary into the gate so that only one script remains? Folding it needs the reference tree's source in a subprocess anyway.
3. The helper module is about the size of speed_common. Keep it as a separate file, or move the needed parts into speed_gate itself?
4. elbow_snap_sweep.py: delete it, or port its round-trip function from git history?
5. Regenerating the golden recordings: with the generator deleted, is the golden folder the only source? If so, keep a copy of speed_golden.py in git history and note where.

## 7. Notes

- Golden links removed from the worktree after the checks. Gate outputs for the checks are in data/safety_filter/speed_gate/port_new/ (gitignored). The old tree's outputs are in the scratchpad.
