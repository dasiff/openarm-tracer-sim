# Sag trim as a separate layer, with a VLA-style source (2026-11-09)

Read only with respect to the code: no file in the real working copy was edited, and no sag trim code was touched (in the real tree or any copy). The trim switch was changed only in-process, for the trim-off run (SAG_LAYER_CFG["enabled"] = False, the existing setting). Outputs and scratch are under data/logs/sagsep/ and scratch/ (not in git). Not committed.

## 1. Code: every reference to the trim outside src/sag_layer.py

| file:line | what it does | who acts |
|---|---|---|
| src/control_loop.py:105 | imports the layer (MANIPULATION_SOURCES, SAG_TRACE, SagLayer, correct_command) | loop |
| src/control_loop.py:608 | builds one SagLayer per arm (posture = sag_home_q, from loop_cfg) | loop |
| src/control_loop.py:657-660 | sag_correct(cmd): calls correct_command with the loop's data, t and the command period | loop |
| src/control_loop.py:662-670 | is_manipulation(name): True for a source named "vla" (MANIPULATION_SOURCES), or for the auto source while its orchestrator status says "module" = "manipulation" | loop (reads source name and orchestrator status) |
| src/control_loop.py:1079 | track_open(data, t) every tick (the reset watch after a caught open) | loop |
| src/control_loop.py:1117 | sets cmd["manip"] = is_manipulation(name) on every source command | loop |
| src/control_loop.py:1120 | the open-catch release calls release_open(t) (arms the reset watch) | loop (open-catch helper) |
| src/control_loop.py:1135 | sag_correct(cmd) applied to every source command, after the source, before the bridge | loop |
| src/control_loop.py:1262 | writes the layers' logs into the run record | logging |
| src/mock_policy.py:38, 151, 173 | sets PLAN_TRACE (the waypoint label and planned pose, logging only) | mock: logging, not trim |
| src/mock_policy.py:236 | hmr.end_load_change at the mock's open (clears the held-load "changing" flag) | mock: state flag, which the trim does not read |
| src/mpvi_modules.py:52, 361, 391 | imports PLAN_TRACE and sets the label and planned pose (logging only) | scripted skill: logging, not trim |
| src/mpvi_modules.py:397 | arm.step(data, p_ref, R_ref), with a comment that the sag correction is a layer now | scripted skill: no trim code |
| src/mpvi_modules.py:298 | hmr.set_target(0) at each pick start (the held mass ramps to 0) | scripted skill: held-mass ramp (not the trim) |
| src/mpvi_modules.py:464 | hmr.set_target(reading) at the handoff weight reading (ramp in) | scripted skill: held-mass ramp (not the trim) |
| src/mpvi_modules.py:33-34 | docstring says the correction integrates in the scripted path (stale: it moved to the layer) | documentation only |
| src/signal_trace.py:25, 51, 159-160 | logs the layer's columns (per hand, when OPENARM_SIGNAL_TRACE=1) | logging |
| src/task_config.py:725-727 | SAG_LAYER_CFG = {"enabled": True}: the switch | config |
| src/task_config.py:438 | PICK_PLACE_CFG sag_gain_per_s 3.0, sag_max_m 0.04, sag_max_rad 0.1 | config |
| experiments/sag_layer_test.py | tests of the layer (its own copies of the calls) | test |
| experiments/held_mass_ramp_test.py:43 | imports _sag_trim_step | test |
| experiments/place_over_turn_measure.py:37, 149-150 | imports _clip_norm and has its own copy of the trim integration | measurement script, not the executed path |
| experiments/bounce_freeze_run.py:5-49, experiments/bounce_handoff_metrics.py:306, 339 | sets/reads sag_reset_at_skill_start (see the dead key below) | experiments only |

Dead key: PICK_PLACE_CFG["sag_reset_at_skill_start"] is no longer defined in src/task_config.py and nothing in src reads it. Only experiments/bounce_freeze_run.py sets it, which has no effect on the executed code. (Not changed.)

Do the skills compute, apply, reset or freeze the trim? **No.** The scripted skill (src/mpvi_modules.py) passes its targets to arm.step without any correction (line 397). The mock (src/mock_policy.py) contains no C, no correct() and no reset. Every trim action (integrating C, the freeze during a held-mass ramp, the takeover reset, the open reset, the IK re-solve) happens in src/sag_layer.py and is applied by the control loop (src/control_loop.py:1135). The skills only set the held-mass ramps (lines 298, 464) and labels for logging.

Verdict: **separate layer: yes.** The trim is one layer in the control loop, the same code for every source, with an on/off switch (SAG_LAYER_CFG["enabled"]). It is applied to the commands of the sources it is told are manipulation sources (see the input table). Two caveats: the trim's freeze reads the held-mass ramp, which the scripted skill also drives (lines 298, 464), and the manipulation flag for the scripted auto source depends on the orchestrator's status.

## 2. Inputs the layer reads

Inputs of SagLayer.correct(q_src, data, t, nominal_dt) and of track_open / release_open / manipulation. (a) = available below the policy whatever the source is; (b) = only because the scripted skills provide it.

| input | where it comes from | class | what happens under the mock |
|---|---|---|---|
| q_src (7 joint targets of the command) | the source's command (scripted module's arm.step, or the mock's chunk rows) | (a) command | the mock's raw chunk rows (verified in section 4) |
| data.qpos, data.qvel (measured joints, other joints) | simulator state | (a) joint states | same |
| t (sim time), nominal_dt (the command period) | loop clock; period from the command | (a) | same |
| MinkArm kinematics, pad frame, joint limits, velocity limit, posture (home_q) | model and MINK_CFG, loop_cfg sag_home_q | (a) config and model | same |
| PICK_PLACE_CFG sag_gain_per_s, sag_max_m, sag_max_rad; SAG_LAYER_CFG enabled | config | (a) config | same |
| hmr.ramp_running(model, side, t) (the freeze) | held-mass record, set by: the weigh helper and open catch (loop, a); the scripted skill at pick start (line 298, b) and at the handoff reading (line 464, b) | (a) mostly; (b) for lines 298 and 464 | 298 runs under the mock (the mock's start() calls the scripted start(), confirmed in src/mock_policy.py:250-258); 464 does NOT run (it is in the scripted step(), which the mock does not run). The loop's weigh helper still ramps to the weighed mass at lift-off (log: "[weigh] left: lift-off at tick 12137 ... estimate 0.1765 kg"). So under the mock the held mass is set by the weigh helper only. The trim still freezes during that ramp; the reading-based ramp is absent |
| manip flag (is_manipulation) | loop: source name in MANIPULATION_SOURCES ("vla"), or the auto source with orchestrator status "module" == "manipulation" | (a) source identity; the status part is orchestrator (b) | the mock is named "vla", so it is always manipulation: the status is not used. Its commands are corrected in every phase |
| previous manipulation flag and takeover (SagLayer.manipulation) | the layer's own state, from the loop's flags | (a) | when the mock takes the arm, C is zeroed once (takeover), as for the scripted source |
| release_open(t) | open-catch release callback (control_loop.py:1120) | (a) helper | the open catch is a loop helper; unchanged |
| track_open(data, t): finger qpos and qvel, stall_done, close_min_s, TASK_OPEN | simulator state and config | (a) | same |
| PLAN_TRACE label and planned pose (trace only) | set by the mock and by the scripted module | (b) but logging only | the layer does not read PLAN_TRACE for control (trace_values only) |
| mock's hmr.end_load_change at its open (mock_policy.py:236) | the mock | state flag | the trim does not read the "changing" flag (only ramp_running); no effect on the trim |

Result of the (b) check: no control input of the trim is a skill name, waypoint, skill start or end, or a script flag. The only skill-dependent control effects are the two held-mass ramp calls (298, 464), and the auto-source status for the scripted auto source. Under the mock, 298 still runs; 464 does not, and the loop's weigh helper sets the mass instead.

## 3. Behavior: 200 g pick and place, mock VLA policy as the source

Setup: make_pipeline_task(manipulation="mock"), the existing mock (state and prompt in, 50-row chunk out, played 10 at a time). Beaker: the default configuration (100 g glass + 100 g liquid = 200 g; BEAKER_CFG glass_mass and liquid_mass). Scripts: scratch/mock_probe.py (MOCK_TRIM=on|off, signal trace on), scratch/mock_waypoint_table.py. Trim off: SAG_LAYER_CFG["enabled"] = False in the process (no file edited). Each run once after the fix (see Not done for the first attempt).

Outcome:
- Trim on: the pick completes (CLOSE passes at 82.3 s). The place's OVER moves the gripper open at 88 s (see the mock bug below), the beaker falls at about 88 s, and the place's completion check fails at 93.5 s: "the beaker is not standing on the hotplate" (offset 0.20 m).
- Trim off: the pick fails at UP: "UP: not reached after 14.1 s: 12.9 mm and 1.6 deg from the waypoint".

Waypoint errors (pad centre minus the waypoint's target, mm, xyz and norm; the target is the mock's planned pose at the end of the waypoint, taken from the trim-on run, which completes every waypoint, so both runs are measured against the same targets; HOME is a joint move):

| waypoint | trim ON: pad error (mm) dx dy dz, norm | trim ON: end time | trim OFF: pad error (mm) dx dy dz, norm | trim OFF: end time |
|---|---|---|---|---|
| pick OVER | +0.0 −0.2 +0.8, 0.8 | 68.8 s | +1.2 −0.3 −1.0, 1.6 | 68.5 s |
| pick DOWN | −0.3 +0.5 −1.2, 1.3 | 73.4 s | +1.9 +0.4 −0.8, 2.1 | 72.2 s |
| pick IN | −0.9 +0.1 +1.9, 2.2 | 78.0 s | +2.0 +0.4 +0.0, 2.1 | 76.2 s |
| pick CLOSE | (no pose target; closed, 37.5 mm) | 82.3 s | (no pose target) | 79.5 s |
| pick UP (carry) | −0.7 +0.6 +2.5, 2.7 | 86.0 s | −1.6 +2.2 −12.3, 12.6 (FAILS, timeout 14 s) | not reached |
| place OVER | −0.2 +1.0 +0.0, 1.0 (beaker drops in this move) | 90.2 s | not reached | — |
| place DOWN | +0.0 +1.2 −1.7, 2.1 | 92.3 s | not reached | — |
| place OPEN | (no pose target) | 93.5 s | not reached | — |
| place BACK | −0.0 +0.0 −0.0, 0.0 | 93.5 s | not reached | — |
| HOME | not reached in either run | — | not reached | — |

Trim values (signal_trace.csv, left arm, from the run's own log):
- Trim on: nonzero and changing through pick and carry: pick |C_p| max 8.5 mm and |C_r| max 3.6°; carry (UP) |C_p| max 13.6 mm and |C_r| max 2.6°; place |C_p| max 15.8 mm and |C_r| max 3.1°. C_p changes on 682 ticks. Nonzero on 2392 of 2400 pick ticks, 900 of 900 carry ticks, 833 of 833 place ticks.
- Trim off: exactly zero throughout (C_p and C_r, 0 of 13830 rows nonzero).

Does trim-off make the waypoint errors worse, as with the scripted source? The pick waypoints (OVER, DOWN, IN) are about the same with and without the trim (0.8–2.2 mm vs 1.6–2.1 mm). The difference is at the carry: UP ends 12.6 mm below the target with the trim off, against 2.7 mm with it, and the pick fails at UP. The scripted trim-off run also failed at UP (7.4 mm, 1.4°). So the same step fails in both sources, and the magnitude is different (12.6 vs 7.4 mm; the mock's carry path and timing differ). The place cannot be compared (the trim-off run never reaches the place).

## 4. The mock's output is raw

Confirmed by measurement, not only by code. The control loop calls the layer on each manipulation command (control_loop.py:1135); a logging wrapper around correct_command (in the probe, no file edited) recorded the source's targets (input) and the layer's output for every manipulation command:
- Trim on: the layer changes the mock's raw targets by up to 4.3° (joint 6), 2.9° (joint 4), 3.8° (joint 5) over the run.
- Trim off: output equals the raw target exactly (0.000° on every joint, 794 commands).
The mock's code has no reference to the trim (its only sag references are PLAN_TRACE logging lines 38, 151, 173). So the mock sends raw absolute joint targets; the trim is added by the layer in the loop.

## 5. HOME stall (known issue)
Not reached in either run: the trim-on run fails at the place's completion (beaker dropped), and the trim-off run fails at the pick's UP. So the HOME stall from the IK re-solve did not show up here and was not observed. Noted for the record only; not fixed.

## Not done
- A scripted trim-off run was not repeated; the scripted numbers (UP 7.4 mm, 1.4°) are from the earlier run.
- The trim-off run never reaches the place or HOME, so the place and HOME comparison is only possible with the place bug fixed (below) and a trim-off run that gets through the pick; not done.
- The mock's gripper bug (below): not fixed; it blocks the place in both configurations. The reason: fixing it means changing src/mock_policy.py, which this task does not allow.
- The 200 g label: I used the default beaker (100 g glass + 100 g liquid = 200 g). I did not check that against a document that names the 200 g case.
- The first attempt of the two runs had a bug in my probe (it dropped the mock source, so the mock never moved the arm; fixed by merging the task's extra sources and passing its loop_cfg). Those first outputs were overwritten by the reruns.
- A deleted old folder: I could not delete the old output folder (a permission denial on rm -rf); the reruns overwrote its files.
- Waypoint targets are the mock's planned poses from the trim-on run (the mock's own plan is not logged as a target); for a waypoint the mock never finishes, the error is against those targets, which matches the module's own failure measure (12.6 mm here vs 12.9 mm in the module's message).

## Decisions for David
1. Mock bug (src/mock_policy.py, reset() called from begin()): it sets the gripper target to open (1.0) at the start of every segment, so the place's OVER opens the gripper while the beaker is still held and the beaker falls (fingers 37.5 → 43.9 mm at 88 s). The scripted skill does not do this. Decide whether to fix the mock (keep the gripper state when a segment starts while holding).
2. Held-mass handoff reading: the reading (mpvi_modules line 464) is in the scripted step(), which the mock does not run, so under the mock the held mass is set only by the loop's weigh helper. Decide whether the reading belongs in the loop (for every source) or should stay with the scripted skill.
3. Trim on joint-only moves (HOME): the IK re-solve (see the HOME note in the earlier report). Decide whether to change it; not changed here.
4. The dead key sag_reset_at_skill_start (experiments only). Decide whether to remove the experiment that sets it, or leave it.

## Files
- This report: docs/reports/sag_layer_separation_20261109.md
- Runs: data/logs/sagsep/on/ (episode/, mock_probe.csv, mock_queries.csv, layer_io.json), data/logs/sagsep/off/ (same), logs data/logs/sagsep/on.log and off.log, status data/logs/sagsep/status.txt
- Scratch: scratch/mock_probe.py, scratch/mock_waypoint_table.py, scratch/run_sagsep_20261109.sh

## Commit state
No commits made.
