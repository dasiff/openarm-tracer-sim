# M264: the sag trim acts only on the manipulation source (Part 1 findings, build, runs)

Written for David. Commits a947420 (build, tests, decision row M264), cc6b81e (run start). Runs at a947420, OPENARM_SIGNAL_TRACE=1, --liquid 0.100. Scratch outputs outside the repo; nothing registered; experiments/baselines/ untouched.

## Part 1 (read-only findings)

**(a) Sources that go through the layer before this change.** Sources are the entries of the loop's `sources` dict (src/episode_runner.py:150-156).

| source | name | arm commands it sends | goes through the layer today |
|---|---|---|---|
| orchestrator (auto source) | "policy" (episode_runner.py:153) | staging ARMS phase: `{"mode": "arms", "arm_targets": both sides, "gripper"}` (src/staging_source.py:841); manipulation module: arms commands for every move and the gripper (src/mpvi_modules.py); staging PEDESTAL and DRIVE/CREEP: pedestal and base only (staging_source.py:259, 689-715) | yes, every arms command (both staging ARMS and manipulation) |
| keyboard (teleop jog) | "keyboard" (episode_runner.py:154) | arms commands when a jog key is held (src/keyboard_source.py:257-261); base and pedestal otherwise | yes, when it sends arms |
| mock policy | "vla" (src/mpvi_pipeline.py:108, mock only) | chunk commands (converted to arms) while the mock plan is active; nothing while inactive (src/mock_policy.py:292-293) | yes, while active |

Routing: in auto mode every route is the auto source (src/control_loop.py:819); the mock adds `arms.left` and `gripper.left` for `("policy", "vla")` (control_loop.py:821, mpvi_pipeline.py:109). So the staging ARMS commands of "policy" reach the layer in both the scripted and the mock run.

**(b) Where the layer's correction goes during navigation.** Lines of the path:

1. The loop corrects each command, then hands it to the bridge (src/control_loop.py:1126-1127):
   ```
   cmd = sag_correct(cmd)     # M262: the sag trim sits between the source and the bridge
   cmd = bridge_take(cmd)
   ```
2. `bridge_take` stores the arm targets as the bridge's target (control_loop.py:663-670): `bridge["target"][s] = np.array(q, float)`.
3. `bridge_update` moves the command toward that target by at most 0.01 rad per update and applies it (control_loop.py:672-705), except when the tracking error is above 0.25 rad (suspended) or a target step is above 0.25 rad (rejected).
4. The bridge log of the last scripted run is empty: `[bridge] 0 log entries, 0 events` (run log). So nothing was suspended or rejected.
5. The corrected targets come from `SagLayer.correct` (src/sag_layer.py:84-107). Its solver step is wrapped in a try block, and on any exception it returns the source's own targets:
   ```
   102  self.arm.step(data, p_ref + self.C["p"], R_t)
   103  except Exception as e:   # the solver found no solution: the source's targets pass
   105      return q_src.copy()
   ```
6. The measured joints show that the arm did not respond to the correction. Measured left_joint1 from the episode log is the same as the registered baseline to nine digits through the whole ARMS phase (t = 0.9 s: -0.000989093 both; t = 10 s: 0.054575954 both; t = 20 s: 0.279908185 both; t = 28 s: 0.387977824 both). The trace shows C growing during that same phase (|C| 6.8 mm at tick 500 in the M263 scripted run, see below).

**Conclusion for (b).** The correction is computed and built, but the arm receives the source's own targets. The only path in `correct()` that returns the source's targets unchanged for an arms command is the solver fallback at lines 103-105. So the most likely explanation is that `arm.step` raised on the navigation commands and the fallback passed the targets through. That is not verified: the fallback writes an entry to `SagLayer.log` (kind "solver_no_solution"), and nothing writes that log out, so the run cannot show it.

**Why the trim saturates there.** Each command integrates `p_ref - p_now` (sag_layer.py:94-95). `p_ref` is the forward kinematics of the staging's targets, and the arm lags those targets while the ARMS move runs (about 30 s, ticks 1 to 4480). So the error is positive for the whole move. Because the correction never reaches the arm, the error never closes: the integrator is open loop. Gain 3.0 per s times that error grows C until it hits the 40 mm limit (sag_max_m, sag_layer.py:56). In the M263 scripted run the norm reached 6.83 mm at tick 500, 37.7 mm at tick 1000 and 40.0 mm at tick 1033, and stayed there.

**(c) Identifying the manipulation source today.** Before this change there was no way to identify it in the loop:
- The scripted staging and the scripted manipulation module are both inside the one "policy" source (the orchestrator, src/orchestrator.py). Their commands arrive in one list with no tag.
- The mock is identifiable by its source name "vla" (mpvi_pipeline.py:108).
- The orchestrator's segments do record the module (`steps[i]["type"]` is "navigation" or "manipulation", orchestrator.py phase_text), but the status it reported did not expose it.

## Part 2 (build)

- `src/sag_layer.py`: `MANIPULATION_SOURCES = ("vla",)`; `SagLayer.manipulation(is_manip, t)` (the takeover: the first manipulation command after a non-manipulation command zeroes C; a non-manipulation command ends the takeover); `correct_command(layers, cmd, data, t, nominal_dt)` (corrects an arms command only when it is tagged manipulation; other commands pass through unchanged and integrate nothing).
- `src/orchestrator.py`: status gets `"module"`: "navigation" or "manipulation" for the current segment, None when the plan is done.
- `src/control_loop.py`: `is_manipulation(name)` (source name in MANIPULATION_SOURCES, or the auto source whose status module is "manipulation"); each cleaned command is tagged `clean["manip"]` (line ~1110); `sag_correct` calls `correct_command`.
- Nothing else changed: the M263 reset, the reference, gains and limits, the held-mass freeze, the helper hold, and the setting that turns the layer off.

## Synthetic tests (experiments/sag_layer_test.py: 14 of 14 pass)

| case | what it checks | result |
|---|---|---|
| M264 (a) | a staging command passes unchanged; no trim step; C stays zero (300 staging commands) | PASS (targets unchanged; trim steps 0) |
| M264 (b) | a manipulation command integrates and equals the layer's own correction (120 commands) | PASS (|C| 28.18 mm; changed from the source) |
| M264 (c) | after a 60-command manipulation run (|C| 16.61 mm) and 400 staging commands, the first manipulation command's trim equals a fresh layer's one step | PASS |
| (a) M263 | no reset on an ordinary open command | PASS |
| (b) M263 | reset when the release completes; none before close_min_s | PASS |
| (c) M263 | no periodic reset over 10 s with the gripper open | PASS |
| (d) M263 | logging columns written, header and row order match, guard counter, reset cause | PASS |
| (g) M263 | one release, one reset | PASS |
| (i) M263 | wiring: one release call site; sag_correct arms nothing | PASS |
| S1 | correction equals the old in-script trim (200 commands) | PASS |
| S2 | frozen during a held-mass ramp | PASS |
| S3 | holds during a helper pause | PASS |
| S4 | setting OFF passes targets through | PASS |
| S5 | two commands in one tick: no integration on the second | PASS |

Other suites: open catch 12 of 12; weigh helper 8 of 8; mock policy 9 of 9; held-mass ramp: 40 numbered checks, RESULT PASS. (The earlier "41 of 41" was a count of the word PASS, including the RESULT line; the suite has 40 checks.)

## Part 3A: scripted 200 g run (run 20261105_052604; exit 1)

Outcome: IN failed twice. Attempt 1: 3.4 mm and 0.1 deg after 15.0 s (timeout). Attempt 2: 3.37 mm and 0.13 deg. The pick never reached CLOSE, so there was no grasp, holding, release or lift.

**Against the registered baseline held_200g_20261104_m258**

- liftoff_ticks.csv: not byte-identical. 58 common columns; no new columns. First differing tick 9688 (9687 for the C columns, which start there). Last differing tick 14318.
- episode_log.csv: not byte-identical. First differing row 647 (t = 66.00 s, start of PICK OVER); last differing row 955 (t = 96.80 s). Rows 1 to 646 are identical, which covers the whole navigation (ARMS, PEDESTAL, DRIVE, CREEP_IN).

| liftoff column | ticks differing | first | last | largest difference |
|---|---|---|---|---|
| pad_x_m | 4631 | 9688 | 14318 | 0.2086 m |
| pad_y_m | 4631 | 9688 | 14318 | 0.0581 m |
| pad_z_m | 4631 | 9688 | 14318 | 0.1066 m |
| err_m | 4375 | 9688 | 14318 | 0.1973 m |
| rot_err_deg | 4375 | 9688 | 14318 | 2.75 deg |
| C_p_x | 4632 | 9687 | 14318 | 0.0154 m |
| C_p_y | 4632 | 9687 | 14318 | 0.0029 m |
| C_p_z | 4632 | 9687 | 14318 | 0.0055 m |
| C_r_x / y / z | 4632 | 9687 | 14318 | 0.029 / 0.023 / 0.044 rad |
| mass_kg | 1833 | 11117 | 12949 | 0.2283 kg |
| target_kg | 1012 | 11788 | 12799 | 0.2248 kg |
| pad_force_N | 1833 | 11117 | 12949 | 2.24 N |
| held_tau_L1 / L2 / L3 / L4 / L5 / L6 / L7 | 1833 to 2944 | 9730 to 11117 | 12949 to 14318 | 1.07 / 0.28 / 0.29 / 0.87 / 0.06 / 0.009 / 0.42 N m |
| beaker_z_m | 3318 | 11001 | 14318 | 0.107 m |
| beaker_bench_contacts | 2717 | 11002 | 14318 | 4 |
| grip_cmd_m, grip_applied_m | 2309, 1951 | 11000, 11000 | 14318, 12950 | 0.044 m |
| ramp_mode, ramp_start_kg, ramp_target_kg | 1283, 3305, 2295 | 11014, 11014, 11014 | 14318 | 0.228 / 0.225 kg |
| ref_z_m | 4067 | 9916 | 14318 | 0.109 m |
| label | 3320 | 10999 | 14318 | text |
| frozen, trace_ok | 1434, 1283 | 11014 | 14318 | 1 |
| tbl_N, pad_N, pad_contacts | 3318, 1950, 1951 | 11001 | 14318 | 1.96 N, 2.09 N, 15 |
| arm_pad_Fz_N, act_L1..L7, bias_L1..L7 | 4631 | 9688 | 14318 | 3.0 N, 2.0 N, 3.0 N, 1.6 N, 1.3 N, 0.4 N, 0.3 N, 0.7 N; bias up to 2.8 N m |

Episode-log differing columns (count of rows): left and right joints, the commanded joints, base_x, base_y, base_yaw_deg, pedestal_m, finger columns, phase, beaker_x, beaker_y, beaker_z, cmd_left_gripper, cmd_left_grip_max_force_N.

**Events (left hand):** CLOSE/UP, holding ON, still ON, weighing end, release start, gripper-open command, BACK start: none reached. Estimated mass: none. M244 lift rise, drop and place DOWN swing: not measurable (no lift).

**Resets:** none (no release, and the recovery's open was not a release).

**Trim at tick 2000:** (0.00, 0.00, 0.00) mm. **At pick start (tick 9687):** (-0.011, +0.006, -0.020) mm. Trim during navigation is zero from tick 1 to tick 9686 (the layer does not act on the staging).

**Trim and pad error at the end of each move step (first attempt)**

| step | end tick | new C (x, y, z) mm | old C (x, y, z) mm | pad error mm | rot deg | tolerance (3 mm, 2 deg) |
|---|---|---|---|---|---|---|
| pick OVER | 9915 | -2.99, -1.65, +1.31 | -2.97, -1.70, +1.29 | 2.46 | 1.45 | inside |
| pick DOWN | 10467 | -3.06, -0.71, +1.02 | -3.07, -0.72, +1.01 | 0.84 | 0.19 | inside |
| pick IN | 12723 | +0.33, -2.59, -3.79 | -2.10, -2.67, -1.30 | 3.37 | 0.13 | outside (timeout) |
| pick UP | not reached | | +2.46, -0.66, -3.60 | | | |
| place OVER, DOWN, BACK | not reached | | -5.60,-0.66,-1.00; -0.94,-0.57,-4.79; +3.22,+0.01,-2.78 | | | |

Second attempt: OVER end C (-3.12, -1.64, +1.17); DOWN end (-3.06, -0.71, +1.02); IN end (+0.35, -2.86, -3.35), error 3.37 mm, 0.13 deg.

## Part 3B: mock policy 200 g run (run 20261105_052604, mock; exit 1)

Outcome: stopped at IN. The IN timeout expired at 15.3 s (limit 3 x 3.017 s + 6 s = 15.05 s). It was not within 1 s of expiring before; no pause.

| step | start (sim s) | end (sim s) | rows | pad error at end (x, y, z mm) | norm mm | trim at end (x, y, z mm) | how it ended |
|---|---|---|---|---|---|---|---|
| OVER | 65.99 | 68.43 | 9688 to 10055 | -0.05, +0.07, -0.08 | 0.11 | +2.35, -0.17, +2.56 | inside tolerance |
| DOWN | 68.44 | 73.65 | 10056 to 10837 | -0.48, +0.67, -1.46 | 1.70 | +1.56, -0.68, -0.29 | inside tolerance |
| IN | 73.65 | 88.99 | 10838 to 13139 | -0.39, +2.63, +2.55 at the stop | 3.7 at the stop (0.1 deg) | +0.27, -2.66, -1.78 | timeout at 15.3 s |

- Where it stopped: IN, the arm stalled at path progress 2.595 s of 3.017 s; the failure message reads 3.8 mm and 0.1 deg from the waypoint.
- Queries: 75 (OVER 8, DOWN 17, IN 50). Wall time per query: mean 0.47 s, min 0.30 s, max 2.82 s.
- Helper pauses: none. Resets: none. Guard counter: 0. Estimated mass and M244: not measured (no lift).
- Trim in the mock run: |C| 5.5 mm at tick 9900.

## Not built, needs decision

1. **Solver fallback.** The windup in navigation needs the fallback at sag_layer.py:103-105 to be the reason the correction never reached the arm. I did not verify it. Default: no counter added. A counter would settle it.
2. **Identification of the manipulation source.** The name "vla" for the mock, plus the orchestrator's module flag for the scripted policy. Default: as built.
3. **Solver warm start at takeover.** The layer keeps its last joint command (q_cmd), so the first manipulation command starts from the last correction, not from the measured joints. Default: as built (David asked for the trim to start from zero, not the warm start).
4. **Reference offset in IN.** Forward kinematics of the solver's targets sits about +2 mm in z above the planned point during IN. With the windup gone, IN still fails at 3.4 mm (scripted) and 3.8 mm (mock). David kept the reference, so it is unchanged.
5. **Skill-start and recovery-start resets.** The loop cannot see them. Default: carry-over.
6. **Known limit.** The trim runs on joint-only moves (recovery HOME).
7. **M262** stays open.
