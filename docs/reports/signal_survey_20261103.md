# Signal survey for the holding, re-weighing and stillness detectors (Nov 3, 2026)

Read-only survey. No code changed, no simulation run, nothing pushed. The report is left uncommitted, as the task asked. Repo HEAD when the survey ran: d0fa806 (the M246 commit). Analysis scripts are outside the repo, in /tmp/signal_survey_20261103/.

Labels used below. **Shown by the data/code**: read directly from a file or a saved value, with file and line. **Calculated from the data**: computed here from saved values; the method is stated. **Guess**: a judgement, not checked. **unknown**: the material does not say.

## Summary in plain words

- **Finger closure** (how far the two fingers are open or shut) exists in the saved data, but only at 10 samples per second, in `episode_log.csv`. The 150 Hz trace (`liftoff_ticks.csv`) does not hold it; it holds only the gripper command. Finger position is a joint reading, so a real robot's finger encoder could give the same thing.
- **Finger squeeze** has one usable signal for a real robot: the finger actuator force (`left_finger_force_N`, 10 Hz). It is the servo's own force output, the same kind of value as a gripper motor current. While holding it sits at the 8 N cap in every saved run, so it does not change with the beaker's mass here. The contact forces on the beaker (`pad_N`, `tbl_N`, `pad_contacts`) exist only in the simulator.
- **Hand (pad centre) position** is logged every tick (150 Hz). Hand speed and acceleration are not logged; they were computed here by differences. The simulator adds no sensor noise (no noise term in `src/`), so the spread seen is physics jitter, not sensor noise. Arm joint speeds are not logged at tick rate; only the 10 Hz joint positions are saved.
- **Main ranges.** While holding and at rest (the WEIGH step), hand speed has a median of 0.00007 to 0.0001 m/s and a maximum of 0.0004 to 0.0012 m/s. In the approach the median is about 0.04 m/s, and in the carry (place OVER) it is about 0.13 m/s, with maxima of about 0.11 and 0.24 m/s. Finger closure while holding is 0.0375 m in all three runs. The longest stretch below 0.001 m/s lasts 4.6 to 6.3 s; it starts in the CLOSE step and ends in the WEIGH step or at the start of UP.
- **Weight over-read** against the true beaker mass at the target: +25.4 g (150 g), +24.5 g (200 g), +0.4 g (250 g). These are shown in the bounce analysis and reproduced here from the saved data.

## Terms

- **Tick**: one control-loop step. Here 1/150 s (0.006667 s), from `meta.json` (`source_hz` 150) and the logged times.
- **Finger closure / finger position**: the finger joint's position in metres. 0 is fully closed, about 0.044 is fully open (`src/mpvi_modules.py` 419 uses `travel = qpos`; the open value is `finger_open` in `src/task_config.py` 294 and 383).
- **Finger actuator force**: the force the gripper servo produces. It is the servo's output, not a contact sensor.
- **Squeeze**: in this report, the force holding the beaker. It is read here from the finger actuator force and from the simulator's contact forces.
- **Hand speed / acceleration**: speed (m/s) and acceleration (m/s²) of the pad centre. Speed is a vector length, so it is never negative.
- **Joint speed**: how fast an arm joint turns (rad/s). Joint velocity is the model's own state (`data.qvel`).
- **qfrc_bias**: the simulator's own computed gravity and Coriolis term for each joint. It is a model value, not a measurement.
- **Percentile (p95)**: the value that 95 % of samples fall at or below.
- **Sim truth**: a value only the simulator knows (for example a contact force), which a real robot does not have directly.

## Signal table

Units, sign and rate are as written by the code. "Real robot / policy" means the signal could also come from hardware or from the openpi policy's inputs. That is a guess until hardware is checked.

| Signal | Column(s) and file | Written by | Units, sign, rate | Source | Real robot / policy | Label |
|---|---|---|---|---|---|---|
| a. Finger closure (left fingers) | `left_finger1`, `left_finger2` in `episode/episode_log.csv` | `src/episode_runner.py` 160 (`finger_qadr`), 209 (written) | m; 0 = shut, about 0.044 = open; 10 Hz (`src/episode_runner.py` 204-205 with `log_hz`; the file has 0.1 s spacing) | simulator joint state (`qpos`) | yes in principle: a finger joint encoder. Not checked on hardware. | shown by the data/code |
| a. Finger closure, tick rate | not logged | (none) | unknown at tick rate | (none) | (none) | unknown |
| a. Gripper command | `grip_applied_m` in `liftoff_ticks.csv` | `experiments/bounce_liftoff_run.py` 134 (`data.ctrl`) | m; the command, not a measurement; 150 Hz | controller command | the command is known to the controller on any robot. It is not a sensor. | shown by the data/code |
| a. Gripper command from the step | `grip_cmd_m` | `experiments/bounce_liftoff_run.py` 154 | m; blank when no step sends one | controller | same as above | shown by the data/code |
| b. Finger squeeze, servo force | `left_finger_force_N` in `episode_log.csv` | `src/episode_runner.py` 214: `abs(data.actuator_force[...]) / fingers_per_actuator` | N, the mean absolute force per finger (magnitude, so no sign); 10 Hz | simulator servo output (MuJoCo actuator force) | yes in principle: the servo's current or torque. Not checked on hardware. | shown by the data/code |
| b. Finger force cap (command) | `cmd_left_grip_max_force_N` in `episode_log.csv` | `src/episode_runner.py` 213 | N per finger; 9 N open, 8 N while gripping the glass beaker (`src/task_config.py` 151-152, `GRIP_FORCE_TABLE`) | controller command | yes (a command) | shown by the data/code |
| b. Pad contact force on beaker | `pad_N` in `liftoff_ticks.csv` | `experiments/bounce_liftoff_run.py` 137-138, 100-111 (`_beaker_forces`) | N; the vertical part (z, up positive) of the finger-pad contact forces on the beaker; 150 Hz | simulator contacts | no; a real robot has no pad sensor in this model | shown by the data/code |
| b. Table contact force on beaker | `tbl_N` in `liftoff_ticks.csv` | same as above | N; the vertical contact force from the bench geoms on the beaker, up positive | simulator contacts | no | shown by the data/code |
| b. Pad contact count | `pad_contacts` in `liftoff_ticks.csv` | `experiments/bounce_liftoff_run.py` 138, 110 | count of pad-beaker contacts | simulator contacts | no | shown by the data/code |
| b. Beaker weight | `beaker_wt_N` in `liftoff_ticks.csv` | `experiments/bounce_liftoff_run.py` 139 | N: beaker body mass times g | simulator model | no (the true mass is not known on a robot) | shown by the data/code |
| b. Arm joint actuator forces (left arm) | `act_L1` to `act_L7` in `liftoff_ticks.csv` | `experiments/bounce_liftoff_run.py` 140, 148 | N m per joint; 150 Hz | simulator servo output | yes in principle: the joint servo's torque or current | shown by the data/code |
| b. Model bias torque | `bias_L1` to `bias_L7` | `experiments/bounce_liftoff_run.py` 141, 149 | N m, `qfrc_bias` (gravity and Coriolis, model-computed) | model, not a measurement | a model term, not a sensor | shown by the data/code |
| b. Force at the pad, estimated | `arm_pad_Fz_N` | `experiments/bounce_liftoff_run.py` 142-146 (least squares of act minus bias through the pad Jacobian) | N, z component; computed from the simulator's joint torques and the model Jacobian | computed from model | only through the model | shown by the data/code (method); calculated |
| c. Hand (pad centre) position | `pad_x_m`, `pad_y_m`, `pad_z_m` in `liftoff_ticks.csv` | `experiments/bounce_liftoff_run.py` 123-124 (tool-centre position plus the pad offset, from the simulator's kinematics), 131 | m, world frame, z up; 150 Hz | simulator state, through the model's forward kinematics | yes in principle: joint encoders with the same kinematic model | shown by the data/code |
| c. Hand speed | not logged | (none) | m/s; computed here by finite differences of the pad position over `t_sim` | calculated | yes in principle, from the same positions | calculated from the data |
| c. Hand acceleration | not logged | (none) | m/s²; computed here as the change of the speed vector over `t_sim` | calculated | same as speed; this needs two differences, so noise is larger | calculated from the data |
| c. Acceleration noise | (none) | (none) | No sensor-noise term in `src/` (search found none). The spread in still phases is physics jitter: median 0.002 to 0.003 m/s² in the WEIGH step (table below). | (none) | (none) | shown by the data/code (no noise term); calculated (spread) |
| c. Arm joint positions | `left_joint1` to `left_joint7` in `episode_log.csv` | `src/episode_runner.py` 209 | rad; 10 Hz | simulator joint state | yes | shown by the data/code |
| c. Arm joint speed, tick rate | not logged | (none). The code reads `data.qvel` in `src/mpvi_modules.py` 313 and 502, and `src/control_loop.py` 672 (a docstring describing the velocity reads). None of these is written to a log. | rad/s | simulator state | yes in principle: joint velocity or differences of encoder readings | unknown at tick rate |
| c. Arm joint speed, 10 Hz | computed here from the 10 Hz joint positions | (calculated) | rad/s; coarse (0.1 s spacing) | calculated | yes | calculated from the data |
| d. Grasp rule, map grasp | `src/grasp_rules.py` 1-6, 71 | (design-time) | Checks the hand's level, approach bin and side grasp for the map (`GRASP_MAP_CFG`). No finger, force or holding input. | design rule | not a runtime signal | shown by the data/code |
| d. Stance checks | `src/stance_checks.py` 1-13 | (design-time) | Checks home clearance, view margin and occlusion (`STANCE_REQUIREMENTS`). No holding input. | design rule | not a runtime signal | shown by the data/code |

### What already decides a grasp, a hold or a pause (step 4d; facts only, not endorsed)

Each line gives the inputs and the thresholds in the code. Values are from `src/task_config.py` unless stated.

- **Gripper closed on nothing / closed too far (CLOSE step), in `src/mpvi_modules.py` 419-433.** Inputs: finger travel `qpos` and finger speed `qvel`. Thresholds: `stall_speed_m_s` 0.002 m/s and `stall_dwell_s` 0.3 s (`src/task_config.py` 445; the fingers count as stopped below the speed for the dwell time). `held_travel_range_m` (0.003, 0.040) m (line 446). `close_min_s` 1.0 s, `close_timeout_s` 5.0 s, `open_timeout_s` 4.0 s (line 444). `open_reached_margin_m` 0.003 m (line 447).
- **Sim oracle "beaker is held" (`src/mpvi_modules.py` 576-586).** Inputs: finger travel `qpos` and the beaker's distance from the pad centre (simulator state). Test: travel in `held_travel_range_m` and distance below 0.02 m (2 cm), as written in the `ok` line in that range. This is a checker for the scripted pipeline, not a runtime rule.
- **Settled (waypoint and still checks), `src/mpvi_modules.py` 311-313.** Inputs: the largest arm joint speed (`data.qvel`). Threshold: `settle_speed_rad_s` 0.02 rad/s for `settle_dwell_s` 0.2 s (`src/task_config.py` 435). The same settle thresholds are also set in `src/control_loop.py` 133-136 for the base, pedestal and arms (labelled "provisional" in that file).
- **Lift-off detection (WEIGH step), `src/weight_estimate.py` 117-124 and `src/mpvi_modules.py` 448-455.** Inputs: pad height above its baseline. Thresholds: `liftoff_rise_mm` 0.3 mm, `liftoff_hold_ticks` 5 ticks (about 33 ms at 150 Hz), `liftoff_baseline_ticks` 10 ticks, `liftoff_rate_n_per_s` 0.5 N/s, `liftoff_max_mass_kg` 1.0 kg (`src/task_config.py` 438-442).
- **Lift-off estimate, `src/weight_estimate.py` 126.** Input: mass at detection and the pad rise. Subtracts the rise divided by the hand's height per kilogram (decision M236).
- **Still-enough reading for the handoff weight, `src/weight_estimate.py` 21-46 (`reading_status`).** Inputs: the largest joint speed for each tick, the handoff time. Thresholds: `still_speed_rad_s` 0.005 rad/s, `window_s` 0.30 s, `max_wait_s` 1.0 s (`src/task_config.py` 146-148). The call is in `src/mpvi_modules.py` 478.
- **Grip force cap, `src/task_config.py` 151-152.** 8.0 N per finger for the glass beaker (decision M083). The arm's fingers have a force limit of 9.0 N and `fingers_per_actuator` 2 (`src/robot_specs.py` 183).
- **Base stop while holding, `src/control_loop.py` 49 and 66 (docstrings).** The docstrings say the base is stopped while holding, at `base_stop_lin_decel` 1.875 m/s² and `base_stop_ang_decel` 2.5 rad/s² (`src/control_loop.py` 137-138). **unknown**: the code that decides "holding" for this rule was not located in this survey.

Missing from the code, as of this survey: no input uses finger force (the only finger-force logic is the cap in the command). Holding is decided from finger travel and from the simulator's distance check. Both are listed above.

## Range tables (step 5)

Per run: phase windows, then per-tick ranges (min / median / p95 / max) for hand speed, hand acceleration, grip command, pad and table contact forces (simulator only), finger closure and finger force (10 Hz), the cap, and arm joint speed (10 Hz, calculated). Phases are defined by the step label column; ticks are inclusive. The pause tables are the longest stretch of consecutive ticks below each speed in the manipulation span.

## run_150g

- trace rows 14241, ticks 0 to 14240, tick spacing 0.006667 s (median); episode log rows 950, spacing 0.100 s.
- beaker weight from the simulator (beaker_wt_N at tick 0 divided by g): 0.1500 kg (calculated).

Phase windows (ticks, from the label column; inclusive):

| phase | labels | ticks | duration s |
|---|---|---|---|
| gripper open, empty (approach) | pick: OVER, pick: DOWN, pick: IN | 9687 to 10998 | 8.75 |
| closing | pick: CLOSE | 10999 to 11149 | 1.01 |
| closed, weighing at rest | pick: WEIGH | 11150 to 11707 | 3.72 |
| lift (pick UP) | pick: UP | 11708 to 12156 | 2.99 |
| carry / over (place OVER) | place: OVER | 12158 to 12409 | 1.68 |
| place DOWN | place: DOWN | 12410 to 12722 | 2.09 |
| opening (place OPEN) | place: OPEN | 12723 to 12931 | 1.39 |
| empty again (BACK, HOME) | place: BACK, place: HOME | 12932 to 13940 | 6.73 |

Per-tick ranges, min / median / p95 / max:

| phase | hand speed m/s | hand accel m/s² | grip command (ctrl) m | pad contact z N | table z N | pad contacts (count) | finger closure m (left_finger1, 10 Hz) | finger force N (left_finger_force_N, 10 Hz) | grip force cap N (cmd, 10 Hz) | arm joint speed max rad/s (10 Hz) | samples at 10 Hz |
|---|---|---|---|---|---|---|---|---|---|---|---|
| gripper open, empty (approach) | 1.638e-05 / 0.04381 / 0.0965 / 0.1096 | 4.261e-05 / 0.05591 / 0.3431 / 0.8945 | 0.044 / 0.044 / 0.044 / 0.044 | 0 / 0 / 0 / 0 | 1.47 / 1.471 / 1.473 / 1.473 | 0 / 0 / 0 / 0 | 0.04393 / 0.04393 / 0.04393 / 0.04393 | 0.02178 / 0.02196 / 0.022 / 0.02231 | 9 / 9 / 9 / 9 | 7.182e-05 / 0.2424 / 0.4554 / 0.4995 | 88 |
| closing | 1.055e-05 / 4.792e-05 / 0.01676 / 0.06254 | 0.0001986 / 0.002139 / 1.161 / 9.373 | 0 / 0 / 0 / 0.044 | -1.557 / -0.02385 / 0.1025 / 1.317 | 0 / 1.495 / 1.604 / 2.231 | 0 / 8 / 10 / 15 | 0.03751 / 0.03752 / 0.03752 / 0.03752 | 8 / 8 / 8 / 8 | 8 / 8 / 8 / 8 | 8.444e-05 / 0.0001007 / 0.01727 / 0.02984 | 10 |
| closed, weighing at rest | 9.328e-06 / 0.0001016 / 0.0002053 / 0.0012 | 5.637e-05 / 0.002246 / 0.009922 / 0.03201 | 0 / 0 / 0 / 0 | -0.02438 / 0.6852 / 1.47 / 1.474 | 0 / 0.7854 / 1.439 / 1.496 | 7 / 8 / 10 / 10 | 0.03751 / 0.03751 / 0.03752 / 0.03752 | 8 / 8 / 8 / 8 | 8 / 8 / 8 / 8 | 8.812e-05 / 0.0003889 / 0.0005738 / 0.001275 | 37 |
| lift (pick UP) | 8.039e-05 / 0.03878 / 0.07332 / 0.07876 | 1.372e-05 / 0.09152 / 0.2259 / 0.4052 | 0 / 0 / 0 / 0 | 1.411 / 1.473 / 1.495 / 1.503 | 0 / 0 / 0 / 0 | 5 / 7 / 10 / 10 | 0.03751 / 0.03752 / 0.03752 / 0.03752 | 8 / 8 / 8 / 8 | 8 / 8 / 8 / 8 | 0.0002137 / 0.2195 / 0.5005 / 0.5113 | 30 |
| carry / over (place OVER) | 0.0002527 / 0.131 / 0.2423 / 0.2432 | 0.0003824 / 0.3137 / 0.674 / 0.7166 | 0 / 0 / 0 / 0 | 1.441 / 1.468 / 1.499 / 1.508 | 0 / 0 / 0 / 0 | 4 / 5 / 7 / 7 | 0.03752 / 0.03752 / 0.03752 / 0.03752 | 8 / 8 / 8 / 8 | 8 / 8 / 8 / 8 | 0.0003574 / 0.2382 / 0.3252 / 0.3315 | 17 |
| place DOWN | 0.0001886 / 0.01505 / 0.09531 / 0.1004 | 0.001335 / 0.1039 / 0.4018 / 1.065 | 0 / 0 / 0 / 0 | 0.9236 / 1.47 / 1.518 / 1.525 | 0 / 0 / 0 / 0 | 4 / 7 / 8 / 8 | 0.03751 / 0.03752 / 0.03752 / 0.03752 | 8 / 8 / 8 / 8 | 8 / 8 / 8 / 8 | 0.008294 / 0.04366 / 0.2804 / 0.2937 | 21 |
| opening (place OPEN) | 0.0001271 / 0.000755 / 0.009827 / 0.01547 | 3.213e-06 / 0.01151 / 0.09114 / 0.7525 | 0 / 0 / 0.044 / 0.044 | -0.1663 / 0.1723 / 1.474 / 1.474 | 0 / 0 / 0 / 0 | 0 / 4 / 8 / 10 | 0.03751 / 0.03752 / 0.04393 / 0.04393 | 0.01438 / 8 / 8 / 8 | 8 / 8 / 9 / 9 | 0.0002225 / 0.002512 / 0.01925 / 0.02885 | 14 |
| empty again (BACK, HOME) | 1.391e-05 / 0.03568 / 0.07781 / 0.08007 | 3.231e-07 / 0.03938 / 0.2362 / 0.5053 | 0.044 / 0.044 / 0.044 / 0.044 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 0.04393 / 0.04393 / 0.04393 / 0.04393 | 0.02174 / 0.02195 / 0.02198 / 0.02205 | 9 / 9 / 9 / 9 | 7.298e-05 / 0.1738 / 0.4602 / 0.5127 | 67 |

Longest stretch of consecutive ticks with hand speed below each value (manipulation span, ticks 9687 to 13940; the stretch is tabulated, no value is chosen):

| speed below (m/s) | longest stretch ticks | seconds | start tick | end tick | label at start | label at end |
|---|---|---|---|---|---|---|
| 0.001 | 691 | 4.607 | 11013 | 11703 | pick: CLOSE | pick: WEIGH |
| 0.002 | 719 | 4.793 | 11013 | 11731 | pick: CLOSE | pick: UP |
| 0.005 | 728 | 4.853 | 11012 | 11739 | pick: CLOSE | pick: UP |
| 0.01 | 737 | 4.913 | 11011 | 11747 | pick: CLOSE | pick: UP |
| 0.02 | 746 | 4.973 | 11009 | 11754 | pick: CLOSE | pick: UP |

Weight estimate: mass_kg at the last WEIGH tick (tick 11707): 0.1791 kg; target_kg there: 0.1754 kg; simulator beaker weight 0.1500 kg (calculated).

## run_200g

- trace rows 14381, ticks 0 to 14380, tick spacing 0.006667 s (median); episode log rows 959, spacing 0.100 s.
- beaker weight from the simulator (beaker_wt_N at tick 0 divided by g): 0.2000 kg (calculated).

Phase windows (ticks, from the label column; inclusive):

| phase | labels | ticks | duration s |
|---|---|---|---|
| gripper open, empty (approach) | pick: OVER, pick: DOWN, pick: IN | 9687 to 10998 | 8.75 |
| closing | pick: CLOSE | 10999 to 11149 | 1.01 |
| closed, weighing at rest | pick: WEIGH | 11150 to 11851 | 4.68 |
| lift (pick UP) | pick: UP | 11852 to 12300 | 2.99 |
| carry / over (place OVER) | place: OVER | 12302 to 12553 | 1.68 |
| place DOWN | place: DOWN | 12554 to 12862 | 2.06 |
| opening (place OPEN) | place: OPEN | 12863 to 13071 | 1.39 |
| empty again (BACK, HOME) | place: BACK, place: HOME | 13072 to 14080 | 6.73 |

Per-tick ranges, min / median / p95 / max:

| phase | hand speed m/s | hand accel m/s² | grip command (ctrl) m | pad contact z N | table z N | pad contacts (count) | finger closure m (left_finger1, 10 Hz) | finger force N (left_finger_force_N, 10 Hz) | grip force cap N (cmd, 10 Hz) | arm joint speed max rad/s (10 Hz) | samples at 10 Hz |
|---|---|---|---|---|---|---|---|---|---|---|---|
| gripper open, empty (approach) | 1.638e-05 / 0.04381 / 0.0965 / 0.1096 | 4.261e-05 / 0.05591 / 0.3431 / 0.8946 | 0.044 / 0.044 / 0.044 / 0.044 | 0 / 0 / 0 / 0 | 1.959 / 1.962 / 1.963 / 1.964 | 0 / 0 / 0 / 0 | 0.04393 / 0.04393 / 0.04393 / 0.04393 | 0.02178 / 0.02196 / 0.022 / 0.02231 | 9 / 9 / 9 / 9 | 7.182e-05 / 0.2424 / 0.4554 / 0.4995 | 88 |
| closing | 2.27e-06 / 2.153e-05 / 0.01095 / 0.06578 | 0.0001928 / 0.002158 / 0.8961 / 9.626 | 0 / 0 / 0 / 0.044 | -1.262 / -0.01248 / 0.09459 / 1.595 | 0 / 1.975 / 2.105 / 2.819 | 0 / 8 / 10 / 15 | 0.03751 / 0.03752 / 0.03752 / 0.03752 | 8 / 8 / 8 / 8 | 8 / 8 / 8 / 8 | 4.157e-05 / 5.71e-05 / 0.01744 / 0.03068 | 10 |
| closed, weighing at rest | 1.577e-06 / 8.285e-05 / 0.0002084 / 0.0007078 | 3.207e-05 / 0.002339 / 0.01443 / 0.04757 | 0 / 0 / 0 / 0 | -0.04096 / 0.8684 / 1.957 / 1.966 | 0 / 1.094 / 1.928 / 2.004 | 5 / 8 / 8 / 8 | 0.03751 / 0.03752 / 0.03752 / 0.03752 | 8 / 8 / 8 / 8 | 8 / 8 / 8 / 8 | 4.196e-05 / 0.0003719 / 0.0006202 / 0.0009263 | 47 |
| lift (pick UP) | 0.0001343 / 0.03887 / 0.07313 / 0.07866 | 1.612e-05 / 0.09125 / 0.23 / 0.3908 | 0 / 0 / 0 / 0 | 1.811 / 1.966 / 1.998 / 2.099 | 0 / 0 / 0 / 0 | 4 / 7 / 7 / 7 | 0.03751 / 0.03752 / 0.03752 / 0.03752 | 8 / 8 / 8 / 8 | 8 / 8 / 8 / 8 | 0.0003404 / 0.2222 / 0.504 / 0.509 | 30 |
| carry / over (place OVER) | 0.0002425 / 0.1309 / 0.2416 / 0.2423 | 2.282e-05 / 0.3119 / 0.6753 / 0.7193 | 0 / 0 / 0 / 0 | 1.816 / 1.962 / 2.043 / 2.098 | 0 / 0 / 0 / 0 | 4 / 6 / 7 / 7 | 0.03752 / 0.03752 / 0.03752 / 0.03752 | 8 / 8 / 8 / 8 | 8 / 8 / 8 / 8 | 0.0003824 / 0.2415 / 0.3266 / 0.3285 | 16 |
| place DOWN | 0.0003452 / 0.009509 / 0.09649 / 0.1014 | 0.007505 / 0.1146 / 0.4132 / 1.261 | 0 / 0 / 0 / 0 | 1.284 / 1.962 / 2.034 / 2.094 | 0 / 0 / 0 / 0 | 4 / 6 / 6 / 8 | 0.03752 / 0.03752 / 0.03752 / 0.03752 | 8 / 8 / 8 / 8 | 8 / 8 / 8 / 8 | 0.01055 / 0.04768 / 0.2747 / 0.2929 | 21 |
| opening (place OPEN) | 6.401e-05 / 0.0007182 / 0.0062 / 0.01257 | 0.000163 / 0.008345 / 0.1147 / 0.7703 | 0 / 0 / 0.044 / 0.044 | -0.2666 / 0.1148 / 1.961 / 1.962 | 0 / 0 / 0 / 0 | 0 / 6 / 6 / 6 | 0.03752 / 0.03752 / 0.04393 / 0.04393 | 0.02196 / 8 / 8 / 8 | 8 / 8 / 9 / 9 | 0.0003502 / 0.003232 / 0.01492 / 0.0189 | 14 |
| empty again (BACK, HOME) | 1.1e-05 / 0.03549 / 0.07792 / 0.08075 | 9.679e-07 / 0.03989 / 0.2382 / 0.5089 | 0.044 / 0.044 / 0.044 / 0.044 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 0.04393 / 0.04393 / 0.04393 / 0.04393 | 0.02173 / 0.02196 / 0.02198 / 0.02221 | 9 / 9 / 9 / 9 | 7.531e-05 / 0.1591 / 0.4597 / 0.5156 | 67 |

Longest stretch of consecutive ticks with hand speed below each value (manipulation span, ticks 9687 to 14080; the stretch is tabulated, no value is chosen):

| speed below (m/s) | longest stretch ticks | seconds | start tick | end tick | label at start | label at end |
|---|---|---|---|---|---|---|
| 0.001 | 859 | 5.727 | 11013 | 11871 | pick: CLOSE | pick: UP |
| 0.002 | 864 | 5.760 | 11012 | 11875 | pick: CLOSE | pick: UP |
| 0.005 | 875 | 5.833 | 11010 | 11884 | pick: CLOSE | pick: UP |
| 0.01 | 883 | 5.887 | 11009 | 11891 | pick: CLOSE | pick: UP |
| 0.02 | 892 | 5.947 | 11008 | 11899 | pick: CLOSE | pick: UP |

Weight estimate: mass_kg at the last WEIGH tick (tick 11851): 0.2280 kg; target_kg there: 0.2245 kg; simulator beaker weight 0.2000 kg (calculated).

## run_250g

- trace rows 14330, ticks 0 to 14329, tick spacing 0.006667 s (median); episode log rows 956, spacing 0.100 s.
- beaker weight from the simulator (beaker_wt_N at tick 0 divided by g): 0.2500 kg (calculated).

Phase windows (ticks, from the label column; inclusive):

| phase | labels | ticks | duration s |
|---|---|---|---|
| gripper open, empty (approach) | pick: OVER, pick: DOWN, pick: IN | 9687 to 10998 | 8.75 |
| closing | pick: CLOSE | 10999 to 11149 | 1.01 |
| closed, weighing at rest | pick: WEIGH | 11150 to 11927 | 5.19 |
| lift (pick UP) | pick: UP | 11928 to 12376 | 2.99 |
| carry / over (place OVER) | place: OVER | 12378 to 12629 | 1.68 |
| place DOWN | place: DOWN | 12630 to 12818 | 1.26 |
| opening (place OPEN) | place: OPEN | 12819 to 13027 | 1.39 |
| empty again (BACK, HOME) | place: BACK, place: HOME | 13028 to 14029 | 6.68 |

Per-tick ranges, min / median / p95 / max:

| phase | hand speed m/s | hand accel m/s² | grip command (ctrl) m | pad contact z N | table z N | pad contacts (count) | finger closure m (left_finger1, 10 Hz) | finger force N (left_finger_force_N, 10 Hz) | grip force cap N (cmd, 10 Hz) | arm joint speed max rad/s (10 Hz) | samples at 10 Hz |
|---|---|---|---|---|---|---|---|---|---|---|---|
| gripper open, empty (approach) | 1.638e-05 / 0.04381 / 0.0965 / 0.1096 | 4.261e-05 / 0.05591 / 0.3431 / 0.8945 | 0.044 / 0.044 / 0.044 / 0.044 | 0 / 0 / 0 / 0 | 2.449 / 2.452 / 2.454 / 2.455 | 0 / 0 / 0 / 0 | 0.04393 / 0.04393 / 0.04393 / 0.04393 | 0.02178 / 0.02196 / 0.022 / 0.02231 | 9 / 9 / 9 / 9 | 7.182e-05 / 0.2424 / 0.4554 / 0.4995 | 88 |
| closing | 2.42e-06 / 2.915e-05 / 0.00606 / 0.064 | 0.0001728 / 0.00231 / 0.6459 / 9.62 | 0 / 0 / 0 / 0.044 | -0.7989 / 0.04913 / 0.1115 / 1.813 | 0 / 2.403 / 2.52 / 3.25 | 0 / 8 / 10 / 15 | 0.03751 / 0.03752 / 0.03752 / 0.03752 | 8 / 8 / 8 / 8 | 8 / 8 / 8 / 8 | 4.036e-05 / 5.341e-05 / 0.01626 / 0.02885 | 10 |
| closed, weighing at rest | 2.863e-06 / 6.883e-05 / 0.0001887 / 0.0003914 | 5.338e-05 / 0.003068 / 0.01761 / 0.03905 | 0 / 0 / 0 / 0 | -0.02735 / 1.022 / 2.089 / 2.438 | 0 / 1.431 / 2.373 / 2.481 | 5 / 8 / 8 / 8 | 0.03752 / 0.03752 / 0.03752 / 0.03752 | 8 / 8 / 8 / 8 | 8 / 8 / 8 / 8 | 4.148e-05 / 0.0003409 / 0.0005888 / 0.0007815 | 52 |
| lift (pick UP) | 0.0001491 / 0.03835 / 0.07769 / 0.08027 | 5.014e-05 / 0.09266 / 0.2389 / 0.3846 | 0 / 0 / 0 / 0 | 2.082 / 2.457 / 2.532 / 2.578 | 0 / 0 / 0.1604 / 0.2918 | 7 / 8 / 8 / 8 | 0.03752 / 0.03752 / 0.03752 / 0.03752 | 8 / 8 / 8 / 8 | 8 / 8 / 8 / 8 | 0.0002182 / 0.2233 / 0.5042 / 0.5096 | 30 |
| carry / over (place OVER) | 0.0001598 / 0.1305 / 0.2412 / 0.2418 | 0.0002821 / 0.3051 / 0.6771 / 0.7198 | 0 / 0 / 0 / 0 | 2.33 / 2.468 / 2.537 / 2.56 | 0 / 0 / 0 / 0 | 5 / 7 / 7 / 8 | 0.03752 / 0.03752 / 0.03752 / 0.03752 | 8 / 8 / 8 / 8 | 8 / 8 / 8 / 8 | 0.0003393 / 0.2424 / 0.3259 / 0.3283 | 16 |
| place DOWN | 0.000432 / 0.01638 / 0.09352 / 0.09532 | 0.002843 / 0.1943 / 0.3965 / 2.75 | 0 / 0 / 0 / 0 | 1.263 / 2.361 / 2.537 / 2.597 | 0 / 0 / 0 / 0 | 3 / 6 / 6 / 7 | 0.03752 / 0.03752 / 0.03752 / 0.03752 | 8 / 8 / 8 / 8 | 8 / 8 / 8 / 8 | 0.005432 / 0.1469 / 0.2776 / 0.2911 | 13 |
| opening (place OPEN) | 4.609e-05 / 0.0003558 / 0.008297 / 0.00921 | 1.173e-05 / 0.01631 / 0.111 / 0.6267 | 0 / 0 / 0.044 / 0.044 | -0.2082 / 0.2901 / 2.27 / 2.377 | 0 / 0 / 0 / 0 | 0 / 6 / 7 / 7 | 0.03751 / 0.03752 / 0.04393 / 0.04393 | 0.02194 / 8 / 8 / 8 | 8 / 8 / 9 / 9 | 0.0003047 / 0.002618 / 0.0124 / 0.01544 | 14 |
| empty again (BACK, HOME) | 3.217e-06 / 0.03613 / 0.07878 / 0.08099 | 8.993e-07 / 0.03995 / 0.2259 / 0.4932 | 0.044 / 0.044 / 0.044 / 0.044 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 0.04393 / 0.04393 / 0.04393 / 0.04393 | 0.02174 / 0.02195 / 0.02198 / 0.02206 | 9 / 9 / 9 / 9 | 7.799e-05 / 0.1536 / 0.4574 / 0.5162 | 67 |

Longest stretch of consecutive ticks with hand speed below each value (manipulation span, ticks 9687 to 14029; the stretch is tabulated, no value is chosen):

| speed below (m/s) | longest stretch ticks | seconds | start tick | end tick | label at start | label at end |
|---|---|---|---|---|---|---|
| 0.001 | 948 | 6.320 | 11012 | 11959 | pick: CLOSE | pick: UP |
| 0.002 | 953 | 6.353 | 11011 | 11963 | pick: CLOSE | pick: UP |
| 0.005 | 959 | 6.393 | 11009 | 11967 | pick: CLOSE | pick: UP |
| 0.01 | 965 | 6.433 | 11008 | 11972 | pick: CLOSE | pick: UP |
| 0.02 | 976 | 6.507 | 11006 | 11981 | pick: CLOSE | pick: UP |

Weight estimate: mass_kg at the last WEIGH tick (tick 11927): 0.2538 kg; target_kg there: 0.2504 kg; simulator beaker weight 0.2500 kg (calculated).

## Pauses that span a skill boundary, and the rows around lift-off

```

## run_150g: pauses spanning a skill boundary (speed below value; stretch must include two different non-empty labels)
  0.001 m/s: 691 ticks (4.607 s), ticks 11013 to 11703, labels passed: pick: CLOSE -> pick: WEIGH
  0.002 m/s: 719 ticks (4.793 s), ticks 11013 to 11731, labels passed: pick: CLOSE -> pick: WEIGH -> pick: UP
  0.005 m/s: 728 ticks (4.853 s), ticks 11012 to 11739, labels passed: pick: CLOSE -> pick: WEIGH -> pick: UP
  0.01 m/s: 737 ticks (4.913 s), ticks 11011 to 11747, labels passed: pick: CLOSE -> pick: WEIGH -> pick: UP
  0.02 m/s: 746 ticks (4.973 s), ticks 11009 to 11754, labels passed: pick: CLOSE -> pick: WEIGH -> pick: UP

  rows around the end of WEIGH (tick 11707): mass_kg, target_kg, ramp_mode, ramp_rate, weigh_state, pad_z
    tick 11687: mass 0.1723  target 0.0000  ramp rate  rate 0.0509683995922528  state lift  pad_z 1.01321
    tick 11702: mass 0.1774  target 0.0000  ramp rate  rate 0.0509683995922528  state lift  pad_z 1.01325
    tick 11707: mass 0.1791  target 0.1754  ramp rate  rate 0.0509683995922528  state lift  pad_z 1.01328
    tick 11712: mass 0.1791  target 0.1754  ramp target  rate nan  state detected  pad_z 1.01332
    tick 11727: mass 0.1790  target 0.1754  ramp target  rate nan  state detected  pad_z 1.01342
  max |left_finger1 - left_finger2| over the episode log: 0.4363 mm

## run_200g: pauses spanning a skill boundary (speed below value; stretch must include two different non-empty labels)
  0.001 m/s: 859 ticks (5.727 s), ticks 11013 to 11871, labels passed: pick: CLOSE -> pick: WEIGH -> pick: UP
  0.002 m/s: 864 ticks (5.760 s), ticks 11012 to 11875, labels passed: pick: CLOSE -> pick: WEIGH -> pick: UP
  0.005 m/s: 875 ticks (5.833 s), ticks 11010 to 11884, labels passed: pick: CLOSE -> pick: WEIGH -> pick: UP
  0.01 m/s: 883 ticks (5.887 s), ticks 11009 to 11891, labels passed: pick: CLOSE -> pick: WEIGH -> pick: UP
  0.02 m/s: 892 ticks (5.947 s), ticks 11008 to 11899, labels passed: pick: CLOSE -> pick: WEIGH -> pick: UP

  rows around the end of WEIGH (tick 11851): mass_kg, target_kg, ramp_mode, ramp_rate, weigh_state, pad_z
    tick 11831: mass 0.2212  target 0.0000  ramp rate  rate 0.0509683995922528  state lift  pad_z 1.01320
    tick 11846: mass 0.2263  target 0.0000  ramp rate  rate 0.0509683995922528  state lift  pad_z 1.01321
    tick 11851: mass 0.2280  target 0.2245  ramp rate  rate 0.0509683995922528  state lift  pad_z 1.01323
    tick 11856: mass 0.2280  target 0.2245  ramp target  rate nan  state detected  pad_z 1.01325
    tick 11871: mass 0.2279  target 0.2245  ramp target  rate nan  state detected  pad_z 1.01334
  max |left_finger1 - left_finger2| over the episode log: 0.4348 mm

## run_250g: pauses spanning a skill boundary (speed below value; stretch must include two different non-empty labels)
  0.001 m/s: 948 ticks (6.320 s), ticks 11012 to 11959, labels passed: pick: CLOSE -> pick: WEIGH -> pick: UP
  0.002 m/s: 953 ticks (6.353 s), ticks 11011 to 11963, labels passed: pick: CLOSE -> pick: WEIGH -> pick: UP
  0.005 m/s: 959 ticks (6.393 s), ticks 11009 to 11967, labels passed: pick: CLOSE -> pick: WEIGH -> pick: UP
  0.01 m/s: 965 ticks (6.433 s), ticks 11008 to 11972, labels passed: pick: CLOSE -> pick: WEIGH -> pick: UP
  0.02 m/s: 976 ticks (6.507 s), ticks 11006 to 11981, labels passed: pick: CLOSE -> pick: WEIGH -> pick: UP

  rows around the end of WEIGH (tick 11927): mass_kg, target_kg, ramp_mode, ramp_rate, weigh_state, pad_z
    tick 11907: mass 0.2470  target 0.0000  ramp rate  rate 0.0509683995922528  state lift  pad_z 1.01320
    tick 11922: mass 0.2521  target 0.0000  ramp rate  rate 0.0509683995922528  state lift  pad_z 1.01322
    tick 11927: mass 0.2538  target 0.2504  ramp rate  rate 0.0509683995922528  state lift  pad_z 1.01322
    tick 11932: mass 0.2538  target 0.2504  ramp target  rate nan  state detected  pad_z 1.01323
    tick 11947: mass 0.2538  target 0.2504  ramp target  rate nan  state detected  pad_z 1.01326
  max |left_finger1 - left_finger2| over the episode log: 0.4354 mm
```

## Hand speed at each skill boundary

```
== run_150g: hand speed (m/s) at the last tick of each step and the first tick of the next
   pick: OVER end 0.00602 | pick: DOWN start 0.00630 (ticks 9915/9916)
   pick: DOWN end 0.00068 | pick: IN start 0.00083 (ticks 10467/10468)
   pick: IN end 0.00254 | pick: CLOSE start 0.00240 (ticks 10998/10999)
   pick: CLOSE end 0.00002 | pick: WEIGH start 0.00002 (ticks 11149/11150)
   pick: WEIGH end 0.00120 | pick: UP start 0.00125 (ticks 11707/11708)
   pick: UP end 0.00025 | place: OVER start 0.00025 (ticks 12156/12158)
   place: OVER end 0.00394 | place: DOWN start 0.00443 (ticks 12409/12410)
   place: DOWN end 0.00404 | place: OPEN start 0.00410 (ticks 12722/12723)
   place: OPEN end 0.00013 | place: BACK start 0.00013 (ticks 12931/12932)
   place: BACK end 0.00090 | place: HOME start 0.00135 (ticks 13132/13133)
== run_200g: hand speed (m/s) at the last tick of each step and the first tick of the next
   pick: OVER end 0.00602 | pick: DOWN start 0.00630 (ticks 9915/9916)
   pick: DOWN end 0.00068 | pick: IN start 0.00083 (ticks 10467/10468)
   pick: IN end 0.00254 | pick: CLOSE start 0.00240 (ticks 10998/10999)
   pick: CLOSE end 0.00003 | pick: WEIGH start 0.00001 (ticks 11149/11150)
   pick: WEIGH end 0.00071 | pick: UP start 0.00078 (ticks 11851/11852)
   pick: UP end 0.00023 | place: OVER start 0.00025 (ticks 12300/12302)
   place: OVER end 0.00389 | place: DOWN start 0.00441 (ticks 12553/12554)
   place: DOWN end 0.00044 | place: OPEN start 0.00049 (ticks 12862/12863)
   place: OPEN end 0.00015 | place: BACK start 0.00015 (ticks 13071/13072)
   place: BACK end 0.00106 | place: HOME start 0.00157 (ticks 13272/13273)
== run_250g: hand speed (m/s) at the last tick of each step and the first tick of the next
   pick: OVER end 0.00602 | pick: DOWN start 0.00630 (ticks 9915/9916)
   pick: DOWN end 0.00068 | pick: IN start 0.00084 (ticks 10467/10468)
   pick: IN end 0.00254 | pick: CLOSE start 0.00240 (ticks 10998/10999)
   pick: CLOSE end 0.00003 | pick: WEIGH start 0.00002 (ticks 11149/11150)
   pick: WEIGH end 0.00032 | pick: UP start 0.00026 (ticks 11927/11928)
   pick: UP end 0.00015 | place: OVER start 0.00016 (ticks 12376/12378)
   place: OVER end 0.00443 | place: DOWN start 0.00498 (ticks 12629/12630)
   place: DOWN end 0.00195 | place: OPEN start 0.00188 (ticks 12818/12819)
   place: OPEN end 0.00005 | place: BACK start 0.00005 (ticks 13027/13028)
   place: BACK end 0.00083 | place: HOME start 0.00115 (ticks 13230/13231)
```

## Weight over-read against the true mass (step 6)

Calculated from the saved runs. "True" is the simulator's beaker weight divided by g (`beaker_wt_N`). The estimate at detection is `mass_kg` at the last WEIGH tick. The target is `target_kg`, the ramp target set at detection after the pad-rise correction. The target column matches the bounce analysis (`docs/reports/bounce_analysis_20261103.md`, table near line 85: 0.1754, 0.2245, 0.2504 kg).

| run | true kg | estimate at detection kg (mass_kg) | target kg (target_kg) | target minus true |
|---|---|---|---|---|
| 150 g | 0.1500 | 0.1791 (tick 11707) | 0.1754 | +25.4 g |
| 200 g | 0.2000 | 0.2280 (tick 11851) | 0.2245 | +24.5 g |
| 250 g | 0.2500 | 0.2538 (tick 11927) | 0.2504 | +0.4 g |

Calculated: the gap between the detection mass and the target is 3.7 g (150 g), 3.5 g (200 g) and 3.4 g (250 g). The trace shows the mass rising at a constant rate until the detection tick, then the target is set. The rate mode is the lift-off rate ramp (`liftoff_rate_n_per_s` 0.5 N/s, `src/task_config.py` 438).

### Phases where the arm is still (ranges from step 5; no threshold chosen)

Hand speed in m/s from the per-tick table above, min / median / p95 / max:

- **Closed, weighing at rest (WEIGH):** median 0.00007 to 0.0001; p95 0.00019 to 0.00021; max 0.0004 to 0.0012. Caveat shown by the data: the table load moves onto the pads inside this window (`tbl_N` falls from about 1.47 N to 0 by the lift-off tick in the trace, and `pad_N` rises to about 1.47 N). So the beaker is still but the load is changing.
- **Closing (CLOSE):** median 0.00003 to 0.00005; p95 0.006 to 0.017; max 0.06 to 0.07. The large max is at the start of the step.
- **Place OPEN:** median 0.00036 to 0.00076; p95 0.006 to 0.010; max 0.009 to 0.015.
- **Skill boundaries** (speed on the last tick of one step and the first tick of the next, `boundary.txt`): 0.00002 to 0.0013 m/s at most boundaries. Exceptions: OVER to DOWN (about 0.006 m/s, all runs), place OVER to DOWN (about 0.004 to 0.005 m/s), and place DOWN to OPEN (0.0004 to 0.004 m/s, by run).
- **Moving phases:** approach median about 0.04 m/s; lift (UP) median 0.039 m/s; carry (place OVER) median 0.13 m/s; place DOWN median 0.0095 to 0.016 m/s with p95 0.09 to 0.10 m/s; empty again (BACK, HOME) median 0.036 m/s.

Whether any of these is "still enough" for a re-weigh is a decision (decision 3 says the limits come later from data). Nothing is chosen here.

## Entry text added to the decision log

Commit d0fa806, in `docs/mpvi_decisions.md`. Row M246 (after the M245 row), and a detail section "M246 detail" above the OPEN section. Dated NOTE lines were added to M201 and M214 (both are every-tick weighing rows), and to the OPEN section (the holding candidate). Nothing was rewritten.

> **M246** | 2026-11-03 | Holding detection, re-weighing and stillness below the policy (David, 2026-11-03). (1) HOLDING, DECIDED: the gripper counts as holding only when finger closure is inside the grasp range AND finger squeeze is above a threshold, held for a short confirming time. It is a plain on/off value, not a smooth 0-to-1 value. It turns off when the squeeze drops. The closure range, the squeeze threshold and the confirming time are not decided; they come later from data. (2) WEIGHING, DECIDED: the beaker is weighed at lift-off and re-weighed whenever the arm is still (at skill boundaries, and before place DOWN). Not continuous. This supersedes the Nov 2 preference for weighing every tick (M201, M214). Weighing during steady horizontal base cruise is an OPEN IDEA: steady motion adds little sideways force, and the arm's own gravity model still has about 15% unexplained torque (M192). (3) STILLNESS, DECIDED: "still enough" is judged from the arm's own hand speed and acceleration staying below limits for a short time. It does not rely on any signal from the scripted skills. The limits are not decided; they come later from data. (4) UNDER DISCUSSION, NOT DECIDED: moving the scripted-skill-triggered weight and trim logic below the policy one piece at a time, as a staged approach (first logging-only signals checked against what the scripted skills do, then one trigger switched at a time, with a baseline comparison after each). Not confirmed. The detail section below the table lists what is not decided. | decides (items 1-3); OPEN IDEA (base-cruise weighing); UNDER DISCUSSION (item 4) | paper silent | n/a | the rules as above; numbers not set | David (2026-11-03) | none (documentation only) |

Detail section "M246 detail", its "Not decided" list:

- The holding numbers: the grasp range for finger closure, the squeeze threshold, and the confirming time.
- The stillness limits: the hand speed limit, the hand acceleration limit, and the short time they must hold.
- Whether to confirm the staged approach in item 4.
- What tells the layer below the policy about skill boundaries. Not needed under item 3; left as written in the Nov 3 decision list.

The progress entry (in `docs/progress.md`) is the same commit.

Note on the Nov 2 anchor: the wording "holding detected from finger position plus force" is not in any Nov 2 entry. The closest text is the Nov 3 OPEN section ("Candidate, not decided: finger position together with finger force"), so the dated NOTE was placed there. David to confirm this placement.

## Commit

- **Hash: d0fa806**, message "Decision M246: holding detection, re-weighing and stillness below the policy (Nov 3); progress entry". Files: `docs/mpvi_decisions.md` and `docs/progress.md` only (22 insertions, 2 deletions in the decisions log; 2 insertions in progress). Not pushed.
- This report is not committed (as the task asked).

## Exact commands

Run from `/home/vboxuser/projects/openarm-tracer-sim` unless stated. No simulation was started.

```
pgrep -af -i 'mujoco|python.*teleop'          # check: only this check's own shell matched
git status --short
git log --oneline -5
ls -la docs/reports/records_positions_20261103.md
grep -n -i "hard-coded\|hard coded" docs/mpvi_decisions.md
git diff --stat -- docs/mpvi_decisions.md docs/progress.md     # empty before the edit
grep -n "^| M" docs/mpvi_decisions.md | tail -12               # next free number: M246
grep -n -i "every tick\|every-tick" docs/mpvi_decisions.md
grep -n -i "finger position plus\|position plus force" docs/*.md docs/reports/*.md
git add docs/mpvi_decisions.md docs/progress.md
git commit -q -m "Decision M246: holding detection, re-weighing and stillness below the policy (Nov 3); progress entry"
source .venv/bin/activate
python -I /tmp/signal_survey_20261103/survey.py          # reads CSV files only; writes /tmp/signal_survey_20261103/results.md
python -I /tmp/signal_survey_20261103/extra.py > /tmp/signal_survey_20261103/extra.txt
python -I /tmp/signal_survey_20261103/boundary.py > /tmp/signal_survey_20261103/boundary.txt
grep -rn -i "noise\|randn\|normal(" src --include=*.py    # no noise term found
grep -n "log_hz\|finger_force\|finger_qadr" src/episode_runner.py
grep -n "liftoff\|_beaker_forces\|pad_N\|tbl_N" experiments/bounce_liftoff_run.py | head
grep -n "held_travel_range_m\|stall_speed_m_s\|settle_speed_rad_s\|liftoff_" src/task_config.py
```

Search and reading tools also used: the CodeGraph explore tool (read-only), and file reads of the source lines cited above.

## Verification (step 7)

Run after this report was written; output in the chat reply.

- `git status --short` and `git diff --stat`: no tracked file changed by this survey. The only new file is this report (untracked). No code file was changed; nothing was pushed.

## NOT BUILT, needs decision

1. **The numbers David said come later from data:** the finger closure range, the squeeze threshold, the confirming time for holding; the hand speed and acceleration limits and their hold time for stillness. Not chosen or suggested here.
2. **Whether to confirm the staged approach** for moving the weight and trim logic below the policy (item 4).
3. **What tells the layer below the policy about skill boundaries.** Not needed under item 3, but left open as the request wrote it.
4. **Finger position is not logged at tick rate** (only 10 Hz in `episode_log.csv`). A logging-only change would be needed to get it at 150 Hz; not built, as the task said no code changes.
5. **Arm joint speed is not logged at tick rate.** Only 10 Hz joint positions exist. Not built.
6. **The finger force signal sits at its 8 N cap while holding** in all three runs, so it does not change with the beaker's mass here. Which squeeze signal the holding rule uses is a design choice: not decided.
7. **No sensor noise is modelled** (no noise term in `src/`). The noise a real robot would show in hand acceleration is unknown.
8. **The WEIGH window is still but the load is moving.** The table load falls to 0 and the pads take it during WEIGH. Whether WEIGH counts as "closed and holding at rest" for the holding rule is a decision: not decided.
9. **The "holding" trigger for the control loop's base stop was not located** (docstrings at `src/control_loop.py` 49 and 66 only). unknown.
10. **The Nov 2 anchor** "holding detected from finger position plus force" is not in the log; the note was placed on the Nov 3 OPEN section. David to confirm.
11. **Crash rules in `CLAUDE.md` say reports must be committed.** The task said to leave this one uncommitted; I followed the task. Decide whether it should be committed.
12. **`docs/design_plan.md` was not edited.** `CLAUDE.md` says the status markers in the plan should match each decision commit. The task limited edits to two files. Left for David.
13. **Git status shows `deps/AgileX_Tracer` as untracked**, and `docs/reports/records_positions_20261103.md` is still untracked. Neither was touched; both appeared outside this task's scope (the first was not in the start-of-session snapshot).
