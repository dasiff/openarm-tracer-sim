# Report 2026-10-20 13:56: the minimal MPVI pipeline runs end to end

Written for: David.

**It runs.** `experiments/mpvi_pipeline_test.py` passes 8/8: the hand-written plan goes through the orchestrator, the scripted staging, the scripted pick and place with mink and the oracle checker to SUCCESS, and the recording replays bit-identically (11,170 ticks, 75.9 s of simulated time, 98 s of wall time). Log: `data/logs/mpvi_pipeline_20261020_135325.log` (the run's episode log and recording are in the git-ignored `data/mpvi_run/`).

## What ran (sim time)
| t (s) | What |
|---|---|
| 1.4 to 47.4 | segment 1/3 navigation: the existing staging (ARMS, PEDESTAL, DRIVE, CREEP_IN, NECK), unchanged |
| 47.4 to 61.1 | segment 2/3 "Pick up the beaker": OVER, DOWN, IN, CLOSE, UP, CHECK; the close event at 58.2 s: oracle verdict PASS, logged only (the pick is not the last step of its unit) |
| 61.1 to 73.9 | segment 3/3 "Place the beaker on the hotplate": OVER, DOWN, OPEN, BACK, HOME; the open event at 67.0 s: the check decides, PASS |
| 75.9 | the runner's success (the plan finished and the beaker on the hotplate, at rest) plus its 2 s linger |

End state: the beaker 1.8 mm and 9.3 mm from the hotplate centre (footprint limit 65 mm), 0.1 mm below the plate height, upright, at rest; the left arm within 0.004 rad of home; the gripper open.

## The pieces (each behind its own interface; all new code in `src/`)
- `mpvi_plan.py`: the paper's segment format, a hand-written 3-segment plan, `postprocess()` with the tags (primitive, object, unit, and `check` on the last manipulation segment of each unit).
- `orchestrator.py`: runs the segments in order, calls the checker after gripper events (it decides at the event that ends a tagged segment's primitive, logs the other verdicts), any module error or failed check stops the plan with PLAN_FAILED and the reason (checked with fake modules; the end-to-end run only exercises success). The status row of the viewer shows `2/3 PICK: DOWN [running]` and the like (17 distinct texts in the run; I have not looked at the live window, the text goes through the existing "episode" row).
- `mpvi_modules.py`: the sim world model, navigation (the staging stage wrapped), manipulation (the scripted pick and place), the oracle checker.
- `mink_arm.py`: mink on the full model with every other degree of freedom frozen (a constraint), the pad centre as the target through the tool point, a posture task toward home, joint limits with a 10 degree margin on the arm's joints, 0.5 rad/s, and collision avoidance of the arm against the bench, hotplate and right arm. 1.2 ms per step.
- `mpvi_pipeline.py`: puts it together as a task for the existing episode runner. Settings: `MINK_CFG`, `PICK_PLACE_CFG` in `src/task_config.py`. mink 1.3.0, qpsolvers 4.13.0 and daqp 0.10.3 are installed and pinned in `requirements.txt`.

## What it took (three things that were not in the plan)
1. **A simple sag correction was needed**, as you allowed if the grasp missed: the first run stopped at UP, 18 mm short, because the simulator's gravity compensation does not carry the beaker's 0.2 kg. An integral on the measured pad pose error (forward kinematics of the measured joints) is added to the waypoint path (3 per second, at most 4 cm and 0.1 rad); it is reset when the gripper opens. Nothing else of the dropped list (bridge, contact audit, pre-flight) was added.
2. **The retreat after the release hit the right hand at home** (it stopped at 39.9 mm, the 40 mm limit between the arms). Now the hand backs out 7 cm along the approach direction (enough to clear the beaker), then goes home diagonally, and the arm-to-arm minimum is 25 mm. This is a real tightness: the hotplate is 18 cm from the right hand's home position. It will matter for the demo generator and for any other hotplate position.
3. **The recorder does not clear old chunk files** in a kept recording directory: a shorter run after a longer one replays extra ticks and "mismatches" at the first stale tick. The test now clears its directory. (Not changed in the recorder.)

## Decisions made on the way (changeable)
- The 20 cm clearance height worked at the finder's stance (a kinematic check of every waypoint first, then the run): no fallback to 12 cm was needed.
- The approach direction is the robot's forward direction; grasp height is the middle of the band (52.5 %); the descent is at a 12 cm standoff, then 12 cm in at 4 cm/s.
- The pick ends with a lift check (the beaker rose at least 3 cm with the hand), the cheapest "dropped" detector; not in your list, a few lines.

## Not done (recorded as LATER in design plan section 9)
The command bridge (the arm speed is limited to 0.5 rad/s by mink's velocity limit, not by a bridge), contact audit, pre-flight, the other forced-failure and variation tests, the LLM planner, the VLM checker, panel polish. The full test suite was not rerun after these changes (only `src/task_config.py` got new blocks; nothing existing was edited except `experiments/run_tests.sh`, which now also runs the new test, and `requirements.txt`).

## Next
Your call. Natural next steps: run the full suite once (about an hour in tmux), then either the LATER items you want first (contact audit and a failure test for each stop path) or the LLM planner behind the same plan format.
