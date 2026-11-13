# Path divergence at OVER: where the arm left the check's solution, and why (read-only, 2026-11-10)

Written for: David. Read-only: no change to src/, experiments/ or config. Scripts: `scratch/over_miss/` (`over_miss_replay.py`, `trace_run.py`, `trajectory_replay.py`). Data: `data/diag/over_trajectory_20261110.json` (per control tick), `data/diag/over_miss_staging_20261110.json` (earlier). Planner summary: `docs/reports/over_path_planner_20261110.md`.

## Answer
The arm stops being on the check's solution at **the OVER waypoint itself, from about 72.5 s (run time)**: the hand had come to 29 mm of the waypoint (its closest point, at 72.6 s), then joints 1 and 3 swung in opposite directions (joint 1 by about -66 deg, joint 3 by about +90 deg between 72.5 and 75.5 s), and the hand went back out to 52 mm. The component is **the planner's per-tick IK** (mink, a differential solve from the current joints toward a straight pad line), which chose a different swivel of the arm's redundant shoulder and elbow. The sag trim, the command bridge and the elbow guard are ruled out (see step 4).

## Step 2: the executed trajectory (verified replay, every control tick)
`data/diag/over_trajectory_20261110.json`: 4623 ticks at 150 Hz from 68.49 s to 99.31 s run time. Each tick has the frame, time, the active skill label (pick: OVER for the whole window; the move never ended), the measured joints, the recorded controller command, the hand (pad) position from forward kinematics, the distance to the waypoint, and the distance to the bench top.

The waypoint is (1.114, 0.698, 1.160) m. The hand's distance to it over the move:
| time (run s) | error (mm) | command minus measured, joints 1 to 3 (deg) |
|---|---|---|
| 68.5 | 88.6 | -0.9, -0.9, -0.8 |
| 71.0 | 51.6 | -2.0, -3.2, -2.1 |
| 72.6 | **29.2 (closest)** | -3.3, -5.0, -2.0 |
| 73.0 | 30.6 | -3.6, -5.7, -2.0 |
| 74.0 | 49.1 | -4.5, -4.1, -2.0 |
| 75.0 | 52.1 | -4.7, -3.0, -1.9 |
| 99.3 | 51.8 (given up) | -3.6, -2.6, -2.9 |

Joint deviation from the check's solution (degrees, joints 1 to 7) at the same times: at 73.0 s [17, 1, -19, 8, -5, 0.5, 12]; at 75.0 s [-31, -2, 37, 5, -2, 0.5, 0]; at 80 s and after [-38, -1, 45, 4, -2, 0.5, -2]. So after 75 s, joints 2, 4, 5, 6 and 7 are within about 4 deg of the check's solution, and joints 1 and 3 hold a swivel 38 and 45 deg away from it.

## Step 3: the check's solution, waypoint by waypoint
The check does not produce a trajectory. It solves the waypoint once from the zombie pose, and that solution (its joints) is 1.1 mm from the waypoint. For this move there is one waypoint, OVER, so the comparison is: the path's start and end against that solution.
- The move starts at the zombie-like pose (joints within 15 deg of home), which is far from the check's solution (88.6 mm off), as expected for a move. The path reduces the error to 29 mm by 72.6 s, along a path that the check did not compute.
- Divergence in configuration starts at 72.5 s: joint 1 has gone 25 deg from the check's value and joint 3 about 45 deg. Joint 3 had already drifted from 35 deg (check) to -11 deg by 72 s, so the arm passed through a configuration the check does not use before it swung to the other branch (+80 deg).
- Endpoint: replacing joints 1 and 3 with the check's values in the executed configuration reduces the FK error to the waypoint from 48.4 mm to 24.4 mm. So the swivel accounts for about half the endpoint error; the rest is the 2 to 4 deg offsets of joints 2, 4, 5, 6 and 7 together.
- The check's own FK agrees with the simulator to within 4.7 mm at the executed joints (earlier report). So the kinematic model is not the cause.

## Step 4: rule out causes
- **Planner.** The per-tick IK is the cause of the swivel. Its costs: frame task (position 1.0, orientation 0.5), a posture task toward home (0.01), joint limits with a 10 deg margin, collision avoidance (furniture 8 mm), and a 0.5 rad/s joint speed limit. The posture task pulls toward home (joint 1 about 23 deg, joint 3 about 16 deg), but the swing went the other way, so the posture term did not hold the swivel. The planner's own commands were not logged (see the caveat), so the swing's exact trigger is not resolved.
- **Sag trim.** Its correction is at most 0.04 m and 0.1 rad (config). The measured command-minus-measured difference is 2 to 5 deg on joints 1 to 3 at the failure, the servo lag under the arm's own weight, not the trim. The trim's own output is not in the replay (the replay re-simulates the recorded command), so its size during the swing is not measured. Not ruled out by data; the magnitude argument makes it an unlikely cause of a 40 deg swivel.
- **Command bridge.** On, but it logged zero events in this run (`[bridge] 0 log entries, 0 events`). It did not reject, clamp or suspend any command. Ruled out.
- **Elbow guard.** Off: `ELBOW_GUARD_CFG["enabled"] = False` (decision M278), so it does not clamp the targets. Ruled out.
- **Arm dynamics.** The servos track within 2 to 5 deg; the swing is 40 to 70 deg, far larger than the lag. Not the cause; it adds the 2 to 4 deg offsets to the final error.

## Caveat: the logging changed the run
A first live re-run with a logging wrapper on `MinkArm.step` (to log the planner's own command) did **not** reproduce the miss: the OVER dry run was 0.80 rad, not 2.07 rad, and the episode failed later, at the place. A second live run **without** the wrapper reproduced the original exactly (2.07 rad, 51.4 mm). So the wrapper changed the arm's behaviour (its cause was not found, and the wrapper was not kept). The planner's own pre-trim command therefore was not logged; the per-tick file records the recorded controller command, and the verified replay of the original recording is the source of the executed joints. The determinism of the rest of the pipeline is confirmed by the unwrapped run.

## Not done
1. The planner's own command per tick (before the sag trim), which would show the swing as the IK output directly. Needs a logging method that does not change the run (for example, recording the pre-trim command inside the recorder).
2. The sag trim's contribution during 72 to 75 s (the trim's C values, not in the replay).
3. Why the solver chose the swivel at 72.5 s (the planner's internal state at that tick).

## Decisions needed (David)
1. The check solves the OVER endpoint from zombie and does not model the path. Should the check also simulate the planner's path (the per-tick IK from the start pose) before accepting a stance?
2. Should the planner hold the swivel near the check's branch (for example a stronger posture term, or a penalty on joint 1 and 3 change), or should the skill accept the swivel and the check model it?
