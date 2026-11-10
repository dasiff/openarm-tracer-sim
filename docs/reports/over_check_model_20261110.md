# OVER miss, staging layout: is the check too optimistic, or is the stance at margin? (2026-11-10)

Written for: David. Read-only: no change to src/, experiments/ or config. Scripts: `scratch/over_miss/over_miss_replay.py`. Data: `data/diag/over_miss_staging_20261110.json` (git-ignored). Log: `data/logs/over_miss_20261110_114021.log`.

## Which run
The brief named seed 2986646089 for the failing staging run. That seed is the earlier pinned run, which **succeeded**. The run that missed OVER is `edge_staging_20261110_111521` (seed 2680074893, selected draw 2: x 0.9860, y 1.0704, heading -101.8 deg, pedestal 0.1056 m). That run is used here.

Times: the log's times are run time. The simulator clock starts at 1.4013 s (the replay's first step is at sim 1.40 s and its last at 119.66 s, the same 118.3 s run). OVER started at run 68.49 s (sim 69.89 s) and was given up at run 99.31 s (sim 100.71 s).

## Step 1: the full check's OVER model (src/full_check.py, src/task_points.py, src/task_config.py)
The brief asked for src/stance_passes.py. That file has the two-pass verdict (`classify`, `evaluate_cell`), which only runs for the old overlay path. The pipeline's full check does not call it. The OVER test of the edge pipeline is in `FullCheck._solve_pose_point` and `FullCheck._admissible`, and the waypoint geometry is in `task_points.pick_waypoints`.

What the check tests for the OVER point:
1. **Target.** `pick_waypoints`: the OVER point is at the standoff point (the grasp point moved `standoff_m` = 0.12 m back along the approach direction for approach bin k), at height `surface + clearance_m` = 0.20 m above the bench top (PICK_PLACE_CFG). The approach bin k is one of 12 (30 deg each). The check tries all bins.
2. **IK.** `solve_ik` from the zombie pose for the first stop of a path (later stops start from the previous stop's answer), with `pose_ik_iterations` = 150, damping and step limits from the arm config. The solution must reach the target within `tol_m` = 3 mm and `tol_rad` = 2 deg (PICK_PLACE_CFG). Random restarts run only as a logged diagnostic.
3. **Admissibility** (`_admissible`): no arm collision with the robot itself; the hand and fingers at least `hand_clearance` = 10 mm from the bench, the hotplate and the other objects (`clear.arm_distances`); the other arm parts at least `arm_clearance` = 0.0 m from the furniture and the objects; and, against the previous stop, no joint change above `ik_flip_rad` = 1.0 rad. The first stop has no previous stop, so the flip rule does not apply to OVER.
4. Other margins that exist but do not gate OVER: `world_margin_m` 0.03 m (the world-model clearance used by the old two-pass slow pass, PROVISIONAL), `home_min_clearance` 40 mm (the arms at home), `body_clearance` 0 m (the body check).

So the check's OVER margins are small: 10 mm for the hand and fingers, 0 m for the arm, and a 3 mm / 2 deg IK tolerance. It checks the end pose of each stop, not the path from the current joints to it.

## Step 2: replay at the OVER move (data/diag/over_miss_staging_20261110.json)
The recording was re-simulated with `replay_run` (video none). The replay verified: `ok` True, no mismatch, 17739 ticks, the same as the run.

- **Replay matches the log.** At the failure, the pad is 51.8 mm from the matched waypoint (the log: 51.4 mm). The matched bin is k = 10 (waypoint 0.20 m above the bench top). The other bins are 28 mm (k = 11) to 220 mm away, so the match is unambiguous.
- **Beaker.** At OVER start and at failure: position (1.000, 0.660, 0.960) m, tilt 0.03 deg. The beaker did not move.
- **Pad (hand centre) at the failure:** (1.073, 0.727, 1.146) m. The error to the waypoint is (-40.7, +28.8, -13.9) mm: mostly along -x (back towards the base), then +y and down. The pad reached its stop by about 76 s (run time) and stayed there for the remaining 20 s.
- **Joints.** Measured q (rad): -0.928, -1.391, 1.395, 1.655, -1.043, -0.601, -0.579. The controller's command minus the measured joint (deg): -3.6, -2.6, -2.9, +0.6, -1.9, -0.3, -2.6. So the servos are 2 to 4 deg short of their commands on joints 1, 2, 3, 5 and 7. Joint 3 is 0.18 rad from its limit and joint 6 is 0.18 rad from its limit at the failure.
- **Bench distances from the pad:** to the bench top box 0.186 m; the pad is 0.186 m above the top. Closest points on the top edges: front (+y) edge 0.200 m (closest point (1.073, 0.80, 0.96)); back (-y) 1.538 m; +x 1.142 m; -x 3.279 m. The front face of the bench is 0.073 m behind the pad's y.
- **Contacts.** The left arm's contacts with the bench, the hotplate and the beaker: none. No arm geom within 0.25 m of any bench or hotplate geom (the nearest-distance list is empty). The beaker's geoms are part of the contact list only, and it has no contact.

## Step 3: what the check would have said for the same waypoint
- **Check's IK for the matched waypoint** (from zombie, 150 iterations): error 1.08 mm, within the 3 mm tolerance, so the check says the OVER pose is reachable and passes.
- **Check's FK of the executed joints versus the simulator's pad:** 4.7 mm apart. The check's kinematics agree with the simulator to within 5 mm at these joints.
- **Joints:** the check's solution (rad): -0.256, -1.370, 0.612, 1.586, -1.010, -0.611, -0.548. The executed joints differ by -38.5, -1.2, +44.9, +3.9, -1.9, +0.5, -1.8 deg. Joints 1 and 3 are the big differences: the executed arm is in a different elbow and shoulder configuration from the one the check solved for.
- **Clearance:** the check's admissibility passes (no arm contact, and the hand is 0.2 m from the bench at the check's solution). The replay shows the executed pose is also clear (no contact, 0.2 m from the bench). So clearance is not the reason for the miss.

## Verdict
The check is **not too optimistic about geometry**: its kinematics match the simulator within 5 mm, and the stance is **not at margin**: the pad is 0.2 m from the bench's front edge with no contact. The check **is optimistic about reachability in execution**: it tests the end pose from zombie, but the execution's solver reached the same area through another joint configuration (joints 1 and 3 about 40 to 45 deg from the check's solution), stopped 52 mm short, and the servos were 2 to 4 deg off their commands. The check has no model of that path, and its flip rule does not apply to the first stop.

## Not done / open
1. The replay shows the configuration difference but not why the executed solver took that branch. That would need the controller's per-tick IK targets (not in the recording).
2. The check's own clearance for OVER (the 10 mm hand margin) was not reproduced with the world-model boxes; the replay used mujoco distances to the scene geoms.
3. The brief's seed (2986646089) was not used (see top).

## Decisions needed (David)
1. Should the check predict the executed configuration for the first stop of a path (for example by solving from the arm's current joints and applying the flip rule to that), or is that a controller problem to fix in the executor?
2. The OVER miss is a reachability-in-execution issue, not a margin issue. Is the 0.2 m clearance over the bench (`clearance_m`) the right height for these stances?
