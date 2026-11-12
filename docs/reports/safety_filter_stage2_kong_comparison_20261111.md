# Stage 2 safety filter: Kong's compiled filter against our resolver on a scripted rolling punch

Run: 2026-11-12, 05:06 (stamp 20261112_050659), full run of 10 punches per hand (1576 ticks). Code at commit 87d1a3c; the script `scripts/safety_filter_kong_comparison.py` is new and not committed. Nothing was committed or pushed.

Files:
- Script: `scripts/safety_filter_kong_comparison.py`
- Raw results: `data/safety_filter/kong_comparison_20261112_050659.json`
- Offset scan (40 candidates): `data/safety_filter/kong_scan_20261112_050659.json`
- Logs: `data/logs/20261112_050659_kong_comparison.log` (runtime 3 min 13 s wall, script 191.9 s, peak RSS 2.2 GB), `data/logs/20261112_050659_kong_scan.log` (3 min 39 s wall)

## Result

Kong's wheel imported in the Linux VM, so all four configurations ran. Required gap per row: rows 1 to 3 use Kong's capsule radii, row 4 uses our shapes. Distances in mm, angles in degrees, time per filter call in ms.

| configuration | required gap | min gap Kong radii (mm) | min gap our shapes (mm) | min gap our shapes, shoulder-mount pair left out (mm) | hold ticks (of 1576) | hand position error vs raw, mean / max (mm) | hand orientation error vs raw, max (deg) | max joint jump per tick (rad) | filter time per tick, mean / max (ms) |
|---|---|---|---|---|---|---|---|---|---|
| 1. no filter (output = raw) | Kong radii | -78.16 | -56.95 | -56.95 | n/a | 0.0 / 0.0 | 0.0 | 0.018 | n/a |
| 2. Kong compiled filter, streamed | Kong radii | +3.41 | -14.22 | -7.20 | n/a | 50.0 / 104.4 | 43.2 | 0.201 | 0.198 / 3.44 |
| 3. our resolver, paper_plain (gap rule off), Kong radii, margins 0.0 | Kong radii | -0.50 | -14.22 | +8.53 | 1387 (88%) | 69.7 / 165.7 | 8.1 | 0.418 | 8.75 / 79.6 |
| 4. our resolver, full gap rule ("ours"), own shapes and margins | our shapes | +54.14 | -14.22 | +63.42 | 1529 (97%) | 72.2 / 165.9 | 8.1 | 0.326 | 96.7 / 260.3 |

Where the smallest gap occurs: row 1 Kong gap at tick 1469 (left and right kong_hand); row 2 at tick 1463; row 3 at tick 85 (-0.50 mm, the resolver's 0.5 mm stop tolerance); row 4 at tick 28. The our-shapes minimum of -14.22 mm is at tick 0 in every row: the left and right shoulder mounts of our model overlap by 14 mm at HOME, so that column as specified is set by one pair in every run. The fourth gap column leaves that pair out and is added for that reason.

## What the numbers mean

In this stream the raw command itself is in collision on 1494 of 1576 ticks (Kong radii): the punching hand ends 5 cm past the midline, about 8 cm from the other hand's rest point at the same x and z, and two 8 cm hand capsules need 16 cm. Kong's compiled filter keeps its own capsules apart (smallest gap +3.4 mm) by moving the hands: on average 50 mm, and up to 104 mm, from the raw command, with the wrist up to 43 deg off and one-tick joint jumps up to 0.20 rad. Our resolver mostly holds still instead: in row 3 its 189 non-hold ticks are 82 PASS and 107 HAND_SLIDE. With the gap rule off and Kong's radii it holds on 1387 of 1576 ticks (88%); its output lags the raw command by 70 mm on average (166 mm max), and its smallest Kong-radii gap is -0.5 mm, i.e. within its own 0.5 mm stop tolerance. With the full gap rule and its own shapes it holds on 1529 ticks (97%), with a 72 mm mean lag, and its Kong-radii gap stays at or above +54 mm. So on this stream our resolver follows the command worse than Kong does and holds a lot; most holds are the 2 mm accuracy rule (1033 of 1387 holds in row 3, 1002 of 1529 in row 4). Its filter time is 8.8 ms mean (80 ms max) in row 3 and 97 ms mean (260 ms max) in row 4, against a 6.7 ms tick and Kong's 0.2 ms, so the time budget is not met either. Its largest one-tick joint jumps (0.42 rad and 0.33 rad) are above Kong's 0.20 rad; I did not find their cause. The comparison reflects how each filter responds to commands that are infeasible under this stream; it does not show how either filter would do on a real punch (see the first open choice below).

## Stream definition

- Rate and length: 150 ticks per second; 1576 ticks, from t = 0 to t = 10.5 s inclusive (t = k / 150).
- Timing: left hand punches start at t = 0, 1, ..., 9 s; right hand at t = 0.5, 1.5, ..., 9.5 s. Each punch is 0.5 s out and 0.5 s back (period 1.0 s), 10 punches per hand. The other hand rests at its start point between its punches.
- Path: straight line in robot base frame, from the hand's HOME point (left (0.1162, 0.1318, 0.8445) m, right (0.1162, -0.1318, 0.8445) m) to a point 5 cm past the midline with the same x and z: left (0.1162, -0.05, 0.8445) m, right (0.1162, +0.05, 0.8445) m.
- Timing law: min-jerk, s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5, with tau = time within the 0.5 s half-period.
- Orientation: the HOME hand orientation, held for the whole stream.
- Raw command: joint targets from repeated single Jacobian steps of our model, up to 30 per arm per tick, from the previous tick's joints, stopping when no joint moves by more than 1e-9 rad. The stream needs 4.57 steps per arm per tick on average (max 5); no tick hit the cap.
- Joint limits: the path as specified violates them (see the first open choice). The raw command used here is the IK trajectory clipped to the joint limits.

## Status and hold-reason counts (ticks)

| configuration | PASS | ELBOW_SWING | HAND_SLIDE | STEP_SHORTENED | HOLD | hold reasons |
|---|---|---|---|---|---|---|
| 3. paper_plain, Kong radii | 82 | 0 | 107 | 0 | 1387 | ACCURACY 1033, BRANCH_GUARD 330, SHORTEN_ZERO 24 |
| 4. full gap rule, own shapes | 33 | 13 | 0 | 1 | 1529 | ACCURACY 1002, BRANCH_GUARD 326, SHORTEN_ZERO 136, JOINT_LIMITS 65 |

Rows 1 and 2 have no statuses (n/a). Reason names are as the resolver reports them; the meaning of JOINT_LIMITS was not checked.

## Deviations and open choices

1. Joint limits (open, David's call). The straight line at constant x and z, with the HOME orientation held, needs joint 2 of each arm past its upper limit at the midline: the IK trajectory reaches +0.315 rad on the left arm against the +0.1745 rad limit. Before clipping, 1024 of the 3152 arm-ticks (2 arms x 1576) are outside a limit (joint 2 by up to 0.141 rad, joint 6 by up to 0.024 rad).
2. Offset search (--scan). The whole path was shifted in x (dx = -0.15 to +0.20 m) and z (dz = -0.10 to +0.10 m), 40 candidates. None clears the limits. The fewest arm-ticks outside a limit is 856, at dx = -0.15, dz = -0.05 (joint 2 still beyond its limit by up to 0.092 rad there), against 1024 at zero offset. The start pose moves up to 1.49 rad from HOME across the grid. Used: dx = dz = 0, so the stream starts at HOME as specified.
3. Clipping as the last resort (open). The IK trajectory is clipped to the joint limits (--clip final). This removes all violations, but the hands then deviate from the straight line by 7.2 mm on average (33.9 mm max), and the orientation is up to 8.1 deg off HOME, so the hands are not level in those sections. This breaks the level-hand rule of the spec and is reported as such. Two scratch checks (not in the repo) for reference: clipping inside every IK step gave 4.3 mm mean, 21.3 mm max, 7.5 deg, and hit the 30-step cap on some ticks; the simplest variant (clip the trajectory) was used.
4. Elbow-swivel alternative (not tried). A scratch check at the midline point with joint 2 held at its limit reached the point exactly, but joint 6 then sat at 0.97 rad (limit 0.785). Whether a limit-respecting choice of the 7th degree of freedom exists along the whole path was not searched. This is a design choice for David, not a fix made here.
5. Stream in collision by construction (open). Raw Kong-radii gap is negative on 1494 of 1576 ticks, first at tick 41 (t = 0.27 s); the raw minimum is -78.16 mm. The stream was built as specified; it means that most of each run is a response to infeasible raw commands. Whether the stream should keep the hands apart is a question for David.
6. Config 4 holds (scratch trace, first 160 ticks). The first hold is at tick 29 (t = 0.19 s, reason JOINT_LIMITS); from tick 48 the reason is mostly ACCURACY; the hand error from the raw command grows from 4 mm at tick 29 to 53 mm at tick 40 and 164 mm at tick 152. Only 47 of 1576 ticks are not holds in this run. Clipping is not the cause: a 200-tick check without clipping (--clip none, scratch) gave 83 holds of 200 in row 3 and 171 of 200 in row 4.
7. Kong's capsules are placed on our model. S, E and W come from our model (ArmKinematics.sew_now); T = W + R7 times the tool vector. The tool vector (0, 0, 0.1) m in link 7 was taken from Kong's solver at HOME. At HOME Kong's solver points equal ours up to a constant z offset of 1.0028 m. The filter itself works on the solver's geometry, so its output can differ from our capsules: in row 2 the output is 7.2 mm inside our shapes (shoulder pair left out, tick 1463).
8. KongShapes choices not given in the spec: segment names upper_arm, forearm, hand (the names the resolver's shapes use); body frames openarm_<side>_link2, link4, link7; largest radius set to 0.08 m (Kong's hand radius) for the resolver's broad phase.
9. Row 3 margins: all three cfg.MARGINS entries are 0.0 during the run and restored afterwards. cfg.PUSH_START_BEYOND_M (0.01 m) and cfg.PUSH_STOP_BEYOND_M (0.02 m) were left at their values; their effect with margins at 0 was not checked.
10. Our-shapes column: the shoulder-mount pair overlaps by 14.2 mm at HOME in every run, so the specified column is -14.22 mm everywhere. The extra column leaves that pair out; it is not a touch rule applied by the script.
11. Timing covers only the filter call (sf.filter or res.step) in one process on the VM. Row 4 is far above the 6.7 ms budget and was not optimised.
12. The first-tick joint jump is measured against HOME. Single run; no repeat and no variance estimate.
13. Not done, as instructed: no commit, no push, no edit of docs/progress.md or any existing file. The project's crash rules would normally ask for a progress entry and a commit.
