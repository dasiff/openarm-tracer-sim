# Stage 2d: why Kong's solver output jumps more than 0.25 rad (diagnosis only)

Date 2026-11-12. Branch safety-filter. Measurement only: nothing in src/safety_filter/ and no config changed, no fix chosen, nothing pushed.
Scripts: scripts/safety_filter_solver_model_check.py (step 1), safety_filter_solver_roundtrip.py (step 2, written by a subagent, reviewed by the main session), safety_filter_solver_jump_replay.py (step 3).
Results (git-ignored): data/safety_filter/solver_model_check.json, solver_roundtrip.{json,npz}, solver_jump_replay.json. Logs: data/logs/20261112_17*_s2d_*.log.

## 1. What the solver is told about our arm

The solver is a compiled wheel. Compared from outside: its spec file (openarm_v1.npz), its own forward kinematics (robot_fk_at) and our MuJoCo model, after fitting one rigid transform between the two world frames (residuals below are the differences). 215 poses per arm (zero pose, each joint alone +-0.3 rad, 200 random).

| item | ours | solver | difference (left; right) |
|---|---|---|---|
| shoulder to elbow, zero pose | 218.0 mm | 220.0 mm | 2.0 mm; 2.0 mm |
| elbow to wrist, zero pose | 218.0 mm | 216.0 mm | 2.0 mm; 2.0 mm |
| wrist to hand reference point | 0 mm (link 7 origin = wrist) | not in the solver (reads directions only) | n/a |
| shoulder point S (all poses) | | | max 0.048 mm |
| elbow point E (all poses) | | | max 1.95 mm (left), 1.95 mm (right), constant over poses |
| wrist point W (all poses) | | | max 0.072 mm |
| joint axes, signs, zero positions (each joint moved alone) | | | wrist within 0.072 mm, link 7 within 0.002 deg, for all 7 joints, both arms |
| hand frame (link 7 orientation, solver R_0_7 vs ours) | | | constant 0.0054 deg, varies 0.011 deg over poses (left; right 0.011) |
| C_HAND | orthonormal, det +1 | | none |
| joint limits | | | 2e-10 rad |

Flags (all above the tolerances 0.1 mm / 0.01 deg): elbow point 1.95 mm both arms; link lengths 2 mm each, opposite signs (the sum is equal); hand orientation 0.011 deg. Joint axes, signs, zero positions and limits agree.
What it means: the two models are the same arm. The only real difference is where the solver puts the elbow point: 2 mm toward the shoulder in the upper arm (the solver shifts the elbow along the joint 4 axis, we use the intersection of the joint 3 and 5 axes). The solver is given directions only, so this changes nothing but a 2 mm length. The 0.011 deg is far below the 0.5 deg accuracy and the 0.25 rad guard. Not checkable from outside: the solver's internal IK settings; whether it applies its joint limits (its option "limited" is off, the outputs show it does not).

## 2. Round trip with no push

4634 cases: home 2 (1 per arm), rolling-punch stream every 5th tick (316 ticks x 2 arms), 2000 random poses inside the limits x 2 arms (seed 0, the other arm at home; poses saved in the npz). Tolerances: SAME 1e-6 rad; hand pose match 2 mm / 0.5 deg; jump 0.25 rad.

| set | arm | cases | SAME | SMALL DRIFT | OTHER SOLUTION | WRONG | output at a limit |
|---|---|---|---|---|---|---|---|
| home | left / right | 1 / 1 | 0 / 0 | 1 / 1 | 0 / 0 | 0 / 0 | 0 / 0 |
| stream | left / right | 316 / 316 | 0 / 0 | 316 / 316 | 0 / 0 | 0 / 0 | 48 / 48 |
| random | left | 2000 | 0 | 1786 | 13 | 201 | 0 |
| random | right | 2000 | 0 | 1825 | 6 | 169 | 0 |

Per-joint change (rad): home and stream: max 3.5e-6 on every joint (the solver is exact to about 3e-6, so SAME at 1e-6 never occurs). Random: mean change below 0.007 on every joint; max change joint 3 2.98 / 3.07 and joint 5 2.98 / 3.08 (left / right), joint 4 0.45 / 0.46, joints 6 and 7 about 0.22 to 0.25, joints 1 and 2 under 0.004. Hand error of the output on our model: home and stream max 0.001 mm and 0.000 deg; random max 48.8 mm (left) and 49.7 mm (right), orientation exact.
OTHER SOLUTION pattern "joint 6 sign flipped, joints 5 and 7 about pi": 0 of 19. The 19 changes are in joints 3 and 5 (0.26 to 1.61 rad).
Dependence on joint 6 (bins of 0.1 rad from -0.8 to 0.8): none; WRONG is 18 to 29 of 208 to 276 in every bin, with no peak near zero. The stream has joint 6 only between 0.4 and 0.8 (limit +-0.785), never near zero. What does matter is joint 4 (elbow): all 389 WRONG or OTHER SOLUTION cases have joint 4 below 0.25 rad (arm nearly straight; OTHER SOLUTION all below 0.02 rad). Only 13% of random poses are that straight, and nearly all of them fail; the home pose and the stream have joint 4 around 2.1 rad.
What it means: with no push the solver gives our poses back to 3e-6 rad on the home pose and on all 632 stream poses. So the solver and our arm agree on the stream (not cause a), and the solver does not take another solution there (not cause b). Where it does go wrong (random poses with an almost straight elbow) the shoulder, elbow and wrist line up, the upper arm and forearm directions are nearly parallel, joints 3 and 5 trade places and the hand position is off by up to 50 mm. That is the elbow-straight singularity, not the wrist singularity. The punch stream never comes near it (joint 4 about 2.1 rad).

## 3. Replay of the real jumps

First 20 BRANCH_GUARD holds seen at the solver output in each row, replayed from tick 0 with the recorded raw stream (the replay output equalled the Stage 2c trace at every tick). Full table with 13 columns per case: solver_jump_replay.json and the log (data/logs/20261112_173007_s2d_replay_full2.log). Summary (all cases are the left arm):

| row | ticks | solver change from last (rad) | raw command vs last (rad) | unpushed solver change (rad) | round trip from last keypoints (rad) | push elbow / wrist (mm) | direction change upper / fore (deg) | limit hit | cause counts |
|---|---|---|---|---|---|---|---|---|---|
| 6 kong_style, ours | 25 to 44 | 0.31 to 1.04 (joint 5 for ticks 25 to 37, joint 4 for 38 to 44) | 0.22 to 0.53 | equals raw vs last (0.22 to 0.53) | 0.000 | 79 to 182 / 101 to 390 | 21 to 49 / 8 to 65 | none | (e) 2, other 14, (c) 4 |
| 4 layered, ours | 46 to 66 | 0.35 to 0.57 (joint 3) | 0.44 to 0.48 | 0.44 to 0.48 | 0.000 | 81 to 101 / 0 | 21 to 26 / 22 to 27 | joint 6 in all 20 (0.784 before, 1.04 to 1.19 after; limit 0.785) | (d) 20 |

Causes found, one example each: (e) row 6 tick 25: raw is 0.218 rad from last, within the guard, but the pushed output moves joint 5 by 0.307 rad. other (raw command already too far from last) row 6 tick 27: even the unpushed raw conversion moves joint 5 by 0.252 rad. (c) row 6 tick 38: joint 6 0.553 to 0.118, near zero, a consequence of a pushed pose 45 degrees from raw. (d) row 4 tick 46: joint 6 output 1.04 against the limit 0.785 (last was at 0.784). Not found in any case: (a), (b), (b2); the round trip from the last filtered keypoints returned the last command to 0.000 rad in all 40 cases; flips and "joints 5 and 7 by about pi" in none.
What the pre-hold ticks of row 6 show (trace in the JSON): the filtered output follows raw until tick 15; from tick 16 the push grows (KONG_PUSH) and the output leaves raw (gap 0.001, 0.016 at tick 19, 0.085 at tick 23, 0.20 at tick 24). Joint 5 of the output changes 0.003, 0.019 and then 0.101 rad per tick at ticks 22 to 24 and the first-sweep solver output at tick 25 asks for another 0.31 rad, while raw moves only 0.016 rad per tick. From tick 27 on, raw alone is more than 0.25 rad from the held pose, so even an unpushed solver call fails the guard: the hold keeps itself going (Stage 2c lag). Row 4: the arm sits at its joint 6 limit; the push needs joint 6 beyond it, the solver (limits off) returns 1.04 to 1.19 and moves joint 3 by 0.35 to 0.57 rad to compensate, and the guard stops it. In row 4 this follows earlier holds of other kinds (ticks 43 to 45 are holds), so the stream was already stuck.
Method notes: the push is measured against the stream raw command with its own link lengths (the guard may have shortened the real command; not visible from outside). Cause rules are in the script docstring; thresholds (joint 6 near zero below 0.15 rad, pi within 0.3 rad) are my choices.

## Which cause explains the jumps

Not (a): zero-push round trip exact (3e-6 rad) on the home pose, all stream poses and the 40 replayed last poses. Not (b) and not (c) as the origin: no flip, joint 6 near zero only after the jump has started (4 of 40, row 6 ticks 38 to 41). Not the elbow singularity of section 2: joint 4 about 2.1 to 2.2 rad here. (d) explains all 20 row 4 cases: joint 6 is at its limit and the solver does not respect limits. In row 6 the jump starts as (e): a push that grows by 80 to 180 mm needs joint 5 to move 0.1 then 0.3 rad per tick, and the first-sweep solver output of the push is not continuous enough (0.003, 0.019, 0.101, 0.31 rad per tick on a smooth raw stream); afterwards 14 of 20 cases are held because the raw command itself is over 0.25 rad from the held pose. Evidence: strong against a solver/model mismatch, a flip or a singularity (many direct measurements); moderate for "push too large per tick" in row 6, since I did not log the push inputs of ticks before 25 and did not vary the push; strong for the joint-6-limit cause in row 4 (all 20 cases, same signature).

## Options for David (none implemented)

1. Let the converter pick the solution nearest the last pose, or clip the solver output to the joint limits before the branch guard: stops the row 4 limit jumps, costs a change to the converter and a decision on clipping.
2. Limit how far the push may move the keypoints per tick (a rate limit on the push, so it grows over several ticks): removes the row 6 trigger, costs a new rule and some lag.
3. Make the branch guard compare against the raw command's own distance (guard the change the push adds, not the total): stops the lock-in after a hold, costs weakening a safety check.
4. After a hold, aim the next raw command at the held pose (command-side catch-up, Stage 2c option 9): breaks the self-sustaining hold, costs an interface change.
5. Keep the arm out of its joint 6 limit near the zombie pose (a different home pose or a pushed-pose limit check before the solver): costs a decision on the pose or on limits.
6. Raise the 0.25 rad guard for the solver output only: needs a decision; it also lets real branch flips through.
7. Add a test that the solver output stays inside the limits and within a few mrad of the commanded pose for straight-elbow poses (joint 4 below 0.25): costs one test; a documented no-go zone, not used by the stream.

## Decisions needed / deviations

- Cause thresholds (joint 6 near zero below 0.15 rad; "about pi" within 0.3 rad; hand match 2 mm / 0.5 deg) are mine.
- Step 2 added a class SOLVER_ERROR (none occurred) and used the state seed = the test pose (solver_current_reset is on).
- The push of each replayed tick is shown against the stream raw command, not the guard-shortened command.
- Not built: logging of the push inputs of the ticks before the first hold; a test of varying the push size; separate runs with the solver seeded other than as the resolver does.
