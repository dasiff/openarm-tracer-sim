# Stage 2e: does the elbow snap near straight (joint 4 below 0.242 rad) explain the holds?

Date 2026-11-12. Branch safety-filter. Addendum to docs/reports/safety_filter_stage2d_solver_jumps_20261112.md. Measurement only: nothing in src/safety_filter/ and no config changed, no fix chosen, nothing pushed.
Scripts: scripts/safety_filter_solver_jump_replay.py (extended: elbow bend, joints 3 and 5, wrist error, a `--joint` selector), scripts/safety_filter_elbow_snap_sweep.py (new).
Results (git-ignored): data/safety_filter/solver_jump_replay.json (first 20 holds per row), data/safety_filter/solver_jump_replay_joint6.json (joint 6 holds), data/safety_filter/elbow_snap_sweep.json. Logs: data/logs/20261112_173800_s2e_replay_full.log, data/logs/20261112_174200_s2e_replay_j6_full.log, data/logs/20261112_173958_s2e_snap_full.log.

**Answer in one paragraph.** The 0.242 rad elbow threshold is real in the solver: when the commanded elbow is nearly straight, the solver's answer for the hand pose starts to drift at exactly 0.242 rad (10 of 12 posture-arm cases), and in 5 of 12 it snaps, with joints 3 and 5 moving 0.45 to 2.6 rad while their sum hardly changes. But none of the filter's holds come from that zone. In the full stream the elbow of every held command stayed at 2.04 rad or more (1530 holds in row 6, 1529 in row 4), and none of the 70 replayed jumps had a solver input bend below 0.75 rad. So the threshold explains neither the joint 3 and 5 jumps nor the joint 6 jumps in the filter. The joint 6 holds (1461 of row 6's solver holds) are a different cause: the solver returns joint 6 past its lower limit (−0.785 rad) after a very large push.

## 1. Replay split by elbow bend (rows 4 and 6)

Bend = angle between the solver's upper-arm and forearm directions. It equals joint 4 exactly at the last filtered command (max difference 0.0 rad in all cases), so it is the elbow bend. "Snap" zone = bend below 0.242 rad. Every replayed case is the left arm.

| set | row | cases | bend of solver input (rad) | joint 4 of last filtered (rad) | bend below 0.242 | max \|sum j3+j5 change\| (rad) | max \|d j3\| / \|d j5\| (rad) | joints 3 and 5 jump with sum almost unchanged | wrist err of solver output (mm) | cause counts |
|---|---|---|---|---|---|---|---|---|---|---|
| first 20 jumps | 6 | 20 | 1.127 to 2.055 | 2.163 | 0 of 20 | 1.137 | 0.57 / 0.68 | 0 of 20 | 0.0 | (e) 2, other 14, (c) 4 |
| first 20 jumps | 4 | 20 | 1.741 to 1.937 | 2.143 | 0 of 20 | 1.037 | 0.57 / 0.47 | 0 of 20 | 0.0 | (d) 20 |
| joint 6 holds, first 20 | 6 | 20 | 0.753 to 0.847 | 2.163 | 0 of 20 | 1.352 | 1.35 / 0.29 | 0 of 20 | 0.0 | (d) 20 |
| joint 6 holds, first 10 (all) | 4 | 10 | 1.857 to 2.156 | 2.143 to 2.201 | 0 of 10 | 0.837 | 0.46 / 0.38 | 0 of 10 | 0.0 | (d) 7, (e) 3 |

What it means: the replayed cases sit far from the snap zone. Their elbow is bent 0.75 rad or more in the solver input and 2.1 rad or more at the last filtered command. The "joint 3 and 5 with sum unchanged" signature (the snap) is absent in all 70 cases. Joint 3 and 5 moves are large in some cases (up to 1.35 rad), but their sums change too, so this is not the straight-elbow snap.

Full stream (no replay; joint 4 of the last filtered command over all holds, either arm):

| row | holds | holds with an arm's last joint 4 below 0.242 | min / max joint 4 of the last filtered command (rad) |
|---|---|---|---|
| 6 | 1530 | 0 | 2.044 / 2.201 |
| 4 | 1529 | 0 | 2.143 / 2.201 |

What it means: no hold in either row starts from a near-straight elbow. This covers every hold, but only for the last filtered command. For the pushed solver input, the bend was measured only in the 70 replayed cases (all at least 0.75 rad); the other holds' pushed bends were not replayed.

## 2. Snap sweep: commanded joint 4 from 0.6 rad down to 0 (other joints fixed, other arm at home)

Round trip per step: the `round_trip()` of scripts/safety_filter_solver_roundtrip.py (solver input = directions and hand orientation of our model at the commanded pose, no push). 301 steps of 0.002 rad. Onset = first step where the output hand pose is more than 2 mm or 0.5 deg off. Snap = first step where a joint of the output moves more than 0.25 rad from the command. Posture families: see section 5.

| posture | arm | hand direction, base frame (x fwd, y left, z up) | direction cos | onset (first wrong, joint 4) | wrong band (rad) | at onset: max change (rad) | snap (first step > 0.25 rad) | at snap: j3 / j5 change (rad) | at snap: j3+j5 before > after | at snap: max \|d j6\| in sweep (rad) | wrist err at snap (mm) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| home (zombie) | left | (+0.59, −0.11, −0.80) | n/a | 0.242 | 0.056 to 0.242 | 0.168 | none | n/a | n/a | 0.040 | 8.6 (hand) |
| home (zombie) | right | (+0.59, +0.11, −0.80) | n/a | 0.242 | 0.056 to 0.242 | 0.168 | none | n/a | n/a | 0.040 | 8.6 (hand) |
| reach forward | left | (+1.00, +0.01, −0.02) | 1.000 | 0.242 | 0.016 to 0.242 | 0.501 | 0.242 | −0.501 / +0.416 | 0.694 > 0.608 | 0.039 | 24.9 |
| reach forward | right | (+1.00, −0.03, +0.00) | 0.999 | 0.242 | 0.012 to 0.242 | 2.625 | 0.242 | −2.599 / +2.625 | −0.442 > −0.415 | 0.057 | 26.8 |
| forward and down | left | (+0.68, −0.00, −0.73) | 0.999 | none | none | none | none | n/a | n/a | 0.001 | 0.2 (max in sweep) |
| forward and down | right | (+0.74, +0.02, −0.67) | 0.998 | 0.242 | 0.054 to 0.242 | 0.224 | none | n/a | n/a | 0.045 | 11.0 (hand) |
| down to the side | left | (−0.01, +0.70, −0.71) | 1.000 | 0.242 | 0.012 to 0.242 | 1.021 | 0.242 | −1.021 / +1.008 | −0.477 > −0.490 | 0.206 | 44.4 |
| down to the side | right | (−0.01, −0.72, −0.70) | 1.000 | 0.242 | 0.074 to 0.242 | 0.130 | none | n/a | n/a | 0.031 | 6.7 (hand) |
| up | left | (−0.00, +0.01, +1.00) | 1.000 | 0.242 | 0.012 to 0.242 | 0.970 | 0.242 | +0.970 / −0.956 | 2.075 > 2.089 | 0.198 | 42.9 |
| up | right | (−0.01, −0.01, +1.00) | 1.000 | 0.242 | 0.194 to 0.242 | 0.052 | none | n/a | n/a | 0.012 | 2.7 (hand) |
| across the body | left | (+0.45, −0.62, +0.64) | 0.617 (weak) | none | none | none | none | n/a | n/a | 0.001 | 0.2 (max in sweep) |
| across the body | right | (−0.25, +0.61, −0.75) | 0.613 (weak) | 0.242 | 0.036 to 0.242 | 0.453 | 0.242 | −0.453 / +0.446 | 0.728 > 0.722 | 0.102 | 22.7 |

Where the snap occurs, the last step without it is 0.244 and the first snap or wrong step is 0.242, so the true point lies in (0.242, 0.244]. Within the 0.002 rad grid this matches the 0.242 value.

What it means: the threshold is real for the solver. In 10 of the 12 posture-arm cases the hand pose goes wrong at joint 4 = 0.242 (within one grid step). In 5 of them the output jumps more than 0.25 rad there, joints 3 and 5 move in opposite directions by 0.45 to 2.6 rad, and their sum changes by at most 0.09 rad. That is the straight-elbow signature: only the sum of joints 3 and 5 is fixed when the elbow is straight. Wrist errors at the snap are 23 to 44 mm, which fits the earlier "up to about 5 cm" note. Joint 6 does not jump in the sweep: at most 0.21 rad, and below 0.25 rad at every step. The home pose drifts with no snap: at 0.242 the hand is 8.6 mm off with a 0.17 rad change (below the 0.25 rad snap limit). Two postures (forward and down on the left, across the body on the left) never go wrong in this range.

## 3. Does the threshold explain the holds? (the four questions)

- **Joint 3 and 5 jumps.** Yes for the solver in general: the straight-elbow snap moves joints 3 and 5 with an almost unchanged sum at 0.242 rad. No for the filter's holds: no held command or replayed jump had its elbow in that zone (sections 1 and 2 above).
- **Joint 6 jumps.** No. The threshold moves joint 6 at most 0.21 rad in the sweep, and no joint 6 hold had a near-straight elbow. The joint 6 holds are: the solver returns joint 6 past its lower limit (−0.785 rad) after a very large push. Example: row 6 tick 48, joint 6 0.553 → −0.822 rad (a 1.37 rad change, across zero), with a 195 mm elbow push and 480 mm wrist push; cause (d) in all 20 replayed cases. Row 4's joint 6 holds are (d) in 7 of 10 cases and (e) in 3.
- **Both.** No, neither, in the stream. The threshold is a real property of the solver, but the filter never commands the arm into that zone (its lowest held joint 4 was 2.04 rad).
- **Verdict:** neither the joint 3 and 5 nor the joint 6 holds come from the 0.242 threshold.

## 4. Limits of this evidence

- The bend of the pushed solver input was measured in 70 replayed cases only. The 0.242 check of the other holds' pushed inputs is by bend of the last filtered command, which is 2.04 rad or more. A push could in principle bring some other hold's input into the zone; none of the 70 did.
- The first-20 replay sets are the earliest holds of each row. Row 6's first 20 are joints 5 and 4, not joint 6. The joint 6 set (20 cases) is the earliest joint 6 holds of row 6 (ticks 48 to 67). The 1461 joint 6 holds of row 6 and 241 joint 7 holds of row 4 are not replayed (see section 6, options).
- The family search reaches only a direction cosine of 0.61 to 0.62 for "across the body", so that family's labels are weak. The other families reach 0.998 to 1.000.
- The sweep holds the other six joints fixed; the snap point depends on them (the onset was 0.242 in 10 of 12 cases, so the dependence is weak here).

## 5. Method and choices

- Reused: `round_trip()` of scripts/safety_filter_solver_roundtrip.py (the step 2 test). Not reused: experiments/safety_filter_roundtrip.py (its frame() builds the solver input from the solver's own forward kinematics, not from the directions that solve_arm gives the solver) and experiments/branch_jump_demo.py (it runs MuJoCo physics and has no sweep function).
- The round trip itself was not re-run with an extra set; the sweep is a separate script that calls the same function. The round trip output (data/safety_filter/solver_roundtrip.*) is unchanged.
- The 0.242 rad value and "joints 3 and 5 jump about 1.5 rad" come from an Oct 5 note that is not in this repo. A search of the code and of docs/ for "0.242" finds only a joint value in src/task_config.py (setdown_q). The value is therefore tested here, not taken as given.
- Posture families: for each arm, 10000 random joint sets inside the limits (numpy seed 0; joint 4 fixed at 1.0 rad during the search), keeping those whose hand lies 0.25 to 0.50 m from the shoulder; the closest in direction to the family's target is used. Targets: reach forward (1,0,0); forward and down (1,0,−1); down to the side (0,s,−1); up (0,0,1); across the body (0,−s,0), with s = +1 left, −1 right. Home: fx.HOME.
- Onset: output hand pose more than 2 mm or 0.5 deg off (the round trip's HAND_POS_TOL and HAND_ORI_TOL). Snap: a joint of the output more than 0.25 rad from the command. Sum tolerance for "almost unchanged": 0.1 rad; joint-jump threshold for the replay split: 0.2 rad (my choices).
- Elbow bend: arccos of the dot product of the upper-arm and forearm directions; checked equal to joint 4 at the last command.

## 6. Options for David (none implemented)

1. Hold or shorten any step whose solver input bend is below a margin above 0.242 rad (e.g. 0.3): stops the straight-elbow snap by rule; costs a new rule and possible extra holds. No hold in the stream would be affected.
2. Clip the solver's joint 6 output to its limits before the branch guard: stops the joint 6 holds (cause d); costs a change to the converter and a decision that clipping is acceptable (the solver's own "limited" option is off).
3. Limit the push size per tick (the 195 to 480 mm pushes behind the joint 6 holds): removes the trigger of the joint 6 holds; costs a new rule and lag (Stage 2d option 2).
4. Replay the 1461 joint 6 holds of row 6 and the 241 joint 7 holds of row 4 in full (not only the first 20): would confirm the joint 6 cause for all of them; costs a run of a few minutes per row (the row 4 set runs to tick 1549).

## 7. Decisions needed

- Source of the 0.242 rad value and of the "about 1.5 rad" jumps (Oct 5 note): not in this repo. Confirm whether they stand; the measurements here match 0.242 at the 0.002 rad grid.
- Which of options 1 to 3 (if any) to build; they change the solver converter, the guard or the push.
- Whether the across-the-body family matters (its search fit is weak).
