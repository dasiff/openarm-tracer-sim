# Moved layout, candidate 1: the first dry-run failure (read-only, 2026-11-10)

Written for: David. Read-only: no code in src/ or experiments/ changed. Scratch analysis: `scratch/over_miss/moved_c1_diag.py` (an instrumented copy of the dry-run's move loop; no physics stepping). Logs: `data/logs/moved_c1_diag.log` (as the code was), `data/logs/moved_c1_diag_corrected.log` (scratch-only correction, see below). Data: `data/diag/moved_candidate1_dryrun_trace.json`. Picture: `data/diag/moved_candidate1_dryrun_fail_topdown.png`.

## Premise correction
Candidate 1 (seed 3342821395) did pass the full check (37 s in the run; 54 s in this rerun). It is the first candidate that reached the dry-run, and that failed it. The run's log line: `[dryrun] candidate 1: FAIL (18.59 s) place OVER: not reached after 21.5 s (52.2 mm from the waypoint)`.

Also a correction to `trajectory_dryrun_gate_20261110.md`: place OVER failures in the moved run were candidates 1 (52 mm), 9 (53 mm), 16 (332 mm), 25 (488 mm), 38 (678 mm), 39 (411 mm). The table there listed only five of them. The three pick failures were candidates 14 (pick OVER, 3.7 mm short), 21 (pick IN, 4.9 mm short) and 28 (pick IN, 3.6 mm short).

## 1. Failure mode
Place OVER: not reached before the 21.5 s timeout, 52.1 mm from the waypoint.

## 2. Candidate stance
x 0.7877 m, y 1.1272 m, heading −109.4 deg, pedestal 0.1711 m, A9 score 3.445.

## 3. Targets (moved layout: beaker at (0.85, 0.66), hotplate at (0.65, 0.66))
| subtask | target (x, y, z) m | result |
|---|---|---|
| pick OVER | (0.8899, 0.7732, 1.1600) | passed (1.5 s) |
| pick DOWN | (0.8899, 0.7732, 1.0125) | passed (3.7 s) |
| pick IN | (0.8500, 0.6600, 1.0125) | passed (3.0 s) |
| pick UP | (0.8500, 0.6600, 1.1205) | passed (2.7 s) |
| place OVER | (0.6505, 0.6604, 1.1209) | **timeout, 52.1 mm** |

The place OVER waypoint is 0.487 m horizontally from the stance centre, at height 1.121 m.

## 4. The last 50 ticks of the failing subtask
The commanded pad converged towards the target, reaching 42.0 mm at t = 3.61 s. It then stopped. From t = 21.19 s to 21.52 s (the last 50 ticks):
- the commanded pad is fixed at (0.645, 0.711, 1.133) m;
- the reference is fixed at the target (0.651, 0.660, 1.121) m, so the reference has finished its move;
- the distance to the target is 52.1 mm and the distance to the reference is 52.1 mm;
- joint speed is zero.

The error is almost all in y (about +51 mm), with 12 mm in z and 5 mm in x. The trend is stuck, not converging or diverging.

## 5. Reachability, compared with the full check
The full check passed this stance, so its place stops were solvable within its tolerance for the chosen hand turn. The place window in the check's own output is −35 to −10 deg, and the chosen turn is −20 deg (`place_info`). The check records no numeric reach distance, so there is no limit in metres to compare the 0.487 m against.

## 6. The cause: a unit bug in the dry-run call
The dry-run call passes the place turn in the wrong unit (`src/stance_select.py`, `verify`, the `dry_run_stance` call). `choose_place_angle` already returns degrees (`src/approach_angle.py`), and `verify` converts it again:

    th = choose_place_angle(...)                       # -20 (degrees)
    dry_run_stance(..., math.degrees(th))              # passes -1145.9 deg

`dry_run_stance` then converts `-1145.9` to radians, so the place hand is rotated by a large, wrong angle, and the IK that holds the hand level cannot reach the place OVER waypoint.

Scratch check (`CORRECT_THETA=1`, the same candidate, with the call's value replaced by the list's −20 deg; no source change): the dry-run **passes**, in 59.6 s (`data/logs/moved_c1_diag_corrected.log`).

So for this candidate the failure is the unit bug, not reachability and not the planner.

## 7. Scope of the bug
The bug affects every dry-run whose place angle is not 0. That includes all dry-runs of both runs that reached the place: the moved run's nine dry-runs, and the staging runs. The dry-run results in `trajectory_dryrun_gate_20261110.md` (both layouts, all criterion versions) are therefore invalid as evidence about the gate. The "no candidate passed" result for the moved layout and the staging yield of candidate 5 are both unverified until the fix is in. The pick subtasks do not use the place turn, so their results stand.

## Proposed fix (not made: the brief asked for no code change)
In `src/stance_select.py` `verify`: pass `place_theta_deg = th` (no `math.degrees`). Then re-run both layouts. The result will show whether the gate is working as intended, and any remaining failures need their own look.

## Decisions needed (David)
1. Approve the one-line fix above, and the re-run of both layouts.
2. The pick IN failures (3.6 to 4.9 mm short after 15 s) and the pick OVER failure (3.7 mm short) are a separate issue. They do not involve the place turn. Should they be looked at after the fix, or only if they recur?
