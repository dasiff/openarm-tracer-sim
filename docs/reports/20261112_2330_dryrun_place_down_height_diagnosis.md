# The dry run's place-down height: cause found, fix proposed (not applied)

Date 2026-11-12. Diagnosis only. Nothing in `src/` was changed, nothing was pushed, and the fix below is a proposal for David. Follows `docs/reports/20261112_2118_imagined_vs_real_arm_pose.md` (commit 7040b2b).

## Short answer

The suspected cause is right. The dry run measures the pad-to-beaker offset after the pick lift, but against the beaker's original position on the bench. The beaker has been lifted with the hand, so the offset contains the whole lift height (106.6 mm) and the place-down target comes out one lift height too high. With the offset taken at the grasp, as the real place measures it, the dry run's place down and retreat agree with the real arm to within 0.033 rad per joint and 7 mm at the hand, and the dry run still passes for both stances tested.

## 1. The two lines compared

**Dry run**, `src/trajectory_dryrun.py`, function `dry_run_stance` (starts at line 145), lines 206 to 217:

```
206    pad_now, R_now = _commanded_pad(model, d, arm, arm.q_cmd)
207    off = pad_now - np.array([bx, by, bench_top])
...
216    over = np.array([hp[0] + off_p[0], hp[1] + off_p[1], pad_now[2]])
217    down = np.array([hp[0] + off_p[0], hp[1] + off_p[1], hp[2] + off_p[2] + cfg["setdown_gap_m"]])
```
(`off_p` at line 214 is `off` turned by the place's hand turn.)

**Real run**, `src/mpvi_modules.py`, function `plan_place` inside `make_scripted_manipulation` (line 158), lines 256 to 270:

```
256        b = world.object_pose(obj)["position"]
258        p, R0 = arm.pad_pose(data)
261        off = p - b      # the pad centre relative to the beaker's bottom centre in the hand (measured now)
269        over = np.array([hp[0] + off[0], hp[1] + off[1], p[2]])
270        down = np.array([hp[0] + off[0], hp[1] + off[1], hp[2] + off[2] + cfg["setdown_gap_m"]])
```
(the turn of `off` by the place's hand turn is line 267.)

In plain words:
* Both put the hand's set-down height at: hotplate top height + the pad's height above the beaker's bottom centre + the 2 mm set-down gap.
* The real run measures "the pad's height above the beaker's bottom centre" from the world as it is when the place starts: the pad and the held beaker, both lifted together (`p - b`). It is the offset at the grasp, because the beaker is rigid in the hand.
* The dry run has no beaker. It takes the pad after the lift (`pad_now`) and subtracts the beaker's original bottom centre on the bench (`bx, by, bench_top`). The beaker's bottom is no longer there: it rose with the hand. So the lift height is counted as part of the offset.
* The place-over height (line 216, `pad_now[2]`) and the horizontal offsets are the same in both, because the lift is straight up. Only the place-down height, and the retreat that starts from it, differ.

## 2. The numbers, staging run (`data/demo_record/imagined_staging_20261112_205653`)

| Quantity | Value |
|---|---|
| Bench top | 0.9600 m |
| Hotplate top | 1.0280 m |
| Set-down gap | 0.002 m |
| Pad after the move in (grasp) | 1.0143 m, so 54.3 mm above the beaker's bottom |
| Pad after the lift | 1.1209 m, so 160.9 mm above the bench top |
| Lift height (grasp to lift) | 106.6 mm |
| Place-down target the dry run used | 1.0280 + 0.1609 + 0.002 = **1.1909 m** (the dry run's last place-down row: pad z 1.1909 m) |
| Place-down target at the grasp offset (the real run's way) | 1.0280 + 0.0543 + 0.002 = **1.0843 m** |
| Difference between the two targets | 106.6 mm = the lift height exactly |
| Height the real pad reached at the end of place down (episode log, 0.1 s rows) | 1.0918 m |
| Dry run against real pad at place down | 99.0 mm too high |

The real arm's own measured offset (forward kinematics at the end of the lift, minus the logged beaker position): staging 59.4 mm, moved 54.4 mm, against 160.9 mm and 160.6 mm in the dry run. The real pad sits 7 mm above the target; that is the real arm's tracking.
Moved run: lift 106.9 mm, dry-run target 1.1906 m, grasp-offset target 1.0837 m, real pad 1.0869 m, dry run 103.5 mm too high.

## 3. Dry run rerun with the real run's way of computing the target

`experiments/scratch_dryrun_setdown_diag.py` copies the source of `dry_run_stance` in memory, makes two small replacements (record the pad after the move in; use it for `off`), runs it on the accepted stance with the same pick angle and place turn, and leaves `src/` untouched. The unchanged function is run first on the same stance: it reproduces the logged dry run to within 0.0003 in every last row (the stance is rounded to 4 decimals in the log).

Then the existing comparison script (`docs/reports/20261112_2118_imagined_vs_real_arm_pose_files/imagined_vs_real_compare.py`) was run unchanged on a copy of each run folder holding the rerun dry run.

Dry run against the real arm, before and after (staging / moved):

| Point | | Largest joint difference (rad) | Hand, world (mm) | Hand, body (mm) | Largest elbow difference (mm) |
|---|---|---|---|---|---|
| Place down | before | 0.465 / 0.431 | 99.0 / 103.5 | 99.1 / 103.6 | 21.3 / 32.9 |
| Place down | after | 0.024 / 0.033 | 7.0 / 2.8 | 8.7 / 5.2 | 1.7 / 2.7 |
| Retreat | before | 0.441 / 0.422 | 101.9 / 106.8 | 101.9 / 106.8 | 26.8 / 45.2 |
| Retreat | after | 0.028 / 0.014 | 4.1 / 1.7 | 5.3 / 3.5 | 1.2 / 2.9 |

Joints after, real minus dry run (rad), joints 1 to 7:
* staging place down: -0.024, 0.006, 0.001, -0.011, -0.006, -0.004, -0.001
* staging retreat: -0.012, 0.004, -0.010, 0.008, -0.007, -0.008, 0.028
* moved place down: -0.023, 0.002, 0.001, -0.033, -0.003, -0.007, -0.023
* moved retreat: -0.011, 0.004, -0.009, -0.014, 0.000, -0.009, 0.004

Place-approach and pick rows are unchanged to within 1 mm. The home row is unchanged.

The dry run **still passes** for both stances (every subtask ok; place down final error 1.25 mm staging, comparable moved). Timing changes: place down 1.74 s becomes 1.0 s (the moves are shorter), place home 1.94 s becomes 2.45 s (it starts from a lower pose).

One side effect on the old comparison: the set-down point of the check pose was chosen as the one nearest to the dry run's pad. With the dry run corrected, a different one of the check's four set-down points (1.075 to 1.09 m) is nearest, so the check-against-real hand figure for place down changes (2.5 to 7.0 mm staging) without anything having moved. This is only a selection effect.

## 4. Impact on earlier results

**Candidates rejected by the dry run in the three logged search folders** (`imagined_staging_20261112_205653`, `imagined_moved_20261112_210418`, `stance_search_proof_20261112_025903`): none. In each, 51 stances were drawn and only the accepted one reached the dry run (the others were not checked, rejected by the body check, or rejected by the full check). So there is no place-subtask rejection in these folders to rerun.

**Accepted candidates:** the staging stance (draw 32; the proof folder holds the same stance) and the moved stance (draw 15) both passed before and still pass after. The proof folder's stance has the same base, heading and pedestal as the staging run, so its result is the same by construction; I did not rerun it separately.

**Other logs with dry-run rejections** (`data/logs`, from the log text, no per-candidate search files exist for them). 22 rejection lines in 9 logs: by their subtask names they fail at pick OVER, pick DOWN, pick IN or place OVER, none at place DOWN, OPEN, BACK or HOME. Two lines have no subtask name (`seedfix_staging_20261110_214446.log` candidate 5 and `place5_staging_20261110_223412.log` candidate 5, "not reached after 12.8 s (3.8 mm)"; 5 s of dry-run time, as the pick failures take, the place ones 17 to 41 s). The wrong offset only enters the place-down target, the retreat and, to under a millimetre in x and y, the place-over target, all computed after the pick has finished. Place OVER itself is unchanged (final error 0.961 mm against 0.968 mm, same duration to 2 ms). So by the code every logged rejection should come out the same with the corrected target. I could not rerun them: those logs do not hold the stance's heading beyond 0.1 deg or the pick angle and place turn the dry run was given.

| Log | Rejected at pick | Rejected at place OVER |
|---|---|---|
| gen_staging_20261110_133124 | | cand 3 |
| gen_staging_20261110_141847 | cand 16 (IN), 17 (OVER) | cand 3, 13 |
| gen_moved_20261110_141847 | cand 4 (OVER), 25 (IN) | cand 13 |
| gen3_staging_20261110_160152 | | cand 1 |
| gen5_moved_20261110_195401 | cand 14 (OVER), 21 (IN), 28 (IN) | cand 1, 9, 16, 25, 38, 39 |
| fix2_staging_20261110_212006 | cand 1 (OVER) | |
| stance_search_smoke_20261112_020330 | cand 5 (DOWN) | |
| seedfix_staging_20261110_214446, place5_staging_20261110_223412 | cand 5 (subtask not named) | |

**What the wrong target could have affected (for David, not judged):**
1. Every dry run that was accepted (13 logs with one pass each) was accepted with a place-down target about 0.107 m too high. Whether each would still pass at the correct height is unknown; both that could be rerun do.
2. The dry run's place down, open, retreat and home traces and times in `dryrun_trajectories.json` (including the three folders above): the pad paths are about 0.107 m too high; the time and the joint path to the home pose differ.
3. The dry run's collision limits and reference check for the place down were exercised at a pose 0.107 m above the real set-down, so the arm's real set-down pose against the hotplate was never tested in the dry run. At the corrected height, both stances pass.
4. The ghost-robot picture of the place down (video) if it is built from the dry run: wrong arm height.
5. The earlier tables of check-and-reject in the reports of Nov 10 to 12: the rejections in them are at pick or place OVER (above); the dry-run's acceptances are the ones that depend on the wrong height.
6. The gate and the full check do not use this function (the full check solves its own set-down points, 1.075 to 1.09 m, which agree with the real arm). Not affected, by reading.
7. The Nov 10 place failures: the dry-run place rejections in those logs are all at place OVER, which this does not touch. Whether any real-run place failure of Nov 10 involved the dry run's set-down height I did not check.

## 5. Proposed fix (NOT applied, NOT committed)

```diff
--- a/src/trajectory_dryrun.py
+++ b/src/trajectory_dryrun.py
@@ -198,13 +198,14 @@
         ok, why = record_and_check(_move(model, d, arm, name, target, R, speed, cfg))
         if not ok:
             return finish(False, name, why)
+    pad_in, _ = _commanded_pad(model, d, arm, arm.q_cmd)             # the pad at the grasp (after the move in): the beaker's bottom centre is then at the bench top
     ok, why = record_and_check(_hold(model, d, arm, "pick CLOSE", cfg["close_min_s"], trace=trace))
     ok, why = record_and_check(_move(model, d, arm, "pick UP", W["lift"], R, cfg["slow_speed_m_s"], cfg))
     if not ok:
         return finish(False, "pick UP", why)
     # the held beaker keeps the grasp's offset: the pad relative to the beaker's bottom centre at the pick
     pad_now, R_now = _commanded_pad(model, d, arm, arm.q_cmd)
-    off = pad_now - np.array([bx, by, bench_top])
+    off = pad_in - np.array([bx, by, bench_top])                        # the offset at the grasp, as the real place measures it (pad and held beaker together); not pad_now, the beaker rose with the hand
     hp = np.array(d.site("hotplate_top").xpos, float)
     th = math.radians(float(place_theta_deg))
     co, si = math.cos(th), math.sin(th)
```

This is what the scratch script runs, applied to a copy in memory. `pad_now` stays in use for the place-over height and the rotation. It changes the dry run's place down, open, retreat and home outputs and could change which stances the dry run accepts, so it is David's decision (decisions-log row M318, PROPOSED). If applied, the baselines that hold dry-run output would need checking first.

## Not done

* The fix was not applied, no test suite was run, no search was rerun for the other 13 logs (their stance details are not in the logs), no real-run set-down check beyond the two runs.
* No push.

## Files

* `experiments/scratch_dryrun_setdown_diag.py` (the rerun with the patch in memory), `experiments/scratch_real_place_offset.py` (the real offset from the episode log).
* Outputs (git-ignored): `data/diag/setdown_staging/`, `data/diag/setdown_moved/` (numbers, patched dry-run file, patched comparison JSON); logs `data/logs/setdown_diag_staging_20261112_230546.log` and `data/logs/setdown_diag_moved_*.log`.
