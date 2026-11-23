# Lag in the hand-to-target gap: measurement (SF-120 pre-work, 2026-11-23)

Branch `safety-filter`, HEAD `04deeb5`. Measurement only: no production code changed. Commit: the measurement script, the analysis script and this report (see the commit on the branch). Not pushed. The earlier uncommitted reports (trace, trim-off) are untouched.

## Summary

- **Part 1 corrects an earlier attribution.** With the sag trim forced to zero (the solver still running), all four new failures are unchanged. So the trim value is not the cause. The cause is the solver's output: since SF-117 the solver succeeds, so the arm follows the solver's output instead of the raw targets, and that output lags. I said earlier that the correction "running" was the cause; that was too coarse.
- **Part 2: "lag equals speed times one number" does not describe this data.** Lag times from valid cells range from −137 ms to +252 ms in translation and from 95 to 336 ms in rotation. They change sign with direction, change with pose, and do not stay constant with speed. Rotation lag falls from 336 ms at 0.1 rad/s to 149 ms at 1 rad/s. The step response (about 140 to 190 ms) does not match the ramp lag (about 45 to 64 ms forward, negative sideways and downward).
- **The workspace near the held-run start pose is limited.** The home pose sits on the margin edge of joint 4 (2.201 rad). Ramps reach a hard joint limit within 0.2 to 1.5 s in most directions. 41 of 96 cells are invalid for that reason or for IK residuals above 1e-4 rad. The valid cells are listed below.
- **Policy streams (d).** Neither speed explains the gap well. R² is 0.06 (commanded speed) and 0.00 (measured speed) for variant B; 0.13 and 0.09 for variant C.

## Part 1: the four new failures with the trim forced to zero (HEAD 04deeb5, trim zeroed, solver running)

| Test | Normal HEAD | Trim forced to zero | Same? |
|---|---|---|---|
| home_pose_test | crash (TypeError at the printout) | crash (same TypeError) | yes |
| grasp_lift_test | FAIL: outcome STAGING_FAILED | FAIL: outcome STAGING_FAILED | yes |
| beaker_hotplate_correction_test | 5/9 cases as expected (cases 1 to 4 fail: STAGING_FAILED) | 5/9, the same four cases fail | yes |
| nav_follow_sim_test | 0/5 (ARMS timed out after 30 s) | 0/5 | yes |

Method: `trimzero_run.py` (scratch, not committed) patches `_sag_trim_step` to zero the trim after each step in the test process only. Logs: `data/logs/lag_part1_<stamp>/`.

## Part 2: method

`experiments/lag_measure.py --pose home|mid|near`, real loop at 150 Hz, left arm only, trim forced to zero (solver running), bridge off, safety filter off, no held object. Schedule per pose: setup (min-jerk from the resting pose, about 10 s), settle 2 s, rest 5 s (droop), then each ramp: 0.5 s hold, 1.5 s ramp (measured over the last 1 s), return (min-jerk, peak 0.5 rad/s) and 1 s hold; then four 20 mm steps (2 s response). Ramp targets come from damped-least-squares IK on the same forward kinematics the layer uses.

Poses:
- **home**: `TASK_CFG home_q_left` (the held run's start pose). Pad (in the spawned frame) at about (0.30, 0.15, 1.47) m relative to the base.
- **mid**: home with joints 1 and 4 moved (+0.4 rad and −0.3 rad). Pad at z 1.27 m.
- **near**: home with joint 2 at −0.20 rad, close to its upper limit (inside the hard limits).

Directions (world frame): up +z, down −z, sideways +y, forward +x. Rotation axes x, y, z (world).

Rest droop (gap at rest, last 1 s), position in mm and rotation vector in rad:
- home: (−1.28, 0.83, −0.91) mm; (−0.0141, 0.0110, 0.0069) rad
- mid: (−1.93, 0.88, 0.21) mm; (−0.0061, 0.0095, 0.0145) rad
- near: (−1.30, 0.72, −1.07) mm; (−0.0139, 0.0116, 0.0061) rad

The droop differs by up to about 1.1 mm between poses. Rotation droop is 0.01 rad or less.

### Validity

A cell is valid only if its ramp reaches the full 1.5 s and its IK residual stays below 1e-4 rad. Invalid cells: "joint limit" (a hard joint limit ended the ramp early) or "IK residual" (the commanded path left the reachable set, e.g. down 200 and 400 mm/s at the home and near poses). Valid cells: 55; invalid: 41.

## Part 2a: rest droop, ramps and lag (valid cells)

Lag time = (gap minus rest droop, along the motion) / speed. A negative value means the pad is ahead of the commanded hand position. "drift" is the change in the gap between the first and second half of the 1 s window. Cells marked "–" are invalid (joint limit or IK residual, see the detail in `lag_report.json`).

### Translation (lag in ms per pose: home / mid / near)

| Direction | 25 mm/s | 50 mm/s | 100 mm/s | 200 mm/s | 400 mm/s |
|---|---|---|---|---|---|
| up | 161 / 32 / 164 | 24 (mid only) / – / – | – | – | – |
| down | −94 / −69 / −100 | −54 / −26 / −56 | −12 / 184 / −14 | – / – / – | – |
| sideways | −137 / −126 / −137 | −74 / −66 / −81 | −38 / −30 / −40 | 238 / 61 / 252 | – |
| forward | 64 / 47 / 63 | 55 / 44 / 55 | 55 / 43 / 55 | 430 (mid only) / – / – | – |

Speed check on the valid cells: at 25–100 mm/s the pad moves at the commanded speed within about 2 mm/s, except upward at 25 mm/s from home (18.6 mm/s, so the lag there is partly a speed shortfall). At 200 mm/s sideways the pad reaches 181 mm/s (home) and 195 mm/s (mid), so the arm starts to lag the command there. Gap drift across the window is up to 43 mm (mid, forward, 200 mm/s), so the high-speed cells are not steady.

Range over valid translation cells: min −137 ms (sideways, 25 mm/s, home), max +252 ms (sideways, 200 mm/s, near).

### Rotation (lag in ms per pose: home / mid / near; rotation axis about the world axis)

| Axis | 0.10 rad/s | 0.25 rad/s | 0.50 rad/s | 1.00 rad/s |
|---|---|---|---|---|
| x | 336 / 309 / 334 | 206 / 199 / 204 | 158 / – / 160 | 149 / – / 147 |
| y | 102 / 119 / 95 | 141 / 129 / 138 | – | – |
| z | 199 / 127 / 207 | – / 132 / – | – | – |

Range over valid rotation cells: min 95 ms (y, 0.1 rad/s, near), max 336 ms (x, 0.1 rad/s, home). Rotation lag falls with speed on the x axis (336 to 149 ms), so it is not proportional to speed.

## Part 2c: steps (20 mm), time to 63 % of the step

| Direction | home | mid | near |
|---|---|---|---|
| up | 153 ms | 173 ms | 153 ms |
| down | 167 ms | 193 ms | 167 ms |
| sideways | 153 ms | 140 ms | 147 ms |
| forward | 173 ms | 153 ms | 173 ms |

The pad reaches about 20 to 22 mm at 2 s (all cases), so it settles on the step. Overshoot peaks at 21 to 35 mm.

## Part 2d: policy-style streams (variants B and C, correction off, bridge off; from `policy_stream_20261123_0708/`)

Fit: |gap| (ref minus pad) = a + b × speed, over the variant (after the 10.3 s setup), with 0.2 s centred differences.

| Variant | speed | intercept a (mm) | b (s) | R² |
|---|---|---|---|---|
| B | commanded (ref) | 20.6 | 0.040 | 0.055 |
| B | measured (pad) | 39.5 | 0.000 | 0.00001 |
| C | commanded (ref) | 10.9 | 0.058 | 0.128 |
| C | measured (pad) | 15.9 | 0.043 | 0.095 |

The commanded speed explains a little more than the measured speed in both variants. Neither explains most of the gap, and the intercepts (11 to 40 mm) are far larger than the rest droop (about 1.5 mm).

## Part 3: answers

1. **Is the gap proportional to speed?** No. Translation lag changes sign (sideways and downward at 25 to 100 mm/s are negative, forward and upward are positive). Within a direction, lag changes with speed, and at 200 mm/s (sideways) the arm lags the command. Rotation lag on x falls from 336 to 149 ms between 0.1 and 1 rad/s. At the highest valid speed (200 mm/s sideways) the lag is 61 to 252 ms and the gap is not steady.
2. **Does lag depend on direction or pose?** Yes on both. Forward is consistent across poses (43 to 64 ms at 25 to 100 mm/s). Sideways and downward are negative at low speed, with up to 137 ms of spread. Mid differs most (up: 32 ms against 161 ms at home; down at 100 mm/s: 184 ms against −12 ms). Over valid translation cells: min −137 ms, max +252 ms; over rotation cells: min 95 ms, max 336 ms.
3. **Which speed explains the gap better (d)?** Neither well. Commanded speed (R² 0.06 and 0.13) beats measured speed (R² 0.00 and 0.09) in both variants.
4. **Step response against the ramp lag.** They do not agree. Step time to 63 % is 140 to 193 ms; the forward ramp lag is 43 to 64 ms; the sideways and downward ramp lags are negative at the same time.
5. **Plain-language answer.** A single lag time times speed is not a good model here. The gap depends on direction, pose, and speed in ways that change sign, and the arm loses speed at high hand speeds. The step response suggests a slower dynamic than the forward ramp. The data point to a more complicated, possibly second-order or direction-dependent response, plus a speed limit near 200 mm/s. I did not choose a model.

## Questions for David (options, not decided)

1. **Model form.** (a) One lag time times speed (the simplest; fails here). (b) One lag time per direction, fitted per pose (more parameters; needs more ramp data and fits only the valid directions). (c) A second-order response fitted to the step data (matches the step times, but the ramp data disagree). (d) A learned correction from measured velocity and pose (flexible; hard to audit, needs more data). Tradeoff: simplicity against fit.
2. **Solver in the measurement.** These numbers include the solver's speed cap (the solver ran; the trim was only zeroed). The raw pass-through response (layer disabled) is not measured. Do you want it measured, since it would show how much of the lag is the solver's and how much is the arm's?
3. **Poses and limits.** The home pose is on the margin edge of joint 4, so most directions hit a joint limit within 1.5 s. Should the next measurement use a pose further from the limits, and should the held run's start pose be redefined?
4. **Speed range.** The arm cannot follow 200 mm/s sideways. Is that a servo limit, a solver limit (the 0.19° per tick cap), or the ramp's IK? Should the correction assume a speed ceiling?

## Not run / limits

- Rotation ramps beyond the joint limits and the IK-residual cells (41 invalid cells; see `lag_report.json`).
- The raw pass-through (layer disabled) response (see question 2).
- The policy-stream fit with the bridge on (the bridge freezes that stream; see the verification report).
- Poses other than the three listed; a broader set would be needed for a pose model.
- The held run's start pose is taken as `TASK_CFG home_q_left`, not a logged held-run pose.

## Files

- `experiments/lag_measure.py` (measurement; committed)
- `experiments/lag_analyze.py` (analysis; committed)
- Outputs (git-ignored): `data/logs/lag_measure_<stamp>/` (per-pose tick CSVs and segment JSONs, `lag_report.json`); `data/logs/lag_part1_<stamp>/` (Part 1 logs)
