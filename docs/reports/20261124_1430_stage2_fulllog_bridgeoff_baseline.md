# Stage 2: full-log diagnosis, policy with the bridge off, baseline check (2026-11-24, David's d0bc brief)

Branch `safety-filter`. Stopped after this report; Stage 3 not started. The sag layer was not changed. Not pushed.

Scratch scripts and data (git-ignored): `data/logs/stage2e/` (`heldstill_log.py`, `baseline_steps.py`, `policy_nobridge/`, `probe/`), diagnostics in `data/logs/stage2d_diag/`.

## 1. Held-still check, bridge on: full per-tick log

Set-up: both arms held at home (measured joints never move); source = zero-mean sweep of joints 1 and 2 plus a **constant +0.01 rad on joint 3**; 1200 ticks (8 s). The bridge is a mirror of `src/control_loop.py` (`bridge_take`, `bridge_update`: buffer clamp at take; per update: tracking-error suspension, step rejection that keeps the previous accepted target, speed cap). The mirror is fed the sag layer's command (source + dq). A second mirror fed the source alone is the side reference (not used). The motor-lag run sets each layer's `tau` to zero as a diagnostic override.

Files: `heldstill_tau_on.csv / .png / .json`, `heldstill_tau_zero.csv / .png / .json`, `heldstill_control_no_const.csv / .png / .json` (in `data/logs/stage2e/`).

| | motor lag on | motor lag zero | control: no constant offset (lag on) |
|---|---|---|---|
| \|C_p\| max / final (mm) | 40.0 / 40.0 (clip) | 40.0 / 40.0 (clip) | 8.1 / 5.1 |
| \|C_r\| max / final (mrad) | 100 / 100 (clip) | 100 / 100 (clip) | 23.4 / 14.7 |
| Position error, last 2 s, min–max (mm) | 2.6 – 12.0 | 2.6 – 11.7 | 0.8 – 10.7 |
| REF minus side REF, max over the run | 0 (1e-14 rad) | 0 | 0 |
| Bridge rules acting | 1 speed-cap event (joint 2, excess 0.010 rad, first update); no rejection, no clamp, no suspension | same | same |
| Max \|command − measured\| (threshold for suspension 0.25 rad) | 0.118 rad | 0.118 rad | 0.038 rad |
| Updates applied / skipped | 1196 applied, 4 skipped (`no_reference`, first ticks); none `suspended` | same | same |

**Plain answer.**

- **Not a leak of dq through REF.** REF equals the side reference to 1e-14 rad at every update. The bridge does not rate-limit this command (one speed-cap event in 8 s), so `output − dq` equals the source. My earlier explanation (the bridge lag carrying dq into REF) does not hold for this test. The Stage 2 bridge-on check was measuring something else.
- **The correction is learning a real, steady error from the held arm.** The constant +0.01 rad on joint 3 is a target the held arm can never reach, so the position error stays at 2.6 mm or more and never returns to zero. The integrator keeps adding to the offset until it reaches the 40 mm / 100 mrad clip. The same growth occurs with the motor-lag term at zero, so the lag term is not the driver.
- **Control.** Removing only that constant offset, the same check stays bounded (8 mm peak, 5 mm final; error oscillates through zero). So the growth is windup against an unreachable target, not an instability in the layer.
- **Suspension.** Does not fire (largest tracking error 0.118 rad, below 0.25). No update is skipped for suspension in any run.
- **Consequence for the Stage 2 test.** The bridge-on check, as written (a constant offset the held arm cannot follow), tests windup, not the speed term. Whether it should be replaced by the zero-mean control is David's decision; I have not changed the test or the layer.

## 2. Policy variants, bridge fully OFF (A–D, 3 seeds, correction on and off)

Log check: no bridge rule acts. The policy summaries record no bridge event (no rejection, no suspension) in any of the 24 runs, since the bridge is not in the loop (`--bridge off`). The only other step on the command that could act is `validate_command` (hard-limit clipping); the left arm's source targets were inside the hard limits on every stream tick in all 12 left-arm logs (the right arm's source targets are not saved by the diagnostic, so they were not checked). The actuators interpolate targets over the source period (`target_mode interpolate`); that is a ramp, not a limit.

Stream: the arms now follow the source directly, so the stream moves (no rejections). Commanded speeds: A 3.6 rad/s, B 1.6–2.1, C 0.8–2.1, D 4.4–5.0 rad/s (the bridge's 0.5 rad/s limit is off).

| Run | Offset peak, left (mm) | Final C_p (mm) | Final \|C_r\| (mrad) | Clip ticks | Hand error on / off (mm, mean) |
|---|---|---|---|---|---|
| A s1 / s2 / s3 | 2.05 | [−0.25, 0.62, 0.67] | 8.4 | 0 | 0.08 / 1.07 |
| B s1 | 26.6 | [−5.4, −1.8, 7.6] | 42.3 | 40 | 32.1 / 39.7 |
| B s2 | 29.2 | [8.2, 4.3, 6.5] | 48.3 | 165 | 40.8 / 44.0 |
| B s3 | 40.0 | [1.8, 0.9, 1.3] | 34.3 | 231 | 37.2 / 39.1 |
| C s1 | 24.0 | [7.5, −1.9, 5.2] | 33.7 | 14 | 29.4 / 31.8 |
| C s2 | 29.1 | [3.8, 4.6, −4.5] | 24.4 | 45 | 33.8 / 35.2 |
| C s3 | 40.0 | [−9.3, −7.5, −2.3] | 36.9 | 122 | 31.5 / 31.4 |
| D s1 | 40.0 | [7.3, 3.6, 14.6] | 11.2 | 4322 | 264.9 / 239.7 |
| D s2 | 40.0 | [15.0, 8.1, 2.5] | 19.6 | 4533 | 255.4 / 233.9 |
| D s3 | 40.0 | [15.1, 3.3, 2.0] | 16.7 | 4102 | 257.3 / 228.9 |

(Hand error is against the accepted reference, which with the bridge off is the source target; "on" is correction on, "off" correction off. The offset is applied on the setup and stream ticks; the only skips are the two no-reference ticks at the start.)

Reading: variant A is well behaved: correction cuts the hand error from 1.07 mm to 0.08 mm (about 13 times). In B and C the correction changes little, and the offset reaches the clip (40 mm) in some runs. In D the arms move at 4–5 rad/s, far faster than the servos follow, so the hand error is mostly lag (about 230–265 mm); the correction is worse than no correction (on 255–265 mm against off 229–240 mm) and winds up to the clip. The layer does not model lag at high speed, which matches Finding 1 of the earlier report: the offset is a windup, not a tracking fix.

## 3. Baselines: timing or real difference? (manual, teleop, policy)

Method: the hand (pad) position at the **end of each step** (end of each key hold in `CC_BENCH_KEYS`; frame f is sim time 1.5 + f/150 s, from the CSV's first row at 1.5 s and its last row at 9.8 s), current run against Stage 1 (`5af2083`); then a timing test, in which the Stage 1 path is shifted by −1 to +1 s over each step's window to see whether the difference disappears.

**Manual** (pad position, mm; left and right):

| Step (keys) | End s | Left diff | Right diff |
|---|---|---|---|
| UP, SPACE, LEFT, SPACE, PAGE_DOWN, PAGE_UP | 2.2 – 6.8 | 0.0 | 0.0 |
| 3 | 7.42 | 27.2 | 0.0 |
| E | 8.22 | 91.5 | 0.0 |
| A | 9.02 | 94.8 | 109.9 |
| 8 | 9.42 | 94.7 | 116.9 |
| I (final, arms home) | 9.82 | 94.7 | 129.3 |

Timing test: the difference is not timing. Best lag reduces the residual only slightly (e.g. step E 79.8 → 77.7 mm; step A 98.6 → 98.6 mm). The arms reach different poses from step "3" on, and stay different. Arms home (step I) is 95 mm (left) and 129 mm (right) away from Stage 1.

**Teleop:** the first difference is at LEFT (left 79.2 mm, right 84.5 mm); the last steps are 37–50 mm (left) and 200–254 mm (right). Timing test: no reduction (step 8: 145.6 → 145.1 mm; step I: 144.3 → 144.3 mm). **Real difference.**

**Policy:** no difference until step "3" (left 8.9 mm). Timing test: 31.8 → 31.0 mm. **Real, small.**

**Probe (current code, offset and clip):**

| Scenario | Left offset peak (mm) / clip ticks | Right offset peak (mm) / clip ticks | Skips |
|---|---|---|---|
| Manual | 6.3 / 0 | 15.7 / 0 | left: 4 no-reference |
| Teleop | 40 (clip) / 147 (p), 136 (rot) | 40 (clip) / 73 (p), 48 (rot) | both arms: 4 no-reference |
| Policy | 6.3 / 0 | 40 (clip) / 69 (p), 22 (rot) | right arm: 45 `suspended` |

So the offset is small in manual, saturates in teleop and in the right arm of the policy run, and the policy's right arm is suspended on 45 ticks (the bridge's tracking-error rule acting on the right arm in that scenario). A 40 mm offset cannot move the hand 200 mm by itself, so the layer is a contributor at most; the scripted sequence also differs. A run of the manual scenario with the sag layer off, on the current code, would separate the two; that check is not done here.

## Answers in one line each

1. Held-still growth: the correction integrates a real steady error from the held arm (the constant joint-3 offset the arm cannot reach), not a dq leak through REF (REF − side REF = 0), not the motor-lag term (same with it zero), and not suspension (never fires). Control without the constant: bounded at 8 mm.
2. Bridge off: no bridge rule acts (no events in any run); correction cuts the hand error from 1.07 to 0.08 mm in A, changes little in B and C, and is worse than off in D, where the arms move at 4–5 rad/s.
3. Baselines: the manual difference from Stage 1 is real (where the arm goes), starting at step "3"; teleop and policy likewise real. Not timing.

## Not run

- Stage 3 and 4 (not started).
- Manual with the sag layer off on the current code (would show how much of the manual difference the layer contributes).
- Right-arm source targets in the bridge-off logs (not saved by the diagnostic).
