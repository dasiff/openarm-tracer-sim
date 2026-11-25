# Stage 2 follow-up: the teleop offset after motion stops, the controlled move-then-stop test, the speed-weighted replay, and the release against the plan (2026-11-25, David's follow-up)

Branch `safety-filter`, HEAD e4dbd5b before this round. Analysis only: the layer, the loop and the gripper are unchanged. One MuJoCo process at a time. Each run in its own folder under `data/logs/rebuild_s2d/` (git-ignored) and `data/logs/rebuild_s2c/`. Nothing pushed. Baseline worktrees kept (`openarm-tracer-sim-bl-1f29da7`, `openarm-tracer-sim-bl-5e6d7be`, `openarm-tracer-sim-bl-7d355b8`).

Criterion C (the held run, perturbed) is recorded as failed for the gripper stop-check reason only, as David decided; the gripper was not investigated here.

## Plain-language answer

1. **What drives the teleop run.** A scripted operator, not recorded human data. The fake operator is in view for 4 s of every 8 s; while in view, its goal for every joint is 0.4·sin(t + joint index), clipped to the joint limits. The 5.4 s pause is the out-of-view half of the cycle.

2. **Where the 22.7 mm at the pause comes from.** Joint 1 carries 15.4 mm of it and joint 4 carries 5.1 mm; the other joints carry 0.4 to 1.6 mm each. The estimate builds up during the engagement motion, from 0.5 mrad at 3.4 s to 37 mrad at 4.9 s (the offset from 0 to 25 mm). It does not come from a static droop: it grows while the joints move at 0.4 to 1.4 rad/s and the sag is the motion lag. The estimate then sits at 38 mrad for the whole 2.6 s pause. The reason is that the layer only updates when the source sends an arm command, and the fake operator sends none while it is out of view: the estimate, the measured sag and the skip counters are constant for those 2.6 s. The bridge's 19 rejection entries fall at 3.0 to 4.0 s (18 entries) and at 8.0 s (one, inside the pause); the joint 2 and joint 4 clamps fall at 3.3 to 5.0 s; the bridge never suspended the arm.

3. **The rule itself is fine when commands keep coming.** In the controlled move-then-stop test (joints 1 to 4 together, 0.3 rad, bridge on, four cycles, 3 s holds), the hand error against the reference 0.5, 1 and 2 s after each stop is 0.01 to 0.03 mm with correction on, against 1.25 to 1.9 mm with it off. The estimate at each stop equals the at-rest droop to within 0.05 mm, at 0.1, 0.25 and 0.5 rad/s, nominal and perturbed.

4. **The speed-weighted replay (offline, approximate).** With a weight 1/(1+(speed/s0)^2) per joint, the teleop estimate at the pause falls from 24.6 mm to 4.9, 6.9 and 11.0 mm for s0 = 0.05, 0.1 and 0.2 rad/s. The approximate hand error 1 s after the pause falls from 22.0 mm to 2.2, 4.3 and 8.3 mm. On the five static holds no single s0 improves every pose: s0 = 0.05 is closer to the droop than the current rule at home and policy B 13.4 s, and farther at pick, policy B 7.4 s and policy B 21.4 s.

5. **Policies B and C do not show the same build-up at their slowdowns.** Policy C has 24 slowdowns in 30 s; across them the hand-space estimate changes by −0.02 mm on average (the first slowdown, during the initial build-up after setup, +0.39 mm). Policy B has one slowdown in 30 s.

6. **Release against the plan (nominal held run).** The hand error against the plan's own waypoints is 2.9 to 4.0 mm over the first 4 s after the release, and 7.2 mm at 5 s. The plan's pad path in the window is 242 mm long.

**Decision for David:** the teleop problem is a property of the source's silence (no arm commands while out of view), together with the fast, bridge-limited engagement motion; the layer's rule is not what fails in the controlled test. The options (not decided, nothing changed): (a) call the layer on every bridge update with the last command, so the estimate keeps moving while the arm holds; (b) let the estimate relax when the arm is still; (c) leave it and say so. Stage 3 should not start on the teleop until this is decided.

## Technical block

### 1a. What drives the teleop run

`src/teleop_source.py`: `_FakeSource` (in view for 4 s, out of view for 4 s, repeating; with `CC_FAKE_TELEOP_SIMCLOCK=1` the cycle runs on sim time) and `_FakeSession.solve` (goal = 0.4·sin(t − t0 + joint index) for every joint of both arms, clipped to the joint limits, ignoring the camera). The applied arm target moves toward the goal by at most TELEOP_MAX_JOINT_VEL per tick. "Tracking lost — holding last pose" is printed when the fake operator goes out of view; the arms then get no arm command.

### 1b. The teleop run, correction on (`data/logs/rebuild_s2d/teleop/on/run.npz`; table `analysis/teleop_offset.md`, build-up `analysis/teleop_replay.md`)

Stop at 5.41 s (the reference unchanged for 1 s; the only stop in the run). Hand error 1 s after it: 22.66 mm (logged), of which the per-joint hand moves of (reference − measured) give 22.64 mm in the linear check.

| joint | reference − measured (deg) | hand move of that error (mm) | estimate (deg) | hand move of the estimate (mm) | prediction error, predicted − measured (deg) | speed before stop (rad/s) | acceleration around stop (rad/s²) |
|---|---|---|---|---|---|---|---|
| 1 | +1.589 | 15.42 | −1.649 | 16.00 | +0.759 | −0.404 | +1.87 |
| 2 | +0.044 | 0.45 | −0.088 | 0.91 | −0.349 | +0.026 | −0.48 |
| 3 | −0.136 | 0.38 | +0.110 | 0.30 | −1.059 | +0.366 | +0.07 |
| 4 | −0.736 | 5.14 | +0.812 | 5.67 | −0.813 | +0.450 | −0.52 |
| 5 | −0.433 | 0.40 | +0.678 | 0.63 | +0.357 | +0.084 | −0.45 |
| 6 | −0.511 | 1.64 | −0.031 | 0.10 | +1.732 | −0.308 | +0.06 |
| 7 | +0.442 | 1.45 | −0.953 | 3.13 | +1.534 | −0.507 | +0.16 |

Build-up around the stop (estimate norm, offset |C_p|, estimate in hand space, prediction error norm, joint speed norm):

| time (s) | estimate (mrad) | offset (mm) | estimate, hand (mm) | prediction error (deg) | speed (rad/s) |
|---|---|---|---|---|---|
| 3.41 | 0.51 | 0.08 | 0.08 | 1.41 | 0.46 |
| 3.91 | 5.05 | 2.39 | 2.39 | 2.56 | 1.41 |
| 4.41 | 19.51 | 13.42 | 13.41 | 4.71 | 0.79 |
| 4.91 | 37.05 | 24.80 | 24.80 | 6.34 | 0.73 |
| 5.41 | 38.10 | 24.63 | 24.62 | 2.09 | 0.61 |
| 5.92 to 7.92 | 38.10 (frozen) | 24.63 (frozen) | 24.70 (frozen) | 2.81 to 2.83 | 0.00 |
| 8.42 | 30.43 | 19.33 | 19.32 | 0.68 | 0.49 |

The freeze: between 5.6 and 6.0 s, and between 8.6 and 9.2 s, the estimate, the measured sag and the source command are constant over every tick, and the skip counters do not rise (no call to the layer). Between 6.0 and 8.0 s only 6 of 300 ticks changed. The estimate starts to move again at 8.4 s, when the fake operator returns.

Bridge events (whole run): 19 "rejected: target step too large" entries (77 counted), all at 3.0 to 4.0 s on joints 1 and 7 (steps 0.25 to 0.32 rad) and one on joint 3 at 8.0 s; no suspension ticks; buffer clamps on joint 2 (12 updates, 3.33 to 3.54 s) and joint 4 (83 updates, 3.33 to 4.96 s). The off run has the same stop with correction off: hand error 1 s after the stop 2.66 mm (the at-rest droop used below).

### 1c. Controlled move-then-stop test (`experiments/move_stop_test.py`; runs `data/logs/rebuild_s2d/move/v{speed}_{physics}_{on|off}/`; table `analysis/move_stop.md`)

Left arm, bridge on, correction on and off, nominal and perturbed physics (`--perturb` of `climb_perturb_diag.py`). Joints 1 to 4 move together by 0.3 rad from the home pose (joint 4 down, because +0.3 from home passes its 2.44 rad limit) at a set joint speed, stop, hold 3 s; four out-and-back cycles (eight stops). Hand error = pad of the reference (bridge accepted target) minus pad of the measured joints. The at-rest droop = the same stop's hand error 2 s after the stop with correction off. Means over the eight stops:

| speed (rad/s) | physics | correction | error 0.5 s (mm) | 1 s (mm) | 2 s (mm) | estimate at stop, hand (mm) | at-rest droop (mm) |
|---|---|---|---|---|---|---|---|
| 0.1 | nominal | on | 0.012 | 0.011 | 0.009 | 1.27 | 1.25 |
| 0.1 | nominal | off | 1.254 | 1.253 | 1.251 | — | 1.25 |
| 0.1 | perturbed | on | 0.014 | 0.012 | 0.010 | 1.90 | 1.89 |
| 0.1 | perturbed | off | 1.891 | 1.891 | 1.892 | — | 1.89 |
| 0.25 | nominal | on | 0.022 | 0.018 | 0.013 | 1.30 | 1.27 |
| 0.25 | nominal | off | 1.268 | 1.267 | 1.266 | — | 1.27 |
| 0.25 | perturbed | on | 0.027 | 0.022 | 0.016 | 1.90 | 1.89 |
| 0.25 | perturbed | off | 1.887 | 1.888 | 1.888 | — | 1.89 |
| 0.5 | nominal | on | 0.027 | 0.024 | 0.016 | 1.31 | 1.27 |
| 0.5 | nominal | off | 1.274 | 1.272 | 1.270 | — | 1.27 |
| 0.5 | perturbed | on | 0.034 | 0.029 | 0.020 | 1.90 | 1.89 |
| 0.5 | perturbed | off | 1.889 | 1.887 | 1.887 | — | 1.89 |

Per-stop values and the bridge events of each run are in `analysis/move_stop.md` (each run's `move/schedule.json` has the stop times and its bridge log: one "target step too large" entry at the first move in the smoke run). The schedule was first run with an absolute clock and its first target was a 0.52 rad step from the command (the bridge held the arm, no motion); it is measured from the first call now.

### 1d. Offline replay with the speed weight (`analysis/replay.md`, `analysis/teleop_replay.md`)

The replay re-runs the layer's low-pass (T = 2 s) on the logged sag, updating on the ticks where the layer was called. With weight 1 it reproduces the logged estimate: largest difference 3.8e-16 rad on the move runs, 8e-8 rad on the teleop run. The hand error in the tables below is an APPROXIMATION: the droop minus the replayed estimate's hand move (scalar), with the logged measurements held fixed (no closed loop).

Teleop run, at the pause (droop 2.66 mm from the correction-off run):

| weight | estimate at the pause, hand (mm) | approx. hand error 1 s after the pause (mm) |
|---|---|---|
| current (1) | 24.62 | 22.03 |
| s0 = 0.05 rad/s | 4.86 | 2.20 |
| s0 = 0.1 rad/s | 6.94 | 4.28 |
| s0 = 0.2 rad/s | 10.97 | 8.31 |

Five static holds (perturbed, correction on; estimate at the stop, hand space, against the at-rest droop, mm):

| pose | droop | current (1) | s0 = 0.05 | s0 = 0.1 | s0 = 0.2 |
|---|---|---|---|---|---|
| home | 0.33 | 1.23 | 0.78 | 0.97 | 1.07 |
| pick | 2.60 | 2.82 | 1.14 | 1.47 | 1.79 |
| policy B, 7.4 s | 3.29 | 3.06 | 2.21 | 2.39 | 2.56 |
| policy B, 13.4 s | 3.56 | 3.37 | 3.57 | 3.43 | 3.29 |
| policy B, 21.4 s | 3.34 | 3.53 | 3.96 | 3.98 | 3.98 |

On the controlled moves the weight changes almost nothing (the logged sag there is already small). On the five holds no weight improves every pose: the current rule is closest to the droop at pick, and s0 = 0.05 is closest at policy B 13.4 s.

### 1e. Policies B and C at their slowdowns (bridge on, correction on, seed 1; `analysis/policy_slowdowns.md`)

A slowdown: the source's joint speed falls below 0.05 rad/s after being above 0.2 rad/s in the previous 0.3 s. Hand-space estimate read 0.5 s before and 1 s after each slowdown.

- Policy B: 1 slowdown in 30 s (at 3.39 s, +0.39 mm).
- Policy C: 24 slowdowns in 30 s; mean change −0.02 mm (the first, +0.39 and +0.27 mm at 3.39 and 4.60 s, are during the initial build-up; after 7 s the change is between −0.08 and +0.02 mm).

The build-up at the teleop pause is not reproduced at the policies' slowdowns: their estimate stays around 1 to 2 mm.

### 2. Release against the plan (nominal held run, `analysis/release_vs_plan.md`)

The plan's waypoint for each call is the source's joint target (the trace's `left_src`); the hand error is the pad of that waypoint minus the pad of the measured joints. Release completed at 75.24 s (log: `release_done`). Window 5 s (751 calls).

| time after release (s) | hand error against the plan (mm) | plan pad moved from its release position (mm) | measured pad moved (mm) |
|---|---|---|---|
| 0.5 | 3.17 | 30.5 | 28.8 |
| 1.0 | 2.89 | 65.5 | 63.9 |
| 2.0 | 3.93 | 86.9 | 86.2 |
| 3.0 | 3.83 | 97.1 | 95.9 |
| 4.0 | 4.03 | 103.1 | 99.8 |
| 5.0 | 7.18 | 135.9 | 130.4 |

Largest hand error in the window 7.18 mm (at 5.0 s); mean 3.98 mm. The perturbed run did not reach its release (criterion C, gripper stop-check), so only the nominal run is reported, as the brief asks.

### Runs and scripts

- Teleop traces, correction on and off, with the extended logs (prediction, applied offset, control, clamp counts, bridge event log): `data/logs/rebuild_s2d/teleop/{on,off}/run.npz`; script `experiments/teleop_tick_log.py` (extended).
- Move-and-stop runs (12): `data/logs/rebuild_s2d/move/v*`; script `experiments/move_stop_test.py` (new), run under `experiments/climb_perturb_diag.py` (now also records the estimate and the measured sag).
- Policy B and C traces, correction on, bridge on: `data/logs/rebuild_s2d/policy_r2/{B,C}/trace.npz` (the first attempt, `policy/`, failed: the trace runner read a removed attribute; fixed; the failed folder is kept). Script `experiments/sag_trace_run.py` (fixed; records the arm side).
- Five-pose trace, correction on, perturbed: `data/logs/rebuild_s2d/static_r2/trace/trace.npz` (same failure and fix).
- Analysis: `experiments/stage2_teleop_offset_analysis.py` (`teleop`, `teleop-replay`, `moves`, `replay`, `policy`); `experiments/stage2_release_vs_plan.py`.
- Smoke run of the move test (first attempt, schedule error): `data/logs/rebuild_s2d/smoke/`, `smoke2/`.

### Caveats

- The replay is offline with fixed logged measurements; its hand errors are approximate. The move-and-stop test is the closed-loop check.
- The silence in 1b is the scripted operator's out-of-view phase. A live operator may keep sending commands; the freeze is a property of the source, not of the rule.
- The bridge's rejections and clamps are in the engagement transient, which also carries the largest joint speeds of the run. I have not separated the contributions of the rejections, the joint 4 clamp and the speed; the teleop build-up is shown, its cause is not isolated.

## Questions for David

1. **The silence.** Should the layer keep updating while the arms hold (the estimate relaxes toward the measured sag), or should it hold as now? This is the decision that stands between the teleop and Stage 3.
2. **The speed weight.** Do you want s0 chosen from these replays? The replay helps the teleop pause and does not help all five poses, and the replay is approximate; a closed-loop run would be needed first.
3. **Criterion C.** Still recorded as failed for the gripper stop-check reason only, as you decided; the release report uses the nominal run only.
