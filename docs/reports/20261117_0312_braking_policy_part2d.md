# Braking policy, Phase D (fixes from the Phase C review, integration rerun)

Branch safety-filter (worktree ~/projects/openarm-tracer-sim-sf-d). NOT pushed. Built on 1857c38 (Phase C).

## 1. Result in one paragraph

All five Phase D fixes are built, tested and committed. The integration run does NOT pass. The grasp pose is now reached (the palm came to 21.8 mm from the bench, the grasp pose is 23 mm), the commanded hand speed never exceeds the 0.20 m/s cap, and no bridge override or look-ahead error was logged in the final run. But the acceleration and jerk limits are broken on free motion (3.3 m/s² and 538 m/s³ against 0.5 and 2.0), the beaker closes on nothing (the fingers shut to 3 mm), the recovery move fails, and two overlaps occur: the sent command puts the left link 7 68 mm into the torso, and the held beaker goes 39 mm into the bench. Two causes are open decisions for David (section 6): the emergency test is switched off while the layer is already braking, and the look-ahead checks the stopping distance only at the current speed.

## 2. Commits (branch safety-filter, not pushed)

| Commit | Content |
|---|---|
| 211555a | Fix 1: work surfaces (bench_top flag; PUSH_STOP_BEYOND_M_WORK 0.005 m; stop target 1.5 cm) |
| ecc4a75 | Fix 2: the cap holds on every tick, on the step (root cause of the overspeed, section 5) |
| b483b06 | Fix 3: emergency hard clamp at 1.5 x cap (measured speed), holds the onset pose, overrides the bridge, logs |
| 0da5ba3 | Fix 4: the beaker is its true cylinder; cylinder-capsule and cylinder-box distances; batched table and evaluate_pairs |
| aeb5430, 59ebb74 | Progress entries (run commands and logs, before each run) |
| 562639d | Measurement fix found by the integration run (speed history restarts after a call gap) |

Full list: `git log 1857c38..HEAD`. 16 files changed, 748 insertions, 51 deletions over Phase C.

## 3. Test results

Last full regression (all 14 safety-filter test files, run.sh, after 562639d): 14 of 14 exit 0.

Not in the set (failing before Phase D, unchanged): `safety_filter_spec_test` (2 baseline checks), `safety_filter_sequential_push_test` (missing fixture).

New test file `experiments/safety_filter_braking_d_test.py`: 47 PASS, 0 FAIL. It covers the work-surface stop target (1.5 cm, and a control at 3 cm), the 4 rad jump and a measured 0.25 m/s start (the cap on every tick), the emergency clamp (0.33 m/s clamps, 0.25 m/s does not, the configured multiplier), the cylinder contacts (flat bottom 0, lateral 2 cm, rim-to-edge sqrt(0.02^2 + 0.03^2), capsule 2 cm), the batched table against the object form, 17 random cylinder-box poses, and the measurement gap rule.

Existing tests changed on purpose (beaker is a cylinder now): braking C step 1 (extent is the half height), touch (held shape and catalog are cylinders), never-touch (helper uses the cylinder centre), braking B (catalog held shape is a cylinder).

## 4. Integration runs

Command for both: `python experiments/safety_filter_braking_episode.py --no-shelf --name <run>` (tmux, log under data/logs/, 6 GB cap). Analysis: `experiments/safety_filter_braking_analyze.py`, plus two scratch checks (palm and bench, stop episodes, arm overlaps).

| Item (Phase D checklist) | brkW (final code, 562639d) | Phase C brkU (d7df3c2) | Result |
|---|---|---|---|
| Outcome | PLAN_FAILED at the pick: OVER, DOWN, IN, CLOSE, UP done; beaker not held after UP (oracle: not held, fingers at 3.0 mm); BACK recovery failed (620 mm) | PLAN_FAILED at IN (834 mm) | fail |
| Hand speed (sent command) every tick | peak 0.2000 m/s; 0 ticks above the cap (fixed-base FK) | peak 0.688 m/s; 1573 ticks above | PASS |
| Hand speed (measured, tick to tick) | peak 0.292 m/s | 0.570 m/s | above the cap, below the hard clamp (0.30) |
| Accel and jerk, free motion (sent) | 3.28 m/s²; 538.5 m/s³ (limits 0.5 and 2.0) | 49.1 m/s²; 14 594 m/s³ (commanded) | fail |
| Accel and jerk, braking (sent) | 1.61 m/s²; 257.8 m/s³ | 48.4 m/s²; 14 594 m/s³ | fail |
| Hand stops at the new target and stays | stops hold: max joint step over the last 100 ticks 0.0015 rad. The palm's stop target (15 mm) was not reached: the palm stopped at 21.8 mm at the IN waypoint | not tested | partial |
| Palm reaches the grasp pose (23 mm) | min palm to bench 21.8 mm at t = 58.5 s | palm stopped 3 cm above | PASS |
| Beaker reaches the hotplate at ceiling speed | not reached (min surface gap 73.8 mm) | not reached | not tested |
| No bridge overrides | 0 override ticks, 0 bridge errors | 2 overrides | PASS |
| No false look-ahead failures | 0 look-ahead errors, 0 hard clamps | 2 emergencies | PASS in the log; see the overlaps below |
| No arm overlaps | sent command: left link 7 at −67.6 mm into the torso (t = 79.15 s, filter record). Measured joints, every call in 78.9 to 79.9 s: left link 4 vs eagle_cam_body −17.8 mm; left palm vs right link 2 −17.1 mm | −73.6 mm (left finger into the right shoulder mount) | fail |
| Held beaker vs bench | −39.5 mm (left_held vs bench_top, t = 66.45 s); 245 ticks below zero from t = 64.8 s | not reported | fail (new) |
| Known issue "gripper did not stop within 5 s" | not seen (0 lines) | not seen | not hit |

Run numbers: brkW 9906 filter calls, 106.8 s sim, 1749 s wall, status counts HOLD 6383, PASS 3364, KONG_PUSH 159; filter time per call mean 107 ms, p95 257 ms, max 1249 ms (analysis tool). brkU 5897 calls, 1439 s wall. Wall times are not directly comparable (the runs went different distances).

### 4.1 The first run (brkV) and what it found

brkV (same code except the measurement fix) showed a false hard clamp: after a 38 s gap between two filter calls the base had moved 2.3 m (the arm did not move); the first call divided that displacement by one tick and read 395 m/s (both arms), and the hard clamp fired at t = 50.1 s. The gap is now a restart of the speed history (call_dt from the plugin, gap > 1.5 ticks). A first attempt to measure the arm in the current base frame was reverted, because it contradicted the Phase A decision that base motion counts (section 6).

brkV also showed the two open decisions in section 6 (the palm approached the bench from 0.05 to 0.2 m/s with no brake until too late, then penetrated it by 42 mm at t = 100.2 s, and no emergency fired).

## 5. Root cause of the 0.57 m/s overspeed (Phase C, brkU)

The cap entered the speed ramp only through its target (min of the commanded speed and the cap). The ramp can brake only at the acceleration limit (0.5 m/s² per tick). So when the ramp starts above the cap (the arm already moving fast), the step was sent at the ramp's own speed, which is not clamped to the cap. Reproduction on the code before fix 2: a layer moving along joint 0 at the cap is held (HOLD) while the measured arm moves at 0.57 m/s: the sent speed is 0.633 m/s (brkU sent 0.688). Fix 2 clamps every step to the cap; the same test sends at most 0.20 m/s.

Limit of the evidence: the brkU recording was lost (its worktree was removed), so the replay on brkU itself was not possible. The reproduction is synthetic. Whether the measured 0.57 m/s start was real arm motion or partly the gap artifact found in brkV could not be checked against brkU.

## 6. Decisions for David

1. **Emergency switched off while braking.** `check()` raises an emergency only when the arm is not already braking (`not was_braking`). In brkV the palm went 42 mm into the bench with no emergency; in brkW the held beaker went 39 mm into the bench with no emergency. My recommendation: drop the `not was_braking` condition (keep the stop, but raise the override when the stopping distance exceeds the room to the margin).
2. **The look-ahead checks the current speed only.** The palm accelerated from 0.05 to 0.2 m/s toward the bench (13 cm to 67 mm) and the stopping distance at the current speed stayed below the room until it was too late. The check should use the speed the ramp can reach (the cap, or the approach ceiling).
3. **Free-motion overshoot (finding F-D1, from Phase D fix 1).** The ramp's target is the lag between the commanded and the last pose, not the command's own speed. When the output catches the lag it overshoots (2.7 times the commanded step), then the "never past the commanded pose" clip cuts it: accelerations of −1.4 m/s² and jerk above the limit. This causes the free-motion limit failures in brkW (3.28 m/s², 538 m/s³). A distance-aware target would fix it. Not built: it changes the free-motion design.
4. **Base motion counts in the measured hand speed** (world frame, as decided in Phase A and tested). Keep it, or measure only the arm? The gap rule (speed history restarts after a call gap longer than 1.5 ticks) is my choice; confirm it.
5. **Hard clamp holds the onset pose.** The clamp does not track the moving arm. A backward step to the onset pose can exceed the cap for that tick. Confirm this is the intent for a last resort.
6. **Approach ceilings are not clamped by the hard cap.** The hotplate ceiling (0.05 m/s) stays on the S-curve, so a held beaker approaching it is not stepped down at once. Confirm.
7. **Work-surface placeholder.** PUSH_STOP_BEYOND_M_WORK = 0.005 m (placeholder); the stop target 1.5 cm is therefore a placeholder too.
8. **Cylinder distance cost.** cylinder-box is GJK in Python, about 1.8 ms per call; cylinder-capsule about 0.7 ms. A held beaker near several fixture boxes can add several milliseconds per tick. Overlap depth for a cylinder against a box is approximate; cylinder against cylinder is not supported (one cylinder object exists). Decide whether a faster method is wanted before the pick is tuned.
9. **Torso overlap.** The sent command put left link 7 68 mm into the torso during the BACK recovery. The torso exemptions are still open (Phase C decision list). Decide the torso shape set.
10. **Grasp that does not hold** (manipulation, not braking): the palm reached the grasp pose, the fingers closed to 3 mm, the beaker was not held after UP. The recovery BACK then failed (620 mm). Out of braking scope; noted only.

## 7. Not done

- The integration pass (brkW does not pass, section 4).
- The hotplate ceiling check (the beaker never reached the plate).
- The palm's stop target check (the palm stopped at its waypoint, not at the target).
- Fixes for decisions 1 to 3 above (they change the design; waiting for David).
- Push: not done (per the standing instruction). The worktree ~/projects/openarm-tracer-sim-sf-d is kept (its data/ holds the brkV and brkW recordings and logs).
- The brkU replay (the recording was lost with the Phase C worktree).

## 8. Files changed in Phase D (main ones)

src/safety_filter/braking.py (fixes 1 to 3, measurement); config.py (PUSH_STOP_BEYOND_M_WORK, bench_top catalog entry, emergency_speed_multiplier, beaker cylinder); world.py (work_surface, cylinder objects); touch.py (work-surface rules, cylinder gap dispatch, evaluate_pairs cylinder batch); shapes.py (cylinder box); tick_arrays.py (cylinder pairs exact-only, bounds); geometry.py (cylinder-capsule, cylinder-box, GJK); safety_filter_plugin.py (call_dt, held cylinder record); experiments/safety_filter_braking_d_test.py (new); experiments/run_tests.sh (braking C and D listed); the four test files changed for the cylinder.
