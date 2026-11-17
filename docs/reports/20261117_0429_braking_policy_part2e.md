# Braking policy, Phase E (fixes from the Phase D review, integration rerun)

Branch safety-filter (worktree ~/projects/openarm-tracer-sim-sf-d). NOT pushed. Built on f835f2a (Phase D report).

## 1. Result in one paragraph

Fixes 1 to 3 are built, tested and committed as decided. Fix 4 (torso check) is read-only and reported in section 4. The integration run brkY is the first run in which the pick succeeds (the gripper holds the beaker, the arm lifts it, navigates, and opens over the hotplate with the beaker on it). It still does not pass: the BACK move after the release fails, and the cause is a false emergency that holds the left arm for about 10 s. The false emergency comes from finger motion: the closing speed of the finger pair includes the gripper opening (6.4 mm in one tick with the arm still). Two further results: sent acceleration and jerk spike at t = 68.3 s (16 m/s² and 4 700 m/s³, the braking output alternates between about 0.03 and 0.13 m/s on successive ticks; the Phase D run had no such spikes, so fix 3 is the prime suspect, not proven), and the distance-aware cap itself gives a 20 m/s³ jerk spike on the final approach of a short move.

## 2. Commits

| Commit | Content |
|---|---|
| d2a1a4f | Fix 1: the emergency also while braking (the `not was_braking` guard removed) |
| d0ab28a | Fix 2: look-ahead uses the speed the ramp will reach (ramp speed x closing share, plus the formula's a^2/(2j) raise) |
| 711ffa6 | Fix 3: distance-aware ramp (stoppable speed for the tool distance left, caps the ramp speed each tick) |
| 3dbe774, 1525134, ac1e592 | Test list, progress entries |

A first version of fix 2 used the commanded closing rate as the reachable speed. It made a slow arm brake at the cap's stopping distance and then cycle (C step 4 stopped at 73 mm, not 30 mm; five test files failed). It was replaced by the profile version above.

## 3. Test results

Full regression after fix 3 (15 safety-filter files, run_e.sh): 15 of 15 exit 0, including the new `safety_filter_braking_e_test.py` (13 checks in that file, all PASS) and `braking_d` (47 checks, the palm tolerance widened, section 5).

Test changes made on purpose (listed in the commit message of 711ffa6): braking shaping and C step 6 used a source that moved with the output (it never builds a lag under the distance-aware cap); they now use independent streams. The shaping convergence check uses a fixed target. The Phase D palm stop tolerance widened from 16.0 to 17.5 mm (the stop lands 1.3 mm further on the safe side).

Not run: spec_test and sequential_push_test (pre-existing failures).

## 4. Fix 4: torso check (read-only, nothing changed)

Question: is link 7 vs torso in the never-close list? If yes, it should not be.

Answer: no. The list (366 pairs) holds 8 torso pairs, and none of them is link 7:

| Pair | On the never-close list? | Smallest measured gap (brkW) | Smallest measured gap (brkY) |
|---|---|---|---|
| left_link1_3 vs torso | yes | **−1.01 mm** (t 69.5 s) | **−0.11 mm** (t 86.3 s) |
| left_link1_2 vs torso | yes | 11.7 mm | not in the top list |
| left_link2 vs torso | yes | 16.6 mm | 16.4 mm |
| right_link2 vs torso | yes | 50.9 mm | 50.9 mm |
| right_link1_1/_2/_3, left_link1_1 vs torso | yes | 28 to 85 mm | not in the top list |
| left_link7_1 / left_link7_2 vs torso | **no** (checked, robot pair, margin 30 mm) | 73 / 60 mm | 83 / 96 mm |
| right_link7_1 / right_link7_2 vs torso | **no** | 83 / 96 mm | 83 / 96 mm |

Measured joints, every 8th call; 31 torso pairs measured in total (all arm shapes except the shoulder mounts and fixed bodies).

Findings:
- Link 7 vs torso is checked (not exempt). The overlap that the Phase D run showed (−67.6 mm) was in the **sent command** (the filter's own record), not in the measured arm, which stayed at 60 mm or more. So the filter let a command through that the arm did not reach.
- Two torso pairs on the never-close list, left link 1 joint 3 and left link 1 (sweep claim: real gap at least margin + 1 cm for all joint poses), were measured in contact: −1.0 mm (brkW) and −0.1 mm (brkY). The never-close claim for that pair is contradicted by the runs. Because the pair is exempt, nothing pushes it back. Not changed (read-only; for David).

## 5. Integration run brkY (Phase E code, commit ac1e592)

Command: `python experiments/safety_filter_braking_episode.py --no-shelf --name brkY_plain_20261117_040458` (tmux, memory cap 6 GB). Log `data/logs/brkY_plain_20261117_040458.log`. Recording `data/safety_filter/plugin_runs/brkY_plain_20261117_040458.npz`. 6225 filter calls, 84.9 s sim, wall 1039 s, PLAN_FAILED.

| Checklist item | brkY (Phase E) | brkW (Phase D) | Verdict |
|---|---|---|---|
| Hand speed, sent, every tick (cap 0.20) | peak 0.164 m/s; 0 ticks over | peak 0.200; 0 over | pass |
| Hand speed, measured | peak 0.163 m/s | peak 0.292 m/s | pass (was failing) |
| Accel and jerk, free motion (sent) | **16.05 m/s²; 4682 m/s³** (limits 0.5 / 2.0); the peak is the chatter at 68.33 to 68.37 s | 3.28; 538 | **fail, worse** |
| Accel and jerk, braking (sent) | 0.92 m/s²; 169 m/s³ | 1.61; 258 | fail |
| Accel and jerk, measured | 2.53 m/s²; 372 m/s³ (62.3 s) | 4.48; 499 | fail |
| Palm reaches the grasp pose (23 mm) | 19.0 mm at t = 59.8 s | 21.8 mm | pass |
| Beaker reaches the hotplate at ceiling speed (0.05 m/s) | placed (surface gap 0.41 mm); commanded speed 0.033 m/s within 2 cm and 0.030 within 5 mm | not reached | pass (the ceiling was never the limit) |
| Bridge overrides | **1** (t = 76.22 s, the false emergency) | 0 | **fail** |
| False look-ahead failures | **2** errors at 76.22 s, both false (section 6) | 0 | **fail** |
| Overlaps, measured | arm to arm min 66.9 mm (every 5th call); arm to torso min 24.6 mm; left link 1 joint 3 vs torso **−0.11 mm** (listed never-close, section 4) | arm to arm −17.1 mm | arm to arm pass; torso fail (tiny) |
| Emergencies | 1 override at 76.22 s; not correct (section 6) | 0 | **fail** |
| Known: gripper did not stop within 5 s | 0 lines | 0 | not hit |

Pick and place sequence (brkY): OVER (50.1 s), DOWN (54.5 s), IN (58.2 s), CLOSE (62.3 s, gripper closed PASS at 66.0 s), UP (66.0 s), navigate to the hotplate (70.2 s), place: OVER (70.2 s), DOWN (72.7 s), OPEN (74.8 s, opened PASS at 76.2 s), BACK (76.2 s): not reached after 10 s, 303 mm from the waypoint. The orchestrator gave up (no retry, the beaker is no longer in the hand).

Stop latch: the look-ahead stopped the left arm from 76.22 s for 1507 ticks (about 10 s). That is the BACK failure.

## 6. Root cause of the remaining failures

**A. BACK failure: a false emergency holds the arm (fix 1 exposed it).** At 76.22 s the left finger pair's closest-point closing speed was 0.89 m/s. The finger opened 6.4 mm in one tick (about 0.96 m/s of finger motion) while every arm joint stayed put. The emergency reads the closing speed of the closest hand point, and that point moves with the fingers. The emergency then held the left arm at its last sent pose (hard clamp) and the stop latch kept it there for 1507 ticks. The BACK move could not run. Fix 1 (removing the guard) is what made this emergency fire; before fix 1 the same pair would have been ignored while braking. The brief does not say whether finger motion should count, so this is a decision for David (section 7).

**B. Acceleration and jerk spike at 68.3 s (not explained).** In a 0.1 s window (68.33 to 68.37 s) the sent speed of the left arm alternates between about 0.03 and 0.14 m/s on successive ticks. The raw command at that time moves at about 0.02 m/s, and the output lags it by about 49 mm. The braking stop flag is off there, the status is PASS, and the held beaker is 15 to 45 mm from the bench (stop target 15 mm). Replaying the recorded inputs through the current layer did not reproduce the alternation: a synthetic arm with the same lag and no obstacle does not chatter (with or without measured-speed lag). The Phase D run had no such spikes (sent peak 3.3 m/s²). Fix 3 is the prime suspect, because the cap depends on the remaining distance, which depends on the output's previous step; that loop can oscillate when the ramp lags the cap. Not proven. The obstacle feed was not rebuilt in the replay.

**C. Fix 3 jerk on the final approach.** A 5 mm move peaks at 20 m/s³ jerk (limit 2) in its last few ticks, when the stoppable speed falls faster than the ramp can follow; the cap then clips the speed. A 50 cm move reaches the cap with the limit exactly met. The speed never overshoots the target (verified per tick).

**D. Free-motion lag from fix 3.** A steady source at 0.05 m/s is followed with a lag of 9.3 mm (the stopping distance at that speed) and a speed ripple of about 2 mm/s, not exact convergence. In the pick, the output lagged the raw command by up to 49 mm. The brief implies this; the cost is the tracking lag, and it should be a decision.

## 7. Decisions for David

1. **Finger motion in the emergency.** Should the closing speed in the look-ahead and the emergency come only from arm motion (the fingers are moved by the gripper, which the arm command does not move)? Without that, the gripper opening can hold the arm for seconds (brkY).
2. **Fix 3 chatter.** Investigate the 68.3 s alternation before the next run (it is the only large jerk in the run). Options: use the remaining distance after this tick, or limit the cap's change per tick.
3. **Fix 3 jerk on the final approach.** Accept the 20 m/s³ spike (the cap clips the speed), or make the envelope follow the ramp (a longer look-ahead of the decel)?
4. **Tracking lag.** The free-motion cap makes the arm trail a steady source by the stopping distance (9 mm at 0.05 m/s, 49 mm seen in the pick). Accept?
5. **Never-close list.** Left link 1 joint 3 vs torso was measured at −1.0 mm (brkW) and −0.1 mm (brkY), and it is on the never-close list. Re-sweep the torso pairs, or add them back to the checks?
6. **Torso and link 7.** Link 7 vs torso is checked (correct). The sent command still reached −67.6 mm in brkW; its cause is open (not found in this phase).
7. **Known issue, not changed:** the gripper "did not stop within 5 s" was not hit in brkY.
8. **Cost, not optimised (known):** the filter averages 73 ms per call in brkY (107 ms in brkW); cylinder distances are a part of it.

## 8. Not done

- The integration pass (brkY does not pass: BACK failed; accel and jerk limits are broken by the chatter and by fix 3).
- The chatter's root cause (section 6B).
- Changes for decisions 1 to 6 (they are for David).
- Push: not done.
- Worktree kept (its data/ holds the brkV, brkW and brkY recordings and logs).
