# Held-beaker bounce in the moved-layout video (diagnosis only, no code or parameter changes)

Written for: David (controller and video review). Diagnostic only: nothing in src/ or experiments/ controller code was changed.

## 1. Run and files analysed

- Video: `data/videos/demo_moved_final_20261029_123416.mp4` (96.2 s, 2887 frames), sidecar `data/videos/demo_moved_final_20261029_123416.json`. Render log `data/logs/render_final_20261029_123411.log`, settings `data/videos/render_final_20261029_123411_settings.txt` (commit bc14a77, clean tree; ramp 0.5625 s, held compensation on, pedestal 0.15 m, bridge on).
- **The video's own recording was deleted** by `experiments/demo_video.py` after the video was finished (the work folder `work_20261029_123416` is gone). Per-tick data therefore comes from the run with the same code and settings that was recorded 12 minutes earlier: `data/demo_record/blend20_20261029_122238/recording` (commit bc14a77, clean; 13,613 ticks, same as the video's replay; same segment timeline in both logs: PLACE starts at 78.74 s, OPEN at 82.45 s, plan done at 90.15 s). I did not compare video frames to this recording pixel by pixel; the match rests on the tick count, the timeline and the settings above.
- Replay check: the extract replays the recording with `replay_run(video="none")`. The replay reproduced all 13,613 states bit-identically (`data/logs/bounce_extract3_20261029_142924.log`, "replay ok True, mismatch None").
- Baselines (same extract, also replay-verified): `data/demo_record/comp3b_20261028_135510` (compensation on, ramp 0.5625, pedestal 0.15, commit 30e1064a, before the blend of M200) and `data/demo_record/comp2off_20261028_135510` (compensation off, but ramp 1.0 s and pedestal 0.10, so not a controlled comparison).
- Extractor: `experiments/bounce_diag_extract.py` (new, read-only). Plot script: `experiments/bounce_diag_plot.py` (new).
- Outputs (git-ignored, under `data/diag/bounce_20261029/`):
  - `ticks.csv`: all 13,612 ticks, both arms, beaker, gripper, held mass, phase.
  - `window_11400_11900.csv`: the bounce window with derived target and measured velocity, acceleration and jerk.
  - `base_comp3b_20261028_135510.csv`, `base_comp2off_20261028_135510.csv`: the baselines.
  - `bounce_1_23_signals.png`: the plot of section 4.

## 2. Timing: video time, simulation time and tick

- Video = 5.5 s of intro stills (`experiments/demo_video.py` INTRO, wall time) + replay frames at 30 fps. Replay frame k is control tick 5k (150 Hz control, 750 Hz physics), and the run starts at sim t = 1.40 s (tick 0).
- So video time t_v maps to tick = (t_v − 5.5) × 150 + 0 and sim time t_s = 1.40 + (t_v − 5.5). **1:23 = 83.0 s = tick 11,636** (sim 79.0 s).
- Worst bounce: tilt peak **3.41° at tick 11,636 (video 83.07 s)**, 35 ticks (0.23 s) after PLACE starts.

## 3. Segment and tick map (blend20 run)

| tick | video s | sim s | event |
|---|---|---|---|
| 11,151 | 79.84 | 75.73 | PICK UP starts |
| 11,599 | 82.83 | 78.73 | **held compensation switches on** (heldmass 0 → 0.205 kg, torque step) |
| 11,600 | 82.83 | 78.73 | NAV 3/4 starts |
| 11,601 | 82.84 | 78.74 | **PLACE starts**: OVER move, 1.7 s planned, phase-start ramp 85 ticks |
| 11,636 | 83.07 | 79.04 | **worst bounce, tilt peak 3.41°** |
| 11,646–11,648 | 83.1 | 79.0–79.1 | target velocity jump inside OVER (see 5B) |
| 11,682 | 83.38 | 79.3 | second peak 2.76° |
| 11,851–11,858 | 84.5 | 80.4 | OVER → DOWN handover (blend M200), target velocity step |
| 11,881 | 84.71 | 80.4 | 0.80° bump in DOWN |
| 12,158 | 86.55 | 82.45 | OPEN; held compensation off at 12,308 (BACK) |

Other tilt bumps (all `find_peaks` > 0.5°): 11,269 (0.95°), 11,325 (1.36°), 11,413 (0.92°) in PICK UP, **before any held compensation**; 11,636, 11,682, 11,725, 11,766, 11,819 (PLACE OVER); 11,881 (PLACE DOWN).

## 4. Signals around the worst bounce

Full table: `data/diag/bounce_20261029/window_11400_11900.csv` (both arms, 1 row per tick). Key values, left arm, tick by tick around the onset:

| tick | tilt (deg) | L7 sag (mrad) | L1 sag (mrad) | L4 sag (mrad) | L7 measured qd (rad/s) | L7 target velocity (rad/s) | L7 held torque (N m) | L7 actuator force (N m) |
|---|---|---|---|---|---|---|---|---|
| 11,596 | 0.22 | +36.2 | +13.8 | −11.2 | 0.000 | −0.030 | 0 | −0.89 |
| 11,599 | 0.22 | +36.8 | +13.7 | −11.1 | 0.000 | −0.030 | −0.379 (step) | −1.27 |
| 11,604 | 0.45 | +33.8 | +12.9 | −10.6 | −0.147 | 0.000 | −0.379 | −1.17 |
| 11,608 | 0.84 | +29.6 | +11.2 | −9.7 | −0.158 | 0.002 | −0.379 | −1.12 |
| 11,616 | 1.78 | +21.3 | +5.8 | −5.7 | −0.138 | 0.017 | −0.379 | −1.05 |
| 11,624 | 2.70 | +12.7 | −1.1 | +0.5 | −0.111 | 0.051 | −0.379 | −0.98 |
| 11,636 | **3.41** | −2.0 | −12.6 | +12.1 | −0.027 | 0.151 | −0.378 | −0.88 |
| 11,646 | 2.92 | −16.7 | −21.8 | +22.1 | +0.001 | 0.268 (jump) | −0.379 | −0.74 |
| 11,660 | 0.60 | −39.7 | −6.4 | −15.1 | +0.294 | 0.401 | −0.379 | −0.68 |
| 11,682 | 2.76 | −5.8 | +17.4 | −15.1 | +0.130 | −0.032 | −0.378 | −0.92 |

Values from `ticks.csv` (sag = measured − skill target; torque and force from the replayed state). Left-arm target velocity is the one-tick difference of the recorded target, times 150. Note: the L7 sag is measured at the tick start and includes the lag behind a target moving at 0.3–0.4 rad/s after tick 11,640.

Derived quantities (not recorded by the run; computed from the replayed state with `gravity_ff.ArmGravityFeedforward`):

- **Feedforward split** at tick 11,599: arm's own gravity term and held term are separated. Held term (N m): L1 −0.961, L2 −0.121, L3 −0.261, L4 +0.765, L5 +0.011, L6 0.000, L7 −0.379. Gravity-only terms are unchanged by the held switch.
- **Actuator torque identity** checked on this run: actuator force = kp·(target + ff/kp − q) − kv·qd, where kv is the damping term in `biasprm` (2.75 L1, 2.5 L2, 2.0 L3–L4, 0.7 L5, 0.6 L6, 0.5 L7). Residual < 0.2 N m (sampling). Without the ff/kp offset the residual is 8 N m on L1, so the loop adds ff/kp to the recorded target (the recording stores the skill's target, not the applied ctrl).
- **Saturation:** none. Peak forces: L1 −8.4 N m (limit 40), L3 2.1 (limit 27), L7 −1.3 (limit 7). Speeds: target velocity max 0.40 rad/s vs the 0.5 rad/s cap; measured max 0.40.
- **Held mass:** 0 before tick 11,599, 0.205 kg from 11,599 to 12,307; grip force 8–9 N throughout.
- **Beaker vs hand:** the beaker's axis stays at a constant 90.59° ± 0.02° from the hand's tool axis across the whole bounce, and the pad-to-beaker offset moves ~2 mm (brx). The beaker does not slip or twist in the grip. Contacts: 4–8 beaker contacts, normal force sum ≈ 16 N (the two fingers at 8–9 N).
- **Beaker velocity:** linear and angular velocities are in `ticks.csv` (bvx..bwz). Peak wz ≈ −0.58 rad/s at tick ~11,750 (yaw in the grip) after the main bounce.

### Measured vs target jerk (the requested comparison)

- Measured joint jerk on the window: at most 1,200 rad/s³ (L7 at the held step), 200 rad/s³ for the other joints.
- Target jerk: thousands of rad/s³ at the two target discontinuities (11,646–11,648 and 11,853–11,858), where the target velocity itself changes by up to 0.41 rad/s in one tick (L4 at 11,856: target acceleration −61.6 rad/s²). The joints filter most of this, but the bumps follow these jumps (section 5B).

## 5. What initiates the bounce

Ranked by how well the timing and signals support each hypothesis.

### A. Held-object compensation step (strongest for the 1:23 peak)

- **Timing:** the compensation switches on at tick 11,599 as a step (0 → −0.96 N m at L1, −0.38 N m at L7, +0.77 N m at L4; see the table). The beaker tilt starts rising at 11,600 (0.23° → 0.45° at 11,604 → 0.84° at 11,608). The skill's own target velocity is below 0.003 rad/s until 11,607, while the measured L7 velocity reaches −0.16 rad/s by 11,607. The arm moves before the commanded motion does, so the early bounce is not driven by the skill's path.
- **Magnitude:** before the switch the measured joints sit off their targets by the beaker's weight torque over the joint stiffness: L7 +36.6 mrad (= 0.37 N m / kp 10), L1 +13.7 mrad, L4 −11.1 mrad. The held term moves each joint toward its target by the same amount (static offsets −37.9, −13.7, +12.7 mrad). The sign is consistent: the compensation removes the sag. A 37 mrad wrist rotation pitches the hand by about 2°, close to the observed 3.4° beaker tilt at the peak.
- **Baseline:** comp3b (compensation on, same ramp and pedestal, older commit) gives the same timing and shape: held switch at −2 ticks from PLACE start, peak 3.42° at +35 ticks (blend20: 3.41° at +35). The bounce is therefore not from the M200 blend.
- **Counter-evidence to a sole cause:** the comp-off run (ramp 1.0, pedestal 0.10, not matched) has no early wrist motion (L7 velocity 0.000 in the first 0.3 s) and no 3° peak at +35; its 3.09° peak comes at +89 ticks (see B). So the compensation step explains the early timing and the first peak, not the whole bouncing.
- **Overshoot:** the wrist overshoots its target (sag reaches −40 mrad at tick 11,660), so the held step excites an underdamped wrist (kp 10, kv 0.5). Part of the negative sag is tracking lag behind a target moving at ~0.3 rad/s; the two cannot be separated from these logs (see section 6).

### B. Discontinuities in the skill's own target (explains the later bumps)

- **11,646–11,648** (inside OVER, no waypoint logged there): L1 target velocity goes from +0.035 to −0.104 rad/s in two ticks (target acceleration −31, −21 rad/s²); L4 from 0.248 to −0.153 rad/s. The tilt peaks at 2.76° at 11,682, 36 ticks later.
- **11,853–11,858** (OVER → DOWN handover, the M200 blend): L4 target velocity goes from +0.195 to −0.215 rad/s in one tick (acceleration −61.6 rad/s²); L7 from 0.112 to −0.056. This is consistent with the blended cubic starting at non-zero acceleration (for u = 0: target acceleration = (−4 v0 T + 6 Δ)/T², which is not zero, while the previous minimum-jerk ends at zero acceleration). The 0.80° bump in DOWN follows at 11,881.
- **Comp-off run:** its largest target-velocity jump (0.14 rad/s per tick, joint-level) is at +87, and its 3.09° peak is at +90. Same pattern: a target discontinuity a few ticks before the bump, with no compensation.
- **Cause of the jumps is not determined** from these logs. Candidates, none verified: (i) the mink per-tick solve, where collision or velocity constraints can switch on and off (`src/mink_arm.py` `step`, `_collision_limits`); (ii) the sag integrator `C["p"]` (gain 3/s, clip 0.04 m in `src/mpvi_modules.py`), which reacts to the measured pad error that the held step creates; (iii) the blend handover acceleration (5B above). These are internal state and are not logged; see section 6.

### C. Position/velocity controller interaction

- The wrist (L7, kp 10, kv 0.5) is low-damped relative to a 1 N m torque step: overshoot of about 40 mrad before settling. The actuator's damping term is the only derivative action (no separate derivative gain). Supported by the overshoot in the sag, not by a separate test. Integrator windup is not present in the logged state (the skill's sag integrator is in `C`, not logged).

### D. Ramp limiter / duration logic

- **Not supported as initiator.** The phase-start envelope starts at tick 11,601 and the target velocity is below 0.003 rad/s until 11,607; the measured motion leads the target, not the reverse. The ramp does matter later: the target speed grows along the min-jerk curve (0.012 rad/s at 11,615, 0.095 at 11,630), and the bumps in B happen on that ramp.

### E. Beaker grasp or contact dynamics

- **Rejected for this bounce.** The beaker moves rigidly with the hand (constant 90.59° to the hand axis; pad offset moves ~2 mm), contact count and normal force are steady. The beaker's own yaw spin (wz ≈ −0.58 rad/s at ~11,750) comes after the main bounce.

### F. Other mechanisms

- PICK UP bumps (11,269: 0.95°, 11,325: 1.36°, 11,413: 0.92°) happen with no held compensation in the lift. So some bouncing is present before the compensation exists, and it is not explained here. Not analysed further.

### Ranking

1. **A (held-compensation torque step)** initiates the worst bounce's first peak (3.4° at 1:23). Strongest evidence: timing, the joint motion before any command motion, the sag magnitude, and the comp3b replay.
2. **B (skill-target velocity discontinuities)** produce the second and later bumps (2.76° at 11,682, 0.80° at 11,881), and the comp-off run has the same pattern. The origin of the jumps is open.
3. **C (wrist overshoot)** amplifies A; supported only by the sag overshoot.
4. D and E: not supported. F: present but unexplained.

Caveat on the claim: "initiates" is inferred from timing and magnitude, not from a counterfactual run. It is not claimed as proven (section 6).

## 6. What would distinguish the leading hypothesis from the alternatives

- **Counterfactual replay (no re-plan):** replay the blend20 recording with the held term forced to zero from tick 11,599 (`heldmass` zeroed) and the recorded targets unchanged. If A is the initiator, the tilt onset at 11,600–11,608 and the 3.4° first peak should largely disappear; B's later bumps (around 11,646 and 11,853) should remain. Cost: about 80 s replay, no planner, no controller change. This is the smallest controlled test.
- **Instrumented live run (for B):** log per tick the mink velocity constraints active (collision and per-joint limit), the sag integrator `C["p"]` and clip flags, and the blend handover values (`S["blend_v"]`, `v0`, `T`). This would show whether the 11,646 and 11,853 jumps come from the solver constraints, the sag integrator, or the handover. Needs a fresh episode (about 160 s).
- **Damping check (for C):** a replay with the wrist kv raised (no controller change in src/; a simulation-only probe) would show whether the overshoot scales with damping. Lower priority.

## 7. Not recorded (explicit)

- The skill's internal reference, the sag integrator `C["p"]`, the blend state, the ramp envelope value per tick, and mink's active constraints are **not logged**. Their values are not in the recording and were not recomputed. The skill's target is the only command signal available (recorded as `ctrl`).
- The video's own recording was deleted by `demo_video.py`; the analysis uses blend20 (same code and settings; not frame-verified against the video).
- Measured torques are the replayed actuator forces (the servo's own torque), so the actual joint torque includes the damping term; no load-cell or contact torque was recorded.
- The beaker's velocity is at the body frame origin, not at the centre of mass.
- Baseline comparisons are confounded: comp3b differs from blend20 only by the M200 blend (and the earlier commit). comp2off differs by ramp (1.0 vs 0.5625) and pedestal (0.10 vs 0.15); its peak timing is not directly comparable.

## 8. Plots

Figure: `data/diag/bounce_20261029/bounce_1_23_signals.png` (ticks 11,336–11,936; ±2 s around 1:23). Panels: beaker tilt; joint sag (L1, L4, L7); held torque per joint (ff_held); velocity (L7 and L4, target dashed and measured solid). Vertical lines at the held switch (11,599), PLACE start (11,601), the peak (11,636), the target jump (11,646) and the OVER → DOWN handover (11,856). The palette is the first three categorical slots of the reference palette (`dataviz/references/palette.md`). The palette validator could not be run: `node` is not installed on this machine, so the colors are taken from the reference palette as documented, not re-validated.

## 9. Implementation notes (read, not changed)

- Held compensation: `src/gravity_ff.py` `ArmGravityFeedforward.torques` (held term = m g J_z of the pad centre); `apply_offsets` adds torques / kp to the target in the loop (`src/control_loop.py`). Activation: `src/mpvi_modules.py` "held" step (lines ~416–427) and `set_held_mass` on the pick's reading; cleared at pick start (line ~251) and after OPEN (line ~447). The step is applied in one tick, with no ramp (M196 notes say so explicitly).
- Phase-start ramp and blend: `src/mpvi_modules.py` `step` (lines ~434–470) and `step_inner` (lines ~301–380): minimum-jerk law for each move, `blend_radius_m` handover (M200) starting the next move with `v0 = blend_v` and cubic reference (line ~342–370).
- Sag correction: `src/mpvi_modules.py` `step_inner` (`C["p"]`, gain `sag_gain_per_s` 3.0, clip `sag_max_m` 0.04), reset at each skill start. It runs on the measured pad pose, so it reacts to the held step.
- Skill start: `arm.reset(data)` snaps the command to the measured joints unless `skill_start_ramp` (True) carries the last command (`src/mpvi_modules.py` `start`, line ~243–246). With the ramp on, the target does not jump at 11,601 (verified in section 4).
- Mink solve per tick: `src/mink_arm.py` `step` (line ~178–203), velocity cap by the uniform or per-joint limit (not reached in this run).

## 10. Suggested next step (not run)

Run the counterfactual replay of section 6 (held term off from 11,599, recorded targets unchanged). It is the smallest test of hypothesis A and needs no code change. Hypothesis B needs one instrumented episode. Both are proposals for David: no run was started here beyond the read-only extracts listed in section 1.
