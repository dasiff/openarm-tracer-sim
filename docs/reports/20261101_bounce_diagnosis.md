# Beaker bounce diagnosis: root cause analysis and next experiment

**Written for:** David (deciding which mechanism to fix). Analysis of existing experimental data; no new runs proposed here.

**Scope:** Synthesizes evidence from the bounce diagnosis (20261029_1445), closed-loop compensation OFF (20261029_1505), and ramp/delay validation (20261101_1240) to identify the root cause of the beaker tilt oscillation during PLACE and rank the candidate mechanisms.

## Part 1: What the existing data establish

### 1a. The bounce: facts vs hypotheses

**Fact:** The beaker tilts 3.41° at tick +35 (0.23 s) after PLACE starts, settles over ~315 ticks (2.1 s), with secondary peaks at +83 and +167 ticks.

**Fact:** The tilt is NOT caused by the held-object compensation step alone. With compensation OFF in closed loop, the first peak is 3.12° at +68 ticks—still present, just delayed.

**Fact:** A 0.5 s ramp of the held-object torque reduces the early peak from 3.41° to 1.15° and delays onset to +31 ticks (0.2 s). But a new peak of 1.53° appears at +107 ticks, just after release.

**Fact:** The skill's target velocity has discontinuities (jumps > 0.3 rad/s per tick) that precede bumps in tilt: the 11,646–11,648 jump precedes the 2.76° peak at +82; the OVER→DOWN blend at 11,853–11,856 precedes the DOWN bump. These are present with and without compensation.

**Fact:** Wrist joints L5–L7 are the most lightly damped: kv/kp = 0.07–0.70 rad/s per rad error, compared to L1–L2 at ~0.04 and L3–L4 at ~0.03. (The control servo is force = kp·(target − q) − kv·qdot.)

### 1b. The servo and underdamping

The beaker is rigidly held in the hand (constant 90.59° angle to the hand frame). The beaker's angular motion is therefore driven entirely by:
1. Hand orientation changes (the hand Jacobian couples arm joint motion to hand orientation)
2. The gravity and held-object feedforward torques applied to the wrist joints
3. The servo lag on those joints due to their damping ratio

**Damping analysis for L7 (worst-affected):**
- Servo gain: kp = 10 N⋅m/rad, kv = 0.5 N⋅m⋅s/rad
- Motor inertia (armature): 0.01 kg⋅m²
- Effective damping ratio ζ ≈ kv / (2√(kp⋅J)) ≈ 0.5 / (2√0.1) ≈ 0.79
- This is **lightly underdamped** (ζ < 1.0), so the servo overshoots when a torque step is applied.

**The held-object torque step:**
- At tick 11,599: the held mass reading switches from 0 to 0.205 kg
- Torque applied: L7 receives −0.379 N m (a step applied in one control tick)
- Equilibrium shift: 0.379 N m / 10 kp = 37.9 mrad = 2.2° (the hand rotates to a new servo equilibrium)
- Measured sag (position error) reaches −40 mrad (overshoot beyond equilibrium)
- Expected for underdamped step response: yes, consistent

### 1c. Timing and causality: what drives the tilt?

**Pre-release motion sequence** (ticks 11,599–11,689):

| event | tick | L7 tgt velocity | L7 measured velocity | tilt (deg) | evidence |
|-------|------|-----------------|----------------------|-----------|----------|
| held switches on | 11,599 | unchanged | 0 → −0.147 | 0.22 → 0.45 | arm moves before command changes |
| onset threshold (+0.1°) | 11,603 | < 0.003 | −0.147 | 0.34 | motion leads command by 4 ticks |
| peak | 11,636 | 0.151 (increased) | −0.027 (settling) | 3.41 | occurs after wrist reversal direction |
| target jump | 11,646 | 0.268 (jump) | +0.001 | 2.92 | command catching up to measured |
| second peak | 11,682 | −0.032 | +0.130 | 2.76 | driven by renewed motion |

**Interpretation:**
1. At 11,599, the compensation torque is applied as a step.
2. The wrist servo is underdamped, so it overshoots backward (L7 velocity goes negative).
3. The skill's target stays nearly zero until 11,607, so the measured motion **precedes and drives** the initial tilt peak.
4. The servo's damping is too low to settle before the next input (the skill's increasing target velocity).
5. At 11,646, a target discontinuity causes a catch-up phase, feeding energy into a second oscillation.

### 1d. Trajectory discontinuities: a separate driver

**Discontinuities present in both compensation ON and OFF:**
- Tick 11,646–11,648: target velocity jump 0.35 rad/s (measured within one tick on L4)
- Tick 11,853–11,858 (blend handover): target velocity jump 0.41 rad/s (ON) / 0.31 rad/s (OFF)
- The blend is the largest and happens at the OVER→DOWN transition
- Down bump (0.80° ON, 1.27° OFF) follows about 25 ticks later

**Source of discontinuities:** Not determined from available logs. Candidates per earlier analysis:
- Mink solver constraints switching (collision or velocity limits)
- Sag integrator reacting to the held step's pad error
- Blend handover cubic starting with non-zero acceleration

The point: these are **not controller bugs but emergent from the planning and constraint solver**. They are smaller than the first peak (0.31–0.41 rad/s vs the compensation step's transient) but persistent.

### 1e. The post-release peak: unexplained

In the ramp run, a 1.53° peak appears at +107 ticks (just after RELEASE at +90 ticks). The base run doesn't show this clearly because the early peak (3.41°) dominates the plot.

**Hypotheses:**
- Wrist spring-back after release: the ramp reduces the initial motion, storing less energy, but the system still recoils as the move ends.
- IK solution change at the handover: the PLACE→OVER→DOWN transitions may alter the hand orientation target, causing a step in the hand motion that the wrist must follow.

Not resolved by existing data.

## Part 2: Hypotheses ranked by evidence

### Rank 1: Underdamped wrist servo response to held-object torque step

**Evidence for:**
- Timing: wrist motion (L7 velocity) starts immediately after the held step (11,600), before the command target changes
- Magnitude: the sag overshoot (−40 mrad) is expected for a 37 mrad step with ζ ≈ 0.79
- Gain: the beaker tilt peak (3.41°) is proportional to the wrist sag (37 mrad ≈ 2°, plus Jacobian coupling)
- Baseline match: the comp3b run (same ramp, pedestal, compensation on) shows the same peak (3.42°)
- Ramp effect: ramping the torque step over 0.5 s delays onset from +1 to +31 ticks and halves the early peak
- Wrist damping comparison: L7 (kv 0.5) is the lowest-damped joint; L1 (kv 2.75) has no observable early peak

**Evidence against:**
- The compensation step does not explain the second peak (2.76° at +83) or the settling time (315 ticks)
- With compensation OFF, the peak is only 21% smaller (3.12° vs 3.41°), not "gone"—the compensation does not cause the bounce, it triggers it

**Conclusion:** The underdamped wrist response is the **primary trigger** of the first peak. Increasing damping on L7 (and possibly L5–L6) would reduce the peak amplitude and onset speed. Whether it would eliminate settling-time oscillations (ranks 2–4 below) is unknown.

### Rank 2: Target velocity discontinuities driving secondary peaks and settling

**Evidence for:**
- Timing: target jumps at 11,646 (0.35 rad/s) and 11,853 (0.41 rad/s) precede peaks/bumps by 35 and 25 ticks
- Magnitude: the jumps are large (0.3–0.4 rad/s) and cause measured jerk > 6000 rad/s³ in both ON and OFF runs
- Presence in both: the comp-OFF run has the same jump pattern and similar bumps (1.27° DOWN vs 0.80° ON), so they are not caused by compensation
- Measured vs target: the joint filtering attenuates the target jerk (6956 rad/s³ target → 398 rad/s³ measured), but not completely

**Evidence against:**
- The jumps are **not in the skill's own target** (the recorded ctrl). They arise from the IK solver or servo updating between recorded ticks.
- The source—mink constraints, sag integrator, or blend state—has not been traced
- Increasing wrist damping will not eliminate these jumps, only slow the response to them

**Conclusion:** Trajectory discontinuities are a **secondary driver** of oscillations during the move. They are not caused by low wrist damping and may require changes to the planner, blend logic, or sag integrator. Solving rank 1 (damping) will not solve rank 2.

### Rank 3: IK solution coupling and hand motion rebound

**Evidence for:**
- The ramp run's post-release peak (1.53°) suggests the wrist stores energy during the move and recoils as motion ends
- The blend handover at 11,853 changes the active move and may re-target the hand orientation, causing a step
- The sag integrator (C["p"]) responds to the pad's measured error and can drive transients

**Evidence against:**
- No direct evidence of hand orientation changes or sag integrator saturation in the logs (the integrator is not logged)
- The closed-loop instrumentation logged blend_v but did not trace its effect on hand pose

**Conclusion:** IK coupling and rebound are **plausible but unconfirmed**. They may explain settling time and the post-release peak, but cannot be ruled in or out without additional logging.

### Rank 4: Velocity limiting and servo saturation

**Evidence for:**
- 64–58 ticks with velocity binding (commanded speed hitting the 0.5 rad/s cap)
- The joint-limit slack is low (0.033–0.034 rad) in the DOWN phase

**Evidence against:**
- Measured peak speed is 0.36–0.40 rad/s, not at the limit
- The bounces are smooth curves, not clipped ramps (saturation would appear as linear portions in the velocity profile)
- Compensation OFF does not reduce the number of velocity-limit ticks (64 → 58), so limiting is not the main driver

**Conclusion:** Velocity limiting is **not a primary cause** but may **contribute to** the settling time by capping the recovery speed.

### Rank 5: Beaker grasp or contact dynamics

**Evidence for:**
- None. The beaker stays at constant angle (90.59° ± 0.02°) to the hand frame; contact normal force is steady; grip force is constant.

**Conclusion:** The beaker does **not slip or rotate in the grip**. The tilt is entirely due to the hand's motion, not grasp dynamics.

## Part 3: The single most informative next experiment

**Goal:** Determine whether wrist damping is sufficient to remove the first peak (rank 1), or whether the first peak is an irreducible consequence of the control architecture.

**Experiment:** One closed-loop episode with **L7 damping increased from 0.5 to 1.5 N⋅m⋅s/rad** (a 3x increase), keeping all other parameters (held ramp 0.5625 s, pedestal 0.15 m, compensation ON) at current values. Do not change production code; monkeypatch the servo gain in `src/arm_actuators.py` or the control loop for this run only.

**Measurements (same as base run and ramp run):**
- Beaker tilt per tick, with peaks (first pre-release, second, DOWN, post-release if any)
- L7 sag (position error) and velocity, target vs measured
- Settle time (ticks to last > 0.5°)
- Track error on all joints during OVER and DOWN

**Expected outcomes:**
1. **Damping is the main cause (Rank 1 confirmed):** First peak drops > 50%, onset is sharper (less undershoot), settling is faster. Second peak may increase slightly (faster energy transfer). Down bump unchanged (driven by rank 2). Post-release peak reduced or gone.
2. **Damping is insufficient (other ranks dominate):** First peak drops < 30%, or doesn't drop at all. Settling time unchanged. Second peak and DOWN bump large.
3. **Over-damping (ζ becomes > 1):** First peak drops to near zero, but settling time and overall oscillation increase (sluggish response). Track error increases.

**Why this is the most informative:**
- It directly tests rank 1 in isolation (no code changes, no new planning, no trajectory edits).
- It takes ~160 s (same as a demo episode).
- The result will show whether the first peak is a tuning problem (damping) or an architecture problem (rank 2 or 3).
- If damping doesn't help, the next step is to investigate the trajectory discontinuities (rank 2), which requires logging the mink solver state or the sag integrator per tick.

**Why not others:**
- Increasing ramp length: already tested (0.5625 and 1.0 s); doesn't eliminate the peak, only delays it and causes DOWN failure.
- Delaying OVER motion: already tested (0.6 s delay); makes settling worse.
- Changing the blend or sag integrator: requires code changes (David's decision) and doesn't isolate the mechanism.
- Logging IK solutions: informative but expensive (requires planner instrumentation); this damping test answers the immediate question.

## Part 4: What remains unknown

1. **The source of target discontinuities** (rank 2): Which part of the planner/solver creates the 11,646 and 11,853 jumps? Is it the blend cubic, mink constraints, or sag integrator?
2. **The post-release peak** (ramp run, 1.53° at +107): Why does the ramp run show a second peak after the base run settles? Is it wrist spring-back or an IK side effect?
3. **Whether rank 2 alone causes settling time:** If damping is fixed (rank 1 solved) but the discontinuities remain, will settling still take 315 ticks?
4. **The interaction between ranks 1 and 2:** A damped servo responding to a discontinuous target may ring. Solving one without the other might not improve the overall bounce.

## Summary table

| rank | mechanism | evidence | fix | cost | priority |
|------|-----------|----------|-----|------|----------|
| 1 | wrist underdamping | strong: timing, magnitude, ramp effect | increase L7 kv | low (one param) | test first with damping run |
| 2 | target discontinuities | strong: timing, magnitude, rank-2-only peaks | investigate mink/sag/blend | medium (logging) | after rank 1 |
| 3 | IK rebound/coupling | plausible: post-release peak | (unclear, needs rank-2 investigation) | high (planner change) | after rank 2 |
| 4 | velocity limiting | weak: not saturated, not reduced by comp OFF | (unlikely to help) | low | skip |
| 5 | grasp dynamics | none | (not the issue) | — | skip |

**Recommendation (for David):**
Run the L7 damping experiment (3x kv: 0.5 → 1.5). The result will tell us whether to prioritize servo tuning (if rank 1 is dominant) or trajectory analysis (if rank 2 dominates). Both paths are feasible; this test selects the right one.
