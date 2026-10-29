# Why the solver demands ~1.6 rad/s on L3 at the start of PLACE OVER (investigation, no change)

Written for: David (controller review). Investigation only. No timing, IK cost, gain, velocity limit or production source was changed. Nothing pushed.

## Summary

- **The reference is not aggressive.** It starts at rest, with the measured pad exactly on it (0.2 mm at tick 11,602), and its speed is 0.0039 m/s at tick 11,610 and 0.037 m/s at 11,630.
- **The large L3 demand comes from the solver's starting point, not from the target.** The solver starts from the **commanded** joints (`q_cmd`), but the reference starts from the **measured** pad. At tick 11,602 the pad computed from the commanded joints is **18.3 mm and 3.6° away** from the target. The command leads the measured arm by 0.003–0.036 rad per joint. The solver corrects that offset, and L3 gets most of the correction.
- **Posture is not the L3 driver.** Its error is large, but its weight is 0.01 and its effect is in the null space. Removing it changes L1 and L4 but leaves L3 nearly the same (1.71 vs 1.64 rad/s).
- **Collision limits and joint limits do not matter here.** Removing either leaves the solution identical to the digit.
- **The phase-start ramp hides most of the demand.** The commanded speed stays below 0.02 rad/s until tick 11,616, because the minimum-jerk ramp (0.5625 s) multiplies the solver's speed. The ramp is the reason the first ~14 ticks show almost no motion.
- **Ordering of the first changes:** the beaker tilt starts moving (+0.05° at 11,602, +0.2° at 11,604) before any commanded motion (commanded speed > 0.02 rad/s at 11,616). Pad-to-reference error grows from 11,605. Section 5 has the numbers and a caveat.

## 1. Commanded vs actual pad at tick 11,602

Base run, production code, from the decomposition dump (`data/diag/l3_demand_20261029/dump_20261029_170000/dump.json`), read-only:

| tick | target (measured pad at the phase start) | pad computed from the commanded joints | distance |
|---|---|---|---|
| 11,602 | measured pad | the solver's own pad | **18.3 mm, 3.6°** |
| 11,610 | | | 18.5 mm, 3.6° |
| 11,630 | | | 15.0 mm, 3.2° |
| 11,640 | | | 5.2 mm, 1.7° |

The reference (`ref_*` in `instr_ticks.csv`) and the measured pad agree at tick 11,602 (0.2 mm, `src/mpvi_modules.py` `step_inner`: `p0 = arm.pad_pose(data)` at the phase start). The mismatch is between the solver's start (`q_cmd`) and the measured joints:

- `q_cmd − q_meas` at 11,602 (rad): [−0.013, −0.003, −0.004, +0.011, −0.008, +0.008, −0.036] for L1..L7.
- `MinkArm.step` sets the configuration from `q_cmd` (`src/mink_arm.py`, `step()`: `q[self.qadr] = self.q_cmd`), not from the measured joints. So the frame error is FK(`q_cmd`) − target.
- The sag integrator is zero at 11,602 (`sag_p` = 0), so the offset is not the sag term.

This is the offset that the reference's measured start hides. The arm's commanded joints lead its measured joints by about a centimetre (typical PD tracking error under the beaker's load, with the held-object compensation on).

## 2. Commanded Cartesian velocity and acceleration at the start

- Reference speed (m/s), base run: 0.0001 at 11,603; 0.0007 at 11,605; 0.0039 at 11,610; 0.0169 at 11,620; 0.0367 at 11,630.
- The move is min-jerk (`src/rate_limit.py`), so velocity and acceleration are zero at its start. The reference is smooth; its start is not a step.
- The commanded position (`ref + sag`) is the reference plus a sag term that is zero at the start.

## 3. IK terms (decomposition at the same inputs)

The dump re-solves each tick with terms removed; the production solve is unchanged. Joint velocities in rad/s, L1..L7, tick 11,602:

| solve | L1 | L2 | L3 | L4 | L5 | L6 | L7 |
|---|---:|---:|---:|---:|---:|---:|---:|
| production, no velocity limit (the solver's own) | 2.49 | −0.36 | **1.64** | −0.69 | 0.20 | −0.08 | 5.59 |
| frame task only (pad position and orientation) | 1.85 | −0.32 | **1.71** | −1.87 | 0.16 | −0.47 | 5.05 |
| frame position only (orientation cost 0) | 2.59 | 2.30 | 2.40 | 0.81 | 3.89 | 7.35 | 7.10 |
| posture task only | 10.51 | 0.93 | 2.02 | 12.33 | 3.93 | 6.01 | 2.80 |
| without collision limits | 2.49 | −0.36 | 1.64 | −0.69 | 0.20 | −0.08 | 5.59 |
| without joint limit | 2.49 | −0.36 | 1.64 | −0.69 | 0.20 | −0.08 | 5.59 |

Reading:

- **Frame task alone explains L3.** The frame-only solve gives 1.71 rad/s on L3, against 1.64 for the full solve.
- **Posture is weak but not zero for the other joints.** Posture alone would demand 10–12 rad/s on L1 and L4, because the arm starts far from its home pose (home L4 = 2.20 rad, measured 1.29 rad). With cost 0.01, its effect in the full solve is to shift L1 by about +0.6 and L4 by about +1.2 rad/s, while L3 barely moves. So the posture is not the reason L3 is fast.
- **Orientation matters for L4, L6 and L7.** The 3.6° rotation residual is what makes the frame-only and position-only solutions differ on L4.
- **Collision and joint limits are inactive.** Removing either leaves the solution unchanged.
- The solver is doing what the residual asks: a 1.8 cm position and 3.6° rotation correction in one tick. It is not choosing an extreme posture.

## 4. Verdict

Of the three options in the prompt:

1. **Is the target moving too aggressively?** No. The reference starts at rest and is smooth; its speed is 0.004 m/s at tick 11,610.
2. **Is the arm starting in a poor posture?** Partly, but not for L3. The posture term is far from home, but its weight is small and it mostly affects L1 and L4. L3's demand is from the frame residual.
3. **Is the IK choosing an unnecessarily extreme joint movement?** No. Given a 1.8 cm, 3.6° residual, the solver's joint velocity is the expected response.

**The actual cause is a mismatch of starting points.** The place reference starts from the measured pad (`arm.pad_pose(data)` at the phase start), while the solver's configuration starts from the commanded joints (`q_cmd`). The two differ by about 1 cm and 3.6°, so the solver's first solution corrects the commanded pose toward the measured pad. L3 gets most of the correction.

Two facts about the existing code bear on this:

- The phase-start ramp (`phase_start_ramp_s` = 0.5625 s, M198/M199) multiplies the solver's speed by the minimum-jerk envelope. The commanded speed is therefore below 0.02 rad/s until tick 11,616, so the first ~14 ticks show very little of the 1.6 rad/s demand. The demand is real, but the ramp holds it back.
- A flag already exists for the opposite choice: `restart_from_commanded` (M195, `src/task_config.py:427`). It starts the reference from the commanded pad. The report `docs/reports/20261028_0237_restart_from_commanded.md` says that version was measured worse (the hand rises 140 mm and the beaker tilts 21° in PLACE OVER). I have not re-tested it, and I have not changed it.

## 5. What changes first (base run, OVER)

First tick each signal passes a threshold (150 Hz):

| signal | first tick |
|---|---:|
| measured joint speed > 0.02 rad/s (any joint) | 11,600 (before OVER: the held-compensation step's settling) |
| beaker tilt +0.05° over the pre-OVER value | 11,602 |
| beaker tilt +0.2° over the pre-OVER value | 11,604 |
| pad-to-reference error > 1 mm | 11,605 |
| pad-to-reference error > 5 mm | 11,612 |
| commanded joint speed > 0.02 rad/s | 11,616 |
| commanded joint speed > 0.1 rad/s | 11,629 |

Interpretation, with a caveat:

- The beaker's tilt starts to change about 14 ticks before the arm's commanded motion becomes visible. The earliest change follows the held-compensation step at tick 11,599 (the weight reading). The closed-loop report `docs/reports/20261029_1505_closed_loop_held_off.md` found that this step starts the early wrist motion (onset +1 tick with compensation, +41 without).
- So for the OVER segment, the first change is the beaker and the measured arm settling after the held step, not the place's commanded motion. The commanded motion (ramped) follows about 14 ticks later, and the pad's growing error from the reference follows that.
- Caveat: the tilt change of 0.05° at 11,602 is small, and the plot (`data/diag/l3_demand_20261029/over_start_base.png`) shows it clearly only after 11,605. The ordering is measured, but its cause is not separated by this data. A test would be to hold the arm still through the held step and check whether the tilt rise follows.

## 6. Plots

`data/diag/l3_demand_20261029/over_start_base.png` (base run, 11,598–11,700): commanded vs measured joint positions and velocities, the solver's unconstrained velocity for L1, L3 and L5, pad minus reference (x, y, z), and the beaker tilt. The 11,602 start and the 11,658 release are marked.

Two things in the plot: the commanded and measured joints diverge by about 0.03–0.05 rad on L2 before tick 11,640 (the measured L2 lags the command), and the pad drifts about 17 mm above the flat reference in z over the first 30 ticks, which matches the frame residual above.

## 7. Limitations

- One run (the production code, verified: the joint commands match the recording on all ticks). The dump is at four ticks only (11,602, 11,610, 11,630, 11,640); the between-tick behaviour comes from the per-tick logs.
- The decomposition re-solves the same inputs with terms removed. It does not change the simulation. Whether a different configuration would change the production trajectory is a separate question, and it was not tested.
- The cause of the 1 cm command-vs-measured offset at the start of OVER was not separated: it could be PD tracking under the beaker's load, the held-compensation torque step, or the start of the arm's own motion. The data here does not separate them.
- The 0.05° tilt rise at 11,602 is below the report's 0.1° onset threshold. Its cause is not established.

## 8. Recommended next steps (not implemented; need David's decision)

1. **Find out why `q_cmd` leads the measured joints by ~1 cm at the start of OVER.** Log the joint tracking error (`q_cmd − q_meas`) and the applied torques (arm gravity and held-object compensation, separately) through the held step at 11,599–11,610. That separates the candidate causes.
2. **Decide the start point for a waypoint move.** Options: start from the measured pad (the current M195 default, which leaves the 1 cm offset in the solver), or start from the commanded pad (`restart_from_commanded`, measured worse on 2026-10-28, but its interaction with the held-compensation step was not tested). This is a trajectory decision for David.
3. Do not change the ramp, costs or velocity limits on this evidence.

## 9. Files

- `data/diag/l3_demand_20261029/run_dump.py`: read-only decomposition launcher (production code).
- `data/diag/l3_demand_20261029/dump_20261029_170000/`: recording, episode, `dump.json` (the decomposition at the four ticks), `launch.json`.
- `data/diag/l3_demand_20261029/plot_over.py`, `over_start_base.png`: the OVER plots.
- `data/logs/l3dump_20261029_170000.log`: the run log.
- This report, and `docs/progress.md` (committed locally, not pushed).

Tracked files: no change to `src/`, `experiments/`, `models/` or `src/task_config.py`. `git status` shows only the pre-existing untracked `deps/AgileX_Tracer`.
