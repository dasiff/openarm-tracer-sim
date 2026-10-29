# Counterfactual replay: held-object compensation off from tick 11,599 (diagnostic, no controller change)

Written for: David (controller review). Diagnostic only: no file in src/, experiments/ or models/ was changed, and the recording on disk was not written.

## Summary

- Removing the held-object compensation from tick 11,599 (recorded skill targets unchanged) **removes the early wrist motion and the first 3.41° peak at 1:23**, but **does not remove the bounce**. The beaker tilt goes **higher, to 5.73° at tick 11,680** (about 0.3 s later than the original 1:23 peak), and the later bumps are larger too.
- So the report's ranking (`bounce_diagnosis.md`, section 5A: "the held-compensation step initiates the 3.4° peak") is **not supported** by this run. The step does start the early wrist motion about 40 ticks sooner (onset 11,603 with it, 11,643 without it). But the bounce itself is present without the step, and the compensation looks like it reduces the amplitude in this window. This is confounded by the open-loop targets (section 6, limitation 1), so it is not proof either way.

## 1. What was run

- Recording: `data/demo_record/blend20_20261029_122238/recording` (13,613 ticks, commit bc14a77 settings: ramp 0.5625 s, held compensation on, pedestal 0.15 m, bridge on).
- Counterfactual: `heldmass` zeroed in memory from tick 11,599 to the end (709 nonzero entries before, 0 after). All recorded skill targets (`ctrl`), base velocity, grip force and the initial state are unchanged. Physics is re-simulated with the same `src/replay.py` path.
- Script: `/tmp/claude-1000/-home-vboxuser-projects-openarm-tracer-sim/a5679fbd-7e1d-4629-8216-57b29dcf71aa/scratchpad/counterfactual.py` (scratch, not committed). It wraps `load_recording` in `src/run_record.py` and `src/replay.py` so that `replay_run(..., verify=False)` gets the zeroed array. The extract is `experiments/bounce_diag_extract.py` (committed in 35fdb2a), unchanged.
- Output: `data/diag/bounce_20261029/counterfactual_ticks.csv` (13,612 rows, same columns as `ticks.csv`). Log: `data/logs/counterfactual_20261029_144013.log` (exit 0, 1 min 28 s, peak RSS 2.7 GB).
- **Who ran it:** the run was already in progress in another session (tmux `cf`, started 14:40:13). I did not start a second MuJoCo process. I waited for that run to finish, then analysed its output.
- Baseline: the original blend20 replay, `data/diag/bounce_20261029/ticks.csv` (same extract, replay verified bit-identical in the earlier step).
- Comparison: `data/diag/bounce_20261029/compare_cf.py`, output `data/logs/cf_compare_20261029_1443.log`, window CSV `data/diag/bounce_20261029/cf_window_11599_11900.csv`.

## 2. Verification

**Recording unchanged.** SHA-256 of all 10 files under `blend20_20261029_122238/` taken before and after the run: identical. Modification times are the original 12:24–12:26.

**The held term is really gone from the control path, not just the label.** The `ff_held` column is computed from the same zeroed array, so it is circular and not used as evidence. The independent check uses the physics: the applied actuator force (`force` column, the servo's own torque) against the identity `force = kp (tgt + ff_grav/kp − q) − kv qd`, where `ff_grav` is the arm's own gravity (no held mass). `kp` was fitted from the pre-compensation stretch (ticks 11,000–11,500) of the original run, since the gains are not in the XML (fitted L1 70.7, L4 58.0, L7 9.71). `kv` from the report.

Max |force − prediction| over ticks 11,599–11,719:

| joint | original, grav-only prediction | original, grav + held prediction | counterfactual, grav-only prediction |
|---|---|---|---|
| L1 | 1.01 N m | 0.07 N m | 0.07 N m |
| L4 | 0.78 N m | 0.02 N m | 0.04 N m |
| L7 | 0.38 N m | 0.006 N m | 0.013 N m |

The counterfactual's applied torque matches the arm's gravity alone within 0.07 N m. The original matches gravity plus the held term. So the held torque is absent from the physics in the counterfactual.

**Targets identical.** The max absolute difference in `tgt` between the two runs is 0.0 over all ticks and all joints.

**Replay verification off.** `verify=False` was used on purpose, so the per-tick state hashes were not compared. The first divergence from the original is at tick 11,602 (tilt differs by more than 0.05°). The model hash check in `replay_run` still ran and raised no error.

## 3. Results

### 3.1 Peak beaker tilt

| run | peak in 11,599–11,700 | tick (sim s) | onset (+0.1° over 0.22° baseline) |
|---|---|---|---|
| original | 3.41° | 11,636 (78.97) | 11,603 |
| counterfactual | **5.73°** | 11,680 (79.27) | 11,643 |

Local peaks (prominence 0.2°) in 11,599–11,900: original 3.41 (11,636), 2.76 (11,682), 1.20 (11,766), 1.13 (11,819); counterfactual 5.73 (11,680), 1.31 (11,728), 1.52 (11,819).

### 3.2 Joints 1, 4 and 7 (sag = measured − skill target, mrad; qd in rad/s)

| tick | orig L1 sag / qd | cf L1 sag / qd | orig L4 sag / qd | cf L4 sag / qd | orig L7 sag / qd | cf L7 sag / qd |
|---|---|---|---|---|---|---|
| 11,599 (switch) | 13.7 / 0.000 | 13.7 / 0.000 | −11.1 / 0.000 | −11.1 / 0.000 | 36.8 / 0.000 | 36.8 / 0.000 |
| 11,604 | 12.9 / −0.047 | 13.7 / 0.000 | −10.6 / 0.022 | −11.0 / 0.000 | 33.8 / −0.147 | 37.1 / 0.000 |
| 11,616 | 5.8 / −0.100 | 13.3 / 0.000 | −5.7 / 0.081 | −10.5 / −0.001 | 21.3 / −0.138 | 36.6 / 0.000 |
| 11,624 | −1.1 / −0.079 | 11.4 / 0.000 | 0.6 / 0.075 | −8.8 / −0.008 | 12.7 / −0.111 | 34.7 / 0.000 |
| 11,636 (orig peak) | −12.6 / 0.007 | 3.5 / 0.027 | 12.1 / −0.006 | −2.4 / −0.051 | −2.0 / −0.027 | 26.2 / 0.000 |
| 11,646 (target jump) | −21.8 / 0.110 | −5.1 / 0.107 | 22.1 / −0.120 | 5.8 / −0.124 | −16.7 / 0.001 | 11.8 / 0.003 |
| 11,660 | −6.4 / 0.172 | 8.7 / 0.151 | −15.1 / −0.057 | −29.4 / −0.027 | −39.7 / 0.294 | −5.1 / 0.350 |
| 11,682 | 17.4 / −0.036 | 30.2 / −0.042 | −15.1 / 0.176 | −26.6 / 0.181 | −5.8 / 0.130 | 30.8 / 0.127 |
| 11,700 | 12.2 / −0.263 | 25.1 / −0.256 | −0.6 / 0.079 | −12.8 / 0.068 | 3.1 / 0.000 | 39.6 / 0.000 |

Over 11,599–11,700:

- Sag range, original: L1 −22.2 to +18.8 mrad, L4 −22.5 to +22.3, L7 −39.8 to +36.8.
- Sag range, counterfactual: L1 −5.6 to +31.5, L4 −35.6 to +6.1, L7 −5.1 to +39.6.
- Max |qd|, original: L1 0.263, L4 0.192, L7 0.390 rad/s. Counterfactual: L1 0.256, L4 0.193, L7 0.415 rad/s.

Reading: without the step, the wrist (L7) holds its 37 mrad static offset until about 11,624, with zero velocity. With the step it starts moving at 11,604 and overshoots to −40 mrad. From about 11,646 the counterfactual's joints move with the target, and the large offset carries into a bigger swing of L1 and L4 (L1 reaches +30 mrad and L4 −29 mrad by 11,682 and 11,660, against +17 and −15 in the original).

### 3.3 First oscillation: onset and peak

- Original: onset 11,603 (wrist moving from 11,604), peak 3.41° at 11,636.
- Counterfactual: onset 11,643, peak 5.73° at 11,680.
- The counterfactual's first oscillation is therefore not the held-step oscillation at all. It starts after the OVER move's target has started to move (measured L7 velocity reaches 0.35 rad/s by 11,660).

### 3.4 Target-velocity discontinuity at 11,646

- Present in both runs (targets identical). Largest per-tick target velocity change over 11,640–11,655: L4 0.355 rad/s, L1 0.244 rad/s. (`bounce_diagnosis.md` quotes larger values because it used different windows; these are the figures from this comparison script.)
- **The bounce after it remains.** Original: peak 2.76° at 11,682. Counterfactual: peak 5.73° at 11,680, the same window.

### 3.5 OVER → DOWN handover at 11,851–11,858

- Present in both runs. L4 target velocity goes 0.195 → −0.215 → −0.489 rad/s per tick (identical in both).
- **The bump remains and is larger.** Original: 0.80° at 11,881. Counterfactual: 1.12° at 11,881. The counterfactual tilt baseline is also higher before the handover (about 1.0° against 0.6° in the original), so part of the difference is the carried-over offset from the earlier swing, not the handover alone.

## 4. Interpretation

1. **The held step causes the early wrist motion.** The counterfactual wrist does not move at the switch, and the original does (L7 velocity −0.147 rad/s by 11,604). This confirms the timing part of hypothesis A.
2. **The held step does not cause the bounce.** Without it the arm still swings, with a larger peak 44 ticks later. The swing is driven by the OVER move's target, acting on joints that now carry the 37 mrad static offset. The compensation looks like it damps this window (3.4° against 5.7°).
3. **This is not a clean answer to "is the compensation the cause."** The recorded targets come from a run in which the skill reacted to the held step (see limitation 1). The real no-compensation controller would have a different target path.

## 5. Recommendation on the report

Section 5A of `docs/reports/20261029_1445_bounce_diagnosis.md` should be revised: the held step is a confirmed cause of the early wrist motion, but not a demonstrated initiator of the bounce, and the counterfactual points the other way for the first peak. Hypothesis B (target discontinuities) is still the best-supported source of the later bumps, and this run keeps them. I have not edited that report (diagnosis report is David's to approve).

## 6. Limitations of the replay counterfactual

1. **Open-loop targets (the main one).** The recorded `ctrl` is the skill's target, and the skill responds to the measured state. In the original run the sag integrator `C["p"]` (gain 3/s, clip 40 mm, `src/mpvi_modules.py`) reacted to the held step's pad error. With compensation off, a real controller would have integrated the 37 mrad static L7 offset out of its target, and the planner timing could differ. This replay keeps the targets, so the arm sits off its target for about 25 ticks, which a real run would not. The result is therefore a test of the physics ("what does this target path do if the held torque is absent"), not of the full controller.
2. **Physics is re-simulated** from the recorded initial state, with the same ctrl, base velocity and grip force. Arm and beaker motion are recomputed, and the feedforward change propagates. This is the part the replay can do faithfully.
3. **Mink solve, collision and the velocity cap are not re-run.** Only the resulting target is recorded, so solver constraints that would differ with a different state are fixed here.
4. **No state hash check** after the divergence (`verify=False`). The run is deterministic, but the states are not compared with a live run, because none exists for this case.
5. **Held mass removed from the feedforward only.** The beaker's physical mass stays in the simulation (correct for this question). The recorded grip force is unchanged, so the grip itself is not modelled differently.
6. **Single run, one recording.** The kp values used in the force check are fitted from the same run (section 2), not read from the model.

## 7. Smallest next experiment (proposed, not run)

A closed-loop live episode with the held step disabled: a scratch launcher (no change to `src/`) that replaces `gravity_ff.set_held_mass` with a no-op at the held step, then runs the same planner from the same seed. About 160 s, one MuJoCo process, output to a log under `data/logs/`. This lets the sag integrator and the planner respond, so it answers the open-loop question. It needs David's go-ahead because it is a live run of a few minutes.

## 8. Files

- `data/diag/bounce_20261029/counterfactual_ticks.csv` (git-ignored): per-tick extract of the counterfactual.
- `data/diag/bounce_20261029/compare_cf.py` (git-ignored): the comparison script used here.
- `data/diag/bounce_20261029/cf_window_11599_11900.csv` (git-ignored): tilt and L7 sag per tick, both runs.
- `data/logs/counterfactual_20261029_144013.log`: replay log.
- `data/logs/cf_compare_20261029_1443.log`: comparison output.
