# Closed-loop run with held-object compensation off (diagnostic, no source change)

Written for: David (controller review). Scratch experiment only: no tracked file changed, nothing pushed.

## Summary

- A live closed-loop rerun with held-object compensation disabled still bounces, with a peak of 3.12° (ON: 3.41°). The compensation is not what causes the bounce.
- The open-loop replay (`20261029_1443_counterfactual_held_off.md`) reported 5.73° with compensation off. **That number does not hold in closed loop.** The replay kept the recorded skill targets, which the sag integrator had built with compensation on. The closed-loop run lets the skill respond, and the bounce is smaller.
- With compensation off the first oscillation starts about 40 ticks later (onset +41 vs +1 ticks from OVER). The compensation step does start the early wrist motion, as before.
- Target-velocity and target-jerk discontinuities are present in both runs (per-tick target velocity change 0.41 rad/s with compensation, 0.31 without). They are the larger effect in both, so they are the bigger lead, not the compensation.
- Recommendation (section 6): keep compensation, and investigate the trajectory discontinuities first. Smoothing the compensation activation is a cheap follow-up that targets the early onset only.

## 1. What was run

- Launcher (scratch, git-ignored copy): `data/diag/bounce_20261029/closed_loop_run.py`. It calls `experiments/demo_video.py` `record_only` (the same path that made blend20), so instruction, LLM cache, layout and command bridge are the same.
- Runs (tmux `cl2`, logs `data/logs/closed_loop_on_20261029_1453.log` and `data/logs/closed_loop_off_20261029_1453.log`):
  - `data/closed_loop/on_20261029_1453` (control, compensation on): SUCCESS, 13,613 ticks.
  - `data/closed_loop/off_20261029_1453` (compensation off): SUCCESS, 13,510 ticks.
- Per-tick extract of both recordings: `experiments/bounce_diag_extract.py` (unchanged), outputs `data/diag/closed_loop_20261029/on_ticks.csv` and `off_ticks.csv`. Both replays report `replay ok True`.
- Solver and sag instrumentation: `data/closed_loop/*/instr_ticks.csv`, written by the launcher (read-only wrappers, see section 2).
- Comparison script: `data/diag/closed_loop_20261029/cl_compare.py`, output `data/logs/cl_compare_20261029_1503.log`.

## 2. Verification

- **Determinism and no interference.** The control run's state hashes match the blend20 recording on all 13,613 ticks, and the skill targets (`ctrl`) are identical. So the instrumentation does not change the run, and the run is reproducible even though the blend20 seed is `None`.
- **The only intended difference.** The launcher drops every nonzero held-mass reading in off mode. The held step still runs as before: it waits for stillness and takes the weight reading (0.205 kg, at tick 11,599 in both runs). The held torque is zero in the physics of the off run: the applied force on L7 matches gravity alone within 0.013 N m (the on run deviates by 0.38 N m, which is the held term).
- **Timeline identical up to DOWN.** OVER (11,602) and DOWN (11,854) are the same ticks in both runs. After DOWN the off run reaches OPEN 115 ticks earlier (12,043 vs 12,158), and BACK, HOME and PLAN DONE shift by the same kind of amount (PLAN DONE 13,210 vs 13,313). This is a consequence of the arm settling sooner, not a changed setting. The comparison below is therefore aligned by segment label, not tick.
- **Tracked files.** `git status` and `git diff HEAD -- src experiments models` show no change. The launcher and the analysis scripts live in git-ignored `data/diag/`.

## 3. Results at corresponding transitions

Ticks are relative to OVER (11,602). The tilt numbers are from the segment OVER → OPEN unless stated.

| metric | compensation ON | compensation OFF |
|---|---|---|
| peak tilt | 3.41° at +34 | 3.12° at +68 |
| first oscillation onset (+0.1° over 0.22° base) | +1 (tick 11,603) | +41 (tick 11,643) |
| local peaks (prominence 0.2°) | 3.41 (+34), 2.76 (+80), 1.20, 1.13 | 3.12 (+68), 0.69 (+155), 1.27 in DOWN (+280) |
| DOWN tilt max | 0.80° | 1.27° |
| settling (last tick above 0.5° before OPEN, after peak) | +312 (tick 11,914) | +329 (tick 11,931) |
| max per-tick skill-target velocity change | 0.411 rad/s (L4) | 0.309 rad/s (L4) |
| max skill-target jerk | 9,241 rad/s³ (L4) | 6,956 rad/s³ (L4) |
| max measured jerk | 604 rad/s³ (L7) | 398 rad/s³ (L7) |
| max measured joint speed | L3 0.40, L7 0.39 rad/s | L3 0.36, L7 0.30 rad/s |
| ticks with a velocity limit binding (OVER..OPEN) | 64 | 58 |
| sag integrator clip (position / rotation) | 0 / 0 ticks | 0 / 0 ticks |
| max sag integrator state \|C_p\| (window 11,601–12,200) | 15.9 mm at 11,749 | 25.7 mm at 11,784 |
| blended handover (OVER → DOWN) | tick 11,853 | tick 11,853 |
| minimum joint-limit slack | 0.034 rad | 0.033 rad |
| collision pairs active (max) | 2 | 2 |

Notes:

- The settling figure is measured from the last tick above 0.5°, so it depends on where OPEN happens. The off run's OPEN is earlier, so the two settling times are not strictly comparable.
- The off run's DOWN bump (1.27° at tick 11,882) is larger than the on run's (0.80°). Both runs have a bump in DOWN, so it is not a compensation-only effect.

## 4. What the discontinuities are

- The skill's target has velocity and jerk discontinuities in both runs. The largest per-tick velocity jump is at tick 11,856 on L4 in both runs (0.41 rad/s with compensation, 0.31 without), at the OVER → DOWN blended handover. The largest target jerks are at the same tick (9,241 and 6,956 rad/s³ in the table, after the one-tick differences). These are much larger than the measured jerks (400–600 rad/s³), so the arm's joint filtering hides part of the reference step.
- The blend (M200) is the only blended handover in either run, and the largest jump is one tick after it (11,856), so the blend is the leading suspect (see the earlier note on the blend cubic). The compensation-on run also has a second jump inside the OVER move, at tick 11,661 on L3 and L5 (0.26 rad/s per tick). The off run does not show it in its top three, so it may be linked to the compensation. I have not traced its cause.

## 5. Comparison with the open-loop replay

| run | peak tilt | note |
|---|---|---|
| original (compensation on, recorded) | 3.41° at 11,636 | |
| open-loop replay, compensation off, recorded targets | 5.73° at 11,680 | targets built with compensation on; the sag integrator never saw the missing torque |
| closed-loop live, compensation off | 3.12° at 11,670 | skill and sag integrator respond; **this is the valid no-compensation result** |

The open-loop replay overstated the penalty of removing compensation. Its limitation 1 said so, and the closed-loop run confirms it.

## 6. Recommendation (not implemented)

1. **Retain compensation.** It does not increase the bounce in closed loop: the first peak is 3.41° (ON) vs 3.12° (OFF), and the DOWN bump is 0.80° (ON) vs 1.27° (OFF). The compensation also keeps the arm from sagging under the beaker's weight, which the off run has to make up with the sag integrator (25.7 mm peak vs 15.9 mm).
2. **Investigate the trajectory discontinuities first.** They are present in both runs and larger than the compensation effect. The most likely source is the handover at 11,853–11,856, where the blend starts the next move with the measured pad speed while the reference was at rest (see the earlier note on the blend cubic). The compensation-on jump at 11,661 inside OVER is not explained by the reference formula and needs a look at the per-tick IK solution.
3. **Then smooth the compensation activation.** The held step applies the full torque in one tick, which causes the early wrist motion (onset +1 vs +41). Ramping the held torque over about 0.5 s would reduce that onset. This is cheap, but it only affects the first part of the bounce, so it comes after item 2.

Both 2 and 3 are decisions for David because they touch the blend and the activation: implementation details are my call, but changing the blend shape or the activation timing changes the trajectory the operator sees.

## 7. Files

- `data/closed_loop/on_20261029_1453/`, `data/closed_loop/off_20261029_1453/`: recordings, episode logs, `instr_ticks.csv`, `launch.json`.
- `data/diag/closed_loop_20261029/on_ticks.csv`, `off_ticks.csv`: per-tick extracts.
- `data/diag/closed_loop_20261029/cl_compare.py`, `data/logs/cl_compare_20261029_1503.log`: comparison.
- `data/diag/bounce_20261029/closed_loop_run.py`: launcher (scratch).
- Logs: `data/logs/closed_loop_on_20261029_1453.log`, `data/logs/closed_loop_off_20261029_1453.log`, `data/logs/cl_extract_on_20261029_1501.log`, `data/logs/cl_extract_off_20261029_1501.log`.
