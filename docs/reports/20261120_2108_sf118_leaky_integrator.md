# SF-118: sag trim as a leaky integrator, tests and held 200 g run (2026-11-20)

Branch `safety-filter`, worktree `~/projects/openarm-tracer-sim-sf-d`. Commits: `52ae8e2` (code, WIP), `92487fb` (tests), and the commit that adds this report. Not pushed.

## Summary

- The per-arm freeze (S5, SF-109) is gone. Each tick the sag trim is multiplied by exp(-dt / sag_leak_tau_s) before the new correction is added (tau 5.0 s, SAG_LAYER_CFG). SF-116 (no new correction on a failed solve; the leak still runs) and SF-117 (warm-start reset) are kept. All five named sag test files pass.
- **Held 200 g run: the pick was not reached.** The episode ended in PLAN_FAILED at segment 1 (Navigate to the beaker): "ARMS timed out after 30 s", at 31.40 s of simulated time. No carry, no place.
- **Both arms' trims reached their clip limits**: the position norm is 40.00 mm (sag_max_m) and the rotation norm 0.1000 rad (sag_max_rad). The leak did not stop the windup in this run.
- The comparison with pre-S5 is not like-for-like. Pre-S5 applied the trim only to the manipulation source (M264 gate), so there was no trim during navigation. In this code the trim runs on every arm command, including navigation.

## Code changes

| File | Change |
|---|---|
| src/sag_layer.py | `_sag_leak` (new): C *= exp(-dt / sag_leak_tau_s); not during a held-mass ramp (M238). `correct()` leaks the trial copy before the new correction, and leaks the committed trim on a failed solve (SF-116). `frozen`, `still_ticks`, `q_last`, `_track_still` and the freeze log are removed. |
| src/task_config.py | SAG_LAYER_CFG: `freeze_tol_rad` and `freeze_still_ticks` removed; `sag_leak_tau_s: 5.0` added (row SF-118). |
| docs/mpvi_decisions.md | Row SF-118. |
| docs/design_plan.md | SF-109 marked superseded by SF-118; SF-118 entry added. |
| experiments/sag_layer_test.py | S5 freeze checks rewritten: SF-118 (a) standing command integrates; new SF-118 (L1), (L2), (L3). |
| experiments/sag_solver_failure_test.py | Checks A and B now expect the leak (A1 and A3) and no freeze (B1). |

## Tests

| Test | Result |
|---|---|
| experiments/sag_layer_test.py | 17 of 17 PASS |
| experiments/sag_solver_failure_test.py | ALL PASS (10 checks) |
| experiments/held_mass_ramp_test.py | RESULT: PASS (40 checks) |
| experiments/open_catch_test.py | 12 of 12 PASS |
| experiments/sag_warm_start_reset_test.py | ALL PASS (7 checks) |

New leak checks (all in sag_layer_test.py, run on the integrator alone with a 1 mm error):

- **L1, constant error, 100 ticks.** After 100 ticks the trim is 1.874 mm, matching the closed form inc (1 - d^100) / (1 - d) to 1e-12. It never passes the steady state. After 5 tau (3750 ticks, 25 s) it is 14.909 mm, against a steady state of 15.010 mm. gain x error x tau = 15.000 mm.
- **L2, alternating +1 mm / -1 mm, 200 ticks.** Peak |C| 0.0200 mm (limit 0.1 mm).
- **L3, 50 ticks of error then 200 ticks of none.** Ratio at tick 250 to tick 50 is 0.765928, equal to exp(-200 dt / tau). Fitted tau 5.000000 s.

Deviations from the brief:

- The brief's steady-state formula, "error x tau / dt", is off by a factor of 1/dt. The per-tick step is gain x error x dt, so the steady state is gain x error x tau (15 mm for 1 mm). The test checks that.
- The brief's 100-tick convergence check cannot pass with tau 5 s: 100 ticks is 0.67 s, which is 12.5 % of the steady state. L1 therefore checks the closed form at 100 ticks and convergence after 5 tau.
- The brief says "_frozen flag". The attribute in the code was `frozen`; it is removed.

## Held 200 g run

Command (tmux session `held_sf118`, through `experiments/run_capped.sh`, 6 GB cap):

```
env MUJOCO_GL=egl timeout 600 .venv/bin/python -u data/logs/held_sf118_shim_20261120_210211.py data/logs/held_sf118_out_20261120_210211 --liquid 0.100
```

- Log: `data/logs/held_sf118_20261120_210211.log`. Outputs: `data/logs/held_sf118_out_20261120_210211/` (run_info.json, liftoff_ticks.csv, sag_summary.json, recording/). Shim: `data/logs/held_sf118_shim_20261120_210211.py`. All are git-ignored.
- Seed 4294874296 (`[sampler] seed 4294874296`), liquid 0.100 kg, glass 0.100 kg, so 200 g held.
- Wall clock 4 min 6 s, exit status 1 (the plan failure), 21:02:37 to 21:06:42. Peak RSS 4.16 GB.

**Shim.** The shim used for the SF-116 and SF-117 held runs was in a scratchpad that has since been emptied, so I rebuilt it from `data/bounce_check_20261103/run_mass.py` with a builder script. Edits: ROOT set to this worktree; the sampler seed pinned by wrapping `iter_verified_stances` (the same pin as before; the `[sampler]` line shows 4294874296); the trim read from `SAG_TRACE` instead of `mpvi_modules.TRACE` (which has no `C_p`); the `frozen` column left blank; INFO logging on; `[shim] sag <side> <kind> t=` lines printed per event; a solver-success counter; `sag_summary.json` written at the end. The SF-116/117 run numbers came from a different shim, so comparisons with them are approximate.

**Result**

| Question | Answer |
|---|---|
| Pick succeeded? | **No.** The pick was not reached. Segment 1 (Navigate to the beaker) failed at 31.40 s: "ARMS timed out after 30 s", no retry. Plan status: failed, pending, pending, pending. |
| Carry and place | Did not happen. |
| Final trim, left arm | C_p (mm) -36.95, 11.73, -9.86: norm **40.00 mm** (clip). C_r (rad) 0.0078, -0.0332, -0.0940: norm **0.1000 rad** (clip). |
| Final trim, right arm | C_p (mm) 10.19, 1.96, 38.63: norm **40.00 mm** (clip). C_r (rad) 0.0764, 0.0030, -0.0645: norm **0.1000 rad** (clip). |
| Solver, each arm | 1016 failures in one streak (1.40 s to 8.17 s), then 3484 successes. No failures after 8.17 s. The streak ended by recovery, not by the episode ending. |
| Solver, SF-116 run (held11, 18:07 on 394567c, other shim) | Left 12,850 failures, right 4,480, never recovered. Possibly the SF-117 warm-start reset; not verified (the two runs differ in code and shim). |
| Bridge | One suspension: tracking error, right joint 5, -0.260 against a limit of 0.250, at 21.88 s; resumed 22.12 s. |

**Pre-S5 reference.** `data/logs/pre_held_20261119_183419.log` (outcome SUCCESS, same seed and liquid). Final left trim: C_p (mm) -1.88, 0.75, -1.90, norm **2.78 mm**; C_r norm 0.017 rad; right arm zero. The log shows takeover events at 69.47 s (pick start), 87.84 s, and a release reset at 93.71 s. The Phase S report names the pre-S5 commit as 7290d3c. In that code the M264 gate (`MANIPULATION_SOURCES = ("vla",)`) applied the trim only to the manipulation source, so there was no trim during navigation.

| | Pre-S5 (2026-11-19 18:34) | SF-118 (this run) |
|---|---|---|
| Outcome | SUCCESS | PLAN_FAILED at navigation, 31.4 s |
| Trim during navigation | none (gate) | active on every arm command |
| Left trim at end | 2.78 mm, 0.017 rad | 40.00 mm, 0.100 rad (both arms at clip) |
| Solver failures | none logged | 1016 per arm, one streak at the start |

## Unexpected

1. **The trim wound up to its clip in the navigation phase.** With tau 5 s, a constant error e gives a steady state of 15 e, so reaching 40 mm needs about 2.7 mm of sustained error. The leak does not prevent this when the error persists. Both arms hit the clip at the same time.
2. **The trim's growth during navigation is not in the data.** So I cannot say when it grew relative to the bridge suspension at 21.88 s, or whether it caused the timeout. The shim does not record the trim during navigation: `liftoff_ticks.csv` has `trace_ok = 0` on every row until the pick, because the scripted trace only fills during manipulation. So the trim history is not in the data. The signal trace (`OPENARM_SIGNAL_TRACE=1`) would show it.
3. The solver streak shortened from "never recovered" (SF-116 run) to 6.8 s (SF-117 run), but it still failed for 1016 ticks at the start.

## Questions for David (not decided; no tuning done)

1. **Should the sag trim run during navigation?** Pre-S5 applied it only to the manipulation source (M264). M262 made the layer apply to every source, and the S5 freeze was meant to cover idle arms. This run shows the trim winding to the clip during navigation. Options: (a) restore the manipulation-only gate; (b) keep every source, with a different tau or clip; (c) first find why the arms do not converge in navigation (the 30 s ARMS timeout).
2. **Is the leak meant to pause during a held-mass ramp?** I kept the M238 hold, so the trim keeps its value during a ramp and the leak does not run then. This is my reading of the brief; please confirm.
3. **The steady-state formula.** The brief has error x tau / dt. The code gives gain x error x tau. I assumed the code is intended; please confirm.

## Not done

- No second held run. A diagnostic run with `OPENARM_SIGNAL_TRACE=1` would show the trim and the arm error during navigation. That is one more 4-minute MuJoCo run; I have not started it, since it depends on your answer to question 1.
- No push.
