# Replay mismatch (five tests) and held-weight ramps in the sag rule (2026-11-25)

Branch `safety-filter`, HEAD e578aee. One MuJoCo process at a time. Runs and scripts in `data/logs/replay_check_20261125/` (git-ignored). No source code changed. Nothing pushed. `docs/progress.md` not touched.

Written for: David (the replay check before any push, and the held-weight ramp check).

## Plain-language answer

**Replay mismatch.**

1. **The loop is deterministic.** Two fresh gripper_force runs are bit-identical over all 10,521 ticks (ctrl, base velocity, held mass, grip force, hashes, initial state). The mismatch is a replay problem, not randomness.

2. **The first mismatch comes from the held-object weight, which the replay does not restore.** The gravity feedforward adds a held-object torque from the held-mass ramp (`src/held_mass_ramp.py`, decisions M217/M220/M221). The replay writes the recorded mass into `HELD_MASS` (M196), which nothing reads. So in the replay the ramp is empty, and the torque is zero. The first tick with a recorded held mass is the first mismatch tick in all five tests (gripper 7934, nav 9540, stance 8208, mpvi_pipeline 8547, mpvi_text 9155). The mass there is the first step of the ramp, 0.00034 kg.

3. **For mpvi_text the held mass explains everything.** Feeding the recorded mass into the held-mass read makes that replay bit-identical over all 9,626 ticks.

4. **For the other four, a second difference starts later and is not identified.** With the recorded mass restored, the first mismatch moves to: gripper 8060 (+126 ticks), nav 9981 (+441), stance 8661 (+453), mpvi_pipeline 8955 (+408). I did not find which state differs. At the first checkpoint the velocities and solver warm start differ.

5. **Cause and rule.** The held-object term is in the code at 1f29da7 too (it predates the rebuild), so it is not state the rebuild added. The second difference is unexplained. Under the brief this is "anything else": I stopped and changed no code. The replay path needs a decision from you: whether the held-mass read should come from the ramp record or the recorded value, and where the replay should restore it.

**Held-weight ramps (item 5).**

6. **The rule skips its updates during the ramps.** Across the weighing rate ramp (62.22 to 64.67 s), the target ramp at weighing end (64.67 to 65.67 s) and the ramp-out (74.09 to 75.09 s), the estimate did not change (every call is `held_ramp`). So the 5.5 mm at release does not come from updates during those ramps.

7. **The 5.5 mm at release comes mostly from the hold before ramp-out.** The hand offset |C_p| went from 4.0 mm at the end of the ramp (65.67 s) to 6.1 mm at ramp-out (74.07 s). Those 8.4 s are a hold with the beaker at full mass and no ramp running, so the rule updates there. The 0.4 s after the ramp ends and before the gripper opens lowered it to 5.5 mm at release (75.49 s).

8. **Two gaps against the Nov 2 decisions (report only, not changed).** (a) M226 freezes the estimate from ramp-out to the gripper-open event. The rule skips only while a ramp runs, so the 0.4 s after the ramp-out is updated (60 ticks; the estimate moved up to 1.3 mrad). The `changing` flag that M226 uses is never read by the sag layer. (b) The hold after the ramp is updated; M226 does not require a freeze there as written, and M230 removed the grasp freeze. M238 says the trim holds only during a ramp ("interpreted, to confirm"). Which reading is right is your call.

## Technical block

### Replay check

- **Determinism.** `gf_run1` vs `gf_run2` (gripper_force_test, TMPDIR per run): ticks 10,521 both; ctrl (10521,5,19), basevel, heldmass, hashes, initial_state, gripforce all identical.
- **Plain replays (as in the tests).** First mismatch: gripper 7934; nav 9540; stance 8208; mpvi_pipeline 8547; mpvi_text 9155. The first recorded nonzero held mass is at the same tick in all five (`analyze.py`): 0.00033979 kg, the first ramp step (rate about 0.0509 kg/s, linear).
- **Mechanism.** `src/control_loop.py` `advance()`: `ff_off = gff.torques(data)/gff.kp`, and `gff.apply_offsets` adds it to `data.ctrl` before each step. `gravity_ff.py` `torques()` reads `held_mass_ramp.mass_at(m, s, data.time)`. The replay (`control_loop.py` replay branch, `set_held_mass`) writes `HELD_MASS`, and `HELD_MASS` is never read (grep: only written in `gravity_ff.py` lines 40 and 55). `mass_at` returns 0 when there is no ramp record, which is the case in a replay. The replay also feeds recorded ctrl, but the offset is recomputed from the replay's own data, so a missing held term changes `data.ctrl` (part of the hash) from the first tick with mass > 0.
- **Confirmation (`confirm.py`, recorded-mass mode).** `held_mass_ramp.mass_at` is replaced by the recorded mass at frame `round((t - t0) * 150)`, with t0 = `initial_state[0]` = 1.4000 s (the post-settle time). First version used `round(t*150)` and was wrong by the 210-tick settle; it showed a false mismatch at 7724 (`debug_mass.py` counted 77 calls differing from the original, first at 7934, which pinpointed the offset). Results after the fix:

| recording | plain first mismatch | held mass restored | result |
|---|---|---|---|
| gripper_force (gf_run1) | 7934 | 8060 (qvel 3.0e-3, qacc_warmstart diff at checkpoint 8100) | differs |
| nav_pipeline_follow | 9540 | 9981 | differs |
| stance_select_pipeline | 8208 | 8661 | differs |
| mpvi_pipeline | 8547 | 8955 | differs |
| mpvi_text | 9155 | ok, 9626 ticks bit-identical | fully explained |

- **Control run.** Wrapping `mass_at` and returning the original value gives the plain first mismatch (7934), so the wrapper alone changes nothing.
- **What I did not establish.** The first differing component at tick 8060 (gripper) or the later ones. The logs contain only hashes and checkpoints every 150 ticks. Localising it needs a per-tick state dump from a live run (the run is deterministic, so that works) and is the next step if you want it.
- **Rule check.** The held-object term and the ramp are in 1f29da7 (`held_mass_ramp.py` added in 8e24282 on 2026-11-02, the held-mass torque in 29860e6 on 2026-10-28); the replay's `set_held_mass` call is at 1f29da7 too. So the mismatch predates the rebuild. The rebuild's sag and bridge state (estimate, motor filter, applied offset, shrink) is not in any of the five recordings' first mismatches. No code changed.

### Held-weight ramps

- **Run.** `ramp_probe.py` wraps `SagLayer.correct` (logging only) around the unchanged `sag_timing_probe.py` with the same arguments as the rebuild_s3 held nominal run. Output `data/logs/replay_check_20261125/held_nominal_b/` (exit 0; `sag_est.npz`: 21,136 calls, left and right). The phase times come from its `liftoff_ticks.csv`. An earlier attempt (`held_nominal/`) lost the per-joint file because the probe rewrote `sys.argv`; fixed.
- **Phases (left arm).** Weighing rate ramp 62.22 s to 64.67 s (mass 0 to 0.1218 kg). Target ramp 64.67 s to 65.67 s. Hold 65.67 s to 74.09 s. Ramp-out (target to 0) 74.09 s to 75.09 s. `place: LIFT` starts 75.49 s, which I use as the release (the gripper-open event, M226, is at or just before it; the probe logged no `release_done`).
- **Skip reasons per window.**

| window | calls | reasons | estimate change (mrad, J1..J7) |
|---|---|---|---|
| before weighing (0 to 62.22 s) | 9123 | update 9121, held_ramp 1, no_offset 1 | (start-up) |
| weighing rate ramp (62.22 to 64.67) | 367 | held_ramp 367 | 0 |
| target ramp (64.67 to 65.67) | 150 | held_ramp 150 | 0 |
| hold (65.67 to 74.09) | 1263 | update 1263 | −3.4, −0.7, −2.4, 3.4, −5.5, 13.0, −22.4 |
| ramp-out (74.09 to 75.09) | 150 | held_ramp 150 | 0 |
| ramp-out end to release (75.09 to 75.49) | 60 | update 60 | 0.5, −0.1, 0.1, −0.3, 0.2, −1.3, 1.0 |

- **Estimate per joint (left arm, mrad, J1..J7) and hand offset (C_p, mm, from the trace at the nearest defined tick).**

| point | t (s) | est J1..J7 | C_p (x, y, z) mm | \|C_p\| mm |
|---|---|---|---|---|
| just before weighing | 62.20 | −0.70, 1.59, 1.67, −0.65, 9.52, −11.00, 7.66 | −1.62, 1.83, −2.50 (trace 61.45) | 3.50 |
| end of weighing | 64.67 | −0.70, 1.60, 1.68, −0.64, 9.52, −11.01, 7.67 | −2.10, 2.29, −2.81 | 4.19 |
| end of the target ramp | 65.67 | −0.71, 1.59, 1.66, −0.62, 9.43, −10.96, 7.63 | −1.83, 2.14, −2.83 | 4.00 |
| before ramp-out | 74.07 | −4.15, 0.91, −0.73, 2.84, 3.91, 2.09, −14.85 | −0.19, −0.52, 6.11 | 6.13 |
| at release (`place: LIFT`) | 75.49 | −3.61, 0.78, −0.63, 2.53, 4.15, 0.73, −13.82 | −0.28, −0.30, 5.49 | 5.50 |

- **Attribution of the 5.5 mm.** None of it comes from the three ramp windows (estimate unchanged). |C_p| rises from 4.0 mm to 6.1 mm during the hold (65.67 to 74.07 s, updates running, no ramp), and falls to 5.5 mm in the 0.4 s after ramp-out (updates running). I did not separate the pose-dependent part of C_p from the estimate part in that 0.4 s.
- **Rule.** `src/sag_layer.py` line 184: `elif hmr.ramp_running(self.model, self.side, t): skip = "held_ramp"`. `ramp_running` is true for a rate ramp or an unfinished target ramp (`src/held_mass_ramp.py` line 58). The M226 freeze flag `changing` is set in `set_target` when a ramp goes to 0 from a held mass (`src/held_mass_ramp.py` line 97) and cleared by `end_load_change` at the gripper-open event (`sag_layer.py` line 259). Its predicate `is_load_changing` (`held_mass_ramp.py` line 102) is read only by the trace (`src/mpvi_modules.py` line 387) and by `experiments/held_mass_ramp_test.py` (check 12b: still changing after the ramp-out until the gripper opens). The sag rule never reads it. Consequence: the 0.4 s between the end of the ramp-out and the gripper-open event is updated, which M226 (c) does not allow as written, and which that test says should be frozen.
- **Decision rows.** M226 (freeze: CLOSE start to end of ramp-in; any held-mass ramp; ramp-out to gripper-open), M230 (grasp freeze removed), M238 (trim holds only during a ramp, "interpreted, to confirm"). Design plan rows at `docs/design_plan.md` line 596 ff.

## Next steps (not done)

1. Replay: decide the held-mass source for replays (the ramp record or the recorded value). Then find the second difference with a per-tick state dump from a live gripper_force run (deterministic, so one dump run and one replay run are enough).
2. Held-weight ramps: decide whether the sag rule should also freeze from ramp-out to the gripper-open event (M226 (c)), and whether the hold after the ramp should update. Not changed.
