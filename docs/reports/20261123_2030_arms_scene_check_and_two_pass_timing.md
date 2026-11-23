# ARMS stall: scene check (Part 1), and the two-pass sag correction timing gate (Part 2, step 0) (2026-11-23)

Branch `safety-filter`, HEAD `92de2d0` before this commit. Measurement only: no production code changed. One MuJoCo process at a time. Not pushed. Part 2 was NOT built: the timing gate failed (below).

## Correction to the previous report

The previous report (`20261123_1940_arms_stall_trace.md`) said the fingers come within 0.05 m of the bench at 3.4 to 3.9 s. That came from the default scene (`experiments_loader.load()`, `chemistry_lab_combined.xml`), which is not the scene the tests build. The tests use `chemistry_lab_beaker_hotplate.xml` through `load_task` (`TASK_CFG["scene"]`). In the task scene the bench is at the origin and the robot base is at (0.7, 3.4) m, about 3.4 m away; no left-arm geom comes within 0.10 m of the bench, the hotplate or the right arm at any time from 3.0 to 11.8 s. The 0.05 m finger-to-bench result is an artifact of the wrong scene and is withdrawn. The default-scene check also left out the task hotplate entirely (the combined scene has no `task_hotplate` body).

## Part 1: scene check

### Method

`experiments/scene_stall_probe.py` wraps one existing script unchanged. It runs the test (or the held shim) in its own scene and records, every 0.05 s from 3.0 s to the stop time (11.8 s):
- pedestal joint height, base pose (`base_link`), bench and task hotplate positions;
- for the left arm's solver collision geoms (the solver's own sets: links 3 to 7, hand, fingers; against `bench` and `task_hotplate`, and against the right arm's non-link-0 geoms), the nearest distance (capped at 0.10 m) and every pair below 0.10 m; a pair below `detect_distance` (0.05 m) is inside the solver's collision constraint;
- the solver's own limit objects: the inequality rows each limit gives at the last solve (`CollisionAvoidanceLimit` rows with infinite slack are inactive);
- the left arm's last solver call and the constraint letters per joint (M margin, C cap, X other).

Runs: grasp_lift_test (its own task scene) and the held run (`gap_held_shim.py`, `--liquid 0.100`, the same stance as before), both with the trim on. Data: `data/logs/sf120_part1_2_20261123_2003/scene_grasp/`, `scene_held/`.

### What differs between the two scenes

| Quantity (3.0 to 11.8 s) | Grasp test | Held run |
|---|---|---|
| Pedestal joint | 0.41745 m, constant | 0.41745 m until 11.753 s, then 0.41604, 0.41261 (the PEDESTAL phase, after the ARMS handover) |
| Base pose | (0.7, 3.4) m, yaw 90° | identical |
| Bench (body position) | (0, 0, 0) | identical |
| Task hotplate (body position) | (0.80, 0.66, 0.96) | (0.65, 0.66, 0.96) |
| Nearest left-arm geom to bench or hotplate | ≥ 0.10 m | ≥ 0.10 m |
| Nearest left-arm geom to the right arm | ≥ 0.10 m | ≥ 0.10 m |
| Pairs inside the 0.05 m detection distance | 0 | 0 |
| `CollisionAvoidanceLimit` rows, slack | 80 rows, slack inf (inactive) | 80 rows, slack inf (inactive) |

The only difference between the scenes during the arms-to-home step is the hotplate's x position (0.80 against 0.65 m). The pedestal height is the same during ARMS (0.41745 m in both, until the handover at 11.753 s).

### Which constraint is active on joint 2, and what the solver does at 3.70 s

- The collision constraint is inactive in both scenes (no pair within 0.10 m). It does not drive joint 2 onto its margin in the test.
- Joint 4's lower margin is the constraint that switches on. The arm starts with joint 4 at 0 rad, below the solver's lower margin bound of 0.1745 rad (10° inside the 0 rad hard limit). The solver's configuration-limit slack for joint 4 is negative at rest and crosses zero at about 3.65 s. In the grasp run, joint 4 is held on its margin for 26 ticks (3.647 to 3.86 s); in the held run for 2 ticks (3.647 to 3.70 s).
- Both scenes' solver outputs are bitwise identical up to 3.69 s (the recorded calls, both arms, all arrays). The first difference is at 3.70 s, the tick after joint 4 reaches its margin. The difference is 2.7e-4 rad on that tick and grows to 0.28 rad by about 4.5 s.
- The hotplate position does not change the solver output for the same inputs: `experiments/arms_hotplate_check.py` rebuilds the left solver on the task scene at 3.70 s with the hotplate at 0.80 and at 0.65 m, and the two outputs are identical (difference 0.0). **Caveat:** the rebuilt solver does not reproduce the recorded outputs (about 0.013 rad off, in both scenes), so the rebuild is missing an input (candidates: the live solver's per-tick dt and warm-start state, and the base and pedestal state at construction). The hotplate result is therefore suggestive, not conclusive.

So the scene difference that is recorded is the hotplate's position, and it does not change the solver output in the rebuild. The solver's own behaviour at the joint-4 margin switch is the nearest explanation for where the runs split. What makes the two runs take different branches after the switch has not been identified.

### Options (for David; not decided)

1. Start the arm with joint 4 inside its margin band (the rest pose sits outside it), or leave the start and accept the margin switch at about 3.65 s.
2. Make the solver's branch choice at a margin switch deterministic and independent of tiny differences (a tie-break rule, or a small warm-start damping), or accept that the staging result depends on that sensitivity.
3. Replay the live solver state (dt, warm start) in the rebuild before deciding anything about the hotplate. This is a measurement-only step.

## Part 2: step 0, timing gate (stopped; nothing built)

### Method

`experiments/sag_timing_probe.py` times today's `SagLayer.correct` and `MinkArm.step` per call. It also runs a scratch Pass 2 on a separate configuration, after the real call, from today's solver result toward its pad pose plus the layer's current learned offset, with no speed limit, the same margins, collisions and freeze, iterated until the pad is within 0.1 mm and 0.001 rad, at most 10 iterations. The arm's own state is not changed.

Gate (David's spec): if both passes together at the 99th percentile exceed 3.3 ms, stop and report.

### Results

| Run | Today's `correct()` mean / p50 / p99 (ms) | Solver alone p99 (ms) | Scratch Pass 2 mean / p99 (ms) | Pass 1 + Pass 2 p99 (ms) | Pass 2 at 10 iterations | Pass 2 final pad error, max |
|---|---|---|---|---|---|---|
| Held run, first 12 s (trim on) | 1.53 / 1.41 / **3.37** | 2.39 | 5.36 / 13.86 | **16.05** | 51 % of calls | 5.3 mm, 0.0007 rad |
| Policy-stream variant B (seed 1, correction on, bridge off, 30 s) | 1.73 / 1.55 / **5.05** | 3.97 | 10.67 / 29.36 | **33.43** | 98 % of calls | 9.4 mm, 0.0058 rad |

Gate: failed in both. Today's layer alone already exceeds 3.3 ms at the 99th percentile (3.37 ms held, 5.05 ms policy B), before any Pass 2. Stopped. Nothing built: no change to `src/sag_layer.py`, no decision row, no tests, no effect runs (Part 2 steps 1 to 3 not run).

Pass 2 as specified does not converge within its iteration budget on most ticks (51 % held, 98 % policy B), so even the time budget would not give the 0.1 mm target.

### Options for Part 2 (questions for David; not decided)

1. The budget: today's layer is already above the 3.3 ms half tick at the 99th percentile. Is the budget per tick (both passes) 6.7 ms (a full tick), or is the 3.3 ms figure a hard limit?
2. Pass 2 convergence: fewer iterations with a looser tolerance, a warm start from the previous tick's Pass 2 result, or a Newton-type step in place of mink's velocity QP.
3. Pass 2 with no speed limit: confirm that a full-distance command per tick is acceptable downstream (the bridge's limits still apply after the layer).
4. A faithful rebuild of the live solver state for the hotplate check (Part 1 caveat above).

## Not run

- Part 2 steps 1 to 3 (build, tests, policy-stream effect runs, held run to SUCCESS): not run, because the timing gate failed.
- The full safety-filter regression and the five sag tests: not run (nothing changed in the sag layer).
- The four staging tests' status after a correction change: not applicable.
- The faithful rebuild of the solver state for the hotplate check.

## Files

- `experiments/scene_stall_probe.py` (scene probe; committed with this report)
- `experiments/sag_timing_probe.py` (timing probe, scratch Pass 2; committed with this report)
- `experiments/arms_hotplate_check.py` (rebuild check on the task scene; committed with this report)
- Outputs (git-ignored): `data/logs/sf120_part1_2_20261123_2003/` (scene_grasp, scene_held, timing_held, timing_policyB, hotplate_check); `data/logs/scene_probe_20261123_2000/` and `_2000b/` (earlier probes, to 8 s).
