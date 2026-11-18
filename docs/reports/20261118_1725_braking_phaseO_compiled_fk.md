# Braking Phase O: compiled arm kinematics (WIP: goal not met, behaviour not identical)

Written for: David (project lead). Branch safety-filter, worktree ~/projects/openarm-tracer-sim-sf-d. NOT pushed.
Commits: f6d5859 (step 2, compiled chain, SF-103), b8c1efd (step 3 wiring, SF-104, WIP), this report (WIP).

## 1. Answer

**No. The full filter call median in the integration is 7.75 ms, not under 6 ms.** The same-window baseline (Phase M code) was 8.22 ms, so the wiring saves 0.47 ms of median (6%).

**The behaviour is not identical.** The compiled chain differs from MuJoCo by about 1e-15 m, and the closed loop amplifies that to about 1e-3 rad within 500 ticks. Stop starts, the closest gap sign and the status counts all change. One test that passes at the Phase M commit (`safety_filter_braking_j_test`) now fails.

Recommendation: revert the wiring (step 3) and keep `arm_fk.py` as an unused module, unless you want the bit-exact route in section 10. The arm kinematics is not the bottleneck: the resolver and braking medians (4.6 and 2.9 ms in the integration) are what remain.

## 2. Step 1: the FK path before the change (read-only)

| Who | MuJoCo call | Returns | Reshaped or copied in Python | Cost (Phase M profile, wrapper-inflated) |
|---|---|---|---|---|
| RobotShapes.set_state / set_arms | mj_kinematics on the robot-only model (after writing qpos) | xpos (nbody, 3), xmat (nbody, 9), xanchor, xaxis | qpos assembled per call; `np.array_equal` cache check | set_arms 2.71 ms, 25 calls per tick |
| RobotShapes.place_arrays | none (reads xpos, xmat) | capsule ends and box frames | einsum on (k, 3, 3) views, 6.6 calls per tick | place_arrays 1.43 ms |
| RobotShapes.hand_frame | mj_name2id on every call, then reads xpos, xmat | link 7 position and rotation | copies; 54 calls per tick | hand_frame 1.23 ms |
| tick.py (travel bounds, static lever) | none (reads xanchor) | joint anchors | `np.stack` and copies | inside Tick._build |
| ArmKinematics.jacobian_step | mj_comPos and mj_jacBody | hand Jacobian at the hand's frame origin | vstack of the arm's dof columns | resolver step |
| ArmKinematics.sew, hand_pose, convert_directions | none per call beyond set_state | S, E, W, hand pose, directions | copies | resolver step |

Finding: the MuJoCo call itself is small (mj_kinematics is about 8 µs, as the Phase M report said). The cost is the Python around it: the qpos assembly, the cache checks, the name lookups and the einsum calls. The brief's 2.6 ms estimate for these three items was the profile's inflated number, and the measured saving below is much smaller.

## 3. Step 2: compiled chain (f6d5859)

`src/safety_filter/arm_fk.py`: a Numba kinematic-tree pass over the whole robot-only model (37 bodies, 22 joints), with the fixed parameters read once at startup. It produces xpos, xmat, xipos, xanchor and xaxis in MjData's layout, plus the hand Jacobian of mj_jacBody.

Two MuJoCo conventions were found by the first check, which failed:
- a slide joint's anchor is taken before the slide moves the frame (pedestal and finger slides);
- mj_jacBody uses the body's frame origin (xpos), not its centre of mass (xipos).

After both were matched, `experiments/safety_filter_fk_test.py` passes on 100 random joint states (base free joint included):

| Quantity | Worst difference vs MuJoCo | Bar |
|---|---|---|
| Body position | 1.3e-15 m | 1e-4 m |
| Body centre of mass | 1.3e-15 m | 1e-4 m |
| Joint anchor | 1.3e-15 m | 1e-4 m |
| Body rotation | 1.3e-15 | 1e-6 |
| Joint axis | 2.2e-15 | 1e-6 |
| Hand Jacobian | 2.2e-15 | 1e-6 |

## 4. Step 3: wiring (b8c1efd, WIP)

- `RobotShapes.set_state` and `set_arms` call `RobotKinematics.update`; the cache check compares against the last qpos.
- `place_arrays` is one compiled pass (`arm_fk.place_kernel`); the formulas are the old einsum ones.
- `hand_frame` uses body ids found at startup.
- `ArmKinematics.jacobian_step` uses `arm_fk.hand_jacobian`.
- The tick's anchors read the same arrays (no change needed in tick.py).

`experiments/safety_filter_fk_wiring_test.py` passes on 100 random states against the old MuJoCo formulas: shape arrays 1.6e-15 m, hand frame 1.3e-15, Jacobian step 2.6e-14 rad.

Remaining MuJoCo kinematics in `src/safety_filter`: `config.arm_reach_m` (one call per process, at startup, cached). Not per tick.

## 5. Replay, back to back, same window (CPU ms per call, 3200 approach ticks)

| Measure | Before (f988735) mean / median / p95 | After (b8c1efd) mean / median / p95 |
|---|---|---|
| Resolver | 3.41 / 2.22 / 14.05 | 3.10 / 2.26 / 11.20 |
| Braking | 3.37 / 3.05 / 5.34 | 3.01 / 2.89 / 4.73 |
| Full tick | 6.78 / 5.38 / 18.15 | 6.11 / 5.40 / 14.26 |

Maximum tick: 163.3 ms before, 94.6 ms after. The medians are unchanged; the mean and the tail improve.

Commands sent: not identical. The first difference is 2.8e-15 at tick 12; the largest is 1.9e-3 (joint angles, rad).

## 6. Integration (plugin recorder, ms per call; same VM window)

| Measure | Before (f988735) | After (b8c1efd) | brkM2 (earlier VM window, Phase M) |
|---|---|---|---|
| Episode wall | 207 s | 200 s | 268 s |
| Filter calls | 5950 | 5843 | 5950 |
| Full call mean | 9.93 | 9.18 | 13.23 |
| Full call **median** | **8.22** | **7.75** | 9.88 |
| Full call p95 | 19.19 | 16.24 | 28.36 |
| Full call max | 151.9 | 231.9 | 743.0 |
| Resolver mean / median | 6.36 / 4.57 | 6.02 / 4.61 | 8.60 / 5.49 |
| Braking mean / median | 3.57 / 3.20 | 3.16 / 2.91 | 4.63 / 4.18 |
| Calls over 6 ms | 78.2 % | 69.9 % | 91.5 % |

## 7. Criteria for step 4

| Criterion | Result | Verdict |
|---|---|---|
| Episode completes | SUCCESS (both) | met |
| Hand speed at or below cap (0.2 m/s) | peak 0.1403 m/s after, 0.1379 before | met |
| No false emergencies | 0 emergency lines (both); 0 overrides (both) | met |
| Full call median under 6 ms | 7.75 ms | **not met** |
| Behaviour identical to brkM2 (stop starts) | 5 starts after, 6 before | **not met** |
| Closest gap within 0.1 mm of brkM2 | +0.031 mm after, −0.027 mm before (difference 0.058 mm) | met, but the sign changed |
| Statuses identical | HOLD 1 / STEP_SHORTENED 23 / KONG_PUSH 565 / PASS 5254 after; 2 / 52 / 670 / 5226 before | **not met** |

Stop starts, before (Phase M code): 60.87, 61.13, 65.69, 73.23, 73.54, 75.39 s (256 stop ticks).
Stop starts, after: 60.89, 61.15, 65.12, 72.29, 72.60 s (201 stop ticks).
The braking-layer accel peak changed from 2.862 to 3.001 m/s² (the deferred 2.86 event; it now reads higher).

## 8. Why the behaviour changed: sensitivity, not a wiring error

- The same code is bit-for-bit reproducible across sessions (replay commands identical, largest difference 0.0 between the Phase M run at 16:42 and the baseline at 16:56). So the timing is not the cause.
- The wiring's numbers match the old formulas to 1e-15 (section 4).
- Injecting 1e-15 noise into the replay's commanded input (the same code, unchanged) gives the same kind of divergence: 1.2e-14 at tick 12, 1.5e-5 at tick 100, 1.3e-3 at tick 500, 3.0e-3 at worst.

So the closed loop, with its discrete decisions (halving steps, pair sets, look-ahead), amplifies any change at the rounding level into a different trajectory. A reimplementation that is not bit-identical to MuJoCo cannot keep the trajectory.

## 9. Test results (28 safety-filter test files, b8c1efd)

| Result | Files | Notes |
|---|---|---|
| exit 0 | 24 | includes the two new tests (fk_test, fk_wiring_test) |
| **braking_j: exit 1, a regression from the wiring** | 1 | Fix 3 checks: adaptive wake on 15.991 mm vs off 16.252 mm (limit 0.05 mm); largest tick difference 0.556 mm. The same file passes at f988735 (exit 0, run in a scratch worktree). |
| spec: exit 1, already failing at f988735 | 1 | "the closing right arm moves" (exact-equality check). Same failure at f988735 (scratch run). Not caused by this change. |
| touch: exit 1, test-code error | 1 | `never_close_list()` calls `mujoco.mj_geomDistance(m, d, a, b, 0.5, fr)`; MuJoCo 3.13 rejects the argument types. Fails before any filter kinematics runs. Not caused by this change; not run at f988735. |
| sequential_push: exit 1, missing fixture | 1 | `data/safety_filter/kong_single_pair_fixtures.json` is produced by `scripts/safety_filter_kong_single_pair_fixtures.py`, which has not been run in these worktrees, and the file is not in the main repo. Not caused by this change. |

## 10. Not done, and decisions for David

Not done:
- The 6 ms goal (7.75 ms median in the integration).
- Identical behaviour (failed; section 8).
- The braking_j regression (section 9).
- Profiling the resolver's remaining median cost (4.6 ms in the integration) and the braking median (2.9 ms).

Decisions for David:
1. **Step 3 wiring.** Options:
   a. Revert it (recommended here). The measured gain is 0.47 ms of median in the integration (6%), and it changes the trajectory and breaks braking_j's fix-3 checks. Keep `arm_fk.py` (tested, unused).
   b. Make the kinematics bit-identical to MuJoCo: mirror MuJoCo's quaternion arithmetic (mju_mulQuat, axis-angle, quat-to-matrix) and check that the replay commands are identical. This keeps the behaviour, but the expected gain is still about 0.5 ms, so it does not reach 6 ms.
   c. Accept the new trajectory as equivalent. Then the braking_j fix-3 expectations need your review, because that test compares two runs that the rounding change moves apart.
2. **Where the remaining time is.** The arm kinematics is not the bottleneck. The next target is the resolver's tick machinery (4.6 ms median in the integration; Phase L profile: Tick.gap, candidate machinery, guard fractions). Decide whether to continue the performance work there.
3. **Pre-existing test problems** (touch's call signature, sequential_push's missing fixture, spec's closing-arm check). Not fixed here; say if you want them fixed.

## 11. Files

- Code: `src/safety_filter/arm_fk.py` (new), `src/safety_filter/shapes.py`, `src/safety_filter/arm_conversion.py`.
- Tests: `experiments/safety_filter_fk_test.py`, `experiments/safety_filter_fk_wiring_test.py` (new).
- Decisions: `docs/mpvi_decisions.md` rows SF-103 (f6d5859) and SF-104 (b8c1efd).
- Replay outputs: `data/bench_O_20261118_165602/` (before), `data/bench_O_20261118_170536/` (after).
- Integration: recordings `data/safety_filter/plugin_runs/brkO_before_20261118_165602.npz` and `brkO_after_20261118_170536.npz`; episode folders `data/braking_episode/brkO_*`; logs `data/logs/brkO_*.log`.
- Regression: `data/logs/o_reg_20261118_170536.summary` (per-test logs `o_reg_20261118_170536.<test>.log`).
- Sensitivity run: `data/bench_O_20261118_170536/pert1e-15.npy`, log `data/logs/o_pert_1e-15.log`.
