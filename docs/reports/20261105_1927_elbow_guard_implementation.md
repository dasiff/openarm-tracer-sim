# Elbow height guard: discrepancy, minimum pedestal, implementation and baseline check

Date: 2026-11-05, 19:27. Commits: 4f2b6d1 (guard, local); this report and the progress log follow it. Not pushed.
Written for: David (the guard value, the minimum pedestal, and the baseline result).

## 1. The discrepancy (1.1152 m vs 1.1825 m): resolved

- **Correct value at the settled base: 1.1152 m** (left), 1.1229 m (right), at pedestal 0.16 m, zombie pose, joint origin `openarm_<side>_link4`. Base height 0.1527 m (`stance_finder.robot_geometry`).
- **Where 1.0225 came from:** `z − pedestal = 1.0225` is what you get at the XML spawn base height of **0.22 m**, not the settled height. At 0.22 m the left elbow reads 1.1825 m at pedestal 0.16 m (and the right 1.1902 m). The difference, 67 mm, is exactly the base height difference (0.22 − 0.1527 = 0.0673 m). The XML's `base_link` is at 0.22 m as a spawn value; the free base settles at 0.1527 m in the pipeline.
- The constant that matters is **z − pedestal = 0.9552 m (left), 0.9629 m (right)**, at the settled base.

## 2. Minimum pedestal

Guard: **table top 0.960 + housing 0.1188 + allowance 0.012 + margin 0.014 = 1.1048 m** (joint centre). Pedestal needed: q + 0.9552 = 1.1048.

| Arm | Constant (z − q) | Minimum pedestal | Elbow at 0.16 m (margin) | Elbow at 0.15 m (margin) |
|---|---|---|---|---|
| Left | 0.9552 | **0.1496 m** | 1.1152 (+10.4 mm) | 1.1052 (**+0.4 mm**) |
| Right | 0.9629 | 0.1419 m | 1.1229 (+18.1 mm) | 1.1129 (+8.1 mm) |

Both 0.16 m and 0.15 m satisfy the guard. 0.15 m passes by only 0.4 mm on the left arm. The current pedestal override is 0.15 m (`BENCH_STANCE_CFG["pedestal_override_m"]`); I did not change it, since that was not asked.

## 3. Implementation (commit 4f2b6d1)

- `src/task_config.py`: `ELBOW_GUARD_CFG` (enabled switch, minimum 1.1048 m, and the four parts with plain-word comments and row M265).
- `src/elbow_guard.py` (new): `ElbowGuard`. It evaluates the elbow joint centre (`xanchor` of `openarm_<side>_joint4`, the link4 origin) on a scratch copy of the state, using the measured base pose and the row's commanded pedestal. A target below the minimum is clamped: the arm moves from its measured angles toward the target only as far as the minimum allows (bisection, 30 steps). If the measured arm is already below the minimum, it holds its measured angles and does not go lower. The clamp counts hits and prints the first hit per arm.
- `src/control_loop.py` (`ctrl_block`): the clamp runs on each physics step's arm row, after the ramp block is built, so the ramp continues from clamped values. The guard is built next to the sag layer.
- `experiments/elbow_guard_test.py` (new, added to `run_tests.sh`): minimum value; zombie at 0.15 m passes unchanged on both arms; zombie at 0.14 m holds; a target below the minimum is clamped onto the segment and ends at or above 1.1048 m; switch off passes unchanged. **All pass.**
- `docs/mpvi_decisions.md`: row M265 (decided by David, 2026-11-05; type addition).

The guard acts on the commanded targets. The actual elbow can still lag the target by a few mm (report 20261105_1700 measured up to 6.8 mm), and that lag is not in the 1.1048 m minimum. The 0.012 m allowance is a value you gave; the logs did not test it.

## 4. Baseline check (identical, as required)

| Check | Before (HEAD 2699075) | After (commit 4f2b6d1) | Result |
|---|---|---|---|
| Manual bench scenario (`cc_bench_scenarios.sh manual`, headless) | 3060ae41…c82 | 3060ae41…c82 | **Byte-identical. Also identical to the registered CURRENT manual baseline (INDEX.csv, sha 3060ae41…).** |
| Held 200 g pick-and-place (`run_mass.py` 200 g, liftoff + episode) | liftoff 9148a081… | liftoff 9148a081… | **Byte-identical (liftoff and episode logs). The guard never fired.** |

Notes on the held run:
- The registered held baseline (`held_200g_20261104_m258`, sha 76c44f66…) **cannot be reproduced at HEAD, with or without this change.** The sag layer (M262 to M264) changed the held run after e50815d, so the reference predates it. The HEAD and after runs match each other, which is the identity that matters for this change.
- At HEAD the held plan fails at segment 2 (IN, 3.4 mm), as recorded in the M264 progress note. Both runs fail identically.
- The launcher `data/bounce_check_20261103/run_mass.py` (git-ignored) reads trace keys renamed by the sag layer (`C_p`), so it crashes at frame 9687 with the trace on. For these runs I used a scratch copy, with those logging keys read defensively. Only logging columns changed; the physics did not.

## 5. What was not checked

- **The clamp did not fire in any run here.** The identity results show that when the guard is idle the output is unchanged. The active path (clamping a target) is covered only by the unit test. A deliberate-violation run (for example pedestal 0.14 m, or the zombie rise at a low pedestal) would show the clamp in the loop, and would change the physics; I did not run one without your decision.
- **Teleop and policy bench scenarios:** no CURRENT baseline exists (NO_CURRENT in INDEX.csv), so nothing to compare.
- **The full test suite** (about an hour) was not run; only `elbow_guard_test` was run.
- **Performance:** the clamp runs two forward-kinematics evaluations per arm per physics step (about 3,000 per second of sim). The wall time of the held runs was similar before and after (about 4 minutes each), but I did not profile it.
- **Real-robot tilt and sag,** the 0.012 m allowance, and the placeholder arm gains (report 20261105_1700) are unchanged by this work.
- **The status of the held baseline:** a new reference for the held run is needed before the next comparison. That is a regeneration decision for you; I did not regenerate it.

## 6. Decisions for David

1. **Pedestal:** the guard holds at 0.15 m with 0.4 mm of margin on the left arm. Keep 0.15 m, or move to 0.16 m (10.4 mm margin)?
2. **New held reference:** regenerate `held_200g` at HEAD (the sag-layer behaviour is now the intended one), or keep the old reference retired?
3. **Clamp test:** run a deliberate-violation scenario (a low pedestal) to see the clamp act in the loop, accepting that the outcome changes?
4. **Performance:** accept the clamp cost as measured, or profile it before the next run?
