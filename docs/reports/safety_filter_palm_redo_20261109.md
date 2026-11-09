# Safety filter: gravity feedforward masses, frame refit without the tool point, palm study redone (2026-11-09)

Read-only, kinematics only (no MuJoCo stepping), no change to src/ or config. Model: `load_task(MOVED, cameras=True)`, code at main.

## 1. Gravity feedforward masses

Which masses are read: `src/gravity_ff.py:76-84` builds `self.bodies` from `robot_specs.ARM_ACTUATOR_SPECS["gravcomp_bodies"]` (`src/robot_specs.py:167-168`): `link1` to `link7`, `hand`, `hand_tcp`, `right_finger`, `left_finger`, for each arm. Line 83 keeps a body only if `model.body_mass[b] > 0`; line 84 stores its mass. Every name in this list is a robot link. **No world object is in this list.**

What feeds it:
- **Arm gravity term** (`gravity_ff.py:114-118`): the arm's own link masses, through `mj_jacBodyCom`, give the gravity holding torque. Active in every run with the robot (always).
- **Held-weight term** (`gravity_ff.py:119-125`): `m_held = held_mass_ramp.mass_at(m, s, data.time)` (line 120) is multiplied by `G` and the pad Jacobian (line 125). It is gated by `BEAKER_GRAVITY_CFG["enabled"]` (`src/task_config.py:711-716`, default `True`; `gravity_ff.py:121`). This term does not read `body_mass` of the beaker.
- `mass_at` (`src/held_mass_ramp.py:31-42`) returns the ramp record's `start`, `target` and time (no model mass). The ramp is set by `set_target` (`src/held_mass_ramp.py:85-99`): its start is the current ramp mass (`cur`, line 93), not the model's beaker mass.

Who sets the held mass (every setter in `src/` and `experiments/`):
- **Normal runs:** `WeighHelper` (`src/weigh_helper.py`) is built for each hand in `src/control_loop.py:614` and always on (`control_loop.py:104` comment), stepped every tick at `control_loop.py:1081`. When the holding verdict turns on, it calls `hmr.set_target(model, side, est, t, tick)` (`weigh_helper.py:115`) with `est = liftoff_estimate(...)` (`src/weight_estimate.py:126`, from the pad's measured rise). If no lift-off up to the cap, it sets 0 (`weigh_helper.py:122`). **This is the robot's own weighing estimate.**
- **Replay:** `control_loop.py:1022` sets the held mass from the recorded `heldmass` of the replay (the recorded measured mass).
- **Diagnostics:** `experiments/bounce_diag_extract.py:94-97` sets the recorded measured mass or `None`.
- `gravity_ff.set_held_mass` (`gravity_ff.py:38-44`) is the wrapper used by the last two.

Answer:
- Does any body mass read at `gravity_ff.py:83-84` belong to the beaker or another world object? **No.**
- Does the held-weight term ever use the model's beaker mass? **No.** The model beaker mass (`src/task_config.py:121-122`: glass 0.100 kg, liquid 0.100 kg, so 0.200 kg) is used only by MuJoCo physics. The held term uses the measured estimate (or 0) from `held_mass_ramp`.
- In which runs is the held term active: every run with a grasp, since weighing is always on and `BEAKER_GRAVITY_CFG["enabled"]` is `True`. It is off only if that flag is set to `False` (the bare-VLA experiment, `task_config.py:711-716`).
- Other places that read model beaker mass: `src/pedestal.py:53` sums `body_mass` over the bodies under the pedestal joint (root = the pedestal joint's body). The beaker is a free-joint body outside that subtree, so it is not included. The bounce logging (`experiments/bounce_check_20261103/run_mass.py`, `beaker_wt_N`) reads the beaker's model weight for diagnostics only.

## 2. Frame refit without the tool point

Fit on the shoulder (S), elbow (E) and wrist (W) points only. 200 random poses per arm, other arm at zombie, same seeds as the facts report (20261109+1 left, +2 right). Residuals are reported for all four points, including the tool point T, which is not in the fit.

| arm | key point | old fit (with T): mean / max (m) | new fit (no T): mean / max (m) |
|---|---|---|---|
| left | S (shoulder) | 0.0469 / 0.0469 | 0.0396 / 0.0396 |
| left | E (elbow) | 0.0132 / 0.0172 | 0.0201 / 0.0238 |
| left | W (wrist) | 0.0128 / 0.0209 | 0.0197 / 0.0270 |
| left | T (tool, not fitted) | 0.0876 / 0.1014 | 0.0895 / 0.1085 |
| right | S | 0.0463 / 0.0463 | 0.0399 / 0.0399 |
| right | E | 0.0141 / 0.0160 | 0.0201 / 0.0217 |
| right | W | 0.0140 / 0.0178 | 0.0199 / 0.0233 |
| right | T (not fitted) | 0.0872 / 0.1008 | 0.0888 / 0.1066 |

Comparison: removing the tool point does not help. The elbow and wrist residuals rise from about 1.3 cm to about 2.0 cm, and the tool residual stays near 9 cm. The tool point was the only point that anchored the hand's orientation in the fit, so the fit without it has more freedom at W. The shoulder residual falls slightly and is still a constant offset.

## 3. Palm study, redone with the new fit

### Hand frame spread (50 poses per arm)

| arm | max vertex displacement (m) | threshold (m) | verdict |
|---|---|---|---|
| left | 0.0450 (mean 0.0243) | 0.005 | **stop for this arm** |
| right | 0.0377 (mean 0.0244) | 0.005 | **stop for this arm** |

Why: the hand body is rigid in MuJoCo's own link-7 frame (zero spread, checked earlier). The solver's `R_0_7` matches link 7 exactly, so the orientation is right. The spread comes from the origin at W, which goes through the fit: its residual rises from 1.3 cm to 2.0 cm without the tool point, and it varies with pose. The spread got worse (left 3.3 cm to 4.5 cm; right 2.8 cm to 3.8 cm), so the origin is not rigid enough for a palm study.

Because the rule stops the arm at step 1, the palm fits (step 2), finger capsule (step 3d), wrist coverage (step 4) and grasp clearance (step 5) are **not reported** for either arm. The script computed them, but they rest on a hand frame that fails the check.

### Not reported (stopped)
- Palm fits k = 1, 2, 3: not reported (stopped at step 1, both arms).
- Wrist coverage by the forearm capsule: not reported (stopped).
- Finger capsule (3d), both arms: not reported (stopped).
- Grasp clearance (step 5), open and closed, for k = 1, 2, 3, the real hand mesh and the finger capsule: not reported (stopped).

Two observations from the same run, given as diagnostics only: the zombie hand is not level (approach axis z −0.042 on both arms; closing axis z +0.029 left, −0.029 right), so the step 5 level check would fail for both arms too. The finger separation reaches 0.070 m at a joint value of 0.0376 m (travel 0 to 0.044 m), so the "closed to the cylinder" state is reachable.

## Not done

- Palm steps 2, 3d (finger capsule), 4 and 5 for both arms: stopped by the 0.005 m spread rule (section 3). Fixing the origin mapping at W is the blocker. The fit without the tool point makes it worse, so the fix needs a different origin definition (for example the link-7 body origin, which is rigid in MuJoCo), or a fit that keeps the tool point and puts the tool error elsewhere. That is a design choice, so I have not made it.
- The hand is not level at the zombie pose, so the step 5 grasp needs a different hand pose first (an arm IK step). Not done.
- The cylinder's gap was not reported; the level check and the spread stop cover it.
