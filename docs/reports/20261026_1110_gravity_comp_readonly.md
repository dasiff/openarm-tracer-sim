# Gravity compensation: read-only findings (Part C, 2026-10-26)

No code was changed and nothing was run. Everything below is from reading the repo.
Parts A (push) and B (place angle) of the earlier prompt were not done; the interrupting message replaced it with Part C only.

## Summary

- The sim drives each arm joint with a MuJoCo **position actuator**: `force = kp*(ctrl - q) - kv*qdot`, clipped to the motor's torque limit. `ctrl` is the target angle. Gains are the real `control_gains.yaml` values.
- **Gravity compensation in the sim is not a controller output.** It is MuJoCo's per-body `gravcomp = 1` on the arm links, hands and fingers. It uses the true link masses and acts inside the physics. It does not appear in `actuator_force`, so the "motor torque" signal the sim exposes does not include the arm's own gravity load. A real motor's reported torque would. This matters for a weight estimate from torque (see item 7).
- The held beaker (0.2 kg) is not compensated. The arm sags, and a **scripted Cartesian integral** (`sag_gain_per_s` 3/s, limits 4 cm and 0.1 rad) corrects it by moving the waypoint. This is the "integral term" that the (c) plan would replace.
- An extra torque on top of the target **is possible**, with no change to the actuator type. Two ways; the cleaner one is `ctrl = q_target + tau_ff/kp` (item 3).
- The repo has **no computation of gravity's pull on each joint** in `src/`. The only use of `qfrc_bias` is a diagnostic experiment.
- Beaker mass is set in `src/task_config.py` (glass 0.100 + liquid 0.100 kg). **Nothing in the control path reads a mass value.** Three near-misses to know about are listed in item 5.
- The real config file is **in the repo's download cache**, not in git: `control_gains.yaml` has kp and kd only. There is **no gravity compensation setting** in it.

## 1. How each arm joint is driven

`src/arm_actuators.py` (applied through MjSpec before compile, always on); numbers from `src/robot_specs.py` `ARM_ACTUATOR_SPECS`.

- Actuator type: position actuator built by `_make_position`: gaintype fixed, `gainprm[0]=kp`, biastype affine, `biasprm=[0,-kp,-kv]`. So `force = kp*(ctrl - q) - kv*qdot`. The velocity term is integrated implicitly by MuJoCo.
- `ctrlrange` = the joint's range. `forcerange` = +-`torque_nm`.
- Per joint (same on both arms):

| joint | motor | kp (N m/rad) | kv (N m s/rad) | torque limit (N m) | armature (kg m^2) |
|---|---|---|---|---|---|
| 1 | DM8009 | 70 | 2.75 | 40 | 0.0081 |
| 2 | DM8009 | 70 | 2.5 | 40 | 0.0081 |
| 3 | DM4340 | 70 | 2.0 | 27 | 0.16 |
| 4 | DM4340 | 60 | 2.0 | 27 | 0.16 |
| 5 | DM4310 | 10 | 0.7 | 7 | 0.0100 |
| 6 | DM4310 | 10 | 0.6 | 7 | 0.0100 |
| 7 | DM4310 | 10 | 0.5 | 7 | 0.0100 |

- Other terms: armature (rotor inertia, restored from the upstream model); integrator set to `implicitfast`; gripper is one position actuator per hand (kp 300, kv 4.6, force limit 9 N, per finger, doubled for the one actuator); gravcomp (item 4). No integral term, no feedforward torque.
- The kp and kv equal the real controller's (item 6); the real one runs at 750 Hz.

## 2. The current sag correction

- Where: `src/mpvi_modules.py` (pick/place module, `step()`, around lines 296-303), settings in `PICK_PLACE_CFG` (`src/task_config.py:417`): `sag_gain_per_s` 3.0, `sag_max_m` 0.04, `sag_max_rad` 0.1.
- What it does: each tick, during a "move" waypoint, it computes the pad pose error `p_ref - FK(measured joints)` (and the rotation error), integrates it (`C += gain * err * dt`, clipped in norm), and adds `C` to the pad target given to the mink IK (`arm.step(...)`). So it is a **Cartesian integral on the waypoint**, not a joint torque. Per the skeleton report it is reset when the gripper opens. Waypoints end only when the measured pad pose is within tolerance and the arm is settled.
- Why: gravcomp does not carry the beaker, so the first run stopped at UP, 18 mm short (`docs/reports/20261020_1356_mpvi_skeleton_runs.md`, item 1). TODO.md notes a 100 mm lift moved the hand 90 mm (placeholder gains then).
- How it was tuned: **not tuned by a recorded sweep.** The design proposal (`20261020_1323_d3_design_proposal.md` section 6) proposed k about 2/s and a 2 cm clamp; the built values are 3/s, 4 cm, 0.1 rad with no recorded reason. Decision row M112 lists them as undocumented, [PENDING David]. They were set against the 0.2 kg beaker, so they implicitly assume that load.
- M035 says this scripted correction is removed when a payload-aware feedforward is built.

## 3. Can an extra torque be added on top of the target?

Yes. The real Damiao motors run MIT mode: `tau = kp*(q* - q) + kd*(qd* - qd) + tau_ff`, then limited. The sim actuator has the same first two terms. Two ways to add `tau_ff`:

- **(a) Fold it into the target (preferred):** write `ctrl = q_target + tau_ff / kp` per joint. The actuator then outputs exactly `kp*(q_target - q) + tau_ff - kv*qdot`, and `forcerange` clips the total as MIT mode would. Caveats: `ctrlrange` equals the joint range, so near a limit the shifted target is clipped; and for joints 5-7 (kp 10) a 1 N m torque shifts the target by 0.1 rad, so it stays inside range only away from limits.
- **(b) `data.qfrc_applied[dofadr]`**: a direct generalised force. It adds exactly, but bypasses `forcerange` and does not show up in `actuator_force`, so the torque signal would no longer match what a real motor reports. Not recommended.

Where it goes: the one place `data.ctrl` is written for the arms is `src/control_loop.py`: arm targets arrive in the command's `arm_targets` (around lines 524-549) and `advance()` writes `data.ctrl[:] = ctrl_steps[k]` (about line 818). A feedforward would be added where `ctrl_steps` is built, using the measured `qpos`. It does not exist yet.

## 4. Does the code compute gravity's pull per joint?

- **In `src/`: no.** There is no `qfrc_bias`, `mj_inverse`, `mj_rne` or similar in the control path.
- **Experiments only:** `experiments/arm_hold_diagnosis.py:252` reads `d.qfrc_bias[dofs]` (gravity generalised force, at qvel 0) and compares it with `d.qfrc_gravcomp` (line 253). This is a one-off diagnostic, with the MuJoCo model's masses.
- What does exist: MuJoCo's own gravcomp (`gravcomp = 1.0` on link1-link7, hand, hand_tcp and both fingers of both arms; `robot_specs.py` `gravcomp_bodies`, `arm_actuators.py` line ~64). It cancels each link's true weight inside the physics. It is not computed by our code, not visible as a torque, and exact because the "model" is the sim itself.
- Related: `src/pedestal.py:42-55` `pedestal_load` uses `body_mass` and `body_gravcomp` to size the lift servo. It is a load-time report, not in the control path.
- M035 plans a model-based feedforward from the arm and gripper links (no payload); it is "not built".

## 5. The beaker's true mass (answer key for tests) and mass reads in the control path

Defined in:
- `src/task_config.py:120-121` `BEAKER_CFG`: `glass_mass` 0.100 kg, `liquid_mass` 0.100 kg (total 0.200 kg). Row M081.
- Built into the model in `src/beaker.py` (glass mass split over wall segments and bottom by volume, lines ~84-87; liquid as a separate massive geom, line ~89-91, mass only, no collision).
- Pour scene: both beakers 0.100 kg glass, no rigid liquid; contents are 40 balls of 2.5 g (`ball_mass` in `POUR_CFG`, `src/pour_scene.py:112`), so total 0.200 kg when full.
- In a test, read it from the compiled model: `model.body_mass[model.body("task_beaker").id]` (the 0.2 kg is the glass plus the liquid geom; `experiments/beaker_hollow_test.py:74` does this).

Does anything in the control path read a mass? **No mass value is read.** What I found:
1. `src/world_model.py:323` and `:361` read `liquid_mass > 0` to set the world model's `"contents": "liquid"/"empty"`. A yes/no flag, not a weight, but it comes from the config, and the world model feeds the planner and checker. It should stay a flag.
2. **Indirect dependence on the weight:** the grip force comes from a fixed table, `GRIP_FORCE_TABLE["glass_beaker"]` (used in `mpvi_modules.py`), which was chosen using the 0.2 kg beaker (`docs/grip_force_sweep.md`). The sag correction constants (item 2) were also set against this load. Neither reads a mass, but both silently assume it.
3. `src/pedestal.py` `pedestal_load` reads `body_mass`, only in `experiments/pedestal_lift_test.py` and as spec documentation. Not in control.
4. Experiments (`grasp_lift_test.py`, `gripper_force_test.py`) read the config masses to compute the weight for their checks. Tests only.

## 6. The real controller settings

Not in git, but in the repo's gitignored download cache (`data/`, entry `data/import_cache`), the pinned `enactic/openarm_description` (commit 1fba2cb, per `robot_specs.py`):

- `data/import_cache/openarm_description/assets/robot/openarm_v1.0/config/arm/control_gains.yaml`
- `data/import_cache/openarm_description/assets/robot/openarm_v2.0/config/arm/control/control_gains.yaml`

Contents (the arm values are identical in both files):
```
joint1: kp 70.0  kd 2.75
joint2: kp 70.0  kd 2.5
joint3: kp 70.0  kd 2.0
joint4: kp 60.0  kd 2.0
joint5: kp 10.0  kd 0.7
joint6: kp 10.0  kd 0.6
joint7: kp 10.0  kd 0.5
hand (v1.0 file only): kp 5.0  kd 0.1
```
**No gravity compensation setting, no feedforward torque, no integral gain** appears in either file; a search of the cache's yaml, md and xacro files for "gravity" found nothing. Whether the real stack (`mobile_bimanual_ros2`) sends a gravity `tau_ff` is an open question for Yuanshao (design plan section 11, M035). What is known from Yuanshao: the Damiao motors use MIT impedance control with Kp and Kd only, no Ki.

The sim's `gravcomp = 1` is therefore an assumption ("assumes the real controller compensates for gravity", comment in `arm_actuators.py` and `robot_specs.py`), not a copy of a real setting. The hand's real kp/kd (5/0.1) are in motor units and are not used; the sim's gripper gains are placeholders.

## 7. Joint signals the sim exposes that the real robot also has

From `src/control_loop.py:148-169` (the observation) and `src/simulator.py:192-208`:
- `joint_angles` = copy of `qpos`: angle (real: encoder). Has it.
- `joint_velocities` = copy of `qvel`: speed (real: motor reports velocity). Has it.
- `joint_torques` = copy of `actuator_force`: the torque the servo applied (real: motors report torque or current). Has it, with the caveats below.
- `ctrl` = the commanded targets: the robot knows what it commanded.
- Not allowed for the estimate: anything from `model` (masses, `body_mass`), contact forces, `qfrc_*`, the beaker's pose from the simulator.

Caveats for a weight estimate:
- Because of MuJoCo gravcomp, `joint_torques` **excludes the arm's own gravity torque**. For an unloaded still arm it is about 0; with the beaker it contains the beaker's torque plus the servo's holding error. A real motor's reported torque would include the arm's own gravity load unless the controller subtracts it. To make the sim honest, one would either build the feedforward (item 3a, so the torque signal includes it) or have the estimator subtract the model's arm-only gravity torque (item 4) from a measured total. This is a design decision for David before building.
- `joint_torques` is the actuator output after `forcerange` clipping, so a saturated joint reads the limit, not the demand.
- The torque is the position servo's `kp*(ctrl - q) - kv*qdot`, so while the arm sags the signal and the angle error carry the same information. The estimate can use either or both; a still arm (settled, speeds below `settle_speed_rad_s`) is needed, as in (c).
