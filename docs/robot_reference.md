# Robot reference values (not read by any code)

These values used to be dictionaries in `src/robot_specs.py` (`TRACER_SPECS`, `OPENARM_SPECS`, `CALIBRATION_STATUS`, `CALIBRATION_TODO`). No code read them, so
they moved here. The values that the simulator does use are still in `src/robot_specs.py`.

## Agilex Tracer X2 mobile base (was `TRACER_SPECS`)

| Item | Value | Note |
|---|---|---|
| Name | Agilex Tracer X2 | |
| Control frequency | 750 Hz | the old `PD_HZ`, equal to the physics rate |
| Drive wheel friction | 1.5 | high grip |
| Drive wheel gear ratio | 10 | motor to wheel |
| Drive wheel maximum torque | 10 N m | estimated from the gearing |
| Caster wheel friction | 0.1 | low friction for turning |
| Pedestal joint | one slide joint, 0 to 0.418 m | the updated description (B1c); 0 = collapsed, the torso is 0.3048 m + the joint value above the pedestal base |
| Pedestal masses | 16.19 kg fixed, 30.05 kg moving | our old estimates (the description gives none) |
| Pedestal joint damping | 50000 N s/m | the old coupled pair's 800 + 50000, so the lift speed for a given force is unchanged |

(The simulated base is velocity-driven, see `src/base_control.py`; the wheels have no traction in the model.)

## OpenArm bimanual manipulator (was `OPENARM_SPECS`)

| Item | Value | Note |
|---|---|---|
| Name | OpenArm Bimanual | |
| Arms | 2 | left and right |
| Degrees of freedom per arm | 7 | joints |
| Gripper motor model | DM4310 | |
| Gripper motor type | position control | one actuator per hand on finger 1, finger 2 follows it (`src/arm_actuators.py`); force ceiling 9 N per finger (assumed per finger, see design plan section 11) |
| Gripper maximum opening | 0.044 m | per finger |

The per-joint motor, gains, armature and torque limits are in `ARM_ACTUATOR_SPECS` in `src/robot_specs.py`.

## Calibration status (was `CALIBRATION_STATUS`)

| Item | Status |
|---|---|
| Inertia matrices | ESTIMATED (from CAD, need real measurement) |
| Friction coefficients | ROUGH ESTIMATE (need measurement) |
| Control latency | MODELED AS 0.002 s (actual ROS latency unknown) |
| Backlash and deadband | NOT MODELED (need measurement when hardware is available) |
| Motor response | MODELED FROM SPECS (need tuning) |
| Camera intrinsics | PLACEHOLDER (Orbbec model unknown; resolution and field of view guessed) |
| Camera mounting | PLACEHOLDER (poses guessed until mounting information arrives) |

## Calibration to-do, for when hardware arrives (was `CALIBRATION_TODO`)

Priority 1 (critical for sim-to-real):
- [ ] Measure actual motor torque constants
- [ ] Measure joint friction and damping
- [ ] Measure control latency with ROS overhead
- [ ] Measure backlash and deadbands

Priority 2 (important):
- [ ] Verify inertia matrices from CAD
- [ ] Measure gripper grip force
- [ ] Characterize wheel slip and friction

Priority 3 (nice to have):
- [ ] Electromagnetic noise and ripple
- [ ] Temperature effects on motors
- [ ] Wear patterns over time

The global joint damping the model used (`GLOBAL_DAMPING` = 0.4) was never applied by any code and was removed.
