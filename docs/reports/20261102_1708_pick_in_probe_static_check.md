# Pick IN move with sag gain 0: error, sag comparison, static gravity check, 3 mm check (branch held-weight-ramp-sag-off)

Written for: David (design review). Read-only probe (data/bounce_ramp_20261102/probe_new/in_probe.csv) and a short static check
(data/bounce_ramp_20261102/static_check.md). Scripts: experiments/pick_error_probe.py, experiments/gravity_static_check.py.
Error = reference minus measured pad centre; "pad" = the left pad centre (the tool point plus the pad offset).

## 1. Error per tick, sag gain 0 (run: sag off everywhere)
- Attempt 1 IN move: 15.07 s long, ends at t = 86.27 s (timeout), error at the end (x, y, z) = (-2.16, -2.35, -0.86) mm, norm 3.31 mm,
  rotation 1.07 deg. VERIFIED.
- Last 1 s of IN (t = 85.27 to 86.27): x moves from -2.22 to -2.16 mm, y from -2.37 to -2.35 mm, z from -0.83 to -0.86 mm; the rotation
  error goes from 1.070 to 1.066 deg. Plain meaning: the error is almost flat, not closing. VERIFIED.
- 1 s after IN (t = 86.27 to 87.17): flat to 0.01 mm and 0.001 deg. The error plateaus; it does not keep shrinking. VERIFIED.
- Attempt 2 IN: the probe's last row is at t = 115.19 s (the run ends there), so no 1 s after window exists for attempt 2. Its last 1 s
  (114.20 to 115.19) looks the same as attempt 1: (-2.17, -2.37, -0.84) mm at the end. VERIFIED for the last 1 s only.
- Meaning: with the sag correction off, the arm settles about 3.3 mm short of the waypoint and stays there. The move times out at 3 mm.

## 2. Sag correction at the end of IN, existing sag-on run (data/closed_loop/on_20261029_1453/instr_ticks.csv, tick 10,998)
- Position: x -2.10, y -2.67, z -1.30 mm (norm 3.64 mm). Rotation: -0.00999, -0.00276, -0.01380 rad (norm 0.0173 rad = 0.99 deg).
  VERIFIED (values read from the file).
- Meaning: with the arm unloaded, the integral correction had learned about the same shortfall the probe measured without it
  (3.31 mm and 1.07 deg here). The two runs differ in time, so the match is INFERRED, not a measured equality.

## 3. Static check at the IN waypoint (hand empty, sag 0, settled 10 s, speed at the end 8.3e-5 rad/s)
Left arm joints (N m for torque; diff over kp in rad). VERIFIED by running the script:
| joint | servo | model | diff (servo - model) | diff / kp (rad) |
|---|---|---|---|---|
| L1 | -5.893 | -5.893 | +0.0007 | +0.00001 |
| L2 | -1.506 | -1.528 | +0.0218 | +0.00031 |
| L3 | -0.638 | -0.561 | -0.0768 | -0.00110 |
| L4 | +3.350 | +3.342 | +0.0078 | +0.00013 |
| L5 | +0.020 | +0.025 | -0.0055 | -0.00055 |
| L6 | -0.033 | +0.027 | -0.0599 | -0.00599 |
| L7 | -0.553 | -0.522 | -0.0304 | -0.00304 |
Plain meaning: at rest the servo torque matches the gravity model to within 0.08 N m on every joint. Joint L6 is the largest mismatch, 0.006 rad.
The residual, moved to the pad centre (Jacobian at the settled pose), is 1.1 mm (x -0.05, y -0.87, z +0.67 mm; the sign convention is
such that the arm sits at target minus diff/kp). INFERRED from that: the model does not explain the 3.3 mm. Its x and z parts point the
other way from the probe's error, and only the y part has the same sense. Not resolved.
A 3 s settle was not at rest (0.038 rad/s); the table above is from 10 s. A first 3 s table is discarded.
Gravity model bodies (src/robot_specs.py, ARM_ACTUATOR_SPECS["gravcomp_bodies"], VERIFIED): link1 to link7, hand, hand_tcp, right_finger,
left_finger (each with the arm's prefix, both arms). Left out (VERIFIED from src/gravity_ff.py docstring): the held object (only its
measured mass, via the ramp, as a straight-down force at the pad), the beaker, the hotplate, the bench, the base, the pedestal and the cameras.
Note: the joint ordering of the probe's joints (arm.qadr) is assumed to be L1 to L7; the static check depends on that. INFERRED.

## 4. The 3 mm check (src/mpvi_modules.py, step_inner, HEAD line 357 to 369) VERIFIED
    err = norm(p - st["target"])                 # p = arm.pad_pose(data): the pad centre position, the tool point plus the pad offset
    ang = norm(rotvec(R.T @ st["R"]))           # rotation error, pad frame vs the move's target frame
    if u >= 1.0 and err < tol_m and ang < tol_rad and settled(t, settle_dwell_s): advance
    elif el > timeout_factor * T + timeout_extra_s: fail("not reached after ...")
- What it compares: the measured pad centre to the move's WAYPOINT (not to the sag-corrected reference, so the sag correction does not enter this test).
  Position is checked with norm (3D); rotation is checked too (2 deg, tol_rad). A move needs both.
- Settling: no fixed wait. The time law must be finished (u >= 1), then the joints must be slower than 0.02 rad/s for 0.2 s (settle_dwell_s),
  and both errors must be inside their limits on that tick. Settle speed and dwell: src/task_config.py PICK_PLACE_CFG.
- Timeout: 3 times the planned time plus 6 s. For IN (3.0 s planned) that is 15.0 s, which matches the failure message ("not reached after 15.1 s").

## Open for David
- The 3.3 mm droop is not explained by the static gravity residual (1.1 mm). Something else (dynamic, or joint friction/damping at the pose) is
  likely; not checked here.
- The sag-on comparison in section 2 is across runs.
