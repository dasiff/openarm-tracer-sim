# Project TODO List

## Current open items (updated 2026-10-02)

The phases below are the original roadmap and are partly out of date: the controller, simulator,
logger, chemistry lab scene, cameras and webcam teleop all exist now (see `git log`).

- [ ] **Test webcam teleop on a real camera.** Only the scripted fake operator
  (`CC_FAKE_TELEOP=1`) has been tried; the VM has no camera. Check the preview colours, the
  engage gesture and how the arms behave when the operator leaves view.
- [ ] **Drive the grippers in teleop.** The solver outputs only the 7 arm joints per side, so the
  grippers stay on the keyboard in teleop.
- [ ] **Arm swings when the base moves** (`combined_controller.py`). Re-test now that the PD loop
  runs at 750 Hz every physics step; the old once-per-frame control update was one suspect.
  Other leads: mixed actuator gain/bias settings, and the 3.5 mm finger overlap at the zero pose.
- [ ] Arm hold fixed: MuJoCo position actuators + upstream armature + gravcomp (see
  data/arm_hold/suite_summary.txt). Remaining: arms swing ~20 cm during fast turns (2 rad/s, 91%
  force on left j1) and end ~2 cm off after stopping. Check sim turn rate/acceleration against
  the real Tracer before tuning.
- [ ] **Base drift during staging.** In the beaker episodes the base meets the 1 cm / 1 degree arrival tolerance at the end of DRIVE, then drifts
  about 3 mm / 0.9 degrees during the PEDESTAL move and 6 mm during ARMS, so the pose at HANDOFF is about 8 mm / 1.9 degrees off (and 4 mm / 0.07 degrees
  more over a 60 s hold). The staging pose search's clearances and reach margins absorb this, but a closed-loop base correction after PEDESTAL would remove it.
- [ ] **Confirm the camera neck specs.** The SO-101 mount (top of the torso plate, `NECK_SPECS`), which 4 joints are active
  (assumed shoulder pan / lift / elbow / wrist flex), the STS3215 gains and the camera module on its end are placeholders;
  the neck adds 0.655 kg, which already changes the arm-swing transient when the base starts (up to 4.6 degrees of yaw in the manual
  baseline). Teleop could follow the operator's head instead of the grippers once a head pose is available from the webcam path.
- [ ] **Real camera parameters.** Eagle and wrist camera poses, field of view and resolution are
  placeholders; replace them when the Orbbec model and mounting are known.
- [ ] **Camera image realism.** The floor and lighting wash out to white; consider lower floor
  reflectance and lighting for the camera views.
- [ ] **Viewer speed on the VM** is about 8-13 fps. The remaining cost is the wrist-camera shadow
  passes; try the VM's 3D acceleration and video memory settings. Recording demos with
  `CC_ROUNDROBIN=0` is slower still.
- [ ] **Do not draw per-frame text with `mjr_overlay`** on this VM: its driver leaks a resource per
  glyph and aborts the process after about 250 frames. Use bitmap text (see the viewer HUD).
- [ ] Report the provisioner problem upstream (it rejects the openarm product).
- [ ] Finish checking for Windows-side commits that never reached GitHub.

---

## Phase 9: Control Layer (Next - Week 2)

- [ ] Create `src/controller.py`
  - [ ] Adapt PD controller from `openarm_mujoco/keyboard_control.py`
  - [ ] Support joint angle targets → motor torques
  - [ ] Test with basic trajectory

- [ ] Update simulator to accept control commands
  - [ ] Modify `MaterialsLabSimulator.step()` to take action input
  - [ ] Validate command format

## Phase 10: Materials Lab Environment (Week 2-3)

- [ ] Design scene in `src/materials_lab_env.py`
  - [ ] Add workbench (collision box)
  - [ ] Add material containers (trays, bins)
  - [ ] Add sample objects (cubes, cylinders)
  - [ ] Position objects realistically

- [ ] Test environment loads and renders
  - [ ] Verify no collision errors
  - [ ] Check object positions make sense

## Phase 11: Data Logging (Week 3)

- [ ] Create `src/logger.py`
  - [ ] Record joint angles, velocities, torques
  - [ ] Record task events (gripper closed, object picked, etc.)
  - [ ] Save to HDF5 format

- [ ] Test logging works
  - [ ] Run simulation, save trajectory
  - [ ] Verify file contents

## Phase 12: Learned Policy Integration (Week 3-4)

- [ ] Understand learned IL model format
  - [ ] Where is it stored? (ask team)
  - [ ] What format? (.pt, .pth, .pkl?)
  - [ ] Input/output interface?

- [ ] Create `src/policy_wrapper.py`
  - [ ] Load trained model
  - [ ] Run inference on simulator state
  - [ ] Interface with controller

- [ ] Validate in simulation
  - [ ] Run policy on sim environment
  - [ ] Log trajectory
  - [ ] Verify task completion

## Calibration (When Hardware Arrives)

### PRIORITY 1: Critical for Sim-to-Real Transfer

- [ ] Measure motor torque constants
- [ ] Measure joint friction and damping
  - [ ] Test each joint individually
  - [ ] Record force/torque vs. velocity
  
- [ ] Measure control latency
  - [ ] Send command from ROS node
  - [ ] Measure time until motor responds
  - [ ] Account for network delay
  
- [ ] Measure backlash and deadbands
  - [ ] Reverse direction, measure lag
  - [ ] Record all joint-specific values
  
- [ ] Update `src/robot_specs.py` with real measurements

- [ ] Test sim matches real robot
  - [ ] Run same trajectory on both
  - [ ] Compare end-effector positions
  - [ ] Measure error metrics

### PRIORITY 2: Important for Accuracy

- [ ] Verify inertia matrices from CAD
  - [ ] Compare simulation motion to real
  - [ ] Adjust if necessary
  
- [ ] Measure gripper grip force
  - [ ] Test with objects of different weights
  
- [ ] Characterize wheel slip
  - [ ] Command base movement
  - [ ] Measure actual vs. expected displacement

### PRIORITY 3: Nice-to-Have

- [ ] Electromagnetic noise/ripple in motors
- [ ] Temperature effects on motor performance
- [ ] Long-term wear patterns

---

## Documentation TODO

- [ ] Write `docs/ARCHITECTURE.md`
  - [ ] System overview
  - [ ] Data flow diagram
  - [ ] Component responsibilities

- [ ] Write `docs/SETUP.md`
  - [ ] Complete setup from scratch
  - [ ] Troubleshooting common issues

- [ ] Write `docs/DECISIONS.md`
  - [ ] Why MuJoCo?
  - [ ] Why this environment structure?
  - [ ] Design rationale

- [ ] Write `docs/CALIBRATION.md`
  - [ ] How to measure robot properties
  - [ ] How to update simulation
  - [ ] Validation procedures

---

## Infrastructure TODO

- [ ] Create `src/validator.py`
  - [ ] Compare sim vs. real trajectories
  - [ ] Compute error metrics
  - [ ] Ready for when hardware arrives

- [ ] Set up domain randomization
  - [ ] Randomize friction, inertia, latency
  - [ ] Make learned policies robust

- [ ] Create CI/CD (optional)
  - [ ] Test that simulator loads
  - [ ] Test that examples run
  - [ ] Catch regressions

---
## Questions for team
- [ ] Understand learned IL model format
  - [ ] Where is it stored? 
  - [ ] What format? (.pt, .pth, .pkl?)
  - [ ] Input/output interface?

- [ ] Verify control interface
  - [ ] ROS 2 topics or Python API?
  - [ ] Expected latency?
  - [ ] Communication protocol?

