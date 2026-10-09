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
- [ ] **Viewer speed on the VM.** The window now redraws at 30 Hz (every 2nd tick) and the operator insets come from their own cheap
  render (shadows off, 320x240, each camera at 1 Hz); a viewer episode runs at ~0.9x real time (was 0.11x) and the launcher at ~0.95x.
  A camera render costs ~25-40 ms on this VM whatever its size, so more inset updates cost real time (2 Hz: ~0.75x, 5 Hz: ~0.65x).
  Live recording plus offline replay of the policy cameras is in (`--record-run`, `experiments/replay_run.py`). Open: the VM's 3D acceleration / video memory settings; why physics is ~2x
  slower per tick with the viewer open than headless (0.7 vs 0.4 ms per step).
- [ ] **VLA evaluation: render the policy cameras only when the policy requests an observation, not every frame.** The loop
  renders policy cameras (shadows on, 30 Hz round robin) on a fixed schedule when `cfg["cameras"]` is on; at ~200 ms per render on this
  VM that is most of the wall-clock time. A policy source should ask for frames when its inference step needs them (the loop's
  `render` hook already renders on demand); demo collection renders them offline from a recorded run instead (src/replay.py).
- [ ] **After an operator hand-back the staging stage restores what it needs, but not everything.** The arm and gripper glide back to their path in a straight
  line in joint space / finger travel, not checked against the bench (the operator may have left the arm anywhere). The pedestal (after its phase) and the neck
  framing (in NECK and after DONE) are put back, and the operator's time does not count toward a phase timeout; the neck's look-ahead aim during DRIVE..ARMS is
  not restored (NECK frames it later anyway). A later stage should plan the way back for the arm.
- [ ] **Gripper force caps on real hardware.** The operator's gripper commands (keyboard, teleop) carry no `max_force`, so they reset the cap to the ceiling
  (10 N per finger, a placeholder: `ARM_ACTUATOR_SPECS["fingers"]["force_limit"]`). On the real gripper they should also carry a force cap. With `kp` = 100 N/m a
  finger cannot push harder than `kp` times its distance from the commanded position (3.7 N on the 70 mm beaker), so caps above that do nothing.
- [ ] **Replay video codec.** The OpenCV build here has no H.264 (no libx264): videos are MPEG-4 part 2 (`mp4v`; `--quality` is ignored by it).
  VP9 (`--codec VP90`, .webm) is ~15% smaller. For smaller / better video add an ffmpeg dependency (imageio-ffmpeg) and write H.264.
  The videos are lossy: do not train from them without deciding that; the dataset format openpi trains from is still open.
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

