# Project TODO List

Consolidated 2026-10-09. "Waiting on" says who has to answer or act before an item can move; items with no name are ours.
The original roadmap (from before this work, partly out of date) is kept at the end, unchanged.

## HAND TESTS (need a real keyboard / display; waiting on: you)

Everything below was only exercised with injected keys (scripted `held()` functions) and screenshots; none of it with a person at the keyboard.

- [ ] **Real keys:** drive (arrows, Space), arm and gripper jog (`1`-`8` / `Q`-`I`, `A`-`K` / `Z`-`,`), `F2` (teleop mode: arms keyboard / webcam), the neck keys
  (`9` `0` `-` `=` / `O` `P` `[` `]`), `F7` (frame the task points), `F8` (neck default aim), `F9` (take over), `Enter` (hand back), `Delete` (abort).
- [ ] **Confirm the alert beep is audible** (it is a generated WAV played with `aplay`; `aplay` exits cleanly on this VM, but nobody has heard it) and that **`Delete` is a
  comfortable abort key** (Esc was not usable: it quits the viewer; the key is the `abort` entry of `hotkeys` in `src/episode_runner.py`).
- [ ] **Test webcam teleop on a real camera** (waiting on: a real camera; the VM has none). Only the scripted fake operator (`CC_FAKE_TELEOP=1`) has been tried. Check the
  preview colours, the engage gesture and how the arms behave when the operator leaves view.

## QUESTIONS FOR THE TEAM

- [ ] **Neck** (waiting on: the team, via Slack): which 4 SO-101 joints are active (assumed shoulder pan / lift / elbow flex / wrist flex), the real mount location and height
  (now the top of the torso plate, `NECK_SPECS`), and the camera mass. The STS3215 gains and the camera module on the end are placeholders; the neck adds 0.655 kg, which already
  changes the arm-swing transient when the base starts (see the base yaw transient below).
- [ ] **Eagle camera** (waiting on: Luke / whoever owns mounting): the real mount position. Also the real Orbbec model, field of view and resolution of the eagle and wrist
  cameras; their poses, field of view and resolution in `CAMERA_SPECS` are all placeholders.
- [ ] **Gripper** (waiting on: the hardware team): the real force limit (replaces the 10 N per finger placeholder ceiling, `ARM_ACTUATOR_SPECS["fingers"]["force_limit"]`),
  stiffness (the placeholder `kp` = 100 N/m means a finger cannot push harder than `kp` times its distance from the commanded position, 3.7 N on the 70 mm beaker, so caps above
  that do nothing), current limit, and whether the real gripper supports force / current readback. **On real hardware the operator's gripper commands (keyboard, teleop)
  should also carry a force cap**: today they carry no `max_force`, so they reset the cap to the ceiling.
- [ ] **Base** (waiting on: Michael): how the real Tracer holds position when stopped. Also check the sim's turn rate and acceleration against the real Tracer before tuning the arm
  swing (the arms swing ~20 cm during fast turns at 2 rad/s and end ~2 cm off after stopping).
- [ ] **Pedestal** (waiting on: Dan): the real lift specs. The actuator is a placeholder (travel, 0.05 m/s jog speed, the lowest command is a torso-contact stop).
- [ ] **Planner** (waiting on: Dan): the MPVI interface, base placement, and the reachability map (see the follow-up below).
- [ ] **Compute** (waiting on: the PACE GPU allocation): a GPU allocation for openpi.
- [ ] (original questions to the team: see the archive at the end: the learned IL model format, the control interface.)

## SIM DECISIONS PENDING (waiting on: you)

- [ ] **5 mm post-ARMS correction trigger.** The closed-loop base correction this list used to ask for exists (after PEDESTAL with the full controller, after ARMS creep-only
  with clearance checks; `correction_*` in `STAGING_CFG`), but it only fires outside 1 cm / 1 degree. The HANDOFF error is now 8.5 mm / 0.36 degrees, about 8 mm of it along the
  heading and away from the bench (clearance a little larger, reach a little shorter: it uses about half of the grasp slack). Enabling the trigger at `correction_aim_frac` x the
  tolerance (5 mm) would correct it; not enabled. (Before the corrections the base drifted about 3 mm / 0.9 degrees during PEDESTAL and 6 mm during ARMS, and 4 mm over a 60 s hold.)
- [ ] **Crush check:** fail the episode if the beaker's contact force exceeds a threshold.
- [ ] **Exclusivity during teleop:** key it on actual motion, not on commands (today it is on in episodes and applies to the operator's keys too).
- [ ] **Launcher:** rename `experiments/combined_controller.py` (it is no longer the controller; the loop is `src/control_loop.py`), and whether the launcher gets an auto mode
  (its `--mode auto` is still the unimplemented placeholder).
- [ ] **Shadows and render rate for the VLA evaluation cameras.** The loop renders the policy cameras (shadows on, 30 Hz, round robin) on a fixed schedule when `cfg["cameras"]` is
  on; at ~200 ms per render on this VM that is most of the wall-clock time. Proposal: render them only when the policy requests an observation (the loop's `render` hook already
  renders on demand), and decide shadows and rate for that case; demo collection renders offline from a recording instead (`src/replay.py`).
- [ ] **Dataset format for openpi training frames**, and with it the **replay video codec**: the OpenCV build has no H.264 (no libx264), so previews are MPEG-4 part 2 (`mp4v`;
  `--quality` is ignored); VP9 (`--codec VP90`) is ~15% smaller; H.264 needs an ffmpeg dependency (imageio-ffmpeg). The preview videos are lossy: do not train from them
  (`--video training` writes lossless PNG frames named by tick, ~230 MB per sim minute) until the format is decided.

## SIM FOLLOW UPS

- [ ] **Lift test:** does a gentle grip (~1 N) hold the beaker while lifting? Needs a pick stage (the grasp test only closes on the beaker, `experiments/gripper_force_test.py`).
- [ ] **Base yaw transient on drive start; sensitive to torso mass.** The arms swing when the base starts (re-test now that the PD loop runs at 750 Hz every physics step; other
  leads: mixed actuator gain / bias settings and the 3.5 mm finger overlap at the zero pose); the neck's 0.655 kg already changed it by up to 4.6 degrees of yaw in the manual baseline.
  The arm hold itself is fixed (MuJoCo position actuators + upstream armature + gravcomp, `data/arm_hold/suite_summary.txt`); what remains is the ~20 cm swing during fast turns
  (91% force on left j1) and the ~2 cm offset after stopping.
- [ ] **Stiffer stopped base in sim** (waiting on: Michael, once the real base behaviour is known).
- [ ] **Snap-back: the arm's way back is not collision checked.** After a hand-back the arm and gripper glide back to their path in a straight line in joint space / finger travel
  (`arm_max_speed` 1.0 rad/s, `gripper_max_speed` 0.05 m/s), which is not checked against the bench; the operator may have left the arm anywhere. The neck's look-ahead aim during
  DRIVE..ARMS is not restored (NECK frames it later anyway). A later stage should plan the way back, or ask the operator to return the arm to a safe pose first.
- [ ] **Remove the test-only switches** `RESTORE_ON_RESUME` and `COMPENSATE_PAUSES` from `src/staging_source.py` into config if they cause confusion (they exist for the control runs of the tests).
- [ ] **Reachability map for the planner** (back burner).
- [ ] **Drive the grippers in teleop.** The solver outputs only the 7 arm joints per side, so the grippers stay on the keyboard in teleop.
- [ ] **Teleop could follow the operator's head** instead of the grippers once a head pose is available from the webcam path.
- [ ] **Viewer speed on the VM.** The window redraws at 30 Hz and the operator insets are their own cheap render (shadows off, 320x240, each camera at 1 Hz): a viewer episode runs at
  ~0.9x real time (was 0.11x), the launcher at ~0.95x. A camera render costs ~25-40 ms on this VM whatever its size (insets at 2 Hz: ~0.75x, 5 Hz: ~0.65x). Open: the VM's 3D acceleration
  and video memory settings; why physics is ~2x slower per tick with the viewer open than headless (0.7 vs 0.4 ms per step).
- [ ] **Camera image realism.** The floor and lighting wash out to white; consider lower floor reflectance and lighting for the camera views.
- [ ] **Do not draw per-frame text with `mjr_overlay`** on this VM (a caution, not a task): its driver leaks a resource per glyph and aborts the process after about 250 frames. Use bitmap text (see the viewer HUD).
- [ ] Housekeeping: report the provisioner problem upstream (it rejects the openarm product); finish checking for Windows-side commits that never reached GitHub.

## NEXT PHASES

- [ ] **Teleop demos** once a working teleop arm exists (record with `--record-run`, render offline with `experiments/replay_run.py`).
- [ ] **openpi integration** (waiting on: the dataset format decision and the PACE GPU allocation above).

---

# ARCHIVE: original roadmap (from before this work; partly out of date, kept unchanged)

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

