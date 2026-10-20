# Project TODO List

Consolidated 2026-10-11. "Waiting on" says who has to answer or act before an item can move; items with no name are ours.
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

- [x] **Planner** (answered by Dan, 2026-10-15): implement MPVI as in the paper first: the LLM decides the step order, the scripted staging stands in for its navigation module, the VLA does the manipulation, no LLM fine-tuning (no annotated lab plans). The A* task-level search is our own extension, LATER. Staging takes a reachability-map stance instead of the paper's handoff distance. Details and build order: `docs/design_plan.md` §§8 and 9. Still to build (B7): the LLM step order, the orchestrator and the world model; staging must accept a stance input by B4.
- [ ] **Neck** (waiting on: the team, via Slack): which 4 SO-101 joints are active (assumed shoulder pan / lift / elbow flex / wrist flex), the real mount location and height
  (now the top of the torso plate, `NECK_SPECS`), and the camera mass. The STS3215 gains and the camera module on the end are placeholders; the neck adds 0.655 kg, which already
  changes the arm-swing transient when the base starts (see the base yaw transient below).
- [ ] **Eagle camera** (waiting on: Luke / whoever owns mounting): the real mount position. Also the real Orbbec model, field of view and resolution of the eagle and wrist
  cameras; their poses, field of view and resolution in `CAMERA_SPECS` are all placeholders.
- [ ] **Wrist cameras and the camera model** (waiting on: the hardware team): where each wrist camera is mounted on the hand (position and angle; the sim has the same placeholder
  offset on both hands, `CAMERA_SPECS`), and the camera model: field of view and resolution (the sim uses one placeholder, `ORBBEC_FOVY_DEG` 58 and `ORBBEC_RESOLUTION` 640 x 480,
  for all three cameras). Once known, matching them is an edit of `CAMERA_SPECS` (plus the neck's mount in `NECK_SPECS` for the eagle camera) and regenerating the demos.
  Also goes into §11 of docs/design_plan.md once that exists.
- [ ] **Gripper** (waiting on: the hardware team): the real force limit (replaces the 9 N per finger ceiling set at B1c, itself assumed per finger, `ARM_ACTUATOR_SPECS["fingers"]["force_limit"]`),
  stiffness (the placeholder `kp` is now 300 N/m: a finger pushes `kp` times its distance from the commanded position, 11 N on the 70 mm beaker, so caps up to the 10 N ceiling
  take effect; it was 100 N/m, 3.7 N), current limit, and whether the real gripper supports force / current readback.
- [ ] **Finger pad torsional (twisting) friction** (waiting on: the hardware team): the real gripper's value; the sim uses 0.005 (`BEAKER_CFG["contact_friction"]`). It decides how freely a pinched beaker pivots about the pinch axis (see the weak-grip item below).
- [ ] **Home pose** (waiting on: the hardware team): the real OpenArm home pose (`home_q_left` / `home_q_right` in `TASK_CFG` is a placeholder "zombie pose"), and where the
  wrist cameras should look from it.
- [ ] **Base** (waiting on: Michael): how the real Tracer holds position when stopped. Also check the sim's turn rate and acceleration against the real Tracer before tuning the arm
  swing (the arms swing ~20 cm during fast turns at 2 rad/s and end ~2 cm off after stopping).
- [ ] **Pedestal** (waiting on: Dan): the real lift specs. The actuator is a placeholder (travel, 0.05 m/s jog speed, the lowest command is a torso-contact stop).
- [ ] (original questions to the team: see the archive at the end: the learned IL model format, the control interface.)

## INVESTIGATE

- [x] **RESOLVED (merge fe8c3d8): the robot leans while driving straight.** Fixed by the low-friction contact points of the base (`src/base_contacts.py`). Original report: the base height rises from 0.153 m to 0.190 m during the 0.3 m/s cruise and settles back over 7+ s, overlapping the PEDESTAL and
  ARMS phases. Likely causes: the velocity-based base driving (`src/base_control.py` writes the free-joint velocity), floor friction, and a high centre of mass. It may explain part of
  the staging drift (the base moves about 3 mm / 0.9 degrees during PEDESTAL and 6 mm during ARMS before the corrections; the HANDOFF error is now 8.5 mm / 0.36 degrees). The episode log
  has no base height column, so start by logging `qpos[2]` and the base roll and pitch during DRIVE.

## SIM DECISIONS PENDING (waiting on: you)

- [ ] **5 mm post-ARMS correction trigger.** The closed-loop base correction this list used to ask for exists (after PEDESTAL with the full controller, after ARMS creep-only
  with clearance checks; `correction_*` in `STAGING_CFG`), but it only fires outside 1 cm / 1 degree. The HANDOFF error is 8.5 mm / 0.36 degrees, about 8 mm of it along the
  heading and away from the bench (clearance a little larger, reach a little shorter: it uses about half of the grasp slack). Enabling the trigger at `correction_aim_frac` x the
  tolerance (5 mm) would correct it; not enabled.
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

## REALISM (planner independent)

- [ ] **Estimated base position** (noise, drift) instead of ground truth: the staging stage and the pose corrections use the true base pose today.
- [ ] **Object positions from camera images or noisy estimates** instead of reading them from the model.
- [ ] **Randomized object positions, with the parking spot chosen per episode** (today the scene and the staging pose are fixed and nothing is randomized).
- [ ] **Policy response time:** the simulation keeps running while a policy thinks (it does not wait for the policy's inference).
- [ ] **Camera image realism.** The floor and lighting wash out to white; consider lower floor reflectance and lighting for the camera views.

## SIM FOLLOW UPS

- [ ] **Lift test:** does a gentle (~1 N) grip hold the beaker while lifting? Needs a pick stage (the grasp test only closes on the beaker, `experiments/gripper_force_test.py`).
- [ ] **Base yaw transient on drive start; sensitive to torso mass.** The arms swing when the base starts (re-test now that the PD loop runs at 750 Hz every physics step; other
  leads: mixed actuator gain / bias settings and the 3.5 mm finger overlap at the zero pose); the neck's 0.655 kg already changed it by up to 4.6 degrees of yaw in the manual baseline.
  The arm hold itself is fixed (MuJoCo position actuators + upstream armature + gravcomp, `data/arm_hold/suite_summary.txt`); what remains is the ~20 cm swing during fast turns
  (91% force on left j1) and the ~2 cm offset after stopping.
- [ ] **Snap-back: the arm's way back is not collision checked.** After a hand-back the arm and gripper glide back to their path in a straight line in joint space / finger travel
  (`arm_max_speed` 1.0 rad/s, `gripper_max_speed` 0.05 m/s), which is not checked against the bench; the operator may have left the arm anywhere. The neck's look-ahead aim during
  DRIVE..ARMS is not restored (NECK frames it later anyway). A later stage should plan the way back, or ask the operator to return the arm to a safe pose first.
- [ ] **Real hardware: the operator's gripper commands should carry a force cap.** The keyboard and teleop gripper commands carry no `max_force`, so they reset the cap to the
  ceiling (10 N per finger, a placeholder).
- [ ] **Reachability map and stance finder** (`docs/design_plan.md` §3, B2): needed by the demo generator's arm rule (the arm with the better reach score for the grasp), by the staging's stance input (B4) and by MPVI (B7).
- [ ] **Stiffer stopped base in sim** (waiting on: Michael, once the real base behaviour is known).
- [ ] **Remove the test-only switches** `RESTORE_ON_RESUME` and `COMPENSATE_PAUSES` from `src/staging_source.py` into config if they cause confusion (they exist for the control runs of the tests).
- [ ] **Drive the grippers in teleop.** The solver outputs only the 7 arm joints per side, so the grippers stay on the keyboard in teleop.
- [ ] **Teleop could follow the operator's head** instead of the grippers once a head pose is available from the webcam path.
- [ ] **Viewer speed on the VM.** The window redraws at 30 Hz and the operator insets are their own cheap render (shadows off, 320x240, each camera at 1 Hz): a viewer episode runs at
  ~0.9x real time (was 0.11x), the launcher at ~0.95x. A camera render costs ~25-40 ms on this VM whatever its size (insets at 2 Hz: ~0.75x, 5 Hz: ~0.65x). Open: the VM's 3D acceleration
  and video memory settings; why physics is ~2x slower per tick with the viewer open than headless (0.7 vs 0.4 ms per step).
- [ ] **Do not draw per-frame text with `mjr_overlay`** on this VM (a caution, not a task): its driver leaks a resource per glyph and aborts the process after about 250 frames. Use bitmap text (see the viewer HUD).
- [ ] **Friction 0.05 breaks PEDESTAL settling** (cause unknown). Found in the friction sweep (0.05, 0.1, 0.2) of the lean fix; the approved value is unchanged. Investigate why the pedestal does not settle at the lowest friction.
- [ ] **Gravity compensation experiment.** MuJoCo's body gravcomp cancels the arms' weight at each body, so arm weight never loads the pedestal, base or wheels: the floor carries 94 N less than the robot's weight.
  Measure how the base lean, wheel contact and drive behave without arm gravcomp (the arms then need the actuators to hold their weight), and decide whether the sim should model the real load path.
- [ ] **Recording size is now ~150 KB per sim second** (physics ctrl values per step). Watch disk space when collecting many demos; consider compressing or thinning the recordings.
- [x] **A grasped beaker creeps in the gripper** (decided: noslip on, `NOSLIP_ITERATIONS = 10` in `src/robot_specs.py`, enabled on this branch). At 5 N per finger (friction 0.4, capacity 4 N against
  the 1.96 N weight) the 100 g + 100 g beaker slid 10.3 mm in a 2 s lift and kept creeping 3.8 mm/s (MuJoCo's soft friction contacts act like a viscous damper). With 10 no-slip iterations:
  2.7 mm of slip, no creep over a 10 s hold (3 iterations: 3.9 mm and still creeping 0.03 mm/s, at the same step cost, so 10 stays). Cost: physics steps take ~1.9x as long (0.95 against 0.49 ms
  on this VM; real time at 750 Hz is 1.33 ms). Side effect: at the 10 N ceiling the two
  fingers' forces on the beaker are 9.5 and 10.5 N (mean 10.0), so `gripper_force_test.py` checks the ceiling closing's mean against the cap (3%) and each finger within 10%.
- [ ] **The arm sags under the beaker's weight** (placeholder arm gains): a lift commanded 100 mm moves the hand 90 mm with the beaker in the gripper. Part of the grasp test's numbers;
  it goes away with the real controller gains.
- [ ] **The right wrist camera does not see the beaker from home** (it sees the hotplate at the left image edge): the objects are on the robot's left, the beaker is ~48 degrees off the
  right camera's axis, and a wrist pitch cannot fix that. Matters only if the policy needs the right wrist view of the beaker.
- [ ] **Search and view scripts read the old solid beaker** (`model.geom("task_beaker_glass")`): `beaker_hotplate_layout_search.py`, `beaker_hotplate_staging_search.py`,
  `beaker_hotplate_eagle_views.py` and `beaker_hotplate_pose_check.py` fail with the hollow beaker (they are design-time tools for the legacy `ready_q`, `ready_via_q`, `grasp_q`
  poses, which are kept). Update them to `BEAKER_CFG` (and to the many-geom beaker) when the layout is next re-searched.
- [ ] **`beaker_hotplate_episode.py` writes a fixed path under `data/`** (`data/beaker_hotplate_eagle/episode_handoff.png`), so a test run overwrites that reference picture; and
  its printed "arrival ... (tolerance 10 mm / 1 deg)" is stale (the yaw tolerance is 0.5 degrees now). Give it an output option / read the tolerance from `STAGING_CFG`.
- [ ] Housekeeping: report the provisioner problem upstream (it rejects the openarm product); finish checking for Windows-side commits that never reached GitHub.

## NEXT PHASES

- [ ] **openpi on PACE** (approved, $1000 budget). Needs the dataset format decision above (what the training frames look like) and a planner-independent policy interface.
- [ ] **Teleop demos** once a working teleop arm exists (record with `--record-run`, render offline with `experiments/replay_run.py`).
- [ ] **RAIL safety filter:** `src/safety_filter.py` (arm joint angles through geo_kin's collision filter) is the starting point.

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
- [ ] **The face's size and look are a first guess** (`FACE_CFG`: a 15 x 18 x 16 cm ellipsoid, the face image 12.5 x 15.6 cm on its front): adjust to taste. It has no neck-joint animation, only
  neutral / smile; a recording made before the face existed cannot be replayed (the replay fingerprint changed to leave the face out, which also drops the materials, textures and names).
- [x] **Neck: frame once per skill, then hold** (decision 2026-10-14, built 2026-10-14; the eagle camera is a policy input, so its behaviour must be the same in demos, recorded teleop and policy runs, and a following neck is a feedback loop with the policy). Repeatable solve, staging NECK solves once and a hand-back re-sends the stored targets, `neck_auto` frames once per stop, the operator's neck keys are off while a replayable recording is written (`neck_once_test.py`). Left to do: the free teleop's own `follow` mode (`NECK_AUTO_SPECS["follow"]`, off) is untested in a real unrecorded session; the decision goes into docs/design_plan.md when that is written; the neck's real mount and the real camera (questions above) change what the framing sees.
- [x] **The 5 N grasp is marginal: its slip flips between two outcomes for tiny changes** (resolved 2026-10-15: the table is now 8 N, see the end of this item; found 2026-10-15 when the battery pack was added; `experiments/grasp_lift_test.py --no-control`, logs `data/logs/battery_grasp_variants_20261015_181727/`). Without the battery, and with a 11.0 kg battery, the beaker slips 2.5 mm and does not creep (the "holds without the 3 mm slip bar" line says True). With 11.3 kg (the real pack) it slips 4.25 mm and still creeps 0.65 mm/s, with 11.6 kg 4.07 mm and 0.46 mm/s (False). (Corrected 2026-10-15: the first version of this note named the wrong masses. Repeats of one configuration are bit-identical, 4 runs with random Python hash seeds and 2 with a fixed one, so the flip is the physics, not run-to-run noise.) The test's pass/fail checks pass in all cases (the beaker is carried up 86 to 88 mm, the cap binds at 5 N), but a 0.3 kg change in an unrelated mass (11.0 to 11.3 kg) flipping the creep says the grasp sits next to a contact-solver bifurcation (the same family as the 2 N creep below). Check before demo generation: sweep small perturbations (start pose, finger gain, friction) and look at the slip distribution, and think about what the generator should do (a stronger grip, a lower lift speed). Done for the force (2026-10-15, `docs/grip_force_sweep.md`): 8 N per finger holds at every tested mass and at friction 0.32; the table change awaits David's approval (design plan B3b).
- [ ] **Weak (2 N) grip: the beaker creeps upward about 15 mm/s while the hand stands still.** Is it real pinch ejection from the beaker's tilt, or a contact solver artifact? Check before demo generation (the 2 N control of `grasp_lift_test.py`; the clip with the beaker's tilt logged is in `data/notebook/`). Finding (2026-10-15, `data/notebook/control_2N_2026-10-14_tilt.csv` / `_tilt.png` / `_clip.mp4`): the "creep" is the beaker tilting, not rising: its tilt grows from 0.5 deg at the start of the lift to 23 deg when the hand stops (2.0 s), 48 deg at 3.2 s, then it topples to 90 deg at 3.5 s and lies on its side (its height 35 mm = its radius). So it pivots about the pinch axis and then falls over. Still open: whether a real glass beaker pinched at two points pivots this freely (the sim's torsional friction at the pads is 0.005) or it is a contact artifact.
- [ ] **PENDING: grasp height vs the beaker's centre of mass.** Compute the centre of mass height (empty, and with the liquid mass setting) and where it falls in the 1/4 to 60 % grasp band; grips below it are unstable about the pinch axis; decide whether to limit the band, at least for pouring. Computed 2026-10-15: centre of mass 43.3 mm above the bottom (43 % of the 100 mm height) for the empty beaker (100 g glass) and 29.9 mm (30 %) with the 100 g liquid setting (200 g in all); both are inside the 25 to 60 % band, so the part of the band below the centre of mass is 25 to 43 % empty (18 of the band's 35 mm) and 25 to 30 % with the liquid (5 mm). The grasp test pinches at 50 % (measured: force-weighted pad contact height 50.1 mm), above the centre of mass in both cases, so the 2 N run was a grip above it: its tilt and topple (docs/TODO.md) is a weak grip losing the beaker, not yet shown to be the below-centre-of-mass instability. A grasp below the centre of mass has not been tested.