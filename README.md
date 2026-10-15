# Robotics Simulation for Materials Lab

Simulation environment for Agilex Tracer mobile base + OpenArm bimanual robot in materials lab tasks.

Design and roadmap: see docs/design_plan.md (status of every planned item).

## Quick Start

The project has its own virtual environment (uv, Python 3.12, MuJoCo 3.13.0), set up to share the
locked versions of `openarm_teleop`. See `docs/session_log.md` for how it was built.

```bash
source .venv/bin/activate        # also sets the geo_kin license variable
python experiments/combined_controller.py                 # drive the robot with the keyboard
python experiments/combined_controller.py --mode teleop   # same, plus webcam teleoperation of the arms
python experiments/beaker_hotplate_episode.py --viewer    # the beaker-on-hotplate task: the task policy in auto mode (see below)
python experiments/replay_run.py DIR                      # replay a recording (--record-run / --record) and render its cameras to video
```

`--mode teleop` additionally needs the `openarm_teleop`, `geo_kin_core` and `XRT_devices` packages
(editable installs in the venv) and a webcam. `CC_FAKE_TELEOP=1` swaps in a scripted operator so
the teleop path can be tried without a camera.

## Controls (`experiments/combined_controller.py`)

| Keys | Action |
|---|---|
| Arrow keys, Space | drive / turn the base, stop |
| `1`-`8` / `Q`-`I` | left arm joints 1-7 and gripper: upper key increases, lower decreases |
| `A`-`K` / `Z`-`,` | right arm joints 1-7 and gripper: same layout |
| `Page Up` / `Page Down` | raise / lower the pedestal (placeholder lift actuator; the real hardware is unconfirmed) |
| `9` `0` `-` `=` / `O` `P` `[` `]` | camera neck pan, shoulder lift, elbow, wrist flex: upper row increases, lower row decreases |
| `F7` | frame the task points with the neck camera (beaker, hotplate and both grippers, when the scene has the beaker task; says so if they cannot all fit) |
| `F8` | neck back to its default aim: looking ahead, 30 degrees down |
| `F2` | (teleop mode) switch the arms between the keyboard and the webcam operator |
| `F3` | save the simulated camera images to `data/camera_snapshots/` |
| `F4` | save a snapshot of the viewer |
| `F5` | toggle trajectory logging |
| `F6` | start / stop recording the viewer image to `data/recordings/viewer_<time>.mp4`, with a matching `_joints.csv` |
| `Esc` | quit |

Columns run shoulder (joint 1) to wrist (joint 7), then the gripper; the gripper's upper key opens
it. The arm keys are held to jog. In teleop the webcam operator owns the arm joints, so only the
grippers respond to the keys then. If the operator leaves the camera's view the arms hold their
last pose. The same legend is shown in the viewer's status panel.

**Operator keys in episodes** (`experiments/beaker_hotplate_episode.py --viewer`; they come on top of the keys above, and the keyboard source is
the same, so the base, arm, neck and pedestal keys drive the robot whenever the operator is in charge):

| Key | Action |
|---|---|
| `F9` | take over from the task policy (auto to teleop); does nothing in teleop mode |
| `Enter` (or keypad Enter) | hand control back to the policy (teleop to auto); the policy continues its current stage; only while the operator has control |
| `Delete` | abort the episode now, outcome `ABORTED_BY_OPERATOR`; only while the operator has control (`Esc` still just quits the viewer) |

When the policy asks for the operator a red banner "OPERATOR NEEDED: <reason>" is shown and a short two-tone beep plays once (`src/alert_sound.py`:
a generated WAV played with `aplay`, terminal bell as a fallback); once the operator presses any key the banner turns blue and lists the keys that
apply ("OPERATOR CONTROL: Enter = hand back to policy, Delete = abort episode"). The keys are set in `src/episode_runner.py` (`hotkeys`).

**Recording (`F6`, or `--record` to start at launch; off by default).** The video is the viewer image (status panel and camera insets included). It plays
at the simulated rate (30 frames per second of sim time: one per redraw), however slowly the VM ran. The CSV has one
row per video frame, written as it goes: `video_frame`, `sim_time_s`, `wall_s`, the base pose, then
every arm joint and finger as `<joint>_pos` (actual) and `<joint>_target` (PD target). Run with
`CC_ROUNDROBIN=0` so the camera insets update at the full rate in the video.

The viewer shows the three simulated cameras (eagle and both wrists) along the bottom, plus the
operator webcam in teleop mode. Camera poses, field of view and resolution are placeholders until
the real mounting and Orbbec model are known (`src/robot_specs.py`).

**Camera neck.** The eagle camera is on the end of a 4-DOF SO-101 arm (the "neck") mounted on top of the torso
(`NECK_SPECS` in `src/robot_specs.py`: the mount and the choice of joints are assumptions pending the real
specs). The model is vendored in `models/so101/` (Apache-2.0, see `PROVENANCE.md`) and attached at load time by
`src/neck.py`. The neck starts looking ahead and 30 degrees down. It moves (it is exempt from the
one-subsystem-at-a-time rule) at no more than 1.5 rad/s. In teleop mode it looks ahead while the base drives and frames the task once each time the base stops, then holds
(see the design decision below); any neck key takes over for 3 s (the keys are off while a replayable recording is written).
`src/neck_framing.py: frame_points()` finds neck angles that keep a set of points in the image with a 10 % margin
(and reports the points that do not fit).

**Design decision (2026-10-14, implemented): the neck frames once per skill, then holds.** The eagle camera is a policy input, so its behaviour must be identical in
generated demos, recorded teleop and policy runs, and a following neck would create a feedback loop with the policy. At the start of each skill (after staging, before the operator or
the policy takes over) the neck frames the scene once and holds until the skill ends. Following stays only where nothing is recorded for training: the look-ahead while driving,
and an unrecorded free teleop mode. What is built: the framing solve (`src/neck_framing.py`) is repeatable (a weak posture term of 0.003 towards the drive pose, no cost for moving from the
start pose, a wide initial simplex; 0.1 degree of noise on the arms moves the neck by under 0.01 rad where it used to move it by about 1 rad); the staging NECK stage solves once per
episode and a hand-back re-sends the stored targets; `neck_auto` (free teleop) looks ahead while driving, frames the task points (as F7 does) once each time the base stops, then holds
(`NECK_AUTO_SPECS["follow"]` = True follows the grippers instead, never while recording); and the operator's neck keys (jog, F7, F8) are off, with a one-line notice, in any run that writes a
replayable recording (`--record` / `--record-run`). `experiments/neck_once_test.py` tests all of it.

### Performance on the VM

The development VM has only a virtual GPU, so the viewer defaults to settings that keep it responsive (about 0.9x real time in a viewer
episode, 0.95x in the launcher). The window redraws at 30 Hz (every 5th tick of the 150 Hz control tick; physics and control run every tick) and is paced to real time. The
camera insets in the window are a separate, cheap render (shadows off, 320x240, each camera at 1 Hz; `CC_INSET_HZ`, `CC_INSET_RES`, `CC_INSET_SHADOWS`,
`CC_DRAW_HZ`); the policy cameras (shadows on, 30 Hz, one camera per tick, the eagle camera without shadows) are rendered live only in policy mode
(`CC_POLICY_CAMS=0|1` overrides) and offline from a recording otherwise (see "Record live, render offline"). A camera render costs 25-40 ms on this VM, so
more inset updates cost real time. All the switches (`CC_*` environment variables, including `CC_BENCH_FRAMES=N` for per-phase timings and
`experiments/viewer_profile.py` for a profile) are documented in `_perf_settings` in `experiments/combined_controller.py`.

## Project Structure

```
openarm-tracer-sim/
├── src/
│   ├── simulator.py         # Generic MuJoCo simulator (injects the cameras at load)
│   ├── cameras.py           # Simulated Orbbec cameras, offscreen rendering
│   ├── control_loop.py      # run_loop(): the control loop (router, command validation, optional exclusivity / hold); headless-capable
│   ├── commands.py          # Mode-tagged command format (base / pedestal / arms / neck / idle / request_teleop; gripper position + max_force) and validate_command()
│   ├── base_control.py      # Twist -> free-joint velocity base drive
│   ├── base_contacts.py     # The base stands on six low-friction contact spheres (the wheel and caster meshes do not collide), added at load
│   ├── keyboard_source.py, teleop_source.py, policy_source.py   # command sources
│   ├── glfw_viewer.py       # Window, HUD, camera insets, recorder, snapshots
│   ├── neck.py, neck_framing.py, neck_source.py   # camera neck: attach the SO-101 at load, frame points, auto follow / look-ahead
│   ├── battery.py           # The battery pack (Anker SOLIX C800, 11.3 kg box on the deck behind the pedestal, BATTERY_SPECS; mount is a placeholder)
│   ├── face.py              # The robot's face: a visual-only head at the end of the neck, neutral / smile images from assets/private/face/ (git-ignored)
│   ├── staging_source.py    # Scripted staging: DRIVE -> PEDESTAL -> ARMS -> NECK -> DONE (HANDOFF), beaker-on-hotplate task
│   ├── sequential_policy.py # Generic policy of stages that run one after the other (any navigation stage, then any manipulation stage); hold / request-operator stages
│   ├── task_policy.py       # The beaker task's policy: the staging stage + an arm stage, combined by sequential_policy.py
│   ├── episode_runner.py    # Generic episode runner (a task supplies reset, policy, success check and time limits): timeouts, success, log, summary
│   ├── robot_helpers.py     # Robot-level helpers: arm actuator / joint ids, the contact monitor
│   ├── run_record.py, replay.py   # record a live run's physics inputs; replay it exactly (state check) and render its cameras to video
│   ├── rate_limit.py, alert_sound.py   # joint-target rate limiter (snap-back safety); the operator alert beep
│   ├── viewer_composite.py  # The live viewer's picture rendered offscreen from a state (replay --video viewer)
│   ├── viewer_layout.py     # The viewer's layout as plain functions (follow camera, status panel, banner, camera tiles), shared by the live window and the offscreen renderer
│   ├── robot_specs.py       # Rates, motor gains, camera specs (placeholders marked)
│   ├── safety_filter.py     # Arm joint angles through geo_kin's collision filter (starting point for the planned safety filter)
│   └── logger.py            # Trajectory logging
├── legacy/                  # controller.py, actuator_mapping.py, joint_addressing.py: kept only for old experiments (see legacy/README.md)
├── models/
│   ├── scenes/              # chemistry_lab*.xml, single_block.xml, ...
│   └── policies/            # Policy implementations
├── deps/OpenArm-Combined/   # Robot model (submodule)
├── models/so101/            # Vendored SO-101 model for the camera neck (upstream files, unmodified; see PROVENANCE.md)
├── experiments/
│   ├── combined_controller.py     # Launcher for the interactive sim: keyboard, webcam teleop, policy, cameras (loop is in src/)
│   ├── cc_bench_scenarios.sh, cc_bench_compare.py, cc_loop_tests.py   # before/after regression scenarios, headless and hold-rule tests
│   ├── episode_test_sources.py    # Test-only helpers: cheat and grasp-force arm stages, base kick source
│   ├── beaker_hotplate_episode.py # Run beaker-on-hotplate episodes (auto / teleop mode, arm stage, viewer, recording)
│   ├── replay_run.py              # Replay a recording: state check and cameras to video / PNG frames
│   ├── cc_mode_tests.py, gripper_force_test.py, beaker_hotplate_correction_test.py   # tests: modes / operator keys / snap-back / restore; gripper force limit; base pose corrections
│   ├── sequential_policy_test.py  # test of the stage combiner with made-up stages (no simulation)
│   ├── target_mode_test.py        # test of the arm / gripper target modes (interpolate / hold) and of the per-source command rate
│   ├── viewer_profile.py          # Where the wall-clock time of a viewer run goes (cProfile)
│   ├── camera_snapshot.py         # Render the simulated cameras once to PNGs
│   ├── chemistry_lab_tracer_teleop.py
│   ├── run_basic_sim.py, run_with_dummy_policy.py, test_controller.py
│   └── workspace_sweep.py, visualize_workspace.py, empty_room_joint_test.py, ...
├── data/                    # Logged trajectories, camera snapshots, diagnostics
├── docs/
│   ├── TODO.md              # Roadmap and open items
│   ├── robot_reference.md   # Reference values no code reads (Tracer base figures, OpenArm summary, calibration status and to-do)
│   └── session_log.md       # Decisions, findings and problems, by session
└── README.md
```

## Beaker-on-hotplate episodes

The controller is told who is in charge, `auto` or `teleop` (`cfg["mode"]` of `src/control_loop.py`). In `auto`, ONE task policy
(`src/task_policy.py`) owns the whole task: its staging stage (`src/staging_source.py`, speeds / ramps / tolerances in `STAGING_CFG`,
poses in `TASK_CFG`) drives to the staging pose, sets the pedestal, moves both arms to the home pose (`home_q_left` / `home_q_right`: the "waiter pose", a placeholder
until the hardware team gives the real one; an optional random variation per joint, `home_variation_rad`, is off by default) and frames the beaker,
the hotplate and both grippers with the neck camera; then its arm stage takes over (for now `hold`, or `request_operator`). The policy can ask for operator
control: the controller switches to `teleop` at once (the base stops, everything holds). In the viewer F9 takes over from the policy, Enter hands control back (the policy
continues its current stage, its arm targets gliding from where the operator left them, `src/rate_limit.py`; the pedestal and the neck framing are put back too, and the operator's time does not count toward a phase timeout) and Delete aborts the episode
(`ABORTED_BY_OPERATOR`); a banner shows "OPERATOR NEEDED: <reason>" (with a sound) after a request and the available keys while the operator
has control. Every switch is logged (`mode_switches` in the summary).
Exclusivity and the hold rule stay in the controller (on in episodes).

The episode runner is generic: `run_episode(task, model, data, ...)` knows nothing about the beaker. A task is a dictionary (see the docstring of
`src/episode_runner.py`): how to reset the scene, which policy to run, how to check success, the time limits, and how replay rebuilds the model. The beaker
task is built by `make_task()` in `src/task_beaker_hotplate.py`; its policy is `src/task_policy.py`, a sequence of stages (`src/sequential_policy.py`: a
planner can replace the staging stage and a learned policy the arm stage without touching the combining code).

`experiments/beaker_hotplate_episode.py [--mode auto|teleop] [--arm-stage hold|request_operator|cheat] [--viewer] [--repeat N]
[--cross-check] [--record] [--out DIR]` (`--mode teleop` needs `--viewer`: the operator drives from the start; `--repeat` reuses the model and compares the logs
byte for byte; `--cross-check` adds a fresh-load run; `--record` records each episode; `--out` defaults to `data/beaker_hotplate_episodes/...`) runs episodes (`src/episode_runner.py`), which only reset the scene, check success, enforce the timeouts and
save the log: success (`check_success`, after the handoff, whoever is in charge; `operator_assisted` is true if the operator had control),
`TIMEOUT` after 60 s of AUTO time after the handoff (paused while the operator has control) or 300 s of OPERATOR time
(`operator_timeout`; a teleop-start episode counts time 0 as the handoff and uses it), or `STAGING_FAILED` if staging cannot reach its pose
(1 cm / 1 degree after 3 corrective re-approaches, or a correction that is too large). Each episode writes `episode_log.csv` (10 Hz) and
`episode_summary.json`. The `hold` arm stage (expect TIMEOUT) and `cheat` (test only: teleports the beaker onto the hotplate 2 s after
the handoff, expect SUCCESS) exist to test the runner; `experiments/cc_mode_tests.py` tests the modes, the operator keys, the timeouts, the snap-back
safety (arm, gripper, base, pedestal) and the restore of the pedestal and neck after a hand-back.

Outcomes: `SUCCESS`, `TIMEOUT` (auto time or operator time ran out; `timeout_kind` says which), `STAGING_FAILED`, `ABORTED_BY_OPERATOR` (Delete),
`OPERATOR_NO_RESPONSE` (the policy asked for the operator and the operator timeout expired with no key pressed at all), `ABORTED` (viewer closed / Esc).
The base pose is corrected after PEDESTAL (full controller) and after ARMS (creep only, with clearance checks) when it is outside 1 cm / 1 degree
(`correction_*` in `STAGING_CFG`; `experiments/beaker_hotplate_correction_test.py` forces each case).

**Snap-back safety.** On resume every subsystem starts from where the operator left it and the policy then moves it toward what it wants: the base controllers
slew from the applied Twist, the arm and gripper targets are rate-limited from the applied targets (`src/rate_limit.py`; `arm_max_speed` 1.0 rad/s,
`gripper_max_speed` 0.05 m/s), the pedestal and the neck are put back, and the operator's time does not count toward a phase timeout.

**Gripper force limit.** A gripper command is a finger travel or `{"position": m, "max_force": N}` (per finger, like the ROS gripper command). The grippers stay
position-controlled; the loop writes `max_force` into the finger actuators' force range each tick, clipped to the hardware ceiling (10 N, a placeholder in
`ARM_ACTUATOR_SPECS["fingers"]["force_limit"]`). Omitted means the ceiling; `max_force <= 0` is rejected. The limit is recorded per tick and replayed.
`experiments/gripper_force_test.py` closes the gripper on the beaker at 1 N, 2 N and the ceiling and measures the contact force. The beaker is hollow glass
(70 mm, 100 mm tall, 2 mm walls: a ring of 24 thin boxes and a bottom disk, 100 g of glass plus a 100 g rigid "liquid" mass low inside), built into the model at load
time from `BEAKER_CFG` (`src/beaker.py`); its friction against the gripper is 0.4 (explicit contact pairs: the finger geoms' priority would otherwise win), and the
per-object gripper force cap is `GRIP_FORCE_TABLE` (glass beaker 5 N). `experiments/beaker_hollow_test.py`, `home_pose_test.py` and `grasp_lift_test.py` test them.

**The robot's face** (`src/face.py`, `FACE_CFG` in `src/robot_specs.py`; visual only). The neck carries a head-sized ellipsoid with a face image on its front, and the eagle camera
sits on its forehead, outside the surface. The two images (`neutral.png`, `smile.png`) are loaded from `assets/private/face/`, which is git-ignored; if one is missing the
head is plain grey and one line is printed, no error. `--no-face` (`combined_controller.py`, `beaker_hotplate_episode.py`, `replay_run.py`) leaves the head out. The episode
runner shows the neutral face and switches to the smile when the success check passes; after a success the episode runs 2 more seconds (`success_linger_s`), then stops.
`beaker_hotplate_episode.py --stop-after-handoff S` ends an episode S seconds after the handoff (outcome `STAGING_COMPLETE`; the default is 2 s with `--viewer` and the hold arm
stage). The head has no mass and no collision, is in geom group 5, which every policy camera and inset leaves out (so no training image contains it), and is drawn only by the
window viewer and the composite picture of the notebook videos. The replay fingerprint (`model_hash`) ignores it, so recordings verify the same with the face on, off or plain.
The camera tiles of the viewer are in the order eagle, left wrist, right wrist (the left hand's view on the left). `experiments/face_test.py` tests all of this.

## Record live, render offline

`combined_controller.py --record-run DIR` (or `beaker_hotplate_episode.py --record`) records what the physics received on every
tick (the ctrl vector of every physics step and the base velocity, exact float64) plus state hashes, about 150 KB per sim second. `experiments/replay_run.py DIR`
replays it headless: the states must match the live run bit for bit (same machine and MuJoCo version), and it renders the policy
cameras to one MP4 per camera (`eagle_cam.mp4`, ...) at 30 Hz, 640x480, each camera with its own shadow setting, about 25 MB
per sim minute for the three cameras. Options: `--video none|preview|training|viewer` (verify only / lossy MP4, the default / lossless PNG frames
`<camera>/<tick>.png`, about 230 MB per sim minute), `--cameras`, `--hz`, `--size`, `--shadows policy|on|off`, `--from-tick/--to-tick` for a clip, `--max-ticks`,
`--codec` and `--quality` (the OpenCV build has no H.264; MPEG-4 and VP9 work). The exit status is 1 if the replay does not match.
`--video viewer` renders the live viewer's picture offscreen (follow camera, status panel, the three camera tiles; `src/viewer_composite.py`) as one 1920x1080
MP4 at 30 frames per sim second with shadows on everywhere, plus a PNG of the same picture at the handoff (`--name STEM`, `--after-handoff S`): e.g.
`replay_run.py DIR --video viewer --after-handoff 3 --out data/notebook --name staging_run_<date>` (about 26 minutes of rendering for the 41 s staging clip, 34 MB). See `src/run_record.py` and `src/replay.py`.

## Features

- **MuJoCo physics**: 750 Hz physics and PD control, matching the OpenArm ros2_control loop.
- **PD control**: proper actuator mapping (20 controlled joints, `right_joint1`, `left_finger2`, ...).
- **Mobile base**: velocity-driven Tracer base (see the note in `combined_controller.py` for why). It stands on six low-friction contact spheres
  (`BASE_CONTACT_SPECS`) so the forced velocity does not tip it; a stopped base is held by the velocity servo, not by friction.
- **150 Hz control tick** (5 physics steps). Arm and gripper targets between two commands follow `cfg["target_mode"]`: `"interpolate"` (default; a linear ramp over the
  source's command period, 33 ms for a 30 Hz policy) or `"hold"`. A source or policy stage declares its command rate with `hz`.
- **Simulated cameras**: eagle and wrist cameras rendered offscreen at 30 Hz.
- **Webcam teleoperation**: MediaPipe pose to SEW-Mimic retargeting through geo_kin.
- **Generic simulator**: works with any scene XML.

## Status

- [x] Project infrastructure, generic simulator, PD controller with actuator mapping
- [x] Chemistry lab scene with the mobile manipulator
- [x] Keyboard control of base and both arms and grippers
- [x] Simulated cameras (placeholder poses)
- [x] Webcam teleop wired into the controller (tested with a scripted operator only)
- [x] Camera neck, beaker-on-hotplate task, scripted staging with base pose corrections, auto / teleop modes with operator handoff, gripper force limits
- [x] Record a live run and replay it exactly, rendering the cameras offline
- [ ] Hand tests with a real keyboard and the real gripper / neck / pedestal specs (see `docs/TODO.md`)
- [ ] Teleop tested with a real camera
- [ ] Learned policy integration
- [ ] Sim-to-real validation

See `docs/TODO.md` for the open items.

## References

- [MuJoCo Documentation](https://mujoco.org/)
- [Robot Danger Lab Wiki](https://github.gatech.edu/pages/Robot-Danger-Lab-GT/robot-danger-lab-wiki/)
- [OpenArm Repository](https://github.gatech.edu/rkothari40/OpenArm-Combined)
