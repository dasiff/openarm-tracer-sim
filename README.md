# Robotics Simulation for Materials Lab

Simulation environment for Agilex Tracer mobile base + OpenArm bimanual robot in materials lab tasks.

## Quick Start

The project has its own virtual environment (uv, Python 3.12, MuJoCo 3.13.0), set up to share the
locked versions of `openarm_teleop`. See `docs/session_log.md` for how it was built.

```bash
source .venv/bin/activate        # also sets the geo_kin license variable
python experiments/combined_controller.py                 # drive the robot with the keyboard
python experiments/combined_controller.py --mode teleop   # same, plus webcam teleoperation of the arms
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
| `F7` | frame the task points with the neck camera (beaker, hotplate and gripper, when the scene has the beaker task; says so if they cannot all fit) |
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

**Recording (`F6`, or `--record` to start at launch; off by default).** The video is the viewer image (status panel and camera insets included). It plays
at the simulated rate (62.5 frames per second of sim time), however slowly the VM ran. The CSV has one
row per video frame, written as it goes: `video_frame`, `sim_time_s`, `wall_s`, the base pose, then
every arm joint and finger as `<joint>_pos` (actual) and `<joint>_target` (PD target). Run with
`CC_ROUNDROBIN=0` so the camera insets update at the full rate in the video.

The viewer shows the three simulated cameras (eagle and both wrists) along the bottom, plus the
operator webcam in teleop mode. Camera poses, field of view and resolution are placeholders until
the real mounting and Orbbec model are known (`src/robot_specs.py`).

**Camera neck.** The eagle camera is on the end of a 4-DOF SO-101 arm (the "neck") mounted on top of the torso
(`NECK_SPECS` in `src/robot_specs.py`: the mount and the choice of joints are assumptions pending the real
specs). The model is vendored in `models/so101/` (Apache-2.0, see `PROVENANCE.md`) and attached at load time by
`src/neck.py`. The neck starts looking ahead and 30 degrees down. It moves while the base or the arms move (it is
exempt from the one-subsystem-at-a-time rule) at no more than 1.5 rad/s. In teleop mode it follows the grippers
while the base is at rest and looks ahead while the base drives; any neck key takes over for 3 s.
`src/neck_framing.py: frame_points()` finds neck angles that keep a set of points in the image with a 10 % margin
(and reports the points that do not fit).

### Performance on the VM

The development VM has only a virtual GPU, so the viewer defaults to settings that keep it
responsive: viewer shadows off, one simulated camera rendered per tick (each updates at 10 Hz,
not the full 30 Hz), the eagle camera without shadows, and vsync off paced to 60 Hz. For recording
demos run with `CC_ROUNDROBIN=0` so every camera updates at 30 Hz (slower on the VM). All the
switches (`CC_*` environment variables, including `CC_BENCH_FRAMES=N` for per-phase timings) are
documented in `_perf_settings` in `experiments/combined_controller.py`.

## Project Structure

```
openarm-tracer-sim/
├── src/
│   ├── simulator.py         # Generic MuJoCo simulator (injects the cameras at load)
│   ├── controller.py        # PD controller (20 actuators)
│   ├── cameras.py           # Simulated Orbbec cameras, offscreen rendering
│   ├── control_loop.py      # run_loop(): the control loop (router, command validation, optional exclusivity / hold); headless-capable
│   ├── commands.py          # Mode-tagged command format (base / pedestal / arms / idle) and validate_command()
│   ├── base_control.py      # Twist -> free-joint velocity base drive
│   ├── keyboard_source.py, teleop_source.py, policy_source.py   # command sources
│   ├── glfw_viewer.py       # Window, HUD, camera insets, recorder, snapshots
│   ├── neck.py, neck_framing.py, neck_source.py   # camera neck: attach the SO-101 at load, frame points, auto follow / look-ahead
│   ├── robot_specs.py       # Rates, motor gains, camera specs, calibration status
│   ├── actuator_mapping.py  # Actuator name -> index mapping
│   ├── joint_addressing.py
│   ├── environment.py
│   ├── logger.py            # Trajectory logging
│   └── utils.py
├── models/
│   ├── scenes/              # chemistry_lab*.xml, single_block.xml, ...
│   └── policies/            # Policy implementations
├── deps/OpenArm-Combined/   # Robot model (submodule)
├── models/so101/            # Vendored SO-101 model for the camera neck (upstream files, unmodified; see PROVENANCE.md)
├── experiments/
│   ├── combined_controller.py     # Launcher for the interactive sim: keyboard, webcam teleop, policy, cameras (loop is in src/)
│   ├── cc_bench_scenarios.sh, cc_bench_compare.py, cc_loop_tests.py   # before/after regression scenarios, headless and hold-rule tests
│   ├── camera_snapshot.py         # Render the simulated cameras once to PNGs
│   ├── chemistry_lab_tracer_teleop.py
│   ├── run_basic_sim.py, run_with_dummy_policy.py, test_controller.py
│   └── workspace_sweep.py, visualize_workspace.py, empty_room_joint_test.py, ...
├── data/                    # Logged trajectories, camera snapshots, diagnostics
├── docs/
│   ├── TODO.md              # Roadmap and open items
│   └── session_log.md       # Decisions, findings and problems, by session
└── README.md
```

## Features

- **MuJoCo physics**: 750 Hz physics and PD control, matching the OpenArm ros2_control loop.
- **PD control**: proper actuator mapping (20 controlled joints, `right_joint1`, `left_finger2`, ...).
- **Mobile base**: velocity-driven Tracer base (see the note in `combined_controller.py` for why).
- **Simulated cameras**: eagle and wrist cameras rendered offscreen at 30 Hz.
- **Webcam teleoperation**: MediaPipe pose to SEW-Mimic retargeting through geo_kin.
- **Generic simulator**: works with any scene XML.

## Status

- [x] Project infrastructure, generic simulator, PD controller with actuator mapping
- [x] Chemistry lab scene with the mobile manipulator
- [x] Keyboard control of base and both arms and grippers
- [x] Simulated cameras (placeholder poses)
- [x] Webcam teleop wired into the controller (tested with a scripted operator only)
- [ ] Teleop tested with a real camera
- [ ] Learned policy integration
- [ ] Sim-to-real validation

See `docs/TODO.md` for the open items.

## References

- [MuJoCo Documentation](https://mujoco.org/)
- [Robot Danger Lab Wiki](https://github.gatech.edu/pages/Robot-Danger-Lab-GT/robot-danger-lab-wiki/)
- [OpenArm Repository](https://github.gatech.edu/rkothari40/OpenArm-Combined)
