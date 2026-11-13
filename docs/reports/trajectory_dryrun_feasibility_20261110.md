# Trajectory dry-run feasibility: the planner's per-tick IK without physics (read-only, 2026-11-10)

Written for: David. Read-only: no code changed, no simulation run. The only timing was one `MinkArm.step` call loop on a built model (no `mj_step`). Sources: `src/mink_arm.py` (`__init__`, `joint_arc`, `reset`, `pad_pose`, `step`, `_collision_limits`), `src/mpvi_modules.py` (`move_duration`, `move`, `step_inner`, `plan_pick`, `plan_place`), `src/task_config.py` (`MINK_CFG`, `PICK_PLACE_CFG`).

## Step 1: the per-tick pipeline
Each control tick of a move (`step_inner`, src/mpvi_modules.py):
1. **Reference.** `p_ref = p0 + (target - p0) * min_jerk(u)`, `R_ref = slerp(R0, R)(min_jerk(u))`, with `u = elapsed / T`. `T` comes from `move_duration`: the longest of the distance at `speed_m_s`, the turn at `turn_rad_s`, `min_move_s`, and `min_jerk_peak_factor` times the largest joint arc of `joint_arc` (a dry run that already runs without physics) divided by `max_joint_speed`.
2. **Start point `p0`.** `arm.pad_pose(data)`: the **measured** pad (from `data.xpos/xmat` of the hand's TCP body). With `restart_from_commanded` = False (current setting), the start is the measured pad. `commanded_pad_pose()` (FK of `q_cmd` on a scratch copy) exists for the other setting.
3. **Solver call.** `arm.step(data, p_ref, R_ref)` (src/mink_arm.py, `MinkArm.step`):
   - `q = data.qpos.copy()`; the arm's seven joints are replaced by `q_cmd` (the solver's own last output). So the arm joints are NOT read from physics.
   - `config.update(q)`: the whole configuration, including the base's free joint, the pedestal, the other arm and the fingers, comes from `data.qpos`.
   - Frame task on the pad (position cost 1.0, orientation 0.5) toward `p_ref`, `R_ref`; posture task toward home (0.01); joint limits with a 10 deg margin; a velocity limit of 0.5 rad/s per joint; collision avoidance for the arm's links 3 to 7 and the hand against the bench and the hotplate (gain 0.85, detect 5 cm, minimum 8 mm), and against the other arm (minimum 40 mm).
   - `solve_ik(..., dt = 1/150, solver = daqp, damping 1e-3)` gives the joint velocity `v`; `q_cmd += v[vadr] * dt`. No physics step happens here.
4. **Advance.** The move ends when the **measured** pad is within 3 mm and 2 deg and settled (`settled`: the measured joint speed under 0.02 rad/s for 0.2 s), or the blend handover (`blend_radius_m` 20 mm, using the **measured** pad speed `v_pad`), or the timeout.
5. **Other sources.** The phase-start ramp (`phase_start_ramp_s` 0.5625 s, min-jerk cap on the command change) acts on `q_cmd` (own output). The sag trim (`sag_layer.py`) acts after the planner, on the controller command, and reads the measured state. The command bridge (`command_bridge`) sits between the sources and the arm targets; it logged no events in the failed run.

## Step 2: physics dependencies per subtask
| solver input or decision | source | in a no-physics dry run |
|---|---|---|
| arm joints in the solve | `q_cmd` (own output) | available: start from zombie or the check's solution |
| base pose, pedestal height, other arm, fingers in `data.qpos` | physics (base is a free joint; pedestal, fingers and other arm are commanded then measured) | must be set: stance pose, pedestal command, other arm at home, fingers as in the run |
| pad start `p0` (pick OVER, DOWN, UP; place OVER) | measured pad (`pad_pose(data)`) | use the commanded pad (FK of `q_cmd`): differs from the measured by the servo lag (2 to 5 deg per joint; up to about 50 mm at the OVER miss) |
| reference `p_ref`, `R_ref` | the move's own min-jerk and slerp | available (targets from the world model: beaker and hotplate poses are static in a dry run) |
| target of pick moves | beaker pose from world model (`object_pose`, physics) | available: beaker at its start pose, height from its properties |
| target of place moves | hotplate pose (static) and the beaker's held offset `off = p - b` (measured now) | assume the offset from the grasp geometry (the beaker is held at the grasp band) |
| carry move (UP to lift, BACK) | the held beaker's position in the hand | assume the grasp geometry |
| settle and advance tests | measured pad error and measured joint speed | replace with commanded: the same tolerances, applied to the commanded pad and `v` |
| blend handover | measured pad speed `v_pad` | replace with the commanded pad speed |
| CLOSE and OPEN steps | finger travel from physics (`data.qpos` of the finger), stall detection, the grip force | a stub: the close takes its minimum time (`close_min_s` 1.0 s) and the gripper reports closed |
| collision avoidance (mink) | geometry of the links from `q`, the static furniture, the other arm's links | available: the furniture is static; the other arm is at home |
| move duration | `joint_arc` dry run (no physics) | available (already a dry run) |
| held-mass ramp, gravity compensation | physics torques and the hold detection | not needed for the arm's joint solution (affects the controller only) |
| sag trim, command bridge, elbow guard | controller layer (not in the planner's solution) | excluded: the dry run checks the planner's own command; the elbow guard is off (M278) |

So the planner's joint solution depends on physics only through (a) the start pad (`p0`, measured), (b) the non-arm joints in `data.qpos` (base, pedestal, other arm, fingers), (c) the settle and blend tests, and (d) the close and open steps. Items (b) and (d) are fixed by the stance and the run's known settings, and (a) and (c) can use the commanded pad.

## Step 3: feasibility
**Yes, the planner can run closed-loop on its own output without physics.** `MinkArm.step` never steps the physics: it copies `data.qpos`, replaces the arm joints with `q_cmd`, and writes `q_cmd` back. A scratch `MjData` with the stance's qpos is enough.

What is needed (moderate work, not a flag alone):
1. **A mock data source.** A `MjData` set to the stance (base pose, pedestal command, other arm at home, fingers open) with `mj_forward` only, and a loop that sets `data.qpos[arm] = q_cmd` before each `step`, so the "measured" pad and joints are the commanded ones.
2. **A commanded-pad mode for the skill's measurements.** `p0`, the settle test, the blend speed, and the OVER and DOWN advance tests read the commanded pad instead of `pad_pose(data)`. This is a small branch in `step_inner` and `move`.
3. **Stubs for the gripper steps** (CLOSE and OPEN take their minimum time) and for the beaker's held offset (taken from the grasp geometry).
4. **A driver** that runs the skill's own step sequence (`plan_pick` then `plan_place`) against the mock source and records the joints, the pad and the errors per tick, with the same tolerances.

There is no physics feedback that is essential to the solver's joint solution. The gap is the measurement layer (the places where the skill reads `pad_pose(data)`), which the mock replaces with the commanded pad. The dry run would then show the planner's own path, including the swivel that happened in the failed run, **if** the swivel comes from the solver and not from the servos. That is the open question from the divergence report (the planner's own output was not logged there).

## Cost estimate
- **Per tick:** one `MinkArm.step` takes about **3.1 ms** (measured on the built model, 200 calls, 4 collision limits; no physics).
- **Per subtask path:** a pick (OVER 8.3 s, DOWN 3.7 s, IN, CLOSE at least 1 s, UP) and a place (OVER 2.7 s, DOWN 1.0 s, OPEN, BACK, HOME) run at 150 Hz. In the failed run the manipulation ticks (pick and place together) were about 5,000 of the 17,739 ticks of the episode, i.e. about 35 s of the 118 s.
- **Single dry run of all subtasks:** about 5,000 ticks x 3.1 ms = **about 15 to 20 s**, plus the `joint_arc` dry runs for the move durations (up to 100 iterations per waypoint, a few seconds in total; not timed).
- **Compare:** one full check of a stance takes about 40 s (the selection log). So a dry run as a final gate adds about half a full check per candidate. If it runs only on the candidates that pass the full check (the selection stops at the first pass), it costs about 20 s per selection; if it runs on every draw, up to 50 draws add about 17 minutes in the worst case.

## Verdict
**Feasible with moderate work**, not with a simple flag. The solver itself needs no physics; the skill's measurement layer does (about four read sites: `pad_pose(data)` in `move`, `step_inner` and `recover`, the settle test, the blend speed, and the close and open tests). A commanded-pad mode plus a mock data source covers them. The main caveat is that the dry run would reproduce the solver's behaviour only, not the servo lag and the controller's gravity offsets, which were 2 to 5 deg per joint in the failed run.

## Not done
1. The `joint_arc` dry run's own timing (not measured).
2. The planner's own per-tick output in a real run (not logged; see the divergence report).
3. Whether the swivel at 72.5 s comes from the solver (the dry run would show it).
