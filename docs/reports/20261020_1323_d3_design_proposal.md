# D3 design proposal: scripted pick, place, release and return home (left arm, mink) plus the viewer status panel

Written for: David (design review before any code). Status: PROPOSAL, nothing built, nothing run. Numbers marked "estimate" are my own and are replaced by measurements in step 1.

## 1. Your five decisions

| # | Decision | My view |
|---|---|---|
| 1 | Path per §3.3: rise, move above, descend, move in sideways; reverse for the place; home | **Agree, with two clarifications (A, B below).** |
| 2 | Enforce the real command limits (0.5 rad/s, 0.25 rad step, 0.25 rad tracking fault) as settings | **Agree, but this is not built yet and enforcing it everywhere changes the baselines (C below).** |
| 3 | Align the gripper on the beaker's axis before closing | **Agree.** The cause of the 4.7 mm is not diagnosed (D below). |
| 4 | Success: upright on the hotplate, gripper open, arm home, no contacts | **Agree, "no contacts" needs a definition (E below).** |
| 5 | mink: pad-centre target, level rule, posture to home, joint limits, collision avoidance | **Agree.** Proposed tasks, weights and limits in §5. mink is not installed yet (F below). |

### Things that conflict with, or are missing from, the code or the plan

- **A. "Move above" means above the standoff point, not above the beaker.** The beaker is 70 mm across and the open pads are 84 mm apart; a vertical descent over it would put the palm on the rim. The approach is: rise to the clearance height, move over a standoff point 12 cm back from the beaker along the approach axis (setting), descend there to the grasp height, then move horizontally in until the pad centre is on the beaker's axis. The place mirrors this over the hotplate.
- **B. The clearance and standoff points have never been checked at this stance.** §3.3 says every sample must be reachable; D2 verified only the 12 grasp-band and set-down points (the skill-path step B2.11 was not built). The clearance height is 20 cm above the surface the target sits on (§3.3): pad centre at 1.16 m for the pick, and for the carry over the hotplate (top at 1.028 m) 1.23 m. If these are out of reach at the finder's stance, D3 will find out in its pre-flight check (step 2), before moving. Fallback: a lower clearance, a setting (the beaker is 10 cm tall, so about 12 cm clears it). That is a change of the §3.3 default, so it would come back to you.
- **C. The command bridge limits are in §1 as "DECIDED, built after B1c", but nothing in the code enforces them.** Today the loop has only the snap-back rate limit (`arm_max_speed` 1.0 rad/s), the teleop limit (3.0 rad/s) and the staging's own speeds. Enforcing 0.5 rad/s in the loop for every source would slow the staging ARMS phase, the teleop and the policy baselines, and change the baselines that are currently byte-identical to `experiments/baselines`. Proposal: a settings block (`COMMAND_BRIDGE_CFG`) and a loop switch, **off by default** (existing launchers, tests and baselines unchanged), **on for D3** and for everything the demo generator (B4) records. Switching it on for the staging and teleop is a separate decision for you, with new baselines.
  - Sub-decision for you: apply the limit at every 150 Hz tick (0.5 / 150 = 0.0033 rad per tick) or at the real 50 Hz (0.01 rad every 3rd tick, §1 left this open). I propose every tick: same average speed, smoother, no stair steps for the 750 Hz servo, one place to look. The recorded action would be the post-bridge applied command.
  - Semantics I would build: (i) a target outside the joint limits, or more than 0.25 rad from the current applied command, is rejected as a whole and logged (the sequencer never produces one at 0.4 rad/s, so a rejection is a bug and fails the run); (ii) the applied command moves toward the target by at most 0.5 rad/s; (iii) if any joint's applied command is more than 0.25 rad from its measured angle, commands are suspended (the applied command holds) until it recovers, and the state is in the observation. Note on (iii): at kp 70 N·m/rad a 0.25 rad error is 17 N·m on joint 1, so this fault is also a crash detector, not just a safety net. D3 treats it as a failure with the reason "tracking fault".
- **D. The 4.7 mm off-axis stop is explained in a comment as "the arm's servos and the solver's tolerance", not measured.** The old grasp test targeted the TCP body with a general IK; the pad centre is a different point, and gravity compensation makes servo sag under 0.05 deg when unloaded. So I expect most of it was target definition and IK tolerance, and mink with the pad-centre site as the target removes that. I will measure it first (step 1) and the ALIGN phase is the fix either way.
- **E. "No contacts along the way" cannot be literal:** the pads touch the beaker by design. Proposed definition, checked every tick by a contact audit: no contact between any robot geom and the bench, the hotplate, the floor or the room; none between the right arm and anything; the left arm touches the beaker only with the two finger pads and the palm and only in the phases GRASP to RELEASE; the beaker touches the bench until LIFT_OUT, the hotplate only from SETDOWN; and the beaker is not moved by more than 2 mm before the close command (so it cannot be pushed during the approach). Any other contact fails the episode with the pair of names and the phase.
- **F. mink is not installed in the venv.** `uv pip install --dry-run mink` resolves to mink 1.3.0, qpsolvers 4.13.0 and daqp 0.10.3 (three new packages; mujoco 3.13.0 untouched). I will install them only after your go and add them to the project's requirements.

## 2. The grasp and place geometry (from the code)

- Stance: base (0.765, 1.085), heading -90 deg, pedestal 0.155 m. Beaker (1.00, 0.66), hotplate (0.80, 0.66); the beaker is 23.5 cm left of the robot's centre line and 42.5 cm ahead, the hotplate 3.5 cm left of the centre line and 42.5 cm ahead. Bench top 0.96 m, hotplate top plate surface 1.028 m.
- Beaker: 70 mm outside diameter, 100 mm tall, glass 100 g + liquid setting 100 g, grip 8 N per finger (`GRIP_FORCE_TABLE`), grasp band 45 to 60 % of its height. D3 grasps at the middle of the band (52.5 %, a setting); the generator will vary it later.
- Approach axis horizontal, finger-closing axis horizontal (the level rule with 0 deg tolerance; the map rule allows 5 deg). Default approach direction: bin 0 (straight ahead, the best-verified bin at this stance); the pre-flight tries the neighbouring 30 deg bins if bin 0 fails and takes the one with the largest joint margin.
- Pad centre, approach and closing axes come from `src/gripper_geometry.py` (derived from the model). A new site at the pad centre, created at load by a spec hook, is what mink targets.

## 3. Phases

Each phase has a name (shown in the panel), a goal, a done-rule and a timeout (2.5 times its nominal time, counted from the pre-flight's planned duration). "Settled" means the loop's existing thresholds: arm joint speeds under `settle_arm_speed` 0.02 rad/s and finger speed under 0.002 m/s for 0.2 s. Position and orientation errors are measured from the forward kinematics of the MEASURED joint angles (what the real robot also has), never from the commanded ones. Nominal times are estimates at the planning speed of 0.4 rad/s (80 % of the bridge limit, so the bridge never clips the plan).

| # | Phase | Motion | Done when | Est. |
|---|---|---|---|---|
| 0 | PREFLIGHT | On a scratch copy: solve every waypoint with the limits and collision avoidance of §5, run the whole path, check clearances. Nothing moves. | Every waypoint solves, joint margin 10 deg or more, clearances at least the limits; else FAILED before any motion, with the waypoint and the reason ("unreachable", "collides with ...") | 1 s wall |
| 1 | RISE | Home pad pose up to the clearance height | position within 5 mm, settled | 3 s |
| 2 | TRANSIT_PICK | At that height over the standoff point, hand turned level, pads facing the beaker | within 5 mm, 1 deg, settled | 3 s |
| 3 | DESCEND | Vertical to the grasp height | within 2 mm in z, 3 mm in xy, settled | 2 s |
| 4 | APPROACH | Horizontal in along the approach axis at 3 cm/s, until the pad centre is 10 mm short of the beaker axis | within 2 mm of the plan, settled | 4 s |
| 5 | ALIGN | Closed-loop fine alignment of the pad centre onto the beaker axis and the grasp height (ground-truth beaker pose now; §12) | lateral error 1 mm or less, depth 2 mm or less, height 2 mm or less, held for 0.3 s | 2 s |
| 6 | GRASP | Close with `{"position": 0, "max_force": 8 N}` | fingers stalled AND the stall position matches the beaker's width within 1.5 mm (a stall at fully closed = "nothing in the hand"; no stall at all = "did not close"; both fail) | 2 s |
| 7 | LIFT_OUT | Straight up 3 cm | beaker follows the hand: slip under 3 mm, hand within 2 mm of the plan, settled (the "held" check of `grasp_lift_test`) | 2 s |
| 8 | RETREAT | Horizontal back out to the standoff point, beaker held | within 3 mm, settled; the beaker is still held | 3 s |
| 9 | RISE_CARRY | Up to the carry height over the hotplate | within 5 mm, settled | 2 s |
| 10 | TRANSIT_PLACE | Over the hotplate's standoff point, level | within 5 mm, 1 deg, settled | 3 s |
| 11 | LOWER | Vertical to 10 mm above the set-down height (beaker bottom 10 mm over the plate) | within 2 mm in z, settled | 2 s |
| 12 | APPROACH_PLACE | Horizontal in so the beaker's axis is over the hotplate centre (target is the beaker, not the pad: the measured offset of the beaker in the hand is applied) | beaker axis within 3 mm of the plate centre, settled | 4 s |
| 13 | SETDOWN | Vertical down the last 10 mm at 1 cm/s to the set-down height | beaker bottom within 1 mm of the plate top (pad-centre target reached) and settled; fails if the tracking error grows (pressing into the plate) | 2 s |
| 14 | RELEASE | Open the gripper | fingers at the open position within 1 mm and stalled; the beaker has not moved more than 2 mm | 2 s |
| 15 | RETREAT_PLACE | Horizontal back out to the standoff point, fingers open | within 3 mm; beaker still on the plate (moved under 2 mm) | 3 s |
| 16 | RISE_PLACE | Up to the clearance height | within 5 mm, settled | 2 s |
| 17 | RETURN_HOME | Joint space to the home pose, the path checked first on a scratch copy (the arms-to-home rule of §1) | every joint within 0.02 rad of `home_q_left`, settled, gripper open | 4 s |
| 18 | VERIFY | Success checks of §4 | all hold | 1 s |

Estimated total about 45 s of simulated time. The "reverse for the place" of your decision 1 is the mirror image of 1 to 5 over the hotplate, with one refinement: the horizontal move happens 10 mm above the plate and the last 10 mm are a slow vertical set-down, so the beaker is never dragged across the plate.

Failure handling: a phase that fails ends the run with outcome, the phase, and a reason string (the failure log of §8). The arm is then left where it is (the operator is requested, as `request_operator` does today); D3 does not try to recover (recovery nudges are the demo generator's, §5).

## 4. Success (VERIFY) and the contact audit

All of: the existing `check_success` (beaker base inside the hotplate footprint, height within tolerance, upright, at rest); the gripper open (finger travel within 1 mm of open); every left-arm joint within 0.02 rad of home; the right arm still at its home (within 0.02 rad); no tracking fault or rejected command at any time; no contact-audit violation (§1 item E); the beaker's final xy within 1 cm of the hotplate centre (tighter than the footprint check; a setting). The oracle verdicts and the measured errors of every phase go into the episode report (they will be the D4 oracle log).

## 5. mink (design)

- **Model for mink.** One extra `MjModel` for mink, compiled from the same task spec (the mink `Configuration` needs its own model object), with the pedestal and everything but the left arm's 7 joints frozen. A one-hour spike in step 1 compares freezing the other DoFs with mink's DoF-freezing task against a reduced model with only the left arm and static obstacles; I will take whichever is simpler and keeps the pad-centre FK within 0.1 mm of the full model's. The beaker and hotplate poses are copied into it at each phase start (the beaker is a free body).
- **Run loop (150 Hz).** The mink configuration is the COMMANDED joints; tasks are solved with `solve_ik`, the result integrated into the command, passed through the bridge (§1 C). The task error that matters is measured on the real arm (next section).
- **Tasks and weights (initial values, tuned in step 3; all are settings in `D3_CFG`):**

| Task / limit | Setting |
|---|---|
| FrameTask on the pad-centre site | position cost 1.0; orientation cost 0.3 in transit, 1.0 from DESCEND on (the level rule: approach and closing axes horizontal, the hand's yaw from the approach direction) |
| PostureTask toward `home_q_left` | cost 0.01 (it only picks the elbow, the 7th DoF); the Phase B per-episode elbow variation of §5 later changes its target |
| ConfigurationLimit | at least 10 deg from every limit (the map rule) |
| VelocityLimit | 0.4 rad/s on all 7 joints (the bridge's 0.5 with 20 % margin) |
| CollisionAvoidanceLimit, hand and fingers against the bench and hotplate | at least 8 mm (open-finger clearance at the set-down is 19.5 mm); detection 5 cm |
| Same, links 3 to 7 against the beaker | at least 15 mm |
| Same, fingers and palm against the beaker | at least 3 mm in phases 2 to 5, EXCLUDED from phase 6 on (the audit watches them instead) |
| Same, left arm against the right arm, torso, pedestal, neck, battery, base | at least 40 mm against the right arm, 20 mm against the rest |
| `solve_ik` damping | 1e-3 |
| Solver | daqp |

The held beaker is treated as part of the hand from phase 7: its collision against the bench is excluded until it is lifted, then against the hotplate and the bench as an obstacle until SETDOWN.

## 6. Closed-loop correction for arm sag

With gravity compensation on, the unloaded arm holds within 0.05 deg (`arm_sag_test`). It does not hold the beaker's weight (0.2 kg, about 2 N, not compensated) and the real robot may have no gravity compensation at all (95 mm of hand drop extended at the real gains, 48 mm at home; §1, B1c). Both are the same problem: the measured tip is not where the command put it.

A plain "solve from the measured joints plus a step" does not fix this: the step it computes is exactly the sag, so the command comes back unchanged. The fix is an integrator on the command: each tick the pad-centre error is measured from the forward kinematics of the MEASURED joints, `err = target - FK(q_measured)`, and a correction `delta` is added to the target the FrameTask sees: `delta += k * err * dt` (k about 2 per second, so a step is corrected in about 1 s), clamped to 2 cm (a larger need is a failure "sag correction exceeds limit"). The command moves until the measured tip meets the target. The done-rules of §3 all read the measured error, so a phase does not end while the sag is uncorrected. In ALIGN and SETDOWN the target is the beaker itself (axis, bottom against the plate), so a slip in the hand is corrected in the same loop. The correction is reset at each phase start and logged per phase (it is also a useful sag measurement).

Test: the same episode with gravity compensation switched off (a model hook), the clamp raised to 12 cm, must still complete within the same tolerances.

## 7. What is new and what is reused

New:
- `src/command_bridge.py` and `COMMAND_BRIDGE_CFG`: the limits of §1 C, and the loop switch (a small edit in `src/control_loop.py` where arm targets are applied).
- `src/mink_arm.py`: builds the mink model and configuration, the pad-centre site hook, tasks, limits and collision pairs per phase; `solve(target, phase) -> command`.
- `src/pick_place_task.py`: the phase sequencer as an arm stage (the stage dict of `make_sequential_policy`: `step`, `done`, `error`, `phase`, `report`), the pre-flight, the world-pose accessor (ground truth now, the single swap point for §12), the contact audit.
- `src/plan_panel.py`: the status panel's data and bitmap (below).
- `D3_CFG` in `src/task_config.py`.
- Tests (§9).

Reused unchanged: `src/sequential_policy.py` and `make_task(arm_stage=...)` / `run_beaker_episode`; `src/gripper_geometry.py` (pad centre, axes); `GRIP_FORCE_TABLE` and the gripper command with `max_force`; `check_success`; `STANCE_REQUIREMENTS`'s clearance numbers; `rate_limit` and `min_jerk`; the clearance monitor of `experiments/episode_test_sources.py` (moved to `src/` if D3 needs it); the staging (unchanged: ends at home with the neck framed); `src/run_record.py` (recording and exact replay); the settle thresholds of `LOOP_CFG`.

Left alone: `grasp_ik_targets.py` and `make_grasp_lift_stage` (the old IK and the test stage still serve `grasp_lift_test` and `gripper_force_test`; D3 does not replace them; I would delete them later only with your agreement).

## 8. Viewer status panel

- **Data.** The arm stage's status gets an optional `plan` entry: the list of step names (here the phases of §3 stand in for plan steps, as the plan says), the current index, and for each step its state (pending, running, checking, done, failed with a reason), plus the current step's phase text (for example "APPROACH 62 %"). `make_sequential_policy.status()` passes it through; D4's orchestrator will fill the same structure with real plan segments. Setting `next_steps` (default 3).
- **Drawing.** `src/plan_panel.py` builds an OpenCV bitmap (the previous step with a check mark, the current step highlighted with its phase and status, the next N steps, "+k more"), in the same style as the existing status panel in `src/viewer_layout.py`, and replaces the bare "mode" row there. It must use bitmaps, not MuJoCo's text overlay (the vmwgfx leak aborts the viewer after about 250 frames on this VM). The same bitmap is used by `src/glfw_viewer.py`, and by `src/viewer_composite.py` so that the replay's "viewer" video and notebook renders show it.
- **Never in the policy images.** The policy cameras do not use `viewer_layout`; a test renders them with the panel on and off and requires bit-identical images.
- **Console and log.** At the start the full plan is printed and written to the episode log; every state change is logged with time and phase; failures with their reason.
- The panel is built and tested before the phases (step 4), so the phases can be watched from the first full run.

## 9. Tests

1. `mink_arm_test`: the pad-centre site against `gripper_geometry` (0.1 mm); reaching known FK poses from home (within 0.5 mm and 0.5 deg); margins and velocity limits never exceeded; a target inside the bench or the hotplate stops short by the configured minimum; hand against the right arm.
2. `command_bridge_test`: step target moves exactly 0.5 rad/s; a target more than 0.25 rad away is rejected whole and logged; out-of-limits rejected; a held-back joint trips the tracking fault, commands suspend and resume on recovery; **off: the three baselines stay byte-identical**.
3. `pick_place_preflight_test`: all waypoints solve at the staging stance (this is also the check of item B); a beaker out of reach, or a hotplate moved onto the path, fails before any motion with the right reason; the clearance fallback.
4. `pick_place_phase_tests`, one forced failure per phase kind: blocked target ("blocked"), grasp on nothing ("empty grasp"), grasp that never closes, beaker slips out of the hand ("dropped"), tracking fault, timeout, contact-audit violation (target lowered into the hotplate), sag correction over its clamp.
5. `pick_place_episode_test`: the nominal episode (outcome success, every §4 criterion, every phase's error and sag correction reported); the same with stance offsets of 1 cm and 1 deg (the stopping error), beaker offsets of 1 cm, beaker friction 0.32, noslip as configured; recorded and replayed bit-identically; gravity compensation off (§6).
6. `plan_panel_test`: layout cases (first step, last step, N next, "+k more", failed with a reason, long names), the bitmap is OpenCV-only, the policy images identical with the panel on and off, a composite frame.
7. The whole suite (`bash experiments/run_tests.sh`) green and the baselines byte-identical with the bridge off.

Test runs follow the standing rules: one MuJoCo process at a time, long runs in a detached tmux session with a log under `data/logs/`, commit before each.

## 10. Order of work and time estimate

Calendar working time with the VM's crashes and one MuJoCo job at a time; the long runs (suite, repeated episodes) go in tmux while I write the next step. Estimates:

| Step | Content | Est. |
|---|---|---|
| 1 | Install mink (after your go); spike: mink on the full model vs a reduced one, pad-centre site, measure the 4.7 mm and the loaded sag; `mink_arm_test` | 1 day |
| 2 | Command bridge and its tests; waypoint generator and PREFLIGHT (answers item B) | 1 day |
| 3 | Phases 1 to 18, the sag integrator, contact audit; phase tests; tune the weights | 1.5 days |
| 4 | Status panel: data, drawing, viewer and composite, console log, tests (can run before step 3's tuning) | 1 day |
| 5 | Full-episode tests and variations, whole suite, baselines, design plan and progress log updates (§1, §3.3, §5, §6, §8, §9 markers), report | 1 day |

Total about 5 working days; the risks that could add time are item B (an unreachable clearance height would send the clearance setting back to you) and mink's collision avoidance with the finger meshes at 3 mm (it may need convex primitives for the pads, half a day).

## 11. What I need from you

1. Go, or changes, on this proposal as a whole.
2. Bridge limits: off by default and on for D3 (my proposal), or on everywhere now (new baselines)?
3. Bridge timing: every 150 Hz tick (my proposal) or the real 50 Hz?
4. OK to install mink 1.3.0 with daqp and qpsolvers (venv and requirements)?
5. OK to use the simulator's beaker and hotplate poses in D3 through one accessor (the swap point of §12)? The hand itself is always read through the measured joint angles.
6. The contact-audit definition of §1 E, and the "no more than 2 mm" disturbance bound before the close.
7. If item B fails: lower the clearance height (to about 12 cm), or move the stance?
