# Braking policy for the safety filter: Part 1 (read-only checks)

Date: 2026-11-16. Brief: David's pasted task "Braking policy for the safety filter (branch safety-filter), Part 1". Scope: checks a) to e) only. Nothing was built, changed or pushed. No algorithm, stopping-distance formula or default value is proposed here.

## 0. What was read, and from where

- Branch **safety-filter** at commit 427f766 (21 commits ahead of main). The checkout stayed on main (b237951). The branch's `src/` tree was exported read-only with `git archive` into the session scratchpad, and that copy was read.
- The copy of `data/safety_filter/robot_shapes.json` is the working-tree file (git-ignored data, mtime 2026-11-16 02:47). It is the file the filter loads.
- `src/safety_filter/` (the package), `src/safety_filter_plugin.py` and `src/control_loop.py` are **byte-identical** on main and on safety-filter, so the findings hold for both.
- `src/task_config.py` differs by four video-export lines (SEARCH_DISPLAY_CFG: shadows, workers, tile_reuse_tol, tile_every), present on main and absent on safety-filter. The filter does not use them.

## 1. Summary

- **Gripper shapes:** 12 capsules in total (per arm: 2 finger, 2 palm, 2 link-7). The beaker is in no shape file. The held object is added at run time by `HeldTracker`, from a matched world object (the beaker is a capsule, radius 0.035 m), or from a fallback capsule (8 cm tall) when nothing matches.
- **MuJoCo contacts are not used.** The filter has its own robot-only model and reads no contacts. The held object is therefore attached by hand. The code already does this.
- **Hand speed** is not computed anywhere in the filter today. The inputs for it are all present each tick, but the base's velocity is not an input; only its pose is.
- **The distance test** is one comparison per candidate pair (not per link). Margins are set per pair kind in `config.MARGINS`.
- **Tick:** 150 Hz (6.67 ms), fixed in simulation time, set by config. The command bridge stores each tick's filtered target and applies the latest one at 50 Hz (`control_loop.py:599`, `:687`).
- **Found while checking (affects the design):** the filter already has a per-pair approach-speed rule (the gap rule); there is no jerk or acceleration limit anywhere in the filter or the bridge; the bridge rejects any target more than 0.25 rad from its ramped command, which can interact with braking outputs.

## 2. a) Collision geometry

### 2.1 What the filter uses for the gripper (robot_shapes.json, 12 capsules)

| Shape id | Part | Radius (m) | Body frame | MuJoCo geom |
|---|---|---|---|---|
| left_finger_left / left_finger_right | finger | 0.03179 each | openarm_left_left_finger / openarm_left_right_finger | *_finger_collision |
| left_palm_1 / left_palm_2 | palm | 0.01239 / 0.02853 | openarm_left_link7 | openarm_left_hand_collision |
| left_link7_1 / left_link7_2 | link7 | 0.02871 / 0.02628 | openarm_left_link7 | openarm_left_link7_collision |
| right_finger_left / right_finger_right | finger | 0.03179 each | openarm_right_left_finger / openarm_right_right_finger | *_finger_collision |
| right_palm_1 / right_palm_2 | palm | 0.00941 / 0.02854 | openarm_right_link7 | openarm_right_hand_collision |
| right_link7_1 / right_link7_2 | link7 | 0.02872 / 0.02628 | openarm_right_link7 | openarm_right_link7_collision |

Source of the list: `data/safety_filter/robot_shapes.json` (the `shapes` block; `part` and `geoms` fields). The file is built by `tools/build_filter_shapes.py`.

### 2.2 Is the held object in the filter?

Yes, but only at run time, and not from any file:

- `src/safety_filter/shapes.py:68`: `part: str  # upper_arm, forearm, shoulder_mount, link7, palm, finger, base, pedestal_base, pedestal_top, torso, eagle_cam, battery`. There is no `held` and no beaker in the file's shape list.
- `src/safety_filter/robot_model.py` (module docstring): "No simulator, no scene, no world data is read here." The filter's model is the robot description only.
- `src/safety_filter/touch.py:28`: `HAND_PARTS = ("palm", "finger", "held")`. The part name `held` exists only as a run-time shape.
- `src/safety_filter/touch.py:252-268` (`HeldTracker.update`): when the holding verdict turns on, it matches the nearest graspable world object to the finger-pad midpoint: `if d - o.bound_radius() <= cfg.HELD_MATCH_EXTRA_M and (best is None or d < best_d)`, then `self.world.remove(best.id)`.
- `src/safety_filter/touch.py:270-276`: no match gives a fallback capsule: `msg = f"{side}: no graspable object at the finger pads; fallback capsule used"` and `self.held[side] = {"kind": "fallback", ...}`.
- `src/safety_filter/touch.py:289-302` (`HeldTracker.placed`): the held shape is `shape_of(f"{side}_held", obj.shape, obj.size, ...)` with `part="held"` and `body=f"openarm_{side}_link7"`.
- `src/safety_filter_plugin.py:196-205` (`inputs_fn`): `tracker.update(s, bool(holding.get(s, False)), placed, pos, rot)`. The `holding` verdict comes from the control loop's `HoldDetector` (`src/control_loop.py:1084` `hold_det[s].update(...)`, `:1143` passes `hold_det[s_].holding`), not from contacts.
- Beaker and hotplate shapes: `src/safety_filter/config.py:108-121` (`OBJECT_CATALOG`). Beaker: capsule, radius 0.035 m, half height 0.05 m. Hotplate: box, half sizes (0.09, 0.09, 0.03), not graspable, not a fixture.

### 2.3 Does MuJoCo's contact system already carry the held object?

- In the simulator, yes: the beaker is a MuJoCo body with collision geoms in the full scene, and its grasp is physical contact in MuJoCo.
- The filter does not use that. A grep of `src/safety_filter/` and `src/safety_filter_plugin.py` for `data.contact`, `ncon`, `mj_collide` and `mj_contactForce` finds nothing. The filter's plugin reads only joint positions and base pose (`src/safety_filter_plugin.py:198`: `state = RobotState(base_xyz=tuple(data.qpos[0:3]), base_quat=tuple(data.qpos[3:7]), ...)`).
- So the filter **would have to attach the held object by hand**, and it already does (`HeldTracker`, above). Its shape comes from the world model's catalog, not from the physics geometry.
- Caveat: the attach happens only when the `HoldDetector` says "holding". If the verdict is on and no object matches, the fallback capsule is used (`touch.py:270-276`, default height `config.py:50`: `HELD_FALLBACK_HEIGHT_M = 0.08`).

## 3. b) Hand speed

### 3.1 Where the hand's velocity could be obtained each tick

Nothing computes hand velocity today. A search of `src/safety_filter/` for "velocity" or "speed" finds only the gap rule's approach speed (`gap_rule.py`, config `GAP_RULE` at `config.py:63-66`) and the joint-speed bound passed to the sleep timer (`config.py:128`). Each tick the filter has everything needed to compute it:

| Input each tick | Where | What it gives |
|---|---|---|
| `raw` (2 x 7 commanded joints) | `src/safety_filter_plugin.py:78`: `self.filter_step(raw, self.last_sent, state, world_objects, held)` | commanded arm motion |
| `last` (last sent command, 2 x 7) | same call | previous command |
| `state` (RobotState: measured joints, finger, base pose) | `src/safety_filter/shapes.py:47-60` | measured motion |
| hand pose from joints | `src/safety_filter/shapes.py:240-243` `RobotShapes.hand_frame(side)` (link-7 world pose, MuJoCo kinematics); `src/safety_filter/arm_conversion.py:67-70` `ArmKinematics.hand_pose_now(side)` | the hand's world position |
| previous tick's state | `src/safety_filter/tick.py:569` and `:596`: `self.state_vec = sv` (stored by the sleep timer; compared at `:561`) | a one-tick memory that already exists |

Two ways to get the hand's speed, both from these inputs: (i) finite difference of `hand_frame(side)` position between consecutive ticks (measured motion); (ii) forward kinematics of `raw` minus that of `last` (commanded motion). Choosing between them is David's (section 5, E1).

### 3.2 How the base's motion would be included

- The base pose is part of the joint state: `RobotState.base_xyz`, `base_quat` (`shapes.py:54-55`), written into the model's `base_float` joint (`shapes.py:145`, `:166-167`). So `hand_frame()` already includes the base's pose, and a finite difference of the hand's world position includes the base's motion automatically.
- The base's **velocity** is not an input. The base is driven by `apply_base_velocity(data, base_dof, vx, vy, wz)` at each physics step (`src/control_loop.py:987`), and the plugin's `process()` call (`control_loop.py:1143`) passes arm targets, holding, current targets, time and the manipulation flag only. The base's commanded twist never reaches the filter.
- Not checked in Part 1: whether the base is commanded while the arms are commanded. `stop_base()` is called in some modes (`control_loop.py:1134`, `:1140`).

## 4. c) Distance test

### 4.1 One comparison per pair, per tick (not per link)

- **Candidate pairs.** Each tick the pairs whose boxes are near enough are kept (`src/safety_filter/tick.py:235-244`, `discover`). Each pair is between two shapes; a link with two capsules gives two shapes and two pairs.
- **The test.** For each pair: `shortfall = max(0, required_gap_next - gap_now)` (`src/safety_filter/gap_rule.py:56-59`, `tick.py:447-448`). Action needs `s > trigger_m` and a closing arm (`tick.py:455-458`, `worst()`), and `s >= stop_m` in the sweep list (`src/safety_filter/kong_style.py:118-121`, `list_pairs()`).
- **The required gap** (`src/safety_filter/gap_rule.py:41-53`, mode "ours", `config.py:63-66`): above the margin, the gap may close at most at `approach_per_s x (gap - margin)`; below it, the gap must grow at `approach_per_s x (margin - gap)`, capped at `max_grow_speed_m_s`.
- **Scalar gap** of a pair: `src/safety_filter/tick.py:220-233` (`_Tick.gap`), from `touch.py:111-122` (`gap_between`).

### 4.2 Where the margin value lives

| Setting | Value | Location | Used by |
|---|---|---|---|
| `MARGINS["hand_hand_m"]` | 0.01 m | `src/safety_filter/config.py:33` | palm/finger/held of one arm vs the other arm |
| `MARGINS["robot_robot_m"]` | 0.03 m | `config.py:34` | other arm-vs-arm and robot-vs-body pairs |
| `MARGINS["arm_world_m"]` | 0.01 m | `config.py:35` | arm parts (not fingers, not held) vs world objects |
| `NEVER_TOUCH_MARGIN_M` | 0.05 m | `config.py:39` | every arm part, fingers and held included, vs never-touch objects |
| `PUSH_START_BEYOND_M` / `PUSH_STOP_BEYOND_M` | 0.01 / 0.02 m | `config.py:41-42` | activation and release distances |
| `GAP_RULE["approach_per_s"]` | 30 /s | `config.py:64` | the approach-speed rule above the margin |

The margin is picked per pair by its kind: `src/safety_filter/touch.py:45-47` (`_rule`), `:59-90` (`_pair_rule`). It is one number per pair kind, not per joint and not per link.

## 5. d) Tick rate

| Item | Value | Where |
|---|---|---|
| Control tick (`source_hz`) | 150.0 Hz, config value | `src/control_loop.py:118`: `"source_hz": 150.0,` |
| Physics timestep (scene) | 0.00133333 s (750 Hz) | `models/scenes/chemistry_lab_beaker_hotplate.xml:12`: `<option timestep="0.0013333333333333333" ...` |
| Steps per tick | 5 | `src/control_loop.py:325`: `steps_per_frame = _every(cfg["source_hz"], "source")` |
| Tick length | 5 x (1/750) = 6.67 ms | `src/control_loop.py:326`: `tick_dt = steps_per_frame * model.opt.timestep` |
| Filter's `dt` | 1/150 s | `src/safety_filter_plugin.py:170`: `Resolver(shapes, 1.0 / float(cfg["source_hz"]), ...)` |
| Command bridge update | 50 Hz (every 3rd tick) | `src/task_config.py:465`: `"rate_hz": 50.0,`; `src/control_loop.py:599`: `bridge_every = max(1, int(round(cfg["source_hz"] / bcfg["rate_hz"])))` |

Fixed? Yes, in simulation time. `_every()` raises `ValueError` if the physics rate is not an integer multiple of the source rate (`control_loop.py:317-320`). The tick is not adaptive. The wall-clock speed depends on the machine (CLAUDE.md: the viewer runs at about 0.9x real time on this VM); the filter's `dt` does not change with it.

## 6. Other findings that bear on the braking design

1. **Output steps are instantaneous.** A push is applied in the same tick (kong_style `_push_pair`). On HOLD the plugin sends the last sent command again: `src/safety_filter_plugin.py:81-82`, `if status.status == "HOLD": filtered = np.asarray(self.hold_policy(self.last_sent.copy(), status, list(self.history)), dtype=float)`. The default hold policy is `repeat_last_safe` (`safety_filter_plugin.py:30-32`), which returns the last sent command unchanged.
2. **A per-pair approach-speed rule already exists.** `src/safety_filter/config.py:64`: `"approach_per_s": 30.0,  # 1/s [M294]: allowed approach speed = this x the gap above the margin`. It is a proportional limit per pair (not per hand, not jerk-limited). It is inside the resolver. The live resolver is `kong_style` (`config.py:88`: `"mode": "kong_style"`).
3. **No jerk or acceleration limit** exists in the filter or in the bridge. `min_jerk` (`src/rate_limit.py`) is used by the held-mass ramp and the staging source, not on this command path.
4. **The bridge sits after the filter** (`src/control_loop.py:1143` filter, `:1147` `cmd = bridge_take(cmd)`). It limits joint speed to 0.5 rad/s, moving at most 0.01 rad per 50 Hz update (`src/task_config.py:466`). It rejects any target more than 0.25 rad from its ramped command (`task_config.py:467`; `control_loop.py` `bridge_update`: "rejected: target step too large", the previous accepted target stays). It also suspends forwarding when tracking error exceeds 0.25 rad (`task_config.py:468`). Any braking output has to fit inside these, or the bridge changes it.
5. **Held objects are exempt from the world.** `src/safety_filter/touch.py:69-71`: fingers and held objects are "never checked against the world" (M289), except never-touch objects (`touch.py:64-67`, margin 0.05 m). So a held beaker is not checked against the hotplate, the bench or any other world object today. Palm and link-7 shapes are checked against world objects with `arm_world_m` 0.01 m (`touch.py:72-74`).
6. **No "allowed-contact surface" concept exists.** The hotplate is an ordinary non-graspable box in `OBJECT_CATALOG` (`config.py:116-120`), not a fixture (the lab-map fixture list has no hotplate). Nothing flags a surface as allowed to touch.
7. **World objects come from ground truth in the simulator.** `make_sim_world_model` (`src/mpvi_modules.py:70-82`) builds them from the simulator state, and `src/safety_filter/world.py` calls the adapter "TEMPORARY (M290) ... Replace with perception later". On hardware the held object's shape and pose have no source yet.

## 7. e) Decisions needed from David before building

| # | Question | Why it matters | What the code does now |
|---|---|---|---|
| E1 | How is the hand's speed measured: finite difference of measured hand pose (`hand_frame`), commanded motion (FK of raw minus last), or both? | Decides whether `RobotState` (or a new input) must carry velocities. | No hand speed; `RobotState` has pose and joints only (`shapes.py:47-60`). |
| E2 | Where does the base's contribution come from: the measured base pose change between ticks, or the base's commanded twist (not passed to the filter today)? | Adding the twist is a new input to `process()`; the measured version needs no interface change. | Base pose only (`safety_filter_plugin.py:198`); base twist never reaches the filter (`control_loop.py:987`, `:1143`). |
| E3 | Which point of the hand is the "hand" for the limit: link-7 origin (`hand_frame`), wrist W, tool point T (Kong's point, `W + R7 @ tool_local`), finger-pad midpoint, or the held object's centre (spill-relevant)? | Each point moves differently when the base or the wrist turns. | No single hand point is defined for speed. |
| E4 | Should a held object be checked against world objects (hotplate, bench, others), and with what margin? Today held objects are exempt (`touch.py:71`). | Decision 6 (allowed-contact surfaces) and decision 2 (spills) both depend on it. | Only never-touch objects bind held objects. |
| E5 | Which surfaces are "allowed contact" (e.g. the hotplate), and how is that flagged (a new `OBJECT_CATALOG` field, a list in config, or per scene)? | Decision 6 needs a list; no flag exists. | Hotplate is an ordinary box (`config.py:116-120`). |
| E6 | Does the braking layer replace the existing gap-rule approach-speed limit (`config.py:64`), sit beside it (two speed limiters), or sit above it? | Decision 8 puts braking outside the resolver, where the gap rule lives. | Gap rule is per pair, inside the resolver, and active by default. |
| E7 | Is the braking maneuver the new HOLD policy (`HOLD_POLICIES`, `safety_filter_plugin.py:35`, replacing `repeat_last_safe`), or a separate layer the hold policy calls? What triggers the fail-safe if the filter itself raises or times out? | Decision 1 says it is always available as the fail-safe. | HOLD sends the last sent command again (`:81-82`). |
| E8 | Should braking output be allowed to meet the bridge's rules (0.25 rad step rejection, 0.5 rad/s cap) as they are, or may the braking layer change those settings for its own outputs? | The bridge rejects targets silently; a braking profile could be dropped. | Bridge applies to every arm command after the filter (`control_loop.py:1147`). |
| E9 | Is the jerk limit counted per filter tick (150 Hz) or per bridge update (50 Hz)? | The bridge applies the filter's latest target at 50 Hz, so the arm sees the profile in 50 Hz steps. | Filter outputs every tick; the bridge applies every third tick (`control_loop.py:599`, `:687`). |
| E10 | When the holding verdict is on but no object matches, is the 8 cm fallback capsule acceptable for braking, or should braking treat it differently? | Decision 7 assumes a matched object. | Fallback used with a warning (`touch.py:270-276`). |
| E11 | Where does the held object's shape and pose come from on hardware? | Decision 7 depends on it; today it is ground truth from the simulator (`mpvi_modules.py:70-82`). | Stand-in only (M290, `world.py`). |

## 8. Not done, and limits of this check

- No code was changed. No commit was made. Nothing was pushed. No simulation was run.
- Not measured: the size of command steps in recorded filter runs (so the claim "speed drops from full to zero in one tick" is confirmed for the code path, not for real runs); how often the bridge rejects a step; whether the base moves while the arms are commanded.
- This report is **not committed**. The brief says no commits in Part 1. CLAUDE.md rule 6 would commit a report. David to say whether to commit it.
- The file name follows the brief (`braking_policy_part1_20261116.md`), not the `<date>_<time>_<topic>.md` pattern of CLAUDE.md rule 6.
- `docs/progress.md` was updated (task entry at the top). It is a log, not code.
- Untracked `experiments/video_export_timing/` files were not touched.
