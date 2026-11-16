"""Every setting of the safety filter (decision rows: SF-1 to SF-7 and M285 to M298 in docs/mpvi_decisions.md).

Kept here, not in src/task_config.py (task brief). Each provisional number is tagged PROVISIONAL with its row.
Plain-word comments: what the setting does, its unit, why this value.
"""

# Build tool (tools/build_filter_shapes.py): how the upper arm and forearm radii are sampled.
SHAPE_BUILD = {
    "random_poses_first_set": 1000,   # count: random joint poses in the first sample set (plus the joint-limit corners); the radius at 1000 samples [SF-5]
    "random_poses_second_set": 1000,  # count: random poses added in the second set, so the total is 2000 (plus corners) [SF-5]
    "seed": 0,                        # the random generator seed for the pose samples (fixed so the build is repeatable) [SF-5]
    "fit_steps": 20000,               # count: the largest number of Nelder-Mead steps in each capsule fit [SF-5]
    "link_capsules": {                # count per link, per arm (SF-18): the fewest capsules that meet SF-16 (1 to 3); set by experiments/safety_filter_link_counts.py
        "left": {1: 1, 2: 1, 3: 1, 4: 1, 5: 1, 6: 1},
        "right": {1: 1, 2: 1, 3: 1, 4: 1, 5: 1, 6: 1},
    },
    "link7_capsules": 2,              # count: capsules for link 7 in the link 7 frame (SF-15)
}

# Lab map (tools/survey_lab_map.py): which scene file to survey (the task scene, src/task_config.py TASK_CFG) [M290].
LAB_MAP_SCENE = "models/scenes/chemistry_lab_beaker_hotplate.xml"

# ---- Filter settings (Stage 1: shared layer). Units in the comments. ----

# Where the filter's files are (relative to the project root). [SF-5]
SHAPES_FILE = "data/safety_filter/robot_shapes.json"
LAB_MAP_FILE = "data/safety_filter/lab_map.json"
# The bench_open setting the lab map must be built for (mirrors TASK_CFG["bench_open"], M078); the filter refuses a map built for the other setting (SF-8).
LAB_MAP_BENCH_OPEN = True

# Touch margins per pair kind (M288). A pair is checked when its gap comes within the margin.
MARGINS = {
    "hand_hand_m": 0.01,        # m, PROVISIONAL [M288]: palm, fingers or held object of one arm vs the same of the other arm
    "robot_robot_m": 0.03,      # m, PROVISIONAL [M288]: every other robot vs robot pair (arm parts of one arm vs the other arm, held vs robot body)
    "arm_world_m": 0.01,        # m, PROVISIONAL [M288]: arm parts (not fingers) vs world objects; must stay below the palm-to-beaker gap at a normal grasp
    "held_world_margin_m": 0.01,  # m, PLACEHOLDER — tuned by test [Part 2 braking brief]: a held object vs every world object that is not an allowed-contact surface; the hand stops at the midpoint of this margin
}
# A held object against an allowed-contact surface (OBJECT_CATALOG "allowed_contact", e.g. the hotplate) has zero margin and the surface's approach ceiling (BRAKING["approach_ceiling_m_s"]).
HELD_SURFACE_MARGIN_M = 0.0     # m [Part 2 braking brief]: it may touch such a surface, slowly
# A never-touch world object (WorldObject.never_touch, SF-65, David 2026-11-13): checked against every arm part INCLUDING fingers and held objects, with this margin
# instead of arm_world_m. Kept out of MARGINS so every existing loop over MARGINS stays as before.
NEVER_TOUCH_MARGIN_M = 0.05     # m [SF-65]: 5 cm; David's value for the cat demo
# Pushing starts this far beyond a pair's margin and stops this far beyond it (M288). Stage 2 uses them.
PUSH_START_BEYOND_M = 0.01      # m, PROVISIONAL [M288]
PUSH_STOP_BEYOND_M = 0.02       # m, PROVISIONAL [M288]

# Torso exemption by REAL geometry (SF-19): an arm shape is exempt from the torso only if its link's real geometry is
# within 1 mm of the real torso mesh at home (it touches the torso by construction). Parts listed here (provisional; see the report).
TORSO_EXEMPT_PARTS = ("shoulder_mount",)

# Held objects (M289, SF-4): match radius beyond an object's own radius (the pad match is a sanity check since Part 2; the object named by the policy command is the primary source; no fallback capsule: no match = empty gripper).
HELD_MATCH_EXTRA_M = 0.01       # m, PROVISIONAL [M289]: the nearest graspable object within its radius + 1 cm of the finger pad midpoint

# Broad phase (M291): padding added to the arm's key-point box: the largest capsule radius plus the
# activation distance plus the world margin, and at least the release distance.
BROAD_WORLD_MARGIN_M = 0.01     # m, PROVISIONAL [M291]: the world margin added to the broad-phase box

# Conversion and checks after a push (M295, SF-2). Stage 2 uses them; listed here so all settings live in one file.
BRANCH_GUARD_RAD = 0.25         # rad, PROVISIONAL [M295]: no joint more than this from the last filtered command
FK_CHECK_ELBOW_HAND_M = 0.002   # m, PROVISIONAL [SF-2]: forward kinematics of the converted joints must reproduce the elbow and hand within this
FK_CHECK_ANGLE_DEG = 0.5        # deg, PROVISIONAL [SF-2]: and the hand orientation within this

# Gap rule (M294, PROVISIONAL). Stage 2 uses it (src/safety_filter/gap_rule.py).
GAP_RULE = {
    "mode": "ours",              # text [SF-29]: "ours" = the gap rule of M294; "paper_plain" = gap rule off (no speed limit, a pair is pushed only when it is below its margin, back to the margin in one step); not a copy of Kong's compiled filter
    "approach_per_s": 30.0,      # 1/s [M294]: allowed approach speed = this x the gap above the margin
    "shrink_fraction": 0.2,      # fraction [M294]: per tick the gap may shrink by at most this share of what is left (30 /s at 150 Hz is 0.2; the code uses the speed times the tick length, so this value is documentation only)
    "max_grow_speed_m_s": 0.05,  # m/s [M294]: inside the margin the gap must grow at 30 /s x the violation, capped here
}

# Resolver (M293, Stage 2): rounds of the push and when it stops. Every value has its decision row (SF-28 to SF-39).
RESOLVER = {
    "rounds": 10,                # count [M293, SF-31]: most rounds of the push per tick
    "stop_m": 0.0005,            # m [M293, SF-31]: stop when the worst remaining shortfall is under this; also the pass line of the shape re-check after a push (SF-35)
    "trigger_m": 1.0e-6,         # m, PROVISIONAL [SF-30]: a pair needs action only when its shortfall is above this and an arm of it is closing (below this is rounding noise; 1 micrometre, my choice)
    "closing_eps_m": 1.0e-9,     # m, PROVISIONAL [SF-30]: an arm is closing when moving only that arm to its raw command makes the pair's gap smaller by more than this
    "shorten_halvings": 12,      # count, PROVISIONAL [SF-34]: halvings of the step fraction in the step shortening search (resolution 1/4096)
    "shorten_min_fraction": 0.001,  # fraction, PROVISIONAL [SF-34]: if the largest passing fraction is below this the result is hold-still (reason SHORTEN_ZERO)
    "shorten_after_failed_push": True,  # on/off [SF-34]: if the shape re-check still fails after the rounds of an elbow swing or hand slide, try step shortening once before holding still (M293: "if that does not clear it, shorten the step")
    "swing_unmovable_m": 0.002,  # m, PROVISIONAL [SF-32]: a contact point closer than this to the straight line from shoulder to wrist cannot be moved by an elbow swing; it is handled by step shortening
    "swing_straight_m": 0.002,   # m, PROVISIONAL [SF-32]: an elbow closer than this to the shoulder-wrist line (arm straight) cannot swing; step shortening instead
    "branch_guard_joints": (0, 1, 2, 3, 4, 5, 6),  # joint indices (0 = joint 1) [SF-35]: the branch guard looks at these joints; the plan names none, so all seven (decision needed)
    "accuracy_pos_m": 0.002,     # m, PROVISIONAL [SF-35, SF-2]: hand position after a push must be within this of its target
    "accuracy_deg": 0.5,         # deg, PROVISIONAL [SF-35, SF-2]: hand orientation after a push must be within this of its target
    "shortening_accuracy_target": "raw",  # text [SF-36]: "raw" = after step shortening the hand must be within the accuracy of the RAW commanded pose (the task text; fails whenever shortening slows the hand by more than 2 mm); "skip" = no accuracy check after shortening
    "jacobian_damping": 1.0e-4,  # damping of the one Jacobian step (Levenberg-Marquardt term, unit rad^2/m^2 mixed; PROVISIONAL) [SF-32]: keeps the single step finite near singular arm poses
    "solver_current_reset": True,   # on/off [SF-32]: the converter is reset to the working pose before each call, so one call never depends on the last one (determinism)
    "elbow_displacement_mode": "plain",  # text [SF-32]: "plain" = the elbow is displaced by the full push vector of the contact (no lever weighting) -- PLACEHOLDER, the plan does not define the lever rule for a fixed shoulder or wrist end
    # ---- Stage 2b (SF-40 to SF-50) ----
    "mode": "kong_style",        # text [SF-40, SF-41]: which resolver runs: "kong_style" = one combined push of the shoulder/elbow/wrist/hand points, then link length restore, then conversion (default, David's decision); "layered" = the separate tools (elbow swing, hand slide, step shortening); "cbf" = not built, raises NotImplementedError
    "elbow_swing_hand_target": "slid",  # text [SF-49]: layered resolver only: "slid" = after a hand slide in the same tick every later elbow swing (and the accuracy check) aims at the slid hand position; "raw" = the old behaviour (aims at the raw commanded hand pose)
    "min_lever_fraction": 0.2,   # fraction [SF-44]: kong_style: an upper arm contact closer than this share of the way from the shoulder to the elbow is not pushed (the elbow would have to move 1/fraction times the push); step shortening is used for it instead
    "lever_weight_elbow": 1.0,   # weight, no unit [SF-43]: kong_style forearm contact: how willing the elbow is to move compared with the wrist (wE in the split formula); 1.0 = equal
    "lever_weight_wrist": 1.0,   # weight, no unit [SF-43]: kong_style forearm contact: how willing the wrist is to move compared with the elbow (wW in the split formula); 1.0 = equal
    "tunnel_guard_min_gap_m": 0.0,  # m [SF-46]: kong_style jump-through guard: the straight joint-space path from the last command to the raw command may not pass any pair closer than this (0.0 = touching)
    "tunnel_guard_max_samples": 50,  # count [SF-46]: kong_style jump-through guard: most poses sampled along that path (the number used is the largest keypoint move divided by its capsule radius, rounded up, but never more than this)
    "tunnel_guard_skip_already_below": True,  # on/off [SF-46]: kong_style jump-through guard: a pair already below the guard gap at the last command is not counted (the gap rule and the push handle it; the forearm already overlaps the torso shape by 27 mm at the zombie pose, which would stop the arm forever); my choice, David to confirm
    "kong_shortened_accuracy": "own_pose",  # text [SF-47]: kong_style: an arm whose step was shortened inside the sweeps is checked for hand accuracy against its own shortened pose ("own_pose", so the check passes) or against the raw commanded pose ("raw", fails when shortening slows the hand by more than the accuracy value); my choice, David to confirm
    # ---- Stage 5 speed (SF-73): the conservative advancement of the jump-through guard only saves work; the sampled test and the halving search decide (kong_style._guard) ----
    "tunnel_guard_advance_max_iter": 200,   # count [SF-73]: most advances of the conservative advancement before it stops on the safe side
    "tunnel_guard_grid_walk_max": 8,        # count [SF-73]: most grid steps (1 / 2^shorten_halvings of the step each) the guard walks up after the advancement stops, while the next grid pose is clear, to land on the cell the halving search ended in
    "tunnel_guard_advance_refine_m": 0.05,  # m [SF-73]: a pair whose boxes are closer than the guard gap plus this gets an exact lower bound of its gap in each advance step; further pairs use their boxes only
    # ---- Stage 2g (SF-52 to SF-58) ----
    "pair_order": "deepest_first",  # text [SF-58]: kong_style: the order in which one sweep pushes the pairs: "deepest_first" = largest shortfall first, ties by pair name (default, David's decision); "by_name" = alphabetical by pair name (for order-independence tests only)
}

# Object catalog (M290): shape and tags of each object type. Sizes are taken from the object model files
# (models/objects/beaker.xml, hotplate.xml), checked by experiments/safety_filter_touch_test.py, not taken from scene positions.
# centre_above_bottom_m: the shape centre relative to the entry's position (its bottom centre).
OBJECT_CATALOG = {
    "beaker": {
        # A round object is a CAPSULE (M287, SF-8): radius 0.035 m, segment half length 0.05 m along the beaker axis.
        # The capsule contains the cylinder exactly (its end caps reach 3.5 cm past the rim, which is conservative).
        "shape": "capsule", "radius_m": 0.035, "half_height_m": 0.05,
        "centre_above_bottom_m": 0.05, "graspable": True, "round": True, "allowed_contact": False,
        "source": "models/objects/beaker.xml geom task_beaker_glass: cylinder radius 0.035, half height 0.05 at z 0.05; catalog shape capsule (M287)",
    },
    "hotplate": {
        "shape": "box", "half_m": (0.09, 0.09, 0.03),
        "centre_above_bottom_m": 0.03, "graspable": False, "round": False, "allowed_contact": True,   # a held object may touch it (zero margin, approach ceiling)
        "source": "models/objects/hotplate.xml geom task_hotplate_base: box half sizes 0.09 0.09 0.03 at z 0.03",
    },
}

# Predictive pair sleep (Stage 5 speed, SF-72; src/safety_filter/tick.py SleepTimers). A pair far from every other part is not looked at
# for as many ticks as its gap lets it close at the fastest speed the arms are allowed. Not a change of the rule: a pair is only skipped
# while it cannot reach its release distance (margin + slowing band) within the sleep, by this speed bound.
SLEEP = {
    "enabled": True,                # on/off [SF-72]: predictive sleep
    # the joint speed bound is NOT a setting here: the caller passes it to Resolver(joint_speed_rad_s=...), read from the command bridge limit (src/task_config.py COMMAND_BRIDGE_CFG max_joint_velocity_rad_s); without it the sleep is off [SF-72]. A command step faster than that x dt x (1 + safety_buffer) wakes the pairs of that arm for that tick.
    "safety_buffer": 0.20,          # fraction [SF-72]: extra on the speed bound for the capsule fitting error and the looseness of the lever arms (David's value)
    "drift_budget_m": 0.005,        # m (sum of the base, pedestal and finger changes), PLACEHOLDER — tuned by test [Part 2]: slow drift of the base, pedestal and fingers is added up since a pair was scheduled; a pair wakes when the total passes this, and its sleep length leaves this much room
    "state_change_m": 2.0e-4,       # m [SF-72]: if the base position and orientation (as numbers), the pedestal and the finger travel together change by more than this since the tick before, every pair wakes for that tick (they are not part of the speed bound)
    "command_step_slack_rad": 1.0e-12,   # rad [SF-72]: added to the largest command step allowed by the speed bound, so rounding never turns an exactly-at-the-bound step into a "fast" one
    "same_pose_atol_rad": 1.0e-9,   # rad [SF-72]: the last command counts as "the pose the filter returned the tick before" when every joint is within this
    "min_cost_m": 1.0e-12,          # m [SF-72]: floor of a pair's approach per tick in the sleep length formula (a pair whose two parts cannot move gets the longest sleep, not a division by zero)
    "stats_cap_ticks": 2000,        # ticks [SF-72]: the sleep length histogram (for the profile) stops counting at this many ticks
}

# Array form of a tick (Stage 5 speed, SF-69 to SF-71; src/safety_filter/tick.py, tick_arrays.py). Sizes and the tolerances of the bounds
# that rule pairs out. The bounds only decide which pairs get the exact check; the exact number is always the scalar function's.
ARRAY_FORM = {
    "pose_cache": 64,               # count [SF-69]: poses kept per tick before the cache is emptied (keeps a long shortening search's memory flat; the value the object form used)
    "table_cache": 12,              # count [SF-69]: pair tables kept per resolver (one per set of world objects and held shapes seen)
    "cc_eps_m": 1.0e-7,             # m [SF-71]: half width of a batched capsule-capsule gap's interval around the scalar value (same formula; the test measures the difference, below 1e-12 m)
    "cb_low_per_m": 4.5e-5,         # per m of capsule length [SF-71]: the exact (4001-sample) capsule-box gap is at least the batched value minus this x length (derivation in tick_arrays.py)
    "cb_high_per_m": 1.5e-4,        # per m of capsule length [SF-71]: the exact capsule-box gap is at most the batched value plus this x length
    "bound_slack_m": 1.0e-9,        # m [SF-71]: added to every bound comparison so that rounding can never turn "cannot decide" into "decided"
}

# Never-close pairs by divide and conquer (Part B, SF-75 to SF-79; tools/never_close_dc.py, offline only: the filter itself reads none of these).
# The proof and its assumptions are in docs/safety_filter_never_close_soundness.md; change that file with any change here that touches the method.
NEVER_CLOSE_DC = {
    "min_split_rad": 0.05,          # rad [SF-75]: a hinge range of a box is not cut once it is this wide or narrower (David's default); a box with nothing left to cut and no proof leaves the pair checked
    "min_split_m": 0.005,           # m [SF-75]: the same for a slide (pedestal, finger travel); 5 mm, so a slide adds at most 2.5 mm of travel at the smallest box (my choice)
    "pair_time_cap_s": 120.0,       # s [SF-78]: longest the tool works on one pair; a pair not finished by then stays checked (David's default, SF-27 "about 2 minutes")
    "batch_boxes": 8192,            # count [SF-75]: boxes evaluated in one numpy pass (memory: a capsule-box pass holds batch x (box_samples + 1) x 3 numbers, about 25 MB)
    "box_samples": 128,             # count K [SF-79]: a capsule's distance to a box is sampled at K + 1 equally spaced points; the centre value is the smallest sample minus length / (2K) (1.2 mm for a 0.3 m capsule)
    "eps_num_m": 1.0e-6,            # m [SF-79]: taken off every centre value and added to the threshold for rounding in the kinematics and the distance formula (their measured error is below 1e-9 m)
    "pre_random": 20000,            # count [SF-75]: random poses tried first; a pose with the gap below the threshold ends the pair at once as "reachable" (cannot be never-close)
    "pre_refine_rounds": 30,        # count [SF-75]: rounds of the local search that follows the random poses (keeps the closest poses and perturbs them, step halved every 5 rounds)
    "pre_refine_keep": 64,          # count [SF-75]: poses kept per round of that search
    "pre_refine_children": 8,       # count [SF-75]: perturbed copies per kept pose and round
    "pre_refine_step": 0.2,         # fraction of each variable's range [SF-75]: first perturbation size of that search
    "seed": 0,                      # the random generator seed (repeatable runs)
    "fk_check_poses": 300,          # count [SF-76]: random joint states on which the tool's batched kinematics is compared with the filter's (RobotShapes.place) before any pair is tested
    "fk_check_tol_m": 1.0e-9,       # m [SF-76]: the tool stops if any shape point differs by more than this in that comparison
    "verify_leaves": 2000,          # count [SF-76]: certificate check: settled boxes per declared pair re-checked with the filter's own scalar functions (the lowest-bound boxes first, the rest at random)
    "verify_poses_per_leaf": 8,     # count [SF-76]: random poses inside each such box whose filter gap must be at least the box's lower bound
}


# ---- Braking layer (Part 2, David's braking brief; src/safety_filter/braking.py). ----
# The four tunables. All values are placeholders until tuned by test.
BRAKING = {
    "hand_speed_cap_m_s": 0.20,     # m/s, PLACEHOLDER — tuned by test: hand-level speed limit at the tool point; separate from the bridge's per-joint 0.5 rad/s cap; enforced every tick by scaling back the joint commands
    "accel_limit_m_s2": 0.50,       # m/s^2, PLACEHOLDER — tuned by test: the one limit for braking and for speeding up, measured at the tool point
    "jerk_limit_m_s3": 2.00,        # m/s^3, PLACEHOLDER — tuned by test: how fast the acceleration may change (per tick: jerk x dt); makes the speed curve S-shaped both ways
    "approach_ceiling_m_s": {"hotplate": 0.05},  # m/s per allowed-contact surface name (the object id); PLACEHOLDER — tuned by test; a surface not listed gets hand_speed_cap_m_s
    # ---- settings the brief leaves implicit: simplest reading, named so they can change (listed as decisions) ----
    "tick_hz": 150.0,               # Hz: the filter tick = the control loop's source_hz (src/control_loop.py cfg); the jerk limit is counted per tick of this rate
    "bridge_reach_m": None,         # m: lever arm that turns the bridge's joint speed cap into a tool speed. None = DERIVED at startup from the arm model (sum of the link lengths from joint 1 to the tool point, larger arm); a number overrides it (tests)
    "tool_offset_local_m": (0.0, 0.0, 0.0),  # m in the link 7 frame: tool point = link 7 origin + this (PLACEHOLDER: the tool point is the link 7 origin until David names another)
    "speed_source": "sent",         # text: which joint series the measured speed is taken from: "sent" (the commands the filter sent) or "state" (the measured joints, which lag the command)
    "emergency_overshoot_ticks": 2,  # ticks of travel: the look-ahead has failed (override the bridge, log an error) when the stopping distance exceeds the room by more than this many ticks at the hand speed cap
    "stop_speed_m_s": 0.002,        # m/s: below this the hand counts as stopped (leaves braking, ends a hold)
    "closing_eps_m": 1e-6,          # m: the gap must shrink by more than this between last and new command to count as closing
}


def approach_ceiling(surface, tun=None):
    """The approach speed ceiling of an allowed-contact surface (m/s): its entry in BRAKING["approach_ceiling_m_s"], else the hand speed cap."""
    t = BRAKING if tun is None else tun
    return float(t["approach_ceiling_m_s"].get(surface, t["hand_speed_cap_m_s"]))


_REACH_CACHE = {}


def arm_reach_m(tun=None):
    """The reach of an arm from the robot model, computed once per process: the sum of the link lengths from the first joint's anchor to
    the last joint's anchor (distances between consecutive arm joint anchors; fixed, pose independent) plus the distance from the last
    anchor to the tool point (link 7 origin + the tool offset); the larger of the two arms. Used as the lever arm of the bridge cap."""
    t = BRAKING if tun is None else tun
    off = tuple(float(x) for x in t["tool_offset_local_m"])
    if off not in _REACH_CACHE:
        import mujoco
        import numpy as np
        from src.safety_filter.robot_model import load_robot_only_model
        m = load_robot_only_model()
        d = mujoco.MjData(m)
        mujoco.mj_kinematics(m, d)
        best = 0.0
        for side in ("left", "right"):
            jids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_{side}_joint{k}") for k in range(1, 8)]
            anc = d.xanchor[jids]
            links = float(np.linalg.norm(np.diff(anc, axis=0), axis=1).sum())
            b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"openarm_{side}_link7")
            R7 = d.xmat[b].reshape(3, 3)
            tool = d.xpos[b] + R7 @ np.asarray(off)
            best = max(best, links + float(np.linalg.norm(tool - anc[-1])))
        _REACH_CACHE[off] = best
    return _REACH_CACHE[off]


def derive(tun=None, bridge=None):
    """The derived constants of the braking layer, computed from the tunables, the tick rate, the arm model and the bridge limits (not
    hard-coded):
      tick_dt_s, tick_cushion_m (= hand speed cap x tick), bridge_reach_m (from the arm model unless overridden), effective_accel_limit
      (= min(accel limit, what the bridge can deliver per tick)), worst_stop_dist_m (stopping distance from the hand speed cap with the
      acceleration at its limit: the fixed wake radius for pruning and the smallest activation cutoff of the look-ahead), arm_lag_rad (the
      bridge's tracking error limit: how far the measured arm may be from the command).
    bridge: COMMAND_BRIDGE_CFG-like dict; None reads src/task_config.py."""
    from src.safety_filter.braking_math import stopping_distance
    t = BRAKING if tun is None else tun
    if bridge is None:
        from src import task_config as tc
        bridge = tc.COMMAND_BRIDGE_CFG
    dt = 1.0 / float(t["tick_hz"])
    cap, a, j = float(t["hand_speed_cap_m_s"]), float(t["accel_limit_m_s2"]), float(t["jerk_limit_m_s3"])
    reach = t["bridge_reach_m"] if t["bridge_reach_m"] is not None else arm_reach_m(t)
    bridge_tool_speed = float(bridge["max_joint_velocity_rad_s"]) * float(reach)   # m/s the bridge's joint cap allows at the tool, at most
    a_eff = min(a, bridge_tool_speed / dt)            # the bridge can change the tool speed by at most its own tool speed cap in one tick
    cushion = cap * dt
    return {"tick_dt_s": dt, "tick_cushion_m": cushion, "effective_accel_limit": a_eff, "bridge_reach_m": float(reach),
            "bridge_tool_speed_m_s": bridge_tool_speed, "arm_lag_rad": float(bridge["max_tracking_error_rad"]),
            "worst_stop_dist_m": stopping_distance(cap, a_eff, a_eff, j, cushion)}


def derived():
    """derive() for the current tunables (cheap: the arm reach is computed once)."""
    return derive()


def __getattr__(name):                 # BRAKING_DERIVED is computed on first use (it needs the arm model), not at import
    if name == "BRAKING_DERIVED":
        return derive()
    raise AttributeError(name)
