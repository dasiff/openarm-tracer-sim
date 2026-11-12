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
}
# Pushing starts this far beyond a pair's margin and stops this far beyond it (M288). Stage 2 uses them.
PUSH_START_BEYOND_M = 0.01      # m, PROVISIONAL [M288]
PUSH_STOP_BEYOND_M = 0.02       # m, PROVISIONAL [M288]

# Torso exemption by REAL geometry (SF-19): an arm shape is exempt from the torso only if its link's real geometry is
# within 1 mm of the real torso mesh at home (it touches the torso by construction). Parts listed here (provisional; see the report).
TORSO_EXEMPT_PARTS = ("shoulder_mount",)

# Held objects (M289, SF-4): match radius beyond an object's own radius, and the fallback capsule.
HELD_MATCH_EXTRA_M = 0.01       # m, PROVISIONAL [M289]: the nearest graspable object within its radius + 1 cm of the finger pad midpoint
HELD_FALLBACK_HEIGHT_M = 0.08   # m, PROVISIONAL [M289]: height of the fallback capsule when no object matches (default height; open)

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
}

# Object catalog (M290): shape and tags of each object type. Sizes are taken from the object model files
# (models/objects/beaker.xml, hotplate.xml), checked by experiments/safety_filter_touch_test.py, not taken from scene positions.
# centre_above_bottom_m: the shape centre relative to the entry's position (its bottom centre).
OBJECT_CATALOG = {
    "beaker": {
        # A round object is a CAPSULE (M287, SF-8): radius 0.035 m, segment half length 0.05 m along the beaker axis.
        # The capsule contains the cylinder exactly (its end caps reach 3.5 cm past the rim, which is conservative).
        "shape": "capsule", "radius_m": 0.035, "half_height_m": 0.05,
        "centre_above_bottom_m": 0.05, "graspable": True, "round": True,
        "source": "models/objects/beaker.xml geom task_beaker_glass: cylinder radius 0.035, half height 0.05 at z 0.05; catalog shape capsule (M287)",
    },
    "hotplate": {
        "shape": "box", "half_m": (0.09, 0.09, 0.03),
        "centre_above_bottom_m": 0.03, "graspable": False, "round": False,
        "source": "models/objects/hotplate.xml geom task_hotplate_base: box half sizes 0.09 0.09 0.03 at z 0.03",
    },
}
