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

# Gap rule (M294, PROVISIONAL). Stage 2 uses it; on/off setting for the paper's plain behaviour.
GAP_RULE = {
    "on": True,                  # on/off [M294]: False gives the paper's plain behaviour for comparison
    "approach_per_s": 30.0,      # 1/s [M294]: allowed approach speed = this x the gap above the margin
    "shrink_fraction": 0.2,      # fraction [M294]: per tick the gap may shrink by at most this share of what is left
    "max_grow_speed_m_s": 0.05,  # m/s [M294]: inside the margin the gap must grow at 30 /s x the violation, capped here
}

# Resolver (M293, Stage 2): rounds of the push and when it stops.
RESOLVER = {
    "rounds": 10,                # count [M293]: most rounds of the push per tick
    "stop_m": 0.0005,            # m [M293]: stop when every pair is within this of its target
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
