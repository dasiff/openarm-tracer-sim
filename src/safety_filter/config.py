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
}

# Lab map (tools/survey_lab_map.py): which scene file to survey (the task scene, src/task_config.py TASK_CFG) [M290].
LAB_MAP_SCENE = "models/scenes/chemistry_lab_beaker_hotplate.xml"
