"""Map actuator names to their state indices."""

# Mapping of actuator names to (position_index, velocity_index)
ACTUATOR_MAPPING = {
    # Tracer base (2 motors)
    "left_motor": {"joint": "left_joint", "idx": 6},      # 6th joint in model
    "right_motor": {"joint": "right_joint", "idx": 7},    # 7th joint
    
    # Left arm (9 actuators: 7 joints + 2 fingers)
    "left_joint1": {"joint": "openarm_left_joint1", "idx": 8},
    "left_joint2": {"joint": "openarm_left_joint2", "idx": 9},
    "left_joint3": {"joint": "openarm_left_joint3", "idx": 10},
    "left_joint4": {"joint": "openarm_left_joint4", "idx": 11},
    "left_joint5": {"joint": "openarm_left_joint5", "idx": 12},
    "left_joint6": {"joint": "openarm_left_joint6", "idx": 13},
    "left_joint7": {"joint": "openarm_left_joint7", "idx": 14},
    "left_finger1": {"joint": "openarm_left_finger_joint1", "idx": 15},
    "left_finger2": {"joint": "openarm_left_finger_joint2", "idx": 16},
    
    # Right arm (9 actuators: 7 joints + 2 fingers)
    "right_joint1": {"joint": "openarm_right_joint1", "idx": 17},
    "right_joint2": {"joint": "openarm_right_joint2", "idx": 18},
    "right_joint3": {"joint": "openarm_right_joint3", "idx": 19},
    "right_joint4": {"joint": "openarm_right_joint4", "idx": 20},
    "right_joint5": {"joint": "openarm_right_joint5", "idx": 21},
    "right_joint6": {"joint": "openarm_right_joint6", "idx": 22},
    "right_joint7": {"joint": "openarm_right_joint7", "idx": 23},
    "right_finger1": {"joint": "openarm_right_finger_joint1", "idx": 24},
    "right_finger2": {"joint": "openarm_right_finger_joint2", "idx": 25},
}

# Indices for actuators (use these for control)
ACTUATOR_INDICES = list(range(6, 26))  # Indices 6-25 are the 20 actuators
# Indices 0-5 are base free joint (don't control, only observe)