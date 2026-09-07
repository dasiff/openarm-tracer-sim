"""Map actuator names to action indices (0-19)."""

# Action space indices (0-19) correspond to these actuators
ACTUATOR_NAMES = [
    # Tracer (2)
    "left_motor",        # 0
    "right_motor",       # 1
    
    # Left arm (9)
    "left_joint1",       # 2
    "left_joint2",       # 3
    "left_joint3",       # 4
    "left_joint4",       # 5
    "left_joint5",       # 6
    "left_joint6",       # 7
    "left_joint7",       # 8
    "left_finger1",      # 9
    "left_finger2",      # 10
    
    # Right arm (9)
    "right_joint1",      # 11
    "right_joint2",      # 12
    "right_joint3",      # 13
    "right_joint4",      # 14
    "right_joint5",      # 15
    "right_joint6",      # 16
    "right_joint7",      # 17
    "right_finger1",     # 18
    "right_finger2",     # 19
]

# Mapping name → action index
NAME_TO_IDX = {name: idx for idx, name in enumerate(ACTUATOR_NAMES)}
IDX_TO_NAME = {idx: name for idx, name in enumerate(ACTUATOR_NAMES)}

# For backward compatibility
ACTUATOR_INDICES = list(range(20))  # Just 0-19