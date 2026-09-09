"""Dummy policy for single block task - hardcoded sequence."""

import numpy as np
from src.actuator_mapping import ACTUATOR_NAMES, NAME_TO_IDX


class DummyPolicy:
    """Hardcoded sequence with smooth trajectories."""
    
    def __init__(self):
        self.n_actuators = len(ACTUATOR_NAMES)
    
    def __call__(self, state, t=None):
        action = np.zeros(self.n_actuators)
        
        # Smooth ramp up
        if t < 2.0:
            ramp = t / 2.0
            action[NAME_TO_IDX["right_joint1"]] = 0.1 * ramp   # Reach forward
            action[NAME_TO_IDX["right_joint2"]] = -0.5 * ramp  # Point DOWN (negative!)
            action[NAME_TO_IDX["right_joint3"]] = -0.2 * ramp  # More down
        
        # Hold
        elif t < 3.0:
            action[NAME_TO_IDX["right_joint1"]] = 0.1
            action[NAME_TO_IDX["right_joint2"]] = -0.5
            action[NAME_TO_IDX["right_joint3"]] = -0.2
        
        return action