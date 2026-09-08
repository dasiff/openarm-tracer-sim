"""Dummy policy for single block task - hardcoded sequence."""

import numpy as np
from src.actuator_mapping import ACTUATOR_NAMES, NAME_TO_IDX


class DummyPolicy:
    """Hardcoded sequence with smooth trajectories."""
    
    def __init__(self):
        self.n_actuators = len(ACTUATOR_NAMES)
    
    def __call__(self, state, t=None):
        """Generate smooth action trajectory."""
        
        action = np.zeros(self.n_actuators)
        
        # Smooth ramp up over first 2 seconds
        if t < 2.0:
            ramp = t / 2.0
            action[NAME_TO_IDX["right_joint1"]] = 0.1 * ramp  # Was 0.5
            action[NAME_TO_IDX["right_joint2"]] = -0.3 * ramp  # Was 0.3
        
        # Hold position
        elif t < 3.0:
            action[NAME_TO_IDX["right_joint1"]] = 0.5
            action[NAME_TO_IDX["right_joint2"]] = 0.3
        
        return action