"""Dummy policy for single block task - hardcoded sequence."""

import numpy as np
from src.actuator_mapping import ACTUATOR_NAMES, NAME_TO_IDX


class DummyPolicy:
    """Hardcoded sequence to push block to the side.
    
    Task: Move right arm forward to push block sideways.
    """
    
    def __init__(self):
        self.n_actuators = len(ACTUATOR_NAMES)
    
    def __call__(self, state, t=None):
        """
        Generate action based on time.
        
        Args:
            state: Current robot state (dict with joint_angles, etc.)
            t: Time since task started (seconds)
        
        Returns:
            action: 20-dim joint angle targets
        """
        
        # Default: stay at current position
        action = np.zeros(self.n_actuators)
        
        # Simple push sequence
        if t < 2.0:
            # Move right arm forward to push block
            action[NAME_TO_IDX["right_joint1"]] = 1.5   # Shoulder forward
            action[NAME_TO_IDX["right_joint2"]] = 1.0   # Elbow neutral
            
        # After 2 seconds, keep pushing
        
        return action