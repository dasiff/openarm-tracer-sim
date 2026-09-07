"""Dummy policy for single block task - hardcoded sequence."""

import numpy as np
from src.actuator_mapping import ACTUATOR_NAMES, NAME_TO_IDX


class DummyPolicy:
    """Hardcoded sequence to demonstrate end-to-end pipeline.
    
    Task: Move right arm down to grasp block, close gripper, lift up.
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
        
        # Hardcoded sequence
        if t < 1.0:
            # Move right arm down to grasp height
            action[NAME_TO_IDX["right_joint1"]] = -0.5
            action[NAME_TO_IDX["right_joint2"]] = -1.0
            
        elif t < 1.5:
            # Close gripper (fingers)
            action[NAME_TO_IDX["right_joint1"]] = -0.5
            action[NAME_TO_IDX["right_joint2"]] = -1.0
            action[NAME_TO_IDX["right_finger1"]] = 0.04
            action[NAME_TO_IDX["right_finger2"]] = 0.04
            
        elif t < 2.5:
            # Lift arm up
            action[NAME_TO_IDX["right_joint1"]] = 0.0
            action[NAME_TO_IDX["right_joint2"]] = -0.5
            action[NAME_TO_IDX["right_finger1"]] = 0.04
            action[NAME_TO_IDX["right_finger2"]] = 0.04
        
        return action