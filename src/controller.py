"""Robot joint controller using PD control."""

import numpy as np
from src import robot_specs
from src.actuator_mapping import ACTUATOR_INDICES


class RobotController:
    """PD controller for robot joints.
    
    Converts target joint angles to motor torques using proportional-derivative control.
    Only controls the 20 actuated joints (indices 6-25).
    Base free joint (indices 0-5) is not controlled.
    """
    
    def __init__(self):
        """Initialize controller with gains from robot_specs."""
        
        # PD gains organized by motor type
        self.gains = {
            "shoulder_elbow": {
                "kp": robot_specs.OPENARM_SPECS["motors"]["shoulder_elbow"]["kp"],
                "kd": robot_specs.OPENARM_SPECS["motors"]["shoulder_elbow"]["kd"],
            },
            "wrist": {
                "kp": robot_specs.OPENARM_SPECS["motors"]["wrist"]["kp"],
                "kd": robot_specs.OPENARM_SPECS["motors"]["wrist"]["kd"],
            },
            "wrist_fine": {
                "kp": robot_specs.OPENARM_SPECS["motors"]["wrist_fine"]["kp"],
                "kd": robot_specs.OPENARM_SPECS["motors"]["wrist_fine"]["kd"],
            },
        }
        
        # Only 20 actuators are controlled
        self.actuator_indices = ACTUATOR_INDICES
    
    def step(self, target_angles, current_state):
        """
        Compute motor torques from target angles.
        
        Args:
            target_angles: Desired joint angles (20-dim for actuators only)
            current_state: Current robot state with "joint_angles" and "joint_velocities"
        
        Returns:
            torques: Motor commands (20-dim, one per actuator)
        """
        
        current_angles = current_state["joint_angles"]
        current_velocities = current_state["joint_velocities"]
        
        # Extract only the actuated joints
        # (indices 0-5 are base free joint, not controlled)
        current_angles_actuated = current_angles[self.actuator_indices]
        current_velocities_actuated = current_velocities[self.actuator_indices]
        
        # Ensure target is correct size
        if len(target_angles) != len(self.actuator_indices):
            raise ValueError(
                f"Target angles size {len(target_angles)} doesn't match "
                f"actuators {len(self.actuator_indices)}"
            )
        
        # Compute PD torques
        position_error = target_angles - current_angles_actuated
        velocity_error = -current_velocities_actuated  # Damping: oppose motion
        
        # Apply gains (simplified - all joints use shoulder_elbow gains for now)
        kp = self.gains["shoulder_elbow"]["kp"]
        kd = self.gains["shoulder_elbow"]["kd"]
        
        torques = kp * position_error + kd * velocity_error
        
        return torques