"""Test the controller."""

import sys
from pathlib import Path

# Auto-detect project root
current_file = Path(__file__).resolve()
project_root = current_file.parent.parent
sys.path.insert(0, str(project_root))

from src.controller import RobotController
from src.simulator import RobotSimulator
from src.actuator_mapping import ACTUATOR_INDICES

def test_controller():
    print("Testing Controller...")
    print(f"Number of actuators: {len(ACTUATOR_INDICES)}")
    
    # Initialize
    sim = RobotSimulator()
    controller = RobotController()
    
    # Test state
    state = sim.get_state()
    print(f"Total joints: {len(state['joint_angles'])}")
    print(f"Total velocities: {len(state['joint_velocities'])}")
    print(f"Controlled actuators: {len(ACTUATOR_INDICES)} (indices {ACTUATOR_INDICES})")
    
    # Target: move only the actuators (20 dims)
    target = np.ones(len(ACTUATOR_INDICES)) * 0.1  # Small perturbation
    
    # Compute torques
    torques = controller.step(target, state)
    print(f"\nComputed {len(torques)} torques")
    print(f"Torque range: [{torques.min():.2f}, {torques.max():.2f}]")
    print(f"Torques shape: {torques.shape}")
    
    print("\n✓ Controller test passed!")

if __name__ == "__main__":
    import numpy as np
    test_controller()