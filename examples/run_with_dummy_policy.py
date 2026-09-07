"""Run simulation with dummy policy."""

import sys
from pathlib import Path
import numpy as np

# Auto-detect project root
current_file = Path(__file__).resolve()
project_root = current_file.parent.parent
sys.path.insert(0, str(project_root))

from src.simulator import RobotSimulator
from src.controller import RobotController
from models.policies.dummy_policy import DummyPolicy


def run_simulation():
    print("=" * 50)
    print("Materials Lab Simulation - Dummy Policy Test")
    print("=" * 50)
    
    # Initialize
    sim = RobotSimulator(
        model_path=str(project_root / "models/scenes/single_block.xml")
    )
    controller = RobotController()
    policy = DummyPolicy()
    
    # Run for 3 seconds
    duration = 3.0
    t = 0.0
    step = 0
    
    print(f"\nRunning simulation for {duration} seconds...")
    
    while t < duration:
        # Get current state
        state = sim.get_state()
        
        # Policy decides action
        action = policy(state, t)
        
        # Controller converts action to torques
        torques = controller.step(action, state)
        
        # Apply torques to simulator
        sim.data.ctrl[:] = torques
        
        # Step physics
        sim.step()
        
        # Print progress
        if step % 250 == 0:  # Every 0.5 seconds
            print(f"  t={t:.2f}s - Block pos: {sim.data.body('block').xpos}")
        
        t += sim.model.opt.timestep
        step += 1
    
    print(f"\n✓ Simulation complete! Ran {step} steps")
    
    # Final state
    final_state = sim.get_state()
    print(f"Final joint angles: {final_state['joint_angles'][:5]}...")  # First 5

if __name__ == "__main__":
    run_simulation()