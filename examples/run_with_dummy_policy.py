"""Run simulation with dummy policy - with visualization."""

import sys
from pathlib import Path
import numpy as np
import mujoco.viewer

# Auto-detect project root
current_file = Path(__file__).resolve()
project_root = current_file.parent.parent
sys.path.insert(0, str(project_root))

from src.simulator import RobotSimulator
from src.controller import RobotController
from models.policies.dummy_policy import DummyPolicy


def run_simulation():
    print("=" * 50)
    print("Materials Lab Simulation - Dummy Policy Test (with viewer)")
    print("=" * 50)
    
    # Initialize
    sim = RobotSimulator(
        model_path=str(project_root / "models/scenes/single_block.xml")
    )
    controller = RobotController()
    policy = DummyPolicy()
    
    # Launch viewer
    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        # Set camera angle
        viewer.cam.distance = 2.0        # Zoom out
        viewer.cam.azimuth = 45          # Rotate 45 degrees
        viewer.cam.elevation = 30        # Tilt up 30 degrees
        viewer.cam.lookat = [0.5, 0, 0.5]  # Look at the block area
        
        duration = 3.0
        t = 0.0
        step = 0
        
        initial_block = sim.data.body('block').xpos.copy()
        print(f"Initial block pos: {initial_block}")
        
        while viewer.is_running() and t < duration:
            state = sim.get_state()
            action = policy(state, t)
            torques = controller.step(action, state)
            sim.data.ctrl[:] = torques
            sim.step()
            viewer.sync()
            
            if step % 250 == 0:
                block_pos = sim.data.body('block').xpos.copy()
                right_j1 = state['joint_angles'][17]
                right_j2 = state['joint_angles'][18]
                
                print(f"t={t:.2f}s - Block: {block_pos}, j1={right_j1:.3f}, j2={right_j2:.3f}")
            
            t += sim.model.opt.timestep
            step += 1
        
        print(f"\n✓ Simulation complete! Ran {step} steps")
        final_block = sim.data.body('block').xpos.copy()
        print(f"Final block pos: {final_block}")
        print(f"Block moved: {np.linalg.norm(final_block - initial_block):.4f} m")

if __name__ == "__main__":
    run_simulation()