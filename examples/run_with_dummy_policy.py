"""Run simulation with dummy policy - with visualization and logging."""

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
from src.logger import TrajectoryLogger
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
    logger = TrajectoryLogger()
    
    # Launch viewer
    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        # Set camera angle
        viewer.cam.distance = 2.5
        viewer.cam.azimuth = 45
        viewer.cam.elevation = -45
        viewer.cam.lookat = [0.3, 0, 0.6]
        
        duration = 3.0
        t = 0.0
        step = 0
        
        initial_block = sim.get_block_position()
        print(f"Initial block pos: {initial_block}")
        
        while viewer.is_running() and t < duration:
            observation = sim.get_observation()
            
            # Log the observation
            logger.record(observation)
            
            # Policy decides action
            action = policy(observation, t)
            
            # Controller converts to torques
            torques = controller.step(action, observation)
            
            # Apply and step
            sim.data.ctrl[:] = torques
            sim.step()
            viewer.sync()
            
            if step % 250 == 0:
                block_pos = sim.get_block_position()
                print(f"t={t:.2f}s - Block: {block_pos}")
            
            t += sim.model.opt.timestep
            step += 1
        
        print(f"\n✓ Simulation complete! {step} steps")
        final_block = sim.get_block_position()
        print(f"Final block pos: {final_block}")
        print(f"Block moved: {np.linalg.norm(final_block - initial_block):.4f} m")
        
        # Save trajectory
        logger.save()

if __name__ == "__main__":
    run_simulation()