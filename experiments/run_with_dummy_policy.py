"""Run simulation with dummy policy - capture visual snapshots (FIXED)."""

import sys
from pathlib import Path
import numpy as np
import mujoco
import mujoco.viewer
from PIL import Image

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
    print("Materials Lab Simulation - Capturing Snapshots")
    print("=" * 50)
    
    # Initialize
    sim = RobotSimulator(
        model_path=str(project_root / "models/scenes/single_block.xml")
    )
    controller = RobotController(sim.model)  # now needs the model to derive correct addressing
    policy = DummyPolicy()
    logger = TrajectoryLogger()
    
    # Create snapshots directory
    snapshots_dir = project_root / "data" / "snapshots"
    snapshots_dir.mkdir(parents=True, exist_ok=True)
    
    # Create renderer
    renderer = mujoco.Renderer(sim.model)
    
    # Launch viewer
    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        # Set camera angle
        viewer.cam.distance = 2.5
        viewer.cam.azimuth = 45
        viewer.cam.elevation = -45
        viewer.cam.lookat = [0.1, -0.3, 1.0]
        
        duration = 3.0
        t = 0.0
        step = 0
        
        # Times to capture screenshots
        capture_times = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
        captured = set()
        
        initial_block = sim.get_block_position()
        print(f"Initial block pos: {initial_block}")
        
        while viewer.is_running() and t < duration:
            observation = sim.get_observation()
            logger.record(observation)
            
            # Policy and control
            action = policy(observation, t)
            torques = controller.step(action, observation)
            sim.data.ctrl[:] = torques
            sim.step()
            viewer.sync()
            
            # Capture screenshot at specific times
            for capture_time in capture_times:
                if abs(t - capture_time) < 0.002 and capture_time not in captured:
                    # Render from camera
                    renderer.update_scene(sim.data, camera=viewer.cam)
                    pixels = renderer.render()
                    
                    # Convert to PIL Image and save
                    # pixels is RGB numpy array (height, width, 3)
                    img = Image.fromarray(pixels, 'RGB')
                    filename = f"snapshot_t{capture_time:.1f}s.png"
                    filepath = snapshots_dir / filename
                    img.save(filepath)
                    
                    print(f"✓ Captured: {filename}")
                    captured.add(capture_time)
            
            if step % 250 == 0:
                block_pos = sim.get_block_position()
                print(f"t={t:.2f}s - Block: {block_pos}")
            
            t += sim.model.opt.timestep
            step += 1
        
        print(f"\n✓ Simulation complete! {step} steps")
        final_block = sim.get_block_position()
        print(f"Final block pos: {final_block}")
        print(f"Block moved: {np.linalg.norm(final_block - initial_block):.4f} m")
        
        logger.save()
        print(f"✓ Snapshots saved to: {snapshots_dir}")

if __name__ == "__main__":
    run_simulation()