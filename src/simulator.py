"""Main simulation environment."""

from pathlib import Path
from src.materials_lab_env import MaterialsLabEnvironment


class MaterialsLabSimulator:
    """Simulation environment for materials lab tasks."""
    
    def __init__(self, model_path: str = None):
        """
        Initialize simulator.
        
        Args:
            model_path: Path to MJCF robot model
        """
        if model_path is None:
            model_path = "deps/OpenArm-Combined/Tracer_Pedestal.xml"
        
        self.env = MaterialsLabEnvironment(model_path)
        print(f"✓ Loaded environment: {Path(model_path).name}")
        print(f"  Bodies: {self.env.model.nbody}")
        print(f"  Joints: {self.env.model.njnt}")
        print(f"  Actuators: {self.env.model.nu}")
    
    def step(self):
        """Step simulation forward."""
        self.env.step()
    
    def get_state(self):
        """Get current robot state."""
        return self.env.get_observation()
    
    def reset(self):
        """Reset to initial state."""
        self.env.reset()
    
    def close(self):
        """Clean up."""
        pass