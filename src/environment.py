"""Base environment template."""

from abc import ABC, abstractmethod
from pathlib import Path
import mujoco


class Environment(ABC):
    """Abstract base class for simulation environments."""
    
    def __init__(self, model_path: str):
        """
        Initialize environment.
        
        Args:
            model_path: Path to MJCF model file
        """
        model_path = Path(model_path)
        if not model_path.exists():
            raise FileNotFoundError(f"Model not found: {model_path}")
        
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.model_path = model_path
    
    @abstractmethod
    def reset(self):
        """Reset environment to initial state."""
        pass
    
    @abstractmethod
    def get_observation(self):
        """Get current observation (state) of environment."""
        pass
    
    @abstractmethod
    def check_task_complete(self) -> bool:
        """Check if task is complete."""
        pass
    
    def step(self):
        """Step physics simulation forward."""
        mujoco.mj_step(self.model, self.data)
    
    def get_time(self) -> float:
        """Get current simulation time."""
        return self.data.time