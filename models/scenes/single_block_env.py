"""Single block environment implementation."""

import mujoco
from src.environment import Environment


class SingleBlockEnvironment(Environment):
    """Materials lab task environment."""
    
    def __init__(self, model_path: str, scene_path: str = None):
        """
        Initialize materials lab environment.
        
        Args:
            model_path: Path to robot model XML
            scene_path: Optional path to scene XML (objects to add)
        """
        super().__init__(model_path)
        
        # If scene provided, load it too
        if scene_path:
            # TODO: Load scene objects
            pass
        
        self.task_complete = False
    
    def reset(self):
        """Reset environment to initial state."""
        mujoco.mj_resetData(self.model, self.data)
        self.task_complete = False
    
    def get_observation(self):
        """Get current state."""
        return {
            "time": self.data.time,
            "joint_angles": self.data.qpos.copy(),
            "joint_velocities": self.data.qvel.copy(),
            "joint_torques": self.data.ctrl.copy(),
        }
    
    def check_task_complete(self) -> bool:
        """Check if task is complete."""
        return self.task_complete