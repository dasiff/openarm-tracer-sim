"""Materials lab environment implementation."""

from src.environment import Environment


class MaterialsLabEnvironment(Environment):
    """Materials lab task environment."""
    
    def __init__(self, model_path: str):
        """
        Initialize materials lab environment.
        
        Args:
            model_path: Path to robot + scene model
        """
        super().__init__(model_path)
        self.task_complete = False
    
    def reset(self):
        """Reset environment to initial state."""
        import mujoco
        mujoco.mj_resetData(self.model, self.data)
        self.task_complete = False
    
    def get_observation(self):
        """Get current state of materials lab."""
        return {
            "time": self.data.time,
            "joint_positions": self.data.qpos.copy(),
            "joint_velocities": self.data.qvel.copy(),
            "joint_torques": self.data.ctrl.copy(),
        }
    
    def check_task_complete(self) -> bool:
        """Check if materials lab task is complete."""
        # TODO: Implement task completion logic
        # For now, just return False
        return self.task_complete