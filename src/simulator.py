"""Main simulation environment."""

from pathlib import Path
import mujoco
from src import robot_specs


class RobotSimulator:
    """simulation environment."""
    
    def __init__(self, model_path: str = None):
        """
        Initialize simulator.
        
        Args:
            model_path: Path to MJCF model/scene file
        """
        if model_path is None:
            project_root = Path(__file__).parent.parent
            model_path = project_root / "deps/OpenArm-Combined/combined_robot.xml"
        
        model_path = Path(model_path)
        if not model_path.exists():
            raise FileNotFoundError(f"Model not found: {model_path}")
        
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        
        print(f"✓ Loaded model: {model_path.name}")
        print(f"  Bodies: {self.model.nbody}")
        print(f"  Joints: {self.model.njnt}")
        print(f"  Actuators: {self.model.nu}")
    
    def step(self):
        """Step simulation forward."""
        mujoco.mj_step(self.model, self.data)
    
    def get_state(self):
        """Get current robot state."""
        return {
            "time": self.data.time,
            "joint_angles": self.data.qpos.copy(),
            "joint_velocities": self.data.qvel.copy(),
            "joint_torques": self.data.ctrl.copy(),
        }
    
    def reset(self):
        """Reset to initial state."""
        mujoco.mj_resetData(self.model, self.data)
    
    def close(self):
        """Clean up."""
        pass

    def get_block_position(self):
        """Get block position in world frame."""
        return self.data.body('block').xpos.copy()

    def get_block_velocity(self):
        """Get block velocity."""
        return self.data.body('block').cvel.copy()

    def get_observation(self):
        """Get full observation (robot + environment)."""
        return {
            "time": self.data.time,
            "joint_angles": self.data.qpos.copy(),
            "joint_velocities": self.data.qvel.copy(),
            "joint_torques": self.data.ctrl.copy(),
            "block_position": self.get_block_position(),
            "block_velocity": self.get_block_velocity(),
        }