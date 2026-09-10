import mujoco.viewer
import sys
from pathlib import Path
#still in development--does not work yet!!!
# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from src.simulator import RobotSimulator

# Use absolute path to the model file
model_path = project_root / "models" / "scenes" / "chemistry_lab.xml"

sim = RobotSimulator(model_path=str(model_path))
sim.data.qpos[0:3] = [2.0, 2.0, 0.0]   # x, y, z
mujoco.mj_forward(sim.model, sim.data)  # propagate before rendering
with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -25
    viewer.cam.distance = 6.0
    viewer.cam.lookat[:] = [0, 0, 1.0]
    while viewer.is_running():
        sim.step()
        viewer.sync()