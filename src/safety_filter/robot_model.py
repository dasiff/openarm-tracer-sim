"""The robot-only model the safety filter and its build tool both use.

It is the robot description (models/robot/combined_robot.xml, no scene) with the two robot
parts that the simulator adds on top of it and that have geometry: the battery pack and the
eagle camera's body (src/battery.py, src/cameras.py, the same specs as src/simulator.py).
Actuators, the face (visual only) and cameras are left out: they have no collision geometry.

The fingerprint stamps this model's compiled geometry (geom sizes, mesh vertices, body
offsets, joint axes and limits, names). robot_shapes.json carries it; the filter refuses to
start when the loaded model's fingerprint differs (SF-5).

No simulator, no scene, no world data is read here.
"""

import hashlib
from pathlib import Path

import mujoco
import numpy as np

from src import robot_specs
from src.battery import add_battery
from src.cameras import add_eagle_body

PROJECT_ROOT = Path(__file__).resolve().parents[2]
ROBOT_XML = PROJECT_ROOT / "models" / "robot" / "combined_robot.xml"


def load_robot_only_model(path=ROBOT_XML):
    """Compiles the robot description with the battery and the eagle camera's body.

    Args:
        path (str or Path): The robot description XML.

    Returns:
        mujoco.MjModel: the compiled robot-only model (not stepped; no scene objects).
    """
    spec = mujoco.MjSpec.from_file(str(Path(path).resolve()))
    add_battery(spec, robot_specs.BATTERY_SPECS)
    add_eagle_body(spec, robot_specs.EAGLE_BODY_SPECS, robot_specs.EAGLE_MOUNT)
    return spec.compile()


def robot_fingerprint(model):
    """SHA-256 over the compiled robot geometry: geom types, sizes, poses and names, mesh vertices,
    body offsets and parents, joint types, axes, anchors, limits and names.

    Args:
        model (mujoco.MjModel): a model from load_robot_only_model().

    Returns:
        str: the hex digest.
    """
    h = hashlib.sha256()
    arrays = [model.geom_type, model.geom_size, model.geom_pos, model.geom_quat, model.geom_bodyid,
              model.geom_contype, model.geom_conaffinity, model.geom_dataid,
              model.mesh_vert, model.mesh_vertadr, model.mesh_vertnum,
              model.body_parentid, model.body_pos, model.body_quat,
              model.jnt_type, model.jnt_axis, model.jnt_pos, model.jnt_range, model.jnt_bodyid]
    for a in arrays:
        h.update(np.ascontiguousarray(a).tobytes())
    for kind, n in ((mujoco.mjtObj.mjOBJ_GEOM, model.ngeom), (mujoco.mjtObj.mjOBJ_BODY, model.nbody),
                    (mujoco.mjtObj.mjOBJ_JOINT, model.njnt)):
        for i in range(n):
            h.update((mujoco.mj_id2name(model, kind, i) or "").encode() + b"\0")
    return h.hexdigest()
