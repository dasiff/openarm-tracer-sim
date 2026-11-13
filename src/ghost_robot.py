"""A see-through "ghost" copy of the whole robot, drawn only in the viewer window and the video (design plan 7.1; decisions-log row M321).

How it is drawn: the ghost has its own MjData (a second set of joint values for the same model). Its base pose, pedestal height and arm joints are written into that copy and the
kinematics are updated (no physics step, the simulation's own data is never touched). After the viewer's normal scene has been built (mjv_updateScene), the copy's geoms are appended to
the same scene with mjv_addGeoms; the appended geoms that do not belong to the robot (floor, bench, beaker...) are moved out of sight, and the robot's own are recoloured with a single
see-through colour and stripped of texture and reflection. Nothing here touches the model, the real data, the physics, or the policy cameras (they render their own scene, never this one).

    ghost = GhostRobot(model, data)                        # data: any state of the real robot; its arm joints are the "home" pose of the ghost
    ghost.set_pose(x, y, yaw, pedestal, left_q=None, right_q=None)
    ghost.draw(scene, opt, rgba)                           # after mujoco.mjv_updateScene(...), before mjr_render(...)
"""

import math

import mujoco
import numpy as np

from src import robot_specs
from src.robot_helpers import arm_ids

HIDDEN_Z = -1000.0      # m: where a geom of the copy that is not part of the robot is moved (far under the floor, with alpha 0 as well)


def _robot_bodies(model, root_name="base_link"):
    """Ids of the root body and of every body below it."""
    root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, root_name)
    ids = set()
    for b in range(model.nbody):
        c = b
        while c > 0 and c != root:
            c = int(model.body_parentid[c])
        if c == root:
            ids.add(b)
    return ids


class GhostRobot:
    """The second copy of the robot's joint values, and the drawing of it as a coloured see-through shape.

    Attributes:
        model (mujoco.MjModel): The robot model (shared, never changed).
        d (mujoco.MjData): The ghost's own joint values; only kinematics are computed on it.
        bodies (set[int]): Ids of the robot's bodies (base_link and below).
    """

    def __init__(self, model, data):
        """Args:
            model (mujoco.MjModel): The model.
            data (mujoco.MjData): A state of the real robot with the arms at home; its qpos is copied (arms at home, fingers open, base height)."""
        self.model = model
        self.d = mujoco.MjData(model)
        self.d.qpos[:] = data.qpos
        self.bodies = _robot_bodies(model)
        self.qadr = {s: arm_ids(model, s)["qadr"] for s in ("left", "right")}
        self.ped = [int(model.joint(j).qposadr[0]) for j in robot_specs.PEDESTAL_SPECS["joints"]]
        self.home = {s: np.array([self.d.qpos[a] for a in self.qadr[s]]) for s in ("left", "right")}
        self.base_z = float(data.qpos[2])
        self.geom_is_robot = np.array([int(model.geom_bodyid[g]) in self.bodies for g in range(model.ngeom)])
        self.n_added = 0

    def set_pose(self, x, y, yaw, pedestal, left_q=None, right_q=None):
        """Puts the copy at a stance with the given arm joints (None = the home pose) and updates its kinematics.

        Args:
            x (float): Base x in metres.
            y (float): Base y in metres.
            yaw (float): Base heading in radians.
            pedestal (float): Pedestal joint value in metres.
            left_q (Sequence[float] or None): The left arm's 7 joint values in radians; None means home.
            right_q (Sequence[float] or None): The right arm's; None means home.
        """
        d = self.d
        d.qpos[0:7] = [x, y, self.base_z, math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
        for a in self.ped:
            d.qpos[a] = pedestal
        for side, q in (("left", left_q), ("right", right_q)):
            for a, v in zip(self.qadr[side], self.home[side] if q is None else q):
                d.qpos[a] = v
        mujoco.mj_kinematics(self.model, d)

    def draw(self, scene, opt, rgba):
        """Appends the ghost to a scene that was just built by mjv_updateScene.

        Args:
            scene (mujoco.MjvScene): The viewer's own scene (never a policy camera's).
            opt (mujoco.MjvOption): The options the scene was built with (which geom groups show).
            rgba (Sequence[float]): The ghost's colour and opacity, each 0 to 1.
        """
        n0 = scene.ngeom
        mujoco.mjv_addGeoms(self.model, self.d, opt, mujoco.MjvPerturb(), mujoco.mjtCatBit.mjCAT_ALL, scene)
        for i in range(n0, scene.ngeom):
            g = scene.geoms[i]
            if g.objtype == int(mujoco.mjtObj.mjOBJ_GEOM) and self.geom_is_robot[g.objid]:
                g.rgba[:] = rgba
                g.texid, g.matid = -1, -1
                g.reflectance, g.emission, g.specular, g.shininess = 0.0, 0.0, 0.0, 0.0
            else:
                g.rgba[:] = (0, 0, 0, 0)
                g.pos[2] = HIDDEN_Z
        self.n_added = scene.ngeom - n0
