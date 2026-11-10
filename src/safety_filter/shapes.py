"""Robot shapes: loads data/safety_filter/robot_shapes.json and places every shape from a joint state.

The forward kinematics is MuJoCo's on the robot-only model (no scene, no world data): the joint state
(base pose, pedestal height, the arm joints and the finger travel) goes into its qpos, mj_kinematics
gives the body frames, and each shape's body-local ends or box are mapped to the world.

Refuses to load when the model's fingerprint differs from the one stamped in the file (SF-5).
"""

import json
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

from src.safety_filter.robot_model import robot_fingerprint

SIDES = ("left", "right")
ARM_JOINTS = 7


class FingerprintMismatch(RuntimeError):
    """The robot model differs from the one the shapes were built from (SF-5)."""


@dataclass(frozen=True)
class RobotState:
    """The robot's joint state as numbers (the only robot input the filter takes).

    base_xyz (m) and base_quat (w, x, y, z): the robot base in the world. pedestal_m (m): the pedestal's
    lift. q_left, q_right (rad): the seven arm joints of each arm. finger_left, finger_right (m): the travel
    of each finger from closed (the same value on both fingers of one hand).
    """
    base_xyz: tuple = (0.0, 0.0, 0.0)
    base_quat: tuple = (1.0, 0.0, 0.0, 0.0)
    pedestal_m: float = 0.0
    q_left: tuple = (0.0,) * ARM_JOINTS
    q_right: tuple = (0.0,) * ARM_JOINTS
    finger_left: float = 0.0
    finger_right: float = 0.0


@dataclass
class PlacedShape:
    """One shape in the world. A capsule has ends a, b and radius r; a box has centre, rot (3x3) and half sizes."""
    id: str
    arm: str           # "left", "right" or "common"
    part: str          # upper_arm, forearm, shoulder_mount, link7, palm, finger, base, pedestal_base, pedestal_top, torso, eagle_cam, battery
    kind: str          # "capsule" or "box"
    body: str          # the body the shape is attached to (its frame)
    a: np.ndarray = None
    b: np.ndarray = None
    r: float = 0.0
    centre: np.ndarray = None
    rot: np.ndarray = None
    half: np.ndarray = None
    geoms: tuple = ()
    segment: str = ""  # the resolver's segment (SF-20): upper_arm (links 1-3: contacts move the elbow only), forearm (links 4-6), hand (link 7, palm, fingers), shoulder, body

    def aabb(self):
        """Axis-aligned box around the shape (lo, hi)."""
        if self.kind == "capsule":
            lo = np.minimum(self.a, self.b) - self.r
            hi = np.maximum(self.a, self.b) + self.r
        else:
            ext = np.abs(self.rot) @ self.half
            lo, hi = self.centre - ext, self.centre + ext
        return lo, hi


def _quat_to_mat(q):
    out = np.zeros(9)
    mujoco.mju_quat2Mat(out, np.asarray(q, dtype=float))
    return out.reshape(3, 3)


class RobotShapes:
    """The shapes of one robot model. Built from the JSON file and the robot-only MjModel."""

    def __init__(self, model, shapes_path):
        """Args: model (mujoco.MjModel): the robot-only model (src/safety_filter/robot_model.load_robot_only_model);
        shapes_path (str or Path): data/safety_filter/robot_shapes.json.

        Raises: FingerprintMismatch if the model's fingerprint is not the file's stamp.
        """
        self.model = model
        self.data = mujoco.MjData(model)
        doc = json.loads(Path(shapes_path).read_text(encoding="utf-8"))
        fp = robot_fingerprint(model)
        if doc.get("fingerprint") != fp:
            raise FingerprintMismatch(
                f"robot model fingerprint {fp[:12]} differs from the shapes file's {str(doc.get('fingerprint'))[:12]}; "
                "rebuild with tools/build_filter_shapes.py")
        self.doc = doc
        self.specs = doc["shapes"]
        self._body = {s["id"]: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, s["frame"]) for s in self.specs}
        self._body_ends = {}
        for s in self.specs:
            if s["kind"] == "capsule":
                self._body_ends[s["id"]] = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
                                            for n in s["ends_body"]]
        self._joint, self._finger = {}, {}
        for side in SIDES:
            self._joint[side] = [self._jid(f"openarm_{side}_joint{k}") for k in range(1, ARM_JOINTS + 1)]
            self._finger[side] = [self._jid(f"openarm_{side}_finger_joint{k}") for k in (1, 2)]
        self._base = self._jid("base_float")
        self._pedestal = self._jid("pedestal_joint")
        self._prepare_arrays()
        self.largest_radius = max((s["radius"] for s in self.specs if s["kind"] == "capsule"), default=0.0)

    def _jid(self, name):
        j = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if j < 0:
            raise ValueError(f"robot model has no joint {name}")
        return j

    def set_state(self, state: RobotState):
        """Puts the joint state into the data and runs the kinematics (no dynamics, no stepping)."""
        m, d = self.model, self.data
        d.qpos[:] = m.qpos0
        a = m.jnt_qposadr[self._base]
        d.qpos[a:a + 3] = state.base_xyz
        d.qpos[a + 3:a + 7] = state.base_quat
        d.qpos[m.jnt_qposadr[self._pedestal]] = state.pedestal_m
        for side, q, f in (("left", state.q_left, state.finger_left), ("right", state.q_right, state.finger_right)):
            q = np.asarray(q, dtype=float)
            if q.shape != (ARM_JOINTS,) or not np.isfinite(q).all():
                raise ValueError(f"{side}: expected {ARM_JOINTS} finite joint angles")
            for j, v in zip(self._joint[side], q):
                d.qpos[m.jnt_qposadr[j]] = v
            for j in self._finger[side]:
                d.qpos[m.jnt_qposadr[j]] = f
        mujoco.mj_kinematics(m, d)

    def _prepare_arrays(self):
        """Index arrays so that place() is one vectorised pass (built once)."""
        cap_idx, box_idx = [], []
        for k, sp in enumerate(self.specs):
            (cap_idx if sp["kind"] == "capsule" else box_idx).append(k)
        self._cap_idx, self._box_idx = np.array(cap_idx, int), np.array(box_idx, int)
        mb = lambda n: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, n)
        if len(cap_idx):
            self._cap_ba = np.array([mb(self.specs[k]["ends_body"][0]) for k in cap_idx])
            self._cap_bb = np.array([mb(self.specs[k]["ends_body"][1]) for k in cap_idx])
            self._cap_la = np.array([self.specs[k]["ends_local"][0] for k in cap_idx], float)
            self._cap_lb = np.array([self.specs[k]["ends_local"][1] for k in cap_idx], float)
            self._cap_r = np.array([self.specs[k]["radius"] for k in cap_idx], float)
            # a capsule may cover only a fraction range [t0, t1] of the segment between its two body-local ends (SF-14)
            self._cap_t0 = np.array([self.specs[k].get("t_range", [0.0, 1.0])[0] for k in cap_idx], float)
            self._cap_t1 = np.array([self.specs[k].get("t_range", [0.0, 1.0])[1] for k in cap_idx], float)
        if len(box_idx):
            self._box_b = np.array([mb(self.specs[k]["frame"]) for k in box_idx])
            self._box_c = np.array([self.specs[k]["center_local"] for k in box_idx], float)
            self._box_rl = np.array([_quat_to_mat(self.specs[k]["quat_local"]) for k in box_idx])
            self._box_h = np.array([self.specs[k]["half"] for k in box_idx], float)

    def place(self, state: RobotState):
        """All shapes in the world for this joint state (one vectorised pass). Returns a list of PlacedShape in spec order."""
        self.set_state(state)
        d = self.data
        xpos = d.xpos
        xmat = d.xmat.reshape(-1, 3, 3)
        out = [None] * len(self.specs)
        if len(self._cap_idx):
            A0 = xpos[self._cap_ba] + np.einsum("kij,kj->ki", xmat[self._cap_ba], self._cap_la)
            B0 = xpos[self._cap_bb] + np.einsum("kij,kj->ki", xmat[self._cap_bb], self._cap_lb)
            A = A0 + self._cap_t0[:, None] * (B0 - A0)
            B = A0 + self._cap_t1[:, None] * (B0 - A0)
            for n, k in enumerate(self._cap_idx):
                sp = self.specs[k]
                out[k] = PlacedShape(id=sp["id"], arm=sp["arm"], part=sp["part"], kind="capsule", body=sp["frame"],
                                     a=A[n], b=B[n], r=float(self._cap_r[n]), geoms=tuple(sp["geoms"]),
                                     segment=sp.get("segment", ""))
        if len(self._box_idx):
            C = xpos[self._box_b] + np.einsum("kij,kj->ki", xmat[self._box_b], self._box_c)
            Rw = np.einsum("kij,kjl->kil", xmat[self._box_b], self._box_rl)
            for n, k in enumerate(self._box_idx):
                sp = self.specs[k]
                out[k] = PlacedShape(id=sp["id"], arm=sp["arm"], part=sp["part"], kind="box", body=sp["frame"],
                                     centre=C[n], rot=Rw[n], half=self._box_h[n], geoms=tuple(sp["geoms"]),
                                     segment=sp.get("segment", ""))
        return out

    def hand_frame(self, side):
        """Pose (position, rotation) of link 7 of one arm in the world, for the held-object offsets."""
        b = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f"openarm_{side}_link7")
        return self.data.xpos[b].copy(), self.data.xmat[b].reshape(3, 3).copy()
