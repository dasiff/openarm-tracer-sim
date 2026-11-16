"""Robot shapes: loads data/safety_filter/robot_shapes.json and places every shape from a joint state.

The forward kinematics is MuJoCo's on the robot-only model (no scene, no world data): the joint state
(base pose, pedestal height, the arm joints and the finger travel) goes into its qpos, mj_kinematics
gives the body frames, and each shape's body-local ends or box are mapped to the world.

Refuses to load when the model's fingerprint differs from the one stamped in the file (SF-5).
"""

import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

from src.safety_filter import config as cfg
from src.safety_filter.geometry import capsule_capsule_batch
from src.safety_filter.robot_model import robot_fingerprint

SIDES = ("left", "right")
ARM_JOINTS = 7


_NEVER_CLOSE = frozenset()   # pairs of shape ids that can never come close (SF-23); set by the last RobotShapes built


def never_close_pairs():
    """The never-close pairs of the robot shapes last loaded: a frozenset of frozenset({id_a, id_b}) (SF-23)."""
    return _NEVER_CLOSE


def torso_settings():
    """The torso guard settings the labels of a merged never-close list depend on (config NEVER_CLOSE_DC; DNC-1, DNC-2)."""
    return {"margin_m": cfg.NEVER_CLOSE_DC["torso_proximity_margin_m"],
            "exclude_links": [int(k) for k in cfg.NEVER_CLOSE_DC["torso_guard_exclude_links"]]}


def torso_guard_counts(shape_entry):
    """Does this shape (an entry of robot_shapes.json) count for torso proximity (DNC-2)? Arm shapes only, without the parts that touch
    the torso by construction (TORSO_EXEMPT_PARTS) and without the links in NEVER_CLOSE_DC["torso_guard_exclude_links"]."""
    excluded = {f"link{int(k)}" for k in cfg.NEVER_CLOSE_DC["torso_guard_exclude_links"]}
    return shape_entry["arm"] in SIDES and shape_entry["part"] not in cfg.TORSO_EXEMPT_PARTS and shape_entry["part"] not in excluded


def never_close_fingerprint(model_fp, pairs, torso):
    """The stamp of a merged never-close list (block never_close_pairs of robot_shapes.json, DNC-5): SHA-256 over the robot model's
    fingerprint, the merged entries (pair, source, torso_dependent: so it covers what was taken from never_close_dc.json as well as
    Stage 1) and the torso settings the labels were made with. The build tool writes it; the loader recomputes it and refuses a block
    that does not match (edited by hand, or built for another model)."""
    rows = sorted((min(p["a"], p["b"]), max(p["a"], p["b"]), p["source"], bool(p["torso_dependent"])) for p in pairs)
    h = hashlib.sha256()
    h.update(json.dumps([model_fp, rows, torso], sort_keys=True).encode())
    return h.hexdigest()


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
    never_touch: bool = False  # SF-65: set on a world shape made from a never-touch object
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

    def __init__(self, model, shapes_path, use_never_close=True):
        """Args: model (mujoco.MjModel): the robot-only model (src/safety_filter/robot_model.load_robot_only_model);
        shapes_path (str or Path): data/safety_filter/robot_shapes.json.
        use_never_close (bool): load the never-close pair list of the file (SF-23); False checks every pair (comparison runs).

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
        global _NEVER_CLOSE
        nc = doc.get("never_close", {})
        merged = doc.get("never_close_pairs")
        if use_never_close and nc and nc.get("fingerprint") != fp:
            raise FingerprintMismatch("the never-close list was built for another robot model")
        self.torso_dependent = frozenset()                  # never-close pairs that are not trusted while an arm shape is near the torso (DNC-1)
        if use_never_close and merged is not None:
            # the merged list (Stage 1 sweep + Stage 2 divide and conquer, tools/build_never_close.py, DNC-4): the one the filter uses
            if merged.get("model_fingerprint") != fp:
                raise FingerprintMismatch("the merged never-close list was built for another robot model")
            if merged.get("fingerprint") != never_close_fingerprint(fp, merged["pairs"], merged.get("torso")):
                raise FingerprintMismatch("the merged never-close list does not match its stamp; rebuild with tools/build_never_close.py")
            if merged.get("torso") != torso_settings():
                raise FingerprintMismatch("the merged never-close list was labelled with other torso settings; label and rebuild")
            self.never_close = frozenset(frozenset((p["a"], p["b"])) for p in merged["pairs"])
            self.torso_dependent = frozenset(frozenset((p["a"], p["b"])) for p in merged["pairs"] if p["torso_dependent"])
        elif use_never_close:
            self.never_close = frozenset(frozenset((p["a"], p["b"])) for p in nc.get("pairs", []))
        else:
            self.never_close = frozenset()
        self.never_close_torso_free = self.never_close - self.torso_dependent   # the list while the torso guard is on
        _NEVER_CLOSE = self.never_close
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
        self._adr_base, self._adr_ped = int(model.jnt_qposadr[self._base]), int(model.jnt_qposadr[self._pedestal])
        self._adr_arm = {s: np.array([model.jnt_qposadr[j] for j in self._joint[s]]) for s in SIDES}
        self._adr_fing = {s: np.array([model.jnt_qposadr[j] for j in self._finger[s]]) for s in SIDES}
        self._qbuf, self._kin_q, self._kin_done = np.array(model.qpos0, float), np.array(model.qpos0, float), False
        self.largest_radius = max((s["radius"] for s in self.specs if s["kind"] == "capsule"), default=0.0)
        self._prepare_torso_guard()

    def _prepare_torso_guard(self):
        """Index arrays of the torso guard (DNC-1, DNC-2): every (arm shape that counts, torso shape) pair, as positions among the capsules."""
        caps = [self.specs[k] for k in self._cap_idx]
        torso = [n for n, sp in enumerate(caps) if sp["part"] == "torso"]
        if any(sp["part"] == "torso" and sp["kind"] != "capsule" for sp in self.specs):
            raise NotImplementedError("the torso guard measures capsule against capsule; a torso box is not supported")
        I, J, side = [], [], []
        for n, sp in enumerate(caps):
            if not torso_guard_counts(sp):
                continue
            for t in torso:
                I.append(n)
                J.append(t)
                side.append(sp["arm"])
        self._tg_i, self._tg_j, self._tg_side = np.array(I, int), np.array(J, int), np.array(side, dtype=object)
        self.torso_guard_ids = [caps[n]["id"] for n in sorted(set(I))]

    def torso_gap(self, sides=None):
        """The smallest gap (m) between an arm shape of the guard set and a torso shape, at the state last set (set_state or place).
        The filter's own capsule distance. sides: only the shapes of these arms (default both; the filter always uses both).
        +inf when no shape counts."""
        i, j = self._tg_i, self._tg_j
        if sides is not None:
            keep = np.isin(self._tg_side, list(sides))
            i, j = i[keep], j[keep]
        if not len(i):
            return float("inf")
        A, B, _, _ = self.place_arrays()
        return float(capsule_capsule_batch(A[i], B[i], self._cap_r[i], A[j], B[j], self._cap_r[j], gap_only=True).min())

    def set_torso_proximate(self, proximate):
        """Torso guard (DNC-1): while an arm shape is within the torso proximity margin, the torso-dependent never-close pairs are not
        trusted: the touch rules then see the list without them, so those pairs are checked like any other pair."""
        global _NEVER_CLOSE
        _NEVER_CLOSE = self.never_close_torso_free if proximate else self.never_close

    def _jid(self, name):
        j = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if j < 0:
            raise ValueError(f"robot model has no joint {name}")
        return j

    def set_state(self, state: RobotState):
        """Puts the joint state into the data and runs the kinematics (no dynamics, no stepping). When the joint state is the
        one the kinematics was last run for, nothing is recomputed (SF-70)."""
        m, d = self.model, self.data
        q = self._qbuf
        q[:] = m.qpos0
        q[self._adr_base:self._adr_base + 3] = state.base_xyz
        q[self._adr_base + 3:self._adr_base + 7] = state.base_quat
        q[self._adr_ped] = state.pedestal_m
        for side, qa, f in (("left", state.q_left, state.finger_left), ("right", state.q_right, state.finger_right)):
            qa = np.asarray(qa, dtype=float)
            if qa.shape != (ARM_JOINTS,) or not np.isfinite(qa).all():
                raise ValueError(f"{side}: expected {ARM_JOINTS} finite joint angles")
            q[self._adr_arm[side]] = qa
            q[self._adr_fing[side]] = f
        if self._kin_done and np.array_equal(q, self._kin_q) and np.array_equal(d.qpos, q):
            return
        d.qpos[:] = q
        mujoco.mj_kinematics(m, d)
        self._kin_q[:] = q
        self._kin_done = True

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

    def place_arrays(self):
        """The shapes of the state last set, as arrays: capsule ends A, B (in the order of the capsule shapes) and box centres C
        and rotations R (in the order of the box shapes). The numbers place() puts into its PlacedShape objects."""
        d = self.data
        xpos = d.xpos
        xmat = d.xmat.reshape(-1, 3, 3)
        A = B = C = Rw = None
        if len(self._cap_idx):
            A0 = xpos[self._cap_ba] + np.einsum("kij,kj->ki", xmat[self._cap_ba], self._cap_la)
            B0 = xpos[self._cap_bb] + np.einsum("kij,kj->ki", xmat[self._cap_bb], self._cap_lb)
            A = A0 + self._cap_t0[:, None] * (B0 - A0)
            B = A0 + self._cap_t1[:, None] * (B0 - A0)
        if len(self._box_idx):
            C = xpos[self._box_b] + np.einsum("kij,kj->ki", xmat[self._box_b], self._box_c)
            Rw = np.einsum("kij,kjl->kil", xmat[self._box_b], self._box_rl)
        return A, B, C, Rw

    def place(self, state: RobotState):
        """All shapes in the world for this joint state (one vectorised pass). Returns a list of PlacedShape in spec order."""
        self.set_state(state)
        A, B, C, Rw = self.place_arrays()
        out = [None] * len(self.specs)
        if len(self._cap_idx):
            for n, k in enumerate(self._cap_idx):
                sp = self.specs[k]
                out[k] = PlacedShape(id=sp["id"], arm=sp["arm"], part=sp["part"], kind="capsule", body=sp["frame"],
                                     a=A[n], b=B[n], r=float(self._cap_r[n]), geoms=tuple(sp["geoms"]),
                                     segment=sp.get("segment", ""))
        if len(self._box_idx):
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
