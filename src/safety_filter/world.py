"""World sources for the filter (M290): numbers in, objects out. No simulator, no scene file.

A world object is a WorldObject: id, shape ("box" or "capsule"), size (box half sizes, or radius and half
length for a capsule), centre and rotation in the world, and the graspable and round tags from the object
catalog (config.OBJECT_CATALOG).

Sources:
  LabMapSource      the fixed fixtures from data/safety_filter/lab_map.json (bench, walls, cabinets, ...).
  WorldModelAdapter TEMPORARY ground-truth stand-in (M290): reads the objects of a WorldModel object
                    (src/world_model.py) by duck typing; it does not import that module. Until perception
                    feeds the world model, its objects come from SimGroundTruthSource.
  WorldView         the objects of a source plus the held objects (removed from the world while held).
"""

import json
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

from src.safety_filter.config import LAB_MAP_BENCH_OPEN, OBJECT_CATALOG


@dataclass(frozen=True)
class WorldObject:
    id: str
    shape: str                 # "box" or "capsule" (a capsule's axis is the rotation's z column)
    size: tuple                # box: half sizes (3); capsule: (radius, half length of the segment)
    centre: tuple              # world, metres
    rot: tuple                 # world rotation, row-major 3x3 (9 numbers)
    graspable: bool = False
    round: bool = False
    fixture: bool = False

    def rot_matrix(self):
        return np.asarray(self.rot, dtype=float).reshape(3, 3)

    def bound_radius(self):
        """Radius of a sphere around the centre that contains the object (for the match rule)."""
        if self.shape == "capsule":
            return float(self.size[0] + self.size[1])
        return float(np.linalg.norm(self.size))


def _rot9(q_wxyz):
    import mujoco  # a math helper only (quaternion to matrix); no model or data is touched
    out = np.zeros(9)
    mujoco.mju_quat2Mat(out, np.asarray(q_wxyz, dtype=float))
    return out


class LabMapMismatch(RuntimeError):
    """The lab map was built for another bench_open setting than the filter expects (SF-8)."""


class LabMapSource:
    """The fixed fixtures of the lab, from lab_map.json (fingerprinted with the scene file, built for one bench_open setting)."""

    def __init__(self, path, expected_bench_open=None):
        """Raises LabMapMismatch if the map's bench_open differs from expected_bench_open (default config.LAB_MAP_BENCH_OPEN)."""
        if expected_bench_open is None:
            expected_bench_open = LAB_MAP_BENCH_OPEN
        doc = json.loads(Path(path).read_text(encoding="utf-8"))
        if doc.get("bench_open") != expected_bench_open:
            raise LabMapMismatch(f"lab map built for bench_open={doc.get('bench_open')}, filter expects {expected_bench_open}; rebuild with tools/survey_lab_map.py")
        self.scene_sha256 = doc["scene_sha256"]
        self.bench_open = doc["bench_open"]
        self._objects = [WorldObject(id=f["name"], shape="box", size=tuple(f["half"]),
                                     centre=tuple(f["center"]), rot=tuple(_rot9(f["quat"])),
                                     fixture=True)
                         for f in doc["fixtures"]]

    def objects(self):
        return list(self._objects)


def catalog_object(entry_id, position, axis=None):
    """A world object for a catalog type at a bottom-centre position (the way the world model keeps entries)."""
    spec = OBJECT_CATALOG.get(entry_id)
    if spec is None:
        raise KeyError(f"no catalog entry for {entry_id!r}")
    centre = np.asarray(position, float) + np.array([0.0, 0.0, spec["centre_above_bottom_m"]])
    if spec["shape"] == "capsule":
        size = (spec["radius_m"], spec["half_height_m"])
    else:
        size = tuple(spec["half_m"])
    return WorldObject(id=entry_id, shape=spec["shape"], size=tuple(size), centre=tuple(centre),
                       rot=tuple(np.eye(3).ravel()), graspable=spec["graspable"], round=spec["round"])


class WorldModelAdapter:
    """TEMPORARY (M290): objects from a world model that holds ground truth from the simulator.

    Works on any object with entries(kind) and get(id); the entries need .id, .position (bottom centre) and
    .kind. Objects not in the catalog are skipped and listed in .skipped. Replace with perception later."""

    def __init__(self, world_model):
        self.world_model = world_model
        self.skipped = []

    def objects(self):
        out = []
        for e in self.world_model.entries(kind="object"):
            if e.id not in OBJECT_CATALOG:
                self.skipped.append(e.id)
                continue
            out.append(catalog_object(e.id, e.position))
        return out


class WorldView:
    """The world the filter sees: a source's objects, minus the held ones, plus the released ones."""

    def __init__(self, source):
        self.source = source
        self._removed = set()
        self._added = []
        self._extra = {}

    def objects(self):
        base = [o for o in self.source.objects() if o.id not in self._removed]
        return base + [o for o in self._added]

    def remove(self, obj_id):
        self._removed.add(obj_id)
        self._added = [o for o in self._added if o.id != obj_id]

    def add(self, obj: WorldObject):
        """Puts an object back (a release). The source's copy stays hidden: it was removed when it was held."""
        self._added.append(obj)

    def find(self, obj_id):
        for o in self.objects():
            if o.id == obj_id:
                return o
        return None
