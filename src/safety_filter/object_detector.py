"""Object identification for a held object (Phase S, S4, SF-110).

When a hand holds something that neither the task plan's name nor the pad match explains, the held-object tracker (src/safety_filter/touch.py,
HeldTracker) asks an ObjectDetector what is between the finger pads. The filter calls only the interface (identify); the implementation is
chosen in config (OBJECT_DETECTOR["implementation"]) through make_object_detector. The sim class is never named outside this module.

Implementations:
  SimObjectDetector  the sim stand-in: the ground-truth object nearest the pads, if its shape is within OBJECT_DETECTOR["sim_radius_m"] of the
                     pad space. Fixtures (the hotplate, the bench) are left out. It stands in for perception, which is not built yet.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import numpy as np

from src.safety_filter import config as cfg


@dataclass(frozen=True)
class FingerState:
    """The finger pads of one hand this tick: the centres of the two finger capsules (pad_a, pad_b, metres) and their radius (m)."""
    pad_a: np.ndarray
    pad_b: np.ndarray
    radius: float

    @property
    def midpoint(self):
        return (np.asarray(self.pad_a, float) + np.asarray(self.pad_b, float)) / 2

    @property
    def gap(self):
        """The finger gap: the distance between the two pad centres (m)."""
        return float(np.linalg.norm(np.asarray(self.pad_a, float) - np.asarray(self.pad_b, float)))


class ObjectDetector(ABC):
    """The interface the filter calls. identify(arm, finger_state, world) returns the world name of the object at the finger pads, or None
    when nothing is there. arm: "left" or "right". world: an object with objects() (the filter's WorldView)."""

    @abstractmethod
    def identify(self, arm: str, finger_state: FingerState, world) -> Optional[str]:
        """The name of the object between the finger pads, or None."""


def _frame_z(z):
    """A rotation matrix whose third column is the unit vector z."""
    z = np.asarray(z, float)
    n = np.linalg.norm(z)
    z = z / n if n > 1e-12 else np.array([0.0, 0.0, 1.0])
    helper = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = np.cross(helper, z)
    x = x / np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def pad_space(arm: str, finger_state: FingerState):
    """The space between the finger pads as a placed capsule: the segment between the two pad centres, with the finger radius."""
    from src.safety_filter.shapes import PlacedShape
    a = np.asarray(finger_state.pad_a, float)
    b = np.asarray(finger_state.pad_b, float)
    return PlacedShape(id=f"{arm}_pad_space", arm=arm, part="finger", kind="capsule", body=f"{arm}_pads", a=a, b=b,
                       r=float(finger_state.radius), centre=(a + b) / 2, rot=_frame_z(b - a))


class SimObjectDetector(ObjectDetector):
    """Sim stand-in (S4): the ground-truth object nearest the pads. Fixtures are not candidates. Returns None when the nearest object's shape
    is more than OBJECT_DETECTOR["sim_radius_m"] from the pad space (nothing is there)."""

    def __init__(self, radius_m=None):
        self.radius_m = float(cfg.OBJECT_DETECTOR["sim_radius_m"]) if radius_m is None else float(radius_m)

    def identify(self, arm, finger_state, world):
        from src.safety_filter.touch import as_placed, gap_between       # lazy: touch.py imports this module
        pads = pad_space(arm, finger_state)
        best_gap, best_name = None, None
        for o in world.objects():
            if o.fixture:
                continue
            g = float(gap_between(pads, as_placed(o))[0])
            if best_gap is None or g < best_gap:
                best_gap, best_name = g, o.id
        if best_name is None or best_gap > self.radius_m:
            return None
        return best_name


DETECTORS = {"sim": SimObjectDetector}     # implementation name (config OBJECT_DETECTOR["implementation"]) -> class; the only place a class is named


def make_object_detector(name=None) -> ObjectDetector:
    """The detector the config selects (or the named one). Raises ValueError for an unknown name."""
    name = cfg.OBJECT_DETECTOR["implementation"] if name is None else name
    if name not in DETECTORS:
        raise ValueError(f"unknown object detector {name!r}; known: {sorted(DETECTORS)}")
    return DETECTORS[name]()
