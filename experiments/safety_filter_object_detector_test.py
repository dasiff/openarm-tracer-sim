"""Safety filter Phase S, S4 (SF-110): the ObjectDetector interface and the sim stand-in, against a known ground-truth scene. No physics.

Checks: the sim stand-in names the beaker whose shape is at the pads, returns None when the nearest object is outside the radius, ignores
fixtures (the hotplate), names the nearest of two objects, the factory returns the configured implementation and rejects unknown names, and
the interface cannot be instantiated and accepts a stub implementation.

    python experiments/safety_filter_object_detector_test.py      (exit code 0 when every check passes)
"""

import dataclasses
import os
import sys

import numpy as np

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, REPO)

from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.object_detector import (DETECTORS, FingerState, ObjectDetector, SimObjectDetector, make_object_detector)  # noqa: E402
from src.safety_filter.world import WorldView, catalog_object  # noqa: E402

PAD_R = 0.005          # m: the finger capsule radius used for the pads in this test
FAILS = []


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    if not ok:
        FAILS.append(name)


class Src:
    """A world source with a fixed object list (the ground-truth scene)."""

    def __init__(self, objs):
        self._objs = list(objs)

    def objects(self):
        return list(self._objs)


def pads_at(x, z):
    """Finger pads along y, at world x and height z (the beaker's axis is the world z axis at x = 0.5)."""
    return FingerState(pad_a=np.array([x, -0.025, z]), pad_b=np.array([x, 0.025, z]), radius=PAD_R)


def main():
    beaker = catalog_object("beaker", (0.5, 0.0, 0.9))            # cylinder, radius 3.5 cm, on the axis x = 0.5, y = 0
    z = float(beaker.centre[2])
    det = SimObjectDetector()
    print(f"sim stand-in radius {det.radius_m} m (config OBJECT_DETECTOR sim_radius_m)")

    # 1. pads 1 cm off the beaker's surface: the beaker is named
    world = WorldView(Src([beaker]))
    got = det.identify("left", pads_at(0.55, z), world)
    check("pads 1 cm off the beaker surface name the beaker", got == "beaker", f"got {got!r}")

    # 2. pads 4 cm off the surface: nothing is there
    got = det.identify("left", pads_at(0.58, z), world)
    check("pads 4 cm off the beaker surface: None (outside the radius)", got is None, f"got {got!r}")

    # 3. a fixture at the pads is not a candidate
    hotplate = catalog_object("hotplate", (0.55, 0.0, 0.9))
    hotplate = dataclasses.replace(hotplate, fixture=True)
    got = det.identify("left", pads_at(0.55, float(hotplate.centre[2])), WorldView(Src([hotplate])))
    check("a fixture (the hotplate) at the pads is not named: None", got is None, f"got {got!r}")

    # 4. two beakers: the nearest to the pads is named
    near = dataclasses.replace(beaker, id="beaker_near")
    far = dataclasses.replace(catalog_object("beaker", (0.72, 0.0, 0.9)), id="beaker_far")
    got = det.identify("left", pads_at(0.55, z), WorldView(Src([far, near])))
    check("two beakers: the one nearest the pads is named (order does not matter)", got == "beaker_near", f"got {got!r}")

    # 5. the factory returns the configured implementation, and it is the interface
    d = make_object_detector()
    check("factory with the config value returns the sim stand-in", isinstance(d, SimObjectDetector) and cfg.OBJECT_DETECTOR["implementation"] == "sim",
          f"config {cfg.OBJECT_DETECTOR['implementation']!r}, class {type(d).__name__}")
    check("factory: an unknown implementation name raises ValueError", _raises(ValueError, lambda: make_object_detector("no_such_detector")))
    check("the registry names the sim class under 'sim' only", set(DETECTORS) == {"sim"} and DETECTORS["sim"] is SimObjectDetector)

    # 6. the interface: abstract, and a stub implementation is accepted in its place
    check("ObjectDetector cannot be instantiated (interface only)", _raises(TypeError, lambda: ObjectDetector()))

    class Stub(ObjectDetector):
        def identify(self, arm, finger_state, world):
            return None
    stub = Stub()
    check("a stub implementation of the interface returns None through the same call", stub.identify("left", pads_at(0.55, z), world) is None and isinstance(stub, ObjectDetector))

    print("FAILED: " + ", ".join(FAILS) if FAILS else "ALL PASS")
    return 0 if not FAILS else 1


def _raises(exc, fn):
    try:
        fn()
    except exc:
        return True
    return False


if __name__ == "__main__":
    sys.exit(main())
