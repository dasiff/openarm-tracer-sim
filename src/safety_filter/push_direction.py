"""Push direction (M293, SF-32). Our own derivation, not the paper's printed formula.

Direction: along the line between the two closest points of the pair (the shared layer's contact: gap, closest
points, push; a deep overlap uses its own rules there). The push of a pair moves each shape along its own direction.
Where a contact lies along a link (contact_fraction) is what kong_style uses to share a push between the link's ends.
The layered resolver's lever rule and its elbow displacement placeholder (SF-32) were removed with that resolver (SF-108).
"""

from dataclasses import dataclass

import numpy as np

from src.safety_filter.touch import gap_between


@dataclass(frozen=True)
class PairPush:
    """The push of one pair. gap (m). For each side of the pair (a, b): the closest point and the unit direction
    in which that side has to move so that the gap grows."""
    gap: float
    point_a: np.ndarray
    point_b: np.ndarray
    dir_a: np.ndarray
    dir_b: np.ndarray


def pair_push(shape_a, shape_b):
    """The push of a pair of placed shapes: closest points and push directions (gap_between, shared layer)."""
    gap, pa, pb, push = gap_between(shape_a, shape_b)
    push = np.asarray(push, float)
    return PairPush(gap=float(gap), point_a=np.asarray(pa, float), point_b=np.asarray(pb, float), dir_a=push, dir_b=-push)


def contact_fraction(point, end_a, end_b):
    """Where a contact lies along a link: 0 at end A, 1 at end B (the projection of the point on the segment, clamped)."""
    d = np.asarray(end_b, float) - np.asarray(end_a, float)
    n = float(d @ d)
    if n < 1e-18:
        return 0.0
    return float(np.clip((np.asarray(point, float) - np.asarray(end_a, float)) @ d / n, 0.0, 1.0))
