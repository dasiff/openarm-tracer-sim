"""Push direction and the lever rule (M293, SF-32). Our own derivation, not the paper's printed formula.

Direction: along the line between the two closest points of the pair (the shared layer's contact: gap, closest
points, push; a deep overlap uses its own rules there). The push of a pair moves each shape along its own direction.
Lever rule (plan M293): along a link the push is shared between the link's two ends by where the contact lies:
a contact 30% from end A gives end A 70% (lever_weights).
What the plan does NOT define: how much the elbow must move when the other end of the link is fixed (the shoulder for
the upper arm, the wrist for the forearm, which an elbow swing never moves). That part is the one function
elbow_displacement(), a PLACEHOLDER that uses the plain direction with no lever weighting (config
RESOLVER["elbow_displacement_mode"]); later rounds make up the rest (SF-32).
"""

from dataclasses import dataclass

import numpy as np

from src.safety_filter import config as cfg
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


def lever_weights(t):
    """The plan's lever rule: a contact a fraction t from end A gives end A the weight 1 - t and end B the weight t
    (30% from end A: A 70%, B 30%). Returns (weight_a, weight_b)."""
    return 1.0 - t, t


def elbow_displacement(push_vector, contact_t, elbow_is_end_b):
    """PLACEHOLDER (SF-32, decision needed): the displacement asked of the elbow for a contact on an upper arm or
    forearm segment that must move by push_vector (m, a vector).

    "plain" mode: the elbow is displaced by the full push vector, with no lever weighting. The contact point
    then moves by (its lever weight) x push_vector, so the first round gets only part of the way; the next
    rounds (up to 10) make up the rest. The lever weights are computed (lever_weights) but not applied: the plan
    does not say how to divide the push when the shoulder or the wrist end cannot move.

    Args: push_vector (3,): the wanted displacement of the contact point. contact_t (float): the contact's
    fraction along the segment from its shoulder-side end to its wrist-side end. elbow_is_end_b (bool): True for the
    upper arm (shoulder to elbow: the elbow is end B), False for the forearm (elbow to wrist: end A).
    """
    if cfg.RESOLVER["elbow_displacement_mode"] != "plain":
        raise ValueError("RESOLVER['elbow_displacement_mode'] must be 'plain' (the only mode built)")
    return np.asarray(push_vector, float)
