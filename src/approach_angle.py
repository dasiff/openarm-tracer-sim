"""Approach angles of the tasks: the per-task lists the stance check records, the one choosing function, and the reason when a list is missing (David, 2026-11-08).

A stance is base x, y, heading and pedestal height; it carries no single approach angle. Each task (each pick step, each place step) has its own list of the
approach angles that pass its whole path at the stance (nominal and error stances). A stance passes only if every task has at least one passing angle
(combine_task_lists). The lists are passed to execution, which TRUSTS them: no re-check at run time.

Units: a pick's approach angle is its bin index k (0 to 11; the direction is heading + 30 deg x k, task_points.approach_unit); a place's approach angle is
the hand turn theta in degrees (counter-clockwise seen from above, the place's own angle, M183/M184). Every list has the same form: a list of those.

The choice from a list is ONE function, choose_approach_angle: "the first passing angle" for now. David's ranking would replace only that function.
"""

import math

import numpy as np

KIND_PICK, KIND_PLACE = "pick", "place"
REASON_MISSING, REASON_EMPTY = "missing", "empty"


class ApproachAngleError(ValueError):
    """A task has no usable approach-angle list. `reason` is "missing" (no list at all) or "empty" (a list with no passing angle)."""

    def __init__(self, reason, message):
        super().__init__(message)
        self.reason = reason


def task_approach_angles(segment):
    """The approach-angle list of one manipulation segment (a plain list, as the stance selection wrote it).

    Raises:
        ApproachAngleError: reason "missing" when the segment carries no list (the key is absent or None), "empty" when the list has no passing angle.
            Nothing falls back to the forward direction: the skill does not run.
    """
    name = segment.get("description") or segment.get("primitive", "task")
    angles = segment.get("approach_angles")
    if angles is None:
        raise ApproachAngleError(REASON_MISSING, f"{name} has no approach-angle list: the stance check gave this task no list (the skill does not run; no forward fallback)")
    if len(angles) == 0:
        raise ApproachAngleError(REASON_EMPTY, f"{name} has an empty approach-angle list: no approach angle passed the task's whole path at this stance (the skill does not run; no forward fallback)")
    return list(angles)


def choose_approach_angle(angles):
    """ONE choice from a task's list: the first passing angle. The only place where a list becomes one angle (replace this function for a ranking).

    Raises:
        ApproachAngleError: if the list is empty (callers check first; this keeps the choice honest).
    """
    if len(angles) == 0:
        raise ApproachAngleError(REASON_EMPTY, "no approach angle to choose from: the list is empty")
    return angles[0]


def choose_place_angle(angles):
    """ONE choice from a place task's list: the middle turn of the chosen window (David 2026-11-10, M299).

    The list is the window's turns from its first to its last (circular: a window across 0 deg is listed through 0, e.g. -15, -10, -5, 0, 5), tested every 5 deg.
    With an odd count the exact middle turn; with an even count the two middle turns are equally near the window's midpoint, and the later one (counter-clockwise
    of the midpoint) is taken. The robot uses exactly this angle; the full check chose it from the same window (choose_place_window, theta_deg).

    Raises:
        ApproachAngleError: if the list is empty.
    """
    if len(angles) == 0:
        raise ApproachAngleError(REASON_EMPTY, "no place angle to choose from: the window list is empty")
    return angles[len(angles) // 2]


def combine_task_lists(task_lists):
    """A stance's verdict from its tasks' lists: the stance passes only if every task has at least one passing angle.

    Args:
        task_lists (list of dict): {"task": id, "kind": "pick" or "place", "angles": list}.

    Returns:
        (bool, list): ok, and the tasks whose list is empty (empty when ok). Every task keeps its own list in the result of the check.
    """
    failing = [t["task"] for t in task_lists if len(t["angles"]) == 0]
    return (len(failing) == 0), failing


def place_approach_xy(forward_xyz, theta_rad):
    """The place's approach direction: the robot's forward turned by the place's hand turn theta about the vertical axis (the same turn plan_place gives its
    OVER, DOWN and back-off; used by plan_place and by recovery, so both back out along the same direction).

    Args:
        forward_xyz (array-like): the robot's forward unit vector (3).
        theta_rad (float): the place's approach angle (radians, counter-clockwise seen from above).

    Returns:
        numpy.ndarray: the turned direction (3).
    """
    co, si = math.cos(theta_rad), math.sin(theta_rad)
    Rz = np.array([[co, -si, 0.0], [si, co, 0.0], [0.0, 0.0, 1.0]])
    return Rz @ np.asarray(forward_xyz, float)
