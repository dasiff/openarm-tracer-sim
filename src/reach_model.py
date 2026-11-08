"""A small Gaussian model of the reach maps (left arm, filtered maps): how many of the 12 approach bins reach a grasp point.

Standalone: not used by the pipeline or the stance selection.

The only inputs are the 3D offset from the shoulder to the grasp point, in the shoulder frame of the arm (x forward along the heading, y left, z up):
    d     horizontal distance (m)
    theta bearing relative to the heading (rad, turning left; atan2(y, x))
    z     vertical offset (m, negative = below the shoulder)
A heading change rotates the offset, which shifts theta and leaves d and z alone, so the model is heading-independent once the caller turns the world offset into the shoulder frame.

    count(d, theta, z) = C(z) * exp(-((d - d0(z)) / sd(z))^2) * exp(-((theta - th0) / st(z))^2),   clipped to [0, 12]
    C = a0 + a1 z,  d0 = b0 + b1 z,  sd = c0 + c1 z,  st = e0 + e1 z      (th0 = 0 in the specified form; `th0` is the one added parameter of the alternative)
"""
import numpy as np

N_BINS = 12
PARAM_NAMES = ("a0", "a1", "b0", "b1", "c0", "c1", "e0", "e1")
MIN_WIDTH = 1e-3                                   # a width never goes below this (m or rad), so the formula stays defined


def predict_count(d, theta, z, p, th0=0.0):
    """Predicted number of approach bins that reach the offset (d, theta, z).

    Args:
        d (array-like): Horizontal distance from the shoulder, metres.
        theta (array-like): Bearing relative to the heading, radians, in (-pi, pi].
        z (array-like): Vertical offset, metres.
        p (sequence of float): The 8 parameters a0, a1, b0, b1, c0, c1, e0, e1.
        th0 (float): Angular centre, radians. Defaults to 0 (the specified model).

    Returns:
        numpy.ndarray: The count, in [0, N_BINS].
    """
    d, theta, z = np.broadcast_arrays(np.asarray(d, float), np.asarray(theta, float), np.asarray(z, float))
    a0, a1, b0, b1, c0, c1, e0, e1 = p
    C = a0 + a1 * z
    d0 = b0 + b1 * z
    sd = np.maximum(c0 + c1 * z, MIN_WIDTH)
    st = np.maximum(e0 + e1 * z, MIN_WIDTH)
    dth = (theta - th0 + np.pi) % (2 * np.pi) - np.pi            # wrapped difference
    return np.clip(C * np.exp(-((d - d0) / sd) ** 2) * np.exp(-(dth / st) ** 2), 0.0, N_BINS)
