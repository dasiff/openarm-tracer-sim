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


def predict_count_asym(r, theta, z, p, z_cut=None):
    """Gaussian in the 3D distance r and in the bearing, with a different angular width on each side of the heading, zero below z_cut.

    count = C exp(-((r - r0) / sr)^2) exp(-(theta / sigma(theta))^2); the peak is at r = r0, theta = 0 whatever the widths.
    sigma(theta) = s_neg if theta < 0 else s_pos, each linear in z (s = s0 + s1 z).

    Bearing sign: theta = atan2(y, x) in the shoulder frame (x forward, y left), so theta > 0 is to the robot's LEFT and theta < 0 to its right.

    Args:
        r (array-like): 3D distance from the shoulder, metres.
        theta (array-like): Bearing relative to the heading, radians, in (-pi, pi].
        z (array-like): Vertical offset, metres.
        p (sequence of float): (C, r0, sr, s_neg0, s_neg1, s_pos0, s_pos1): peak count (bins), radial centre (m), radial width (m), angular width for theta < 0 at z = 0 (rad) and its change per metre
            of z (rad/m), the same for theta >= 0.
        z_cut (float or None): Below this vertical offset the count is 0.

    Returns:
        numpy.ndarray: The count, in [0, N_BINS].
    """
    r, theta, z = np.broadcast_arrays(np.asarray(r, float), np.asarray(theta, float), np.asarray(z, float))
    C, r0, sr, sn0, sn1, sp0, sp1 = p
    sig = np.maximum(np.where(theta < 0, sn0 + sn1 * z, sp0 + sp1 * z), MIN_WIDTH)
    out = np.clip(C * np.exp(-((r - r0) / max(sr, MIN_WIDTH)) ** 2) * np.exp(-(theta / sig) ** 2), 0.0, N_BINS)
    return out if z_cut is None else np.where(z >= z_cut, out, 0.0)
