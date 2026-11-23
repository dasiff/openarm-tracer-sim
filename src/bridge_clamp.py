"""Joint-buffer clamp of the command bridge (SF-122, stage 1 of the staged rebuild, David 2026-11-23).

Every joint target that reaches the bridge is clamped into a buffer: [hard lower limit + margin, hard upper limit - margin], where the margin
is the 10-degree joint margin the arm solver uses today (MINK_CFG joint_margin_rad, the same config value). A target outside a joint's hard
range is clamped the same way, instead of being rejected. Pure functions, no simulator: the bridge (src/control_loop.py) calls them.
"""

import numpy as np

from src.task_config import MINK_CFG


def buffer_limits(hard_lo, hard_hi, margin=None):
    """The buffer edges of one arm: (lo + margin, hi - margin), arrays of 7 radians. margin defaults to MINK_CFG joint_margin_rad (0.1745 rad)."""
    m = float(MINK_CFG["joint_margin_rad"]) if margin is None else float(margin)
    return np.asarray(hard_lo, float) + m, np.asarray(hard_hi, float) - m


def clamp_to_buffer(q, lo_buf, hi_buf):
    """Clamps the 7 joint targets q into [lo_buf, hi_buf] per joint. Returns (clamped (7,), clamped_mask (7,) bool: True where the value changed).
    A value inside its buffer passes through exactly (the same float)."""
    q = np.asarray(q, float)
    clamped = np.clip(q, lo_buf, hi_buf)
    return clamped, clamped != q
