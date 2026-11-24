"""Joint-buffer clamp of the command bridge (SF-122 stage 1, David 2026-11-23; SF-125 range-based buffer, David 2026-11-24).

Every joint target that reaches the bridge is clamped into a buffer: [hard lower limit + margin_j, hard upper limit - margin_j]. The margin of
each joint is set by BRIDGE_BUFFER_CFG: mode "range" = range_fraction of the joint's own range, capped at cap_deg (SF-125); mode "fixed" = the
10-degree joint margin of the arm solver (MINK_CFG joint_margin_rad, SF-122 stage 1). A target outside a joint's hard range is clamped the same
way, instead of being rejected. Pure functions, no simulator: the bridge (src/control_loop.py) calls them.
"""

import numpy as np

from src.task_config import BRIDGE_BUFFER_CFG, MINK_CFG


def buffer_margins(hard_lo, hard_hi):
    """The buffer margin of each joint (7 radians) for one arm: per BRIDGE_BUFFER_CFG. mode "range": min(range_fraction x joint range, cap_deg);
    mode "fixed": MINK_CFG joint_margin_rad on every joint."""
    lo = np.asarray(hard_lo, float)
    hi = np.asarray(hard_hi, float)
    if BRIDGE_BUFFER_CFG["mode"] == "fixed":
        return np.full(lo.shape, float(MINK_CFG["joint_margin_rad"]))
    cap = np.radians(float(BRIDGE_BUFFER_CFG["cap_deg"]))
    return np.minimum(float(BRIDGE_BUFFER_CFG["range_fraction"]) * (hi - lo), cap)


def buffer_limits(hard_lo, hard_hi, margin=None):
    """The buffer edges of one arm: (lo + margin, hi - margin), arrays of 7 radians. margin defaults to buffer_margins (the config's rule)."""
    m = buffer_margins(hard_lo, hard_hi) if margin is None else np.asarray(margin, float)
    return np.asarray(hard_lo, float) + m, np.asarray(hard_hi, float) - m


def clamp_to_buffer(q, lo_buf, hi_buf):
    """Clamps the 7 joint targets q into [lo_buf, hi_buf] per joint. Returns (clamped (7,), clamped_mask (7,) bool: True where the value changed).
    A value inside its buffer passes through exactly (the same float)."""
    q = np.asarray(q, float)
    clamped = np.clip(q, lo_buf, hi_buf)
    return clamped, clamped != q
