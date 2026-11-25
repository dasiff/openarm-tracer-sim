"""The command bridge's one uniform shrink (David d0bc, stage 3, 2026-11-24): the bridge's speed cap and its step-size rule are replaced by ONE factor per
update, applied to every joint's move on both arms.

Rules that limit a move (before stage 3; all in src/control_loop.py bridge_update):
  - step-size rule: a target more than max_target_step_rad (0.25 rad) from the command, on any joint, was REJECTED as a whole (the accepted target stayed).
  - speed cap: the command moved at most max_joint_velocity_rad_s / rate_hz per joint per update (0.5 / 50 = 0.01 rad), joint by joint (np.clip).
Kept: the tracking-error suspension (max_tracking_error_rad), the buffer clamp on the target (bridge_take, before the shrink), and the braking override
(src/bridge_override.py, the safety filter's jump, which is not a limit on a move).

The shrink (this module): for each rule, the factor it needs is min(1, its limit / the largest per-joint move of any joint of either arm at this
update); the smallest of those factors is applied to every joint's move on both arms: cmd <- cmd + f * (target - cmd). A move inside every limit
has f = 1 and passes exactly; a zero move stays zero; the direction of each arm's move is kept.
"""

import numpy as np


def shrink_factor(moves, step_limit, speed_limit):
    """moves: {side: 7-vector} the move of each arm toward its accepted target at this update (rad).
    Returns (factor, rule, side, joint, peak): factor = min(1, step_limit / peak, speed_limit / peak) with peak the largest |move| of any joint of
    any arm; rule = the binding rule ("speed" or "step") or None when the factor is 1; side and joint (1-based) locate the peak; peak in rad."""
    peak, side, joint = 0.0, None, None
    for s, d in moves.items():
        d = np.asarray(d, float)
        if d.size:
            m = float(np.max(np.abs(d)))
            if m > peak:
                peak, side, joint = m, s, int(np.argmax(np.abs(d))) + 1
    if peak <= 0.0:
        return 1.0, None, None, None, 0.0
    f_step = min(1.0, float(step_limit) / peak)
    f_speed = min(1.0, float(speed_limit) / peak)
    f = min(f_step, f_speed)
    if f >= 1.0:
        return 1.0, None, None, None, peak
    rule = "speed" if f_speed <= f_step else "step"
    return f, rule, side, joint, peak


def apply_shrink(cmd, target, factor):
    """The command after one update: cmd + factor * (target - cmd). With factor 1 the command is the target exactly (a move inside the limits)."""
    target = np.asarray(target, float)
    if factor >= 1.0:
        return target.copy()
    cmd = np.asarray(cmd, float)
    return cmd + factor * (target - cmd)
