"""Braking override of the command bridge (Part 2, David's braking brief).

The bridge (src/control_loop.py) moves its command toward the latest target at most max_joint_velocity_rad_s / rate_hz per update and rejects a
target more than max_target_step_rad away. Braking normally stays inside those limits. If braking must exceed them to avoid a collision (the
look-ahead failed to catch the obstacle early enough: a bug to investigate), the bridge's command jumps to the latest target at once and an
error is logged. This is the bridge side, as a pure function so it can be tested without the simulator.
"""
import numpy as np


def force_to_target(bridge, rate_step_rad, step_reject_rad):
    """Sets the bridge's command (bridge["cmd"][side]) and its accepted target to the latest target of every arm, skipping the speed cap and the
    step rejection. bridge: the loop's bridge dict ("cmd", "target", "accepted"). rate_step_rad: the bridge's usual most-per-update move.
    step_reject_rad: its usual step rejection limit. Returns (arm_targets {side: list of 7} to apply, errors [str]): an error line for each arm
    whose jump was beyond one of the usual limits (the override was needed); an empty list means the override was not necessary."""
    if not bridge.get("target") or bridge.get("cmd") is None:
        return {}, []
    out, errors = {}, []
    for side, tgt in bridge["target"].items():
        jump = float(np.max(np.abs(np.asarray(tgt, float) - bridge["cmd"][side])))
        if jump > min(rate_step_rad, step_reject_rad) + 1e-12:
            errors.append(f"{side}: the bridge command jumped {jump:.4f} rad to the braking target (usual most {rate_step_rad:.4f} rad per update, "
                          f"rejection above {step_reject_rad:.2f} rad): the look-ahead did not catch the obstacle early enough")
        bridge["cmd"][side] = np.array(tgt, float)
        bridge["accepted"][side] = np.array(tgt, float)
        out[side] = [float(v) for v in bridge["cmd"][side]]
    return out, errors
