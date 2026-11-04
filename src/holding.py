"""Holding verdict for one hand (decisions M246, M248, M250, M251). One copy of the rule, used by the signal trace (log only) and by the
open-catch helper in the control loop.

Rule: holding is ON only while BOTH tests pass: the squeeze (the closing force is at or below -squeeze_threshold_N, total for both fingers)
and the finger position (inside grasp_closure_min_m to grasp_closure_max_m). It turns ON after both have passed for holding_confirm_ticks
ticks in a row; it turns OFF at once when either test fails. All numbers are in HOLD_STILL_CFG (src/task_config.py), PROVISIONAL and sim-only.
"""

from src.task_config import HOLD_STILL_CFG


class HoldDetector:
    """One hand. update(force, finger1) is called once per control tick, in time order."""

    def __init__(self):
        self.count = 0          # consecutive ticks with both tests passing
        self.holding = False

    def update(self, force, finger1):
        """force: the gripper actuator force (N, negative when closing). finger1: the finger joint position (m). Returns (squeeze_ok, closure_ok, holding)."""
        cfg = HOLD_STILL_CFG
        squeeze_ok = force <= -cfg["squeeze_threshold_N"]
        closure_ok = cfg["grasp_closure_min_m"] <= finger1 <= cfg["grasp_closure_max_m"]
        both = squeeze_ok and closure_ok
        self.count = self.count + 1 if both else 0
        self.holding = both and (self.holding or self.count >= cfg["holding_confirm_ticks"])
        return squeeze_ok, closure_ok, self.holding
