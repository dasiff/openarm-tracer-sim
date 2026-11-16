"""The braking layer (Part 2, David's braking brief): sits ABOVE the resolver; the gap rule inside the resolver is unchanged.

Works on (2, 7) joint arrays (left arm, right arm, rad) like the plug-in. Everything is measured at the tool point.
  measure()          measured speed and acceleration: finite difference of the tool point on consecutive ticks (world frame, so the base is in it)
  commanded_speed()  speed of a commanded joint change: forward kinematics of the change at the tool point (base contributes zero)
"""
import math
from dataclasses import replace

import numpy as np

from src.safety_filter import config as cfg
from src.safety_filter.braking_math import JerkRamp, stopping_distance

SIDES = ("left", "right")


def make_tool_fn(shapes, offset=None):
    """Tool point function of the real model: tool_fn(q (2, 7), state) -> (2, 3) world positions. state (RobotState) carries the base,
    pedestal and fingers; its own arm joints are replaced by q. Tool point = link 7 origin + R7 @ offset (config tool_offset_local_m)."""
    off = np.asarray(cfg.BRAKING["tool_offset_local_m"] if offset is None else offset, float)

    def tool_fn(q, state):
        s = replace(state, q_left=tuple(q[0]), q_right=tuple(q[1]))
        shapes.set_state(s)
        out = np.zeros((2, 3))
        for i, side in enumerate(SIDES):
            pos, rot = shapes.hand_frame(side)
            out[i] = pos + rot @ off
        return out
    return tool_fn


class BrakingLayer:
    def __init__(self, tool_fn, tun=None, bridge=None):
        """tool_fn(q (2, 7), state) -> (2, 3). tun: tunables (default config.BRAKING); the derived constants are computed once here."""
        self.tool_fn = tool_fn
        self.tun = cfg.BRAKING if tun is None else tun
        self.d = cfg.derive(self.tun, bridge)
        self.dt = self.d["tick_dt_s"]
        self._prev_pos = None          # (2, 3) tool points of the previous measure() call
        self._prev_speed = None        # (2,) speeds of the previous measure() call
        self.last_measure = None

    # ---- step 2: hand speed ----
    def measure(self, q, state):
        """Measured hand speed: |tool point now - tool point at the previous call| / dt, per arm (world frame). Acceleration: difference of
        consecutive speeds / dt (signed: positive = speeding up). Returns {"pos": (2,3), "vel": (2,3), "speed": (2,), "accel": (2,)}; the
        first call has no history and gives zeros."""
        pos = np.array(self.tool_fn(np.asarray(q, float), state), float)
        if self._prev_pos is None:
            vel = np.zeros((2, 3))
            speed = np.zeros(2)
            acc = np.zeros(2)
        else:
            vel = (pos - self._prev_pos) / self.dt
            speed = np.linalg.norm(vel, axis=1)
            acc = (speed - self._prev_speed) / self.dt
        self._prev_pos, self._prev_speed = pos, speed
        self.last_measure = {"pos": pos, "vel": vel, "speed": speed, "accel": acc}
        return self.last_measure

    def commanded_speed(self, last, cmd, state):
        """Speed of the commanded change last -> cmd at the tool point, per arm (2,): |FK(cmd) - FK(last)| / dt. The base is not in it:
        arms and base never move at the same time in an episode (exclusivity on)."""
        a = np.asarray(self.tool_fn(np.asarray(last, float), state), float)
        b = np.asarray(self.tool_fn(np.asarray(cmd, float), state), float)
        return np.linalg.norm(b - a, axis=1) / self.dt

    def reset(self):
        self._prev_pos = self._prev_speed = self.last_measure = None
