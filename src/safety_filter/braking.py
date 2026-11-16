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
        self.ramp = JerkRamp(self.d["effective_accel_limit"], float(self.tun["jerk_limit_m_s3"]), self.dt)
        self.v = np.zeros(2)           # profile speed of each arm (m/s), the speed of the last command this layer produced
        self.a = np.zeros(2)           # profile acceleration of each arm (m/s^2)
        self._dir = np.zeros((2, 7))   # unit joint direction of each arm's last step (the way it was going; used when stopping)

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
        self.v[:] = 0.0
        self.a[:] = 0.0
        self._dir[:] = 0.0

    # ---- step 3: hand speed cap ----
    def scale_step(self, i, q_from, step, speed, state):
        """Scales the joint step of arm i (7,) by one factor so that the tool point moves at `speed` m/s over this tick (speed 0 -> no
        motion). Forward kinematics is nonlinear, so the factor is corrected a few times; it never ends above the wanted speed.
        Returns the scaled step (7,)."""
        step = np.asarray(step, float)
        if speed <= 0.0 or not np.any(step):
            return np.zeros(7)
        base = np.array(q_from, float)

        def moved(s):
            q = base.copy()
            q[i] = base[i] + s * step
            a = self.tool_fn(base, state)[i]
            return float(np.linalg.norm(self.tool_fn(q, state)[i] - a)) / self.dt

        s = 1.0
        for _ in range(8):
            v = moved(s)
            if v < 1e-12:
                break
            if abs(v - speed) <= 1e-6 * speed:
                break
            s *= speed / v
        while moved(s) > speed * (1.0 + 1e-6) and s > 1e-12:    # never above the wanted speed
            s *= 0.999
        return s * step

    def cap(self, last, raw, state):
        """The hand speed cap: for each arm whose commanded tool speed exceeds hand_speed_cap_m_s, all its joint changes are scaled back
        proportionally so the tool point moves at the cap. Returns (command (2, 7), capped (2,) bool)."""
        last, raw = np.asarray(last, float), np.asarray(raw, float)
        out = raw.copy()
        capped = np.zeros(2, bool)
        vs = self.commanded_speed(last, raw, state)
        capv = float(self.tun["hand_speed_cap_m_s"])
        for i in range(2):
            if vs[i] > capv:
                out[i] = last[i] + self.scale_step(i, last, raw[i] - last[i], capv, state)
                capped[i] = True
        return out, capped

    # ---- step 4: S-curve profile with the jerk limit ----
    def _va(self, i):
        """Speed and acceleration the ramp starts from: the profile of the commands sent ("sent"), or the measured ones ("state")."""
        if self.tun["speed_source"] == "state" and self.last_measure is not None:
            return float(self.last_measure["speed"][i]), float(self.last_measure["accel"][i])
        return float(self.v[i]), float(self.a[i])

    def step(self, last, raw, state, stop=None, speed_limit=None):
        """One tick of the profile. Each arm's speed moves toward its target through the jerk-limited ramp (S-shaped both ways); the joint
        step is the commanded direction scaled to the ramp's speed.
        stop: (2,) bool or None; True = target speed 0 along the way the arm was going (a stop; the command is not followed).
        speed_limit: (2,) extra speed limit per arm (m/s) or None (used by the look-ahead for allowed-contact surfaces).
        Otherwise the target is min(commanded speed, hand speed cap); the arm never goes past the commanded pose. Returns (command (2,7), info)."""
        last, raw = np.asarray(last, float), np.asarray(raw, float)
        out = last.copy()
        stop = np.zeros(2, bool) if stop is None else np.asarray(stop, bool)
        vs = self.commanded_speed(last, raw, state)
        capv = float(self.tun["hand_speed_cap_m_s"])
        for i in range(2):
            v0, a0 = self._va(i)
            delta = raw[i] - last[i]
            if stop[i] or not np.any(delta):
                direction = self._dir[i]
                target = 0.0 if stop[i] else 0.0
            else:
                direction = delta
                target = min(float(vs[i]), capv)
            if speed_limit is not None:
                target = min(target, float(speed_limit[i]))
            v1, a1 = self.ramp.step(v0, a0, target)
            if not np.any(direction):
                v1, a1 = 0.0, 0.0
            step = self.scale_step(i, last, direction, v1, state) if v1 > 0.0 else np.zeros(7)
            if not stop[i] and np.any(step) and np.linalg.norm(step) > np.linalg.norm(delta):   # never past the commanded pose
                step = delta
            out[i] = last[i] + step
            n = np.linalg.norm(step)
            if n > 0:
                self._dir[i] = step / n
            self.v[i], self.a[i] = v1, a1
        return out, {"speed": self.v.copy(), "accel": self.a.copy()}
