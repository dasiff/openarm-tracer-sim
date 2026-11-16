"""Pure numbers of the braking layer (no MuJoCo, no config): the stopping distance and the jerk-limited speed ramp.

Decision source: David's braking brief (Part 2). All units SI: m, m/s, m/s^2, m/s^3, s.
"""
import math


def stopping_distance(v, a0, a_lim, j_lim, cushion=0.0):
    """Distance the hand travels while it brakes to rest, jerk-aware, cautious, closed form.

    v: current hand speed toward the obstacle (m/s, >= 0). a0: current hand acceleration toward the obstacle (m/s^2; a negative value,
    i.e. already slowing, is treated as 0: the cautious reading). a_lim: acceleration limit used for braking. j_lim: jerk limit.
    cushion: added at the end (tick_cushion_m).
      Step 1  ease off the current acceleration: time a0/j, speed rises to v1 = v + a0^2/(2j), distance v*a0/j + a0^3/(3 j^2).
      Step 2  brake from v1: v1 >= a^2/j: v1^2/(2a) + v1*a/(2j);   else: v1^1.5 / sqrt(j).
    """
    v = max(0.0, float(v))
    a0 = max(0.0, float(a0))
    a, j = float(a_lim), float(j_lim)
    d1 = v * a0 / j + a0 ** 3 / (3.0 * j * j)
    v1 = v + a0 * a0 / (2.0 * j)
    if v1 >= a * a / j:
        d2 = v1 * v1 / (2.0 * a) + v1 * a / (2.0 * j)
    else:
        d2 = v1 ** 1.5 / math.sqrt(j)
    return d1 + d2 + float(cushion)


class JerkRamp:
    """Jerk-limited speed ramp: moves (v, a) toward a target speed with |a| <= a_lim and |da/dt| <= j_lim, giving an S-shaped speed curve
    both when speeding up and when slowing down. One call = one tick."""

    def __init__(self, a_lim, j_lim, dt):
        self.a_lim, self.j_lim, self.dt = float(a_lim), float(j_lim), float(dt)

    def step(self, v, a, v_target):
        """Returns (v_new, a_new) after one tick. The acceleration wanted is the largest one from which the jerk limit can still bring the
        acceleration to 0 exactly when the speed reaches the target: sign(dv) * sqrt(2 j |dv|), capped at a_lim; the acceleration then
        follows it by at most j*dt per tick."""
        dt, j = self.dt, self.j_lim
        dv = v_target - v
        a_des = math.copysign(min(self.a_lim, math.sqrt(2.0 * j * abs(dv))), dv) if dv != 0.0 else 0.0
        a_new = a + max(-j * dt, min(j * dt, a_des - a))
        a_new = max(-self.a_lim, min(self.a_lim, a_new))
        v_new = v + 0.5 * (a + a_new) * dt            # trapezoid: exact for the piecewise linear acceleration
        if (v_new - v_target) * (v - v_target) < 0.0 or abs(v_new - v_target) < 1e-9:   # crossed or reached the target: land on it
            v_new, a_new = v_target, 0.0
        if v_new < 0.0:
            v_new, a_new = 0.0, 0.0
        return v_new, a_new
