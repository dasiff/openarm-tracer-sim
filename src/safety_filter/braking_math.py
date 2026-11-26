"""Pure numbers of the braking layer (no MuJoCo, no config): the stopping distance and the jerk-limited speed ramp.

Decision source: David's braking brief (Part 2). All units SI: m, m/s, m/s^2, m/s^3, s.
"""
import math

from numba import njit


@njit(cache=True)                                  # Phase J fix 2 (SF-93): compiled (pure arithmetic; the same numbers as the Python version it replaces)
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


def stopping_time(v, a0, a_lim, j_lim):
    """Time the same two-step stop takes (ease off a0, then brake from v1): a0/j + (v1/a + a/j, or 2 sqrt(v1/j))."""
    v = max(0.0, float(v))
    a0 = max(0.0, float(a0))
    a, j = float(a_lim), float(j_lim)
    v1 = v + a0 * a0 / (2.0 * j)
    t2 = v1 / a + a / j if v1 >= a * a / j else 2.0 * math.sqrt(v1 / j)
    return a0 / j + t2


def slowing_distance(v, a0, c, a_lim, j_lim, cushion=0.0):
    """Distance the hand travels while it slows from speed v (acceleration a0) to the ceiling c and arrives at c with zero acceleration:
    the stopping distance of the excess speed (v - c) plus c times the slowing time, plus the cushion. 0 if v <= c."""
    if v <= c:
        return 0.0
    u = v - c
    return stopping_distance(u, a0, a_lim, j_lim, 0.0) + c * stopping_time(u, a0, a_lim, j_lim) + float(cushion)


class JerkRamp:
    """Jerk-limited speed ramp: moves (v, a) toward a target speed with |a| <= a_lim and |da| <= j_lim*dt per tick, giving an S-shaped
    speed curve both when speeding up and when slowing down. One call = one tick. Discrete and exact: the acceleration chosen each tick is
    the largest one from which ramping the acceleration back to 0 at the jerk limit still ends at or below the target speed, so the
    acceleration reaches 0 smoothly (no snap) when the speed arrives."""

    LAND_TOL = 1e-5   # m/s: this close to the target (with the acceleration within one jerk step of 0) the speed lands on it

    def __init__(self, a_lim, j_lim, dt):
        self.a_lim, self.j_lim, self.dt = float(a_lim), float(j_lim), float(dt)

    def _ramp_down_gain(self, a):
        """Speed gained (or lost, for a < 0) while the acceleration a is ramped to 0 at the jerk limit, one trapezoid per tick."""
        sgn = 1.0 if a >= 0 else -1.0
        a = abs(a)
        d = self.j_lim * self.dt
        n = math.floor(a / d)
        r = a / d - n
        return sgn * self.dt * (n * a - d * n * n / 2.0 + r * d / 2.0)

    def step(self, v, a, v_target):
        """Returns (v_new, a_new) after one tick."""
        dt, d = self.dt, self.j_lim * self.dt
        sgn = 1.0 if v_target >= v else -1.0          # work in the direction of the speed change
        dv, ap = abs(v_target - v), sgn * a
        if dv <= self.LAND_TOL and abs(a) <= d + 1e-12:
            return v_target, 0.0
        lo, hi = max(ap - d, -self.a_lim), min(ap + d, self.a_lim)

        def total(an):                                  # speed change this tick plus the ramp-down afterwards
            return 0.5 * (ap + an) * dt + self._ramp_down_gain(an)

        if total(lo) > dv:
            an = lo                                     # already too fast for the target: bring the acceleration down as fast as allowed
        elif total(hi) <= dv:
            an = hi
        else:
            for _ in range(60):
                mid = 0.5 * (lo + hi)
                if total(mid) <= dv:
                    lo = mid
                else:
                    hi = mid
            an = lo
        v_new = v + sgn * 0.5 * (ap + an) * dt
        a_new = sgn * an
        if v_new < 0.0:
            v_new, a_new = 0.0, 0.0
        return v_new, a_new
