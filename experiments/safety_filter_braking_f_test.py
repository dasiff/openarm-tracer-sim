"""Braking Phase F fixes 5 and 6 (SF-82 adaptive wake radius; SF-83 HOLD pruning): unit tests of the wake radius and of the cutoff the
look-ahead feed uses. Kinematics only (no simulator).

Run: python experiments/safety_filter_braking_f_test.py   (exit 0 = all pass).
"""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.safety_filter import config as cfg  # noqa: E402
from src.safety_filter.braking import BrakingLayer, ObstacleFeed  # noqa: E402
from src.safety_filter.braking_math import stopping_distance  # noqa: E402
from src.safety_filter.touch import wake_dist  # noqa: E402

fails = []


def check(name, ok, info=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  [{info}]" if info else ""))
    if not ok:
        fails.append(name)


def tool_lin(q, state):
    q = np.asarray(q, float)
    out = np.zeros((2, 3))
    out[:, 0] = 0.5 * q[:, 0]
    return out


DT = cfg.derived()["tick_dt_s"]
A_EFF = cfg.derived()["effective_accel_limit"]
JL = float(cfg.BRAKING["jerk_limit_m_s3"])
CUSH = cfg.derived()["tick_cushion_m"]
CAP = float(cfg.BRAKING["hand_speed_cap_m_s"])
WORST = cfg.derived()["worst_stop_dist_m"]
MARGIN = cfg.MARGINS["arm_world_m"]
START = cfg.PUSH_START_BEYOND_M


def layer_at(v, a=0.0, measured=None, measured_a=0.0):
    """A layer whose left arm runs at the profile speed v and acceleration a; an optional measured speed (the state source)."""
    lay = BrakingLayer(tool_lin)
    lay.v[0] = v
    lay.a[0] = a
    if measured is not None:
        lay.last_measure = {"speed": np.array([measured, 0.0]), "accel": np.array([measured_a, 0.0])}
    return lay


def rule(margin=MARGIN):
    """A pair rule as touch.pair_rule gives it (its static cutoff)."""
    return SimpleNamespace(margin=margin, activation=max(margin + START, wake_dist()))


class Recorder:
    """A stand-in for ObstacleFeed: records the wake radii and held arms the layer sets."""

    def __init__(self):
        self.calls = []

    def set_wake(self, wake, hold_arms):
        self.calls.append((None if wake is None else np.array(wake), np.array(hold_arms)))

    def __call__(self, q, state):
        return [[], []]


# ---- fix 5: the wake radius follows the current stopping distance ----
slow = layer_at(0.02)._wake_radii()[0]
fast = layer_at(0.20)._wake_radii()[0]
check("fix 5: a slow hand (0.02 m/s) has a smaller wake radius than a fast hand (0.20 m/s), both at rest acceleration",
      slow < fast, f"slow {slow * 1000:.1f} mm, fast {fast * 1000:.1f} mm")
worst_now = layer_at(CAP, a=A_EFF)._wake_radii()[0]
check("fix 5: the cap at the full effective acceleration gives the worst-case stopping distance (no pruning there)",
      abs(worst_now - WORST) < 1e-4, f"{worst_now * 1000:.1f} mm vs worst {WORST * 1000:.1f} mm")
check("fix 5: the radius never exceeds the worst-case stopping distance, at any speed and acceleration up to the limits",
      all(layer_at(v, a)._wake_radii()[0] <= WORST + 1e-9 for v in np.linspace(0.0, CAP, 11) for a in np.linspace(0.0, A_EFF, 6)))

r = rule()
dists = np.linspace(0.0, 0.2, 201)
woken_slow = int(sum(ObstacleFeed._activation(r, slow, False) >= d for d in dists))
woken_fast = int(sum(ObstacleFeed._activation(r, fast, False) >= d for d in dists))
check("fix 5: a slow hand wakes fewer pairs than a fast hand (the same pair at 201 distances, 0 to 20 cm)",
      woken_slow < woken_fast, f"slow {woken_slow}, fast {woken_fast} of 201")
check("fix 5: the cutoff is never wider than the static cutoff of the pair rule",
      all(ObstacleFeed._activation(r, w, h) <= r.activation + 1e-12 for w in (0.0, 0.05, 0.2, 1.0) for h in (False, True)))

# speed jump between ticks: a pair inside the new stopping distance must not be left asleep; one beyond it may stay asleep
jump = layer_at(0.02, measured=CAP)._wake_radii()[0]        # the measured speed jumped to the cap; the profile is still slow
check("fix 5 (speed jump): a measured jump to the cap wakes a pair inside its stopping distance (5 cm) in the same tick",
      ObstacleFeed._activation(r, jump, False) >= 0.05, f"wake {jump * 1000:.1f} mm")
check("fix 5 (speed jump): and leaves asleep a pair beyond it (12 cm, more than the stop from the cap)",
      ObstacleFeed._activation(r, jump, False) < 0.12, f"wake {jump * 1000:.1f} mm")
floor = layer_at(CAP)._wake_radii()[0]                       # the sent speed is at the cap, the measured speed lags at rest
check("fix 5 (floor): a lagging measurement does not hide a pair inside the stop from the cap (5 cm), the sent speed is at the cap",
      ObstacleFeed._activation(r, floor, False) >= 0.05, f"wake {floor * 1000:.1f} mm")
still = layer_at(0.0)._wake_radii()[0]
check("fix 5 (floor): a stopped hand's radius is the stopping distance from the highest speed the next tick can send (a_eff x tick)",
      abs(still - stopping_distance(min(CAP, A_EFF * DT), 0.0, A_EFF, JL, CUSH)) < 1e-9, f"{still * 1000:.2f} mm")

# ---- fix 6: HOLD keeps only the pairs within the remaining braking distance plus the margin ----
radii = [layer_at(v, a=(A_EFF if v > 0.0 else 0.0))._wake_radii()[0] for v in (CAP, 0.10, 0.05, 0.02, 0.0)]   # a stopped arm has no acceleration
hold_cut = [ObstacleFeed._activation(r, w, True) for w in radii]
check("fix 6: during HOLD the cutoff shrinks as the arm decelerates (speeds 0.20, 0.10, 0.05, 0.02, 0 m/s)",
      all(hold_cut[k] >= hold_cut[k + 1] - 1e-12 for k in range(len(hold_cut) - 1)) and hold_cut[0] > hold_cut[-1],
      ", ".join(f"{c * 1000:.1f}" for c in hold_cut) + " mm")
check("fix 6: once stopped, only pairs within the margin stay awake (the pair's own cutoff: margin + push start)",
      abs(hold_cut[-1] - (MARGIN + START)) < 1e-9, f"{hold_cut[-1] * 1000:.1f} mm = margin {MARGIN * 1000:.0f} + push start {START * 1000:.0f} mm")
check("fix 6: HOLD is never wider than the static cutoff", all(ObstacleFeed._activation(r, w, True) <= r.activation + 1e-12 for w in (0.0, 0.1, WORST, 0.3)))
check("fix 6: outside HOLD the pair keeps the wake radius alone (no margin added)",
      abs(ObstacleFeed._activation(r, radii[0], False) - min(r.activation, max(MARGIN + START, radii[0]))) < 1e-12)

# ---- the wire: the layer gives the feed this tick's wake radii and held arms before the look-ahead reads it ----
q0 = np.zeros((2, 7))
cfg.BRAKING["adaptive_wake"] = True          # the wiring is tested with the switch on (its default is off, pending David's decision on the floor)
rec = Recorder()
lay_w = layer_at(0.05)
lay_w._hold_arms = np.array([True, True])            # what process() sets from its HOLD trigger, before check()
lay_w.check(q0, q0, None, rec)
check("wire: check() hands the wake radii and the HOLD arms to the feed before the look-ahead reads it",
      len(rec.calls) >= 1 and rec.calls[0][0] is not None and rec.calls[0][0].shape == (2,) and bool(rec.calls[0][1].all()), f"{len(rec.calls)} calls")
rec2 = Recorder()
lay_w._hold_arms = np.zeros(2, bool)
lay_w.check(q0, q0, None, rec2)
check("wire: outside HOLD the feed is told that no arm is held", len(rec2.calls) >= 1 and not rec2.calls[0][1].any())
cfg_saved = cfg.BRAKING["adaptive_wake"]
cfg.BRAKING["adaptive_wake"] = False
try:
    rec3 = Recorder()
    BrakingLayer(tool_lin).check(q0, q0, None, rec3)
    check("switch: adaptive_wake False gives the feed no wake radii (the static cutoffs, as in Phase E)",
          len(rec3.calls) >= 1 and rec3.calls[0][0] is None)
finally:
    cfg.BRAKING["adaptive_wake"] = cfg_saved

print(f"{len(fails)} failing check(s)" if fails else "ALL PASS")
sys.exit(1 if fails else 0)
