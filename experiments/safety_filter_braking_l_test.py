"""Braking Phase L (performance, continued): one check per fix. Kinematics only (no simulator) except where noted.

Fix 1 (SF-99): the plugin's per-call recorder holds the braking layer's time next to the resolver's time, so the full call (resolver plus
braking) is in the recording.

Run: python experiments/safety_filter_braking_l_test.py   (exit 0 = all pass).
"""
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.safety_filter.resolver import FilterStatus  # noqa: E402
from src.safety_filter.shapes import RobotState  # noqa: E402
from src.safety_filter_plugin import SafetyFilterPlugin  # noqa: E402

fails = []


def check(name, ok, info=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  [{info}]" if info else ""))
    if not ok:
        fails.append(name)


# ---- fix 1 (SF-99): the recorder holds the braking time next to the resolver time ----
HOME = np.zeros((2, 7))


def slow_resolver(raw, last, state, world, held):
    time.sleep(0.005)                                  # the resolver's share of the tick: about 5 ms
    return np.asarray(raw, float), FilterStatus(status="PASS")


class SlowBraking:
    """Stands in for the braking layer: about 10 ms, returns the command as it is with the fields the plugin reads."""

    errors = []

    def process(self, last, filtered, state, obstacle_fn=None, hold=False, call_dt=None):
        time.sleep(0.010)
        binfo = {"lookahead": [None, None], "stop": [False, False], "override": [False, False],
                 "limit": [np.inf, np.inf], "speed": [0.0, 0.0]}
        return np.asarray(filtered, float), binfo


state = RobotState(q_left=(0.0,) * 7, q_right=(0.0,) * 7)
plugin = SafetyFilterPlugin("on", slow_resolver, lambda holding: (state, [], None), run_name="test_phase_l",
                            braking=SlowBraking(), obstacle_fn=None)
arm_targets = {"left": [0.0] * 7, "right": [0.0] * 7}
current = {"left": [0.0] * 7, "right": [0.0] * 7}
for k in range(5):
    plugin.process(arm_targets, {"left": False, "right": False}, current, t=0.01 * k)
rec = plugin.rec
ft = np.asarray(rec["filter_time"], float)
bt = np.asarray(rec["braking_time"], float)
check("fix 1 (SF-99): the recorder has the braking time, one value per call", len(bt) == len(ft) == 5, f"calls {len(ft)}, braking values {len(bt)}")
check("fix 1 (SF-99): the resolver time is recorded as before (about 5 ms per call)", bool(np.all(ft >= 0.004)), f"mean {ft.mean() * 1e3:.2f} ms")
check("fix 1 (SF-99): the braking time is recorded (about 10 ms per call)", bool(np.all(bt >= 0.009)), f"mean {bt.mean() * 1e3:.2f} ms")
full = ft + bt
check("fix 1 (SF-99): the full call (resolver plus braking) is the sum: about 15 ms per call", bool(np.all(full >= 0.013)),
      f"full call mean {full.mean() * 1e3:.2f} ms")
plugin_off = SafetyFilterPlugin("on", slow_resolver, lambda holding: (state, [], None), run_name="test_phase_l_nb", braking=None)
plugin_off.process(arm_targets, {"left": False, "right": False}, current, t=0.0)
check("fix 1 (SF-99): without a braking layer the braking time is recorded as zero", plugin_off.rec["braking_time"][-1] == 0.0)

print(f"{len(fails)} failing check(s)" if fails else "ALL PASS")
sys.exit(1 if fails else 0)
