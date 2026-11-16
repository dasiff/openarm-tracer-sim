"""Safety filter plug-in (src/safety_filter_plugin.py): modes, hold policy, last-sent bookkeeping, recording. Stub filter, no MuJoCo.
Run: python experiments/safety_filter_plugin_test.py   (exit 0 = pass)"""
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src import task_config as tc  # noqa: E402
from src.safety_filter_plugin import SafetyFilterPlugin, repeat_last_safe  # noqa: E402

fails = []


def check(name, ok, info=""):
    print(("PASS " if ok else "FAIL ") + name + (f"  {info}" if info and not ok else ""))
    if not ok:
        fails.append(name)


def status(s="PASS", reason=""):
    return SimpleNamespace(status=s, hold_reason=reason, rounds=0, worst_gap=float("nan"), worst_shortfall=0.0, fraction=float("nan"),
                           worst_pair=None, methods=(), pairs_pushed=0, pairs_skipped=0, pairs_shortened=0, solver_calls=0)


class Stub:
    def __init__(self, shift=0.1, hold_at=None):
        self.calls, self.shift, self.hold_at = [], shift, hold_at

    def __call__(self, raw, last, state, world, held):
        self.calls.append((raw.copy(), last.copy()))
        if self.hold_at is not None and len(self.calls) in self.hold_at:
            return last.copy(), status("HOLD", "NO_SOLVER")
        return raw - self.shift, status("HOLD" if False else "KONG_PUSH")


inputs = lambda holding: (None, [], None)  # noqa: E731
cur = {"left": [0.0] * 7, "right": [0.0] * 7}
cmd = lambda v: {"left": [v] * 7, "right": [v] * 7}  # noqa: E731

check("default mode is off", tc.SAFETY_FILTER_PLUGIN_CFG["mode"] == "off")
st = Stub()
p = SafetyFilterPlugin("off", st, inputs)
c = cmd(0.5)
out = p.process(c, {}, cur)
check("off: filter never called, raw returned unchanged (same object), nothing recorded", out is c and not st.calls and p.n_calls == 0 and not p.rec["t"])
check("off: limit never reached", not p.limit_reached())

st = Stub()
p = SafetyFilterPlugin("shadow", st, inputs)
out = p.process(cmd(0.5), {}, cur, 1.0)
check("shadow: filter called, raw command returned", len(st.calls) == 1 and np.allclose(out["left"], 0.5) and np.allclose(out["right"], 0.5))
check("shadow: the first last-safe command is the command the robot holds", np.allclose(st.calls[0][1], 0.0))
p.process(cmd(0.7), {}, cur, 1.1)
check("shadow: next last-safe command is the command actually sent (raw)", np.allclose(st.calls[1][1], 0.5))
check("shadow: filtered differs from sent in the record", np.allclose(p.arrays()["filtered"][0], 0.4) and np.allclose(p.arrays()["sent"][0], 0.5))

st = Stub()
p = SafetyFilterPlugin("on", st, inputs)
out = p.process(cmd(0.5), {}, cur)
check("on: filtered command returned", np.allclose(out["left"], 0.4))
p.process(cmd(0.7), {}, cur)
check("on: next last-safe command is the command actually sent (filtered)", np.allclose(st.calls[1][1], 0.4))
out = p.process({"left": [0.9] * 7}, {}, cur)
check("one-arm command: only that arm comes back; the other arm's raw = last sent", list(out) == ["left"] and np.allclose(st.calls[2][0][1], 0.6))

st = Stub(hold_at={2})
seen = []
def pol(last, status_, history):
    seen.append((last.copy(), status_.status, len(history)))
    return last + 0.01
p = SafetyFilterPlugin("on", st, inputs, hold_policy=pol)
p.process(cmd(0.5), {}, cur)
out = p.process(cmd(0.7), {}, cur)
check("hold: swapped policy called with (last sent, status, history) and its command sent", len(seen) == 1 and seen[0][1] == "HOLD" and seen[0][2] == 1 and np.allclose(out["left"], 0.41), str(seen))
st = Stub(hold_at={2})
p = SafetyFilterPlugin("on", st, inputs)
p.process(cmd(0.5), {}, cur)
out = p.process(cmd(0.7), {}, cur)
check("hold: default policy repeats the last sent command", np.allclose(out["left"], 0.4) and np.allclose(repeat_last_safe(np.ones((2, 7)), None, []), 1.0))
st = Stub(hold_at={1})
p = SafetyFilterPlugin("shadow", st, inputs, hold_policy=pol)
out = p.process(cmd(0.5), {}, cur)
check("hold in shadow: policy asked and recorded, robot still gets raw", np.allclose(out["left"], 0.5) and np.allclose(p.arrays()["filtered"][0], 0.01))

with tempfile.TemporaryDirectory() as d:
    p = SafetyFilterPlugin("shadow", Stub(hold_at={2}), inputs, run_name="t", out_dir=d, flush_every=2)
    for k in range(3):
        p.process(cmd(0.1 * k), {}, cur, 0.01 * k)
    check("recording flushed during the run (no temporary file left)", (Path(d) / "t.npz").exists() and not (Path(d) / "t.tmp.npz").exists())
    p.close()
    z = np.load(Path(d) / "t.npz")
    need = {"t", "raw", "filtered", "sent", "status", "reason", "filter_time", "rounds", "worst_gap", "worst_shortfall", "fraction", "worst_pair", "methods", "meta"}
    check("recording fields exist", need <= set(z.files), str(need - set(z.files)))
    check("recording has 3 ticks, status and reason of the hold tick", z["raw"].shape == (3, 2, 7) and z["status"][1] == "HOLD" and z["reason"][1] == "NO_SOLVER" and (z["filter_time"] >= 0).all())

p = SafetyFilterPlugin("shadow", Stub(), inputs, max_ticks=2)
p.process(cmd(0.1), {}, cur)
a = p.limit_reached()
p.process(cmd(0.1), {}, cur)
check("max_ticks limit", not a and p.limit_reached())
try:
    SafetyFilterPlugin("sometimes", Stub(), inputs)
    check("unknown mode raises", False)
except ValueError:
    check("unknown mode raises", True)

print(f"\n{'ALL PASS' if not fails else 'FAILED: ' + ', '.join(fails)}")
sys.exit(1 if fails else 0)
