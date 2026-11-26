"""Phase K benchmark: wall time of the filter tick over one integration episode, with light wrappers (one perf_counter pair per call).
Times: the whole plugin tick (SafetyFilterPlugin.process), the resolver step (what filter_time records), and the braking layer's process.
Usage: python bench_k.py <json out> <episode args...>"""
import atexit
import functools
import json
import runpy
import sys
import time
from pathlib import Path

ROOT = Path("/home/vboxuser/projects/openarm-tracer-sim-sf-d")
sys.path.insert(0, str(ROOT))
OUT = sys.argv[1]
EP_ARGS = sys.argv[2:]

import src.safety_filter.braking as BR  # noqa: E402
import src.safety_filter.resolver as RES  # noqa: E402
import src.safety_filter_plugin as PL  # noqa: E402

perf = time.process_time
rec = {"plugin": [], "resolver": [], "braking": []}


def _timed(fn, key):
    @functools.wraps(fn)
    def w(*a, **k):
        t0 = perf()
        r = fn(*a, **k)
        rec[key].append(perf() - t0)
        return r
    return w


PL.SafetyFilterPlugin.process = _timed(PL.SafetyFilterPlugin.process, "plugin")
RES.Resolver.step = _timed(RES.Resolver.step, "resolver")
BR.BrakingLayer.process = _timed(BR.BrakingLayer.process, "braking")


def _stats(v):
    a = 1e3 * __import__("numpy").asarray(v)
    if len(a) == 0:
        return None
    return {"n": int(len(a)), "mean": float(a.mean()), "p95": float(__import__("numpy").percentile(a, 95)), "max": float(a.max())}


def dump():
    out = {k: _stats(v) for k, v in rec.items()}
    json.dump(out, open(OUT, "w"), indent=1)
    print("\nBENCH_K", json.dumps(out))


atexit.register(dump)
sys.argv = [str(ROOT / "experiments" / "safety_filter_braking_episode.py")] + EP_ARGS
try:
    runpy.run_path(sys.argv[0], run_name="__main__")
except SystemExit:
    pass
