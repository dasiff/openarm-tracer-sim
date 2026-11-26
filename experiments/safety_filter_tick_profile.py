"""Phase K step 1: exclusive per-call timing of the safety filter tick during one integration episode. Read-only: it wraps methods at run time
(no repo file is changed). Usage: python prof_k.py <json out> <episode args...>   (run from the worktree, with the venv active)."""
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
import src.safety_filter.pair_jit as PJ  # noqa: E402
import src.safety_filter.resolver as RES  # noqa: E402
import src.safety_filter.shapes as SH  # noqa: E402
import src.safety_filter.tick as TK  # noqa: E402
import src.safety_filter_plugin as PL  # noqa: E402

perf = time.perf_counter
stack = []                      # [name, start, child time]
excl, incl, ncalls = {}, {}, {}


def _wrap(fn, name):
    @functools.wraps(fn)
    def w(*a, **k):
        t0 = perf()
        stack.append([name, t0, 0.0])
        try:
            return fn(*a, **k)
        finally:
            fr = stack.pop()
            el = perf() - fr[1]
            excl[name] = excl.get(name, 0.0) + (el - fr[2])
            incl[name] = incl.get(name, 0.0) + el
            ncalls[name] = ncalls.get(name, 0) + 1
            if stack:
                stack[-1][2] += el
    return w


def wrap_class(cls, prefix=None, only=None):
    prefix = prefix or cls.__name__
    for attr, val in list(cls.__dict__.items()):
        if attr.startswith("__") and attr != "__init__":
            continue
        if only is not None and attr not in only:
            continue
        if isinstance(val, (staticmethod, classmethod, property)) or not callable(val):
            continue
        setattr(cls, attr, _wrap(val, f"{prefix}.{attr}"))


wrap_class(PL.SafetyFilterPlugin, "Plugin")
wrap_class(RES.Resolver, "Resolver")
wrap_class(TK._Tick, "Tick")
wrap_class(TK.Statics, "Statics")
wrap_class(TK.SleepTimers, "Sleep")
wrap_class(BR.BrakingLayer, "Braking")
wrap_class(BR.ObstacleFeed, "Feed")
wrap_class(SH.RobotShapes, "Shapes")
for name in ("look_kernel", "aabb_all"):
    setattr(BR, name, _wrap(getattr(BR, name), f"kernel.{name}"))
BR._shape_arrays = _wrap(BR._shape_arrays, "Feed._shape_arrays")
_orig_mtf = BR.make_tool_fn


def _mtf(*a, **k):
    f = _orig_mtf(*a, **k)
    return _wrap(f, "Braking.tool_fn")


BR.make_tool_fn = _mtf


def dump():
    n = ncalls.get("Plugin.process", 0)
    if n == 0:
        return
    rows = sorted(excl.items(), key=lambda kv: -kv[1])
    res = {"calls": n, "per_call_ms": {}, "exclusive_ms": {}, "calls_per_tick": {}}
    for k in ("Plugin.process", "Resolver.step", "Braking.process"):
        if k in incl:
            res["per_call_ms"][k] = 1e3 * incl[k] / n
    for k, v in rows:
        res["exclusive_ms"][k] = 1e3 * v / n
        res["calls_per_tick"][k] = ncalls[k] / n
    json.dump(res, open(OUT, "w"), indent=1)
    print(f"\nPROFILE calls {n}")
    for k in ("Plugin.process", "Resolver.step", "Braking.process"):
        if k in incl:
            print(f"  inclusive {k:18s} {1e3 * incl[k] / n:8.3f} ms per tick (timed run; wrapper overhead included)")
    print(f"  {'exclusive function':44s} {'ms/tick':>9s} {'calls/tick':>10s}")
    for k, v in rows[:40]:
        print(f"  {k:44s} {1e3 * v / n:9.3f} {ncalls[k] / n:10.1f}")


atexit.register(dump)
sys.argv = [str(ROOT / "experiments" / "safety_filter_braking_episode.py")] + EP_ARGS
try:
    runpy.run_path(sys.argv[0], run_name="__main__")
except SystemExit:
    pass
