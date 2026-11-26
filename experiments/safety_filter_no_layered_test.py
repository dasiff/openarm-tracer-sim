"""Safety filter Phase S, S6 (SF-108 and the Phase S verification): no layered resolver code is left; kong_style is the only resolver.

Structural checks (no physics, no scene):
  1. the config has no layered switch or layered-only keys (mode, elbow_swing_hand_target, shorten_after_failed_push, swing_unmovable_m,
     swing_straight_m, elbow_displacement_mode), and RESOLVER still has the kong_style settings;
  2. Resolver.step hands every tick to kong_style.step (the one resolver), and no resolver module names a swing, slide, layered or CBF function;
  3. the layered status names are only the constants and the status tuple (kept so older recordings keep their indices): nothing produces them;
  4. the filter's resolver modules define no function whose name is a removed layered step (hand slide, elbow swing, lever rule, CBF).

    python experiments/safety_filter_no_layered_test.py      (exit code 0 when every check passes)
"""

import importlib
import inspect
import os
import re
import sys

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, REPO)

from src.safety_filter import config as cfg  # noqa: E402

FAILS = []
REMOVED_KEYS = ("mode", "elbow_swing_hand_target", "shorten_after_failed_push", "swing_unmovable_m", "swing_straight_m", "elbow_displacement_mode")
RESOLVER_MODULES = ("src.safety_filter.resolver", "src.safety_filter.kong_style", "src.safety_filter.push_direction", "src.safety_filter.link_rules")
REMOVED_NAME = re.compile(r"swing|slide|layered|cbf", re.IGNORECASE)


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    if not ok:
        FAILS.append(name)


def main():
    present = [k for k in REMOVED_KEYS if k in cfg.RESOLVER]
    check("S6 (1) the config has none of the layered keys (mode, elbow_swing_hand_target, shorten_after_failed_push, swing_unmovable_m, "
          "swing_straight_m, elbow_displacement_mode)", not present, f"present: {present}")
    check("S6 (1) the kong_style settings are still in RESOLVER", all(k in cfg.RESOLVER for k in ("stop_m", "rounds", "pair_order", "min_lever_fraction")))

    from src.safety_filter import resolver as R
    from src.safety_filter import kong_style as K
    step_src = inspect.getsource(R.Resolver.step)
    check("S6 (2) Resolver.step hands every tick to kong_style.step (the one resolver)", "kong_style.step" in step_src and "layered" not in step_src.lower(),
          "Resolver.step calls kong_style.step" if "kong_style.step" in step_src else "no kong_style call")

    bad = []
    for mod_name in RESOLVER_MODULES:
        mod = importlib.import_module(mod_name)
        for name, obj in vars(mod).items():
            if callable(obj) and getattr(obj, "__module__", "") == mod_name and REMOVED_NAME.search(name):
                bad.append(f"{mod_name.split('.')[-1]}.{name}")
    check("S6 (2, 4) no resolver module defines a swing, slide, layered or CBF function", not bad, f"found: {bad}")

    src = inspect.getsource(R)
    code_lines = [ln.split("#")[0] for ln in src.splitlines()]
    produced = [ln.strip() for ln in code_lines if ("ELBOW_SWING" in ln or "HAND_SLIDE" in ln) and "=" not in ln and "STATUSES" not in ln]
    check("S6 (3) the layered status names are only defined (constants and the status tuple), nothing produces them", not produced, f"producing lines: {produced}")

    check("S6 (3) the layered status names are not used by kong_style", "ELBOW_SWING" not in inspect.getsource(K) and "HAND_SLIDE" not in inspect.getsource(K))
    print("FAILED: " + ", ".join(FAILS) if FAILS else "ALL PASS")
    return 0 if not FAILS else 1


if __name__ == "__main__":
    sys.exit(main())
