"""Runs one experiment script with the sag trim forced to zero, in this process only (SF-120 trace, 2026-11-23).

The layer runs as it is (its solver included); only the trim output C is set to zero after each integration step.
Usage: python experiments/trim_zero_run.py <script name without .py> [args...]
       e.g. python experiments/trim_zero_run.py combined_controller   (the manual baseline's command, see cc_bench_scenarios.sh)
"""

import os
import runpy
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "experiments"))
os.environ.setdefault("MUJOCO_GL", "egl")

import src.sag_layer as sag_layer  # noqa: E402

real_step = sag_layer._sag_trim_step


def zeroed(C, cfg, model, side, t, p_err, r_err, dt):
    real_step(C, cfg, model, side, t, p_err, r_err, dt)
    C["p"][:] = 0.0
    C["r"][:] = 0.0


sag_layer._sag_trim_step = zeroed

script = str(PROJECT_ROOT / "experiments" / (sys.argv[1] + ".py"))
sys.argv = [script] + sys.argv[2:]
sys.path.insert(0, str(Path(script).parent))
runpy.run_path(script, run_name="__main__")
