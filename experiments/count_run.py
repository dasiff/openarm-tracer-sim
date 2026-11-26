"""Runs one existing script unchanged and saves the bridge's clamp counts when it ends (SF-122 stage 1, logging only).

    COUNT_OUT=<file.json> python experiments/count_run.py <script.py> [args...]

The counts are src.control_loop.BRIDGE_CLAMP (per arm and joint, the times the bridge clamped a source's target into its joint buffer).
Writes the JSON at exit, also after a normal end of the script (atexit). Without COUNT_OUT nothing is written.
"""

import atexit
import json
import os
import runpy
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "experiments"))
os.environ.setdefault("MUJOCO_GL", "egl")


def _dump():
    out = os.environ.get("COUNT_OUT")
    if not out:
        return
    import src.control_loop as cl
    from src.task_config import COMMAND_BRIDGE_CFG
    counts = {side: [int(v) for v in arr] for side, arr in cl.BRIDGE_CLAMP["target"].items()}
    upd = {side: [int(v) for v in arr] for side, arr in cl.BRIDGE_CLAMP["updates"].items()}
    bridge_dt = 1.0 / COMMAND_BRIDGE_CFG["rate_hz"]
    with open(out, "w") as f:
        json.dump({"bridge_clamp_target_counts_per_joint": counts, "total": {s: sum(v) for s, v in counts.items()},
                   "bridge_clamp_update_counts_per_joint": upd,
                   "bridge_clamp_seconds_per_joint": {side: [round(v * bridge_dt, 3) for v in arr] for side, arr in upd.items()}}, f, indent=1)
    print(f"count_run: wrote {out}: targets {counts}; updates {upd}", flush=True)


def main():
    if len(sys.argv) < 2:
        raise SystemExit("usage: COUNT_OUT=file.json count_run.py <script.py> [args]")
    atexit.register(_dump)
    script = sys.argv[1]
    sys.argv = [script] + sys.argv[2:]
    sys.path.insert(0, str(Path(script).resolve().parent))
    runpy.run_path(script, run_name="__main__")


if __name__ == "__main__":
    main()
