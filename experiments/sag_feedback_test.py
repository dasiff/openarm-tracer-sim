"""SF-119 (David, 2026-11-22): the sag correction is cut out of the routine's view. The sources see the arm targets they commanded,
before the correction, the bridge and the safety filter; the arms receive the corrected targets.

    python experiments/sag_feedback_test.py      (exit code 0 when every check passes)

The loop (src/control_loop.run_loop) runs for real. The correction is a stand-in: src.control_loop.correct_command is replaced
by a function that adds a fixed offset (OFFSET, on joint 3 of every arm command), so the wiring can be checked without the
arm solver, which fails on this scene at the start. The real layer (src/sag_layer.py) is covered by sag_layer_test.py and
sag_solver_failure_test.py. The command bridge is off and the arm targets are held (no ramp) in these runs, so the actuator targets are the
last command at once (the bridge is on by default in the held runs; its own test is experiments/command_bridge_test.py).

Two source types:
  A. absolute targets (the staging, mock and policy style): the source sends whole arm targets and never reads them back;
  B. incremental (the keyboard and policy-delta style): each command is its last view plus a step, so it reads the view back.
For each source:
  1. each time the source is called, its view equals the targets it commanded the last time it was called (exact);
  2. the correction is applied to every arm command (the stand-in ran on each command);
  3. the actuators get the corrected targets: joint 3 is the commanded value plus OFFSET, the other joints are the commanded values.
Source B also checks that its commanded joint 2 accumulates exactly its own steps (no offset in the sum).
"""

import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import src.control_loop as control_loop  # noqa: E402
from src.control_loop import run_loop  # noqa: E402
from experiments_loader import load, steps_per_frame, SPAWN  # noqa: E402

ROUTES = {k: "script" for k in ("base", "pedestal", "arms.left", "arms.right", "gripper.left", "gripper.right")}
RESULTS = []
TICKS = 300
STEP_START = 30
OFFSET = 0.01                                              # rad, the stand-in correction on joint 3 of every arm command
STUB = {"calls": 0, "commands": 0}


def check(name, ok, detail=""):
    RESULTS.append(bool(ok))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"   ({detail})" if detail else ""), flush=True)


def stand_in_correction(layers, cmd, data, t, nominal_dt):
    """Stands in for correct_command: adds OFFSET to joint 3 of each arm target of an arms command. Counts the calls."""
    if cmd.get("mode") != "arms" or not cmd.get("arm_targets"):
        return cmd
    STUB["calls"] += 1
    out = {s: [float(v) + (OFFSET if k == 2 else 0.0) for k, v in enumerate(q)] for s, q in cmd["arm_targets"].items()}
    return {**cmd, "arm_targets": out}


def run(make_cmd):
    """Runs the loop for TICKS ticks. make_cmd(tick, view) -> an arms command (dict) or None. Returns the log.

    The source is called once per source frame (every few ticks). Each call logs the view it received, the targets it sent at its
    previous call, and the actuator targets (ctrl) of the left arm."""
    model, data = load()
    spf = steps_per_frame(model)
    ctrl_idx = [int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"left_joint{k + 1}_ctrl")) for k in range(7)]
    log = {"view": [], "sent": [], "ctrl": [], "view_gap": []}
    last = {"cmd": None}

    def step(obs):
        view = np.array(obs["applied"]["arm_targets"]["left"], float)
        log["view"].append(view)
        log["view_gap"].append(None if last["cmd"] is None else float(np.max(np.abs(view - last["cmd"]))))
        log["ctrl"].append(np.array([obs["ctrl"][i] for i in ctrl_idx], float))
        cmd = make_cmd(obs["tick"], view)
        if cmd is not None:
            last["cmd"] = np.array(cmd["arm_targets"]["left"], float)
            log["sent"].append(last["cmd"].copy())
            STUB["commands"] += 1
        return cmd

    cfg = {"source_hz": 1.0 / model.opt.timestep / spf, "spawn_pos": SPAWN, "cameras": False, "routes": ROUTES, "bench_frames": TICKS,
           "command_bridge": {"enabled": False}, "target_mode": "hold"}
    run_loop(model, data, {"script": {"name": "script", "step": step}}, cfg, viewer=None)
    return log


def view_gap(log):
    """Largest distance between a call's view and the targets the source sent at its previous call (radians)."""
    gaps = [g for g in log["view_gap"] if g is not None]
    return max(gaps) if gaps else float("nan")


def actuator_check(log):
    """Error of the joint-3 actuator target against the commanded joint 3 plus OFFSET, at the last call (the joint the stand-in touches).

    The actuator follows its hold through the ramp of the last command, so the error is a few 1e-5 rad at most; the offset is 1e-2 rad.
    """
    return float(abs(log["ctrl"][-1][2] - (log["sent"][-1][2] + OFFSET)))


def case_absolute():
    print("A. absolute-target source: a 0.2 rad step on joint 3 from its start pose, then hold", flush=True)
    rest = {"q": None}

    def make(k, view):
        if k < STEP_START:
            return None
        if rest["q"] is None:
            rest["q"] = view.copy()                          # the pose the source starts from; its targets are absolute from here on
        return {"mode": "arms", "arm_targets": {"left": [float(x) for x in (rest["q"] + np.where(np.arange(7) == 2, 0.2, 0.0))]}}

    STUB["calls"] = 0
    log = run(make)
    worst = view_gap(log)
    check("A1 the routine's view equals the targets it commanded at each call", worst < 1e-12, f"largest difference {worst:.2e} rad")
    check("A2 the correction ran on every arm command", STUB["calls"] == len(log["sent"]) and len(log["sent"]) > 0,
          f"{STUB['calls']} corrections for {len(log['sent'])} arm commands")
    err = actuator_check(log)
    check("A3 the actuators get the corrected targets (commanded plus OFFSET on joint 3)", err < 1e-4, f"largest error {err:.2e} rad")


def case_incremental():
    print("B. incremental source: each command is its last view plus 0.005 rad on joint 2, for 15 calls", flush=True)
    step, n = 0.005, 15                                      # short enough to stay inside joint 2's limit (a command beyond it is rejected)
    calls = {"n": 0}

    def make(k, view):
        if k < STEP_START or calls["n"] >= n:
            return None
        calls["n"] += 1
        return {"mode": "arms", "arm_targets": {"left": [float(x) for x in (view + np.where(np.arange(7) == 1, step, 0.0))]}}

    STUB["calls"] = 0
    log = run(make)
    worst = view_gap(log)
    check("B1 the routine's view equals the targets it commanded at each call", worst < 1e-12, f"largest difference {worst:.2e} rad")
    check("B2 the correction ran on every arm command", STUB["calls"] == len(log["sent"]) and len(log["sent"]) > 0,
          f"{STUB['calls']} corrections for {len(log['sent'])} arm commands")
    start = float(log["view"][0][1])                         # joint 2 at the first call (before any step)
    final = float(log["sent"][-1][1])
    expected = start + len(log["sent"]) * step
    check("B3 the commanded joint 2 accumulates exactly its own steps (no offset in the sum)", abs(final - expected) < 1e-9,
          f"final {final:.6f} rad, expected {expected:.6f} rad after {len(log['sent'])} steps, error {final - expected:+.2e}")
    err = actuator_check(log)
    check("B4 the actuators get the corrected targets (commanded plus OFFSET on joint 3)", err < 1e-4, f"largest error {err:.2e} rad")
    drift = float(log["sent"][-1][2] - log["sent"][0][2])
    check("B5 the commanded joint 3 does not drift (the source never commands the correction)", abs(drift) < 1e-9, f"drift {drift:+.6f} rad over {len(log['sent'])} calls")


def main():
    original = control_loop.correct_command
    control_loop.correct_command = stand_in_correction
    try:
        case_absolute()
        case_incremental()
    finally:
        control_loop.correct_command = original
    print(f"{sum(RESULTS)} of {len(RESULTS)} checks passed", flush=True)
    return 0 if all(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
