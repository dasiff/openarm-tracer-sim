"""Per-tick trace of the sag layer during a policy stream run (stage 2 item A, David d0bc 2026-11-24): the predicted joints, the measured joints, the
gravity offset the loop applied (the offset computed at the start of the loop's tick, applied to the actuators), the offset the layer recomputes,
the control the actuators received, the measured sag and the estimate, on every call of the layer. Works in the production tree (src/sag_layer.py,
SagLayer) and in the test-copy tree (src/sag_measure.py, MeasureSagLayer, at 7d355b8 with SAG_RULE_CFG rule "measure"). Logging only: the wrappers
call the original methods and return what they return.

    python experiments/sag_trace_run.py --out DIR/trace.npz -- experiments/policy_stream_test.py --variant A --seed 1 --correction on --bridge off --seconds 30 --out DIR

The wrapped script runs unchanged (runpy). The trace is saved at exit.
"""

import argparse
import atexit
import json
import os
import runpy
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

LOOP = {"off": None, "t": None}      # the gravity offset (tau/kp, all actuators) the loop computed at the start of its last tick
REC = {k: [] for k in ("t", "q_src", "q_meas", "ctrl_now", "tau_now", "loop_off", "pred_pre", "pred", "est", "last_sag", "dq", "cmd", "skip", "scale")}


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    a, rest = ap.parse_known_args(argv)
    if rest and rest[0] == "--":
        rest = rest[1:]
    from src import gravity_ff
    from src.task_config import SAG_LAYER_CFG

    test_copy = (REPO / "src" / "sag_measure.py").exists()
    if test_copy:
        from src.task_config import SAG_RULE_CFG
        import src.sag_measure as mod
        SAG_RULE_CFG["rule"] = "measure"
        SAG_RULE_CFG["measure_T_s"] = 2.0
        cls = mod.MeasureSagLayer
        tag = "test_copy"
    else:
        from src import sag_layer as mod
        cls = mod.SagLayer
        tag = "production"

    orig_torques = gravity_ff.ArmGravityFeedforward.torques

    def torques_wrapped(self, data):
        tau = orig_torques(self, data)
        if sys._getframe(1).f_code.co_name == "advance":          # the loop's own offset, computed once per tick
            LOOP["off"] = np.asarray(tau, float) / np.asarray(self.kp, float)
            LOOP["t"] = float(data.time)
        return tau

    gravity_ff.ArmGravityFeedforward.torques = torques_wrapped
    orig_correct = cls.correct

    def correct_wrapped(self, q_src, data, t, nominal_dt):
        off = self.off
        acts = self.acts
        pre_pred = None if self.pred is None else np.asarray(self.pred, float).copy()
        tau_now = np.asarray(orig_torques(self.gff, data), float)[off:off + 7] / self.kp_arm
        loop_off = None if LOOP["off"] is None else LOOP["off"][off:off + 7]
        ctrl_now = np.asarray(data.ctrl[acts], float).copy()
        q_meas = np.asarray(data.qpos[self.arm.qadr], float).copy()
        cmd = orig_correct(self, q_src, data, t, nominal_dt)
        REC["t"].append(float(t))
        REC["q_src"].append(np.asarray(q_src, float).copy())
        REC["q_meas"].append(q_meas)
        REC["ctrl_now"].append(ctrl_now)
        REC["tau_now"].append(np.asarray(tau_now, float))
        REC["loop_off"].append(np.full(7, np.nan) if loop_off is None else np.asarray(loop_off, float).copy())
        REC["pred_pre"].append(np.full(7, np.nan) if pre_pred is None else pre_pred)
        REC["pred"].append(np.full(7, np.nan) if self.pred is None else np.asarray(self.pred, float).copy())
        REC["est"].append(np.asarray(self.est, float).copy())
        REC["last_sag"].append(np.full(7, np.nan) if getattr(self, "last_sag", None) is None else np.asarray(self.last_sag, float).copy())
        REC["dq"].append(np.asarray(self.dq, float).copy())
        REC["cmd"].append(np.asarray(cmd, float).copy())
        REC["skip"].append("" if not hasattr(self, "skips") else json.dumps(self.skips, sort_keys=True))
        REC["scale"].append(float(getattr(self, "clip_scale", 1.0)))
        return cmd

    cls.correct = correct_wrapped

    def save():
        out = {k: np.array(v) for k, v in REC.items() if len(v)}
        out["info"] = np.array(json.dumps({"tree": tag, "ticks": len(REC["t"]), "correction": SAG_LAYER_CFG["enabled"]}))
        np.savez(a.out, **out)
        print(f"sag_trace_run: wrote {a.out} ({len(REC['t'])} calls, tree {tag})", flush=True)

    atexit.register(save)
    script = rest[0]
    sys.argv = [script] + rest[1:]
    sys.path.insert(0, str(Path(script).resolve().parent))
    runpy.run_path(script, run_name="__main__")


if __name__ == "__main__":
    main(sys.argv[1:])
