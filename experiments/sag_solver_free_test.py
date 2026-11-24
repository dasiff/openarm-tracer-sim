"""Tests of the solver-free sag layer (SF-122 stage 2, David 2026-11-23): the learned offset from the error of the expected joints (REF(t-1) minus the
motor lag tau_j * refspeed_j), the linear step from the offset to joints, the reference REF(t) = bridge output - dq(t), and the skip rules.

Harness: the task scene (load_task, no cameras), the left arm at its home pose, a bridge emulator (targets clamped into the joint buffer, the
command moves at most 0.01 rad per update, one update every third tick, the bridge's output is what the layer observes), and measured joints
that lag the bridge output by the model's own motor lag (measured = output - droop - tau * speed). The fade of SF-118 is switched off where a test
needs the integration alone.

    python experiments/sag_solver_free_test.py      (exit code 0 when every check passes)
"""

import os
import sys
from collections import deque

import numpy as np

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, REPO)
os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402

from src.robot_helpers import arm_ids  # noqa: E402
from src.sag_layer import SagLayer  # noqa: E402
from src.task_beaker_hotplate import load_task  # noqa: E402
from src.task_config import MINK_CFG, SAG_LAYER_CFG, TASK_CFG  # noqa: E402

HOME = np.asarray(TASK_CFG["home_q_left"], float)
NOMINAL = 1.0 / 150.0
EVERY = 3                                    # the bridge updates every third tick (50 Hz)
RATE = 0.5 / 50.0                            # 0.01 rad per update (0.5 rad/s)
MARGIN = float(MINK_CFG["joint_margin_rad"])
FAILS = []
RESULTS = {}


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}", flush=True)
    if not ok:
        FAILS.append(name)


class BridgeEmu:
    """The bridge as the layer sees it: a target clamped into the joint buffer; the output moves toward it at most RATE per update."""

    def __init__(self, q0, lo_buf, hi_buf):
        self.out = np.array(q0, float)
        self.target = np.array(q0, float)
        self.lo, self.hi = lo_buf, hi_buf

    def take(self, cmd):
        self.target = np.clip(np.asarray(cmd, float), self.lo, self.hi)

    def update(self):
        self.out = self.out + np.clip(self.target - self.out, -RATE, RATE)


def rig(q0=HOME, delta=None):
    sim = load_task(cameras=False)
    model, data = sim.model, sim.data
    layer = SagLayer(model, data, "left", HOME)
    qadr = layer.arm.qadr
    data.qpos[qadr] = q0
    mujoco.mj_forward(model, data)
    jnt = arm_ids(model, "left")["jnt"]
    lo = model.jnt_range[jnt, 0] + MARGIN
    hi = model.jnt_range[jnt, 1] - MARGIN
    emu = BridgeEmu(q0, lo, hi)
    delta = np.zeros(7) if delta is None else np.asarray(delta, float)
    return model, data, layer, qadr, emu, delta


def run(model, data, layer, qadr, emu, delta, src_of_k, n, k0=0, bridge=True, suspend_at=None, pad_droop=None):
    """n ticks. Each tick: the layer's command for the source's targets; the bridge takes it, updates every third tick, and the layer observes the
    output; the measured joints lag the output by the model's motor lag (and the droop delta). Returns the per-tick records."""
    tau = layer.tau
    outs = deque(maxlen=EVERY + 1)
    recs = []
    for k in range(k0, k0 + n):
        t = k * NOMINAL
        cmd = layer.correct(src_of_k(k), data, t, NOMINAL)
        if bridge:
            emu.take(cmd)
            if k % EVERY == 0:
                emu.update()
                susp = suspend_at is not None and suspend_at(k)
                layer.observe(emu.out, t, susp)
            out = emu.out.copy()
        else:
            layer.observe(cmd, t, False)
            out = np.asarray(cmd, float).copy()
        outs.append(out)
        v = (outs[-1] - outs[0]) / ((len(outs) - 1) * NOMINAL) if len(outs) > 1 else np.zeros(7)
        if pad_droop is not None:                       # a droop fixed in the hand: the joint offset that gives this pad offset at the current pose
            J = layer._jacobian(data)
            d_t = np.linalg.pinv(J) @ np.asarray(pad_droop, float)
        else:
            d_t = delta
        meas = out - d_t - tau * v
        data.qpos[qadr] = meas
        mujoco.mj_forward(model, data)
        recs.append({"k": k, "t": t, "C_p": layer.C["p"].copy(), "C_r": layer.C["r"].copy(), "out": out, "meas": meas.copy(),
                     "err": None if layer.last_error is None else layer.last_error.copy(),
                     "expected": None if layer.last_expected is None else layer.last_expected.copy(),
                     "refspeed": None if layer.last_refspeed is None else layer.last_refspeed.copy()})
    return recs


def droop_twist(model, data, layer, qadr, delta):
    """The pad-space droop that the joint droop delta produces at HOME: J delta, (6): position m, rotation rad."""
    data.qpos[qadr] = HOME
    mujoco.mj_forward(model, data)
    J = layer._jacobian(data)
    return J @ np.asarray(delta, float)


def main():
    leak_saved = SAG_LAYER_CFG["sag_leak_tau_s"]
    try:
        test_converges_to_droop()
        test_offset_held_still_under_a_step()
        test_constant_speed_ramp()
        test_held_on_buffer_clamp()
        test_skip_rules()
    finally:
        SAG_LAYER_CFG["sag_leak_tau_s"] = leak_saved
    print(f"{len(FAILS)} failed of the checks above", flush=True)
    return 1 if FAILS else 0


def test_converges_to_droop():
    """A fixed droop (joint 4 sags 4 mrad), at rest: the learned offset converges to the droop (the pad-space droop J delta), not a multiple of
    it, and the error goes to about zero."""
    PAD = np.array([0.0, 0.0, 0.002, 0.0, 0.0, 0.0])   # a droop fixed in the hand: 2 mm, as a pad offset (position m, rotation rad)
    SAG_LAYER_CFG["sag_leak_tau_s"] = 1e9            # the integrator alone; the 5 s fade is checked below
    model, data, layer, qadr, emu, delta = rig(HOME, None)
    target = PAD.copy()
    recs = run(model, data, layer, qadr, emu, delta, lambda k: HOME.copy(), 1500, pad_droop=PAD)
    C = np.concatenate([recs[-1]["C_p"], recs[-1]["C_r"]])
    err = [r["err"] for r in recs if r["err"] is not None][-50:]
    err_norm = float(np.max(np.linalg.norm(np.array(err), axis=1))) if err else float("nan")
    ratio = float(np.dot(C, target) / np.dot(target, target))
    r_nofade = ratio
    RESULTS["converge"] = {"C": np.round(C, 6).tolist(), "droop": np.round(target, 6).tolist(), "ratio": round(ratio, 4), "err_last_mm": round(1000 * err_norm, 4)}
    check("from zero: the learned offset converges to the droop (the pad-space droop J delta), not a multiple of it",
          abs(ratio - 1.0) < 0.1 and np.linalg.norm(C - target) < 0.1 * np.linalg.norm(target),
          f"C/droop ratio {ratio:.3f} (1.000 = the droop)")
    check("from zero: the error goes to about zero (last 50 updates within 0.05 mm)", err_norm < 0.05e-3, f"max error {1000 * err_norm:.4f} mm")
    # with the 5 s fade (SF-118) the fixed point is gain*tau/(1+gain*tau) of the droop: 3 * 5 / 16 = 15/16 (documented, not a pass/fail of the integrator)
    SAG_LAYER_CFG["sag_leak_tau_s"] = 5.0
    model, data, layer, qadr, emu, delta = rig(HOME, None)
    recs = run(model, data, layer, qadr, emu, delta, lambda k: HOME.copy(), 1500, pad_droop=PAD)
    C5 = np.concatenate([recs[-1]["C_p"], recs[-1]["C_r"]])
    r5 = float(np.dot(C5, target) / np.dot(target, target))
    fixed = 3.0 * 5.0 / (1.0 + 3.0 * 5.0)
    predicted = fixed * r_nofade                       # the fade's factor times the integrator's own fixed point (the damping bias, r_nofade)
    RESULTS["fade_fixed_point"] = {"ratio_with_5s_fade": round(r5, 4), "predicted": round(predicted, 4), "integrator_only_ratio": round(r_nofade, 4)}
    check("with the 5 s fade: the offset settles at gain*tau/(1+gain*tau) (15/16) of the integrator's own fixed point", abs(r5 - predicted) < 0.01,
          f"ratio {r5:.4f} (predicted {predicted:.4f})")


def test_offset_held_still_under_a_step():
    """Start at rest with the offset already equal to the droop; step the target by more than one tick's bridge speed limit: the error stays about
    zero and the offset does not change (the fade is switched off here)."""
    SAG_LAYER_CFG["sag_leak_tau_s"] = 1e9
    PAD = np.array([0.0, 0.0, 0.002, 0.0, 0.0, 0.0])
    model, data, layer, qadr, emu, delta = rig(HOME, None)
    target = PAD.copy()
    layer.C = {"p": target[:3].copy(), "r": target[3:].copy()}
    step = HOME.copy()
    step[0] += 0.1                                     # 0.1 rad on joint 1: ten times the bridge's per-update limit
    recs = run(model, data, layer, qadr, emu, delta, lambda k: HOME.copy() if k < 60 else step, 400, pad_droop=PAD)
    C_rest = recs[59]["C_p"].copy()                    # the offset at rest just before the step (the damped law settles off the preset first)
    C0 = np.concatenate([layer.C["p"], layer.C["r"]])
    errs = [np.linalg.norm(r["err"][:3]) for r in recs[60:] if r["err"] is not None]
    max_err_mm = 1000 * max(errs) if errs else float("nan")
    drift_mm = 1000 * float(np.linalg.norm(recs[-1]["C_p"] - C_rest))
    settle_mm = 1000 * float(np.linalg.norm(C_rest - target[:3]))
    RESULTS["step"] = {"max_err_mm": round(max_err_mm, 4), "C_drift_mm": round(drift_mm, 4)}
    check("step larger than the bridge speed limit: the error stays about zero (within 0.5 mm)", max_err_mm <= 0.5,
          f"max position error after the step {max_err_mm:.3f} mm")
    check("step larger than the bridge speed limit: the offset does not change (within 0.1 mm of its value at rest before the step)", drift_mm <= 0.1,
          f"offset change after the step {drift_mm:.4f} mm (settling from the preset before the step: {settle_mm:.4f} mm)")
    _ = C0


def test_constant_speed_ramp():
    """A constant-speed joint ramp (joint 1, 0.1 rad/s) with the correction on: the learned offset stays within 0.5 mm of its at-rest value (the
    motor-lag subtraction works). Also reports the residual lag per joint: measured minus expected, on the ramp."""
    SAG_LAYER_CFG["sag_leak_tau_s"] = 1e9
    model, data, layer, qadr, emu, delta = rig(HOME)
    recs0 = run(model, data, layer, qadr, emu, delta, lambda k: HOME.copy(), 120)
    C_rest = layer.C["p"].copy()
    v = 0.1
    ramp = lambda k: HOME + np.array([v * (k - 120) * NOMINAL, 0, 0, 0, 0, 0, 0])
    recs = run(model, data, layer, qadr, emu, delta, ramp, 450, k0=120)
    dC = 1000 * float(np.max(np.linalg.norm(np.array([r["C_p"] for r in recs]) - C_rest, axis=1)))
    resid = [r["meas"] - r["expected"] for r in recs if r["expected"] is not None and r["k"] > 160]
    resid_mrad = 1000 * np.mean(np.array(resid), axis=0) if resid else np.full(7, np.nan)
    RESULTS["ramp"] = {"max_C_change_mm": round(dC, 4), "residual_mrad_mean": np.round(resid_mrad, 3).tolist()}
    check("constant-speed ramp: the offset stays within 0.5 mm of its at-rest value", dC <= 0.5, f"largest change {dC:.4f} mm")
    _ = recs0


def test_held_on_buffer_clamp():
    """A joint held on its buffer clamp (joint 4 sent at 0 rad, below its buffer edge): the offset does not wind up (it stays bounded, near zero)."""
    SAG_LAYER_CFG["sag_leak_tau_s"] = 1e9
    model, data, layer, qadr, emu, delta = rig(HOME)
    tgt = HOME.copy()
    tgt[3] = 0.0
    recs = run(model, data, layer, qadr, emu, delta, lambda k: tgt.copy(), 900)
    C_end = float(np.linalg.norm(recs[-1]["C_p"])) * 1000
    C_r_end = float(np.linalg.norm(recs[-1]["C_r"]))
    RESULTS["clamp"] = {"C_p_mm": round(C_end, 5), "C_r_rad": round(C_r_end, 7), "out_joint4": round(float(recs[-1]["out"][3]), 5)}
    check("held on its buffer clamp: the offset does not wind up (position within 0.1 mm, rotation within 1e-4 rad)",
          C_end <= 0.1 and C_r_end <= 1e-4, f"|C_p| {C_end:.4f} mm, |C_r| {C_r_end:.6f} rad; joint 4 output {recs[-1]['out'][3]:.4f} rad")


def test_skip_rules():
    """No update without REF(t-1) and REF(t-2); none while the bridge is suspended (the fade still runs); none after a non-finite step 3."""
    SAG_LAYER_CFG["sag_leak_tau_s"] = 1e9
    model, data, layer, qadr, emu, delta = rig(HOME)
    layer.C = {"p": np.array([1e-3, 0, 0]), "r": np.zeros(3)}
    C0 = layer.C["p"].copy()
    layer.correct(HOME.copy(), data, 0.0, NOMINAL)
    check("no reference yet: the first tick does not update the offset", np.allclose(layer.C["p"], C0, atol=1e-9, rtol=0) and layer.skips["no_reference"] >= 1,
          f"skips {layer.skips}")
    layer.observe(HOME.copy(), 1 * NOMINAL, False)
    layer.correct(HOME.copy(), data, 1 * NOMINAL, NOMINAL)
    check("one reference only: the second tick does not update the offset", np.allclose(layer.C["p"], C0, atol=1e-9, rtol=0), "")
    layer.observe(HOME.copy(), 2 * NOMINAL, True)
    layer.correct(HOME.copy(), data, 2 * NOMINAL, NOMINAL)
    check("the bridge suspended: no update on that tick", np.allclose(layer.C["p"], C0, atol=1e-9, rtol=0) and layer.skips["suspended"] >= 1, f"skips {layer.skips}")
    layer.observe(HOME.copy(), 3 * NOMINAL, False)
    real_j = layer._jacobian
    layer._jacobian = lambda data_: np.full((6, 7), np.nan)
    cmd = layer.correct(HOME + 0.01, data, 3 * NOMINAL, NOMINAL)
    layer._jacobian = real_j
    check("non-finite step 3: the source's targets are sent unchanged", np.array_equal(cmd, HOME + 0.01) and layer.nonfinite_count >= 1,
          f"nonfinite count {layer.nonfinite_count}")
    layer.observe(HOME.copy(), 4 * NOMINAL, False)
    C1 = layer.C["p"].copy()
    layer.correct(HOME.copy(), data, 4 * NOMINAL, NOMINAL)
    check("the tick after a non-finite step: no update", np.allclose(layer.C["p"], C1, atol=1e-9, rtol=0) and layer.skips["nonfinite"] >= 1, f"skips {layer.skips}")


if __name__ == "__main__":
    rc = main()
    print("RESULTS", RESULTS, flush=True)
    sys.exit(rc)
