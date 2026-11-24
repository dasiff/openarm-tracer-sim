"""Tests of the solver-free sag layer (SF-122 stage 2, David 2026-11-23, amended 2026-11-24): the learned offset from the error of the expected joints
(a first-order motor filter of the reference, expected_j(t) = expected_j(t-1) + dt/tau_j * (REF_j(t-1) - expected_j(t-1))), the linear step from the
offset to joints, the reference REF(t) = bridge output - dq(t), and the skip rules.

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


def info(name, detail=""):
    """An informational line: printed, never a pass or fail."""
    print(f"INFO  {name}  {detail}", flush=True)


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


def set_control(layer, data, out):
    """The control the actuators receive, as the loop sends it: the bridge output plus the gravity offset tau_ff / kp (the measure rule reads it).
    out: the bridge output (the bridge off: the command itself)."""
    data.ctrl[layer.acts] = np.asarray(out, float) + layer.gff.torques(data)[layer.off:layer.off + 7] / layer.kp_arm


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
        set_control(layer, data, out)
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
        # the pad error of the measured joints against the source's targets (position m, rotation rad, linearised with the pad Jacobian)
        J_now = layer._jacobian(data)
        perr = J_now @ (np.asarray(src_of_k(k), float) - meas)
        # the layer's own error (the measured sag, pad space): predicted minus measured joints, through the pad Jacobian
        lerr = J_now @ layer.last_error if layer.last_error is not None else None
        recs.append({"k": k, "t": t, "C_p": layer.C["p"].copy(), "C_r": layer.C["r"].copy(), "out": out, "meas": meas.copy(),
                     "perr": perr.copy(), "lerr": None if lerr is None else lerr.copy(),
                     "estpad": float(np.linalg.norm((J_now @ layer.est)[:3])),
                     "expected": None if layer.expected is None else layer.expected.copy()})
    return recs


def droop_twist(model, data, layer, qadr, delta):
    """The pad-space droop that the joint droop delta produces at HOME: J delta, (6): position m, rotation rad."""
    data.qpos[qadr] = HOME
    mujoco.mj_forward(model, data)
    J = layer._jacobian(data)
    return J @ np.asarray(delta, float)


def main():
    test_converges_to_droop()
    test_offset_held_still_under_a_step()
    test_constant_speed_ramp()
    test_staircase_policy_like()
    test_single_step_report()
    test_held_on_buffer_clamp()
    test_held_still_bridge_on()
    test_skip_rules()
    print(f"{len(FAILS)} failed of the checks above", flush=True)
    return 1 if FAILS else 0


def test_converges_to_droop():
    """A fixed droop (joint 4 sags 4 mrad), at rest: the learned offset converges to the droop (the pad-space droop J delta), not a multiple of
    it, and the error goes to about zero."""
    PAD = np.array([0.0, 0.0, 0.002, 0.0, 0.0, 0.0])   # a droop fixed in the hand: 2 mm, as a pad offset (position m, rotation rad)
    model, data, layer, qadr, emu, delta = rig(HOME, None)
    target = PAD.copy()
    recs = run(model, data, layer, qadr, emu, delta, lambda k: HOME.copy(), 1500, pad_droop=PAD)
    C = np.concatenate([recs[-1]["C_p"], recs[-1]["C_r"]])
    err = [r["perr"][:3] for r in recs][-50:]
    err_norm = float(np.max(np.linalg.norm(np.array(err), axis=1))) if err else float("nan")
    ratio = float(np.dot(C, target) / np.dot(target, target))
    r_nofade = ratio
    RESULTS["converge"] = {"C": np.round(C, 6).tolist(), "droop": np.round(target, 6).tolist(), "ratio": round(ratio, 4), "err_last_mm": round(1000 * err_norm, 4)}
    check("from zero: the learned offset converges to the droop (the pad-space droop J delta), not a multiple of it",
          abs(ratio - 1.0) < 0.1 and np.linalg.norm(C - target) < 0.1 * np.linalg.norm(target),
          f"C/droop ratio {ratio:.3f} (1.000 = the droop)")
    check("from zero: the error goes to about zero (last 50 updates within 0.05 mm)", err_norm < 0.05e-3, f"max error {1000 * err_norm:.4f} mm")
    RESULTS["converge_ratio"] = round(r_nofade, 4)


def test_offset_held_still_under_a_step():
    """Start at rest with the offset already equal to the droop; step the target by more than one tick's bridge speed limit: the error stays about
    zero and the offset does not change (the fade is switched off here)."""
    PAD = np.array([0.0, 0.0, 0.002, 0.0, 0.0, 0.0])
    model, data, layer, qadr, emu, delta = rig(HOME, None)
    target = PAD.copy()
    J0 = layer._jacobian(data)
    layer.est = np.linalg.pinv(J0) @ target            # the measure rule's state: the joint droop that the pad droop needs (the learned state is est)
    step = HOME.copy()
    step[0] += 0.1                                     # 0.1 rad on joint 1: ten times the bridge's per-update limit
    recs = run(model, data, layer, qadr, emu, delta, lambda k: HOME.copy() if k < 60 else step, 400, pad_droop=PAD)
    C_rest = recs[59]["C_p"].copy()                    # the offset at rest just before the step (the damped law settles off the preset first)
    errs = [np.linalg.norm(r["lerr"][:3]) for r in recs[60:] if r["lerr"] is not None]
    max_err_mm = 1000 * max(errs) if errs else float("nan")
    drift_mm = 1000 * float(np.linalg.norm(recs[-1]["C_p"] - C_rest))
    settle_mm = 1000 * float(np.linalg.norm(C_rest - target[:3]))
    RESULTS["step"] = {"max_err_mm": round(max_err_mm, 4), "C_drift_mm": round(drift_mm, 4)}
    check("step larger than the bridge speed limit: the error stays about zero (within 0.5 mm)", max_err_mm <= 0.5,
          f"max position error after the step {max_err_mm:.3f} mm")
    check("step larger than the bridge speed limit: the offset does not change (within 0.1 mm of its value at rest before the step)", drift_mm <= 0.1,
          f"offset change after the step {drift_mm:.4f} mm (settling from the preset before the step: {settle_mm:.4f} mm)")


def test_constant_speed_ramp():
    """(2a, David 2026-11-24) A constant-speed joint ramp (joint 1, 0.1 rad/s) with the correction on: the learned offset stays within 0.5 mm of its
    at-rest value (the motor filter's lag is removed correctly). Also reports the residual lag per joint: measured minus expected, on the ramp."""
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


def test_staircase_policy_like():
    """(2b, David 2026-11-24) A staircase reference like the policy's stream: a new 30 Hz target every 5 ticks (joints 1 and 2), with a fixed droop
    in the hand (2 mm). The offset, converged at rest first, must stay within 0.5 mm of its rest value through the staircase (the fade is off)."""
    PAD = np.array([0.0, 0.0, 0.002, 0.0, 0.0, 0.0])
    model, data, layer, qadr, emu, delta = rig(HOME)

    def src(k):
        if k < 1500:
            return HOME.copy()
        m = (k - 1500) // 5                            # one target per 5 ticks, i.e. 30 Hz
        return HOME + np.array([0.04 * np.sin(0.3 * m), 0.03 * np.cos(0.2 * m), 0, 0, 0, 0, 0])

    recs = run(model, data, layer, qadr, emu, delta, src, 2400, pad_droop=PAD)
    C_rest = recs[1499]["C_p"].copy()
    C_stream = np.array([r["C_p"] for r in recs[1500:]])
    dev_mm = 1000 * float(np.max(np.linalg.norm(C_stream - C_rest, axis=1)))
    R_rest = recs[1499]["C_r"].copy()
    dev_r = 1000 * float(np.max(np.linalg.norm(np.array([r["C_r"] for r in recs[1500:]]) - R_rest, axis=1)))
    RESULTS["staircase"] = {"max_C_p_dev_mm": round(dev_mm, 4), "max_C_r_dev_mrad": round(dev_r, 4), "C_rest_mm": np.round(1000 * C_rest, 3).tolist()}
    check("staircase reference (30 Hz targets, each held for 5 ticks), fixed droop: the offset stays within 0.5 mm of its rest value",
          dev_mm <= 0.5, f"largest position change {dev_mm:.4f} mm (rotation {dev_r:.3f} mrad, reported)")


def test_single_step_report():
    """(2c, David 2026-11-24) REPORT ONLY, not a pass criterion: a single 0.02 rad step on all seven joints at once. Reports the largest difference
    between the filter's expected joint and the measured joint, per joint, after the step (joints 3 and 4 ring)."""
    model, data, layer, qadr, emu, delta = rig(HOME)
    step = HOME + 0.02
    recs = run(model, data, layer, qadr, emu, delta, lambda k: HOME.copy() if k < 300 else step.copy(), 600)
    diffs = np.array([np.abs(r["expected"] - r["meas"]) for r in recs if r["expected"] is not None and r["k"] >= 300])
    per_joint = 1000 * diffs.max(axis=0)
    RESULTS["single_step_report_mrad"] = np.round(per_joint, 3).tolist()
    info("single 0.02 rad step, all joints: largest |expected - measured| per joint (mrad, report only)", str(np.round(per_joint, 3).tolist()))


def test_held_on_buffer_clamp():
    """A joint held on its buffer clamp (joint 4 sent at 0 rad, below its buffer edge): the offset does not wind up (it stays bounded, near zero)."""
    model, data, layer, qadr, emu, delta = rig(HOME)
    tgt = HOME.copy()
    tgt[3] = 0.0
    recs = run(model, data, layer, qadr, emu, delta, lambda k: tgt.copy(), 900)
    C_end = float(np.linalg.norm(recs[-1]["C_p"])) * 1000
    C_r_end = float(np.linalg.norm(recs[-1]["C_r"]))
    RESULTS["clamp"] = {"C_p_mm": round(C_end, 5), "C_r_rad": round(C_r_end, 7), "out_joint4": round(float(recs[-1]["out"][3]), 5)}
    check("held on its buffer clamp: the offset does not wind up (position within 0.1 mm, rotation within 1e-4 rad)",
          C_end <= 0.1 and C_r_end <= 1e-4, f"|C_p| {C_end:.4f} mm, |C_r| {C_r_end:.6f} rad; joint 4 output {recs[-1]['out'][3]:.4f} rad")


def _held_bridge_on(src_fn, n=600):
    """The arm held still (its measured joints never move), the bridge emulated (every third tick). Returns |C_p| (m) and |C_r| (rad) per tick."""
    model, data, layer, qadr, emu, delta = rig(HOME)
    cp, cr = [], []
    for k in range(n):
        t = k * NOMINAL
        cmd = layer.correct(src_fn(k), data, t, NOMINAL)
        emu.take(cmd)
        if k % EVERY == 0:
            emu.update()
            layer.observe(emu.out, t, False)
        set_control(layer, data, emu.out)
        cp.append(float(np.linalg.norm(layer.C["p"])))
        cr.append(float(np.linalg.norm(layer.C["r"])))
    return np.array(cp), np.array(cr)


def _zero_mean_src(k):
    """(decision 1, David 2026-11-24) A zero-mean joint sweep (joints 1 and 2) of the source's targets: nothing the held arm cannot reach."""
    return HOME + np.array([0.03 * np.sin(0.05 * k), 0.02 * np.cos(0.05 * k), 0, 0, 0, 0, 0])


def _unreachable_src(k):
    """The same sweep plus a constant +0.01 rad on joint 3: a target the held arm can never reach (informational only)."""
    return HOME + np.array([0.03 * np.sin(0.05 * k), 0.02 * np.cos(0.05 * k), 0.01, 0, 0, 0, 0])


def test_held_still_bridge_on():
    """(decision 1, David 2026-11-24) The arm held still, the bridge on (emulated). PASS: the zero-mean sweep keeps the offset bounded (|C_p| <= 10 mm,
    |C_r| <= 10 mrad, no growth late in the run). INFORMATIONAL: the sweep with a constant +0.01 rad on joint 3, which the held arm cannot reach: it
    reports the offset reaching the clip (not a pass criterion)."""
    cp, cr = _held_bridge_on(_zero_mean_src)
    max_p_mm, max_r_mrad = 1000 * cp.max(), 1000 * cr.max()
    grew = bool(cp[-200:].max() > 1.5 * cp[100:300].max() + 1e-4 or cr[-200:].max() > 1.5 * cr[100:300].max() + 1e-4)   # 0.1 mm / 0.1 mrad slack
    RESULTS["held_zero_mean"] = {"max_C_p_mm": round(max_p_mm, 4), "max_C_r_mrad": round(max_r_mrad, 4), "growth_late": grew}
    check("held still, bridge on, zero-mean sweep: the offset stays bounded (|C_p| <= 10 mm, |C_r| <= 10 mrad, no growth late in the run)",
          max_p_mm <= 10.0 and max_r_mrad <= 10.0 and not grew,
          f"max |C_p| {max_p_mm:.3f} mm, max |C_r| {max_r_mrad:.3f} mrad, growth late in the run {grew}")
    cp_u, cr_u = _held_bridge_on(_unreachable_src)
    RESULTS["held_unreachable"] = {"max_C_p_mm": round(1000 * cp_u.max(), 4), "max_C_r_mrad": round(1000 * cr_u.max(), 4)}
    info("held still, bridge on, constant +0.01 rad on joint 3 (unreachable target): the offset reaches the clip (informational)",
         f"max |C_p| {1000 * cp_u.max():.3f} mm, max |C_r| {1000 * cr_u.max():.3f} mrad")


def test_skip_rules():
    """Skip rules of the measure rule (stage 2): no update of the estimate on the first tick (no prediction yet); none while the bridge is
    suspended; a non-finite hand move sends the source's targets unchanged; no update on the tick after a non-finite step. (The integrator's
    'two references' requirement is gone with the integrator: the measure rule updates from the second tick on.)"""
    model, data, layer, qadr, emu, delta = rig(HOME)
    layer.est = np.array([1e-3, 0, 0, 0, 0, 0, 0])      # the measure rule's state (the estimate), preset so a skipped update is visible
    C0 = layer.est.copy()
    layer.correct(HOME.copy(), data, 0.0, NOMINAL)
    check("no prediction yet: the first tick does not update the estimate", np.allclose(layer.est, C0, atol=1e-12, rtol=0) and layer.skips["no_reference"] >= 1,
          f"skips {layer.skips}")
    layer.observe(HOME.copy(), 1 * NOMINAL, True)
    C1 = layer.est.copy()
    layer.correct(HOME.copy(), data, 1 * NOMINAL, NOMINAL)
    check("the bridge suspended: no update on that tick", np.allclose(layer.est, C1, atol=1e-12, rtol=0) and layer.skips["suspended"] >= 1, f"skips {layer.skips}")
    layer.observe(HOME.copy(), 2 * NOMINAL, False)
    real_j = layer._jacobian
    layer._jacobian = lambda data_: np.full((6, 7), np.nan)
    cmd = layer.correct(HOME + 0.01, data, 2 * NOMINAL, NOMINAL)
    layer._jacobian = real_j
    check("non-finite hand move: the source's targets are sent unchanged", np.array_equal(cmd, HOME + 0.01) and layer.nonfinite_count >= 1,
          f"nonfinite count {layer.nonfinite_count}")
    layer.observe(HOME.copy(), 3 * NOMINAL, False)
    C2 = layer.est.copy()
    layer.correct(HOME.copy(), data, 3 * NOMINAL, NOMINAL)
    check("the tick after a non-finite step: no update", np.allclose(layer.est, C2, atol=1e-12, rtol=0) and layer.skips["nonfinite"] >= 1, f"skips {layer.skips}")


if __name__ == "__main__":
    rc = main()
    print("RESULTS", RESULTS, flush=True)
    sys.exit(rc)
