"""Synthetic tests for the sag correction as a control-loop layer (stage 2, David 2026-11-24: the measure rule with T = 2 s is production, SF-127):
the hold during a held-mass ramp, the hold during a helper pause, the setting OFF, the gripper-open reset (the events the loop can see), the
skip rules and the logging. The arm is the robot-only model at its home pose; nothing is stepped in time.

Retired in stage 2 (they tested the removed integrator and its 5 s leak, SF-118): S1 (the layer equals the old in-script trim), L1, L2 and L3
(the leak's steady state, alternating errors, decay), and the SF-118 (a) check that a standing command integrates and leaks. The state of the
measure rule is the estimate (layer.est, joints); layer.C is the hand-space equivalent of dq (logging).

    python experiments/sag_layer_test.py      (exit code 0 when every case passes)
"""

import os
import sys

import numpy as np

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, REPO)

import mujoco  # noqa: E402

from src import held_mass_ramp as hmr  # noqa: E402
import src.sag_layer as sag_layer  # noqa: E402
from src.robot_helpers import arm_ids  # noqa: E402
from src.sag_layer import SagLayer, correct_command  # noqa: E402
from src.task_config import PICK_PLACE_CFG, SAG_LAYER_CFG, TASK_CFG  # noqa: E402

HOME = np.asarray(TASK_CFG["home_q_left"], float)
NOMINAL = 1.0 / 150.0
T_CONST = float(SAG_LAYER_CFG["measure_T_s"])


def robot():
    model = mujoco.MjModel.from_xml_path(os.path.join(REPO, "models", "robot", "combined_robot.xml"))
    data = mujoco.MjData(model)
    data.qpos[arm_ids(model, "left")["qadr"]] = HOME
    mujoco.mj_forward(model, data)
    return model, data


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    return bool(ok)


def source_targets(data, offset):
    """Joint targets a source would send: the home pose plus an offset (so the hand is off the measured pose)."""
    return HOME + np.asarray(offset, float)


def sweep(k):
    """Targets that change on every command (a sine sweep), so the cases that check the estimate send these."""
    return source_targets(None, [0.03 * np.sin(0.05 * k), 0.0, 0, 0, 0, 0, 0])


def _control(layer, data, cmd):
    """The control the actuators receive, as the loop sends it: the command plus the gravity offset tau_ff / kp."""
    data.ctrl[layer.acts] = np.asarray(cmd, float) + layer.gff.torques(data)[layer.off:layer.off + 7] / layer.kp_arm


def _cc(layer, q, data, t, dt):
    """The layer's command, the control set on the actuators, and the bridge output fed back as the layer sees it (bridge off: the command itself)."""
    cmd = layer.correct(q, data, t, dt)
    _control(layer, data, cmd)
    layer.observe(cmd, t, False)
    return cmd


def _cc_cmd(layers, cmd, data, t, dt):
    """correct_command for every arm of a command, then the control and the bridge output fed back to each layer (as the loop does)."""
    out = correct_command(layers, cmd, data, t, dt)
    if out.get("mode") == "arms" and out.get("arm_targets"):
        for s, q in out["arm_targets"].items():
            _control(layers[s], data, np.array(q, float))
            layers[s].observe(np.array(q, float), t, False)
    return out


def _skipped(layer):
    return sum(layer.skips.values())


def _arm_cmd(q, manip):
    """A command as the loop builds it: an arms command with the safety filter's manipulation tag (the sag layer does not read it)."""
    return {"mode": "arms", "arm_targets": {"left": [float(v) for v in q]}, "gripper": {"left": 0.044}, "manip": manip, "period_steps": 3}


def _pair(model, data):
    layer = SagLayer(model, data, "left", HOME)
    return layer, {"left": layer, "right": SagLayer(model, data, "right", np.asarray(TASK_CFG["home_q_right"], float))}


def _moving(layers, data, k0, n, manip=False):
    """n commands from tick k0 that all change the targets (a sine sweep). Returns the next tick."""
    for k in range(k0, k0 + n):
        q = source_targets(data, [0.03 * np.sin(0.05 * k), 0.02 * np.cos(0.05 * k), 0.01, 0, 0, 0, 0])
        _cc_cmd(layers, _arm_cmd(q, manip), data, k * NOMINAL, NOMINAL)
    return k0 + n


def case_b(model, data):
    """The estimate does not change while a held-mass ramp runs; it resumes after the ramp."""
    layer = SagLayer(model, data, "left", HOME)
    hmr.reset(model)
    hmr.start_rate(model, "left", 0.05, 0.0, 0)
    before = layer.est.copy()
    t = 0.0
    for k in range(60):
        _cc(layer, sweep(k), data, t, NOMINAL)
        t += NOMINAL
    frozen = np.array_equal(layer.est, before)
    hmr.stop_rate(model, "left", t)
    for k in range(60):
        _cc(layer, sweep(k), data, t, NOMINAL)
        t += NOMINAL
    resumed = not np.array_equal(layer.est, before)
    hmr.reset(model)
    ok = frozen and resumed
    return check("(S2) the estimate is frozen during a held-mass ramp, updating again after it", ok, f"frozen during the ramp: {frozen}; resumed after: {resumed}")


def case_c(model, data):
    """A helper pause: no commands, nothing updated (the estimate holds its value); the first command after the pause updates it with one nominal
    period, not the pause (the low-pass step is (NOMINAL / T) times the measured sag minus the estimate)."""
    layer = SagLayer(model, data, "left", HOME)
    t = 0.0
    for k in range(30):
        _cc(layer, sweep(k), data, t, NOMINAL)
        t += NOMINAL
    held = layer.est.copy()
    t += 5.0                                             # the pause: five simulated seconds with no commands
    before = layer.est.copy()
    _cc(layer, sweep(30), data, t, NOMINAL)
    expected = before + (NOMINAL / T_CONST) * (layer.last_sag - before)
    ok = np.array_equal(held, before) and bool(np.allclose(layer.est, expected, atol=1e-12, rtol=0))
    return check("(S3) holds its value during a helper pause; the first command after the pause updates it with one nominal period", ok,
                 f"estimate step equals (nominal dt / T) x (sag - estimate): {np.allclose(layer.est, expected, atol=1e-12, rtol=0)}")


def case_d(model, data):
    """With the setting OFF the layer passes the source's targets through unchanged, and nothing is updated."""
    layer = SagLayer(model, data, "left", HOME)
    q = source_targets(data, [0.05, -0.03, 0.02, 0.01, 0.0, 0.0, 0.0])
    SAG_LAYER_CFG["enabled"] = False
    try:
        out = layer.correct(q, data, 0.0, NOMINAL)
        out2 = layer.correct(q, data, NOMINAL, NOMINAL)
    finally:
        SAG_LAYER_CFG["enabled"] = True
    ok = np.array_equal(out, q) and np.array_equal(out2, q) and not np.any(layer.est) and not np.any(layer.C["p"]) and not np.any(layer.C["r"])
    return check("(S4) the setting OFF passes the targets through unchanged; nothing is updated", ok, f"targets unchanged: {np.array_equal(out, q)}")


def case_e(model, data):
    """The reset rule (M263): the open-catch release arms the watch, and the learned correction is reset when the release completes (the fingers at
    rest, at least close_min_s after the release). Before close_min_s nothing resets."""
    layer = SagLayer(model, data, "left", HOME)
    layer.est = np.array([0.001, 0.002, 0.003, 0.0, 0.0, 0.0, 0.0])
    hmr.reset(model)
    hmr.set_target(model, "left", 0.0, 0.0, 0)
    rec = hmr._record(model, "left")
    rec["changing"] = True
    ids = arm_ids(model, "left")
    fq = int(model.jnt_qposadr[ids["finger_jnt"][0]])
    data.qpos[fq] = sag_layer.TASK_OPEN                  # the fingers are open and at rest
    data.qvel[:] = 0.0
    layer.release_open(0.0)
    layer.track_open(data, 0.5)                          # before close_min_s: no reset
    early = layer.est.copy()
    layer.track_open(data, 1.6)                          # stalled and past close_min_s: the reset
    reset_ok = not np.any(layer.est) and rec["changing"] is False and layer.reset_at is not None and layer.reset_at[1] == "release_open_done"
    kept = bool(np.array_equal(early, np.array([0.001, 0.002, 0.003, 0.0, 0.0, 0.0, 0.0])))
    hmr.reset(model)
    data.qpos[fq] = 0.0
    ok = kept and reset_ok
    return check("(b) reset rule: the release arms the watch; nothing resets before close_min_s; the reset comes when the release completes", ok,
                 f"kept before close_min_s {kept}; reset at completion {reset_ok}")


def case_g(model, data):
    """A release that completes is one reset: after it, the watch is not armed again by the layer (only a new release arms it)."""
    layer = SagLayer(model, data, "left", HOME)
    layer.est = np.array([0.004, 0, 0, 0, 0, 0, 0])
    ids = arm_ids(model, "left")
    fq = int(model.jnt_qposadr[ids["finger_jnt"][0]])
    data.qpos[fq] = sag_layer.TASK_OPEN
    data.qvel[:] = 0.0
    layer.release_open(0.0)
    t = 0.0
    while layer.reset_at is None and t < 5.0:            # the release completes after the stall dwell and close_min_s
        t += NOMINAL
        layer.track_open(data, t)
    first = layer.reset_at is not None
    layer.est = np.array([0.002, 0, 0, 0, 0, 0, 0])      # the estimate builds again after the reset
    for k in range(200):
        t += NOMINAL
        layer.track_open(data, t)                        # 200 ticks with the fingers open at rest, no release
    ok = first and bool(np.array_equal(layer.est, np.array([0.002, 0, 0, 0, 0, 0, 0]))) and sum(1 for e in layer.log if e["kind"] == "release_reset") == 1
    return check("(g) one release, one reset: no second reset without a new release", ok, f"estimate kept after the first reset: {ok}")


def case_h(model, data):
    """Logging only (M263): the trace values have the declared length, the reset and the guard counter appear in them, and the signal trace header
    has the sag columns in the same order as its rows (per hand, after the verdicts)."""
    from src import signal_trace
    layer = SagLayer(model, data, "left", HOME)
    sag_layer.PLAN_TRACE["left"] = {"label": "pick: OVER", "p": layer.arm.pad_pose(data)[0], "R": layer.arm.pad_pose(data)[1]}
    q = source_targets(data, [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    layer.correct(q, data, 0.0, NOMINAL)
    layer.correct(q, data, 0.0, NOMINAL)                 # the same tick again: the guard fires
    vals = layer.trace_values(NOMINAL)                  # the row written at the start of the next tick: it shows the previous tick
    cols = signal_trace.columns()
    width_ok = len(vals) == len(sag_layer.TRACE_COLUMNS) and vals[0] == "pick: OVER" and vals[-1] == "1"
    left_at = cols.index("left_sag_label")
    right_at = cols.index("right_sag_label")
    order_ok = left_at == cols.index("left_still_verdict") + 1 and right_at == cols.index("right_still_verdict") + 1 and cols.index("right_finger1_m") == left_at + len(sag_layer.TRACE_COLUMNS)
    layer.track_open(data, 0.0)                          # nothing armed: no reset value
    layer.reset_at = (0.5, "release_open_done")
    reset_value = layer.trace_values(0.5 + NOMINAL)[sag_layer.TRACE_COLUMNS.index("reset")] == "release_open_done"
    reset_once = layer.trace_values(0.5 + 3 * NOMINAL)[sag_layer.TRACE_COLUMNS.index("reset")] == ""
    ok = width_ok and order_ok and reset_value and reset_once
    sag_layer.PLAN_TRACE.pop("left", None)
    return check("(d) logging: the trace values and header columns are written; the guard counter and the reset cause appear", ok,
                 f"values {len(vals)} of {len(sag_layer.TRACE_COLUMNS)}; guard counter {vals[-1]}; header order {order_ok}; reset cause {reset_value}")


def case_i(model, data):
    """Wiring check (M263): the ordinary gripper-open command in the control loop no longer arms the watch; only the release does (the call
    sites in src/control_loop.py: the release callback calls release_open, and the sag correction does not)."""
    text = open(os.path.join(REPO, "src", "control_loop.py"), encoding="utf-8").read()
    a = text.index("    def sag_correct(cmd):")
    b = text.index("    def bridge_take(cmd):")
    body = text[a:b]
    ok = "release_open" not in body and "start_open" not in body and text.count("sag[s_].release_open(t)") == 1 and text.count("start_open") == 0
    return check("(i) wiring: only the open-catch release arms the watch (sag_correct does not; one release call site)", ok,
                 f"release call sites {text.count('sag[s_].release_open(t)')}; sag_correct arms the watch: {('release_open' in body) or ('start_open' in body)}")


def case_f(model, data):
    """Two commands for the same arm in one tick: the second does not update or divide by zero; the outputs stay finite and the state is kept."""
    layer = SagLayer(model, data, "left", HOME)
    q = source_targets(data, [0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    first = layer.correct(q, data, 0.0, NOMINAL)
    again = layer.correct(q, data, 0.0, NOMINAL)
    nxt = layer.correct(q, data, NOMINAL, NOMINAL)
    ok = bool(np.all(np.isfinite(again)) and np.array_equal(again, first) and np.all(np.isfinite(nxt)))
    return check("(S5) two commands in one tick: no update on the second, outputs stay finite", ok,
                 f"second command equals the first: {np.array_equal(again, first)}; later output finite: {bool(np.all(np.isfinite(nxt)))}")


def case_a_open(model, data):
    """(a) no reset on an ordinary gripper-open command: the fingers open at rest, the estimate kept, and no release (only the release arms it)."""
    layer = SagLayer(model, data, "left", HOME)
    layer.est = np.array([0.003, 0, 0, 0, 0, 0, 0])
    ids = arm_ids(model, "left")
    fq = int(model.jnt_qposadr[ids["finger_jnt"][0]])
    data.qpos[fq] = sag_layer.TASK_OPEN
    data.qvel[:] = 0.0
    for k in range(30):
        layer.track_open(data, k * NOMINAL)              # the loop's per-tick watch, with no release armed
    ok = bool(np.array_equal(layer.est, np.array([0.003, 0, 0, 0, 0, 0, 0]))) and layer.reset_at is None
    return check("(a) no reset on an ordinary gripper-open command (no release, the fingers open at rest)", ok, f"estimate kept {ok}")


def case_c_periodic(model, data):
    """(c) no periodic reset: 10 s of control ticks with the gripper open and at rest, no release, the estimate updating; no reset at all."""
    layer = SagLayer(model, data, "left", HOME)
    ids = arm_ids(model, "left")
    fq = int(model.jnt_qposadr[ids["finger_jnt"][0]])
    data.qpos[fq] = sag_layer.TASK_OPEN
    data.qvel[:] = 0.0
    t = 0.0
    n_ticks = int(10.0 / NOMINAL)
    skipped0 = _skipped(layer)
    for k in range(n_ticks):
        _cc(layer, sweep(k), data, t, NOMINAL)
        layer.track_open(data, t)
        t += NOMINAL
    updates = n_ticks - (_skipped(layer) - skipped0)
    # the first tick has no prediction yet (the designed no_reference skip); every other tick updates the estimate
    ok = layer.reset_at is None and not any(e["kind"] == "release_reset" for e in layer.log) and updates == n_ticks - 1 and np.any(layer.est)
    return check("(c) no periodic reset over 10 s with the gripper open (the estimate keeps updating)", ok,
                 f"resets {0 if layer.reset_at is None else 1}; estimate updates {updates} of {n_ticks} ticks; |est| {float(np.linalg.norm(layer.est)):.4f} rad")


def case_s5_a(model, data):
    """SF-118 (replaces the S5 freeze test): a standing command does not freeze the estimate. 60 standing commands after a moving run each update
    the estimate (none is skipped)."""
    layer, layers = _pair(model, data)
    k = _moving(layers, data, 0, 120)
    q = source_targets(data, [0.01, 0.0, 0.0, 0, 0, 0, 0])
    cs = []
    skipped0 = _skipped(layer)
    for j in range(60):
        _cc_cmd(layers, _arm_cmd(q, False), data, (k + j) * NOMINAL, NOMINAL)
        cs.append(layer.est.copy())
    updates = 60 - (_skipped(layer) - skipped0)
    changed = all(not np.array_equal(cs[i], cs[i - 1]) for i in range(1, len(cs)))
    ok = updates == 60 and changed
    return check("SF-118 (a) a standing command does not freeze the estimate: all 60 standing commands update it", ok,
                 f"estimate updates in 60 standing commands {updates} (expected 60); estimate changed on every one {changed}")


def case_s5_d(model, data):
    """S5 (d): no source gate. The same command stream gives the same corrected output whatever the manipulation tag says (the tag is the
    safety filter's, not the layer's), and a non-manipulation source's moving command is corrected."""
    la, lla = _pair(model, data)
    model2, data2 = robot()                              # the second stream has its own simulator state (the control of one must not reach the other)
    lb, llb = _pair(model2, data2)
    same, changed = True, False
    for j in range(120):
        q = source_targets(data, [0.03 * np.sin(0.05 * j), 0.02 * np.cos(0.05 * j), 0.01, 0, 0, 0, 0])
        out_a = np.array(_cc_cmd(lla, _arm_cmd(q, False), data, j * NOMINAL, NOMINAL)["arm_targets"]["left"])
        out_b = np.array(_cc_cmd(llb, _arm_cmd(q, True), data2, j * NOMINAL, NOMINAL)["arm_targets"]["left"])
        same = same and np.array_equal(out_a, out_b)
        changed = changed or not np.allclose(out_a, q)
    ok = same and changed and float(np.linalg.norm(la.est)) > 0.0
    return check("S5 (d) no source gate: the tag does not change the correction; a non-manipulation source's moving command is corrected", ok,
                 f"same output with either tag {same}; corrected {changed}; |est| {float(np.linalg.norm(la.est)):.4f} rad")


def case_s5_f(model, data):
    """S5 (f): the source-name gate is gone from the layer: sag_layer.py names no source and has no manipulation gate."""
    import inspect
    text = inspect.getsource(sag_layer)
    ok = '"vla"' not in text and "MANIPULATION_SOURCES" not in text and "manipulation(" not in text and "manip_active" not in text
    return check("S5 (f) wiring: sag_layer.py has no source-name gate (no 'vla', no MANIPULATION_SOURCES, no manipulation step)", ok,
                 f"source-name gate text present: {not ok}")


def main():
    model, data = robot()
    results = [case_a_open(model, data), case_b(model, data), case_c(model, data), case_c_periodic(model, data), case_d(model, data),
               case_e(model, data), case_f(model, data), case_g(model, data), case_h(model, data), case_i(model, data),
               case_s5_a(model, data), case_s5_d(model, data), case_s5_f(model, data)]
    print(f"{sum(results)} of {len(results)} passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
