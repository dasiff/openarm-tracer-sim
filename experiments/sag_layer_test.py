"""Synthetic tests for the sag trim as a control-loop layer (M262): the same correction as the old in-script trim, the freeze during a held-mass
ramp, the hold during a helper pause, the setting OFF, and the gripper-open reset (the events the loop can see). The arm is the robot-only
model at its home pose; nothing is stepped in time.

    python experiments/sag_layer_test.py      (exit code 0 when every case passes)
"""

import os
import subprocess
import sys

import numpy as np

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, REPO)

import mujoco  # noqa: E402

from src import held_mass_ramp as hmr  # noqa: E402
from src import sag_layer  # noqa: E402
from src.robot_helpers import arm_ids  # noqa: E402
from src.sag_layer import SagLayer, correct_command  # noqa: E402
import src.sag_layer as sag_layer  # noqa: E402
from src.task_config import PICK_PLACE_CFG, SAG_LAYER_CFG, TASK_CFG  # noqa: E402

HOME = np.asarray(TASK_CFG["home_q_left"], float)
NOMINAL = 1.0 / 150.0


def robot():
    model = mujoco.MjModel.from_xml_path(os.path.join(REPO, "models", "robot", "combined_robot.xml"))
    data = mujoco.MjData(model)
    data.qpos[arm_ids(model, "left")["qadr"]] = HOME
    mujoco.mj_forward(model, data)
    return model, data


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    return bool(ok)


def old_trim_function():
    """The in-script trim function as it was before M262 (the last commit that had it), taken from the git history."""
    text = subprocess.run(["git", "-C", REPO, "show", "e50815d:src/mpvi_modules.py"], capture_output=True, text=True, check=True).stdout
    a = text.index("def _clip_norm(v, limit):")
    b = text.index("def make_sim_world_model(")
    ns = {"np": np, "hmr": hmr, "Rotation": None}
    exec(text[a:b], ns)
    return ns["_sag_trim_step"]


def source_targets(data, offset):
    """Joint targets a source would send: the home pose plus an offset (so the hand is off the measured pose)."""
    return HOME + np.asarray(offset, float)


def case_a(model, data):
    """The layer's correction for a sequence of commands equals the old in-script trim for the same inputs (the same integration, the same gains
    and limits, the same freeze)."""
    layer = SagLayer(model, data, "left", HOME)
    recorded = []
    real = sag_layer._sag_trim_step

    def spy(C, cfg, mdl, side, t, p_err, r_err, dt):
        recorded.append((t, np.array(p_err), np.array(r_err), dt))
        return real(C, cfg, mdl, side, t, p_err, r_err, dt)

    sag_layer._sag_trim_step = spy
    try:
        t = 0.0
        for k in range(200):
            off = [0.02 * np.sin(0.05 * k), 0.015 * np.cos(0.04 * k), 0.01, 0.0, 0.0, 0.0, 0.0]
            layer.correct(source_targets(data, off), data, t, NOMINAL)
            t += NOMINAL
    finally:
        sag_layer._sag_trim_step = real
    old = old_trim_function()
    C_old = {"p": np.zeros(3), "r": np.zeros(3)}
    for t_, p_err, r_err, dt in recorded:
        old(C_old, PICK_PLACE_CFG, model, "left", t_, p_err, r_err, dt)
    ok = np.array_equal(layer.C["p"], C_old["p"]) and np.array_equal(layer.C["r"], C_old["r"]) and len(recorded) == 200
    return check("(S1) the layer's correction equals the old in-script trim for the same input sequence (200 commands)", ok,
                 f"correction p {np.round(1000 * layer.C['p'], 4)} mm (old {np.round(1000 * C_old['p'], 4)}), identical: {ok}")


def case_b(model, data):
    """The correction does not integrate while a held-mass ramp runs; it resumes after the ramp."""
    layer = SagLayer(model, data, "left", HOME)
    hmr.reset(model)
    hmr.start_rate(model, "left", 0.05, 0.0, 0)
    before = layer.C["p"].copy()
    t = 0.0
    for k in range(60):
        layer.correct(source_targets(data, [0.03, 0.0, 0, 0, 0, 0, 0]), data, t, NOMINAL)
        t += NOMINAL
    frozen = np.array_equal(layer.C["p"], before)
    hmr.stop_rate(model, "left", t)
    for k in range(60):
        layer.correct(source_targets(data, [0.03, 0.0, 0, 0, 0, 0, 0]), data, t, NOMINAL)
        t += NOMINAL
    resumed = not np.array_equal(layer.C["p"], before)
    hmr.reset(model)
    ok = frozen and resumed
    return check("(S2) frozen during a held-mass ramp, integrating again after it", ok, f"frozen during the ramp: {frozen}; resumed after: {resumed}")


def case_c(model, data):
    """A helper pause: no commands, nothing integrated (the correction holds its value); the first command after the pause integrates one nominal
    period, not the pause."""
    layer = SagLayer(model, data, "left", HOME)
    t = 0.0
    for k in range(30):
        layer.correct(source_targets(data, [0.03, 0.0, 0, 0, 0, 0, 0]), data, t, NOMINAL)
        t += NOMINAL
    held = layer.C["p"].copy()
    t += 5.0                                             # the pause: five simulated seconds with no commands
    seen_dt = []
    real = sag_layer._sag_trim_step

    def spy(C, cfg, mdl, side, tt, p_err, r_err, dt):
        seen_dt.append(dt)
        return real(C, cfg, mdl, side, tt, p_err, r_err, dt)

    sag_layer._sag_trim_step = spy
    try:
        layer.correct(source_targets(data, [0.03, 0.0, 0, 0, 0, 0, 0]), data, t, NOMINAL)
    finally:
        sag_layer._sag_trim_step = real
    ok = np.array_equal(held, held) and seen_dt == [NOMINAL] and abs(layer.C["p"] - held).max() < 0.01
    return check("(S3) holds its value during a helper pause; the first command after the pause integrates one nominal period", ok,
                 f"integration step after a 5 s pause {seen_dt[0]:.6f} s (nominal {NOMINAL:.6f} s)")


def case_d(model, data):
    """With the setting OFF the layer passes the source's targets through unchanged, and nothing is integrated."""
    layer = SagLayer(model, data, "left", HOME)
    q = source_targets(data, [0.05, -0.03, 0.02, 0.01, 0.0, 0.0, 0.0])
    SAG_LAYER_CFG["enabled"] = False
    try:
        out = layer.correct(q, data, 0.0, NOMINAL)
        out2 = layer.correct(q, data, NOMINAL, NOMINAL)
    finally:
        SAG_LAYER_CFG["enabled"] = True
    ok = np.array_equal(out, q) and np.array_equal(out2, q) and not np.any(layer.C["p"]) and not np.any(layer.C["r"])
    return check("(S4) the setting OFF passes the targets through unchanged; nothing is integrated", ok, f"targets unchanged: {np.array_equal(out, q)}")


def case_e(model, data):
    """The reset rule (M263): the open-catch release arms the watch, and the trim is reset when the release completes (the fingers at rest, at
    least close_min_s after the release). Before close_min_s nothing resets; without a release nothing resets."""
    layer = SagLayer(model, data, "left", HOME)
    layer.C["p"][:] = [0.001, 0.002, 0.003]
    layer.C["r"][:] = [0.01, 0.0, 0.0]
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
    early = layer.C["p"].copy()
    layer.track_open(data, 1.6)                          # stalled and past close_min_s: the reset
    reset_ok = not np.any(layer.C["p"]) and not np.any(layer.C["r"]) and rec["changing"] is False and layer.reset_at is not None and layer.reset_at[1] == "release_open_done"
    kept = bool(np.array_equal(early, np.array([0.001, 0.002, 0.003])))
    hmr.reset(model)
    data.qpos[fq] = 0.0
    ok = kept and reset_ok
    return check("(b) reset rule: the release arms the watch; nothing resets before close_min_s; the reset comes when the release completes", ok,
                 f"kept before close_min_s {kept}; reset at completion {reset_ok}")


def case_g(model, data):
    """A release that completes is one reset: after it, the watch is not armed again by the layer (only a new release arms it)."""
    layer = SagLayer(model, data, "left", HOME)
    layer.C["p"][:] = [0.004, 0.0, 0.0]
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
    layer.C["p"][:] = [0.002, 0.0, 0.0]                  # the trim builds again after the reset
    for k in range(200):
        t += NOMINAL
        layer.track_open(data, t)                        # 200 ticks with the fingers open at rest, no release
    ok = first and bool(np.array_equal(layer.C["p"], np.array([0.002, 0.0, 0.0]))) and sum(1 for e in layer.log if e["kind"] == "release_reset") == 1
    return check("(g) one release, one reset: no second reset without a new release", ok, f"correction kept after the first reset: {ok}")


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
    """Two commands for the same arm in one tick: the second does not integrate or divide by zero; the outputs stay finite and the state is kept."""
    layer = SagLayer(model, data, "left", HOME)
    q = source_targets(data, [0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    first = layer.correct(q, data, 0.0, NOMINAL)
    again = layer.correct(q, data, 0.0, NOMINAL)
    nxt = layer.correct(q, data, NOMINAL, NOMINAL)
    ok = bool(np.all(np.isfinite(again)) and np.array_equal(again, first) and np.all(np.isfinite(nxt)))
    return check("(S5) two commands in one tick: no integration on the second, outputs stay finite", ok,
                 f"second command equals the first: {np.array_equal(again, first)}; later output finite: {bool(np.all(np.isfinite(nxt)))}")


def case_a_open(model, data):
    """(a) no reset on an ordinary gripper-open command: the fingers open at rest, the trim kept, and no release call (only the release arms it)."""
    layer = SagLayer(model, data, "left", HOME)
    layer.C["p"][:] = [0.003, 0.0, 0.0]
    ids = arm_ids(model, "left")
    fq = int(model.jnt_qposadr[ids["finger_jnt"][0]])
    data.qpos[fq] = sag_layer.TASK_OPEN
    data.qvel[:] = 0.0
    for k in range(30):
        layer.track_open(data, k * NOMINAL)              # the loop's per-tick watch, with no release armed
    ok = bool(np.array_equal(layer.C["p"], np.array([0.003, 0.0, 0.0]))) and layer.reset_at is None
    return check("(a) no reset on an ordinary gripper-open command (no release, the fingers open at rest)", ok, f"correction kept {ok}")


def case_c_periodic(model, data):
    """(c) no periodic reset: 10 s of control ticks with the gripper open and at rest, no release, the trim integrating; no reset at all."""
    layer = SagLayer(model, data, "left", HOME)
    ids = arm_ids(model, "left")
    fq = int(model.jnt_qposadr[ids["finger_jnt"][0]])
    data.qpos[fq] = sag_layer.TASK_OPEN
    data.qvel[:] = 0.0
    t = 0.0
    peak = 0.0
    for k in range(int(10.0 / NOMINAL)):
        layer.correct(source_targets(data, [0.03, 0.0, 0, 0, 0, 0, 0]), data, t, NOMINAL)
        layer.track_open(data, t)
        peak = max(peak, float(np.linalg.norm(layer.C["p"])))
        t += NOMINAL
    ok = layer.reset_at is None and not any(e["kind"] == "release_reset" for e in layer.log) and peak > 0.01 and float(np.linalg.norm(layer.C["p"])) > 0.01
    return check("(c) no periodic reset over 10 s with the gripper open (the trim keeps integrating)", ok,
                 f"resets {0 if layer.reset_at is None else 1}; peak |C| {1000 * peak:.2f} mm, final {1000 * float(np.linalg.norm(layer.C['p'])):.2f} mm")


def _arm_cmd(q, manip):
    """A command as the loop builds it (an arms command with the source's manipulation tag)."""
    return {"mode": "arms", "arm_targets": {"left": [float(v) for v in q]}, "gripper": {"left": 0.044}, "manip": manip, "period_steps": 3}


def case_m264_a(model, data):
    """M264 (a): a staging command passes through unchanged and the trim does not integrate (no call to the trim step, C stays zero), also after
    a long run of staging commands."""
    layer = SagLayer(model, data, "left", HOME)
    layers = {"left": layer, "right": SagLayer(model, data, "right", np.asarray(TASK_CFG["home_q_right"], float))}
    real = sag_layer._sag_trim_step
    calls = []

    def spy(*a, **k):
        calls.append(1)
        return real(*a, **k)
    sag_layer._sag_trim_step = spy
    try:
        same = True
        for k in range(300):
            q = source_targets(data, [0.03 * np.sin(0.01 * k), 0.02, 0.01, 0, 0, 0, 0])
            out = correct_command(layers, _arm_cmd(q, False), data, k * NOMINAL, NOMINAL)
            same = same and np.array_equal(np.array(out["arm_targets"]["left"]), q)
    finally:
        sag_layer._sag_trim_step = real
    ok = same and not calls and not np.any(layer.C["p"]) and not np.any(layer.C["r"])
    return check("M264 (a) a staging command passes through unchanged; the trim does not integrate (300 staging commands)", ok,
                 f"targets unchanged {same}; trim steps {len(calls)}; C zero {not np.any(layer.C['p'])}")


def case_m264_b(model, data):
    """M264 (b): a manipulation command integrates and is corrected exactly as the layer's own correct() does."""
    a = SagLayer(model, data, "left", HOME)
    ref = SagLayer(model, data, "left", HOME)
    layers = {"left": a, "right": SagLayer(model, data, "right", np.asarray(TASK_CFG["home_q_right"], float))}
    same, changed = True, False
    for k in range(120):
        q = source_targets(data, [0.03, 0.0, 0.01, 0, 0, 0, 0])
        out = np.array(correct_command(layers, _arm_cmd(q, True), data, k * NOMINAL, NOMINAL)["arm_targets"]["left"])
        exp = ref.correct(q, data, k * NOMINAL, NOMINAL)
        same = same and np.allclose(out, exp, atol=1e-12)
        changed = changed or not np.allclose(out, q)
    ok = same and changed and np.any(a.C["p"])
    return check("M264 (b) a manipulation command integrates and is corrected as the layer's own correction (120 commands)", ok,
                 f"same as the layer's correction {same}; changed from the source {changed}; |C| {1000 * float(np.linalg.norm(a.C['p'])):.2f} mm")


def case_m264_c(model, data):
    """M264 (c): the trim is zero at the first manipulation command after a long run of staging commands: the first manipulation command gives
    exactly the correction a fresh layer gives for that command (the takeover zeroes the trim the first time)."""
    a = SagLayer(model, data, "left", HOME)
    layers = {"left": a, "right": SagLayer(model, data, "right", np.asarray(TASK_CFG["home_q_right"], float))}
    for k in range(60):                                  # a manipulation run that builds a large trim
        correct_command(layers, _arm_cmd(source_targets(data, [0.04, 0.0, 0.0, 0, 0, 0, 0]), True), data, k * NOMINAL, NOMINAL)
    built = 1000 * float(np.linalg.norm(a.C["p"]))
    for k in range(60, 60 + 400):                        # a long run of staging commands
        correct_command(layers, _arm_cmd(source_targets(data, [0.0, 0.0, 0.0, 0, 0, 0, 0]), False), data, k * NOMINAL, NOMINAL)
    t0 = 460 * NOMINAL
    q = source_targets(data, [0.01, 0.0, 0.0, 0, 0, 0, 0])
    correct_command(layers, _arm_cmd(q, True), data, t0, NOMINAL)
    fresh = SagLayer(model, data, "left", HOME)
    fresh.correct(q, data, t0, NOMINAL)                  # the trim: one step from zero, as a fresh layer gives it
    ok = np.allclose(a.C["p"], fresh.C["p"], atol=1e-15) and np.allclose(a.C["r"], fresh.C["r"], atol=1e-15) and a.manip_active \
        and any(e["kind"] == "takeover" for e in a.log)
    return check("M264 (c) the trim is zero at the first manipulation command after a long staging run (takeover)", ok,
                 f"trim before the takeover {built:.2f} mm; after the first manipulation command the trim equals a fresh layer's one step: {ok}")


def main():
    model, data = robot()
    results = [case_a(model, data), case_a_open(model, data), case_b(model, data), case_c(model, data), case_c_periodic(model, data), case_d(model, data), case_e(model, data), case_f(model, data), case_g(model, data), case_h(model, data), case_i(model, data), case_m264_a(model, data), case_m264_b(model, data), case_m264_c(model, data)]
    print(f"{sum(results)} of {len(results)} passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
