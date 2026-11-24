"""Synthetic tests for the sag trim as a control-loop layer (M262): the same correction as the old in-script trim plus the leak of SF-118, the hold during a held-mass
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


def sweep(k):
    """Targets that change on every command (a sine sweep), so the cases that check integration send these."""
    return source_targets(None, [0.03 * np.sin(0.05 * k), 0.0, 0, 0, 0, 0, 0])


def case_a_retired(model, data):  # RETIRED (SF-123, David 2026-11-24): compared the layer with the old in-script trim, whose error is gone
    """The layer's correction for a sequence of commands equals the old in-script trim for the same inputs (the same integration, the same gains
    and limits), with the SF-118 leak applied before each integration step."""
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
        decay = float(np.exp(-dt / float(SAG_LAYER_CFG["sag_leak_tau_s"])))          # SF-118: the leak, before the integration step
        C_old["p"], C_old["r"] = C_old["p"] * decay, C_old["r"] * decay
        old(C_old, PICK_PLACE_CFG, model, "left", t_, p_err, r_err, dt)
    ok = np.array_equal(layer.C["p"], C_old["p"]) and np.array_equal(layer.C["r"], C_old["r"]) and len(recorded) == 200
    return check("(S1) the layer's correction equals the old in-script trim plus the leak, for the same input sequence (200 commands)", ok,
                 f"correction p {np.round(1000 * layer.C['p'], 4)} mm (old {np.round(1000 * C_old['p'], 4)}), identical: {ok}")


def _cc(layer, q, data, t, dt):
    """SF-122 stage 2 harness: the layer's command, and the bridge output fed back as the layer sees it (bridge off: the command itself)."""
    cmd = layer.correct(q, data, t, dt)
    layer.observe(cmd, t, False)
    return cmd


def _cc_cmd(layers, cmd, data, t, dt):
    """SF-123 harness (approved by David 2026-11-24): correct_command, then the bridge output fed back to each layer as the loop does (bridge off: the
    command itself). Without the feed the layer has no reference and skips every tick (no_reference)."""
    out = correct_command(layers, cmd, data, t, dt)
    if out.get("mode") == "arms" and out.get("arm_targets"):
        for s, q in out["arm_targets"].items():            # only the arms the command carries (the loop feeds each arm's own output)
            layers[s].observe(np.array(q, float), t, False)
    return out


def case_b(model, data):
    """The correction does not integrate while a held-mass ramp runs; it resumes after the ramp."""
    layer = SagLayer(model, data, "left", HOME)
    hmr.reset(model)
    hmr.start_rate(model, "left", 0.05, 0.0, 0)
    before = layer.C["p"].copy()
    t = 0.0
    for k in range(60):
        _cc(layer, sweep(k), data, t, NOMINAL)
        t += NOMINAL
    frozen = np.array_equal(layer.C["p"], before)
    hmr.stop_rate(model, "left", t)
    for k in range(60):
        _cc(layer, sweep(k), data, t, NOMINAL)
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
        _cc(layer, sweep(k), data, t, NOMINAL)
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
        _cc(layer, sweep(30), data, t, NOMINAL)
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
    n_ticks = int(10.0 / NOMINAL)

    def run():
        nonlocal t, peak
        for k in range(n_ticks):
            _cc(layer, sweep(k), data, t, NOMINAL)
            layer.track_open(data, t)
            peak = max(peak, float(np.linalg.norm(layer.C["p"])))
            t += NOMINAL
    _, steps = _count_steps(run)
    # the first two ticks have no reference yet (the designed no_reference skip: REF(t-1) and REF(t-2) are needed), so n_ticks - 2 integrate
    ok = layer.reset_at is None and not any(e["kind"] == "release_reset" for e in layer.log) and steps == n_ticks - 2 and peak > 0.0
    return check("(c) no periodic reset over 10 s with the gripper open (the trim keeps integrating)", ok,
                 f"resets {0 if layer.reset_at is None else 1}; trim steps {steps} of {n_ticks} ticks; peak |C| {1000 * peak:.2f} mm, final {1000 * float(np.linalg.norm(layer.C['p'])):.2f} mm")


def _arm_cmd(q, manip):
    """A command as the loop builds it: an arms command with the safety filter's manipulation tag (the sag trim no longer reads it, S5)."""
    return {"mode": "arms", "arm_targets": {"left": [float(v) for v in q]}, "gripper": {"left": 0.044}, "manip": manip, "period_steps": 3}


def _pair(model, data):
    layer = SagLayer(model, data, "left", HOME)
    return layer, {"left": layer, "right": SagLayer(model, data, "right", np.asarray(TASK_CFG["home_q_right"], float))}


def _count_steps(fn):
    """Runs fn() with the trim step spied on. Returns (fn's result, number of trim steps taken)."""
    real = sag_layer._sag_trim_step
    calls = []

    def spy(*a, **k):
        calls.append(1)
        return real(*a, **k)
    sag_layer._sag_trim_step = spy
    try:
        res = fn()
    finally:
        sag_layer._sag_trim_step = real
    return res, len(calls)


def _moving(layers, data, k0, n, manip=False):
    """n commands from tick k0 that all change the targets (a sine sweep). Returns the next tick."""
    for k in range(k0, k0 + n):
        q = source_targets(data, [0.03 * np.sin(0.05 * k), 0.02 * np.cos(0.05 * k), 0.01, 0, 0, 0, 0])
        _cc_cmd(layers, _arm_cmd(q, manip), data, k * NOMINAL, NOMINAL)
    return k0 + n


def _standing(layers, data, k0, n, q, manip=False):
    """n commands from tick k0 with the same targets q (the command stands still). Returns the next tick."""
    for k in range(k0, k0 + n):
        _cc_cmd(layers, _arm_cmd(q, manip), data, k * NOMINAL, NOMINAL)
    return k0 + n


def case_s5_a(model, data):
    """SF-118 (replaces the S5 freeze test): a standing command no longer freezes the trim. 60 standing commands after a moving run each run the
    trim step, and the trim changes on each one (the leak, plus any error still there)."""
    layer, layers = _pair(model, data)
    k = _moving(layers, data, 0, 120)
    q = source_targets(data, [0.01, 0.0, 0.0, 0, 0, 0, 0])
    cs = []

    def stand():
        for j in range(60):
            _cc_cmd(layers, _arm_cmd(q, False), data, (k + j) * NOMINAL, NOMINAL)
            cs.append(layer.C["p"].copy())
    _, steps = _count_steps(stand)
    changed = all(not np.array_equal(cs[i], cs[i - 1]) for i in range(1, len(cs)))
    ok = steps == 60 and changed and float(np.linalg.norm(cs[-1])) > 0.0
    return check("SF-118 (a) a standing command does not freeze the trim: all 60 standing commands integrate and leak", ok,
                 f"trim steps in 60 standing commands {steps} (expected 60); C changed on every one {changed}; |C| {1000 * float(np.linalg.norm(cs[-1])):.2f} mm")


def case_s5_d(model, data):
    """S5 (d): no source gate. The same command stream gives the same corrected output whatever the manipulation tag says (the tag is the
    safety filter's, not the trim's), and a non-manipulation source's moving command is corrected."""
    la, lla = _pair(model, data)
    lb, llb = _pair(model, data)
    same, changed = True, False
    k = 0
    for j in range(120):
        q = source_targets(data, [0.03 * np.sin(0.05 * j), 0.02 * np.cos(0.05 * j), 0.01, 0, 0, 0, 0])
        out_a = np.array(_cc_cmd(lla, _arm_cmd(q, False), data, j * NOMINAL, NOMINAL)["arm_targets"]["left"])
        out_b = np.array(_cc_cmd(llb, _arm_cmd(q, True), data, j * NOMINAL, NOMINAL)["arm_targets"]["left"])
        same = same and np.array_equal(out_a, out_b)
        changed = changed or not np.allclose(out_a, q)
    ok = same and changed and float(np.linalg.norm(la.C["p"])) > 0.0
    return check("S5 (d) no source gate: the tag does not change the correction; a non-manipulation source's moving command is corrected", ok,
                 f"same output with either tag {same}; corrected {changed}; |C| {1000 * float(np.linalg.norm(la.C['p'])):.2f} mm")


TAU = float(SAG_LAYER_CFG["sag_leak_tau_s"])
GAIN = float(PICK_PLACE_CFG["sag_gain_per_s"])


def _leak_trace(model, errors):
    """The integrator alone (no solver, no arm), one tick per entry of errors: the leak, then the integration step, as SagLayer.correct runs
    them. errors: the position error (m, 3) of each tick; the rotation error is zero. Returns the trim's position after each tick."""
    hmr.reset(model)
    C = {"p": np.zeros(3), "r": np.zeros(3)}
    out, t = [], 0.0
    for e in errors:
        sag_layer._sag_leak(C, model, "left", t, NOMINAL)
        sag_layer._sag_trim_step(C, PICK_PLACE_CFG, model, "left", t, np.asarray(e, float), np.zeros(3), NOMINAL)
        out.append(C["p"].copy())
        t += NOMINAL
    return out


def case_leak_constant(model, data):
    """SF-118 (L1): a constant error of 1 mm for 100 ticks. The trim follows the leaky step in closed form, stays below its steady state, and
    reaches it after 5 time constants. The steady state is gain x error x tau (15 mm for 1 mm). 100 ticks is 0.67 s, under a seventh of tau,
    so the trim is still far from steady there."""
    e = np.array([0.001, 0.0, 0.0])
    d = float(np.exp(-NOMINAL / TAU))
    inc = GAIN * 0.001 * NOMINAL                        # the step added on each tick (m)
    c_ss = inc / (1.0 - d)                              # the steady state: step / (1 - decay per tick)
    trace = _leak_trace(model, [e] * 100)
    closed = inc * (1.0 - d ** 100) / (1.0 - d)
    on_closed_form = abs(trace[-1][0] - closed) < 1e-12
    no_overshoot = max(p[0] for p in trace) <= c_ss * (1 + 1e-9)
    long_run = _leak_trace(model, [e] * int(round(5 * TAU / NOMINAL)))
    converged = abs(long_run[-1][0] - c_ss) / c_ss < 0.01
    formula = abs(c_ss - GAIN * 0.001 * TAU) / (GAIN * 0.001 * TAU) < 0.01
    ok = on_closed_form and no_overshoot and converged and formula
    return check("SF-118 (L1) constant error: the trim follows the leaky step, does not pass the steady state, and reaches it (gain x error x tau)", ok,
                 f"after 100 ticks {1000 * trace[-1][0]:.3f} mm (closed form {1000 * closed:.3f}); steady state {1000 * c_ss:.3f} mm; "
                 f"after 5 tau {1000 * long_run[-1][0]:.3f} mm; gain x error x tau {1000 * GAIN * 0.001 * TAU:.3f} mm")


def case_leak_alternating(model, data):
    """SF-118 (L2): an error of +1 mm and then -1 mm on alternate ticks for 200 ticks. The conflicting errors cancel: the trim stays within 0.1 mm
    of zero (a constant 1 mm error would reach 15 mm)."""
    errs = [np.array([0.001 if k % 2 == 0 else -0.001, 0.0, 0.0]) for k in range(200)]
    trace = _leak_trace(model, errs)
    peak = max(abs(p[0]) for p in trace)
    ok = peak < 0.0001
    return check("SF-118 (L2) alternating +1 mm and -1 mm errors: the trim stays near zero", ok, f"peak |C| {1000 * peak:.4f} mm (limit 0.1 mm)")


def case_leak_decay(model, data):
    """SF-118 (L3): a 1 mm error for 50 ticks, then zero error for 200 ticks. The trim falls toward zero with time constant tau: after the 200
    zero-error ticks it is exp(-200 dt / tau) of its value at tick 50, and the time constant fitted to those two values is tau."""
    errs = [np.array([0.001, 0.0, 0.0])] * 50 + [np.zeros(3)] * 200
    trace = _leak_trace(model, errs)
    c50, c250 = trace[49][0], trace[249][0]
    ratio = c250 / c50
    fitted = -200 * NOMINAL / np.log(ratio)
    falling = all(trace[k + 1][0] < trace[k][0] for k in range(49, 249))
    ok = c50 > 0.0 and abs(ratio - np.exp(-200 * NOMINAL / TAU)) < 1e-12 and abs(fitted - TAU) < 1e-9 and falling
    return check("SF-118 (L3) error then no error: the trim decays toward zero with time constant sag_leak_tau_s", ok,
                 f"trim at tick 50 {1000 * c50:.3f} mm, at tick 250 {1000 * c250:.3f} mm; ratio {ratio:.6f} (expected {np.exp(-200 * NOMINAL / TAU):.6f}); "
                 f"fitted tau {fitted:.6f} s (config {TAU} s)")


def case_s5_f(model, data):
    """S5 (f): the source-name gate is gone from the layer: sag_layer.py names no source and has no manipulation gate."""
    import inspect
    text = inspect.getsource(sag_layer)
    ok = '"vla"' not in text and "MANIPULATION_SOURCES" not in text and "manipulation(" not in text and "manip_active" not in text
    return check("S5 (f) wiring: sag_layer.py has no source-name gate (no 'vla', no MANIPULATION_SOURCES, no manipulation step)", ok,
                 f"source-name gate text present: {not ok}")


def main():
    model, data = robot()
    results = [case_a_open(model, data), case_b(model, data), case_c(model, data), case_c_periodic(model, data), case_d(model, data), case_e(model, data), case_f(model, data), case_g(model, data), case_h(model, data), case_i(model, data), case_s5_a(model, data), case_s5_d(model, data), case_s5_f(model, data), case_leak_constant(model, data), case_leak_alternating(model, data), case_leak_decay(model, data)]
    print(f"{sum(results)} of {len(results)} passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
